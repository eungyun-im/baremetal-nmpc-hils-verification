#include "bmpc_safety.h"

#include <string.h>

#define RAD2DEG_F 57.29577951f

void bs_default_cfg(bs_cfg_t *cfg)
{
  cfg->accel_gain = 3.0f; /* 팀 safety_bridge.py accel_gain 기본값 */
  cfg->brake_gain = 5.0f; /* brake_gain 기본값 */
}

void bs_default_safety_config(sc_config_t *scfg)
{
  sc_default_config(scfg);
  /* 2026-11-04 HILS·원본 랩의 명령 기준 운영 영역: 조향 최대 28 deg, 변화율 최대 101 deg/s (p99.9 65),
   * 횡가속도 최대 약 7.5 m/s2 (L = 4.0), 가감속 -5..+3 m/s2 (페달 변환 끝값). 정상 주행에는 개입하지 않고
   * 그 밖의 명령(급변, 범위 밖)만 막도록 운영 영역 위에 둔다. */
  scfg->steer_rate_dps = 120.0f;     /* TSR-02: MPC 조향 슬루 한계(1.8 rad/s = 103 deg/s) 위 */
  scfg->wheelbase_m = 4.0f;          /* TSR-05: MPC 모델 Lf 와 같은 유효 축거 (MORAI 차량의 실제 조향 응답 기준) */
  scfg->lat_acc_max_mps2 = 10.0f;    /* TSR-05 */
  scfg->lat_acc_meas_max_mps2 = 10.0f; /* TSR-11 */
  scfg->lat_acc_mismatch_mps2 = 3.0f;  /* TSR-12 */
  /* TSR-03: 명령이 50 ms 마다 오므로 한 주기 놓쳐도 SAFE_STOP 이 되지 않게 (팀 통합안과 같은 200 ms) */
  scfg->cmd_timeout_ms = 200u;
}

static int bs_can_send(void *ctx, const bmpc_can_frame_t *f);

void bs_init(bs_app_t *a, const bs_cfg_t *cfg, const sc_config_t *scfg, const bmpc_hal_t *hal)
{
  bmpc_hal_t inner;
  memset(a, 0, sizeof(*a));
  a->cfg = *cfg;
  a->hal = *hal;
  inner.now_us = hal->now_us;
  inner.can_send = bs_can_send; /* MPC 응답을 가로채 CTRL_CMD 를 만든다 */
  inner.ctx = a;
  bmpc_app_init(&a->app, &inner);
  sc_init(&a->sc, scfg);
  a->out.state = SC_STATE_INIT;
}

/* 물리값 x 100 을 반올림해 int16 (포화), 리틀 엔디안 */
static void put_i16_x100(uint8_t *p, float v)
{
  float r = v * 100.0f;
  int16_t q;
  if (!(r == r)) r = 0.0f;
  if (r > 32767.0f) r = 32767.0f;
  else if (r < -32768.0f) r = -32768.0f;
  q = (int16_t)((r >= 0.0f) ? (r + 0.5f) : (r - 0.5f));
  p[0] = (uint8_t)((uint16_t)q & 0xFFu);
  p[1] = (uint8_t)(((uint16_t)q >> 8) & 0xFFu);
}

void bs_make_ctrl_frame(const bs_cfg_t *cfg, const bmpc_response_t *r, uint8_t d[8])
{
  const float steer_deg = (float)r->steer_cmd * RAD2DEG_F; /* 응답 steer_cmd 는 /ctrl_cmd 와 같은 부호 */
  const float accel = (float)r->accel * cfg->accel_gain - (float)r->brake * cfg->brake_gain;
  memset(d, 0, 8);
  put_i16_x100(&d[0], steer_deg);         /* SteerCmd 0|16 0.01 deg */
  put_i16_x100(&d[2], accel);             /* AccelCmd 16|16 0.01 m/s2 */
  d[6] = (uint8_t)(r->seq & 0x0Fu);       /* CtrlAlive 48|4 = 요청 seq 하위 4 비트 (PC 가 짝짓는다) */
  sc_frame_seal(d, SC_DATA_ID_CTRL_CMD);  /* CtrlCrc 56|8 */
}

#ifdef BS_PLAUS
/* 결함 주입 뒤, CTRL_CMD 를 만들기 전에 본다. 정상 명령은 건드리지 않는다.
 * 넘으면 직전 조향과 감속을 낸다. 명령이 곧 정상으로 돌아오면 일찍 풀고, 아니면 유지 시간 뒤 한 주기 변화를 제한하며 감속한 채 돌아간다.
 * 재시작이나 명령 공백 뒤 첫 명령은 비교하지 않는다. 브레이크는 더 큰 요청을 줄이지 않는다. */
static void bs_plaus_brake(bmpc_response_t *r)
{
  r->accel = (bmpc_real)0;
  if ((float)r->brake < BS_PLAUS_BRAKE) r->brake = (bmpc_real)BS_PLAUS_BRAKE;
}

static void bs_plaus_apply(bs_app_t *a, bmpc_response_t *r)
{
  const int64_t now = a->hal.now_us(a->hal.ctx);
  const float step = BS_PLAUS_STEP_DEG / RAD2DEG_F, ramp = BS_PLAUS_RAMP_DEG / RAD2DEG_F;
  const float cmd = (float)r->steer_cmd;
  const int64_t last_us = a->plaus_last_us;
  float diff;
  a->plaus_last_us = now;
  if (a->plaus_reset_req || !a->plaus_have || now - last_us > BS_PLAUS_GAP_US) {
    if (a->plaus_reset_req) a->n_plaus = 0u;
    a->plaus_reset_req = 0;
    a->plaus_have = 1;
    a->plaus_returning = 0;
    a->plaus_until_us = 0;
    a->plaus_last = cmd;
    return;
  }
  diff = cmd - a->plaus_last;
  if (now < a->plaus_until_us) {
    if (diff <= step && diff >= -step) {
      if (++a->plaus_ok >= BS_PLAUS_OK_CMDS) {
        a->plaus_until_us = 0;
        a->plaus_returning = 0;
        a->plaus_last = cmd;
        return;
      }
    } else {
      a->plaus_ok = 0;
    }
    r->steer_cmd = (bmpc_real)a->plaus_last;
    bs_plaus_brake(r);
    return;
  }
  if (a->plaus_returning) {
    if (diff > ramp) a->plaus_last += ramp;
    else if (diff < -ramp) a->plaus_last -= ramp;
    else {
      a->plaus_last = cmd;
      a->plaus_returning = 0;
      return;
    }
    r->steer_cmd = (bmpc_real)a->plaus_last;
    bs_plaus_brake(r);
    return;
  }
  if (diff <= step && diff >= -step) {
    a->plaus_last = cmd;
    return;
  }
  ++a->n_plaus;
  a->plaus_until_us = now + BS_PLAUS_HOLD_US;
  a->plaus_returning = 1;
  a->plaus_ok = 0;
  r->steer_cmd = (bmpc_real)a->plaus_last;
  bs_plaus_brake(r);
}
#endif

#ifdef BS_TEST_HOOK
static void bs_hook_apply(bs_app_t *a, bmpc_response_t *r)
{
  const int64_t now = a->hal.now_us(a->hal.ctx);
  const float steer = (float)r->steer_cmd;
  if (a->hook_pending) {
    a->hook_pending = 0;
    a->hook_active = a->hook_kind;
    a->hook_until_us = now + (int64_t)a->hook_ms * 1000;
    ++a->n_hook;
  }
  if (a->hook_active != BS_HOOK_OFF && now >= a->hook_until_us) a->hook_active = BS_HOOK_OFF;
  if (a->hook_active == BS_HOOK_OFFSET) {
    r->steer_cmd = (bmpc_real)(steer + (float)a->hook_centideg * 0.01f / RAD2DEG_F);
  } else if (a->hook_active == BS_HOOK_HOLD) {
    r->steer_cmd = (bmpc_real)a->hook_last_steer;
  } else {
    a->hook_last_steer = steer;
  }
}
#endif

static int bs_can_send(void *ctx, const bmpc_can_frame_t *f)
{
  bs_app_t *a = (bs_app_t *)ctx;
  const int rc = a->hal.can_send(a->hal.ctx, f);
  if (f->id >= BMPC_ID_RESP_BASE && f->id < BMPC_ID_RESP_BASE + BMPC_RESP_FRAMES) {
    const unsigned i = (unsigned)(f->id - BMPC_ID_RESP_BASE);
    if (i == 0) a->resp_mask = 0;
    a->resp[i] = *f;
    a->resp_mask |= (uint8_t)(1u << i);
    if (a->resp_mask == (uint8_t)((1u << BMPC_RESP_FRAMES) - 1u)) {
      bmpc_response_t r;
      a->resp_mask = 0;
      /* 수신 오류·설정 전 응답에는 명령이 없다. 그 밖(풀이 실패 포함)은 controller 가 유지한 명령이 들어 있다 */
      if (bmpc_decode_response(a->resp, &r) == 0 && r.status != BMPC_ERR_RX && r.status != BMPC_ERR_NOT_CONFIGURED) {
#ifdef BS_TEST_HOOK
        bs_hook_apply(a, &r);
#endif
#ifdef BS_PLAUS
        bs_plaus_apply(a, &r);
#endif
        bs_make_ctrl_frame(&a->cfg, &r, a->ctrl);
        a->ctrl_pending = 1;
      }
    }
  }
  return rc;
}

int bs_is_safety_frame(uint32_t id)
{
  return id == BS_ID_ESC_SPEED || id == BS_ID_SAS_STEER || id == BS_ID_VCU_PEDAL || id == BS_ID_ACU_IMU ||
         id == BS_ID_BENCH_RESET;
}

void bs_on_safety_frame(bs_app_t *a, const bmpc_can_frame_t *f, uint32_t now_ms)
{
  if (f->id == BS_ID_BENCH_RESET) {
#ifdef BS_TEST_HOOK
    if (f->dlc >= 6u && f->data[0] == BS_HOOK_MAGIC) {
      a->hook_kind = f->data[1];
      a->hook_centideg = (int16_t)((uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8));
      a->hook_ms = (uint16_t)((uint16_t)f->data[4] | ((uint16_t)f->data[5] << 8));
      a->hook_pending = 1;
      return;
    }
#endif
    if (f->dlc >= 2u && f->data[0] == 0x5Au && f->data[1] == 0xA5u) {
      sc_reset(&a->sc); /* 시험대 전용: SAFE_STOP 래치 해제 */
#ifdef BS_PLAUS
      a->plaus_reset_req = 1;
#endif
      ++a->n_resets;
    }
    return;
  }
  if (f->dlc != 8u) return; /* 차량 CAN 은 모두 8 바이트 (vehicle.dbc) */
  if (f->id == BS_ID_ESC_SPEED) sc_on_esc_frame(&a->sc, f->data, now_ms);
  else if (f->id == BS_ID_ACU_IMU) sc_on_imu_frame(&a->sc, f->data, now_ms);
  else if (f->id == BS_ID_VCU_PEDAL) sc_on_pedal_frame(&a->sc, f->data, now_ms);
  else if (f->id == BS_ID_SAS_STEER) sc_on_sas_frame(&a->sc, f->data, now_ms);
  else return;
  ++a->n_safety_rx;
}

void bs_on_mpc_frame(bs_app_t *a, const bmpc_can_frame_t *f) { bmpc_app_on_frame(&a->app, f); }

int bs_feed_ctrl(bs_app_t *a, uint32_t now_ms)
{
  if (!a->ctrl_pending) return 0;
  a->ctrl_pending = 0;
  sc_on_ctrl_frame(&a->sc, a->ctrl, now_ms);
  ++a->n_ctrl;
  return 1;
}

void bs_safety_tick(bs_app_t *a, uint32_t now_ms)
{
  bmpc_can_frame_t f;
  sc_step(&a->sc, now_ms, &a->out);
  f.dlc = 8u;
  f.id = BS_ID_SAFE_OUT;
  sc_pack_out(&a->out, a->out_alive, f.data);
  (void)a->hal.can_send(a->hal.ctx, &f);
  f.id = BS_ID_SAFE_STAT;
  sc_pack_stat(&a->out, a->stat_alive, f.data);
  (void)a->hal.can_send(a->hal.ctx, &f);
  a->out_alive = (uint8_t)((a->out_alive + 1u) & 0x0Fu);
  a->stat_alive = (uint8_t)((a->stat_alive + 1u) & 0x0Fu);
  ++a->n_steps;
  if (a->out.state == SC_STATE_NORMAL || a->out.state == SC_STATE_DEGRADED) {
    a->applied_steer_deg = a->out.steer_deg;
    a->applied_pending = 1;
  }
}

void bs_note_applied(bs_app_t *a, int64_t now_us)
{
  float deg;
  if (!a->applied_pending) return;
  deg = a->applied_steer_deg;
  a->applied_pending = 0;
  a->n_notes += (uint32_t)bmpc_controller_note_applied(&a->app.ctrl, now_us, (bmpc_real)(deg / RAD2DEG_F));
}

void bs_on_bus_off(bs_app_t *a) { sc_on_bus_off(&a->sc); }
