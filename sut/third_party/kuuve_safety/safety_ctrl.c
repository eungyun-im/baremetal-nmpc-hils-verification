#include "safety_ctrl.h"

#include <math.h>
#include <string.h>

#define RAD2DEG 57.2957795f
#define KPH2MPS 0.2777778f

static int16_t raw_i16(const uint8_t *p)
{
    return (int16_t)(uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static uint16_t raw_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static void put_i16(uint8_t *p, float physical)
{
    float r = physical * 100.0f;
    int16_t v;

    if (r > 32767.0f) {
        r = 32767.0f;
    } else if (r < -32768.0f) {
        r = -32768.0f;
    } else {
        /* 범위 안: 그대로 */
    }
    if (r >= 0.0f) {
        r = r + 0.5f;
    } else {
        r = r - 0.5f;
    }
    v = (int16_t)r;
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static uint8_t crc8_update(uint8_t crc_in, uint8_t byte)
{
    uint8_t crc = (uint8_t)(crc_in ^ byte);
    uint8_t b;

    for (b = 0; b < 8u; b++) {
        if ((crc & 0x80u) != 0u) {
            crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x1Du);
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

uint8_t sc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    uint8_t i;

    for (i = 0; i < len; i++) {
        crc = crc8_update(crc, data[i]);
    }
    return (uint8_t)(crc ^ 0xFFu);
}

uint8_t sc_crc8_e2e(uint16_t data_id, const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    uint8_t i;

    crc = crc8_update(crc, (uint8_t)(data_id & 0xFFu));
    crc = crc8_update(crc, (uint8_t)(data_id >> 8));
    for (i = 0; i < len; i++) {
        crc = crc8_update(crc, data[i]);
    }
    return (uint8_t)(crc ^ 0xFFu);
}

void sc_frame_seal(uint8_t data[8], uint16_t data_id)
{
    data[7] = sc_crc8_e2e(data_id, data, 7u);
}

void sc_default_config(sc_config_t *cfg)
{
    cfg->steer_limit_deg      = 40.0f;   /* TSR-01 */
    cfg->steer_rate_dps       = 35.0f;   /* TSR-02 */
    cfg->cmd_timeout_ms       = 100u;    /* TSR-03 */
    cfg->alive_freeze_limit   = 3u;      /* TSR-04 */
    cfg->init_valid_frames    = 3u;
    cfg->recover_valid_frames = 10u;
    cfg->accel_min_mps2       = -5.0f;
    cfg->accel_max_mps2       = 3.0f;
    cfg->safe_decel_mps2      = -3.0f;
    cfg->wheelbase_m          = 2.7f;    /* TSR-05: 차량 제원에 맞게 조정 */
    cfg->lat_acc_max_mps2     = 3.0f;    /* TSR-05 */
    cfg->speed_max_kph        = 120.0f;  /* TSR-06 */
    cfg->speed_jump_max_kph   = 30.0f;   /* TSR-06 */
    cfg->speed_reject_limit   = 5u;      /* TSR-06: 이 횟수 연속 거부되면 실제 변화로 보고 수용 */
    cfg->speed_timeout_ms     = 200u;    /* TSR-09 */
    cfg->veh_input_timeout_ms = 200u;
    cfg->brake_override_pct   = 20.0f;   /* TSR-10 */
    cfg->lat_acc_meas_max_mps2 = 4.0f;   /* TSR-11 */
    cfg->lat_acc_mismatch_mps2 = 2.0f;   /* TSR-12 */
    cfg->mismatch_frames      = 10u;     /* TSR-12 */
    cfg->steer_track_enable   = 0u;      /* TSR-13: MORAI 조향각 센서 부호/배율 보정 전 (오차는 보고) */
    cfg->sas_scale            = 1.0f;
    cfg->steer_track_err_deg  = 10.0f;   /* TSR-13 */
    cfg->steer_track_ms       = 500u;    /* TSR-13 */
}

void sc_init(sc_t *sc, const sc_config_t *cfg)
{
    (void)memset(sc, 0, sizeof(*sc));
    sc->cfg = *cfg;
    sc->state = SC_STATE_INIT;
}

void sc_reset(sc_t *sc)
{
    sc_config_t cfg = sc->cfg;

    sc_init(sc, &cfg);
}

/* TSR-04: CRC 는 맞지만 Alive 가 그대로인 프레임 -> 명령은 폐기, 정지가 이어지면 DEGRADED */
static void on_stale_frame(sc_t *sc)
{
    sc->pending_flags |= SC_FLAG_ALIVE_FREEZE;
    sc->fresh_count = 0;
    if (sc->stale_count < 255u) {
        sc->stale_count++;
    }
    if ((sc->state == SC_STATE_NORMAL) && (sc->stale_count >= sc->cfg.alive_freeze_limit)) {
        sc->state = SC_STATE_DEGRADED;
    }
}

/* 새 명령 프레임: 명령 갱신, INIT/DEGRADED 에서 NORMAL 로 갈지 판정 */
static void on_fresh_frame(sc_t *sc, const uint8_t data[8], uint8_t alive)
{
    sc->alive_prev = alive;
    sc->alive_seen = 1u;
    sc->stale_count = 0;
    if (sc->fresh_count < 255u) {
        sc->fresh_count++;
    }
    sc->cmd_steer_deg = (float)raw_i16(&data[0]) * 0.01f;
    sc->cmd_accel_mps2 = (float)raw_i16(&data[2]) * 0.01f;

    if ((sc->state == SC_STATE_INIT) && (sc->fresh_count >= sc->cfg.init_valid_frames)) {
        sc->state = SC_STATE_NORMAL;
    } else if ((sc->state == SC_STATE_DEGRADED) && (sc->fresh_count >= sc->cfg.recover_valid_frames)) {
        sc->state = SC_STATE_NORMAL;
    } else {
        /* 상태 유지 */
    }
}

void sc_on_ctrl_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms)
{
    if (sc_crc8_e2e(SC_DATA_ID_CTRL_CMD, data, 7u) != data[7]) {   /* TSR-04/07: CRC 불일치 -> 폐기, 직전 유효값 유지 */
        sc->pending_flags |= SC_FLAG_CRC_ERR;
    } else {
        uint8_t alive = (uint8_t)(data[6] & 0x0Fu);

        sc->last_rx_ms = now_ms;
        if ((sc->alive_seen != 0u) && (alive == sc->alive_prev)) {
            on_stale_frame(sc);
        } else {
            on_fresh_frame(sc, data, alive);
        }
    }
}

/* TSR-06 차속 검사 (VEH_STATE, ESC_SPEED 공통). CRC 가 맞은 프레임이면 수신 시각은 갱신한다 (TSR-09) */
static void accept_speed(sc_t *sc, float v, uint32_t now_ms)
{
    sc->last_speed_ms = now_ms;
    if (v > sc->cfg.speed_max_kph) {                   /* TSR-06: 물리적으로 불가능한 값 */
        sc->pending_flags |= SC_FLAG_SPEED_REJECT;
        return;
    }
    if ((sc->speed_seen != 0u) && (fabsf(v - sc->speed_kph) > sc->cfg.speed_jump_max_kph)) {
        sc->speed_reject_run++;                        /* 한계값(최대 255)에 닿으면 아래에서 수용 후 0 */
        if (sc->speed_reject_run < sc->cfg.speed_reject_limit) {   /* 튐 -> 거부, 직전값 유지 */
            sc->pending_flags |= SC_FLAG_SPEED_REJECT;
            return;
        }
    }
    sc->speed_reject_run = 0;
    sc->speed_kph = v;
    sc->speed_seen = 1u;
}

void sc_on_veh_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms)
{
    if (sc_crc8_e2e(SC_DATA_ID_VEH_STATE, data, 7u) != data[7]) {
        sc->pending_flags |= SC_FLAG_CRC_ERR;
        return;
    }
    accept_speed(sc, (float)raw_u16(&data[0]) * 0.01f, now_ms);
}

/* 차량 CAN 프레임 CRC (vehicle.dbc Data ID). 틀리면 SAFE_STAT 에 표시하고 버린다 */
static int veh_crc_ok(sc_t *sc, uint16_t data_id, const uint8_t data[8])
{
    int ok = 1;

    if (sc_crc8_e2e(data_id, data, 7u) != data[7]) {
        sc->pending_flags2 |= SC_FLAG2_VEH_CRC;
        ok = 0;
    }
    return ok;
}

/* 입력이 한 번이라도 왔고 최근(veh_input_timeout) 이면 1 */
static int input_fresh(const sc_t *sc, uint8_t seen, uint32_t last_ms, uint32_t now_ms)
{
    return (seen != 0u) && ((uint32_t)(now_ms - last_ms) < sc->cfg.veh_input_timeout_ms);
}

void sc_on_esc_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms)
{
    if (veh_crc_ok(sc, SC_DATA_ID_ESC_SPEED, data) != 0) {
        sc->esc_lat_mps2 = (float)raw_i16(&data[4]) * 0.001f;
        sc->esc_lat_seen = 1u;
        sc->last_esc_ms = now_ms;
        accept_speed(sc, (float)raw_u16(&data[0]) * 0.01f, now_ms);
    }
}

/* TSR-12: IMU 가 올 때마다 ESC 횡가속도와 비교한다 (ESC 가 최근일 때만) */
void sc_on_imu_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms)
{
    if (veh_crc_ok(sc, SC_DATA_ID_ACU_IMU, data) != 0) {
        sc->imu_lat_mps2 = (float)raw_i16(&data[4]) * 0.001f;
        sc->imu_seen = 1u;
        sc->last_imu_ms = now_ms;
        if ((input_fresh(sc, sc->esc_lat_seen, sc->last_esc_ms, now_ms) != 0) &&
            (fabsf(sc->esc_lat_mps2 - sc->imu_lat_mps2) > sc->cfg.lat_acc_mismatch_mps2)) {
            if (sc->mismatch_run < 255u) {
                sc->mismatch_run++;
            }
        } else {
            sc->mismatch_run = 0;
        }
    }
}

void sc_on_pedal_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms)
{
    if (veh_crc_ok(sc, SC_DATA_ID_VCU_PEDAL, data) != 0) {
        sc->brake_pct = (float)data[1] * 0.4f;
        sc->pedal_seen = 1u;
        sc->last_pedal_ms = now_ms;
    }
}

void sc_on_sas_frame(sc_t *sc, const uint8_t data[8], uint32_t now_ms)
{
    if (veh_crc_ok(sc, SC_DATA_ID_SAS_STEER, data) != 0) {
        sc->sas_deg = (float)raw_i16(&data[0]) * 0.01f;
        sc->sas_seen = 1u;
        sc->last_sas_ms = now_ms;
    }
}

void sc_on_bus_off(sc_t *sc)
{
    sc->bus_off_seen = 1u;
    if ((sc->state == SC_STATE_NORMAL) || (sc->state == SC_STATE_DEGRADED)) {   /* TSR-08 */
        sc->state = SC_STATE_SAFE_STOP;
    }
}

/* value 를 [-limit, limit] 로 자르고, 잘렸으면 flag 를 세운다 */
static float clamp_sym(sc_t *sc, float value, float limit, uint8_t flag)
{
    float r = value;

    if (value > limit) {
        r = limit;
        sc->pending_flags |= flag;
    } else if (value < -limit) {
        r = -limit;
        sc->pending_flags |= flag;
    } else {
        /* 한계 안 */
    }
    return r;
}

/* TSR-01: 가감속 범위 [accel_min, accel_max] */
static float clamp_accel(sc_t *sc, float value)
{
    float r = value;

    if (value > sc->cfg.accel_max_mps2) {
        r = sc->cfg.accel_max_mps2;
        sc->pending_flags |= SC_FLAG_ACCEL_CLAMP;
    } else if (value < sc->cfg.accel_min_mps2) {
        r = sc->cfg.accel_min_mps2;
        sc->pending_flags |= SC_FLAG_ACCEL_CLAMP;
    } else {
        /* 범위 안 */
    }
    return r;
}

/* TSR-03: 유효한 명령 프레임이 timeout 동안 없으면 SAFE_STOP (INIT 에서는 아직 감시하지 않는다) */
static void check_timeout(sc_t *sc, uint32_t now_ms)
{
    if (((sc->state == SC_STATE_NORMAL) || (sc->state == SC_STATE_DEGRADED)) &&
        ((uint32_t)(now_ms - sc->last_rx_ms) >= sc->cfg.cmd_timeout_ms)) {
        sc->state = SC_STATE_SAFE_STOP;
        sc->pending_flags |= SC_FLAG_TIMEOUT;
    }
}

/* TSR-09: 차속을 한 번이라도 받은 뒤 speed_timeout 동안 없으면 DEGRADED (가속 금지).
 * 정상 명령으로 NORMAL 에 돌아가도 같은 주기에 여기서 다시 DEGRADED 가 되므로 두절 중에는 복귀하지 않는다 */
static void check_speed_timeout(sc_t *sc, uint32_t now_ms)
{
    if ((sc->speed_seen != 0u) && ((uint32_t)(now_ms - sc->last_speed_ms) >= sc->cfg.speed_timeout_ms)) {
        sc->speed_lost = 1u;
        sc->pending_flags2 |= SC_FLAG2_SPEED_TIMEOUT;
        if (sc->state == SC_STATE_NORMAL) {
            sc->state = SC_STATE_DEGRADED;
        }
    } else {
        sc->speed_lost = 0u;
    }
}

/* 가속 금지: 양수 가속을 0 으로 */
static float no_accel(float a)
{
    return (a > 0.0f) ? 0.0f : a;
}

/* TSR-13 보정용: 조향각 센서 x sas_scale 과 직전 출력의 차이를 늘 계산하고, 한계 밖이 이어진 시간을 센다 */
static void steer_track_update(sc_t *sc, uint32_t now_ms, uint32_t dt_ms, sc_out_t *out)
{
    float err = 0.0f;

    if (input_fresh(sc, sc->sas_seen, sc->last_sas_ms, now_ms) != 0) {
        err = (sc->sas_deg * sc->cfg.sas_scale) - sc->out_steer_deg;
        if (fabsf(err) > sc->cfg.steer_track_err_deg) {
            sc->track_err_ms += dt_ms;
        } else {
            sc->track_err_ms = 0u;
        }
    } else {
        sc->track_err_ms = 0u;
    }
    out->steer_track_err_deg = err;
    out->lat_acc_meas_mps2 = sc->imu_lat_mps2;
}

/* TSR-10: 시스템이 감속하지 않는데 브레이크가 밟히면 운전자 개입 */
static float driver_override(sc_t *sc, uint32_t now_ms, float accel_t)
{
    float a = accel_t;

    if ((input_fresh(sc, sc->pedal_seen, sc->last_pedal_ms, now_ms) != 0) &&
        (sc->brake_pct >= sc->cfg.brake_override_pct) && (sc->out_accel_mps2 >= 0.0f)) {
        a = no_accel(a);                                          /* TSR-10: 운전자 개입 -> 자율 가속 금지 (감속은 허용: 막으면 제동이 0 에 묶인다) */
        sc->pending_flags2 |= SC_FLAG2_DRIVER_OVERRIDE;
    }
    return a;
}

/* TSR-10~13: 차량 CAN 으로 본 실제 차량 상태로 가감속 목표를 제한한다 (NORMAL/DEGRADED 에서만) */
static float vehicle_checks(sc_t *sc, uint32_t now_ms, float accel_t)
{
    float a = accel_t;
    int imu_ok = input_fresh(sc, sc->imu_seen, sc->last_imu_ms, now_ms);

    if ((sc->state != SC_STATE_NORMAL) && (sc->state != SC_STATE_DEGRADED)) {
        return a;
    }
    a = driver_override(sc, now_ms, a);
    if ((imu_ok != 0) && (fabsf(sc->imu_lat_mps2) > sc->cfg.lat_acc_meas_max_mps2)) {
        a = no_accel(a);                                          /* TSR-11 */
        sc->pending_flags2 |= SC_FLAG2_LATACC_MEAS;
    }
    if ((imu_ok != 0) && (sc->mismatch_run >= sc->cfg.mismatch_frames)) {
        a = no_accel(a);                                          /* TSR-12 */
        sc->pending_flags2 |= SC_FLAG2_SENSOR_MISMATCH;
    }
    if ((sc->cfg.steer_track_enable != 0u) && (sc->track_err_ms >= sc->cfg.steer_track_ms)) {
        a = no_accel(a);                                          /* TSR-13 */
        sc->pending_flags2 |= SC_FLAG2_STEER_TRACK;
    }
    return a;
}

/* 상태별 목표 조향/가감속 (SM) */
static void select_targets(const sc_t *sc, float *steer_t, float *accel_t)
{
    float s;
    float a;

    switch (sc->state) {
    case SC_STATE_NORMAL:
        s = sc->cmd_steer_deg;
        a = sc->cmd_accel_mps2;
        break;
    case SC_STATE_DEGRADED:                       /* 조향은 유지, 가속은 금지 */
        s = sc->cmd_steer_deg;
        a = (sc->cmd_accel_mps2 > 0.0f) ? 0.0f : sc->cmd_accel_mps2;
        break;
    case SC_STATE_SAFE_STOP:                      /* 조향 유지, 규정 감속 */
        s = sc->out_steer_deg;
        a = sc->cfg.safe_decel_mps2;
        break;
    case SC_STATE_INIT:
    default:
        s = 0.0f;
        a = 0.0f;
        break;
    }
    *steer_t = s;
    *accel_t = a;
}

/* TSR-05: 속도 기반 횡가속도 한계 (a_lat = v^2 * tan(delta) / L). limit 보다 작으면 그 값 */
static float lat_acc_limit(const sc_t *sc, float limit)
{
    float r = limit;
    float v_mps = sc->speed_kph * KPH2MPS;

    if (v_mps > 1.0f) {
        float lat_lim = atanf((sc->cfg.lat_acc_max_mps2 * sc->cfg.wheelbase_m) / (v_mps * v_mps)) * RAD2DEG;

        if (lat_lim < r) {
            r = lat_lim;
        }
    }
    return r;
}

/* TSR-02: 조향 변화율 제한. 출력(sc->out_steer_deg)을 목표 쪽으로 최대 rate x dt 만큼 움직인다 */
static void rate_limit(sc_t *sc, float steer_t, float dt_s)
{
    float max_delta = sc->cfg.steer_rate_dps * dt_s;
    float delta = steer_t - sc->out_steer_deg;

    if (delta > max_delta) {
        sc->out_steer_deg += max_delta;
        sc->pending_flags |= SC_FLAG_RATE_LIMIT;
    } else if (delta < -max_delta) {
        sc->out_steer_deg -= max_delta;
        sc->pending_flags |= SC_FLAG_RATE_LIMIT;
    } else {
        sc->out_steer_deg = steer_t;
    }
}

void sc_step(sc_t *sc, uint32_t now_ms, sc_out_t *out)
{
    float dt_s = 0.0f;
    float steer_t;
    float accel_t;
    float limit;
    float dt_ms_f;
    uint32_t age;

    if (sc->step_seen != 0u) {
        dt_s = (float)(uint32_t)(now_ms - sc->last_step_ms) * 0.001f;
    }
    sc->step_seen = 1u;
    sc->last_step_ms = now_ms;

    check_timeout(sc, now_ms);                                        /* TSR-03 */
    check_speed_timeout(sc, now_ms);                                  /* TSR-09 */
    select_targets(sc, &steer_t, &accel_t);                           /* SM */
    accel_t = clamp_accel(sc, accel_t);                               /* TSR-01 */
    dt_ms_f = (dt_s * 1000.0f) + 0.5f;
    steer_track_update(sc, now_ms, (uint32_t)dt_ms_f, out);              /* TSR-13 */
    accel_t = vehicle_checks(sc, now_ms, accel_t);                    /* TSR-10~13 */
    steer_t = clamp_sym(sc, steer_t, sc->cfg.steer_limit_deg, SC_FLAG_STEER_CLAMP);   /* TSR-01 */
    limit = lat_acc_limit(sc, sc->cfg.steer_limit_deg);               /* TSR-05 */
    steer_t = clamp_sym(sc, steer_t, limit, SC_FLAG_LATACC_LIMIT);
    rate_limit(sc, steer_t, dt_s);                                    /* TSR-02 */
    /* 속도가 올라 한계가 내려간 경우에도 출력은 항상 현재 한계 안에 둔다 (변화율보다 우선) */
    sc->out_steer_deg = clamp_sym(sc, sc->out_steer_deg, limit, SC_FLAG_LATACC_LIMIT);

    out->steer_deg = sc->out_steer_deg;
    out->accel_mps2 = accel_t;
    out->state = sc->state;
    out->flags = sc->pending_flags;
    age = (uint32_t)(now_ms - sc->last_rx_ms) / 10u;
    out->rx_age_10ms = (uint8_t)((age > 15u) ? 15u : age);
    out->bus_off_seen = sc->bus_off_seen;
    out->flags2 = sc->pending_flags2;
    out->inputs_fresh = (uint8_t)(((sc->speed_lost == 0u) && (sc->speed_seen != 0u)) ? SC_IN_SPEED : 0u);
    out->inputs_fresh |= (uint8_t)((input_fresh(sc, sc->imu_seen, sc->last_imu_ms, now_ms) != 0) ? SC_IN_IMU : 0u);
    out->inputs_fresh |= (uint8_t)((input_fresh(sc, sc->pedal_seen, sc->last_pedal_ms, now_ms) != 0) ? SC_IN_PEDAL : 0u);
    out->inputs_fresh |= (uint8_t)((input_fresh(sc, sc->sas_seen, sc->last_sas_ms, now_ms) != 0) ? SC_IN_SAS : 0u);
    out->ctrl_alive_echo = sc->alive_prev;
    sc->out_accel_mps2 = accel_t;
    sc->pending_flags = 0;
    sc->pending_flags2 = 0;
}

void sc_pack_out(const sc_out_t *out, uint8_t alive, uint8_t data[8])
{
    put_i16(&data[0], out->steer_deg);
    put_i16(&data[2], out->accel_mps2);
    data[4] = (uint8_t)(((uint8_t)out->state & 0x0Fu) | (uint8_t)((out->rx_age_10ms & 0x0Fu) << 4));
    data[5] = out->flags;
    data[6] = (uint8_t)((alive & 0x0Fu) | (out->bus_off_seen ? 0x10u : 0x00u));
    sc_frame_seal(data, SC_DATA_ID_SAFE_OUT);
}

void sc_pack_stat(const sc_out_t *out, uint8_t alive, uint8_t data[8])
{
    data[0] = out->flags2;
    data[1] = (uint8_t)((out->ctrl_alive_echo & 0x0Fu) | (uint8_t)((out->inputs_fresh & 0x0Fu) << 4));
    put_i16(&data[2], out->steer_track_err_deg);
    put_i16(&data[4], out->lat_acc_meas_mps2);
    data[6] = (uint8_t)(alive & 0x0Fu);
    sc_frame_seal(data, SC_DATA_ID_SAFE_STAT);
}
