/* 솔버 실패 대응 통합 시험 (SWR-45, TC-FS-05). MPC 코어 + 통합 계층 + 안전 제어기, 시험대 훅(BS_TEST_HOOK, BMPC_TEST_HOOK).
 * 보드와 같은 0x7F0 결함 프레임(종류 4 = 풀이 실패)으로 실패를 넣는다. HAL 은 가짜 시계와 송신 기록이다.
 *   1. 결함 없음: 8 s 동안 NORMAL, 풀이 실패 응답 0
 *   2. 100 ms 실패(2 주기): 응답 status 3, 그동안 직전 조향 유지, SAFE_STOP 없음, 끝나면 경로를 다시 따른다
 *   3. 1 s 실패(20 주기): 첫 실패 응답부터 290 ms 안에 SAFE_STOP, 정지 중 조향 유지·감속 -3 m/s2, 끝나도 SAFE_STOP 래치
 *   4. 메인 루프 멈춤(종류 3) 결함 표시는 MPC 프레임 처리에서 지우지 않는다(main.c 가 처리)
 * 시나리오: 10 m/s 로 R30 곡선 2 s, 결함 시작과 같은 주기부터 경로가 직선(곡선 출구). 한 주기 50 ms = 요청 + 안전 주기 5 번 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmpc_safety.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                            \
  do {                                              \
    if (cond) {                                     \
      ++g_pass;                                     \
    } else {                                        \
      ++g_fail;                                     \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                          \
      printf("\n");                                 \
    }                                               \
  } while (0)

static int64_t g_us;
static uint32_t g_ms;
static bmpc_can_frame_t g_out, g_resp[BMPC_RESP_FRAMES];
static int64_t fake_now(void *ctx) { (void)ctx; return g_us; }
static int fake_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  if (f->id == BS_ID_SAFE_OUT) g_out = *f;
  if (f->id >= BMPC_ID_RESP_BASE && f->id < BMPC_ID_RESP_BASE + (uint32_t)BMPC_RESP_FRAMES) g_resp[f->id - BMPC_ID_RESP_BASE] = *f;
  return 0;
}
static int16_t i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

static void send_config(bs_app_t *a)
{
  bmpc_config_t cfg;
  bmpc_can_frame_t f;
  uint16_t i;
  bmpc_config_default(&cfg);
  cfg.sqp_iters = 2; /* HILS 운영 설정 */
  cfg.qp_max_iters = 16;
  for (i = 0; i < BMPC_CONFIG_COUNT; ++i) {
    bmpc_encode_cfg_set(i, bmpc_config_get(&cfg, i), &f);
    bs_on_mpc_frame(a, &f);
  }
  bmpc_encode_cfg_commit(bmpc_config_hash(&cfg), (uint16_t)BMPC_CONFIG_COUNT, &f);
  bs_on_mpc_frame(a, &f);
}

static void send_esc(bs_app_t *a, float kph)
{
  static uint8_t alive = 0;
  bmpc_can_frame_t f;
  const uint16_t raw = (uint16_t)(kph * 100.0f + 0.5f);
  memset(&f, 0, sizeof(f));
  f.id = BS_ID_ESC_SPEED;
  f.dlc = 8;
  f.data[0] = (uint8_t)raw;
  f.data[1] = (uint8_t)(raw >> 8);
  f.data[6] = (uint8_t)(alive++ & 0x0F);
  sc_frame_seal(f.data, SC_DATA_ID_ESC_SPEED);
  bs_on_safety_frame(a, &f, g_ms);
}

static void send_hook(bs_app_t *a, uint8_t kind, uint16_t ms)
{
  bmpc_can_frame_t f;
  memset(&f, 0, sizeof(f));
  f.id = BS_ID_BENCH_RESET;
  f.dlc = 6;
  f.data[0] = BS_HOOK_MAGIC;
  f.data[1] = kind;
  f.data[4] = (uint8_t)(ms & 0xFFu);
  f.data[5] = (uint8_t)(ms >> 8);
  bs_on_safety_frame(a, &f, g_ms);
}

typedef struct {
  int status;
  float mpc_deg, safe_deg, safe_acc;
  int state;
} cyc_t;

/* 요청 1 회 (50 ms) */
static cyc_t cycle(bs_app_t *a, uint16_t seq, double kappa, double v)
{
  bmpc_request_t r;
  bmpc_response_t resp;
  bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
  cyc_t c;
  int k;
  memset(&r, 0, sizeof(r));
  r.seq = seq;
  r.c[2] = (bmpc_real)(0.5 * kappa);
  r.x_max = 20;
  r.ref_v = (bmpc_real)v;
  r.v_ub = (bmpc_real)(v + 1);
  r.post_cap = -1;
  r.snap = -1;
  r.velocity = (bmpc_real)v;
  r.yaw_rate = (bmpc_real)(v * kappa);
  r.period = (bmpc_real)0.05;
  r.odom_age = (bmpc_real)0.002;
  memset(g_resp, 0, sizeof(g_resp));
  bmpc_encode_request(&r, fr);
  for (k = 0; k < BMPC_REQ_FRAMES; ++k) bs_on_mpc_frame(a, &fr[k]);
  (void)bs_feed_ctrl(a, g_ms);
  (void)bmpc_decode_response(g_resp, &resp);
  for (k = 0; k < 5; ++k) {
    g_ms += 10;
    g_us += 10000;
    if (k == 0) send_esc(a, (float)(v * 3.6));
    bs_safety_tick(a, g_ms);
    bs_note_applied(a, g_us);
  }
  c.status = resp.status;
  c.mpc_deg = (float)resp.steer_cmd * 57.29578f;
  c.safe_deg = i16(&g_out.data[0]) * 0.01f;
  c.safe_acc = i16(&g_out.data[2]) * 0.01f;
  c.state = (int)a->out.state;
  return c;
}

/* 결함 길이 fail_ms 로 한 번 달린다. 결과를 돌려준다 */
typedef struct {
  int n_fail_resp, first_fail_cycle, stop_cycle, max_state_during, hold_ok, final_state;
  float steer_before, stop_acc, steer_after;
} run_t;

static run_t run(uint16_t fail_ms, int verbose)
{
  static bs_app_t a;
  bs_cfg_t bcfg;
  sc_config_t scfg;
  bmpc_hal_t hal;
  run_t o;
  const int T_FAULT = 40, T_END = 160;
  int c;
  memset(&o, 0, sizeof(o));
  o.first_fail_cycle = -1;
  o.stop_cycle = -1;
  o.hold_ok = 1;
  g_us = 1000000;
  g_ms = 1000;
  hal.now_us = fake_now;
  hal.can_send = fake_send;
  hal.ctx = NULL;
  bs_default_cfg(&bcfg);
  bs_default_safety_config(&scfg);
  bs_init(&a, &bcfg, &scfg, &hal);
  send_config(&a);
  for (c = 0; c < T_END; ++c) {
    cyc_t y;
    if (c == T_FAULT && fail_ms > 0u) send_hook(&a, BS_HOOK_SOLVE_FAIL, fail_ms);
    y = cycle(&a, (uint16_t)(c + 1), c < T_FAULT ? 1.0 / 30.0 : 0.0, 10.0);
    if (c == T_FAULT - 1) o.steer_before = y.safe_deg;
    if (y.status == BMPC_ERR_NUMERIC) {
      ++o.n_fail_resp;
      if (o.first_fail_cycle < 0) o.first_fail_cycle = c;
      if (fabsf(y.mpc_deg - o.steer_before) > 0.02f) o.hold_ok = 0;
      if (y.state > o.max_state_during) o.max_state_during = y.state;
    }
    if (y.state == SC_STATE_SAFE_STOP && o.stop_cycle < 0) {
      o.stop_cycle = c;
      o.stop_acc = y.safe_acc;
    }
    if (verbose && c >= T_FAULT - 1 && c <= T_FAULT + 6)
      printf("    cycle %3d status %d mpc %.2f deg  safe state %d steer %.2f deg accel %.2f\n", c, y.status, y.mpc_deg,
             y.state, y.safe_deg, y.safe_acc);
    o.steer_after = y.safe_deg;
    o.final_state = y.state;
  }
  return o;
}

int main(void)
{
  run_t r;

  printf("[no fault]\n");
  r = run(0, 0);
  CHECK(r.n_fail_resp == 0, "no failed responses (%d)", r.n_fail_resp);
  CHECK(r.stop_cycle < 0 && r.final_state == SC_STATE_NORMAL, "stays NORMAL (final %d)", r.final_state);
  CHECK(r.steer_before > 7.0f && r.steer_before < 9.0f, "steady curve steering %.2f deg", r.steer_before);

  printf("[100 ms solver failure]\n");
  r = run(100, 1);
  CHECK(r.n_fail_resp == 2, "two failed responses (%d)", r.n_fail_resp);
  CHECK(r.hold_ok, "previous steering held while failing");
  CHECK(r.stop_cycle < 0 && r.final_state == SC_STATE_NORMAL, "no SAFE_STOP (stop cycle %d, final %d)", r.stop_cycle,
        r.final_state);
  CHECK(fabsf(r.steer_after) < 0.1f, "follows the straight path again (%.2f deg)", r.steer_after);

  printf("[1 s solver failure]\n");
  r = run(1000, 1);
  CHECK(r.n_fail_resp == 20, "twenty failed responses (%d)", r.n_fail_resp);
  CHECK(r.hold_ok, "previous steering held while failing");
  CHECK(r.stop_cycle >= 0, "SAFE_STOP reached");
  /* 응답은 주기 시작에 나가고 SAFE_STOP 은 그 주기의 안전 주기(최대 50 ms 뒤)에 보인다 -> 상한 (주기 차 + 1) x 50 ms */
  CHECK(r.stop_cycle >= 0 && (r.stop_cycle - r.first_fail_cycle + 1) * 50 <= 290, "SAFE_STOP within 290 ms of the first failure (%d ms)",
        (r.stop_cycle - r.first_fail_cycle + 1) * 50);
  CHECK(r.stop_acc <= -2.9f, "decelerates in SAFE_STOP (%.2f m/s2)", r.stop_acc);
  CHECK(r.final_state == SC_STATE_SAFE_STOP, "SAFE_STOP stays latched after the solver recovers (final %d)", r.final_state);
  printf("  failed responses %d, SAFE_STOP at +%d ms, accel %.2f, steering held %.2f deg\n", r.n_fail_resp,
         (r.stop_cycle - r.first_fail_cycle + 1) * 50, r.stop_acc, r.steer_after);

  printf("[watchdog hang hook stays pending for the main loop]\n");
  {
    static bs_app_t a;
    bs_cfg_t bcfg;
    sc_config_t scfg;
    bmpc_hal_t hal;
    hal.now_us = fake_now;
    hal.can_send = fake_send;
    hal.ctx = NULL;
    bs_default_cfg(&bcfg);
    bs_default_safety_config(&scfg);
    bs_init(&a, &bcfg, &scfg, &hal);
    send_config(&a);
    send_hook(&a, 3u, 0u);
    (void)cycle(&a, 1, 0.0, 10.0);
    CHECK(a.hook_pending && a.hook_kind == 3u, "kind 3 still pending after an MPC request (pending %d)", a.hook_pending);
  }

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
