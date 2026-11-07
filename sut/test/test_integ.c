/* MPC + 안전 제어기 통합 계층 (integ/bmpc_safety) 시험. HAL 은 가짜 시계와 송신 기록이다.
 *   1. 정상 흐름: 설정 -> 차속 -> 요청마다 CTRL_CMD -> NORMAL, SAFE_OUT = MPC 명령, SAFE_STAT Alive 에코 = seq
 *   2. 조향 급변: 안전 제어기가 변화율로 자르면 실제 출력이 MPC 명령 이력에 들어간다
 *   3. 조향 명령 결함 (BS_TEST_HOOK): 더하기와 고정이 CTRL_CMD 에 들어가고, 안전 제어기가 변화율로 자른다
 *   4. 요청이 끊기면 (명령 타임아웃 200 ms) SAFE_STOP */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmpc_safety.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                    \
  do {                                                      \
    if (cond) {                                             \
      ++g_pass;                                             \
    } else {                                                \
      ++g_fail;                                             \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);         \
      printf(__VA_ARGS__);                                  \
      printf("\n");                                         \
    }                                                       \
  } while (0)

static int64_t g_us = 1000000;
static uint32_t g_ms = 1000;
static bmpc_can_frame_t g_last_out, g_last_stat, g_last_resp0;
static int g_n_out = 0;

static int64_t fake_now(void *ctx) { (void)ctx; return g_us; }
static int fake_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  if (f->id == BS_ID_SAFE_OUT) { g_last_out = *f; ++g_n_out; }
  if (f->id == BS_ID_SAFE_STAT) g_last_stat = *f;
  if (f->id == BMPC_ID_RESP_BASE) g_last_resp0 = *f;
  return 0;
}

static int16_t i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

static void send_config(bs_app_t *a)
{
  bmpc_config_t cfg;
  bmpc_can_frame_t f;
  uint16_t i;
  bmpc_config_default(&cfg);
  cfg.sqp_iters = 2;
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

static void make_req(bmpc_request_t *r, uint16_t seq, double c0, double v)
{
  memset(r, 0, sizeof(*r));
  r->seq = seq;
  r->c[0] = c0;
  r->x_max = 20;
  r->ref_v = v;
  r->v_ub = v + 1;
  r->post_cap = -1;
  r->snap = -1;
  r->velocity = v;
  r->period = 0.044;
  r->odom_age = 0.01;
}

/* 요청 1 회 (50 ms): 요청 -> 응답 -> CTRL_CMD 넣기 -> 안전 주기 5 번 (10 ms 마다), 그 사이 차속 */
static void cycle(bs_app_t *a, uint16_t seq, double c0, double v, int with_request)
{
  int k;
  if (with_request) {
    bmpc_request_t r;
    bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
    make_req(&r, seq, c0, v);
    bmpc_encode_request(&r, fr);
    for (k = 0; k < BMPC_REQ_FRAMES; ++k) bs_on_mpc_frame(a, &fr[k]);
    (void)bs_feed_ctrl(a, g_ms);
  }
  for (k = 0; k < 5; ++k) {
    g_ms += 10;
    g_us += 10000;
    if (k == 0) send_esc(a, (float)(v * 3.6));
    bs_safety_tick(a, g_ms);
    bs_note_applied(a, g_us);
  }
}

int main(void)
{
  static bs_app_t a;
  bs_cfg_t bcfg;
  sc_config_t scfg;
  bmpc_hal_t hal;
  uint16_t seq = 1;
  int k;

  hal.now_us = fake_now;
  hal.can_send = fake_send;
  hal.ctx = NULL;
  bs_default_cfg(&bcfg);
  bs_default_safety_config(&scfg);
  bs_init(&a, &bcfg, &scfg, &hal);
  send_config(&a);
  CHECK(a.app.ctrl.configured, "configured");

  printf("[normal]\n");
  for (k = 0; k < 10; ++k, ++seq) cycle(&a, seq, 0.05, 8.0, 1);
  {
    bmpc_response_t r;
    const float out_deg = i16(&g_last_out.data[0]) * 0.01f;
    memset(&r, 0, sizeof(r));
    CHECK(a.out.state == SC_STATE_NORMAL, "state NORMAL (got %d)", (int)a.out.state);
    CHECK((g_last_stat.data[1] & 0x0F) == ((seq - 1) & 0x0F), "alive echo %d vs seq %d", g_last_stat.data[1] & 0x0F, (seq - 1) & 0x0F);
    CHECK(a.n_ctrl == 10, "10 ctrl frames fed (%u)", a.n_ctrl);
    CHECK(fabsf(out_deg - (float)a.app.ctrl.post.prev_steer_cmd * 57.29578f) < 0.02f, "SAFE_OUT steer %.3f = MPC %.3f deg",
          out_deg, (double)a.app.ctrl.post.prev_steer_cmd * 57.29578);
    CHECK((a.out.flags & SC_FLAG_RATE_LIMIT) == 0, "no rate limiting in normal driving (flags 0x%02x)", a.out.flags);
    CHECK(a.n_notes == 0, "nothing to note when the safety output equals the MPC command (%u)", a.n_notes);
    printf("  state %d, SAFE_OUT steer %.3f deg accel %.2f, notes %u\n", (int)a.out.state, out_deg,
           i16(&g_last_out.data[2]) * 0.01f, a.n_notes);
  }

  printf("[steer step -> rate limit -> history]\n");
  {
    const uint32_t notes0 = a.n_notes;
    const int hist0 = a.app.ctrl.hist.count;
    int limited = 0;
    /* 운영 한계(120 deg/s)는 MPC 조향 슬루(103 deg/s)보다 커서 정상 명령에는 걸리지 않는다. 동작 확인용으로 팀 기본값으로 낮춘다 */
    a.sc.cfg.steer_rate_dps = 35.0f;
    for (k = 0; k < 6; ++k, ++seq) {
      cycle(&a, seq, 3.0, 2.0, 1); /* 3 m 옆 경로, 저속: 조향을 크게 꺾는다 */
      if (a.out.flags & SC_FLAG_RATE_LIMIT) limited = 1;
    }
    {
      const int last = (a.app.ctrl.hist.head + a.app.ctrl.hist.count - 1) % BMPC_HIST_MAX;
      const double hist_deg = a.app.ctrl.hist.delta[last] * a.app.ctrl.cfg.steer_sign * 57.29578;
      printf("  rate limited %d, notes +%u, history %d -> %d, last history %.2f deg, SAFE_OUT %.2f deg\n", limited,
             a.n_notes - notes0, hist0, a.app.ctrl.hist.count, hist_deg, a.out.steer_deg);
      CHECK(limited, "safety limited the steering rate");
      CHECK(a.n_notes > notes0, "applied steering noted");
      CHECK(fabs(hist_deg - a.out.steer_deg) < 0.01, "last history = applied steering");
    }
  }

#ifdef BS_TEST_HOOK
  printf("[steering command fault hook]\n");
  {
    const float max_step = 120.0f * 0.05f + 0.1f; /* 통합 프로파일 120 deg/s, 명령 한 주기 */
    float before, out_before, faulted;
    bmpc_can_frame_t f;
    a.sc.cfg.steer_rate_dps = 120.0f;
    for (k = 0; k < 40; ++k, ++seq) cycle(&a, seq, 0.05, 8.0, 1);
    before = i16(&a.ctrl[0]) * 0.01f;
    out_before = a.out.steer_deg;

    memset(&f, 0, sizeof(f));
    f.id = BS_ID_BENCH_RESET;
    f.dlc = 6;
    f.data[0] = BS_HOOK_MAGIC;
    f.data[1] = BS_HOOK_OFFSET;
    f.data[2] = (uint8_t)(3000u & 0xFFu);
    f.data[3] = (uint8_t)(3000u >> 8);
    f.data[4] = 50u;
    bs_on_safety_frame(&a, &f, g_ms);
    cycle(&a, seq++, 0.05, 8.0, 1);
    faulted = i16(&a.ctrl[0]) * 0.01f;
    printf("  offset: CTRL %.2f -> %.2f deg, SAFE_OUT %.2f -> %.2f deg\n", before, faulted, out_before, a.out.steer_deg);
    CHECK(fabsf(faulted - before - 30.0f) < 3.0f, "offset reaches CTRL_CMD (%.2f -> %.2f)", before, faulted);
    CHECK(fabsf(a.out.steer_deg - out_before) <= max_step, "safety limits the output (%.2f -> %.2f)", out_before, a.out.steer_deg);
    CHECK(a.out.flags & SC_FLAG_RATE_LIMIT, "rate limit flag set");
    cycle(&a, seq++, 0.05, 8.0, 1);
    CHECK(fabsf(i16(&a.ctrl[0]) * 0.01f - before) < 3.0f, "50 ms offset lasts one command (%.2f)", i16(&a.ctrl[0]) * 0.01f);

    for (k = 0; k < 40; ++k, ++seq) cycle(&a, seq, 0.05, 8.0, 1);
    before = i16(&a.ctrl[0]) * 0.01f;
    f.data[1] = BS_HOOK_HOLD;
    f.data[2] = f.data[3] = 0u;
    f.data[4] = (uint8_t)(500u & 0xFFu);
    f.data[5] = (uint8_t)(500u >> 8);
    bs_on_safety_frame(&a, &f, g_ms);
    for (k = 0; k < 5; ++k, ++seq) cycle(&a, seq, 3.0, 8.0, 1); /* 경로가 옆으로 옮겨도 */
    printf("  hold: CTRL %.2f -> %.2f deg\n", before, i16(&a.ctrl[0]) * 0.01f);
    CHECK(fabsf(i16(&a.ctrl[0]) * 0.01f - before) < 0.02f, "hold keeps the steering (%.2f)", i16(&a.ctrl[0]) * 0.01f);
    for (k = 0; k < 6; ++k, ++seq) cycle(&a, seq, 3.0, 8.0, 1); /* 500 ms 를 넘긴다 */
    CHECK(fabsf(i16(&a.ctrl[0]) * 0.01f - before) > 0.1f, "hold ends (%.2f)", i16(&a.ctrl[0]) * 0.01f);
    CHECK(a.n_hook == 2u, "two hooks (%u)", a.n_hook);
    for (k = 0; k < 40; ++k, ++seq) cycle(&a, seq, 0.05, 8.0, 1);
  }
#endif

  printf("[requests stop -> SAFE_STOP]\n");
  {
    const int n_out0 = g_n_out;
    for (k = 0; k < 6; ++k) cycle(&a, seq, 0.0, 2.0, 0); /* 300 ms 요청 없음 */
    CHECK(a.out.state == SC_STATE_SAFE_STOP, "SAFE_STOP after command timeout (got %d)", (int)a.out.state);
    CHECK(g_n_out - n_out0 == 30, "SAFE_OUT keeps coming every 10 ms (%d)", g_n_out - n_out0);
    printf("  state %d, accel %.2f m/s2\n", (int)a.out.state, a.out.accel_mps2);
  }

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
