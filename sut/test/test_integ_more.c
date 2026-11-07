/* 통합 계층 (integ/bmpc_safety) 추가 시험. test_integ.c 가 흐름을 본다면 여기서는 경계와 거부 경로를 본다.
 *   1. 안전 프레임 판별, 기본 설정이 운영 영역과 맞는지
 *   2. MPC 응답 -> CTRL_CMD 바이트: 단위, 부호, 반올림, 포화, NaN, CRC
 *   3. 응답 가로채기: 네 프레임이 다 모일 때만, 잘못된 응답·다른 ID 는 상태를 건드리지 않는다
 *   4. 안전 프레임 분배: DLC, ID, 입력별 처리기
 *   5. 시험대 리셋과 결함 주입 프레임 해석
 *   6. 주기 출력: SAFE_OUT·SAFE_STAT 형식, Alive, 상태별 조향 이력 반영, 버스오프 */
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
static bmpc_can_frame_t g_sent[16];
static int g_nsent = 0;
static int g_send_rc = 0;

static bs_app_t g_a, g_snap;

static int64_t fake_now(void *ctx) { (void)ctx; return g_us; }
static int fake_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  g_sent[g_nsent % 16] = *f;
  ++g_nsent;
  return g_send_rc;
}

static const bmpc_hal_t g_hal = {fake_now, fake_send, NULL};

static int16_t i16(const uint8_t *p) { return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static void fresh(bs_app_t *a)
{
  bs_cfg_t bcfg;
  sc_config_t scfg;
  bs_default_cfg(&bcfg);
  bs_default_safety_config(&scfg);
  bs_init(a, &bcfg, &scfg, &g_hal);
  g_nsent = 0;
  g_send_rc = 0;
}

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

/* MPC 응답 프레임은 앱이 내부 HAL 로 보낸다. 시험에서는 그 HAL 을 직접 불러 임의의 순서로 넣는다 */
static int inner_send(bs_app_t *a, const bmpc_can_frame_t *f) { return a->app.hal.can_send(a->app.hal.ctx, f); }

static void make_resp_frames(uint16_t seq, uint8_t status, double steer, double accel, double brake,
                             bmpc_can_frame_t fr[BMPC_RESP_FRAMES])
{
  bmpc_response_t r;
  memset(&r, 0, sizeof(r));
  r.seq = seq;
  r.status = status;
  r.steer_cmd = (bmpc_real)steer;
  r.accel = (bmpc_real)accel;
  r.brake = (bmpc_real)brake;
  bmpc_encode_response(&r, fr);
}

static void send_group(bs_app_t *a, const bmpc_can_frame_t fr[BMPC_RESP_FRAMES])
{
  int i;
  for (i = 0; i < BMPC_RESP_FRAMES; ++i) (void)inner_send(a, &fr[i]);
}

static void seal_frame(bmpc_can_frame_t *f, uint32_t id, uint16_t data_id)
{
  memset(f, 0, sizeof(*f));
  f->id = id;
  f->dlc = 8;
  sc_frame_seal(f->data, data_id);
}

/* 안전 제어기를 NORMAL 로: CTRL_CMD 를 init_valid_frames 번 (Alive 는 매번 바뀐다) */
static void feed_cmd(bs_app_t *a, double steer, double accel, uint16_t seq)
{
  bmpc_response_t r;
  memset(&r, 0, sizeof(r));
  r.seq = seq;
  r.steer_cmd = (bmpc_real)steer;
  r.accel = (bmpc_real)accel;
  bs_make_ctrl_frame(&a->cfg, &r, a->ctrl);
  a->ctrl_pending = 1;
  (void)bs_feed_ctrl(a, g_ms);
}

static void tick(bs_app_t *a)
{
  g_ms += 10;
  g_us += 10000;
  bs_safety_tick(a, g_ms);
}

static void go_normal(bs_app_t *a)
{
  uint16_t s;
  for (s = 1; s <= 3; ++s) feed_cmd(a, 0.0, 0.0, s);
  tick(a);
}

/* ---------------- 1. 판별과 기본 설정 ---------------- */
static void test_ids_and_defaults(void)
{
  static const uint32_t yes[] = {BS_ID_ESC_SPEED, BS_ID_SAS_STEER, BS_ID_VCU_PEDAL, BS_ID_ACU_IMU, BS_ID_BENCH_RESET};
  static const uint32_t no[] = {0u, BMPC_ID_CFG_SET, BMPC_ID_CFG_COMMIT, BMPC_ID_REQ_BASE, BMPC_ID_RESP_BASE,
                                BMPC_ID_CFG_ACK, BS_ID_SAFE_OUT, BS_ID_SAFE_STAT, 0x1A1u, 0x19Fu, 0x1B1u, 0x1C1u,
                                0x1D1u, 0x7F1u, 0x7EFu};
  size_t i;
  bs_cfg_t b;
  sc_config_t s;
  bmpc_config_t m;
  printf("[frame ids, defaults]\n");
  for (i = 0; i < sizeof(yes) / sizeof(yes[0]); ++i)
    CHECK(bs_is_safety_frame(yes[i]) == 1, "0x%03X is a safety frame", (unsigned)yes[i]);
  for (i = 0; i < sizeof(no) / sizeof(no[0]); ++i)
    CHECK(bs_is_safety_frame(no[i]) == 0, "0x%03X is not a safety frame", (unsigned)no[i]);

  /* 값을 베끼지 않고 주석에 적힌 관계를 본다: 정상 MPC 명령에는 개입하지 않는다 */
  bs_default_cfg(&b);
  bs_default_safety_config(&s);
  bmpc_config_default(&m);
  CHECK(b.accel_gain > 0.0f && b.brake_gain > 0.0f, "pedal gains positive");
  CHECK((double)s.accel_max_mps2 >= (double)b.accel_gain, "full throttle fits the accel limit");
  CHECK((double)s.accel_min_mps2 <= -(double)b.brake_gain, "full brake fits the decel limit");
  CHECK((double)s.steer_rate_dps > (double)m.st_rate_limit * 57.29578, "safety rate above the MPC slew (%.1f deg/s)",
        (double)s.steer_rate_dps);
  CHECK((double)s.steer_limit_deg > (double)m.steer_ub * 57.29578, "safety steer limit above the MPC bound");
  CHECK(fabs((double)s.wheelbase_m - (double)m.Lf) < 1e-6, "wheelbase = MPC Lf");
  CHECK(s.cmd_timeout_ms >= 2u * 50u, "a single missed 50 ms command must not time out (%u ms)", (unsigned)s.cmd_timeout_ms);
}

/* ---------------- 2. CTRL_CMD 바이트 ---------------- */
static void test_ctrl_frame(void)
{
  bs_cfg_t c = {3.0f, 5.0f};
  bmpc_response_t r;
  uint8_t d[8];
  sc_t sc;
  sc_config_t scfg;
  printf("[CTRL_CMD encoding]\n");

  memset(&r, 0, sizeof(r));
  r.seq = 0x1234u;
  r.steer_cmd = (bmpc_real)0.1; /* 5.7296 deg */
  r.accel = (bmpc_real)0.5;     /* 1.5 m/s2 */
  memset(d, 0xAA, sizeof(d));
  bs_make_ctrl_frame(&c, &r, d);
  CHECK(i16(&d[0]) == 573, "steer 0.1 rad -> %d (0.01 deg)", i16(&d[0]));
  CHECK(i16(&d[2]) == 150, "accel 0.5 pedal -> %d (0.01 m/s2)", i16(&d[2]));
  CHECK(d[4] == 0u && d[5] == 0u, "unused bytes cleared");
  CHECK(d[6] == 0x04u, "alive = low nibble of seq (0x%02X)", d[6]);
  CHECK(d[7] == sc_crc8_e2e(SC_DATA_ID_CTRL_CMD, d, 7u), "CTRL_CMD data id CRC");
  CHECK(d[7] != sc_crc8_e2e(SC_DATA_ID_SAFE_OUT, d, 7u), "CRC depends on the data id");

  /* 안전 제어기가 같은 값으로 읽는다 */
  bs_default_safety_config(&scfg);
  sc_init(&sc, &scfg);
  sc_on_ctrl_frame(&sc, d, 1000u);
  CHECK(fabsf(sc.cmd_steer_deg - 5.73f) < 1e-3f && fabsf(sc.cmd_accel_mps2 - 1.5f) < 1e-3f, "safety reads %.2f deg %.2f m/s2",
        (double)sc.cmd_steer_deg, (double)sc.cmd_accel_mps2);
  CHECK((sc.pending_flags & SC_FLAG_CRC_ERR) == 0u && sc.alive_prev == 4u, "accepted by the safety controller");

  /* 페달 -> 가감속: a = accel * accel_gain - brake * brake_gain, gain 은 설정에서 */
  r.accel = 0;
  r.brake = (bmpc_real)0.4;
  r.steer_cmd = (bmpc_real)-0.1;
  bs_make_ctrl_frame(&c, &r, d);
  CHECK(i16(&d[0]) == -573, "negative steer %d", i16(&d[0]));
  CHECK(i16(&d[2]) == -200, "brake 0.4 -> %d", i16(&d[2]));
  {
    bs_cfg_t c2 = {2.0f, 4.0f};
    r.accel = (bmpc_real)0.5;
    r.brake = (bmpc_real)0.25;
    bs_make_ctrl_frame(&c2, &r, d);
    CHECK(i16(&d[2]) == 0, "both pedals cancel with the configured gains (%d)", i16(&d[2]));
  }

  /* 반올림은 0 에서 먼 쪽으로 대칭 */
  {
    bs_cfg_t c1 = {1.0f, 1.0f};
    memset(&r, 0, sizeof(r));
    r.accel = (bmpc_real)0.0156;
    bs_make_ctrl_frame(&c1, &r, d);
    CHECK(i16(&d[2]) == 2, "1.56 rounds to 2 (%d)", i16(&d[2]));
    r.accel = (bmpc_real)0.0144;
    bs_make_ctrl_frame(&c1, &r, d);
    CHECK(i16(&d[2]) == 1, "1.44 rounds to 1 (%d)", i16(&d[2]));
    r.accel = 0;
    r.brake = (bmpc_real)0.0156;
    bs_make_ctrl_frame(&c1, &r, d);
    CHECK(i16(&d[2]) == -2, "-1.56 rounds to -2 (%d)", i16(&d[2]));
    r.brake = (bmpc_real)0.0144;
    bs_make_ctrl_frame(&c1, &r, d);
    CHECK(i16(&d[2]) == -1, "-1.44 rounds to -1 (%d)", i16(&d[2]));
  }

  /* 포화: 큰 값이 부호를 뒤집어 거꾸로 나가면 안 된다 */
  memset(&r, 0, sizeof(r));
  r.steer_cmd = (bmpc_real)1000.0;
  r.accel = (bmpc_real)1e6;
  bs_make_ctrl_frame(&c, &r, d);
  CHECK(i16(&d[0]) == 32767 && i16(&d[2]) == 32767, "positive saturation %d %d", i16(&d[0]), i16(&d[2]));
  r.steer_cmd = (bmpc_real)-1000.0;
  r.accel = 0;
  r.brake = (bmpc_real)1e6;
  bs_make_ctrl_frame(&c, &r, d);
  CHECK(i16(&d[0]) == -32768 && i16(&d[2]) == -32768, "negative saturation %d %d", i16(&d[0]), i16(&d[2]));
  CHECK(d[7] == sc_crc8_e2e(SC_DATA_ID_CTRL_CMD, d, 7u), "CRC valid at saturation");

  /* NaN 은 0, 무한대는 포화 */
  memset(&r, 0, sizeof(r));
  r.steer_cmd = (bmpc_real)NAN;
  r.accel = (bmpc_real)HUGE_VAL;
  bs_make_ctrl_frame(&c, &r, d);
  CHECK(i16(&d[0]) == 0, "NaN steer -> 0 (%d)", i16(&d[0]));
  CHECK(i16(&d[2]) == 32767, "inf accel saturates (%d)", i16(&d[2]));
  r.steer_cmd = (bmpc_real)-HUGE_VAL;
  r.accel = (bmpc_real)HUGE_VAL;
  r.brake = (bmpc_real)HUGE_VAL;
  bs_make_ctrl_frame(&c, &r, d);
  CHECK(i16(&d[0]) == -32768, "-inf steer saturates (%d)", i16(&d[0]));
  CHECK(i16(&d[2]) == 0, "inf - inf = NaN accel -> 0 (%d)", i16(&d[2]));
}

/* ---------------- 3. 응답 가로채기 ---------------- */
static void test_intercept(void)
{
  bmpc_can_frame_t fr[BMPC_RESP_FRAMES], other[BMPC_RESP_FRAMES], f;
  static const uint32_t foreign[] = {BMPC_ID_RESP_BASE - 1u, BMPC_ID_RESP_BASE + BMPC_RESP_FRAMES, BMPC_ID_CFG_ACK,
                                     BMPC_ID_CFG_SET, BMPC_ID_REQ_BASE, 0u, 0x7FFu};
  uint8_t expect[8];
  bmpc_response_t r;
  size_t i;
  int st, n0;
  printf("[response interception]\n");

  fresh(&g_a);
  CHECK(g_a.out.state == SC_STATE_INIT && g_a.app.ctrl.configured == 0, "init state");
  CHECK(g_a.app.hal.ctx == (void *)&g_a && g_a.app.hal.can_send != fake_send, "app sends through the integration layer");
  CHECK(g_a.sc.cfg.cmd_timeout_ms == 200u && fabsf(g_a.cfg.brake_gain - 5.0f) < 1e-6f, "configs copied");

  /* 네 프레임이 모이면 CTRL_CMD 하나, 플랫폼으로도 그대로 나간다 */
  make_resp_frames(0x0127u, BMPC_OK, 0.1, 0.2, 0.0, fr);
  for (i = 0; i < BMPC_RESP_FRAMES; ++i) {
    n0 = g_nsent;
    g_send_rc = (int)i + 5;
    st = inner_send(&g_a, &fr[i]);
    CHECK(st == (int)i + 5, "platform send result returned (%d)", st);
    CHECK(g_nsent == n0 + 1 && g_sent[n0 % 16].id == fr[i].id, "frame %d forwarded", (int)i);
    CHECK(g_a.ctrl_pending == (i == BMPC_RESP_FRAMES - 1 ? 1 : 0), "pending only after the last frame (i=%d)", (int)i);
  }
  g_send_rc = 0;
  CHECK(g_a.resp_mask == 0u, "mask cleared after a full group");
  bmpc_decode_response(fr, &r);
  bs_make_ctrl_frame(&g_a.cfg, &r, expect);
  CHECK(memcmp(expect, g_a.ctrl, 8) == 0, "CTRL_CMD = the response");

  /* 한 번 쓴 프레임을 다시 조합하면 안 된다: 마지막 프레임만 다시 와도 명령이 생기지 않는다 */
  g_a.ctrl_pending = 0;
  (void)inner_send(&g_a, &fr[3]);
  CHECK(g_a.ctrl_pending == 0, "a repeated last frame alone gives no command");
  (void)inner_send(&g_a, &fr[0]);
  (void)inner_send(&g_a, &fr[1]);
  CHECK(g_a.ctrl_pending == 0, "two frames give no command");
  (void)inner_send(&g_a, &fr[3]);
  CHECK(g_a.ctrl_pending == 0, "a missing frame gives no command");

  /* 첫 프레임이 오면 이전 조각은 버리고 새로 모은다 */
  make_resp_frames(0x0128u, BMPC_OK, -0.1, 0.0, 0.1, other);
  (void)inner_send(&g_a, &other[0]);
  CHECK(g_a.resp_mask == 1u, "first frame restarts the group (mask 0x%02X)", g_a.resp_mask);
  (void)inner_send(&g_a, &other[1]);
  (void)inner_send(&g_a, &other[2]);
  (void)inner_send(&g_a, &other[3]);
  CHECK(g_a.ctrl_pending == 1 && (g_a.ctrl[6] & 0x0Fu) == 0x08u, "new group gives a new command (alive %d)", g_a.ctrl[6] & 0x0F);

  /* 다른 묶음의 프레임이 섞이면 CRC 로 걸러진다 */
  fresh(&g_a);
  (void)inner_send(&g_a, &fr[0]);
  (void)inner_send(&g_a, &other[1]);
  (void)inner_send(&g_a, &fr[2]);
  (void)inner_send(&g_a, &fr[3]);
  CHECK(g_a.ctrl_pending == 0, "mixed groups fail the CRC");

  /* CRC 가 깨진 응답 (decode 가 status 0 으로 멈춘다) */
  make_resp_frames(9u, BMPC_OK, 0.1, 0.2, 0.0, fr);
  fr[1].data[0] ^= 0x01u;
  fresh(&g_a);
  send_group(&g_a, fr);
  CHECK(g_a.ctrl_pending == 0, "bad CRC gives no command");

  /* 수신 오류·설정 전 응답에는 명령이 없다. 풀이 실패에는 유지한 명령이 있다 */
  {
    static const struct {
      uint8_t status;
      int pending;
    } cases[] = {{BMPC_OK, 1},
                 {BMPC_ERR_MAXITER, 1},
                 {BMPC_ERR_INFEASIBLE, 1},
                 {BMPC_ERR_NUMERIC, 1},
                 {BMPC_ERR_RX, 0},
                 {BMPC_ERR_NOT_CONFIGURED, 0}};
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
      fresh(&g_a);
      make_resp_frames(1u, cases[i].status, 0.1, 0.0, 0.0, fr);
      send_group(&g_a, fr);
      CHECK(g_a.ctrl_pending == cases[i].pending, "status %u -> pending %d", cases[i].status, g_a.ctrl_pending);
      CHECK(bs_feed_ctrl(&g_a, g_ms) == cases[i].pending, "feed status %u", cases[i].status);
      CHECK(g_a.n_ctrl == (uint32_t)cases[i].pending, "n_ctrl status %u", cases[i].status);
    }
  }

  /* 응답이 아닌 ID: 그대로 플랫폼으로 나가고 통합 계층 상태는 그대로다 */
  fresh(&g_a);
  make_resp_frames(1u, BMPC_OK, 0.1, 0.0, 0.0, fr);
  (void)inner_send(&g_a, &fr[0]);
  for (i = 0; i < sizeof(foreign) / sizeof(foreign[0]); ++i) {
    memset(&f, 0x5A, sizeof(f));
    f.id = foreign[i];
    f.dlc = 8;
    n0 = g_nsent;
    memcpy(&g_snap, &g_a, sizeof(g_a));
    (void)inner_send(&g_a, &f);
    CHECK(g_nsent == n0 + 1, "0x%03X forwarded", (unsigned)foreign[i]);
    CHECK(memcmp(&g_snap, &g_a, sizeof(g_a)) == 0, "0x%03X leaves the state alone", (unsigned)foreign[i]);
  }
  (void)inner_send(&g_a, &fr[1]);
  (void)inner_send(&g_a, &fr[2]);
  (void)inner_send(&g_a, &fr[3]);
  CHECK(g_a.ctrl_pending == 1, "group survives foreign frames in between");
}

/* 앱을 거치는 경로: 설정 전, 깨진 요청, 수치 오류 */
static void test_request_paths(void)
{
  bmpc_request_t req;
  bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
  int i, k;
  uint16_t seq = 1;
  float held;
  printf("[request paths]\n");

  fresh(&g_a);
  make_req(&req, 40, 0.0, 8.0);
  bmpc_encode_request(&req, fr);
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bs_on_mpc_frame(&g_a, &fr[i]);
  CHECK(g_nsent == BMPC_RESP_FRAMES, "NOT_CONFIGURED response goes to the PC (%d frames)", g_nsent);
  CHECK(g_a.ctrl_pending == 0, "no command before configuration");
  CHECK(bs_feed_ctrl(&g_a, g_ms) == 0 && g_a.n_ctrl == 0u, "nothing to feed");

  send_config(&g_a);
  CHECK(g_a.app.ctrl.configured == 1, "configured");
  for (k = 0; k < 5; ++k) {
    make_req(&req, seq++, 1.0, 8.0);
    bmpc_encode_request(&req, fr);
    for (i = 0; i < BMPC_REQ_FRAMES; ++i) bs_on_mpc_frame(&g_a, &fr[i]);
    CHECK(g_a.ctrl_pending == 1, "command after a good request");
    CHECK(bs_feed_ctrl(&g_a, g_ms) == 1, "fed once");
    CHECK(bs_feed_ctrl(&g_a, g_ms) == 0, "second feed has nothing");
    g_us += 50000;
    g_ms += 50;
  }
  CHECK(g_a.n_ctrl == 5u, "n_ctrl %u", (unsigned)g_a.n_ctrl);
  CHECK(g_a.sc.alive_prev == ((seq - 1u) & 0x0Fu), "safety saw seq %u", (unsigned)g_a.sc.alive_prev);

  /* CRC 가 깨진 요청: RX 오류 응답만 가고 명령은 없다 */
  make_req(&req, seq++, 1.0, 8.0);
  bmpc_encode_request(&req, fr);
  fr[2].data[0] ^= 0x40u;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bs_on_mpc_frame(&g_a, &fr[i]);
  CHECK(g_a.app.stats.rx_errors == 1u && g_a.ctrl_pending == 0, "RX error gives no command");

  /* NaN 경로 계수: 풀지 않지만 직전 조향을 CTRL_CMD 로 유지한다 */
  held = (float)g_a.app.ctrl.post.prev_steer_cmd;
  CHECK(fabsf(held) > 0.01f, "steering built up (%.3f rad)", (double)held);
  make_req(&req, seq++, 1.0, 8.0);
  req.c[1] = (bmpc_real)NAN;
  bmpc_encode_request(&req, fr);
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bs_on_mpc_frame(&g_a, &fr[i]);
  CHECK(g_a.app.ctrl.errors == 1u, "numeric error counted");
  CHECK(g_a.ctrl_pending == 1, "held command still goes to the safety controller");
  CHECK(fabsf(i16(&g_a.ctrl[0]) * 0.01f - held * 57.29578f) < 0.011f, "held steer %.3f deg vs %.3f deg", i16(&g_a.ctrl[0]) * 0.01,
        (double)held * 57.29578);
}

/* ---------------- 4. 안전 프레임 분배 ---------------- */
static int seen_mask(const bs_app_t *a)
{
  return (a->sc.speed_seen ? 1 : 0) | (a->sc.imu_seen ? 2 : 0) | (a->sc.pedal_seen ? 4 : 0) | (a->sc.sas_seen ? 8 : 0);
}

static void test_safety_routing(void)
{
  static const struct {
    uint32_t id;
    uint16_t data_id;
    int mask;
  } routes[] = {{BS_ID_ESC_SPEED, SC_DATA_ID_ESC_SPEED, 1},
                {BS_ID_ACU_IMU, SC_DATA_ID_ACU_IMU, 2},
                {BS_ID_VCU_PEDAL, SC_DATA_ID_VCU_PEDAL, 4},
                {BS_ID_SAS_STEER, SC_DATA_ID_SAS_STEER, 8}};
  bmpc_can_frame_t f;
  size_t i;
  printf("[safety frame routing]\n");

  for (i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
    fresh(&g_a);
    seal_frame(&f, routes[i].id, routes[i].data_id);
    bs_on_safety_frame(&g_a, &f, 1234u);
    CHECK(seen_mask(&g_a) == routes[i].mask, "0x%03X reaches its handler only (mask %d)", (unsigned)routes[i].id, seen_mask(&g_a));
    CHECK(g_a.n_safety_rx == 1u, "0x%03X counted (%u)", (unsigned)routes[i].id, (unsigned)g_a.n_safety_rx);

    /* 차량 CAN 은 모두 8 바이트다. DLC 가 다르면 CRC 가 맞아도 버린다 */
    fresh(&g_a);
    f.dlc = 7;
    bs_on_safety_frame(&g_a, &f, 1234u);
    CHECK(seen_mask(&g_a) == 0 && g_a.n_safety_rx == 0u, "0x%03X DLC 7 dropped", (unsigned)routes[i].id);
    f.dlc = 0;
    bs_on_safety_frame(&g_a, &f, 1234u);
    CHECK(seen_mask(&g_a) == 0 && g_a.n_safety_rx == 0u, "0x%03X DLC 0 dropped", (unsigned)routes[i].id);

    /* 다른 메시지의 Data ID 로 봉인한 프레임은 처리기가 CRC 로 버린다 */
    fresh(&g_a);
    seal_frame(&f, routes[i].id, SC_DATA_ID_CTRL_CMD);
    bs_on_safety_frame(&g_a, &f, 1234u);
    CHECK(seen_mask(&g_a) == 0, "0x%03X wrong data id rejected", (unsigned)routes[i].id);
    CHECK(g_a.sc.pending_flags2 & SC_FLAG2_VEH_CRC, "0x%03X CRC flag raised", (unsigned)routes[i].id);
  }

  /* 값이 제어기까지 간다 */
  fresh(&g_a);
  memset(&f, 0, sizeof(f));
  f.id = BS_ID_ESC_SPEED;
  f.dlc = 8;
  f.data[0] = 0xE8u; /* 10.00 kph */
  f.data[1] = 0x03u;
  sc_frame_seal(f.data, SC_DATA_ID_ESC_SPEED);
  bs_on_safety_frame(&g_a, &f, 777u);
  CHECK(fabsf(g_a.sc.speed_kph - 10.0f) < 1e-3f, "speed %.2f kph", (double)g_a.sc.speed_kph);
  CHECK(g_a.sc.last_speed_ms == 777u, "receive time passed through");

  /* 모르는 ID 는 DLC 가 맞아도 세지 않는다 */
  fresh(&g_a);
  seal_frame(&f, 0x300u, SC_DATA_ID_ESC_SPEED);
  memcpy(&g_snap, &g_a, sizeof(g_a));
  bs_on_safety_frame(&g_a, &f, 1234u);
  CHECK(memcmp(&g_snap, &g_a, sizeof(g_a)) == 0, "unknown id leaves the state alone");
}

/* ---------------- 5. 시험대 프레임 ---------------- */
static void test_bench_frames(void)
{
  bmpc_can_frame_t f;
  uint32_t resets = 0;
  printf("[bench reset]\n");

  fresh(&g_a);
  go_normal(&g_a);
  CHECK(g_a.out.state == SC_STATE_NORMAL, "normal before bus off (%d)", (int)g_a.out.state);
  bs_on_bus_off(&g_a);
  tick(&g_a);
  CHECK(g_a.out.state == SC_STATE_SAFE_STOP, "bus off -> SAFE_STOP (%d)", (int)g_a.out.state);
  CHECK(g_a.out.bus_off_seen == 1u, "bus off reported");
  CHECK(g_sent[(g_nsent - 2) % 16].id == BS_ID_SAFE_OUT && (g_sent[(g_nsent - 2) % 16].data[6] & 0x10u), "SAFE_OUT bus-off bit");

  memset(&f, 0, sizeof(f));
  f.id = BS_ID_BENCH_RESET;

  /* 짧거나 값이 틀린 프레임은 래치를 풀지 않는다 (각 조건이 따로 막아야 한다) */
  f.dlc = 1;
  f.data[0] = 0x5Au;
  f.data[1] = 0xA5u;
  bs_on_safety_frame(&g_a, &f, g_ms);
  CHECK(g_a.n_resets == resets && g_a.sc.state == SC_STATE_SAFE_STOP, "DLC 1 does not reset");
  f.dlc = 2;
  f.data[0] = 0x5Au;
  f.data[1] = 0x00u;
  bs_on_safety_frame(&g_a, &f, g_ms);
  CHECK(g_a.n_resets == resets && g_a.sc.state == SC_STATE_SAFE_STOP, "wrong second byte does not reset");
  f.data[0] = 0x00u;
  f.data[1] = 0xA5u;
  bs_on_safety_frame(&g_a, &f, g_ms);
  CHECK(g_a.n_resets == resets && g_a.sc.state == SC_STATE_SAFE_STOP, "wrong first byte does not reset");
  f.dlc = 0;
  f.data[0] = 0x00u;
  f.data[1] = 0x00u;
  bs_on_safety_frame(&g_a, &f, g_ms);
  CHECK(g_a.n_resets == resets && g_a.sc.state == SC_STATE_SAFE_STOP, "empty frame does not reset");

  f.dlc = 2;
  f.data[0] = 0x5Au;
  f.data[1] = 0xA5u;
  bs_on_safety_frame(&g_a, &f, g_ms);
  ++resets;
  CHECK(g_a.n_resets == resets && g_a.sc.state == SC_STATE_INIT, "shortest valid frame resets");
  CHECK(g_a.n_safety_rx == 0u, "bench frames are not vehicle frames");
  tick(&g_a);
  CHECK(g_a.out.state == SC_STATE_INIT, "stays out of SAFE_STOP after the reset (%d)", (int)g_a.out.state);
  go_normal(&g_a);
  /* 같은 Alive 1,2,3 이지만 리셋으로 Alive 기록도 지워져 다시 NORMAL 이 된다 */
  CHECK(g_a.out.state == SC_STATE_NORMAL, "back to NORMAL after the reset (%d)", (int)g_a.out.state);
  f.dlc = 8;
  bs_on_safety_frame(&g_a, &f, g_ms);
  ++resets;
  CHECK(g_a.n_resets == resets, "longer frame resets too");

#ifdef BS_TEST_HOOK
  printf("[fault hook frames]\n");
  {
    bmpc_can_frame_t fr[BMPC_RESP_FRAMES];
    const float base = 0.1f;

    fresh(&g_a);
    memset(&f, 0, sizeof(f));
    f.id = BS_ID_BENCH_RESET;
    f.dlc = 6;
    f.data[0] = BS_HOOK_MAGIC;
    f.data[1] = BS_HOOK_OFFSET;
    f.data[2] = 0x2Eu; /* -1234 = 0xFB2E */
    f.data[3] = 0xFBu;
    f.data[4] = 0xE8u; /* 1000 ms = 0x03E8 */
    f.data[5] = 0x03u;
    bs_on_safety_frame(&g_a, &f, g_ms);
    CHECK(g_a.hook_pending == 1 && g_a.hook_kind == BS_HOOK_OFFSET, "hook armed");
    CHECK(g_a.hook_centideg == -1234 && g_a.hook_ms == 1000u, "little-endian fields (%d, %u)", (int)g_a.hook_centideg, (unsigned)g_a.hook_ms);
    CHECK(g_a.n_resets == 0u, "hook frame is not a reset");

    /* 음수 더하기가 CTRL_CMD 까지 간다 */
    make_resp_frames(1u, BMPC_OK, (double)base, 0.0, 0.0, fr);
    send_group(&g_a, fr);
    CHECK(g_a.n_hook == 1u && g_a.hook_pending == 0, "hook activated once");
    CHECK(fabs(i16(&g_a.ctrl[0]) * 0.01 - ((double)base * 57.29578 - 12.34)) < 0.02, "offset applied (%.2f deg)", i16(&g_a.ctrl[0]) * 0.01);

    /* 길이가 모자라거나 매직이 틀리면 결함을 넣지 않는다 */
    fresh(&g_a);
    f.dlc = 5;
    bs_on_safety_frame(&g_a, &f, g_ms);
    CHECK(g_a.hook_pending == 0 && g_a.n_resets == 0u, "DLC 5 is not a hook");
    f.dlc = 6;
    f.data[0] = (uint8_t)(BS_HOOK_MAGIC ^ 0x01u);
    bs_on_safety_frame(&g_a, &f, g_ms);
    CHECK(g_a.hook_pending == 0 && g_a.n_resets == 0u, "wrong magic is not a hook");
    /* 매직이 틀리면 리셋 형식인지 본다 (같은 ID 를 나눠 쓴다) */
    f.data[0] = 0x5Au;
    f.data[1] = 0xA5u;
    bs_on_safety_frame(&g_a, &f, g_ms);
    CHECK(g_a.hook_pending == 0 && g_a.n_resets == 1u, "reset frame of DLC 6 still resets");

    /* 시간 0: 켜지자마자 꺼져 명령이 바뀌지 않는다 */
    fresh(&g_a);
    f.dlc = 6;
    f.data[0] = BS_HOOK_MAGIC;
    f.data[1] = BS_HOOK_OFFSET;
    f.data[2] = 0xB8u; /* 3000 */
    f.data[3] = 0x0Bu;
    f.data[4] = 0u;
    f.data[5] = 0u;
    bs_on_safety_frame(&g_a, &f, g_ms);
    send_group(&g_a, fr);
    CHECK(g_a.n_hook == 1u && g_a.hook_active == BS_HOOK_OFF, "zero duration ends at once");
    CHECK(fabs(i16(&g_a.ctrl[0]) * 0.01 - (double)base * 57.29578) < 0.02, "zero duration leaves the command (%.2f)", i16(&g_a.ctrl[0]) * 0.01);

    /* 모르는 종류는 명령을 바꾸지 않고 기준 조향만 갱신한다 */
    fresh(&g_a);
    f.data[1] = 7u;
    f.data[4] = 0xE8u;
    f.data[5] = 0x03u;
    bs_on_safety_frame(&g_a, &f, g_ms);
    send_group(&g_a, fr);
    CHECK(g_a.hook_active == 7u, "unknown kind recorded");
    CHECK(fabs(i16(&g_a.ctrl[0]) * 0.01 - (double)base * 57.29578) < 0.02, "unknown kind leaves the command");
    CHECK(fabsf(g_a.hook_last_steer - base) < 1e-6f, "reference steering tracked");

    /* 더하기 중에는 기준 조향이 그대로다: 뒤따르는 고정 결함이 더하기 전 값을 쓴다 */
    fresh(&g_a);
    send_group(&g_a, fr);
    CHECK(fabsf(g_a.hook_last_steer - base) < 1e-6f, "reference before the fault");
    f.data[1] = BS_HOOK_OFFSET;
    bs_on_safety_frame(&g_a, &f, g_ms);
    send_group(&g_a, fr);
    CHECK(fabsf(g_a.hook_last_steer - base) < 1e-6f, "offset does not move the reference");
    f.data[1] = BS_HOOK_HOLD;
    bs_on_safety_frame(&g_a, &f, g_ms);
    make_resp_frames(2u, BMPC_OK, 0.3, 0.0, 0.0, fr);
    send_group(&g_a, fr);
    CHECK(fabs(i16(&g_a.ctrl[0]) * 0.01 - (double)base * 57.29578) < 0.02, "hold uses the reference (%.2f)", i16(&g_a.ctrl[0]) * 0.01);
    /* 시간이 지나면 다시 그대로 */
    g_us += 1000000;
    send_group(&g_a, fr);
    CHECK(g_a.hook_active == BS_HOOK_OFF, "hold expired");
    CHECK(fabs(i16(&g_a.ctrl[0]) * 0.01 - 0.3 * 57.29578) < 0.02, "command passes again (%.2f)", i16(&g_a.ctrl[0]) * 0.01);
  }
#endif
}

/* ---------------- 6. 주기 출력 ---------------- */
static void test_tick_output(void)
{
  int k;
  printf("[periodic output]\n");
  fresh(&g_a);
  for (k = 0; k < 18; ++k) {
    const bmpc_can_frame_t *o, *s;
    g_nsent = 0;
    tick(&g_a);
    CHECK(g_nsent == 2, "two frames per tick (%d)", g_nsent);
    o = &g_sent[0];
    s = &g_sent[1];
    CHECK(o->id == BS_ID_SAFE_OUT && o->dlc == 8u, "SAFE_OUT header");
    CHECK(s->id == BS_ID_SAFE_STAT && s->dlc == 8u, "SAFE_STAT header");
    CHECK(o->data[7] == sc_crc8_e2e(SC_DATA_ID_SAFE_OUT, o->data, 7u), "SAFE_OUT CRC");
    CHECK(s->data[7] == sc_crc8_e2e(SC_DATA_ID_SAFE_STAT, s->data, 7u), "SAFE_STAT CRC");
    CHECK((o->data[6] & 0x0F) == (k & 0x0F), "SAFE_OUT alive %d at tick %d", o->data[6] & 0x0F, k);
    CHECK((s->data[6] & 0x0F) == (k & 0x0F), "SAFE_STAT alive %d at tick %d", s->data[6] & 0x0F, k);
  }
  CHECK(g_a.n_steps == 18u, "n_steps %u", (unsigned)g_a.n_steps);
}

static void test_applied(void)
{
  int hist0;
  printf("[applied steering]\n");

  /* INIT: 아직 차량에 명령이 나가지 않으므로 이력에 넣을 것이 없다 */
  fresh(&g_a);
  send_config(&g_a);
  tick(&g_a);
  CHECK(g_a.out.state == SC_STATE_INIT && g_a.applied_pending == 0, "INIT notes nothing");
  bs_note_applied(&g_a, g_us);
  CHECK(g_a.n_notes == 0u && g_a.app.ctrl.hist.count == 0, "no history from INIT");

  /* NORMAL: 실제 나간 조향이 MPC 부호·rad 로 들어간다 */
  go_normal(&g_a);
  CHECK(g_a.out.state == SC_STATE_NORMAL, "normal");
  feed_cmd(&g_a, 0.02, 0.0, 4);
  tick(&g_a);
  CHECK(g_a.applied_pending == 1, "NORMAL schedules a note");
  CHECK(fabsf(g_a.applied_steer_deg - g_a.out.steer_deg) < 1e-6f, "the note is the output");
  hist0 = g_a.app.ctrl.hist.count;
  bs_note_applied(&g_a, g_us);
  CHECK(g_a.applied_pending == 0, "note consumed");
  CHECK(g_a.n_notes == 1u && g_a.app.ctrl.hist.count == hist0 + 1, "one history entry (notes %u)", (unsigned)g_a.n_notes);
  {
    const bmpc_hist_t *h = &g_a.app.ctrl.hist;
    const double want = (double)g_a.out.steer_deg / 57.29578 / (double)g_a.app.ctrl.cfg.steer_sign;
    CHECK(fabs((double)bmpc_hist_delta(h, h->count - 1) - want) < 1e-5, "history %.5f rad, want %.5f", (double)bmpc_hist_delta(h, h->count - 1), want);
    CHECK(bmpc_hist_time(h, h->count - 1) == g_us, "history time");
  }
  bs_note_applied(&g_a, g_us);
  CHECK(g_a.n_notes == 1u, "a consumed note is not applied twice");
  /* 같은 값이 다시 나가면 이력은 늘지 않는다 */
  tick(&g_a);
  bs_note_applied(&g_a, g_us);
  CHECK(g_a.n_notes == 1u && g_a.applied_pending == 0, "same steering adds nothing");

  /* DEGRADED: Alive 정지. 조향은 계속 나가므로 이력에 넣고, 양의 가속은 막는다 */
  feed_cmd(&g_a, 0.02, 0.5, 5);
  feed_cmd(&g_a, 0.02, 0.5, 5);
  feed_cmd(&g_a, 0.02, 0.5, 5);
  feed_cmd(&g_a, 0.02, 0.5, 5);
  g_a.applied_pending = 0;
  tick(&g_a);
  CHECK(g_a.out.state == SC_STATE_DEGRADED, "alive freeze -> DEGRADED (%d)", (int)g_a.out.state);
  CHECK(g_a.applied_pending == 1, "DEGRADED still schedules a note");
  CHECK(g_a.out.accel_mps2 <= 0.0f, "no acceleration in DEGRADED (%.2f)", (double)g_a.out.accel_mps2);

  /* SAFE_STOP: 지연보상이 볼 일이 없다 */
  bs_on_bus_off(&g_a);
  g_a.applied_pending = 0;
  tick(&g_a);
  CHECK(g_a.out.state == SC_STATE_SAFE_STOP, "SAFE_STOP");
  CHECK(g_a.applied_pending == 0, "SAFE_STOP notes nothing");

  /* 설정 전이면 controller 가 거절한다 */
  fresh(&g_a);
  go_normal(&g_a);
  feed_cmd(&g_a, 0.05, 0.0, 4);
  tick(&g_a);
  CHECK(g_a.applied_pending == 1, "scheduled");
  bs_note_applied(&g_a, g_us);
  CHECK(g_a.n_notes == 0u && g_a.applied_pending == 0, "unconfigured controller records nothing");
}

#ifdef BS_PLAUS
static double ctrl_steer_deg(void) { return i16(&g_a.ctrl[0]) * 0.01; }
static double ctrl_accel(void) { return i16(&g_a.ctrl[2]) * 0.01; }

static void plaus_send(unsigned seq, double deg, bmpc_can_frame_t *fr)
{
  make_resp_frames((uint16_t)seq, BMPC_OK, deg / 57.29578, 0.3, 0.0, fr);
  send_group(&g_a, fr);
  g_ms += 50;
  g_us += 50000;
}

static void plaus_send_b(unsigned seq, double deg, double brake, bmpc_can_frame_t *fr)
{
  make_resp_frames((uint16_t)seq, BMPC_OK, deg / 57.29578, 0.0, brake, fr);
  send_group(&g_a, fr);
  g_us += 50000;
}

static void plaus_reset(void)
{
  bmpc_can_frame_t f;
  memset(&f, 0, sizeof(f));
  f.id = BS_ID_BENCH_RESET;
  f.dlc = 2;
  f.data[0] = 0x5Au;
  f.data[1] = 0xA5u;
  bs_on_safety_frame(&g_a, &f, g_ms);
}

static void test_plaus(void)
{
  bmpc_can_frame_t fr[BMPC_RESP_FRAMES];
  unsigned s = 1;
  int i;
  printf("[steering plausibility]\n");
  fresh(&g_a);
  /* 첫 명령은 비교 대상이 없다. 상한 안의 변화는 그대로 지난다 */
  plaus_send(s++, 30.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 30.0) < 0.02 && g_a.n_plaus == 0u, "first command passes");
  plaus_send(s++, 34.9, fr);
  CHECK(fabs(ctrl_steer_deg() - 34.9) < 0.02 && g_a.n_plaus == 0u, "step 4.9 passes (%f)", ctrl_steer_deg());
  CHECK(ctrl_accel() > 0.0, "accel untouched");
  /* 넘으면 직전 조향 유지 + 감속, 계속 벗어나 있으면 유지 시간 내내 */
  plaus_send(s++, 40.1, fr);
  CHECK(g_a.n_plaus == 1u && fabs(ctrl_steer_deg() - 34.9) < 0.02, "step 5.2 held (%f)", ctrl_steer_deg());
  CHECK(fabs(ctrl_accel() + 2.0) < 0.02, "brake 2 m/s2 (%f)", ctrl_accel());
  for (i = 0; i < 18; ++i) {
    plaus_send(s++, 60.0, fr);
    CHECK(g_a.n_plaus == 1u && fabs(ctrl_steer_deg() - 34.9) < 0.02, "still held");
    CHECK(fabs(ctrl_accel() + 2.0) < 0.02, "still braking");
  }
  /* 유지가 끝나면 한 주기 3.7 deg 로 돌아간다. 돌아가는 동안 감속하고 다시 세지 않는다 */
  g_us += 100000;
  plaus_send(s++, 45.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 38.6) < 0.02 && g_a.n_plaus == 1u, "ramp 1 (%f)", ctrl_steer_deg());
  CHECK(fabs(ctrl_accel() + 2.0) < 0.02, "braking while returning");
  plaus_send(s++, 45.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 42.3) < 0.02, "ramp 2 (%f)", ctrl_steer_deg());
  plaus_send(s++, 45.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 45.0) < 0.02 && ctrl_accel() > 0.0, "caught up (%f)", ctrl_steer_deg());
  /* 음수 방향도 잡고, 더 큰 브레이크 요청은 줄이지 않는다 */
  plaus_send(s++, 41.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 41.0) < 0.02 && g_a.n_plaus == 1u, "normal again");
  plaus_send_b(s++, 30.0, 1.0, fr);
  CHECK(g_a.n_plaus == 2u && fabs(ctrl_steer_deg() - 41.0) < 0.02, "negative step held");
  CHECK(fabs(ctrl_accel() + 5.0) < 0.02, "larger brake kept (%f)", ctrl_accel());
  /* 짧은 결함: 명령이 두 번 연속 정상이면 일찍 푼다 */
  plaus_reset();
  plaus_send(s++, 10.0, fr);
  CHECK(g_a.n_plaus == 0u, "reset clears the count");
  plaus_send(s++, 40.0, fr);
  CHECK(g_a.n_plaus == 1u && fabs(ctrl_steer_deg() - 10.0) < 0.02, "spike held");
  plaus_send(s++, 10.5, fr);
  CHECK(fabs(ctrl_steer_deg() - 10.0) < 0.02 && ctrl_accel() < 0.0, "one good command still held");
  plaus_send(s++, 11.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 11.0) < 0.02 && ctrl_accel() > 0.0, "two good commands release");
  /* 명령 공백 뒤 첫 명령은 비교하지 않는다 */
  g_us += 300000;
  plaus_send(s++, -30.0, fr);
  CHECK(fabs(ctrl_steer_deg() + 30.0) < 0.02 && g_a.n_plaus == 1u, "gap rebases");
  /* 리셋 요청은 메인 문맥에서 처리되고 다음 명령을 비교하지 않는다 */
  plaus_reset();
  CHECK(g_a.plaus_reset_req == 1, "reset deferred to main context");
  plaus_send(s++, 30.0, fr);
  CHECK(fabs(ctrl_steer_deg() - 30.0) < 0.02 && g_a.n_plaus == 0u && g_a.plaus_reset_req == 0, "reset clears the check");
}
#endif

int main(void)
{
#ifdef BS_PLAUS
  test_plaus();
#endif
  test_ids_and_defaults();
  test_ctrl_frame();
  test_intercept();
  test_request_paths();
  test_safety_routing();
  test_bench_frames();
  test_tick_output();
  test_applied();
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
