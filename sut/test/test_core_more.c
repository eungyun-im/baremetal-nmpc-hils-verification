/* core 추가 시험: 후처리, 가중치, controller, 앱의 경계와 거부 경로. 기대값은 주석·헤더에 적힌 규칙에서 계산한다.
 *   1. bmpc_post: 조향 (부호, 범위, 변화율, 속도별 LPF), 종방향 (정지, 상한, 적분, 피드포워드, 데드밴드), 속도 가드
 *   2. bmpc_weights: 구간 선택, 우선순위, smoothstep, 배율 범위
 *   3. bmpc_controller: 입력 검사, STOP, 풀이 실패 유지, warm start, 지연 측정, 이력, 플래그, note_applied
 *   4. bmpc_app: 설정 확정 조건, 프레임 조립, 통계
 *   5. 임의 요청: CAN 으로 올 수 있는 값이면 응답이 지켜야 하는 불변식
 * BMPC_USE_FLOAT 빌드에서도 돈다 (허용오차만 다르다). */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmpc/bmpc_app.h"
#include "bmpc/bmpc_controller.h"
#include "bmpc/bmpc_post.h"
#include "bmpc/bmpc_weights.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                                                    \
  do {                                                                                      \
    if (cond) {                                                                             \
      ++g_pass;                                                                             \
    } else {                                                                                \
      ++g_fail;                                                                             \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);                                         \
      printf(__VA_ARGS__);                                                                  \
      printf("\n");                                                                         \
    }                                                                                       \
  } while (0)

#ifdef BMPC_USE_FLOAT
#define REL_TOL 2e-5
#else
#define REL_TOL 1e-11
#endif
static int near_(double a, double b) { return fabs(a - b) <= REL_TOL * (1.0 + fabs(b)); }
#define NEAR(a, b) near_((double)(a), (double)(b))
#define BM_FABS_(a) fabs((double)(a))

/* ---------------- 1. 후처리: 조향 ---------------- */
static void steer_cfg(bmpc_config_t *c)
{
  bmpc_config_default(c);
  c->st_rate_limit = 0;
  c->st_lpf_enabled = 0;
}

static void test_steer_post(void)
{
  bmpc_config_t c;
  bmpc_post_state_t st;
  bmpc_real u;
  printf("[post: steering]\n");

  /* 부호와 범위 */
  steer_cfg(&c);
  bmpc_post_init(&st);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.3, 5);
  CHECK(NEAR(u, 0.3), "sign flip %g", (double)u);
  CHECK(st.have_prev == 1 && NEAR(st.prev_steer_cmd, 0.3), "state updated");
  u = bmpc_steer_post(&c, &st, (bmpc_real)-2.0, 5);
  CHECK(NEAR(u, c.steer_ub), "upper bound %g", (double)u);
  u = bmpc_steer_post(&c, &st, (bmpc_real)2.0, 5);
  CHECK(NEAR(u, c.steer_lb), "lower bound %g", (double)u);

  /* 첫 명령에는 비교할 직전 값이 없다: 변화율·LPF 를 걸지 않는다 */
  steer_cfg(&c);
  c.st_rate_limit = (bmpc_real)1.8;
  bmpc_post_init(&st);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.5, 5);
  CHECK(NEAR(u, 0.5), "first command skips the rate limit (%g)", (double)u);
  steer_cfg(&c);
  c.st_lpf_enabled = 1;
  bmpc_post_init(&st);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.5, 5);
  CHECK(NEAR(u, 0.5), "first command skips the LPF (%g)", (double)u);

  /* 변화율: 주기당 rate * dt = 0.09 rad, 양쪽 방향 */
  steer_cfg(&c);
  c.st_rate_limit = (bmpc_real)1.8;
  bmpc_post_init(&st);
  (void)bmpc_steer_post(&c, &st, 0, 5);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.5, 5);
  CHECK(NEAR(u, 0.09), "rate limited up %g", (double)u);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.5, 5);
  CHECK(NEAR(u, 0.18), "rate limited again %g", (double)u);
  u = bmpc_steer_post(&c, &st, (bmpc_real)0.5, 5);
  CHECK(NEAR(u, 0.09), "rate limited down %g", (double)u);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.1, 5);
  CHECK(NEAR(u, 0.1), "small change passes %g", (double)u);

  /* 변화율 0 = 끔 */
  steer_cfg(&c);
  bmpc_post_init(&st);
  (void)bmpc_steer_post(&c, &st, 0, 5);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.5, 5);
  CHECK(NEAR(u, 0.5), "rate limit 0 means off (%g)", (double)u);

  /* LPF: 저속 alpha, 고속 alpha_high, 사이는 직선 보간 */
  steer_cfg(&c);
  c.st_lpf_enabled = 1;
  {
    static const struct {
      double speed, alpha;
    } cases[] = {{0.0, 0.65}, {5.0, 0.65}, {11.0, 0.65}, {18.0, 0.685}, {14.5, 0.6675}, {25.0, 0.72}, {30.0, 0.72}};
    size_t i;
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
      bmpc_post_init(&st);
      (void)bmpc_steer_post(&c, &st, (bmpc_real)0.2, 5); /* prev = -0.2 */
      u = bmpc_steer_post(&c, &st, (bmpc_real)-0.4, (bmpc_real)cases[i].speed);
      CHECK(NEAR(u, cases[i].alpha * 0.4 + (1.0 - cases[i].alpha) * -0.2), "speed %g: %g", cases[i].speed, (double)u);
    }
  }
  /* 저속 끝과 고속 끝이 같으면 0 으로 나누지 않고, 그 속도에서는 저속 쪽을 쓴다 */
  c.st_lpf_v_low = 10;
  c.st_lpf_v_high = 10;
  bmpc_post_init(&st);
  (void)bmpc_steer_post(&c, &st, (bmpc_real)0.2, 5);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.4, 10);
  CHECK(NEAR(u, 0.65 * 0.4 + 0.35 * -0.2), "degenerate speed range uses the low alpha (%g)", (double)u);
  bmpc_post_init(&st);
  (void)bmpc_steer_post(&c, &st, (bmpc_real)0.2, 5);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.4, 11);
  CHECK(NEAR(u, 0.72 * 0.4 + 0.28 * -0.2), "above it the high alpha (%g)", (double)u);

  /* LPF 끔이면 그대로 */
  steer_cfg(&c);
  bmpc_post_init(&st);
  (void)bmpc_steer_post(&c, &st, (bmpc_real)0.2, 5);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.4, 5);
  CHECK(NEAR(u, 0.4), "LPF off passes %g", (double)u);

  /* 순서: 변화율 다음에 LPF */
  bmpc_config_default(&c);
  bmpc_post_init(&st);
  (void)bmpc_steer_post(&c, &st, 0, 5);
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.5, 5);
  CHECK(NEAR(u, 0.65 * 0.09), "rate limit then LPF %g", (double)u);

  /* 직전 값이 범위 밖이어도 (범위 설정이 줄어든 경우) 출력은 범위 안 */
  steer_cfg(&c);
  c.st_lpf_enabled = 1;
  bmpc_post_init(&st);
  st.have_prev = 1;
  st.prev_steer_cmd = 2;
  u = bmpc_steer_post(&c, &st, (bmpc_real)-0.6, 5);
  CHECK(u <= c.steer_ub && NEAR(u, c.steer_ub), "output clamped after the LPF %g", (double)u);
}

/* ---------------- 1. 후처리: 종방향 ---------------- */
/* P 제어만 남긴 설정 */
static void pi_cfg(bmpc_config_t *c)
{
  bmpc_config_default(c);
  c->lg_kp = 1;
  c->lg_ki = 0;
  c->lg_int_band = 0;
  c->lg_ff_gain = 0;
  c->lg_deadband = 0;
}

static void thr(const bmpc_config_t *c, bmpc_post_state_t *st, double target, double actual, double dt, double blim,
                double alim, double steer, bmpc_real *a, bmpc_real *b)
{
  bmpc_throttle(c, st, (bmpc_real)target, (bmpc_real)actual, (bmpc_real)dt, (bmpc_real)blim, (bmpc_real)alim, (bmpc_real)steer,
                a, b);
}

static void test_throttle_stop_and_limits(void)
{
  bmpc_config_t c;
  bmpc_post_state_t st;
  bmpc_real a, b;
  int k;
  printf("[post: stop, limits]\n");

  /* 목표 0.05 이하 = 정지: 가속 0, 제동은 주기당 0.1 씩 상한까지, 상태 정리 */
  bmpc_config_default(&c);
  bmpc_post_init(&st);
  st.integral = 3;
  st.ff_target_accel = 2;
  st.ff_prev_target = 9;
  a = 1;
  thr(&c, &st, 0.05, 5, 0.05, -1, -1, 0, &a, &b);
  CHECK(a == 0 && NEAR(b, 0.1), "stop at the threshold: accel %g brake %g", (double)a, (double)b);
  CHECK(st.integral == 0 && st.ff_prev_target == 0 && st.ff_target_accel == 0, "stop clears the integrator and FF");
  CHECK(NEAR(st.last_brake, 0.1), "last brake");
  for (k = 2; k <= 8; ++k) {
    thr(&c, &st, 0, 5, 0.05, -1, -1, 0, &a, &b);
    CHECK(NEAR(b, k * 0.1 < 0.5 ? k * 0.1 : 0.5), "stop ramp step %d: %g", k, (double)b);
  }
  thr(&c, &st, -1, 5, 0.05, (double)c.lg_kick_brake_limit, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.6), "larger limit continues the ramp %g", (double)b);
  /* 이미 상한보다 세게 밟고 있으면 바로 상한으로 */
  st.last_brake = (bmpc_real)0.8;
  thr(&c, &st, 0, 5, 0.05, 0.3, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.3), "brake drops to the limit at once %g", (double)b);
  /* 정지 판정은 목표로만 한다: 이미 서 있어도 제동을 건다 */
  bmpc_post_init(&st);
  thr(&c, &st, 0.05, 0.05, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.1), "standing still still brakes (%g)", (double)b);

  /* 문턱 바로 위는 PI */
  pi_cfg(&c);
  bmpc_post_init(&st);
  thr(&c, &st, 0.06, 0, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, 0.06) && b == 0, "just above the threshold is PI: %g %g", (double)a, (double)b);

  /* 음수 인자 = 설정값, 0 은 0 */
  pi_cfg(&c);
  bmpc_post_init(&st);
  thr(&c, &st, 10, 0, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, c.lg_accel_limit), "accel limit from config %g", (double)a);
  thr(&c, &st, 10, 0, 0.05, -1, 0.7, 0, &a, &b);
  CHECK(NEAR(a, 0.7), "accel limit argument %g", (double)a);
  thr(&c, &st, 10, 0, 0.05, -1, 0, 0, &a, &b);
  CHECK(a == 0, "accel limit 0 means no accel (%g)", (double)a);
  bmpc_post_init(&st);
  thr(&c, &st, 1, 10, 1.0, -1, -1, 0, &a, &b);
  CHECK(NEAR(b, c.lg_brake_limit) && a == 0, "brake limit from config %g", (double)b);
  bmpc_post_init(&st);
  thr(&c, &st, 1, 10, 1.0, 0.3, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.3), "brake limit argument %g", (double)b);
  bmpc_post_init(&st);
  thr(&c, &st, 1, 10, 1.0, 0, -1, 0, &a, &b);
  CHECK(b == 0, "brake limit 0 means no brake (%g)", (double)b);

  /* 제동 증가만 변화율 제한, 감소는 바로 */
  bmpc_post_init(&st);
  thr(&c, &st, 1, 10, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.1), "brake ramp 1: %g", (double)b);
  thr(&c, &st, 1, 10, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.2) && NEAR(st.last_brake, 0.2), "brake ramp 2: %g", (double)b);
  thr(&c, &st, 1, 1.05, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(b, 0.05), "brake release is immediate: %g", (double)b);
  thr(&c, &st, 5, 1, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, c.lg_accel_limit) && b == 0 && st.last_brake == 0, "accelerating clears the brake");
}

static void test_throttle_integral(void)
{
  bmpc_config_t c;
  bmpc_post_state_t st;
  bmpc_real a, b;
  printf("[post: integrator]\n");

  pi_cfg(&c);
  c.lg_kp = 0;
  c.lg_ki = 1;
  c.lg_int_band = 1;
  c.lg_int_limit = 20;

  /* band > 0: 목표 근처에서만 적분 */
  bmpc_post_init(&st);
  thr(&c, &st, 2, 1.5, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.05) && NEAR(a, 0.05), "inside the band: %g", (double)st.integral);
  thr(&c, &st, 2, 1, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.05), "error = band: no integration (%g)", (double)st.integral);
  thr(&c, &st, 5, 1, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.05), "outside the band: no integration (%g)", (double)st.integral);
  /* 양의 오차에서는 양의 적분을 지우지 않는다 */
  thr(&c, &st, 2, 1.5, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.1), "positive error keeps the integral (%g)", (double)st.integral);

  /* band <= 0: 언제나 적분 */
  c.lg_int_band = 0;
  bmpc_post_init(&st);
  thr(&c, &st, 6, 1, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.5), "band 0 always integrates (%g)", (double)st.integral);
  c.lg_int_band = -1;
  bmpc_post_init(&st);
  thr(&c, &st, 6, 1, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.5), "negative band always integrates (%g)", (double)st.integral);
  /* band <= 0 이면 빨라도 지우지 않는다 */
  c.lg_int_band = 0;
  bmpc_post_init(&st);
  st.integral = 10;
  thr(&c, &st, 1, 6, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 9.5), "band 0 never clears (%g)", (double)st.integral);

  /* band > 0, 목표보다 0.1 넘게 빠른데 양의 적분: 지운다 */
  c.lg_int_band = 1;
  bmpc_post_init(&st);
  st.integral = (bmpc_real)0.5;
  thr(&c, &st, 2, 2.5, 0.1, -1, -1, 0, &a, &b);
  CHECK(st.integral == 0, "overspeed clears a positive integral (%g)", (double)st.integral);
  /* 0.1 안쪽이면 그대로 적분 */
  bmpc_post_init(&st);
  st.integral = (bmpc_real)0.5;
  thr(&c, &st, 2, 2.05, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 0.495), "small overspeed keeps it (%g)", (double)st.integral);
  /* 음의 적분은 지우지 않는다 */
  bmpc_post_init(&st);
  st.integral = (bmpc_real)-0.5;
  thr(&c, &st, 2, 2.5, 0.1, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, -0.55), "negative integral stays (%g)", (double)st.integral);
  /* 오차가 정확히 -0.1 이면 아직 지우지 않는다: 부동소수로 정확히 나오는 값을 찾는다 */
  {
    bmpc_real v = (bmpc_real)0.16;
    int k, found = 0;
    for (k = 0; k < 4096 && !found; ++k, v += (bmpc_real)(1.0 / 1024.0)) {
      const bmpc_real t = v + (bmpc_real)-0.1;
      if (t - v == (bmpc_real)-0.1 && t > (bmpc_real)0.05) {
        found = 1;
        bmpc_post_init(&st);
        st.integral = (bmpc_real)0.5;
        bmpc_throttle(&c, &st, t, v, (bmpc_real)0.1, -1, -1, 0, &a, &b);
        CHECK(st.integral > 0, "error of exactly -0.1 keeps the integral (%g)", (double)st.integral);
      }
    }
    CHECK(found, "found an exact -0.1 pair");
  }

  /* 적분 상한 */
  c.lg_int_band = 0;
  c.lg_int_limit = 6;
  bmpc_post_init(&st);
  thr(&c, &st, 101, 1, 1.0, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, 6), "upper integral limit (%g)", (double)st.integral);
  bmpc_post_init(&st);
  thr(&c, &st, 1, 101, 1.0, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.integral, -6), "lower integral limit (%g)", (double)st.integral);
}

static void test_throttle_ff(void)
{
  bmpc_config_t c;
  bmpc_post_state_t st;
  bmpc_real a, b;
  const double dt = 0.05;
  double k, want;
  printf("[post: feed-forward]\n");

  pi_cfg(&c);
  c.lg_kp = 0;
  c.lg_ff_gain = 1; /* 기본값: per_pedal 3.5, tau 0.3, min 0.6, taper 1, steer_zero 0.2 */
  k = dt / ((double)c.lg_ff_tau + dt);

  /* 첫 주기: 직전 목표(-1)가 없다. 필터는 움직이지 않는다 */
  bmpc_post_init(&st);
  st.ff_target_accel = (bmpc_real)0.7;
  thr(&c, &st, 5, 0, dt, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.ff_target_accel, 0.7), "no previous target: filter holds (%g)", (double)st.ff_target_accel);
  CHECK(NEAR(st.ff_prev_target, 5), "previous target stored");

  /* 램프: 필터 -> 페달 */
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  thr(&c, &st, 5.5, 4, dt, -1, -1, 0, &a, &b);
  want = k * 10.0;
  CHECK(NEAR(st.ff_target_accel, want), "filtered accel %g, want %g", (double)st.ff_target_accel, want);
  CHECK(NEAR(a, want / 3.5), "pedal %g, want %g", (double)a, want / 3.5);

  /* 필터는 이전 값에서 출발한다 */
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  st.ff_target_accel = 1;
  thr(&c, &st, 5.5, 4, dt, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.ff_target_accel, 1.0 + k * (10.0 - 1.0)), "first-order filter (%g)", (double)st.ff_target_accel);

  /* 정지 뒤 첫 주기: 직전 목표 0 은 유효한 값이다 */
  bmpc_post_init(&st);
  thr(&c, &st, 0, 0, dt, -1, -1, 0, &a, &b);
  CHECK(st.ff_prev_target == 0, "stop leaves previous target 0");
  thr(&c, &st, 1, 0, dt, -1, -1, 0, &a, &b);
  CHECK(NEAR(st.ff_target_accel, k * 20.0), "ramp from 0 counts (%g)", (double)st.ff_target_accel);

  /* 오차 테이퍼: 오차 / taper, 0..1 */
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  thr(&c, &st, 5.5, 5, dt, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, 0.5 * want / 3.5), "half taper %g", (double)a);
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  thr(&c, &st, 5.5, 6, dt, -1, -1, 0, &a, &b);
  CHECK(a == 0, "no FF when already faster (%g)", (double)a);

  c.lg_ff_taper = 2;
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  thr(&c, &st, 5.5, 5, dt, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, 0.25 * want / 3.5), "taper length scales the error (%g)", (double)a);
  c.lg_ff_taper = 1;

  /* 조향 테이퍼: 1 - |조향| / steer_zero, 부호 무관 */
  {
    static const struct {
      double steer, factor;
    } cases[] = {{0.1, 0.5}, {-0.1, 0.5}, {0.15, 0.25}, {0.2, 0.0}, {-0.3, 0.0}};
    size_t i;
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
      bmpc_post_init(&st);
      st.ff_prev_target = 5;
      thr(&c, &st, 5.5, 4, dt, -1, -1, cases[i].steer, &a, &b);
      CHECK(NEAR(a, cases[i].factor * want / 3.5), "steer %g: pedal %g", cases[i].steer, (double)a);
    }
  }

  /* 피드포워드 가속도는 페달 1.0 에 해당하는 값에서 자른다, gain 을 곱한다 */
  c.lg_ff_gain = (bmpc_real)0.5;
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  st.ff_target_accel = 10;
  thr(&c, &st, 5.5, 4, dt, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, 0.5), "saturated FF x gain %g", (double)a);
  c.lg_ff_gain = 1;

  /* 최소 가속도 이하는 0 */
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  st.ff_target_accel = (bmpc_real)0.5;
  thr(&c, &st, 5, 4, dt, -1, -1, 0, &a, &b); /* 목표가 그대로면 필터는 줄기만 한다 */
  CHECK(a == 0, "below min accel: no FF (%g)", (double)a);
  /* 필터 값이 최소 가속도와 정확히 같을 때도 0: 같은 식으로 값을 만들어 설정에 넣는다 */
  {
    bmpc_post_state_t probe;
    bmpc_post_init(&probe);
    probe.ff_prev_target = 5;
    probe.ff_target_accel = 1;
    thr(&c, &probe, 5, 4, dt, -1, -1, 0, &a, &b);
    CHECK(a > 0, "above min accel gives FF (%g)", (double)a);
    c.lg_ff_min_accel = probe.ff_target_accel;
    bmpc_post_init(&st);
    st.ff_prev_target = 5;
    st.ff_target_accel = 1;
    thr(&c, &st, 5, 4, dt, -1, -1, 0, &a, &b);
    CHECK(a == 0, "exactly min accel: no FF (%g)", (double)a);
  }

  /* gain 0 = 끔 */
  pi_cfg(&c);
  c.lg_kp = 0;
  bmpc_post_init(&st);
  st.ff_prev_target = 5;
  st.ff_target_accel = 10;
  thr(&c, &st, 5.5, 4, dt, -1, -1, 0, &a, &b);
  CHECK(a == 0, "FF off (%g)", (double)a);
  CHECK(NEAR(st.ff_prev_target, 5.5), "previous target tracked even when off");
}

static void test_throttle_deadband(void)
{
  bmpc_config_t c;
  bmpc_post_state_t st;
  bmpc_real a, b;
  printf("[post: deadband]\n");
  pi_cfg(&c);
  c.lg_deadband = (bmpc_real)0.04;

  bmpc_post_init(&st);
  st.last_brake = (bmpc_real)0.4;
  thr(&c, &st, 10, 9.97, 0.05, -1, -1, 0, &a, &b);
  CHECK(a == 0 && b == 0 && st.last_brake == 0, "small positive pedal coasts (%g %g)", (double)a, (double)b);
  bmpc_post_init(&st);
  st.last_brake = (bmpc_real)0.4;
  thr(&c, &st, 10, 10.03, 0.05, -1, -1, 0, &a, &b);
  CHECK(a == 0 && b == 0 && st.last_brake == 0, "small negative pedal coasts (%g %g)", (double)a, (double)b);
  bmpc_post_init(&st);
  thr(&c, &st, 10, 9.5, 0.05, -1, -1, 0, &a, &b);
  CHECK(NEAR(a, 0.5), "large pedal passes (%g)", (double)a);
  bmpc_post_init(&st);
  thr(&c, &st, 10, 10.05, 0.05, -1, -1, 0, &a, &b);
  CHECK(a == 0 && NEAR(b, 0.05), "large brake passes (%g)", (double)b);

  /* 페달이 데드밴드와 정확히 같으면 밖이다 */
  {
    const bmpc_real target = 10, actual = (bmpc_real)9.5;
    c.lg_deadband = target - actual;
    bmpc_post_init(&st);
    bmpc_throttle(&c, &st, target, actual, (bmpc_real)0.05, -1, -1, 0, &a, &b);
    CHECK(a == c.lg_deadband, "pedal = deadband is outside (%g)", (double)a);
  }
  /* 데드밴드 0 = 끔 */
  c.lg_deadband = 0;
  bmpc_post_init(&st);
  thr(&c, &st, 10, 9.999, 0.05, -1, -1, 0, &a, &b);
  CHECK(a > 0, "deadband off passes a tiny pedal (%g)", (double)a);
}

static void test_legal_guard(void)
{
  bmpc_config_t c;
  bmpc_post_state_t st;
  bmpc_real a, b;
  printf("[post: legal guard]\n");
  bmpc_config_default(&c); /* 16.2 에서 가속 끊고 16.4 넘으면 제동 0.3 */
  bmpc_post_init(&st);

  a = (bmpc_real)0.5;
  b = 0;
  bmpc_legal_guard(&c, &st, c.legal_cut, &a, &b);
  CHECK(NEAR(a, 0.5) && b == 0, "at the cut speed nothing changes (%g)", (double)a);
  a = (bmpc_real)0.5;
  bmpc_legal_guard(&c, &st, c.legal_cut + (bmpc_real)0.01, &a, &b);
  CHECK(a == 0 && b == 0, "above the cut: no accel, no brake yet (%g %g)", (double)a, (double)b);
  a = 0;
  b = (bmpc_real)0.1;
  bmpc_legal_guard(&c, &st, c.legal_brake_at, &a, &b);
  CHECK(NEAR(b, 0.1) && st.last_brake == 0, "at the brake speed nothing changes (%g)", (double)b);
  bmpc_legal_guard(&c, &st, c.legal_brake_at + (bmpc_real)0.01, &a, &b);
  CHECK(NEAR(b, c.legal_brake) && NEAR(st.last_brake, c.legal_brake), "above: brake raised (%g)", (double)b);
  /* 이미 충분히 밟고 있으면 그대로 */
  b = (bmpc_real)0.5;
  st.last_brake = (bmpc_real)0.5;
  bmpc_legal_guard(&c, &st, 20, &a, &b);
  CHECK(NEAR(b, 0.5) && NEAR(st.last_brake, 0.5), "stronger brake stays (%g)", (double)b);
  b = c.legal_brake;
  st.last_brake = (bmpc_real)0.1;
  bmpc_legal_guard(&c, &st, 20, &a, &b);
  CHECK(NEAR(st.last_brake, 0.1), "equal brake is not an intervention");
  /* 저속에서는 아무것도 하지 않는다 */
  a = (bmpc_real)0.5;
  b = 0;
  st.last_brake = 0;
  bmpc_legal_guard(&c, &st, 10, &a, &b);
  CHECK(NEAR(a, 0.5) && b == 0 && st.last_brake == 0, "low speed untouched");
}

/* ---------------- 2. 가중치 ---------------- */
static void test_weights_more(void)
{
  bmpc_config_t c;
  bmpc_weights_t w;
  const double ss = 0.4, cs = 0.4; /* 기본 steer_scale, steer_change_scale */
  double prev;
  int k;
  printf("[weights]\n");
  bmpc_config_default(&c);

  /* 회피 > 고속 > 도심, 속도와 무관 */
  bmpc_compute_weights(&c, 0, 1, 3.0, &w);
  CHECK(NEAR(w.cte, c.a_cte) && NEAR(w.epsi, c.a_epsi) && NEAR(w.delta, c.a_delta * ss) && NEAR(w.ddelta, c.a_ddelta * cs),
        "avoidance set");
  bmpc_compute_weights(&c, 1, 1, 30.0, &w);
  CHECK(NEAR(w.cte, c.a_cte) && NEAR(w.ddelta, c.a_ddelta * cs), "avoidance beats highway");
  bmpc_compute_weights(&c, 1, 0, 3.0, &w);
  CHECK(NEAR(w.cte, c.h_cte) && NEAR(w.epsi, c.h_epsi) && NEAR(w.delta, c.h_delta * ss) && NEAR(w.ddelta, c.h_ddelta * cs),
        "highway set at low speed");

  /* 고정 항은 구간과 무관 */
  CHECK(w.v == c.w_v && w.vt == c.w_vt && w.dvt == c.w_dvt, "fixed terms");
  CHECK(NEAR(w.delta_start, c.w_delta_start * ss), "delta_start scaled");

  /* 도심 끝점 */
  {
    static const double low[] = {-5.0, 0.0, 8.5};
    static const double high[] = {11.5, 12.0, 40.0};
    size_t i;
    for (i = 0; i < 3; ++i) {
      bmpc_compute_weights(&c, 0, 0, (bmpc_real)low[i], &w);
      CHECK(NEAR(w.cte, c.u_cte_start) && NEAR(w.epsi, c.u_epsi) && NEAR(w.delta, c.u_delta_start * ss) &&
                NEAR(w.ddelta, c.u_ddelta_start * cs),
            "urban start set at %g", low[i]);
      bmpc_compute_weights(&c, 0, 0, (bmpc_real)high[i], &w);
      CHECK(NEAR(w.cte, c.u_cte_urban) && NEAR(w.epsi, c.u_epsi) && NEAR(w.delta, c.u_delta_high * ss) &&
                NEAR(w.ddelta, c.u_ddelta_urban * cs),
            "urban full set at %g", high[i]);
    }
  }

  /* smoothstep: 비율 0.25 -> 0.15625, 0.75 -> 0.84375 */
  bmpc_compute_weights(&c, 0, 0, 9.25, &w);
  CHECK(NEAR(w.cte, 5000.0 + 0.15625 * (750.0 - 5000.0)), "smoothstep 1/4 cte %g", (double)w.cte);
  CHECK(NEAR(w.delta, (2500.0 + 0.15625 * (60000.0 - 2500.0)) * ss), "smoothstep 1/4 delta %g", (double)w.delta);
  bmpc_compute_weights(&c, 0, 0, 10.75, &w);
  CHECK(NEAR(w.cte, 5000.0 + 0.84375 * (750.0 - 5000.0)), "smoothstep 3/4 cte %g", (double)w.cte);
  CHECK(NEAR(w.ddelta, (350000.0 + 0.84375 * (1000000.0 - 350000.0)) * cs), "smoothstep 3/4 ddelta %g", (double)w.ddelta);

  /* 전 구간에서 단조, 끊김 없음 */
  prev = 5000.0;
  for (k = 0; k <= 200; ++k) {
    const double v = 8.0 + 4.0 * k / 200.0;
    bmpc_compute_weights(&c, 0, 0, (bmpc_real)v, &w);
    CHECK((double)w.cte <= prev + 1e-3 && prev - (double)w.cte < 70.0, "cte monotonic and continuous at %g (%g)", v, (double)w.cte);
    prev = (double)w.cte;
  }

  /* 배율은 0.05..5 로 자른다. steer_scale 은 조향 항, change_scale 은 변화 항만 */
  c.steer_scale = 0;
  c.steer_change_scale = 100;
  bmpc_compute_weights(&c, 1, 0, 3.0, &w);
  CHECK(NEAR(w.delta, c.h_delta * 0.05) && NEAR(w.delta_start, c.w_delta_start * 0.05), "steer_scale floor");
  CHECK(NEAR(w.ddelta, c.h_ddelta * 5.0), "change_scale ceiling");
  c.steer_scale = 100;
  c.steer_change_scale = 0;
  bmpc_compute_weights(&c, 1, 0, 3.0, &w);
  CHECK(NEAR(w.delta, c.h_delta * 5.0) && NEAR(w.ddelta, c.h_ddelta * 0.05), "opposite ends");

  /* 시작 속도와 끝 속도가 같아도 0 으로 나누지 않는다 */
  bmpc_config_default(&c);
  c.u_speed_full = c.u_speed_start;
  bmpc_compute_weights(&c, 0, 0, c.u_speed_start, &w);
  CHECK(NEAR(w.cte, c.u_cte_start), "degenerate span: start set");
  bmpc_compute_weights(&c, 0, 0, c.u_speed_start + 1, &w);
  CHECK(NEAR(w.cte, c.u_cte_urban), "degenerate span: full set");
}

/* ---------------- 3. controller ---------------- */
static bmpc_controller_t g_c, g_c2;
static bmpc_mpc_work_t g_work;
static bmpc_mpc_result_t g_res;

/* 부를 때마다 step_us 씩 가는 시계 */
typedef struct {
  int64_t t, step;
} clk_t;
static int64_t clk_now(void *ctx)
{
  clk_t *k = (clk_t *)ctx;
  const int64_t t = k->t;
  k->t += k->step;
  return t;
}

static void make_req(bmpc_request_t *r, uint16_t seq, double c0, double v)
{
  memset(r, 0, sizeof(*r));
  r->seq = seq;
  r->c[0] = (bmpc_real)c0;
  r->x_max = 20;
  r->ref_v = (bmpc_real)v;
  r->v_ub = (bmpc_real)(v + 1);
  r->post_cap = -1;
  r->snap = -1;
  r->velocity = (bmpc_real)v;
  r->period = (bmpc_real)0.044;
  r->odom_age = (bmpc_real)0.01;
}

/* 주기 간격 50 ms 로 몇 번 돌려 조향·warm start 를 만들어 둔다 */
static void warm_up(bmpc_controller_t *c, clk_t *k, double c0, int n)
{
  bmpc_request_t r;
  bmpc_response_t p;
  int i;
  for (i = 0; i < n; ++i) {
    make_req(&r, (uint16_t)(i + 1), c0, 8.0);
    (void)bmpc_controller_step(c, &r, clk_now, k, &p);
    k->t += 50000;
  }
}

static void test_ctrl_input_checks(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  int st, i, m;
  printf("[controller: input checks]\n");
  bmpc_config_default(&cfg);

  /* 설정 전 */
  memset(&g_c, 0, sizeof(g_c));
  make_req(&r, 77, 0, 8);
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_ERR_NOT_CONFIGURED && p.status == BMPC_ERR_NOT_CONFIGURED && p.seq == 77, "not configured");
  CHECK(p.steer_cmd == 0 && p.accel == 0 && p.brake == 0 && g_c.errors == 0u, "no command, not an error");

  /* 경로·속도 항목 중 하나라도 유한하지 않으면 풀지 않고 직전 조향 유지. 항목마다 따로 본다 */
  bmpc_controller_init(&g_c, &cfg);
  warm_up(&g_c, &k, 1.0, 5);
  {
    const bmpc_real steer0 = g_c.post.prev_steer_cmd, delta0 = g_c.steering_angle, vt0 = g_c.target_velocity;
    const int hist0 = g_c.hist.count, warm0 = g_c.warm_valid;
    const uint32_t err0 = g_c.errors;
    uint32_t n = 0;
    CHECK(BM_FABS_(steer0) > 0.01, "steering built up");
    for (m = 0; m < 2; ++m) {
      const bmpc_real bad = m == 0 ? (bmpc_real)NAN : (bmpc_real)HUGE_VAL;
      for (i = 0; i < 9; ++i) {
        make_req(&r, (uint16_t)(100 + i), 1.0, 8.0);
        switch (i) {
          case 0: r.c[0] = bad; break;
          case 1: r.c[1] = bad; break;
          case 2: r.c[2] = bad; break;
          case 3: r.c[3] = bad; break;
          case 4: r.velocity = bad; break;
          case 5: r.yaw_rate = bad; break;
          case 6: r.ref_v = bad; break;
          case 7: r.v_lb = bad; break;
          default: r.v_ub = bad; break;
        }
        st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
        ++n;
        CHECK(st == BMPC_ERR_NUMERIC && p.status == BMPC_ERR_NUMERIC && p.seq == r.seq, "field %d rejected (%d)", i, st);
        CHECK(p.steer_cmd == steer0 && p.accel == 0 && p.brake == 0, "field %d: previous steering, no pedals", i);
      }
    }
    CHECK(g_c.errors == err0 + n, "each rejection counted");
    CHECK(g_c.hist.count == hist0 && g_c.warm_valid == warm0, "history and warm start untouched");
    CHECK(g_c.steering_angle == delta0 && g_c.target_velocity == vt0, "targets untouched");
  }

  /* 그 밖의 항목은 양자화된 u16 으로 오므로 NaN 일 수 없지만, 직접 부르더라도 결과를 망치지 않는다 */
  for (i = 0; i < 6; ++i) {
    make_req(&r, 200, 0.5, 8.0);
    switch (i) {
      case 0: r.x_max = (bmpc_real)NAN; break;
      case 1: r.period = (bmpc_real)NAN; break;
      case 2: r.odom_age = (bmpc_real)NAN; break;
      case 3: r.lf_est = (bmpc_real)NAN; break;
      case 4: r.snap = (bmpc_real)NAN; break;
      default: r.post_cap = (bmpc_real)NAN; break;
    }
    bmpc_controller_init(&g_c, &cfg);
    st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
    CHECK(st == BMPC_OK && isfinite(p.steer_cmd) && isfinite(p.accel) && isfinite(p.vt_final), "unchecked field %d is harmless (%d)", i, st);
  }
}

static void test_ctrl_outputs(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  int st, i, nu;
  printf("[controller: solve path]\n");

  /* 지연보상 끔: predict_future_state 식 */
  bmpc_config_default(&cfg);
  cfg.dc_enabled = 0;
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0.5, 10.0);
  r.c[1] = (bmpc_real)0.1;
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_OK, "solved");
  {
    const double epsi = -atan(0.1);
    CHECK(NEAR(g_c.last_s0[BMPC_SX], 10.0 * 0.05) && g_c.last_s0[BMPC_SY] == 0 && NEAR(g_c.last_s0[BMPC_SPSI], 0.0), "x y psi");
    CHECK(NEAR(g_c.last_s0[BMPC_SV], 10.0 + 0.4 * (0.0 - 10.0) * 0.05), "v with target 0: %g", (double)g_c.last_s0[BMPC_SV]);
    CHECK(NEAR(g_c.last_s0[BMPC_SCTE], 0.5 + 10.0 * sin(epsi) * 0.05), "cte %g", (double)g_c.last_s0[BMPC_SCTE]);
    CHECK(NEAR(g_c.last_s0[BMPC_SEPSI], epsi), "epsi %g", (double)g_c.last_s0[BMPC_SEPSI]);
  }
  CHECK(p.state_delay == 0 && p.compute == 0 && p.actuation == 0, "no delay info with delay comp off");
  CHECK(NEAR(g_c.compute_s, cfg.dc_compute_init), "compute estimate not touched with delay comp off");

  /* 둘째 주기: 직전 조향이 psi, epsi 에 들어간다 (yaw rate = -v delta / Lf), 직전 목표가 v 에 들어간다 */
  {
    const double vt = (double)g_c.target_velocity;
    const double delta = (double)g_c.post.prev_steer_cmd / (double)cfg.steer_sign;
    const double psi = 10.0 * -delta * 0.05 / 4.0;
    const double epsi = -atan(0.1);
    bmpc_response_t p2;
    CHECK(fabs(delta) > 1e-3, "steering built up (%g)", delta);
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p2);
    CHECK(NEAR(g_c.last_s0[BMPC_SPSI], psi), "psi %g, want %g", (double)g_c.last_s0[BMPC_SPSI], psi);
    CHECK(NEAR(g_c.last_s0[BMPC_SEPSI], epsi + psi), "epsi %g, want %g", (double)g_c.last_s0[BMPC_SEPSI], epsi + psi);
    CHECK(NEAR(g_c.last_s0[BMPC_SV], 10.0 + 0.4 * (vt - 10.0) * 0.05), "v follows the previous target (%g)", (double)g_c.last_s0[BMPC_SV]);
    bmpc_controller_init(&g_c, &cfg);
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  }

  /* 응답 필드: delta0 는 MPC 부호, steer_cmd 는 후처리 뒤 MORAI 부호 */
  CHECK(p.delta0 == g_c.steering_angle && p.vt_raw == g_c.target_velocity && p.vt_final == g_c.target_velocity, "response fields");
  CHECK(p.steer_cmd == g_c.post.prev_steer_cmd, "steer_cmd is the post-processed command");
  CHECK(p.steer_cmd * p.delta0 <= 0, "MORAI sign is opposite to the MPC sign");
  CHECK(p.seq == 1 && p.iters == (uint8_t)g_c.result.qp_iters, "seq and iterations");

  /* warm start: 한 칸 밀고 마지막은 반복 (두 묶음 모두) */
  nu = cfg.N - 1;
  CHECK(g_c.warm_valid == 1, "warm start valid");
  for (i = 0; i < nu; ++i) {
    const int src = i + 1 < nu ? i + 1 : nu - 1;
    CHECK(g_c.z_warm[i] == g_c.result.z[src], "steer warm %d", i);
    CHECK(g_c.z_warm[nu + i] == g_c.result.z[nu + src], "speed warm %d", i);
  }

  /* 플래그가 가중치를 고른다: 같은 문제를 직접 풀어 비교한다 */
  {
    static const struct {
      uint8_t flags;
      int highway, avoidance;
    } cases[] = {{0u, 0, 0},
                 {BMPC_FLAG_HIGHWAY, 1, 0},
                 {BMPC_FLAG_AVOIDANCE, 0, 1},
                 {BMPC_FLAG_HIGHWAY | BMPC_FLAG_AVOIDANCE, 1, 1},
                 {BMPC_FLAG_KICK, 0, 0}};
    size_t n;
    bmpc_config_default(&cfg);
    cfg.dc_enabled = 0;
    for (n = 0; n < sizeof(cases) / sizeof(cases[0]); ++n) {
      bmpc_problem_t pr;
      bmpc_controller_init(&g_c, &cfg);
      make_req(&r, 1, 1.0, 10.0);
      r.c[2] = (bmpc_real)0.02;
      r.flags = cases[n].flags;
      (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
      memset(&pr, 0, sizeof(pr));
      memcpy(pr.s0, g_c.last_s0, sizeof(pr.s0));
      memcpy(pr.c, r.c, sizeof(pr.c));
      pr.ref_v = r.ref_v;
      pr.v_lb = r.v_lb;
      pr.v_ub = r.v_ub;
      pr.steer_lb = g_c.cfg.steer_lb;
      pr.steer_ub = g_c.cfg.steer_ub;
      pr.prev_delta_ref = 0;
      bmpc_compute_weights(&g_c.cfg, cases[n].highway, cases[n].avoidance, r.velocity, &pr.w);
      (void)bmpc_mpc_solve(&g_c.cfg, &pr, NULL, &g_work, &g_res);
      CHECK(memcmp(g_res.z, g_c.result.z, sizeof(bmpc_real) * (size_t)g_res.nz) == 0, "flags 0x%02x select the weights", cases[n].flags);
    }
  }

  /* RESET: warm start 없이 푼다. 없을 때는 warm start 로 푼다. sqp 1 번이면 시작점이 답에 드러난다 */
  {
    int reset;
    bmpc_real warm_saved[BMPC_NZ_MAX];
    bmpc_config_default(&cfg);
    cfg.dc_enabled = 0;
    cfg.sqp_iters = 1;
    for (reset = 0; reset < 2; ++reset) {
      bmpc_problem_t pr;
      bmpc_controller_init(&g_c, &cfg);
      warm_up(&g_c, &k, 1.0, 4);
      memcpy(warm_saved, g_c.z_warm, sizeof(warm_saved));
      make_req(&r, 9, -1.0, 8.0);
      r.flags = reset ? BMPC_FLAG_RESET : 0u;
      memset(&pr, 0, sizeof(pr));
      pr.prev_delta_ref = g_c.post.prev_steer_cmd / g_c.cfg.steer_sign;
      (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
      memcpy(pr.s0, g_c.last_s0, sizeof(pr.s0));
      memcpy(pr.c, r.c, sizeof(pr.c));
      pr.ref_v = r.ref_v;
      pr.v_lb = r.v_lb;
      pr.v_ub = r.v_ub;
      pr.steer_lb = g_c.cfg.steer_lb;
      pr.steer_ub = g_c.cfg.steer_ub;
      bmpc_compute_weights(&g_c.cfg, 0, 0, r.velocity, &pr.w);
      (void)bmpc_mpc_solve(&g_c.cfg, &pr, reset ? NULL : warm_saved, &g_work, &g_res);
      CHECK(memcmp(g_res.z, g_c.result.z, sizeof(bmpc_real) * (size_t)g_res.nz) == 0, "reset=%d uses the right start point", reset);
    }
  }

  /* snap, post_cap: 0 도 유효한 값이다, 음수 = 없음, 상한이 snap 보다 우선 */
  {
    static const struct {
      double snap, cap, want; /* want < 0: vt_raw 그대로 */
    } cases[] = {{-1, -1, -1}, {5, -1, 5}, {0, -1, 0}, {-1, 3, 3}, {-1, 0, 0}, {5, 3, 3}, {3, 5, 3}, {-1, 100, -1}};
    size_t n;
    bmpc_config_default(&cfg);
    for (n = 0; n < sizeof(cases) / sizeof(cases[0]); ++n) {
      bmpc_controller_init(&g_c, &cfg);
      make_req(&r, 1, 0, 8.0);
      r.snap = (bmpc_real)cases[n].snap;
      r.post_cap = (bmpc_real)cases[n].cap;
      (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
      CHECK(p.vt_raw > 7, "raw target ignores snap and cap (%g)", (double)p.vt_raw);
      CHECK(NEAR(p.vt_final, cases[n].want < 0 ? (double)p.vt_raw : cases[n].want), "snap %g cap %g: %g", cases[n].snap, cases[n].cap, (double)p.vt_final);
    }
  }

  /* 속도 하한이 상한보다 크면 상한에 맞춘다 (풀이 실패가 아니다) */
  bmpc_config_default(&cfg);
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0, 8.0);
  r.v_lb = 5;
  r.v_ub = 3;
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_OK && NEAR(p.vt_raw, 3), "v_lb above v_ub (%d, %g)", st, (double)p.vt_raw);

  /* 음의 전송 시간은 0 */
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0, 8.0);
  r.transport = -1;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.state_delay, 1.0 * 0.044 - 0.004 + 0.01), "negative transport ignored (%g)", (double)p.state_delay);

  /* 반복 수는 u8 로 포화 (줄어서 작은 값이 되면 안 된다) */
  bmpc_config_default(&cfg);
  cfg.sqp_iters = 300;
  cfg.sqp_tol = -1;
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 1.0, 8.0);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(g_c.result.qp_iters > 255 && p.iters == 255, "iterations saturate (%d -> %d)", g_c.result.qp_iters, p.iters);
}

static void test_ctrl_failures(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  bmpc_real delta0, vt0, lb, ub;
  int st;
  printf("[controller: solve failures]\n");
  bmpc_config_default(&cfg);

  /* 풀이 실패: 직전 조향·목표 속도 유지, warm start 버림, 오류 수, 다음 주기에 회복 */
  bmpc_controller_init(&g_c, &cfg);
  warm_up(&g_c, &k, 1.0, 5);
  delta0 = g_c.steering_angle;
  vt0 = g_c.target_velocity;
  CHECK(BM_FABS_(delta0) > 0.01 && g_c.warm_valid == 1, "prepared");
  make_req(&r, 50, 1.0, 8.0);
  r.c[1] = (bmpc_real)1e10; /* 유한하지만 비용이 넘친다 */
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_ERR_NUMERIC && p.status == BMPC_ERR_NUMERIC, "numeric failure status %d", st);
  CHECK(g_c.errors == 1u && g_c.warm_valid == 0, "failure counted, warm start dropped");
  CHECK(p.delta0 == delta0 && NEAR(p.vt_raw, vt0), "steering and target held (%g, %g)", (double)p.delta0, (double)p.vt_raw);
  CHECK(isfinite(p.steer_cmd) && p.steer_cmd <= g_c.cfg.steer_ub && p.steer_cmd >= g_c.cfg.steer_lb, "command in range (%g)", (double)p.steer_cmd);
  k.t += 50000;
  make_req(&r, 51, 1.0, 8.0);
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_OK && g_c.warm_valid == 1 && g_c.errors == 1u, "recovers on the next request (%d)", st);

  /* 모순된 범위: INFEASIBLE 을 그대로 보고하고 유지 */
  bmpc_controller_init(&g_c, &cfg);
  warm_up(&g_c, &k, 1.0, 5);
  delta0 = g_c.steering_angle;
  vt0 = g_c.target_velocity;
  lb = g_c.cfg.steer_lb;
  ub = g_c.cfg.steer_ub;
  g_c.cfg.steer_lb = 1;
  g_c.cfg.steer_ub = 0;
  make_req(&r, 60, 1.0, 8.0);
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  g_c.cfg.steer_lb = lb;
  g_c.cfg.steer_ub = ub;
  CHECK(st == BMPC_ERR_INFEASIBLE && p.status == BMPC_ERR_INFEASIBLE, "infeasible status %d", st);
  CHECK(g_c.errors == 1u && g_c.warm_valid == 0, "failure counted");
  CHECK(p.delta0 == delta0 && NEAR(p.vt_raw, vt0), "held (%g, %g)", (double)p.delta0, (double)p.vt_raw);

  /* QP 반복 상한: 그때까지의 최선 해를 쓴다. 오류로 세지 않고 warm start 도 유지 */
  bmpc_config_default(&cfg);
  cfg.qp_max_iters = 1;
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 70, 2.0, 8.0);
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_ERR_MAXITER && p.status == BMPC_ERR_MAXITER, "max iterations status %d", st);
  CHECK(g_c.errors == 0u && g_c.warm_valid == 1, "best solution is used");
  CHECK(BM_FABS_(p.delta0) > 0.01, "best solution steers (%g)", (double)p.delta0);

  /* Lf = 0 같은 잘못된 설정: 오류 상태, 값은 유한 */
  bmpc_config_default(&cfg);
  cfg.Lf = 0;
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 80, 1.0, 8.0);
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_ERR_NUMERIC, "Lf 0 fails cleanly (%d)", st);
  CHECK(isfinite(p.steer_cmd) && isfinite(p.accel) && isfinite(p.brake), "finite output");
}

static void test_ctrl_stop(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  bmpc_real delta0, compute0, lim;
  int st, i, qp0;
  printf("[controller: stop flag]\n");
  bmpc_config_default(&cfg);
  bmpc_controller_init(&g_c, &cfg);
  warm_up(&g_c, &k, 1.0, 5);
  delta0 = g_c.steering_angle;
  compute0 = g_c.compute_s;
  qp0 = g_c.result.qp_iters;
  k.step = 1000; /* 풀었다면 solve_us 가 0 이 아니다 */

  make_req(&r, 90, 1.0, 8.0);
  r.flags = BMPC_FLAG_STOP;
  r.snap = 5; /* 정지가 우선 */
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_OK && p.solve_us == 0u && p.iters == 0u && g_c.result.qp_iters == qp0, "MPC is not solved");
  CHECK(p.delta0 == delta0 && p.vt_raw == 0 && p.vt_final == 0, "steering held, target zero");
  CHECK(p.accel == 0 && NEAR(p.brake, 0.1), "brake ramp starts (%g %g)", (double)p.accel, (double)p.brake);
  CHECK(g_c.warm_valid == 0, "warm start dropped");
  CHECK(g_c.compute_s == compute0, "no delay sample without a prediction");
  lim = 0;
  for (i = 0; i < 10; ++i) {
    r.seq = (uint16_t)(91 + i);
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
    lim = p.brake;
  }
  CHECK(NEAR(lim, cfg.lg_brake_limit), "stop brake limit %g", (double)lim);
  r.flags = BMPC_FLAG_STOP | BMPC_FLAG_KICK;
  for (i = 0; i < 10; ++i) (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.brake, cfg.lg_kick_brake_limit), "kick brake limit %g", (double)p.brake);

  /* 다시 풀면 warm start 없이 시작 */
  make_req(&r, 120, 1.0, 8.0);
  st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(st == BMPC_OK && g_c.warm_valid == 1 && p.iters > 0u, "resume after stop");
}

static void test_ctrl_flags_longitudinal(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  int i;
  printf("[controller: flags on longitudinal limits]\n");

  /* 속도 가드: 예외 구간 밖에서 설정이 켜져 있을 때만 */
  bmpc_config_default(&cfg);
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0, 17.0);
  r.ref_v = 25;
  r.v_ub = 25;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(p.accel == 0 && NEAR(p.brake, cfg.legal_brake), "guard active above the legal speed (%g %g)", (double)p.accel, (double)p.brake);
  bmpc_controller_init(&g_c, &cfg);
  r.flags = BMPC_FLAG_HIGHWAY;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(p.accel > 0 && p.brake == 0, "highway exempt (%g %g)", (double)p.accel, (double)p.brake);
  cfg.legal_enabled = 0;
  bmpc_controller_init(&g_c, &cfg);
  r.flags = 0;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(p.accel > 0 && p.brake == 0, "guard disabled by config (%g %g)", (double)p.accel, (double)p.brake);

  /* 가속 상한 */
  bmpc_config_default(&cfg);
  cfg.lg_accel_limit = (bmpc_real)0.2;
  cfg.lg_accel_limit_exempt = (bmpc_real)0.9;
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0, 1.0);
  r.ref_v = 30;
  r.v_ub = 30;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.accel, 0.2), "urban accel limit %g", (double)p.accel);
  bmpc_controller_init(&g_c, &cfg);
  r.flags = BMPC_FLAG_HIGHWAY;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.accel, 0.9), "highway accel limit %g", (double)p.accel);

  /* 제동 상한: 킥일 때만 큰 값 */
  bmpc_config_default(&cfg);
  cfg.legal_enabled = 0;
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0, 20.0);
  r.ref_v = 1;
  r.v_ub = 1;
  for (i = 0; i < 12; ++i) {
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
    k.t += 50000;
  }
  CHECK(NEAR(p.brake, cfg.lg_brake_limit), "normal brake limit %g", (double)p.brake);
  r.flags = BMPC_FLAG_KICK;
  for (i = 0; i < 12; ++i) {
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
    k.t += 50000;
  }
  CHECK(NEAR(p.brake, cfg.lg_kick_brake_limit), "kick brake limit %g", (double)p.brake);
}

static void test_ctrl_delay_measurement(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  double est;
  printf("[controller: delay measurement]\n");
  bmpc_config_default(&cfg);
  bmpc_controller_init(&g_c, &cfg);
  CHECK(NEAR(g_c.compute_s, cfg.dc_compute_init), "initial estimate");

  /* now_fn 호출: 예측(0), 풀이 전(1), 풀이 뒤(2), 이력 기록(3), 끝(4) */
  k.step = 20000;
  make_req(&r, 1, 0, 8.0);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(p.solve_us == 20000u && p.total_us == 80000u, "solve %u total %u us", (unsigned)p.solve_us, (unsigned)p.total_us);
  CHECK(NEAR(p.compute, cfg.dc_compute_init), "this cycle used the old estimate (%g)", (double)p.compute);
  CHECK(NEAR(g_c.compute_s, 0.06), "estimate jumps up to a larger sample (%g)", (double)g_c.compute_s);

  /* 내려갈 때는 10 % 씩 */
  k.step = 1000;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  est = 0.06 + 0.1 * (0.003 - 0.06);
  CHECK(NEAR(g_c.compute_s, est), "estimate falls slowly (%g, want %g)", (double)g_c.compute_s, est);
  CHECK(NEAR(p.compute, 0.06), "response reports the estimate it used (%g)", (double)p.compute);

  /* 전송 시간: 응답 몫(1 - share)은 표본에, 요청 몫(share)은 상태 지연에 */
  bmpc_controller_init(&g_c, &cfg);
  k.step = 20000;
  make_req(&r, 1, 0, 8.0);
  r.transport = (bmpc_real)0.01;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(g_c.compute_s, 0.06 + 0.004), "response share in the sample (%g)", (double)g_c.compute_s);
  CHECK(NEAR(p.state_delay, 1.0 * 0.044 - 0.004 + 0.01 + 0.006), "request share in the state delay (%g)", (double)p.state_delay);
  CHECK(NEAR(p.actuation, 0.06 + 0.5 * 0.044), "actuation delay (%g)", (double)p.actuation);

  /* 표본 상한 */
  bmpc_controller_init(&g_c, &cfg);
  k.step = 100000;
  make_req(&r, 1, 0, 8.0);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(g_c.compute_s, cfg.dc_compute_max), "sample capped (%g)", (double)g_c.compute_s);
  k.step = 0;

  /* 고정값이 있으면 고정값, 0 이면 실측 */
  cfg.dc_compute_fixed = (bmpc_real)0.02;
  bmpc_controller_init(&g_c, &cfg);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.compute, 0.02), "fixed compute delay (%g)", (double)p.compute);
  cfg.dc_compute_fixed = 0;
  bmpc_controller_init(&g_c, &cfg);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.compute, cfg.dc_compute_init), "fixed 0 means measured (%g)", (double)p.compute);

  /* 실효 Lf */
  bmpc_config_default(&cfg);
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 0, 8.0);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.lf_used, cfg.dc_model_lf), "no estimate: model Lf (%g)", (double)p.lf_used);
  r.lf_est = (bmpc_real)4.4;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(NEAR(p.lf_used, 4.4), "online Lf (%g)", (double)p.lf_used);
}

static void test_ctrl_history(void)
{
  bmpc_config_t cfg;
  bmpc_request_t r;
  bmpc_response_t p;
  clk_t k = {1000000, 0};
  int i;
  printf("[controller: command history]\n");
  bmpc_config_default(&cfg);
  bmpc_controller_init(&g_c, &cfg);
  make_req(&r, 1, 1.0, 8.0);

  /* 첫 명령이 하나 들어간다. 값은 MPC 부호 */
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(g_c.hist.count == 1, "one entry after the first step (%d)", g_c.hist.count);
  CHECK(NEAR(bmpc_hist_delta(&g_c.hist, 0), p.steer_cmd / cfg.steer_sign), "stored in the MPC sign");
  CHECK(bmpc_hist_time(&g_c.hist, 0) == 1000000, "stored at now");

  /* 0.5 s 간격: 두 번째 항목이 1 s 넘게 지나야 맨 앞을 버린다 */
  for (i = 1; i <= 3; ++i) {
    k.t = 1000000 + (int64_t)i * 500000;
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  }
  CHECK(g_c.hist.count == 4, "second entry exactly 1 s old is kept (%d)", g_c.hist.count);
  k.t = 3000000;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(g_c.hist.count == 4 && bmpc_hist_time(&g_c.hist, 0) == 1500000, "older entries dropped (%d, %lld)", g_c.hist.count,
        (long long)bmpc_hist_time(&g_c.hist, 0));

  /* 1 s 넘게 끊겼다 오면 이력을 비운다 */
  k.t = 5000000;
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(g_c.hist.count == 1 && bmpc_hist_time(&g_c.hist, 0) == 5000000, "history restarts after a gap (%d)", g_c.hist.count);

  /* 크기가 1 이 아닌 부호: 이력과 직전 조향 기준값은 부호로 나눈 값이다 */
  bmpc_config_default(&cfg);
  cfg.steer_sign = (bmpc_real)-0.5;
  cfg.dc_enabled = 0;
  bmpc_controller_init(&g_c, &cfg);
  k.t = 1000000;
  make_req(&r, 1, 1.0, 8.0);
  (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
  CHECK(fabs((double)p.steer_cmd) > 1e-3 && NEAR(bmpc_hist_delta(&g_c.hist, 0), p.steer_cmd / cfg.steer_sign), "history divided by the sign (%g)",
        (double)bmpc_hist_delta(&g_c.hist, 0));
  {
    bmpc_problem_t pr;
    const bmpc_real ref = g_c.post.prev_steer_cmd / cfg.steer_sign;
    make_req(&r, 2, 1.0, 8.0);
    r.flags = BMPC_FLAG_RESET;
    k.t += 50000;
    (void)bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
    memset(&pr, 0, sizeof(pr));
    memcpy(pr.s0, g_c.last_s0, sizeof(pr.s0));
    memcpy(pr.c, r.c, sizeof(pr.c));
    pr.ref_v = r.ref_v;
    pr.v_lb = r.v_lb;
    pr.v_ub = r.v_ub;
    pr.steer_lb = g_c.cfg.steer_lb;
    pr.steer_ub = g_c.cfg.steer_ub;
    pr.prev_delta_ref = ref;
    bmpc_compute_weights(&g_c.cfg, 0, 0, r.velocity, &pr.w);
    (void)bmpc_mpc_solve(&g_c.cfg, &pr, NULL, &g_work, &g_res);
    CHECK(memcmp(g_res.z, g_c.result.z, sizeof(bmpc_real) * (size_t)g_res.nz) == 0, "previous steering reference divided by the sign");
  }

  /* 부호 0 은 "없음": 나누지 않고 이력도 쌓지 않는다 */
  bmpc_config_default(&cfg);
  cfg.steer_sign = 0;
  bmpc_controller_init(&g_c, &cfg);
  k.t = 1000000;
  for (i = 0; i < 3; ++i) {
    int st;
    make_req(&r, (uint16_t)i, 0.0, 8.0);
    st = bmpc_controller_step(&g_c, &r, clk_now, &k, &p);
    k.t += 50000;
    CHECK(st == BMPC_OK && p.steer_cmd == 0 && isfinite(p.delta0), "sign 0: status %d, steer %g", st, (double)p.steer_cmd);
  }
  CHECK(g_c.hist.count == 0, "sign 0 stores no history");
  CHECK(bmpc_controller_note_applied(&g_c, k.t, (bmpc_real)0.1) == 0, "sign 0 rejects notes");

  /* 부호 크기 문턱(1e-6) 은 0 과 똑같이 다룬다 */
  {
    int s;
    double angle[2] = {0, 0};
    for (s = 0; s < 2; ++s) {
      bmpc_controller_t *c = s ? &g_c2 : &g_c;
      bmpc_config_default(&cfg);
      cfg.steer_sign = s ? (bmpc_real)1e-6 : (bmpc_real)0;
      bmpc_controller_init(c, &cfg);
      k.t = 1000000;
      for (i = 0; i < 4; ++i) {
        make_req(&r, (uint16_t)i, 1.0, 8.0);
        (void)bmpc_controller_step(c, &r, clk_now, &k, &p);
        k.t += 50000;
      }
      angle[s] = (double)c->steering_angle;
    }
    CHECK(g_c2.hist.count == 0, "sign at the threshold stores no history (%d)", g_c2.hist.count);
    CHECK(angle[0] == angle[1] && fabs(angle[0]) > 0.01, "sign at the threshold behaves as zero (%g, %g)", angle[0], angle[1]);
    CHECK(bmpc_controller_note_applied(&g_c2, k.t, (bmpc_real)0.1) == 0, "sign at the threshold rejects notes");
  }
}

static void test_note_applied(void)
{
  bmpc_config_t cfg;
  const bmpc_real eps = (bmpc_real)2e-4;
  printf("[controller: note_applied]\n");
  bmpc_config_default(&cfg);

  /* 설정 전 */
  memset(&g_c, 0, sizeof(g_c));
  g_c.cfg.steer_sign = -1;
  CHECK(bmpc_controller_note_applied(&g_c, 100, (bmpc_real)0.1) == 0, "not configured");
  CHECK(g_c.hist.count == 0, "nothing stored");

  /* 빈 이력이면 값이 0 이어도 넣는다 */
  bmpc_controller_init(&g_c, &cfg);
  CHECK(bmpc_controller_note_applied(&g_c, 100, 0) == 1 && g_c.hist.count == 1, "empty history takes any value");
  CHECK(bmpc_hist_time(&g_c.hist, 0) == 100 && bmpc_hist_delta(&g_c.hist, 0) == 0, "stored");

  /* 차이가 양자화 문턱보다 작으면 같은 명령, 같으면 다른 명령. 부호를 바꿔 넣는다 */
  CHECK(bmpc_controller_note_applied(&g_c, 200, eps / 2) == 0 && g_c.hist.count == 1, "below the threshold");
  CHECK(bmpc_controller_note_applied(&g_c, 200, -eps) == 1 && g_c.hist.count == 2, "at the threshold");
  CHECK(NEAR(bmpc_hist_delta(&g_c.hist, 1), eps), "stored divided by the sign (%g)", (double)bmpc_hist_delta(&g_c.hist, 1));

  /* 비교 대상은 마지막 이력 값이다: 0 이 아닌 값과 같은 명령이면 넣지 않고, 부호가 다르면 넣는다 */
  CHECK(bmpc_controller_note_applied(&g_c, 200, -eps) == 0 && g_c.hist.count == 2, "same as the last entry");
  CHECK(bmpc_controller_note_applied(&g_c, 200, eps) == 1 && g_c.hist.count == 3, "opposite sign is different");
  CHECK(bmpc_controller_note_applied(&g_c, 200, -eps) == 1 && g_c.hist.count == 4, "back again");

  /* 크기가 1 이 아닌 부호는 나눈다 */
  {
    bmpc_config_t c2 = cfg;
    c2.steer_sign = (bmpc_real)-0.5;
    bmpc_controller_init(&g_c2, &c2);
    CHECK(bmpc_controller_note_applied(&g_c2, 100, (bmpc_real)0.1) == 1, "stored");
    CHECK(NEAR(bmpc_hist_delta(&g_c2.hist, 0), -0.2), "note divided by the sign (%g)", (double)bmpc_hist_delta(&g_c2.hist, 0));
  }

  /* 시각이 되돌아가면 넣지 않는다. 같은 시각은 넣는다 */
  CHECK(bmpc_controller_note_applied(&g_c, 199, (bmpc_real)0.2) == 0 && g_c.hist.count == 4, "older time rejected");
  CHECK(bmpc_controller_note_applied(&g_c, 200, (bmpc_real)0.2) == 1 && g_c.hist.count == 5, "same time accepted");
}

/* ---------------- 4. 앱 ---------------- */
static bmpc_can_frame_t g_out[64];
static int g_nout = 0;
static clk_t g_clk = {1000000, 0};
static bmpc_app_t g_app, g_app_snap;

static int64_t app_now(void *ctx)
{
  CHECK(ctx == (void *)&g_clk, "HAL context passed to the clock");
  return clk_now(ctx);
}
static int app_send(void *ctx, const bmpc_can_frame_t *f)
{
  CHECK(ctx == (void *)&g_clk, "HAL context passed to send");
  if (g_nout < 64) g_out[g_nout++] = *f;
  return 0;
}

static void app_fresh(void)
{
  bmpc_hal_t hal;
  hal.now_us = app_now;
  hal.can_send = app_send;
  hal.ctx = &g_clk;
  bmpc_app_init(&g_app, &hal);
  g_nout = 0;
  g_clk.step = 0;
}

static void app_set(const bmpc_config_t *c, uint16_t idx)
{
  bmpc_can_frame_t f;
  bmpc_encode_cfg_set(idx, bmpc_config_get(c, idx), &f);
  bmpc_app_on_frame(&g_app, &f);
}

static void app_commit(uint32_t hash, uint16_t count)
{
  bmpc_can_frame_t f;
  bmpc_encode_cfg_commit(hash, count, &f);
  bmpc_app_on_frame(&g_app, &f);
}

static uint8_t last_ack(uint32_t *hash, uint16_t *count)
{
  uint8_t st = 0xFFu;
  CHECK(g_nout > 0 && g_out[g_nout - 1].id == BMPC_ID_CFG_ACK, "ack frame");
  bmpc_decode_cfg_ack(&g_out[g_nout - 1], hash, count, &st);
  return st;
}

static void app_configure(const bmpc_config_t *c)
{
  uint16_t i;
  for (i = 0; i < BMPC_CONFIG_COUNT; ++i) app_set(c, i);
  app_commit(bmpc_config_hash(c), (uint16_t)BMPC_CONFIG_COUNT);
}

static void test_app_config(void)
{
  bmpc_config_t c;
  bmpc_can_frame_t f;
  uint32_t h;
  uint16_t n, i;
  uint8_t st;
  uint32_t hash;
  const uint16_t all = (uint16_t)BMPC_CONFIG_COUNT;
  printf("[app: configuration]\n");
  bmpc_config_default(&c);
  c.sqp_iters = 2;
  hash = bmpc_config_hash(&c);

  /* 해시, PC 개수, MCU 개수, 전체 개수 조건을 하나씩 깨본다 */
  app_fresh();
  for (i = 0; i < all; ++i) app_set(&c, i);
  CHECK(g_app.cfg_count == all, "count %u", (unsigned)g_app.cfg_count);
  app_commit(hash ^ 0x100u, all);
  st = last_ack(&h, &n);
  CHECK(st == 1u && h == hash && n == all && g_app.ctrl.configured == 0, "wrong hash rejected, ack reports MCU values");
  CHECK(g_app.stats.configs == 0u, "failed commit not counted");
  CHECK(g_app.cfg_count == 0, "ready for the next set");

  app_fresh();
  for (i = 0; i < all; ++i) app_set(&c, i);
  app_commit(hash, (uint16_t)(all - 1u));
  st = last_ack(&h, &n);
  CHECK(st == 1u && n == all && g_app.ctrl.configured == 0, "PC count too small rejected");

  /* MCU 가 한 프레임을 놓쳤다: 기본값과 같은 항목이면 해시는 맞는다 */
  app_fresh();
  bmpc_config_default(&c);
  hash = bmpc_config_hash(&c);
  for (i = 1; i < all; ++i) app_set(&c, i);
  app_commit(hash, all);
  st = last_ack(&h, &n);
  CHECK(st == 1u && h == hash && n == (uint16_t)(all - 1u) && g_app.ctrl.configured == 0, "lost frame rejected though the hash matches");

  /* PC 와 MCU 가 둘 다 72 개로 같아도 전체가 아니면 거부 */
  app_fresh();
  for (i = 1; i < all; ++i) app_set(&c, i);
  app_commit(hash, (uint16_t)(all - 1u));
  st = last_ack(&h, &n);
  CHECK(st == 1u && g_app.ctrl.configured == 0, "partial set rejected though the counts agree");

  /* 같은 번호가 다시 오면 값은 덮고 개수는 그대로 */
  app_fresh();
  app_set(&c, 3);
  app_set(&c, 3);
  CHECK(g_app.cfg_count == 1, "duplicate index counted once (%u)", (unsigned)g_app.cfg_count);
  bmpc_encode_cfg_set(3, 0.7f, &f);
  bmpc_app_on_frame(&g_app, &f);
  CHECK(g_app.cfg_count == 1 && g_app.staging.k_v == (bmpc_real)0.7f, "duplicate overwrites the value (%g)", (double)g_app.staging.k_v);

  /* 범위 밖 번호는 세지 않는다 */
  app_fresh();
  memcpy(&g_app_snap, &g_app, sizeof(g_app));
  bmpc_encode_cfg_set(all, 1.0f, &f);
  bmpc_app_on_frame(&g_app, &f);
  bmpc_encode_cfg_set(0xFFFFu, 1.0f, &f);
  bmpc_app_on_frame(&g_app, &f);
  CHECK(memcmp(&g_app_snap, &g_app, sizeof(g_app)) == 0, "out-of-range index leaves the state alone");

  /* 성공하면 controller 를 그 설정으로 초기화. 실패한 다음 재전송으로 회복 */
  app_fresh();
  c.N = 20;
  app_commit(0u, all);
  CHECK(g_app.ctrl.configured == 0, "rejected");
  app_configure(&c);
  st = last_ack(&h, &n);
  CHECK(st == 0u && h == bmpc_config_hash(&c) && n == all, "accepted");
  CHECK(g_app.ctrl.configured == 1 && g_app.ctrl.cfg.N == 20 && g_app.stats.configs == 1u, "controller configured");

  /* 다시 설정하면 controller 를 처음부터 */
  {
    bmpc_request_t r;
    bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
    int k;
    make_req(&r, 1, 1.0, 8.0);
    bmpc_encode_request(&r, fr);
    for (k = 0; k < BMPC_REQ_FRAMES; ++k) bmpc_app_on_frame(&g_app, &fr[k]);
    CHECK(g_app.ctrl.hist.count == 1 && g_app.ctrl.steps == 1u, "ran a cycle");
    c.N = 25;
    app_configure(&c);
    CHECK(g_app.stats.configs == 2u && g_app.ctrl.cfg.N == 25, "reconfigured");
    CHECK(g_app.ctrl.hist.count == 0 && g_app.ctrl.steps == 0u && g_app.ctrl.post.have_prev == 0, "controller state reset");
  }

  /* 거부된 설정은 돌아가던 controller 를 건드리지 않는다 */
  c.N = 15;
  for (i = 0; i < all; ++i) app_set(&c, i);
  app_commit(hash ^ 1u, all);
  CHECK(g_app.ctrl.configured == 1 && g_app.ctrl.cfg.N == 25, "rejected set does not touch the running controller");
  /* 값은 남겨 둔다: 이번에는 번호만 다시 세면 된다 (헤더 주석) */
  CHECK(g_app.staging.N == 15, "staging values kept");
  CHECK(bmpc_config_hash(&g_app.staging) == bmpc_config_hash(&c), "staging = last sent");
}

static void test_app_frames(void)
{
  bmpc_config_t c;
  bmpc_request_t r;
  bmpc_response_t p;
  bmpc_can_frame_t fr[BMPC_REQ_FRAMES], f;
  int i;
  static const uint32_t foreign[] = {BMPC_ID_REQ_BASE - 1u, BMPC_ID_REQ_BASE + BMPC_REQ_FRAMES, BMPC_ID_RESP_BASE, BMPC_ID_CFG_ACK, 0u, 0x7FFu};
  size_t n;
  printf("[app: request assembly]\n");
  bmpc_config_default(&c);
  c.sqp_iters = 2;
  app_fresh();
  app_configure(&c);

  /* 마지막 프레임(0x105)이 와야 계산한다 */
  make_req(&r, 300, 0.5, 8.0);
  bmpc_encode_request(&r, fr);
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES - 1; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(g_nout == 0 && g_app.stats.requests == 0u && g_app.req_mask == 0x1Fu, "no response before the trigger");

  /* 요청 프레임이 아닌 ID 는 조립 상태를 건드리지 않는다 */
  for (n = 0; n < sizeof(foreign) / sizeof(foreign[0]); ++n) {
    memset(&f, 0x33, sizeof(f));
    f.id = foreign[n];
    f.dlc = 8;
    memcpy(&g_app_snap, &g_app, sizeof(g_app));
    bmpc_app_on_frame(&g_app, &f);
    CHECK(g_nout == 0 && memcmp(&g_app_snap, &g_app, sizeof(g_app)) == 0, "0x%03X ignored", (unsigned)foreign[n]);
  }
  bmpc_app_on_frame(&g_app, &fr[5]);
  CHECK(g_nout == BMPC_RESP_FRAMES && g_app.stats.requests == 1u && g_app.req_mask == 0u, "response after the trigger");
  CHECK(bmpc_decode_response(g_out, &p) == 0 && p.seq == 300 && p.status == BMPC_OK, "response %u %u", p.seq, p.status);
  for (i = 0; i < BMPC_RESP_FRAMES; ++i) CHECK(g_out[i].id == BMPC_ID_RESP_BASE + (uint32_t)i, "frame %d id", i);

  /* 처리한 프레임을 다시 쓰지 않는다: 트리거만 다시 오면 RX 오류, 첫 프레임이 없으니 seq 0 */
  g_nout = 0;
  bmpc_app_on_frame(&g_app, &fr[5]);
  CHECK(bmpc_decode_response(g_out, &p) == 0 && p.seq == 0 && p.status == BMPC_ERR_RX, "lone trigger after a request (%u %u)", p.seq, p.status);
  CHECK(g_app.stats.rx_errors == 1u && g_app.stats.requests == 2u, "counted");

  /* 첫 프레임이 오면 앞의 조각을 버린다 */
  g_nout = 0;
  bmpc_app_on_frame(&g_app, &fr[3]);
  bmpc_app_on_frame(&g_app, &fr[4]);
  CHECK(g_app.req_mask == 0x18u, "fragments collected");
  bmpc_app_on_frame(&g_app, &fr[0]);
  CHECK(g_app.req_mask == 0x01u, "first frame restarts the request");
  bmpc_app_on_frame(&g_app, &fr[5]);
  CHECK(bmpc_decode_response(g_out, &p) == 0 && p.seq == 300 && p.status == BMPC_ERR_RX, "fragments do not complete it (%u)", p.status);

  /* DLC 가 틀린 프레임은 수신 오류다 (다른 요청에는 영향이 없다) */
  make_req(&r, 301, 0.5, 8.0);
  bmpc_encode_request(&r, fr);
  fr[1].dlc = 7;
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(bmpc_decode_response(g_out, &p) == 0 && p.seq == 301 && p.status == BMPC_ERR_RX, "wrong DLC (%u %u)", p.seq, p.status);
}

static void test_app_stats(void)
{
  bmpc_config_t c;
  bmpc_request_t r;
  bmpc_response_t p;
  bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
  int i;
  uint32_t total0, solve0;
  printf("[app: statistics]\n");
  bmpc_config_default(&c);
  c.sqp_iters = 2;
  app_fresh();

  /* 설정 전 응답도 status != OK 로 센다 */
  make_req(&r, 1, 0.5, 8.0);
  bmpc_encode_request(&r, fr);
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(g_app.stats.solver_errors == 1u && g_app.stats.rx_errors == 0u, "not configured counts as non-OK (%u)", (unsigned)g_app.stats.solver_errors);

  app_configure(&c);
  g_app.stats.solver_errors = 0;

  /* 정상 요청은 오류가 아니다 */
  g_clk.step = 100;
  make_req(&r, 2, 0.5, 8.0);
  bmpc_encode_request(&r, fr);
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(bmpc_decode_response(g_out, &p) == 0 && p.status == BMPC_OK, "ok response");
  CHECK(g_app.stats.solver_errors == 0u, "OK is not a solver error");
  /* 처리 시간은 수신 완료부터: 앱(1) + controller(5) 뒤 앱이 다시 읽는다 = 6 칸 */
  CHECK(p.total_us == 600u && g_app.stats.last_total_us == 600u, "total %u us", (unsigned)p.total_us);
  CHECK(p.solve_us == 100u && g_app.stats.last_solve_us == 100u, "solve %u us", (unsigned)p.solve_us);
  CHECK(g_app.stats.max_total_us == 600u && g_app.stats.max_solve_us == 100u, "max recorded");
  total0 = g_app.stats.max_total_us;
  solve0 = g_app.stats.max_solve_us;

  /* 짧아지면 last 만, 길어지면 max 도 */
  g_clk.step = 10;
  g_clk.t += 50000;
  r.seq = 3;
  bmpc_encode_request(&r, fr);
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(g_app.stats.last_total_us == 60u && g_app.stats.last_solve_us == 10u, "last follows");
  CHECK(g_app.stats.max_total_us == total0 && g_app.stats.max_solve_us == solve0, "max stays");
  g_clk.step = 200;
  g_clk.t += 50000;
  r.seq = 4;
  bmpc_encode_request(&r, fr);
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(g_app.stats.max_total_us == 1200u && g_app.stats.max_solve_us == 200u, "max rises");
  g_clk.step = 0;

  /* 수치 오류는 solver_errors, 수신 오류는 rx_errors */
  make_req(&r, 5, 0.5, 8.0);
  r.c[0] = (bmpc_real)NAN;
  bmpc_encode_request(&r, fr);
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
  CHECK(bmpc_decode_response(g_out, &p) == 0 && p.status == BMPC_ERR_NUMERIC, "numeric response");
  CHECK(g_app.stats.solver_errors == 1u && g_app.stats.rx_errors == 0u, "solver error counted");
  CHECK(g_app.stats.requests == 5u, "requests %u", (unsigned)g_app.stats.requests);
}

/* ---------------- 5. 임의 요청 불변식 ---------------- */
/* CAN 으로 올 수 있는 어떤 요청이든 (CRC 는 맞게) 앱이 지켜야 하는 것. 값은 큰 범위에서 고른다. */
static uint32_t g_lcg = 20261105u;
static uint32_t rnd_u32(void)
{
  g_lcg = g_lcg * 1664525u + 1013904223u;
  return g_lcg ^ (g_lcg >> 15);
}
static double rnd01(void) { return (double)(rnd_u32() >> 8) / 16777216.0; }

/* 부호 있는 로그 균등 (1e-6 .. 1e38), 가끔 0, NaN, 무한대 */
static bmpc_real rnd_coef(void)
{
  const uint32_t k = rnd_u32() % 32u;
  const double mag = pow(10.0, -6.0 + 44.0 * rnd01());
  if (k == 0u) return 0;
  if (k == 1u) return (bmpc_real)NAN;
  if (k == 2u) return (bmpc_real)HUGE_VAL;
  return (bmpc_real)((rnd_u32() & 1u) ? mag : -mag);
}

/* 대부분은 운영 범위, 가끔 표현 범위 전체 */
static bmpc_real rnd_speed(void)
{
  const uint32_t k = rnd_u32() % 8u;
  if (k == 0u) return (bmpc_real)(rnd01() * 655.35);
  if (k == 1u) return 0;
  return (bmpc_real)(rnd01() * 30.0);
}

static void test_random_requests(void)
{
  bmpc_config_t c;
  int n, bad_status = 0, i;
  int counts[6] = {0, 0, 0, 0, 0, 0};
  double prev = 0;
  int have_prev = 0;
  printf("[random requests over CAN]\n");
  bmpc_config_default(&c);
  c.sqp_iters = 2;
  app_fresh();
  app_configure(&c);
  for (n = 0; n < 3000; ++n) {
    bmpc_request_t r;
    bmpc_response_t p;
    bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
    memset(&r, 0, sizeof(r));
    r.seq = (uint16_t)n;
    r.flags = (uint8_t)(rnd_u32() & 0x1Fu);
    /* 경로는 절반쯤 멀쩡한 값 */
    if (rnd_u32() & 1u) {
      r.c[0] = (bmpc_real)(rnd01() * 6.0 - 3.0);
      r.c[1] = (bmpc_real)(rnd01() * 2.0 - 1.0);
      r.c[2] = (bmpc_real)(rnd01() * 0.1 - 0.05);
      r.c[3] = (bmpc_real)(rnd01() * 2e-3 - 1e-3);
    } else {
      for (i = 0; i < 4; ++i) r.c[i] = rnd_coef();
    }
    r.x_max = (bmpc_real)(rnd01() * 100.0);
    r.ref_v = rnd_speed();
    r.v_lb = (rnd_u32() & 3u) ? 0 : rnd_speed();
    r.v_ub = rnd_speed();
    r.post_cap = (rnd_u32() & 3u) ? (bmpc_real)-1 : rnd_speed();
    r.snap = (rnd_u32() & 7u) ? (bmpc_real)-1 : rnd_speed();
    r.velocity = rnd_speed();
    r.yaw_rate = (bmpc_real)(rnd01() * 6.5 - 3.25);
    r.lf_est = (rnd_u32() & 1u) ? 0 : (bmpc_real)(rnd01() * 65.0);
    r.period = (bmpc_real)(rnd01() * 0.65);
    r.odom_age = (bmpc_real)(rnd01() * 0.65);
    r.transport = (bmpc_real)(rnd01() * 0.65);
    bmpc_encode_request(&r, fr);
    g_nout = 0;
    for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&g_app, &fr[i]);
    g_clk.t += 40000 + (int64_t)(rnd_u32() % 20000u);
    if (g_nout != BMPC_RESP_FRAMES || bmpc_decode_response(g_out, &p) != 0) {
      CHECK(0, "request %d: no valid response", n);
      continue;
    }
    if (p.status > BMPC_ERR_NUMERIC) ++bad_status;
    else ++counts[p.status];
    CHECK(p.seq == r.seq, "request %d: seq %u", n, p.seq);
    /* 조향: 유한, 범위 안, 변화율 안 (설정이 낸 값과 CAN f32 오차만큼 여유) */
    CHECK(isfinite(p.steer_cmd) && p.steer_cmd >= c.steer_lb - 1e-5 && p.steer_cmd <= c.steer_ub + 1e-5, "request %d: steer %g", n, (double)p.steer_cmd);
    if (have_prev) {
      CHECK(fabs((double)p.steer_cmd - prev) <= (double)c.st_rate_limit * (double)c.dt + 1e-5, "request %d: steer step %g -> %g", n, prev, (double)p.steer_cmd);
    }
    prev = (double)p.steer_cmd;
    have_prev = 1;
    /* 페달: 범위 안, 같이 밟지 않는다, 정지 플래그면 가속 없음 */
    CHECK(p.accel >= 0 && p.accel <= (r.flags & BMPC_FLAG_HIGHWAY ? c.lg_accel_limit_exempt : c.lg_accel_limit) + 1e-3, "request %d: accel %g", n, (double)p.accel);
    CHECK(p.brake >= 0 && p.brake <= c.lg_kick_brake_limit + 1e-3, "request %d: brake %g", n, (double)p.brake);
    CHECK(!(p.accel > 0 && p.brake > 0), "request %d: both pedals %g %g", n, (double)p.accel, (double)p.brake);
    if (r.flags & BMPC_FLAG_STOP) CHECK(p.accel == 0 && p.vt_final == 0, "request %d: stop flag", n);
    /* 유효하지 않은 경로는 오류로 답한다 */
    if (!(isfinite(r.c[0]) && isfinite(r.c[1]) && isfinite(r.c[2]) && isfinite(r.c[3])))
      CHECK(p.status == BMPC_ERR_NUMERIC, "request %d: non-finite path gives status %u", n, p.status);
  }
  CHECK(bad_status == 0, "unexpected status %d times", bad_status);
  CHECK(g_app.stats.requests == 3000u, "all answered");
  printf("  3000 requests: OK %d, MAXITER %d, INFEASIBLE %d, NUMERIC %d\n", counts[0], counts[1], counts[2], counts[3]);
}

int main(void)
{
  test_steer_post();
  test_throttle_stop_and_limits();
  test_throttle_integral();
  test_throttle_ff();
  test_throttle_deadband();
  test_legal_guard();
  test_weights_more();
  test_ctrl_input_checks();
  test_ctrl_outputs();
  test_ctrl_failures();
  test_ctrl_stop();
  test_ctrl_flags_longitudinal();
  test_ctrl_delay_measurement();
  test_ctrl_history();
  test_note_applied();
  test_app_config();
  test_app_frames();
  test_app_stats();
  test_random_requests();
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
