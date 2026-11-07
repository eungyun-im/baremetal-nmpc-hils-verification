/* bare_metal_mpc core 단위 시험 (ROS 없이 실행).
 *   1. CAN 프레임 왕복과 CRC
 *   2. 설정 전송·해시
 *   3. 상하한 QP: 최적 조건(KKT)과 투영 경사법 기준해 비교
 *   4. 가중치 블렌딩 값
 *   5. MPC 한 번 풀이: 직선·곡선에서 부호와 크기
 *   6. 간이 폐루프: 원형 경로를 따라가며 오차가 줄어드는지 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bmpc/bmpc_controller.h"
#include "bmpc/bmpc_protocol.h"
#include "bmpc/bmpc_qp.h"
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

static double urand(double a, double b) { return a + (b - a) * ((double)rand() / (double)RAND_MAX); }

/* ---------------- 1. 프로토콜 ---------------- */
static void test_protocol(void)
{
  int trial;
  printf("[protocol]\n");
  for (trial = 0; trial < 2000; ++trial) {
    bmpc_request_t a, b;
    bmpc_response_t ra, rb;
    bmpc_can_frame_t fr[BMPC_REQ_FRAMES], fs[BMPC_RESP_FRAMES];
    memset(&a, 0, sizeof(a));
    a.seq = (uint16_t)(rand() & 0xFFFF);
    a.flags = (uint8_t)(rand() & 0x1F);
    a.c[0] = urand(-3, 3);
    a.c[1] = urand(-1, 1);
    a.c[2] = urand(-0.05, 0.05);
    a.c[3] = urand(-1e-3, 1e-3);
    a.x_max = urand(4, 60);
    a.path_counter = (uint16_t)(rand() & 0xFFFF);
    a.ref_v = urand(0, 25);
    a.v_lb = urand(0, 5);
    a.v_ub = urand(5, 40);
    a.post_cap = (rand() & 1) ? urand(0, 25) : -1;
    a.snap = (rand() & 1) ? urand(0, 25) : -1;
    a.velocity = urand(0, 26);
    a.yaw_rate = urand(-1, 1);
    a.lf_est = (rand() & 1) ? urand(3.5, 5) : 0;
    a.period = urand(0.01, 0.1);
    a.odom_age = urand(0, 0.2);
    a.transport = urand(0, 0.03);
    bmpc_encode_request(&a, fr);
    CHECK(bmpc_decode_request(fr, &b) == 0, "decode request");
    CHECK(b.seq == a.seq && b.flags == a.flags && b.path_counter == a.path_counter, "request ints");
    CHECK(fabs(b.c[0] - a.c[0]) <= 1e-6 * fabs(a.c[0]) + 1e-12, "c0 %g %g", a.c[0], b.c[0]);
    CHECK(fabs(b.c[3] - a.c[3]) <= 1e-6 * fabs(a.c[3]) + 1e-15, "c3");
    CHECK(fabs(b.ref_v - a.ref_v) <= 0.005 + 1e-9 && fabs(b.v_ub - a.v_ub) <= 0.005 + 1e-9, "speeds");
    CHECK((a.post_cap < 0) == (b.post_cap < 0) && (a.post_cap < 0 || fabs(b.post_cap - a.post_cap) <= 0.005 + 1e-9),
          "post_cap %g %g", a.post_cap, b.post_cap);
    CHECK((a.snap < 0) == (b.snap < 0), "snap none");
    CHECK(fabs(b.yaw_rate - a.yaw_rate) <= 0.5e-4 + 1e-12, "yaw rate");
    CHECK(fabs(b.period - a.period) <= 0.5e-5 + 1e-12 && fabs(b.transport - a.transport) <= 0.5e-5 + 1e-12, "times");
    CHECK(fabs(b.lf_est - a.lf_est) <= 0.5e-3 + 1e-12, "lf");

    /* 한 비트만 바꿔도 CRC 로 걸러야 한다 */
    fr[rand() % BMPC_REQ_FRAMES].data[rand() % 6] ^= (uint8_t)(1u << (rand() % 8));
    CHECK(bmpc_decode_request(fr, &b) == -2, "crc must catch a flipped bit");

    memset(&ra, 0, sizeof(ra));
    ra.seq = a.seq;
    ra.status = (uint8_t)(rand() % 6);
    ra.iters = (uint8_t)(rand() & 0xFF);
    ra.steer_cmd = urand(-0.7, 0.7);
    ra.accel = urand(0, 1);
    ra.brake = urand(0, 1);
    ra.delta0 = urand(-0.7, 0.7);
    ra.vt_final = urand(0, 25);
    ra.vt_raw = urand(0, 25);
    ra.state_delay = urand(0, 0.12);
    ra.compute = urand(0, 0.1);
    ra.actuation = urand(0, 0.12);
    ra.lf_used = urand(3, 5);
    ra.solve_us = (uint32_t)(rand() % 70000);
    ra.total_us = (uint32_t)(rand() % 70000);
    bmpc_encode_response(&ra, fs);
    CHECK(bmpc_decode_response(fs, &rb) == 0, "decode response");
    CHECK(rb.seq == ra.seq && rb.status == ra.status && rb.iters == ra.iters, "response ints");
    CHECK(fabs(rb.steer_cmd - ra.steer_cmd) <= 1e-7, "steer f32");
    CHECK(fabs(rb.accel - ra.accel) <= 0.5e-4 + 1e-12 && fabs(rb.brake - ra.brake) <= 0.5e-4 + 1e-12, "pedals");
    CHECK(fabs(rb.delta0 - ra.delta0) <= 0.5e-4 + 1e-12, "delta0");
    CHECK(rb.solve_us == (ra.solve_us > 0xFFFF ? 0xFFFF : ra.solve_us), "solve_us saturate");
  }
}

/* ---------------- 2. 설정 ---------------- */
static void test_config(void)
{
  bmpc_config_t a, b;
  uint16_t i;
  printf("[config] %d fields\n", (int)BMPC_CONFIG_COUNT);
  bmpc_config_default(&a);
  memset(&b, 0, sizeof(b));
  for (i = 0; i < BMPC_CONFIG_COUNT; ++i) {
    bmpc_can_frame_t f;
    uint16_t idx;
    float v;
    bmpc_encode_cfg_set(i, bmpc_config_get(&a, i), &f);
    bmpc_decode_cfg_set(&f, &idx, &v);
    CHECK(bmpc_config_set(&b, idx, v) == 0, "set %u", i);
  }
  CHECK(bmpc_config_hash(&a) == bmpc_config_hash(&b), "hash after transfer");
  CHECK(b.N == 30 && fabs(b.Lf - 4.0) < 1e-9 && b.dc_enabled == 1, "values");
  b.u_epsi = 79999.0;
  CHECK(bmpc_config_hash(&a) != bmpc_config_hash(&b), "hash must change");
  CHECK(bmpc_config_set(&b, (uint16_t)BMPC_CONFIG_COUNT, 1.0f) == -1, "out of range index");
}

/* ---------------- 3. QP ---------------- */
static double qp_obj(int n, const double *P, const double *q, const double *x)
{
  double f = 0;
  int i, j;
  for (i = 0; i < n; ++i) {
    double s = 0;
    for (j = 0; j < n; ++j) s += P[i * n + j] * x[j];
    f += 0.5 * x[i] * s + q[i] * x[i];
  }
  return f;
}

/* 기준해: 가속 투영 경사법 (FISTA) 을 충분히 오래 돌린다 */
static void qp_reference(int n, const double *P, const double *q, const double *lb, const double *ub, double *x)
{
  static double y[BMPC_NZ_MAX], xn[BMPC_NZ_MAX], g[BMPC_NZ_MAX];
  double L = 0, t = 1;
  int i, j, it;
  for (i = 0; i < n; ++i) {
    double s = 0;
    for (j = 0; j < n; ++j) s += fabs(P[i * n + j]);
    if (s > L) L = s;
  }
  for (i = 0; i < n; ++i) x[i] = y[i] = fmin(fmax(0.0, lb[i]), ub[i]);
  for (it = 0; it < 50000; ++it) {
    double tn;
    for (i = 0; i < n; ++i) {
      double s = q[i];
      for (j = 0; j < n; ++j) s += P[i * n + j] * y[j];
      g[i] = s;
    }
    for (i = 0; i < n; ++i) xn[i] = fmin(fmax(y[i] - g[i] / L, lb[i]), ub[i]);
    tn = 0.5 * (1 + sqrt(1 + 4 * t * t));
    for (i = 0; i < n; ++i) {
      y[i] = xn[i] + (t - 1) / tn * (xn[i] - x[i]);
      x[i] = xn[i];
    }
    t = tn;
  }
}

static void test_qp(void)
{
  static double P[BMPC_NZ_MAX * BMPC_NZ_MAX], A[(BMPC_NZ_MAX + 5) * BMPC_NZ_MAX];
  static double q[BMPC_NZ_MAX], lb[BMPC_NZ_MAX], ub[BMPC_NZ_MAX], x[BMPC_NZ_MAX], xr[BMPC_NZ_MAX];
  static bmpc_qp_work_t w;
  int trial, worst_it = 0;
  double worst_kkt = 0, worst_gap = 0;
  printf("[qp]\n");
  for (trial = 0; trial < 30; ++trial) {
    const int n = 4 + rand() % (2 * (30 - 1) - 3);
    const int m = n + 5;
    int i, j, k, it = 0, st;
    double f_as, f_ref, kkt, scale = 0;
    for (i = 0; i < m * n; ++i) A[i] = urand(-1, 1);
    for (i = 0; i < n; ++i)
      for (j = 0; j < n; ++j) {
        double s = (i == j) ? 1e-3 : 0;
        for (k = 0; k < m; ++k) s += A[k * n + i] * A[k * n + j];
        P[i * n + j] = s;
      }
    for (i = 0; i < n; ++i) {
      q[i] = urand(-5, 5);
      lb[i] = urand(-1.5, 0.0);
      ub[i] = lb[i] + urand(0.0, 2.0);
      x[i] = 0;
      if (fabs(q[i]) > scale) scale = fabs(q[i]);
    }
    st = bmpc_boxqp_solve(n, P, q, lb, ub, x, 400, &w, &it);
    CHECK(st == BMPC_OK, "qp status %d (n=%d)", st, n);
    kkt = bmpc_boxqp_kkt_residual(n, P, q, lb, ub, x) / (1 + scale);
    qp_reference(n, P, q, lb, ub, xr);
    f_as = qp_obj(n, P, q, x);
    f_ref = qp_obj(n, P, q, xr);
    CHECK(kkt < 1e-9, "kkt %g (n=%d)", kkt, n);
    CHECK(f_as <= f_ref + 1e-8 * (1 + fabs(f_ref)), "objective %g vs ref %g", f_as, f_ref);
    if (kkt > worst_kkt) worst_kkt = kkt;
    if (f_as - f_ref > worst_gap) worst_gap = f_as - f_ref;
    if (it > worst_it) worst_it = it;
  }
  printf("  worst KKT %.2e, worst objective gap vs FISTA %.2e, max iterations %d\n", worst_kkt, worst_gap, worst_it);
}

/* ---------------- 4. 가중치 ---------------- */
static void test_weights(void)
{
  bmpc_config_t cfg;
  bmpc_weights_t w;
  printf("[weights]\n");
  bmpc_config_default(&cfg);
  bmpc_compute_weights(&cfg, 0, 0, 10.0, &w); /* 비율 0.5 -> smoothstep 0.5 */
  CHECK(fabs(w.cte - 2875.0) < 1e-9, "cte %g", w.cte);
  CHECK(fabs(w.delta - 0.4 * 31250.0) < 1e-6, "delta %g", w.delta);
  CHECK(fabs(w.ddelta - 0.4 * 675000.0) < 1e-6, "ddelta %g", w.ddelta);
  CHECK(fabs(w.delta_start - 12000.0) < 1e-9, "delta_start %g", w.delta_start);
  bmpc_compute_weights(&cfg, 1, 0, 25.0, &w);
  CHECK(w.cte == 1000.0 && fabs(w.delta - 20800.0) < 1e-9 && fabs(w.ddelta - 300000.0) < 1e-6, "highway");
  bmpc_compute_weights(&cfg, 1, 1, 25.0, &w);
  CHECK(w.cte == 4000.0 && fabs(w.delta - 4800.0) < 1e-9, "avoidance has priority");
  bmpc_compute_weights(&cfg, 0, 0, 3.0, &w);
  CHECK(w.cte == 5000.0 && fabs(w.delta - 1000.0) < 1e-9, "urban low");
}

/* ---------------- 5. MPC 한 번 ---------------- */
static void make_problem(const bmpc_config_t *cfg, bmpc_problem_t *p, double v, double kappa)
{
  memset(p, 0, sizeof(*p));
  p->s0[BMPC_SV] = v;
  p->c[2] = 0.5 * kappa;
  p->ref_v = v;
  p->v_lb = 0;
  p->v_ub = 40;
  p->steer_lb = cfg->steer_lb;
  p->steer_ub = cfg->steer_ub;
  bmpc_compute_weights(cfg, 0, 0, v, &p->w);
}

static void test_mpc_single(void)
{
  static bmpc_mpc_work_t w;
  static bmpc_mpc_result_t r;
  bmpc_config_t cfg;
  bmpc_problem_t p;
  const int nu = 29;
  printf("[mpc]\n");
  bmpc_config_default(&cfg);

  make_problem(&cfg, &p, 10.0, 0.0);
  bmpc_mpc_solve(&cfg, &p, NULL, &w, &r);
  CHECK(r.status == BMPC_OK, "straight status %d", r.status);
  CHECK(fabs(r.z[0]) < 1e-9, "straight delta %g", r.z[0]);
  CHECK(fabs(r.z[nu] - 10.0) < 1e-6, "straight v_target %g", r.z[nu]);

  /* 왼쪽 곡선(곡률 +1/30): yaw rate = -v delta / Lf 이므로 delta 는 음수, 크기 ~ Lf*kappa */
  make_problem(&cfg, &p, 10.0, 1.0 / 30.0);
  p.prev_delta_ref = -4.0 / 30.0;
  bmpc_mpc_solve(&cfg, &p, NULL, &w, &r);
  CHECK(r.status == BMPC_OK, "curve status %d", r.status);
  CHECK(r.z[0] < 0 && r.z[0] > -0.4, "curve delta %g (expect around %g)", r.z[0], -4.0 / 30.0);
  printf("  curve kappa 1/30 v 10: delta0 %.4f rad (Lf*kappa %.4f), sqp %d, qp %d, cost %.3f\n", r.z[0],
         -4.0 / 30.0, r.sqp_iters, r.qp_iters, r.cost);

  /* 속도 상한이 걸리면 v_target 은 상한을 넘지 않는다 */
  make_problem(&cfg, &p, 12.0, 0.0);
  p.ref_v = 15.0;
  p.v_ub = 13.0;
  bmpc_mpc_solve(&cfg, &p, NULL, &w, &r);
  CHECK(r.z[nu] <= 13.0 + 1e-9 && r.z[nu] > 12.9, "speed bound %g", r.z[nu]);
}

/* ---------------- 6. 간이 폐루프 ---------------- */
typedef struct {
  int64_t t_us;
} sim_clock_t;

static int64_t sim_now(void *ctx) { return ((sim_clock_t *)ctx)->t_us; }

/* 원 (중심 (0,R), 반지름 R) 위 점들을 차량 좌표로 옮겨 3차식으로 근사 */
static void fit_circle(double px, double py, double yaw, double R, double coeffs[4])
{
  double ATA[4][5] = {{0}};
  int i, j, k, r;
  const double th0 = atan2(px - 0.0, -(py - R)); /* 원 위 각도 (중심 기준, 시계 반대 방향 진행) */
  for (i = 0; i < 40; ++i) {
    const double s = 0.5 * i;
    const double th = th0 + s / R;
    const double wx = R * sin(th), wy = R - R * cos(th);
    const double dx = wx - px, dy = wy - py;
    const double vx = dx * cos(-yaw) - dy * sin(-yaw);
    const double vy = dy * cos(-yaw) + dx * sin(-yaw);
    double pw[4];
    pw[0] = 1;
    for (j = 1; j < 4; ++j) pw[j] = pw[j - 1] * vx;
    for (j = 0; j < 4; ++j) {
      for (k = 0; k < 4; ++k) ATA[j][k] += pw[j] * pw[k];
      ATA[j][4] += pw[j] * vy;
    }
  }
  for (j = 0; j < 4; ++j) {
    for (r = j + 1; r < 4; ++r) {
      const double f = ATA[r][j] / ATA[j][j];
      for (k = j; k < 5; ++k) ATA[r][k] -= f * ATA[j][k];
    }
  }
  for (j = 3; j >= 0; --j) {
    double s = ATA[j][4];
    for (k = j + 1; k < 4; ++k) s -= ATA[j][k] * coeffs[k];
    coeffs[j] = s / ATA[j][j];
  }
}

static void test_closed_loop(int dc_enabled)
{
  static bmpc_controller_t ctrl;
  bmpc_config_t cfg;
  sim_clock_t clk = {1000000};
  const double R = 30.0, Lf = 4.0, dt = 0.05;
  double px = 0, py = -0.5, yaw = 0.05, v = 8.0, steer_cmd_prev = 0;
  double max_late = 0, late_err = 0;
  int k, bad = 0;
  int64_t max_solve = 0;
  printf("[closed loop] circle R=%.0f m, start 0.5 m off, delay comp %s\n", R, dc_enabled ? "on" : "off");
  bmpc_config_default(&cfg);
  cfg.dc_enabled = dc_enabled;
  bmpc_controller_init(&ctrl, &cfg);
  for (k = 0; k < 600; ++k) {
    bmpc_request_t req;
    bmpc_response_t resp;
    double c[4], delta_mpc;
    memset(&req, 0, sizeof(req));
    fit_circle(px, py, yaw, R, c);
    req.seq = (uint16_t)k;
    req.c[0] = c[0];
    req.c[1] = c[1];
    req.c[2] = c[2];
    req.c[3] = c[3];
    req.ref_v = 10.0;
    req.v_lb = 0;
    req.v_ub = 10.0;
    req.post_cap = -1;
    req.snap = -1;
    req.velocity = v;
    req.yaw_rate = -v * (steer_cmd_prev / cfg.steer_sign) / Lf;
    req.period = 0.044;
    req.odom_age = 0.0;
    bmpc_controller_step(&ctrl, &req, sim_now, &clk, &resp);
    if (resp.status != BMPC_OK) ++bad;
    if ((int64_t)resp.solve_us > max_solve) max_solve = resp.solve_us;
    /* 차량: 조향은 바로, 속도는 목표로 1 차 지연 */
    delta_mpc = resp.steer_cmd / cfg.steer_sign;
    steer_cmd_prev = resp.steer_cmd;
    px += v * cos(yaw) * dt;
    py += v * sin(yaw) * dt;
    yaw += -v * delta_mpc / Lf * dt;
    v += (resp.vt_final - v) * 0.4 * dt;
    clk.t_us += 50000;
    {
      const double dist = sqrt(px * px + (py - R) * (py - R));
      const double err = dist - R;
      if (k >= 400) {
        if (fabs(err) > max_late) max_late = fabs(err);
        late_err = err;
      }
    }
  }
  CHECK(bad == 0, "controller errors %d", bad);
  /* 조향 크기 비용 때문에 정상상태에서도 바깥쪽 오차가 조금 남는다 (원본 NMPC 도 같다) */
  CHECK(max_late < 0.5, "late radial error %.3f m", max_late);
  printf("  radial error over last 10 s: max %.3f m, final %.3f m, final v %.2f m/s\n", max_late, late_err, v);
}

int main(void)
{
  srand(12345);
  test_protocol();
  test_config();
  test_qp();
  test_weights();
  test_mpc_single();
  test_closed_loop(0);
  test_closed_loop(1);
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
