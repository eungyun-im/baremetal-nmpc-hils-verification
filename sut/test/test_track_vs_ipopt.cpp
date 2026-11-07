// 폐루프 트랙 주행으로 LTV-MPC 와 원본 NMPC(IPOPT)를 비교한다.
//   경로: 직선 40 m -> 왼쪽 R15 90도 -> 직선 30 m -> 오른쪽 R11 90도 -> 직선 30 m -> 왼쪽 R30 60도 -> 직선 60 m
//   차량: 운동학 자전거 모델(Lf 4.0, 속도 1차 지연 k_v 0.4), 0.5 m 옆에서 출발
//   매 주기: 원본처럼 경로 점을 차량 좌표로 옮겨 3차 polyfit -> 같은 초기 상태로
//            LTV(warm start, 반복 3회), LTV(warm start, 1회), IPOPT(원본처럼 cold start)를 모두 푼다
//   랩 A: LTV(3회)가 운전하고 나머지는 같은 입력으로 함께 풀어 출력 차이를 잰다
//   랩 B: IPOPT 가 운전한다. 두 랩의 경로 오차를 비교한다.
// 후처리(조향 LPF 등)는 넣지 않고 MPC 출력을 그대로 차에 준다(MPC 자체 비교).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "bmpc/bmpc_mpc.h"
#include "bmpc/bmpc_weights.h"
#include "ref_nmpc.hpp"

struct Path {
  std::vector<double> x, y, h, k;
};

static Path build_path()
{
  struct Seg {
    double len, kappa;
  };
  const Seg segs[] = {{40, 0},           {15 * M_PI / 2, 1.0 / 15}, {30, 0}, {11 * M_PI / 2, -1.0 / 11},
                      {30, 0},           {30 * M_PI / 3, 1.0 / 30}, {60, 0}};
  Path p;
  const double ds = 0.1;
  double x = 0, y = 0, h = 0;
  p.x.push_back(x);
  p.y.push_back(y);
  p.h.push_back(h);
  p.k.push_back(0);
  for (const Seg &s : segs) {
    const int n = (int)std::lround(s.len / ds);
    for (int i = 0; i < n; ++i) {
      const double hm = h + 0.5 * s.kappa * ds;
      x += ds * std::cos(hm);
      y += ds * std::sin(hm);
      h += s.kappa * ds;
      p.x.push_back(x);
      p.y.push_back(y);
      p.h.push_back(h);
      p.k.push_back(s.kappa);
    }
  }
  return p;
}

static int nearest(const Path &p, double x, double y, int hint)
{
  int best = hint;
  double bd = 1e18;
  const int lo = std::max(0, hint - 50), hi = std::min((int)p.x.size() - 1, hint + 400);
  for (int i = lo; i <= hi; ++i) {
    const double d = (p.x[i] - x) * (p.x[i] - x) + (p.y[i] - y) * (p.y[i] - y);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best;
}

// 원본 polyfit(Householder QR) 대신 정규방정식 (x 를 10 m 로 나눠 조건수를 낮춘다)
static void polyfit3(const std::vector<double> &xs, const std::vector<double> &ys, double c[4])
{
  double A[4][5] = {{0}};
  for (size_t i = 0; i < xs.size(); ++i) {
    const double u = xs[i] / 10.0;
    double pw[4] = {1, u, u * u, u * u * u};
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) A[j][k] += pw[j] * pw[k];
      A[j][4] += pw[j] * ys[i];
    }
  }
  for (int j = 0; j < 4; ++j)
    for (int r = j + 1; r < 4; ++r) {
      const double f = A[r][j] / A[j][j];
      for (int k = j; k < 5; ++k) A[r][k] -= f * A[j][k];
    }
  double cu[4];
  for (int j = 3; j >= 0; --j) {
    double s = A[j][4];
    for (int k = j + 1; k < 4; ++k) s -= A[j][k] * cu[k];
    cu[j] = s / A[j][j];
  }
  c[0] = cu[0];
  c[1] = cu[1] / 10.0;
  c[2] = cu[2] / 100.0;
  c[3] = cu[3] / 1000.0;
}

struct Stats {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  double mean() const
  {
    double s = 0;
    for (double x : v) s += x;
    return v.empty() ? 0 : s / v.size();
  }
  double rms() const
  {
    double s = 0;
    for (double x : v) s += x * x;
    return v.empty() ? 0 : std::sqrt(s / v.size());
  }
  double pct(double q) const
  {
    if (v.empty()) return 0;
    std::vector<double> w(v);
    std::sort(w.begin(), w.end());
    return w[(size_t)std::min<double>(w.size() - 1, std::floor(q * (w.size() - 1) + 0.5))];
  }
  double max() const { return v.empty() ? 0 : *std::max_element(v.begin(), v.end()); }
};

struct LoopResult {
  Stats lat, lat_after, d_ltv3, d_ltv1, dv_ltv3, t_ltv3, t_ltv1, t_ipopt, sqp_it, qp_it;
  int steps = 0, ipopt_fail = 0, ltv_err = 0;
  bool finished = false;
};

static void shift_warm(const bmpc_real *z, int nu, bmpc_real *zw)
{
  for (int k = 0; k < nu; ++k) {
    const int src = k + 1 < nu ? k + 1 : nu - 1;
    zw[k] = z[src];
    zw[nu + k] = z[nu + src];
  }
}

// driver: 0 = LTV(3회) 가 운전, 1 = IPOPT 가 운전
static LoopResult run_loop(const Path &path, int driver)
{
  bmpc_config_t cfg;
  bmpc_config_default(&cfg);
  const int nu = cfg.N - 1;
  const double dt = cfg.dt, Lf = cfg.Lf;
  static bmpc_mpc_work_t work;
  static bmpc_mpc_result_t r3, r1;
  static bmpc_real warm3[BMPC_NZ_MAX], warm1[BMPC_NZ_MAX];
  bool have_warm3 = false, have_warm1 = false;
  LoopResult out;

  double px = 0.0, py = -0.5, yaw = 0.0, v = 6.0, delta_applied = 0.0;
  int idx = 0;
  const int last = (int)path.x.size() - 1;

  for (int step = 0; step < 4000; ++step) {
    idx = nearest(path, px, py, idx);
    if (idx >= last - 150) {
      out.finished = true;
      break;
    }
    // 경로 오차 (경로 방향에 수직)
    {
      const double dx = px - path.x[idx], dy = py - path.y[idx];
      const double e = std::fabs(-dx * std::sin(path.h[idx]) + dy * std::cos(path.h[idx]));
      out.lat.add(e);
      if (step >= 60) out.lat_after.add(e); /* 출발 3 s 뒤 (처음 0.5 m 오프셋 제외) */
    }
    // 원본처럼 속도에 따라 12~24 m 를 fit
    const double fit_dist = std::max(12.0, std::min(24.0, v * 1.3));
    std::vector<double> xs, ys;
    double s = 0;
    for (int i = idx; i <= last; ++i) {
      if (i > idx) s += std::hypot(path.x[i] - path.x[i - 1], path.y[i] - path.y[i - 1]);
      const double dx = path.x[i] - px, dy = path.y[i] - py;
      xs.push_back(dx * std::cos(-yaw) - dy * std::sin(-yaw));
      ys.push_back(dy * std::cos(-yaw) + dx * std::sin(-yaw));
      if (s >= fit_dist && xs.size() >= 4) break;
    }
    double c[4];
    polyfit3(xs, ys, c);
    // 곡률 속도: 앞 30 m 의 최대 곡률로 a_lat 6.5
    double kmax = 0;
    for (int i = idx; i <= std::min(last, idx + 300); ++i) kmax = std::max(kmax, std::fabs(path.k[i]));
    const double ref_v = std::min(12.0, kmax > 1e-6 ? std::sqrt(6.5 / kmax) : 12.0);

    bmpc_problem_t pr;
    std::memset(&pr, 0, sizeof(pr));
    for (int i = 0; i < 4; ++i) pr.c[i] = c[i];
    pr.s0[BMPC_SV] = v;
    pr.s0[BMPC_SCTE] = c[0];
    pr.s0[BMPC_SEPSI] = -std::atan(c[1]);
    pr.ref_v = ref_v;
    pr.v_lb = 0;
    pr.v_ub = ref_v;
    pr.steer_lb = cfg.steer_lb;
    pr.steer_ub = cfg.steer_ub;
    pr.prev_delta_ref = delta_applied;
    bmpc_compute_weights(&cfg, 0, 0, v, &pr.w);

    // LTV 3 회 (warm)
    bmpc_config_t c3 = cfg;
    c3.sqp_iters = 3;
    auto a0 = std::chrono::steady_clock::now();
    const int st3 = bmpc_mpc_solve(&c3, &pr, have_warm3 ? warm3 : NULL, &work, &r3);
    auto a1 = std::chrono::steady_clock::now();
    out.t_ltv3.add(std::chrono::duration<double, std::milli>(a1 - a0).count());
    out.sqp_it.add(r3.sqp_iters);
    out.qp_it.add(r3.qp_iters);
    if (st3 != BMPC_OK) ++out.ltv_err;
    shift_warm(r3.z, nu, warm3);
    have_warm3 = true;

    // LTV 1 회 (warm)
    bmpc_config_t c1 = cfg;
    c1.sqp_iters = 1;
    a0 = std::chrono::steady_clock::now();
    bmpc_mpc_solve(&c1, &pr, have_warm1 ? warm1 : NULL, &work, &r1);
    a1 = std::chrono::steady_clock::now();
    out.t_ltv1.add(std::chrono::duration<double, std::milli>(a1 - a0).count());
    shift_warm(r1.z, nu, warm1);
    have_warm1 = true;

    // IPOPT (원본: 매번 0 에서 시작)
    RefParams rp;
    rp.N = cfg.N;
    rp.dt = cfg.dt;
    rp.Lf = cfg.Lf;
    rp.k_v = cfg.k_v;
    rp.ref_v = pr.ref_v;
    rp.steer_lowerBound = cfg.steer_lb;
    rp.steer_upperBound = cfg.steer_ub;
    rp.speed_lowerBound = pr.v_lb;
    rp.speed_upperBound = pr.v_ub;
    rp.v_weight = pr.w.v;
    rp.delta_start_cost_weight = pr.w.delta_start;
    rp.v_target_cost_weight = pr.w.vt;
    rp.v_target_change_cost_weight = pr.w.dvt;
    rp.cte_cost_effective_weight = pr.w.cte;
    rp.epsi_cost_effective_weight = pr.w.epsi;
    rp.delta_cost_effective_weight = pr.w.delta;
    rp.delta_change_cost_effective_weight = pr.w.ddelta;
    rp.previous_delta_cost_reference = pr.prev_delta_ref;
    rp.index();
    double s0d[6];
    for (int q = 0; q < 6; ++q) s0d[q] = pr.s0[q];
    std::vector<double> coeffs{c[0], c[1], c[2], c[3]}, z_ref;
    a0 = std::chrono::steady_clock::now();
    const bool ok = ref_solve(rp, s0d, coeffs, z_ref);
    a1 = std::chrono::steady_clock::now();
    out.t_ipopt.add(std::chrono::duration<double, std::milli>(a1 - a0).count());
    if (!ok) ++out.ipopt_fail;

    out.d_ltv3.add(std::fabs((double)r3.z[0] - z_ref[0]) * 180.0 / M_PI);
    out.d_ltv1.add(std::fabs((double)r1.z[0] - z_ref[0]) * 180.0 / M_PI);
    out.dv_ltv3.add(std::fabs((double)r3.z[nu] - z_ref[nu]));

    const double delta = driver == 0 ? (double)r3.z[0] : z_ref[0];
    const double vt = driver == 0 ? (double)r3.z[nu] : z_ref[nu];
    delta_applied = delta;
    px += v * std::cos(yaw) * dt;
    py += v * std::sin(yaw) * dt;
    yaw += -v * delta / Lf * dt;
    v += cfg.k_v * (vt - v) * dt;
    ++out.steps;
  }
  return out;
}

int main()
{
  const Path path = build_path();
  std::printf("track: %.0f m, R15 left / R11 right / R30 left, bmpc_real=%s\n", (path.x.size() - 1) * 0.1,
              sizeof(bmpc_real) == 4 ? "float" : "double");
  const LoopResult A = run_loop(path, 0);
  const LoopResult B = run_loop(path, 1);

  std::printf("\n[lap A: LTV(3) drives, %d steps, finished %s]\n", A.steps, A.finished ? "yes" : "NO");
  std::printf("  first steering diff vs IPOPT (deg)  LTV(3): mean %.5f  p95 %.5f  max %.5f\n", A.d_ltv3.mean(),
              A.d_ltv3.pct(0.95), A.d_ltv3.max());
  std::printf("                                     LTV(1): mean %.5f  p95 %.5f  max %.5f\n", A.d_ltv1.mean(),
              A.d_ltv1.pct(0.95), A.d_ltv1.max());
  std::printf("  first v_target diff vs IPOPT (m/s)  LTV(3): mean %.5f  max %.5f\n", A.dv_ltv3.mean(), A.dv_ltv3.max());
  std::printf("  LTV(3) iterations: SQP mean %.2f, QP mean %.1f / max %.0f, solver errors %d\n", A.sqp_it.mean(),
              A.qp_it.mean(), A.qp_it.max(), A.ltv_err);
  std::printf("  time ms  LTV(3): mean %.3f p99 %.3f max %.3f | LTV(1): mean %.3f | IPOPT: mean %.2f max %.2f (fail %d)\n",
              A.t_ltv3.mean(), A.t_ltv3.pct(0.99), A.t_ltv3.max(), A.t_ltv1.mean(), A.t_ipopt.mean(), A.t_ipopt.max(),
              A.ipopt_fail);
  std::printf("  path error (m): rms %.4f  max %.4f | after first 3 s: rms %.4f  max %.4f\n", A.lat.rms(), A.lat.max(),
              A.lat_after.rms(), A.lat_after.max());
  std::printf("\n[lap B: IPOPT drives, %d steps, finished %s]\n", B.steps, B.finished ? "yes" : "NO");
  std::printf("  path error (m): rms %.4f  max %.4f | after first 3 s: rms %.4f  max %.4f\n", B.lat.rms(), B.lat.max(),
              B.lat_after.rms(), B.lat_after.max());

  int fail = 0;
  if (!A.finished || !B.finished) ++fail;
  if (A.ltv_err) ++fail;
  if (A.d_ltv3.max() > 0.5) ++fail;
  if (std::fabs(A.lat_after.max() - B.lat_after.max()) > 0.02) ++fail;
  std::printf(fail ? "\nRESULT: FAIL\n" : "\nRESULT: PASS\n");
  return fail ? 1 : 0;
}
