// LTV-MPC(bare_metal_mpc) 와 원본 NMPC(CppAD + IPOPT) 를 같은 문제에서 비교한다.
// 기준 NMPC 는 원본 mpc.cpp 의 FG_eval / Solve 를 ROS 호출(nh.param)만 값으로 바꿔 그대로 옮겼다.
// 가중치는 양쪽 모두 bmpc_compute_weights() 결과를 써서 같은 문제를 풀게 한다.
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>


#include "bmpc/bmpc_mpc.h"
#include "bmpc/bmpc_weights.h"


#include "ref_nmpc.hpp"

// ------------------------------------------------------------------
struct Scenario {
  double v, kappa, cte0, epsi0, ref_v, v_ub, prev_delta;
  int highway, avoidance;
};

int main()
{
  bmpc_config_t cfg;
  bmpc_config_default(&cfg);

  std::vector<Scenario> sc;
  const double speeds[] = {3, 7, 10, 13, 16, 20, 25};
  const double kappas[] = {0.0, 1.0 / 100, 1.0 / 50, 1.0 / 30, 1.0 / 15, 1.0 / 11, -1.0 / 30, -1.0 / 11};
  const double offs[][2] = {{0, 0}, {0.3, 0.03}, {-0.5, -0.05}};
  for (double v : speeds)
    for (double k : kappas)
      for (auto &o : offs) {
        // 곡률 속도 한계를 넘는 조합(a_lat > 7.5)은 대회에서 나오지 않으므로 뺀다
        if (v * v * std::fabs(k) > 7.5) continue;
        Scenario s{v, k, o[0], o[1], v, std::min(40.0, v + 3.0), -cfg.Lf * k, v >= 16 ? 1 : 0, 0};
        sc.push_back(s);
      }
  // 속도 변화: 상한이 현재 속도보다 낮음(감속), ref_v 가 높음(가속), 회피 세트
  sc.push_back(Scenario{12, 0, 0, 0, 15, 9, 0, 0, 0});
  sc.push_back(Scenario{14, 1.0 / 30, 0.2, 0, 16, 10, -cfg.Lf / 30, 0, 0});
  sc.push_back(Scenario{4, 0, 0, 0, 12, 12, 0, 0, 0});
  sc.push_back(Scenario{8, 1.0 / 20, 0.8, 0.1, 8, 8, 0, 0, 1});
  sc.push_back(Scenario{8, -1.0 / 20, -0.8, -0.1, 8, 8, 0, 0, 1});

  const int iters_list[] = {1, 2, 3, 5};
  const int NI = 4;
  double max_dd[NI] = {0}, sum_dd[NI] = {0}, max_dv[NI] = {0}, sum_dv[NI] = {0}, worst_cost[NI] = {0};
  double max_t[NI] = {0}, sum_t[NI] = {0};
  double ipopt_t_sum = 0, ipopt_t_max = 0;
  int ipopt_fail = 0;
  static bmpc_mpc_work_t work;
  static bmpc_mpc_result_t res;
  static bmpc_real sbar[BMPC_N_MAX][BMPC_NX];

  std::printf("%zu scenarios, N=%d dt=%.2f Lf=%.1f, bmpc_real=%s\n", sc.size(), cfg.N, (double)cfg.dt, (double)cfg.Lf,
              sizeof(bmpc_real) == 4 ? "float" : "double");
  std::printf("%-5s %-8s %-6s %-6s | %-9s %-9s | %-9s %-9s | %-8s %-8s\n", "v", "kappa", "cte0", "mode",
              "d0_ipopt", "d0_ltv3", "vt_ipopt", "vt_ltv3", "J_ipopt", "J_ltv3");

  for (size_t i = 0; i < sc.size(); ++i) {
    const Scenario &s = sc[i];
    bmpc_problem_t pr;
    std::memset(&pr, 0, sizeof(pr));
    pr.c[0] = s.cte0;
    pr.c[1] = -std::tan(s.epsi0);
    pr.c[2] = 0.5 * s.kappa;
    pr.c[3] = 0;
    pr.s0[BMPC_SV] = s.v;
    pr.s0[BMPC_SCTE] = pr.c[0];
    pr.s0[BMPC_SEPSI] = -std::atan(pr.c[1]);
    pr.ref_v = s.ref_v;
    pr.v_lb = 0;
    pr.v_ub = s.v_ub;
    pr.steer_lb = cfg.steer_lb;
    pr.steer_ub = cfg.steer_ub;
    pr.prev_delta_ref = s.prev_delta;
    bmpc_compute_weights(&cfg, s.highway, s.avoidance, s.v, &pr.w);

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

    std::vector<double> coeffs{pr.c[0], pr.c[1], pr.c[2], pr.c[3]};
    std::vector<double> z_ref;
    auto t0 = std::chrono::steady_clock::now();
    double s0d[6];
    for (int q = 0; q < 6; ++q) s0d[q] = pr.s0[q];
    const bool ok = ref_solve(rp, s0d, coeffs, z_ref);
    auto t1 = std::chrono::steady_clock::now();
    const double t_ipopt = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ipopt_t_sum += t_ipopt;
    ipopt_t_max = std::max(ipopt_t_max, t_ipopt);
    if (!ok) ++ipopt_fail;
    std::vector<bmpc_real> zr(z_ref.begin(), z_ref.end());
    const double J_ref = bmpc_mpc_cost(&cfg, &pr, zr.data(), sbar);
    const int nu = cfg.N - 1;

    double d_ltv3 = 0, vt_ltv3 = 0, J_ltv3 = 0;
    for (int m = 0; m < NI; ++m) {
      bmpc_config_t c2 = cfg;
      c2.sqp_iters = iters_list[m];
      c2.sqp_tol = 0;  // 정해진 횟수만큼 돈다
      auto a0 = std::chrono::steady_clock::now();
      bmpc_mpc_solve(&c2, &pr, NULL, &work, &res);
      auto a1 = std::chrono::steady_clock::now();
      const double t_ltv = std::chrono::duration<double, std::milli>(a1 - a0).count();
      const double dd = std::fabs(res.z[0] - z_ref[0]) * 180.0 / M_PI;
      const double dv = std::fabs(res.z[nu] - z_ref[nu]);
      const double rel = (res.cost - J_ref) / std::max(1e-9, std::fabs(J_ref));
      max_dd[m] = std::max(max_dd[m], dd);
      sum_dd[m] += dd;
      max_dv[m] = std::max(max_dv[m], dv);
      sum_dv[m] += dv;
      worst_cost[m] = std::max(worst_cost[m], rel);
      max_t[m] = std::max(max_t[m], t_ltv);
      sum_t[m] += t_ltv;
      if (iters_list[m] == 3) {
        d_ltv3 = res.z[0];
        vt_ltv3 = res.z[nu];
        J_ltv3 = res.cost;
      }
    }
    if (i % 6 == 0 || i + 5 >= sc.size()) {
      std::printf("%-5.1f %-8.4f %-6.2f %-6s | %-9.5f %-9.5f | %-9.3f %-9.3f | %-8.2f %-8.2f%s\n", s.v, s.kappa,
                  s.cte0, s.avoidance ? "avoid" : (s.highway ? "hwy" : "urban"), z_ref[0], d_ltv3, z_ref[nu],
                  vt_ltv3, J_ref, J_ltv3, ok ? "" : "  (ipopt not converged)");
    }
  }

  const double n = (double)sc.size();
  std::printf("\nIPOPT: mean %.2f ms, max %.2f ms, not converged %d/%zu\n", ipopt_t_sum / n, ipopt_t_max,
              ipopt_fail, sc.size());
  std::printf("%-10s | %-22s | %-22s | %-14s | %-18s\n", "LTV iters", "|d0 diff| deg mean/max",
              "|vt0 diff| m/s mean/max", "cost vs IPOPT", "time ms mean/max");
  int fail = 0;
  for (int m = 0; m < NI; ++m) {
    std::printf("%-10d | %9.4f / %-10.4f | %9.4f / %-10.4f | %+12.4f%% | %7.3f / %-8.3f\n", iters_list[m],
                sum_dd[m] / n, max_dd[m], sum_dv[m] / n, max_dv[m], worst_cost[m] * 100.0, sum_t[m] / n, max_t[m]);
  }
  // 기본값(반복 3 회)의 합격 기준: 첫 조향 차 0.5 deg 이하, 비용이 IPOPT 보다 0.5 % 넘게 나쁘지 않음
  if (max_dd[2] > 0.5) {
    std::printf("FAIL: steering differs by %.3f deg (> 0.5)\n", max_dd[2]);
    ++fail;
  }
  if (worst_cost[2] > 0.005) {
    std::printf("FAIL: cost worse than IPOPT by %.3f %% (> 0.5 %%)\n", worst_cost[2] * 100.0);
    ++fail;
  }
  std::printf(fail ? "\nRESULT: FAIL\n" : "\nRESULT: PASS\n");
  return fail ? 1 : 0;
}
