// 기준 NMPC: 원본 mpc.cpp 의 FG_eval / Solve 를 ROS 호출(nh.param)만 값으로 바꿔 그대로 옮겼다.
// test_vs_ipopt.cpp 와 test_track_vs_ipopt.cpp 가 함께 쓴다.
#ifndef BMPC_TEST_REF_NMPC_HPP
#define BMPC_TEST_REF_NMPC_HPP

#include <string>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// ------------------------------------------------------------------
// 기준: 원본 NMPC
// ------------------------------------------------------------------
struct RefParams {
  size_t N;
  double dt, Lf, k_v, ref_v;
  double steer_lowerBound, steer_upperBound, speed_lowerBound, speed_upperBound;
  double v_weight, delta_start_cost_weight, v_target_cost_weight, v_target_change_cost_weight;
  double cte_cost_effective_weight, epsi_cost_effective_weight, delta_cost_effective_weight,
      delta_change_cost_effective_weight;
  double previous_delta_cost_reference;
  size_t x_start, y_start, psi_start, v_start, cte_start, epsi_start, delta_start, v_target_start;
  void index()
  {
    x_start = 0;
    y_start = x_start + N;
    psi_start = y_start + N;
    v_start = psi_start + N;
    cte_start = v_start + N;
    epsi_start = cte_start + N;
    delta_start = epsi_start + N;
    v_target_start = delta_start + N - 1;
  }
};

class FG_eval {
 public:
  const RefParams &p;
  std::vector<double> coeffs;
  const double ref_cte = 0, ref_epsi = 0;
  FG_eval(const RefParams &prm, const std::vector<double> &c) : p(prm), coeffs(c) {}
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector &fg, const ADvector &vars)
  {
    const size_t N = p.N;
    const double dt = p.dt, Lf = p.Lf, k_v = p.k_v, ref_v = p.ref_v;
    const size_t x_start = p.x_start, y_start = p.y_start, psi_start = p.psi_start, v_start = p.v_start,
                 cte_start = p.cte_start, epsi_start = p.epsi_start, delta_start = p.delta_start,
                 v_target_start = p.v_target_start;
    // ---- 원본 FG_eval::operator() 본문 ----
    fg[0] = 0;
    fg[0] += p.delta_start_cost_weight *
             CppAD::pow(vars[delta_start] - p.previous_delta_cost_reference, 2);
    for (int t = 0; t < (int)N; t++) {
      fg[0] += p.cte_cost_effective_weight * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += p.epsi_cost_effective_weight * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      if (t < (int)N - 1) {
        fg[0] += p.v_weight * CppAD::pow(vars[v_start + t] - vars[v_target_start + t], 2);
      } else {
        fg[0] += p.v_weight * CppAD::pow(vars[v_start + t] - vars[v_target_start + (N - 2)], 2);
      }
    }
    for (int t = 0; t < (int)N - 1; t++) {
      fg[0] += p.delta_cost_effective_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += p.v_target_cost_weight * CppAD::pow(vars[v_target_start + t] - ref_v, 2);
    }
    for (int t = 0; t < (int)N - 2; t++) {
      fg[0] += p.delta_change_cost_effective_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += p.v_target_change_cost_weight * CppAD::pow(vars[v_target_start + t + 1] - vars[v_target_start + t], 2);
    }
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    for (int t = 1; t < (int)N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> v_target0 = vars[v_target_start + t - 1];
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + k_v * (v_target0 - v0) * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 / Lf * delta0 * dt);
      (void)cte0;
    }
  }
};

// 원본 MPC::Solve() 본문 (가중치 갱신 부분만 RefParams 로 대체). 해 z 와 성공 여부를 돌려준다.
static inline bool ref_solve(const RefParams &p, const double state[6], const std::vector<double> &coeffs,
                      std::vector<double> &z_out)
{
  typedef CPPAD_TESTVECTOR(double) Dvector;
  const size_t N = p.N;
  const double x = state[0], y = state[1], psi = state[2], v = state[3], cte = state[4], epsi = state[5];
  size_t n_vars = 6 * N + (N - 1) * 2;
  size_t n_constraints = N * 6;
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) vars[i] = 0;
  vars[p.x_start] = x;
  vars[p.y_start] = y;
  vars[p.psi_start] = psi;
  vars[p.v_start] = v;
  vars[p.cte_start] = cte;
  vars[p.epsi_start] = epsi;
  Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);
  for (size_t i = 0; i < p.delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (size_t i = p.delta_start; i < p.v_target_start; i++) {
    vars_lowerbound[i] = p.steer_lowerBound;
    vars_upperbound[i] = p.steer_upperBound;
  }
  for (size_t i = p.v_target_start; i < n_vars; i++) {
    vars_lowerbound[i] = p.speed_lowerBound;
    vars_upperbound[i] = p.speed_upperBound;
  }
  Dvector constraints_lowerbound(n_constraints), constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[p.x_start] = x;
  constraints_lowerbound[p.y_start] = y;
  constraints_lowerbound[p.psi_start] = psi;
  constraints_lowerbound[p.v_start] = v;
  constraints_lowerbound[p.cte_start] = cte;
  constraints_lowerbound[p.epsi_start] = epsi;
  constraints_upperbound[p.x_start] = x;
  constraints_upperbound[p.y_start] = y;
  constraints_upperbound[p.psi_start] = psi;
  constraints_upperbound[p.v_start] = v;
  constraints_upperbound[p.cte_start] = cte;
  constraints_upperbound[p.epsi_start] = epsi;
  FG_eval fg_eval(p, coeffs);
  std::string options;
  options += "Integer print_level  0\n";
  options += "String  sb           yes\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                        constraints_upperbound, fg_eval, solution);
  const bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  z_out.resize(2 * (N - 1));
  for (size_t k = 0; k < N - 1; ++k) {
    z_out[k] = solution.x[p.delta_start + k];
    z_out[N - 1 + k] = solution.x[p.v_target_start + k];
  }
  return ok;
}


#endif
