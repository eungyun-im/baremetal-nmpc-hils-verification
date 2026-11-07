// rosparam -> MCU 설정. 브리지 노드의 private 이름공간(~)에서 읽는다 (값: config/bmpc_params.yaml).
// 파라미터 이름과 "없을 때 기본값"은 원본 MPC 코드와 같게 두었다 (원본이 읽던 값을 그대로 MCU 로 보내야 같은 동작이 나온다).
#ifndef BMPC_BRIDGE_MCU_CONFIG_H
#define BMPC_BRIDGE_MCU_CONFIG_H

#include <ros/ros.h>

#include "bmpc/bmpc_config.h"

namespace bmpc_bridge {

inline void loadMcuConfig(ros::NodeHandle &nh, bmpc_config_t *c)
{
  bmpc_config_default(c);
  auto rd = [&nh](const char *name, double def) {
    double v = def;
    nh.param<double>(name, v, def);
    return v;
  };
  auto rb = [&nh](const char *name, bool def) {
    bool v = def;
    nh.param<bool>(name, v, def);
    return v;
  };
  int N = 50;
  nh.param<int>("N", N, 50);
  c->N = N;
  c->dt = rd("dt", 0.1);
  c->Lf = rd("Lf", 1.5);
  c->k_v = rd("k_v", 0.4);
  float fub = 0.6981f, flb = -0.6981f;  // 원본은 float 로 읽는다
  nh.param<float>("steer/upperBound", fub, 0.6981f);
  nh.param<float>("steer/lowerBound", flb, -0.6981f);
  c->steer_ub = fub;
  c->steer_lb = flb;
  c->steer_sign = rd("steer/command_sign", -1.0);

  c->w_v = rd("cost_weight/v_start_weight", 20.0);
  c->w_delta_start = rd("cost_weight/delta_start_cost_weight", 30000.0);
  c->w_vt = rd("cost_weight/v_target_cost_weight", 300.0);
  c->w_dvt = rd("cost_weight/v_target_change_cost_weight", 100.0);
  c->steer_scale = rd("cost_weight/steer_scale", 1.0);
  c->steer_change_scale = rd("cost_weight/steer_change_scale", c->steer_scale);

  c->u_cte_start = rd("urban_control/cte_cost_start_weight", 3500.0);
  c->u_cte_urban = rd("urban_control/cte_cost_urban_weight", 1500.0);
  c->u_epsi = rd("urban_control/epsi_cost_weight", 2400.0);
  c->u_delta_start = rd("urban_control/delta_cost_start_weight", 3500.0);
  c->u_delta_high = rd("urban_control/delta_cost_high_speed_weight", 38000.0);
  c->u_speed_start = rd("urban_control/delta_cost_start_mps", 7.0);
  c->u_speed_full = rd("urban_control/delta_cost_full_mps", 15.0);
  c->u_ddelta_start = rd("urban_control/delta_change_cost_start_weight", 450000.0);
  c->u_ddelta_urban = rd("urban_control/delta_change_cost_urban_weight", 650000.0);
  c->h_cte = rd("highway_control/cte_cost_weight", 1500.0);
  c->h_epsi = rd("highway_control/epsi_cost_weight", 4200.0);
  c->h_delta = rd("highway_control/delta_cost_weight", 48000.0);
  c->h_ddelta = rd("highway_control/delta_change_cost_weight", 900000.0);
  c->a_cte = rd("avoidance_control/cte_cost_weight", 4000.0);
  c->a_epsi = rd("avoidance_control/epsi_cost_weight", 4000.0);
  c->a_delta = rd("avoidance_control/delta_cost_weight", 12000.0);
  c->a_ddelta = rd("avoidance_control/delta_change_cost_weight", 350000.0);

  c->dc_enabled = rb("delay_comp/enabled", false) ? 1 : 0;
  c->dc_state_k = rd("delay_comp/state_k", 1.0);
  c->dc_state_c = rd("delay_comp/state_c_s", 0.010);
  c->dc_state_max = rd("delay_comp/state_max_s", 0.12);
  c->dc_act_base = rd("delay_comp/actuation_base_s", 0.06);
  c->dc_act_frame_k = rd("delay_comp/actuation_frame_k", 0.5);
  c->dc_model_lf = rd("delay_comp/model_lf", 4.2);
  c->dc_lf_online = rb("delay_comp/lf_online", true) ? 1 : 0;
  c->dc_compute_max = rd("delay_comp/compute_max_s", 0.10);
  c->dc_compute_init = rd("bmpc/dc_compute_init_s", 0.03);
  c->dc_lf_min = 3.5;
  c->dc_lf_max = 5.0;
  c->dc_transport_req_share = rd("bmpc/transport_req_share", 0.6);
  c->dc_compute_fixed = rd("bmpc/dc_compute_fixed_s", 0.0);

  c->st_rate_limit = rd("steer/command_rate_limit_radps", 1.0);
  c->st_lpf_enabled = rb("steer/output_lpf_enabled", false) ? 1 : 0;
  c->st_lpf_alpha = rd("steer/output_lpf_alpha", 1.0);
  c->st_lpf_alpha_high = rd("steer/output_lpf_alpha_high_speed", 0.75);
  c->st_lpf_v_low = 11.0;
  c->st_lpf_v_high = 25.0;

  c->lg_kp = rd("longitudinal/throttle_kp", 0.25);
  c->lg_ki = rd("longitudinal/throttle_ki", 0.05);
  c->lg_int_limit = rd("longitudinal/throttle_integral_limit", 4.0);
  c->lg_int_band = rd("longitudinal/throttle_integral_band_mps", 0.0);
  c->lg_accel_limit = rd("longitudinal/throttle_accel_limit", 0.60);
  c->lg_accel_limit_exempt = rd("longitudinal/throttle_accel_limit_exempt", 1.0);
  c->lg_brake_limit = rd("longitudinal/throttle_brake_limit", 0.60);
  c->lg_kick_brake_limit = rd("longitudinal/kick_brake_limit", 0.60);
  c->lg_brake_rate = rd("longitudinal/brake_rate_limit_per_sec", 3.0);
  c->lg_deadband = rd("longitudinal/throttle_pedal_deadband", 0.0);
  c->lg_ff_gain = rd("longitudinal/throttle_ff_gain", 0.0);
  c->lg_ff_accel_per_pedal = rd("longitudinal/throttle_ff_accel_per_pedal_mps2", 3.5);
  c->lg_ff_tau = rd("longitudinal/throttle_ff_tau_s", 0.3);
  c->lg_ff_min_accel = rd("longitudinal/throttle_ff_min_accel_mps2", 0.6);
  c->lg_ff_taper = rd("longitudinal/throttle_ff_taper_mps", 1.0);
  c->lg_ff_steer_zero = rd("longitudinal/throttle_ff_steer_zero_rad", 0.10);
  c->legal_enabled = rb("longitudinal/legal_guard/enabled", true) ? 1 : 0;
  c->legal_cut = rd("longitudinal/legal_guard/cut_mps", 16.2);
  c->legal_brake_at = rd("longitudinal/legal_guard/brake_at_mps", 16.4);
  c->legal_brake = rd("longitudinal/legal_guard/brake", 0.3);

  int sqp = 3, qp = 200;
  nh.param<int>("bmpc/sqp_iters", sqp, 3);
  nh.param<int>("bmpc/qp_max_iters", qp, 200);
  c->sqp_iters = sqp;
  c->qp_max_iters = qp;
  c->sqp_tol = rd("bmpc/sqp_tol", 1e-4);
}

}  // namespace bmpc_bridge

#endif
