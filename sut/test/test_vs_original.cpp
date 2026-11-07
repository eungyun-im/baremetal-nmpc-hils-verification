// bare_metal_mpc 의 C 이식이 원본 mpc_path_tracking C++ 와 같은 값을 내는지 비교한다.
//   - 지연보상 수식: 원본 steering_delay.h 를 그대로 include 해서 비교
//   - 지연보상 v2 블록: 원본 modelPredictiveController() 의 해당 부분을 그대로 옮긴 기준 함수
//   - 가중치 스케줄: 원본 updateCostWeights() + Solve() 의 steer_scale 부분을 그대로 옮긴 기준 함수
//   - 후처리: 원본 makeSteeringCommand(), setThrottleCommand(), 속도 가드를 그대로 옮긴 기준 클래스
// 기준 함수 본문은 원본에서 ROS 호출(nh.param, publish)만 값으로 바꿨다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <vector>

#include "reference/steering_delay.h"  // 원본 그대로 (고정 사본)

#include "bmpc/bmpc_controller.h"
#include "bmpc/bmpc_delay.h"
#include "bmpc/bmpc_post.h"
#include "bmpc/bmpc_weights.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                     \
  do {                                                       \
    if (cond) {                                              \
      ++g_pass;                                              \
    } else {                                                 \
      ++g_fail;                                              \
      std::printf("  FAIL %s:%d: ", __FILE__, __LINE__);     \
      std::printf(__VA_ARGS__);                              \
      std::printf("\n");                                     \
    }                                                        \
  } while (0)

static double urand(double a, double b) { return a + (b - a) * (std::rand() / (double)RAND_MAX); }
static double clampValue(double value, double lower, double upper) { return std::max(lower, std::min(value, upper)); }
static double wrapToPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ------------------------------------------------------------------
// 기준 1: 원본 modelPredictiveController() 지연보상 v2 블록
// ------------------------------------------------------------------
struct RefDelayParams {
  double dc_state_k_, dc_state_c_s_, dc_state_max_s_, dc_act_base_s_, dc_act_frame_k_, dc_model_lf_;
  double dc_compute_max_s_, Lf;
  bool dc_lf_online_;
};

static void ref_delay_block(const RefDelayParams &p, double now, double period, double odom_age, double dc_compute_s_,
                            double velocity, double odom_yaw_rate_, double dc_lf_est_, const double coeffs[4],
                            const std::deque<mpc_tracking::TimedSteering> &steering_history_, double state[6])
{
  const double state_delay = clampValue(p.dc_state_k_ * period + p.dc_state_c_s_ + odom_age, 0.0, p.dc_state_max_s_);
  const double compute = clampValue(dc_compute_s_, 0.0, p.dc_compute_max_s_);
  const double actuation = std::max(0.0, p.dc_act_base_s_ + p.dc_act_frame_k_ * period);
  const auto lag_pose = mpc_tracking::measuredMotion(velocity, odom_yaw_rate_, state_delay);
  double lf = p.dc_model_lf_ > 0.0 ? p.dc_model_lf_ : p.Lf;
  if (p.dc_lf_online_ && dc_lf_est_ > 0.0) lf = clampValue(dc_lf_est_, 3.5, 5.0);
  const auto act_pose = mpc_tracking::predictSteeringDelay(velocity, lf, now + compute, actuation + compute,
                                                           steering_history_);
  const auto pose = mpc_tracking::compose(lag_pose, act_pose);
  const auto errors =
      mpc_tracking::trackingErrorsAt(pose, std::array<double, 4>{{coeffs[0], coeffs[1], coeffs[2], coeffs[3]}});
  state[0] = pose.x;
  state[1] = pose.y;
  state[2] = pose.yaw;
  state[3] = velocity;
  state[4] = errors[0];
  state[5] = wrapToPi(errors[1]);
}

// ------------------------------------------------------------------
// 기준 2: 원본 updateCostWeights() + Solve() steer_scale
// ------------------------------------------------------------------
struct RefWeights {
  double cte_cost_effective_weight, epsi_cost_effective_weight, delta_cost_effective_weight,
      delta_change_cost_effective_weight, delta_start_cost_weight;
};

static RefWeights ref_weights(const bmpc_config_t &c, bool is_highway_mode, bool is_avoidance_mode,
                              double actual_speed_mps)
{
  RefWeights r;
  double cte_cost_effective_weight, epsi_cost_effective_weight, delta_cost_effective_weight,
      delta_change_cost_effective_weight;
  double delta_start_cost_weight = c.w_delta_start;
  const double urban_cte_start = c.u_cte_start, urban_cte_urban = c.u_cte_urban, urban_epsi = c.u_epsi;
  const double urban_delta_start = c.u_delta_start, urban_delta_high = c.u_delta_high;
  const double urban_speed_start = c.u_speed_start, urban_speed_full = c.u_speed_full;
  const double urban_delta_change_start = c.u_ddelta_start, urban_delta_change_urban = c.u_ddelta_urban;
  const double hwy_cte = c.h_cte, hwy_epsi = c.h_epsi, hwy_delta = c.h_delta, hwy_delta_change = c.h_ddelta;

  // ---- 원본 본문 시작 ----
  if (is_avoidance_mode) {
    cte_cost_effective_weight = c.a_cte;
    epsi_cost_effective_weight = c.a_epsi;
    delta_cost_effective_weight = c.a_delta;
    delta_change_cost_effective_weight = c.a_ddelta;
  } else if (is_highway_mode) {
    cte_cost_effective_weight = hwy_cte;
    epsi_cost_effective_weight = hwy_epsi;
    delta_cost_effective_weight = hwy_delta;
    delta_change_cost_effective_weight = hwy_delta_change;
  } else {
    const double eval_speed = std::max(0.0, actual_speed_mps);
    if (eval_speed <= urban_speed_start) {
      cte_cost_effective_weight = urban_cte_start;
      epsi_cost_effective_weight = urban_epsi;
      delta_cost_effective_weight = urban_delta_start;
      delta_change_cost_effective_weight = urban_delta_change_start;
    } else if (eval_speed >= urban_speed_full) {
      cte_cost_effective_weight = urban_cte_urban;
      epsi_cost_effective_weight = urban_epsi;
      delta_cost_effective_weight = urban_delta_high;
      delta_change_cost_effective_weight = urban_delta_change_urban;
    } else {
      const double span = std::max(1.0e-3, urban_speed_full - urban_speed_start);
      double ratio = (eval_speed - urban_speed_start) / span;
      ratio = std::max(0.0, std::min(1.0, ratio));
      const double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
      cte_cost_effective_weight = urban_cte_start + smooth_ratio * (urban_cte_urban - urban_cte_start);
      epsi_cost_effective_weight = urban_epsi;
      delta_cost_effective_weight = urban_delta_start + smooth_ratio * (urban_delta_high - urban_delta_start);
      delta_change_cost_effective_weight =
          urban_delta_change_start + smooth_ratio * (urban_delta_change_urban - urban_delta_change_start);
    }
  }
  {
    double steer_scale = c.steer_scale;
    steer_scale = std::max(0.05, std::min(steer_scale, 5.0));
    double change_scale = c.steer_change_scale;
    change_scale = std::max(0.05, std::min(change_scale, 5.0));
    delta_start_cost_weight *= steer_scale;
    delta_cost_effective_weight *= steer_scale;
    delta_change_cost_effective_weight *= change_scale;
  }
  // ---- 원본 본문 끝 ----
  r.cte_cost_effective_weight = cte_cost_effective_weight;
  r.epsi_cost_effective_weight = epsi_cost_effective_weight;
  r.delta_cost_effective_weight = delta_cost_effective_weight;
  r.delta_change_cost_effective_weight = delta_change_cost_effective_weight;
  r.delta_start_cost_weight = delta_start_cost_weight;
  return r;
}

// ------------------------------------------------------------------
// 기준 3: 원본 makeSteeringCommand() / setThrottleCommand() / 속도 가드
// ------------------------------------------------------------------
struct RefPost {
  double dt, steering_command_sign_, steer_lowerBound, steer_upperBound, steering_command_rate_limit_radps_;
  bool steering_output_lpf_enabled_;
  double steering_output_lpf_alpha_, steering_output_lpf_alpha_high_speed_;
  bool have_competition_feedback_ = true;
  double competition_speed_mps_ = 0, cur_speed_ = 0;
  double previous_steering_command_rad_ = 0;
  bool have_previous_steering_command_ = false;

  double throttle_brake_limit_, throttle_accel_limit_, throttle_integral_ = 0, throttle_integral_band_mps_,
      throttle_integral_limit_, throttle_ff_gain_, ff_prev_target_mps_ = -1.0, ff_target_accel_ = 0,
      throttle_ff_tau_s_, throttle_ff_min_accel_, throttle_ff_accel_per_pedal_, throttle_ff_taper_mps_,
      throttle_ff_steer_zero_rad_, throttle_kp_, throttle_ki_, throttle_pedal_deadband_, brake_rate_limit_per_sec_,
      last_commanded_brake_ = 0;
  double steering_angle = 0;
  double legal_guard_cut_mps_, legal_guard_brake_at_mps_, legal_guard_brake_;
  struct {
    double accel = 0, brake = 0;
  } ackermann_msg_;

  double makeSteeringCommand(double mpc_steering_rad)
  {
    double command = steering_command_sign_ * mpc_steering_rad;
    command = clampValue(command, steer_lowerBound, steer_upperBound);

    if (steering_command_rate_limit_radps_ > 0.0 && have_previous_steering_command_) {
      const double max_step = steering_command_rate_limit_radps_ * dt;
      command = clampValue(command,
                           previous_steering_command_rad_ - max_step,
                           previous_steering_command_rad_ + max_step);
    }

    if (steering_output_lpf_enabled_ && have_previous_steering_command_) {
      double spd = have_competition_feedback_ ? competition_speed_mps_ : cur_speed_;
      double effective_lpf_alpha = steering_output_lpf_alpha_;
      if (spd <= 11.0) {
        effective_lpf_alpha = steering_output_lpf_alpha_;
      } else if (spd >= 25.0) {
        effective_lpf_alpha = steering_output_lpf_alpha_high_speed_;
      } else {
        const double r = (spd - 11.0) / (25.0 - 11.0);
        effective_lpf_alpha = steering_output_lpf_alpha_ + r * (steering_output_lpf_alpha_high_speed_ - steering_output_lpf_alpha_);
      }
      command = effective_lpf_alpha * command
          + (1.0 - effective_lpf_alpha) * previous_steering_command_rad_;
      command = clampValue(command, steer_lowerBound, steer_upperBound);
    }

    previous_steering_command_rad_ = command;
    have_previous_steering_command_ = true;
    return command;
  }

  void setThrottleCommand(double target_speed_mps, double actual_speed_mps, double dt,
                          double brake_limit = -1.0, double accel_limit = -1.0)
  {
    if (brake_limit < 0.0) brake_limit = throttle_brake_limit_;
    if (accel_limit < 0.0) accel_limit = throttle_accel_limit_;

    if (target_speed_mps <= 0.05) {
      throttle_integral_ = 0.0;
      ff_prev_target_mps_ = 0.0;
      ff_target_accel_ = 0.0;
      ackermann_msg_.accel = 0.0;
      double desired_stop_brake = brake_limit;
      const double max_brake_increase = brake_rate_limit_per_sec_ * dt;
      if (desired_stop_brake > last_commanded_brake_ + max_brake_increase) {
        desired_stop_brake = last_commanded_brake_ + max_brake_increase;
      }
      last_commanded_brake_ = desired_stop_brake;
      ackermann_msg_.brake = desired_stop_brake;
      return;
    }

    const double speed_error = target_speed_mps - actual_speed_mps;
    if (throttle_integral_band_mps_ <= 0.0 || std::fabs(speed_error) < throttle_integral_band_mps_) {
      throttle_integral_ += speed_error * dt;
    }
    if (throttle_integral_band_mps_ > 0.0 && speed_error < -0.1 && throttle_integral_ > 0.0) {
      throttle_integral_ = 0.0;
    }
    throttle_integral_ = clampValue(throttle_integral_, -throttle_integral_limit_,
                                    throttle_integral_limit_);

    double pedal_ff = 0.0;
    if (throttle_ff_gain_ > 0.0) {
      if (ff_prev_target_mps_ >= 0.0) {
        const double accel_raw = (target_speed_mps - ff_prev_target_mps_) / dt;
        ff_target_accel_ += dt / (throttle_ff_tau_s_ + dt) * (accel_raw - ff_target_accel_);
      }
      const double accel_ff = ff_target_accel_ > throttle_ff_min_accel_
          ? std::min(ff_target_accel_, throttle_ff_accel_per_pedal_) : 0.0;
      const double taper = clampValue(speed_error / throttle_ff_taper_mps_, 0.0, 1.0);
      const double steer_taper = clampValue(
          1.0 - std::fabs(steering_angle) / throttle_ff_steer_zero_rad_, 0.0, 1.0);
      pedal_ff = throttle_ff_gain_ * taper * steer_taper * accel_ff / throttle_ff_accel_per_pedal_;
    }
    ff_prev_target_mps_ = target_speed_mps;

    const double pedal = throttle_kp_ * speed_error + throttle_ki_ * throttle_integral_ + pedal_ff;

    if (throttle_pedal_deadband_ > 0.0
        && std::fabs(pedal) < throttle_pedal_deadband_) {
      ackermann_msg_.accel = 0.0;
      ackermann_msg_.brake = 0.0;
      last_commanded_brake_ = 0.0;
      return;
    }

    ackermann_msg_.accel = clampValue(pedal, 0.0, accel_limit);
    double desired_brake = clampValue(-pedal, 0.0, brake_limit);
    const double max_brake_increase = brake_rate_limit_per_sec_ * dt;
    if (desired_brake > last_commanded_brake_ + max_brake_increase) {
      desired_brake = last_commanded_brake_ + max_brake_increase;
    }
    last_commanded_brake_ = desired_brake;
    ackermann_msg_.brake = desired_brake;
  }

  void legalGuard(double actual_speed)
  {
    if (actual_speed > legal_guard_cut_mps_) {
      ackermann_msg_.accel = 0.0;
    }
    if (actual_speed > legal_guard_brake_at_mps_ && ackermann_msg_.brake < legal_guard_brake_) {
      ackermann_msg_.brake = legal_guard_brake_;
      last_commanded_brake_ = legal_guard_brake_;
    }
  }
};

// ------------------------------------------------------------------
// 시험
// ------------------------------------------------------------------
static void test_delay_functions()
{
  std::printf("[delay functions vs steering_delay.h]\n");
  double worst = 0;
  for (int trial = 0; trial < 3000; ++trial) {
    const double speed = urand(0, 25), yaw_rate = urand(-0.7, 0.7), seconds = urand(0, 0.15);
    const auto a = mpc_tracking::measuredMotion(speed, yaw_rate, seconds);
    const bmpc_pose2_t b = bmpc_measured_motion(speed, yaw_rate, seconds);
    worst = std::max(worst, std::max(std::fabs(a.x - b.x), std::max(std::fabs(a.y - b.y), std::fabs(a.yaw - b.yaw))));

    // 이력: 1 s 안의 명령 0~25 개, 시각은 us 단위 정수
    const int64_t now_us = 1000000000LL + (int64_t)(std::rand() % 1000);
    std::deque<mpc_tracking::TimedSteering> hist_ref;
    bmpc_hist_t hist;
    bmpc_hist_clear(&hist);
    const int n = std::rand() % 26;
    int64_t t = now_us - 1000000;
    for (int i = 0; i < n; ++i) {
      t += 20000 + std::rand() % 40000;
      if (t > now_us) break;
      const double d = urand(-0.6, 0.6);
      hist_ref.push_back({t * 1e-6, d});
      bmpc_hist_push(&hist, t, d);
    }
    const double lf = urand(3.5, 5), compute = urand(0, 0.1), act = urand(0.05, 0.12);
    const auto pa = mpc_tracking::predictSteeringDelay(speed, lf, now_us * 1e-6 + compute, act + compute, hist_ref);
    const bmpc_pose2_t pb = bmpc_predict_steering_delay(speed, lf, compute, act + compute, &hist, now_us);
    worst = std::max(worst, std::max(std::fabs(pa.x - pb.x), std::max(std::fabs(pa.y - pb.y), std::fabs(pa.yaw - pb.yaw))));

    const auto ca = mpc_tracking::compose(a, pa);
    const bmpc_pose2_t cb = bmpc_compose(b, pb);
    worst = std::max(worst, std::fabs(ca.x - cb.x) + std::fabs(ca.y - cb.y) + std::fabs(ca.yaw - cb.yaw));

    const double c[4] = {urand(-2, 2), urand(-0.5, 0.5), urand(-0.03, 0.03), urand(-1e-3, 1e-3)};
    const auto ea = mpc_tracking::trackingErrorsAt(ca, std::array<double, 4>{{c[0], c[1], c[2], c[3]}});
    double eb[2];
    bmpc_tracking_errors_at(cb, c, eb);
    worst = std::max(worst, std::fabs(ea[0] - eb[0]) + std::fabs(ea[1] - eb[1]));
  }
  // 이력 시각은 원본이 1e9 us 근처 절대 시각(double s)이라 1e-10 수준 반올림 차가 난다
  CHECK(worst < 1e-9, "worst diff %g", worst);
  std::printf("  worst |diff| %.2e over 3000 random cases\n", worst);
}

static void test_delay_block()
{
  std::printf("[delay comp v2 block]\n");
  bmpc_config_t cfg;
  bmpc_config_default(&cfg);
  RefDelayParams p{cfg.dc_state_k, cfg.dc_state_c, cfg.dc_state_max, cfg.dc_act_base, cfg.dc_act_frame_k,
                   cfg.dc_model_lf, cfg.dc_compute_max, cfg.Lf, cfg.dc_lf_online != 0};
  double worst = 0;
  for (int trial = 0; trial < 3000; ++trial) {
    const int64_t now_us = 2000000000LL;
    std::deque<mpc_tracking::TimedSteering> hist_ref;
    bmpc_hist_t hist;
    bmpc_hist_clear(&hist);
    int64_t t = now_us - 900000;
    while (true) {
      t += 45000 + std::rand() % 10000;
      if (t > now_us) break;
      const double d = urand(-0.4, 0.4);
      hist_ref.push_back({t * 1e-6, d});
      bmpc_hist_push(&hist, t, d);
    }
    const double period = urand(0.005, 0.2), odom_age = urand(0, 0.05), compute = urand(0, 0.12);
    const double v = urand(0, 25), wz = urand(-0.6, 0.6), lf_est = (std::rand() & 1) ? urand(3, 6) : -1;
    const double c[4] = {urand(-1, 1), urand(-0.3, 0.3), urand(-0.03, 0.03), urand(-1e-3, 1e-3)};
    double sr[6], sb[6];
    ref_delay_block(p, now_us * 1e-6, period, odom_age, compute, v, wz, lf_est, c, hist_ref, sr);
    bmpc_delay_comp(&cfg, &hist, now_us, v, wz, period, odom_age, 0.0, compute, lf_est, c, sb, NULL);
    for (int i = 0; i < 6; ++i) worst = std::max(worst, std::fabs(sr[i] - sb[i]));
  }
  CHECK(worst < 1e-9, "worst state diff %g", worst);
  std::printf("  worst |state diff| %.2e over 3000 random cases\n", worst);
}

static void test_weights()
{
  std::printf("[weights]\n");
  bmpc_config_t cfg;
  bmpc_config_default(&cfg);
  double worst = 0;
  for (int trial = 0; trial < 5000; ++trial) {
    const bool hwy = (std::rand() % 4) == 0, avoid = (std::rand() % 5) == 0;
    const double v = urand(-1, 30);
    cfg.steer_scale = urand(0.01, 6);
    cfg.steer_change_scale = urand(0.01, 6);
    const RefWeights r = ref_weights(cfg, hwy, avoid, v);
    bmpc_weights_t w;
    bmpc_compute_weights(&cfg, hwy, avoid, v, &w);
    worst = std::max(worst, std::fabs(r.cte_cost_effective_weight - w.cte) + std::fabs(r.epsi_cost_effective_weight - w.epsi) +
                                std::fabs(r.delta_cost_effective_weight - w.delta) +
                                std::fabs(r.delta_change_cost_effective_weight - w.ddelta) +
                                std::fabs(r.delta_start_cost_weight - w.delta_start));
  }
  CHECK(worst < 1e-6, "worst weight diff %g", worst);
  std::printf("  worst |diff| %.2e over 5000 random cases\n", worst);
}

static void test_post()
{
  std::printf("[post-processing, 20000-step random sequence]\n");
  bmpc_config_t cfg;
  bmpc_config_default(&cfg);
  RefPost ref;
  ref.dt = cfg.dt;
  ref.steering_command_sign_ = cfg.steer_sign;
  ref.steer_lowerBound = cfg.steer_lb;
  ref.steer_upperBound = cfg.steer_ub;
  ref.steering_command_rate_limit_radps_ = cfg.st_rate_limit;
  ref.steering_output_lpf_enabled_ = cfg.st_lpf_enabled != 0;
  ref.steering_output_lpf_alpha_ = cfg.st_lpf_alpha;
  ref.steering_output_lpf_alpha_high_speed_ = cfg.st_lpf_alpha_high;
  ref.throttle_brake_limit_ = cfg.lg_brake_limit;
  ref.throttle_accel_limit_ = cfg.lg_accel_limit;
  ref.throttle_integral_band_mps_ = cfg.lg_int_band;
  ref.throttle_integral_limit_ = cfg.lg_int_limit;
  ref.throttle_ff_gain_ = cfg.lg_ff_gain;
  ref.throttle_ff_tau_s_ = cfg.lg_ff_tau;
  ref.throttle_ff_min_accel_ = cfg.lg_ff_min_accel;
  ref.throttle_ff_accel_per_pedal_ = cfg.lg_ff_accel_per_pedal;
  ref.throttle_ff_taper_mps_ = cfg.lg_ff_taper;
  ref.throttle_ff_steer_zero_rad_ = cfg.lg_ff_steer_zero;
  ref.throttle_kp_ = cfg.lg_kp;
  ref.throttle_ki_ = cfg.lg_ki;
  ref.throttle_pedal_deadband_ = cfg.lg_deadband;
  ref.brake_rate_limit_per_sec_ = cfg.lg_brake_rate;
  ref.legal_guard_cut_mps_ = cfg.legal_cut;
  ref.legal_guard_brake_at_mps_ = cfg.legal_brake_at;
  ref.legal_guard_brake_ = cfg.legal_brake;

  bmpc_post_state_t st;
  bmpc_post_init(&st);
  double delta = 0, speed = 5, target = 8, worst = 0;
  for (int k = 0; k < 20000; ++k) {
    delta = clampValue(delta + urand(-0.05, 0.05), -0.8, 0.8);
    speed = clampValue(speed + urand(-0.4, 0.4), 0, 26);
    if (std::rand() % 50 == 0) target = (std::rand() % 6 == 0) ? 0.0 : urand(0, 25);
    else target = clampValue(target + urand(-0.2, 0.3), 0, 25);
    const bool kick = std::rand() % 10 == 0, hwy = std::rand() % 3 == 0;
    const double brake_limit = kick ? cfg.lg_kick_brake_limit : cfg.lg_brake_limit;
    const double accel_limit = hwy ? cfg.lg_accel_limit_exempt : cfg.lg_accel_limit;

    ref.competition_speed_mps_ = speed;
    ref.steering_angle = delta;
    const double s_ref = ref.makeSteeringCommand(delta);
    ref.setThrottleCommand(target, speed, 0.05, brake_limit, accel_limit);
    if (!hwy) ref.legalGuard(speed);

    double a, b;
    const double s_c = bmpc_steer_post(&cfg, &st, delta, speed);
    bmpc_throttle(&cfg, &st, target, speed, 0.05, brake_limit, accel_limit, delta, &a, &b);
    if (cfg.legal_enabled && !hwy) bmpc_legal_guard(&cfg, &st, speed, &a, &b);

    worst = std::max(worst, std::fabs(s_ref - s_c) + std::fabs(ref.ackermann_msg_.accel - a) +
                                std::fabs(ref.ackermann_msg_.brake - b));
  }
  CHECK(worst < 1e-12, "worst output diff %g", worst);
  std::printf("  worst |steer|+|accel|+|brake| diff %.2e\n", worst);
}

int main()
{
  std::srand(777);
  test_delay_functions();
  test_delay_block();
  test_weights();
  test_post();
  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
