#include "bmpc/bmpc_post.h"

#include "bmpc/bmpc_math.h"

void bmpc_post_init(bmpc_post_state_t *st)
{
  st->prev_steer_cmd = 0;
  st->have_prev = 0;
  st->integral = 0;
  st->ff_prev_target = -1;
  st->ff_target_accel = 0;
  st->last_brake = 0;
}

bmpc_real bmpc_steer_post(const bmpc_config_t *cfg, bmpc_post_state_t *st, bmpc_real mpc_delta,
                          bmpc_real speed)
{
  bmpc_real command = cfg->steer_sign * mpc_delta;
  command = bm_clamp(command, cfg->steer_lb, cfg->steer_ub);

  if (cfg->st_rate_limit > 0 && st->have_prev) {
    const bmpc_real max_step = cfg->st_rate_limit * cfg->dt;
    command = bm_clamp(command, st->prev_steer_cmd - max_step, st->prev_steer_cmd + max_step);
  }

  if (cfg->st_lpf_enabled && st->have_prev) {
    bmpc_real alpha = cfg->st_lpf_alpha;
    if (speed <= cfg->st_lpf_v_low) {
      alpha = cfg->st_lpf_alpha;
    } else if (speed >= cfg->st_lpf_v_high) {
      alpha = cfg->st_lpf_alpha_high;
    } else {
      const bmpc_real r = (speed - cfg->st_lpf_v_low) / (cfg->st_lpf_v_high - cfg->st_lpf_v_low);
      alpha = cfg->st_lpf_alpha + r * (cfg->st_lpf_alpha_high - cfg->st_lpf_alpha);
    }
    command = alpha * command + ((bmpc_real)1.0 - alpha) * st->prev_steer_cmd;
    command = bm_clamp(command, cfg->steer_lb, cfg->steer_ub);
  }

  st->prev_steer_cmd = command;
  st->have_prev = 1;
  return command;
}

void bmpc_throttle(const bmpc_config_t *cfg, bmpc_post_state_t *st, bmpc_real target_speed,
                   bmpc_real actual_speed, bmpc_real dt, bmpc_real brake_limit,
                   bmpc_real accel_limit, bmpc_real steering_angle, bmpc_real *accel,
                   bmpc_real *brake)
{
  bmpc_real speed_error, pedal, pedal_ff = 0, desired_brake, max_brake_increase;

  if (brake_limit < 0) brake_limit = cfg->lg_brake_limit;
  if (accel_limit < 0) accel_limit = cfg->lg_accel_limit;

  if (target_speed <= (bmpc_real)0.05) {
    bmpc_real desired_stop_brake = brake_limit;
    st->integral = 0;
    st->ff_prev_target = 0;
    st->ff_target_accel = 0;
    *accel = 0;
    max_brake_increase = cfg->lg_brake_rate * dt;
    if (desired_stop_brake > st->last_brake + max_brake_increase) {
      desired_stop_brake = st->last_brake + max_brake_increase;
    }
    st->last_brake = desired_stop_brake;
    *brake = desired_stop_brake;
    return;
  }

  speed_error = target_speed - actual_speed;
  /* band > 0 이면 목표 근처에서만 적분한다 */
  if (cfg->lg_int_band <= 0 || BM_FABS(speed_error) < cfg->lg_int_band) {
    st->integral += speed_error * dt;
  }
  /* 목표보다 빠른데 양의 적분이 남아 있으면 지운다 */
  if (cfg->lg_int_band > 0 && speed_error < (bmpc_real)-0.1 && st->integral > 0) {
    st->integral = 0;
  }
  st->integral = bm_clamp(st->integral, -cfg->lg_int_limit, cfg->lg_int_limit);

  /* 목표속도 램프 피드포워드 */
  if (cfg->lg_ff_gain > 0) {
    bmpc_real accel_ff, taper, steer_taper;
    if (st->ff_prev_target >= 0) {
      const bmpc_real accel_raw = (target_speed - st->ff_prev_target) / dt;
      st->ff_target_accel += dt / (cfg->lg_ff_tau + dt) * (accel_raw - st->ff_target_accel);
    }
    accel_ff = st->ff_target_accel > cfg->lg_ff_min_accel
                   ? bm_min(st->ff_target_accel, cfg->lg_ff_accel_per_pedal)
                   : (bmpc_real)0;
    taper = bm_clamp(speed_error / cfg->lg_ff_taper, (bmpc_real)0.0, (bmpc_real)1.0);
    /* 조향 중(곡선 출구)에는 쓰지 않는다 */
    steer_taper = bm_clamp((bmpc_real)1.0 - BM_FABS(steering_angle) / cfg->lg_ff_steer_zero,
                           (bmpc_real)0.0, (bmpc_real)1.0);
    pedal_ff = cfg->lg_ff_gain * taper * steer_taper * accel_ff / cfg->lg_ff_accel_per_pedal;
  }
  st->ff_prev_target = target_speed;

  pedal = cfg->lg_kp * speed_error + cfg->lg_ki * st->integral + pedal_ff;

  /* 데드밴드: 가속·제동 모두 0 (타행) */
  if (cfg->lg_deadband > 0 && BM_FABS(pedal) < cfg->lg_deadband) {
    *accel = 0;
    *brake = 0;
    st->last_brake = 0;
    return;
  }

  *accel = bm_clamp(pedal, (bmpc_real)0.0, accel_limit);
  desired_brake = bm_clamp(-pedal, (bmpc_real)0.0, brake_limit);
  max_brake_increase = cfg->lg_brake_rate * dt;
  if (desired_brake > st->last_brake + max_brake_increase) {
    desired_brake = st->last_brake + max_brake_increase;
  }
  st->last_brake = desired_brake;
  *brake = desired_brake;
}

void bmpc_legal_guard(const bmpc_config_t *cfg, bmpc_post_state_t *st, bmpc_real actual_speed,
                      bmpc_real *accel, bmpc_real *brake)
{
  if (actual_speed > cfg->legal_cut) *accel = 0;
  if (actual_speed > cfg->legal_brake_at && *brake < cfg->legal_brake) {
    *brake = cfg->legal_brake;
    st->last_brake = cfg->legal_brake;
  }
}
