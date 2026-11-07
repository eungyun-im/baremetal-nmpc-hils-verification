#include "bmpc/bmpc_delay.h"

#include "bmpc/bmpc_math.h"

static bmpc_real us_to_s(int64_t dt_us) { return (bmpc_real)dt_us * (bmpc_real)1e-6; }

void bmpc_hist_clear(bmpc_hist_t *h)
{
  h->head = 0;
  h->count = 0;
}

void bmpc_hist_push(bmpc_hist_t *h, int64_t t_us, bmpc_real delta)
{
  int pos;
  if (h->count == BMPC_HIST_MAX) bmpc_hist_pop_front(h);
  pos = (h->head + h->count) % BMPC_HIST_MAX;
  h->t_us[pos] = t_us;
  h->delta[pos] = delta;
  ++h->count;
}

void bmpc_hist_pop_front(bmpc_hist_t *h)
{
  if (h->count == 0) return;
  h->head = (h->head + 1) % BMPC_HIST_MAX;
  --h->count;
}

int64_t bmpc_hist_time(const bmpc_hist_t *h, int i) { return h->t_us[(h->head + i) % BMPC_HIST_MAX]; }

bmpc_real bmpc_hist_delta(const bmpc_hist_t *h, int i)
{
  return h->delta[(h->head + i) % BMPC_HIST_MAX];
}

void bmpc_hist_expire_check(bmpc_hist_t *h, int64_t now_us)
{
  if (h->count > 0) {
    const int64_t back = bmpc_hist_time(h, h->count - 1);
    if (now_us < back || us_to_s(now_us - back) > (bmpc_real)1.0) bmpc_hist_clear(h);
  }
}

bmpc_pose2_t bmpc_measured_motion(bmpc_real speed, bmpc_real yaw_rate, bmpc_real seconds)
{
  bmpc_pose2_t p;
  const bmpc_real angle = yaw_rate * seconds;
  if (BM_FABS(yaw_rate) < (bmpc_real)1e-6) {
    p.x = speed * seconds;
    p.y = 0;
    p.yaw = angle;
    return p;
  }
  p.x = speed / yaw_rate * BM_SIN(angle);
  p.y = speed / yaw_rate * ((bmpc_real)1.0 - BM_COS(angle));
  p.yaw = angle;
  return p;
}

bmpc_pose2_t bmpc_predict_steering_delay(bmpc_real speed, bmpc_real lf, bmpc_real now_rel,
                                         bmpc_real delay, const bmpc_hist_t *hist,
                                         int64_t t_ref_us)
{
  bmpc_pose2_t pose = {0, 0, 0};
  bmpc_real source_time;
  bmpc_real delta = 0;
  int next = 0;
  const int n = hist->count;

  if (delay <= 0 || lf <= 0) return pose;
  source_time = now_rel - delay;
  while (next < n && us_to_s(bmpc_hist_time(hist, next) - t_ref_us) <= source_time) {
    delta = bmpc_hist_delta(hist, next);
    ++next;
  }
  while (source_time < now_rel) {
    const bmpc_real next_t = next < n ? us_to_s(bmpc_hist_time(hist, next) - t_ref_us) : now_rel;
    const bmpc_real end = next < n ? bm_min(now_rel, next_t) : now_rel;
    const bmpc_real duration = end - source_time;
    const bmpc_real rate = -speed * delta / lf;
    const bmpc_real yaw_end = pose.yaw + rate * duration;
    if (BM_FABS(rate) < (bmpc_real)1e-9) {
      pose.x += speed * BM_COS(pose.yaw) * duration;
      pose.y += speed * BM_SIN(pose.yaw) * duration;
    } else {
      pose.x += speed / rate * (BM_SIN(yaw_end) - BM_SIN(pose.yaw));
      pose.y += speed / rate * (BM_COS(pose.yaw) - BM_COS(yaw_end));
    }
    /* 원본의 reference_offset 항은 기본값 0 이라 생략 */
    pose.yaw = yaw_end;
    source_time = end;
    if (next < n && us_to_s(bmpc_hist_time(hist, next) - t_ref_us) <= source_time) {
      delta = bmpc_hist_delta(hist, next);
      ++next;
    }
  }
  return pose;
}

bmpc_pose2_t bmpc_compose(bmpc_pose2_t a, bmpc_pose2_t b)
{
  bmpc_pose2_t p;
  p.x = a.x + BM_COS(a.yaw) * b.x - BM_SIN(a.yaw) * b.y;
  p.y = a.y + BM_SIN(a.yaw) * b.x + BM_COS(a.yaw) * b.y;
  p.yaw = a.yaw + b.yaw;
  return p;
}

void bmpc_tracking_errors_at(bmpc_pose2_t pose, const bmpc_real c[4], bmpc_real out[2])
{
  const bmpc_real path_y = c[0] + pose.x * (c[1] + pose.x * (c[2] + pose.x * c[3]));
  const bmpc_real slope = c[1] + pose.x * ((bmpc_real)2.0 * c[2] + pose.x * (bmpc_real)3.0 * c[3]);
  out[0] = path_y - pose.y;
  out[1] = pose.yaw - BM_ATAN(slope);
}

void bmpc_delay_comp(const bmpc_config_t *cfg, const bmpc_hist_t *hist, int64_t now_us,
                     bmpc_real velocity, bmpc_real yaw_rate, bmpc_real period,
                     bmpc_real odom_age, bmpc_real transport_req, bmpc_real compute,
                     bmpc_real lf_est, const bmpc_real c[4], bmpc_real s0[BMPC_NX],
                     bmpc_dc_info_t *info)
{
  bmpc_pose2_t lag_pose, act_pose, pose;
  bmpc_real errors[2];
  bmpc_real state_delay, actuation, lf;

  period = bm_clamp(period, (bmpc_real)0.005, (bmpc_real)0.2);
  /* ① 상태 지연: 원본 k*T + c + odom_age 에 HILS 의 요청 전송 시간을 더한다 */
  state_delay = bm_clamp(cfg->dc_state_k * period + cfg->dc_state_c + odom_age + transport_req,
                         (bmpc_real)0.0, cfg->dc_state_max);
  /* ② 계산 지연 */
  compute = bm_clamp(compute, (bmpc_real)0.0, cfg->dc_compute_max);
  /* ③ 명령 반영 지연 */
  actuation = bm_max((bmpc_real)0.0, cfg->dc_act_base + cfg->dc_act_frame_k * period);

  /* 1) 상태는 state_delay 만큼 늦다: 측정 속도·yaw rate 로 지금까지 당긴다. */
  lag_pose = bmpc_measured_motion(velocity, yaw_rate, state_delay);
  /* 2) 새 명령이 반영될 때까지(계산 + 조향 지연) 이미 보낸 명령을 따른다: 이력을 실효 Lf 로 적분 */
  lf = cfg->dc_model_lf > 0 ? cfg->dc_model_lf : cfg->Lf;
  if (cfg->dc_lf_online && lf_est > 0) lf = bm_clamp(lf_est, cfg->dc_lf_min, cfg->dc_lf_max);
  act_pose = bmpc_predict_steering_delay(velocity, lf, compute, actuation + compute, hist, now_us);
  pose = bmpc_compose(lag_pose, act_pose);
  bmpc_tracking_errors_at(pose, c, errors);

  s0[BMPC_SX] = pose.x;
  s0[BMPC_SY] = pose.y;
  s0[BMPC_SPSI] = pose.yaw;
  s0[BMPC_SV] = velocity;
  s0[BMPC_SCTE] = errors[0];
  s0[BMPC_SEPSI] = bm_wrap_pi(errors[1]);

  if (info) {
    info->state_delay = state_delay;
    info->compute = compute;
    info->actuation = actuation;
    info->lf = lf;
  }
}
