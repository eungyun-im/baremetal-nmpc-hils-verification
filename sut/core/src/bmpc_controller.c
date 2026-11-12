#include "bmpc/bmpc_controller.h"

#include <string.h>

#include "bmpc/bmpc_math.h"

#ifdef BMPC_TEST_HOOK
volatile int bmpc_test_force_solve_fail = 0;
#endif

static bmpc_real us_to_s(int64_t dt_us) { return (bmpc_real)dt_us * (bmpc_real)1e-6; }

void bmpc_controller_init(bmpc_controller_t *c, const bmpc_config_t *cfg)
{
  memset(c, 0, sizeof(*c));
  c->cfg = *cfg;
  bmpc_config_sanitize(&c->cfg);
  c->configured = 1;
  bmpc_hist_clear(&c->hist);
  bmpc_post_init(&c->post);
  c->steering_angle = 0;
  c->target_velocity = 0;
  c->compute_s = c->cfg.dc_compute_init;
  c->warm_valid = 0;
}

static int request_finite(const bmpc_request_t *r)
{
  int i;
  for (i = 0; i < 4; ++i)
    if (!bm_isfinite(r->c[i])) return 0;
  return bm_isfinite(r->velocity) && bm_isfinite(r->yaw_rate) && bm_isfinite(r->ref_v) &&
         bm_isfinite(r->v_lb) && bm_isfinite(r->v_ub);
}

/* 원본 predict_FutureState() (지연보상을 끈 경우) */
static void predict_future_state(const bmpc_config_t *cfg, bmpc_real v, bmpc_real delta, bmpc_real cte,
                                 bmpc_real epsi, bmpc_real target_v, bmpc_real s0[BMPC_NX])
{
  const bmpc_real dt = cfg->dt;
  const bmpc_real psi = v * (-delta) * dt / cfg->Lf;
  s0[BMPC_SX] = v * dt;
  s0[BMPC_SY] = 0;
  s0[BMPC_SPSI] = psi;
  s0[BMPC_SV] = v + cfg->k_v * (target_v - v) * dt;
  s0[BMPC_SCTE] = cte + v * BM_SIN(epsi) * dt;
  s0[BMPC_SEPSI] = epsi + psi;
}

/* makeSteeringCommand() 뒤쪽: 이력 기록, ② 실측 갱신, 1 s 넘은 이력 정리 */
static void record_command(bmpc_controller_t *c, bmpc_real steer_cmd, int64_t now_us, int pred_pending,
                           int64_t t_pred_us, bmpc_real transport_resp)
{
  const bmpc_config_t *cfg = &c->cfg;
  bmpc_hist_expire_check(&c->hist, now_us);
  if (BM_FABS(cfg->steer_sign) > (bmpc_real)1e-6) bmpc_hist_push(&c->hist, now_us, steer_cmd / cfg->steer_sign);
  if (pred_pending) {
    /* 예측 시점 -> 명령 확정까지 걸린 시간 + 응답이 PC 에 닿는 시간.
     * 올라갈 때는 바로, 내려갈 때는 천천히 따라간다. */
    const bmpc_real sample = bm_min(us_to_s(now_us - t_pred_us) + transport_resp, cfg->dc_compute_max);
    c->compute_s = sample > c->compute_s ? sample : c->compute_s + (bmpc_real)0.1 * (sample - c->compute_s);
  }
  while (c->hist.count > 1 && us_to_s(now_us - bmpc_hist_time(&c->hist, 1)) > (bmpc_real)1.0) {
    bmpc_hist_pop_front(&c->hist);
  }
}

int bmpc_controller_step(bmpc_controller_t *c, const bmpc_request_t *req, bmpc_now_fn now_fn,
                         void *now_ctx, bmpc_response_t *resp)
{
  const bmpc_config_t *cfg = &c->cfg;
  const int highway = (req->flags & BMPC_FLAG_HIGHWAY) != 0;
  const int avoidance = (req->flags & BMPC_FLAG_AVOIDANCE) != 0;
  const int kick = (req->flags & BMPC_FLAG_KICK) != 0;
  const int64_t t_pred = now_fn(now_ctx);
  const bmpc_real transport = bm_max((bmpc_real)0.0, req->transport);
  const bmpc_real transport_req = cfg->dc_transport_req_share * transport;
  const bmpc_real transport_resp = transport - transport_req;
  const bmpc_real velocity = req->velocity;
  bmpc_real accel = 0, brake = 0, steer_cmd;
  bmpc_real brake_limit, accel_limit;
  int pred_pending = 0;
  int status = BMPC_OK;

  memset(resp, 0, sizeof(*resp));
  resp->seq = req->seq;
  ++c->steps;

  if (!c->configured) {
    resp->status = BMPC_ERR_NOT_CONFIGURED;
    return resp->status;
  }
  if (!request_finite(req)) {
    ++c->errors;
    resp->status = BMPC_ERR_NUMERIC;
    resp->steer_cmd = c->post.prev_steer_cmd;
    return resp->status;
  }
  if (req->flags & BMPC_FLAG_RESET) c->warm_valid = 0;

  if (req->flags & BMPC_FLAG_STOP) {
    /* 경로 끝 정지·ACC 0: MPC 를 풀지 않고 직전 조향을 유지한 채 목표 0 으로 후처리만 한다 */
    c->target_velocity = 0;
    c->warm_valid = 0;
    resp->delta0 = c->steering_angle;
    resp->vt_raw = 0;
  } else {
    bmpc_problem_t prob;
    bmpc_dc_info_t dc;
    const int nu = cfg->N - 1;
    const bmpc_real actual_steering =
        (c->post.have_prev && BM_FABS(cfg->steer_sign) > (bmpc_real)1e-6) ? c->post.prev_steer_cmd / cfg->steer_sign
                                                                          : c->steering_angle;
    int64_t t_solve0, t_solve1;

    memcpy(prob.c, req->c, sizeof(prob.c));
    prob.prev_delta_ref = actual_steering;
    prob.ref_v = req->ref_v;
    prob.v_ub = req->v_ub;
    prob.v_lb = req->v_lb > req->v_ub ? req->v_ub : req->v_lb;
    prob.steer_lb = cfg->steer_lb;
    prob.steer_ub = cfg->steer_ub;

    if (cfg->dc_enabled) {
      bmpc_hist_expire_check(&c->hist, t_pred);
      bmpc_delay_comp(cfg, &c->hist, t_pred, velocity, req->yaw_rate, req->period, req->odom_age,
                      transport_req, cfg->dc_compute_fixed > 0 ? cfg->dc_compute_fixed : c->compute_s,
                      req->lf_est, req->c, prob.s0, &dc);
      pred_pending = 1;
      resp->state_delay = dc.state_delay;
      resp->compute = dc.compute;
      resp->actuation = dc.actuation;
      resp->lf_used = dc.lf;
    } else {
      const bmpc_real cte = req->c[0];
      const bmpc_real epsi = -BM_ATAN(req->c[1]);
      predict_future_state(cfg, velocity, actual_steering, cte, epsi, c->target_velocity, prob.s0);
    }
    memcpy(c->last_s0, prob.s0, sizeof(c->last_s0));

    bmpc_compute_weights(cfg, highway, avoidance, velocity, &prob.w);

    t_solve0 = now_fn(now_ctx);
    status = bmpc_mpc_solve(cfg, &prob, c->warm_valid ? c->z_warm : NULL, &c->work, &c->result);
#ifdef BMPC_TEST_HOOK
    if (bmpc_test_force_solve_fail != 0) status = BMPC_ERR_NUMERIC;
#endif
    t_solve1 = now_fn(now_ctx);
    resp->solve_us = (uint32_t)(t_solve1 - t_solve0);
    resp->iters = (uint8_t)(c->result.qp_iters > 255 ? 255 : c->result.qp_iters);
    c->last_cost = c->result.cost;

    if (status == BMPC_ERR_NUMERIC || status == BMPC_ERR_INFEASIBLE ||
        !bm_isfinite(c->result.z[0]) || !bm_isfinite(c->result.z[nu])) {
      /* 풀지 못했으면 직전 조향·목표 속도를 유지한다 */
      ++c->errors;
      c->warm_valid = 0;
      if (status == BMPC_OK) status = BMPC_ERR_NUMERIC;
    } else {
      int k;
      c->steering_angle = c->result.z[0];
      c->target_velocity = c->result.z[nu];
      /* 다음 주기 시작점: 한 칸 앞으로 민다 */
      for (k = 0; k < nu; ++k) {
        const int src = k + 1 < nu ? k + 1 : nu - 1;
        c->z_warm[k] = c->result.z[src];
        c->z_warm[nu + k] = c->result.z[nu + src];
      }
      c->warm_valid = 1;
    }
    resp->delta0 = c->steering_angle;
    resp->vt_raw = c->target_velocity;

    /* 속도 후처리: 회피·정적 장애물이 켜진 첫 주기는 바로 맞추고, 구간 상한·경로 끝 감속으로 자른다 */
    if (req->snap >= 0) c->target_velocity = req->snap;
    if (req->post_cap >= 0) c->target_velocity = bm_min(c->target_velocity, req->post_cap);
  }

  /* publishControlCommand(): 조향 후처리 -> 종방향 -> 속도 가드 */
  steer_cmd = bmpc_steer_post(cfg, &c->post, c->steering_angle, velocity);
  record_command(c, steer_cmd, now_fn(now_ctx), pred_pending, t_pred, transport_resp);

  brake_limit = kick ? cfg->lg_kick_brake_limit : cfg->lg_brake_limit;
  accel_limit = highway ? cfg->lg_accel_limit_exempt : cfg->lg_accel_limit;
  bmpc_throttle(cfg, &c->post, c->target_velocity, velocity, cfg->dt, brake_limit, accel_limit,
                c->steering_angle, &accel, &brake);
  if (cfg->legal_enabled && !highway) bmpc_legal_guard(cfg, &c->post, velocity, &accel, &brake);

  resp->status = (uint8_t)status;
  resp->steer_cmd = steer_cmd;
  resp->accel = accel;
  resp->brake = brake;
  resp->vt_final = c->target_velocity;
  resp->total_us = (uint32_t)(now_fn(now_ctx) - t_pred);
  return status;
}

int bmpc_controller_note_applied(bmpc_controller_t *c, int64_t now_us, bmpc_real steer_cmd)
{
  const bmpc_config_t *cfg = &c->cfg;
  bmpc_real delta;
  if (!c->configured || BM_FABS(cfg->steer_sign) <= (bmpc_real)1e-6) return 0;
  delta = steer_cmd / cfg->steer_sign;
  if (c->hist.count > 0) {
    const int last = (c->hist.head + c->hist.count - 1) % BMPC_HIST_MAX;
    /* 안전 제어기 명령 형식(0.01 deg) 양자화보다 큰 차이만 (그보다 작으면 같은 명령으로 본다) */
    if (BM_FABS(c->hist.delta[last] - delta) < (bmpc_real)2e-4) return 0;
    if (now_us < c->hist.t_us[last]) return 0;
  }
  bmpc_hist_push(&c->hist, now_us, delta);
  return 1;
}
