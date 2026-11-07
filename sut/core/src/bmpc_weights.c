#include "bmpc/bmpc_weights.h"

#include "bmpc/bmpc_math.h"

void bmpc_compute_weights(const bmpc_config_t *cfg, int highway, int avoidance,
                          bmpc_real speed, bmpc_weights_t *w)
{
  bmpc_real steer_scale, change_scale;

  w->v = cfg->w_v;
  w->delta_start = cfg->w_delta_start;
  w->vt = cfg->w_vt;
  w->dvt = cfg->w_dvt;

  if (avoidance) {
    w->cte = cfg->a_cte;
    w->epsi = cfg->a_epsi;
    w->delta = cfg->a_delta;
    w->ddelta = cfg->a_ddelta;
  } else if (highway) {
    w->cte = cfg->h_cte;
    w->epsi = cfg->h_epsi;
    w->delta = cfg->h_delta;
    w->ddelta = cfg->h_ddelta;
  } else {
    const bmpc_real eval_speed = bm_max((bmpc_real)0.0, speed);
    if (eval_speed <= cfg->u_speed_start) {
      w->cte = cfg->u_cte_start;
      w->epsi = cfg->u_epsi;
      w->delta = cfg->u_delta_start;
      w->ddelta = cfg->u_ddelta_start;
    } else if (eval_speed >= cfg->u_speed_full) {
      w->cte = cfg->u_cte_urban;
      w->epsi = cfg->u_epsi;
      w->delta = cfg->u_delta_high;
      w->ddelta = cfg->u_ddelta_urban;
    } else {
      const bmpc_real span = bm_max((bmpc_real)1.0e-3, cfg->u_speed_full - cfg->u_speed_start);
      bmpc_real ratio = (eval_speed - cfg->u_speed_start) / span;
      bmpc_real s;
      ratio = bm_max((bmpc_real)0.0, bm_min((bmpc_real)1.0, ratio));
      s = ratio * ratio * ((bmpc_real)3.0 - (bmpc_real)2.0 * ratio);
      w->cte = cfg->u_cte_start + s * (cfg->u_cte_urban - cfg->u_cte_start);
      w->epsi = cfg->u_epsi;
      w->delta = cfg->u_delta_start + s * (cfg->u_delta_high - cfg->u_delta_start);
      w->ddelta = cfg->u_ddelta_start + s * (cfg->u_ddelta_urban - cfg->u_ddelta_start);
    }
  }

  /* Solve(): 조향 항(delta_start, delta)은 steer_scale, 조향 변화 항은 steer_change_scale */
  steer_scale = bm_clamp(cfg->steer_scale, (bmpc_real)0.05, (bmpc_real)5.0);
  change_scale = bm_clamp(cfg->steer_change_scale, (bmpc_real)0.05, (bmpc_real)5.0);
  w->delta_start *= steer_scale;
  w->delta *= steer_scale;
  w->ddelta *= change_scale;
}
