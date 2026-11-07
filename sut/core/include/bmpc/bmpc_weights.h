/* 가중치 스케줄. 원본 mpc.cpp updateCostWeights() + Solve() 의 steer_scale 적용과 같다. */
#ifndef BMPC_WEIGHTS_H
#define BMPC_WEIGHTS_H

#include "bmpc/bmpc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bmpc_real cte, epsi, delta, ddelta; /* 구간별로 바뀌는 4 개 (steer_scale 적용 후) */
  bmpc_real delta_start;              /* 직전 조향과의 연속성 (steer_scale 적용 후) */
  bmpc_real v, vt, dvt;               /* 고정 */
} bmpc_weights_t;

/* 우선순위: 회피 > 고속 구간 > 도심(속도 8.5~11.5 m/s 사이 smoothstep 블렌딩) */
void bmpc_compute_weights(const bmpc_config_t *cfg, int highway, int avoidance,
                          bmpc_real speed, bmpc_weights_t *w);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_WEIGHTS_H */
