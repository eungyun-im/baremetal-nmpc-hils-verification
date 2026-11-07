/* MCU 한 주기: 요청 -> 지연보상 -> 가중치 -> LTV-MPC -> 후처리 -> 응답.
 * 원본 modelPredictiveController() 에서 PC 에 남긴 부분(polyfit, 속도 제한 등)을 뺀 나머지다.
 * 시각은 now_fn(MCU 타이머, us)으로만 얻는다. 동적 메모리는 쓰지 않는다. */
#ifndef BMPC_CONTROLLER_H
#define BMPC_CONTROLLER_H

#include "bmpc/bmpc_config.h"
#include "bmpc/bmpc_delay.h"
#include "bmpc/bmpc_mpc.h"
#include "bmpc/bmpc_post.h"
#include "bmpc/bmpc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int64_t (*bmpc_now_fn)(void *ctx);

typedef struct {
  bmpc_config_t cfg;
  int configured;

  bmpc_hist_t hist;            /* 최종 조향 이력 (지연보상 ②③) */
  bmpc_post_state_t post;      /* 후처리 상태 */
  bmpc_real steering_angle;    /* 직전 MPC 원출력 조향 (MPC 부호) */
  bmpc_real target_velocity;   /* 직전 최종 목표 속도 */
  bmpc_real compute_s;         /* ② 실측 EMA (MCU 처리 + 응답 전송) */

  bmpc_real z_warm[BMPC_NZ_MAX]; /* 다음 주기 선형화 시작점 */
  int warm_valid;

  bmpc_mpc_work_t work;
  bmpc_mpc_result_t result;

  /* 진단 */
  bmpc_real last_s0[BMPC_NX];
  bmpc_real last_cost;
  uint32_t steps, errors;
} bmpc_controller_t;

void bmpc_controller_init(bmpc_controller_t *c, const bmpc_config_t *cfg);

/* 요청 하나를 처리해 resp 를 채운다. total_us 는 호출한 쪽(app)이 덮어쓴다. */
int bmpc_controller_step(bmpc_controller_t *c, const bmpc_request_t *req, bmpc_now_fn now_fn,
                         void *now_ctx, bmpc_response_t *resp);

/* 실제로 차량에 나간 조향이 이 controller 가 낸 명령과 다를 때 (예: 같은 MCU 의 안전 제어기가 변화율을 제한)
 * 그 값을 지연보상 이력에 덧붙인다. steer_cmd 는 응답의 steer_cmd 와 같은 부호(MORAI)·단위(rad).
 * 직전 이력 값과 거의 같으면 아무것도 하지 않는다 (0). 이력에 넣었으면 1. SILS 와 안전 제어기 없는 HILS 는 부르지 않는다. */
int bmpc_controller_note_applied(bmpc_controller_t *c, int64_t now_us, bmpc_real steer_cmd);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_CONTROLLER_H */
