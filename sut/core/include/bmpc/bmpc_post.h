/* 조향·종방향 후처리. 원본 mpc_path_tracking.h 의 makeSteeringCommand(),
 * setThrottleCommand(), publishControlCommand() 의 속도 가드를 그대로 옮겼다. */
#ifndef BMPC_POST_H
#define BMPC_POST_H

#include "bmpc/bmpc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bmpc_real prev_steer_cmd; /* 직전 최종 조향 (MORAI 부호) */
  int have_prev;
  bmpc_real integral;
  bmpc_real ff_prev_target; /* 원본 초기값 -1 */
  bmpc_real ff_target_accel;
  bmpc_real last_brake;
} bmpc_post_state_t;

void bmpc_post_init(bmpc_post_state_t *st);

/* makeSteeringCommand(): 부호 -> 범위 -> 변화율 -> 속도별 LPF -> 범위. MORAI 부호 명령을 돌려준다.
 * (이력 기록과 계산 지연 측정은 controller 가 한다) */
bmpc_real bmpc_steer_post(const bmpc_config_t *cfg, bmpc_post_state_t *st, bmpc_real mpc_delta,
                          bmpc_real speed);

/* setThrottleCommand(). steering_angle 은 MPC 원출력 조향(피드포워드 조향 테이퍼용). */
void bmpc_throttle(const bmpc_config_t *cfg, bmpc_post_state_t *st, bmpc_real target_speed,
                   bmpc_real actual_speed, bmpc_real dt, bmpc_real brake_limit,
                   bmpc_real accel_limit, bmpc_real steering_angle, bmpc_real *accel,
                   bmpc_real *brake);

/* publishControlCommand() 의 규정 속도 가드 (예외 구간 밖에서만 부른다) */
void bmpc_legal_guard(const bmpc_config_t *cfg, bmpc_post_state_t *st, bmpc_real actual_speed,
                      bmpc_real *accel, bmpc_real *brake);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_POST_H */
