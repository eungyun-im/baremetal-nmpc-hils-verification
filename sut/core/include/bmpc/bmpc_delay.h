/* 지연보상 v2. 원본 mpc_path_tracking 의 steering_delay.h 와 modelPredictiveController()
 * 지연보상 블록을 그대로 옮겼다. 시각은 MCU 타이머(us)를 쓰고, 적분은 예측 시각 기준
 * 상대 시간(s)으로 한다. */
#ifndef BMPC_DELAY_H
#define BMPC_DELAY_H

#include "bmpc/bmpc_config.h"
#include "bmpc/bmpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BMPC_HIST_MAX
#define BMPC_HIST_MAX 64 /* 1 s @ 20 Hz 면 20 개면 되지만 여유를 둔다 */
#endif

typedef struct {
  bmpc_real x, y, yaw;
} bmpc_pose2_t;

/* 명령 이력 (MPC 부호 조향). 오래된 것부터 저장하는 링버퍼. */
typedef struct {
  int64_t t_us[BMPC_HIST_MAX];
  bmpc_real delta[BMPC_HIST_MAX];
  int head;  /* 가장 오래된 항목 위치 */
  int count;
} bmpc_hist_t;

void bmpc_hist_clear(bmpc_hist_t *h);
void bmpc_hist_push(bmpc_hist_t *h, int64_t t_us, bmpc_real delta);
void bmpc_hist_pop_front(bmpc_hist_t *h);
int64_t bmpc_hist_time(const bmpc_hist_t *h, int i); /* i = 0 이 가장 오래된 항목 */
bmpc_real bmpc_hist_delta(const bmpc_hist_t *h, int i);

/* steering_delay.h measuredMotion(): 측정 속도·yaw rate 로 seconds 동안 이동한 자세 */
bmpc_pose2_t bmpc_measured_motion(bmpc_real speed, bmpc_real yaw_rate, bmpc_real seconds);

/* steering_delay.h predictSteeringDelay(). now_rel, 이력 시각은 t_ref_us 기준 상대 시간(s). */
bmpc_pose2_t bmpc_predict_steering_delay(bmpc_real speed, bmpc_real lf, bmpc_real now_rel,
                                         bmpc_real delay, const bmpc_hist_t *hist,
                                         int64_t t_ref_us);

/* steering_delay.h compose(): b 를 a 좌표계에서 표현 */
bmpc_pose2_t bmpc_compose(bmpc_pose2_t a, bmpc_pose2_t b);

/* steering_delay.h trackingErrorsAt(): cte = f(x) - y, epsi = psi - atan(f'(x)) */
void bmpc_tracking_errors_at(bmpc_pose2_t pose, const bmpc_real c[4], bmpc_real out[2]);

typedef struct {
  bmpc_real state_delay; /* ① (s) */
  bmpc_real compute;     /* ② (s) */
  bmpc_real actuation;   /* ③ (s) */
  bmpc_real lf;          /* 이력 적분에 쓴 실효 Lf */
} bmpc_dc_info_t;

/* 지연보상 v2 블록. t3 시점 자세를 예측해 MPC 초기 상태 s0[6] 를 만든다.
 *   period      센서 주기 T (PC 가 fallback·범위 처리를 끝낸 값)
 *   odom_age    PC 가 보낼 때의 상태 나이
 *   transport_req  요청 전송 시간 (①에 더한다)
 *   compute     ② (MCU 처리 EMA + 응답 전송)
 *   lf_est      실효 Lf 추정 (<= 0 = 없음)
 * 이력은 호출 전에 bmpc_hist_expire_check() 로 정리해 둔다. */
void bmpc_delay_comp(const bmpc_config_t *cfg, const bmpc_hist_t *hist, int64_t now_us,
                     bmpc_real velocity, bmpc_real yaw_rate, bmpc_real period,
                     bmpc_real odom_age, bmpc_real transport_req, bmpc_real compute,
                     bmpc_real lf_est, const bmpc_real c[4], bmpc_real s0[BMPC_NX],
                     bmpc_dc_info_t *info);

/* 원본: 이력이 있는데 시계가 역행했거나 마지막 항목이 1 s 넘게 지났으면 비운다 */
void bmpc_hist_expire_check(bmpc_hist_t *h, int64_t now_us);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_DELAY_H */
