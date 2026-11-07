/* MCU 설정값. 시작할 때 PC 가 CAN 설정 프레임(0x0F0)으로 하나씩 보내고 해시로 확인한다.
 *
 * 필드 목록은 X 매크로 하나에만 적는다: (번호, 이름, 타입, 기본값).
 *   타입 I = int32_t, R = bmpc_real.
 *   번호는 CAN 설정 프레임의 index 라서 바꾸면 PC 와 MCU 를 같이 바꿔야 한다.
 *   기본값은 2026-10-29 mpc_params.yaml 값이다(브리지는 rosparam 값을 보낸다).
 */
#ifndef BMPC_CONFIG_H
#define BMPC_CONFIG_H

#include "bmpc/bmpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMPC_CONFIG_FIELDS(X)                                                              \
  /* 모델과 예측 구간 */                                                                  \
  X(0, N, I, 30)                                                                           \
  X(1, dt, R, 0.05)                                                                        \
  X(2, Lf, R, 4.0)                                                                         \
  X(3, k_v, R, 0.4)                                                                        \
  X(4, steer_lb, R, -0.6981)                                                               \
  X(5, steer_ub, R, 0.6981)                                                                \
  X(6, steer_sign, R, -1.0) /* MPC 부호 -> MORAI 부호 */                                   \
  /* 고정 가중치 (cost_weight/...) */                                                      \
  X(7, w_v, R, 20.0)                                                                       \
  X(8, w_delta_start, R, 30000.0)                                                          \
  X(9, w_vt, R, 300.0)                                                                     \
  X(10, w_dvt, R, 100.0)                                                                   \
  X(11, steer_scale, R, 0.40)                                                              \
  X(12, steer_change_scale, R, 0.40)                                                       \
  /* 도심 세트 (urban_control/...) */                                                      \
  X(13, u_cte_start, R, 5000.0)                                                            \
  X(14, u_cte_urban, R, 750.0)                                                             \
  X(15, u_epsi, R, 80000.0)                                                                \
  X(16, u_delta_start, R, 2500.0)                                                          \
  X(17, u_delta_high, R, 60000.0)                                                          \
  X(18, u_speed_start, R, 8.5)                                                             \
  X(19, u_speed_full, R, 11.5)                                                             \
  X(20, u_ddelta_start, R, 350000.0)                                                       \
  X(21, u_ddelta_urban, R, 1000000.0)                                                      \
  /* 고속 구간 세트 (highway_control/...) */                                               \
  X(22, h_cte, R, 1000.0)                                                                  \
  X(23, h_epsi, R, 10500.0)                                                                \
  X(24, h_delta, R, 52000.0)                                                               \
  X(25, h_ddelta, R, 750000.0)                                                             \
  /* 회피 세트 (avoidance_control/...) */                                                  \
  X(26, a_cte, R, 4000.0)                                                                  \
  X(27, a_epsi, R, 4000.0)                                                                 \
  X(28, a_delta, R, 12000.0)                                                               \
  X(29, a_ddelta, R, 350000.0)                                                             \
  /* 지연보상 v2 (delay_comp/...) */                                                       \
  X(30, dc_enabled, I, 1)                                                                  \
  X(31, dc_state_k, R, 1.0)                                                                \
  X(32, dc_state_c, R, -0.004)                                                             \
  X(33, dc_state_max, R, 0.12)                                                             \
  X(34, dc_act_base, R, 0.06)                                                              \
  X(35, dc_act_frame_k, R, 0.5)                                                            \
  X(36, dc_model_lf, R, 4.0)                                                               \
  X(37, dc_lf_online, I, 1)                                                                \
  X(38, dc_compute_max, R, 0.10)                                                           \
  X(39, dc_compute_init, R, 0.03)                                                          \
  X(40, dc_lf_min, R, 3.5)                                                                 \
  X(41, dc_lf_max, R, 5.0)                                                                 \
  X(42, dc_transport_req_share, R, 0.6) /* 전송 시간 중 요청 쪽 비율(프레임 수 6:4) */     \
  /* 조향 후처리 (steer/...) */                                                            \
  X(43, st_rate_limit, R, 1.8)                                                             \
  X(44, st_lpf_enabled, I, 1)                                                              \
  X(45, st_lpf_alpha, R, 0.65)                                                             \
  X(46, st_lpf_alpha_high, R, 0.72)                                                        \
  X(47, st_lpf_v_low, R, 11.0)                                                             \
  X(48, st_lpf_v_high, R, 25.0)                                                            \
  /* 종방향 (longitudinal/...) */                                                          \
  X(49, lg_kp, R, 0.22)                                                                    \
  X(50, lg_ki, R, 0.02)                                                                    \
  X(51, lg_int_limit, R, 6.0)                                                              \
  X(52, lg_int_band, R, 1.0)                                                               \
  X(53, lg_accel_limit, R, 1.0)                                                            \
  X(54, lg_accel_limit_exempt, R, 1.0)                                                     \
  X(55, lg_brake_limit, R, 0.50)                                                           \
  X(56, lg_kick_brake_limit, R, 1.0)                                                       \
  X(57, lg_brake_rate, R, 2.0)                                                             \
  X(58, lg_deadband, R, 0.04)                                                              \
  X(59, lg_ff_gain, R, 1.0)                                                                \
  X(60, lg_ff_accel_per_pedal, R, 3.5)                                                     \
  X(61, lg_ff_tau, R, 0.3)                                                                 \
  X(62, lg_ff_min_accel, R, 0.6)                                                           \
  X(63, lg_ff_taper, R, 1.0)                                                               \
  X(64, lg_ff_steer_zero, R, 0.20)                                                         \
  X(65, legal_enabled, I, 1)                                                               \
  X(66, legal_cut, R, 16.2)                                                                \
  X(67, legal_brake_at, R, 16.4)                                                           \
  X(68, legal_brake, R, 0.3)                                                               \
  /* 풀이기 */                                                                             \
  X(69, sqp_iters, I, 3)                                                                   \
  X(70, qp_max_iters, I, 200)                                                              \
  X(71, sqp_tol, R, 1e-4)                                                                   \
  /* 시험용: > 0 이면 ② 계산 지연을 실측 대신 이 값(s)으로 고정 (원본 IPOPT 계산 시간 재현 등) */ \
  X(72, dc_compute_fixed, R, 0.0)

#define BMPC_CFG_CTYPE_I int32_t
#define BMPC_CFG_CTYPE_R bmpc_real
#define BMPC_CFG_DECL(idx, name, type, def) BMPC_CFG_CTYPE_##type name;

typedef struct {
  BMPC_CONFIG_FIELDS(BMPC_CFG_DECL)
} bmpc_config_t;

#define BMPC_CFG_COUNT_ONE(idx, name, type, def) +1
enum { BMPC_CONFIG_COUNT = 0 BMPC_CONFIG_FIELDS(BMPC_CFG_COUNT_ONE) };

void bmpc_config_default(bmpc_config_t *cfg);

/* CAN 으로는 float32 로 오간다. 범위 밖 index 면 -1. */
int bmpc_config_set(bmpc_config_t *cfg, uint16_t idx, float value);
float bmpc_config_get(const bmpc_config_t *cfg, uint16_t idx);
const char *bmpc_config_name(uint16_t idx);

/* (index, float32 값) 순서의 FNV-1a 32. PC 와 MCU 가 같은 값을 받았는지 확인한다. */
uint32_t bmpc_config_hash(const bmpc_config_t *cfg);

/* 원본 C++ 가 파라미터를 읽을 때 걸던 하한·상한을 똑같이 건다. */
void bmpc_config_sanitize(bmpc_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_CONFIG_H */
