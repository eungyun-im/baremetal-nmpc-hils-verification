/* LTV-MPC. 원본 NMPC(mpc.cpp FG_eval)와 같은 모델·비용함수를 쓰고, IPOPT 대신
 * "궤적을 따라 선형화 -> 상태 소거(condensing) -> 상하한 QP" 를 몇 번 반복한다
 * (비용이 잔차 제곱합이라 Gauss-Newton SQP 가 된다).
 *
 * 모델 (t -> t+1, 상태 x y psi v cte epsi, 입력 delta v_target):
 *   x'    = x + v cos(psi) dt
 *   y'    = y + v sin(psi) dt
 *   psi'  = psi - v/Lf delta dt
 *   v'    = v + k_v (v_target - v) dt
 *   cte'  = (f(x) - y) + v sin(epsi) dt
 *   epsi' = (psi - atan(f'(x))) - v/Lf delta dt
 * 결정 변수 z = [delta_0 .. delta_{N-2}, vt_0 .. vt_{N-2}] (2(N-1) 개). */
#ifndef BMPC_MPC_H
#define BMPC_MPC_H

#include "bmpc/bmpc_config.h"
#include "bmpc/bmpc_qp.h"
#include "bmpc/bmpc_weights.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bmpc_real s0[BMPC_NX]; /* 지연보상 뒤 초기 상태 */
  bmpc_real c[4];        /* 경로 3차식 계수 */
  bmpc_real ref_v, v_lb, v_ub;
  bmpc_real steer_lb, steer_ub;
  bmpc_real prev_delta_ref; /* 직전 조향 (MPC 부호) */
  bmpc_weights_t w;
} bmpc_problem_t;

/* 비용 행렬에 쓰는 민감도 저장 크기: 입력 열마다 (영향을 주는 시간 수) x (v, cte, epsi). 합은 3 (N^2 - 1). */
#define BMPC_SC_MAX (3 * BMPC_N_MAX * BMPC_N_MAX)

/* 작업 메모리. MCU 의 대기 없는 RAM(DTCM 64 KB)에 들어가도록 6x6 야코비안은 저장하지 않고,
 * 비용에 쓰는 세 상태(v, cte, epsi)의 민감도만 열마다 모아 둔다 (float, N_MAX 32 에서 약 48 KB). */
typedef struct {
  bmpc_real sbar[BMPC_N_MAX][BMPC_NX];
  bmpc_real A[BMPC_NX * BMPC_NX];          /* 지금 단계의 야코비안 */
  bmpc_real B[BMPC_NX * BMPC_NU];
  bmpc_real X[BMPC_NZ_MAX][BMPC_NX];       /* 지금 상태의 입력 민감도 (열 j = 입력 j, 6 개씩 연속) */
  bmpc_real C[BMPC_SC_MAX];                /* 열 i 가 영향을 주는 시간마다 (v, cte, epsi) 민감도, 시간 순서 */
  int c_off[BMPC_NZ_MAX];                  /* 열 i 의 C 시작 위치 */
  int c_t0[BMPC_NZ_MAX];                   /* 열 i 의 첫 시간 */
  bmpc_real res[BMPC_N_MAX][3];            /* 시간 t 의 잔차 (v - vt, cte, epsi) */
  bmpc_real P[BMPC_NZ_MAX * BMPC_NZ_MAX];
  bmpc_real q[BMPC_NZ_MAX];
  bmpc_real lb[BMPC_NZ_MAX], ub[BMPC_NZ_MAX];
  bmpc_real dz[BMPC_NZ_MAX];
  bmpc_real ztry[BMPC_NZ_MAX];
  bmpc_qp_work_t qp;
} bmpc_mpc_work_t;

typedef struct {
  bmpc_real z[BMPC_NZ_MAX]; /* 해 */
  int nz;
  int status;    /* BMPC_OK 또는 마지막 QP 의 오류 */
  int sqp_iters; /* 선형화 횟수 */
  int qp_iters;  /* QP 반복 합 */
  bmpc_real cost;
} bmpc_mpc_result_t;

/* z_init: 선형화 시작점(warm start). NULL 이면 (직전 조향, 현재 속도)로 시작한다. */
int bmpc_mpc_solve(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *z_init,
                   bmpc_mpc_work_t *w, bmpc_mpc_result_t *res);

/* 원본 FG_eval 과 같은 비선형 비용 (시험·진단용). */
bmpc_real bmpc_mpc_cost(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *z,
                        bmpc_real sbar[][BMPC_NX]);

/* z 로 비선형 모델을 굴린 상태 궤적 sbar[0..N-1] */
void bmpc_mpc_rollout(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *z,
                      bmpc_real sbar[][BMPC_NX]);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_MPC_H */
