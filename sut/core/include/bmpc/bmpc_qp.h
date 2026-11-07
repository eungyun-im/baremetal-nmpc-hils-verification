/* 상하한 제약만 있는 볼록 QP:  min 1/2 x'Px + q'x,  lb <= x <= ub  (P 대칭 양정치).
 *
 * primal active-set: 고정된 변수 집합(working set)을 두고 나머지로 Newton 스텝을 푼다.
 * 스텝이 경계에 막히면 그 변수를 고정하고, 스텝이 0 이면 라그랑주 승수가 가장 음수인
 * 변수 하나를 푼다. P 가 양정치면 유한 번 안에 끝난다. 동적 메모리는 쓰지 않는다. */
#ifndef BMPC_QP_H
#define BMPC_QP_H

#include "bmpc/bmpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bmpc_real g[BMPC_NZ_MAX];                 /* 기울기 */
  bmpc_real d[BMPC_NZ_MAX];                 /* 스텝 */
  bmpc_real M[BMPC_NZ_MAX * BMPC_NZ_MAX];   /* P_FF Cholesky (자유 변수 순서, 행 간격 BMPC_NZ_MAX) */
  bmpc_real t[BMPC_NZ_MAX];                 /* 인수 갱신용 */
  int free_idx[BMPC_NZ_MAX];                /* 자유 변수 F (인수의 행 순서) */
  signed char ws[BMPC_NZ_MAX];              /* 0 = 자유, -1 = 하한 고정, +1 = 상한 고정 */
} bmpc_qp_work_t;

/* x: 들어올 때 초기값, 나갈 때 해. 반환값 BMPC_OK / BMPC_ERR_MAXITER / BMPC_ERR_NUMERIC /
 * BMPC_ERR_INFEASIBLE. iters 에 반복 횟수를 쓴다. P 는 n x n 행 우선. */
int bmpc_boxqp_solve(int n, const bmpc_real *P, const bmpc_real *q, const bmpc_real *lb,
                     const bmpc_real *ub, bmpc_real *x, int max_iter, bmpc_qp_work_t *w,
                     int *iters);

/* 시험용: KKT 위반 정도. 자유 변수의 |기울기|, 하한 변수의 max(0,-g), 상한 변수의 max(0,g),
 * 범위 위반 중 최댓값. */
bmpc_real bmpc_boxqp_kkt_residual(int n, const bmpc_real *P, const bmpc_real *q,
                                  const bmpc_real *lb, const bmpc_real *ub, const bmpc_real *x);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_QP_H */
