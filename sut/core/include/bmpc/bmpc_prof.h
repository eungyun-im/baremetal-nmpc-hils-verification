/* 구간별 시간 측정 지점. 기본 빌드에서는 아무것도 하지 않는다.
 * MCU 펌웨어를 BMPC_PROFILE 로 빌드하면 플랫폼이 bmpc_prof_now() (사이클 카운터)와 bmpc_prof[] 를 주고,
 * 각 구간의 사이클이 bmpc_prof[구간] 에 누적된다. */
#ifndef BMPC_PROF_H
#define BMPC_PROF_H

#include <stdint.h>

enum {
  BMPC_PROF_BUILD = 0, /* build_qp 전체 (선형화, 민감도, 비용 행렬) */
  BMPC_PROF_PROP,      /* 그중 야코비안 + 민감도 전파 */
  BMPC_PROF_TERM,      /* 그중 비용 항 누적 (add_term) */
  BMPC_PROF_COST,      /* 비선형 비용 (rollout, 처음 + 선 탐색) */
  BMPC_PROF_QP,        /* bmpc_boxqp_solve 전체 */
  BMPC_PROF_CHOL,      /* 그중 Cholesky 분해 + 풀이 */
  BMPC_PROF_GRAD,      /* 그중 기울기 */
  BMPC_PROF_STEP,      /* controller 한 주기 전체 */
  BMPC_PROF_COUNT
};

#ifdef BMPC_PROFILE
#ifdef __cplusplus
extern "C" {
#endif
uint32_t bmpc_prof_now(void);
extern volatile uint32_t bmpc_prof[BMPC_PROF_COUNT];
#ifdef __cplusplus
}
#endif
#define BMPC_PROF_BEGIN(v) const uint32_t v = bmpc_prof_now()
#define BMPC_PROF_END(i, v) (bmpc_prof[(i)] += bmpc_prof_now() - (v))
#else
#define BMPC_PROF_BEGIN(v) ((void)0)
#define BMPC_PROF_END(i, v) ((void)0)
#endif

#endif /* BMPC_PROF_H */
