/* 공용 타입과 크기 상한. MCU 와 PC 가 같은 정의를 쓴다. */
#ifndef BMPC_TYPES_H
#define BMPC_TYPES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 실수 타입. MCU FPU 가 double 을 지원하지 않으면 BMPC_USE_FLOAT 로 빌드한다. */
#ifdef BMPC_USE_FLOAT
typedef float bmpc_real;
#else
typedef double bmpc_real;
#endif

/* 예측 구간 상한. 작업 메모리가 N^2 에 비례하므로 MCU RAM 에 맞춰 둔다. */
#ifndef BMPC_N_MAX
#define BMPC_N_MAX 32
#endif

#define BMPC_NX 6 /* 상태: x, y, psi, v, cte, epsi */
#define BMPC_NU 2 /* 입력: delta, v_target */
#define BMPC_NZ_MAX (BMPC_NU * (BMPC_N_MAX - 1))

enum { BMPC_SX = 0, BMPC_SY, BMPC_SPSI, BMPC_SV, BMPC_SCTE, BMPC_SEPSI };

/* 결과 상태. CAN 응답의 status 바이트로 그대로 나간다. */
enum {
  BMPC_OK = 0,
  BMPC_ERR_MAXITER = 1,     /* QP 반복 상한: 그때까지의 최선 해 */
  BMPC_ERR_INFEASIBLE = 2,  /* 상하한이 모순 */
  BMPC_ERR_NUMERIC = 3,     /* NaN, Cholesky 실패 */
  BMPC_ERR_RX = 4,          /* 수신 오류(프레임 누락, CRC) */
  BMPC_ERR_NOT_CONFIGURED = 5
};

#ifdef __cplusplus
}
#endif

#endif /* BMPC_TYPES_H */
