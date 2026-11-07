/* bmpc_real 에 맞는 수학 함수. 원본 C++ 의 clampValue/wrapToPi 와 같은 동작이다. */
#ifndef BMPC_MATH_H
#define BMPC_MATH_H

#include <float.h>
#include <math.h>

#include "bmpc/bmpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BM_PI 3.14159265358979323846

#ifdef BMPC_USE_FLOAT
#define BM_EPS ((bmpc_real)FLT_EPSILON)
#define BM_SIN sinf
#define BM_COS cosf
#define BM_ATAN atanf
#define BM_SQRT sqrtf
#define BM_FABS fabsf
#define BM_EXP expf
#define BM_POW powf
#else
#define BM_EPS ((bmpc_real)DBL_EPSILON)
#define BM_SIN sin
#define BM_COS cos
#define BM_ATAN atan
#define BM_SQRT sqrt
#define BM_FABS fabs
#define BM_EXP exp
#define BM_POW pow
#endif

/* 내적. 누산기 4 개로 나눠 FPU 의 곱셈-덧셈 지연이 줄줄이 이어지지 않게 한다 (Cortex-M7 에서 약 3 배). */
static inline bmpc_real bm_dot(const bmpc_real *a, const bmpc_real *b, int n)
{
  bmpc_real s0 = 0, s1 = 0, s2 = 0, s3 = 0;
  int k = 0;
  for (; k + 4 <= n; k += 4) {
    s0 += a[k] * b[k];
    s1 += a[k + 1] * b[k + 1];
    s2 += a[k + 2] * b[k + 2];
    s3 += a[k + 3] * b[k + 3];
  }
  for (; k < n; ++k) s0 += a[k] * b[k];
  return (s0 + s1) + (s2 + s3);
}

static inline bmpc_real bm_min(bmpc_real a, bmpc_real b) { return b < a ? b : a; }
static inline bmpc_real bm_max(bmpc_real a, bmpc_real b) { return a < b ? b : a; }

/* 원본 clampValue = max(lower, min(value, upper)). lower > upper 면 lower 를 돌려준다. */
static inline bmpc_real bm_clamp(bmpc_real v, bmpc_real lo, bmpc_real hi)
{
  return bm_max(lo, bm_min(v, hi));
}

static inline bmpc_real bm_wrap_pi(bmpc_real a)
{
  while (a > (bmpc_real)BM_PI) a -= (bmpc_real)(2.0 * BM_PI);
  while (a < (bmpc_real)-BM_PI) a += (bmpc_real)(2.0 * BM_PI);
  return a;
}

static inline int bm_isfinite(bmpc_real v) { return isfinite(v) ? 1 : 0; }

#ifdef __cplusplus
}
#endif

#endif /* BMPC_MATH_H */
