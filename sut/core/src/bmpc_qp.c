#include "bmpc/bmpc_qp.h"

#include "bmpc/bmpc_math.h"
#include "bmpc/bmpc_prof.h"

/* M (m x m, 행 간격 ld) 을 아래삼각 L 로 바꾼다. 실패하면 -1. */
static int cholesky(bmpc_real *M, int m, int ld)
{
  int i, j;
  for (j = 0; j < m; ++j) {
    const bmpc_real *rj = &M[j * ld];
    bmpc_real s = M[j * ld + j] - bm_dot(rj, rj, j), inv;
    if (!(s > 0)) return -1;
    s = BM_SQRT(s);
    M[j * ld + j] = s;
    inv = (bmpc_real)1.0 / s; /* 나눗셈은 곱셈보다 훨씬 느리다 */
    for (i = j + 1; i < m; ++i) M[i * ld + j] = (M[i * ld + j] - bm_dot(&M[i * ld], rj, j)) * inv;
  }
  return 0;
}

/* L L' y = b, 결과를 b 에 덮어쓴다 */
static void chol_solve(const bmpc_real *L, int m, int ld, bmpc_real *b)
{
  int i, k;
  for (i = 0; i < m; ++i) b[i] = (b[i] - bm_dot(&L[i * ld], b, i)) / L[i * ld + i];
  for (i = m - 1; i >= 0; --i) {
    bmpc_real t0 = b[i], t1 = 0, t2 = 0, t3 = 0;
    for (k = i + 1; k + 4 <= m; k += 4) {
      t0 -= L[k * ld + i] * b[k];
      t1 -= L[(k + 1) * ld + i] * b[k + 1];
      t2 -= L[(k + 2) * ld + i] * b[k + 2];
      t3 -= L[(k + 3) * ld + i] * b[k + 3];
    }
    for (; k < m; ++k) t0 -= L[k * ld + i] * b[k];
    b[i] = ((t0 + t1) + (t2 + t3)) / L[i * ld + i];
  }
}

static void gradient(int n, const bmpc_real *P, const bmpc_real *q, const bmpc_real *x, bmpc_real *g)
{
  int i;
  BMPC_PROF_BEGIN(t_grad);
  for (i = 0; i < n; ++i) g[i] = q[i] + bm_dot(&P[i * n], x, n);
  BMPC_PROF_END(BMPC_PROF_GRAD, t_grad);
}

/* 자유 변수 F 순서대로 P_FF 의 Cholesky 인수 L 을 w->M 에 (행 간격 LD) 들고 다닌다.
 * working set 이 한 변수씩 바뀔 때마다 처음부터 다시 분해(O(nf^3))하지 않고 갱신(O(nf^2))한다. */
#define LD BMPC_NZ_MAX

static int refactor(int n, const bmpc_real *P, bmpc_qp_work_t *w, int nf)
{
  int k, l;
  for (k = 0; k < nf; ++k) {
    const bmpc_real *pr = &P[w->free_idx[k] * n];
    for (l = 0; l <= k; ++l) w->M[k * LD + l] = pr[w->free_idx[l]];
  }
  return cholesky(w->M, nf, LD);
}

/* F 의 위치 p 변수를 뺀다. 아래 행·열을 한 칸씩 당긴 뒤, 남은 아래 블록에 빠진 열 l 로 l l' 를 더한다
 * (L22 L22' + l l' 의 rank-1 갱신). 실패하면 -1 (호출한 쪽이 다시 분해한다). */
static int chol_remove(bmpc_qp_work_t *w, int nf, int p)
{
  bmpc_real *L = w->M, *x = w->t;
  const int m = nf - 1 - p;
  int i, j, k;
  for (i = 0; i < m; ++i) x[i] = L[(p + 1 + i) * LD + p];
  for (i = p + 1; i < nf; ++i) {
    const bmpc_real *src = &L[i * LD];
    bmpc_real *dst = &L[(i - 1) * LD];
    for (j = 0; j < p; ++j) dst[j] = src[j];
    for (j = p + 1; j <= i; ++j) dst[j - 1] = src[j];
  }
  for (k = 0; k < m; ++k) {
    bmpc_real *lkk = &L[(p + k) * LD + (p + k)];
    const bmpc_real r = BM_SQRT(*lkk * *lkk + x[k] * x[k]);
    bmpc_real c, s, inv_c;
    if (!(r > 0) || !(*lkk > 0)) return -1;
    c = r / *lkk;
    s = x[k] / *lkk;
    inv_c = (bmpc_real)1.0 / c;
    *lkk = r;
    for (i = k + 1; i < m; ++i) {
      bmpc_real *lik = &L[(p + i) * LD + (p + k)];
      *lik = (*lik + s * x[i]) * inv_c;
      x[i] = c * x[i] - s * *lik;
    }
  }
  for (i = p; i < nf - 1; ++i) w->free_idx[i] = w->free_idx[i + 1];
  return 0;
}

/* 변수 v 를 F 끝에 붙인다: L y = P_Fv, 대각 = sqrt(P_vv - y'y). 실패하면 -1. */
static int chol_append(int n, const bmpc_real *P, bmpc_qp_work_t *w, int nf, int v)
{
  bmpc_real *L = w->M, *row = &w->M[nf * LD];
  const bmpc_real *pv = &P[v * n];
  bmpc_real dd;
  int k;
  for (k = 0; k < nf; ++k) row[k] = (pv[w->free_idx[k]] - bm_dot(&L[k * LD], row, k)) / L[k * LD + k];
  dd = pv[v] - bm_dot(row, row, nf);
  if (!(dd > 0)) return -1;
  row[nf] = BM_SQRT(dd);
  return 0;
}

static int boxqp_solve(int n, const bmpc_real *P, const bmpc_real *q, const bmpc_real *lb, const bmpc_real *ub,
                       bmpc_real *x, int max_iter, bmpc_qp_work_t *w, int *iters)
{
  int i, it, k, nf = 0;
  int sub_solved = 0;   /* 현재 working set 에서 부분 문제의 최소점에 있음 */
  int factored = 0;     /* w->M 이 지금 F 의 인수인지 */
  int last_released = -1;
  bmpc_real qmax = 0, pmax = 0, xmax = 0, lambda_tol, step_tol;

  *iters = 0;
  if (n <= 0 || n > BMPC_NZ_MAX) return BMPC_ERR_NUMERIC;
  for (i = 0; i < n; ++i) {
    if (!(lb[i] <= ub[i])) return BMPC_ERR_INFEASIBLE;
    if (!bm_isfinite(x[i])) x[i] = (bmpc_real)0.5 * (lb[i] + ub[i]);
    x[i] = bm_clamp(x[i], lb[i], ub[i]);
  }

  /* 허용오차는 숫자 타입 정밀도(BM_EPS)와 문제 크기에 맞춘다. double 과 float 에서 같은 코드가 끝나게 한다. */
  for (i = 0; i < n; ++i) {
    int j;
    qmax = bm_max(qmax, BM_FABS(q[i]));
    xmax = bm_max(xmax, bm_max(BM_FABS(lb[i]), BM_FABS(ub[i])));
    for (j = 0; j < n; ++j) pmax = bm_max(pmax, BM_FABS(P[i * n + j]));
  }
  lambda_tol = (bmpc_real)1e3 * BM_EPS * ((bmpc_real)1.0 + qmax + pmax * xmax);
  step_tol = (bmpc_real)1e2 * BM_EPS;

  /* 처음 working set: 경계에 있고 기울기가 바깥을 향하는 변수 */
  gradient(n, P, q, x, w->g);
  for (i = 0; i < n; ++i) {
    if (lb[i] == ub[i]) w->ws[i] = -1;
    else if (x[i] <= lb[i] && w->g[i] > 0) w->ws[i] = -1;
    else if (x[i] >= ub[i] && w->g[i] < 0) w->ws[i] = 1;
    else w->ws[i] = 0;
    if (w->ws[i] == 0) w->free_idx[nf++] = i;
  }

  for (it = 0; it < max_iter; ++it) {
    bmpc_real dmax = 0, xn = 0;
    *iters = it + 1;
    if (it > 0) gradient(n, P, q, x, w->g);
    for (i = 0; i < n; ++i) xn = bm_max(xn, BM_FABS(x[i]));

    if (!sub_solved && nf > 0) {
      /* 자유 변수 부분 문제: P_FF d = -g_F */
      BMPC_PROF_BEGIN(t_chol);
      if (!factored) {
        if (refactor(n, P, w, nf) != 0) return BMPC_ERR_NUMERIC;
        factored = 1;
      }
      for (k = 0; k < nf; ++k) w->d[k] = -w->g[w->free_idx[k]];
      chol_solve(w->M, nf, LD, w->d);
      BMPC_PROF_END(BMPC_PROF_CHOL, t_chol);
      for (k = 0; k < nf; ++k) {
        if (!bm_isfinite(w->d[k])) return BMPC_ERR_NUMERIC;
        dmax = bm_max(dmax, BM_FABS(w->d[k]));
      }
      if (dmax <= step_tol * ((bmpc_real)1.0 + xn)) sub_solved = 1;
    } else if (nf == 0) {
      sub_solved = 1;
    }

    if (sub_solved) {
      /* 부분 문제의 최소점: 고정 변수의 승수를 본다 (하한은 g, 상한은 -g 가 0 이상이어야 한다) */
      int worst = -1;
      bmpc_real worst_lambda = -lambda_tol;
      for (i = 0; i < n; ++i) {
        bmpc_real lambda;
        if (w->ws[i] == 0 || lb[i] == ub[i]) continue;
        lambda = (w->ws[i] < 0) ? w->g[i] : -w->g[i];
        if (lambda < worst_lambda) {
          worst_lambda = lambda;
          worst = i;
        }
      }
      if (worst < 0) return BMPC_OK;
      w->ws[worst] = 0;
      if (factored) {
        BMPC_PROF_BEGIN(t_app);
        if (chol_append(n, P, w, nf, worst) != 0) factored = 0;
        BMPC_PROF_END(BMPC_PROF_CHOL, t_app);
      }
      w->free_idx[nf++] = worst;
      last_released = worst;
      sub_solved = 0;
      continue;
    }

    /* 경계에 막히는 만큼만 간다. 막히지 않고 끝까지 가면 그 점이 부분 문제의 최소점이다. */
    {
      bmpc_real alpha = 1;
      int blk = -1, blk_pos = -1;
      signed char blk_side = 0;
      for (k = 0; k < nf; ++k) {
        const int r = w->free_idx[k];
        const bmpc_real dk = w->d[k];
        if (dk > 0 && x[r] + dk > ub[r]) {
          const bmpc_real a = (ub[r] - x[r]) / dk;
          if (a < alpha) {
            alpha = a;
            blk = r;
            blk_pos = k;
            blk_side = 1;
          }
        } else if (dk < 0 && x[r] + dk < lb[r]) {
          const bmpc_real a = (lb[r] - x[r]) / dk;
          if (a < alpha) {
            alpha = a;
            blk = r;
            blk_pos = k;
            blk_side = -1;
          }
        }
      }
      if (alpha < 0) alpha = 0;
      for (k = 0; k < nf; ++k) {
        const int r = w->free_idx[k];
        x[r] = bm_clamp(x[r] + alpha * w->d[k], lb[r], ub[r]);
      }
      if (blk >= 0) {
        /* alpha == 0 이면 x 는 그대로이고, 같은 d 로 길이 0 인 바깥 스텝을 가진 변수도 한 번에 고정한다 (풀이는 다음 반복에 한 번).
         * 방금 푼 변수는 blk 일 때만 고정한다 (아래 종료 규칙). 뒤에서부터 빼야 앞쪽 위치와 d 의 순서가 유지된다. */
        const int batch = alpha == 0 && blk != last_released;
        for (k = nf - 1; k >= 0; --k) {
          const int r = w->free_idx[k];
          signed char side = 0;
          int j;
          if (k == blk_pos) side = blk_side;
          else if (batch && r != last_released) {
            if (w->d[k] > 0 && x[r] >= ub[r] && x[r] + w->d[k] > ub[r]) side = 1;
            else if (w->d[k] < 0 && x[r] <= lb[r] && x[r] + w->d[k] < lb[r]) side = -1;
          }
          if (side == 0) continue;
          x[r] = side > 0 ? ub[r] : lb[r];
          w->ws[r] = side;
          if (factored) {
            BMPC_PROF_BEGIN(t_rem);
            if (chol_remove(w, nf, k) != 0) factored = 0;
            BMPC_PROF_END(BMPC_PROF_CHOL, t_rem);
          }
          if (!factored)
            for (j = k; j < nf - 1; ++j) w->free_idx[j] = w->free_idx[j + 1];
          --nf;
        }
        /* 방금 푼 변수가 움직이지 못하고 바로 다시 막히면 정밀도 한계: 지금 점을 해로 본다 */
        if (blk == last_released && alpha <= step_tol) return BMPC_OK;
        /* 막혔으면 "방금 푼 변수" 가 아니다. 남겨 두면 뒤의 막힘이 승수 확인 없이 위 규칙으로 끝날 수 있다 */
        last_released = -1;
      } else {
        sub_solved = 1;
      }
    }
  }
  return BMPC_ERR_MAXITER;
}

int bmpc_boxqp_solve(int n, const bmpc_real *P, const bmpc_real *q, const bmpc_real *lb,
                     const bmpc_real *ub, bmpc_real *x, int max_iter, bmpc_qp_work_t *w,
                     int *iters)
{
  int st;
  BMPC_PROF_BEGIN(t_qp);
  st = boxqp_solve(n, P, q, lb, ub, x, max_iter, w, iters);
  BMPC_PROF_END(BMPC_PROF_QP, t_qp);
  return st;
}

bmpc_real bmpc_boxqp_kkt_residual(int n, const bmpc_real *P, const bmpc_real *q,
                                  const bmpc_real *lb, const bmpc_real *ub, const bmpc_real *x)
{
  int i, j;
  bmpc_real r = 0;
  for (i = 0; i < n; ++i) {
    bmpc_real g = q[i];
    bmpc_real v;
    for (j = 0; j < n; ++j) g += P[i * n + j] * x[j];
    if (lb[i] == ub[i]) v = BM_FABS(x[i] - lb[i]);
    else if (x[i] < lb[i]) v = lb[i] - x[i];
    else if (x[i] > ub[i]) v = x[i] - ub[i];
    else if (x[i] <= lb[i]) v = bm_max((bmpc_real)0.0, -g);
    else if (x[i] >= ub[i]) v = bm_max((bmpc_real)0.0, g);
    else v = BM_FABS(g);
    r = bm_max(r, v);
  }
  return r;
}
