#include "bmpc/bmpc_mpc.h"

#include <string.h>

#include "bmpc/bmpc_math.h"
#include "bmpc/bmpc_prof.h"

/* 경로 3차식 f(x), f'(x), f''(x) */
static bmpc_real poly_f(const bmpc_real c[4], bmpc_real x) { return c[0] + x * (c[1] + x * (c[2] + x * c[3])); }
static bmpc_real poly_df(const bmpc_real c[4], bmpc_real x)
{
  return c[1] + x * ((bmpc_real)2.0 * c[2] + x * (bmpc_real)3.0 * c[3]);
}
static bmpc_real poly_ddf(const bmpc_real c[4], bmpc_real x) { return (bmpc_real)2.0 * c[2] + (bmpc_real)6.0 * c[3] * x; }

static void model_step(const bmpc_config_t *cfg, const bmpc_real c[4], const bmpc_real s[BMPC_NX],
                       bmpc_real delta, bmpc_real vt, bmpc_real out[BMPC_NX])
{
  const bmpc_real dt = cfg->dt, Lf = cfg->Lf;
  const bmpc_real x = s[BMPC_SX], y = s[BMPC_SY], psi = s[BMPC_SPSI], v = s[BMPC_SV];
  const bmpc_real epsi = s[BMPC_SEPSI];
  const bmpc_real f0 = poly_f(c, x);
  const bmpc_real psi_des = BM_ATAN(poly_df(c, x));
  out[BMPC_SX] = x + v * BM_COS(psi) * dt;
  out[BMPC_SY] = y + v * BM_SIN(psi) * dt;
  out[BMPC_SPSI] = psi - v / Lf * delta * dt;
  out[BMPC_SV] = v + cfg->k_v * (vt - v) * dt;
  out[BMPC_SCTE] = (f0 - y) + v * BM_SIN(epsi) * dt;
  out[BMPC_SEPSI] = (psi - psi_des) - v / Lf * delta * dt;
}

/* A (6x6 행 우선), B (6x2) */
static void model_jacobian(const bmpc_config_t *cfg, const bmpc_real c[4], const bmpc_real s[BMPC_NX],
                           bmpc_real delta, bmpc_real *A, bmpc_real *B)
{
  const bmpc_real dt = cfg->dt, Lf = cfg->Lf;
  const bmpc_real x = s[BMPC_SX], psi = s[BMPC_SPSI], v = s[BMPC_SV], epsi = s[BMPC_SEPSI];
  const bmpc_real df = poly_df(c, x);
  const bmpc_real dpsi_des = poly_ddf(c, x) / ((bmpc_real)1.0 + df * df);
  memset(A, 0, sizeof(bmpc_real) * BMPC_NX * BMPC_NX);
  memset(B, 0, sizeof(bmpc_real) * BMPC_NX * BMPC_NU);
#define AA(r, k) A[(r)*BMPC_NX + (k)]
#define BB(r, k) B[(r)*BMPC_NU + (k)]
  AA(BMPC_SX, BMPC_SX) = 1;
  AA(BMPC_SX, BMPC_SPSI) = -v * BM_SIN(psi) * dt;
  AA(BMPC_SX, BMPC_SV) = BM_COS(psi) * dt;
  AA(BMPC_SY, BMPC_SY) = 1;
  AA(BMPC_SY, BMPC_SPSI) = v * BM_COS(psi) * dt;
  AA(BMPC_SY, BMPC_SV) = BM_SIN(psi) * dt;
  AA(BMPC_SPSI, BMPC_SPSI) = 1;
  AA(BMPC_SPSI, BMPC_SV) = -delta / Lf * dt;
  AA(BMPC_SV, BMPC_SV) = 1 - cfg->k_v * dt;
  AA(BMPC_SCTE, BMPC_SX) = df;
  AA(BMPC_SCTE, BMPC_SY) = -1;
  AA(BMPC_SCTE, BMPC_SV) = BM_SIN(epsi) * dt;
  AA(BMPC_SCTE, BMPC_SEPSI) = v * BM_COS(epsi) * dt;
  AA(BMPC_SEPSI, BMPC_SX) = -dpsi_des;
  AA(BMPC_SEPSI, BMPC_SPSI) = 1;
  AA(BMPC_SEPSI, BMPC_SV) = -delta / Lf * dt;
  BB(BMPC_SPSI, 0) = -v / Lf * dt;
  BB(BMPC_SEPSI, 0) = -v / Lf * dt;
  BB(BMPC_SV, 1) = cfg->k_v * dt;
#undef AA
#undef BB
}

void bmpc_mpc_rollout(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *z,
                      bmpc_real sbar[][BMPC_NX])
{
  const int N = cfg->N, nu = N - 1;
  int t;
  memcpy(sbar[0], pr->s0, sizeof(bmpc_real) * BMPC_NX);
  for (t = 0; t < N - 1; ++t) model_step(cfg, pr->c, sbar[t], z[t], z[nu + t], sbar[t + 1]);
}

bmpc_real bmpc_mpc_cost(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *z,
                        bmpc_real sbar[][BMPC_NX])
{
  const int N = cfg->N, nu = N - 1;
  const bmpc_weights_t *w = &pr->w;
  bmpc_real J, d;
  int t;
  BMPC_PROF_BEGIN(t_cost);
  bmpc_mpc_rollout(cfg, pr, z, sbar);
  d = z[0] - pr->prev_delta_ref;
  J = w->delta_start * d * d;
  for (t = 0; t < N; ++t) {
    const int iv = t < N - 1 ? t : N - 2;
    const bmpc_real ev = sbar[t][BMPC_SV] - z[nu + iv];
    J += w->cte * sbar[t][BMPC_SCTE] * sbar[t][BMPC_SCTE];
    J += w->epsi * sbar[t][BMPC_SEPSI] * sbar[t][BMPC_SEPSI];
    J += w->v * ev * ev;
  }
  for (t = 0; t < N - 1; ++t) {
    const bmpc_real evt = z[nu + t] - pr->ref_v;
    J += w->delta * z[t] * z[t];
    J += w->vt * evt * evt;
  }
  for (t = 0; t < N - 2; ++t) {
    const bmpc_real dd = z[t + 1] - z[t];
    const bmpc_real dv = z[nu + t + 1] - z[nu + t];
    J += w->ddelta * dd * dd;
    J += w->dvt * dv * dv;
  }
  BMPC_PROF_END(BMPC_PROF_COST, t_cost);
  return J;
}

static void add_one(bmpc_real *P, bmpc_real *q, int n, bmpc_real wt, bmpc_real a, int i)
{
  P[i * n + i] += wt;
  q[i] += wt * a;
}

static void add_diff(bmpc_real *P, bmpc_real *q, int n, bmpc_real wt, bmpc_real a, int i, int j)
{
  /* (z_j - z_i) */
  P[i * n + i] += wt;
  P[j * n + j] += wt;
  P[i * n + j] -= wt;
  P[j * n + i] -= wt;
  q[i] -= wt * a;
  q[j] += wt * a;
}

/* 선형화와 condensing. 편차 dz 에 대한 min 1/2 dz'P dz + q'dz 를 만든다. */
static void build_qp(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *zbar,
                     bmpc_mpc_work_t *w)
{
  const int N = cfg->N, nu = N - 1, n = 2 * nu;
  const bmpc_weights_t *wt = &pr->w;
  int t, k, r, j;
  BMPC_PROF_BEGIN(t_build);

  bmpc_mpc_rollout(cfg, pr, zbar, w->sbar);

  memset(w->P, 0, sizeof(bmpc_real) * (size_t)(n * n));
  memset(w->q, 0, sizeof(bmpc_real) * (size_t)n);

  /* 첫 조향과 직전 조향의 연속성 */
  add_one(w->P, w->q, n, wt->delta_start, zbar[0] - pr->prev_delta_ref, 0);

  /* 비용 항 cte_t, epsi_t (t >= 1) 와 (v_t - vt_{min(t,N-2)}) (t >= 0) 의 입력 민감도를 열마다 모은다.
   * 열 i: 조향 k (i = k) 는 시간 k+1..N-1, 속도 k (i = nu+k) 는 시간 k..N-1 에 영향을 준다
   * (속도 비용에서 vt_k 자신이 -1 로 들어간다). 시간마다 (v, cte, epsi) 3 개. */
  {
    int i, off = 0;
    for (i = 0; i < n; ++i) {
      w->c_t0[i] = i < nu ? i + 1 : i - nu;
      w->c_off[i] = off;
      off += 3 * (N - w->c_t0[i]);
    }
    memset(w->C, 0, sizeof(bmpc_real) * (size_t)off);
    for (k = 0; k < nu; ++k) w->C[w->c_off[nu + k]] = -1; /* (v_k - vt_k) 의 vt_k */
  }
  for (t = 0; t < N; ++t) {
    const int iv = t < N - 1 ? t : N - 2;
    w->res[t][0] = w->sbar[t][BMPC_SV] - zbar[nu + iv];
    w->res[t][1] = w->sbar[t][BMPC_SCTE];
    w->res[t][2] = w->sbar[t][BMPC_SEPSI];
  }

  /* 상태 t+1 의 입력 민감도: X <- A_t X, 입력 t 열 = B_t. 상태는 입력 0..t 에만 의존한다. */
  memset(w->X, 0, sizeof(w->X));
  for (t = 0; t < N - 1; ++t) {
    BMPC_PROF_BEGIN(t_prop);
    model_jacobian(cfg, pr->c, w->sbar[t], zbar[t], w->A, w->B);
    for (k = 0; k < 2 * t; ++k) {
      bmpc_real *x = w->X[k < t ? k : nu + (k - t)];
      const bmpc_real x0 = x[0], x1 = x[1], x2 = x[2], x3 = x[3], x4 = x[4], x5 = x[5];
      const bmpc_real *A = w->A;
      for (r = 0; r < BMPC_NX; ++r, A += BMPC_NX)
        x[r] = A[0] * x0 + A[1] * x1 + A[2] * x2 + A[3] * x3 + A[4] * x4 + A[5] * x5;
    }
    for (r = 0; r < BMPC_NX; ++r) {
      w->X[t][r] = w->B[r * BMPC_NU + 0];
      w->X[nu + t][r] = w->B[r * BMPC_NU + 1];
    }
    for (k = 0; k < 2 * (t + 1); ++k) {
      const int i = k <= t ? k : nu + (k - t - 1);
      bmpc_real *c = &w->C[w->c_off[i] + 3 * (t + 1 - w->c_t0[i])];
      c[0] = w->X[i][BMPC_SV];
      c[1] = w->X[i][BMPC_SCTE];
      c[2] = w->X[i][BMPC_SEPSI];
    }
    BMPC_PROF_END(BMPC_PROF_PROP, t_prop);
  }
  w->C[w->c_off[nu + N - 2] + 3] -= 1; /* 마지막 v 비용은 vt_{N-2} 를 한 번 더 쓴다 */

  /* P = sum w c c' (아래 삼각), q = sum w a c. 원소마다 레지스터에서 내적하고 한 번만 쓴다. */
  {
    const bmpc_real wr[3] = {wt->v, wt->cte, wt->epsi};
    int i;
    BMPC_PROF_BEGIN(t_term);
    for (i = 0; i < n; ++i) {
      const int ti = w->c_t0[i], len = 3 * (N - ti);
      const bmpc_real *ci = &w->C[w->c_off[i]];
      bmpc_real wc[3 * BMPC_N_MAX];
      bmpc_real *row = &w->P[i * n];
      for (k = 0; k < len; ++k) wc[k] = wr[k % 3] * ci[k];
      w->q[i] += bm_dot(wc, &w->res[ti][0], len);
      for (j = 0; j <= i; ++j) {
        const int tj = w->c_t0[j], t0 = ti > tj ? ti : tj;
        row[j] += bm_dot(&wc[3 * (t0 - ti)], &w->C[w->c_off[j] + 3 * (t0 - tj)], 3 * (N - t0));
      }
    }
    BMPC_PROF_END(BMPC_PROF_TERM, t_term);
  }
  for (t = 0; t < N - 1; ++t) {
    add_one(w->P, w->q, n, wt->delta, zbar[t], t);
    add_one(w->P, w->q, n, wt->vt, zbar[nu + t] - pr->ref_v, nu + t);
  }
  for (t = 0; t < N - 2; ++t) {
    add_diff(w->P, w->q, n, wt->ddelta, zbar[t + 1] - zbar[t], t, t + 1);
    add_diff(w->P, w->q, n, wt->dvt, zbar[nu + t + 1] - zbar[nu + t], nu + t, nu + t + 1);
  }
  /* 아래 삼각을 위로 복사 (P 대칭) */
  for (r = 0; r < n; ++r)
    for (j = r + 1; j < n; ++j) w->P[r * n + j] = w->P[j * n + r];
  BMPC_PROF_END(BMPC_PROF_BUILD, t_build);

  for (k = 0; k < nu; ++k) {
    w->lb[k] = pr->steer_lb - zbar[k];
    w->ub[k] = pr->steer_ub - zbar[k];
    w->lb[nu + k] = pr->v_lb - zbar[nu + k];
    w->ub[nu + k] = pr->v_ub - zbar[nu + k];
  }
}

static void clamp_z(const bmpc_problem_t *pr, int nu, bmpc_real *z)
{
  int k;
  for (k = 0; k < nu; ++k) {
    z[k] = bm_clamp(z[k], pr->steer_lb, pr->steer_ub);
    z[nu + k] = bm_clamp(z[nu + k], pr->v_lb, pr->v_ub);
  }
}

int bmpc_mpc_solve(const bmpc_config_t *cfg, const bmpc_problem_t *pr, const bmpc_real *z_init,
                   bmpc_mpc_work_t *w, bmpc_mpc_result_t *res)
{
  const int N = cfg->N, nu = N - 1, n = 2 * nu;
  bmpc_real *z = res->z;
  bmpc_real J;
  int it, k;

  res->nz = n;
  res->status = BMPC_OK;
  res->sqp_iters = 0;
  res->qp_iters = 0;
  if (N < 3 || N > BMPC_N_MAX) {
    res->status = BMPC_ERR_NUMERIC;
    return res->status;
  }
  if (!(pr->v_lb <= pr->v_ub) || !(pr->steer_lb <= pr->steer_ub)) {
    res->status = BMPC_ERR_INFEASIBLE;
    return res->status;
  }

  if (z_init) {
    memcpy(z, z_init, sizeof(bmpc_real) * (size_t)n);
  } else {
    const bmpc_real v0 = pr->s0[BMPC_SV];
    for (k = 0; k < nu; ++k) {
      z[k] = pr->prev_delta_ref;
      z[nu + k] = v0;
    }
  }
  clamp_z(pr, nu, z);
  J = bmpc_mpc_cost(cfg, pr, z, w->sbar);

  for (it = 0; it < cfg->sqp_iters; ++it) {
    int qp_it = 0, st;
    bmpc_real step = 0, alpha = 1;
    int accepted = 0, ls;

    build_qp(cfg, pr, z, w);
    memset(w->dz, 0, sizeof(bmpc_real) * (size_t)n);
    st = bmpc_boxqp_solve(n, w->P, w->q, w->lb, w->ub, w->dz, cfg->qp_max_iters, &w->qp, &qp_it);
    res->sqp_iters = it + 1;
    res->qp_iters += qp_it;
    if (st == BMPC_ERR_NUMERIC || st == BMPC_ERR_INFEASIBLE) {
      res->status = st;
      break;
    }
    if (st != BMPC_OK) res->status = st;

    /* 선형화가 틀린 만큼 비용이 오르면 스텝을 줄인다 (반씩 최대 3 번) */
    for (ls = 0; ls < 4; ++ls) {
      bmpc_real Jt;
      for (k = 0; k < n; ++k) w->ztry[k] = z[k] + alpha * w->dz[k];
      clamp_z(pr, nu, w->ztry);
      Jt = bmpc_mpc_cost(cfg, pr, w->ztry, w->sbar);
      if (Jt <= J || ls == 3) {
        accepted = 1;
        J = Jt;
        break;
      }
      alpha *= (bmpc_real)0.5;
    }
    if (accepted) {
      for (k = 0; k < n; ++k) {
        step = bm_max(step, BM_FABS(w->ztry[k] - z[k]));
        z[k] = w->ztry[k];
      }
    }
    if (step < cfg->sqp_tol) break;
  }
  res->cost = J;
  return res->status;
}
