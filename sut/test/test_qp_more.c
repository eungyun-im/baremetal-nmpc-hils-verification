/* bmpc_qp 추가 시험: 반복 상한 경로, 워크셋 규칙, 인수 갱신, 퇴화 입력.
 * 기대값은 손으로 푼 값과 솔버와 코드를 공유하지 않는 기준으로 계산한다. 솔버 출력을 기대값으로 복사하지 않는다.
 *   1. 독립 기준: 자체 KKT 검사, 3^n 열거, 손으로 푼 문제, 대각 P 의 닫힌 해
 *   2. 상한 경로: 상한 1, 2, 4, 8, 10, 16 의 status, iters, 범위, 목적함수, 반복점
 *   3. warm start: 유효, 범위 밖, NaN/Inf 시작점
 *   4. 퇴화와 잘못된 입력: lb == ub, lb > ub, NaN/Inf, 부정치/특이 P, n 한계
 *   5. 경계: 허용오차, 동률, 처음 working set, 작업 메모리 침범, 인수 갱신 경로 (풀기/막기 각각), 스텝 클램프
 *   6. double 과 BMPC_USE_FLOAT 양쪽에서 돈다. 허용오차는 BM_EPS 에서 계산한다 (아래 설명).
 *   7. 실제 규모: bmpc_mpc_solve 가 만든 QP (pmax 약 3e5 ~ 1e6) 의 상한 경로. 운용 속도 (15~16 m/s) 와 highway (20~25 m/s) 를 포함한다
 *   9. MPC 수준: SQP 1, 2 x QP 상한의 각 칸에서 "상한에 걸린 해가 실제로 쓰이는가". z 를 시험 안의 SQP 기준 (같은 QP 를 직접 풀고
 *      선 탐색을 따로 계산) 과 비교한다. warm start 와 폐루프 프레임은 bmpc_controller_step 을 그대로 돌려 본다.
 *   8. -DBMPC_PROFILE 빌드에서는 비용 모델(기울기, 분해/풀이 횟수)을 정확히 센다. 기본 빌드에서는 빠진다.
 *
 * 약속의 출처 구분:
 *   [약속] 헤더/주석에 적힌 동작. bmpc_types.h:33 "그때까지의 최선 해" 는 시작점(클램프 후)과 앞선 반복점보다
 *          나쁘지 않다는 뜻으로 읽는다. 진전이 있다는 약속은 아니다.
 *   [고정] 주석에 약속은 없지만 상한 그리드 결과를 바꾸는 현재 동작을 고정한다. 의도해서 바꾸면 여기도 고친다.
 *
 * 허용오차 근거:
 *   - 솔버가 멈추는 승수 허용오차가 코드 스스로 lambda_tol = 1e3 eps (1 + qmax + pmax xmax) 다 (bmpc_qp.c:133).
 *     KKT 검사는 이 값을 기준으로 쓴다. 해의 오차는 2 lambda_tol / lambda_min(P) 로 잡는다 (P = A'A + mu I, lambda_min >= mu).
 *   - 반복점 비교: 작은 문제(조건수 약 100 이하)에서 Cholesky 와 가우스 소거의 반올림 차이. 2e3 eps (1 + |x|).
 *     기준은 double 로 계산하므로 float 빌드의 차이는 float 쪽 반올림이다.
 *     이 비교는 조건이 좋은 문제에서만 뜻이 있다. 실제 규모 QP 에서는 float 경로가 double 과 갈린다 (test_real_scale).
 *   - 실제 규모의 KKT: lambda_tol 에 자유 변수 항 n pmax step_tol (1 + |x|) 를 더한다 (bmpc_qp.c:166 의 정지 규칙에서 유도).
 *
 * 이 파일이 일부러 고정하지 않는 것: 결함으로 보는 동작은 통과 시험에 넣지 않고 after_qp/repro_*.c 에 둔다
 * (bmpc_qp.c:254 의 거짓 OK 두 경로, 고정 변수의 Inf q, float lambda_tol, SQP status 가 지워지지 않는 것).
 * 0 길이 스텝은 test_zero_step_batch 가 고정한다. test_mpc_* 의 기대값은 QP 직접 호출에서 나오므로 함께 바뀐다.
 * 단 처음 working set 규칙 (bmpc_qp.c:136 주석, 기울기 0 은 바깥이 아니다) 을 바꾸는 수정은 그 규칙을 고정한 칸에 걸린다. 주석이 바뀌는 수정이므로 함께 고친다. */
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmpc/bmpc_config.h"
#include "bmpc/bmpc_controller.h"
#include "bmpc/bmpc_math.h"
#include "bmpc/bmpc_mpc.h"
#include "bmpc/bmpc_prof.h"
#include "bmpc/bmpc_qp.h"
#include "bmpc/bmpc_weights.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                                                    \
  do {                                                                                      \
    if (cond) {                                                                             \
      ++g_pass;                                                                             \
    } else {                                                                                \
      ++g_fail;                                                                             \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);                                         \
      printf(__VA_ARGS__);                                                                  \
      printf("\n");                                                                         \
    }                                                                                       \
  } while (0)

#ifdef BMPC_USE_FLOAT
#define REL_TOL 2e-5
#else
#define REL_TOL 1e-11
#endif
static int near_(double a, double b) { return fabs(a - b) <= REL_TOL * (1.0 + fabs(b)); }
#define NEAR(a, b) near_((double)(a), (double)(b))
#define R(v) ((bmpc_real)(v))
#define EPS ((double)BM_EPS)
#define NMAX BMPC_NZ_MAX
#define STEP_TOL (R(1e2) * BM_EPS) /* bmpc_qp.c:134 */
#ifdef BMPC_USE_FLOAT
#define BMPC_REAL_MAX FLT_MAX
#else
#define BMPC_REAL_MAX DBL_MAX
#endif

/* -DBMPC_PROFILE: 시계 대신 호출마다 1 씩 느는 값을 준다. 안쪽에 다른 구간이 없는 구간 (GRAD, CHOL) 은 호출 횟수를 그대로 센다. */
#ifdef BMPC_PROFILE
volatile uint32_t bmpc_prof[BMPC_PROF_COUNT];
static uint32_t g_tick = 0;
uint32_t bmpc_prof_now(void) { return ++g_tick; }
#define PROF_RESET() memset((void *)bmpc_prof, 0, sizeof(bmpc_prof))
#else
#define PROF_RESET() ((void)0)
#endif

/* ---------------- 시험대: 배열 끝의 파수꾼과 작업 메모리 독 ---------------- */
/* n 다음 칸에 파수꾼을 둔다. 고른 값이 읽히면 허용오차가 커지고 (q, lb, ub, P), 범위 검사가 INFEASIBLE 을 내고 (lb > ub),
 * 스텝 허용오차가 커진다 (x). 바르게 짠 코드는 읽지 않는다. */
typedef struct {
  int n;
  bmpc_real P[(NMAX + 2) * (NMAX + 2)];
  bmpc_real q[NMAX + 2], lb[NMAX + 2], ub[NMAX + 2], x[NMAX + 2];
} prob_t;

static prob_t g_p;
static bmpc_qp_work_t g_w, g_w0;
static int g_d_nan = 0; /* 독 선택: 1 이면 d 는 NaN, t 는 유한 값. 0 이면 반대 */

#define SENT 1e30

static void prob_begin(prob_t *p, int n)
{
  int i;
  p->n = n;
  for (i = 0; i < (NMAX + 2) * (NMAX + 2); ++i) p->P[i] = R(SENT);
  for (i = 0; i < NMAX + 2; ++i) {
    p->q[i] = R(SENT);
    p->lb[i] = R(SENT);
    p->ub[i] = R(-SENT);
    p->x[i] = R(SENT);
  }
}

/* 코드가 쓰기 전에 읽으면 드러나는 값으로 채운다. 쓰지 말아야 할 칸은 끝난 뒤 비트로 비교한다. */
static void poison_work(void)
{
  int i;
  const bmpc_real nan_ = (bmpc_real)NAN;
  memset(&g_w, 0, sizeof(g_w));
  for (i = 0; i < NMAX; ++i) {
    g_w.g[i] = R(-SENT);
    g_w.d[i] = g_d_nan ? nan_ : R(3.5e20);
    g_w.t[i] = g_d_nan ? R(-6.5e20) : nan_; /* NaN: 읽히면 r 이 NaN 이라 재분해로 빠진다. 유한 값: 쓰이면 비트 비교에 걸린다 */
    g_w.free_idx[i] = 0x3fffffff - i; /* 범위 밖 번호. 칸마다 달라 칸끼리 옮겨도 드러난다 */
    g_w.ws[i] = -1;
  }
  for (i = 0; i < NMAX * NMAX; ++i) g_w.M[i] = R(2.5e10); /* 양수: 대각으로 잘못 읽혀도 fallback 에 숨지 못한다 */
  memcpy(&g_w0, &g_w, sizeof(g_w));
}

static int same_bits(const bmpc_real *a, const bmpc_real *b) { return memcmp(a, b, sizeof(bmpc_real)) == 0; }

/* n 밖의 칸, 입력 x 의 n 번째, 인수의 윗삼각을 건드리면 안 된다. */
static void check_guards(const prob_t *p, const char *where)
{
  const int n = p->n;
  int i, j, bad = 0;
  for (i = n; i < NMAX; ++i) {
    if (!same_bits(&g_w.g[i], &g_w0.g[i])) ++bad;
    if (!same_bits(&g_w.d[i], &g_w0.d[i])) ++bad;
    if (g_w.free_idx[i] != g_w0.free_idx[i]) ++bad;
    if (g_w.ws[i] != g_w0.ws[i]) ++bad;
  }
  for (i = n > 0 ? n - 1 : 0; i < NMAX; ++i) /* chol_remove 는 t[0 .. nf-2] 만 쓴다 */
    if (!same_bits(&g_w.t[i], &g_w0.t[i])) ++bad;
  for (i = 0; i < NMAX; ++i)
    for (j = 0; j < NMAX; ++j)
      if ((i >= n || j > i) && !same_bits(&g_w.M[i * NMAX + j], &g_w0.M[i * NMAX + j])) ++bad;
  if (p->x[n] != R(SENT)) ++bad;
  CHECK(bad == 0, "%s: n=%d work memory written outside its range (%d slots)", where, n, bad);
}

/* 시작점 x0 으로 풀고 경계 검사까지. 결과는 p->x. */
static int solve(prob_t *p, const bmpc_real *x0, int cap, int *iters)
{
  int st, i;
  for (i = 0; i < p->n; ++i) p->x[i] = x0[i];
  poison_work();
  *iters = -7;
  PROF_RESET();
  st = bmpc_boxqp_solve(p->n, p->P, p->q, p->lb, p->ub, p->x, cap, &g_w, iters);
  check_guards(p, "solve");
  return st;
}

static double grad_i(const prob_t *p, const bmpc_real *x, int i)
{
  double g = p->q[i];
  int j;
  for (j = 0; j < p->n; ++j) g += (double)p->P[i * p->n + j] * (double)x[j];
  return g;
}

static double objective(const prob_t *p, const bmpc_real *x)
{
  double f = 0;
  int i, j;
  for (i = 0; i < p->n; ++i) {
    double s = 0;
    for (j = 0; j < p->n; ++j) s += (double)p->P[i * p->n + j] * (double)x[j];
    f += (double)x[i] * (0.5 * s + (double)p->q[i]);
  }
  return f;
}

/* bmpc_qp.c:133 의 약속된 눈금. 솔버 내부 값을 읽지 않고 문제에서 다시 계산한다. */
static double lambda_scale(const prob_t *p)
{
  double qm = 0, pm = 0, xm = 0;
  int i, j;
  for (i = 0; i < p->n; ++i) {
    qm = fmax(qm, fabs((double)p->q[i]));
    xm = fmax(xm, fmax(fabs((double)p->lb[i]), fabs((double)p->ub[i])));
    for (j = 0; j < p->n; ++j) pm = fmax(pm, fabs((double)p->P[i * p->n + j]));
  }
  return 1e3 * EPS * (1.0 + qm + pm * xm);
}

/* 자체 KKT 검사. bmpc_boxqp_kkt_residual 을 쓰지 않는다. 범위는 반올림 여유 없이 정확히 본다. */
static int kkt_ok(const prob_t *p, const bmpc_real *x, double tol)
{
  int i;
  for (i = 0; i < p->n; ++i) {
    const double g = grad_i(p, x, i);
    if (!(x[i] >= p->lb[i] && x[i] <= p->ub[i])) return 0;
    if (p->lb[i] == p->ub[i]) continue;
    if (x[i] <= p->lb[i]) {
      if (g < -tol) return 0;
    } else if (x[i] >= p->ub[i]) {
      if (g > tol) return 0;
    } else if (fabs(g) > tol) {
      return 0;
    }
  }
  return 1;
}

/* ---------------- 독립 기준 ---------------- */
static int gauss_solve(int m, double *A, double *b) /* A: m x m 행 우선, 부분 피벗 */
{
  int i, j, k;
  for (k = 0; k < m; ++k) {
    int pv = k;
    for (i = k + 1; i < m; ++i)
      if (fabs(A[i * m + k]) > fabs(A[pv * m + k])) pv = i;
    if (fabs(A[pv * m + k]) < 1e-300) return -1;
    if (pv != k) {
      double t;
      for (j = 0; j < m; ++j) {
        t = A[k * m + j];
        A[k * m + j] = A[pv * m + j];
        A[pv * m + j] = t;
      }
      t = b[k];
      b[k] = b[pv];
      b[pv] = t;
    }
    for (i = k + 1; i < m; ++i) {
      const double f = A[i * m + k] / A[k * m + k];
      for (j = k; j < m; ++j) A[i * m + j] -= f * A[k * m + j];
      b[i] -= f * b[k];
    }
  }
  for (i = m - 1; i >= 0; --i) {
    for (j = i + 1; j < m; ++j) b[i] -= A[i * m + j] * b[j];
    b[i] /= A[i * m + i];
  }
  return 0;
}

/* 3^n 열거: 각 변수를 하한/상한/자유로 두고 정확히 풀어 가능한 점 중 목적함수 최소. P 양정치면 유일. n <= 7. */
static void enumerate_solution(const prob_t *p, double *xbest)
{
  const int n = p->n;
  int total = 1, c, i, j;
  double best = HUGE_VAL;
  static double A[49], b[7], xx[7];
  for (i = 0; i < n; ++i) total *= 3;
  for (c = 0; c < total; ++c) {
    int code = c, fr[7], m = 0, ok = 1;
    double f = 0;
    for (i = 0; i < n; ++i) {
      const int s = code % 3;
      code /= 3;
      if (s == 0) xx[i] = (double)p->lb[i];
      else if (s == 1) xx[i] = (double)p->ub[i];
      else {
        xx[i] = 0;
        fr[m++] = i;
      }
    }
    for (i = 0; i < m; ++i) {
      double r = -(double)p->q[fr[i]];
      for (j = 0; j < n; ++j) r -= (double)p->P[fr[i] * n + j] * xx[j]; /* 자유 변수는 아직 0 */
      b[i] = r;
      for (j = 0; j < m; ++j) A[i * m + j] = (double)p->P[fr[i] * n + fr[j]];
    }
    if (m > 0 && gauss_solve(m, A, b) != 0) continue;
    for (i = 0; i < m; ++i) xx[fr[i]] = b[i];
    for (i = 0; i < n; ++i)
      if (xx[i] < (double)p->lb[i] || xx[i] > (double)p->ub[i]) ok = 0;
    if (!ok) continue;
    for (i = 0; i < n; ++i) {
      double s = 0;
      for (j = 0; j < n; ++j) s += (double)p->P[i * n + j] * xx[j];
      f += xx[i] * (0.5 * s + (double)p->q[i]);
    }
    if (f < best) {
      best = f;
      for (i = 0; i < n; ++i) xbest[i] = xx[i];
    }
  }
}

/* 규칙 기준기: 헤더(bmpc_qp.h:3-5)와 주석(bmpc_qp.c:136, 197, 253)의 규칙을 그대로 따르는 단순한 active-set.
 * 매 반복 가우스 소거로 풀고 double 로 계산한다. 인수 갱신과 코드를 공유하지 않지만, 규칙은 솔버와 같으므로
 * 규칙 자체의 옳음을 보여 주지는 않는다 (그건 열거와 KKT 가 본다). 동률은 F 의 앞쪽을 택한다. */
#define REF_KMAX 400
typedef struct {
  int K;      /* 끝나는 데 든 반복 수 */
  int status; /* 끝났을 때의 상태 */
  int newton; /* 부분 문제를 푼 횟수 */
  int updates; /* 인수 갱신 횟수: 막힘 + 인수가 있는 상태의 풀기 */
  double xs[REF_KMAX + 1][NMAX];  /* xs[k]: 상한 k 일 때 돌려줄 x */
} ref_t;
static ref_t g_ref;
static double g_rA[NMAX * NMAX], g_rb[NMAX];
static double g_ref_eps = 0; /* 0 이면 빌드의 eps 로 멈춘다 (경로 비교). DBL_EPSILON 이면 정확한 해에 가까운 기준 (실제 규모) */

static void ref_run(const prob_t *p, const bmpc_real *x0)
{
  const int n = p->n;
  double x[NMAX], g[NMAX];
  int ws[NMAX], F[NMAX];
  int nf = 0, i, j, k, it, sub = 0, last_rel = -1, factored = 0;
  const double eps = g_ref_eps > 0 ? g_ref_eps : EPS;
  const double ltol = lambda_scale(p) * (eps / EPS), stol = 1e2 * eps;
  for (i = 0; i < n; ++i) {
    double v = (double)x0[i];
    if (!isfinite(v)) v = 0.5 * ((double)p->lb[i] + (double)p->ub[i]);
    v = fmin(fmax(v, (double)p->lb[i]), (double)p->ub[i]);
    x[i] = v;
  }
  for (i = 0; i < n; ++i) {
    double gi = (double)p->q[i];
    for (j = 0; j < n; ++j) gi += (double)p->P[i * n + j] * x[j];
    if (p->lb[i] == p->ub[i]) ws[i] = -1;
    else if (x[i] <= (double)p->lb[i] && gi > 0) ws[i] = -1;
    else if (x[i] >= (double)p->ub[i] && gi < 0) ws[i] = 1;
    else ws[i] = 0;
    if (ws[i] == 0) F[nf++] = i;
  }
  for (i = 0; i < n; ++i) g_ref.xs[0][i] = x[i];
  g_ref.status = BMPC_ERR_MAXITER;
  g_ref.K = REF_KMAX;
  g_ref.newton = 0;
  g_ref.updates = 0;
  for (it = 0; it < REF_KMAX; ++it) {
    double dmax = 0, xn = 0;
    for (i = 0; i < n; ++i) {
      g[i] = (double)p->q[i];
      for (j = 0; j < n; ++j) g[i] += (double)p->P[i * n + j] * x[j];
      xn = fmax(xn, fabs(x[i]));
    }
    if (!sub && nf > 0) {
      for (i = 0; i < nf; ++i) {
        g_rb[i] = -g[F[i]];
        for (j = 0; j < nf; ++j) g_rA[i * nf + j] = (double)p->P[F[i] * n + F[j]];
      }
      gauss_solve(nf, g_rA, g_rb);
      ++g_ref.newton;
      factored = 1;
      for (k = 0; k < nf; ++k) dmax = fmax(dmax, fabs(g_rb[k]));
      if (dmax <= stol * (1.0 + xn)) sub = 1;
    } else if (nf == 0) {
      sub = 1;
    }
    if (sub) {
      int worst = -1;
      double wl = -ltol;
      for (i = 0; i < n; ++i) {
        double l;
        if (ws[i] == 0 || p->lb[i] == p->ub[i]) continue;
        l = ws[i] < 0 ? g[i] : -g[i];
        if (l < wl) {
          wl = l;
          worst = i;
        }
      }
      if (worst < 0) {
        g_ref.K = it + 1;
        g_ref.status = BMPC_OK;
        for (i = 0; i < n; ++i) g_ref.xs[it + 1][i] = x[i];
        return;
      }
      ws[worst] = 0;
      if (factored) ++g_ref.updates;
      F[nf++] = worst;
      last_rel = worst;
      sub = 0;
    } else {
      double alpha = 1;
      int blk = -1, side = 0, pos = -1;
      for (k = 0; k < nf; ++k) {
        const int r = F[k];
        if (g_rb[k] > 0 && x[r] + g_rb[k] > (double)p->ub[r]) {
          const double a = ((double)p->ub[r] - x[r]) / g_rb[k];
          if (a < alpha) {
            alpha = a;
            blk = r;
            pos = k;
            side = 1;
          }
        } else if (g_rb[k] < 0 && x[r] + g_rb[k] < (double)p->lb[r]) {
          const double a = ((double)p->lb[r] - x[r]) / g_rb[k];
          if (a < alpha) {
            alpha = a;
            blk = r;
            pos = k;
            side = -1;
          }
        }
      }
      for (k = 0; k < nf; ++k) {
        const int r = F[k];
        x[r] = fmin(fmax(x[r] + alpha * g_rb[k], (double)p->lb[r]), (double)p->ub[r]);
      }
      if (blk >= 0) {
        /* alpha == 0: 같은 d 로 길이 0 인 바깥 스텝을 가진 다른 변수도 함께 고정한다 (방금 푼 변수는 blk 일 때만) */
        const int batch = alpha == 0 && blk != last_rel;
        for (k = nf - 1; k >= 0; --k) {
          const int r = F[k];
          int s = 0;
          if (k == pos) s = side;
          else if (batch && r != last_rel) {
            if (g_rb[k] > 0 && x[r] >= (double)p->ub[r] && x[r] + g_rb[k] > (double)p->ub[r]) s = 1;
            else if (g_rb[k] < 0 && x[r] <= (double)p->lb[r] && x[r] + g_rb[k] < (double)p->lb[r]) s = -1;
          }
          if (s == 0) continue;
          x[r] = s > 0 ? (double)p->ub[r] : (double)p->lb[r];
          ws[r] = s;
          ++g_ref.updates;
          for (j = k; j < nf - 1; ++j) F[j] = F[j + 1];
          --nf;
        }
        if (blk == last_rel && alpha <= stol) {
          g_ref.K = it + 1;
          g_ref.status = BMPC_OK;
          for (i = 0; i < n; ++i) g_ref.xs[it + 1][i] = x[i];
          return;
        }
      } else {
        sub = 1;
      }
    }
    for (i = 0; i < n; ++i) g_ref.xs[it + 1][i] = x[i];
  }
}

/* 나갈 때의 인수: w->M 이 정말 지금 F 의 P_FF 인수인지 (L L' == P_FF), F 와 ws 가 서로 맞는지, 고정 변수가 경계에 있는지.
 * 처음에 자유 변수가 있어 한 번 이상 분해한 양정치 문제에서만 부른다 (그래야 factored 가 참이다). */
static void check_factor(const prob_t *p, const char *where)
{
  const int n = p->n;
  int nf = 0, i, j, k, bad = 0;
  double pm = 0, emax = 0;
  for (i = 0; i < n; ++i) {
    if (g_w.ws[i] == 0) ++nf;
    if (g_w.ws[i] < 0 && p->x[i] != p->lb[i]) ++bad;
    if (g_w.ws[i] > 0 && p->x[i] != p->ub[i]) ++bad;
  }
  for (k = 0; k < nf; ++k) {
    const int v = g_w.free_idx[k];
    if (v < 0 || v >= n || g_w.ws[v] != 0) {
      CHECK(0, "%s: free_idx[%d]=%d is not a free variable", where, k, v);
      return;
    }
    for (j = 0; j < k; ++j)
      if (g_w.free_idx[j] == v) ++bad;
  }
  CHECK(bad == 0, "%s: working set and x / free list disagree (%d)", where, bad);
  for (i = 0; i < n * n; ++i) pm = fmax(pm, fabs((double)p->P[i]));
  for (i = 0; i < nf; ++i)
    for (j = 0; j <= i; ++j) {
      double s = 0;
      for (k = 0; k <= j; ++k) s += (double)g_w.M[i * NMAX + k] * (double)g_w.M[j * NMAX + k];
      emax = fmax(emax, fabs(s - (double)p->P[g_w.free_idx[i] * n + g_w.free_idx[j]]));
    }
  /* 갱신을 거듭한 인수의 반올림: 1e3 eps pmax 안쪽 */
  CHECK(emax <= 1e3 * EPS * pm, "%s: |L L' - P_FF| = %g (pmax %g, nf %d)", where, emax, pm, nf);
}

/* 같은 F 를 처음부터 분해했을 때와 비트까지 같은가 (bmpc_qp.c:17 과 같은 연산 순서). 다르면 갱신 경로를 거친 인수다. */
static int factor_is_fresh(const prob_t *p)
{
  static bmpc_real L[NMAX * NMAX];
  const int n = p->n;
  int nf = 0, i, j;
  for (i = 0; i < n; ++i)
    if (g_w.ws[i] == 0) ++nf;
  for (i = 0; i < nf; ++i)
    for (j = 0; j <= i; ++j) L[i * NMAX + j] = p->P[g_w.free_idx[i] * n + g_w.free_idx[j]];
  for (j = 0; j < nf; ++j) {
    const bmpc_real *rj = &L[j * NMAX];
    bmpc_real s = L[j * NMAX + j] - bm_dot(rj, rj, j), inv;
    if (!(s > 0)) return 0;
    s = BM_SQRT(s);
    L[j * NMAX + j] = s;
    inv = (bmpc_real)1.0 / s;
    for (i = j + 1; i < nf; ++i) L[i * NMAX + j] = (L[i * NMAX + j] - bm_dot(&L[i * NMAX], rj, j)) * inv;
  }
  for (i = 0; i < nf; ++i)
    if (memcmp(&L[i * NMAX], &g_w.M[i * NMAX], sizeof(bmpc_real) * (size_t)(i + 1)) != 0) return 0;
  return 1;
}

/* ---------------- 난수 문제 ---------------- */
static unsigned g_seed = 20261105u;
static double rnd(void)
{
  g_seed = g_seed * 1664525u + 1013904223u;
  return (double)(g_seed >> 8) / 16777216.0;
}
static double rnd2(double a, double b) { return a + (b - a) * rnd(); }

/* P = A'A + mu I, 대칭 양정치. lambda_min(P) >= mu. A 항목은 [-s, s]. */
static void gen_P(prob_t *p, int n, double s, double mu)
{
  static double A[NMAX * NMAX];
  int i, j, k;
  for (i = 0; i < n * n; ++i) A[i] = rnd2(-s, s);
  for (i = 0; i < n; ++i)
    for (j = 0; j <= i; ++j) {
      double v = (i == j) ? mu : 0.0;
      for (k = 0; k < n; ++k) v += A[k * n + i] * A[k * n + j];
      p->P[i * n + j] = R(v);
      p->P[j * n + i] = R(v);
    }
}

/* 시작점 종류: 0 = 가운데, 1 = 경계, 2 = 범위 밖, 3 = 임의 내부. */
static void gen_x0(const prob_t *p, bmpc_real *x0, int kind)
{
  int i;
  for (i = 0; i < p->n; ++i) {
    switch (kind) {
      case 0: x0[i] = R(0.5 * ((double)p->lb[i] + (double)p->ub[i])); break;
      case 1: x0[i] = rnd() < 0.5 ? p->lb[i] : p->ub[i]; break;
      case 2: x0[i] = R(rnd() < 0.5 ? (double)p->lb[i] - 3.0 : (double)p->ub[i] + 3.0); break;
      default: x0[i] = R(rnd2((double)p->lb[i], (double)p->ub[i])); break;
    }
  }
}

static void gen_prob(prob_t *p, int n, double qs, double fixed_frac)
{
  int i;
  prob_begin(p, n);
  gen_P(p, n, 1.0, 1.0);
  for (i = 0; i < n; ++i) {
    p->q[i] = R(rnd2(-qs, qs));
    p->lb[i] = R(rnd2(-2.0, -0.1));
    p->ub[i] = R(rnd2(0.1, 2.0));
    if (rnd() < fixed_frac) {
      const double v = rnd2(-1.0, 1.0);
      p->lb[i] = R(v);
      p->ub[i] = R(v);
    }
  }
}

/* ---------------- 1. 손으로 푼 문제와 독립 기준 ---------------- */
static void set2(prob_t *p, double p00, double p01, double p11, double q0, double q1, double l0, double u0, double l1,
                 double u1)
{
  prob_begin(p, 2);
  p->P[0] = R(p00);
  p->P[1] = R(p01);
  p->P[2] = R(p01);
  p->P[3] = R(p11);
  p->q[0] = R(q0);
  p->q[1] = R(q1);
  p->lb[0] = R(l0);
  p->ub[0] = R(u0);
  p->lb[1] = R(l1);
  p->ub[1] = R(u1);
}

static void test_hand_solved(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX];
  int it, st;
  printf("[qp: hand-solved cases]\n");

  /* min x^2 - 2x (P=2, q=-2), 자유 해 x=1 */
  prob_begin(p, 1);
  p->P[0] = R(2);
  p->q[0] = R(-2);
  p->lb[0] = R(-10);
  p->ub[0] = R(10);
  x0[0] = R(5);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 2 && NEAR(p->x[0], 1.0), "n=1 free: st %d it %d x %g", st, it, (double)p->x[0]);

  /* 같은 문제, 상한에 막힘: x=0.5, g=-1 < 0 */
  p->ub[0] = R(0.5);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && p->x[0] == R(0.5), "n=1 upper: st %d x %g", st, (double)p->x[0]);
  /* 하한에 막힘 */
  p->lb[0] = R(3);
  p->ub[0] = R(4);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && p->x[0] == R(3), "n=1 lower: st %d x %g", st, (double)p->x[0]);

  /* P=[2,1;1,2], q=(-3,0), 박스 [0,10]^2: 무제약 해 (2,-1). x2 가 하한에 막혀 x1 = 1.5, g2 = 1.5 > 0 */
  set2(p, 2, 1, 2, -3, 0, 0, 10, 0, 10);
  x0[0] = R(1);
  x0[1] = R(1);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && NEAR(p->x[0], 1.5) && p->x[1] == 0, "2x2 bound: st %d x %g %g", st, (double)p->x[0],
        (double)p->x[1]);

  /* 음의 결합: P=[1,-1;-1,2], q=(-1,0.5). 해 x1 = 0.5 를 먼저 고정에서 풀어야 나온다.
   * x0 = (0,0) 에서 g = (-1, 0.5) 라 x2 는 하한 고정. x1 = 1 로 간 뒤 g2 = -0.5, 풀면 무제약 해 (1.5, 0.5). */
  set2(p, 1, -1, 2, -1, 0.5, -10, 10, 0, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && NEAR(p->x[0], 1.5) && NEAR(p->x[1], 0.5), "release: st %d x %g %g", st, (double)p->x[0],
        (double)p->x[1]);
  /* 끝까지 든 반복: F={0} Newton, 승수 보고 풀기, F={0,1} Newton, 확인 = 4 */
  CHECK(it == 4, "release iteration count %d", it);
  check_factor(p, "release");

  /* 상한 쪽 대칭: x -> -x. 해 (-1.5,-0.5) */
  set2(p, 1, -1, 2, 1, -0.5, -10, 10, -10, 0);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 4 && NEAR(p->x[0], -1.5) && NEAR(p->x[1], -0.5), "release upper: st %d it %d x %g %g", st,
        it, (double)p->x[0], (double)p->x[1]);
}

/* 대각 P 의 닫힌 해: 무제약 최소점 u_i = -q_i/P_ii 로 곧게 가다가 경계에서 멈춘다. */
static void test_diag_path(void)
{
  enum { N = 20 };
  prob_t *p = &g_p;
  bmpc_real x0[NMAX];
  int sign, i, c, it, st;
  printf("[qp: diagonal closed form, every capped iterate]\n");
  for (sign = -1; sign <= 1; sign += 2) {
    double frac[N];
    prob_begin(p, N);
    for (i = 0; i < N * N; ++i) p->P[i] = 0;
    for (i = 0; i < N; ++i) {
      /* 변수 i < 16 은 직선 경로의 t = (i+1)/32 에서 경계에 닿고, 나머지는 t = 2 라 닿지 않는다. 비율이 모두 이진 유리수다. */
      const double dist = 1.0 + (double)(i % 3);
      const double t = (i < 16) ? (double)(i + 1) / 32.0 : 2.0;
      const double u = sign * dist / t;
      p->P[i * N + i] = R(1.0 + (double)(i % 4));
      p->q[i] = R(-u * (double)p->P[i * N + i]);
      p->lb[i] = R(sign > 0 ? -1.0 : -dist);
      p->ub[i] = R(sign > 0 ? dist : 1.0);
      x0[i] = 0;
      frac[i] = t;
    }
    /* K = 막히는 수 16 + 막히지 않는 스텝 1 + 확인 1 */
    for (c = 0; c <= 19; ++c) {
      const double tc = (c == 0) ? 0.0 : (c <= 16 ? (double)c / 32.0 : 1.0);
      st = solve(p, x0, c, &it);
      CHECK(st == (c >= 18 ? BMPC_OK : BMPC_ERR_MAXITER), "sign %d cap %d status %d", sign, c, st);
      CHECK(it == (c < 18 ? c : 18), "sign %d cap %d iters %d", sign, c, it);
      for (i = 0; i < N; ++i) {
        const double u = -(double)p->q[i] / (double)p->P[i * N + i];
        const double want = (tc >= frac[i]) ? (u > 0 ? (double)p->ub[i] : (double)p->lb[i]) : tc * u;
        CHECK(NEAR(p->x[i], want), "sign %d cap %d x[%d]=%g want %g", sign, c, i, (double)p->x[i], want);
      }
      if (c >= 1) check_factor(p, "diag");
    }
  }
}

static void test_enumeration_oracle(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX];
  double xe[7];
  int trial, i, st, it;
  printf("[qp: 3^n enumeration and own KKT check]\n");
  for (trial = 0; trial < 900; ++trial) {
    const int n = 1 + trial % 6;
    /* 쉬운 눈금 + 경계에 많이 막히는 눈금 + lb == ub 를 섞는다 */
    gen_prob(p, n, (trial % 3 == 0) ? 1.0 : 8.0, (trial % 5 == 0) ? 0.3 : 0.0);
    gen_x0(p, x0, trial % 4);
    st = solve(p, x0, 1000, &it);
    CHECK(st == BMPC_OK, "trial %d status %d", trial, st);
    {
      const double lt = lambda_scale(p);
      /* mu = 1 이므로 x 오차는 2 lambda_tol 안쪽 */
      enumerate_solution(p, xe);
      CHECK(kkt_ok(p, p->x, 1.5 * lt), "trial %d KKT failed", trial);
      for (i = 0; i < n; ++i)
        CHECK(fabs((double)p->x[i] - xe[i]) <= 2.0 * lt + 8 * EPS * (1.0 + fabs(xe[i])), "trial %d x[%d]=%.9g enum %.9g", trial,
              i, (double)p->x[i], xe[i]);
    }
  }
}

/* ---------------- 2. 상한 경로 ---------------- */
static const int CAPS[] = {1, 2, 4, 8, 10, 16};
#define NCAPS ((int)(sizeof(CAPS) / sizeof(CAPS[0])))
#define REF_TOL(x) (2e3 * EPS * (1.0 + fabs(x)))

/* 갱신 경로를 지난 인수 비율을 막힘 뒤와 풀기 뒤로 나눠 센다. 합쳐 세면 한쪽이 전부 재분해여도 다른 쪽이 가린다. */
static int g_update_used = 0, g_update_seen = 0;
static int g_block_used = 0, g_block_seen = 0, g_rel_used = 0, g_rel_seen = 0;
static int g_prev_nf = 0;

/* 상한 c 로 푼 결과를 기준기의 c 번째 반복점과 비교한다. 약속: status, iters, 범위, 목적함수. 고정: 반복점. */
static void check_capped(prob_t *p, const bmpc_real *x0, int c, int nf0, double *f_prev, const char *tag)
{
  int it = -7, st, i, viol = 0;
  double f, xerr = 0;
  const double ftol = 64.0 * EPS * (1.0 + fabs(*f_prev));
  st = solve(p, x0, c, &it);
  if (c < g_ref.K) {
    CHECK(st == BMPC_ERR_MAXITER, "%s cap %d (K %d): status %d", tag, c, g_ref.K, st);
    CHECK(it == c, "%s cap %d: iters %d", tag, c, it);
  } else {
    CHECK(st == g_ref.status, "%s cap %d (K %d): status %d", tag, c, g_ref.K, st);
    CHECK(it == g_ref.K, "%s cap %d: iters %d want %d", tag, c, it, g_ref.K);
  }
  for (i = 0; i < p->n; ++i) {
    const int k = c < g_ref.K ? c : g_ref.K;
    if (!(p->x[i] >= p->lb[i] && p->x[i] <= p->ub[i])) ++viol;
    xerr = fmax(xerr, fabs((double)p->x[i] - g_ref.xs[k][i]) / REF_TOL(g_ref.xs[k][i]));
  }
  CHECK(viol == 0, "%s cap %d: %d bound violations", tag, c, viol);
  CHECK(xerr <= 1.0, "%s cap %d: iterate off the reference by %g tolerances", tag, c, xerr);
  f = objective(p, p->x);
  CHECK(f <= *f_prev + ftol, "%s cap %d: objective rose %.12g -> %.12g", tag, c, *f_prev, f);
  *f_prev = f;
  if (c >= 1 && nf0 > 0) {
    int nf_now = 0;
    int fresh;
    check_factor(p, tag);
    fresh = factor_is_fresh(p);
    for (i = 0; i < p->n; ++i)
      if (g_w.ws[i] == 0) ++nf_now;
    ++g_update_seen;
    if (!fresh) ++g_update_used;
    /* 이 상한의 마지막 반복이 무엇을 했는지는 바로 앞 상한과 F 의 크기를 비교하면 안다 */
    if (nf_now == g_prev_nf - 1) {
      ++g_block_seen;
      if (!fresh) ++g_block_used;
    } else if (nf_now == g_prev_nf + 1) {
      ++g_rel_seen;
      if (!fresh) ++g_rel_used;
    }
    g_prev_nf = nf_now;
  }
}

static void sweep_problem(prob_t *p, const bmpc_real *x0, int kmax, const char *tag)
{
  int c, it, st, nf0 = 0, i;
  double f;
  bmpc_real xc[NMAX + 2];
  ref_run(p, x0);
  CHECK(g_ref.status == BMPC_OK, "%s: reference did not finish", tag);
  /* 상한 0: 클램프한 시작점, iters 0 (bmpc_qp.c:119-124, 146). "최선 해" 의 기준점이다. */
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 0, "%s cap 0: status %d iters %d", tag, st, it);
  for (i = 0; i < p->n; ++i) {
    xc[i] = p->x[i];
    CHECK(NEAR(xc[i], g_ref.xs[0][i]), "%s cap 0: x[%d]", tag, i);
    if (g_w.ws[i] == 0) ++nf0;
  }
  f = objective(p, xc);
  g_prev_nf = nf0;
  for (c = 1; c <= kmax; ++c) check_capped(p, x0, c, nf0, &f, tag);
  /* 끝까지 푼 결과: 기준기의 마지막 반복점과 같은 x, 시작점보다 나쁠 수 없다 */
  {
    const double fc = objective(p, xc);
    double xerr = 0;
    st = solve(p, x0, 100000, &it);
    CHECK(st == BMPC_OK && it == g_ref.K, "%s uncapped: st %d it %d K %d", tag, st, it, g_ref.K);
    for (i = 0; i < p->n; ++i) xerr = fmax(xerr, fabs((double)p->x[i] - g_ref.xs[g_ref.K][i]) / REF_TOL(g_ref.xs[g_ref.K][i]));
    CHECK(xerr <= 1.0, "%s uncapped: x off the reference by %g tolerances", tag, xerr);
    CHECK(objective(p, p->x) <= fc + 64.0 * EPS * (1.0 + fabs(fc)), "%s: solution worse than the start", tag);
#ifdef BMPC_PROFILE
    /* 비용 모델: 기울기는 반복마다 한 번, Cholesky 구간은 부분 문제 풀이와 인수 갱신마다 한 번 */
    CHECK((int)bmpc_prof[BMPC_PROF_GRAD] == it, "%s: %u gradients for %d iterations", tag, (unsigned)bmpc_prof[BMPC_PROF_GRAD], it);
    CHECK((int)bmpc_prof[BMPC_PROF_CHOL] == g_ref.newton + g_ref.updates, "%s: %u factor sections, reference %d solves + %d updates", tag,
          (unsigned)bmpc_prof[BMPC_PROF_CHOL], g_ref.newton, g_ref.updates);
#endif
  }
}

/* 상한이 먹는 문제: 조밀한 P 에 q 가 커서 많이 막힌다. 각 상한에서 K > 상한 인 문제만 센다. */
static void test_cap_grid(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int trial, c, hits[NCAPS];
  printf("[qp: cap grid 1, 2, 4, 8, 10, 16]\n");
  memset(hits, 0, sizeof(hits));
  for (trial = 0; trial < 60; ++trial) {
    const int n = 20 + trial % 15;
    int i;
    gen_prob(p, n, 60.0, (trial % 7 == 0) ? 0.1 : 0.0);
    gen_x0(p, x0, trial % 4);
    sweep_problem(p, x0, 18, "grid");
    for (i = 0; i < NCAPS; ++i)
      if (CAPS[i] < g_ref.K) ++hits[i];
  }
  for (c = 0; c < NCAPS; ++c) CHECK(hits[c] >= 40, "cap %d was reached on only %d of 60 problems", CAPS[c], hits[c]);

  /* 작은 문제의 모든 상한 */
  for (trial = 0; trial < 300; ++trial) {
    const int n = 1 + trial % 12;
    gen_prob(p, n, (trial % 2) ? 3.0 : 15.0, (trial % 5 == 0) ? 0.25 : 0.0);
    gen_x0(p, x0, trial % 4);
    sweep_problem(p, x0, 30, "small");
    g_d_nan = !g_d_nan; /* d 의 독을 번갈아 */
  }
  g_d_nan = 0;
}

/* 손으로 만든 상한 경로: 정확한 값이 나오는 작은 문제 */
static void test_cap_hand(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: cap path, hand cases]\n");

  /* [고정] 정확한 해에 있어도 확인 반복을 못 하면 MAXITER. 상태 1 이 곧 나쁜 해라는 뜻은 아니다. */
  prob_begin(p, 1);
  p->P[0] = R(2);
  p->q[0] = R(-2);
  p->lb[0] = R(-10);
  p->ub[0] = R(10);
  x0[0] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 1 && NEAR(p->x[0], 1.0), "n=1 cap 1: st %d it %d x %g", st, it, (double)p->x[0]);
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_OK && it == 2 && NEAR(p->x[0], 1.0), "n=1 cap 2: st %d it %d", st, it);
  /* [고정] 확인 반복은 부분 문제를 다시 풀지 않는다 (bmpc_qp.c:152, sub_solved). 다시 풀면 O(nf^2) 이 반복마다 늘고 w->d 가 0 이 된다.
   * 첫 반복의 스텝 d = 1 이 그대로 남아 있어야 한다 (다시 풀면 반올림 크기). */
  CHECK(NEAR(g_w.d[0], 1.0), "verification iteration solved again: d %g", (double)g_w.d[0]);

  /* 상한 0 과 음수: 반복 없이 클램프한 시작점 */
  x0[0] = R(20);
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 0 && p->x[0] == R(10), "cap 0: st %d it %d x %g", st, it, (double)p->x[0]);
  st = solve(p, x0, -3, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 0 && p->x[0] == R(10), "cap -3: st %d it %d x %g", st, it, (double)p->x[0]);
  x0[0] = (bmpc_real)NAN;
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 0 && p->x[0] == 0, "cap 0 NaN start: midpoint, x %g", (double)p->x[0]);

  /* [고정] 스텝 비율: P=I, 시작 0, 무제약 해 (4,2), 상한 (1,10). 비율 1/4 에서 (1, 0.5) */
  set2(p, 1, 0, 1, -4, -2, -10, 1, -10, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(1) && p->x[1] == R(0.5), "fraction upper: x %g %g", (double)p->x[0],
        (double)p->x[1]);
  CHECK(g_w.ws[0] == 1 && g_w.ws[1] == 0, "blocked variable goes to the upper set");
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(1) && p->x[1] == R(2), "cap 2: x %g %g", (double)p->x[0], (double)p->x[1]);
  st = solve(p, x0, 3, &it);
  CHECK(st == BMPC_OK && it == 3, "cap 3: st %d it %d", st, it);
  /* 하한 쪽 */
  set2(p, 1, 0, 1, 4, 2, -1, 10, -10, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(-1) && p->x[1] == R(-0.5), "fraction lower: x %g %g", (double)p->x[0],
        (double)p->x[1]);
  CHECK(g_w.ws[0] == -1 && g_w.ws[1] == 0, "blocked variable goes to the lower set");

  /* 경계에서 기울기 0 인 변수가 있는 문제: x0 = 0, P=[2,1;1,2], q=(-3,0), 해 (1.5, 0). 진전 약속은 없으므로 (bmpc_types.h:33)
   * 상한에서는 상태, 범위, 목적함수만 본다. 첫 반복이 바깥 스텝을 고정하는 동작은 test_zero_step_batch 가 본다. */
  set2(p, 2, 1, 2, -3, 0, 0, 10, 0, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 1, "bound with zero gradient, cap 1: st %d it %d", st, it);
  CHECK(p->x[0] >= 0 && p->x[0] <= R(10) && p->x[1] >= 0 && p->x[1] <= R(10), "cap 1: x %g %g", (double)p->x[0], (double)p->x[1]);
  CHECK(objective(p, p->x) <= 0.0, "cap 1: objective above the start: %g", objective(p, p->x));
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && NEAR(p->x[0], 1.5) && p->x[1] == 0, "uncapped: st %d x %g %g", st, (double)p->x[0], (double)p->x[1]);
}

/* [고정] 길이 0 스텝의 한꺼번 고정 (bmpc_qp.c 의 alpha == 0 경로). 기대값은 손으로 푼다.
 * P = 2I + 11' 이면 P^-1 = (I - 11'/(n+2))/2. q 가 일부만 -1 이면 움직이는 변수 m 개의 해는 x = 1/(m+2), 나머지의 Newton 스텝은 음수다.
 * 첫 반복이 바깥 스텝을 가진 변수를 전부 고정하고, 두 번째 반복이 움직이고, 세 번째 반복이 확인한다. 풀이는 반복마다 한 번이다. */
static void set_2i11(prob_t *p, int n, const signed char *mover, const signed char *sign, bmpc_real inward, bmpc_real *x0)
{
  int i, j;
  prob_begin(p, n);
  for (i = 0; i < n; ++i) {
    const int si = sign ? sign[i] : 1;
    for (j = 0; j < n; ++j) {
      const int sj = sign ? sign[j] : 1;
      p->P[i * n + j] = R(si * sj * ((i == j ? 2 : 0) + 1));
    }
    p->q[i] = mover[i] ? R(-si) : R(si) * (-inward);
    p->lb[i] = si > 0 ? R(0) : R(-10);
    p->ub[i] = si > 0 ? R(10) : R(0);
    x0[i] = 0;
  }
}

static void check_batch(prob_t *p, const bmpc_real *x0, const signed char *mover, const signed char *sign, const char *tag)
{
  const int n = p->n;
  int it, st, i, m = 0, nf = 0;
  for (i = 0; i < n; ++i) m += mover[i] ? 1 : 0;
  /* 상한 1: x 는 그대로, 움직이지 못하는 변수는 전부 제 경계에 고정, F 는 원래 순서, 인수는 F 의 것 */
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 1, "%s cap 1: st %d it %d", tag, st, it);
  for (i = 0; i < n; ++i) {
    const int want = mover[i] ? 0 : ((sign && sign[i] < 0) ? 1 : -1);
    CHECK(p->x[i] == 0, "%s cap 1: x[%d]=%g", tag, i, (double)p->x[i]);
    CHECK(g_w.ws[i] == want, "%s cap 1: ws[%d]=%d want %d", tag, i, g_w.ws[i], want);
    if (mover[i]) {
      CHECK(g_w.free_idx[nf] == i, "%s cap 1: free_idx[%d]=%d want %d", tag, nf, g_w.free_idx[nf], i);
      ++nf;
    }
  }
  check_factor(p, tag);
#ifdef BMPC_PROFILE
  CHECK((int)bmpc_prof[BMPC_PROF_CHOL] == 1 + (n - m), "%s cap 1: %u factor sections, want 1 solve + %d removals", tag,
        (unsigned)bmpc_prof[BMPC_PROF_CHOL], n - m);
#endif
  /* 상한 2: 이미 해. 상한 2 에서 움직인다는 것이 이 수정의 목적이다 */
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 2, "%s cap 2: st %d it %d", tag, st, it);
  for (i = 0; i < n; ++i) {
    const double si = sign ? (double)sign[i] : 1.0;
    const double want = mover[i] ? si / (double)(m + 2) : 0.0;
    CHECK(fabs((double)p->x[i] - want) <= REF_TOL(want), "%s cap 2: x[%d]=%.9g want %.9g", tag, i, (double)p->x[i], want);
  }
  check_factor(p, tag);
#ifdef BMPC_PROFILE
  CHECK((int)bmpc_prof[BMPC_PROF_GRAD] == 2, "%s cap 2: %u gradients", tag, (unsigned)bmpc_prof[BMPC_PROF_GRAD]);
  CHECK((int)bmpc_prof[BMPC_PROF_CHOL] == 2 + (n - m), "%s cap 2: %u factor sections, want 2 solves + %d removals", tag,
        (unsigned)bmpc_prof[BMPC_PROF_CHOL], n - m);
#endif
  st = solve(p, x0, 3, &it);
  CHECK(st == BMPC_OK && it == 3, "%s cap 3: st %d it %d", tag, st, it);
  check_factor(p, tag);
}

/* 묶어서 고정한 뒤 오래된 last_released 로 승수 확인 없이 끝나면 안 된다 (f = 4, kkt = 5 인 거짓 OK 였다) */
static void test_stale_last_released(void)
{
  static bmpc_qp_work_t w;
  bmpc_real P[16] = {1, 0, -1, 0, 0, 1, 0, 1, -1, 0, 2, -2, 0, 1, -2, 6};
  bmpc_real q[4] = {-6, -2, 1, 3}, lb[4] = {-1, -1, -2, 0}, ub[4] = {0, 2, 2, 2}, x[4] = {-1, 2, 2, 2};
  int iters = 0;
  const int st = bmpc_boxqp_solve(4, P, q, lb, ub, x, 800, &w, &iters);
  CHECK(st == BMPC_OK, "status %d", st);
  CHECK(bmpc_boxqp_kkt_residual(4, P, q, lb, ub, x) < (bmpc_real)1e-3, "kkt");
  CHECK(x[2] < (bmpc_real)0, "x2 %g", (double)x[2]);
}

static void test_zero_step_batch(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: zero-length steps fixed together]\n");
  {
    /* 앞 변수만 움직인다: 기울기 정확히 0 */
    static const signed char mv[6] = {1, 0, 0, 0, 0, 0};
    set_2i11(p, 6, mv, NULL, R(0), x0);
    check_batch(p, x0, mv, NULL, "front mover, g = 0");
    /* 기울기가 안쪽이어도 Newton 스텝이 바깥이면 같다: d_i = 0.005 - 1.05/16 < 0 */
    set_2i11(p, 6, mv, NULL, R(0.01), x0);
    check_batch(p, x0, mv, NULL, "front mover, inward g");
  }
  {
    /* 뒤쪽 변수가 움직인다: 빠지는 위치가 남는 변수보다 앞이라 chol_remove 가 실제로 갱신한다 */
    static const signed char mv[8] = {0, 0, 0, 1, 1, 1, 1, 1};
    set_2i11(p, 8, mv, NULL, R(0), x0);
    check_batch(p, x0, mv, NULL, "back movers");
  }
  {
    /* 번갈아: 앞에서부터 빼면 위치와 d 가 어긋난다 */
    static const signed char mv[9] = {0, 1, 0, 1, 0, 1, 0, 1, 0};
    set_2i11(p, 9, mv, NULL, R(0), x0);
    check_batch(p, x0, mv, NULL, "interleaved");
  }
  {
    /* 하한과 상한이 섞인다: 부호를 뒤집은 같은 문제 */
    static const signed char mv[7] = {0, 1, 0, 0, 1, 0, 0};
    static const signed char sg[7] = {1, -1, -1, 1, 1, -1, 1};
    set_2i11(p, 7, mv, sg, R(0), x0);
    check_batch(p, x0, mv, sg, "both sides");
  }
  /* 2x2 손 계산: 상한 2 에서 해 (1.5, 0) */
  set2(p, 2, 1, 2, -3, 0, 0, 10, 0, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && NEAR(p->x[0], 1.5) && p->x[1] == 0, "2x2 cap 2: st %d x %g %g", st, (double)p->x[0], (double)p->x[1]);
  /* 같은 판정: 반올림 크기로 미는 변수는 함께 고정하지 않는다 (x + d 가 실제로 넘을 때만). a 는 상한, d_a 는 약 1e-20, b 는 진짜 바깥 */
  prob_begin(p, 3);
  {
    static const double PP[9] = {1, 0, -1e-20, 0, 2, 1, -1e-20, 1, 2};
    int i;
    for (i = 0; i < 9; ++i) p->P[i] = R(PP[i]);
    p->q[0] = R(-1);
    p->q[1] = 0;
    p->q[2] = R(-1);
    p->lb[0] = R(-5);
    p->ub[0] = R(1);
    p->lb[1] = 0;
    p->ub[1] = R(10);
    p->lb[2] = R(-10);
    p->ub[2] = R(10);
    x0[0] = R(1);
    x0[1] = 0;
    x0[2] = 0;
  }
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && g_w.ws[0] == 0 && g_w.ws[1] == -1 && g_w.ws[2] == 0, "rounding push: ws %d %d %d", g_w.ws[0], g_w.ws[1], g_w.ws[2]);
  /* 거울상: a 는 하한, d_a 는 약 -1e-20 */
  p->P[2] = p->P[6] = R(1e-20);
  p->q[0] = R(1);
  p->lb[0] = R(-1);
  p->ub[0] = R(5);
  x0[0] = R(-1);
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && g_w.ws[0] == 0 && g_w.ws[1] == -1 && g_w.ws[2] == 0, "rounding push, lower: ws %d %d %d", g_w.ws[0], g_w.ws[1], g_w.ws[2]);
  /* 한꺼번에 빼도 인수를 버리지 않는다: 다음 풀이가 재분해면 이 경로의 시간 근거가 사라진다. 갱신한 인수는 새 분해와 비트까지 같지 않다 */
  {
    static const signed char mv[11] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1};
    set_2i11(p, 11, mv, NULL, R(0), x0);
    st = solve(p, x0, 2, &it);
    CHECK(st == BMPC_ERR_MAXITER && !factor_is_fresh(p), "batch kept the updated factor: st %d fresh %d", st, factor_is_fresh(p));
  }
}

/* [고정] 처음 working set (bmpc_qp.c:136): 상한 0 으로 ws 를 직접 본다. */
static void test_initial_working_set(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st, i;
  /* P = I, x0 를 경계에 두고 q 로 기울기를 정한다. 변수별 기대 ws */
  static const struct {
    double x0, l, u, g;
    int ws;
  } C[] = {
      {0, 0, 1, 2, -1},   /* 하한, 기울기 양수: 바깥 */
      {0, 0, 1, -2, 0},   /* 하한, 기울기 음수: 안쪽 */
      {0, 0, 1, 0, 0},    /* 하한, 기울기 0: 바깥이 아니다 */
      {1, 0, 1, -2, 1},   /* 상한, 기울기 음수: 바깥 */
      {1, 0, 1, 2, 0},    /* 상한, 기울기 양수: 안쪽 */
      {1, 0, 1, 0, 0},    /* 상한, 기울기 0 */
      {.5, 0, 1, 2, 0},   /* 내부 */
      {.5, 0, 1, -2, 0},  /* 내부 */
      {3, 3, 3, 2, -1},   /* lb == ub, 기울기 무관 */
      {3, 3, 3, -2, -1},  /* lb == ub */
  };
  enum { N = sizeof(C) / sizeof(C[0]) };
  printf("[qp: initial working set]\n");
  prob_begin(p, N);
  for (i = 0; i < N * N; ++i) p->P[i] = 0;
  for (i = 0; i < N; ++i) {
    p->P[i * N + i] = 1;
    p->q[i] = R(C[i].g - C[i].x0);
    p->lb[i] = R(C[i].l);
    p->ub[i] = R(C[i].u);
    x0[i] = R(C[i].x0);
  }
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 0, "cap 0: st %d it %d", st, it);
  for (i = 0; i < N; ++i) CHECK(g_w.ws[i] == C[i].ws, "case %d: ws %d want %d", i, g_w.ws[i], C[i].ws);
  {
    int nf = 0;
    for (i = 0; i < N; ++i)
      if (C[i].ws == 0) {
        CHECK(g_w.free_idx[nf] == i, "free_idx[%d]=%d want %d", nf, g_w.free_idx[nf], i);
        ++nf;
      }
  }

  /* 경계에서 바깥을 향하는 변수가 고정되지 않으면 첫 반복이 통째로 사라진다: P=I, q=(1,-1), x0=(0,0), 박스 [0,10] */
  set2(p, 1, 0, 1, 1, -1, 0, 10, 0, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == 0 && p->x[1] == R(1), "outward lower: x %g %g", (double)p->x[0], (double)p->x[1]);
  /* 상한 쪽 */
  set2(p, 1, 0, 1, -11, -1, 0, 10, 0, 10);
  x0[0] = R(10);
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(10) && p->x[1] == R(1), "outward upper: x %g %g", (double)p->x[0], (double)p->x[1]);
}

/* ---------------- 3. warm start ---------------- */
static void test_warm_start(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  double xc[7];
  int it, st, i, trial;
  printf("[qp: warm start]\n");

  /* 내부 해에서 시작: d = 0, 첫 반복에 끝난다 */
  set2(p, 1, 0, 1, -1, 2, -5, 5, -5, 5);
  x0[0] = R(1);
  x0[1] = R(-2);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 1 && p->x[0] == R(1) && p->x[1] == R(-2), "at interior optimum: st %d it %d", st, it);

  /* 경계 해에서 시작: x2 는 바깥 기울기로 고정, x1 은 d = 0 */
  set2(p, 1, 0, 1, -1, 2, -5, 5, 0, 5);
  x0[0] = R(1);
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 1 && p->x[0] == R(1) && p->x[1] == 0, "at bound optimum: st %d it %d", st, it);

  /* 범위 밖 시작점은 클램프된다. 상한 0 에서 값으로 확인 */
  set2(p, 1, 0, 1, -1, 2, -5, 5, -5, 5);
  x0[0] = R(100);
  x0[1] = R(-100);
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(5) && p->x[1] == R(-5), "clamp: x %g %g", (double)p->x[0], (double)p->x[1]);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && NEAR(p->x[0], 1) && NEAR(p->x[1], -2), "outside start solves: st %d", st);

  /* 유한하지 않은 시작점: 범위 가운데 (bmpc_qp.c:122). lb, ub 가 대칭이 아니어야 합과 차를 구별한다. */
  set2(p, 1, 0, 1, 0, 0, 2, 6, -8, -2);
  x0[0] = (bmpc_real)NAN;
  x0[1] = (bmpc_real)INFINITY;
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(4) && p->x[1] == R(-5), "midpoint: x %g %g", (double)p->x[0], (double)p->x[1]);
  x0[0] = (bmpc_real)-INFINITY;
  x0[1] = R(-3); /* 유한한 쪽은 그대로 */
  st = solve(p, x0, 0, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(4) && p->x[1] == R(-3), "mixed start: x %g %g", (double)p->x[0], (double)p->x[1]);
  x0[0] = (bmpc_real)NAN;
  x0[1] = (bmpc_real)NAN;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && p->x[0] == R(2) && p->x[1] == R(-2), "NaN start solves: st %d x %g %g", st, (double)p->x[0], (double)p->x[1]);

  /* 임의 시작점: P 양정치면 해가 유일하다. 어떤 시작점에서든 같은 해. */
  for (trial = 0; trial < 240; ++trial) {
    const int n = 1 + trial % 6;
    double lt;
    gen_prob(p, n, 6.0, (trial % 4 == 0) ? 0.2 : 0.0);
    lt = lambda_scale(p);
    enumerate_solution(p, xc);
    gen_x0(p, x0, trial % 4);
    if (trial % 8 == 5) x0[trial % n] = (bmpc_real)NAN;
    st = solve(p, x0, 1000, &it);
    CHECK(st == BMPC_OK, "warm trial %d: status %d", trial, st);
    for (i = 0; i < n; ++i)
      CHECK(fabs((double)p->x[i] - xc[i]) <= 2.0 * lt + 8 * EPS * (1.0 + fabs(xc[i])), "warm trial %d x[%d]=%.9g want %.9g", trial, i,
            (double)p->x[i], xc[i]);
  }
}

/* ---------------- 4. 퇴화와 잘못된 입력 ---------------- */
static void test_fixed_variables(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st, i;
  printf("[qp: lb == ub]\n");

  /* 전부 고정: 첫 반복에 끝, x = lb, q 와 무관 */
  set2(p, 2, 1, 2, 100, -100, 1, 1, -2, -2);
  x0[0] = R(9);
  x0[1] = R(9);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 1 && p->x[0] == R(1) && p->x[1] == R(-2), "all fixed: st %d it %d", st, it);

  /* 경계에 묶여 전부 working set: 첫 반복에 끝 */
  set2(p, 1, 0, 1, 1, 1, 0, 1, 0, 1);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 1, "all at bound: st %d it %d", st, it);

  /* x1 = 1 고정, x2 만 자유. P=[2,1;1,2], q=(-5,0): x2 = -0.5. x1 의 승수 g1 = 2 - 0.5 - 5 = -3.5 는 음수지만 풀면 안 된다 */
  set2(p, 2, 1, 2, -5, 0, 1, 1, -10, 10);
  x0[0] = R(1);
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 2 && p->x[0] == R(1) && NEAR(p->x[1], -0.5), "fixed with negative multiplier: st %d it %d x %g %g", st, it,
        (double)p->x[0], (double)p->x[1]);
  /* 승수 부호 반대 */
  set2(p, 2, 1, 2, 5, 0, 1, 1, -10, 10);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 2 && p->x[0] == R(1) && NEAR(p->x[1], -0.5), "fixed with positive multiplier: st %d it %d", st, it);

  /* 조밀한 P 에서 고정 변수를 뺀 축소 문제와 같은 해 */
  {
    enum { N = 8 };
    static prob_t r;
    bmpc_real xr[NMAX + 2];
    int j, a, b;
    gen_prob(p, N, 5.0, 0.0);
    p->lb[2] = p->ub[2] = R(0.25);
    p->lb[5] = p->ub[5] = R(-0.5);
    for (i = 0; i < N; ++i) x0[i] = 0;
    st = solve(p, x0, 200, &it);
    CHECK(st == BMPC_OK && p->x[2] == R(0.25) && p->x[5] == R(-0.5), "dense fixed: st %d", st);
    prob_begin(&r, N - 2);
    for (i = 0, a = 0; i < N; ++i) {
      double qq;
      if (i == 2 || i == 5) continue;
      qq = (double)p->q[i] + 0.25 * (double)p->P[i * N + 2] - 0.5 * (double)p->P[i * N + 5];
      r.q[a] = R(qq);
      r.lb[a] = p->lb[i];
      r.ub[a] = p->ub[i];
      xr[a] = 0;
      for (j = 0, b = 0; j < N; ++j) {
        if (j == 2 || j == 5) continue;
        r.P[a * (N - 2) + b++] = p->P[i * N + j];
      }
      ++a;
    }
    {
      static bmpc_real keep[NMAX + 2];
      double lt = lambda_scale(p);
      memcpy(keep, p->x, sizeof(keep));
      st = solve(&r, xr, 200, &it);
      CHECK(st == BMPC_OK, "reduced: st %d", st);
      for (i = 0, a = 0; i < N; ++i) {
        if (i == 2 || i == 5) continue;
        CHECK(fabs((double)keep[i] - (double)r.x[a]) <= 4.0 * lt, "reduced x[%d] %.9g vs %.9g", i, (double)keep[i], (double)r.x[a]);
        ++a;
      }
    }
  }
}

static void test_invalid_input(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  const bmpc_real nan_ = (bmpc_real)NAN, inf_ = (bmpc_real)INFINITY;
  int it, st, i;
  printf("[qp: invalid input]\n");

  /* n 범위: NUMERIC, iters 0, x 그대로 (bmpc_qp.c:119) */
  prob_begin(p, 1);
  p->P[0] = 1;
  p->q[0] = 0;
  p->lb[0] = R(-1);
  p->ub[0] = R(1);
  {
    static const int bad_n[] = {0, -1, NMAX + 1, NMAX + 100};
    for (i = 0; i < 4; ++i) {
      bmpc_real xs = R(77);
      it = -7;
      st = bmpc_boxqp_solve(bad_n[i], p->P, p->q, p->lb, p->ub, &xs, 10, &g_w, &it);
      CHECK(st == BMPC_ERR_NUMERIC && it == 0 && xs == R(77), "n=%d: st %d it %d x %g", bad_n[i], st, it, (double)xs);
    }
  }

  /* lb > ub: INFEASIBLE, iters 0 */
  set2(p, 1, 0, 1, 0, 0, 2, 1, 0, 1);
  x0[0] = R(7);
  x0[1] = R(8);
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_ERR_INFEASIBLE && it == 0, "lb>ub first: st %d it %d", st, it);
  /* [고정] 모순을 만나기 전 칸만 클램프되고 나머지는 그대로 (bmpc_qp.c:120-123). x 가 망가지지는 않는다 */
  CHECK(p->x[0] == R(7) && p->x[1] == R(8), "lb>ub first: x %g %g", (double)p->x[0], (double)p->x[1]);
  set2(p, 1, 0, 1, 0, 0, 0, 1, 2, 1);
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_ERR_INFEASIBLE && it == 0, "lb>ub second: st %d it %d", st, it);
  CHECK(p->x[0] == R(1) && p->x[1] == R(8), "lb>ub second: x %g %g", (double)p->x[0], (double)p->x[1]);
  x0[0] = 0;
  x0[1] = 0;
  /* 경계의 NaN 도 같은 경로 (bmpc_qp.c:121). bmpc_types.h:35 의 "NaN -> NUMERIC" 와는 다르다: 보고서의 발견. 여기서는 코드대로. */
  set2(p, 1, 0, 1, 0, 0, 0, 1, 0, 1);
  p->lb[1] = nan_;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_ERR_INFEASIBLE && it == 0, "NaN lb: st %d", st);
  set2(p, 1, 0, 1, 0, 0, 0, 1, 0, 1);
  p->ub[0] = nan_;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_ERR_INFEASIBLE && it == 0, "NaN ub: st %d", st);

  /* q, P 의 NaN/Inf: 자유 변수에 닿으면 NUMERIC, 정의된 상태로 끝난다. [고정] x 는 시작점 그대로: 잘못된 데이터가 x 에 번지지 않는다. */
  {
    static const struct {
      int where; /* 0 q0, 1 P00, 2 P01(양쪽), 3 P11 */
      int val;   /* 0 NaN, 1 +Inf, 2 -Inf */
    } C[] = {{0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2}};
    for (i = 0; i < (int)(sizeof(C) / sizeof(C[0])); ++i) {
      const bmpc_real v = C[i].val == 0 ? nan_ : (C[i].val == 1 ? inf_ : -inf_);
      set2(p, 1, 0.25, 1, -1, -1, -5, 5, -5, 5);
      if (C[i].where == 0) p->q[0] = v;
      if (C[i].where == 1) p->P[0] = v;
      if (C[i].where == 2) p->P[1] = p->P[2] = v;
      if (C[i].where == 3) p->P[3] = v;
      x0[0] = R(1);
      x0[1] = R(1);
      st = solve(p, x0, 1000, &it);
      CHECK(st == BMPC_ERR_NUMERIC, "bad data case %d: st %d", i, st);
      CHECK(it >= 1 && it <= 1000, "bad data case %d: iters %d", i, it);
      CHECK(p->x[0] == R(1) && p->x[1] == R(1), "bad data case %d: x %g %g", i, (double)p->x[0], (double)p->x[1]);
    }
  }

  /* 부정치: 첫 분해에서 NUMERIC */
  set2(p, 1, 2, 1, 0, 0, -1, 1, -1, 1);
  x0[0] = 0;
  x0[1] = 0;
  x0[0] = R(0.25);
  x0[1] = R(-0.5);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC && it == 1, "indefinite: st %d it %d", st, it);
  CHECK(p->x[0] == R(0.25) && p->x[1] == R(-0.5), "indefinite: x moved %g %g", (double)p->x[0], (double)p->x[1]);
  x0[0] = 0;
  x0[1] = 0;
  /* 음의 대각 */
  set2(p, -1, 0, 1, 0, 0, -1, 1, -1, 1);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC && it == 1, "negative diagonal: st %d it %d", st, it);
  /* 특이: 정확히 0 인 피벗 (bmpc_qp.c:13) */
  set2(p, 1, 1, 1, 0, 0, -1, 1, -1, 1);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC && it == 1, "singular: st %d it %d", st, it);
  /* P = 0, 자유 변수 */
  set2(p, 0, 0, 0, 1, 1, -1, 1, -1, 1);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC && it == 1, "zero P: st %d it %d", st, it);

  /* 부분 문제까지는 양정치, 풀어 넣으면 부정치: chol_append 실패 뒤 재분해에서 NUMERIC (bmpc_qp.c:188, 104, 156).
   * P=[1,-2;-2,1], q=(-1,0.5): x2 는 하한 고정, x1 -> 1, g2 = -1.5 라 풀고, dd = 1 - 4 < 0. */
  set2(p, 1, -2, 1, -1, 0.5, -10, 10, 0, 10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC && it == 3, "indefinite after release: st %d it %d", st, it);
  CHECK(p->x[0] == R(1) && p->x[1] == 0, "x stays at the last good point: %g %g", (double)p->x[0], (double)p->x[1]);
  /* 상한 2 에서는 아직 문제가 안 드러난다. 풀기 전까지 x 는 정확하고 상태는 MAXITER */
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && it == 2, "indefinite, cap before it shows: st %d it %d", st, it);
  /* 풀어 넣을 때 정확히 0 (dd == 0, 특이): P=[1,-1;-1,1], q=(-1,.5) */
  set2(p, 1, -1, 1, -1, 0.5, -10, 10, 0, 10);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC && it == 3, "singular after release: st %d it %d", st, it);

  /* 고정 변수의 q 가 NaN 이어도 쓰이지 않는다: 정의된 상태 OK */
  set2(p, 1, 0, 1, 0, -1, 1, 1, -5, 5);
  p->q[0] = nan_;
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && p->x[0] == R(1) && p->x[1] == R(1), "NaN q on a fixed variable: st %d", st);

  /* 무한 경계의 시작점: 양쪽 무한 + NaN 시작은 가운데가 NaN 이라 NUMERIC 으로 끝난다. 상태만 본다.
   * 이때 x[0] 은 -inf 로 남는다 (유한하지 않음): after_qp/repro_inf_box_nan_start.c 에 따로 둔다. */
  set2(p, 1, 0, 1, -1, -1, 0, 1, 0, 1);
  p->lb[0] = -inf_;
  p->ub[0] = inf_;
  x0[0] = nan_;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_ERR_NUMERIC, "infinite box NaN start: st %d", st);
  CHECK(it >= 1 && it <= 100, "infinite box: iters %d", it);
}

/* 양정치인 P 인데 chol_append 만 반올림으로 dd == 0 을 보는 경우: 잘못된 NUMERIC 대신 재분해로 복구한다 (bmpc_qp.c:95, 104, 188).
 * 재분해는 역수를 곱하고(:16-17) 갱신은 나누므로(:102) 반올림이 달라 s > 0 이 된다. a, b 는 임의 탐색으로 찾았고,
 * det(P) > 0 은 그 탐색에서 __float128 로 확인했다. c 는 dd 가 정확히 0 이 되도록 시험 안에서 계산한다.
 * 해: x2 는 상한 10, x1 = (1 - 10 b) / a, g2 = 0.25 + b x1 + 10 c < 0. */
static void test_append_zero_pivot(void)
{
#ifdef BMPC_USE_FLOAT
  const bmpc_real a = (bmpc_real)0x1.b2146ap+1f, b = (bmpc_real)-0x1.2f0254p+0f;
#else
  const bmpc_real a = (bmpc_real)0x1.3171b5p+1, b = (bmpc_real)-0x1.afcd6ap+0;
#endif
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  volatile bmpc_real l, r1, r2, c, dd, s, inv, sq;
  int it, st;
  double x1;
  printf("[qp: rounding-size zero pivot in chol_append]\n");
  /* 제곱을 volatile 에 먼저 담는다. 보드 플래그(VFMA, gnu99 기본 contraction)에서 c - r1 * r1 이 한 번에 계산되면 dd 가 반올림 오차가 된다. */
  l = BM_SQRT(a);
  r1 = b / l;
  sq = r1 * r1;
  c = sq;
  dd = c - sq;
  inv = (bmpc_real)1.0 / l;
  r2 = b * inv;
  sq = r2 * r2;
  s = c - sq;
  CHECK(dd == 0 && s > 0, "setup: dd %g s %g", (double)dd, (double)s);
  prob_begin(p, 2);
  p->P[0] = a;
  p->P[1] = b;
  p->P[2] = b;
  p->P[3] = c;
  p->q[0] = R(-1);
  p->q[1] = R(0.25);
  p->lb[0] = R(-10);
  p->ub[0] = R(10);
  p->lb[1] = 0;
  p->ub[1] = R(10);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  x1 = (1.0 - 10.0 * (double)b) / (double)a;
  CHECK(st == BMPC_OK, "positive definite P must not end in NUMERIC: st %d", st);
  CHECK(p->x[1] == R(10) && fabs((double)p->x[0] - x1) <= 2e3 * EPS * (1.0 + x1), "x %.9g %.9g, want %.9g 10", (double)p->x[0],
        (double)p->x[1], x1);
  CHECK(kkt_ok(p, p->x, 1.5 * lambda_scale(p)), "KKT");
}

/* n 한계: BMPC_NZ_MAX 는 받아야 하고 작업 메모리 안에서 끝나야 한다. */
static void test_size_limit(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int i, it, st;
  static const int NS[] = {NMAX, NMAX - 1};
  int s;
  printf("[qp: n at the limit]\n");
  for (s = 0; s < 2; ++s) {
    const int n = NS[s];
    /* 대각 지배로 조건수를 낮게 둔다 (float 도 같은 시험을 통과하도록) */
    prob_begin(p, n);
    gen_P(p, n, 0.2, 6.0);
    for (i = 0; i < n; ++i) {
      p->q[i] = R(rnd2(-40.0, 40.0));
      p->lb[i] = R(-1.0 - 0.01 * i);
      p->ub[i] = R(1.0 + 0.02 * i);
      x0[i] = 0;
    }
    sweep_problem(p, x0, 18, "nmax");
    CHECK(g_ref.K > 18, "n=%d needs more than the largest grid cap (K %d)", n, g_ref.K);
    st = solve(p, x0, 1000, &it);
    CHECK(st == BMPC_OK && kkt_ok(p, p->x, 1.5 * lambda_scale(p)), "n=%d: st %d, KKT", n, st);
    check_factor(p, "nmax");
    /* 경계 시작, 전부 풀어 넣는 경로도 */
    for (i = 0; i < n; ++i) x0[i] = p->lb[i];
    sweep_problem(p, x0, 10, "nmax-bound");
  }
}

/* ---------------- 5. 경계 ---------------- */
/* 스텝 허용오차 (bmpc_qp.c:166): dmax <= step_tol (1 + max|x|) 면 이미 부분 문제의 최소점이다. */
static void test_step_tolerance(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: step tolerance]\n");
  prob_begin(p, 1);
  p->P[0] = 1;
  p->lb[0] = R(-10);
  p->ub[0] = R(10);

  /* x = 0: 눈금 1. 정확히 같으면 멈춘다 (<=). */
  x0[0] = 0;
  p->q[0] = -STEP_TOL;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 1 && p->x[0] == 0, "d == tol: st %d it %d x %g", st, it, (double)p->x[0]);
  p->q[0] = R(-2) * STEP_TOL;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 2 && p->x[0] == R(2) * STEP_TOL, "d == 2 tol: st %d it %d x %g", st, it, (double)p->x[0]);

  /* x = 3: 눈금 1 + 3 = 4. d = 2 tol 은 멈추고 d = 8 tol 은 움직인다. */
  x0[0] = R(3);
  p->q[0] = R(-3) - R(2) * STEP_TOL;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 1 && p->x[0] == R(3), "x=3, d=2tol: st %d it %d x %g", st, it, (double)p->x[0]);
  p->q[0] = R(-3) - R(8) * STEP_TOL;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 2 && p->x[0] > R(3), "x=3, d=8tol: st %d it %d x %g", st, it, (double)p->x[0]);

  /* 눈금의 |x| 는 지금 반복의 x 다 (bmpc_qp.c:150), 시작점이 아니다. P = I, x0 = 0, 변수 0 은 d = 6 으로 상한 3 에 비율 1/2 로 막힌다.
   * 변수 1 은 d = 4 tol 이라 반만 가서 x1 = 2 tol, 남은 d = 2 tol. 이제 눈금은 1 + 3 = 4 라 멈춘다: OK, 반복 2. */
  set2(p, 1, 0, 1, -6, 0, -10, 3, -10, 10);
  p->q[1] = R(-4) * STEP_TOL;
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 2 && p->x[0] == R(3) && p->x[1] == R(2) * STEP_TOL, "scale after a step: st %d it %d x %g %g", st, it, (double)p->x[0], (double)p->x[1]);

  /* 눈금의 |x| 는 고정 변수까지 포함한다 (bmpc_qp.c:150). x1 = 0, 고정 변수 |x2| = 3 이라 눈금 4: d = 2 tol 은 멈춘다.
   * 스텝을 넣으면 안 되는 쪽이므로 it == 1, x1 그대로. 세 가지로 고정: 경계 고정 하한, 경계 고정 상한, lb == ub. */
  {
    static const struct {
      double l, u, q;
      const char *name;
    } C[] = {{3, 5, 1, "lower bound"}, {-5, -3, -1, "upper bound"}, {-3, -3, 0, "lb == ub"}};
    int c;
    for (c = 0; c < 3; ++c) {
      set2(p, 1, 0, 1, 0, C[c].q, -10, 10, C[c].l, C[c].u);
      p->q[0] = R(-2) * STEP_TOL;
      x0[0] = 0;
      x0[1] = C[c].q > 0 ? R(C[c].l) : R(C[c].u);
      st = solve(p, x0, 10, &it);
      CHECK(st == BMPC_OK && it == 1 && p->x[0] == 0, "%s: d = 2 tol must stop, st %d it %d x %g", C[c].name, st, it, (double)p->x[0]);
    }
  }
}

/* 승수 허용오차 (bmpc_qp.c:133): 변수 1 이 하한에 고정된 채 x0 가 움직여 승수가 -delta 가 된다.
 * delta = 0.5 tol 이면 풀지 않고 (OK, iters 2), 2 tol 이면 푼다. 변수 2 가 눈금의 한 항만 크게 만든다.
 * 눈금의 약속은 주석뿐이므로 [고정]: float 빌드에서 이 눈금이 해의 정확도를 정한다. */
static void test_lambda_tolerance(void)
{
  static const struct {
    double p2, q2, x2, l2, u2; /* 변수 2: 대각, q, 시작, 범위 */
    const char *name;
    double p1; /* 변수 1 의 대각 */
  } C[] = {
      {1, 0, 0, -2, 2, "base", 1},
      {1, -2000, 2, -2, 2, "qmax", 1},      /* x2 는 상한에 바깥 기울기로 고정 */
      {1000, 0, 0, -2, 2, "pmax", 1},       /* pmax xmax = 2000, 마지막 대각에만 있다 */
      {1, 0, 0, -2000, 2000, "xmax", 1},
      {1, 0, 0, -2000, 2, "xmax lb only", 1},   /* 눈금은 |lb| 와 |ub| 를 모두 본다. MPC 박스는 비대칭이다 */
      {1, 0, 0, -2, 2000, "xmax ub only", 1},
      {1, -2000, 2, 2, 2, "qmax lb == ub", 1},  /* 고정 변수의 q 도 눈금에 든다 */
      {1, 0, 0, -2, 2, "real pmax 3e5", 3e5},   /* MPC QP 의 눈금 (pmax 약 3e5). 큰 눈금에서만 달라지는 구현을 잡는다 */
      {1, 0, 0, -2, 2, "real pmax 1.5e6", 1.5e6}, /* highway QP 의 눈금 */
  };
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int c, it, st;
  printf("[qp: multiplier tolerance]\n");
  for (c = 0; c < (int)(sizeof(C) / sizeof(C[0])); ++c) {
    int k;
    for (k = 0; k < 2; ++k) {
      const double mult = k == 0 ? 0.5 : 2.0;
      double tol, delta, rho;
      int a;
      prob_begin(p, 3);
      for (a = 0; a < 9; ++a) p->P[a] = 0;
      p->P[0] = 1;
      p->P[4] = R(C[c].p1);
      p->P[8] = R(C[c].p2);
      p->q[0] = R(-1);
      p->q[1] = R(0.25);
      p->q[2] = R(C[c].q2);
      p->lb[0] = R(-2);
      p->ub[0] = R(2);
      p->lb[1] = 0;
      p->ub[1] = R(2);
      p->lb[2] = R(C[c].l2);
      p->ub[2] = R(C[c].u2);
      tol = lambda_scale(p);
      delta = mult * tol;
      rho = -(0.25 + delta);
      CHECK(rho * rho < C[c].p1, "%s: test setup, rho %g must stay positive definite", C[c].name, rho);
      p->P[1] = p->P[3] = R(rho);
      x0[0] = 0;
      x0[1] = 0;
      x0[2] = R(C[c].x2);
      st = solve(p, x0, 50, &it);
      CHECK(st == BMPC_OK, "%s x%g: status %d", C[c].name, mult, st);
      if (k == 0) {
        CHECK(it == 2 && g_w.ws[1] == -1 && p->x[1] == 0, "%s: 0.5 tol must not release, it %d ws %d", C[c].name, it, g_w.ws[1]);
      } else {
        CHECK(it > 2 && g_w.ws[1] == 0 && p->x[1] > 0, "%s: 2 tol must release, it %d ws %d x %g", C[c].name, it, g_w.ws[1], (double)p->x[1]);
      }
    }
  }
}

/* [고정] 경계에서 반올림 크기로 바깥을 미는 방향이 스텝 전체를 막으면 안 된다 (bmpc_qp.c:205, 213: x + d 가 실제로 넘을 때만 막힘).
 * x1 은 경계, 결합 1e-20 때문에 d1 = 1e-20 > 0. x1 + d1 == 경계. x2 는 온전히 움직여야 한다. */
static void test_rounding_push(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: rounding-size push at a bound]\n");
  set2(p, 1, -1e-20, 1, -1, -1, -5, 1, -10, 10);
  x0[0] = R(1);
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(1) && NEAR(p->x[1], 1.0) && g_w.ws[0] == 0, "upper: x %g %g ws %d", (double)p->x[0],
        (double)p->x[1], g_w.ws[0]);
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 2, "upper K: st %d it %d", st, it);

  set2(p, 1, -1e-20, 1, 1, 1, -1, 5, -10, 10);
  x0[0] = R(-1);
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(-1) && NEAR(p->x[1], -1.0) && g_w.ws[0] == 0, "lower: x %g %g ws %d", (double)p->x[0],
        (double)p->x[1], g_w.ws[0]);
  st = solve(p, x0, 10, &it);
  CHECK(st == BMPC_OK && it == 2, "lower K: st %d it %d", st, it);
}

/* [고정] 동률: 두 변수가 같은 비율에서 막히면 F 앞쪽을 고정한다. 어느 쪽을 고르느냐에 따라 상한 2 의 x 가 다르다.
 * P=[1,-3/4;-3/4,13/16] 는 L=[1;-3/4,1/2] 라 계산이 정확하다. d=(2,4), 상한 (1,2): 비율 1/2 에서 둘 다 닿고 g=(1/2,-7/8).
 * x1 을 고정하면 x2 는 다시 막혀 x 가 그대로, x2 를 고정했다면 x1 이 0.5 로 움직인다. */
static void test_block_ties(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: ties]\n");
  set2(p, 1, -0.75, 0.8125, 1, -1.75, -1, 1, -1, 2);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(1) && p->x[1] == R(2) && g_w.ws[0] == 1 && g_w.ws[1] == 0, "upper tie cap 1: ws %d %d",
        g_w.ws[0], g_w.ws[1]);
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(1) && p->x[1] == R(2), "upper tie cap 2: x %g %g", (double)p->x[0], (double)p->x[1]);
  st = solve(p, x0, 100, &it);
  /* 해: x2 = 2 에서 x1 = 3/4*2 - 1 = 0.5, g2 = -1.75 - 0.375 + 1.625 = -0.5 < 0 */
  CHECK(st == BMPC_OK && NEAR(p->x[0], 0.5) && p->x[1] == R(2), "upper tie solution: st %d x %g %g", st, (double)p->x[0], (double)p->x[1]);

  set2(p, 1, -0.75, 0.8125, -1, 1.75, -1, 1, -2, 1);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 1, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(-1) && p->x[1] == R(-2) && g_w.ws[0] == -1 && g_w.ws[1] == 0, "lower tie cap 1: ws %d %d",
        g_w.ws[0], g_w.ws[1]);
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && p->x[0] == R(-1) && p->x[1] == R(-2), "lower tie cap 2: x %g %g", (double)p->x[0], (double)p->x[1]);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && NEAR(p->x[0], -0.5) && p->x[1] == R(-2), "lower tie solution: st %d x %g %g", st, (double)p->x[0], (double)p->x[1]);
}

/* [고정] 승수 동률: 같은 승수면 앞 변수를 먼저 푼다 (bmpc_qp.c:179). 어느 쪽이냐에 따라 상한에서 끊긴 x 가 다르다.
 * P=[1,0,-1;0,1,-1;-1,-1,4], q=(.5,.5,-4). x3 -> 1 (L = 2 라 정확), g1 = g2 = -0.5. */
static void test_multiplier_tie(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  static const double PP[9] = {1, 0, -1, 0, 1, -1, -1, -1, 4};
  int it, st, i;
  printf("[qp: multiplier tie]\n");
  prob_begin(p, 3);
  for (i = 0; i < 9; ++i) p->P[i] = R(PP[i]);
  p->q[0] = R(0.5);
  p->q[1] = R(0.5);
  p->q[2] = R(-4);
  for (i = 0; i < 3; ++i) {
    p->lb[i] = 0;
    p->ub[i] = R(10);
    x0[i] = 0;
  }
  st = solve(p, x0, 2, &it);
  CHECK(st == BMPC_ERR_MAXITER && g_w.ws[0] == 0 && g_w.ws[1] == -1 && p->x[2] == R(1), "tie: st %d ws %d %d", st, g_w.ws[0], g_w.ws[1]);
  st = solve(p, x0, 100, &it);
  /* 해: 전부 자유. x1 = x2 = x3 - 0.5, 4 x3 - x1 - x2 = 4 에서 x3 = 1.5, x1 = x2 = 1 */
  CHECK(st == BMPC_OK && NEAR(p->x[0], 1.0) && NEAR(p->x[1], 1.0) && NEAR(p->x[2], 1.5), "tie solution: st %d x %g %g %g", st,
        (double)p->x[0], (double)p->x[1], (double)p->x[2]);
}

/* 인수 갱신이 실패해도 재분해로 복구한다 (bmpc_qp.c:62-63, 246, 249-250). 양정치 P 에서 이 길은 대각이 표현 가능한 최댓값일 때만
 * 열린다: lkk^2 + x^2 가 반올림으로 무한대가 되고 다음 회전에서 NaN. 이 시드들은 double, float 양쪽에서 그 길을 지나는 것을
 * 계측한 사본으로 확인했다. 시험은 결과만 본다: 기준기와 같은 반복점, status OK, KKT. 다른 빌드 옵션에서 이 길을 안 지나도 통과한다. */
static void test_remove_failure_recovery(void)
{
  static const unsigned SEEDS[] = {4, 17, 20, 29, 45, 51, 73, 143};
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  const double rmax = (double)BMPC_REAL_MAX;
  const unsigned keep = g_seed;
  unsigned s;
  printf("[qp: recovery after a failed factor update]\n");
  for (s = 0; s < sizeof(SEEDS) / sizeof(SEEDS[0]); ++s) {
    enum { N = 6 };
    int i, j, c, it, st;
    g_seed = SEEDS[s];
    prob_begin(p, N);
    for (i = 0; i < N; ++i) {
      p->P[i * N + i] = (bmpc_real)BMPC_REAL_MAX;
      for (j = 0; j < i; ++j) {
        const bmpc_real v = R(rnd2(-0.15, 0.15) * rmax);
        p->P[i * N + j] = v;
        p->P[j * N + i] = v;
      }
      p->q[i] = R(rnd2(-1e-3, 1e-3) * rmax);
      p->lb[i] = R(-5e-4);
      p->ub[i] = R(5e-4);
      x0[i] = 0;
    }
    ref_run(p, x0);
    for (c = 1; c <= g_ref.K; ++c) {
      double e = 0;
      st = solve(p, x0, c, &it);
      CHECK(st == (c < g_ref.K ? BMPC_ERR_MAXITER : g_ref.status) && it == c, "seed %u cap %d: st %d it %d K %d", SEEDS[s], c, st, it, g_ref.K);
      for (i = 0; i < N; ++i) e = fmax(e, fabs((double)p->x[i] - g_ref.xs[c][i]));
      /* 박스 폭 1e-3 기준의 반올림 */
      CHECK(e <= 2e3 * EPS * 5e-4, "seed %u cap %d: x off the reference by %g", SEEDS[s], c, e);
    }
    /* 자유 변수의 기울기는 lambda_tol 이 아니라 스텝 정지 규칙(bmpc_qp.c:166)이 정한다: |d| <= step_tol (1 + |x|) 면 멈추므로
     * |g| 는 pmax n step_tol 까지 남는다. 박스가 1 보다 훨씬 작으면 이 항이 lambda_tol 보다 크다 (float 시드 20, 45, 73).
     * 엄격한 형태는 after_qp/repro_step_tol_small_box.c 에 따로 둔다. */
    CHECK(kkt_ok(p, p->x, 1.5 * lambda_scale(p) + (double)N * rmax * (double)STEP_TOL), "seed %u: KKT", SEEDS[s]);
  }
  g_seed = keep;
}

/* [고정] 방금 푼 변수가 바로 막히면 정밀도 한계로 보고 끝낸다 (bmpc_qp.c:253-254, alpha <= step_tol 포함).
 * P=[1,-1;-1,2], q=(-1,0.5): x1 -> 1, g2 = -0.5 라 x2 를 푼다. L = [1;-1,1] 이라 d = (0.5, 0.5) 가 정확하다.
 * x2 의 폭이 w = 0.5 step_tol 이면 비율 = step_tol, 끝. 폭이 그 두 배면 계속 간다. */
static void test_release_precision_exit(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: release precision exit]\n");
  set2(p, 1, -1, 2, -1, 0.5, -10, 10, 0, (double)(R(0.5) * STEP_TOL));
  p->ub[1] = R(0.5) * STEP_TOL;
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 50, &it);
  CHECK(st == BMPC_OK && it == 3, "alpha == step_tol: st %d it %d", st, it);
  CHECK(p->x[1] == p->ub[1], "x2 at its upper bound: %g", (double)p->x[1]);
  /* 다른 고정 변수가 없는 이 경우는 정말 해다. 다른 고정 변수의 승수가 음수인 경우의 거짓 OK 는 after_qp/repro_line240_false_ok.c */
  CHECK(kkt_ok(p, p->x, 1.5 * lambda_scale(p) + 2.0 * 2.0 * (double)STEP_TOL * (1.0 + fabs((double)p->x[0]))), "precision exit: KKT");
  p->ub[1] = STEP_TOL;
  st = solve(p, x0, 50, &it);
  CHECK(st == BMPC_OK && it > 3 && p->x[1] == p->ub[1], "alpha == 2 step_tol: st %d it %d", st, it);
}

/* 시험용 함수의 약속 (bmpc_qp.h:31-32). 값은 손으로 계산한다. */
static void test_kkt_residual(void)
{
  prob_t *p = &g_p;
  bmpc_real *x = p->x; /* x[2] 와 P 의 다음 칸은 파수꾼: 읽으면 값이 튄다 */
  printf("[qp: kkt residual helper]\n");
  /* P = I, q = (1, -1). 박스 [0,2]. */
  set2(p, 1, 0, 1, 1, -1, 0, 2, 0, 2);
  x[0] = 0; /* 하한, g = 1: 위반 없음 */
  x[1] = 1; /* 내부, g = 0 */
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 0.0), "optimal point");
  x[0] = R(1); /* 내부, g = 2 */
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 2.0), "free |g|");
  /* 하한에서 g < 0: x = lb, 위반은 -g */
  set2(p, 1, 0, 1, -3, -1, 0, 2, 0, 2);
  x[0] = 0;
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 3.0), "lower with negative g");
  /* 상한에서 g > 0 */
  set2(p, 1, 0, 1, 1, -1, 0, 2, 0, 2);
  x[0] = R(2);
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 3.0), "upper with positive g");
  /* 경계에서 안쪽을 향하는 기울기는 위반이 아니다 */
  set2(p, 1, 0, 1, -7, -1, 0, 2, 0, 2);
  x[0] = R(2); /* 상한, g = -5 */
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 0.0), "upper with negative g");
  set2(p, 1, 0, 1, 5, -1, 0, 2, 0, 2);
  x[0] = 0; /* 하한, g = 5 */
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 0.0), "lower with positive g");
  /* 범위 밖: 위반 거리. g 를 0 으로 맞춰 거리만 남긴다 */
  set2(p, 1, 0, 1, 1.5, -1, 1, 2, 0, 2);
  x[0] = R(-1.5);
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 2.5), "below the box: distance");
  set2(p, 1, 0, 1, -5, -1, 1, 2, 0, 2);
  x[0] = R(5);
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 3.0), "above the box: distance");
  /* lb == ub: x 와 고정값의 거리, 기울기 무관. lb 가 0 이 아니어야 합과 차를 구별한다 */
  set2(p, 1, 0, 1, 50, -1, 1.5, 1.5, 0, 2);
  x[0] = R(1.0);
  x[1] = 1;
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 0.5), "fixed: |x - lb|");
  x[0] = R(1.5);
  CHECK(NEAR(bmpc_boxqp_kkt_residual(2, p->P, p->q, p->lb, p->ub, x), 0.0), "fixed at its value");
}

/* 헤더 주석(bmpc_qp.c:48-49): working set 이 바뀌면 처음부터 분해하지 않고 갱신한다. 타이밍 그리드에 직접 영향을 준다.
 * 출력으로는 안 보이므로 인수의 비트로 본다: 갱신을 거친 인수는 처음부터 분해한 값과 반올림만큼 다르다.
 * 강제 재분해로 바뀌면 전부 같아져서 0 이 된다. 반올림에 의존하지만 IEEE 754 에서 결정적이다. */
static void test_update_path_used(void)
{
  printf("[qp: factor update path is used]\n");
  CHECK(g_update_seen > 1000, "checked %d capped exits", g_update_seen);
  CHECK(g_update_used * 5 > g_update_seen, "only %d of %d capped exits hold an updated factor", g_update_used, g_update_seen);
  printf("  updated factor at %d of %d capped exits\n", g_update_used, g_update_seen);
  /* 경로별. chol_append 는 나누고 재분해는 역수를 곱하므로 풀기 뒤의 인수도 처음부터 분해한 값과 비트가 다르다.
   * 풀기 때마다 또는 막힐 때마다 재분해(O(nf^3))하는 구현이면 그 경로의 비율이 0 이 된다. */
  CHECK(g_block_seen > 200, "checked %d exits right after a block", g_block_seen);
  CHECK(g_rel_seen >= 100, "checked %d exits right after a release", g_rel_seen);
  CHECK(g_block_used * 5 > g_block_seen, "only %d of %d exits after a block hold an updated factor", g_block_used, g_block_seen);
  CHECK(g_rel_used * 5 > g_rel_seen, "only %d of %d exits after a release hold an updated factor", g_rel_used, g_rel_seen);
  printf("  after a block %d of %d, after a release %d of %d\n", g_block_used, g_block_seen, g_rel_used, g_rel_seen);
}

/* 스텝 클램프 (bmpc_qp.c:226): 막는 변수는 경계로 옮겨 놓지만 (:242), 거의 같은 비율의 다른 변수는 x + alpha d 가 반올림으로 경계를 넘을 수 있다.
 * 비율이 이진 유리수가 아닌 거의 동률 문제. 넘는 사례가 실제로 생기는지 시험 안에서 같은 연산으로 세고, 범위는 정확히 본다. */
static void test_step_clamp(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int trial, sign, overshoot = 0, it, st;
  printf("[qp: step clamp on a near tie]\n");
  for (sign = -1; sign <= 1; sign += 2) {
    for (trial = 0; trial < 4000; ++trial) {
      const double r = 0.1 + rnd(), k = 0.1 + 3.0 * rnd(), u = 2.0 + 3.0 * rnd();
      const double s = (double)sign;
      volatile bmpc_real d0, d1, a0, a1, alpha, y0, y1;
      prob_begin(p, 2);
      p->P[0] = 1;
      p->P[1] = 0;
      p->P[2] = 0;
      p->P[3] = 1;
      p->q[0] = R(-s * r * u);
      p->q[1] = R(-s * r * k * u);
      p->lb[0] = sign > 0 ? R(-9) : R(-r);
      p->ub[0] = sign > 0 ? R(r) : R(9);
      p->lb[1] = sign > 0 ? R(-9) : R(-r * k);
      p->ub[1] = sign > 0 ? R(r * k) : R(9);
      x0[0] = 0;
      x0[1] = 0;
      d0 = -p->q[0];
      d1 = -p->q[1];
      a0 = (sign > 0 ? p->ub[0] : p->lb[0]) / d0;
      a1 = (sign > 0 ? p->ub[1] : p->lb[1]) / d1;
      alpha = a0 < a1 ? a0 : a1;
      y0 = alpha * d0;
      y1 = alpha * d1;
      if (y0 > p->ub[0] || y0 < p->lb[0] || y1 > p->ub[1] || y1 < p->lb[1]) ++overshoot;
      st = solve(p, x0, 1, &it);
      CHECK(st == BMPC_ERR_MAXITER && it == 1, "near tie: st %d it %d", st, it);
      CHECK(p->x[0] >= p->lb[0] && p->x[0] <= p->ub[0] && p->x[1] >= p->lb[1] && p->x[1] <= p->ub[1],
            "near tie: x (%.9g, %.9g) outside [%.9g, %.9g] x [%.9g, %.9g]", (double)p->x[0], (double)p->x[1], (double)p->lb[0], (double)p->ub[0],
            (double)p->lb[1], (double)p->ub[1]);
    }
  }
  CHECK(overshoot >= 20, "only %d of 8000 cases would overshoot without the clamp", overshoot);
  printf("  x + alpha d passes the bound before clamping in %d of 8000 cases\n", overshoot);
}

/* ---------------- 7. 실제 규모 ---------------- */
static bmpc_mpc_work_t g_mw;
static bmpc_mpc_result_t g_mr;

static void mpc_problem_hw(const bmpc_config_t *c, bmpc_problem_t *pr, double v0, double kappa, double cte, double ref_v, double v_ub, int highway)
{
  memset(pr, 0, sizeof(*pr));
  pr->s0[BMPC_SV] = R(v0);
  pr->s0[BMPC_SCTE] = R(cte);
  pr->c[0] = R(cte);
  pr->c[2] = R(0.5 * kappa);
  pr->ref_v = R(ref_v);
  pr->v_lb = 0;
  pr->v_ub = R(v_ub);
  pr->steer_lb = c->steer_lb;
  pr->steer_ub = c->steer_ub;
  bmpc_compute_weights(c, highway, 0, pr->s0[BMPC_SV], &pr->w);
}
static void mpc_problem(const bmpc_config_t *c, bmpc_problem_t *pr, double v0, double kappa, double cte, double ref_v, double v_ub)
{
  mpc_problem_hw(c, pr, v0, kappa, cte, ref_v, v_ub, 0);
}

/* bmpc_mpc_solve 가 실제로 만드는 QP 를 w 에서 그대로 꺼내 상한을 건다. 작은 문제의 float 기준은 여기서 뜻이 없으므로 반복점은 비교하지 않고,
 * [약속] status, iters, 범위, 목적함수와 끝점의 KKT 만 본다. 해 오차는 double 정밀도로 푼 기준과 비교해 목적함수 차이로 잰다.
 * 묶음: 시내 저속, 운용 속도 (yaml 15~16 m/s), highway 가중치 (20~25 m/s). 묶음별로 float 오차가 다르다.
 *
 * KKT 허용오차 (bmpc_qp.c 에서 유도):
 *  - working set 의 고정 변수: 나가는 조건이 승수 >= -lambda_tol 이다 (:174, :184). float 기울기와 시험의 double 기울기 차이는 n eps pmax xmax 로
 *    lambda_tol 의 약 6 % 이하이므로 1.1 lambda_tol. 승수 허용오차를 키운 구현은 여기서 걸린다.
 *  - 자유 변수: :166 의 정지 규칙은 |d| 를 재므로 |g| 는 n pmax step_tol (1 + |x|) 까지 남을 수 있다. 이 항은 크다. 약속이 그렇다.
 * 목적함수 차이 (float 에는 약속이 없는 회귀 방어선, 측정에서 정했다. 아래 GAP_* 참고):
 *  f(x0) - f* 에 대한 비율로 묶는다. 시작점을 돌려주면 비율이 1 이라 반드시 걸린다. 위 KKT 만으로는 x0 가 통과하는 QP 가 있다. */
#ifdef BMPC_USE_FLOAT
#define GAP_RATIO 1e-3 /* 측정 최악 3.5e-4 (시내 저속) 의 약 3 배. 잰 값은 실행 때 출력한다 */
#else
#define GAP_RATIO 1e-12 /* 측정 최악 8e-16 */
#endif
static void test_real_scale(void)
{
  static const struct {
    const char *name;
    int hw;
    double V0[3];
    int nv0;
    double VB[3][2]; /* ref_v, v_ub */
  } G[] = {
      {"urban low speed", 0, {1, 3, 5}, 3, {{3, 3}, {5, 5}, {5, 40}}},
      {"operating speed", 0, {14, 16, 0}, 2, {{15, 15}, {16, 16}, {16, 40}}},
      {"highway", 1, {20, 25, 0}, 2, {{25, 25}, {22, 25}, {25, 40}}},
  };
  static const double KA[] = {0, 0.1, -0.3}, CT[] = {0.3, -1.0};
  size_t gi, a, b, d, e;
  bmpc_config_t c;
  bmpc_problem_t pr;
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  double pmin = HUGE_VAL;
  int count = 0, hits[NCAPS], guarded = 0, x0_kkt_pass = 0;
  printf("[qp: real problem scale, QPs built by bmpc_mpc_solve]\n");
  memset(hits, 0, sizeof(hits));
  bmpc_config_default(&c);
  bmpc_config_sanitize(&c);
  for (gi = 0; gi < sizeof(G) / sizeof(G[0]); ++gi) {
    double pg = 0, worst_steer = 0, worst_v = 0, worst_gap = 0, worst_ratio = 0, worst_kkt = 0, worst_act = 0;
    int cg = 0;
    for (a = 0; a < (size_t)G[gi].nv0; ++a)
      for (b = 0; b < 3; ++b)
        for (d = 0; d < 2; ++d)
          for (e = 0; e < 6; ++e) {
            int i, k, n, K, st, it;
            double pm = 0, f_prev, tol, acttol, width = 0, xn = 0, g_max = 0, act_max = 0, fs, f0, fstar, ds = 0, dv = 0, ratio_den;
            /* e < 3: 첫 선형화의 QP. e >= 3: 둘째 선형화의 QP (w 에는 마지막 QP 가 남는다) */
            c.sqp_iters = e < 3 ? 1 : 2;
            c.qp_max_iters = e < 3 ? 1 : 200;
            mpc_problem_hw(&c, &pr, G[gi].V0[a], KA[b], CT[d], G[gi].VB[e % 3][0], G[gi].VB[e % 3][1], G[gi].hw);
            bmpc_mpc_solve(&c, &pr, NULL, &g_mw, &g_mr);
            n = g_mr.nz;
            prob_begin(p, n);
            for (i = 0; i < n * n; ++i) {
              p->P[i] = g_mw.P[i];
              pm = fmax(pm, fabs((double)p->P[i]));
            }
            for (i = 0; i < n; ++i) {
              p->q[i] = g_mw.q[i];
              p->lb[i] = g_mw.lb[i];
              p->ub[i] = g_mw.ub[i];
              x0[i] = 0;
              width += (double)p->ub[i] - (double)p->lb[i];
            }
            pmin = fmin(pmin, pm);
            pg = fmax(pg, pm);
            st = solve(p, x0, 2000, &K);
            CHECK(st == BMPC_OK, "scale %d: uncapped status %d", count, st);
            fs = objective(p, p->x);
            f0 = objective(p, x0);
            for (i = 0; i < n; ++i) xn = fmax(xn, fabs((double)p->x[i]));
            tol = 1.1 * lambda_scale(p) + (double)n * pm * (double)STEP_TOL * (1.0 + xn);
            acttol = 1.1 * lambda_scale(p);
            CHECK(kkt_ok(p, p->x, tol), "scale %d: KKT at the end, tol %g", count, tol);
            for (i = 0; i < n; ++i) {
              const double g = grad_i(p, p->x, i);
              if (p->lb[i] == p->ub[i]) continue;
              g_max = fmax(g_max, p->x[i] <= p->lb[i] ? fmax(0.0, -g) : (p->x[i] >= p->ub[i] ? fmax(0.0, g) : fabs(g)));
              if (g_w.ws[i] != 0) act_max = fmax(act_max, g_w.ws[i] < 0 ? fmax(0.0, -g) : fmax(0.0, g));
            }
            CHECK(act_max <= acttol, "scale %d: fixed variable multiplier %g below -lambda_tol, limit %g", count, act_max, acttol);
            worst_kkt = fmax(worst_kkt, g_max / lambda_scale(p));
            worst_act = fmax(worst_act, act_max / lambda_scale(p));
            if (kkt_ok(p, x0, tol)) ++x0_kkt_pass;
            /* 정확한 해에 가까운 기준: double 로, double 의 허용오차로 */
            g_ref_eps = DBL_EPSILON;
            ref_run(p, x0);
            g_ref_eps = 0;
            CHECK(g_ref.status == BMPC_OK, "scale %d: reference status %d", count, g_ref.status);
            for (i = 0; i < n; ++i) {
              const double e1 = fabs((double)p->x[i] - g_ref.xs[g_ref.K][i]);
              if (i < n / 2) ds = fmax(ds, e1);
              else dv = fmax(dv, e1);
            }
            {
              double xr[NMAX];
              for (i = 0; i < n; ++i) xr[i] = g_ref.xs[g_ref.K][i];
              fstar = 0;
              for (i = 0; i < n; ++i) {
                double s1 = 0;
                for (k = 0; k < n; ++k) s1 += (double)p->P[i * n + k] * xr[k];
                fstar += xr[i] * (0.5 * s1 + (double)p->q[i]);
              }
            }
            /* 볼록성에서 유도되는 상한 */
            CHECK(fs - fstar <= tol * width, "scale %d: f - f* = %g above the derived bound %g", count, fs - fstar, tol * width);
            /* 시작점과의 비율. f0 - f* 가 반올림 크기면 뜻이 없으므로 그런 QP 는 세지 않는다 */
            ratio_den = f0 - fstar;
            if (ratio_den > 1e3 * EPS * (1.0 + fabs(fstar))) {
              ++guarded;
              worst_ratio = fmax(worst_ratio, (fs - fstar) / ratio_den);
              CHECK(fs - fstar <= GAP_RATIO * ratio_den, "scale %d (%s): f - f* = %g is %g of f(x0) - f* = %g", count, G[gi].name, fs - fstar,
                    (fs - fstar) / ratio_den, ratio_den);
            }
            worst_gap = fmax(worst_gap, fs - fstar);
            worst_steer = fmax(worst_steer, ds);
            worst_v = fmax(worst_v, dv);
#ifndef BMPC_USE_FLOAT
            /* double 빌드는 허용오차가 아주 작아 기준과 거의 같아야 한다. float 빌드는 다르다 (아래 출력, 보고서의 발견) */
            CHECK(ds <= 1e-6 && dv <= 1e-6, "scale %d: double build off the reference, steer %g v %g", count, ds, dv);
#endif
            /* 상한 경로 */
            f_prev = f0;
            for (k = 0; k < NCAPS; ++k) {
              int viol = 0;
              double f;
              const int cap = CAPS[k];
              const double ftol = 64.0 * EPS * (1.0 + fabs(f_prev));
              st = solve(p, x0, cap, &it);
              CHECK(st == (cap < K ? BMPC_ERR_MAXITER : BMPC_OK), "scale %d cap %d (K %d): status %d", count, cap, K, st);
              CHECK(it == (cap < K ? cap : K), "scale %d cap %d (K %d): iters %d", count, cap, K, it);
              for (i = 0; i < n; ++i)
                if (!(p->x[i] >= p->lb[i] && p->x[i] <= p->ub[i])) ++viol;
              CHECK(viol == 0, "scale %d cap %d: %d bound violations", count, cap, viol);
              f = objective(p, p->x);
              CHECK(f <= f_prev + ftol, "scale %d cap %d: objective rose %.9g -> %.9g", count, cap, f_prev, f);
              f_prev = f;
              if (cap < K) ++hits[k];
            }
            ++count;
            ++cg;
          }
    /* 잰 값. 약속이 아니다 */
    printf("  %s: %d QPs, pmax up to %.3g. at status OK against the double reference: worst steer %.3g rad, speed %.3g m/s, f - f* %.3g, "
           "(f - f*) / (f(x0) - f*) %.3g, KKT %.3g lambda_tol, fixed multipliers %.3g lambda_tol\n",
           G[gi].name, cg, pg, worst_steer, worst_v, worst_gap, worst_ratio, worst_kkt, worst_act);
  }
  CHECK(pmin >= 1e4, "setup: these QPs should have large P entries, smallest pmax %g", pmin);
  /* 시험이 빈말이 되지 않게: 가장 큰 그리드 상한도 실제로 걸려야 하고, 비율 검사가 대부분의 QP 에 걸려야 한다 */
  for (a = 0; a < (size_t)NCAPS; ++a) CHECK(hits[a] >= 5, "cap %d reached on only %d of %d real QPs", CAPS[a], hits[a], count);
  CHECK(guarded * 10 >= count * 9, "gap ratio applied to only %d of %d QPs", guarded, count);
  printf("  cap reached on");
  for (a = 0; a < (size_t)NCAPS; ++a) printf(" %d:%d", CAPS[a], hits[a]);
  printf(" of %d QPs; gap ratio applied to %d; the start point alone passes the free-variable KKT tolerance on %d\n", count, guarded, x0_kkt_pass);
}

/* ---------------- 9. MPC 수준: 상한에 걸린 해가 쓰이는가 ---------------- */
/* SQP 기준: bmpc_mpc.c 의 규칙(시작점 클램프, dz = 0 에서 QP, 선 탐색 반씩 최대 3 번, 마지막은 수용, step < sqp_tol 이면 끝)을 시험 안에서 다시 따른다.
 * QP 는 bmpc_boxqp_solve 를 직접 부른다: 같은 QP (bmpc_mpc_solve 를 상한 1, SQP 1 로 불러 w 에서 꺼낸다) 를 같은 상한으로, dz = 0 에서.
 * 그래서 QP 풀이가 바뀌면 (0 길이 스텝 수정 포함) 기대값도 함께 바뀐다. 이 시험이 보는 것은 QP 결과를 SQP 층이 어떻게 쓰느냐다. */
typedef struct {
  bmpc_real z[NMAX];
  bmpc_real J;
  int sqp, qp, last_st, any_cap, failed;
  int cut_capped;   /* 상한에 걸린 QP 에서 선 탐색이 스텝을 줄인 횟수 */
  int moved_capped; /* 상한에 걸린 QP 의 스텝이 sqp_tol 이상으로 쓰인 횟수 */
  int still_capped; /* 상한에 걸렸는데 스텝이 sqp_tol 미만. 잰 값, 약속 아님 */
  int qp_bounds_ok;
} sqp_ref_t;
static bmpc_mpc_work_t g_mx;
static bmpc_mpc_result_t g_mrx;
static bmpc_qp_work_t g_qx;
static bmpc_real g_dz[NMAX], g_zt[NMAX];
static sqp_ref_t g_sr;

static void sqp_reference(const bmpc_config_t *c, const bmpc_problem_t *pr, const bmpc_real *z_init, sqp_ref_t *r)
{
  const int nu = c->N - 1, n = 2 * nu;
  bmpc_config_t c1 = *c;
  int it, k;
  memset(r, 0, sizeof(*r));
  r->last_st = BMPC_OK;
  r->qp_bounds_ok = 1;
  for (k = 0; k < nu; ++k) {
    r->z[k] = bm_clamp(z_init ? z_init[k] : pr->prev_delta_ref, pr->steer_lb, pr->steer_ub);
    r->z[nu + k] = bm_clamp(z_init ? z_init[nu + k] : pr->s0[BMPC_SV], pr->v_lb, pr->v_ub);
  }
  r->J = bmpc_mpc_cost(c, pr, r->z, g_mx.sbar);
  c1.sqp_iters = 1;
  c1.qp_max_iters = 1;
  for (it = 0; it < c->sqp_iters; ++it) {
    int st, qi = 0, ls;
    bmpc_real alpha = 1;
    double step = 0;
    (void)bmpc_mpc_solve(&c1, pr, r->z, &g_mx, &g_mrx);
    /* 꺼낸 QP 가 정말 지금 z 에서 만든 것인지 (bmpc_mpc.c 의 박스는 범위 - z). z_init 을 무시한 구현은 여기와 z 비교에서 걸린다 */
    for (k = 0; k < nu; ++k)
      if (g_mx.lb[k] != pr->steer_lb - r->z[k] || g_mx.ub[nu + k] != pr->v_ub - r->z[nu + k]) r->qp_bounds_ok = 0;
    memset(g_dz, 0, sizeof(bmpc_real) * (size_t)n);
    st = bmpc_boxqp_solve(n, g_mx.P, g_mx.q, g_mx.lb, g_mx.ub, g_dz, c->qp_max_iters, &g_qx, &qi);
    r->sqp = it + 1;
    r->qp += qi;
    r->last_st = st;
    if (st == BMPC_ERR_NUMERIC || st == BMPC_ERR_INFEASIBLE) {
      r->failed = st;
      break;
    }
    if (st == BMPC_ERR_MAXITER) ++r->any_cap;
    for (ls = 0; ls < 4; ++ls) {
      bmpc_real Jt;
      for (k = 0; k < nu; ++k) {
        g_zt[k] = bm_clamp(r->z[k] + alpha * g_dz[k], pr->steer_lb, pr->steer_ub);
        g_zt[nu + k] = bm_clamp(r->z[nu + k] + alpha * g_dz[nu + k], pr->v_lb, pr->v_ub);
      }
      Jt = bmpc_mpc_cost(c, pr, g_zt, g_mx.sbar);
      if (Jt <= r->J || ls == 3) {
        r->J = Jt;
        break;
      }
      alpha *= (bmpc_real)0.5;
    }
    if (st == BMPC_ERR_MAXITER && ls > 0) ++r->cut_capped;
    for (k = 0; k < n; ++k) {
      step = fmax(step, fabs((double)(g_zt[k] - r->z[k])));
      r->z[k] = g_zt[k];
    }
    if (st == BMPC_ERR_MAXITER) {
      if (step < (double)c->sqp_tol) ++r->still_capped;
      else ++r->moved_capped;
    }
    if (step < (double)c->sqp_tol) break;
  }
}

/* 결과를 기준과 비교한다. z 허용오차: 같은 연산을 같은 빌드에서 하므로 반올림 몇 번 (FMA 로 합쳐지는 빌드에서 차이가 날 수 있다). */
static void check_against_reference(const bmpc_config_t *c, const bmpc_problem_t *pr, int st, const bmpc_mpc_result_t *res, const sqp_ref_t *r, const char *tag)
{
  const int nu = c->N - 1, n = 2 * nu;
  const int cap = c->qp_max_iters;
  int k, viol = 0;
  double e = 0;
  bmpc_real sb[BMPC_N_MAX][BMPC_NX];
  CHECK(st == res->status, "%s: returned %d, res->status %d", tag, st, res->status);
  CHECK(r->qp_bounds_ok, "%s: extracted QP is not built at the reference z", tag);
  CHECK(res->sqp_iters == r->sqp, "%s sqp %d cap %d: sqp_iters %d, reference %d", tag, c->sqp_iters, cap, res->sqp_iters, r->sqp);
  CHECK(res->qp_iters == r->qp, "%s sqp %d cap %d: qp_iters %d, reference %d", tag, c->sqp_iters, cap, res->qp_iters, r->qp);
  CHECK(res->qp_iters <= c->sqp_iters * cap, "%s: qp_iters %d above sqp x cap", tag, res->qp_iters);
  for (k = 0; k < n; ++k) e = fmax(e, fabs((double)res->z[k] - (double)r->z[k]) / (1.0 + fabs((double)r->z[k])));
  CHECK(e <= 16.0 * EPS, "%s sqp %d cap %d: z off the reference by %g (sqp %d qp %d, capped QPs %d)", tag, c->sqp_iters, cap, e, res->sqp_iters, res->qp_iters, r->any_cap);
  for (k = 0; k < nu; ++k) {
    if (!(res->z[k] >= pr->steer_lb && res->z[k] <= pr->steer_ub)) ++viol;
    if (!(res->z[nu + k] >= pr->v_lb && res->z[nu + k] <= pr->v_ub)) ++viol;
  }
  CHECK(viol == 0, "%s sqp %d cap %d: %d z outside the box", tag, c->sqp_iters, cap, viol);
  /* res->cost 는 z 의 비용 (선 탐색이 마지막에 계산한 값) */
  CHECK(NEAR(res->cost, bmpc_mpc_cost(c, pr, res->z, sb)), "%s: reported cost %g is not the cost of z %g", tag, (double)res->cost, (double)bmpc_mpc_cost(c, pr, res->z, sb));
  CHECK(fabs((double)res->cost - (double)r->J) <= 1e3 * EPS * (1.0 + fabs((double)r->J)), "%s: cost %.9g, reference %.9g", tag, (double)res->cost, (double)r->J);
  /* 상태: 마지막 QP 가 상한에 걸렸으면 MAXITER, 어느 QP 도 안 걸렸으면 OK (bmpc_mpc.h:58). 앞 QP 만 걸린 경우는 헤더와 코드가 달라 (after_qp/repro_mpc_sticky_status.c) 묻지 않는다. */
  if (r->last_st == BMPC_ERR_MAXITER) CHECK(st == BMPC_ERR_MAXITER, "%s: last QP capped, status %d", tag, st);
  if (r->any_cap == 0 && !r->failed) CHECK(st == BMPC_OK, "%s: no QP capped, status %d", tag, st);
  if (r->failed) CHECK(st == r->failed, "%s: QP failed with %d, status %d", tag, r->failed, st);
}

/* MPC 수준 상한 그리드: 계획된 SQP 1, 2 x QP 상한의 모든 칸을 차가운 시작으로.
 * 약속: 상태, 범위, res->cost, qp_iters 합계. 그리고 z 자체. 비용 상승은 약속이 없어(bmpc_mpc.c:294) 세어 출력만 한다. */
static void test_mpc_cap_grid(void)
{
  static const double V0[] = {1, 3, 5, 15}, KA[] = {0, 0.1, -0.1, 0.3, -0.3}, CT[] = {0, 0.3, -0.3, 1.0};
  static const double VB[][2] = {{3, 3}, {5, 5}, {15, 15}, {8, 40}}; /* ref_v, v_ub */
  static const int MCAPS[] = {1, 2, 4, 8, 10, 16, 200};
  bmpc_config_t c;
  bmpc_problem_t pr;
  size_t q, sq;
  printf("[mpc: SQP 1, 2 x QP cap, cold start, z against the reference]\n");
  bmpc_config_default(&c);
  bmpc_config_sanitize(&c);
  for (sq = 1; sq <= 2; ++sq) {
    c.sqp_iters = (int)sq;
    for (q = 0; q < sizeof(MCAPS) / sizeof(MCAPS[0]); ++q) {
      size_t a, b, d, f;
      int capped = 0, rose = 0, total = 0, moved = 0, still = 0, cut = 0, one = 0;
      c.qp_max_iters = MCAPS[q];
      for (a = 0; a < 4; ++a)
        for (b = 0; b < 5; b += 1)
          for (d = 0; d < 4; d += 1)
            for (f = 0; f < 4; ++f) {
              int st, k;
              double j0;
              bmpc_real zs[NMAX];
              const int nu = c.N - 1;
              /* 칸 수를 줄인다: 모든 조합의 약 1/3 */
              if ((a + b + d + f) % 3 != 0) continue;
              mpc_problem(&c, &pr, V0[a], KA[b], CT[d], VB[f][0], VB[f][1]);
              for (k = 0; k < nu; ++k) {
                zs[k] = bm_clamp(pr.prev_delta_ref, pr.steer_lb, pr.steer_ub);
                zs[nu + k] = bm_clamp(pr.s0[BMPC_SV], pr.v_lb, pr.v_ub);
              }
              j0 = (double)bmpc_mpc_cost(&c, &pr, zs, g_mw.sbar);
              st = bmpc_mpc_solve(&c, &pr, NULL, &g_mw, &g_mr);
              sqp_reference(&c, &pr, NULL, &g_sr);
              check_against_reference(&c, &pr, st, &g_mr, &g_sr, "cold");
              ++total;
              if (st == BMPC_ERR_MAXITER) {
                ++capped;
                CHECK(g_mr.qp_iters >= MCAPS[q], "cap %d: MAXITER with qp_iters %d", MCAPS[q], g_mr.qp_iters);
              }
              moved += g_sr.moved_capped;
              still += g_sr.still_capped;
              cut += g_sr.cut_capped;
              if (g_mr.sqp_iters == 1) ++one;
              if ((double)g_mr.cost > j0) ++rose;
            }
      printf("  sqp %d cap %3d: %d solves, MAXITER %d, capped QPs with z moved %d, below sqp_tol %d, line search cut on capped %d, one linearisation %d, final cost above start %d\n",
             (int)sq, MCAPS[q], total, capped, moved, still, cut, one, rose);
      /* 빈말 방지: 상한 칸마다 상한에 걸린 스텝이 실제로 쓰여야 한다. 상한 200 은 걸리지 않아야 한다. */
      if (MCAPS[q] <= 16) CHECK(moved >= 5, "sqp %d cap %d: only %d capped QPs moved z", (int)sq, MCAPS[q], moved);
      if (MCAPS[q] == 200) CHECK(capped == 0, "sqp %d cap 200 reached in %d solves", (int)sq, capped);
    }
  }

  /* 상한에 걸린 스텝에도 선 탐색이 듣는가. 기본 설정에서는 상한에 걸린 스텝이 비용을 올리는 경우를 임의 탐색 20000 번에서 찾지 못했다 (잰 값).
   * 그래서 모델의 비선형이 큰 시험용 설정 (dt 0.3, Lf 1, N 20) 을 쓴다. 비용이 오르면 줄여야 하고 (bmpc_mpc.c:288), z 는 기준과 같아야 한다. */
  {
    static const int CC[] = {1, 2, 4, 8};
    int t, cut = 0, capped = 0;
    bmpc_config_t c3;
    bmpc_config_default(&c3);
    bmpc_config_sanitize(&c3);
    c3.dt = R(0.3);
    c3.Lf = R(1.0);
    c3.N = 20;
    for (t = 0; t < 400; ++t) {
      const double v0 = rnd2(2.0, 30.0), ct = rnd2(-4.0, 4.0), ep = rnd2(-0.6, 0.6), rv = rnd2(2.0, 30.0);
      int st;
      c3.qp_max_iters = CC[t % 4];
      c3.sqp_iters = 1 + (t / 4) % 2;
      memset(&pr, 0, sizeof(pr));
      pr.s0[BMPC_SV] = R(v0);
      pr.s0[BMPC_SCTE] = R(ct);
      pr.s0[BMPC_SEPSI] = R(ep);
      pr.c[0] = R(ct);
      pr.c[1] = R(-tan(ep));
      pr.c[2] = R(rnd2(-0.3, 0.3));
      pr.c[3] = R(rnd2(-0.01, 0.01));
      pr.ref_v = R(rv);
      pr.v_ub = R(40);
      pr.steer_lb = c3.steer_lb;
      pr.steer_ub = c3.steer_ub;
      bmpc_compute_weights(&c3, 0, 0, pr.s0[BMPC_SV], &pr.w);
      st = bmpc_mpc_solve(&c3, &pr, NULL, &g_mw, &g_mr);
      sqp_reference(&c3, &pr, NULL, &g_sr);
      check_against_reference(&c3, &pr, st, &g_mr, &g_sr, "line search stress");
      capped += g_sr.any_cap;
      cut += g_sr.cut_capped;
    }
    printf("  stress config: %d capped QPs, line search cut %d of them\n", capped, cut);
    CHECK(cut >= 3, "line search cut only %d capped steps of %d", cut, capped);
  }
}

/* warm start 입력 (z_init): 유효, 범위 밖, 유한하지 않음, 모순된 범위. 값의 약속은 bmpc_mpc.h 주석 + clamp_z. NaN/Inf 는 약속이 없어 "정의된 상태, 범위 안의 z" 만 본다. */
static void test_mpc_warm_input(void)
{
  static const int MCAPS[] = {1, 2, 4, 8, 10, 16};
  bmpc_config_t c;
  bmpc_problem_t pr;
  bmpc_real zi[NMAX], zc[NMAX];
  size_t q;
  int sq, k, kind;
  printf("[mpc: warm start input]\n");
  bmpc_config_default(&c);
  bmpc_config_sanitize(&c);
  for (sq = 1; sq <= 2; ++sq)
    for (q = 0; q < sizeof(MCAPS) / sizeof(MCAPS[0]); ++q) {
      const int nu = c.N - 1;
      c.sqp_iters = sq;
      c.qp_max_iters = MCAPS[q];
      mpc_problem(&c, &pr, 5, 0.1, 0.3, 6, 8);
      /* 유효: 박스 안 임의 점 */
      for (k = 0; k < nu; ++k) {
        zi[k] = R(rnd2(0.5 * (double)pr.steer_lb, 0.5 * (double)pr.steer_ub));
        zi[nu + k] = R(rnd2(1.0, 8.0));
      }
      {
        const int st = bmpc_mpc_solve(&c, &pr, zi, &g_mw, &g_mr);
        sqp_reference(&c, &pr, zi, &g_sr);
        check_against_reference(&c, &pr, st, &g_mr, &g_sr, "warm valid");
      }
      /* 범위 밖: 클램프한 점과 같은 결과여야 한다 */
      for (k = 0; k < nu; ++k) {
        zi[k] = R(k % 2 ? (double)pr.steer_ub + 1.0 : (double)pr.steer_lb - 1.0);
        zi[nu + k] = R(k % 2 ? 100.0 : -100.0);
        zc[k] = bm_clamp(zi[k], pr.steer_lb, pr.steer_ub);
        zc[nu + k] = bm_clamp(zi[nu + k], pr.v_lb, pr.v_ub);
        CHECK(zc[k] == (k % 2 ? pr.steer_ub : pr.steer_lb), "test setup clamp");
      }
      {
        const int st = bmpc_mpc_solve(&c, &pr, zi, &g_mw, &g_mr);
        static bmpc_real z1[NMAX];
        double e = 0;
        memcpy(z1, g_mr.z, sizeof(z1));
        sqp_reference(&c, &pr, zc, &g_sr);
        check_against_reference(&c, &pr, st, &g_mr, &g_sr, "warm outside");
        (void)bmpc_mpc_solve(&c, &pr, zc, &g_mw, &g_mr);
        for (k = 0; k < 2 * nu; ++k) e = fmax(e, fabs((double)z1[k] - (double)g_mr.z[k]));
        CHECK(e == 0.0, "outside start differs from its clamp by %g", e);
      }
      /* NaN, +Inf, -Inf: 끝나고, 정의된 상태, 범위 안 */
      for (kind = 0; kind < 3; ++kind) {
        const bmpc_real v = kind == 0 ? (bmpc_real)NAN : (kind == 1 ? (bmpc_real)INFINITY : (bmpc_real)-INFINITY);
        int st, viol = 0;
        for (k = 0; k < nu; ++k) {
          zi[k] = k % 3 == 0 ? v : 0;
          zi[nu + k] = k % 3 == 1 ? v : R(5);
        }
        st = bmpc_mpc_solve(&c, &pr, zi, &g_mw, &g_mr);
        CHECK(st == BMPC_OK || st == BMPC_ERR_MAXITER || st == BMPC_ERR_NUMERIC, "non-finite warm kind %d: status %d", kind, st);
        for (k = 0; k < nu; ++k) {
          if (!(g_mr.z[k] >= pr.steer_lb && g_mr.z[k] <= pr.steer_ub)) ++viol;
          if (!(g_mr.z[nu + k] >= pr.v_lb && g_mr.z[nu + k] <= pr.v_ub)) ++viol;
        }
        CHECK(viol == 0, "non-finite warm kind %d: %d z outside the box or non-finite", kind, viol);
      }
      /* 모순된 범위: INFEASIBLE, 선형화 없음 (bmpc_mpc.c:256) */
      for (k = 0; k < 2 * nu; ++k) zi[k] = 0;
      pr.v_lb = R(9);
      pr.v_ub = R(8);
      CHECK(bmpc_mpc_solve(&c, &pr, zi, &g_mw, &g_mr) == BMPC_ERR_INFEASIBLE && g_mr.sqp_iters == 0 && g_mr.qp_iters == 0, "v_lb > v_ub with warm start");
      pr.v_lb = 0;
      pr.v_ub = R(8);
      pr.steer_lb = R(0.1);
      pr.steer_ub = R(-0.1);
      CHECK(bmpc_mpc_solve(&c, &pr, zi, &g_mw, &g_mr) == BMPC_ERR_INFEASIBLE && g_mr.sqp_iters == 0, "steer_lb > steer_ub with warm start");
      pr.v_ub = (bmpc_real)NAN;
      pr.steer_lb = c.steer_lb;
      pr.steer_ub = c.steer_ub;
      CHECK(bmpc_mpc_solve(&c, &pr, zi, &g_mw, &g_mr) == BMPC_ERR_INFEASIBLE, "NaN v_ub with warm start");
    }
}

/* 폐루프: bmpc_controller_step 을 프레임마다 그대로 돌린다 (warm start, 지연 보상 포함). 프레임마다 그 프레임의 문제를 다시 만들어
 * SQP 기준과 z 를 비교하고, 상한에 걸린 프레임에서 warm start 가 유지되는지 본다. 차량은 경로 좌표의 기구학 모델, 경로 곡률은 일정 주기.
 * 트랙이 아니라 시험대다. 약속만 CHECK 하고, 프레임 수와 "상한 없이 풀었다면 조향이 얼마나 달랐을까" 는 잰 값으로 출력한다. */
typedef struct {
  int64_t t;
} tclk_t;
static int64_t tclk_now(void *ctx)
{
  tclk_t *k = (tclk_t *)ctx;
  k->t += 500;
  return k->t;
}
static bmpc_controller_t g_ctl;

static void test_closed_loop(void)
{
  static const struct {
    const char *name;
    double vref, vub;
    int hw;
    double jump;
    double a; /* 곡률 진폭 (1/m). 파장은 2 pi 25 m */
  } SC[] = {
      {"16 m/s v_ub==ref", 16, 0, 0, 0, 1.0 / 60.0},
      {"6 m/s v_ub==ref", 6, 0, 0, 0, 1.0 / 25.0},
      {"6 m/s lateral jumps", 6, 0, 0, 0.3, 1.0 / 25.0},
      {"16 m/s v_ub=40", 16, 40, 0, 0, 1.0 / 60.0},
      {"25 m/s highway", 25, 0, 1, 0, 1.0 / 150.0},
  };
  static const int MCAPS[] = {2, 4, 8, 10, 16, 200};
  enum { FRAMES = 48 };
  size_t sc, q;
  int sq, warm_capped_total = 0, warm_moved_total = 0;
  printf("[mpc: closed loop through bmpc_controller_step, warm start]\n");
  for (sc = 0; sc < sizeof(SC) / sizeof(SC[0]); ++sc)
    for (sq = 1; sq <= 2; ++sq)
      for (q = 0; q < sizeof(MCAPS) / sizeof(MCAPS[0]); ++q) {
        bmpc_config_t cfg;
        tclk_t clk;
        double e = 0.2, th = 0, v = 0.9 * SC[sc].vref, d = 0, s = 37.0 * (double)sc, worst_d0 = 0, r = SC[sc].vref;
        int f, capped = 0, moved = 0, still = 0, cold_ok = 0, longest = 0, run = 0;
        bmpc_config_default(&cfg);
        cfg.sqp_iters = sq;
        cfg.qp_max_iters = MCAPS[q];
        bmpc_controller_init(&g_ctl, &cfg);
        clk.t = 1000000;
        for (f = 0; f < FRAMES; ++f) {
          const bmpc_config_t *c = &g_ctl.cfg;
          const int nu = c->N - 1;
          bmpc_request_t rq;
          bmpc_response_t rp;
          bmpc_problem_t pb;
          static bmpc_real zw[NMAX];
          int warm, st, k, i;
          const uint32_t err0 = g_ctl.errors;
          const double kap = SC[sc].a * sin(s / 25.0);
          /* 속도 프로파일: 앞 곡률로 정한 목표에 비율 제한. vub == 0 이면 v_ub 는 ref_v 와 같다 (bridge_node.cpp:1315-1347 이 같은 값으로 자른다) */
          const double target = fmin(SC[sc].vref, sqrt(3.0 / fmax(fabs(SC[sc].a * sin((s + v) / 25.0)), 1e-6)));
          r = target > r ? fmin(target, r + 0.15) : fmax(target, r - 0.15);
          memset(&rq, 0, sizeof(rq));
          rq.seq = (uint16_t)f;
          rq.c[0] = R(-e);
          rq.c[1] = R(-tan(th));
          rq.c[2] = R(0.5 * kap);
          rq.c[3] = R(SC[sc].a * cos(s / 25.0) / 25.0 / 6.0);
          rq.x_max = R(40);
          rq.ref_v = R(r);
          rq.v_lb = 0;
          rq.v_ub = SC[sc].vub > 0 ? R(SC[sc].vub) : R(r);
          rq.post_cap = R(-1);
          rq.snap = R(-1);
          rq.velocity = R(v);
          rq.yaw_rate = R(v * tan(d) / 4.0);
          rq.period = R(0.05);
          rq.odom_age = R(0.01);
          rq.transport = R(0.002);
          if (SC[sc].hw) rq.flags |= BMPC_FLAG_HIGHWAY;
          /* 프레임 30: 리셋 요청은 warm start 를 버리고 차가운 시작으로 간다 */
          if (f == 30) rq.flags |= BMPC_FLAG_RESET;
          warm = g_ctl.warm_valid && !(rq.flags & BMPC_FLAG_RESET);
          CHECK(f == 0 || f == 31 || g_ctl.warm_valid, "%s: frame %d lost the warm start", SC[sc].name, f);
          memcpy(zw, g_ctl.z_warm, sizeof(zw));
          /* 프레임의 문제: bmpc_controller.c 가 만드는 것과 같게. s0 는 끝난 뒤 last_s0 로 받는다 */
          memset(&pb, 0, sizeof(pb));
          memcpy(pb.c, rq.c, sizeof(pb.c));
          pb.prev_delta_ref = (g_ctl.post.have_prev && BM_FABS(c->steer_sign) > (bmpc_real)1e-6) ? g_ctl.post.prev_steer_cmd / c->steer_sign : g_ctl.steering_angle;
          pb.ref_v = rq.ref_v;
          pb.v_ub = rq.v_ub;
          pb.v_lb = rq.v_lb;
          pb.steer_lb = c->steer_lb;
          pb.steer_ub = c->steer_ub;
          bmpc_compute_weights(c, SC[sc].hw, 0, rq.velocity, &pb.w);

          st = bmpc_controller_step(&g_ctl, &rq, tclk_now, &clk, &rp);
          memcpy(pb.s0, g_ctl.last_s0, sizeof(pb.s0));

          CHECK(st == BMPC_OK || st == BMPC_ERR_MAXITER, "%s frame %d: status %d", SC[sc].name, f, st);
          sqp_reference(c, &pb, warm ? zw : NULL, &g_sr);
          check_against_reference(c, &pb, st, &g_ctl.result, &g_sr, SC[sc].name);
          /* bmpc_controller.c:143-162: 응답 칸, 명령, 오류 수, warm start. MAXITER 는 오류가 아니다 (test_core_more.c:924) */
          CHECK(rp.status == (uint8_t)st, "%s: resp status %d, returned %d", SC[sc].name, rp.status, st);
          CHECK((int)rp.iters == (g_ctl.result.qp_iters > 255 ? 255 : g_ctl.result.qp_iters), "%s: resp iters %d", SC[sc].name, rp.iters);
          CHECK(g_ctl.errors == err0, "%s frame %d: status %d counted as an error", SC[sc].name, f, st);
          CHECK(rp.delta0 == g_ctl.result.z[0] && rp.vt_raw == g_ctl.result.z[nu], "%s: command is not z[0], z[nu]", SC[sc].name);
          i = 1;
          for (k = 0; k < nu; ++k) {
            const int src = k + 1 < nu ? k + 1 : nu - 1;
            if (g_ctl.z_warm[k] != g_ctl.result.z[src] || g_ctl.z_warm[nu + k] != g_ctl.result.z[nu + src]) i = 0;
          }
          CHECK(g_ctl.warm_valid == 1 && i, "%s frame %d: warm start is not the shifted result", SC[sc].name, f);
          if (!warm) ++cold_ok;

          /* 잰 값: 상한 없이 같은 점에서 풀었다면 */
          if (st == BMPC_ERR_MAXITER && warm) {
            static bmpc_mpc_result_t full;
            bmpc_config_t c2 = *c;
            c2.qp_max_iters = 2000;
            ++capped;
            warm_capped_total += g_sr.any_cap;
            warm_moved_total += g_sr.moved_capped;
            moved += g_sr.moved_capped > 0;
            if (g_sr.moved_capped == 0) {
              ++still;
              ++run;
              longest = run > longest ? run : longest;
            } else {
              run = 0;
            }
            (void)bmpc_mpc_solve(&c2, &pb, zw, &g_mx, &full);
            worst_d0 = fmax(worst_d0, fabs((double)full.z[0] - (double)g_ctl.result.z[0]));
          } else {
            run = 0;
          }

          /* 차량: 명령을 그대로 따른다 */
          for (i = 0; i < 10; ++i) {
            const double h = 0.005;
            const double k2 = SC[sc].a * sin(s / 25.0);
            d += ((double)rp.steer_cmd - d) * h / 0.1;
            v += ((double)rp.vt_final - v) * h;
            e += v * sin(th) * h;
            th += (v * tan(d) / 4.0 - k2 * v) * h;
            s += v * h;
          }
          if (SC[sc].jump > 0 && f % 10 == 9) e += (f % 20 == 9) ? SC[sc].jump : -SC[sc].jump;
        }
        CHECK(cold_ok == 2, "%s: %d cold frames, expected the first and the reset", SC[sc].name, cold_ok);
        CHECK(fabs(e) < 5.0 && isfinite(e), "%s sqp %d cap %d: car left the path, e %g", SC[sc].name, sq, MCAPS[q], e);
        if (capped > 0)
          printf("  %-20s sqp %d cap %2d: %d warm frames at MAXITER, z moved %d, below sqp_tol %d (longest run %d), uncapped steer differs at most %.3g rad, final e %.3f\n",
                 SC[sc].name, sq, MCAPS[q], capped, moved, still, longest, worst_d0, e);
        if (MCAPS[q] == 200) CHECK(capped == 0, "%s sqp %d: cap 200 reached", SC[sc].name, sq);
      }
  /* 빈말 방지: warm 프레임에서 상한이 걸리고 그 해가 z 를 옮긴 경우가 있어야 한다 */
  CHECK(warm_capped_total >= 20, "only %d capped QPs on warm frames", warm_capped_total);
  CHECK(warm_moved_total >= 5, "only %d capped QPs moved z on warm frames", warm_moved_total);
  printf("  warm frames: %d capped QPs, %d of them moved z\n", warm_capped_total, warm_moved_total);
}

#ifdef BMPC_PROFILE
/* 손으로 센 비용 모델. 확인 반복은 풀지 않고, 자유 변수가 없으면 Cholesky 구간에 들어가지 않고, 첫 반복의 기울기는 한 번만 계산한다. */
static void test_cost_model(void)
{
  prob_t *p = &g_p;
  bmpc_real x0[NMAX + 2];
  int it, st;
  printf("[qp: cost model, counted through the profile hooks]\n");
  /* 전부 경계에 고정: 기울기 1, 구간 0 */
  set2(p, 1, 0, 1, 1, 1, 0, 1, 0, 1);
  x0[0] = 0;
  x0[1] = 0;
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 1, "all at bound: st %d it %d", st, it);
  CHECK(bmpc_prof[BMPC_PROF_GRAD] == 1, "all at bound: %u gradients", (unsigned)bmpc_prof[BMPC_PROF_GRAD]);
  CHECK(bmpc_prof[BMPC_PROF_CHOL] == 0, "all at bound: %u factor sections", (unsigned)bmpc_prof[BMPC_PROF_CHOL]);
  /* test_hand_solved 의 release 경우: 풀이, 풀기(갱신), 풀이, 확인 = 반복 4, 기울기 4, 구간 3 */
  set2(p, 1, -1, 2, -1, 0.5, -10, 10, 0, 10);
  st = solve(p, x0, 100, &it);
  CHECK(st == BMPC_OK && it == 4, "release: st %d it %d", st, it);
  CHECK(bmpc_prof[BMPC_PROF_GRAD] == 4, "release: %u gradients", (unsigned)bmpc_prof[BMPC_PROF_GRAD]);
  CHECK(bmpc_prof[BMPC_PROF_CHOL] == 3, "release: %u factor sections", (unsigned)bmpc_prof[BMPC_PROF_CHOL]);
  /* 상한 0: 처음 working set 의 기울기 한 번뿐 */
  st = solve(p, x0, 0, &it);
  CHECK(bmpc_prof[BMPC_PROF_GRAD] == 1 && bmpc_prof[BMPC_PROF_CHOL] == 0, "cap 0: %u gradients %u sections", (unsigned)bmpc_prof[BMPC_PROF_GRAD],
        (unsigned)bmpc_prof[BMPC_PROF_CHOL]);
}
#endif

int main(void)
{
#ifdef BMPC_PROFILE
  test_cost_model();
#endif
  test_hand_solved();
  test_diag_path();
  test_enumeration_oracle();
  test_cap_hand();
  test_cap_grid();
  test_zero_step_batch();
  test_stale_last_released();
  test_initial_working_set();
  test_warm_start();
  test_fixed_variables();
  test_invalid_input();
  test_append_zero_pivot();
  test_size_limit();
  test_step_tolerance();
  test_step_clamp();
  test_lambda_tolerance();
  test_rounding_push();
  test_block_ties();
  test_multiplier_tie();
  test_remove_failure_recovery();
  test_release_precision_exit();
  test_kkt_residual();
  test_update_path_used();
  test_real_scale();
  test_mpc_cap_grid();
  test_mpc_warm_input();
  test_closed_loop();
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
