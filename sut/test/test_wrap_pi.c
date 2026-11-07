/* DEF-5 regression: bm_wrap_pi must return for any input, and keep its old result where the old loop ended.
 * Build twice: default (double) and -DBMPC_USE_FLOAT. An alarm turns a hang into a failure. */
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "bmpc/bmpc_math.h"

static int g_pass, g_fail;
#define CHECK(c, ...) do { if (c) { g_pass++; } else { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void on_alarm(int s)
{
  static const char m[] = "  FAIL: bm_wrap_pi did not return within 5 s\n";
  ssize_t w = write(1, m, sizeof(m) - 1);
  (void)s; (void)w; _exit(1);
}

/* The loop before the fix. Only called where it ends. */
static bmpc_real old_wrap(bmpc_real a)
{
  while (a > (bmpc_real)BM_PI) a -= (bmpc_real)(2.0 * BM_PI);
  while (a < (bmpc_real)-BM_PI) a += (bmpc_real)(2.0 * BM_PI);
  return a;
}

int main(void)
{
  const double big[] = {1e30, -1e30, 1e12, -1e12, 3e8, -3e8, 1e6, -1e6, HUGE_VAL, -HUGE_VAL};
  int i, same = 0, n = 0;
  signal(SIGALRM, on_alarm); alarm(5);
  printf("[DEF-5: bm_wrap_pi returns for very large and non-finite inputs]\n");
  for (i = 0; i < (int)(sizeof(big) / sizeof(big[0])); ++i) {
    const bmpc_real r = bm_wrap_pi((bmpc_real)big[i]);
    if (isfinite(big[i]) && isfinite((double)(bmpc_real)big[i]))
      CHECK(r >= (bmpc_real)-BM_PI && r <= (bmpc_real)BM_PI, "%g -> %g is in [-pi, pi]", big[i], (double)r);
    else
      CHECK(r != r, "%g -> NaN (%g)", big[i], (double)r);
  }
  CHECK(bm_wrap_pi((bmpc_real)NAN) != bm_wrap_pi((bmpc_real)NAN), "NaN stays NaN");
  printf("[DEF-5: same result as before for |a| <= 64 pi]\n");
  for (i = -2000000; i <= 2000000; i += 7) {
    const bmpc_real a = (bmpc_real)((double)i * 1e-4);
    ++n; same += (bm_wrap_pi(a) == old_wrap(a));
  }
  CHECK(same == n, "bitwise equal on %d of %d inputs from -200 to 200 rad", same, n);
  CHECK(bm_wrap_pi((bmpc_real)BM_PI) == (bmpc_real)BM_PI && bm_wrap_pi((bmpc_real)-BM_PI) == (bmpc_real)-BM_PI, "pi and -pi unchanged");
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail != 0;
}
