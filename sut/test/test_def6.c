/* DEF-6 regression: bmpc_config_set must not convert an out-of-range or NaN float to an int32 field
 * (undefined behaviour, C99 6.3.1.4). Found by fuzzing on 2026-11-08 (test/fuzz/regress/DEF6_cfg_float_to_int.bin).
 * Build with -fsanitize=undefined,float-cast-overflow so the old code fails at run time, not only by value. */
#include <math.h>
#include <stdio.h>
#include "bmpc/bmpc_config.h"

static int fails;
#define EXPECT(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

int main(void)
{
  bmpc_config_t c;
  const uint16_t N_IDX = 0, QP_IDX = 70, DT_IDX = 1;
  const float bad[] = {-4.72893e29f, 4.72893e29f, 2147483648.0f, -2147483904.0f, INFINITY, -INFINITY, NAN};
  unsigned i;

  bmpc_config_default(&c);
  for (i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
    EXPECT(bmpc_config_set(&c, N_IDX, bad[i]) == -1);
    EXPECT(c.N == 30); /* unchanged */
    EXPECT(bmpc_config_set(&c, QP_IDX, bad[i]) == -1);
    EXPECT(c.qp_max_iters == 200);
  }
  /* edges that fit */
  EXPECT(bmpc_config_set(&c, N_IDX, -2147483648.0f) == 0 && c.N == (int32_t)(-2147483647 - 1));
  EXPECT(bmpc_config_set(&c, N_IDX, 2147483520.0f) == 0 && c.N == 2147483520);
  EXPECT(bmpc_config_set(&c, N_IDX, 25.9f) == 0 && c.N == 25); /* truncation, as before */
  EXPECT(bmpc_config_set(&c, N_IDX, -0.5f) == 0 && c.N == 0);
  /* real-valued fields keep the old behaviour */
  EXPECT(bmpc_config_set(&c, DT_IDX, 0.04f) == 0 && fabs((double)c.dt - 0.04) < 1e-6);
  EXPECT(bmpc_config_set(&c, 9999u, 1.0f) == -1);
  printf("%s def6 (%d failures)\n", fails ? "FAIL" : "PASS", fails);
  return fails != 0;
}
