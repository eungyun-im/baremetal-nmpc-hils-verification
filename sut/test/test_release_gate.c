/* BENCH_RESET (0x7F0) must be ignored by a release build and accepted only by a bench build.
 * Built twice by run_tests.sh: without flags (release: the valid magic must NOT clear SAFE_STOP)
 * and with -DBS_BENCH_RESET (bench: the same frame must clear it), so the test cannot pass by doing nothing. */
#include <stdio.h>
#include <string.h>
#include "bmpc_safety.h"

static int64_t g_us;
static int64_t fake_now(void *ctx) { (void)ctx; return g_us; }
static int fake_send(void *ctx, const bmpc_can_frame_t *f) { (void)ctx; (void)f; return 0; }
static const bmpc_hal_t g_hal = {fake_now, fake_send, NULL};
static int fails;
#define EXPECT(c, msg) do { if (!(c)) { printf("FAIL %s\n", msg); ++fails; } } while (0)

int main(void)
{
  static bs_app_t a;
  bs_cfg_t bcfg;
  sc_config_t scfg;
  bmpc_can_frame_t f;
  uint32_t ms = 1000u;

  bs_default_cfg(&bcfg);
  bs_default_safety_config(&scfg);
  bs_init(&a, &bcfg, &scfg, &g_hal);
  /* white box: put the controller in the latched state directly; the test is about whether 0x7F0 is processed */
  a.sc.state = SC_STATE_SAFE_STOP;

  memset(&f, 0, sizeof f);
  f.id = BS_ID_BENCH_RESET;
  f.dlc = 2;
  f.data[0] = 0x5Au;
  f.data[1] = 0xA5u;
  bs_on_safety_frame(&a, &f, ms);
#ifdef BS_BENCH_RESET
  EXPECT(bs_is_safety_frame(BS_ID_BENCH_RESET) == 1, "bench: 0x7F0 is routed to the safety layer");
  EXPECT(a.n_resets == 1u && a.sc.state != SC_STATE_SAFE_STOP, "bench: valid BENCH_RESET clears the latch");
  printf("%s release_gate bench (%d failures)\n", fails ? "FAIL" : "PASS", fails);
#else
  EXPECT(bs_is_safety_frame(BS_ID_BENCH_RESET) == 0, "release: 0x7F0 is not a safety input");
  EXPECT(a.n_resets == 0u && a.sc.state == SC_STATE_SAFE_STOP, "release: BENCH_RESET does not clear the latch");
  printf("%s release_gate release (%d failures)\n", fails ? "FAIL" : "PASS", fails);
#endif
  return fails != 0;
}
