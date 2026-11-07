/* Requirement-based tests for the safety controller (third_party/kuuve_safety).
 * Expected behavior comes from docs/acceptance_criteria.md and the TSR comments in
 * safety_ctrl.h, not from reading the implementation. Times avoid exact boundaries. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "safety_ctrl.h"

static int g_pass, g_fail;
#define CHECK(c, ...) do { if (c) { g_pass++; } else { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void put_i16(uint8_t *p, int v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); }

static void ctrl(sc_t *sc, float steer, float accel, uint8_t alive, uint32_t t, int corrupt)
{
  uint8_t d[8] = {0};
  put_i16(&d[0], (int)lroundf(steer * 100.0f)); put_i16(&d[2], (int)lroundf(accel * 100.0f));
  d[6] = (uint8_t)(alive & 0x0F); d[7] = sc_crc8_e2e(SC_DATA_ID_CTRL_CMD, d, 7u);
  if (corrupt) d[7] ^= 0x5A;
  sc_on_ctrl_frame(sc, d, t);
}

static void esc(sc_t *sc, float kph, uint32_t t, int corrupt)
{
  uint8_t d[8] = {0};
  put_i16(&d[0], (int)lroundf(kph * 100.0f)); d[7] = sc_crc8_e2e(SC_DATA_ID_ESC_SPEED, d, 7u);
  if (corrupt) d[7] ^= 0x5A;
  sc_on_esc_frame(sc, d, t);
}

static void pedal(sc_t *sc, float brake_pct, uint32_t t)
{
  uint8_t d[8] = {0};
  d[1] = (uint8_t)lroundf(brake_pct / 0.4f); d[7] = sc_crc8_e2e(SC_DATA_ID_VCU_PEDAL, d, 7u);
  sc_on_pedal_frame(sc, d, t);
}

/* Reach NORMAL with steady commands every 50 ms and speed every 10 ms, as on the bench. */
static uint32_t start(sc_t *sc, sc_out_t *o, float accel)
{
  sc_config_t c; uint32_t t; uint8_t a = 0;
  sc_default_config(&c); sc_init(sc, &c);
  for (t = 0; t <= 500u; t += 10u) {
    if (t % 50u == 0u) ctrl(sc, 0.0f, accel, a++, t, 0);
    esc(sc, 30.0f, t, 0);
    sc_step(sc, t, o);
  }
  return t;
}

static void test_tsr03_command_timeout(void)
{
  sc_t sc; sc_out_t o; uint32_t t = start(&sc, &o, 0.5f), t0 = t - 10u; uint8_t seen = 0;
  printf("[TSR-03: command blackout -> SAFE_STOP]\n");
  CHECK(o.state == SC_STATE_NORMAL, "NORMAL before the fault (%d)", (int)o.state);
  for (; t <= t0 + 60u; t += 10u) { esc(&sc, 30.0f, t, 0); sc_step(&sc, t, &o); }
  CHECK(o.state == SC_STATE_NORMAL, "still NORMAL 60 ms after the last command (%d)", (int)o.state);
  for (; t <= t0 + 290u; t += 10u) { esc(&sc, 30.0f, t, 0); sc_step(&sc, t, &o); seen |= o.flags; }
  CHECK(o.state == SC_STATE_SAFE_STOP, "SAFE_STOP within 290 ms (%d)", (int)o.state);
  CHECK(o.accel_mps2 < 0.0f, "SAFE_STOP brakes (%.2f)", (double)o.accel_mps2);
  CHECK(seen & SC_FLAG_TIMEOUT, "timeout flag raised in the transition cycle");   /* flags are per cycle */
}

static void test_tsr09_speed_loss(void)
{
  sc_t sc; sc_out_t o; uint32_t t = start(&sc, &o, 1.0f), t0 = t - 10u; uint8_t a = 20;
  printf("[TSR-09: vehicle speed loss -> DEGRADED, no SAFE_STOP]\n");
  for (; t <= t0 + 290u; t += 10u) { if (t % 50u == 0u) ctrl(&sc, 0.0f, 1.0f, a++, t, 0); sc_step(&sc, t, &o); }
  CHECK(o.state == SC_STATE_DEGRADED, "DEGRADED within 290 ms (%d)", (int)o.state);
  CHECK(o.accel_mps2 <= 0.0f, "no acceleration while speed is lost (%.2f)", (double)o.accel_mps2);
  CHECK(o.flags2 & SC_FLAG2_SPEED_TIMEOUT, "speed timeout flag set");
  CHECK(!(o.inputs_fresh & SC_IN_SPEED), "speed reported as not fresh");
}

static void test_tsr08_bus_off(void)
{
  sc_t sc; sc_out_t o; uint32_t t = start(&sc, &o, 0.5f);
  printf("[TSR-08: CAN bus-off -> SAFE_STOP]\n");
  sc_on_bus_off(&sc); sc_step(&sc, t, &o);
  CHECK(o.state == SC_STATE_SAFE_STOP, "SAFE_STOP after bus-off (%d)", (int)o.state);
  CHECK(o.bus_off_seen == 1u, "bus-off reported");
  CHECK(o.accel_mps2 < 0.0f, "SAFE_STOP brakes (%.2f)", (double)o.accel_mps2);
}

static void test_tsr07_corrupted_frames(void)
{
  sc_t sc; sc_out_t o; uint32_t t = start(&sc, &o, 0.5f), k; uint8_t a = 20; float before;
  printf("[TSR-07: corrupted frames are dropped, no SAFE_STOP]\n");
  before = sc.speed_kph;
  esc(&sc, 90.0f, t, 1); sc_step(&sc, t, &o);
  CHECK(fabsf(sc.speed_kph - before) < 1e-3f, "corrupted speed ignored (%.2f)", (double)sc.speed_kph);
  CHECK(o.flags2 & SC_FLAG2_VEH_CRC, "vehicle CRC flag set");
  ctrl(&sc, 20.0f, 2.0f, a++, t + 10u, 1); sc_step(&sc, t + 10u, &o);
  CHECK(fabsf(o.steer_deg) < 1.0f, "corrupted command ignored (%.2f deg)", (double)o.steer_deg);
  CHECK(o.flags & SC_FLAG_CRC_ERR, "command CRC flag set");
  for (k = 0; k < 30u; ++k) {   /* three corrupted speed frames in every ten, commands keep coming */
    t += 10u;
    if (t % 50u == 0u) ctrl(&sc, 0.0f, 0.5f, a++, t, 0);
    esc(&sc, 30.0f, t, (k % 10u) < 3u);
    sc_step(&sc, t, &o);
    CHECK(o.state != SC_STATE_SAFE_STOP, "no SAFE_STOP from dropped frames at %u ms", (unsigned)t);
  }
}

static void test_tsr10_driver_brake(void)
{
  sc_t sc; sc_out_t o; uint32_t t = start(&sc, &o, 1.0f), k; uint8_t a = 20;
  printf("[TSR-10: driver brake -> no acceleration, no SAFE_STOP]\n");
  CHECK(o.accel_mps2 > 0.0f, "accelerating before the brake (%.2f)", (double)o.accel_mps2);
  for (k = 0; k < 20u; ++k, t += 10u) {
    if (t % 50u == 0u) ctrl(&sc, 0.0f, 1.0f, a++, t, 0);
    esc(&sc, 30.0f, t, 0); pedal(&sc, 50.0f, t); sc_step(&sc, t, &o);
  }
  CHECK(o.accel_mps2 <= 0.0f, "no acceleration under driver brake (%.2f)", (double)o.accel_mps2);
  CHECK(o.flags2 & SC_FLAG2_DRIVER_OVERRIDE, "driver override flag set");
  CHECK(o.state != SC_STATE_SAFE_STOP, "no SAFE_STOP (%d)", (int)o.state);
}

int main(void)
{
  test_tsr03_command_timeout(); test_tsr09_speed_loss(); test_tsr08_bus_off(); test_tsr07_corrupted_frames(); test_tsr10_driver_brake();
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail != 0;
}
