/* Test vectors for test_dbc.py: encode random messages with the firmware's own encoder
 * (core/src/bmpc_protocol.c) and print each frame with the field values the decoder returns.
 * Output lines:  F <id hex> <8 data bytes hex>   then   V <name>=<value> ...  */
#include <stdio.h>
#include <stdlib.h>
#include "bmpc/bmpc_protocol.h"

static double rnd(double lo, double hi) { return lo + (hi - lo) * ((double)rand() / RAND_MAX); }

static void print_frames(const bmpc_can_frame_t *f, int n)
{
  int i, k;
  for (i = 0; i < n; ++i) {
    printf("F %03X", (unsigned)f[i].id);
    for (k = 0; k < 8; ++k) printf(" %02X", f[i].data[k]);
    printf("\n");
  }
}

int main(int argc, char **argv)
{
  int n = argc > 1 ? atoi(argv[1]) : 200, t;
  srand(20261108u);
  for (t = 0; t < n; ++t) {
    bmpc_request_t q, qd;
    bmpc_response_t r, rd;
    bmpc_can_frame_t f[6];
    uint16_t idx; float v; uint32_t h; uint16_t c; uint8_t s;

    q.seq = (uint16_t)rand(); q.flags = (uint8_t)(rand() & 0x1F); q.ref_type = 0;
    q.c[0] = (bmpc_real)rnd(-5, 5); q.c[1] = (bmpc_real)rnd(-1, 1); q.c[2] = (bmpc_real)rnd(-0.1, 0.1); q.c[3] = (bmpc_real)rnd(-1e-3, 1e-3);
    q.x_max = (bmpc_real)rnd(0, 80); q.path_counter = (uint16_t)rand();
    q.ref_v = (bmpc_real)rnd(0, 30); q.v_lb = (bmpc_real)rnd(0, 10); q.v_ub = (bmpc_real)rnd(10, 30);
    q.post_cap = (t % 3 == 0) ? (bmpc_real)-1 : (bmpc_real)rnd(0, 30);
    q.snap = (t % 4 == 0) ? (bmpc_real)-1 : (bmpc_real)rnd(0, 30);
    q.velocity = (bmpc_real)rnd(0, 30); q.yaw_rate = (bmpc_real)rnd(-1, 1);
    q.lf_est = (t % 5 == 0) ? (bmpc_real)0 : (bmpc_real)rnd(1, 2);
    q.period = (bmpc_real)rnd(0.02, 0.06); q.odom_age = (bmpc_real)rnd(0, 0.1); q.transport = (bmpc_real)rnd(0, 0.05);
    bmpc_encode_request(&q, f);
    if (bmpc_decode_request(f, &qd) != 0) { fprintf(stderr, "request decode failed\n"); return 1; }
    print_frames(f, 6);
    printf("V REQ Seq=%u Flags=%u RefType=%u C0=%.9g C1=%.9g C2=%.9g C3=%.9g XMax=%.9g PathCounter=%u RefV=%.9g VLb=%.9g VUb=%.9g "
           "PostCapRaw=%d SnapRaw=%d Velocity=%.9g YawRate=%.9g LfEst=%.9g Period=%.9g OdomAge=%.9g Transport=%.9g\n",
           qd.seq, qd.flags, qd.ref_type, (double)qd.c[0], (double)qd.c[1], (double)qd.c[2], (double)qd.c[3], (double)qd.x_max,
           qd.path_counter, (double)qd.ref_v, (double)qd.v_lb, (double)qd.v_ub, qd.post_cap < 0 ? -1 : 0, qd.snap < 0 ? -1 : 0,
           (double)qd.velocity, (double)qd.yaw_rate, (double)qd.lf_est, (double)qd.period, (double)qd.odom_age, (double)qd.transport);
    printf("V REQ2 PostCap=%.9g Snap=%.9g\n", (double)qd.post_cap, (double)qd.snap);

    r.seq = (uint16_t)rand(); r.status = (uint8_t)(rand() % 6); r.iters = (uint8_t)rand();
    r.steer_cmd = (bmpc_real)rnd(-0.6, 0.6); r.accel = (bmpc_real)rnd(0, 1); r.brake = (bmpc_real)rnd(0, 1);
    r.delta0 = (bmpc_real)rnd(-0.6, 0.6); r.vt_final = (bmpc_real)rnd(0, 30); r.vt_raw = (bmpc_real)rnd(0, 30);
    r.state_delay = (bmpc_real)rnd(0, 0.2); r.compute = (bmpc_real)rnd(0, 0.05); r.actuation = (bmpc_real)rnd(0, 0.1);
    r.lf_used = (bmpc_real)rnd(1, 2); r.solve_us = (uint32_t)(rand() % 70000); r.total_us = (uint32_t)(rand() % 70000);
    bmpc_encode_response(&r, f);
    if (bmpc_decode_response(f, &rd) != 0) { fprintf(stderr, "response decode failed\n"); return 1; }
    print_frames(f, 4);
    printf("V RESP Seq=%u Status=%u Iters=%u SteerCmd=%.9g Accel=%.9g Brake=%.9g Delta0=%.9g VtFinal=%.9g StateDelay=%.9g "
           "Compute=%.9g Actuation=%.9g VtRaw=%.9g SolveUs=%u TotalUs=%u LfUsed=%.9g\n",
           rd.seq, rd.status, rd.iters, (double)rd.steer_cmd, (double)rd.accel, (double)rd.brake, (double)rd.delta0, (double)rd.vt_final,
           (double)rd.state_delay, (double)rd.compute, (double)rd.actuation, (double)rd.vt_raw, (unsigned)rd.solve_us, (unsigned)rd.total_us,
           (double)rd.lf_used);

    bmpc_encode_cfg_set((uint16_t)(rand() % 80), (float)rnd(-100, 100), f);
    bmpc_decode_cfg_set(f, &idx, &v);
    print_frames(f, 1);
    printf("V CFGSET CfgIdx=%u CfgValue=%.9g\n", idx, (double)v);
    bmpc_encode_cfg_commit((uint32_t)rand() * 2654435761u, (uint16_t)rand(), f);
    bmpc_decode_cfg_commit(f, &h, &c);
    print_frames(f, 1);
    printf("V CFGCOMMIT CfgHash=%u CfgCount=%u\n", (unsigned)h, c);
    bmpc_encode_cfg_ack((uint32_t)rand() * 2246822519u, (uint16_t)rand(), (uint8_t)rand(), f);
    bmpc_decode_cfg_ack(f, &h, &c, &s);
    print_frames(f, 1);
    printf("V CFGACK AckHash=%u AckCount=%u AckStatus=%u\n", (unsigned)h, c, s);
  }
  return 0;
}
