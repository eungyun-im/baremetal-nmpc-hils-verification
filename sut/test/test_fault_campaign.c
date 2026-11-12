/* 결함 주입 캠페인 (호스트 통합, SR-01 / SR-04). MPC 코어 + 통합 계층 + 안전 제어기, 보드와 같은 정의(float, PLAUS, 시험 훅).
 * 같은 로직을 HILS 랩보다 훨씬 많이 돌려, 안전 로직이 조건에 따라 맞게 동작하는지를 통계로 본다. 보드 타이밍은 보지 못한다.
 *
 * 한 번의 실행: 무작위 속도·곡률로 주행 -> 무작위 주기에 결함 시작 -> 결함 길이 d 주기 -> 결함 뒤 3 s 관찰.
 *   결함 종류 solve : 0x7F0 종류 4 (풀이 실패, SWR-45). d 주기 동안 응답 status 3
 *   결함 종류 loss  : 요청이 오지 않음 (명령 두절, TSR-03). d 주기 동안 요청 없음
 *   그 밖에: 결함 시작 때 경로가 곡선 -> 직선으로 바뀌는 경우(곡선 출구)를 절반 섞는다. 안전 주기와 요청의 위상(0~40 ms)도 무작위
 * 판정 (test/req_test_spec.md SR-01, SR-04):
 *   d <= 2 (100 ms 이하): SAFE_STOP 없음
 *   d >= 6 (300 ms 이상): SAFE_STOP, 첫 결함 주기부터 290 ms 이하
 *   d = 3~5: 경계 구간, 결과만 기록
 *   결함 없는 실행: SAFE_STOP 없음 (오검출)
 * 결함 길이는 층화한다: 짧음 30 %, 경계 20 %, 김 50 %. 결함 없는 실행은 10 %.
 * 사용: bmpc_test_fault_campaign [실행 수, 기본 2000] [결과 CSV]  -> 종료 코드 0 = 모든 판정 통과 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bmpc_safety.h"

static uint32_t g_rng = 20261111u;
static double urand(void)
{
  g_rng = g_rng * 1664525u + 1013904223u;
  return (double)(g_rng >> 8) / 16777216.0;
}

static int64_t g_us;
static uint32_t g_ms;
static bmpc_can_frame_t g_out;
static int64_t fake_now(void *ctx) { (void)ctx; return g_us; }
static int fake_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  if (f->id == BS_ID_SAFE_OUT) g_out = *f;
  return 0;
}

static void send_config(bs_app_t *a)
{
  bmpc_config_t cfg;
  bmpc_can_frame_t f;
  uint16_t i;
  bmpc_config_default(&cfg);
  cfg.sqp_iters = 2;
  cfg.qp_max_iters = 16;
  for (i = 0; i < BMPC_CONFIG_COUNT; ++i) {
    bmpc_encode_cfg_set(i, bmpc_config_get(&cfg, i), &f);
    bs_on_mpc_frame(a, &f);
  }
  bmpc_encode_cfg_commit(bmpc_config_hash(&cfg), (uint16_t)BMPC_CONFIG_COUNT, &f);
  bs_on_mpc_frame(a, &f);
}

static void send_esc(bs_app_t *a, float kph)
{
  static uint8_t alive = 0;
  bmpc_can_frame_t f;
  const uint16_t raw = (uint16_t)(kph * 100.0f + 0.5f);
  memset(&f, 0, sizeof(f));
  f.id = BS_ID_ESC_SPEED;
  f.dlc = 8;
  f.data[0] = (uint8_t)raw;
  f.data[1] = (uint8_t)(raw >> 8);
  f.data[6] = (uint8_t)(alive++ & 0x0F);
  sc_frame_seal(f.data, SC_DATA_ID_ESC_SPEED);
  bs_on_safety_frame(a, &f, g_ms);
}

static void send_hook(bs_app_t *a, uint16_t ms)
{
  bmpc_can_frame_t f;
  memset(&f, 0, sizeof(f));
  f.id = BS_ID_BENCH_RESET;
  f.dlc = 6;
  f.data[0] = BS_HOOK_MAGIC;
  f.data[1] = BS_HOOK_SOLVE_FAIL;
  f.data[4] = (uint8_t)(ms & 0xFFu);
  f.data[5] = (uint8_t)(ms >> 8);
  bs_on_safety_frame(a, &f, g_ms);
}

/* 10 ms 안전 주기 하나 */
static void tick(bs_app_t *a, double v, int esc)
{
  g_ms += 10;
  g_us += 10000;
  if (esc) send_esc(a, (float)(v * 3.6));
  bs_safety_tick(a, g_ms);
  bs_note_applied(a, g_us);
}

typedef struct {
  int kind;   /* 0 없음, 1 solve, 2 loss */
  int d;      /* 결함 주기 수 */
  double v, kappa;
  int exit_curve, phase_ticks, start_cycle;
  int stopped;          /* SAFE_STOP 에 갔는가 */
  int stop_ms;          /* 첫 결함 주기 시작부터 SAFE_STOP 이 처음 보인 안전 주기까지 (10 ms 해상도) */
  int stop_before_fault; /* 결함 전 SAFE_STOP (오검출) */
} run_t;

static void run(run_t *r)
{
  static bs_app_t a;
  bs_cfg_t bcfg;
  sc_config_t scfg;
  bmpc_hal_t hal;
  const int T_END = r->start_cycle + r->d + 60;
  int c, k;
  int64_t fault_t0 = -1;
  g_us = 1000000;
  g_ms = 1000;
  hal.now_us = fake_now;
  hal.can_send = fake_send;
  hal.ctx = NULL;
  bs_default_cfg(&bcfg);
  bs_default_safety_config(&scfg);
  bs_init(&a, &bcfg, &scfg, &hal);
  send_config(&a);
  r->stopped = 0;
  r->stop_ms = -1;
  r->stop_before_fault = 0;
  for (c = 0; c < T_END; ++c) {
    const int in_fault = (r->kind != 0) && (c >= r->start_cycle) && (c < r->start_cycle + r->d);
    const double kap = (r->exit_curve && c >= r->start_cycle) ? 0.0 : r->kappa;
    if (r->kind != 0 && c == r->start_cycle) fault_t0 = g_us + (int64_t)r->phase_ticks * 10000; /* 결함 시작 = 이 주기의 요청 시각 */
    if (r->kind == 1 && c == r->start_cycle) send_hook(&a, (uint16_t)(r->d * 50 - 25)); /* d 주기만 실패 */
    /* 요청은 이 주기의 phase_ticks 번째 안전 주기 뒤에 온다 (안전 주기와 요청의 위상 0~40 ms) */
    for (k = 0; k < r->phase_ticks; ++k) {
      tick(&a, r->v, k == 0);
      if (a.out.state == SC_STATE_SAFE_STOP && !r->stopped) {
        r->stopped = 1;
        if (fault_t0 < 0) r->stop_before_fault = 1;
        else r->stop_ms = (int)((g_us - fault_t0) / 1000);
      }
    }
    if (!(r->kind == 2 && in_fault)) {
      bmpc_request_t q;
      bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
      memset(&q, 0, sizeof(q));
      q.seq = (uint16_t)(c + 1);
      q.c[2] = (bmpc_real)(0.5 * kap);
      q.x_max = 20;
      q.ref_v = (bmpc_real)r->v;
      q.v_ub = (bmpc_real)(r->v + 1);
      q.post_cap = -1;
      q.snap = -1;
      q.velocity = (bmpc_real)r->v;
      q.yaw_rate = (bmpc_real)(r->v * kap);
      q.period = (bmpc_real)0.05;
      q.odom_age = (bmpc_real)0.002;
      bmpc_encode_request(&q, fr);
      for (k = 0; k < BMPC_REQ_FRAMES; ++k) bs_on_mpc_frame(&a, &fr[k]);
      (void)bs_feed_ctrl(&a, g_ms);
    }
    for (k = r->phase_ticks; k < 5; ++k) {
      tick(&a, r->v, k == 0);
      if (a.out.state == SC_STATE_SAFE_STOP && !r->stopped) {
        r->stopped = 1;
        if (fault_t0 < 0) r->stop_before_fault = 1;
        else r->stop_ms = (int)((g_us - fault_t0) / 1000);
      }
    }
  }
}

int main(int argc, char **argv)
{
  const int n = argc > 1 ? atoi(argv[1]) : 2000;
  FILE *out = fopen(argc > 2 ? argv[2] : "fault_campaign.csv", "w");
  int i, fails = 0;
  int cnt[3][3] = {{0}}, ok[3][3] = {{0}}; /* [kind][class 0:short 1:boundary 2:long] */
  int stop_max[3] = {0, 0, 0}, bstop[3][6] = {{0}}, bcnt[3][6] = {{0}};
  int stop_min[3] = {100000, 100000, 100000};
  if (out == NULL) return 2;
  fprintf(out, "run,kind,d,v,kappa,exit_curve,phase_ticks,start_cycle,stopped,stop_ms,stop_before_fault,verdict\n");
  for (i = 0; i < n; ++i) {
    run_t r;
    int cls, pass;
    const char *verdict;
    double kmax;
    memset(&r, 0, sizeof(r));
    r.kind = i % 10 == 0 ? 0 : (i % 2 == 0 ? 1 : 2);
    r.v = 3.0 + 17.0 * urand();
    kmax = fmin(1.0 / 15.0, 7.0 / (r.v * r.v)); /* 횡가속도 7 m/s2 안 */
    r.kappa = urand() < 0.3 ? 0.0 : (urand() < 0.5 ? -1.0 : 1.0) * kmax * (0.2 + 0.8 * urand());
    r.exit_curve = r.kappa != 0.0 && urand() < 0.5;
    r.phase_ticks = (int)(urand() * 5.0);
    r.start_cycle = 30 + (int)(urand() * 20.0);
    { /* 결함 길이 층화: 짧음(1~2) 30 %, 경계(3~5) 20 %, 김(6~24) 50 % */
      const double u = urand();
      r.d = r.kind == 0 ? 0 : (u < 0.3 ? 1 + (int)(urand() * 2.0) : (u < 0.5 ? 3 + (int)(urand() * 3.0) : 6 + (int)(urand() * 19.0)));
    }
    run(&r);
    cls = r.d <= 2 ? 0 : (r.d <= 5 ? 1 : 2);
    if (r.kind == 0) {
      pass = !r.stopped;
      verdict = pass ? "pass" : "FAIL_false_stop";
      cls = 0;
    } else if (r.stop_before_fault) {
      pass = 0;
      verdict = "FAIL_stop_before_fault";
    } else if (cls == 0) {
      pass = !r.stopped;
      verdict = pass ? "pass" : "FAIL_stop_on_short";
    } else if (cls == 2) {
      pass = r.stopped && r.stop_ms >= 0 && r.stop_ms <= 290;
      verdict = pass ? "pass" : "FAIL_no_stop_or_late";
    } else {
      pass = 1;
      verdict = r.stopped ? "boundary_stop" : "boundary_hold";
    }
    ++cnt[r.kind][cls];
    if (cls == 1 && r.kind != 0) { ++bcnt[r.kind][r.d]; bstop[r.kind][r.d] += r.stopped; }
    ok[r.kind][cls] += pass;
    if (!pass) ++fails;
    if (r.stopped && r.stop_ms >= 0) {
      if (r.stop_ms > stop_max[r.kind]) stop_max[r.kind] = r.stop_ms;
      if (r.stop_ms < stop_min[r.kind]) stop_min[r.kind] = r.stop_ms;
    }
    fprintf(out, "%d,%d,%d,%.3f,%.5f,%d,%d,%d,%d,%d,%d,%s\n", i, r.kind, r.d, r.v, r.kappa, r.exit_curve, r.phase_ticks,
            r.start_cycle, r.stopped, r.stop_ms, r.stop_before_fault, verdict);
  }
  fclose(out);
  printf("runs %d\n", n);
  printf("  no fault          : %d runs, no false SAFE_STOP %d\n", cnt[0][0], ok[0][0]);
  printf("  solve fail  d<=2  : %d runs, pass %d | d 3-5: %d (report) | d>=6: %d runs, pass %d | SAFE_STOP %d..%d ms\n",
         cnt[1][0], ok[1][0], cnt[1][1], cnt[1][2], ok[1][2], stop_min[1], stop_max[1]);
  printf("  command loss d<=2 : %d runs, pass %d | d 3-5: %d (report) | d>=6: %d runs, pass %d | SAFE_STOP %d..%d ms\n",
         cnt[2][0], ok[2][0], cnt[2][1], cnt[2][2], ok[2][2], stop_min[2], stop_max[2]);
  for (i = 1; i <= 2; ++i)
    printf("  %s boundary SAFE_STOP share: d=3 %d/%d, d=4 %d/%d, d=5 %d/%d\n", i == 1 ? "solve fail  " : "command loss", bstop[i][3], bcnt[i][3],
           bstop[i][4], bcnt[i][4], bstop[i][5], bcnt[i][5]);
  printf("RESULT: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
  return fails ? 1 : 0;
}
