/* 오프라인 재생: sim_mcu --record 로 남긴 수신 프레임을 같은 앱 코드에 그대로 다시 넣는다.
 * 같은 녹화를 double 빌드와 float 빌드(또는 수정 전·후 코드)에 넣어 응답을 비교하는 back-to-back 시험용.
 *
 *   bmpc_replay_frames <frames.bin> <out.csv>
 * out.csv: src(live|replay), t_us, seq, status, iters, steer_cmd(f32 그대로)
 *
 * 시각: 프레임을 넣을 때 now_us 는 그 프레임의 녹화 시각을 돌려준다. 같은 프레임 처리 중 두 번째 이후 호출
 * (풀이 뒤 시각)은 녹화에서 그 요청 다음에 나간 첫 응답 프레임의 시각을 돌려준다. 그래서 지연 보상의 계산 시간
 * 표본이 실제 주행과 같아진다. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bmpc/bmpc_app.h"

typedef struct {
  int64_t t;
  uint8_t dir;
  bmpc_can_frame_t f;
} rec_t;

static rec_t *g_r;
static long g_n, g_cur, g_calls;
static FILE *g_out;

static int64_t next_tx_time(long i)
{
  long k;
  for (k = i + 1; k < g_n; ++k)
    if (g_r[k].dir == 1u) return g_r[k].t;
  return g_r[i].t;
}

static int64_t hal_now(void *ctx)
{
  (void)ctx;
  g_calls++;
  return (g_calls == 1) ? g_r[g_cur].t : next_tx_time(g_cur);
}

static void put_resp(const char *src, int64_t t, const bmpc_can_frame_t *f)
{
  float steer;
  if (f->id != 0x1E0u) return;
  memcpy(&steer, &f->data[4], 4);
  fprintf(g_out, "%s,%lld,%u,%u,%u,%.9g\n", src, (long long)t, (unsigned)(f->data[0] | (f->data[1] << 8)),
          (unsigned)f->data[2], (unsigned)f->data[3], (double)steer);
}

static int hal_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  put_resp("replay", g_r[g_cur].t, f);
  return 0;
}

int main(int argc, char **argv)
{
  static bmpc_app_t app;
  bmpc_hal_t hal;
  FILE *in;
  long cap = 1 << 20;
  unsigned char b[22];

  if (argc < 3) { fprintf(stderr, "usage: %s frames.bin out.csv\n", argv[0]); return 2; }
  in = fopen(argv[1], "rb");
  g_out = fopen(argv[2], "w");
  if (in == NULL || g_out == NULL) { fprintf(stderr, "open failed\n"); return 1; }
  g_r = malloc((size_t)cap * sizeof *g_r);
  while (fread(b, 1, 22, in) == 22) {
    if (g_n == cap) { cap *= 2; g_r = realloc(g_r, (size_t)cap * sizeof *g_r); }
    memcpy(&g_r[g_n].t, b, 8);
    g_r[g_n].dir = b[8];
    memcpy(&g_r[g_n].f.id, b + 9, 4);
    g_r[g_n].f.dlc = b[13];
    memcpy(g_r[g_n].f.data, b + 14, 8);
    g_n++;
  }
  fclose(in);
  fprintf(g_out, "src,t_us,seq,status,iters,steer\n");
  hal.now_us = hal_now;
  hal.can_send = hal_send;
  hal.ctx = NULL;
  bmpc_app_init(&app, &hal);
  for (g_cur = 0; g_cur < g_n; ++g_cur) {
    if (g_r[g_cur].dir == 1u) { put_resp("live", g_r[g_cur].t, &g_r[g_cur].f); continue; }
    g_calls = 0;
    bmpc_app_on_frame(&app, &g_r[g_cur].f);
  }
  fclose(g_out);
  printf("frames=%ld requests=%u rx_err=%u solver_err=%u real=%s\n", g_n, app.stats.requests, app.stats.rx_errors,
         app.stats.solver_errors, sizeof(bmpc_real) == 4 ? "float" : "double");
  return 0;
}
