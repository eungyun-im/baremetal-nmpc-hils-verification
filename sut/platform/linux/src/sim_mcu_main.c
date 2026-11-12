/* SILS 가상 MCU. MCU 펌웨어와 같은 앱 코드(bmpc_app)를 리눅스에서 돌린다.
 * 다른 점은 HAL 두 개뿐이다: 시각은 clock_gettime, CAN 은 UDP.
 *
 *   bmpc_sim_mcu [--listen 47001] [--peer-host 127.0.0.1] [--peer-port 47002] [--stats 10] [--record DIR]
 * --record DIR ("-" 이면 녹화 안 함): 받은 프레임과 보낸 프레임을 시각과 함께 DIR/frames_<시작 epoch>.bin 에 남긴다 (오프라인 재생 시험용).
 *   레코드 = int64 t_us | uint8 dir(0 수신, 1 송신) | uint32 id | uint8 dlc | uint8 data[8], 리틀엔디언, 패딩 없음 (22 B).
 * roslaunch 가 붙이는 __name:= 같은 인자는 무시한다. */
#define _POSIX_C_SOURCE 200809L
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bmpc/bmpc_app.h"
#include "bmpc/bmpc_udp.h"

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int sig) { (void)sig; g_stop = 1; }

static int64_t hal_now_us(void *ctx)
{
  struct timespec ts;
  (void)ctx;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

static FILE *g_rec = NULL;
static int64_t hal_now_us(void *ctx);

static void rec_frame(int64_t t_us, uint8_t dir, const bmpc_can_frame_t *f)
{
  if (g_rec == NULL) return;
  fwrite(&t_us, 8, 1, g_rec);
  fwrite(&dir, 1, 1, g_rec);
  fwrite(&f->id, 4, 1, g_rec);
  fwrite(&f->dlc, 1, 1, g_rec);
  fwrite(f->data, 1, 8, g_rec);
}

static int hal_can_send(void *ctx, const bmpc_can_frame_t *f)
{
  rec_frame(hal_now_us(NULL), 1u, f);
  return bmpc_udp_send((bmpc_udp_t *)ctx, f);
}

int main(int argc, char **argv)
{
  static bmpc_app_t app; /* controller 작업 메모리가 커서 정적 영역에 둔다 (MCU 와 같게) */
  bmpc_udp_t udp;
  bmpc_hal_t hal;
  int listen_port = BMPC_UDP_PORT_MCU, peer_port = BMPC_UDP_PORT_PC;
  const char *peer_host = "127.0.0.1";
  double stats_period = 10.0;
  int64_t next_stats;
  uint32_t last_requests = 0;
  int i;

  for (i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--listen") && i + 1 < argc) listen_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--peer-host") && i + 1 < argc) peer_host = argv[++i];
    else if (!strcmp(argv[i], "--peer-port") && i + 1 < argc) peer_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--stats") && i + 1 < argc) stats_period = atof(argv[++i]);
    else if (!strcmp(argv[i], "--record") && i + 1 < argc && strcmp(argv[i + 1], "-") != 0) {
      char path[512];
      snprintf(path, sizeof path, "%s/frames_%ld.bin", argv[++i], (long)time(NULL));
      g_rec = fopen(path, "wb");
      if (g_rec == NULL) fprintf(stderr, "[sim_mcu] cannot open %s\n", path);
      else printf("[sim_mcu] recording frames to %s\n", path);
    }
  }

  if (bmpc_udp_open(&udp, (uint16_t)listen_port, peer_host, (uint16_t)peer_port) != 0) {
    fprintf(stderr, "[sim_mcu] cannot open UDP %d -> %s:%d\n", listen_port, peer_host, peer_port);
    return 1;
  }
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  hal.now_us = hal_now_us;
  hal.can_send = hal_can_send;
  hal.ctx = &udp;
  bmpc_app_init(&app, &hal);
  printf("[sim_mcu] virtual MCU listening on 127.0.0.1:%d, replies to %s:%d, bmpc_real=%s, config fields=%d\n",
         listen_port, peer_host, peer_port, sizeof(bmpc_real) == 4 ? "float" : "double", (int)BMPC_CONFIG_COUNT);
  fflush(stdout);

  next_stats = hal_now_us(NULL) + (int64_t)(stats_period * 1e6);
  while (!g_stop) {
    bmpc_can_frame_t f;
    const int r = bmpc_udp_recv(&udp, &f, 100000);
    if (r == 1) { rec_frame(hal_now_us(NULL), 0u, &f); bmpc_app_on_frame(&app, &f); }
    if (stats_period > 0 && hal_now_us(NULL) >= next_stats) {
      const bmpc_app_stats_t *s = &app.stats;
      printf("[sim_mcu] configured=%d requests=%u (+%u) rx_err=%u solver_err=%u solve_us last/max %u/%u total_us last/max %u/%u\n",
             app.ctrl.configured, s->requests, s->requests - last_requests, s->rx_errors, s->solver_errors,
             s->last_solve_us, s->max_solve_us, s->last_total_us, s->max_total_us);
      fflush(stdout);
      last_requests = s->requests;
      next_stats += (int64_t)(stats_period * 1e6);
    }
  }
  printf("[sim_mcu] stop: requests=%u rx_err=%u solver_err=%u max_solve_us=%u max_total_us=%u\n",
         app.stats.requests, app.stats.rx_errors, app.stats.solver_errors, app.stats.max_solve_us,
         app.stats.max_total_us);
  if (g_rec != NULL) fclose(g_rec);
  bmpc_udp_close(&udp);
  return 0;
}
