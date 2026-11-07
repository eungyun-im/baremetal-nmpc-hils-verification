/* MCU 앱 계층 시험: 프레임 조립, 설정 확정, 오류 처리. HAL 은 가짜 시계와 송신 버퍼다. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmpc/bmpc_app.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                    \
  do {                                                      \
    if (cond) {                                             \
      ++g_pass;                                             \
    } else {                                                \
      ++g_fail;                                             \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);         \
      printf(__VA_ARGS__);                                  \
      printf("\n");                                         \
    }                                                       \
  } while (0)

static bmpc_can_frame_t g_out[64];
static int g_nout = 0;
static int64_t g_t = 1000000;

static int64_t fake_now(void *ctx) { (void)ctx; return g_t; }
static int fake_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  if (g_nout < 64) g_out[g_nout++] = *f;
  return 0;
}

static void send_config(bmpc_app_t *app, const bmpc_config_t *cfg, int corrupt_hash)
{
  bmpc_can_frame_t f;
  uint16_t i;
  for (i = 0; i < BMPC_CONFIG_COUNT; ++i) {
    bmpc_encode_cfg_set(i, bmpc_config_get(cfg, i), &f);
    bmpc_app_on_frame(app, &f);
  }
  bmpc_encode_cfg_commit(bmpc_config_hash(cfg) ^ (corrupt_hash ? 1u : 0u), (uint16_t)BMPC_CONFIG_COUNT, &f);
  bmpc_app_on_frame(app, &f);
}

static void make_request(bmpc_request_t *r, uint16_t seq, double kappa, double v)
{
  memset(r, 0, sizeof(*r));
  r->seq = seq;
  r->c[0] = 0.1;
  r->c[2] = 0.5 * kappa;
  r->x_max = 20;
  r->ref_v = v;
  r->v_lb = 0;
  r->v_ub = v + 1;
  r->post_cap = -1;
  r->snap = -1;
  r->velocity = v;
  r->yaw_rate = 0.05;
  r->period = 0.044;
  r->odom_age = 0.01;
  r->transport = 0.001;
}

static int read_response(bmpc_response_t *resp)
{
  int i;
  if (g_nout < BMPC_RESP_FRAMES) return -9;
  for (i = 0; i < BMPC_RESP_FRAMES; ++i)
    if (g_out[g_nout - BMPC_RESP_FRAMES + i].id != BMPC_ID_RESP_BASE + (uint32_t)i) return -8;
  return bmpc_decode_response(&g_out[g_nout - BMPC_RESP_FRAMES], resp);
}

int main(void)
{
  static bmpc_app_t app;
  static bmpc_controller_t direct;
  bmpc_hal_t hal = {fake_now, fake_send, NULL};
  bmpc_config_t cfg;
  bmpc_request_t req;
  bmpc_response_t resp;
  bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
  int i, k;
  double worst = 0;

  bmpc_config_default(&cfg);
  bmpc_app_init(&app, &hal);

  printf("[not configured]\n");
  make_request(&req, 7, 0, 10);
  bmpc_encode_request(&req, fr);
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&app, &fr[i]);
  CHECK(read_response(&resp) == 0, "response frames");
  CHECK(resp.seq == 7 && resp.status == BMPC_ERR_NOT_CONFIGURED, "seq %u status %u", resp.seq, resp.status);

  printf("[config handshake]\n");
  g_nout = 0;
  send_config(&app, &cfg, 1);
  CHECK(g_nout == 1 && g_out[0].id == BMPC_ID_CFG_ACK, "ack frame");
  {
    uint32_t h;
    uint16_t c;
    uint8_t st;
    bmpc_decode_cfg_ack(&g_out[0], &h, &c, &st);
    CHECK(st == 1 && !app.ctrl.configured, "wrong hash must be rejected (status %u)", st);
  }
  g_nout = 0;
  send_config(&app, &cfg, 0);
  {
    uint32_t h;
    uint16_t c;
    uint8_t st;
    bmpc_decode_cfg_ack(&g_out[0], &h, &c, &st);
    CHECK(st == 0 && h == bmpc_config_hash(&cfg) && c == BMPC_CONFIG_COUNT && app.ctrl.configured, "ack ok");
  }

  printf("[requests]\n");
  /* 앱을 거친 결과와 같은 설정의 controller 를 직접 부른 결과가 같아야 한다 */
  bmpc_controller_init(&direct, &app.staging);
  for (k = 0; k < 200; ++k) {
    bmpc_request_t dec;
    bmpc_response_t r2;
    make_request(&req, (uint16_t)(100 + k), (k % 50 < 25) ? 1.0 / 30.0 : -1.0 / 20.0, 8 + 0.02 * k);
    bmpc_encode_request(&req, fr);
    g_nout = 0;
    for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&app, &fr[i]);
    CHECK(read_response(&resp) == 0 && resp.seq == req.seq && resp.status == BMPC_OK, "k=%d seq %u status %u", k,
          resp.seq, resp.status);
    bmpc_decode_request(fr, &dec);
    bmpc_controller_step(&direct, &dec, fake_now, NULL, &r2);
    worst = fmax(worst, fabs((double)r2.steer_cmd - (double)resp.steer_cmd));
    worst = fmax(worst, fabs((double)r2.accel - (double)resp.accel) - 0.5e-4);
    g_t += 50000;
  }
  CHECK(worst < 1e-6, "app vs direct controller diff %g", worst);
  printf("  200 requests ok, app vs direct controller max diff %.2e\n", worst);

  printf("[errors]\n");
  make_request(&req, 555, 0, 10);
  bmpc_encode_request(&req, fr);
  fr[2].data[1] ^= 0x10u;
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&app, &fr[i]);
  CHECK(read_response(&resp) == 0 && resp.seq == 555 && resp.status == BMPC_ERR_RX, "crc error status %u", resp.status);

  make_request(&req, 556, 0, 10);
  bmpc_encode_request(&req, fr);
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i)
    if (i != 3) bmpc_app_on_frame(&app, &fr[i]);
  CHECK(read_response(&resp) == 0 && resp.seq == 556 && resp.status == BMPC_ERR_RX, "missing frame status %u",
        resp.status);

  /* 트리거 프레임만 오면 (첫 프레임 없음) seq 0, RX 오류 */
  g_nout = 0;
  bmpc_app_on_frame(&app, &fr[5]);
  CHECK(read_response(&resp) == 0 && resp.status == BMPC_ERR_RX, "lone trigger");

  /* 오류 뒤 정상 요청은 다시 정상 처리 */
  make_request(&req, 557, 0, 10);
  bmpc_encode_request(&req, fr);
  g_nout = 0;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) bmpc_app_on_frame(&app, &fr[i]);
  CHECK(read_response(&resp) == 0 && resp.seq == 557 && resp.status == BMPC_OK, "recovered");
  CHECK(app.stats.rx_errors == 3, "rx error count %u", app.stats.rx_errors);

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
