#include "bmpc/bmpc_app.h"

#include <string.h>

void bmpc_app_init(bmpc_app_t *app, const bmpc_hal_t *hal)
{
  memset(app, 0, sizeof(*app));
  app->hal = *hal;
  bmpc_config_default(&app->staging);
  app->ctrl.configured = 0; /* 설정 확정 전에는 요청에 NOT_CONFIGURED 로 답한다 */
}

static void send_response(bmpc_app_t *app, const bmpc_response_t *resp)
{
  bmpc_can_frame_t out[BMPC_RESP_FRAMES];
  int i;
  bmpc_encode_response(resp, out);
  for (i = 0; i < BMPC_RESP_FRAMES; ++i) app->hal.can_send(app->hal.ctx, &out[i]);
}

static void on_config_commit(bmpc_app_t *app, const bmpc_can_frame_t *f)
{
  uint32_t hash_pc, hash_mcu;
  uint16_t count_pc;
  uint8_t status;
  bmpc_can_frame_t ack;

  bmpc_decode_cfg_commit(f, &hash_pc, &count_pc);
  hash_mcu = bmpc_config_hash(&app->staging);
  status = (hash_mcu == hash_pc && app->cfg_count == count_pc && count_pc == (uint16_t)BMPC_CONFIG_COUNT) ? 0u : 1u;
  if (status == 0u) {
    bmpc_controller_init(&app->ctrl, &app->staging);
    ++app->stats.configs;
  }
  bmpc_encode_cfg_ack(hash_mcu, app->cfg_count, status, &ack);
  app->hal.can_send(app->hal.ctx, &ack);
  /* 다음 설정 묶음을 받을 준비 (값은 남겨 둔다) */
  memset(app->cfg_seen, 0, sizeof(app->cfg_seen));
  app->cfg_count = 0;
}

static void on_request_complete(bmpc_app_t *app)
{
  bmpc_request_t req;
  bmpc_response_t resp;
  const int64_t t_rx = app->hal.now_us(app->hal.ctx);
  const int complete = app->req_mask == (uint8_t)((1u << BMPC_REQ_FRAMES) - 1u);
  const int rc = complete ? bmpc_decode_request(app->req, &req) : -1;

  ++app->stats.requests;
  if (rc != 0) {
    memset(&resp, 0, sizeof(resp));
    /* 첫 프레임이 있으면 seq 를 돌려줘서 PC 가 어느 요청이 깨졌는지 알게 한다 */
    resp.seq = (app->req_mask & 1u) ? (uint16_t)(app->req[0].data[0] | ((uint16_t)app->req[0].data[1] << 8)) : 0u;
    resp.status = BMPC_ERR_RX;
    ++app->stats.rx_errors;
    send_response(app, &resp);
    return;
  }
  bmpc_controller_step(&app->ctrl, &req, app->hal.now_us, app->hal.ctx, &resp);
  if (resp.status != BMPC_OK) ++app->stats.solver_errors;
  /* 처리 시간: 요청 수신 완료 ~ 응답 송신 직전. PC 는 왕복에서 이 값을 빼 CAN 전송 시간을 구한다. */
  resp.total_us = (uint32_t)(app->hal.now_us(app->hal.ctx) - t_rx);
  app->stats.last_total_us = resp.total_us;
  app->stats.last_solve_us = resp.solve_us;
  if (resp.total_us > app->stats.max_total_us) app->stats.max_total_us = resp.total_us;
  if (resp.solve_us > app->stats.max_solve_us) app->stats.max_solve_us = resp.solve_us;
  send_response(app, &resp);
}

void bmpc_app_on_frame(bmpc_app_t *app, const bmpc_can_frame_t *f)
{
  if (f->id == BMPC_ID_CFG_SET) {
    uint16_t idx;
    float v;
    bmpc_decode_cfg_set(f, &idx, &v);
    if (bmpc_config_set(&app->staging, idx, v) == 0 && !app->cfg_seen[idx]) {
      app->cfg_seen[idx] = 1;
      ++app->cfg_count;
    }
    return;
  }
  if (f->id == BMPC_ID_CFG_COMMIT) {
    on_config_commit(app, f);
    return;
  }
  if (f->id >= BMPC_ID_REQ_BASE && f->id < BMPC_ID_REQ_BASE + BMPC_REQ_FRAMES) {
    const unsigned i = (unsigned)(f->id - BMPC_ID_REQ_BASE);
    if (i == 0) app->req_mask = 0; /* 새 요청 시작 */
    app->req[i] = *f;
    app->req_mask |= (uint8_t)(1u << i);
    if (i == BMPC_REQ_FRAMES - 1) {
      on_request_complete(app);
      app->req_mask = 0;
    }
  }
  /* 그 밖의 ID 는 무시한다 */
}
