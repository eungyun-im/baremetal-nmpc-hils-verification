#include "bmpc/bmpc_protocol.h"

#include <string.h>

#include "bmpc/bmpc_math.h"

#define U16_NONE 0xFFFFu

uint16_t bmpc_crc16(const uint8_t *data, size_t len, uint16_t crc)
{
  size_t i;
  int b;
  for (i = 0; i < len; ++i) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (b = 0; b < 8; ++b) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* ---- 바이트 쓰기·읽기 (little-endian) ---- */

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_f32(uint8_t *p, bmpc_real v)
{
  const float f = (float)v;
  uint32_t bits;
  memcpy(&bits, &f, sizeof(bits));
  put_u32(p, bits);
}

static bmpc_real get_f32(const uint8_t *p)
{
  const uint32_t bits = get_u32(p);
  float f;
  memcpy(&f, &bits, sizeof(f));
  return (bmpc_real)f;
}

/* 배율 양자화. 반올림하고 표현 범위로 포화한다. */
static uint16_t q_u16(bmpc_real v, bmpc_real scale, uint16_t max_code)
{
  bmpc_real c;
  if (!bm_isfinite(v) || v <= 0) return 0;
  c = v / scale + (bmpc_real)0.5;
  if (c >= (bmpc_real)max_code) return max_code;
  return (uint16_t)c;
}

static int16_t q_i16(bmpc_real v, bmpc_real scale)
{
  bmpc_real c;
  if (!bm_isfinite(v)) return 0;
  c = v / scale;
  c += (c >= 0) ? (bmpc_real)0.5 : (bmpc_real)-0.5;
  if (c >= (bmpc_real)32767) return 32767;
  if (c <= (bmpc_real)-32767) return -32767;
  return (int16_t)c;
}

/* 선택 값: 음수(없음)는 0xFFFF, 나머지는 0..0xFFFE */
static uint16_t q_opt_u16(bmpc_real v, bmpc_real scale)
{
  if (!bm_isfinite(v) || v < 0) return U16_NONE;
  return q_u16(v, scale, (uint16_t)(U16_NONE - 1u));
}

static bmpc_real dq_opt_u16(uint16_t code, bmpc_real scale)
{
  return code == U16_NONE ? (bmpc_real)-1 : (bmpc_real)code * scale;
}

static void frame_init(bmpc_can_frame_t *f, uint32_t id)
{
  f->id = id;
  f->dlc = 8;
  memset(f->data, 0, sizeof(f->data));
}

/* 프레임 묶음의 CRC: 모든 바이트에서 마지막 프레임 6,7 번 바이트만 뺀다 */
static uint16_t group_crc(const bmpc_can_frame_t *fr, int n)
{
  uint16_t crc = 0xFFFFu;
  int i;
  for (i = 0; i < n - 1; ++i) crc = bmpc_crc16(fr[i].data, 8, crc);
  return bmpc_crc16(fr[n - 1].data, 6, crc);
}

/* ---- 요청 ---- */

#define S_POS ((bmpc_real)0.01)  /* m */
#define S_VEL ((bmpc_real)0.01)  /* m/s */
#define S_YAWR ((bmpc_real)1e-4) /* rad/s */
#define S_LF ((bmpc_real)1e-3)   /* m */
#define S_TIME ((bmpc_real)1e-5) /* s */
#define S_PEDAL ((bmpc_real)1e-4)
#define S_ANG ((bmpc_real)1e-4) /* rad */

void bmpc_encode_request(const bmpc_request_t *r, bmpc_can_frame_t out[BMPC_REQ_FRAMES])
{
  int i;
  uint16_t crc;
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) frame_init(&out[i], BMPC_ID_REQ_BASE + (uint32_t)i);

  put_u16(&out[0].data[0], r->seq);
  out[0].data[2] = r->flags;
  out[0].data[3] = r->ref_type;
  put_f32(&out[0].data[4], r->c[0]);

  put_f32(&out[1].data[0], r->c[1]);
  put_f32(&out[1].data[4], r->c[2]);

  put_f32(&out[2].data[0], r->c[3]);
  put_u16(&out[2].data[4], q_u16(r->x_max, S_POS, 0xFFFFu));
  put_u16(&out[2].data[6], r->path_counter);

  put_u16(&out[3].data[0], q_u16(r->ref_v, S_VEL, 0xFFFFu));
  put_u16(&out[3].data[2], q_u16(r->v_lb, S_VEL, 0xFFFFu));
  put_u16(&out[3].data[4], q_u16(r->v_ub, S_VEL, 0xFFFFu));
  put_u16(&out[3].data[6], q_opt_u16(r->post_cap, S_VEL));

  put_u16(&out[4].data[0], q_opt_u16(r->snap, S_VEL));
  put_u16(&out[4].data[2], q_u16(r->velocity, S_VEL, 0xFFFFu));
  put_u16(&out[4].data[4], (uint16_t)q_i16(r->yaw_rate, S_YAWR));
  put_u16(&out[4].data[6], q_u16(r->lf_est, S_LF, 0xFFFFu));

  put_u16(&out[5].data[0], q_u16(r->period, S_TIME, 0xFFFFu));
  put_u16(&out[5].data[2], q_u16(r->odom_age, S_TIME, 0xFFFFu));
  put_u16(&out[5].data[4], q_u16(r->transport, S_TIME, 0xFFFFu));
  crc = group_crc(out, BMPC_REQ_FRAMES);
  put_u16(&out[5].data[6], crc);
}

int bmpc_decode_request(const bmpc_can_frame_t in[BMPC_REQ_FRAMES], bmpc_request_t *r)
{
  int i;
  memset(r, 0, sizeof(*r));
  for (i = 0; i < BMPC_REQ_FRAMES; ++i) {
    if (in[i].id != BMPC_ID_REQ_BASE + (uint32_t)i || in[i].dlc != 8) return -1;
  }
  r->seq = get_u16(&in[0].data[0]);
  if (group_crc(in, BMPC_REQ_FRAMES) != get_u16(&in[5].data[6])) return -2;

  r->flags = in[0].data[2];
  r->ref_type = in[0].data[3];
  r->c[0] = get_f32(&in[0].data[4]);
  r->c[1] = get_f32(&in[1].data[0]);
  r->c[2] = get_f32(&in[1].data[4]);
  r->c[3] = get_f32(&in[2].data[0]);
  r->x_max = (bmpc_real)get_u16(&in[2].data[4]) * S_POS;
  r->path_counter = get_u16(&in[2].data[6]);

  r->ref_v = (bmpc_real)get_u16(&in[3].data[0]) * S_VEL;
  r->v_lb = (bmpc_real)get_u16(&in[3].data[2]) * S_VEL;
  r->v_ub = (bmpc_real)get_u16(&in[3].data[4]) * S_VEL;
  r->post_cap = dq_opt_u16(get_u16(&in[3].data[6]), S_VEL);

  r->snap = dq_opt_u16(get_u16(&in[4].data[0]), S_VEL);
  r->velocity = (bmpc_real)get_u16(&in[4].data[2]) * S_VEL;
  r->yaw_rate = (bmpc_real)(int16_t)get_u16(&in[4].data[4]) * S_YAWR;
  r->lf_est = (bmpc_real)get_u16(&in[4].data[6]) * S_LF;

  r->period = (bmpc_real)get_u16(&in[5].data[0]) * S_TIME;
  r->odom_age = (bmpc_real)get_u16(&in[5].data[2]) * S_TIME;
  r->transport = (bmpc_real)get_u16(&in[5].data[4]) * S_TIME;
  return 0;
}

/* ---- 응답 ---- */

static uint16_t sat_u16(uint32_t v) { return v > 0xFFFFu ? (uint16_t)0xFFFFu : (uint16_t)v; }

void bmpc_encode_response(const bmpc_response_t *r, bmpc_can_frame_t out[BMPC_RESP_FRAMES])
{
  int i;
  uint16_t crc;
  for (i = 0; i < BMPC_RESP_FRAMES; ++i) frame_init(&out[i], BMPC_ID_RESP_BASE + (uint32_t)i);

  put_u16(&out[0].data[0], r->seq);
  out[0].data[2] = r->status;
  out[0].data[3] = r->iters;
  put_f32(&out[0].data[4], r->steer_cmd);

  put_u16(&out[1].data[0], q_u16(r->accel, S_PEDAL, 0xFFFFu));
  put_u16(&out[1].data[2], q_u16(r->brake, S_PEDAL, 0xFFFFu));
  put_u16(&out[1].data[4], (uint16_t)q_i16(r->delta0, S_ANG));
  put_u16(&out[1].data[6], q_u16(r->vt_final, S_VEL, 0xFFFFu));

  put_u16(&out[2].data[0], q_u16(r->state_delay, S_TIME, 0xFFFFu));
  put_u16(&out[2].data[2], q_u16(r->compute, S_TIME, 0xFFFFu));
  put_u16(&out[2].data[4], q_u16(r->actuation, S_TIME, 0xFFFFu));
  put_u16(&out[2].data[6], q_u16(r->vt_raw, S_VEL, 0xFFFFu));

  put_u16(&out[3].data[0], sat_u16(r->solve_us));
  put_u16(&out[3].data[2], sat_u16(r->total_us));
  put_u16(&out[3].data[4], q_u16(r->lf_used, S_LF, 0xFFFFu));
  crc = group_crc(out, BMPC_RESP_FRAMES);
  put_u16(&out[3].data[6], crc);
}

int bmpc_decode_response(const bmpc_can_frame_t in[BMPC_RESP_FRAMES], bmpc_response_t *r)
{
  int i;
  memset(r, 0, sizeof(*r));
  for (i = 0; i < BMPC_RESP_FRAMES; ++i) {
    if (in[i].id != BMPC_ID_RESP_BASE + (uint32_t)i || in[i].dlc != 8) return -1;
  }
  r->seq = get_u16(&in[0].data[0]);
  if (group_crc(in, BMPC_RESP_FRAMES) != get_u16(&in[3].data[6])) return -2;

  r->status = in[0].data[2];
  r->iters = in[0].data[3];
  r->steer_cmd = get_f32(&in[0].data[4]);
  r->accel = (bmpc_real)get_u16(&in[1].data[0]) * S_PEDAL;
  r->brake = (bmpc_real)get_u16(&in[1].data[2]) * S_PEDAL;
  r->delta0 = (bmpc_real)(int16_t)get_u16(&in[1].data[4]) * S_ANG;
  r->vt_final = (bmpc_real)get_u16(&in[1].data[6]) * S_VEL;
  r->state_delay = (bmpc_real)get_u16(&in[2].data[0]) * S_TIME;
  r->compute = (bmpc_real)get_u16(&in[2].data[2]) * S_TIME;
  r->actuation = (bmpc_real)get_u16(&in[2].data[4]) * S_TIME;
  r->vt_raw = (bmpc_real)get_u16(&in[2].data[6]) * S_VEL;
  r->solve_us = get_u16(&in[3].data[0]);
  r->total_us = get_u16(&in[3].data[2]);
  r->lf_used = (bmpc_real)get_u16(&in[3].data[4]) * S_LF;
  return 0;
}

/* ---- 설정 ---- */

void bmpc_encode_cfg_set(uint16_t idx, float value, bmpc_can_frame_t *out)
{
  uint32_t bits;
  frame_init(out, BMPC_ID_CFG_SET);
  put_u16(&out->data[0], idx);
  memcpy(&bits, &value, sizeof(bits));
  put_u32(&out->data[4], bits);
}

void bmpc_decode_cfg_set(const bmpc_can_frame_t *in, uint16_t *idx, float *value)
{
  const uint32_t bits = get_u32(&in->data[4]);
  *idx = get_u16(&in->data[0]);
  memcpy(value, &bits, sizeof(*value));
}

void bmpc_encode_cfg_commit(uint32_t hash, uint16_t count, bmpc_can_frame_t *out)
{
  frame_init(out, BMPC_ID_CFG_COMMIT);
  put_u32(&out->data[0], hash);
  put_u16(&out->data[4], count);
}

void bmpc_decode_cfg_commit(const bmpc_can_frame_t *in, uint32_t *hash, uint16_t *count)
{
  *hash = get_u32(&in->data[0]);
  *count = get_u16(&in->data[4]);
}

void bmpc_encode_cfg_ack(uint32_t hash, uint16_t count, uint8_t status, bmpc_can_frame_t *out)
{
  frame_init(out, BMPC_ID_CFG_ACK);
  put_u32(&out->data[0], hash);
  put_u16(&out->data[4], count);
  out->data[6] = status;
}

void bmpc_decode_cfg_ack(const bmpc_can_frame_t *in, uint32_t *hash, uint16_t *count, uint8_t *status)
{
  *hash = get_u32(&in->data[0]);
  *count = get_u16(&in->data[4]);
  *status = in->data[6];
}
