#include "bmpc_bridge/mcu_link.h"

#include <algorithm>
#include <chrono>

namespace bmpc_bridge {

namespace {
double now_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

bool McuLink::isLinkFrame(const bmpc_can_frame_t &f) const
{
  return (f.id >= BMPC_ID_RESP_BASE && f.id < BMPC_ID_RESP_BASE + BMPC_RESP_FRAMES) || f.id == BMPC_ID_CFG_ACK;
}

void McuLink::other(const bmpc_can_frame_t &f)
{
  if (side_ && !isLinkFrame(f)) side_(f);
  else ++stats_.stale_frames;
}

void McuLink::drain()
{
  bmpc_can_frame_t f;
  while (t_->recv(&f, 0.0) == 1) other(f);
}

bool McuLink::waitSide(const std::function<bool()> &done, double timeout_s)
{
  const double deadline = now_s() + timeout_s;
  while (!done()) {
    const double remain = deadline - now_s();
    if (remain <= 0) return false;
    bmpc_can_frame_t f;
    if (t_->recv(&f, remain) == 1) other(f);
  }
  return true;
}

bool McuLink::configure(const bmpc_config_t &cfg, double timeout_s, std::string *msg)
{
  drain();
  bmpc_can_frame_t f;
  for (uint16_t i = 0; i < (uint16_t)BMPC_CONFIG_COUNT; ++i) {
    bmpc_encode_cfg_set(i, bmpc_config_get(&cfg, i), &f);
    if (!t_->send(f)) {
      if (msg) *msg = "send failed";
      return false;
    }
  }
  const uint32_t hash = bmpc_config_hash(&cfg);
  bmpc_encode_cfg_commit(hash, (uint16_t)BMPC_CONFIG_COUNT, &f);
  if (!t_->send(f)) {
    if (msg) *msg = "send failed";
    return false;
  }
  const double deadline = now_s() + timeout_s;
  for (;;) {
    const double remain = deadline - now_s();
    if (remain <= 0) {
      if (msg) *msg = "no ACK from MCU";
      return false;
    }
    if (t_->recv(&f, remain) != 1) continue;
    if (f.id != BMPC_ID_CFG_ACK) {
      other(f);
      continue;
    }
    uint32_t h;
    uint16_t c;
    uint8_t st;
    bmpc_decode_cfg_ack(&f, &h, &c, &st);
    if (st == 0 && h == hash) {
      if (msg) *msg = "MCU configured, hash " + std::to_string(h);
      return true;
    }
    if (msg) *msg = "MCU rejected config (status " + std::to_string(st) + ", got " + std::to_string(c) + " fields)";
    return false;
  }
}

McuLink::Result McuLink::exchange(bmpc_request_t &req, bmpc_response_t *resp, double timeout_s)
{
  ++stats_.exchanges;
  drain();
  req.seq = next_seq_++;
  if (next_seq_ == 0) next_seq_ = 1;

  bmpc_can_frame_t out[BMPC_REQ_FRAMES];
  bmpc_encode_request(&req, out);
  if (corrupt_next_) {
    out[3].data[2] ^= 0x04;
    corrupt_next_ = false;
  }
  const double t0 = now_s();
  for (int i = 0; i < BMPC_REQ_FRAMES; ++i) t_->send(out[i]);

  bmpc_can_frame_t in[BMPC_RESP_FRAMES];
  unsigned mask = 0;
  const double deadline = t0 + timeout_s;
  for (;;) {
    const double remain = deadline - now_s();
    if (remain <= 0) {
      ++stats_.timeouts;
      return TIMEOUT;
    }
    bmpc_can_frame_t f;
    if (t_->recv(&f, remain) != 1) continue;
    if (f.id < BMPC_ID_RESP_BASE || f.id >= BMPC_ID_RESP_BASE + BMPC_RESP_FRAMES) {
      other(f);
      continue;
    }
    const unsigned idx = f.id - BMPC_ID_RESP_BASE;
    if (idx == 0) {
      const uint16_t seq = (uint16_t)(f.data[0] | (f.data[1] << 8));
      if (seq != req.seq) {  // 이전 주기의 늦은 응답
        ++stats_.stale_frames;
        mask = 0;
        continue;
      }
      mask = 1;
      in[0] = f;
    } else {
      if (!(mask & 1u)) {
        ++stats_.stale_frames;
        continue;
      }
      in[idx] = f;
      mask |= 1u << idx;
    }
    if (mask == (1u << BMPC_RESP_FRAMES) - 1u) break;
  }
  const double rtt = now_s() - t0;
  if (bmpc_decode_response(in, resp) != 0) {
    ++stats_.rx_errors;
    return RX_ERROR;
  }
  stats_.rtt_last = rtt;
  stats_.rtt_max = std::max(stats_.rtt_max, rtt);
  stats_.rtt_sum += rtt;
  ++stats_.ok;
  // 왕복에서 MCU 처리 시간을 빼면 CAN 전송 시간(요청 + 응답)이다.
  // 늘어날 때는 바로, 줄어들 때는 천천히 따라간다(지연보상 ② 와 같은 규칙).
  const double sample = std::max(0.0, rtt - resp->total_us * 1e-6);
  transport_est_ = sample > transport_est_ ? sample : transport_est_ + 0.1 * (sample - transport_est_);
  return OK;
}

}  // namespace bmpc_bridge
