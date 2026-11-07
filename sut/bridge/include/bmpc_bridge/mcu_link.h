// PC 쪽 MCU 링크: 설정 전송·확인, 요청 -> 응답 동기 교환, 왕복 시간과 CAN 전송 시간 추정.
// 같은 주기 안에서 요청을 보내고 응답을 기다린다(동기식). seq 로 응답을 짝지어
// 늦게 온 이전 응답을 버린다.
#ifndef BMPC_BRIDGE_MCU_LINK_H
#define BMPC_BRIDGE_MCU_LINK_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "bmpc/bmpc_config.h"
#include "bmpc/bmpc_protocol.h"
#include "bmpc_bridge/can_transport.h"

namespace bmpc_bridge {

struct LinkStats {
  uint64_t exchanges = 0, ok = 0, timeouts = 0, rx_errors = 0, stale_frames = 0;
  double rtt_last = 0, rtt_max = 0, rtt_sum = 0;
};

class McuLink {
 public:
  enum Result { OK = 0, TIMEOUT = 1, RX_ERROR = 2 };

  explicit McuLink(std::unique_ptr<CanTransport> t) : t_(std::move(t)) {}

  // 설정 72 개 + 확정 프레임을 보내고 ACK 를 기다린다. 해시가 맞으면 true.
  bool configure(const bmpc_config_t &cfg, double timeout_s, std::string *msg);

  // req.seq 는 여기서 채운다. OK 면 resp 에 응답(상태는 resp.status)이 들어 있다.
  Result exchange(bmpc_request_t &req, bmpc_response_t *resp, double timeout_s);

  // 응답·설정 ACK 가 아닌 프레임 (안전 제어기 SAFE_OUT/SAFE_STAT 등) 을 받으면 부른다. 없으면 stale 로 센다.
  using SideHandler = std::function<void(const bmpc_can_frame_t &)>;
  void setSideHandler(SideHandler h) { side_ = std::move(h); }
  // 결함 주입: 다음 요청의 프레임 한 비트를 뒤집어 보낸다 (MCU 는 CRC 오류로 RX_ERROR 응답)
  void corruptNextRequest() { corrupt_next_ = true; }
  // 결함 주입: 어댑터 비트레이트 바꾸기 (CH340 만)
  bool setBitrate(int bitrate) { return t_ && t_->setBitrate(bitrate); }
  // 프레임 하나를 그대로 보낸다 (차량 CAN 등)
  bool sendRaw(const bmpc_can_frame_t &f) { return t_ && t_->send(f); }
  // done() 이 참이 될 때까지 받은 프레임을 처리한다 (응답은 늦은 것으로 버린다). 참이 되면 true.
  bool waitSide(const std::function<bool()> &done, double timeout_s);

  // 직전 주기들로 추정한 CAN 전송 시간(왕복 - MCU 처리). 다음 요청의 transport 에 넣는다.
  double transportEstimate() const { return transport_est_; }
  const LinkStats &stats() const { return stats_; }
  std::string describe() const { return t_ ? t_->describe() : "none"; }

 private:
  void drain();
  bool isLinkFrame(const bmpc_can_frame_t &f) const;
  void other(const bmpc_can_frame_t &f);
  std::unique_ptr<CanTransport> t_;
  SideHandler side_;
  bool corrupt_next_ = false;
  uint16_t next_seq_ = 1;
  double transport_est_ = 0.0;
  LinkStats stats_;
};

}  // namespace bmpc_bridge

#endif
