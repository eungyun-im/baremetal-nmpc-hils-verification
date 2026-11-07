// CAN 송수신 계층. SILS 와 HILS 의 PC 쪽 차이는 이 계층뿐이다.
//   udp       : SILS 가상 버스 (bare_metal_mpc 의 bmpc_udp 를 그대로 쓴다, 가상 MCU 와 같은 코드)
//   socketcan : HILS 실제 CAN (can0 등. SocketCAN 으로 잡히는 USB-CAN 어댑터)
//   usbcan    : HILS 실제 CAN (CH340 "USB-CAN Analyzer", /dev/ttyUSB0. 시리얼 2 Mbaud, 0xAA ... 0x55 프레임)
#ifndef BMPC_BRIDGE_CAN_TRANSPORT_H
#define BMPC_BRIDGE_CAN_TRANSPORT_H

#include <memory>
#include <string>

#include "bmpc/bmpc_protocol.h"

namespace bmpc_bridge {

class CanTransport {
 public:
  virtual ~CanTransport() {}
  virtual bool send(const bmpc_can_frame_t &f) = 0;
  // 1 = 받음, 0 = 시간 초과, -1 = 오류
  virtual int recv(bmpc_can_frame_t *f, double timeout_s) = 0;
  virtual std::string describe() const = 0;
  // 결함 주입용: 어댑터 비트레이트를 바꾼다 (지원하지 않으면 false). 잘못된 비트레이트로 보내면 버스에 오류가 난다.
  virtual bool setBitrate(int bitrate) { (void)bitrate; return false; }
};

std::unique_ptr<CanTransport> makeUdpTransport(int local_port, const std::string &peer_host, int peer_port,
                                               std::string *err);
std::unique_ptr<CanTransport> makeSocketCanTransport(const std::string &ifname, std::string *err);
// min_gap_s: 송신 프레임 사이 최소 간격. 이 어댑터는 붙여 쓴 프레임이 가끔 섞인다(팀 capstone_hils 실측 4 ms).
std::unique_ptr<CanTransport> makeUsbCanTransport(const std::string &port, int bitrate, double min_gap_s,
                                                  std::string *err);

}  // namespace bmpc_bridge

#endif
