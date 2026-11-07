#include "bmpc_bridge/can_transport.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#include "bmpc/bmpc_udp.h"

namespace bmpc_bridge {

// ---------------- UDP (SILS) ----------------
class UdpCanTransport : public CanTransport {
 public:
  UdpCanTransport() { u_.fd = -1; }
  ~UdpCanTransport() override { bmpc_udp_close(&u_); }
  bool open(int local_port, const std::string &host, int peer_port)
  {
    desc_ = "udp 127.0.0.1:" + std::to_string(local_port) + " -> " + host + ":" + std::to_string(peer_port);
    return bmpc_udp_open(&u_, (uint16_t)local_port, host.c_str(), (uint16_t)peer_port) == 0;
  }
  bool send(const bmpc_can_frame_t &f) override { return bmpc_udp_send(&u_, &f) == 0; }
  int recv(bmpc_can_frame_t *f, double timeout_s) override
  {
    return bmpc_udp_recv(&u_, f, timeout_s < 0 ? -1 : (int64_t)(timeout_s * 1e6));
  }
  std::string describe() const override { return desc_; }

 private:
  bmpc_udp_t u_;
  std::string desc_;
};

std::unique_ptr<CanTransport> makeUdpTransport(int local_port, const std::string &peer_host, int peer_port,
                                               std::string *err)
{
  std::unique_ptr<UdpCanTransport> t(new UdpCanTransport());
  if (!t->open(local_port, peer_host, peer_port)) {
    if (err) *err = std::string("cannot open UDP port ") + std::to_string(local_port) + ": " + std::strerror(errno);
    return nullptr;
  }
  return std::move(t);
}

// ---------------- SocketCAN (HILS) ----------------
class SocketCanTransport : public CanTransport {
 public:
  ~SocketCanTransport() override
  {
    if (fd_ >= 0) ::close(fd_);
  }
  bool open(const std::string &ifname, std::string *err)
  {
    desc_ = "socketcan " + ifname;
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      if (err) *err = std::string("socket(PF_CAN): ") + std::strerror(errno);
      return false;
    }
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      if (err) *err = "no CAN interface " + ifname;
      return false;
    }
    // MCU 가 보내는 응답(0x1E0..0x1E3), 설정 ACK(0x1E8), 안전 제어기 SAFE_OUT/SAFE_STAT(0x200, 0x201)만 받는다
    struct can_filter flt[4];
    flt[0].can_id = BMPC_ID_RESP_BASE;
    flt[0].can_mask = 0x7FC;
    flt[1].can_id = BMPC_ID_CFG_ACK;
    flt[1].can_mask = 0x7FF;
    flt[2].can_id = 0x200;
    flt[2].can_mask = 0x7FE;
    flt[3].can_id = 0x3F4;  // 안전 주기 진단
    flt[3].can_mask = 0x7FF;
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt));
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      if (err) *err = std::string("bind CAN: ") + std::strerror(errno);
      return false;
    }
    return true;
  }
  bool send(const bmpc_can_frame_t &f) override
  {
    struct can_frame cf;
    std::memset(&cf, 0, sizeof(cf));
    cf.can_id = f.id;
    cf.can_dlc = f.dlc;
    std::memcpy(cf.data, f.data, 8);
    return ::write(fd_, &cf, sizeof(cf)) == (ssize_t)sizeof(cf);
  }
  int recv(bmpc_can_frame_t *f, double timeout_s) override
  {
    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int r = ::poll(&pfd, 1, timeout_s < 0 ? -1 : (int)((timeout_s * 1e3) + 0.999));
    if (r <= 0) return r < 0 && errno != EINTR ? -1 : 0;
    struct can_frame cf;
    if (::read(fd_, &cf, sizeof(cf)) != (ssize_t)sizeof(cf)) return -1;
    f->id = cf.can_id & CAN_SFF_MASK;
    f->dlc = cf.can_dlc;
    std::memcpy(f->data, cf.data, 8);
    return 1;
  }
  std::string describe() const override { return desc_; }

 private:
  int fd_ = -1;
  std::string desc_;
};

std::unique_ptr<CanTransport> makeSocketCanTransport(const std::string &ifname, std::string *err)
{
  std::unique_ptr<SocketCanTransport> t(new SocketCanTransport());
  if (!t->open(ifname, err)) return nullptr;
  return std::move(t);
}

// ---------------- CH340 USB-CAN Analyzer (HILS) ----------------
// 프로토콜은 팀 capstone_hils 의 bench/usbcan.py 와 같다 (그쪽에서 실보드로 확인한 것).
//   설정 : AA 55 12 <속도> 01(표준 프레임) 필터 4 + 마스크 4 (0) 00(일반 모드) 01 00 00 00 00 <합(3 번째 바이트부터)>
//   프레임: AA | C0 | dlc (0x20 = 확장 ID) | ID (리틀엔디언 2 또는 4 바이트) | 데이터 | 55
// 설정 직후 바로 프레임을 쓰면 어댑터가 설정을 반영하지 못해 300 ms 기다린다.
class UsbCanTransport : public CanTransport {
 public:
  ~UsbCanTransport() override
  {
    if (fd_ >= 0) ::close(fd_);
  }
  bool open(const std::string &port, int bitrate, double min_gap_s, std::string *err)
  {
    static const int kBitrate[] = {1000000, 800000, 500000, 400000, 250000, 200000, 125000, 100000, 50000};
    int speed = 0;
    for (int i = 0; i < 9; ++i)
      if (kBitrate[i] == bitrate) speed = i + 1;
    if (!speed) {
      if (err) *err = "unsupported bitrate " + std::to_string(bitrate);
      return false;
    }
    min_gap_ = std::chrono::duration<double>(min_gap_s);
    desc_ = "usbcan " + port + " " + std::to_string(bitrate / 1000) + " kbps, gap " +
            std::to_string((int)std::lround(min_gap_s * 1e3)) + " ms";
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      if (err) *err = "open " + port + ": " + std::strerror(errno);
      return false;
    }
    // 다른 프로그램이 같은 포트를 열어 바이트를 나눠 가져가지 않게 배타로 잡는다
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      if (err) *err = port + " is busy (another program holds it)";
      return false;
    }
    ::ioctl(fd_, TIOCEXCL);
    struct termios tio;
    if (::tcgetattr(fd_, &tio) != 0) {
      if (err) *err = std::string("tcgetattr: ") + std::strerror(errno);
      return false;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    ::cfsetispeed(&tio, B2000000);
    ::cfsetospeed(&tio, B2000000);
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
      if (err) *err = std::string("tcsetattr: ") + std::strerror(errno);
      return false;
    }
    if (!configure(speed)) {
      if (err) *err = std::string("write config: ") + std::strerror(errno);
      return false;
    }
    return true;
  }
  bool setBitrate(int bitrate) override
  {
    static const int kBitrate[] = {1000000, 800000, 500000, 400000, 250000, 200000, 125000, 100000, 50000};
    for (int i = 0; i < 9; ++i)
      if (kBitrate[i] == bitrate) return configure(i + 1);
    return false;
  }
  // 어댑터 설정 프레임 (속도 코드). 설정 직후 바로 쓰면 반영되지 않아 300 ms 기다린다.
  bool configure(int speed)
  {
    uint8_t cfg[20] = {0xAA, 0x55, 0x12, (uint8_t)speed, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x01, 0, 0, 0, 0, 0};
    unsigned sum = 0;
    for (int i = 2; i < 19; ++i) sum += cfg[i];
    cfg[19] = (uint8_t)sum;
    if (!writeAll(cfg, sizeof(cfg))) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    ::tcflush(fd_, TCIFLUSH);
    buf_.clear();
    last_tx_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    return true;
  }
  bool send(const bmpc_can_frame_t &f) override
  {
    const auto ready = last_tx_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(min_gap_);
    if (std::chrono::steady_clock::now() < ready) std::this_thread::sleep_until(ready);
    const uint8_t dlc = f.dlc > 8 ? 8 : f.dlc;
    uint8_t pkt[13];
    pkt[0] = 0xAA;
    pkt[1] = (uint8_t)(0xC0 | dlc);
    pkt[2] = (uint8_t)(f.id & 0xFF);
    pkt[3] = (uint8_t)((f.id >> 8) & 0x07);
    std::memcpy(pkt + 4, f.data, dlc);
    pkt[4 + dlc] = 0x55;
    const bool ok = writeAll(pkt, (size_t)(5 + dlc));
    last_tx_ = std::chrono::steady_clock::now();
    return ok;
  }
  int recv(bmpc_can_frame_t *f, double timeout_s) override
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout_s < 0 ? 1e9 : timeout_s));
    for (;;) {
      int r;
      while ((r = parse(f)) == 1) {
        // MCU 응답(0x1E0..0x1E3), 설정 ACK(0x1E8), SAFE_OUT/SAFE_STAT(0x200, 0x201)만 넘긴다 (socketcan 필터와 같게).
        // 하트비트, 진단 프레임은 버린다.
        if ((f->id & ~0x3u) == BMPC_ID_RESP_BASE || f->id == BMPC_ID_CFG_ACK || (f->id & ~0x1u) == 0x200u || f->id == 0x3F4u)
          return 1;
      }
      const auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
      struct pollfd pfd;
      pfd.fd = fd_;
      pfd.events = POLLIN;
      const int ms = remain.count() > 0 ? (int)remain.count() : 0;
      const int p = ::poll(&pfd, 1, ms ? ms : (timeout_s > 0 ? 1 : 0));
      if (p < 0) {
        if (errno == EINTR) continue;
        return -1;
      }
      if (p > 0) {
        uint8_t tmp[512];
        const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
        if (n < 0 && errno != EAGAIN) return -1;
        if (n > 0) buf_.insert(buf_.end(), tmp, tmp + n);
        continue;
      }
      if (std::chrono::steady_clock::now() >= deadline) return 0;
    }
  }
  std::string describe() const override { return desc_; }

 private:
  bool writeAll(const uint8_t *p, size_t n)
  {
    while (n > 0) {
      const ssize_t w = ::write(fd_, p, n);
      if (w < 0) {
        if (errno != EAGAIN && errno != EINTR) return false;
        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLOUT;
        ::poll(&pfd, 1, 10);
        continue;
      }
      p += w;
      n -= (size_t)w;
    }
    return true;
  }
  // 버퍼에서 완성된 프레임 하나를 꺼낸다. 1 = 꺼냄, 0 = 더 받아야 함
  int parse(bmpc_can_frame_t *f)
  {
    for (;;) {
      size_t i = 0;
      while (i < buf_.size() && buf_[i] != 0xAA) ++i;
      buf_.erase(buf_.begin(), buf_.begin() + i);
      if (buf_.size() < 2) return 0;
      const uint8_t t = buf_[1];
      if ((t & 0xC0) != 0xC0) {
        buf_.erase(buf_.begin());
        continue;
      }
      const size_t dlc = t & 0x0F, idlen = (t & 0x20) ? 4 : 2, total = 2 + idlen + dlc + 1;
      if (dlc > 8) {
        buf_.erase(buf_.begin());
        continue;
      }
      if (buf_.size() < total) return 0;
      uint32_t id = 0;
      for (size_t k = 0; k < idlen; ++k) id |= (uint32_t)buf_[2 + k] << (8 * k);
      f->id = id;
      f->dlc = (uint8_t)dlc;
      std::memset(f->data, 0, 8);
      std::memcpy(f->data, &buf_[2 + idlen], dlc);
      buf_.erase(buf_.begin(), buf_.begin() + total);
      return 1;
    }
  }

  int fd_ = -1;
  std::string desc_;
  std::vector<uint8_t> buf_;
  std::chrono::duration<double> min_gap_{0.004};
  std::chrono::steady_clock::time_point last_tx_;
};

std::unique_ptr<CanTransport> makeUsbCanTransport(const std::string &port, int bitrate, double min_gap_s,
                                                  std::string *err)
{
  std::unique_ptr<UsbCanTransport> t(new UsbCanTransport());
  if (!t->open(port, bitrate, min_gap_s, err)) return nullptr;
  return std::move(t);
}

}  // namespace bmpc_bridge
