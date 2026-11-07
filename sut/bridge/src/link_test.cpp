// MORAI 없이 PC <-> 가상 MCU 왕복을 확인하는 도구 (ROS 불필요).
//   --phase a : 설정 확인 -> 연속 요청(최대 속도 1000 회 + 20 Hz 200 회) -> CRC 오류 주입
//   --phase b : MCU 가 꺼진 상태에서 타임아웃이 정해진 시간 안에 나는지
//   --phase c : MCU 재시작 직후 NOT_CONFIGURED 를 받고 재설정하면 다시 정상인지
// 요청 내용은 곡률·속도를 바꿔 가며 실제와 비슷하게 만든다.
//
// 실제 보드 (HILS):  bmpc_link_test --transport usbcan [--port /dev/ttyUSB0] [--gap 0.004] [--timeout 0.045] [--count 1000]
//                    [--sqp-iters N]  (설정의 SQP 반복 횟수만 바꿔서 보낸다)
//   CRC 오류 주입은 같은 포트로 보낸다 (가상 버스처럼 두 번째 포트를 열 수 없다).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "bmpc/bmpc_udp.h"
#include "bmpc_bridge/mcu_link.h"

using namespace bmpc_bridge;

static int g_fail = 0;
#define EXPECT(cond, ...)                \
  do {                                   \
    if (!(cond)) {                       \
      ++g_fail;                          \
      std::printf("  FAIL: ");           \
      std::printf(__VA_ARGS__);          \
      std::printf("\n");                 \
    }                                    \
  } while (0)

static void make_req(bmpc_request_t &r, int k)
{
  std::memset(&r, 0, sizeof(r));
  const double kappa = 0.03 * std::sin(k * 0.02);
  r.c[0] = 0.2 * std::sin(k * 0.05);
  r.c[1] = 0.02 * std::cos(k * 0.03);
  r.c[2] = 0.5 * kappa;
  r.c[3] = 1e-4 * std::sin(k * 0.01);
  r.x_max = 20;
  r.ref_v = 10 + 3 * std::sin(k * 0.004);
  r.v_lb = 0;
  r.v_ub = r.ref_v;
  r.post_cap = 16.0;
  r.snap = -1;
  r.velocity = r.ref_v - 0.5;
  r.yaw_rate = -r.velocity * kappa;
  r.period = 0.044;
  r.odom_age = 0.008;
  if (k == 0) r.flags |= BMPC_FLAG_RESET;
}

struct Acc {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  double mean() const
  {
    double s = 0;
    for (double x : v) s += x;
    return v.empty() ? 0 : s / v.size();
  }
  double pct(double q) const
  {
    if (v.empty()) return 0;
    std::vector<double> w(v);
    std::sort(w.begin(), w.end());
    return w[(size_t)std::floor(q * (w.size() - 1) + 0.5)];
  }
  double max() const { return v.empty() ? 0 : *std::max_element(v.begin(), v.end()); }
};

int main(int argc, char **argv)
{
  std::string phase = "a", kind = "udp", port = "/dev/ttyUSB0", ifname = "can0";
  double gap = 0.004, timeout = 0.030;
  int count = 1000, sqp_iters = -1;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--phase") && i + 1 < argc) phase = argv[++i];
    else if (!std::strcmp(argv[i], "--transport") && i + 1 < argc) kind = argv[++i];
    else if (!std::strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
    else if (!std::strcmp(argv[i], "--ifname") && i + 1 < argc) ifname = argv[++i];
    else if (!std::strcmp(argv[i], "--gap") && i + 1 < argc) gap = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--timeout") && i + 1 < argc) timeout = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--count") && i + 1 < argc) count = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--sqp-iters") && i + 1 < argc) sqp_iters = std::atoi(argv[++i]);
  }

  std::string err;
  std::unique_ptr<CanTransport> t;
  if (kind == "usbcan") t = makeUsbCanTransport(port, 500000, gap, &err);
  else if (kind == "socketcan") t = makeSocketCanTransport(ifname, &err);
  else t = makeUdpTransport(BMPC_UDP_PORT_PC, "127.0.0.1", BMPC_UDP_PORT_MCU, &err);
  if (!t) {
    std::printf("transport: %s\n", err.c_str());
    return 2;
  }
  std::printf("transport: %s, timeout %.0f ms\n", t->describe().c_str(), timeout * 1e3);
  CanTransport *raw_tx = t.get(); // CRC 오류 주입용 (udp 가 아니면 같은 포트)
  McuLink link(std::move(t));
  bmpc_config_t cfg;
  bmpc_config_default(&cfg);
  if (sqp_iters > 0) cfg.sqp_iters = sqp_iters;

  if (phase == "a") {
    std::string msg;
    const bool ok = link.configure(cfg, 0.5, &msg);
    std::printf("[a] configure: %s\n", msg.c_str());
    EXPECT(ok, "configure");

    Acc rtt, mcu_total, mcu_solve, transport;
    int status_bad = 0, link_bad = 0;
    for (int pass = 0; pass < 2; ++pass) {
      const int n = pass == 0 ? count : 200;
      for (int k = 0; k < n; ++k) {
        bmpc_request_t req;
        bmpc_response_t resp;
        make_req(req, k);
        const auto t0 = std::chrono::steady_clock::now();
        const McuLink::Result r = link.exchange(req, &resp, timeout);
        if (r != McuLink::OK) {
          ++link_bad;
          continue;
        }
        if (resp.status != BMPC_OK) ++status_bad;
        if (resp.seq != req.seq) ++link_bad;
        rtt.add(link.stats().rtt_last * 1e3);
        mcu_total.add(resp.total_us * 1e-3);
        mcu_solve.add(resp.solve_us * 1e-3);
        transport.add(link.transportEstimate() * 1e3);
        if (pass == 1) std::this_thread::sleep_until(t0 + std::chrono::milliseconds(50));
      }
    }
    std::printf("[a] %d exchanges: link errors %d, MCU status errors %d\n", count + 200, link_bad, status_bad);
    std::printf("    round trip ms   mean %.3f p99 %.3f max %.3f\n", rtt.mean(), rtt.pct(0.99), rtt.max());
    std::printf("    MCU total ms    mean %.3f p99 %.3f max %.3f\n", mcu_total.mean(), mcu_total.pct(0.99), mcu_total.max());
    std::printf("    MCU solve ms    mean %.3f p99 %.3f max %.3f\n", mcu_solve.mean(), mcu_solve.pct(0.99), mcu_solve.max());
    std::printf("    transport ms    mean %.3f max %.3f (estimate fed back to the MCU)\n", transport.mean(), transport.max());
    EXPECT(link_bad == 0 && status_bad == 0, "all exchanges ok");
    EXPECT(rtt.max() < timeout * 1e3, "round trip under timeout");

    // CRC 오류 주입: 요청 프레임 한 비트를 뒤집어 보내면 MCU 는 RX_ERROR 로 답해야 한다
    {
      std::string e2;
      std::unique_ptr<CanTransport> raw_udp;
      CanTransport *raw = raw_tx;
      if (kind == "udp") {
        raw_udp = makeUdpTransport(BMPC_UDP_PORT_PC + 10, "127.0.0.1", BMPC_UDP_PORT_MCU, &e2);
        raw = raw_udp.get();
      }
      bmpc_request_t req;
      make_req(req, 5);
      req.seq = 4242;
      bmpc_can_frame_t fr[BMPC_REQ_FRAMES];
      bmpc_encode_request(&req, fr);
      fr[3].data[2] ^= 0x04;
      for (auto &f : fr) raw->send(f);
      std::this_thread::sleep_for(std::chrono::milliseconds(kind == "udp" ? 20 : 150));
      // 응답은 링크 포트(47002)로 온다: 다음 교환이 늦은 응답을 버리고 정상인지 본다
      bmpc_request_t r2;
      bmpc_response_t resp2;
      make_req(r2, 6);
      const McuLink::Result r = link.exchange(r2, &resp2, timeout);
      std::printf("[a] corrupted request -> next exchange %s (stale frames dropped %lu)\n",
                  r == McuLink::OK ? "ok" : "FAILED", (unsigned long)link.stats().stale_frames);
      EXPECT(r == McuLink::OK && resp2.status == BMPC_OK, "recover after corrupted request");
      EXPECT(link.stats().stale_frames >= 4, "the RX_ERROR response must have arrived and been dropped");
    }
  } else if (phase == "b") {
    int timeouts = 0;
    Acc dur;
    for (int k = 0; k < 20; ++k) {
      bmpc_request_t req;
      bmpc_response_t resp;
      make_req(req, k);
      const auto t0 = std::chrono::steady_clock::now();
      const McuLink::Result r = link.exchange(req, &resp, timeout);
      dur.add(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      if (r == McuLink::TIMEOUT) ++timeouts;
    }
    std::printf("[b] MCU down: %d/20 timeouts, each took mean %.1f ms max %.1f ms (timeout %.0f ms)\n", timeouts,
                dur.mean(), dur.max(), timeout * 1e3);
    EXPECT(timeouts == 20, "all must time out");
    EXPECT(dur.max() < timeout * 1e3 + 5.0, "timeout must not overshoot");
  } else if (phase == "c") {
    bmpc_request_t req;
    bmpc_response_t resp;
    make_req(req, 0);
    McuLink::Result r = link.exchange(req, &resp, timeout);
    std::printf("[c] before config: link %d status %d\n", (int)r, (int)resp.status);
    EXPECT(r == McuLink::OK && resp.status == BMPC_ERR_NOT_CONFIGURED, "must report NOT_CONFIGURED");
    std::string msg;
    EXPECT(link.configure(cfg, 0.5, &msg), "reconfigure");
    make_req(req, 1);
    r = link.exchange(req, &resp, timeout);
    std::printf("[c] after reconfig: link %d status %d steer %.4f\n", (int)r, (int)resp.status, (double)resp.steer_cmd);
    EXPECT(r == McuLink::OK && resp.status == BMPC_OK, "ok after reconfig");
  }
  std::printf(g_fail ? "RESULT: FAIL (%d)\n" : "RESULT: PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
