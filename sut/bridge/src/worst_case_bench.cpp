// MCU 계산 시간 벤치: 고정 악조건과 무작위 요청을 SQP/QP 설정별로 보내 계산 시간과 QP 반복 수를 기록한다.
// 주행과 달리 요청이 주기마다 크게 바뀌어 warm start 가 잘 맞지 않으므로, QP 상한까지 도는 경우를 일부러 많이 만든다.
// 설정은 브리지와 같은 yaml 을 private 이름공간에서 읽고 SQP/QP 값만 바꾼다. 주행 중인 스택과 함께 돌리지 않는다.
//
//   rosparam load config/bmpc_params.yaml /worst_case_bench
//   worst_case_bench --transport usbcan --gap 0.001 --configs 1:10,1:16,2:10,2:16,3:10,3:16 --count 2000 --out r.csv
#include <ros/ros.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "bmpc/bmpc_udp.h"
#include "bmpc_bridge/mcu_config.h"
#include "bmpc_bridge/mcu_link.h"

using namespace bmpc_bridge;

namespace {

struct Case {
  const char *name;
  double c0, c1, kappa, v, ref_v;
};

// 2026-11-04 악조건 6종 (경로 위 직진, R30 커브, 정지, 0.5 m 이탈, 2 m 이탈, 출발 위치) 과 같은 뜻의 요청.
// 출발 위치는 경로가 앞에 있어 차량 좌표계 3차식이 크게 벗어난 상태로 근사한다.
const Case kCases[] = {
    {"straight", 0.0, 0.0, 0.0, 15.0, 15.0},
    {"r30", 0.0, 0.0, 1.0 / 30.0, 10.0, 10.0},
    {"stop", 0.0, 0.0, 0.0, 0.0, 0.0},
    {"off0.5", 0.5, 0.0, 0.0, 10.0, 10.0},
    {"off2", 2.0, 0.0, 0.0, 10.0, 10.0},
    {"start22", 1.5, 0.3, 0.01, 0.0, 10.0},
};

void fillRequest(bmpc_request_t &r, double c0, double c1, double kappa, double v, double ref_v, bool reset)
{
  std::memset(&r, 0, sizeof(r));
  r.c[0] = c0;
  r.c[1] = c1;
  r.c[2] = 0.5 * kappa;
  r.c[3] = 0.0;
  r.x_max = 40;
  r.ref_v = ref_v;
  r.v_lb = 0;
  r.v_ub = ref_v;
  r.post_cap = 25.0;
  r.snap = -1;
  r.velocity = v;
  r.yaw_rate = -v * kappa;
  r.period = 0.05;
  r.odom_age = 0.008;
  if (reset) r.flags |= BMPC_FLAG_RESET;
}

std::vector<std::pair<int, int>> parseConfigs(const std::string &text)
{
  std::vector<std::pair<int, int>> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const size_t colon = item.find(':');
    if (colon != std::string::npos) out.emplace_back(std::stoi(item.substr(0, colon)), std::stoi(item.substr(colon + 1)));
  }
  return out;
}

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worst_case_bench", ros::init_options::NoRosout);
  std::string kind = "udp", port = "/dev/ttyUSB0", configs = "1:10,1:16,2:10,2:16,3:10,3:16", out_path = "worst_case.csv";
  double gap = 0.001, timeout = 0.2;
  int count = 2000, seed = 20261105, hold = 2;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--transport") && i + 1 < argc) kind = argv[++i];
    else if (!std::strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
    else if (!std::strcmp(argv[i], "--gap") && i + 1 < argc) gap = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--timeout") && i + 1 < argc) timeout = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--configs") && i + 1 < argc) configs = argv[++i];
    else if (!std::strcmp(argv[i], "--count") && i + 1 < argc) count = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) seed = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--hold") && i + 1 < argc) hold = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out_path = argv[++i];
  }

  ros::NodeHandle pnh("~");
  if (!pnh.hasParam("N") || !pnh.hasParam("Lf")) {
    std::printf("%s/N or Lf not set: rosparam load config/bmpc_params.yaml first\n", pnh.getNamespace().c_str());
    return 2;
  }
  bmpc_config_t base;
  loadMcuConfig(pnh, &base);

  std::string err;
  std::unique_ptr<CanTransport> t;
  if (kind == "usbcan") t = makeUsbCanTransport(port, 500000, gap, &err);
  else t = makeUdpTransport(BMPC_UDP_PORT_PC, "127.0.0.1", BMPC_UDP_PORT_MCU, &err);
  if (!t) {
    std::printf("transport: %s\n", err.c_str());
    return 2;
  }
  std::printf("transport: %s, N=%d dt=%.3f\n", t->describe().c_str(), (int)base.N, (double)base.dt);
  McuLink link(std::move(t));

  FILE *csv = std::fopen(out_path.c_str(), "w");
  if (!csv) {
    std::printf("cannot open %s\n", out_path.c_str());
    return 2;
  }
  std::fprintf(csv, "sqp,qp,case,k,c0,c1,kappa,v,ref_v,link,status,iters,solve_ms,total_ms,rtt_ms\n");

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> du(0.0, 1.0);
  auto uni = [&](double lo, double hi) { return lo + (hi - lo) * du(rng); };
  int failed = 0;

  for (const auto &sq : parseConfigs(configs)) {
    bmpc_config_t cfg = base;
    cfg.sqp_iters = sq.first;
    cfg.qp_max_iters = sq.second;
    std::string msg;
    if (!link.configure(cfg, 1.0, &msg)) {
      std::printf("SQP %d QP %d: configure failed: %s\n", sq.first, sq.second, msg.c_str());
      ++failed;
      continue;
    }
    double solve_max = 0.0, total_max = 0.0, rtt_max = 0.0;
    int iters_max = 0, capped = 0, bad_status = 0, link_bad = 0, sent = 0;
    const char *worst_case = "-";

    auto exchange = [&](const char *name, int k, double c0, double c1, double kappa, double v, double ref_v, bool reset) {
      bmpc_request_t req;
      bmpc_response_t resp;
      std::memset(&resp, 0, sizeof(resp));
      fillRequest(req, c0, c1, kappa, v, ref_v, reset);
      const McuLink::Result r = link.exchange(req, &resp, timeout);
      ++sent;
      const double solve = resp.solve_us * 1e-3, total = resp.total_us * 1e-3, rtt = link.stats().rtt_last * 1e3;
      std::fprintf(csv, "%d,%d,%s,%d,%.3f,%.4f,%.4f,%.2f,%.2f,%d,%d,%d,%.3f,%.3f,%.3f\n", sq.first, sq.second, name, k, c0, c1,
                   kappa, v, ref_v, (int)r, (int)resp.status, (int)resp.iters, solve, total, rtt);
      if (r != McuLink::OK) {
        ++link_bad;
        return;
      }
      if (resp.status == BMPC_ERR_MAXITER) ++capped;
      else if (resp.status != BMPC_OK) ++bad_status;
      if (solve > solve_max) {
        solve_max = solve;
        worst_case = name;
      }
      if (total > total_max) total_max = total;
      if (rtt > rtt_max) rtt_max = rtt;
      if (resp.iters > iters_max) iters_max = resp.iters;
    };

    // 고정 악조건: 다른 조건에서 넘어오는 첫 주기가 warm start 가 가장 안 맞는다
    for (const Case &c : kCases) {
      for (const Case &prev : kCases) {
        exchange(prev.name, 0, prev.c0, prev.c1, prev.kappa, prev.v, prev.ref_v, false);
        for (int k = 0; k < 3; ++k) exchange(c.name, k, c.c0, c.c1, c.kappa, c.v, c.ref_v, k == 0 && &prev == &c);
      }
    }
    // 무작위: hold 주기마다 경로와 속도를 크게 바꾼다
    for (int k = 0; k < count; ++k) {
      const double c0 = uni(-3.0, 3.0), c1 = uni(-0.6, 0.6), kappa = uni(-1.0 / 15.0, 1.0 / 15.0), v = uni(0.0, 25.0),
                   ref_v = uni(5.0, 25.0);
      for (int h = 0; h < hold; ++h) exchange("random", k, c0, c1, kappa, v, ref_v, false);
    }
    const double bound = sq.first * (6.2 + 0.51 * sq.second);
    std::printf("SQP %d QP %2d: %d requests, solve max %.2f ms (%s), total max %.2f ms, rtt max %.2f ms, iters max %d, "
                "QP cap %d (%.1f%%), other status %d, link errors %d, bound %.1f ms\n",
                sq.first, sq.second, sent, solve_max, worst_case, total_max, rtt_max, iters_max, capped,
                100.0 * capped / (sent ? sent : 1), bad_status, link_bad, bound);
    std::fflush(stdout);
    std::fflush(csv);
    if (link_bad || bad_status) ++failed;
  }
  std::fclose(csv);
  std::printf(failed ? "RESULT: FAIL (%d)\n" : "RESULT: PASS\n", failed);
  return failed ? 1 : 0;
}
