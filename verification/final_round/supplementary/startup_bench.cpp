// S4 (PLAN.md): SILS 녹화의 출발 구간 요청을 운영 설정(SQP 2 / QP 16)으로 보드에 보내고, 보드가 잰 풀이 시간을 기록한다.
// 펌웨어는 바꾸지 않는다. 녹화 속 설정 프레임(0x0F0/0x0F1)은 보내지 않고, 시작할 때 브리지 yaml + SQP/QP 로 한 번 설정한다.
// 출발 구간 = 처음부터 "마지막 비정상 요청 + 100개" (비정상: |c0| > 10 또는 |2 c2| > 1, 처음 2000개 안에서 찾음).
// 비정상이 없으면 처음 600개.
//
//   ROS_MASTER_URI=http://localhost:11400 rosparam load bmpc_params.yaml /startup_bench
//   startup_bench --port /dev/ttyUSB1 --out out.csv frames_*.bin ...
#include <ros/ros.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "bmpc/bmpc_protocol.h"
#include "bmpc_bridge/mcu_config.h"
#include "bmpc_bridge/mcu_link.h"

using namespace bmpc_bridge;

struct Rec { int64_t t; uint8_t dir; bmpc_can_frame_t f; };

static std::vector<bmpc_request_t> readRequests(const std::string &path)
{
  std::vector<bmpc_request_t> out;
  FILE *in = std::fopen(path.c_str(), "rb");
  if (!in) return out;
  unsigned char b[22];
  bmpc_can_frame_t grp[BMPC_REQ_FRAMES];
  int have = 0;
  while (std::fread(b, 1, 22, in) == 22) {
    Rec x;
    std::memcpy(&x.t, b, 8); x.dir = b[8]; std::memcpy(&x.f.id, b + 9, 4); x.f.dlc = b[13]; std::memcpy(x.f.data, b + 14, 8);
    if (x.dir != 0) continue;
    if (x.f.id < BMPC_ID_REQ_BASE || x.f.id >= BMPC_ID_REQ_BASE + BMPC_REQ_FRAMES) continue;
    const int k = (int)(x.f.id - BMPC_ID_REQ_BASE);
    if (k == 0) have = 0;
    if (k != have) { have = 0; continue; }
    grp[k] = x.f;
    ++have;
    if (have == BMPC_REQ_FRAMES) {
      bmpc_request_t r;
      if (bmpc_decode_request(grp, &r) == 0) out.push_back(r);
      have = 0;
    }
  }
  std::fclose(in);
  return out;
}

static bool abnormal(const bmpc_request_t &r) { return std::fabs((double)r.c[0]) > 10.0 || std::fabs(2.0 * (double)r.c[2]) > 1.0; }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "startup_bench", ros::init_options::NoRosout);
  std::string port = "/dev/ttyUSB0", out_path = "startup_bench.csv";
  double gap = 0.001, timeout = 0.2;
  int sqp = 2, qp = 16;
  long deadline = 0;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
    else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out_path = argv[++i];
    else if (!std::strcmp(argv[i], "--sqp") && i + 1 < argc) sqp = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--qp") && i + 1 < argc) qp = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--deadline") && i + 1 < argc) deadline = std::atol(argv[++i]);
    else files.push_back(argv[i]);
  }
  ros::NodeHandle pnh("~");
  if (!pnh.hasParam("N")) { std::printf("params not loaded\n"); return 2; }
  bmpc_config_t cfg;
  loadMcuConfig(pnh, &cfg);
  cfg.sqp_iters = sqp;
  cfg.qp_max_iters = qp;
  std::string err;
  std::unique_ptr<CanTransport> t = makeUsbCanTransport(port, 500000, gap, &err);
  if (!t) { std::printf("transport: %s\n", err.c_str()); return 2; }
  McuLink link(std::move(t));
  std::string msg;
  if (!link.configure(cfg, 1.0, &msg)) { std::printf("configure failed: %s\n", msg.c_str()); return 2; }
  std::printf("configured SQP %d QP %d N=%d\n", sqp, qp, (int)cfg.N);
  FILE *csv = std::fopen(out_path.c_str(), "w");
  std::fprintf(csv, "file,k,rel,abn,c0,kappa,v,link,status,iters,solve_ms,total_ms,rtt_ms\n");
  double solve_max = 0;
  long n = 0, link_bad = 0;
  for (const std::string &f : files) {
    if (deadline > 0 && std::time(nullptr) > deadline) { std::printf("deadline reached before %s\n", f.c_str()); break; }
    std::vector<bmpc_request_t> rq = readRequests(f);
    long last_abn = -1;
    for (long k = 0; k < (long)rq.size() && k < 2000; ++k) if (abnormal(rq[k])) last_abn = k;
    const long end = last_abn >= 0 ? std::min<long>((long)rq.size(), last_abn + 101) : std::min<long>((long)rq.size(), 600);
    const std::string base = f.substr(f.find_last_of('/') + 1);
    double fmax = 0;
    for (long k = 0; k < end; ++k) {
      bmpc_request_t r = rq[k];
      bmpc_response_t resp;
      std::memset(&resp, 0, sizeof(resp));
      const McuLink::Result res = link.exchange(r, &resp, timeout);
      const double s = resp.solve_us * 1e-3, tt = resp.total_us * 1e-3;
      if (res != McuLink::OK) ++link_bad;
      else { if (s > solve_max) solve_max = s; if (s > fmax) fmax = s; }
      ++n;
      std::fprintf(csv, "%s,%ld,%ld,%d,%.3f,%.4f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f\n", base.c_str(), k, last_abn >= 0 ? k - last_abn : -99999,
                   abnormal(rq[k]) ? 1 : 0, (double)rq[k].c[0], 2.0 * (double)rq[k].c[2], (double)rq[k].velocity, (int)res,
                   (int)resp.status, (int)resp.iters, s, tt, link.stats().rtt_last * 1e3);
    }
    std::fflush(csv);
    std::printf("%s reqs=%zu window=%ld last_abn=%ld max=%.2f ms\n", base.c_str(), rq.size(), end, last_abn, fmax);
    std::fflush(stdout);
  }
  std::fclose(csv);
  std::printf("TOTAL n=%ld link_bad=%ld solve_max=%.3f ms\n", n, link_bad, solve_max);
  return 0;
}
