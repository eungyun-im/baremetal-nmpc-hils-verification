#!/usr/bin/python3
# 교차 시험: 같은 세션에서 수정 전/후 펌웨어와 SQP, QP 설정을 블록마다 무작위 순서로 돈다. 중단 뒤 다시 실행하면 끝난 조건을 건너뛴다.
import csv, json, os, random, subprocess, sys, time
HOME = os.path.expanduser("~"); A = HOME + "/catkin_ws/bags/exp_20261105"; X = A + "/analysis/xover"
S = HOME + "/catkin_ws/src/bare-metal-mpc/bmpc_bridge/scripts"
CFG = [(1, 2), (1, 4), (2, 4), (2, 16)]; FW = ["before", "after"]; BLOCKS = 6
rng = random.Random(20261107)
order = []
for b in range(1, BLOCKS + 1):
    c = [(f, s, q) for f in FW for s, q in CFG]; rng.shuffle(c); order += [(b, *x) for x in c]
json.dump(order, open(X + "/order.json", "w"))
def valid(n):
    return sum(1 for r in csv.DictReader(open(A + "/summary.csv")) if r["condition"] == n and r["valid"] == "1")
cur = None; cf = X + "/current_fw"
if os.path.exists(cf): cur = open(cf).read().strip()
env = dict(os.environ, LD_LIBRARY_PATH=HOME + "/tools/pemicro/libusb/lib/x86_64-linux-gnu:" + os.environ.get("LD_LIBRARY_PATH", ""))
for b, f, s, q in order:
    name = "xover_%s_sqp%d_qp%d" % (f, s, q)
    if valid(name) >= b: continue
    if cur != f:
        print("FLASH", f, flush=True)
        for t in range(2):
            rc = subprocess.run(["bash", "flash.sh"], cwd=X + "/" + f + "/platform/s32k312", env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
            if rc == 0: break
        if rc != 0: print("RuntimeError flash failed", flush=True); sys.exit(3)
        cur = f; open(cf, "w").write(f); time.sleep(3)
    p = X + "/plans/cur.json"
    args = ["sqp_iters:=%d" % s, "qp_max_iters:=%d" % q] if (s, q) != (2, 16) else []
    json.dump([{"group": 10, "name": name, "mode": "hils", "laps": b, "dnf_is_result": True, "args": args}], open(p, "w"))
    print("XOVER block", b, name, flush=True)
    rc = subprocess.run(["/usr/bin/python3", "-u", S + "/run_laps.py", "--plan", p, "--out", A]).returncode
    if rc != 0: print("RuntimeError run_laps rc", rc, flush=True); sys.exit(rc)
print("XOVER 전체 완료", flush=True)
