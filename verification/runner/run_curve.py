#!/usr/bin/python3
# 곡선 조향 결함: 타당성 검사 있는/없는 펌웨어를 블록마다 무작위 순서로 돈다. 중단 뒤 다시 실행하면 끝난 조건을 건너뛴다.
import csv, json, os, random, subprocess, sys, time
HOME = os.path.expanduser("~"); A = HOME + "/catkin_ws/bags/exp_20261105"; X = A + "/analysis/xover"
S = HOME + "/catkin_ws/src/bare-metal-mpc/bmpc_bridge/scripts"
CELLS = [("m4", 0, 5400, 4), ("m4", 0, 5400, 6), ("m4", 0, 5400, 10), ("csv3", 3, 9800, 10)]; FW = ["after", "noplaus"]; BLOCKS = 3
rng = random.Random(20261108); order = []
for b in range(1, BLOCKS + 1):
    c = [(f, *x) for f in FW for x in CELLS]; rng.shuffle(c); order += [(b, *x) for x in c]
json.dump(order, open(X + "/curve_order.json", "w"))
def valid(n):
    return sum(1 for r in csv.DictReader(open(A + "/summary.csv")) if r["condition"] == n and r["valid"] == "1")
cf = X + "/current_fw"; cur = open(cf).read().strip() if os.path.exists(cf) else None
env = dict(os.environ, LD_LIBRARY_PATH=HOME + "/tools/pemicro/libusb/lib/x86_64-linux-gnu:" + os.environ.get("LD_LIBRARY_PATH", ""))
for b, f, loc, c, i, deg in order:
    name = "curve_%s_%s_step%d" % (f, loc, deg)
    if valid(name) >= b: continue
    if cur != f:
        for t in range(2):
            rc = subprocess.run(["bash", "flash.sh"], cwd=X + "/" + f + "/platform/s32k312", env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
            if rc == 0: break
        if rc != 0: print("RuntimeError flash failed", flush=True); sys.exit(3)
        cur = f; open(cf, "w").write(f); time.sleep(3)
    p = X + "/plans/cur.json"
    json.dump([{"group": 11, "name": name, "mode": "hils", "laps": b, "dnf_is_result": True, "args": [], "fault": {"csv": c, "index": i, "cmd": "ctrl_offset %d 1000" % deg}}], open(p, "w"))
    rc = subprocess.run(["/usr/bin/python3", "-u", S + "/run_laps.py", "--plan", p, "--out", A]).returncode
    if rc != 0: print("RuntimeError run_laps rc", rc, flush=True); sys.exit(rc)
print("CURVE 전체 완료", flush=True)
