#!/usr/bin/env python3
"""MORAI 랩 자동 반복: 리셋 -> 스택 실행 -> 판정 -> bag 기록 -> 요약 한 줄.

  source ~/catkin_ws/devel/setup.bash
  python3 run_laps.py --plan laps_plan.json --out ~/catkin_ws/bags/exp_20261105     # 실험 계획 (중단 후 이어서 가능)
  python3 run_laps.py --mode hils --laps 1 --out ~/catkin_ws/bags/test              # 한 조건만

- 리셋은 시뮬레이터 UI 를 xdotool 로 누른다(시나리오, 센서, 네트워크 다시 불러오기). 도는 동안 마우스를 쓰면 안 된다.
- 주행 중 시뮬레이터 창을 옮기면 시뮬레이터가 멈춰 그 랩이 무효가 될 수 있다(2026-11-05: 19.5 s 끊김).
- 클릭 좌표는 리셋할 때마다 지금 창 위치·크기로 계산한다. 창이 MIN_WINDOW 보다 작으면 시작하지 않는다.
- 리셋 뒤 GPS 가 출발점에서 START_TOL_M 안에 없으면 다시 시도하고, 그래도 아니면 멈춘다.
- 완주 = 마지막 경로(csv 3)의 끝에서 정지.

무효(valid=0) 사유는 invalid 칸에 남긴다. 행과 bag 은 지우지 않는다.
  sim_gap      시뮬레이터 토픽이 MAX_SIM_GAP_S 넘게 끊김 (도는 중 SIM_GAP_ABORT_S 넘으면 바로 중단)
  sim_stale    0 이 아닌 속도가 MAX_SIM_STALE_S 넘게 똑같이 옴. 시뮬레이터가 멈춘 채 값만 보내면 EKF 가 그 속도로 앞서 나간다
  inject_lost  조향 결함 명령 직후 링크가 끊겨 결함이 들어갔는지 알 수 없음
  no_inject    계획의 결함이 주입되지 않음 (min_v 속도에 닿지 않은 경우 등)
  ekf_diverged 터널(M8) 진입 전에 EKF 와 시뮬레이터 실제 위치 차이가 MAX_EKF_TRUTH_M 초과. 터널 안과 출구 뒤의 벌어짐은 주행 스택의 동작이라 결과로 남긴다
  long_stop    신호를 무시하는 주행에서 MAX_STOP_S 넘게 정지
  stalled      코스 끝 전에 STALL_S 넘게 멈춤, no_start  출발하지 않음, timeout  시간 초과
제어기 탓일 수 있는 조건은 계획에 dnf_is_result 를 켠다. 그러면 미완주와 정지도 유효한 결과로 남긴다.
충돌, 안전 정지, 링크 실패, 풀이 실패는 무효 사유가 아니라 결과다.
"""
import argparse
import bisect
import collections
import csv
import datetime
import glob
import json
import os
import random
import re
import signal
import socket
import subprocess
import time

import numpy as np
import pyproj
import rosbag

EAST_OFFSET, NORTH_OFFSET = 302595.0, 4124145.0  # 시나리오 mapInfo (UTM52N)
START_ENU = (-131.5, -427.96)  # 2026_molit_comp_sample_scene.json 의 자차 시작점
START_TOL_M = 3.0
FINISH_CSV, FINISH_INDEX = 3, 11000  # 코스 끝은 csv 3 의 인덱스 11067 (2026-11-05 측정)
STALL_S = 60.0
MAX_STOP_S = 15.0  # 유효 랩의 정지는 최대 4.3 s (2026-11-05)
FIRST_MOVE_S = 150.0
READY_TIMEOUT_S = 120.0
READY_SETTLE_S = 15.0  # 경로가 나온 뒤 인지 노드와 모델이 뜨기를 기다린다
SIM_GAP_ABORT_S = 3.0
MAX_EKF_TRUTH_M = 3.0
START_RESET_TRIES = 5
EGO_CONTACT_M = 5.0  # 차체 중심 사이 거리. 승용차 두 대가 닿을 수 있는 최대값 (4.64 m x 1.89 m 의 대각선). AC-08 판정용
EKF_LAG_S = 0.04  # EKF 는 실제 위치보다 약 40 ms 늦다 (2026-11-05 측정)
LAP_TIMEOUT_S = 600.0
FAULT_WINDOW_S = 10.0
SKIP_START_S = 20.0  # 출발 직후는 경로가 차보다 앞에서 시작해 오차가 크다
MAX_SIM_STALE_S = 1.0
MAX_SIM_GAP_S = 1.0  # 시뮬레이터 토픽이 이보다 오래 끊기면 무효 랩 (정상 랩 최대 0.53 s, 2026-11-05)
# 좌표를 잰 창 (2026-11-05). 클릭 좌표는 이 창 기준 값을 지금 창에 맞춰 옮긴다
REF_POS, REF_SIZE = (474, 325), (2931, 1838)
MIN_WINDOW = (2400, 1700)
WS = os.path.expanduser("~/catkin_ws")
LOAD_WAIT_S = 25.0  # 시나리오 로드 대기. Player.log 는 늦게 기록돼 완료 판단에 못 쓴다. 결과는 GPS 로 확인한다

COMMON_ARGS = ["sim_ip:=127.0.0.1", "use_traffic_light:=false", "record_bmpc:=true"]
HILS_ARGS = ["mcu:=real", "transport:=usbcan", "usbcan_gap_s:=0.001", "sqp_iters:=2", "qp_max_iters:=16",
             "response_timeout_s:=0.045", "safety:=true"]


def pause(sec):
    time.sleep(sec)


def xdo(*args):
    return subprocess.run(["xdotool"] + [str(a) for a in args], check=True, capture_output=True, text=True).stdout


def click(x, y, wait=1.0):
    # 터미널 입력 등으로 포커스를 뺏기면 클릭이 빗나가 시나리오가 안 불러와진다 (2026-11-05 추정). 클릭마다 창을 올린다
    xdo("windowactivate", xdo("search", "--name", "^Simulator$").split()[0])
    xdo("mousemove", x, y)
    pause(0.4)
    xdo("click", 1)
    pause(wait)


def window_anchors(win):
    """UI 는 크기가 고정이고 메뉴와 센서 패널은 왼쪽 위, 대화상자는 창 가운데에 붙는다."""
    geometry = xdo("getwindowgeometry", win)
    x, y = [int(v) for v in re.search(r"Position: (-?\d+),(-?\d+)", geometry).groups()]
    w, h = [int(v) for v in re.search(r"Geometry: (\d+)x(\d+)", geometry).groups()]
    if w < MIN_WINDOW[0] or h < MIN_WINDOW[1]:
        raise RuntimeError("시뮬레이터 창이 너무 작다: %s" % geometry)

    def top_left(px, py):
        return px - REF_POS[0] + x, py - REF_POS[1] + y

    def center(px, py):
        return (px - REF_POS[0] - REF_SIZE[0] // 2 + x + w // 2,
                py - REF_POS[1] - REF_SIZE[1] // 2 + y + h // 2)

    return top_left, center


def ui_reset():
    """시나리오를 불러오면 네트워크가 끊기므로 센서와 네트워크까지 다시 불러온다.

    네트워크 창을 닫으면 일시정지가 풀려 차가 제어 없이 굴러간다(2026-11-05: 30 s 에 60 m). 그래서 창은 열어 둔 채로
    스택을 올리고, 준비되면 ui_start 로 닫아 출발한다.
    """
    win = xdo("search", "--name", "^Simulator$").split()[0]
    top_left, center = window_anchors(win)
    xdo("windowactivate", "--sync", win)
    pause(1)
    click(*top_left(521, 257))  # Edit
    click(*top_left(565, 432))  # Scenario
    xdo("mousemove", *top_left(1034, 432))
    pause(0.4)
    click(*top_left(1034, 517), 2)  # Load Scenario
    click(*center(2016, 1596))  # Load (첫 줄 2026_molit_comp_sample_scene)
    pause(LOAD_WAIT_S)
    xdo("key", "F3")
    pause(3)
    click(*top_left(564, 1804), 2)  # 센서 Load
    click(*center(1686, 1160))  # sils_local.json
    click(*center(2016, 1560), 3)
    xdo("key", "F3")
    pause(1.5)
    xdo("key", "F4")
    pause(2.5)
    click(*center(1604, 1572), 2)  # 네트워크 Load
    click(*center(1070, 1174))  # sils_local.json
    click(*center(1400, 1572), 3)
    click(*center(1918, 1352))  # 로드 알림 OK
    click(*center(1204, 1572))  # 파일 창 Cancel


def ui_start():
    win = xdo("search", "--name", "^Simulator$").split()[0]
    _, center = window_anchors(win)
    xdo("windowactivate", "--sync", win)
    pause(1)
    click(*center(2506, 722))  # 네트워크 창 닫기 = 일시정지 해제


def gps_enu(timeout=5.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 1234))
    sock.settimeout(timeout)
    try:
        text = sock.recvfrom(65535)[0].decode("latin1")
    finally:
        sock.close()
    m = re.search(r"\$G[PN]GGA,[^,]*,([\d.]+),N,([\d.]+),E", text) or \
        re.search(r"\$G[PN]RMC,[^,]*,[AV],([\d.]+),N,([\d.]+),E", text)
    if not m:
        raise RuntimeError("GPS 문장을 읽지 못했다: %r" % text[:80])
    lat_raw, lon_raw = float(m.group(1)), float(m.group(2))
    lat = int(lat_raw / 100) + (lat_raw % 100) / 60
    lon = int(lon_raw / 100) + (lon_raw % 100) / 60
    east, north = pyproj.Proj(proj="utm", zone=52, ellps="WGS84")(lon, lat)
    return east - EAST_OFFSET, north - NORTH_OFFSET


def reset_to_start():
    for attempt in range(2):
        ui_reset()
        try:
            pos = gps_enu()
        except socket.timeout:  # Wi-Fi 가 끊긴 동안 시뮬레이터가 멈춰 GPS 가 안 올 수 있다 (2026-11-05). 다시 불러온다
            print("  리셋 %d: GPS 수신 없음" % (attempt + 1), flush=True)
            continue
        dist =float(np.hypot(pos[0] - START_ENU[0], pos[1] - START_ENU[1]))
        print("  리셋 %d: GPS (%.1f, %.1f), 출발점까지 %.1f m" % (attempt + 1, pos[0], pos[1], dist), flush=True)
        if dist <= START_TOL_M:
            return
    raise RuntimeError("차가 출발점으로 돌아오지 않았다")


WATCH_CODE = r"""
import json, os, rospy
from mission_supervisor.msg import SectionState
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Float64MultiArray, String
s = {'c': 0, 'i': 0, 'v': 0.0, 't': None, 'state': None}
def on_section(m):
    s['c'], s['i'] = m.csv, m.index
def on_speed(m):
    s['v'], s['t'] = m.velocity.x, rospy.get_time()
def on_safety(m):
    s['state'] = m.data[3]
rospy.init_node('lap_watch', anonymous=True)
rospy.Subscriber('/mission/active_section', SectionState, on_section)
rospy.Subscriber('/Competition_topic', EgoVehicleStatus, on_speed)
rospy.Subscriber('/bmpc/safety', Float64MultiArray, on_safety)
pub = rospy.Publisher('/bmpc/inject', String, queue_size=5, latch=False)
fault = json.loads(os.environ.get('LAP_FAULT', 'null'))
fired = None
resets, last_reset = 0, 0.0
rate = rospy.Rate(5)
while not rospy.is_shutdown():
    if fault:
        now = rospy.get_time()
        if fired is None and s['c'] == fault['csv'] and s['i'] >= fault['index'] and s['v'] >= fault.get('min_v', 0.0):
            pub.publish(String(fault['cmd']))
            fired = now
            print('INJECT', fault['cmd'], s['v'], flush=True)
        if fired is not None and fault.get('reset_after') and now - fired > fault['reset_after']:
            # 0x7F0 은 MCU 프레임보다 ID 가 커서 중재에서 지면 사라질 수 있다. SAFE_STOP 이 남으면 다시 보낸다
            if resets == 0 or (s['state'] == 3 and now - last_reset > 1.0 and resets < 5):
                pub.publish(String('safety_reset'))
                resets, last_reset = resets + 1, now
                print('INJECT safety_reset', s['v'], flush=True)
            elif s['state'] != 3 or resets >= 5:
                fault = None
    age = -1.0 if s['t'] is None else rospy.get_time() - s['t']
    print(s['c'], s['i'], s['v'], age, flush=True)
    rate.sleep()
"""


def wait_lap(env, fault):
    """별도 프로세스로 토픽을 읽어 lap 상태를 판정한다. 결함이 있으면 지정 위치에서 주입한다."""
    watch_env = dict(env, LAP_FAULT=json.dumps(fault))
    proc = subprocess.Popen(["/usr/bin/python3", "-c", WATCH_CODE], env=watch_env, stdout=subprocess.PIPE, text=True)
    t0 = last_move = first_move = time.time()
    moved, escaped = False, False
    csv_i, index = 0, 0
    result, inject_speed = "timeout", ""
    try:
        for line in proc.stdout:
            if line.startswith("INJECT"):
                print("   ", line.strip(), flush=True)
                if "safety_reset" not in line:
                    inject_speed = line.split()[-1]
                continue
            parts = line.split()
            if len(parts) != 4:
                continue
            csv_i, index, speed, age = int(parts[0]), int(parts[1]), float(parts[2]), float(parts[3])
            now = time.time()
            if age > SIM_GAP_ABORT_S:
                result = "sim_gap"
                break
            if speed > 0.3:
                if not moved:
                    first_move = now
                moved, last_move = True, now
            elif not moved:
                # 설정 창을 닫아도 일시정지가 안 풀리는 경우 한 번만 Esc 를 누른다
                if not escaped and now - t0 > 60 and age >= 0:
                    xdo("key", "Escape")
                    escaped = True
                    print("    60 s 동안 출발하지 않아 Esc 를 눌렀다", flush=True)
                if now - t0 > FIRST_MOVE_S:
                    result = "no_start"
                    break
                continue
            if csv_i >= FINISH_CSV and index >= FINISH_INDEX and now - last_move > 5:
                result = "finished"
                break
            if now - last_move > STALL_S:
                result = "stalled"
                break
            if now - first_move > LAP_TIMEOUT_S:
                break
    finally:
        proc.terminate()
        proc.wait()
    return result, csv_i, index, inject_speed


def wait_ready(env):
    """브리지가 MCU 와 통신하고 경로가 나오면 준비 완료다."""
    for topic in ("/local_path", "/bmpc/debug"):
        subprocess.run(["rostopic", "echo", "-n1", topic], env=env, check=True, timeout=READY_TIMEOUT_S,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def safety_state(env):
    """/bmpc/safety 의 안전 상태 (3 = SAFE_STOP). 안전 제어기가 꺼져 있거나 못 읽으면 None."""
    try:
        out = subprocess.run(["rostopic", "echo", "-n1", "/bmpc/safety/data"], env=env, capture_output=True, text=True,
                             timeout=6.0).stdout
        return int(float(out.split("---")[0].strip().strip("[]").split(",")[3]))
    except (subprocess.TimeoutExpired, IndexError, ValueError):
        return None


def release_safety_latch(env, args):
    """브리지는 BENCH_RESET(0x7F0)을 응답 확인 없이 한 번만 보낸다. 이 ID 는 MCU 프레임보다 커서 중재에서 지면 CH340 이
    약 100 ms 멈추고 프레임이 사라진 것으로 보인다 (2026-11-05 3회: 리셋 99~101 ms 뒤 MCU 타임아웃, 상태 3 유지).
    그러면 지난 랩의 SAFE_STOP 래치가 남아 차가 출발하지 않으므로 출발 전에 다시 보낸다.
    주행 중에는 보내지 않으므로 결함 시험의 SAFE_STOP 래치에는 영향이 없다."""
    sent = 0
    if "safety:=true" not in args:
        return sent
    while sent < START_RESET_TRIES and safety_state(env) == 3:
        subprocess.run(["rostopic", "pub", "-1", "/bmpc/inject", "std_msgs/String", "data: 'safety_reset'"], env=env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15.0)
        sent += 1
        print("  출발 전 SAFE_STOP 래치 해제 %d" % sent, flush=True)
    return sent


def stop_stack(proc):
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=40)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def analyze(bag_path):
    row = collections.OrderedDict()
    speed, debug, safety, ekf, paths, diag, truth, injects, sections, gps, lead = [], [], [], [], [], [], [], [], [], [], []
    to_utm = pyproj.Proj(proj="utm", zone=52, ellps="WGS84")
    collisions = 0
    others = []
    with rosbag.Bag(bag_path) as bag:
        topics = ["/Competition_topic", "/bmpc/debug", "/bmpc/safety", "/bmpc/mcu_diag", "/CollisionData",
                  "/ekf_pose_with_covariance", "/local_path", "/bmpc/inject_log", "/mission/active_section",
                  "/gps", "/smpc/acc_active_lead"]
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec()
            if topic == "/Competition_topic":
                speed.append((t, msg.velocity.x))
            elif topic == "/bmpc/debug":
                debug.append((t,) + tuple(msg.data))
            elif topic == "/bmpc/safety":
                safety.append((t,) + tuple(msg.data))
            elif topic == "/bmpc/mcu_diag":
                diag.append(tuple(msg.data))
            elif topic == "/CollisionData":
                # type -1 은 자차 항목이다. 다른 물체가 있을 때만 센다
                collisions += any(o.type != -1 for o in msg.collision_object)
                truth += [(t, o.position.x, o.position.y) for o in msg.collision_object if o.type == -1]
                others += [(t, o.position.x, o.position.y) for o in msg.collision_object if o.type != -1]
            elif topic == "/bmpc/inject_log":
                injects.append((t, msg.data))
            elif topic == "/mission/active_section":
                sections.append((t, msg.section_id))
            elif topic == "/gps":
                gps.append((t,) + to_utm(msg.longitude, msg.latitude))
            elif topic == "/smpc/acc_active_lead":
                lead.append((t, msg.data))
            elif topic == "/ekf_pose_with_covariance":
                ekf.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
            else:
                paths.append((t, np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])))
    speed = np.array(speed)
    moving = speed[speed[:, 1] > 0.3]
    t0, t1 = moving[0, 0], moving[-1, 0]
    dt = np.diff(speed[:, 0])
    # 시뮬레이터가 멈추면 스택은 도는 채로 EKF 만 앞으로 나간다. 그런 랩은 지표를 믿을 수 없어 표시한다
    in_lap = (speed[:-1, 0] >= t0) & (speed[:-1, 0] <= t1)
    row["sim_gap_max_s"] = float(dt[in_lap].max())
    stale_max, start = 0.0, None
    for i in range(1, len(speed)):
        if speed[i, 1] == speed[i - 1, 1] and speed[i, 1] > 0.3:
            start = speed[i - 1, 0] if start is None else start
            stale_max = max(stale_max, speed[i, 0] - start)
        else:
            start = None
    row["sim_stale_max_s"] = float(stale_max)
    stopped = speed[:, 1] < 0.3
    stop_max, start = 0.0, None
    for i in range(len(speed)):
        if t0 <= speed[i, 0] <= t1 and stopped[i]:
            start = speed[i, 0] if start is None else start
            stop_max = max(stop_max, speed[i, 0] - start)
        else:
            start = None
    row["stop_max_s"] = float(stop_max)
    row["moving_s"] = float(np.sum(dt[speed[:-1, 1] > 0.3]))
    row["max_v"] = float(speed[:, 1].max())
    row["collision_msgs"] = collisions

    dbg = np.array(debug)
    dbg = dbg[(dbg[:, 0] >= t0) & (dbg[:, 0] <= t1)]
    row["cycles"] = len(dbg)
    row["link_fail"] = int((dbg[:, 3] != 0).sum())
    row["maxiter_ratio"] = float((dbg[:, 2] == 1).mean())
    row["status_other"] = int((dbg[:, 2] > 1).sum())
    row["rtt_mean"], row["rtt_p99"], row["rtt_max"] = dbg[:, 4].mean(), np.percentile(dbg[:, 4], 99), dbg[:, 4].max()
    row["solve_mean"], row["solve_p99"], row["solve_max"] = dbg[:, 6].mean(), np.percentile(dbg[:, 6], 99), dbg[:, 6].max()

    if safety:
        saf = np.array(safety)
        saf = saf[(saf[:, 0] >= t0) & (saf[:, 0] <= t1)]
        row["safety_normal"] = float((saf[:, 4] == 1).mean())
        row["safety_stop"] = int((saf[:, 4] == 3).sum())
        row["safety_wait_max"] = float(saf[:, 3].max())
    if diag:
        row["sc_gap_max_ms"] = float(np.array(diag)[:, 1].max())

    # 경로 오차는 두 기준으로 낸다. 실제 위치(/CollisionData 의 자차 항목)는 간헐적으로만 온다
    path_t = [p[0] for p in paths]

    def path_error(t, x, y):
        i = bisect.bisect_right(path_t, t) - 1
        if i < 0 or len(paths[i][1]) < 2:
            return None
        pts = paths[i][1]
        seg = pts[1:] - pts[:-1]
        u = np.clip(((np.array([x, y]) - pts[:-1]) * seg).sum(1) / ((seg ** 2).sum(1) + 1e-9), 0, 1)
        distance = np.hypot(*(pts[:-1] + seg * u[:, None] - [x, y]).T)
        nearest = int(distance.argmin())
        # 경로 양 끝 밖이면 횡 오차가 아니다 (랩 끝, 시각이 튄 GPS 표본)
        if (nearest == 0 and u[0] == 0.0) or (nearest == len(seg) - 1 and u[-1] == 1.0):
            return None
        return distance[nearest]

    def summarize(name, values):
        values = np.array([v for v in values if v is not None])
        row[name + "_n"] = len(values)
        row[name + "_mean"] = values.mean() if len(values) else ""
        row[name + "_p95"] = np.percentile(values, 95) if len(values) else ""
        row[name + "_max"] = values.max() if len(values) else ""

    late = t0 + SKIP_START_S
    summarize("err_ekf", [path_error(t, x, y) for t, x, y in ekf if late <= t <= t1])
    ego = np.array([p for p in truth if late <= p[0] <= t1])
    pose = np.array(ekf)
    if len(ego):
        summarize("err_truth", [path_error(t, x + EAST_OFFSET, y + NORTH_OFFSET) for t, x, y in ego])
        ex = np.interp(ego[:, 0] + EKF_LAG_S, pose[:, 0], pose[:, 1]) - EAST_OFFSET
        ey = np.interp(ego[:, 0] + EKF_LAG_S, pose[:, 0], pose[:, 2]) - NORTH_OFFSET
        gaps = np.hypot(ex - ego[:, 1], ey - ego[:, 2])
        tunnel_start = next((t for t, name in sections if name == "M8"), float("inf"))
        before = ego[:, 0] < tunnel_start
        row["ekf_truth_mean"] = gaps[before].mean() if before.any() else ""
        row["ekf_truth_max"] = gaps[before].max() if before.any() else ""
        row["ekf_truth_tunnel_max"] = gaps[~before].max() if (~before).any() else ""
    else:
        summarize("err_truth", [])
        row["ekf_truth_mean"], row["ekf_truth_max"], row["ekf_truth_tunnel_max"] = "", "", ""

    # GPS 는 터널까지 코스 전체에 있어 표본이 많다. 다만 NMEA 분 소수 4 자리라 해상도가 약 0.15~0.18 m 이고, 약 40 ms 늦다
    fixes = np.array(gps) if gps else np.empty((0, 3))
    if len(fixes):
        fixes = fixes[np.hypot(fixes[:, 1] - EAST_OFFSET, fixes[:, 2] - NORTH_OFFSET) < 5000.0]
    section_times = [item[0] for item in sections]

    # /CollisionData 는 자차와 무관한 NPC 끼리의 충돌도 싣는다. 자차 항목은 접촉과 상관없이 오므로 쓰지 않고, 거리로 가른다
    row["collision_ego_msgs"], row["collision_ego_min_m"] = 0, ""
    if others:
        where = fixes[:, :3] - [0.0, EAST_OFFSET, NORTH_OFFSET] if len(fixes) >= 2 else \
            pose - [0.0, EAST_OFFSET, NORTH_OFFSET] if len(pose) >= 2 else None
        if where is None:
            row["collision_ego_msgs"] = ""  # 판정 불가. report_laps 는 collision_msgs 로 판정한다
        else:
            hit = np.array(others)
            dist = np.hypot(np.interp(hit[:, 0], where[:, 0], where[:, 1]) - hit[:, 1],
                            np.interp(hit[:, 0], where[:, 0], where[:, 2]) - hit[:, 2])
            row["collision_ego_msgs"] = len(set(hit[dist <= EGO_CONTACT_M, 0]))
            row["collision_ego_min_m"] = float(dist.min())

    def in_tunnel(t):
        return bool(sections) and sections[max(bisect.bisect_right(section_times, t) - 1, 0)][1] == "M8"

    # 터널(M8)은 GPS 표본이 적고 EKF 가 몇 m 벌어져, 추종 오차가 아니라 위치 추정 오차가 나온다. 값만 따로 남긴다
    summarize("err_gps", [path_error(t - EKF_LAG_S, x, y) for t, x, y in fixes if late <= t <= t1 and not in_tunnel(t)])
    tunnel_errors = [path_error(t - EKF_LAG_S, x, y) for t, x, y in fixes if late <= t <= t1 and in_tunnel(t)]
    tunnel_errors = [v for v in tunnel_errors if v is not None]
    row["err_gps_tunnel_max"] = max(tunnel_errors) if tunnel_errors else ""
    row["m7_lead_share"] = ""
    m7_times = [t for t, name in sections if name == "M7"]
    if lead and m7_times:
        states = [text for t, text in lead if m7_times[0] <= t <= m7_times[-1]]
        if states:
            row["m7_lead_share"] = sum(text != "none" for text in states) / float(len(states))

    # 첫 결함 주입 시각. 주입 뒤 FAULT_WINDOW_S 동안의 경로 오차가 차로 이탈 판단의 근거다
    fault_time = next((t for t, text in injects if text.startswith("start ") and "safety_reset" not in text), None)
    row["fault_err_gps_max"] = ""
    row["inject_ok"] = int(fault_time is not None)  # 브리지가 결함 명령을 알아듣고 시작했는지
    # 조향 결함 명령은 0x7F0 으로 가므로 중재에서 지면 사라지고 링크가 약 100 ms 멈춘다. 그런 랩은 결함 시험이 아니다
    row["inject_stall"] = ""
    if fault_time is not None and any(text.startswith("start ctrl_") for _, text in injects):
        near = dbg[(dbg[:, 0] >= fault_time) & (dbg[:, 0] <= fault_time + 0.3)]
        row["inject_stall"] = int((near[:, 3] != 0).any())
    if fault_time is not None:
        window = [path_error(t - EKF_LAG_S, x, y) for t, x, y in fixes if fault_time <= t <= fault_time + FAULT_WINDOW_S]
        window = [v for v in window if v is not None]
        row["fault_err_gps_max"] = max(window) if window else ""

    if safety:
        # 첫 결함 주입 뒤 안전 상태 변화까지 걸린 시간
        row["inject_log"] = " | ".join(text for _, text in injects)[:300]
        row["reaction_ms"], row["reaction_state"] = "", ""
        if fault_time is not None:
            after = saf[saf[:, 0] > fault_time]
            changed = after[after[:, 4] != 1]
            if len(changed):
                row["reaction_ms"] = (changed[0, 0] - fault_time) * 1000.0
                row["reaction_state"] = int(changed[0, 4])
            row["worst_state"] = int(after[:, 4].max()) if len(after) else ""
    return row


def invalid_reason(result, row, dnf_is_result):
    if result == "sim_gap" or row.get("sim_gap_max_s", 0) > MAX_SIM_GAP_S:
        return "sim_gap"
    if row.get("sim_stale_max_s", 0) > MAX_SIM_STALE_S:
        return "sim_stale"
    if result == "no_start":
        return "no_start"
    if row.get("inject_stall") == 1:
        return "inject_lost"
    if row.get("fault") and (not row.get("inject_speed") or row.get("inject_ok") == 0):
        return "no_inject"
    truth_max = row.get("ekf_truth_max", "")
    if truth_max != "" and truth_max > MAX_EKF_TRUTH_M:
        return "ekf_diverged"
    if dnf_is_result:
        return ""
    if result != "finished":
        return result
    if row.get("stop_max_s", 0) > MAX_STOP_S:
        return "long_stop"
    return ""


def append_row(csv_path, row):
    """조건마다 칸이 달라도 되도록 머리글을 합쳐 다시 쓴다."""
    rows = []
    if os.path.exists(csv_path):
        with open(csv_path, newline="") as f:
            rows = list(csv.DictReader(f))
    rows.append(row)
    fields = []
    for item in rows:
        fields += [k for k in item if k not in fields]
    with open(csv_path + ".tmp", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(csv_path + ".tmp", csv_path)


def reanalyze(out_dir, plan):
    """판정 기준을 고친 뒤 끝난 랩의 지표와 유효 판정을 bag 에서 다시 계산한다."""
    csv_path = os.path.join(out_dir, "summary.csv")
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    dnf = {c["name"]: c.get("dnf_is_result", False) for c in plan}
    os.replace(csv_path, csv_path + ".bak")
    for old in rows:
        row = collections.OrderedDict(old)
        if old["bag"]:
            row.update(analyze(os.path.join(out_dir, "bags", old["bag"])))
        for key, value in row.items():
            row[key] = "" if value is None else value
        reason = invalid_reason(old["result"], {k: v for k, v in row.items() if v != "" and not isinstance(v, str)},
                                dnf.get(old["condition"], False))
        if old["invalid"] == "manual_stop":
            reason = "manual_stop"  # 사람이 끊은 랩은 지표와 상관없이 무효로 둔다
        row["valid"], row["invalid"] = int(not reason), reason
        append_row(csv_path, row)
        print(old["condition"], old["attempt"], "->", row["valid"], reason, flush=True)


def merged_args(mode, extra):
    """같은 이름은 뒤의 값으로 덮어쓴다."""
    merged = collections.OrderedDict()
    for item in COMMON_ARGS + (HILS_ARGS if mode == "hils" else []) + extra:
        merged[item.split(":=")[0]] = item
    return list(merged.values())


def run_lap(cond, attempt, out_dir, env):
    name = cond["name"]
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    prefix = os.path.join(out_dir, "bags", "%s_%02d" % (name, attempt))
    os.makedirs(os.path.dirname(prefix), exist_ok=True)
    print("[%s] %s #%d" % (stamp, name, attempt), flush=True)
    reset_to_start()
    load0 = os.getloadavg()[0]
    log_path = "%s_%s.log" % (prefix, stamp)
    with open(log_path, "w") as log:
        proc = subprocess.Popen(["roslaunch", "bmpc_bridge", "sils.launch", "record_prefix:=" + prefix] +
                                merged_args(cond["mode"], cond.get("args", [])),
                                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        pause(5)
        wait_ready(env)
        pause(READY_SETTLE_S)
        start_resets = release_safety_latch(env, merged_args(cond["mode"], cond.get("args", [])))
        ui_start()
        result, csv_i, index, inject_speed = wait_lap(env, cond.get("fault"))
        load1 = os.getloadavg()[0]
    finally:
        stop_stack(proc)
    pause(3)
    row = collections.OrderedDict(condition=name, attempt=attempt, stamp=stamp, mode=cond["mode"],
                                  args=" ".join(cond.get("args", [])), fault=json.dumps(cond.get("fault")) if cond.get("fault") else "",
                                  result=result, end_csv=csv_i, end_index=index, inject_speed=inject_speed,
                                  load_start=load0, load_end=load1, start_resets=start_resets)
    bags = sorted(glob.glob(prefix + "_*.bag"))
    row["bag"] = os.path.basename(bags[-1]) if bags else ""
    try:
        row.update(analyze(bags[-1]))
    except (IndexError, ValueError, rosbag.ROSBagException) as error:
        row["analyze_error"] = str(error)[:200]
    reason = invalid_reason(result, row, cond.get("dnf_is_result", False))
    if "analyze_error" in row and not reason:
        reason = "analyze_error"
    row["valid"] = int(not reason)
    row["invalid"] = reason
    row["finished"] = int(result == "finished")
    append_row(os.path.join(out_dir, "summary.csv"), row)
    print("  %s valid=%d %s | 주행 %.1f s, 최고속 %.2f, 왕복 최대 %.1f ms, 풀이 최대 %.2f ms, 충돌 %s, 끊김 %.2f s" % (
        result, row["valid"], reason, float(row.get("moving_s", 0)), float(row.get("max_v", 0)), float(row.get("rtt_max", 0)),
        float(row.get("solve_max", 0)), row.get("collision_msgs", "-"), float(row.get("sim_gap_max_s", 0))), flush=True)
    return row["valid"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", help="조건 목록 JSON: [{name, mode, laps, args, fault, dnf_is_result, group}]")
    ap.add_argument("--mode", choices=["sils", "hils"], default="hils")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--name", default="")
    ap.add_argument("--args", nargs="*", default=[], help="roslaunch 인자 추가 (예: cmd_delay_s:=0.05)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=20261105)
    ap.add_argument("--reanalyze", action="store_true", help="끝난 랩을 다시 판정하고 끝낸다")
    opt = ap.parse_args()

    out_dir = os.path.expanduser(opt.out)
    os.makedirs(out_dir, exist_ok=True)
    if opt.plan:
        with open(opt.plan) as f:
            plan = json.load(f)
    else:
        plan = [dict(name=opt.name or opt.mode, mode=opt.mode, laps=opt.laps, args=opt.args)]
    if opt.reanalyze:
        reanalyze(out_dir, plan)
        return

    env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":1"), YOLO_AUTOINSTALL="false")
    env["PATH"] = ":".join(p for p in env["PATH"].split(":") if "anaconda" not in p)
    env.pop("PYTHONPATH", None)
    env.pop("CONDA_PREFIX", None)
    setup = subprocess.run(["bash", "-c", "source %s/devel/setup.bash && env -0" % WS], env=env, check=True,
                           capture_output=True, text=True).stdout
    env = dict(item.split("=", 1) for item in setup.split("\0") if "=" in item)
    os.environ["DISPLAY"] = env["DISPLAY"]

    # 이어서 돌릴 수 있도록 지금까지의 시도 수를 CSV 에서 읽는다. 조건 순서는 묶음 안에서 라운드마다 섞는다
    csv_path = os.path.join(out_dir, "summary.csv")
    rng = random.Random(opt.seed)
    groups = []
    for cond in plan:
        if cond.get("group", 0) not in groups:
            groups.append(cond.get("group", 0))
    for group in groups:
        members = [c for c in plan if c.get("group", 0) == group]
        while True:
            rows = list(csv.DictReader(open(csv_path, newline=""))) if os.path.exists(csv_path) else []
            todo = []
            for cond in members:
                mine = [r for r in rows if r["condition"] == cond["name"]]
                good = sum(int(r["valid"]) for r in mine)
                if good < cond["laps"] and len(mine) < 2 * cond["laps"] + 2:
                    todo.append((cond, len(mine) + 1))
            if not todo:
                break
            rng.shuffle(todo)
            for cond, attempt in todo:
                run_lap(cond, attempt, out_dir, env)
    print("계획 완료", flush=True)
    subprocess.run(["/usr/bin/python3", os.path.join(os.path.dirname(os.path.abspath(__file__)), "report_laps.py"), out_dir], env=env)


if __name__ == "__main__":
    main()
