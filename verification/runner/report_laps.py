#!/usr/bin/env python3
"""run_laps.py 의 summary.csv 를 합격 기준(docs/acceptance_criteria.md)으로 판정해 report.md 를 쓴다.

  source ~/catkin_ws/devel/setup.bash
  python3 report_laps.py ~/catkin_ws/bags/exp_20261105
"""
import csv
import json
import os
import statistics
import sys


CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "acceptance.json")
GRID_M = 1.0
MAX_BAGS = 3


def num(row, key):
    value = row.get(key, "")
    return float(value) if value not in ("", None) else None


def stats(rows, key):
    values = [num(r, key) for r in rows]
    values = [v for v in values if v is not None]
    if not values:
        return None
    return dict(mean=statistics.mean(values), std=statistics.stdev(values) if len(values) > 1 else 0.0, max=max(values))


def ego_collisions(row):
    """AC-08: 자차와 관련된 충돌 메시지 수. 거리 계산 전의 행은 다른 차량끼리의 충돌까지 센 값으로 판정한다."""
    value = num(row, "collision_ego_msgs")
    return num(row, "collision_msgs") if value is None else value


def lap_failures(row, cfg, name):
    lap = cfg["lap"]
    failures = []
    rtt, solve = num(row, "rtt_max"), num(row, "solve_max")
    if rtt is None or rtt >= lap["rtt_max_lt"]:
        failures.append("AC-01 rtt %s" % rtt)
    if name in cfg["solve_applies_to"] and (solve is None or solve > lap["solve_max_le"]):
        failures.append("AC-02 solve %s" % solve)
    if num(row, "link_fail"):
        failures.append("AC-03 link")
    if num(row, "status_other"):
        failures.append("AC-04 status")
    if num(row, "safety_stop"):
        failures.append("AC-05 stop")
    normal = num(row, "safety_normal")
    if normal is not None and normal < 1.0:
        failures.append("AC-06 normal %.4f" % normal)
    gap = num(row, "sc_gap_max_ms")
    if gap is not None and gap > lap["safety_gap_le"]:
        failures.append("AC-07 gap %.2f" % gap)
    if ego_collisions(row):
        failures.append("AC-08 collision")
    if row["finished"] != "1":
        failures.append("AC-09 finish")
    # /gps 기록 전 랩은 판정하지 않는다. 표본이 없는 것을 통과로 치지 않는다
    error = num(row, "err_gps_max")
    if error is not None and error > lap["path_error_le"]:
        failures.append("AC-10 error %.2f" % error)
    return failures


def unjudged(rows):
    return sum(num(r, "err_gps_max") is None for r in rows)


def fault_failures(row, rule):
    state = num(row, "worst_state")
    failures = []
    if state is None:
        return ["주입 기록 없음"]
    if "state_ge" in rule and state < rule["state_ge"]:
        failures.append("상태 %d" % state)
    if "state_lt" in rule and state >= rule["state_lt"]:
        failures.append("상태 %d" % state)
    reaction = num(row, "reaction_ms")
    if "reaction_ms_le" in rule and (reaction is None or reaction > rule["reaction_ms_le"]):
        failures.append("반응 %s ms" % reaction)
    if ego_collisions(row):
        failures.append("충돌")
    return failures


def steer_profile(path):
    import numpy as np  # bag analysis only, so the verdict rules import without ROS
    import rosbag
    """같은 위치끼리 비교하도록 조향을 주행 거리 격자에 놓는다."""
    pose, steer = [], []
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=["/ekf_pose_with_covariance", "/ctrl_cmd_mpc"]):
            if topic == "/ctrl_cmd_mpc":
                steer.append((stamp.to_sec(), msg.steering))
            else:
                pose.append((stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y))
    pose, steer = np.array(pose), np.array(steer)
    step = np.hypot(np.diff(pose[:, 1]), np.diff(pose[:, 2]))
    # 첫 값은 EKF 초기화 전이라 튄다. 한 스텝 5 m 를 넘는 점프는 거리에 넣지 않는다
    step[step > 5.0] = 0.0
    distance = np.concatenate([[0.0], np.cumsum(step)])
    grid = np.arange(0.0, distance[-1], GRID_M)
    keep = np.concatenate([[True], np.diff(distance) > 1e-3])
    times = np.interp(grid, distance[keep], pose[keep, 0])
    return np.interp(times, steer[:, 0], steer[:, 1])


def steer_corr(out_dir, rows_a, rows_b):
    import numpy as np
    def mean_profile(rows):
        profiles = [steer_profile(os.path.join(out_dir, "bags", r["bag"])) for r in rows[:MAX_BAGS]]
        size = min(len(p) for p in profiles)
        return np.mean([p[:size] for p in profiles], axis=0)

    a, b = mean_profile(rows_a), mean_profile(rows_b)
    size = min(len(a), len(b))
    return float(np.corrcoef(a[:size], b[:size])[0, 1]), size


def m7_target(out_dir, row, cache):
    import numpy as np  # bag analysis only, so the verdict rules import without ROS
    import rosbag
    """M7(고주로) 구간의 최고 목표 속도. 랩마다 약 20 또는 25 m/s 로 갈려 주행 시간과 오차를 그대로 평균 내면 안 된다."""
    bag_name = row["bag"]
    if bag_name not in cache:
        section, target = [], []
        with rosbag.Bag(os.path.join(out_dir, "bags", bag_name)) as bag:
            for topic, msg, stamp in bag.read_messages(topics=["/mission/active_section", "/control_feedback/mpc_target_speed_mps"]):
                if topic == "/mission/active_section":
                    if msg.section_id == "M7":
                        section.append(stamp.to_sec())
                else:
                    target.append((stamp.to_sec(), msg.data))
        values = [v for t, v in target if section and section[0] <= t <= section[-1]]
        cache[bag_name] = max(values) if values else None
    return cache[bag_name]


def fmt(item, key="mean", digits=3):
    return "-" if item is None else ("%%.%df" % digits) % item[key]


def main():
    out_dir = os.path.expanduser(sys.argv[1])
    with open(CONFIG) as f:
        cfg = json.load(f)
    with open(os.path.join(out_dir, "summary.csv"), newline="") as f:
        rows = list(csv.DictReader(f))

    names = []
    for row in rows:
        if row["condition"] not in names:
            names.append(row["condition"])

    lines = ["# 주행 시험 판정", "", "기준: `docs/acceptance_criteria.md`, 데이터: `summary.csv` (%d랩)" % len(rows), ""]
    invalid = sum(r["valid"] != "1" for r in rows)
    lines += ["전체 무효 %d랩 / %d랩" % (invalid, len(rows)), "", "## 조건별 결과", "",
              "| 조건 | 유효 / 시도 | 완주 | 판정 | 주행 s | 최고속 m/s | 왕복 최대 ms | 계산 최대 ms | QP 상한 도달 | "
              "경로 오차 평균 / 최대 m (GPS) | AC-10 미판정 랩 | 무효 사유 |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    details = []
    for name in names:
        mine = [r for r in rows if r["condition"] == name]
        valid = [r for r in mine if r["valid"] == "1"]
        reasons = sorted({r["invalid"] for r in mine if r["invalid"]})
        if name.startswith("fault_"):
            kind = name[len("fault_"):].rsplit("_", 1)[0]
            failures = [(r["attempt"], fault_failures(r, cfg["faults"][kind])) for r in valid]
        elif name in cfg["report_only"]:
            failures = None
        else:
            failures = [(r["attempt"], lap_failures(r, cfg, name)) for r in valid]
        if failures is None:
            verdict = "비교용"
        elif not valid:
            verdict = "데이터 없음"
        elif any(f for _, f in failures):
            verdict = "**불합격**"
            details += ["- `%s` #%s: %s" % (name, a, ", ".join(f)) for a, f in failures if f]
        else:
            verdict = "합격"
        error_mean, error_max = stats(valid, "err_gps_mean"), stats(valid, "err_gps_max")
        lines.append("| %s | %d / %d | %d | %s | %s | %s | %s | %s | %s | %s / %s | %d | %s |" % (
            name, len(valid), len(mine), sum(r["finished"] == "1" for r in valid), verdict,
            fmt(stats(valid, "moving_s"), digits=1), fmt(stats(valid, "max_v"), "max", 2), fmt(stats(valid, "rtt_max"), "max", 1),
            fmt(stats(valid, "solve_max"), "max", 2), fmt(stats(valid, "maxiter_ratio"), digits=3),
            fmt(error_mean), fmt(error_max, "max"), unjudged(valid), ", ".join(reasons) or "-"))

    lines += ["", "## 불합격 내역", ""] + (details or ["없음"])

    lines += ["", "## M7 목표 속도별 비교", "",
              "M7 목표 속도는 계획 단계에서 정해지고 구성과 무관하게 갈린다. 주행 시간과 오차는 같은 속도끼리 비교한다.", "",
              "| 조건 | M7 목표 | 랩 | 주행 s (평균 ± 표준편차) | 경로 오차 평균 m | 경로 오차 최대 m |", "|---|---|---|---|---|---|"]
    cache_path = os.path.join(out_dir, "m7_cache.json")
    cache = json.load(open(cache_path)) if os.path.exists(cache_path) else {}
    for name in names:
        valid = [r for r in rows if r["condition"] == name and r["valid"] == "1" and r["bag"]]
        for label, low, high in (("약 20 m/s", 0.0, 22.0), ("약 25 m/s", 22.0, 99.0)):
            group = [r for r in valid if m7_target(out_dir, r, cache) is not None and low <= m7_target(out_dir, r, cache) < high]
            if group:
                moving, error, error_max = stats(group, "moving_s"), stats(group, "err_gps_mean"), stats(group, "err_gps_max")
                lines.append("| %s | %s | %d | %s ± %s | %s | %s |" % (name, label, len(group), fmt(moving, digits=1), fmt(moving, "std", 1), fmt(error), fmt(error_max, "max")))
    with open(cache_path, "w") as f:
        json.dump(cache, f)

    lines += ["", "## 결함별 반응", "",
              "반응 시간은 주입 뒤 안전 상태가 NORMAL 에서 처음 바뀔 때까지다. 해상도는 약 50 ms 이고, 구간(slow, fast)별 값이다. "
              "`bus_off` 는 PC 어댑터도 끊겨 시간이 늦게 잡히므로 참고만 한다.", "",
              "| 조건 | 랩 | 상태 변화 랩 | 반응 최소 / 평균 / 최대 ms | 최악 상태 (0 INIT, 1 NORMAL, 2 DEGRADED, 3 SAFE_STOP) | 주입 속도 m/s |",
              "|---|---|---|---|---|---|"]
    for name in names:
        if not name.startswith("fault_"):
            continue
        valid = [r for r in rows if r["condition"] == name and r["valid"] == "1"]
        reactions = [num(r, "reaction_ms") for r in valid if num(r, "reaction_ms") is not None]
        worst = sorted({r.get("worst_state", "") for r in valid if r.get("worst_state", "") != ""})
        speeds = [num(r, "inject_speed") for r in valid if num(r, "inject_speed") is not None]
        lines.append("| %s | %d | %d | %s | %s | %s |" % (
            name, len(valid), len(reactions),
            "%.0f / %.0f / %.0f" % (min(reactions), statistics.mean(reactions), max(reactions)) if reactions else "-",
            ", ".join(worst) or "-", "%.1f" % statistics.mean(speeds) if speeds else "-"))

    ctrl = [n for n in names if n.startswith("ctrl_")]
    if ctrl:
        lines += ["", "## 조향 명령 결함: 안전 제어기 켬과 끔", "",
                  "주입 뒤 10 초 동안의 GPS 경로 오차 최대값이다. 차로 여유는 약 0.80 m (차선을 직접 잰 값이 아니다).", "",
                  "| 결함 | 구간 | 안전 제어기 | 랩 | 주입 속도 m/s | 경로 오차 최대 평균 / 최악 m | 완주 | 충돌 랩 |", "|---|---|---|---|---|---|---|---|"]
        for name in ctrl:
            _, fault, zone, switch = name.split("_")
            valid = [r for r in rows if r["condition"] == name and r["valid"] == "1"]
            errors = [num(r, "fault_err_gps_max") for r in valid if num(r, "fault_err_gps_max") is not None]
            speeds = [num(r, "inject_speed") for r in valid if num(r, "inject_speed") is not None]
            lines.append("| %s | %s | %s | %d | %s | %s | %d | %d |" % (
                fault, zone, switch, len(valid), "%.1f" % statistics.mean(speeds) if speeds else "-",
                "%.2f / %.2f" % (statistics.mean(errors), max(errors)) if errors else "-",
                sum(r["finished"] == "1" for r in valid), sum(bool(ego_collisions(r)) for r in valid)))

    eq = cfg["equivalence"]
    hils = [r for r in rows if r["condition"] == eq["hils"] and r["valid"] == "1"]
    sils = [r for r in rows if r["condition"] == eq["sils"] and r["valid"] == "1"]
    lines += ["", "## SILS와 HILS 동등성", ""]
    if hils and sils:
        corr, size = steer_corr(out_dir, hils, sils)
        a, b = stats(hils, "err_gps_mean"), stats(sils, "err_gps_mean")
        if a is None or b is None:
            a, b = dict(mean=0.0, std=0.0), dict(mean=0.0, std=0.0)
            lines.append("- AC-21 은 GPS 기록이 있는 랩이 양쪽에 생기면 판정합니다.")
        diff, allowed = abs(a["mean"] - b["mean"]), a["std"] + b["std"]
        lines += ["- AC-20 조향 상관 %.3f (기준 %.2f, %d m 비교): %s" % (corr, eq["steer_corr_ge"], size, "합격" if corr >= eq["steer_corr_ge"] else "**불합격**"),
                  "- AC-21 경로 오차 평균 차이 %.3f m (허용 %.3f m, HILS %d랩, SILS %d랩): %s" % (diff, allowed, len(hils), len(sils), "합격" if diff <= allowed else "**불합격**")]
        if len(hils) < 2 or len(sils) < 2:
            lines.append("- 랩이 부족해 AC-21 허용 범위가 의미 없습니다.")
    else:
        lines.append("데이터 없음")

    text = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, "report.md"), "w") as f:
        f.write(text)
    print(text)


if __name__ == "__main__":
    main()
