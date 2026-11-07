"""Run: python3 verification/traceability/test_build_rtm.py"""
import build_rtm as b

CFG = dict(lap=dict(rtt_max_lt=45.0, solve_max_le=29.0, safety_gap_le=10.5, path_error_le=0.8), solve_applies_to=["base"], report_only=["skip"],
           faults=dict(blackout=dict(state_ge=3, reaction_ms_le=290)), equivalence=dict(hils="h", sils="s"))


def row(**k):
    r = dict(condition="base", attempt="1", valid="1", fault="", mode="hils", finished="1", rtt_max="30", solve_max="20", err_gps_max="0.5", link_fail="0", status_other="0",
             safety_stop="0", safety_normal="1", sc_gap_max_ms="10", collision_msgs="0", collision_ego_msgs="0", worst_state="3", reaction_ms="200", err_gps_mean="0.1")
    r.update(k); return r


def lap(i, rows): return b.judge_lap(dict(id=i), b.driving_rows(rows, CFG), CFG)


assert lap("AC-10", [row()]) == (1, [])
assert lap("AC-10", [row(err_gps_max="0.81")])[1] == ["base #1"]
assert lap("AC-10", [row(err_gps_max="")]) == (0, []), "a missing sample is not a pass"
assert lap("AC-10", [row(valid="0", err_gps_max="9")]) == (0, [])
assert lap("AC-10", [row(condition="skip", err_gps_max="9")]) == (0, [])
assert lap("AC-02", [row(condition="other", solve_max="99")]) == (0, [])
assert lap("AC-05", [row(mode="sils", safety_stop="1")]) == (0, [])
assert lap("AC-01", [row(rtt_max="45")])[1] == ["base #1"]
F = dict(fault="blackout")
B = '{"csv": 0, "index": 1400, "cmd": "blackout 0.35"}'
assert b.judge_fault(F, [row(fault=B)], CFG) == (1, [])
assert b.judge_fault(F, [row(fault=B, reaction_ms="291")], CFG)[1] == ["base #1"]
assert b.judge_fault(F, [row(fault=B, worst_state="1")], CFG)[1] == ["base #1"]
assert b.judge_fault(F, [row(fault=B, condition="skip")], CFG) == (0, [])
assert b.judge_equivalence([row(condition="h")], CFG)[2] is None
print("ok")
