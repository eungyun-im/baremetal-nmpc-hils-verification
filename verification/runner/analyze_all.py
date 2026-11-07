#!/usr/bin/python3
# 전체 랩 요약과 그림. 입력은 summary.csv, load_flags.csv, xover/results 뿐이다. bag 을 읽지 않는다. 실행: python3 analyze_all.py
import csv, json, os, re, statistics as st
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, "analysis/final/figs"); os.makedirs(OUT, exist_ok=True)
GREY, BLUE, RED, INK, QUIET = "#b8b6ad", "#2878d6", "#c0443c", "#222222", "#666666"
plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": QUIET, "axes.labelcolor": QUIET, "xtick.color": QUIET, "ytick.color": QUIET, "font.size": 10, "axes.unicode_minus": False, "figure.dpi": 150})
R = list(csv.DictReader(open(os.path.join(ROOT, "summary.csv"))))
num = lambda x, k: float(x[k]) if x.get(k) not in ("", None) else None
def vals(rows, k): return [v for v in (num(x, k) for x in rows) if v is not None]
def group(name):
    return ("delay" if name.startswith("delay") else "fault" if name.startswith("fault") else "steering_fault" if name.startswith(("ctrl_", "plaus_", "curve_")) else "grid" if re.match(r"sqp\d", name) else "crossover" if name.startswith("xover") else "watchdog" if name.startswith("wdt") else "baseline")
cond = {}
for x in R: cond.setdefault(x["condition"], []).append(x)
with open(os.path.join(OUT, "all_conditions.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow("condition group attempts valid finished err_mean_valid err_mean_sd err_max_median over_0p8 solve_max rtt_max maxiter_mean ego_laps".split())
    for c, a in sorted(cond.items()):
        v = [x for x in a if x["valid"] == "1"]; e = vals(v, "err_gps_mean"); m = vals(v, "err_gps_max")
        w.writerow([c, group(c), len(a), len(v), sum(x["finished"] == "1" for x in a), round(st.mean(e), 4) if e else "", round(st.stdev(e), 4) if len(e) > 1 else "", round(st.median(m), 3) if m else "", sum(y > 0.8 for y in m), max(vals(v, "solve_max"), default=""), max(vals(v, "rtt_max"), default=""), round(st.mean(vals(v, "maxiter_ratio")), 4) if vals(v, "maxiter_ratio") else "", sum(int(x["collision_ego_msgs"] or 0) > 0 for x in a)])
def save(fig, name): fig.tight_layout(); fig.savefig(os.path.join(OUT, name)); plt.close(fig)
# 1 지연 보상 (core 값은 final_analysis 4장)
d = [0, 25, 50, 75, 100]; off = [.090, .130, .208, .418, .499]; offs = [.021, .034, .058, .051, .190]; on = [.057, .057, .055, .067, .059]; ons = [.005, .005, .002, .021, .004]
fig, ax = plt.subplots(figsize=(6.4, 3.6)); ax.errorbar(d, off, yerr=offs, color=GREY, marker="o", capsize=3, label="Compensation off"); ax.errorbar(d, on, yerr=ons, color=BLUE, marker="o", capsize=3, lw=2.2, label="Compensation on")
ax.set(xlabel="Injected delay (ms)", ylabel="Core path error (m)", ylim=(0, .72)); ax.annotate("0.418 m", (75, .418), xytext=(-48, 8), textcoords="offset points", color=INK); ax.annotate("0.067 m", (75, .067), xytext=(-20, 10), textcoords="offset points", color=INK); ax.legend(frameon=False, loc="upper left")
ax.set_title("Delay compensation: 0.418 m to 0.067 m at 75 ms", loc="left", fontsize=11, color=INK); save(fig, "fig1_delay.png")
# 2 계산 시간
cfg = [("sqp1_qp2", "SQP1\nQP2"), ("sqp1_qp4", "SQP1\nQP4"), ("sqp1_qp8", "SQP1\nQP8"), ("sqp2_qp2", "SQP2\nQP2"), ("sqp2_qp4", "SQP2\nQP4"), ("sqp2_qp8", "SQP2\nQP8"), ("hils_base_recheck", "SQP2\nQP16")]
drive = [max(vals([x for x in cond[c] if x["valid"] == "1"], "solve_max")) for c, _ in cfg]
fig, ax = plt.subplots(figsize=(6.4, 3.6)); x_ = np.arange(len(cfg)); ax.bar(x_, drive, .55, color=GREY); ax.bar(x_[0], drive[0], .55, color=BLUE); ax.bar(x_[-1], drive[-1], .55, color="#7f7d75")
for i, v in enumerate(drive): ax.text(i, v + .5, f"{v:.1f}", ha="center", color=INK, fontsize=9)
ax.axhline(50, color=RED, lw=1, ls="--"); ax.text(6.4, 50.8, "Control period 50 ms", ha="right", color=RED, fontsize=9); ax.axhline(29, color=QUIET, lw=1, ls=":"); ax.text(6.4, 29.8, "AC-02 29 ms", ha="right", color=QUIET, fontsize=9)
ax.set_xticks(x_); ax.set_xticklabels([n for _, n in cfg]); ax.set(ylabel="Max solve time while driving (ms)", ylim=(0, 56)); ax.set_title(f"Fewer iterations: max solve {drive[-1]:.1f} to {drive[0]:.1f} ms (GCC 13.3)", loc="left", fontsize=11, color=INK); save(fig, "fig2_budget.png")
# 3 교차 시험
rows = open(os.path.join(ROOT, "analysis/xover/results.txt")).read()
lab, bf, af = [], [], []
for c, n in (("sqp1_qp2", "SQP1 QP2"), ("sqp1_qp4", "SQP1 QP4"), ("sqp2_qp4", "SQP2 QP4"), ("sqp2_qp16", "SQP2 QP16")):
    b = re.search(rf"{c} before core \([^)]*\) m5w \(([\d.]+), ([\d.]+), (\d+)\)", rows); a = re.search(rf"{c} after core \([^)]*\) m5w \(([\d.]+), ([\d.]+), (\d+)\)", rows)
    lab.append(n); bf.append(tuple(map(float, b.groups()))); af.append(tuple(map(float, a.groups())))
fig, ax = plt.subplots(figsize=(6.4, 3.6)); x_ = np.arange(4)
ax.bar(x_ - .18, [v[0] for v in bf], .34, yerr=[v[1] for v in bf], color=GREY, capsize=3, label="Before fix"); ax.bar(x_ + .18, [v[0] for v in af], .34, yerr=[v[1] for v in af], color=BLUE, capsize=3, label="After fix")
ax.set_xticks(x_); ax.set_xticklabels(lab); ax.set(ylabel="M5 60-80 m error (m)", ylim=(0, .26)); ax.legend(frameon=False); ax.set_title("Randomized blocks: low-cap degradation disappears after the QP fix", loc="left", fontsize=11, color=INK); save(fig, "fig3_xover.png")
# 4 결함 반응
kinds = [("blackout", "blackout"), ("veh_loss", "veh_loss")]
fig, ax = plt.subplots(figsize=(6.4, 3.0)); y_ = 0
for k, n in kinds:
    a = [x for x in R if x["condition"].startswith("fault_" + k) and x["reaction_ms"] and x["inject_speed"]]
    ax.scatter(vals(a, "reaction_ms"), [y_] * len(a), s=30, color=BLUE, alpha=.7); ax.text(150, y_ + .22, f"{n} (n={len(a)}, max {max(vals(a, 'reaction_ms')):.1f} ms)", color=INK, fontsize=9); y_ += 1
ax.axvline(290, color=RED, lw=1, ls="--"); ax.text(287, 1.45, "Limit 290 ms", ha="right", color=RED, fontsize=9); ax.set_yticks([]); ax.set(xlabel="Time to safe state after fault (ms)", xlim=(140, 300), ylim=(-.4, 1.7)); ax.spines["left"].set_visible(False); ax.set_title("Communication fault reaction is within the limit (straight road)", loc="left", fontsize=11, color=INK); save(fig, "fig4_faults.png")
# 5 곡선 조향 결함
cells = [("curve_%s_csv3_step10", "15.8 m/s\n10° step"), ("curve_%s_m4_step10", "11 m/s\n10° step"), ("curve_%s_m4_step6", "11 m/s\n6° step"), ("curve_%s_m4_step4", "11 m/s\n4° step")]
fig, ax = plt.subplots(figsize=(6.4, 3.6))
for i, (c, n) in enumerate(cells):
    for f_, col, dx in (("noplaus", GREY, -.15), ("after", BLUE, .15)):
        v = [y for y in vals([x for x in cond.get(c % f_, []) if x.get("inject_ok") == "1" and x["fault_err_gps_max"]], "fault_err_gps_max")]
        ax.scatter([i + dx] * len(v), v, color=col, s=34, alpha=.85, label={"noplaus": "No check", "after": "With check"}[f_] if i == 0 else None)
ax.axhline(.8, color=QUIET, lw=1, ls=":"); ax.text(3.45, .83, "AC-10 0.8 m", ha="right", color=QUIET, fontsize=9); ax.set_xticks(range(4)); ax.set_xticklabels([n for _, n in cells]); ax.set(ylabel="Max error within 10 s of fault (m)", ylim=(0, 3.3)); ax.legend(frameon=False); ax.set_title("Curve steering faults: error from large steps falls by half or more", loc="left", fontsize=11, color=INK); save(fig, "fig5_curve.png")
# 6 AC-10 위치
fig, ax = plt.subplots(figsize=(6.4, 2.6)); loc = [("M3", 15), ("Tunnel exit", 10), ("M6", 3), ("M2", 1)]; ax.barh([n for n, _ in loc][::-1], [v for _, v in loc][::-1], color=GREY, height=.55)
for i, (n, v) in enumerate(loc[::-1]): ax.text(v + .2, i, str(v), va="center", color=INK)
ax.set(xlabel="Failed laps"); ax.set_title("Most of the 29 AC-10 failures are at M3 and the tunnel exit", loc="left", fontsize=11, color=INK); save(fig, "fig6_ac10.png")
# 7 무효 사유
inv = {}
for x in R:
    if x["valid"] != "1": inv[x["invalid"]] = inv.get(x["invalid"], 0) + 1
it = sorted(inv.items(), key=lambda t: t[1]); fig, ax = plt.subplots(figsize=(6.4, 2.8)); ax.barh([k for k, _ in it], [v for _, v in it], color=GREY, height=.55)
for i, (_, v) in enumerate(it): ax.text(v + .3, i, str(v), va="center", color=INK)
ax.set(xlabel="Laps"); ax.set_title(f"Reasons for {sum(inv.values())} invalid laps ({len(R)} rows)", loc="left", fontsize=11, color=INK); save(fig, "fig7_invalid.png")
# 8 기준 주행 시간 추세
from datetime import datetime
from scipy import stats
BASE = ("hils_base", "hils_base_recheck", "hook_base", "hook_base_newbridge", "qpfix_sqp2_qp16", "swt_base", "plaus_base", "final_base")
B = [x for x in R if x["condition"] in BASE and x["valid"] == "1" and x["solve_max"]]
B.sort(key=lambda x: x["stamp"])
fig, ax = plt.subplots(figsize=(6.4, 3.4)); seen = []
for i, x in enumerate(B):
    c = x["condition"]
    if not seen or seen[-1][0] != c: seen.append((c, i))
    ax.scatter(i, float(x["solve_max"]), s=22, color=BLUE, alpha=.75)
for c, i in seen[1:]: ax.axvline(i - .5, color="#dddddd", lw=.8, zorder=0)
ax.axhline(29, color=QUIET, lw=1, ls=":"); ax.text(len(B) - 1, 29.4, "AC-02 29 ms", ha="right", color=QUIET, fontsize=9)
ax.set(xlabel="Baseline laps in time order (lines mark build or session changes)", ylabel="Max solve time (ms)", ylim=(0, 32)); ax.set_title(f"Max solve time stays at or below {max(float(x['solve_max']) for x in B):.1f} ms over {len(B)} baseline laps", loc="left", fontsize=11, color=INK); save(fig, "fig8_solve_trend.png")
# 9 PC 부하와 왕복 시간
H = [x for x in R if x["mode"] == "hils" and x["valid"] == "1" and x["load_start"] and x["rtt_max"]]
lx, ly = [float(x["load_start"]) for x in H], [float(x["rtt_max"]) for x in H]
rho, rp = stats.spearmanr(lx, ly); lxs, lss = lx, [float(x["solve_max"]) for x in H]; rho2, rp2 = stats.spearmanr(lxs, lss)
fig, ax = plt.subplots(figsize=(6.4, 3.4)); ax.scatter(lx, ly, s=16, color=BLUE, alpha=.45); ax.set(xlabel="PC load at lap start (1-min average)", ylabel="Max round-trip time (ms)"); ax.set_title(f"PC load vs max round-trip time: Spearman rho={rho:.2f} (n={len(H)})", loc="left", fontsize=11, color=INK); save(fig, "fig9_load_rtt.png")
# 10 HILS 와 SILS
def pick(names): return [float(x["err_gps_mean"]) for x in R if x["condition"] in names and x["valid"] == "1" and x["err_gps_mean"]]
hb, sb = pick(("hils_base", "hils_base_recheck")), pick(("sils_base", "sils_base_recheck", "b2b_sils")); nn = pick(("sils_no_npc",))
U, up = stats.mannwhitneyu(hb, sb, alternative="two-sided")
fig, ax = plt.subplots(figsize=(6.4, 3.2))
for i, (n, v, col) in enumerate((("HILS baseline", hb, BLUE), ("SILS baseline", sb, GREY), ("SILS without NPCs", nn, "#7f7d75"))):
    ax.scatter(v, [i + d for d in np.linspace(-.12, .12, len(v))], s=24, color=col, alpha=.8); ax.text(0, i + .24, f"{n} (n={len(v)}, median {st.median(v):.3f} m)" if v else f"{n} (no data)", color=INK, fontsize=9)
ax.set_yticks([]); ax.spines["left"].set_visible(False); ax.set(xlabel="Lap mean path error, GPS (m)", ylim=(-.4, 2.6), xlim=(0, None)); ax.set_title("Lap mean error in HILS and SILS", loc="left", fontsize=11, color=INK); save(fig, "fig10_hils_sils.png")
with open(os.path.join(OUT, "stats.txt"), "w") as f:
    print(f"rows {len(R)} valid {sum(x['valid']=='1' for x in R)} hils {sum(x['mode']=='hils' for x in R)} sils {sum(x['mode']=='sils' for x in R)}", file=f)
    print(f"invalid reasons {inv}", file=f)
    print(f"base laps n={len(B)} solve_max max={max(float(x['solve_max']) for x in B):.2f} median={st.median(float(x['solve_max']) for x in B):.2f}", file=f)
    print(f"load vs rtt_max spearman {rho:.3f} p={rp:.4g} n={len(H)}; load vs solve_max {rho2:.3f} p={rp2:.4g}", file=f)
    print(f"HILS vs SILS err_gps_mean median {st.median(hb):.3f} (n={len(hb)}) vs {st.median(sb):.3f} (n={len(sb)}), MWU p={up:.4f}", file=f)
    print(f"no NPC err_gps_mean {[round(v,3) for v in nn]}", file=f)
print(open(os.path.join(OUT, "stats.txt")).read())
# 11 NPC 유무와 M3
nn_ = json.load(open(os.path.join(OUT, 'no_npc_m3.json')))
A_ = [o['m3'] for o in nn_ if o['g'] == 'nonpc']; B_ = [o['m3'] for o in nn_ if o['g'] == 'base']
fig, ax = plt.subplots(figsize=(6.4, 2.8))
ax.scatter(B_, [1] * len(B_), s=30, color=GREY, alpha=.85); ax.scatter(A_, [0] * len(A_), s=30, color=BLUE, alpha=.85)
ax.text(0, 1.2, f'With NPCs (n={len(B_)}, {sum(v >= .5 for v in B_)} laps at 0.5 m or more)', color=INK, fontsize=9); ax.text(0, .2, f'Without NPCs (n={len(A_)}, {sum(v >= .5 for v in A_)} laps at 0.5 m or more)', color=INK, fontsize=9)
ax.axvline(.8, color=QUIET, lw=1, ls=':'); ax.set_yticks([]); ax.spines['left'].set_visible(False); ax.set(xlabel='Max path error in M3 (m, SILS)', ylim=(-.4, 1.6), xlim=(0, 2.3)); ax.set_title('Large M3 errors also appear without NPCs', loc='left', fontsize=11, color=INK); save(fig, 'fig11_m3_no_npc.png')
# 12 M3 경로 점프 (sils_base #7, 시각별 측정값)
tt = [-4, -3, -2, -1.5, -1, -.5, 0, .5, 1, 1.5, 2, 3, 4, 6]; ego = [.54, 1.49, 1.65, 1.69, 1.81, 1.87, 1.97, 1.99, 1.97, 1.97, 1.99, 1.40, 1.18, .59]; pth = [.50, 1.48, 1.65, 1.70, 1.80, 1.87, 1.96, .01, .01, .01, 0, 0, 0, 0]
fig, ax = plt.subplots(figsize=(6.4, 3.4)); ax.plot(tt, pth, color=GREY, lw=2.2, marker='o', ms=4, label='Reference path'); ax.plot(tt, ego, color=BLUE, lw=2.2, marker='o', ms=4, label='Vehicle'); ax.axvline(0, color=RED, lw=1, ls='--'); ax.text(.1, 2.28, 'M3 entry', color=RED, fontsize=9); ax.set(xlabel='Time from M3 entry (s)', ylabel='Left offset from centerline (m)', ylim=(-.2, 2.5)); ax.legend(frameon=False, loc='lower left'); ax.set_title('Only the reference returns to the centerline at M3 entry (1 SILS lap)', loc='left', fontsize=11, color=INK); save(fig, 'fig12_m3_jump.png')
print("font", plt.rcParams["font.family"], "rows", len(R), "valid", sum(x["valid"] == "1" for x in R), "drive", [round(v, 1) for v in drive], inv)
# 14 초과 랩 원인, 15 EKF 차이 방향
fig, ax = plt.subplots(figsize=(6.4, 2.8)); it = [("M3: reference change", 21, GREY), ("Tunnel, M8: localization", 10, GREY), ("M6: tracking after lane change", 3, BLUE), ("M2: avoidance replanning", 1, GREY)]
ax.barh([n for n, _, _ in it][::-1], [v for _, v, _ in it][::-1], color=[c for _, _, c in it][::-1], height=.55)
for i, (_, v, _) in enumerate(it[::-1]): ax.text(v + .25, i, str(v), va="center", color=INK)
ax.set(xlabel="Laps above 0.8 m (35 of 70 judged laps)"); ax.set_title("Classification of the 35 laps above 0.8 m", loc="left", fontsize=11, color=INK); save(fig, "fig14_exceed_causes.png")
