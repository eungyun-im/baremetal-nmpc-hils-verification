#!/usr/bin/python3
# 저장소와 프로필의 머리 그림. 곡선은 장식용 합성 데이터이며 측정값이 아니다.
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
BG, INK, QUIET, LINE, BLUE, CYAN = "#0d1420", "#f2f5f9", "#93a1b5", "#2b3a50", "#3b8cf0", "#48d1c4"
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "figure.dpi": 200})
def canvas(w, h):
    f = plt.figure(figsize=(w, h), facecolor=BG); a = f.add_axes([0, 0, 1, 1]); a.set_xlim(0, w); a.set_ylim(0, h); a.axis("off"); return f, a
def traces(a, w, h, y0=0):
    x = np.linspace(w * .52, w, 500); u = (x - x[0]) / (x[-1] - x[0])
    for off in (-.34, -.17, .17, .34): a.plot(x, y0 + h * (.5 + off) + .25 * np.sin(u * 5), color=LINE, lw=.7)
    r = y0 + h * .5 + .25 * np.sin(u * 5) + .32 * np.exp(-((u - .55) / .1) ** 2)
    a.plot(x, r, color=QUIET, lw=1, ls=(0, (4, 3)))
    t = np.convolve(r, np.ones(30) / 30, "same"); t[:30], t[-30:] = r[:30], r[-30:]
    a.plot(x, t, color=CYAN, lw=2)
def profile(path):
    w, h = 12, 3; f, a = canvas(w, h); traces(a, w, h)
    a.text(.7, 1.72, "Eungyun Im", color=INK, fontsize=38, fontweight="bold", va="center")
    a.text(.72, 1.03, "Automotive software quality  ·  verification  ·  field testing", color=QUIET, fontsize=13.5, va="center")
    f.savefig(path, facecolor=BG); plt.close(f)
def capstone(path):
    w, h = 12, 4.1; f, a = canvas(w, h); traces(a, w, 1.9, 2.15)
    a.text(.6, 3.3, "Bare-Metal NMPC", color=INK, fontsize=33, fontweight="bold", va="center")
    a.text(.62, 2.65, "Software quality and HILS verification on an NXP S32K312", color=QUIET, fontsize=13.5, va="center")
    S = [("24.41 ms", "max solve, 50 ms period"), ("468", "HILS and SILS laps recorded"), ("97.0%", "line coverage, host"), ("6", "defects found, 1 open")]
    for i, (v, l) in enumerate(S):
        x = .6 + i * 2.8
        a.add_patch(FancyBboxPatch((x, .5), 2.55, 1.35, boxstyle="round,pad=0,rounding_size=.08", fc="#152033", ec=LINE, lw=.8))
        a.text(x + .2, 1.33, v, color=INK, fontsize=21, fontweight="bold", va="center"); a.text(x + .2, .8, l, color=QUIET, fontsize=9.5, va="center")
    f.savefig(path, facecolor=BG); plt.close(f)
def arch(path):
    w, h = 12, 3.4; f, a = canvas(w, h)
    B = [(.45, "MORAI K-City", "Simulator: vehicle,\nsensors, traffic"), (3.35, "PC, ROS", "Perception, path, ROS bridge,\nfault injection"), (6.25, "CAN 500 kbps", "Control commands and\nvehicle state"), (9.15, "NXP S32K312", "NMPC, 50 ms\nSafety, 10 ms, watchdog")]
    for i, (x, t, d) in enumerate(B):
        hi = i == 3
        a.add_patch(FancyBboxPatch((x, .85), 2.4, 1.7, boxstyle="round,pad=0,rounding_size=.08", fc="#14304f" if hi else "#152033", ec=BLUE if hi else LINE, lw=1.3 if hi else .8))
        a.text(x + 1.2, 2.13, t, color=INK, fontsize=13, fontweight="bold", ha="center", va="center"); a.text(x + 1.2, 1.4, d, color=QUIET, fontsize=9.5, ha="center", va="center", linespacing=1.45)
        if i < 3: a.add_patch(FancyArrowPatch((x + 2.45, 1.7), (x + 2.85, 1.7), arrowstyle="<|-|>", mutation_scale=11, color=QUIET, lw=1.1))
    a.text(6, 3.0, "Hardware-in-the-loop setup", color=INK, fontsize=13, ha="center", va="center"); a.text(6, .42, "The MCU is a real board. The vehicle, road and traffic are simulated.", color=QUIET, fontsize=10, ha="center", va="center")
    f.savefig(path, facecolor=BG); plt.close(f)
if __name__ == "__main__":
    profile(sys.argv[1]); capstone(sys.argv[2]); arch(sys.argv[3])
