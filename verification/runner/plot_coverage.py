#!/usr/bin/python3
import json, os, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np
H = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 150, "font.size": 9})
D = json.load(open(H + "/results/coverage_summary.json")); F = sorted(D["files"], key=lambda f: f["branch_percent"])
n = [os.path.basename(f["filename"]) for f in F]; y = np.arange(len(F))
fig, ax = plt.subplots(figsize=(6.4, 3.8)); ax.barh(y + .19, [f["line_percent"] for f in F], .36, color="#b8b6ad", label="Line"); ax.barh(y - .19, [f["branch_percent"] for f in F], .36, color="#2878d6", label="Branch")
ax.set_yticks(y); ax.set_yticklabels(n); ax.set(xlabel="Coverage (%)", xlim=(0, 108)); ax.legend(frameon=False, loc="lower right")
ax.set_title(f"Overall: line {D['line_percent']:.1f}%, branch {D['branch_percent']:.1f}% (measured on host)", loc="left", fontsize=11)
fig.tight_layout(); fig.savefig(H + "/figures/fig15_coverage.png")
