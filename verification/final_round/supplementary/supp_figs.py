#!/usr/bin/env python3
"""보충 시험 그림 (무채색, 포스터 그림과 같은 모양).  -> ../../poster/img/g_delay_supp.png, g_startup_bench.png"""
import csv, os
import numpy as np, pandas as pd
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, '..', '..', 'poster', 'img')
f = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'; fm.fontManager.addfont(f)
plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name()
K, D, M, L, G = '#111111', '#4a4a4a', '#7d7d7d', '#b5b5b5', '#e3e3e3'
plt.rcParams.update({'axes.unicode_minus': False, 'figure.facecolor': 'white', 'axes.edgecolor': M, 'xtick.color': D,
                     'ytick.color': D, 'axes.grid': True, 'grid.color': G, 'axes.spines.top': False, 'axes.spines.right': False, 'font.size': 10})


def save(fig, name):
    fig.tight_layout(); fig.savefig(os.path.join(OUT, name), dpi=170); plt.close(fig)


# 지연 보상: 기존(11-05~07) 끔·켬 + 보충(11-12) 켬
old = [r for r in csv.DictReader(open(os.path.expanduser('~/catkin_ws/bags/exp_20261105/summary.csv'))) if r['valid'] == '1']
new = [r for r in csv.DictReader(open(os.path.expanduser('~/catkin_ws/bags/exp_20261112_supp_dose/summary.csv'))) if r['valid'] == '1']
fig, ax = plt.subplots(figsize=(6.4, 3.8))
series = [(old, 'sils_dose_d%d_off', M, 's', 'white', '-', '보상 끔 (기존)'),
          (old, 'sils_dose_d%d_on', K, 'o', K, '-', '보상 켬 (기존)'),
          (new, 'supp_dose_d%d_on', K, 'D', 'white', '--', '보상 켬 (보충 11-12)')]
for rows, pat, col, mk, face, ls, name in series:
    xs, ms, sd = [], [], []
    for d in [0, 50, 75, 100, 125, 150]:
        v = [float(r['err_gps_mean']) for r in rows if r['condition'] == pat % d]
        if v: xs.append(d + (2 if 'supp' in pat else 0)); ms.append(np.mean(v)); sd.append(np.std(v))
    ax.errorbar(xs, ms, yerr=sd, color=col, marker=mk, mfc=face, mec=col, ms=6.5, lw=1.6, ls=ls, capsize=3, label=name)
ax.set_xlabel('명령 지연 주입 (ms)'); ax.set_ylabel('랩 평균 경로 오차 (m, GPS)'); ax.set_xlim(-8, 165)
ax.set_title('SILS 명령 지연별 경로 오차 (평균 ± 표준편차, 조건당 4~6랩)', loc='left', fontsize=10)
ax.legend(frameon=False, fontsize=8.5, loc='upper left'); save(fig, 'g_delay_supp.png')

# 보드 벤치: 출발 구간 풀이 시간 분포
d = pd.read_csv(os.path.join(HERE, 'S4_startup.csv')); d = d[d.link == 0]
fig, ax = plt.subplots(figsize=(6.4, 3.4))
ax.hist(d.solve_ms, bins=np.arange(0, 31, 0.5), color=L, edgecolor=D, linewidth=0.4)
ax.set_yscale('log')
for x, lab, ls in [(28.5, '기존 벤치 최대 28.5 ms', ':'), (29.0, '예산 29 ms', '-'), (30.7, '장시간 시험 30.7 ms (재현 안 됨)', '--')]:
    ax.axvline(x, color=K, ls=ls, lw=1.2, label=lab)
ax.legend(frameon=True, framealpha=1, edgecolor=G, fontsize=8, loc='upper left', title='이번 최대 %.2f ms' % d.solve_ms.max(), title_fontsize=8.5)
ax.set_xlim(0, 33); ax.set_xlabel('보드 풀이 시간 (ms, 운영 설정 S2·Q16)'); ax.set_ylabel('응답 수 (로그)')
ax.set_title('출발 구간 요청 %s개 (SILS 녹화 119개)' % format(len(d), ','), loc='left', fontsize=10)
save(fig, 'g_startup_bench.png')
print('ok')
