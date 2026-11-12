#!/usr/bin/env python3
"""HILS 주행 중 MCU 풀이 시간의 극값 분석 (측정 기반 확률적 최악값 추정, WCET 증명 아님).
입력: 공식 HILS 20랩(final3_hils_*) + 밤샘 HILS 57랩(soak_hils) 의 /bmpc/debug [5] = 풀이 시간(ms), [1] = status.
방법: 블록 최대값(블록 = 100주기 = 5 s) 에 GEV 를 맞추고 블록당 초과 확률 1e-3 ~ 1e-9 의 값을 구한다.
      형상 모수가 음수(유계 꼬리)면 추정 상한(endpoint)도 낸다. 반복 상한(SQP 2 × QP 16)이 실제 상한을 만든다는 설계와 맞는지 본다.
  python3 pwcet.py   -> pwcet.json, solve_samples.npz"""
import csv, glob, json, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'solve_samples.npz')
SETS = [(os.path.expanduser('~/catkin_ws/bags/exp_20261111_final'), lambda c: c.startswith('final3_hils_')),
        (os.path.expanduser('~/catkin_ws/bags/exp_20261112_soak'), lambda c: c.startswith('soak_hils'))]
if os.path.exists(CACHE):
    z = np.load(CACHE, allow_pickle=True); laps = list(z['laps'])
else:
    import rosbag
    laps = []
    for base, sel in SETS:
        for r in csv.DictReader(open(os.path.join(base, 'summary.csv'))):
            if not (sel(r['condition']) and r['valid'] == '1'): continue
            s = []
            with rosbag.Bag(os.path.join(base, 'bags', r['bag'])) as b:
                for _, m, _ in b.read_messages(topics=['/bmpc/debug']):
                    s.append((m.data[5], m.data[1], m.data[2]))   # solve ms, status, link result
            laps.append(np.array(s))
    np.savez(CACHE, laps=np.array(laps, dtype=object))
from scipy import stats
blocks, allv = [], []
for a in laps:
    ok = a[(a[:, 2] == 0)]           # 링크 정상 주기만
    v = ok[:, 0]; v = v[v > 0]       # 풀이를 한 주기만 (정지 요청 0 ms 제외)
    allv.append(v)
    for i in range(0, len(v) - 99, 100): blocks.append(v[i:i + 100].max())
allv = np.concatenate(allv); blocks = np.array(blocks)
c, loc, scale = stats.genextreme.fit(blocks)
out = dict(laps=len(laps), cycles=int(len(allv)), blocks=int(len(blocks)), obs_max=float(allv.max()),
           obs_p999=float(np.percentile(allv, 99.9)), gev=dict(c=float(c), loc=float(loc), scale=float(scale)),
           ks_p=float(stats.kstest(blocks, 'genextreme', args=(c, loc, scale)).pvalue))
out['quantiles'] = {('%.0e' % p): float(stats.genextreme.isf(p, c, loc, scale)) for p in (1e-3, 1e-6, 1e-9)}
out['endpoint'] = float(loc + scale / c) if c > 0 else None    # scipy: c > 0 = 유계 (Weibull 형)
# 부트스트랩으로 1e-9 분위수 범위
rng = np.random.default_rng(0); qs = []
for _ in range(200):
    bb = rng.choice(blocks, len(blocks)); cc, ll, ss = stats.genextreme.fit(bb)
    qs.append(stats.genextreme.isf(1e-9, cc, ll, ss))
out['q1e-9_boot_p5_p95'] = [float(np.percentile(qs, 5)), float(np.percentile(qs, 95))]
json.dump(out, open(os.path.join(HERE, 'pwcet.json'), 'w'), indent=1)
print(json.dumps(out, indent=1))
