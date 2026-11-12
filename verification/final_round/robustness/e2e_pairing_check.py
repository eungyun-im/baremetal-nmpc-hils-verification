#!/usr/bin/env python3
"""E2E 짝짓기 편향 확인 (판정 아님). req_verdict.py 는 delay_comp 와 controller_time 을 20 ms 안에서만 짝지었다.
원본 NMPC 노드는 delay_comp 를 풀이 전에, controller_time 을 풀이 뒤에 발행하므로 풀이가 20 ms 를 넘은 주기가 빠진다.
여기서는 delay_comp 뒤 처음 나오는 controller_time (0.3 s 안)과 짝짓는다. 구간은 req_verdict 캐시의 t0~t1."""
import csv, json, os
import numpy as np, rosbag
B = os.path.expanduser('~/catkin_ws/bags/exp_20261111_final'); C = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'verdict', 'cache')
out = {}
for k in ('hils', 'sils', 'nmpc'):
    E, n_dc, n_ct, old = [], 0, 0, []
    for r in csv.DictReader(open(os.path.join(B, 'summary.csv'))):
        if not (r['condition'].startswith('final3_%s_' % k) and r['valid'] == '1'): continue
        key = r['bag'][:-4]; m = json.load(open(os.path.join(C, key + '.json'))); t0, t1 = m['t0'], m['t1']
        old.append(np.load(os.path.join(C, key + '_e2e.npy')))
        dc, ct = [], []
        with rosbag.Bag(os.path.join(B, 'bags', r['bag'])) as b:
            for tp, msg, st in b.read_messages(topics=['/control_feedback/delay_comp', '/mpc/controller_time_ms']):
                t = st.to_sec()
                if not (t0 <= t <= t1 + 0.3): continue
                if tp == '/mpc/controller_time_ms': ct.append((t, msg.data))
                elif t <= t1: dc.append((t, msg.data[1]))
        dc, ct = np.array(dc), np.array(ct); n_dc += len(dc); n_ct += len(ct)
        if k == 'nmpc':   # 풀이 전 발행 -> 뒤에 오는 첫 controller_time
            j = np.searchsorted(ct[:, 0], dc[:, 0], side='right')
        else:             # 브리지는 둘을 같은 시점에 발행 -> 가장 가까운 것
            j = np.searchsorted(ct[:, 0], dc[:, 0]); j = np.clip(j, 1, len(ct) - 1)
            j = np.where(np.abs(ct[j - 1, 0] - dc[:, 0]) < np.abs(ct[j, 0] - dc[:, 0]), j - 1, j)
        ok = (j < len(ct)); j = np.clip(j, 0, len(ct) - 1); ok &= np.abs(ct[j, 0] - dc[:, 0]) < 0.3
        E.append(dc[ok, 1] * 1e3 + ct[j[ok], 1])
    E = np.concatenate(E); O = np.concatenate(old)
    out[k] = dict(dc=n_dc, paired=int(len(E)), old_paired=int(len(O)), mean=float(E.mean()), p99=float(np.percentile(E, 99)), max=float(E.max()),
                  old_mean=float(O.mean()), old_p99=float(np.percentile(O, 99)), old_max=float(O.max()))
    print(k, out[k])
json.dump(out, open('e2e_pairing_check.json', 'w'), indent=1)
