#!/usr/bin/env python3
"""HILS E2E 지연 분해 (판정 아님). 공식 HILS 20랩(final3_hils_*), 판정과 같은 구간 [t0, t1] (verdict/cache 의 값).
같은 주기끼리 묶는다: /bmpc/debug 한 건(링크 정상, 풀이를 한 주기)마다 같은 seq 의 /bmpc/safety,
가장 가까운 /mpc/controller_time_ms 와 /control_feedback/delay_comp (20 ms 안)를 찾고, 넷이 모두 있는 주기만 쓴다.
  /bmpc/debug: [0] seq, [2] 링크 결과, [3] 왕복 ms, [4] MCU 처리 ms (요청 수신 완료~응답 송신 직전), [5] 풀이 ms
  /bmpc/safety: [0] seq, [2] 안전 제어기 확인 대기 ms
  delay_comp: [1] odom_age s (요청을 채울 때의 자세 나이)
E2E(판정 정의) = odom_age + 제어 한 주기.
'전송 구간' = 왕복 - MCU 처리: CAN 선로 시간만이 아니라 USB 직렬, 커널, 브리지의 요청 프레임 간격 대기를 포함한 나머지 값.
'PC 그 밖' = 제어 한 주기 - 왕복 - 안전 확인 대기: 역시 나머지 값.
이력: 1판은 bag 전체(8.6만 주기), 2판은 판정 구간이지만 항목마다 모집단이 달랐다(풀이 없는 1,272주기가 일부 항목에만 포함). 3판(이 파일)은 같은 주기끼리 묶었다."""
import csv, json, os
import numpy as np, rosbag
HERE = os.path.dirname(os.path.abspath(__file__)); C = os.path.join(HERE, '..', 'verdict', 'cache')
B = os.path.expanduser('~/catkin_ws/bags/exp_20261111_final')
rows = [r for r in csv.DictReader(open(os.path.join(B, 'summary.csv'))) if r['condition'].startswith('final3_hils_') and r['valid'] == '1']
REC = []; n_dbg = 0; n_all_ct = 0
def nearest(ts, t):
    i = np.searchsorted(ts, t); i = np.clip(i, 1, len(ts) - 1)
    return np.where(np.abs(ts[i - 1] - t) < np.abs(ts[i] - t), i - 1, i)
for r in rows:
    m = json.load(open(os.path.join(C, r['bag'][:-4] + '.json'))); t0, t1 = m['t0'], m['t1']
    dbg, saf, ct, dc = [], [], [], []
    with rosbag.Bag(os.path.join(B, 'bags', r['bag'])) as b:
        for tp, msg, st in b.read_messages(topics=['/bmpc/debug', '/mpc/controller_time_ms', '/control_feedback/delay_comp', '/bmpc/safety']):
            t = st.to_sec()
            if not (t0 - 0.1 <= t <= t1 + 0.1): continue
            if tp == '/bmpc/debug': dbg.append((t, msg.data[0], msg.data[2], msg.data[3], msg.data[4], msg.data[5]))
            elif tp == '/bmpc/safety': saf.append((t, msg.data[0], msg.data[2]))
            elif tp == '/mpc/controller_time_ms': ct.append((t, msg.data))
            else: dc.append((t, msg.data[1] * 1e3))
    dbg, saf, ct, dc = map(np.array, (dbg, saf, ct, dc))
    n_all_ct += int(((ct[:, 0] >= t0) & (ct[:, 0] <= t1)).sum())
    d = dbg[(dbg[:, 0] >= t0) & (dbg[:, 0] <= t1) & (dbg[:, 2] == 0) & (dbg[:, 5] > 0)]; n_dbg += len(d)
    js, jc, jd = nearest(saf[:, 0], d[:, 0]), nearest(ct[:, 0], d[:, 0]), nearest(dc[:, 0], d[:, 0])
    ok = (saf[js, 1] == d[:, 1]) & (np.abs(saf[js, 0] - d[:, 0]) < 0.02) & (np.abs(ct[jc, 0] - d[:, 0]) < 0.02) & (np.abs(dc[jd, 0] - d[:, 0]) < 0.02)
    REC.append(np.column_stack([dc[jd, 1], ct[jc, 1], d[:, 3], d[:, 4], d[:, 5], saf[js, 2]])[ok])
R = np.vstack(REC); age, ctrl, rtt, mcu, solve, wait = R.T
comp = dict(age=age, pc_other=ctrl - rtt - wait, link=rtt - mcu, mcu_total=mcu, safe_wait=wait)
out = dict(cycles_all=n_all_ct, cycles_solved=n_dbg, cycles_used=int(len(R)))
out['mean'] = {k: float(v.mean()) for k, v in comp.items()}
out['mean'].update(solve=float(solve.mean()), rtt=float(rtt.mean()), ctrl=float(ctrl.mean()), e2e=float((age + ctrl).mean()))
out['p99'] = {k: float(np.percentile(v, 99)) for k, v in comp.items()}
M = out['mean']
out['e2e_minus_link'] = M['e2e'] - M['link']
out['e2e_minus_link_and_safe_wait'] = M['e2e'] - M['link'] - M['safe_wait']
e = json.load(open(os.path.join(HERE, 'e2e_pairing_check.json')))
out['official'] = dict(hils=41.4, sils=10.8, nmpc_corrected=e['nmpc']['mean'])
json.dump(out, open(os.path.join(HERE, 'latency_split.json'), 'w'), indent=1)
print(json.dumps(out, indent=1))
