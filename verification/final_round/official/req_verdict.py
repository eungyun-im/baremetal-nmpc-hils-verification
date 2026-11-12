#!/usr/bin/env python3
"""요구사항 16개(명세 표기는 15개였음, v0.3b에서 정정) 판정표와 그림을 한 번에 만든다 (req_test_spec.md v0.3 의 판정 규칙 그대로).

  source ~/catkin_ws/devel/setup.bash
  nice -n 19 /usr/bin/python3 req_verdict.py            # 표 + 그림
  nice -n 19 /usr/bin/python3 req_verdict.py --no-figs  # 표만

입력 (읽기만 한다)
  랩      ~/catkin_ws/bags/exp_20261111_final/summary.csv 와 bags/  (laps_plan_final3.json)
  호스트  ../sw_quality/{unit_tests.txt,coverage.txt,static_*.txt}, ../sw_quality/stack/bound.txt,
          ../host_runs/{campaign.txt,fault_campaign.csv,vs_ipopt_*.txt}, ../pil_fma/compare_result.csv
          ../../ref_solver/operating_config.json, ../../ref_solver/pil/pil_frames_*.csv, ../../../bench/worst_case_solvefail.txt
          ../dump/result.json (TC-D01)
출력  verdict.md, verdict.csv, lap_metrics.csv, figs/*.png   (랩별 bag 지표는 cache/ 에 저장)

판정 규칙: 요구사항에 걸린 테스트 케이스가 모두 기대 결과를 만족하면 "충족", 하나라도 어기면 "미충족",
기준이 없는 항목은 "보고", 근거가 아직 없으면 "진행 전". 무효 랩은 판정에서 빼고 개수를 함께 적는다.
"""
import argparse, bisect, csv, glob, json, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
F3 = os.path.dirname(HERE)                                   # analysis/final/final3_20261111
FINAL = os.path.dirname(F3)                                  # analysis/final
ANALYSIS = os.path.dirname(FINAL)
LAPS = os.path.expanduser('~/catkin_ws/bags/exp_20261111_final')
CACHE = os.path.join(HERE, 'cache'); FIGS = os.path.join(HERE, 'figs')
SKIP_START_S, EKF_LAG_S, CHORD, GRID_M = 20.0, 0.04, 5, 1.0
D = 180 / np.pi
LIM = dict(rtt=45.0, solve=29.0, sc_gap=10.5, path=0.80, ctrl=50.0, react=290.0, corr=0.90, sc_exec_us=1000.0)
CFG = {'hils': 'HILS (MCU)', 'sils': 'SILS 동일 설정', 'nmpc': '원본 NMPC'}
COL = {'hils': '#2a78d6', 'sils': '#eb6834', 'nmpc': '#1baf7a'}          # 범주 1~3 (모든 쌍 검증된 3개)
GOOD, BAD, INK, INK2, GRID = '#0ca30c', '#d03b3b', '#0b0b0b', '#52514e', '#e4e3df'
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi


def fnum(v):
    try:
        x = float(v)
        return x if np.isfinite(x) else None
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------- 랩 bag 지표
def lap_metrics(bag_path):
    """bag 하나 -> 지표 dict (+ 조향 거리 격자 프로파일). 결과는 cache 에 둔다."""
    key = os.path.basename(bag_path)[:-4]
    jpath, ppath = os.path.join(CACHE, key + '.json'), os.path.join(CACHE, key + '_steer.npy')
    if os.path.exists(jpath) and os.path.exists(ppath):
        return json.load(open(jpath)), np.load(ppath)
    import rosbag
    ekf, ego, sec, paths, dc, ct, diag, steer, saf, inj = [], [], [], [], [], [], [], [], [], []
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[
                '/ekf_pose_with_covariance', '/Competition_topic', '/mission/active_section', '/local_path',
                '/control_feedback/delay_comp', '/mpc/controller_time_ms', '/bmpc/mcu_diag', '/ctrl_cmd_mpc',
                '/bmpc/safety', '/bmpc/inject_log']):
            t = stamp.to_sec()
            if topic == '/ekf_pose_with_covariance':
                q = msg.pose.pose.orientation
                ekf.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                            np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
            elif topic == '/Competition_topic': ego.append((t, msg.velocity.x))
            elif topic == '/mission/active_section': sec.append((t, msg.section_id))
            elif topic == '/local_path': paths.append((t, np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])))
            elif topic == '/control_feedback/delay_comp': dc.append((t,) + tuple(msg.data)[:6])
            elif topic == '/mpc/controller_time_ms': ct.append((t, msg.data))
            elif topic == '/bmpc/mcu_diag': diag.append((t,) + tuple(msg.data)[:4])
            elif topic == '/ctrl_cmd_mpc': steer.append((t, msg.steering))
            elif topic == '/bmpc/safety': saf.append((t, msg.data[3]))
            else: inj.append((t, msg.data))
    ekf, ego, ct, steer = np.array(ekf), np.array(ego), np.array(ct), np.array(steer)
    mv = ego[ego[:, 1] > 0.3]
    t0, t1 = mv[0, 0], mv[-1, 0]; late = t0 + SKIP_START_S
    sect = [s[0] for s in sec]
    def section(t):
        i = bisect.bisect_right(sect, t) - 1
        return sec[i][1] if i >= 0 else ''
    path_t = [p[0] for p in paths]
    lat, he = [], []
    for t, x, y, yaw in ekf:
        if not (late <= t <= t1) or section(t) == 'M8':
            continue
        i = bisect.bisect_right(path_t, t) - 1
        if i < 0 or len(paths[i][1]) < 2 * CHORD + 2: continue
        pts = paths[i][1]; seg = pts[1:] - pts[:-1]
        u = np.clip(((np.array([x, y]) - pts[:-1]) * seg).sum(1) / ((seg ** 2).sum(1) + 1e-9), 0, 1)
        foot = pts[:-1] + seg * u[:, None]; d = np.hypot(*(foot - [x, y]).T); n = int(d.argmin())
        if (n == 0 and u[0] == 0.0) or (n == len(seg) - 1 and u[-1] == 1.0): continue
        a, b = max(n - CHORD, 0), min(n + CHORD + 1, len(pts) - 1)
        lat.append(d[n]); he.append(wrap(yaw - np.arctan2(pts[b, 1] - pts[a, 1], pts[b, 0] - pts[a, 0])) * D)
    lat, he = np.array(lat), np.array(he)
    m = dict(t0=t0, t1=t1, lat_rms=float(np.sqrt((lat ** 2).mean())), lat_p95=float(np.percentile(lat, 95)), lat_max=float(lat.max()),
             he_rms=float(np.sqrt((he ** 2).mean())), he_p95=float(np.percentile(np.abs(he), 95)), he_max=float(np.abs(he).max()))
    cc = ct[(ct[:, 0] >= t0) & (ct[:, 0] <= t1)]
    m.update(ctrl_n=len(cc), ctrl_mean=float(cc[:, 1].mean()), ctrl_p99=float(np.percentile(cc[:, 1], 99)), ctrl_max=float(cc[:, 1].max()))
    start = np.diff(cc[:, 0] - cc[:, 1] * 1e-3) * 1e3
    m.update(start_std=float(start.std()), start_p01=float(np.percentile(start, 1)), start_p99=float(np.percentile(start, 99)),
             start_min=float(start.min()), start_max=float(start.max()))
    if dc:
        c = np.array(dc); c = c[(c[:, 0] >= t0) & (c[:, 0] <= t1)]
        if '_nmpc_' in key:
            # 정정 2026-11-12 (errata_20261112/ERRATA.md E-01): 원본 NMPC 노드는 delay_comp 를 풀이 전에, controller_time 을
            # 풀이 뒤에 발행한다. 20 ms 안에서만 짝지으면 풀이가 20 ms 를 넘은 주기(약 30 %)가 빠져 E2E 가 낮게 나온다.
            # delay_comp 뒤에 처음 오는 controller_time (0.3 s 안)과 짝짓는다. 브리지(HILS·SILS)는 두 토픽을 같은 시점에 내므로 그대로 둔다.
            j = np.searchsorted(ct[:, 0], c[:, 0], side='right'); ok = j < len(ct); j = np.clip(j, 0, len(ct) - 1)
            ok &= (ct[j, 0] - c[:, 0]) < 0.3
            e2e = c[ok, 2] * 1e3 + ct[j[ok], 1]
        else:
            j = np.clip(np.searchsorted(cc[:, 0], c[:, 0]), 0, len(cc) - 1); ok = np.abs(cc[j, 0] - c[:, 0]) < 0.02
            e2e = c[ok, 2] * 1e3 + cc[j[ok], 1]
        m.update(e2e_n=int(ok.sum()), e2e_mean=float(e2e.mean()), e2e_p99=float(np.percentile(e2e, 99)), e2e_max=float(e2e.max()))
        np.save(os.path.join(CACHE, key + '_e2e.npy'), e2e)
    if diag:
        g = np.array(diag); g = g[(g[:, 0] >= t0) & (g[:, 0] <= t1)]
        if len(g):
            pc = g[:, 4].astype(np.int64) >> 4
            m.update(sc_gap_max=float(g[:, 2].max()), sc_exec_max_us=float(g[:, 3].max()), plaus_count=int(pc.max() - pc.min()))
    # 결함 뒤 안전 상태: SAFE_STOP 다음에 NORMAL 로 돌아왔는지 (wdt), DEGRADED 이상이 나왔는지 (plaus)
    fault_t = next((t for t, s in inj if s.startswith('start ') and 'safety_reset' not in s), None)
    if fault_t is not None and saf:
        s = np.array(saf); s = s[s[:, 0] > fault_t]
        stop = np.where(s[:, 1] == 3)[0]
        m['fault_worst'] = int(s[:, 1].max()) if len(s) else None
        m['fault_recovered'] = bool(len(stop) and (s[stop[0]:, 1] == 1).any())
        m['fault_degraded'] = bool((s[:, 1] >= 2).any())
    np.save(os.path.join(CACHE, key + '_ctrl.npy'), cc[:, 1])
    # 조향 프로파일 (report_laps.steer_profile 과 같은 방식: EKF 주행 거리 1 m 격자)
    step = np.hypot(np.diff(ekf[:, 1]), np.diff(ekf[:, 2])); step[step > 5.0] = 0.0
    dist = np.concatenate([[0.0], np.cumsum(step)]); grid = np.arange(0.0, dist[-1], GRID_M)
    keep = np.concatenate([[True], np.diff(dist) > 1e-3])
    prof = np.interp(np.interp(grid, dist[keep], ekf[keep, 0]), steer[:, 0], steer[:, 1])
    json.dump(m, open(jpath, 'w')); np.save(ppath, prof)
    return m, prof


# ---------------------------------------------------------------- 근거 파일
def read(path):
    return open(path).read() if os.path.exists(path) else None


def host_evidence():
    ev = {}
    t = read(os.path.join(F3, 'sw_quality', 'unit_tests.txt'))
    if t:
        lines = [l for l in t.splitlines() if l.strip()]
        bad = [l for l in lines if not re.search(r'\b0 failed|\(0 failures\)', l)]
        ev['unit'] = dict(ok=not bad, n=len(lines), text='실행 파일 %d개, 실패 %d' % (len(lines), len(bad)))
    t = read(os.path.join(F3, 'sw_quality', 'coverage.txt'))
    if t:
        cov = dict(re.findall(r'(lines|functions|branches): ([\d.]+)%', t))
        ev['cov'] = cov
    c = os.path.join(F3, 'host_runs', 'fault_campaign.csv')
    if os.path.exists(c):
        rows = list(csv.DictReader(open(c)))
        # boundary_stop = 150~250 ms 경계 구간(명세 4절: 기준 없음, 판정 제외)
        fails = [r for r in rows if r['verdict'] not in ('pass', 'boundary_stop')]
        nb = sum(r['verdict'] == 'boundary_stop' for r in rows)
        long_ = [r for r in rows if r['kind'] != '0' and int(r['d']) >= 6]
        stops = [float(r['stop_ms']) for r in long_ if fnum(r['stop_ms']) is not None and float(r['stop_ms']) >= 0]
        ev['campaign'] = dict(ok=not fails and len(rows) > 0, n=len(rows), fails=len(fails), boundary=nb,
                              stop_max=max(stops) if stops else None)
    for v in ['f64', 'f32']:
        t = read(os.path.join(F3, 'host_runs', 'vs_ipopt_%s.txt' % v))
        if t: ev['vs_ipopt_' + v] = dict(ok=bool(re.search(r'\b0 failed|PASS', t)) and 'FAIL' not in t, tail=t.strip().splitlines()[-1][:160])
    oc = os.path.join(FINAL, 'ref_solver', 'operating_config.json')
    if os.path.exists(oc):
        d = json.load(open(oc))['float_S2Q16']   # 보드 운영 설정 (SQP 2·QP 16, float)
        ev['ref'] = dict(max=d['d0_max'], n=d['n_moving'], ok=d['d0_max'] <= 0.5)
    pil = sorted(glob.glob(os.path.join(FINAL, 'ref_solver', 'pil', 'pil_frames_*.csv')))
    if pil:
        import pandas as pd
        mx, mxd, n, st_ok, miss = 0.0, 0.0, 0, True, 0
        for f in pil:
            d = pd.read_csv(f); l = d[d.src == 'live'].reset_index(drop=True); p = d[(d.src == 'pil') & (d.t_us >= 0)].reset_index(drop=True)
            k = min(len(l), len(p)); miss += len(l) - len(p) if len(l) > len(p) else 0
            ds = (l.steer[:k] - p.steer[:k]).abs() * D
            mx = max(mx, ds.max()); mxd = max(mxd, ds[500:].max()); n += k
            st_ok &= bool((l.status[:k].values == p.status[:k].values).all())
        ev['pil'] = dict(ok=mx <= 0.01 and st_ok and miss == 0, max=mx, max_drive=mxd, n=n, status_ok=st_ok, missing=miss)
    t = read(os.path.join(ANALYSIS, 'bench', 'worst_case_solvefail.txt'))
    if t:
        m = re.search(r'SQP 2 QP 16: (\d+) requests, solve max ([\d.]+) ms.*?rtt max ([\d.]+) ms.*?link errors (\d+)', t)
        if m: ev['bench'] = dict(n=int(m.group(1)), solve=float(m.group(2)), rtt=float(m.group(3)), link=int(m.group(4)))
    t = read(os.path.join(F3, 'sw_quality', 'stack', 'bound.txt'))
    if t:
        m = re.search(r'BOUND (\d+) of (\d+)', t); ev['stack'] = dict(bound=int(m.group(1)), total=int(m.group(2)), ok='dynamic []' in t and int(m.group(1)) <= 8192)
    d = os.path.join(F3, 'dump', 'result.json')
    if os.path.exists(d): ev['dump'] = json.load(open(d))
    for k in ['static_cppcheck', 'static']:
        t = read(os.path.join(F3, 'sw_quality', k + '.txt'))
        if t: ev['static'] = t.strip().splitlines()[-3:]
    return ev


# ---------------------------------------------------------------- 판정
def judge(rows, ev):
    valid = lambda prefix: [r for r in rows if r['condition'].startswith(prefix) and r['valid'] == '1']
    invalid = lambda prefix: [r for r in rows if r['condition'].startswith(prefix) and r['valid'] != '1']
    base = {k: valid('final3_%s_' % k) for k in CFG}
    bad = {k: invalid('final3_%s_' % k) for k in CFG}
    for k in base:
        for r in base[k]:
            r['_m'], r['_p'] = lap_metrics(os.path.join(LAPS, 'bags', r['bag']))
    H, S, N = base['hils'], base['sils'], base['nmpc']
    out = []

    def add(rid, cat, req, crit, result, verdict, tcs, extra=''):
        out.append(dict(id=rid, cat=cat, req=req, crit=crit, result=result, verdict=verdict, tc=tcs, extra=extra))

    def every(laps, key, ok):
        vals = [fnum(r.get(key)) for r in laps]
        miss = sum(v is None for v in vals); vals = [v for v in vals if v is not None]
        return vals, miss, (not miss and len(vals) > 0 and all(ok(v) for v in vals))

    def need(n, laps):
        return '' if len(laps) >= n else ' (유효 %d/%d랩, 진행 중)' % (len(laps), n)

    # FR-01
    v, miss, ok = every(H, 'err_gps_max', lambda x: x <= LIM['path'])
    over = [r for r in H if fnum(r.get('err_gps_max')) and fnum(r['err_gps_max']) > LIM['path']]
    res = 'HILS %d랩 중 %d랩 초과, 최대 %.2f m, 랩 RMS 중앙값 %.3f m' % (len(H), len(over), max(v) if v else float('nan'),
                                                                 np.median([r['_m']['lat_rms'] for r in H]) if H else float('nan'))
    for k in ('sils', 'nmpc'):
        vv = [fnum(r.get('err_gps_max')) for r in base[k] if fnum(r.get('err_gps_max')) is not None]
        if vv: res += '. %s: %d/%d랩 초과, 최대 %.2f m' % (CFG[k], sum(x > LIM['path'] for x in vv), len(vv), max(vv))
    v03 = ('충족' if ok else '미충족') if len(H) >= 20 else ('미충족' if over else '진행 전')
    t4 = os.path.join(HERE, 'tracking_v04.csv')
    v04, r4 = '진행 전', ''
    if os.path.exists(t4):
        T = [x for x in csv.DictReader(open(t4)) if x['prereg'] == '1']
        th, tn = [x for x in T if x['config'] == 'hils'], [x for x in T if x['config'] == 'nmpc']
        if th:
            bad4 = [x for x in th if x['ok'] != '1']
            v04 = ('충족' if not bad4 else '미충족') if len(th) >= 15 else ('미충족' if bad4 else '진행 전')
            r4 = ('. **v0.4 (EKF 기준, 경로 전환 뒤 3 s 제외, 판정 표본):** HILS %d/%d랩 초과, 최대 %.2f m, RMS 중앙값 %.3f m'
                  % (len(bad4), len(th), max(float(x['max']) for x in th), float(np.median([float(x['rms']) for x in th]))))
            if tn:
                r4 += '; 원본 NMPC %d/%d랩 초과, 최대 %.2f m' % (sum(x['ok'] != '1' for x in tn), len(tn), max(float(x['max']) for x in tn))
            r4 += '. 초과는 모두 M3 재출발 경로 전환 뒤 3 s 시점(수렴 중)'
    add('FR-01', '기능', '지역 경로를 횡방향으로 추종한다', 'TC-H01 유효 20랩 모두 GPS 경로 오차 최대 ≤ 0.80 m (AC-10)',
        res + need(20, H), v03, 'TC-H01', extra=('v0.4 %s' % v04 + r4) if r4 else '')
    # FR-02
    res = '; '.join('%s 헤딩 RMS %.2f° (p95 %.2f°, 최대 %.1f°)' % (CFG[k], np.mean([r['_m']['he_rms'] for r in base[k]]),
                    np.mean([r['_m']['he_p95'] for r in base[k]]), max(r['_m']['he_max'] for r in base[k])) for k in CFG if base[k])
    add('FR-02', '기능', '경로 방향을 추종한다', '보고 (구성 간 차이는 AC-22)', res or '-', '보고', 'TC-H01, TC-S02, TC-N01')
    # FR-03
    u = [ev.get('vs_ipopt_f64'), ev.get('vs_ipopt_f32')]
    r_, p_ = ev.get('ref'), ev.get('pil')
    parts = []
    parts.append('TC-U01 %s' % ('통과' if all(x and x['ok'] for x in u) else ('불합격' if any(x and not x['ok'] for x in u) else '미실행')))
    if r_: parts.append('TC-R01 최대 %.3f° (%d 문제) %s' % (r_['max'], r_['n'], '통과' if r_['ok'] else '불합격'))
    if p_: parts.append('TC-P01 **불합격**: 전체 최대 %.1f°(출발 전 비정상 입력), 주행 중 최대 %.4f° > 0.01°' % (p_['max'], p_['max_drive']) if not p_['ok'] else 'TC-P01 통과')
    p4 = os.path.join(F3, 'pil_fma', 'v04', 'result.txt')
    v4txt = open(p4).read().strip() if os.path.exists(p4) else ''
    x4 = ('v0.4 %s: TC-P01 v0.4 (새 녹화 3개, 1 m/s 이상 0.5°) %s' % ('충족' if v4txt.startswith('통과') else '미충족', v4txt)) if v4txt else ''
    allok = all(x and x['ok'] for x in u) and r_ and r_['ok'] and p_ and p_['ok']
    allok4 = all(x and x['ok'] for x in u) and r_ and r_['ok'] and v4txt.startswith('통과')
    add('FR-03', '기능', 'MCU 솔버가 원본 NMPC(Ipopt)와 같은 해를 낸다', '첫 조향 차이 ≤ 0.5°, 비용 ≤ +0.5 % (U01, R01). PIL 조향 차이 ≤ 0.01° (P01)',
        '; '.join(parts), '충족' if allok else '미충족', 'TC-U01, TC-R01, TC-P01', extra=x4)
    # FR-04
    eq = {}
    def compare(a, b):
        if len(a) < 2 or len(b) < 2: return None
        pa = [r['_p'] for r in a]; pb = [r['_p'] for r in b]
        size = min(min(map(len, pa)), min(map(len, pb)))
        ma, mb = np.mean([p[:size] for p in pa], 0), np.mean([p[:size] for p in pb], 0)
        corr = float(np.corrcoef(ma, mb)[0, 1])
        ea, eb = np.array([float(r['err_gps_mean']) for r in a]), np.array([float(r['err_gps_mean']) for r in b])
        ha, hb = np.array([r['_m']['he_rms'] for r in a]), np.array([r['_m']['he_rms'] for r in b])
        return dict(corr=corr, size=size, d21=abs(ea.mean() - eb.mean()), a21=ea.std(ddof=1) + eb.std(ddof=1),
                    d22=abs(ha.mean() - hb.mean()), a22=ha.std(ddof=1) + hb.std(ddof=1), ma=ma, mb=mb,
                    ok=corr >= LIM['corr'] and abs(ea.mean() - eb.mean()) <= ea.std(ddof=1) + eb.std(ddof=1)
                    and abs(ha.mean() - hb.mean()) <= ha.std(ddof=1) + hb.std(ddof=1))
    for name, a, b in [('H-N', H, N), ('H-S', H, S), ('S-N', S, N)]:
        eq[name] = compare(a, b)
    fmt = lambda c: ('상관 %.3f, 경로 오차 평균 차 %.3f m (허용 %.3f), 헤딩 RMS 차 %.2f° (허용 %.2f)' %
                     (c['corr'], c['d21'], c['a21'], c['d22'], c['a22'])) if c else '데이터 부족'
    res = 'HILS 대 원본 NMPC: %s. [분해] HILS 대 SILS: %s. SILS 대 NMPC: %s' % (fmt(eq['H-N']), fmt(eq['H-S']), fmt(eq['S-N']))
    v4 = '진행 전' if not eq['H-N'] else ('충족' if eq['H-N']['ok'] else '미충족')
    if eq['H-N'] and (len(H) < 20 or len(N) < 20): v4 += ' (잠정)'
    add('FR-04', '기능', 'MCU 제어 성능이 PC 원본 NMPC와 같다', 'HILS 대 원본 NMPC: AC-20 ≥ 0.90, AC-21·AC-22 차이 ≤ 랩 간 표준편차 합',
        res + need(20, H) + need(20, N), v4, 'TC-H01 + TC-N01 (TC-S02 분해)')
    # NFR-01
    a1 = every(H, 'rtt_max', lambda x: x < LIM['rtt']); a11 = every(H, 'ctrl_time_max', lambda x: x < LIM['ctrl'])
    a3 = every(H, 'link_fail', lambda x: x == 0); a4 = every(H, 'status_solver', lambda x: x == 0)
    b = ev.get('bench')
    ok = a1[2] and a11[2] and a3[2] and a4[2] and b and b['rtt'] < LIM['rtt'] and b['link'] == 0
    res = '왕복 최대 %.2f ms, 제어 한 주기 최대 %.2f ms, 링크 실패 %d, 풀이 오류 %d (HILS %d랩, %d 주기). 벤치 왕복 최대 %s ms' % (
        max(a1[0] or [np.nan]), max(a11[0] or [np.nan]), int(sum(a3[0])), int(sum(a4[0])), len(H), int(sum(float(r['cycles']) for r in H)),
        b['rtt'] if b else '-')
    add('NFR-01', '비기능', '한 제어 주기가 50 ms 안에 끝난다', '왕복 < 45 ms, 제어 한 주기 < 50 ms, 링크 실패 0, 풀이 오류 0 (20랩 모두)',
        res + need(20, H), ('충족' if ok else '미충족') if len(H) >= 20 else ('미충족' if H and not ok else '진행 전'), 'TC-H01, TC-B01')
    # NFR-02
    a2 = every(H, 'solve_max', lambda x: x <= LIM['solve'])
    ok = a2[2] and b and b['solve'] <= LIM['solve']
    add('NFR-02', '비기능', 'MPC 계산 시간에 상한이 있다', '운영 설정 풀이 최대 ≤ 29 ms (벤치 + 20랩)',
        '주행 최대 %.2f ms, 벤치 최대 %s ms (%s 요청). 반복 상한 SQP 2 × QP 16 으로 종료 보장, WCET 증명 아님' % (
            max(a2[0] or [np.nan]), b['solve'] if b else '-', b['n'] if b else '-') + need(20, H),
        ('충족' if ok else '미충족') if len(H) >= 20 else ('미충족' if H and not ok else '진행 전'), 'TC-B01, TC-H01')
    # NFR-03
    gaps = [r['_m'].get('sc_gap_max') for r in H if r['_m'].get('sc_gap_max') is not None]
    ex = [r['_m'].get('sc_exec_max_us') for r in H if r['_m'].get('sc_exec_max_us') is not None]
    ok = len(gaps) == len(H) and len(H) > 0 and max(gaps) <= LIM['sc_gap'] and ex and max(ex) < LIM['sc_exec_us']
    add('NFR-03', '비기능', '안전 태스크가 10 ms 주기로 돈다', '간격 최대 ≤ 10.5 ms, 1회 실행 < 1 ms (20랩 모두)',
        '간격 최대 %s ms, 실행 최대 %s µs' % ('%.2f' % max(gaps) if gaps else '-', '%.0f' % max(ex) if ex else '-') + need(20, H),
        ('충족' if ok else '미충족') if len(H) >= 20 else ('미충족' if H and not ok else '진행 전'), 'TC-H01')
    # NFR-04
    res = '; '.join('%s E2E 평균 %.1f / p99 %.1f ms, 제어 한 주기 평균 %.1f ms, 루프 시작 간격 표준편차 %.2f ms' % (
        CFG[k], np.mean([r['_m']['e2e_mean'] for r in base[k] if 'e2e_mean' in r['_m']]),
        np.mean([r['_m']['e2e_p99'] for r in base[k] if 'e2e_p99' in r['_m']]),
        np.mean([r['_m']['ctrl_mean'] for r in base[k]]), np.mean([r['_m']['start_std'] for r in base[k]])) for k in CFG if base[k])
    add('NFR-04', '비기능', '제어 루프 지터와 E2E 지연', '보고', res or '-', '보고', 'TC-H01, TC-S02, TC-N01')
    # NFR-05
    st, dp = ev.get('stack'), ev.get('dump')
    res = '정적 상한 %s B / 8,192 B' % (st['bound'] if st else '-')
    if dp: res += ', 실측 %s B, 플래시 %s' % (dp.get('stack_used'), '일치' if dp.get('flash_match') else '**불일치**')
    ok = st and st['ok'] and dp and dp.get('flash_match') and dp.get('stack_used') is not None and dp['stack_used'] <= st['bound']
    add('NFR-05', '비기능', '메모리가 정적이고 스택이 넘치지 않는다', '동적 할당 없음, 정적 상한 ≤ 8,192 B, 실측 ≤ 정적 상한, 플래시 = 최종 ELF',
        res, ('충족' if ok else '미충족') if dp else '진행 전', 'TC-A01, TC-D01')

    # 결함 랩
    def faults(prefix):
        return [r for r in rows if r['condition'].startswith(prefix)]
    def fault_line(rs, rule):
        vs = [r for r in rs if r['valid'] == '1']
        good, txt = 0, []
        for r in vs:
            st_, rt = fnum(r.get('worst_state')), fnum(r.get('reaction_ms'))
            ok = rule(r, st_, rt)
            good += ok
            txt.append('%s#%s 상태 %s, 반응 %s ms%s' % (r['condition'].split('_')[-1], r['attempt'], '-' if st_ is None else int(st_),
                                                    '-' if rt is None else '%.0f' % rt, '' if ok else ' ✗'))
        return vs, good, txt
    plan_n = {}
    for c in json.load(open(os.path.expanduser('~/catkin_ws/src/bare-metal-mpc/bmpc_bridge/scripts/laps_plan_final3.json'))):
        plan_n[c['name']] = c['laps']
    def need_f(prefix, vs):
        n = sum(v for k, v in plan_n.items() if k.startswith(prefix))
        return n, (len(vs) >= n)
    camp = ev.get('campaign')
    stop_ok = lambda lim: (lambda r, s, t: s is not None and s >= 3 and t is not None and t <= lim)
    nostop = lambda r, s, t: s is not None and s < 3
    def sr(rid, req, crit, tcs, groups, host=None):
        lines, allok, complete = [], True, True
        for prefix, rule, label in groups:
            vs, good, txt = fault_line(faults(prefix), rule)
            n, full = need_f(prefix, vs)
            complete &= full; allok &= good == len(vs)
            lines.append('%s %d/%d%s' % (label, good, n, '' if full else ' (유효 %d)' % len(vs)))
        if host is not None:
            if host[0] is None: complete = False      # 호스트 근거 파일이 아직 없음
            else: allok &= host[0]
            lines.insert(0, host[1])
        verdict = ('충족' if allok else '미충족') if complete else ('미충족' if not allok else '진행 전')
        add(rid, '기능안전', req, crit, '; '.join(lines), verdict, tcs)
    hc = (None if not camp else bool(camp['ok']), '호스트 캠페인 %s회 실패 %s (경계 구간 %s회 판정 제외)' % (camp['n'] if camp else '-', camp['fails'] if camp else '-', camp['boundary'] if camp else '-'))
    sr('SR-01', '제어 명령이 끊기면 안전 정지한다', '300 ms 이상 두절: SAFE_STOP ≤ 290 ms (HILS 3/3)', 'TC-C01, TC-H11',
       [('fault_blackout_', stop_ok(LIM['react']), 'blackout 0.35')], hc)
    sr('SR-02', '차속 신호가 끊기면 성능 저하 모드로 간다', 'DEGRADED 이상 ≤ 290 ms (2/2)', 'TC-H12',
       [('fault_veh_loss_', lambda r, s, t: s is not None and s >= 2 and t is not None and t <= LIM['react'], 'veh_loss 0.6')])
    sr('SR-03', 'CAN 버스오프면 안전 정지한다', 'SAFE_STOP (2/2)', 'TC-H13', [('fault_bus_off_', lambda r, s, t: s is not None and s >= 3, 'bus_off 0.5')])
    u = ev.get('unit')
    sr('SR-04', '솔버가 실패하면 유지하고, 이어지면 안전 정지한다', '≤ 100 ms 실패: SAFE_STOP 없음 (4/4). ≥ 300 ms: SAFE_STOP ≤ 290 ms (10/10). 호스트 통과',
       'TC-U02, TC-I01, TC-C01, TC-H14, TC-H15',
       [('fault_solve_fail_', stop_ok(LIM['react']), 'solve_fail 1000'), ('fault_solve_blip_', nostop, 'solve_fail 100')],
       (None if (hc[0] is None or not u) else bool(hc[0] and u['ok']), hc[1] + ', 단위 시험 %s' % ('통과' if u and u['ok'] else '미확인')))
    # SR-05: 검출 + 10 s 경로 오차, 무결함 오검출 0
    pl = faults('plaus_step10_')
    for r in pl:
        if r['valid'] == '1' and r.get('bag'): r['_m'], _ = lap_metrics(os.path.join(LAPS, 'bags', r['bag']))
    def plaus_ok(r, s, t):
        m = r.get('_m', {})
        det = (s is not None and s >= 2) or m.get('plaus_count', 0) > 0
        e = fnum(r.get('fault_err_gps_max'))
        return det and e is not None and e <= LIM['path']
    fp = every(H, 'safety_stop', lambda x: x == 0)
    fpn = [fnum(r.get('safety_normal')) for r in H]
    fp_ok = fp[2] and all(x is not None and x >= 1.0 for x in fpn)
    sr('SR-05', '그럴듯하지 않은 조향 명령을 막는다', '10° 스텝: 검출 + 10 s 경로 오차 ≤ 0.80 m (2/2). 무결함 오검출 0',
       'TC-H16, TC-H01', [('plaus_step10_', plaus_ok, '10° 스텝')],
       (fp_ok or len(H) == 0, '무결함 HILS %d랩 SAFE_STOP %d, NORMAL 비율 최소 %s' % (len(H), int(sum(fp[0])),
                                                                       '%.4f' % min(x for x in fpn if x is not None) if any(x is not None for x in fpn) else '-')))
    wd = faults('wdt_hang_')
    for r in wd:
        if r['valid'] == '1' and r.get('bag'): r['_m'], _ = lap_metrics(os.path.join(LAPS, 'bags', r['bag']))
    sr('SR-06', 'MCU가 멈추면 리셋한다', 'SAFE_STOP 기록 ≤ 290 ms + 정상 복귀 (2/2)', 'TC-H17',
       [('wdt_hang_', lambda r, s, t: stop_ok(LIM['react'])(r, s, t) and r.get('_m', {}).get('fault_recovered', False), 'wdt_hang')])
    sr('SR-07', '손상된 프레임은 버리고 정상 주행을 유지한다', 'CRC 오류: SAFE_STOP 없음 (각 2/2)', 'TC-H18',
       [('fault_req_crc_', nostop, 'req_crc 2'), ('fault_veh_crc_', nostop, 'veh_crc 3')])
    return out, base, bad, eq


# ---------------------------------------------------------------- 그림
def figures(base, eq, rows):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for f in ['NanumGothic', 'Noto Sans CJK', 'Noto Sans KR', 'UnDotum']:
        if any(f in x.name for x in font_manager.fontManager.ttflist):
            plt.rcParams['font.family'] = [x.name for x in font_manager.fontManager.ttflist if f in x.name][0]; break
    plt.rcParams.update({'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2, 'ytick.color': INK2, 'axes.grid': True,
                         'grid.color': GRID, 'grid.linewidth': 0.8, 'axes.spines.top': False, 'axes.spines.right': False,
                         'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb', 'font.size': 10, 'axes.unicode_minus': False})
    os.makedirs(FIGS, exist_ok=True)
    keys = [k for k in CFG if base[k]]
    rng = np.random.default_rng(0)

    def strip(ax, key, title, unit):
        for i, k in enumerate(keys):
            v = np.array([r['_m'][key] for r in base[k]])
            ax.scatter(i + rng.uniform(-0.12, 0.12, len(v)), v, s=36, color=COL[k], edgecolor='#fcfcfb', linewidth=1.5, zorder=3)
            ax.plot([i - 0.25, i + 0.25], [v.mean()] * 2, color=INK, lw=2, zorder=4)
        ax.set_xticks(range(len(keys)))
        ax.set_xticklabels(['%s (%d랩)\n평균 %.3g ± %.2g' % (CFG[k], len(base[k]), np.mean([r['_m'][key] for r in base[k]]),
                            np.std([r['_m'][key] for r in base[k]], ddof=1)) for k in keys], fontsize=9)
        ax.set_title(title, loc='left', color=INK); ax.set_ylabel(unit); ax.set_xlim(-0.5, len(keys) - 0.1)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    strip(axs[0], 'lat_rms', '랩별 횡 오차 RMS (EKF, 터널 제외)', 'm')
    strip(axs[1], 'he_rms', '랩별 헤딩 오차 RMS (EKF, 터널 제외)', '°')
    fig.suptitle('FR-01·02·04: 세 구성의 추종 성능 (점 = 랩, 막대 = 평균)', x=0.01, ha='left', color=INK)
    fig.tight_layout(); fig.savefig(os.path.join(FIGS, 'fr04_tracking.png'), dpi=160); plt.close(fig)

    c = eq.get('H-N')
    if c:
        fig, ax = plt.subplots(figsize=(10, 3.4))
        x = np.arange(c['size']) * GRID_M
        ax.plot(x, np.degrees(c['mb']), color=COL['nmpc'], lw=2, label='원본 NMPC (평균)')
        ax.plot(x, np.degrees(c['ma']), color=COL['hils'], lw=1.2, label='HILS (평균)')
        ax.set_xlabel('주행 거리 [m]'); ax.set_ylabel('조향 명령 [°]')
        ax.set_title('AC-20: 같은 위치의 조향 명령, 상관 %.3f (기준 0.90)' % c['corr'], loc='left', color=INK)
        ax.legend(frameon=False, loc='upper right'); fig.tight_layout(); fig.savefig(os.path.join(FIGS, 'fr04_steer_profile.png'), dpi=160); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for ax, suffix, title in [(axs[0], '_e2e.npy', '자세 측정 → 명령 발행 (E2E)'), (axs[1], '_ctrl.npy', 'PC 제어 한 주기 소요')]:
        data, labels = [], []
        for k in keys:
            arr = [np.load(os.path.join(CACHE, r['bag'][:-4] + suffix)) for r in base[k] if os.path.exists(os.path.join(CACHE, r['bag'][:-4] + suffix))]
            if arr: data.append(np.concatenate(arr)); labels.append(k)
        if not data: continue
        bp = ax.boxplot(data, whis=(1, 99), showfliers=False, widths=0.5, patch_artist=True, medianprops=dict(color=INK, lw=2))
        for p, k in zip(bp['boxes'], labels): p.set_facecolor(COL[k]); p.set_edgecolor(COL[k]); p.set_alpha(0.85)
        for i, d in enumerate(data): ax.annotate('평균 %.1f\np99 %.1f\n최대 %.1f' % (d.mean(), np.percentile(d, 99), d.max()), (i + 1.3, np.median(d)), fontsize=8.5, color=INK2, va='center')
        if suffix == '_ctrl.npy': ax.axhline(50, color=BAD, lw=1.2, ls='--'); ax.annotate('주기 50 ms', (0.55, 50.8), color=BAD, fontsize=9)
        ax.set_xticks(range(1, len(labels) + 1)); ax.set_xticklabels([CFG[k] for k in labels]); ax.set_ylabel('ms')
        ax.set_title(title + ' (상자 p25~p75, 수염 p1~p99)', loc='left', color=INK, fontsize=10)
    fig.suptitle('NFR-01·04: 지연과 주기 소요 (모든 제어 주기)', x=0.01, ha='left', color=INK)
    fig.tight_layout(); fig.savefig(os.path.join(FIGS, 'nfr_latency.png'), dpi=160); plt.close(fig)

    kinds = [('fault_solve_fail_', 'solve_fail 1000'), ('fault_blackout_', 'blackout 0.35'), ('fault_veh_loss_', 'veh_loss 0.6'), ('wdt_hang_', 'wdt_hang')]
    fig, ax = plt.subplots(figsize=(10, 3.6))
    for i, (p, lab) in enumerate(kinds):
        vs = [r for r in rows if r['condition'].startswith(p) and r['valid'] == '1' and fnum(r.get('reaction_ms')) is not None]
        y = [float(r['reaction_ms']) for r in vs]
        ax.scatter(y, i + rng.uniform(-0.15, 0.15, len(y)), s=40, color=COL['hils'], edgecolor='#fcfcfb', linewidth=1.5, zorder=3)
        if y: ax.annotate('n=%d, 최대 %.0f ms' % (len(y), max(y)), (max(y) + 6, i), va='center', fontsize=9, color=INK2)
    ax.axvline(LIM['react'], color=BAD, lw=1.2, ls='--'); ax.annotate('기준 290 ms', (LIM['react'] + 3, -0.35), color=BAD, fontsize=9)
    ax.set_yticks(range(len(kinds))); ax.set_yticklabels([k[1] for k in kinds]); ax.set_xlabel('주입 → 안전 상태 변화 [ms] (기록 해상도 약 50 ms)')
    ax.set_ylim(-0.5, len(kinds) - 0.5); ax.set_xlim(0, 360); ax.set_title('SR-01·02·04·06: HILS 결함 반응 시간 (새 펌웨어)', loc='left', color=INK)
    fig.tight_layout(); fig.savefig(os.path.join(FIGS, 'sr_reaction.png'), dpi=160); plt.close(fig)


def verdict_figure(out):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 0.42 * len(out) + 0.8)); ax.axis('off')
    mark = {'충족': ('충족', GOOD), '미충족': ('미충족', BAD), '보고': ('보고', INK2)}
    for i, r in enumerate(out):
        y = len(out) - i
        v = r['verdict']
        lab, col = mark.get(v.split(' (')[0], ('… ' + v, INK2))
        if r.get('extra'):
            ax.text(0.80, y, '(추가 분석 ' + r['extra'].split(':')[0].split(' (')[0] + ', 사후)', fontsize=8.5, color=INK2, va='center')
        ax.text(0.0, y, r['id'], fontsize=10, color=INK, weight='bold', va='center')
        ax.text(0.09, y, r['req'], fontsize=10, color=INK, va='center')
        ax.plot([0.645], [y], 'o', ms=8, color=col, transform=ax.transData)
        ax.text(0.66, y, lab, fontsize=10, color=col, weight='bold', va='center')
    ax.set_ylim(0.3, len(out) + 0.8); ax.set_xlim(0, 1)
    fin = [r['verdict'].split(' (')[0] for r in out]
    ax.set_title('요구사항 %d개 공식 판정 (측정 전 고정 v0.3, 89랩): 충족 %d · 미충족 %d · 보고 %d' % (
        len(out), fin.count('충족'), fin.count('미충족'), fin.count('보고')), loc='left')
    fig.tight_layout(); fig.savefig(os.path.join(FIGS, 'verdict_table.png'), dpi=160); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--no-figs', action='store_true'); opt = ap.parse_args()
    os.makedirs(CACHE, exist_ok=True)
    rows = list(csv.DictReader(open(os.path.join(LAPS, 'summary.csv')))) if os.path.exists(os.path.join(LAPS, 'summary.csv')) else []
    ev = host_evidence()
    if rows:   # v0.4 추종 오차(tracking_v04.py)와 PIL v0.4(pil_v04.py)를 먼저 계산한다
        import subprocess
        subprocess.run([sys.executable, os.path.join(HERE, 'tracking_v04.py')], check=True, stdout=subprocess.DEVNULL)
        V = os.path.join(F3, 'pil_fma', 'v04'); pil = sorted(glob.glob(os.path.join(V, 'pil_frames_*.csv')))
        if pil:
            o = subprocess.run([sys.executable, os.path.join(F3, 'pil_fma', 'pil_v04.py')] + pil, capture_output=True, text=True, cwd=V).stdout
            open(os.path.join(V, 'pil_v04_out.txt'), 'w').write(o)
            mx = max(float(m) for m in re.findall(r'max_moving=([\d.]+)', o)); p99 = max(float(m) for m in re.findall(r'p99_moving=([\d.]+)', o))
            n = sum(int(m) for m in re.findall(r' n=(\d+)', o))
            ok = 'TC-P01 v0.4: PASS' in o
            open(os.path.join(V, 'result.txt'), 'w').write('%s: 응답 %d, 누락 0, 1 m/s 이상 최대 %.2f°(출발 직후 5~7주기), p99 %.4f°' % ('통과' if ok else '**불합격**', n, mx, p99))
    out, base, bad, eq = judge(rows, ev)
    with open(os.path.join(HERE, 'verdict.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['id', 'cat', 'req', 'crit', 'result', 'verdict', 'tc', 'extra']); w.writeheader(); w.writerows(out)
    lm = []
    for k in base:
        for r in base[k]:
            lm.append(dict(config=k, condition=r['condition'], attempt=r['attempt'], bag=r['bag'], err_gps_mean=r.get('err_gps_mean'),
                           err_gps_max=r.get('err_gps_max'), **{x: y for x, y in r['_m'].items() if x not in ('t0', 't1')}))
    if lm:
        keys = []
        for d in lm: keys += [k for k in d if k not in keys]
        with open(os.path.join(HERE, 'lap_metrics.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(lm)
    n_all = len(rows); n_valid = sum(r['valid'] == '1' for r in rows)
    reasons = {}
    for r in rows:
        if r['valid'] != '1': reasons[r['invalid']] = reasons.get(r['invalid'], 0) + 1
    L = ['# 요구사항 판정표 (자동 생성: `req_verdict.py`)', '',
         '- **공식 판정 = 측정 전에 고정한 명세 v0.3 의 기준과 89랩 계획(`laps_plan_final3.json`)만으로 낸 판정.** 사후에 사용자 요청으로 정한 v0.4 정의와 확인용 20랩(`final4_*`)은 판정에 쓰지 않고 마지막 칸 "추가 분석"에만 적는다.',
         '- 명세: `docs/aspice/req_test_spec.md`. 랩: `%s` (시도 %d, 유효 %d, 무효 %s)' % (LAPS, n_all, n_valid, reasons or '없음'),
         '- 구성별 유효 무결함 랩: ' + ', '.join('%s %d' % (CFG[k], len(base[k])) for k in CFG) +
         '. 무효: ' + ', '.join('%s %d' % (CFG[k], len(bad[k])) for k in CFG),
         '- 커버리지(최종 소스): %s' % (', '.join('%s %s%%' % kv for kv in ev.get('cov', {}).items()) or '-'),
         '- **정정 (2026-11-12 17:26):** NFR-04 의 원본 NMPC E2E 를 평균 26.6 / p99 39.4 ms 에서 28.7 / 46.0 ms 로 고쳤다. 집계 스크립트가 풀이 20 ms 초과 주기(약 30 %)를 빠뜨린 오류였다(`../errata_20261112/ERRATA.md` E-01). 기준·랩·판정은 그대로이며 다른 15개 행은 바뀌지 않았다. 정정 전 파일은 `../errata_20261112/src_before_20261112_1726/`.',
         '- **참고 (2026-11-12 18:44):** 원본 NMPC 의 제어 한 주기 최대 163.1 ms 와 루프 시작 간격 표준편차 1.03 ms 는 주행 중 값이 아니다. 출발 전 비정상 입력 구간(한 주기 약 150 ms)의 마지막 주기가 20랩 중 8랩에서 판정 구간의 첫 표본으로 잡혔다. 랩의 첫 주기를 빼면 최대 61.7 ms(50 ms 초과 4 / 77,222), 표준편차 0.54 ms 다. 판정표의 값은 고정한 구간 정의대로 둔다(`../errata_20261112/ERRATA.md` E-15).', '',
         '| ID | 분류 | 요구사항 | 기준 (v0.3) | 결과 | 판정 (v0.3, 공식) | 테스트 케이스 | 추가 분석 (v0.4, 사후, 판정 아님) |', '|---|---|---|---|---|---|---|---|']
    for r in out:
        v = r['verdict']; v = '**%s**' % v if v.startswith('미충족') else v
        L.append('| %s | %s | %s | %s | %s | %s | %s | %s |' % (r['id'], r['cat'], r['req'], r['crit'], r['result'].replace('|', '/'), v, r['tc'], r['extra'] or '-'))
    cnt = {}
    for r in out:
        k3 = r['verdict'].split(' (')[0]; cnt[k3] = cnt.get(k3, 0) + 1
    L += ['', '공식 집계 (v0.3): ' + ', '.join('%s %d' % kv for kv in cnt.items()),
          '', '요구사항 개수: 명세 표에 정의된 요구사항은 16개다(FR 4, NFR 5, SR 7). 명세 본문이 "15개"로 잘못 적었던 것을 v0.3b 에서 정정했고, 항목을 빼거나 합치지 않았다.']
    open(os.path.join(HERE, 'verdict.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))
    if not opt.no_figs and rows:
        figures(base, eq, rows)
        verdict_figure(out)


if __name__ == '__main__':
    main()
