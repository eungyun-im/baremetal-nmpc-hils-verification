#!/usr/bin/env python3
"""S4~S6 결과 정리 (PLAN.md 의 판정 문장을 그대로 계산). -> results.json, stdout"""
import csv, glob, json, os, sys
import numpy as np, pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
V = os.path.join(HERE, '..', 'verdict'); sys.path.insert(0, V)
OUT = {}

# ---------------- S4
p = os.path.join(HERE, 'S4_startup.csv')
if os.path.exists(p):
    d = pd.read_csv(p)
    ok = d[d.link == 0]
    s = ok.solve_ms
    over = ok[ok.solve_ms > 28.5]
    OUT['S4'] = dict(
        files=int(d.file.nunique()), requests=int(len(d)), link_bad=int((d.link != 0).sum()),
        solve_max=float(s.max()), solve_p999=float(np.percentile(s, 99.9)), solve_p99=float(np.percentile(s, 99)),
        total_max=float(ok.total_ms.max()), rtt_max=float(ok.rtt_ms.max()),
        over_28_5=int(len(over)), over_29=int((s > 29.0).sum()),
        over_rows=over[['file', 'k', 'rel', 'abn', 'v', 'status', 'iters', 'solve_ms']].to_dict('records'),
        worst=ok.loc[s.idxmax(), ['file', 'k', 'rel', 'abn', 'v', 'status', 'iters', 'solve_ms', 'total_ms']].to_dict(),
        status=ok.status.value_counts().to_dict(),
        # 출발 순간 근처(마지막 비정상 요청 뒤 0~5개)와 그 밖
        near_max=float(ok[(ok.rel >= 0) & (ok.rel <= 5)].solve_ms.max()) if ((ok.rel >= 0) & (ok.rel <= 5)).any() else None,
        far_max=float(ok[~((ok.rel >= 0) & (ok.rel <= 5))].solve_ms.max()),
        statement=bool(s.max() <= 29.0 and (d.link != 0).sum() == 0))

# ---------------- S5
new_p = os.path.expanduser('~/catkin_ws/bags/exp_20261112_supp_dose/summary.csv')
old_p = os.path.expanduser('~/catkin_ws/bags/exp_20261105/summary.csv')
if os.path.exists(new_p):
    new = pd.read_csv(new_p); old = pd.read_csv(old_p)
    new = new[new.valid == 1]; old = old[old.valid == 1]
    r = {}
    for dly in (0, 50, 75, 100):
        on = new[new.condition == 'supp_dose_d%d_on' % dly].err_gps_mean.astype(float)
        off = old[old.condition == 'sils_dose_d%d_off' % dly].err_gps_mean.astype(float)
        old_on = old[old.condition == 'sils_dose_d%d_on' % dly].err_gps_mean.astype(float)
        r[dly] = dict(on_n=int(len(on)), on_mean=float(on.mean()) if len(on) else None, on_sd=float(on.std()) if len(on) > 1 else None,
                      on_laps=[round(x, 4) for x in on],
                      off_n=int(len(off)), off_mean=float(off.mean()) if len(off) else None, off_sd=float(off.std()) if len(off) > 1 else None,
                      old_on=[round(x, 4) for x in old_on])
        if dly < 100 and len(on) and len(off):
            r[dly]['statement'] = bool(on.mean() <= off.mean())
        if dly == 100 and len(on) and len(old_on):
            r[dly]['bridge_in_range'] = bool(old_on.min() <= on.mean() <= old_on.max())
    a = pd.read_csv(new_p)
    r['attempts'] = int(len(a)); r['valid'] = int((a.valid == 1).sum())
    r['invalid'] = a[a.valid != 1].invalid.value_counts().to_dict() if 'invalid' in a else {}
    OUT['S5'] = r

# ---------------- S6 (공식 규칙과 같은 판정, req_verdict 의 lap_metrics 재사용)
f_p = os.path.expanduser('~/catkin_ws/bags/exp_20261112_supp_fault/summary.csv')
if os.path.exists(f_p):
    import req_verdict as rv
    rv.CACHE = os.path.join(HERE, 'cache'); os.makedirs(rv.CACHE, exist_ok=True)
    LAPS = os.path.expanduser('~/catkin_ws/bags/exp_20261112_supp_fault')
    rows = list(csv.DictReader(open(f_p)))
    fnum = rv.fnum; LIM = rv.LIM['react']
    def metrics(r):
        if r.get('bag') and '_m' not in r:
            r['_m'], _ = rv.lap_metrics(os.path.join(LAPS, 'bags', r['bag']))
        return r.get('_m', {})
    stop_ok = lambda r, s, t: s is not None and s >= 3 and t is not None and t <= LIM
    nostop = lambda r, s, t: s is not None and s < 3
    def plaus_ok(r, s, t):
        m = metrics(r); det = (s is not None and s >= 2) or m.get('plaus_count', 0) > 0
        e = fnum(r.get('fault_err_gps_max')); return det and e is not None and e <= rv.LIM['path']
    rules = [('SR-01', 'blackout 0.35', 'fault_blackout_', stop_ok),
             ('SR-02', 'veh_loss 0.6', 'fault_veh_loss_', lambda r, s, t: s is not None and s >= 2 and t is not None and t <= LIM),
             ('SR-03', 'bus_off 0.5', 'fault_bus_off_', lambda r, s, t: s is not None and s >= 3),
             ('SR-05', '10° 스텝', 'plaus_step10_', plaus_ok),
             ('SR-06', 'wdt_hang', 'wdt_hang_', lambda r, s, t: stop_ok(r, s, t) and metrics(r).get('fault_recovered', False)),
             ('SR-07', 'req_crc 2', 'fault_req_crc_', nostop),
             ('SR-07', 'veh_crc 3', 'fault_veh_crc_', nostop)]
    off = list(csv.DictReader(open(os.path.expanduser('~/catkin_ws/bags/exp_20261111_final/summary.csv'))))
    res = []
    allok = True; nvalid = 0
    for sr, label, prefix, rule in rules:
        vs = [r for r in rows if r['condition'].startswith('supp_' + prefix) and r['valid'] == '1']
        good = 0; times = []; fails = []
        for r in vs:
            s, t = fnum(r.get('worst_state')), fnum(r.get('reaction_ms'))
            ok = rule(r, s, t); good += ok; nvalid += 1
            if t is not None: times.append(t)
            if not ok: fails.append('%s#%s state=%s react=%s err=%s' % (r['condition'], r['attempt'], s, t, r.get('fault_err_gps_max')))
        allok &= good == len(vs)
        ov = [r for r in off if r['condition'].startswith(prefix) and r['valid'] == '1']
        otimes = [fnum(r.get('reaction_ms')) for r in ov if fnum(r.get('reaction_ms')) is not None]
        res.append(dict(sr=sr, fault=label, valid=len(vs), passed=good, react=[round(x) for x in times], fails=fails,
                        official_valid=len(ov), official_react=[round(x) for x in otimes],
                        combined_range=[round(min(times + otimes)), round(max(times + otimes))] if times + otimes else None))
    a = rows
    OUT['S6'] = dict(rows=res, attempts=len(a), valid=nvalid, statement=bool(allok and nvalid >= 28),
                     invalid={k: sum(1 for r in a if r['valid'] != '1' and r.get('invalid') == k) for k in set(r.get('invalid') for r in a if r['valid'] != '1')})

json.dump(OUT, open(os.path.join(HERE, 'results.json'), 'w'), ensure_ascii=False, indent=1, default=str)
print(json.dumps(OUT, ensure_ascii=False, indent=1, default=str)[:8000])
