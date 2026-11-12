#!/usr/bin/env python3
"""관측 실패율의 95 % 클로퍼-피어슨 상한. 판정 아님, 보고용.  -> stats_bounds.json"""
import json
from scipy.stats import beta
def ub(k, n, a=0.05): return 1.0 if k >= n else float(beta.ppf(1 - a, k + 1, n - k))
items = [
 ('호스트 결함 캠페인 (SR-01·04 로직, 판정 대상)', 0, 8149),
 # HILS 결함 57랩(공식 29 + 보충 28)은 같은 조건 반복이라 독립 표본이 아니므로 상한을 계산하지 않는다 (ERRATA E-28)
 ('HILS 무결함 SAFE_STOP 오검출 (공식 20랩 + 밤샘 57랩, 주기)', 0, 76739 + 218697),
 ('MCU 풀이 > 29 ms (밤샘 HILS 주기)', 1, 218697),
 ('MCU 풀이 > 29 ms (보드 벤치: 합성 4,144 + 출발 구간 실입력 63,360)', 0, 4144 + 63360),
 ('제어 한 주기 ≥ 50 ms (밤샘 HILS 57랩, 랩 단위)', 2, 57),
]
out = []
for name, k, n in items:
    out.append(dict(item=name, fail=k, n=n, rate=k / n, ub95=ub(k, n)))
    print('%-60s %d/%d  95%% 상한 %.2e' % (name, k, n, ub(k, n)))
json.dump(out, open('stats_bounds.json', 'w'), ensure_ascii=False, indent=1)
