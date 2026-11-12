# 주행 시험 판정

기준: `docs/acceptance_criteria.md`, 데이터: `summary.csv` (117랩)

전체 무효 8랩 / 117랩

## 조건별 결과

| 조건 | 유효 / 시도 | 완주 | 판정 | 주행 s | 최고속 m/s | 왕복 최대 ms | 계산 최대 ms | QP 상한 도달 | 경로 오차 평균 / 최대 m (GPS) | AC-10 미판정 랩 | 무효 사유 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| final3_hils_b1 | 5 / 5 | 5 | **불합격** | 190.8 | 24.91 | 32.4 | 21.66 | 0.011 | 0.089 / 2.289 | 0 | - |
| final3_sils_b1 | 5 / 5 | 5 | 비교용 | 186.5 | 24.75 | 11.0 | 0.59 | 0.009 | 0.069 / 1.375 | 0 | - |
| final3_nmpc_b1 | 5 / 5 | 5 | 비교용 | 189.8 | 24.73 | - | - | - | 0.063 / 1.214 | 0 | - |
| final3_sils_b2 | 5 / 6 | 5 | 비교용 | 186.9 | 24.84 | 8.9 | 0.40 | 0.009 | 0.061 / 1.174 | 0 | stalled |
| final3_nmpc_b2 | 5 / 6 | 5 | 비교용 | 183.5 | 24.81 | - | - | - | 0.062 / 1.256 | 0 | sim_gap |
| final3_hils_b2 | 5 / 5 | 5 | 합격 | 184.5 | 24.79 | 33.8 | 21.53 | 0.012 | 0.057 / 0.584 | 0 | - |
| final3_nmpc_b3 | 5 / 6 | 5 | 비교용 | 188.6 | 24.65 | - | - | - | 0.062 / 1.054 | 0 | stalled |
| final3_hils_b3 | 5 / 6 | 5 | **불합격** | 183.6 | 24.77 | 36.7 | 21.53 | 0.011 | 0.068 / 1.683 | 0 | stalled |
| final3_sils_b3 | 5 / 5 | 5 | 비교용 | 184.6 | 24.76 | 6.9 | 0.44 | 0.012 | 0.066 / 1.522 | 0 | - |
| final3_hils_b4 | 5 / 6 | 5 | **불합격** | 184.6 | 24.81 | 36.1 | 21.52 | 0.012 | 0.061 / 1.859 | 0 | stalled |
| final3_sils_b4 | 5 / 5 | 5 | 비교용 | 185.5 | 24.75 | 7.4 | 0.34 | 0.010 | 0.069 / 2.111 | 0 | - |
| final3_nmpc_b4 | 5 / 6 | 5 | 비교용 | 185.3 | 24.77 | - | - | - | 0.065 / 1.944 | 0 | stalled |
| fault_solve_fail_l7curve16b | 1 / 1 | 1 | 합격 | 183.1 | 24.82 | 34.6 | 21.52 | 0.011 | 0.075 / 1.323 | 0 | - |
| fault_solve_fail_l8curve20 | 1 / 1 | 1 | 합격 | 180.8 | 24.79 | 31.8 | 21.52 | 0.011 | 0.080 / 0.942 | 0 | - |
| fault_solve_fail_l2straight12 | 1 / 1 | 1 | 합격 | 190.9 | 19.79 | 32.0 | 21.51 | 0.017 | 0.076 / 1.362 | 0 | - |
| fault_bus_off_slow | 1 / 1 | 1 | 합격 | 185.0 | 24.75 | 31.9 | 21.52 | 0.009 | 0.062 / 0.489 | 0 | - |
| fault_veh_crc_slow | 1 / 1 | 1 | 합격 | 186.1 | 24.74 | 33.5 | 21.51 | 0.008 | 0.062 / 0.342 | 0 | - |
| fault_blackout_fast | 1 / 1 | 1 | 합격 | 194.3 | 20.10 | 33.6 | 21.51 | 0.013 | 0.085 / 2.086 | 0 | - |
| fault_solve_blip_s1curve7 | 1 / 1 | 1 | 합격 | 181.5 | 20.10 | 35.3 | 21.53 | 0.012 | 0.054 / 0.298 | 0 | - |
| fault_solve_fail_l9m6accel18 | 1 / 1 | 1 | 합격 | 190.7 | 20.62 | 31.9 | 21.51 | 0.011 | 0.065 / 0.464 | 0 | - |
| plaus_step10_slow | 1 / 1 | 1 | 비교용 | 184.2 | 24.84 | 32.5 | 21.52 | 0.020 | 0.084 / 1.683 | 0 | - |
| fault_solve_fail_l1curve7 | 1 / 1 | 1 | 합격 | 182.4 | 19.87 | 35.5 | 21.52 | 0.007 | 0.056 / 0.290 | 0 | - |
| fault_solve_blip_s2straight20 | 1 / 1 | 1 | 합격 | 177.9 | 24.82 | 33.1 | 21.53 | 0.011 | 0.064 / 1.590 | 0 | - |
| fault_solve_fail_l3curve10 | 1 / 1 | 1 | 합격 | 184.7 | 24.81 | 32.1 | 21.53 | 0.007 | 0.057 / 0.314 | 0 | - |
| fault_veh_loss_slow | 1 / 1 | 1 | 합격 | 182.4 | 19.97 | 31.6 | 21.52 | 0.011 | 0.073 / 1.227 | 0 | - |
| fault_solve_blip_s3curve10 | 1 / 1 | 1 | 합격 | 175.6 | 24.80 | 39.1 | 21.51 | 0.012 | 0.073 / 1.094 | 0 | - |
| fault_veh_loss_fast | 1 / 1 | 1 | 합격 | 182.1 | 24.88 | 32.2 | 21.51 | 0.012 | 0.072 / 1.052 | 0 | - |
| fault_blackout_slow | 2 / 2 | 2 | 합격 | 185.3 | 24.64 | 33.0 | 21.52 | 0.010 | 0.062 / 0.730 | 0 | - |
| fault_req_crc_slow | 1 / 1 | 1 | **불합격** | 182.5 | 24.69 | 31.7 | 21.57 | 0.011 | 0.079 / 1.134 | 0 | - |
| fault_solve_fail_l10exitcurve15 | 1 / 1 | 1 | 합격 | 187.7 | 24.65 | 35.4 | 21.51 | 0.012 | 0.084 / 1.362 | 0 | - |
| wdt_hang_slow | 1 / 1 | 1 | 비교용 | 195.7 | 19.87 | 31.5 | 21.53 | 0.012 | 0.055 / 0.330 | 0 | - |
| fault_solve_fail_l6straight20 | 1 / 1 | 1 | 합격 | 184.3 | 24.74 | 33.9 | 21.53 | 0.011 | 0.067 / 1.214 | 0 | - |
| fault_veh_crc_fast | 1 / 1 | 1 | 합격 | 176.9 | 24.84 | 34.4 | 21.53 | 0.009 | 0.054 / 0.322 | 0 | - |
| fault_solve_blip_s4curve16b | 1 / 2 | 1 | 합격 | 185.1 | 19.79 | 31.7 | 21.51 | 0.007 | 0.082 / 0.919 | 0 | no_start |
| wdt_hang_fast | 1 / 1 | 1 | 비교용 | 186.8 | 24.80 | 31.6 | 21.51 | 0.010 | 0.056 / 0.367 | 0 | - |
| fault_bus_off_fast | 1 / 1 | 1 | 합격 | 190.2 | 23.74 | 32.3 | 21.52 | 0.009 | 0.059 / 0.448 | 0 | - |
| fault_solve_fail_l4curve16 | 1 / 1 | 1 | 합격 | 192.7 | 24.68 | 32.4 | 21.54 | 0.016 | 0.099 / 1.683 | 0 | - |
| fault_solve_fail_l5curve7b | 1 / 1 | 1 | 합격 | 190.7 | 19.95 | 34.2 | 21.52 | 0.013 | 0.058 / 0.342 | 0 | - |
| fault_req_crc_fast | 1 / 1 | 1 | 합격 | 184.4 | 24.64 | 31.6 | 21.53 | 0.012 | 0.062 / 0.583 | 0 | - |
| plaus_step10_fast | 1 / 1 | 1 | 비교용 | 179.3 | 24.78 | 31.6 | 21.53 | 0.008 | 0.056 / 0.309 | 0 | - |
| final4_hils_c1 | 5 / 6 | 5 | **불합격** | 184.8 | 24.82 | 36.7 | 21.54 | 0.011 | 0.062 / 1.388 | 0 | stalled |
| final4_nmpc_c2 | 5 / 5 | 5 | **불합격** | 182.9 | 24.82 | - | - | - | 0.065 / 1.451 | 0 | - |
| final4_hils_c3 | 5 / 5 | 5 | **불합격** | 185.3 | 24.76 | 34.5 | 21.55 | 0.012 | 0.062 / 1.048 | 0 | - |
| final4_nmpc_c4 | 5 / 5 | 5 | **불합격** | 181.4 | 24.91 | - | - | - | 0.064 / 1.752 | 0 | - |

## 불합격 내역

- `final3_hils_b1` #1: AC-10 error 1.50
- `final3_hils_b1` #2: AC-10 error 2.29
- `final3_hils_b1` #4: AC-10 error 0.93
- `final3_hils_b3` #1: AC-10 error 0.99
- `final3_hils_b3` #4: AC-10 error 1.68
- `final3_hils_b3` #6: AC-10 error 1.11
- `final3_hils_b4` #1: AC-10 error 1.17
- `final3_hils_b4` #6: AC-10 error 1.86
- `fault_req_crc_slow` #1: 충돌
- `final4_hils_c1` #4: AC-03 link
- `final4_hils_c1` #5: AC-10 error 1.39
- `final4_nmpc_c2` #1: AC-01 rtt None, AC-10 error 1.45
- `final4_nmpc_c2` #2: AC-01 rtt None
- `final4_nmpc_c2` #3: AC-01 rtt None, AC-11 step 159.0
- `final4_nmpc_c2` #4: AC-01 rtt None, AC-11 step 153.7, AC-10 error 0.89
- `final4_nmpc_c2` #5: AC-01 rtt None
- `final4_hils_c3` #2: AC-10 error 1.05
- `final4_nmpc_c4` #1: AC-01 rtt None, AC-11 step 154.4, AC-10 error 1.75
- `final4_nmpc_c4` #2: AC-01 rtt None, AC-11 step 157.8
- `final4_nmpc_c4` #3: AC-01 rtt None, AC-11 step 153.4
- `final4_nmpc_c4` #4: AC-01 rtt None, AC-11 step 156.1, AC-10 error 0.93
- `final4_nmpc_c4` #5: AC-01 rtt None, AC-11 step 151.2

## M7 목표 속도별 비교

M7 목표 속도는 계획 단계에서 정해지고 구성과 무관하게 갈린다. 주행 시간과 오차는 같은 속도끼리 비교한다.

| 조건 | M7 목표 | 랩 | 주행 s (평균 ± 표준편차) | 경로 오차 평균 m | 경로 오차 최대 m |
|---|---|---|---|---|---|
| final3_hils_b1 | 약 20 m/s | 1 | 192.5 ± 0.0 | 0.155 | 2.289 |
| final3_hils_b1 | 약 25 m/s | 4 | 190.4 ± 2.4 | 0.073 | 1.501 |
| final3_sils_b1 | 약 20 m/s | 1 | 189.3 ± 0.0 | 0.057 | 0.297 |
| final3_sils_b1 | 약 25 m/s | 4 | 185.7 ± 4.6 | 0.071 | 1.375 |
| final3_nmpc_b1 | 약 20 m/s | 2 | 191.9 ± 1.3 | 0.064 | 1.214 |
| final3_nmpc_b1 | 약 25 m/s | 3 | 188.4 ± 7.0 | 0.063 | 0.560 |
| final3_sils_b2 | 약 20 m/s | 2 | 194.5 ± 0.1 | 0.065 | 1.174 |
| final3_sils_b2 | 약 25 m/s | 3 | 181.8 ± 2.9 | 0.059 | 0.651 |
| final3_nmpc_b2 | 약 20 m/s | 2 | 186.6 ± 0.2 | 0.061 | 1.201 |
| final3_nmpc_b2 | 약 25 m/s | 3 | 181.4 ± 3.8 | 0.063 | 1.256 |
| final3_hils_b2 | 약 20 m/s | 2 | 187.1 ± 0.8 | 0.056 | 0.584 |
| final3_hils_b2 | 약 25 m/s | 3 | 182.8 ± 3.7 | 0.058 | 0.584 |
| final3_nmpc_b3 | 약 20 m/s | 3 | 189.7 ± 1.6 | 0.067 | 1.054 |
| final3_nmpc_b3 | 약 25 m/s | 2 | 186.8 ± 0.8 | 0.055 | 0.302 |
| final3_hils_b3 | 약 20 m/s | 2 | 186.8 ± 1.5 | 0.065 | 1.108 |
| final3_hils_b3 | 약 25 m/s | 3 | 181.5 ± 4.1 | 0.070 | 1.683 |
| final3_sils_b3 | 약 20 m/s | 3 | 187.6 ± 2.6 | 0.074 | 1.522 |
| final3_sils_b3 | 약 25 m/s | 2 | 180.1 ± 3.4 | 0.055 | 0.309 |
| final3_hils_b4 | 약 20 m/s | 3 | 185.6 ± 1.7 | 0.062 | 1.859 |
| final3_hils_b4 | 약 25 m/s | 2 | 183.2 ± 10.3 | 0.061 | 1.174 |
| final3_sils_b4 | 약 20 m/s | 2 | 189.9 ± 6.8 | 0.071 | 2.111 |
| final3_sils_b4 | 약 25 m/s | 3 | 182.6 ± 2.8 | 0.067 | 1.188 |
| final3_nmpc_b4 | 약 20 m/s | 2 | 186.2 ± 2.4 | 0.057 | 0.322 |
| final3_nmpc_b4 | 약 25 m/s | 3 | 184.7 ± 2.2 | 0.071 | 1.944 |
| fault_solve_fail_l7curve16b | 약 25 m/s | 1 | 183.1 ± 0.0 | 0.075 | 1.323 |
| fault_solve_fail_l8curve20 | 약 25 m/s | 1 | 180.8 ± 0.0 | 0.080 | 0.942 |
| fault_solve_fail_l2straight12 | 약 20 m/s | 1 | 190.9 ± 0.0 | 0.076 | 1.362 |
| fault_bus_off_slow | 약 25 m/s | 1 | 185.0 ± 0.0 | 0.062 | 0.489 |
| fault_veh_crc_slow | 약 25 m/s | 1 | 186.1 ± 0.0 | 0.062 | 0.342 |
| fault_blackout_fast | 약 25 m/s | 1 | 194.3 ± 0.0 | 0.085 | 2.086 |
| fault_solve_blip_s1curve7 | 약 20 m/s | 1 | 181.5 ± 0.0 | 0.054 | 0.298 |
| fault_solve_fail_l9m6accel18 | 약 20 m/s | 1 | 190.7 ± 0.0 | 0.065 | 0.464 |
| plaus_step10_slow | 약 25 m/s | 1 | 184.2 ± 0.0 | 0.084 | 1.683 |
| fault_solve_fail_l1curve7 | 약 20 m/s | 1 | 182.4 ± 0.0 | 0.056 | 0.290 |
| fault_solve_blip_s2straight20 | 약 25 m/s | 1 | 177.9 ± 0.0 | 0.064 | 1.590 |
| fault_solve_fail_l3curve10 | 약 25 m/s | 1 | 184.7 ± 0.0 | 0.057 | 0.314 |
| fault_veh_loss_slow | 약 20 m/s | 1 | 182.4 ± 0.0 | 0.073 | 1.227 |
| fault_solve_blip_s3curve10 | 약 25 m/s | 1 | 175.6 ± 0.0 | 0.073 | 1.094 |
| fault_veh_loss_fast | 약 25 m/s | 1 | 182.1 ± 0.0 | 0.072 | 1.052 |
| fault_blackout_slow | 약 20 m/s | 1 | 184.8 ± 0.0 | 0.063 | 0.730 |
| fault_blackout_slow | 약 25 m/s | 1 | 185.8 ± 0.0 | 0.061 | 0.477 |
| fault_req_crc_slow | 약 25 m/s | 1 | 182.5 ± 0.0 | 0.079 | 1.134 |
| fault_solve_fail_l10exitcurve15 | 약 25 m/s | 1 | 187.7 ± 0.0 | 0.084 | 1.362 |
| wdt_hang_slow | 약 20 m/s | 1 | 195.7 ± 0.0 | 0.055 | 0.330 |
| fault_solve_fail_l6straight20 | 약 25 m/s | 1 | 184.3 ± 0.0 | 0.067 | 1.214 |
| fault_veh_crc_fast | 약 25 m/s | 1 | 176.9 ± 0.0 | 0.054 | 0.322 |
| fault_solve_blip_s4curve16b | 약 20 m/s | 1 | 185.1 ± 0.0 | 0.082 | 0.919 |
| wdt_hang_fast | 약 25 m/s | 1 | 186.8 ± 0.0 | 0.056 | 0.367 |
| fault_bus_off_fast | 약 25 m/s | 1 | 190.2 ± 0.0 | 0.059 | 0.448 |
| fault_solve_fail_l4curve16 | 약 25 m/s | 1 | 192.7 ± 0.0 | 0.099 | 1.683 |
| fault_solve_fail_l5curve7b | 약 20 m/s | 1 | 190.7 ± 0.0 | 0.058 | 0.342 |
| fault_req_crc_fast | 약 25 m/s | 1 | 184.4 ± 0.0 | 0.062 | 0.583 |
| plaus_step10_fast | 약 25 m/s | 1 | 179.3 ± 0.0 | 0.056 | 0.309 |
| final4_hils_c1 | 약 25 m/s | 5 | 184.8 ± 3.6 | 0.062 | 1.388 |
| final4_nmpc_c2 | 약 20 m/s | 2 | 187.8 ± 7.1 | 0.064 | 0.893 |
| final4_nmpc_c2 | 약 25 m/s | 3 | 179.7 ± 5.4 | 0.065 | 1.451 |
| final4_hils_c3 | 약 20 m/s | 2 | 187.3 ± 2.9 | 0.066 | 1.048 |
| final4_hils_c3 | 약 25 m/s | 3 | 183.9 ± 5.0 | 0.059 | 0.448 |
| final4_nmpc_c4 | 약 20 m/s | 1 | 187.4 ± 0.0 | 0.062 | 0.932 |
| final4_nmpc_c4 | 약 25 m/s | 4 | 179.9 ± 3.6 | 0.064 | 1.752 |

## 결함별 반응

반응 시간은 주입 뒤 안전 상태가 NORMAL 에서 처음 바뀔 때까지다. 해상도는 약 50 ms 이고, 구간(slow, fast)별 값이다. `bus_off` 는 PC 어댑터도 끊겨 시간이 늦게 잡히므로 참고만 한다.

| 조건 | 랩 | 상태 변화 랩 | 반응 최소 / 평균 / 최대 ms | 최악 상태 (0 INIT, 1 NORMAL, 2 DEGRADED, 3 SAFE_STOP) | 주입 속도 m/s |
|---|---|---|---|---|---|
| fault_solve_fail_l7curve16b | 1 | 1 | 210 / 210 / 210 | 3 | 15.6 |
| fault_solve_fail_l8curve20 | 1 | 1 | 209 / 209 / 209 | 3 | 19.7 |
| fault_solve_fail_l2straight12 | 1 | 1 | 212 / 212 / 212 | 3 | 11.8 |
| fault_bus_off_slow | 1 | 1 | 830 / 830 / 830 | 3 | 12.1 |
| fault_veh_crc_slow | 1 | 0 | - | 1 | 12.4 |
| fault_blackout_fast | 1 | 1 | 185 / 185 / 185 | 3 | 15.7 |
| fault_solve_blip_s1curve7 | 1 | 0 | - | 1 | 7.1 |
| fault_solve_fail_l9m6accel18 | 1 | 1 | 202 / 202 / 202 | 3 | 18.1 |
| fault_solve_fail_l1curve7 | 1 | 1 | 209 / 209 / 209 | 3 | 7.1 |
| fault_solve_blip_s2straight20 | 1 | 0 | - | 1 | 24.7 |
| fault_solve_fail_l3curve10 | 1 | 1 | 227 / 227 / 227 | 3 | 10.1 |
| fault_veh_loss_slow | 1 | 1 | 173 / 173 / 173 | 2 | 12.7 |
| fault_solve_blip_s3curve10 | 1 | 0 | - | 1 | 10.1 |
| fault_veh_loss_fast | 1 | 1 | 181 / 181 / 181 | 2 | 24.7 |
| fault_blackout_slow | 2 | 2 | 186 / 186 / 186 | 3 | 12.3 |
| fault_req_crc_slow | 1 | 0 | - | 1 | 12.3 |
| fault_solve_fail_l10exitcurve15 | 1 | 1 | 211 / 211 / 211 | 3 | 15.6 |
| fault_solve_fail_l6straight20 | 1 | 1 | 208 / 208 / 208 | 3 | 24.7 |
| fault_veh_crc_fast | 1 | 0 | - | 1 | 24.8 |
| fault_solve_blip_s4curve16b | 1 | 0 | - | 1 | 15.6 |
| fault_bus_off_fast | 1 | 1 | 831 / 831 / 831 | 3 | 16.7 |
| fault_solve_fail_l4curve16 | 1 | 1 | 211 / 211 / 211 | 3 | 15.9 |
| fault_solve_fail_l5curve7b | 1 | 1 | 205 / 205 / 205 | 3 | 7.6 |
| fault_req_crc_fast | 1 | 0 | - | 1 | 22.4 |

## SILS와 HILS 동등성

데이터 없음
