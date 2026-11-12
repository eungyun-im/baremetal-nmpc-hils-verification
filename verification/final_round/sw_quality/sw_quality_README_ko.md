# 최종 소스 품질 측정 (G3, 2026-11-11)

대상: catkin 최종 소스(`final_solvefail_20261111` 펌웨어와 같은 코어·통합·안전 소스). 사본 `sut/`(md5 목록 `source_manifest.md5`), 스크립트는 공개용 저장소(`~/portfolio/hils-verification/sut`)의 것을 복사해 `test_solver_fail` 을 더했다. 공개 저장소는 바꾸지 않았다.

| 항목 | 결과 | 파일 |
|---|---|---|
| 단위·통합 시험 | 실행 파일 15개, 검사 약 28만 개, 실패 0 | `unit_tests.txt` |
| sanitizer (ASan, UBSan) | 14개 모두 오류 없음 | `coverage.txt` |
| 구조 커버리지 (core + integ + 안전 제어기) | 줄 97.0 % (1,500/1,547), 함수 97.8 % (133/136), 분기 84.8 % (880/1,038). MC/DC 는 판정 뒤 clang 18 로 측정: 합산 76.4 % (120/157), 안전 제어기 `safety_ctrl.c` 47.1 % (`mcdc/README.md`) | `coverage.txt` |
| 정적 분석 (cppcheck 2.13 MISRA addon, clang-tidy 18) | 402건, 기준선 대비 새 지적 2건(DEF-37, 시험 훅 코드), 해결 0 | `static.txt`, `sut/build_static/` |
| 정적 스택 상한 | 2,236 B / 8,192 B (여유 72.7 %), 재귀·가변 프레임 없음. 이전 빌드 2,204 B | `stack/bound.txt` |
| Ipopt 비교 (TC-U01) | `test_vs_ipopt`, `test_track_vs_ipopt` double·float 4종 PASS | `../host_runs/*vs_ipopt_*.txt` |
| 결함 주입 캠페인 (TC-C01) | 10,000회, 실패 0 | `../host_runs/campaign.txt`, `fault_campaign.csv` |
| PIL 차이 원인 | FMA 축약이 주원인 중 하나로 확인, 나머지 미확인 (DEF-36) | `../pil_fma/README.md` |

이전 측정(2026-11-05~10)과 비교: 줄 95.2~99.1 %, 분기 71.4~84.8 %는 파일별 범위였고, 이번 값은 전체 합산이다.
