# MC/DC 커버리지 (2026-11-12, 코드 변경 없음)

- 도구: Ubuntu 24.04 컨테이너의 clang 18 / llvm-cov 18 (`-fcoverage-mcdc`). gcc 9 와 clang 10 은 MC/DC 를 지원하지 않는다.
- 대상·시험: `quality.sh` 와 같은 소스 11개(core, integ, 안전 제어기)와 같은 시험 14가지 빌드. 스크립트 `mcdc_inner.sh`, 실행 로그 `run.log`, 전체 보고 `report_all.txt`.
- 모든 시험 실행 종료 코드 0.

| 지표 (14개 빌드 합산) | 결과 |
|---|---|
| 줄 | 96.0 % (1,852/1,929) |
| 함수 | 97.8 % (133/136) |
| 분기 | 81.8 % (1,031/1,260) |
| **MC/DC 조건** | **76.4 % (120/157)** |

- gcov 측정(줄 97.0 %, 분기 84.8 %)과 숫자가 조금 다른 것은 도구의 집계 단위 차이다(clang 은 매크로·인라인 영역을 따로 센다).
- 매크로가 달라 매핑이 다른 함수 경고(mismatched): 0건.
- 덮지 못한 MC/DC 조건 37개는 `report_all.txt` 에 파일별로 있다. 판정 기준이 없던 항목이라 보고만 한다.
- **파일별로 크게 다르다.** 합산값만 보면 안 된다.

| 파일 | MC/DC | 분기 |
|---|---|---|
| `third_party/kuuve_safety/safety_ctrl.c` (안전 제어기) | **47.1 % (16/34)** | 72.1 % |
| `core/src/bmpc_protocol.c` | 37.5 % (3/8) | 82.0 % |
| `core/src/bmpc_delay.c` | 50.0 % (5/10) | 80.0 % |
| `core/src/bmpc_mpc.c` | 62.5 % (5/8) | 94.3 % |
| `core/src/bmpc_controller.c` | 88.2 % (15/17) | 96.3 % |
| `core/src/bmpc_qp.c` | 89.3 % (25/28) | 96.8 % |
| `integ/bmpc_safety.c` | 96.7 % (29/30) | 98.2 % |
| `bmpc_post.c`, `bmpc_app.c`, `bmpc_config.c` | 100 % (13/13, 7/7, 2/2) | 100 / 100 / 66.6 % |

- 대상은 core·integ·안전 제어기 11개 파일이다. 플랫폼 코드(`main.c`, 인터럽트, CAN 드라이버, 워치독)는 범위 밖이다.
