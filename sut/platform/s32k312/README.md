# platform/s32k312

HILS용 S32K312EVB-Q172 펌웨어입니다. SILS 가상 MCU(`platform/linux`)와 같은 앱 코드(`core/`, `bmpc_app`)를 그대로 빌드하고, HAL 두 개만 보드용입니다.

| HAL | SILS (`platform/linux`) | HILS (여기) |
|---|---|---|
| `now_us()` | `clock_gettime` | DWT 사이클 카운터 누적. 코어 클럭은 부팅 때 FlexCAN 타이머로 잽니다 |
| `can_send()` | UDP 패킷 1개 | FlexCAN0 송신 메일박스 (ID마다 하나) |
| 수신 | `recv` 루프 | 수신 인터럽트 → 링 버퍼(128) → 메인 루프에서 `bmpc_app_on_frame` |

핀, FlexCAN, 메시지 RAM ECC 초기화, 스타트업은 팀 `capstone_hils`의 `fw_mpc_s32k312`(실보드 검증본)에서 가져왔습니다.

## 파일

| 파일 | 내용 |
|---|---|
| `main.c` | 하드웨어 설정, PLL 120 MHz, HAL, 수신 인터럽트, 메인 루프, LED, 하트비트, (선택) 안전 제어기 주기, 구간별 시간 |
| `startup.S` | 부트 헤더, 벡터 테이블(IRQ 110 = FlexCAN0 MB 0~31, SysTick), SRAM·DTCM ECC 초기화 |
| `linker.ld` | flash = 코드, DTCM = .data/.bss(작업 메모리 포함)/스택 8 KB, SRAM = RAM 실행 함수(플래시 대기 상태) |
| `build.sh` | 빌드 (`-DBMPC_USE_FLOAT` 필수, `-Wdouble-promotion -Werror`, 소프트 double 링크 검사) |
| `flash.sh` | P&E(OpenSDA) GDB 서버로 플래시 → 보드 플래시를 ELF와 비교 → 리셋 |

## 코어 클럭 120 MHz

FXOSC 16 MHz → PLL VCO 960 MHz → /2 → PHI0 /4 = 120 MHz. 주변 분주는 AIPS_PLAT /2, AIPS_SLOW /4, HSE /2, DCM /4이고, 플래시 읽기 대기는 3입니다.
순서와 값은 NXP RTD `Clock_Ip`(Zephyr `hal_nxp` s32k3)를 따랐습니다.

- 팀 `capstone_hils`의 시도는 PLL 레지스터를 건드리는 순간 칩이 리셋됐습니다. 원인은 그 전에 MC_ME에서 FXOSC(REQ53)·PLL(REQ56) 블록 클럭을 켜지 않은 것이었습니다.
- 플래시 대기 상태는 플래시에서 명령을 가져오는 중에 바꾸지 않습니다. RTD와 같이 SRAM에서 실행하는 함수(`.ramfunc`)로 바꿉니다.
- 부팅 직후 1초 기다린 뒤 PLL을 켭니다. 설정이 잘못돼 리셋이 반복되더라도 디버거가 붙어 다시 플래시할 수 있게 하기 위해서입니다.
- 어느 단계든 시간 안에 안 끝나면 48 MHz(FIRC)로 남습니다. 하트비트 6번째 바이트에 실제 코어 MHz가 나옵니다. FlexCAN은 FIRC를 그대로 써서 비트 타이밍이 바뀌지 않습니다.

## CAN

500 kbps, 보드 J32. 프로토콜은 SILS와 같습니다(`core/include/bmpc/bmpc_protocol.h`). PC → MCU ID는 모두 MCU → PC보다 작습니다(CH340 중재 문제).

| 방향 | ID | 메일박스 |
|---|---|---|
| PC → MCU | 0x0F0 설정, 0x0F1 설정 확정, 0x100~0x105 요청 | 수신 0~7 |
| PC → MCU | 0x1A0 ESC, 0x1B0 조향각, 0x1C0 페달, 0x1D0 IMU, 0x7F0 BENCH_RESET (안전 제어기 빌드만 사용) | 수신 8~12 |
| MCU → PC | 0x1E0~0x1E3 응답, 0x1E8 설정 ACK | 송신 16~20 |
| MCU → PC | 0x200 SAFE_OUT, 0x201 SAFE_STAT (안전 제어기 빌드, 10 ms) | 송신 24, 25 |
| MCU → PC | 0x321 하트비트 `'B' 'M'` 500 ms, 0x3F0~0x3F3 구간별 시간 (`PROFILE=1`) | 송신 21~23 |

## 실시간 성능 (120 MHz, float)

| 항목 | 값 |
|---|---|
| 작업 메모리 | 50.6 KB, 대기 없는 DTCM (64 KB 중 52.8 KB + 스택 8 KB) |
| SQP 1회 (경로 위, QP 1회) | 6.7 ms |
| 악조건 6종 최악 (SQP 2, QP 상한 16) | 22.2 ms 관측, 분석 상한 약 29 ms (SQP당 고정 6.2 ms + QP 반복당 0.51 ms) |
| MORAI 1020 한 랩 (SQP 2) | 계산 평균 13 / 최대 21.5 ms, 왕복 최대 30~33 ms, 링크 실패 0 |

48 MHz 첫 빌드(작업 메모리 66 KB가 SRAM)는 SQP 1회에 약 35 ms, 최대 110 ms였습니다. 줄인 방법은 다음과 같습니다.
- 민감도를 시간 순서로 전파하고 비용 항에 쓰는 세 상태만 열별로 저장해 DTCM에 들어가게 했습니다.
- 내적을 누산기 4개로 나눴고, 비용 행렬은 레지스터 내적으로 원소마다 한 번만 씁니다.
- QP는 Cholesky를 반복마다 새로 분해하지 않고 갱신합니다(rank-1 / 행 추가).
- PLL 120 MHz로 올렸습니다.
- `-ffreestanding`을 빼서 `fabsf`/`sqrtf`가 FPU 명령 하나로 들어가게 했습니다.

## 빌드와 플래시

```
ARM_GCC_BIN=~/tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin bash build.sh   # build/fw_bmpc.elf
SAFETY=1 ... bash build.sh      # 팀 안전 제어기 통합 (integ/bmpc_safety + third_party/kuuve_safety)
PROFILE=1 ... bash build.sh     # 구간별 시간 0x3F0~0x3F3 (부팅 때 캐시 상태·내적 벤치 포함)
PLL=0 ... bash build.sh         # 48 MHz (비교용)
bash flash.sh                   # P&E GDB 서버 (PNE_DIR, 기본 ~/tools/pemicro/lin) + gdb load + compare-sections + reset
```

- 툴체인은 `ARM_GCC_BIN`, S32DS(`~/NXP/S32DS.3.6.*`), PATH의 `arm-none-eabi-gcc` 순서로 찾습니다.
- P&E 서버는 이클립스 플러그인 `com.pemicro.debug.gdbjtag.pne_6.2.7.202610202056.jar`의 `lin/`과 `supportFiles_ARM/`을 같은 폴더에 풀어 둡니다.
- Linux에서는 P&E 장치(USB 1357:0089)에 쓰기 권한이 필요합니다(udev 규칙).
- 링크 뒤 `__aeabi_d*`(소프트웨어 double)가 있으면 경고합니다. 이 FPU는 단정밀도만 있습니다.
- SILS 가상 MCU는 double, 보드는 float입니다. float와 IPOPT의 차이는 호스트 시험에서 조향 0.0014° 이하였습니다.

## 진단 (디버거)

DTCM에 있어 데이터 캐시와 무관하게 읽힙니다. 다만 디버거를 붙이면 코어가 멈추므로 CAN 측정 중에는 붙이지 않습니다.

`g_dbg_cpm`(코어 사이클/ms), `g_dbg_pll`(0x7F 성공, 0x8x 실패 단계), `g_dbg_dcache`, `g_dbg_requests`, `g_dbg_max_solve_us`, `g_dbg_max_total_us`, `g_dbg_busoff`, `g_dbg_tx_overrun`, `g_dbg_id_mismatch`
