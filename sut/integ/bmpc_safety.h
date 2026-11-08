/* MPC(bmpc_app) + 안전 제어기(third_party/kuuve_safety) 통합 응용 계층. 한 MCU 에서 MPC 가 계산한 명령을
 * 안전 제어기가 검사해 차량 명령(SAFE_OUT)으로 낸다. 플랫폼과 무관한 C99 (펌웨어, 호스트 시험 공용).
 *
 *   PC -> MCU   0x0F0/0x0F1/0x100..0x105  MPC 설정·요청            -> bmpc_app (메인 문맥)
 *               0x1A0 ESC_SPEED, 0x1B0 SAS_STEER, 0x1C0 VCU_PEDAL, 0x1D0 ACU_IMU, 0x7F0 BENCH_RESET -> 안전 제어기
 *   MCU 내부    MPC 응답(0x1E0..0x1E3) 을 가로채 CTRL_CMD (조향 deg, 가감속 m/s2, Alive = seq 하위 4 비트) 로 바꿔 넣는다
 *   MCU -> PC   0x1E0..0x1E3 MPC 응답 (진단·지연 측정), 0x200 SAFE_OUT, 0x201 SAFE_STAT (10 ms, 차량에 적용할 값)
 *
 * 페달 <-> 가감속: a = accel * accel_gain - brake * brake_gain (팀 안전 브리지 규칙). PC 는 SAFE_OUT 을 같은 gain 으로
 * 페달로 되돌린다. 안전 제어기가 값을 바꾸지 않으면 MPC 명령이 그대로 차량에 간다.
 * SAFE_STAT.CtrlAliveEcho 가 요청 seq 의 하위 4 비트와 같으면 그 SAFE_OUT 에 이번 MPC 명령이 반영된 것이다.
 *
 * 지연보상: 안전 제어기가 조향을 바꾸면 (변화율·횡가속도 제한) 실제로 나간 조향을 MPC 의 명령 이력에 덧붙인다
 * (bmpc_controller_note_applied). 같은 MCU 라서 가능한 일이다.
 *
 * 실행 문맥 (펌웨어): bs_on_safety_frame = CAN 수신 인터럽트, bs_safety_tick = 10 ms 타이머 인터럽트 (둘은 같은 우선순위),
 * bs_on_mpc_frame / bs_feed_ctrl / bs_note_applied = 메인 루프 (bs_feed_ctrl 은 인터럽트를 막고 부른다). */
#ifndef BMPC_SAFETY_H
#define BMPC_SAFETY_H

#include <stdint.h>

#include "bmpc/bmpc_app.h"
#include "safety_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BS_ID_ESC_SPEED 0x1A0u
#define BS_ID_SAS_STEER 0x1B0u
#define BS_ID_VCU_PEDAL 0x1C0u
#define BS_ID_ACU_IMU 0x1D0u
#define BS_ID_SAFE_OUT 0x200u
#define BS_ID_SAFE_STAT 0x201u
#define BS_ID_BENCH_RESET 0x7F0u
/* BENCH_RESET (0x7F0) is test-bench only (hils.dbc: "must be excluded from production builds").
 * It is accepted only when BS_BENCH_RESET is defined (build.sh BENCH=1). The fault hook rides on the same
 * frame, so BS_TEST_HOOK implies it. A release build ignores 0x7F0 entirely. */
#if defined(BS_TEST_HOOK) && !defined(BS_BENCH_RESET)
#define BS_BENCH_RESET
#endif
#define BS_PERIOD_MS 10u

#ifdef BS_TEST_HOOK
/* 시험대 전용 (양산 빌드 제외): 안전 제어기에 들어가기 전의 조향 명령에 결함을 넣는다.
 * 0x7F0, DLC 6: 0xFA | 종류 | 조향 deg x 100 (i16 LE) | 시간 ms (u16 LE).
 * 시간은 다음 명령부터 잰다. 50 ms 면 명령 한 번만 바뀐다. */
#define BS_HOOK_MAGIC 0xFAu
#define BS_HOOK_OFF 0u
#define BS_HOOK_OFFSET 1u /* 조향에 값을 더한다 */
#define BS_HOOK_HOLD 2u   /* 결함 직전 조향으로 고정한다 */
#endif

#ifdef BS_PLAUS
/* 조향 명령 타당성 검사 (팀 안전 제어기 앞단, 통합 계층). 한 명령 사이 변화가 상한을 넘으면 직전 조향을 유지하고 감속한다.
 * 상한은 후처리의 한 주기 상한 3.713 deg 와 측정 최대 3.686 deg 위. exp_20261105 final_analysis 9 장 */
#define BS_PLAUS_STEP_DEG 5.0f
#define BS_PLAUS_HOLD_US 1000000 /* 시험한 결함 길이(최대 1 s)를 덮는다 */
#define BS_PLAUS_BRAKE 0.4f      /* 브레이크 0.4 = 2 m/s2 (brake_gain 5) */
#define BS_PLAUS_RAMP_DEG 3.7f   /* 유지가 끝난 뒤 명령 쪽으로 돌아가는 한 주기 변화 */
#define BS_PLAUS_GAP_US 200000   /* 명령이 이만큼 끊기면 다시 기준을 잡는다 (안전 제어기 명령 시간 초과와 같다) */
#define BS_PLAUS_OK_CMDS 2       /* 유지 중 이만큼 연속으로 정상이면 짧은 결함으로 보고 일찍 푼다 */
#endif

typedef struct {
  float accel_gain; /* 가속 페달 1.0 = 이 가속도 [m/s2] (PC 브리지와 같아야 한다) */
  float brake_gain; /* 브레이크 1.0 = 이 감속도 [m/s2] */
} bs_cfg_t;

typedef struct {
  bmpc_app_t app;
  sc_t sc;
  bs_cfg_t cfg;
  bmpc_hal_t hal;                          /* 플랫폼 HAL (CAN 송신은 인터럽트에서도 불린다) */
  bmpc_can_frame_t resp[BMPC_RESP_FRAMES]; /* 가로챈 MPC 응답 */
  uint8_t resp_mask;
  uint8_t ctrl[8];                         /* 안전 제어기에 넣을 CTRL_CMD */
  int ctrl_pending;
  sc_out_t out;                            /* 마지막 sc_step 결과 */
  uint8_t out_alive;
  uint8_t stat_alive;
  volatile int applied_pending;            /* tick 이 쓰고 메인 루프가 읽는다 */
  volatile float applied_steer_deg;
#ifdef BS_TEST_HOOK
  volatile int hook_pending; /* 수신 인터럽트가 쓰고 메인 문맥이 읽는다 */
  volatile uint8_t hook_kind;
  volatile int16_t hook_centideg;
  volatile uint16_t hook_ms;
  uint8_t hook_active;
  int64_t hook_until_us;
  float hook_last_steer; /* 결함 넣기 전 조향 [rad] */
  uint32_t n_hook;
#endif
#ifdef BS_PLAUS
  int plaus_have;      /* 직전 조향이 있다 */
  int plaus_returning; /* 유지 뒤 명령으로 돌아가는 중 */
  volatile int plaus_reset_req; /* 수신 인터럽트가 쓰고 메인 문맥이 지운다 */
  int plaus_ok;        /* 유지 중 연속 정상 명령 수 */
  int64_t plaus_last_us; /* 직전 명령 시각 */
  float plaus_last;    /* 내보낸 직전 조향 [rad] */
  int64_t plaus_until_us;
  uint32_t n_plaus;    /* 검출 횟수 */
#endif
  uint32_t n_ctrl;
  uint32_t n_safety_rx;
  uint32_t n_resets;
  uint32_t n_steps;
  uint32_t n_notes; /* 안전 출력을 이력에 넣은 횟수 */
} bs_app_t;

void bs_default_cfg(bs_cfg_t *cfg);
/* 팀 sc_default_config 에서 MORAI 1020 코스의 운영 영역(명령 기준 최대값 + 여유)으로 바꾼 값 */
void bs_default_safety_config(sc_config_t *scfg);
void bs_init(bs_app_t *a, const bs_cfg_t *cfg, const sc_config_t *scfg, const bmpc_hal_t *hal);

int bs_is_safety_frame(uint32_t id);
void bs_on_safety_frame(bs_app_t *a, const bmpc_can_frame_t *f, uint32_t now_ms);
/* MPC 설정·요청 프레임. 응답이 나가면 ctrl_pending 이 선다 */
void bs_on_mpc_frame(bs_app_t *a, const bmpc_can_frame_t *f);
/* 새 CTRL_CMD 가 있으면 안전 제어기에 넣고 1 */
int bs_feed_ctrl(bs_app_t *a, uint32_t now_ms);
/* 10 ms 마다: sc_step -> SAFE_OUT, SAFE_STAT 송신 */
void bs_safety_tick(bs_app_t *a, uint32_t now_ms);
/* 안전 제어기 출력 조향을 MPC 명령 이력에 반영 (메인 문맥) */
void bs_note_applied(bs_app_t *a, int64_t now_us);
void bs_on_bus_off(bs_app_t *a);

/* MPC 응답 -> CTRL_CMD 8 바이트 (hils.dbc, E2E CRC 포함) */
void bs_make_ctrl_frame(const bs_cfg_t *cfg, const bmpc_response_t *r, uint8_t d[8]);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_SAFETY_H */
