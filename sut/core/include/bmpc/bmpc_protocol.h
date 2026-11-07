/* PC <-> MCU CAN 프레임 정의. SILS(UDP 가상 버스)와 HILS(실제 CAN)가 같은 프레임을 쓴다.
 *
 * 요청 (PC -> MCU, 매 주기 6프레임, 0x105 를 받으면 계산 시작)
 *   0x100 seq u16 | flags u8 | ref_type u8 | c0 f32
 *   0x101 c1 f32 | c2 f32
 *   0x102 c3 f32 | x_max u16 (0.01 m) | path_counter u16
 *   0x103 ref_v | v_lb | v_ub | post_cap            u16 x4 (0.01 m/s, post_cap 0xFFFF = 없음)
 *   0x104 snap u16 (0xFFFF = 없음) | velocity u16 | yaw_rate i16 (1e-4 rad/s) | lf_est u16 (1e-3 m, 0 = 없음)
 *   0x105 period | odom_age | transport              u16 x3 (1e-5 s) | crc16
 *
 * 응답 (MCU -> PC, 매 주기 4프레임)
 *   0x1E0 seq u16 | status u8 | iters u8 | steer_cmd f32 (rad, MORAI 부호)
 *   0x1E1 accel u16 (1e-4) | brake u16 (1e-4) | delta0 i16 (1e-4 rad, MPC 부호) | vt_final u16 (0.01 m/s)
 *   0x1E2 state_delay | compute | actuation          u16 x3 (1e-5 s) | vt_raw u16 (0.01 m/s)
 *   0x1E3 solve_us u16 | total_us u16 | lf_used u16 (1e-3 m) | crc16
 *
 * 설정 (시작할 때 한 번)
 *   0x0F0 idx u16 | 0 u16 | value f32           필드마다 한 프레임
 *   0x0F1 hash u32 | count u16 | 0 u16          보낸 값의 해시
 *   0x1E8 hash u32 | count u16 | status u8 | 0   MCU 가 받은 값의 해시 (MCU -> PC)
 *
 * 바이트 순서는 little-endian. CRC 는 CRC-16/CCITT-FALSE 로, 해당 묶음의 모든 바이트에서
 * 마지막 프레임의 crc 2바이트만 뺀 범위에 건다.
 */
#ifndef BMPC_PROTOCOL_H
#define BMPC_PROTOCOL_H

#include "bmpc/bmpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ID 배치: PC -> MCU 는 모두 MCU -> PC 보다 작다 (우선순위가 높다). CH340 USB-CAN 은 자기가 보낸 프레임이
 * 중재에서 지면 약 100 ms 멈추기 때문이다 (팀 capstone_hils 실측). 안전 제어기 통합 시 PC 가 보내는
 * 차량 CAN 0x1A0..0x1D0 보다도 응답(0x1E0..)·ACK(0x1E8)·SAFE_OUT(0x200..) 이 위에 온다. */
#define BMPC_ID_CFG_SET 0x0F0u
#define BMPC_ID_CFG_COMMIT 0x0F1u
#define BMPC_ID_CFG_ACK 0x1E8u
#define BMPC_ID_REQ_BASE 0x100u
#define BMPC_REQ_FRAMES 6
#define BMPC_ID_RESP_BASE 0x1E0u
#define BMPC_RESP_FRAMES 4

/* 요청 flags */
#define BMPC_FLAG_RESET 0x01u     /* warm start 를 버린다 */
#define BMPC_FLAG_HIGHWAY 0x02u   /* 60 kph 예외 구간: 고속 가중치, 가속 상한, 속도 가드 해제 */
#define BMPC_FLAG_AVOIDANCE 0x04u /* 회피 속도 명령이 살아 있음: 회피 가중치 */
#define BMPC_FLAG_KICK 0x08u      /* 정적 장애물 감속 중: 킥 브레이크 상한 */
#define BMPC_FLAG_STOP 0x10u      /* MPC 를 풀지 않고 목표 0 으로 후처리만 (경로 끝, ACC 0) */

#define BMPC_REF_CUBIC 0u

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} bmpc_can_frame_t;

typedef struct {
  uint16_t seq;
  uint8_t flags;
  uint8_t ref_type;
  bmpc_real c[4];       /* 차량 좌표계 3차식 y = c0 + c1 x + c2 x^2 + c3 x^3 */
  bmpc_real x_max;      /* fit 에 쓴 점의 최대 x (m) */
  uint16_t path_counter;
  bmpc_real ref_v;      /* m/s */
  bmpc_real v_lb, v_ub; /* 목표 속도 하한·상한 (m/s) */
  bmpc_real post_cap;   /* 후처리 속도 상한, < 0 = 없음 */
  bmpc_real snap;       /* 이번 주기에 바로 맞출 목표 속도, < 0 = 없음 */
  bmpc_real velocity;   /* 실제 속도 (m/s) */
  bmpc_real yaw_rate;   /* EKF yaw rate (rad/s, 반시계 +) */
  bmpc_real lf_est;     /* 실효 Lf 추정, <= 0 = 없음 */
  bmpc_real period;     /* 센서 주기 T (s) */
  bmpc_real odom_age;   /* 보낼 때의 상태 나이 (s) */
  bmpc_real transport;  /* 직전 주기 CAN 왕복 - MCU 처리 (s) */
} bmpc_request_t;

typedef struct {
  uint16_t seq;
  uint8_t status;
  uint8_t iters;
  bmpc_real steer_cmd; /* 최종 조향 (rad, MORAI 부호) */
  bmpc_real accel;     /* 0..1 */
  bmpc_real brake;     /* 0..1 */
  bmpc_real delta0;    /* MPC 원출력 조향 (rad, MPC 부호) */
  bmpc_real vt_final;  /* 후처리 뒤 목표 속도 (m/s) */
  bmpc_real vt_raw;    /* MPC 원출력 목표 속도 (m/s) */
  bmpc_real state_delay, compute, actuation; /* 지연보상에 쓴 값 (s) */
  bmpc_real lf_used;
  uint32_t solve_us; /* QP 풀이 시간 */
  uint32_t total_us; /* 요청 수신 완료 ~ 응답 송신 직전 */
} bmpc_response_t;

uint16_t bmpc_crc16(const uint8_t *data, size_t len, uint16_t crc);

void bmpc_encode_request(const bmpc_request_t *req, bmpc_can_frame_t out[BMPC_REQ_FRAMES]);
/* 0 = 정상, -1 = ID/DLC 불일치, -2 = CRC 불일치 (seq 는 읽을 수 있으면 채운다) */
int bmpc_decode_request(const bmpc_can_frame_t in[BMPC_REQ_FRAMES], bmpc_request_t *req);

void bmpc_encode_response(const bmpc_response_t *resp, bmpc_can_frame_t out[BMPC_RESP_FRAMES]);
int bmpc_decode_response(const bmpc_can_frame_t in[BMPC_RESP_FRAMES], bmpc_response_t *resp);

void bmpc_encode_cfg_set(uint16_t idx, float value, bmpc_can_frame_t *out);
void bmpc_decode_cfg_set(const bmpc_can_frame_t *in, uint16_t *idx, float *value);
void bmpc_encode_cfg_commit(uint32_t hash, uint16_t count, bmpc_can_frame_t *out);
void bmpc_decode_cfg_commit(const bmpc_can_frame_t *in, uint32_t *hash, uint16_t *count);
void bmpc_encode_cfg_ack(uint32_t hash, uint16_t count, uint8_t status, bmpc_can_frame_t *out);
void bmpc_decode_cfg_ack(const bmpc_can_frame_t *in, uint32_t *hash, uint16_t *count, uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_PROTOCOL_H */
