/* MCU 펌웨어 앱 계층. 플랫폼과 무관한 C 코드라 SILS 가상 MCU 와 S32K312 에 똑같이 들어간다.
 *
 * 플랫폼(HAL)이 주는 것은 두 가지뿐이다.
 *   now_us()   단조 증가 시각 (us)          SILS: clock_gettime / MCU: 타이머
 *   can_send() CAN 프레임 1 개 송신          SILS: UDP 패킷 / MCU: FlexCAN
 * 수신은 플랫폼이 프레임을 받을 때마다 bmpc_app_on_frame() 을 부른다.
 *
 * 동작
 *   0x0F0 설정 프레임  : staging 설정에 값을 넣는다
 *   0x0F1 설정 확정    : 해시·개수가 맞으면 controller 를 그 설정으로 초기화하고 ACK(0x1E8)
 *   0x100..0x105 요청  : 6 개가 모이면(0x105 가 트리거) controller 한 주기를 돌려 응답 4 프레임을 보낸다
 *   프레임 누락·CRC 오류면 status=RX_ERROR 응답을 보낸다. 설정 전이면 NOT_CONFIGURED.
 */
#ifndef BMPC_APP_H
#define BMPC_APP_H

#include "bmpc/bmpc_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int64_t (*now_us)(void *ctx);
  int (*can_send)(void *ctx, const bmpc_can_frame_t *f); /* 0 = 성공 */
  void *ctx;
} bmpc_hal_t;

typedef struct {
  uint32_t requests;       /* 처리한 요청 */
  uint32_t rx_errors;      /* 누락·CRC 오류 */
  uint32_t solver_errors;  /* status 가 OK 가 아닌 응답 */
  uint32_t configs;        /* 성공한 설정 */
  uint32_t last_total_us;
  uint32_t max_total_us;
  uint32_t last_solve_us;
  uint32_t max_solve_us;
} bmpc_app_stats_t;

typedef struct {
  bmpc_hal_t hal;
  bmpc_controller_t ctrl;
  bmpc_config_t staging;
  uint8_t cfg_seen[BMPC_CONFIG_COUNT];
  uint16_t cfg_count;
  bmpc_can_frame_t req[BMPC_REQ_FRAMES];
  uint8_t req_mask;
  bmpc_app_stats_t stats;
} bmpc_app_t;

void bmpc_app_init(bmpc_app_t *app, const bmpc_hal_t *hal);
void bmpc_app_on_frame(bmpc_app_t *app, const bmpc_can_frame_t *f);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_APP_H */
