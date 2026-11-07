/* SILS 가상 CAN 버스: CAN 프레임 1 개를 UDP 패킷 1 개로 보낸다 (localhost).
 * 패킷 16 바이트: id u32 (LE) | dlc u8 | 0 0 0 | data[8]
 * 가상 MCU(bmpc_sim_mcu)와 PC 브리지가 같은 코드를 쓴다. HILS 에서는 이 계층만 실제 CAN 으로 바뀐다. */
#ifndef BMPC_UDP_H
#define BMPC_UDP_H

#include <netinet/in.h>

#include "bmpc/bmpc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMPC_UDP_PORT_MCU 47001 /* 가상 MCU 가 받는 포트 */
#define BMPC_UDP_PORT_PC 47002  /* PC 브리지가 받는 포트 */

typedef struct {
  int fd;
  struct sockaddr_in peer;
} bmpc_udp_t;

/* 0 = 성공 */
int bmpc_udp_open(bmpc_udp_t *u, uint16_t local_port, const char *peer_host, uint16_t peer_port);
int bmpc_udp_send(bmpc_udp_t *u, const bmpc_can_frame_t *f);
/* 1 = 받음, 0 = 시간 초과, -1 = 오류. timeout_us < 0 이면 무한 대기 */
int bmpc_udp_recv(bmpc_udp_t *u, bmpc_can_frame_t *f, int64_t timeout_us);
void bmpc_udp_close(bmpc_udp_t *u);

#ifdef __cplusplus
}
#endif

#endif /* BMPC_UDP_H */
