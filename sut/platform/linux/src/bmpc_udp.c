#define _POSIX_C_SOURCE 200809L
#include "bmpc/bmpc_udp.h"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define PKT_LEN 16

int bmpc_udp_open(bmpc_udp_t *u, uint16_t local_port, const char *peer_host, uint16_t peer_port)
{
  struct sockaddr_in local;
  int one = 1;
  memset(u, 0, sizeof(*u));
  u->fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (u->fd < 0) return -1;
  setsockopt(u->fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  memset(&local, 0, sizeof(local));
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons(local_port);
  if (bind(u->fd, (struct sockaddr *)&local, sizeof(local)) != 0) {
    close(u->fd);
    u->fd = -1;
    return -1;
  }
  u->peer.sin_family = AF_INET;
  u->peer.sin_port = htons(peer_port);
  if (inet_pton(AF_INET, peer_host, &u->peer.sin_addr) != 1) {
    close(u->fd);
    u->fd = -1;
    return -1;
  }
  return 0;
}

int bmpc_udp_send(bmpc_udp_t *u, const bmpc_can_frame_t *f)
{
  unsigned char pkt[PKT_LEN];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = (unsigned char)(f->id & 0xFFu);
  pkt[1] = (unsigned char)((f->id >> 8) & 0xFFu);
  pkt[2] = (unsigned char)((f->id >> 16) & 0xFFu);
  pkt[3] = (unsigned char)(f->id >> 24);
  pkt[4] = f->dlc;
  memcpy(&pkt[8], f->data, 8);
  return sendto(u->fd, pkt, sizeof(pkt), 0, (const struct sockaddr *)&u->peer, sizeof(u->peer)) == PKT_LEN ? 0 : -1;
}

int bmpc_udp_recv(bmpc_udp_t *u, bmpc_can_frame_t *f, int64_t timeout_us)
{
  unsigned char pkt[64];
  struct pollfd pfd;
  ssize_t n;
  int timeout_ms = timeout_us < 0 ? -1 : (int)((timeout_us + 999) / 1000);
  pfd.fd = u->fd;
  pfd.events = POLLIN;
  pfd.revents = 0;
  for (;;) {
    const int r = poll(&pfd, 1, timeout_ms);
    if (r < 0 && errno == EINTR) return 0;
    if (r < 0) return -1;
    if (r == 0) return 0;
    break;
  }
  n = recv(u->fd, pkt, sizeof(pkt), 0);
  if (n != PKT_LEN) return n < 0 ? -1 : 0;
  f->id = (uint32_t)pkt[0] | ((uint32_t)pkt[1] << 8) | ((uint32_t)pkt[2] << 16) | ((uint32_t)pkt[3] << 24);
  f->dlc = pkt[4];
  memcpy(f->data, &pkt[8], 8);
  return 1;
}

void bmpc_udp_close(bmpc_udp_t *u)
{
  if (u->fd >= 0) close(u->fd);
  u->fd = -1;
}
