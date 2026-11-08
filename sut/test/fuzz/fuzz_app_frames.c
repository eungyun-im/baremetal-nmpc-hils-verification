/* libFuzzer 하네스: MCU 앱이 받는 CAN 프레임 열을 무작위로 바꿔 넣는다.
 * 입력 = 13 B 레코드 반복: uint32 id | uint8 dlc | uint8 data[8] (리틀엔디언).
 * 매 입력마다 앱을 새로 초기화하고, 시각은 프레임마다 10 ms 씩 늘린다.
 * 찾는 것: 메모리 오류(ASan), 정의되지 않은 동작(UBSan), 멈춤(-timeout), 응답 형식 위반(assert 대신 abort).
 * 빌드: clang -g -O1 -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all -Icore/include (core/src 의 .c 전부) test/fuzz/fuzz_app_frames.c -lm */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bmpc/bmpc_app.h"

static int64_t g_t;
static uint32_t g_sent;

static int64_t hal_now(void *ctx) { (void)ctx; return g_t; }

static int hal_send(void *ctx, const bmpc_can_frame_t *f)
{
  (void)ctx;
  /* 앱이 내는 프레임은 응답(0x1E0..0x1E3)이나 설정 ACK(0x1E8)뿐이고 DLC 는 8 이어야 한다 */
  if (!((f->id >= 0x1E0u && f->id <= 0x1E3u) || f->id == 0x1E8u)) abort();
  if (f->dlc != 8u) abort();
  g_sent++;
  return 0;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
  static bmpc_app_t app;
  bmpc_hal_t hal;
  size_t i;

  hal.now_us = hal_now;
  hal.can_send = hal_send;
  hal.ctx = NULL;
  g_t = 1000000;
  g_sent = 0;
  bmpc_app_init(&app, &hal);
  for (i = 0; i + 13u <= size; i += 13u) {
    bmpc_can_frame_t f;
    memcpy(&f.id, data + i, 4);
    f.dlc = data[i + 4];
    memcpy(f.data, data + i + 5, 8);
    g_t += 10000;
    bmpc_app_on_frame(&app, &f);
  }
  return 0;
}
