#!/bin/bash
# S32K312 bmpc 펌웨어 빌드. 사용: bash build.sh
# 산출물: build/fw_bmpc.elf / .bin / .map, 메모리 사용량 build/size.txt
#
# 툴체인 (먼저 찾은 것): $ARM_GCC_BIN, S32DS 의 GCC 11.4 (Linux ~/NXP/S32DS.3.6.*, Windows C:/NXP/S32DS.3.6.12), PATH 의 arm-none-eabi-gcc
# 옵션 (환경변수)
#   DCACHE=0        데이터 캐시 끔 (기본 켬)
#   APP_SRAM=1      bmpc_app 을 SRAM 에 (비교용. 기본은 DTCM)
#   N_MAX=30        BMPC_N_MAX (기본 32, 설정 N 보다 작으면 MCU 가 N 을 잘라 버린다)
#   HEARTBEAT_MS=0  하트비트 주기 (기본 500, 0 = 끔)
#   OPT=-O2         최적화 (기본 -O3)
#   PROFILE=1       구간별 시간을 0x3F0/0x3F1 로 보낸다 (main.c 머리말)
#   PLL=0           코어를 FIRC 48 MHz 로 둔다 (기본은 PLL 120 MHz)
#   SAFETY=1        팀 안전 제어기 통합 (integ/bmpc_safety + third_party/kuuve_safety)
#   PLAUS=1         조향 명령 타당성 검사 (integ/bmpc_safety.c). SAFETY=1 과 함께
#   SWT=1           워치독 켜기 (200 ms, 메인 루프에서 서비스). FAULT_HOOK=1 이면 hook_kind 3 으로 메인 루프를 멈춰 시험한다
#   FAULT_HOOK=1    시험대 전용 조향 명령 결함 주입 (0x7F0, integ/bmpc_safety.h). 양산 빌드 제외. SAFETY=1 과 함께
set -e
cd "$(dirname "$0")"
CORE=../../core

if [ -n "$ARM_GCC_BIN" ]; then
  GCC=$ARM_GCC_BIN
elif [ "$(uname -s)" = Linux ] && ls -d "$HOME"/NXP/S32DS.3.6.*/S32DS/build_tools/gcc_v11.4/gcc-11.4-arm32-eabi/bin >/dev/null 2>&1; then
  GCC=$(ls -d "$HOME"/NXP/S32DS.3.6.*/S32DS/build_tools/gcc_v11.4/gcc-11.4-arm32-eabi/bin | tail -1)
elif [ -d /c/NXP/S32DS.3.6.12/S32DS/build_tools/gcc_v11.4/gcc-11.4-arm32-eabi/bin ]; then
  GCC=/c/NXP/S32DS.3.6.12/S32DS/build_tools/gcc_v11.4/gcc-11.4-arm32-eabi/bin
elif command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  GCC=$(dirname "$(command -v arm-none-eabi-gcc)")
else
  echo "arm-none-eabi-gcc 를 찾지 못했습니다. ARM_GCC_BIN=<.../bin> 으로 지정하세요." >&2
  exit 1
fi
EXE=
[ -x "$GCC/arm-none-eabi-gcc.exe" ] && EXE=.exe
CC=$GCC/arm-none-eabi-gcc$EXE

# 이 FPU(fpv5-sp-d16)는 단정밀도만 있어서 float 빌드가 필수다. double 승격은 -Wdouble-promotion 으로 막는다.
# -ffreestanding 은 쓰지 않는다: 쓰면 fabsf/sqrtf 가 FPU 명령 하나가 아니라 함수 호출이 된다 (QP 1 회에 수천 번).
DEFS="-DBMPC_USE_FLOAT"
[ "${DCACHE:-1}" = 1 ] && DEFS="$DEFS -DBMPC_FW_DCACHE"
[ "$APP_SRAM" = 1 ] && DEFS="$DEFS -DBMPC_FW_APP_SRAM"
[ -n "$N_MAX" ] && DEFS="$DEFS -DBMPC_N_MAX=$N_MAX"
[ -n "$HEARTBEAT_MS" ] && DEFS="$DEFS -DBMPC_FW_HEARTBEAT_MS=${HEARTBEAT_MS}u"
[ "$PROFILE" = 1 ] && DEFS="$DEFS -DBMPC_PROFILE"
[ "${PLL:-1}" = 1 ] && DEFS="$DEFS -DBMPC_FW_PLL"
[ "$SWT" = 1 ] && DEFS="$DEFS -DBMPC_FW_SWT"
[ "$PLAUS" = 1 ] && DEFS="$DEFS -DBS_PLAUS"
EXTRA_SRC=""
if [ "$SAFETY" = 1 ]; then
  DEFS="$DEFS -DBMPC_FW_SAFETY -I../../integ -I../../third_party/kuuve_safety"
  EXTRA_SRC="../../integ/bmpc_safety.c ../../third_party/kuuve_safety/safety_ctrl.c"
  [ "$FAULT_HOOK" = 1 ] && DEFS="$DEFS -DBS_TEST_HOOK"
fi

mkdir -p build
echo "GCC: $GCC"
echo "DEFS: $DEFS"
$CC -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -std=gnu99 ${OPT:--O3} -g $DEFS \
  -Wall -Wextra -Werror -Wdouble-promotion -fno-math-errno -ffunction-sections -fdata-sections \
  -I$CORE/include -nostartfiles --specs=nano.specs -T linker.ld -Wl,--gc-sections -Wl,-Map=build/fw_bmpc.map \
  -Wl,--print-memory-usage \
  startup.S main.c $CORE/src/*.c $EXTRA_SRC \
  -o build/fw_bmpc.elf -lm 2>&1 | tee build/build_log.txt
[ "${PIPESTATUS[0]}" = 0 ] || exit 1
$GCC/arm-none-eabi-objcopy$EXE -O binary build/fw_bmpc.elf build/fw_bmpc.bin
$GCC/arm-none-eabi-size$EXE -A build/fw_bmpc.elf | tee build/size.txt | grep -E "text|data|bss|Total"

# 소프트웨어 double 연산이 링크됐으면 계산이 크게 느려진다 (라이브러리 함수가 double 을 쓰는 경우 등)
SOFT_D=$($GCC/arm-none-eabi-nm$EXE build/fw_bmpc.elf | grep -E ' __aeabi_(d[a-z]+|f2d|d2f|[iul]+2d|d2[iul]+z)$' || true)
if [ -n "$SOFT_D" ]; then
  echo "경고: 소프트웨어 double 함수가 링크됐습니다:"
  echo "$SOFT_D"
fi
