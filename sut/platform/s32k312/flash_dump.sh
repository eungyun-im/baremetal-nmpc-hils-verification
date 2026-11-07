#!/bin/bash
# 보드를 쓰지 않고 읽기만 한다: 플래시와 DTCM 을 파일로 받고, 주어진 ELF 들과 플래시를 비교한 뒤 리셋한다.
# 플래시를 덮어쓰기 전에 지금 펌웨어를 남기고, 스택 사용량을 볼 때 쓴다. 사용: bash flash_dump.sh <출력 폴더> [ELF ...]
# 주의: 디버거가 붙으면 코어가 멈추고 끝날 때 리셋된다. 주행 중에는 쓰지 않는다.
cd "$(dirname "$0")"
OUT=${1:?출력 폴더를 지정하세요}
shift
mkdir -p "$OUT"
PNE_DIR=${PNE_DIR:-$HOME/tools/pemicro/lin}
SRV="$PNE_DIR/pegdbserver_console"
GDB=${GDB:-$PNE_DIR/gdb/arm-none-eabi-gdb}
LOG="$OUT/pegdb_dump.log"

"$SRV" -startserver -singlesession -serverport=7224 \
  -interface=USBMULTILINK -port=USB1 -device=NXP_S32K3xx_S32K312 -speed=${PNE_SPEED:-5000} > "$LOG" 2>&1 &
SERVER=$!
for i in $(seq 1 40); do
  grep -q "All Servers Running" "$LOG" 2>/dev/null && break
  sleep 0.5
done
if ! grep -q "All Servers Running" "$LOG"; then
  echo "P&E 서버가 시작되지 않았습니다. 로그:"; tail -20 "$LOG"; kill $SERVER 2>/dev/null; exit 1
fi

# 코어가 재시작하기 전에 받는다. flash 0x00400000 (부트 헤더 포함 64 KB), DTCM 0x20000000 (64 KB)
ARGS=(-ex "set pagination off" -ex "set confirm off" -ex "target remote localhost:7224"
      -ex "dump binary memory $OUT/dtcm.bin 0x20000000 0x20010000"
      -ex "dump binary memory $OUT/flash.bin 0x00400000 0x00410000")
for ELF in "$@"; do
  ARGS+=(-ex "echo === compare $ELF\n" -ex "file $ELF" -ex "compare-sections")
done
ARGS+=(-ex "monitor reset" -ex "detach")
OUTPUT=$(timeout 150 "$GDB" -batch -nx "${ARGS[@]}" 2>&1)
RC=$?
echo "$OUTPUT" | grep -E "=== compare|matched|MIS-MATCHED|Error|error|closed"
sleep 1
kill $SERVER 2>/dev/null
if [ $RC -ne 0 ] || [ ! -s "$OUT/flash.bin" ] || [ ! -s "$OUT/dtcm.bin" ] || echo "$OUTPUT" | grep -qE "Remote connection closed|could not connect"; then
  echo "DUMP FAILED (서버 로그: $LOG)"; exit 1
fi
md5sum "$OUT/flash.bin" "$OUT/dtcm.bin"
echo "dump done"
