#!/bin/bash
# P&E(OpenSDA)로 S32K312 에 펌웨어를 올리고, 보드 플래시를 ELF 와 비교한 뒤 리셋한다. 사용: bash flash.sh [elf]  (기본 build/fw_bmpc.elf)
# CAN 어댑터와 무관하다. 디버거 USB(OpenSDA)만 있으면 된다. 팀 capstone_hils 의 fw_s32k312/flash.sh (Linux 실측본)과 같은 순서다.
#
#   Linux:   P&E 이클립스 플러그인(com.pemicro.debug.gdbjtag.pne_6.2.7)의 lin/ (기본 ~/tools/pemicro/lin, PNE_DIR 로 변경)
#            USB 권한: /dev/bus/usb 의 P&E 장치(1357:0089)에 쓰기 권한이 있어야 한다 (udev 규칙)
#   Windows: S32DS 에 들어 있는 P&E GDB 서버
# SWD 속도는 PNE_SPEED (kHz, 기본 5000). 허브를 거쳐 "Failed to power up DAP" 가 나면 PNE_SPEED=1000.
# 주의: CAN 측정 중에는 디버거를 붙이지 않는다 (붙을 때마다 코어가 멈추고 리셋된다).
cd "$(dirname "$0")"
ELF=${1:-build/fw_bmpc.elf}
[ -f "$ELF" ] || { echo "펌웨어 파일이 없습니다: $ELF (먼저 bash build.sh)"; exit 1; }
LOG=build/pegdb.log
mkdir -p build

if [ "$(uname -s)" = Linux ]; then
  PNE_DIR=${PNE_DIR:-$HOME/tools/pemicro/lin}
  SRV="$PNE_DIR/pegdbserver_console"
  GDB=${GDB:-$PNE_DIR/gdb/arm-none-eabi-gdb}
else
  PNE_DIR=${PNE_DIR:-$(ls -d /c/NXP/S32DS.3.6.*/eclipse/plugins/com.pemicro.debug.gdbjtag.pne_*/win32 2>/dev/null | tail -1)}
  SRV="$PNE_DIR/pegdbserver_console.exe"
  GDB=${GDB:-$PNE_DIR/gdb/arm-none-eabi-gdb.exe}
fi
[ -x "$SRV" ] || { echo "P&E 서버가 없습니다: $SRV (PNE_DIR 로 지정)"; exit 1; }
[ -x "$GDB" ] || { echo "gdb 가 없습니다: $GDB (GDB 로 지정)"; exit 1; }
echo "플래시: $ELF"

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

# load 뒤 같은 연결에서 보드 플래시를 ELF 와 비교한다. 연결이 끊기면 gdb 는 로컬 파일끼리 비교해 "matched" 를
# 내므로, 연결 끊김/오류 문구가 있으면 실패로 본다 (capstone_hils 2026-10-28 에 쓰기 실패가 "flash done" 으로 숨겨진 적이 있다)
OUT=$(timeout 150 "$GDB" -batch -nx \
  -ex "set pagination off" -ex "set confirm off" \
  -ex "target remote localhost:7224" -ex "load $ELF" -ex "compare-sections" \
  -ex "monitor reset" -ex "detach" "$ELF" 2>&1)
echo "$OUT" | grep -E "Loading section|Start address|Transfer|matched|MIS-MATCHED|Error|error|closed"

sleep 1
kill $SERVER 2>/dev/null
[ "$(uname -s)" = Linux ] || taskkill //F //IM pegdbserver_console.exe >/dev/null 2>&1
if echo "$OUT" | grep -qE "Remote connection closed|Error|MIS-MATCHED|could not connect" || ! echo "$OUT" | grep -q "Start address"; then
  echo "FLASH FAILED (서버 로그: $LOG)"; grep -E "rror|ARP|ermission" "$LOG" | head -5; exit 1
fi
echo "flash done (보드 플래시 내용 확인됨)"
