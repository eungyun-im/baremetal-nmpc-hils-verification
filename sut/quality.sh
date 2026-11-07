#!/bin/bash
# 커버리지와 sanitizer 측정. 테스트마다 빌드 폴더를 나눈다. 한 폴더에서 다시 빌드하면 .gcno 가 덮여 마지막 테스트만 집계된다.
set -u
cd "$(dirname "$0")"; R=$PWD; B=$R/build_quality; rm -rf "$B"; mkdir -p "$B"
SRC=""; for s in core/src/bmpc_config.c core/src/bmpc_protocol.c core/src/bmpc_delay.c core/src/bmpc_weights.c core/src/bmpc_qp.c core/src/bmpc_mpc.c core/src/bmpc_post.c core/src/bmpc_controller.c core/src/bmpc_app.c integ/bmpc_safety.c third_party/kuuve_safety/safety_ctrl.c; do SRC="$SRC $R/$s"; done
INC="-I$R/core/include -I$R/integ -I$R/third_party/kuuve_safety -I$R/test"; CF="-std=gnu99 -D_POSIX_C_SOURCE=200809L -w"; rc=0; ADD=""
one() { n=$1; fl=$2; t=$3; d=$B/$n; mkdir -p "$d/cov" "$d/san"
  (cd "$d/cov" && gcc $CF -O0 -g --coverage $fl $INC "$R/test/$t.c" $SRC -lm -o t && ./t >/dev/null 2>&1) || { echo "$n coverage run FAILED"; rc=1; }
  (cd "$d/cov" && gcovr -r "$R" . --filter "$R/core/src/" --filter "$R/integ/" --filter "$R/third_party/" --json "$B/$n.json" 2>/dev/null)
  (cd "$d/san" && gcc $CF -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=undefined $fl $INC "$R/test/$t.c" $SRC -lm -o t && ./t > out.txt 2>&1) && echo "$n sanitizer OK: $(tail -1 "$d/san/out.txt")" || { echo "$n sanitizer FAILED"; rc=1; }
  ADD="$ADD --add-tracefile $B/$n.json"; }
one core "" test_core; one core_more "" test_core_more; one qp_more "" test_qp_more; one integ -DBS_TEST_HOOK test_integ; one integ_more -DBS_TEST_HOOK test_integ_more; one integ_plaus "-DBS_TEST_HOOK -DBS_PLAUS" test_integ_more; one app "" test_app
gcovr -r . $ADD --txt-summary --json-summary "$B/coverage_summary.json" 2>/dev/null | tail -3
exit $rc
