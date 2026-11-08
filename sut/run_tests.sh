#!/bin/bash
# 보드와 ROS 없이 단위 시험을 돌린다. 빌드 결과는 ./build_tests 에 둔다.
set -u
cd "$(dirname "$0")"; B=build_tests; mkdir -p $B
INC="-Icore/include -Iinteg -Ithird_party/kuuve_safety -Itest"
SRC="core/src/bmpc_config.c core/src/bmpc_protocol.c core/src/bmpc_delay.c core/src/bmpc_weights.c core/src/bmpc_qp.c core/src/bmpc_mpc.c core/src/bmpc_post.c core/src/bmpc_controller.c core/src/bmpc_app.c integ/bmpc_safety.c third_party/kuuve_safety/safety_ctrl.c"
rc=0
run() { name=$1; flags=$2; t=$3
  if gcc -std=gnu99 -D_POSIX_C_SOURCE=200809L -O2 -w $flags $INC test/$t.c $SRC -lm -o $B/$name; then
    printf '%-28s %s\n' "$name" "$($B/$name 2>&1 | tail -1)"; else echo "$name BUILD FAILED"; rc=1; fi; }
run def6 "-fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all" test_def6; run def6_float "-DBMPC_USE_FLOAT -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all" test_def6
run core ""  test_core; run core_more "" test_core_more; run qp_more "" test_qp_more
run integ "-DBS_TEST_HOOK" test_integ; run integ_more "-DBS_TEST_HOOK" test_integ_more; run integ_plaus "-DBS_TEST_HOOK -DBS_PLAUS" test_integ_more; run app "" test_app
run release_gate "" test_release_gate; run release_gate_bench -DBS_BENCH_RESET test_release_gate
for fl in "" "-DBMPC_USE_FLOAT"; do n=wrap_pi${fl:+_float}; if gcc -std=gnu99 -O2 -w $fl -Icore/include test/test_wrap_pi.c -lm -o $B/$n; then printf '%-28s %s\n' $n "$($B/$n 2>&1 | tail -1)"; else echo "$n BUILD FAILED"; rc=1; fi; done
if gcc -std=c99 -O2 -w -Ithird_party/kuuve_safety test/test_safety_reqs.c third_party/kuuve_safety/safety_ctrl.c -lm -o $B/safety_reqs; then
  printf '%-28s %s\n' safety_reqs "$($B/safety_reqs 2>&1 | tail -1)"; else echo "safety_reqs BUILD FAILED"; rc=1; fi
exit $rc
