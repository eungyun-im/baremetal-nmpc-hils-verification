#!/bin/bash
# 컨테이너 안에서 실행: clang 18 MC/DC. quality.sh 와 같은 테스트·플래그. 소스는 읽기 전용 /sut, 결과는 /out.
set -u
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null && apt-get install -y -qq clang-18 llvm-18 >/dev/null || { echo "apt failed"; exit 2; }
R=/sut; B=/out/build; rm -rf $B; mkdir -p $B
SRC=""; for s in core/src/bmpc_config.c core/src/bmpc_protocol.c core/src/bmpc_delay.c core/src/bmpc_weights.c core/src/bmpc_qp.c core/src/bmpc_mpc.c core/src/bmpc_post.c core/src/bmpc_controller.c core/src/bmpc_app.c integ/bmpc_safety.c third_party/kuuve_safety/safety_ctrl.c; do SRC="$SRC $R/$s"; done
INC="-I$R/core/include -I$R/integ -I$R/third_party/kuuve_safety -I$R/test"
CF="-std=gnu99 -D_POSIX_C_SOURCE=200809L -w -O0 -g -fprofile-instr-generate -fcoverage-mapping -fcoverage-mcdc"
BINS=""
one() { n=$1; fl=$2; t=$3; d=$B/$n; mkdir -p $d
  clang-18 $CF $fl $INC $R/test/$t.c $SRC -lm -o $d/t || { echo "$n build FAILED"; return; }
  (cd $d && LLVM_PROFILE_FILE=$d/p.profraw ./t >/dev/null 2>&1; echo "$n exit $?")
  BINS="$BINS $d/t"; }
one core "" test_core; one core_more "" test_core_more; one qp_more "" test_qp_more; one integ -DBS_TEST_HOOK test_integ; one integ_more -DBS_TEST_HOOK test_integ_more; one integ_plaus "-DBS_TEST_HOOK -DBS_PLAUS" test_integ_more; one app "" test_app; one release_gate "" test_release_gate; one release_gate_bench -DBS_BENCH_RESET test_release_gate; one safety_reqs "" test_safety_reqs; one wrap_pi "" test_wrap_pi; one wrap_pi_float -DBMPC_USE_FLOAT test_wrap_pi; one def6 "" test_def6; one solver_fail "-DBS_TEST_HOOK -DBS_PLAUS -DBMPC_TEST_HOOK -DBMPC_USE_FLOAT" test_solver_fail
# 바이너리마다 매핑이 달라(플래그 차이) 따로 보고하고, 같은 플래그 묶음끼리만 합친다
for d in $B/*/; do n=$(basename $d)
  [ -f $d/p.profraw ] || continue
  llvm-profdata-18 merge -o $d/p.profdata $d/p.profraw
  llvm-cov-18 report $d/t -instr-profile=$d/p.profdata --show-mcdc-summary $SRC > $d/report.txt 2>&1
  echo "== $n"; tail -1 $d/report.txt
done
# 전체 합산: 모든 profraw 를 합치고 모든 바이너리를 -object 로 준다 (매크로가 달라 매핑이 다른 함수는 llvm-cov 가 경고와 함께 처리)
llvm-profdata-18 merge -o $B/all.profdata $B/*/p.profraw
FIRST=""; OBJ=""; for d in $B/*/; do [ -f $d/t ] || continue; if [ -z "$FIRST" ]; then FIRST=$d/t; else OBJ="$OBJ -object $d/t"; fi; done
llvm-cov-18 report $FIRST $OBJ -instr-profile=$B/all.profdata --show-mcdc-summary $SRC > /out/report_all.txt 2> /out/report_all.err
echo "== ALL"; tail -1 /out/report_all.txt; grep -c "mismatched" /out/report_all.err
