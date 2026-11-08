#!/bin/bash
# libFuzzer on the MCU app's CAN input (fuzz_app_frames.c), double and float builds.
#   1. regression: every file in regress/ (past findings) and corpus/ must run clean
#   2. fuzz for FUZZ_SECONDS (default 300) per build; any crash, leak, UB or timeout fails
# Sanitizers: ASan + UBSan including float-cast-overflow (DEF-6 class). Needs clang with libFuzzer.
# New findings are written to sut/build_fuzz/<build>/artifacts/.
set -u
cd "$(dirname "$0")/../.."
CLANG=${CLANG:-clang}; SECS=${FUZZ_SECONDS:-300}; B=build_fuzz; rc=0
SAN="-fsanitize=fuzzer,address,undefined,float-cast-overflow -fno-sanitize-recover=all"
for v in double float; do
  d=$B/$v; mkdir -p "$d/artifacts" "$d/corpus"; fl=""; [ $v = float ] && fl=-DBMPC_USE_FLOAT
  $CLANG -g -O1 $SAN $fl -std=c99 -Icore/include core/src/*.c test/fuzz/fuzz_app_frames.c -lm -o "$d/fuzz" || { echo "$v: build FAILED"; exit 1; }
  if "$d/fuzz" -runs=0 test/fuzz/regress test/fuzz/corpus > "$d/regress.log" 2>&1; then
    echo "$v: regression OK ($(ls test/fuzz/regress test/fuzz/corpus | grep -vc ':$') inputs)"
  else
    echo "$v: regression FAILED"; grep -m5 -E "runtime error|ERROR|SUMMARY" "$d/regress.log"; rc=1; continue
  fi
  cp test/fuzz/corpus/* "$d/corpus/"
  "$d/fuzz" -max_total_time="$SECS" -timeout=10 -max_len=4000 -rss_limit_mb=2048 -print_final_stats=1 \
    -artifact_prefix="$d/artifacts/" "$d/corpus" test/fuzz/seed > "$d/fuzz.log" 2>&1
  r=$?
  execs=$(grep -m1 stat::number_of_executed_units "$d/fuzz.log" | awk '{print $2}')
  cov=$(grep -E "^#[0-9]+" "$d/fuzz.log" | tail -1 | grep -oE "cov: [0-9]+")
  if [ $r -eq 0 ]; then echo "$v: fuzz ${SECS}s OK, ${execs:-?} executions, ${cov:-cov ?}"
  else echo "$v: fuzz FOUND A PROBLEM (exit $r)"; grep -m5 -E "runtime error|ERROR|SUMMARY|Test unit written" "$d/fuzz.log"; rc=1; fi
done
exit $rc
