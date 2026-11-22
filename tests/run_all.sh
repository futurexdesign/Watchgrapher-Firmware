#!/usr/bin/env bash
# Builds and runs every host-side unit test, then prints a PASS/FAIL/SKIP table.
#
# Usage (from anywhere):
#   tests/run_all.sh [BUILD_DIR]        default BUILD_DIR: <repo>/build/host-tests
#
# Environment:
#   CC, CXX     C / C++ compilers (default: cc, c++)
#   PYTHON      Python interpreter for the Python test (default: python3)
#   SANITIZE=1  add -fsanitize=address,undefined -fno-omit-frame-pointer to the
#               C / C++ tests (the quartz tests from build_quartz_tests.sh are
#               not sanitized; that script has no flag hook)
#
# Every C/C++ test is listed once in the table below with the exact sources it
# links against. A test whose source file is absent is reported as SKIP
# (missing). A "?" prefix marks an optional source that is linked only when it
# exists. The quartz tests are built and run by tests/build_quartz_tests.sh; the
# acceptance vectors (docs/handoff/vectors/*.wav) are git-ignored, so the vector
# and engine runs are skipped when no .wav is present.
#
# Exit status: 0 when nothing FAILed (SKIP does not fail the run), 1 otherwise.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="${1:-$root/build/host-tests}"
mkdir -p "$out/bin" "$out/logs"
out="$(cd "$out" && pwd)"

cc="${CC:-cc}"
cxx="${CXX:-c++}"
py="${PYTHON:-python3}"

# Common flags. Asserts stay enabled (no -DNDEBUG): the tests check via assert().
# Expanded with ${arr[@]+"${arr[@]}"} so empty arrays are safe under set -u on
# the bash 3.2 that macOS ships.
warn=(-Wall -Wextra -Werror -O1 -g -I"$root/include")
san=()
if [[ "${SANITIZE:-0}" == 1 ]]; then
  san=(-fsanitize=address,undefined -fno-omit-frame-pointer)
fi
cxxflags=(-std=c++17 "${warn[@]}" ${san[@]+"${san[@]}"})
cflags=(-std=c11 "${warn[@]}" ${san[@]+"${san[@]}"})

# Result table (parallel arrays; bash 3.2 has no associative arrays).
res_name=()
res_status=()
res_note=()

record() { # name status note
  res_name+=("$1")
  res_status+=("$2")
  res_note+=("$3")
  case "$2" in
    PASS) echo "--> PASS  $1" ;;
    FAIL) echo "--> FAIL  $1: $3" ;;
    SKIP) echo "--> SKIP  $1: $3" ;;
  esac
}

# Prints the tail of a log so a failure is visible without opening the file.
show_tail() { # logfile
  echo "    ---- last lines of $1 ----"
  tail -n 25 "$1" | sed 's/^/    /'
  echo "    ----"
}

# Compile and run one C or C++ test.
#   entry NAME LANG TEST [SRC ...]
#   LANG: c | cxx. TEST and SRC are relative to the repository root.
entry() {
  local name="$1" lang="$2" test="$3"
  shift 3
  if [[ ! -f "$root/$test" ]]; then
    echo "SKIP (missing) $name: $test does not exist yet"
    record "$name" SKIP "missing $test"
    return 0
  fi

  local args=("$root/$test") s
  for s in ${1+"$@"}; do
    case "$s" in
      \?*)
        if [[ -f "$root/${s#?}" ]]; then args+=("$root/${s#?}"); fi
        ;;
      *)
        if [[ ! -f "$root/$s" ]]; then
          echo "--> FAIL  $name: source $s is missing"
          record "$name" FAIL "missing source $s"
          return 0
        fi
        args+=("$root/$s")
        ;;
    esac
  done

  local bin="$out/bin/$name" log="$out/logs/$name.log"
  local status=PASS
  if [[ "$lang" == c ]]; then
    "$cc" "${cflags[@]}" "${args[@]}" -o "$bin" >"$log" 2>&1 || status=BUILD
  else
    "$cxx" "${cxxflags[@]}" "${args[@]}" -o "$bin" >"$log" 2>&1 || status=BUILD
  fi
  if [[ "$status" == BUILD ]]; then
    echo "--> FAIL  $name: compile error"
    show_tail "$log"
    record "$name" FAIL "compile error (see $log)"
    return 0
  fi

  if (cd "$root" && "$bin") >"$log" 2>&1; then
    record "$name" PASS ""
  else
    echo "--> FAIL  $name: test exited non-zero"
    show_tail "$log"
    record "$name" FAIL "test failed (see $log)"
  fi
  return 0
}

# ---- Table: one line per C / C++ host test -------------------------------
# C tests (standalone mirrors; they link nothing from src/).
entry pcm_conversion      c   tests/test_pcm_conversion.c
entry ring_buffer         c   tests/test_ring_buffer.c
# C++ tests.
entry pcm1808_conversion  cxx tests/test_pcm1808_pcm_conversion.cpp
entry pcm1808_rate        cxx tests/test_pcm1808_rate.cpp
entry beat_fit            cxx tests/test_beat_fit.cpp src/mechanical/beat_fit.cpp
entry pps_filter          cxx tests/test_pps_filter.cpp
entry nmea                cxx tests/test_nmea.cpp
entry ui_format           cxx tests/test_ui_format.cpp
entry dsp_blocks          cxx tests/test_dsp_blocks.cpp src/dsp_blocks.cpp
entry timebase_log        cxx tests/test_timebase_log.cpp
entry device_cli_format   cxx tests/test_device_cli_format.cpp src/dsp_blocks.cpp

# ---- Quartz tests: tests/build_quartz_tests.sh --------------------------
# The script builds the vendored CMSIS-DSP kernels for the host and runs
# test_quartz_units, test_hid_json and test_quartz_cli_format, plus
# test_quartz_vectors and test_quartz_engine when the .wav vectors exist.
quartz_log="$out/logs/quartz.log"
if ls "$root"/docs/handoff/vectors/*.wav >/dev/null 2>&1; then
  skip_wav=0
  wav_note=""
else
  skip_wav=1
  wav_note="no docs/handoff/vectors/*.wav (git-ignored)"
fi
if QUARTZ_SKIP_VECTORS="$skip_wav" "$root/tests/build_quartz_tests.sh" \
     "$out/quartz" >"$quartz_log" 2>&1; then
  record quartz-units-hid-cli PASS ""
  if [[ "$skip_wav" == 1 ]]; then
    echo "SKIP (missing) quartz-vectors / quartz-engine: $wav_note"
    record quartz-vectors SKIP "$wav_note"
    record quartz-engine SKIP "$wav_note"
  else
    record quartz-vectors PASS ""
    record quartz-engine PASS ""
  fi
else
  echo "--> FAIL  quartz: build_quartz_tests.sh failed"
  show_tail "$quartz_log"
  record quartz-units-hid-cli FAIL "see $quartz_log"
  if [[ "$skip_wav" == 1 ]]; then
    record quartz-vectors SKIP "$wav_note"
    record quartz-engine SKIP "$wav_note"
  else
    record quartz-vectors FAIL "see $quartz_log"
    record quartz-engine FAIL "see $quartz_log"
  fi
fi

# ---- Python: NTP calibration fit (stdlib only) ----------------------------
py_test="tests/test_calibrate_timebase.py"
if [[ ! -f "$root/$py_test" ]]; then
  echo "SKIP (missing) calibrate-timebase: $py_test does not exist"
  record calibrate-timebase SKIP "missing $py_test"
elif ! command -v "$py" >/dev/null 2>&1; then
  echo "SKIP (missing) calibrate-timebase: $py not found"
  record calibrate-timebase SKIP "$py not installed"
else
  py_log="$out/logs/calibrate-timebase.log"
  if (cd "$root" && "$py" "$py_test") >"$py_log" 2>&1; then
    record calibrate-timebase PASS ""
  else
    echo "--> FAIL  calibrate-timebase: test exited non-zero"
    show_tail "$py_log"
    record calibrate-timebase FAIL "see $py_log"
  fi
fi

# ---- Summary ---------------------------------------------------------------
echo
echo "Host test summary (build dir: $out)"
printf '%-24s %-6s %s\n' "TEST" "STATUS" "NOTE"
printf '%-24s %-6s %s\n' "------------------------" "------" "----"
pass=0
fail=0
skip=0
i=0
while [[ $i -lt ${#res_name[@]} ]]; do
  printf '%-24s %-6s %s\n' "${res_name[$i]}" "${res_status[$i]}" "${res_note[$i]}"
  case "${res_status[$i]}" in
    PASS) pass=$((pass + 1)) ;;
    FAIL) fail=$((fail + 1)) ;;
    SKIP) skip=$((skip + 1)) ;;
  esac
  i=$((i + 1))
done
echo
echo "PASS: $pass  FAIL: $fail  SKIP: $skip"
if [[ "$fail" -ne 0 ]]; then
  exit 1
fi
