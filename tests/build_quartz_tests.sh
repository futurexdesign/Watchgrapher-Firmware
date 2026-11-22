#!/usr/bin/env bash
# Builds and runs the quartz timing host tests.
#
# Usage (from anywhere):
#   tests/build_quartz_tests.sh [BUILD_DIR]
#
# BUILD_DIR receives the CMSIS-DSP objects and the test binaries; a
# fresh mktemp directory is used when it is omitted. The CMSIS kernels are
# the vendored third_party/CMSIS-DSP sources the firmware compiles, built
# for the host without ARM_MATH_CM33 (portable C paths).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="${1:-$(mktemp -d -t quartz-tests.XXXXXX)}"
mkdir -p "$out/cmsis"
out="$(cd "$out" && pwd)"

cc="${CC:-cc}"
cxx="${CXX:-c++}"
# __GNUC_PYTHON__ is CMSIS-DSP's host-build switch: arm_math_types.h then skips
# cmsis_compiler.h (Arm-only intrinsics). Apple clang gets the same path via
# __APPLE_CC__, but Linux gcc/clang need it set explicitly.
defs=(-DARM_MATH_LOOPUNROLL -D__GNUC_PYTHON__)
inc=(-I"$root/include" -I"$root/tests" -I"$root/third_party/CMSIS-DSP/Include")
srcs=("$root"/src/quartz/quartz_dsp.cpp "$root"/src/quartz/motor_timing.cpp
      "$root"/src/quartz/lcd_timing.cpp "$root"/src/quartz/tone_timing.cpp)

objs=()
for c in "$root"/third_party/CMSIS-DSP/Source/*.c; do
  o="$out/cmsis/$(basename "${c%.c}").o"
  "$cc" -O2 "${defs[@]}" -I"$root/third_party/CMSIS-DSP/Include" -c "$c" -o "$o"
  objs+=("$o")
done

for t in units vectors; do
  "$cxx" -std=c++17 -O2 -Wall -Wextra "${defs[@]}" "${inc[@]}" \
    "$root/tests/test_quartz_$t.cpp" "${srcs[@]}" "${objs[@]}" \
    -o "$out/test_quartz_$t"
done

# The engine (firmware-side owner) on top of the library.
"$cxx" -std=c++17 -O2 -Wall -Wextra "${defs[@]}" "${inc[@]}" \
  "$root/tests/test_quartz_engine.cpp" "$root/src/quartz/quartz_engine.cpp" \
  "${srcs[@]}" "${objs[@]}" -o "$out/test_quartz_engine"

# The HID quartz JSON encoders, with the engine's real name tables.
"$cxx" -std=c++17 -O2 -Wall -Wextra "${defs[@]}" "${inc[@]}" \
  "$root/tests/test_hid_json.cpp" "$root/src/hid_json.cpp" \
  "$root/src/dsp_blocks.cpp" "$root/src/quartz/quartz_engine.cpp" "${srcs[@]}" "${objs[@]}" \
  -o "$out/test_hid_json"

# The quartz CLI parsers / formatters (header-only, no objects).
"$cxx" -std=c++17 -O2 -Wall -Wextra "${inc[@]}" \
  "$root/tests/test_quartz_cli_format.cpp" -o "$out/test_quartz_cli_format"

echo "== units"
"$out/test_quartz_units"
# The acceptance vectors are git-ignored (*.wav); tests/run_all.sh sets
# QUARTZ_SKIP_VECTORS=1 when they are absent (CI). The binaries still build.
if [[ "${QUARTZ_SKIP_VECTORS:-0}" == 1 ]]; then
  echo "== vectors, engine: SKIPPED (no docs/handoff/vectors/*.wav)"
else
  echo "== vectors"
  "$out/test_quartz_vectors" "$root/docs/handoff/vectors"
  echo "== engine"
  "$out/test_quartz_engine" "$root/docs/handoff/vectors"
fi
echo "== hid json"
"$out/test_hid_json"
echo "== cli format"
"$out/test_quartz_cli_format"
echo "build dir: $out"
