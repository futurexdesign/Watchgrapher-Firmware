#!/usr/bin/env bash
# Builds the host DSP harness (dsp_wav) from the real firmware DSP sources.
#
# Usage: tests/dsp_host/build.sh <OUT_DIR> [--profile]
# Writes <OUT_DIR>/dsp_wav. Environment: CXX (default c++).
# --profile enables per-stage wall-clock timing in the --metrics output.
# The firmware sources are compiled unmodified; shim/ supplies Arduino.h,
# arm_math.h and the timebase / diagnostics services (see dsp_wav.cpp).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
out="${1:?usage: build.sh <OUT_DIR> [--profile]}"
shift
if [[ "$#" -gt 0 ]]; then
  if [[ "$#" -ne 1 || "$1" != "--profile" ]]; then
    echo 'usage: build.sh <OUT_DIR> [--profile]' >&2
    exit 2
  fi
  set -- -DTG_HOST_PROFILE_TICK_SHAPE
fi
mkdir -p "$out"

"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -DAUDIO_SOURCE=SYNTHETIC "$@" \
  -I"$here/shim" -I"$root/include" \
  "$here/dsp_wav.cpp" "$here/shim/host_stubs.cpp" \
  "$root/src/dsp_graph.cpp" "$root/src/dsp_blocks.cpp" \
  "$root/src/perf_timing.cpp" \
  "$root/src/mechanical/tick_detect.cpp" \
  "$root/src/mechanical/tick_shape.cpp" \
  "$root/src/mechanical/beat_analysis.cpp" \
  "$root/src/mechanical/beat_fit.cpp" \
  -o "$out/dsp_wav"
