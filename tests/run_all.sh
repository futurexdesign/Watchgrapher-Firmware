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
# exists. A word that starts with "-" (for example -pthread) is passed to the
# compiler as a flag instead of being treated as a source file. The quartz
# tests are built and run by tests/build_quartz_tests.sh; the acceptance
# vectors (docs/handoff/vectors/*.wav) are git-ignored, so the vector
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
      -*) args+=("$s") ;;
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
entry beat_math           cxx tests/test_beat_math.cpp
entry pps_filter          cxx tests/test_pps_filter.cpp
entry nmea                cxx tests/test_nmea.cpp
entry ui_format           cxx tests/test_ui_format.cpp
entry dsp_blocks          cxx tests/test_dsp_blocks.cpp src/dsp_blocks.cpp
entry timebase_log        cxx tests/test_timebase_log.cpp
entry device_cli_format   cxx tests/test_device_cli_format.cpp src/dsp_blocks.cpp
entry control_frame       cxx tests/test_control_frame.cpp src/control/frame.cpp
entry frame_ring          cxx tests/test_frame_ring.cpp -pthread
entry beats_packed       cxx tests/test_beats_packed.cpp src/hid_json.cpp src/dsp_blocks.cpp
entry ble_pacer           cxx tests/test_ble_pacer.cpp
entry ble_core            cxx tests/test_ble_core.cpp
entry control_session     cxx tests/test_control_session.cpp src/control/frame.cpp src/control/session.cpp
entry control_arbiter     cxx tests/test_control_arbiter.cpp src/control/frame.cpp src/control/session.cpp src/control/arbiter.cpp

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

# ---- Static checks: radio core selection (CORE-02, CORE-04) --------------
core_log="$out/logs/ble-core-static.log"
if (cd "$root" && bash tests/check_ble_core.sh) >"$core_log" 2>&1; then
  record ble-core-static PASS ""
else
  echo "--> FAIL  ble-core-static: tests/check_ble_core.sh failed"
  show_tail "$core_log"
  record ble-core-static FAIL "see $core_log"
fi

# ---- Host DSP harness: real mechanical chain over recorded clips ----------
# tests/dsp_host builds src/dsp_graph.cpp + mechanical/*.cpp against shims and
# runs them over clips from assets/synth_clips (clips.json has the values).
dsp_build_log="$out/logs/dsp-host-build.log"
dsp_built=0
if CXX="$cxx" "$root/tests/dsp_host/build.sh" "$out/dsp_host" >"$dsp_build_log" 2>&1; then
  dsp_built=1
fi

# dsp_clip_row <name> <clip> <python check reading the log> <harness args...>
dsp_clip_row() {
  local name="$1" clip="$2" check="$3"
  shift 3
  local log="$out/logs/$name.log"
  if [[ ! -f "$root/$clip" ]]; then
    record "$name" SKIP "missing $clip"
  elif [[ "$dsp_built" -ne 1 ]]; then
    echo "--> FAIL  $name: tests/dsp_host/build.sh failed"
    show_tail "$dsp_build_log"
    record "$name" FAIL "build error (see $dsp_build_log)"
  elif (cd "$root" && "$out/dsp_host/dsp_wav" "$clip" "$@") >"$log" 2>&1 &&
       python3 -c "$check" "$log"; then
    record "$name" PASS ""
  else
    echo "--> FAIL  $name"
    show_tail "$log"
    record "$name" FAIL "see $log"
  fi
}

# Rate: designed loop rate +4.05 s/d.
dsp_clip_row dsp-host-nh35 assets/synth_clips/nh35_21600.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
sys.exit(0 if (last["locked"] and last["vphNearest"] == 21600
               and last["avgLong"]["rateValid"]
               and abs(last["avgLong"]["rateSDay"] - 4.05) <= 0.5) else 1)
' --loops 6 --lift 52 --avg 20 --every 100

# Amplitude on a clipped recording (every loud pulse at the same ceiling):
# drop = strongest pulse by 0.5 ms mean, not by peak sample. ChronoPulse 327.
dsp_clip_row dsp-host-st3600-amp assets/synth_clips/st3600_21600.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (a["amplitudeValid"] and abs(a["amplitudeDeg"] - 327) <= 15
               and last["beatsAmplitudeValid"] >= 0.9 * last["beatsRecorded"]) else 1)
' --loops 4 --lift 52 --avg 20 --every 100

# Watch change 21600 -> 28800: the rolling mean restarts with the new rate
# and never carries the old one (it overshot to ~34 s/d before the fix).
dsp_clip_row dsp-host-switch assets/synth_clips/switch_21600_28800.wav '
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
fast = [r for r in rows if not r.get("summary") and r["locked"]
        and r["vphNearest"] == 28800 and r["avgLong"]["rateValid"]]
last = rows[-1]
sys.exit(0 if (fast and all(r["stretchS"] >= 5 and (not r["settled"] or
                            abs(r["avgLong"]["rateSDay"] - 3.3) <= 2.0)
                            for r in fast)
               and last["vphNearest"] == 28800) else 1)
' --loops 1 --lift 52 --avg 10 --every 1

# Cuts and no watch (beat_analysis.h, item 4 of the timing engine plan).
#
# Watch change with 5 s of handling noise from 12 s (g_change_21600_28800;
# truth +8.0 s/d, then -5.0 s/d from 17 s): the no-watch rule fires within 2 s
# of the noise, no rate is reported while locked to a rate other than 21600
# until the stretch spans 5 s (the new watch starts at 17 s), and the final
# settled average is within 0.5 s/d of -5. Run with both timing landmarks.
nowatch_check='
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
last = rows[-1]
body = [r for r in rows if not r.get("summary")]
fast = [r for r in body if r["locked"] and r["vphNearest"] != 21600
        and (r["rateValid"] or r["avgLong"]["rateValid"])]
ok = (12.0 <= last["firstNoWatchS"] <= 14.0
      and all(r["stretchS"] >= 5 and r["vphNearest"] == 28800 and r["t"] >= 22
              for r in fast)
      and body[-1]["settled"]
      and last["avgLong"]["rateValid"] and abs(last["avgLong"]["rateSDay"] + 5.0) <= 0.5)
sys.exit(0 if ok else 1)
'
dsp_clip_row dsp-host-nowatch-change assets/synth_clips/generated/g_change_21600_28800.wav "$nowatch_check" --loops 1 --lift 52 --avg 10 --every 0.5
dsp_clip_row dsp-host-nowatch-change-drop assets/synth_clips/generated/g_change_21600_28800.wav "$nowatch_check" --timing drop --loops 1 --lift 52 --avg 10 --every 0.5

# A single knock or a short dropout is not "no watch": the state never reads
# noWatch and the rate stays within 0.3 s/d of the +5.0 s/d truth.
quiet_check='
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (last["noWatchBlocks"] == 0 and a["rateValid"]
               and abs(a["rateSDay"] - 5.0) <= 0.3) else 1)
'
for clip in knocks dropouts; do
  dsp_clip_row "dsp-host-quiet-$clip" "assets/synth_clips/generated/g_21600_$clip.wav" "$quiet_check" --loops 1 --lift 52 --avg 10 --every 100
  dsp_clip_row "dsp-host-quiet-$clip-drop" "assets/synth_clips/generated/g_21600_$clip.wav" "$quiet_check" --timing drop --loops 1 --lift 52 --avg 10 --every 100
done

# Recorded Waltham (18,000 A/h, real stray detections): not "no watch".
dsp_clip_row dsp-host-quiet-waltham assets/synth_clips/waltham_18000.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (last["noWatchBlocks"] == 0 and a["rateValid"]
               and abs(a["rateSDay"] - 20.05) <= 1.0) else 1)
' --loops 10 --lift 52 --avg 20 --every 100

# A step of the clock correction (20 ppm at 8 s) is a cut with reason "clock".
dsp_clip_row dsp-host-cut-clock assets/synth_clips/generated/g_21600_clean.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
sys.exit(0 if last["cuts"] == 1 and last["lastCut"] == "clock" else 1)
' --loops 1 --lift 52 --avg 10 --every 100 --ppm-at 8,20

# settleS: with 15 s of settling no sample enters the average before the
# stretch is 15 s old (the stretch itself starts at about 1 s).
dsp_clip_row dsp-host-settle assets/synth_clips/generated/g_21600_clean.wav '
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
body = [r for r in rows if not r.get("summary")]
a = rows[-1]["avgLong"]
sys.exit(0 if (all(not r["avgLong"]["rateValid"] for r in body if r["t"] < 16)
               and a["rateValid"] and abs(a["rateSDay"] - 6.0) <= 0.3) else 1)
' --loops 1 --lift 52 --avg 10 --every 1 --settle 15

# Drop landmark (timingLandmark = drop, tick_shape.h): the beat-template stage
# times the drop from the shape of the whole beat. Truth is known exactly for
# the generated clips (scripts/synth_clips/gen_ticks.py, *.truth.json).
#
# 20 dB SNR clip, +5.0 s/d: the unlock-timed rate reads -17.8 s/d here.
dsp_clip_row dsp-host-drop-snr20 assets/synth_clips/generated/g_21600_snr20.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (last["locked"] and a["rateValid"]
               and abs(a["rateSDay"] - 5.0) <= 0.5) else 1)
' --timing drop --loops 1 --lift 52 --avg 20 --every 100

# Unlock louder than the drop, +5.0 s/d: no jumping between the two landmarks.
dsp_clip_row dsp-host-drop-unlock-louder assets/synth_clips/generated/g_21600_unlock_louder.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (last["locked"] and a["rateValid"]
               and abs(a["rateSDay"] - 5.0) <= 0.2) else 1)
' --timing drop --loops 1 --lift 52 --avg 20 --every 100

# Recorded NH35 (designed loop rate +4.05 s/d): drop timing keeps the rate and
# lowers the fit jitter. Runs the harness with both landmarks.
if [[ ! -f "$root/assets/synth_clips/nh35_21600.wav" ]]; then
  record dsp-host-drop-nh35 SKIP "missing assets/synth_clips/nh35_21600.wav"
elif [[ "$dsp_built" -ne 1 ]]; then
  echo "--> FAIL  dsp-host-drop-nh35: tests/dsp_host/build.sh failed"
  show_tail "$dsp_build_log"
  record dsp-host-drop-nh35 FAIL "build error (see $dsp_build_log)"
else
  nh35_log="$out/logs/dsp-host-drop-nh35.log"
  if (cd "$root" &&
      for mode in unlock drop; do
        echo "== --timing $mode"
        "$out/dsp_host/dsp_wav" assets/synth_clips/nh35_21600.wav \
          --timing "$mode" --loops 6 --lift 52 --avg 20 --every 100
      done) >"$nh35_log" 2>&1 &&
     python3 -c '
import json, sys
rows, mode = {}, None
for l in open(sys.argv[1]):
    if l.startswith("== --timing "):
        mode = l.split()[-1]
    elif l.startswith("{\"summary\""):
        rows[mode] = json.loads(l)
d, u = rows["drop"], rows["unlock"]
ok = (d["locked"] and d["avgLong"]["rateValid"]
      and abs(d["avgLong"]["rateSDay"] - 4.05) <= 0.3
      and d["jitterUs"] < u["jitterUs"])
sys.exit(0 if ok else 1)
' "$nh35_log"; then
    record dsp-host-drop-nh35 PASS ""
  else
    echo "--> FAIL  dsp-host-drop-nh35"
    show_tail "$nh35_log"
    record dsp-host-drop-nh35 FAIL "see $nh35_log"
  fi
fi

# Hybrid start (drop mode): the fit runs on unlock times while the templates
# learn, then its points are shifted onto the drop time base (not a cut). The
# first reading comes at about 6 s in both modes (provisional from a 5 s
# stretch) and is settled by 12 s, there is exactly one handover, and the
# settled average stays within 0.3 s/d of the designed +4.05 once it has 3 s
# of settled readings.
if [[ ! -f "$root/assets/synth_clips/nh35_21600.wav" ]]; then
  record dsp-host-handover-nh35 SKIP "missing assets/synth_clips/nh35_21600.wav"
elif [[ "$dsp_built" -ne 1 ]]; then
  echo "--> FAIL  dsp-host-handover-nh35: tests/dsp_host/build.sh failed"
  show_tail "$dsp_build_log"
  record dsp-host-handover-nh35 FAIL "build error (see $dsp_build_log)"
else
  handover_log="$out/logs/dsp-host-handover-nh35.log"
  if (cd "$root" &&
      for mode in unlock drop; do
        echo "== --timing $mode"
        "$out/dsp_host/dsp_wav" assets/synth_clips/nh35_21600.wav --timing "$mode" \
          --loops 10 --lift 52 --avg 20 --every 0.5 --metrics
      done) >"$handover_log" 2>&1 &&
     python3 -c '
import json, sys
first, rows, summ, mode, all_rows = {}, [], {}, None, {"unlock": [], "drop": []}
for l in open(sys.argv[1]):
    if l.startswith("== --timing "):
        mode = l.split()[-1]
    elif l.startswith("{\"file\""):
        first[mode] = json.loads(l)["first_valid_s"]
    elif l.startswith("{\"summary\""):
        summ[mode] = json.loads(l)
    elif l.startswith("{\"t\""):
        all_rows[mode].append(json.loads(l))
        if mode == "drop":
            rows.append(all_rows[mode][-1])
settled_t = {m: min([r["t"] for r in rws if r["settled"]] or [99])
             for m, rws in all_rows.items()}
late = [r for r in rows if r["avgLong"]["rateValid"] and r["settled"]
        and r["t"] >= settled_t["drop"] + 3]
ok = (first["drop"] <= first["unlock"] + 1.0 and summ["drop"]["handovers"] == 1
      and first["unlock"] <= 6.5 and first["drop"] <= 6.5
      and settled_t["unlock"] <= 12 and settled_t["drop"] <= 12
      and late and all(abs(r["avgLong"]["rateSDay"] - 4.05) <= 0.3 for r in late))
sys.exit(0 if ok else 1)
' "$handover_log"; then
    record dsp-host-handover-nh35 PASS ""
  else
    echo "--> FAIL  dsp-host-handover-nh35"
    show_tail "$handover_log"
    record dsp-host-handover-nh35 FAIL "see $handover_log"
  fi
fi

# Shape windows (drop mode, items 2 and 3 of the timing engine plan): per-side
# amplitude from averaged 2 s windows and a signed, unlock-corrected beat
# error. Tick is the side that is late in the drop beat error, so with the
# generated truth (tick = even k, BE positive when Tick is late) the labels
# swap when be + lift_tick - lift_tock < 0. Truth values are in the *.truth.json.
#
# Unequal lift (truth: tick 306.9 deg / tock 287.8 deg, BE +0.2 ms with tick
# lift 9.0 ms and tock 9.6 ms): the drop BE is -0.4 ms, so the firmware's Tick
# is the truth's Tock; the signed unlock beat error is -0.2 ms. Before the
# windows existed neither per-side amplitudes nor a signed figure were reported.
dsp_clip_row dsp-host-windows-lift-unequal assets/synth_clips/generated/g_21600_lift_unequal.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
ok = (last["amplitudeTickValid"] and last["amplitudeTockValid"]
      and abs(last["amplitudeTickDeg"] - 287.8) <= 3.0
      and abs(last["amplitudeTockDeg"] - 306.9) <= 3.0
      and last["windowsUsed"] >= 4
      and last["beatErrorSource"] == "unlock"
      and abs(last["beatErrorSignedMs"] + 0.2) <= 0.05
      and abs(last["avgLong"]["beatErrorMs"] - 0.2) <= 0.05)
sys.exit(0 if ok else 1)
' --timing drop --loops 1 --lift 52 --avg 20 --every 100

# Beat error -0.8 ms (Tick early): the late side is the truth's Tock, which
# the firmware labels Tick, so the signed figure reads +0.8; +0.3 reads +0.3.
for be in m0.8:0.8 p0.3:0.3; do
  dsp_clip_row "dsp-host-windows-be-${be%%:*}" "assets/synth_clips/generated/g_21600_be_${be%%:*}.wav" "
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith('{\"summary\"')][-1]
ok = (last['beatErrorSource'] == 'unlock'
      and abs(last['beatErrorSignedMs'] - ${be##*:}) <= 0.05
      and abs(last['avgLong']['beatErrorMs'] - ${be##*:}) <= 0.05
      and abs(last['amplitudeTickDeg'] - 297.0) <= 3.0
      and abs(last['amplitudeTockDeg'] - 297.0) <= 3.0)
sys.exit(0 if ok else 1)
" --timing drop --loops 1 --lift 52 --avg 20 --every 100
done

# Per-side amplitude within 3 deg of the 297.0 deg truth on clean, clipped,
# jittered and 20 dB SNR recordings.
for clip in clean clip30 jitter300 snr20; do
  dsp_clip_row "dsp-host-windows-amp-$clip" "assets/synth_clips/generated/g_21600_$clip.wav" '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
ok = (a["amplitudeValid"] and abs(a["amplitudeDeg"] - 297.0) <= 3.0
      and last["amplitudeTickValid"] and last["amplitudeTockValid"]
      and abs(last["amplitudeTickDeg"] - 297.0) <= 3.0
      and abs(last["amplitudeTockDeg"] - 297.0) <= 3.0
      and last["windowsUsed"] >= 3)
sys.exit(0 if ok else 1)
' --timing drop --loops 1 --lift 52 --avg 20 --every 100
done

# Recorded clips in drop mode: the window amplitude stays within the bounds the
# unlock-mode rows hold (ST3600, clipped, ChronoPulse 327 deg: +-15; NH35,
# ChronoPulse 261 deg: +-10), and the per-side figures exist.
dsp_clip_row dsp-host-windows-st3600 assets/synth_clips/st3600_21600.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (a["amplitudeValid"] and abs(a["amplitudeDeg"] - 327) <= 15
               and last["amplitudeTickValid"] and last["amplitudeTockValid"]) else 1)
' --timing drop --loops 4 --lift 52 --avg 20 --every 100
dsp_clip_row dsp-host-windows-nh35 assets/synth_clips/nh35_21600.wav '
import json, sys
last = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{\"summary\"")][-1]
a = last["avgLong"]
sys.exit(0 if (a["amplitudeValid"] and abs(a["amplitudeDeg"] - 261) <= 10
               and last["windowsUsed"] >= 5) else 1)
' --timing drop --loops 6 --lift 52 --avg 20 --every 100

# USB microphone mode (TIMING_ENGINE_PLAN.md section 8, U2): only tick
# detection and the A/h lock run. Recorded NH35 in drop mode, which would use
# stage B in standalone: locked at 21600, analysis off, no valid rate, beat
# error or amplitude, no beat records, no drop-timed beats, no shape windows.
dsp_clip_row dsp-host-usb-nh35 assets/synth_clips/nh35_21600.wav '
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
last = rows[-1]
a = last["avgLong"]
ok = (last["locked"] and last["vphNearest"] == 21600 and last["state"] == "locked"
      and all(r["analysis"] == "off" for r in rows) and last["analysis"] == "off"
      and not a["rateValid"] and not a["beatErrorValid"] and not a["amplitudeValid"]
      and last["beatsRecorded"] == 0 and last["dropTimedBeats"] == 0
      and last["windowsTotal"] == 0 and last["windowsUsed"] == 0
      and not last["amplitudeTickValid"] and not last["amplitudeTockValid"]
      and all(not r["rateValid"] for r in rows if not r.get("summary")))
sys.exit(0 if ok else 1)
' --mode usb --timing drop --loops 6 --lift 52 --avg 20 --every 5

# USB to standalone at 20 s: the engine starts clean (analysis on, the lock and
# the handover to drop times restart), and the rate settles within 0.3 s/d of
# the designed +4.05 in drop mode.
dsp_clip_row dsp-host-usb-switch-nh35 assets/synth_clips/nh35_21600.wav '
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
last = rows[-1]
a = last["avgLong"]
body = [r for r in rows if not r.get("summary")]
ok = (all(r["analysis"] == "off" and not r["rateValid"] for r in body if r["t"] <= 20)
      and last["analysis"] == "on" and last["locked"] and last["vphNearest"] == 21600
      and a["rateValid"] and abs(a["rateSDay"] - 4.05) <= 0.3
      and last["handovers"] == 1 and last["dropTimedBeats"] > 0)
sys.exit(0 if ok else 1)
' --mode usb --mode-at 20,standalone --timing drop --loops 10 --lift 52 --avg 20 --every 5

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
