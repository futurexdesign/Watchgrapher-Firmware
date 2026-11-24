# Handoff: resume the timing-engine plan

| | |
|---|---|
| **Written** | 2026-11-24, after the user paused the work on 2026-11-23 ("pause for now") |
| **Plan** | [`docs/plans/TIMING_ENGINE_PLAN.md`](../plans/TIMING_ENGINE_PLAN.md): read it first. §3 has the items, §4 the task table, §6 the user's decisions, §8 the USB-mode request |
| **Hardware check** | [`TIMING_VALIDATION_HANDOFF.md`](TIMING_VALIDATION_HANDOFF.md): how to check readings on the board against the recorded clips |
| **Branch** | `BLE`. **Nothing is committed** and nothing may be: the user commits. `git add -N` on new files is fine |

This file tells you what is done, what is half done, and what is left, in the order to do it. It also covers how to check each step and what tripped up the previous session.

---

## 1. Ground rules (from the user)

- **Never commit, and never offer to.** Leave the changes in the working tree.
- **Manager workflow.** The manager agent (Opus) splits the work, gives tasks to sub-agents and reviews every one before accepting it:
  - **Sonnet** for algorithms, firmware and non-trivial UI; **Haiku** for doc syncs, CI YAML and simple cards.
  - **Review for:** bugs, house style, full Javadoc-style `/** */` comments on every function and constant, modern C++17 on the firmware and strict TypeScript (no `any`) on the web.
  - **Kick back** with concrete findings until a task passes, then run the builds and tests yourself. Don't trust a sub-agent's "all tests pass".
- **Don't change sub-agents' work in parallel.** Two agents editing `beat_analysis.cpp` at once made one report false failures.
- **Code voice.** Match the surrounding files:
  - Short declarative comments that explain *why*.
  - `{@link}` / `{@code}` in doc comments.
  - Constants in `kCamelCase` with units in the name (`kSearchMs`).
  - No `std::` containers that allocate on the firmware.
- **Protocol.** Additions only, as new fields; `protocolVersion` stays 1. Every new `getBeat` field goes into `docs/HID_CONTROL_PLANE.md`, `docs/hid/timegrapher_hid.d.ts` and, where relevant, `docs/BLE_CONTROL_PLANE.md`.

## 2. State when paused

### Accepted (built, tested and reviewed)

| Plan item | What exists | Where |
|---|---|---|
| Phase 0 | Generated scenarios with exact truth (20), harness flags `--metrics` / `--beats` / `--timing` / `--mode` / `--mode-at` / `--settle` / `--ppm-at`, comparison table, OTG reference, firmware baseline | `scripts/synth_clips/{gen_ticks,compare,reference_otg}.py`, `assets/synth_clips/generated/`, `tests/dsp_host/`, `tests/dsp_host/baseline_firmware.json` |
| 1: drop timing | Stage B: 1.5 kHz HP envelope at 24 kHz into an 82 ms ring, per-side templates, NCC drop timing, quality gate 0.4. Parameter `timingLandmark` (0 = unlock, 1 = drop); **the default is still unlock** | `src/mechanical/tick_shape.{h,cpp}` (header in `include/mechanical/`), `beat_analysis.cpp` |
| 4: cuts and no watch | Cut detector, `state: "noWatch"`, provisional readings from 5 s, `settled` at 10 s, `settleS` | `beat_analysis.cpp` |
| Hybrid handover (§6) | Drop mode starts on unlock times and hands over on the fly by shifting the fit's points onto the drop time base; this is not a cut | `beat_fit_shift_points` in `src/mechanical/beat_fit.cpp` |
| 2 + 3 | Per-side 2 s shape windows, half-height landmarks, per-side amplitude, signed unlock-corrected BE with a drop-based fallback | `tick_shape.cpp` windows, `beat_analysis.cpp` |
| 8 | BE angle; rate formula changed to `(nominal/P − 1)·86400` | `beat_analysis.cpp` |
| RAM decision | Matched filter retired (`getMatched` is gone); BLD-02 raised to 60 % (314,572 B on the W builds); the JSON `beats` stream stays | `dsp_graph.cpp`, `dsp_blocks.cpp`, `dsp_types.h` |
| U1 (OLED) | USB mic mode: "USB AUDIO" fills the main section and the status bar is unchanged. `DISPLAY_ROTATION` defaults to 3 (90° CCW); the renderer uses 0 | `src/display_ui.cpp` (`draw_usb_audio`), `include/board_config.h`, `docs/DISPLAY.md`, `docs/images/oled/` |
| Radio core | Core 1 is the BLE core for all W builds; `pico2w_synthetic_core1` was removed | `platformio.ini` |
| Web W7a | State, `settled`, BE angle and timing chip; matched-filter UI removed | `../timegrapher-web` (`BeatDiagnosticsSection.vue` and others) |

### Delivered but NOT verified: U2 (plan §8)

In USB microphone mode only tick detection and the beat-rate lock run. The engine, stage B, windows and quartz engine are off, and `getBeat` / `getQuartz` report `analysis: "off"`.

The code is in the tree:
- `dsp_graph_set_analysis` in `src/dsp_graph.cpp:604`, called from `src/device_mode.cpp:144,170`;
- the `if (!g_analysis)` early return in `beat_analysis_on_event`;
- `quartz::engine::set_analysis`.

Its agent reported all host tests passing. The manager's own check was cut short by the pause, so **treat U2 as unreviewed.**

### Board state

Last flashed with an **older** `nh35_21600` clip build on core 1, so it does not have U2 or the latest changes. Reflash before any board measurement.

## 3. What to do, in order

### Step 1: verify U2 (manager, no sub-agent unless it fails)

1. Run `tests/run_all.sh`. Expect every row PASS or SKIP. Check the **exit code**, not just the tail: the previous session misread a blank tail as a pass once.
2. Build every CI firmware env:
   ```sh
   for e in pico2_pcm1808 kiwi_pcm1808 pico2 pico2_synthetic pico2_pcm1808_cli pico2w_pcm1808 pico2w_synthetic; do
     pio run -e $e >/dev/null 2>&1 && echo "ok $e" || echo "FAIL $e"
   done
   ```
   Record RAM per env from the build output, against BLD-02.
3. Run the static checks the way CI does (`.github/workflows/ci.yml`, job "static analysis"):
   - `git clang-format --diff` over `src include tests`;
   - `cppcheck --enable=warning,performance,portability --inline-suppr --std=c++17 -I include` with the same `-i` exclusions as CI.
4. Review the U2 diff:
   - **Switching to standalone** must restart the engine cleanly, like a cut: no stale fit, windows or templates.
   - **Switching to USB mode** must keep `locked` / `vphNearest` live.
   - **The playback (USB stream) filters** must be untouched.
   - **`analysis: "off"`** must be documented in the HID doc and the d.ts.
   - **A host test** must show that in USB mode there is no rate, BE or amplitude, but lock and bph are present (look for the `--mode` / `--mode-at` rows in `run_all.sh`).
5. **On the board**, in USB mode: check that `getBeat.analysis == "off"` while it is locked, and that `getTiming.dspBlockUsMax` is lower than in standalone. Check that the OLED shows USB AUDIO.

### Step 2: performance task (sonnet, must pass before drop timing becomes the default)

**Problem, as measured on the board** (`nh35_21600` clip, 2026-11-23, `getTiming`):

| Mode | Worst DSP block | Budget |
|---|---|---|
| Unlock timing | 1,673 µs | 5,000 µs |
| Drop timing | **5,517 µs** | 5,000 µs |

The worst loop must also stay under about 8,000 µs (the USB audio FIFO slack, 2352 B).

**Where the time goes.** The cost is per beat, inside the DSP block that closes the event:

- `beat_analysis_on_event` calls `tick_shape_measure` (`tick_shape.cpp:898`), which calls `match_template`:
  - a coarse pass of 49 lags × 336 products, plus 5 fine lags × 672;
  - the per-side window work (`tick_shape_window_add` / `_step` / landmark search);
  - on a learning beat, the template fold.
- The per-sample cost in `tick_shape_push` is two float biquads, a boxcar and decimation. Measure it, but it is unlikely to be the peak.

**Measure first.** Get a per-stage breakdown before optimising. Options:
- temporary `micros()` stamps around `tick_shape_measure`, the window step and the fit, reported through `getTiming`;
- or `--metrics` CPU ns on the host. Host ratios are only a proxy; the board number is the one that counts.

**Candidate fixes, roughly cheapest first:**
1. **Narrower search.** Search ±0.5 ms around the predicted drop (the fit's predicted time plus the side's template offset) instead of ±2 ms. Widen only when quality falls below the gate.
2. **Move work out of the closing block.** Run window landmark finding (`tick_shape_window_step`) from `beat_analysis_on_block_end` or a later block. The `finish_pending_window` plumbing already supports deferring it.
3. **Faster MACs.** Use SMLAD-style packed q15 dot products for the int16 × int16 correlation (`__SMLAD` / CMSIS `arm_dot_prod_q15`). Keep a scalar fallback for the host build.
4. **Put the hot loops in RAM.** Mark `match_template`, `ncc_full` and the push loop `__not_in_flash_func` to avoid XIP cache misses.
5. **Integer biquads.** Only if the per-sample push shows up in the breakdown.

**Acceptance:**
- Board drop mode: worst DSP block < 5,000 µs, with margin (aim < 3,500 µs), and worst loop < 8,000 µs, on at least `nh35_21600`, `r3235_starbucks_28800` (fastest beat) and `st3600_21600`.
- Host: `run_all.sh` all pass. `compare.py` readings unchanged within ±0.05 s/d rate, ±1° amplitude and ±0.02 ms BE.
- Board readings in drop mode unchanged (rate +4.05 on `nh35_21600`, jitter about 51 µs).

### Step 3: make hybrid drop timing the default

This waits for Step 2. Then:
- change the `timingLandmark` default to drop: `g_timing_drop` is set in two places in `beat_analysis.cpp`, its initialiser (line 210) and the parameter reset (line 924);
- update the docs that state the default (HID doc, d.ts, plan §6);
- update any `run_all.sh` rows that assume unlock;
- recheck the comparison table.

The hybrid path starts on unlock times and hands over once templates are learnt, so first readings are not delayed. Check that on the switch clip (`switch_21600_28800`).

### Step 4: item 5, span fit by sufficient statistics (sonnet)

The plan's §3 item 5 holds the design. In short:
- per-1 s block sums for `t = a + P·k + c·s`;
- the span rate is one exact solve over the blocks in the span and the current stretch, never across a cut (item 4's cuts already exist);
- 10 s windows every 5 s for the ± and the 5–95 % spread;
- new `getBeat.avgLong` fields `rateUncertaintySDay`, `rateP5SDay`, `rateP95SDay`, `beatsUsed`;
- the existing 6 s sliding fit stays for residuals and outlier rejection.

**Check:** generated scenarios with a known rate and drift give a span rate error under 0.05 s/d, and the ± covers the truth about 68 % of the time across seeds. **RAM:** 1 to 4.3 KB, checked against BLD-02.

### Step 5: item 6, knock gate and detectFilter front end (sonnet)

The plan's §3 item 6 holds the design. In short:
- In stage B, reject a beat whose envelope energy exceeds 2× the running median of accepted beats.
- Let stage A's threshold run on the envelope; keep this switchable for A/B.
- Bring in `BLE_TRANSPORT_PLAN.md` §13's `detectFilter` (notch + linear-phase band-pass) as the configurable front end; A11 is superseded.
- **Check:** with the generated `g_21600_knocks` and `g_21600_hum60` scenarios, no knock is timed as a beat and the rate is unaffected.

### Step 6: item 7, support figures (haiku)

Add `beatsUsed`, `beatsExpected` and `windowsUsed` to `getBeat` (most come from items 2 and 5), with docs and d.ts.

### Step 7: web W7b (sonnet, in `../timegrapher-web`)

- Show per-side amplitude (`amplitudeTickDeg` / `amplitudeTockDeg`), signed BE with its source (`beatErrorSignedMs`, `beatErrorSource`), and the ± and spread from Step 4.
- Show the support chips from Step 6.
- When `analysis: "off"`, replace the readings with "USB Audio mode: timing runs on the host".
- Update the virtual device (`src/sim/VirtualDevice.ts`) to emit the new fields.
- **Verify:** `npm run lint && npm run typecheck && npm test && npm run build`, checking **exit codes**. The previous session broke web CI by reading a quiet `tail` as a lint pass.

### Step 8: loose ends

- **OLED rotation:** confirm `DISPLAY_ROTATION` 3 is the right way up on the real panel, in the case. If it is upside down, the fix is 1.
- **Hardware run:** run `scripts/bench/validate_clips_hw.py` end to end on the board (see `TIMING_VALIDATION_HANDOFF.md`). It has only been syntax-checked so far, and its first run is part of the job.
- **Final record:** after Steps 2–3, re-run the full clip validation in both timing modes and record it in `docs/acceptance_results.md`.
- **Memory note:** update the timing-engine status note for the user, if you keep one.

## 4. How to measure

- **Host:** `tests/run_all.sh` runs every row (about 51). For readings against truth and ChronoPulse:
  - `python3 scripts/synth_clips/compare.py` gives the before/after table every firmware task must include in its report;
  - `tests/dsp_host` builds `dsp_wav`, for example `dsp_wav assets/synth_clips/nh35_21600.wav --timing drop --metrics`.
- **Board build:** `PLATFORMIO_BUILD_FLAGS=-DTG_SYNTH_CLIP=<id> pio run -e pico2w_synthetic -t upload` loops a recorded clip. `hello.source` then reads `SYNTHETIC:<id>`.
- **Board control:** `scripts/bench/tgclient.py` (HID: VID 0xCAFE, PID 0x4003):
  - send `claim` with `takeover`, then `getBeat` / `getTiming`;
  - `patchState` needs the wrapper: `patch={"blocks":{"beatRate":{"params":{...}}}}`;
  - **check `ok` on every reply.**
- **CPU on the board:** `getTiming.dspBlockUsMax` and `loopUsMax`. Call `resetTiming` after changing settings, then let it run at least 40 s.

## 5. Pitfalls the previous session hit

- **`patchState` without the `patch` wrapper** returns `badRequest`. An unchecked reply meant a whole run measured the wrong settings.
- **Reflashing resets settings** (lift angle, averaging time, `timingLandmark`). Set them again after every upload.
- **USB mic mode now turns the engine off.** Measure timing in standalone (`setDevice mode=standalone`, then `start`).
- **Rate truth** for a clip is its designed loop rate in `clips.json`, not ChronoPulse. ChronoPulse's rate drifts with the original recordings. The switch clip has no designed rate.
- **No board found on upload.** If `ls /dev/cu.usbmodem*` finds nothing, PlatformIO falls back to a Bluetooth serial port and hangs. Check first.
- **Arm vs host types:** `std::max(int, int32_t)` builds on the host and fails on Arm. Build a firmware env, not just the host tests.
- **zsh traps:** `noclobber` and an interactive `cp` alias block overwrites. Use `>|` and `/bin/cp -f`.
- **Messages to running sub-agents can be missed.** If a follow-up matters, make it a new task.

## 6. Key files

| Area | Files |
|---|---|
| Stage A detection | `src/mechanical/tick_detect.cpp` |
| Stage B, templates, windows | `src/mechanical/tick_shape.cpp`, `include/mechanical/tick_shape.h` |
| Fit, handover | `src/mechanical/beat_fit.cpp`, `include/mechanical/beat_fit.h` |
| Readings, cuts, lock, params, analysis switch | `src/mechanical/beat_analysis.cpp`, `include/mechanical/beat_analysis.h` |
| Graph glue, block timing | `src/dsp_graph.cpp` (`analyze()`, `dsp_graph_set_analysis`) |
| Mode switching | `src/device_mode.cpp` |
| Protocol fields | `src/control/control_handler.cpp` (`getBeat`), `docs/HID_CONTROL_PLANE.md`, `docs/hid/timegrapher_hid.d.ts` |
| Host harness, tests | `tests/dsp_host/`, `tests/run_all.sh`, `tests/test_beat_math.cpp` |
| Clips, truth | `assets/synth_clips/` (`clips.json`, README, `generated/`) |
| Web | `../timegrapher-web/src/components/timegrapher/`, `src/sim/VirtualDevice.ts` |
