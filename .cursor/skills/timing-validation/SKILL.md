---
name: timing-validation
description: Validates the Timegrapher Stand mechanical timing engine against ChronoPulse recordings in the host harness and on a Pico 2 W. Invoke explicitly for timing validation, unlock/drop comparisons, or verification of the DSP timing budget.
disable-model-invocation: true
---

# Timegrapher timing validation

Run from the Timegrapher Stand repository root. Read [the timing validation handoff](../../../docs/handoff/TIMING_VALIDATION_HANDOFF.md) for the fixtures, HID procedure, and acceptance limits. Check [the timing engine plan](../../../docs/plans/TIMING_ENGINE_PLAN.md) and prior [acceptance results](../../../docs/acceptance_results.md) for historical behavior. Record the current commit and dirty-tree state. Preserve existing work and test the current tree unless the user specifies a different revision.

- `$timing-validation` runs the host checks and proceeds to the hardware matrix when the board and device-write authorization are available.
- `$timing-validation host-only` runs the host checks and produces the host reference tables.
- `$timing-validation compile-only` checks PlatformIO with a fresh `pico2w_synthetic` build. A request to plan or review stops at the requested readback and checks.

## Ownership

Delegate host testing to `gpt-6.2-sol` (medium) and bench tooling preparation to `gpt-6.2-sol` (high) when delegation is available. These tasks can run in parallel. The host agent owns host results. The tooling agent owns any required changes to the bench validator and fixture-build preparation. The lead agent owns the physical board, all uploads and HID measurements, restoration, and final acceptance. Delegate difficult DSP or host/board divergence analysis to `gpt-6.2-astra` (high) after collecting evidence. Run the same phases directly if delegation is unavailable.

## Host reference

Use a C++17 compiler and a Python virtual environment with the dependencies in the handoff. Create a fresh run directory for host logs and JSON output. From the repository root, with `run_dir` set to that directory:

```sh
tests/run_all.sh "$run_dir/host"
.venv/bin/python scripts/synth_clips/compare.py --json "$run_dir/unlock.json" \
  --baseline tests/dsp_host/baseline_firmware.json --harness-args "--timing unlock"
.venv/bin/python scripts/synth_clips/compare.py --json "$run_dir/drop.json" \
  --harness-args "--timing drop" --no-otg
```

Verify the DSP host rows and confirm the comparisons use 52° lift and 20 s averaging. Preserve per-clip host results for each mode. Use the designed loop rate for steady-rate acceptance, ChronoPulse onset for the amplitude comparison, and host results for same-mode parity. Use generated scenarios with exact truth on the host. The original ChronoPulse recordings are needed only to regenerate the supplied loops.

## Board run

1. Apply the PlatformIO PIO Manager skill. Resolve the Pico 2 W's USB target explicitly. Ensure the HID client selects the same device, especially if multiple boards are present. Close competing control clients. Capture pre-run firmware identity, mode, settings, and the agreed restoration image before the first flash. The validator's `--restore-default` selects a plain synthetic build and its parameter snapshots occur after flashing.
2. Build the requested `TG_SYNTH_CLIP` variant with PIO Agent using an explicit environment and verify the selected artifact, including flash fit for the switch clip. Flash serially through PIO Agent with a resolved target and the applicable hardware-write authorization. Use the validator's `--no-flash` mode to measure each already-flashed clip. Its built-in upload helper invokes raw `pio` and relies on port autodetection.
3. Smoke-test `nh35_21600` in unlock and drop. Check `hello.source`, parameter write acknowledgments and readback, standalone mode, and the drop handover after template learning. Use the handoff's reset, wait, and sampling cadence. Verify that all samples entering the medians are settled, locked at the expected bph, and valid. Collect per-mode DSP and loop maxima with a known counter reset or measurement scope. If needed, fix bench issues around sample validity, switch-clip CPU capture, or cleanup after failure before starting the full matrix.
4. Run the nine steady clips in both modes and log the switch clip in both modes for the full observation period. Preserve raw samples and JSONL in the run directory, using a fresh file each run. Restore the agreed image and settings and verify restoration after completion or recoverable failure.

## Acceptance and reporting

Use the handoff's pass criteria. For steady clips, check lock, settled state, expected bph, analysis on, rate within ±0.3 s/d of loop truth, amplitude within ±15° of ChronoPulse onset and approximately 1° of the matching host mode, and beat error within approximately 0.05 ms of the host. Record ChronoPulse beat error for context. Check DSP block max below 5,000 µs and loop max below approximately 8,000 µs. Remeasure the historical drop-mode overrun. Verify that the switch clip relocks in both directions without mixing watch readings and that settled readings at 28,800 bph stay within 2 s/d of approximately +3.3 s/d.

Append a dated entry to `docs/acceptance_results.md` with the tested revision and working-tree state, board and environment, settings, per-mode results, CPU maxima, switch evidence, and raw data paths. Give numerical details for failed criteria. Mark blocked or skipped checks as unverified, and report any restoration issue.
