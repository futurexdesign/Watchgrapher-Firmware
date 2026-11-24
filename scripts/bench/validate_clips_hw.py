#!/usr/bin/env python3
"""Validates the timing engine on hardware against the recorded watch clips.

For each clip in assets/synth_clips/clips.json: builds a synthetic firmware
that loops the clip (``-DTG_SYNTH_CLIP=<id>``), flashes it over the USB CDC
port (1200-baud reset into BOOTSEL), then over USB HID: checks the clip is
running (``hello.source``), sets lift angle / averaging / timing landmark,
runs the device standalone, waits for a settled 20 s average and samples
``getBeat`` and ``getTiming``. Prints one JSON line per clip and timing mode
and a comparison table against the clip's designed loop rate and the
ChronoPulse reference. After each clip, the validator restores the device's
original beatRate parameters, operating mode, and run latch even if a
measurement fails. Changing operating mode writes to flash, which may
save a previously unsaved mode when it is restored.

Use ``--no-flash --clips <id>`` with firmware that was uploaded separately.
In the legacy upload workflow, ``--restore-default`` uploads the plain build
after the measurements finish. If an upload or measurement aborts the run,
upload the intended firmware separately to recover from the clip build.

See docs/handoff/TIMING_VALIDATION_HANDOFF.md for the procedure, the
tolerances and how to read the results.

Usage (from the repo root, with a venv holding hidapi):
  .venv/bin/python scripts/bench/validate_clips_hw.py [--clips id ...]
      [--timing unlock|drop|both] [--env pico2w_synthetic] [--lift 52]
      [--avg 20] [--wait 48] [--no-flash] [--restore-default] [--out file]
"""
import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tgclient import HidClient  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CLIPS_JSON = os.path.join(REPO, "assets", "synth_clips", "clips.json")
TIMING = {"unlock": 0, "drop": 1}
BEAT_PARAMS = ("liftAngle", "avgS", "timingLandmark")


def flash(env, clip_id, build_dir):
    """Builds and uploads the clip firmware; raises with the log tail on failure."""
    flags = f"-DTG_SYNTH_CLIP={clip_id}" if clip_id else ""
    cmd_env = dict(os.environ, PLATFORMIO_BUILD_DIR=build_dir, PLATFORMIO_BUILD_FLAGS=flags)
    r = subprocess.run(["pio", "run", "-e", env, "-t", "upload"], cwd=REPO, env=cmd_env,
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"upload failed for {env} (exit {r.returncode}):\n"
                           + (r.stdout + r.stderr)[-1500:])


def cleanup(action, description):
    """Runs a cleanup action without hiding an error already in flight."""
    handling_error = sys.exc_info()[0] is not None
    try:
        action()
    except Exception as error:
        if not handling_error:
            raise
        print(f"{description} failed: {error}", file=sys.stderr)


def connect(timeout_s=40.0):
    """Opens the HID control interface once the device has enumerated."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            c = HidClient()
            try:
                ok(c.request("claim", takeover=True), "claim")
            except BaseException:
                cleanup(c.close, "closing HID client after claim failure")
                raise
            return c
        except (StopIteration, OSError, TimeoutError) as error:
            if time.monotonic() >= deadline:
                raise RuntimeError("could not open and claim the HID device") from error
            time.sleep(1.0)


def ok(reply, what):
    """Returns the result of a reply, or raises with the device's error."""
    if not reply.get("ok"):
        raise RuntimeError(f"{what} failed: {reply.get('error')}")
    return reply.get("result")


def beat_params(c):
    """Reads back the live beatRate parameters."""
    return ok(c.request("getState"), "getState")["blocks"]["beatRate"]["params"]


def finite_number(value):
    """Checks that a JSON value is a number that can be used in a reading."""
    return type(value) is int or (type(value) is float and math.isfinite(value))


def required_param(params, name):
    """Reads a numeric parameter and rejects a missing or invalid value."""
    value = params.get(name)
    if not finite_number(value):
        raise RuntimeError(f"invalid readback for beatRate.{name}: {value!r}")
    return value


def set_beat_params(c, params):
    """Patches beatRate and verifies every changed parameter by reading it back."""
    ok(c.request("patchState", patch={"blocks": {"beatRate": {"params": params}}}), "patchState")
    now = beat_params(c)
    for name, expected in params.items():
        actual = required_param(now, name)
        # The device prints six significant digits and rounds near-integers.
        if not math.isclose(actual, expected, rel_tol=5e-6, abs_tol=1e-4):
            raise RuntimeError(f"param {name} did not apply: wanted {expected}, device has {actual}")


def device_status(c):
    """Reads and validates the operating mode and capture run latch."""
    status = ok(c.request("getDevice"), "getDevice")
    mode = status.get("mode")
    run_requested = status.get("runRequested")
    if mode not in ("standalone", "usbMic") or type(run_requested) is not bool:
        raise RuntimeError(f"invalid readback for getDevice: {status}")
    return mode, run_requested


def set_device_status(c, current, target):
    """Changes and verifies the operating mode and run latch as needed."""
    mode, run_requested = target
    if current[0] != mode:
        ok(c.request("setDevice", mode=mode), "setDevice")
        current = device_status(c)
    if current[1] != run_requested:
        op = "start" if run_requested else "stop"
        ok(c.request(op), op)
        current = device_status(c)
    if current != target:
        raise RuntimeError(f"device settings did not apply: wanted {target}, device has {current}")


def median(values):
    """Returns the rounded median of the available numeric readings."""
    vals = [v for v in values if finite_number(v)]
    return round(statistics.median(vals), 3) if vals else None


def valid_reading(beat, field, flag):
    """Returns a finite reading only when its corresponding flag is true."""
    value = beat.get(field)
    return value if beat.get(flag) is True and finite_number(value) else None


def sample_beat(c, started_at):
    """Captures the raw result and exposes lock and long-average validity alongside it."""
    beat = ok(c.request("getBeat"), "getBeat")
    avg = beat["avgLong"]
    return {
        "t": round(time.monotonic() - started_at, 3),
        "locked": beat.get("locked"), "state": beat.get("state"), "settled": beat.get("settled"),
        "analysis": beat.get("analysis"), "timingSource": beat.get("timingSource"),
        "vph": beat.get("vphNearest"),
        "rateValid": avg.get("rateValid"), "beatErrorValid": avg.get("beatErrorValid"),
        "amplitudeValid": avg.get("amplitudeValid"),
        "rate": valid_reading(avg, "rateSDay", "rateValid"),
        "be": valid_reading(avg, "beatErrorMs", "beatErrorValid"),
        "amp": valid_reading(avg, "amplitudeDeg", "amplitudeValid"),
        "beat": beat,
    }


def sample_locked(sample):
    """Checks both the nominal-rate lock flag and the timing-engine state."""
    return sample["locked"] is True and sample["state"] == "locked"


def measure(c, wait_s, expected_vph, mode, samples=5, every_s=2.0):
    """Samples settled timing and reports whether every sample met the steady-clip checks."""
    started_at = time.monotonic()
    time.sleep(wait_s)
    rows = []
    for i in range(samples):
        if i:
            time.sleep(every_s)
        rows.append(sample_beat(c, started_at))
    last = rows[-1]
    ready = [
        r for r in rows
        if sample_locked(r) and r["settled"] is True and r["analysis"] == "on"
        and r["vph"] == expected_vph and r["timingSource"] == mode
    ]
    beats = [r["beat"] for r in ready]
    all_samples_valid = len(ready) == len(rows) and all(
        r["rate"] is not None and r["be"] is not None and r["amp"] is not None for r in ready
    )
    return {
        "locked": last["locked"], "state": last["state"], "settled": last["settled"],
        "analysis": last["analysis"], "timingSource": last["timingSource"], "vph": last["vph"],
        "expected_vph": expected_vph,
        "all_samples_locked": all(sample_locked(r) for r in rows),
        "all_samples_settled": all(r["settled"] is True for r in rows),
        "all_samples_valid": all_samples_valid,
        "rate": median(r["rate"] for r in ready),
        "be": median(r["be"] for r in ready),
        "be_signed": median(valid_reading(b, "beatErrorSignedMs", "beatErrorValid") for b in beats),
        "be_source": last["beat"].get("beatErrorSource"),
        "amp": median(r["amp"] for r in ready),
        "amp_tick": median(b.get("amplitudeTickDeg") for b in beats),
        "amp_tock": median(b.get("amplitudeTockDeg") for b in beats),
        "jitter_us": median(b.get("jitterUs") for b in beats),
        "samples": rows,
    }


def follow(c, seconds, every_s=1.0):
    """Logs each watch-change sample, including provisional and unlocked readings."""
    log = []
    started_at = time.monotonic()
    end_at = started_at + seconds
    while time.monotonic() < end_at:
        log.append(sample_beat(c, started_at))
        time.sleep(max(0.0, min(every_s, end_at - time.monotonic())))
    return log


def timing_stats(c):
    """Captures CPU counters and keeps the summary fields used by the bench table."""
    timing = ok(c.request("getTiming"), "getTiming")
    stats = {"timing_stats": timing}
    for field, output in (("dspBlockUsMax", "dsp_us_max"), ("loopUsMax", "loop_us_max")):
        value = timing.get(field)
        if not finite_number(value) or value < 0:
            raise RuntimeError(f"invalid readback for getTiming.{field}: {value!r}")
        stats[output] = value
    return stats


def main():
    """Validates the requested clips and returns a failure status for invalid steady samples."""
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--clips", nargs="+", help="clip ids (default: all in clips.json)")
    ap.add_argument("--timing", choices=["unlock", "drop", "both"], default="both")
    ap.add_argument("--env", default="pico2w_synthetic", help="PlatformIO env with AUDIO_SOURCE=SYNTHETIC")
    ap.add_argument("--lift", type=float, default=52.0, help="lift angle (ChronoPulse reference: 52)")
    ap.add_argument("--avg", type=float, default=20.0, help="averaging time s (ChronoPulse reference: 20)")
    ap.add_argument("--wait", type=float, default=48.0, help="seconds after start before sampling")
    ap.add_argument("--no-flash", action="store_true", help="use the firmware on the board (one clip)")
    ap.add_argument("--restore-default", action="store_true", help="flash the plain build at the end")
    ap.add_argument("--build-dir", default=os.path.join(tempfile.gettempdir(), "tg_clip_build"))
    ap.add_argument("--out", help="also append the JSON lines to this file")
    args = ap.parse_args()

    if args.no_flash and (args.clips is None or len(args.clips) != 1):
        ap.error("--no-flash requires exactly one clip id via --clips")
    if args.no_flash and args.restore_default:
        ap.error("--restore-default cannot be used with --no-flash")
    if not (math.isfinite(args.lift) and math.isfinite(args.avg)):
        ap.error("--lift and --avg must be finite")
    if not math.isfinite(args.wait) or args.wait < 0:
        ap.error("--wait must be a finite, nonnegative duration")

    with open(CLIPS_JSON) as clip_file:
        clips = json.load(clip_file)["clips"]
    if args.clips:
        unknown = sorted(set(args.clips) - {c["id"] for c in clips})
        if unknown:
            ap.error("unknown clip id: " + ", ".join(unknown))
        clips = [c for c in clips if c["id"] in args.clips]
    modes = ["unlock", "drop"] if args.timing == "both" else [args.timing]
    results = []
    for clip in clips:
        cid = clip["id"]
        if not args.no_flash:
            flash(args.env, cid, args.build_dir)
            time.sleep(4.0)
        c = connect()
        original = None
        original_device = None
        restore_needed = False
        try:
            source = ok(c.request("hello"), "hello")["source"]
            if source != f"SYNTHETIC:{cid}":
                raise RuntimeError(f"board runs {source!r}, expected SYNTHETIC:{cid}")
            saved = beat_params(c)
            original = {name: required_param(saved, name) for name in BEAT_PARAMS}
            original_device = device_status(c)
            set_device_status(c, original_device, ("standalone", True))  # USB mode turns analysis off
            for mode in modes:
                restore_needed = True
                set_beat_params(c, {"liftAngle": args.lift, "avgS": args.avg, "timingLandmark": TIMING[mode]})
                ok(c.request("resetTiming"), "resetTiming")
                row = {"clip": cid, "timing": mode, "lift": args.lift, "avgS": args.avg,
                       "expected_rate": (clip.get("expected") or {}).get("rate_s_day"),
                       "chronopulse": clip.get("chronopulse")}
                if clip.get("segments"):
                    row["log"] = follow(c, 75.0)
                else:
                    row.update(measure(c, args.wait, clip["bph"], mode))
                row.update(timing_stats(c))
                line = json.dumps(row)
                print(line, flush=True)
                if args.out:
                    with open(args.out, "a") as f:
                        f.write(line + "\n")
                results.append(row)
        finally:
            try:
                if restore_needed:
                    cleanup(lambda: set_beat_params(c, original), "restoring beatRate parameters")
            finally:
                try:
                    if original_device is not None:
                        cleanup(lambda: set_device_status(c, device_status(c), original_device),
                                "restoring device mode and run latch")
                finally:
                    cleanup(c.close, "closing HID client")
    if args.restore_default:
        flash(args.env, None, args.build_dir)

    print("\n| clip | timing | rate | loop | err | CP rate | BE | CP BE | amp | CP onset | tick/tock | "
          "dsp max us | loop max us | all samples valid |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in results:
        if "log" in r:
            continue
        cp = r.get("chronopulse") or {}
        err = (None if r["rate"] is None or r["expected_rate"] is None
               else round(r["rate"] - r["expected_rate"], 2))
        print(f"| {r['clip']} | {r['timing']} | {r['rate']} | {r['expected_rate']} | {err} | "
              f"{cp.get('rate_s_day')} | {r['be']} | {cp.get('beat_error_ms')} | {r['amp']} | "
              f"{cp.get('amp_onset_deg')} | {r['amp_tick']}/{r['amp_tock']} | {r['dsp_us_max']} | "
              f"{r['loop_us_max']} | {r['all_samples_valid']} |")
    return int(any(r.get("all_samples_valid") is False for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
