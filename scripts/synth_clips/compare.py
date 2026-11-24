#!/usr/bin/env python3
"""Compare the firmware timing chain against truth, ChronoPulse and the OTG reference.

Runs the host harness (tests/dsp_host/dsp_wav, the real firmware chain) with
--metrics over every assets/synth_clips/*.wav and
assets/synth_clips/generated/*.wav, optionally runs
scripts/synth_clips/reference_otg.py on the same files, and prints one
markdown table per metric family. Phase 0b of docs/plans/TIMING_ENGINE_PLAN.md.

Reference values per file:
  * generated/<name>.wav: <name>.truth.json, the segment covering the end of
    the file (steady state);
  * the firmware and the reference call the side that is late in the drop beat
    error "Tick" (OTG M5), which can be the truth's Tock; the per-side and
    signed beat-error tables map the truth to that labelling
    (truth_late_tick, truth_be_signed);
  * recorded clips: clips.json (designed loop rate and beat error estimate,
    ChronoPulse rate, beat error and onset amplitude). The "truth" columns
    hold the designed loop rate; ChronoPulse has its own columns.

Usage:
  compare.py [HARNESS] [--json out.json] [--baseline old.json]
             [--only SUBSTR] [--no-otg] [--otg-python PY]
             [--harness-args '--timing drop']

HARNESS is the dsp_wav binary; without it the harness is built with
tests/dsp_host/build.sh into a temporary directory. Needs only the standard
library (reference_otg.py may need numpy; pass --otg-python to select the
interpreter that has it).
"""

import argparse
import json
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CLIPS = ROOT / "assets" / "synth_clips"
GENERATED = CLIPS / "generated"
OTG = Path(__file__).resolve().parent / "reference_otg.py"

LIFT_DEG = 52
AVG_S = 20
LOOPS_WHOLE_BEAT = 10  # recorded loops hold a whole number of beats
LOOPS_ONCE = ("switch_21600_28800",)  # one pass: the loop is a watch change


def build_harness(out_dir):
    """Builds dsp_wav into out_dir and returns its path."""
    subprocess.run([str(ROOT / "tests" / "dsp_host" / "build.sh"), str(out_dir)],
                   check=True)
    return Path(out_dir) / "dsp_wav"


def loops_for(wav):
    """Number of times the harness replays a file."""
    if wav.parent == GENERATED or wav.stem in LOOPS_ONCE:
        return 1
    return LOOPS_WHOLE_BEAT


def run_firmware(harness, wav, extra_args=()):
    """Runs the harness on wav; returns its Result dict or None on failure.

    extra_args are appended to the harness command line (for example
    ["--timing", "drop"]).
    """
    cmd = [str(harness), str(wav), "--metrics", "--lift", str(LIFT_DEG),
           "--avg", str(AVG_S), "--loops", str(loops_for(wav)), "--every", "1000",
           *extra_args]
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if proc.returncode != 0:
        print(f"warning: harness failed on {wav.name}: {proc.stderr.strip()}",
              file=sys.stderr)
        return None
    return last_result(proc.stdout, "firmware")


def run_otg(python, wav):
    """Runs reference_otg.py on wav; returns its Result dict or None."""
    proc = subprocess.run([python, str(OTG), str(wav)], capture_output=True,
                          text=True)
    if proc.returncode != 0:
        print(f"warning: reference_otg failed on {wav.name}: "
              f"{proc.stderr.strip()[-200:]}", file=sys.stderr)
        return None
    return last_result(proc.stdout, "otg_ref")


def last_result(text, method):
    """Returns the last JSON line of text whose method matches, else None."""
    found = None
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if obj.get("method") == method:
            found = obj
    return found


def load_clips_json():
    """Returns {clip id: entry} from clips.json."""
    data = json.loads((CLIPS / "clips.json").read_text())
    return {c["id"]: c for c in data["clips"]}


def reference_for(wav, clips):
    """Returns the reference dict for a file.

    Keys (all optional, None when unknown): truth_rate, truth_be,
    truth_amp, truth_amp_tick, truth_amp_tock, truth_bph, cp_rate, cp_be,
    cp_amp, kind ("truth" or "clip").
    """
    ref = {"kind": "clip"}
    truth_file = wav.with_suffix(".truth.json")
    if wav.parent == GENERATED and truth_file.exists():
        t = json.loads(truth_file.read_text())
        segs = t.get("segments", [])
        if segs:
            seg = max(segs, key=lambda s: s.get("to_s", 0.0))
            ref.update(kind="truth", truth_rate=seg.get("rate_s_day"),
                       truth_be=abs(seg["be_ms"]) if "be_ms" in seg else None,
                       truth_amp=seg.get("amp_deg"),
                       truth_amp_tick=seg.get("amp_tick_deg"),
                       truth_amp_tock=seg.get("amp_tock_deg"),
                       truth_bph=seg.get("bph"))
            if "be_ms" in seg and "lift_ms_tick" in seg:
                # drop beat error = unlock beat error + lift_tick - lift_tock
                late = seg["be_ms"] + seg["lift_ms_tick"] - seg["lift_ms_tock"] >= 0
                ref["truth_late_tick"] = late
                ref["truth_be_signed"] = seg["be_ms"] if late else -seg["be_ms"]
                ref["truth_amp_tick_lab"] = seg["amp_tick_deg"] if late else seg["amp_tock_deg"]
                ref["truth_amp_tock_lab"] = seg["amp_tock_deg"] if late else seg["amp_tick_deg"]
        return ref
    entry = clips.get(wav.stem)
    if entry is None:
        return ref
    exp = entry.get("expected")
    if exp is None and entry.get("segments"):
        steady = [s for s in entry["segments"] if "rate_s_day" in s]
        exp = steady[-1] if steady else None  # the loop ends in this segment
    if exp:
        ref["truth_rate"] = exp.get("rate_s_day")
        ref["truth_be"] = exp.get("beat_error_ms_est")
    if entry.get("bph"):
        ref["truth_bph"] = entry["bph"]
    elif entry.get("segments"):
        ref["truth_bph"] = steady[-1].get("bph") if steady else None
    cp = entry.get("chronopulse", {})
    ref["cp_rate"] = cp.get("rate_s_day")
    ref["cp_be"] = cp.get("beat_error_ms")
    ref["cp_amp"] = cp.get("amp_onset_deg")
    return ref


def fmt(v, digits=1, sign=False):
    """Formats a number for a table cell; '-' for None."""
    if v is None:
        return "-"
    return f"{v:+.{digits}f}" if sign else f"{v:.{digits}f}"


def err(a, b):
    """a - b, or None if either is missing."""
    return None if a is None or b is None else a - b


def table(title, header, rows):
    """Prints a markdown table."""
    print(f"\n### {title}\n")
    print("| " + " | ".join(header) + " |")
    print("|" + "|".join("---" for _ in header) + "|")
    for r in rows:
        print("| " + " | ".join(r) + " |")


def get(row, method, key):
    """row[method][key] or None."""
    r = row.get(method)
    return None if r is None else r.get(key)


def print_tables(rows):
    """Prints every metric family for the collected rows."""
    names = sorted(rows)

    def line(name, cells):
        return [name] + cells

    table("Rate (s/d): reference = truth (generated) or designed loop rate",
          ["file", "reference", "ChronoPulse", "firmware", "fw err", "otg_ref",
           "otg err", "fw err vs CP"],
          [line(n, [fmt(rows[n]["ref"].get("truth_rate"), 2),
                    fmt(rows[n]["ref"].get("cp_rate"), 2),
                    fmt(get(rows[n], "firmware", "rate_s_day"), 2),
                    fmt(err(get(rows[n], "firmware", "rate_s_day"),
                            rows[n]["ref"].get("truth_rate")), 2, True),
                    fmt(get(rows[n], "otg_ref", "rate_s_day"), 2),
                    fmt(err(get(rows[n], "otg_ref", "rate_s_day"),
                            rows[n]["ref"].get("truth_rate")), 2, True),
                    fmt(err(get(rows[n], "firmware", "rate_s_day"),
                            rows[n]["ref"].get("cp_rate")), 2, True)])
           for n in names])

    table("Beat error (ms, magnitude): reference = truth or designed estimate",
          ["file", "reference", "ChronoPulse", "firmware", "fw err", "otg_ref",
           "otg err", "otg signed"],
          [line(n, [fmt(rows[n]["ref"].get("truth_be"), 3),
                    fmt(rows[n]["ref"].get("cp_be"), 3),
                    fmt(get(rows[n], "firmware", "be_ms"), 3),
                    fmt(err(get(rows[n], "firmware", "be_ms"),
                            rows[n]["ref"].get("truth_be")), 3, True),
                    fmt(get(rows[n], "otg_ref", "be_ms"), 3),
                    fmt(err(get(rows[n], "otg_ref", "be_ms"),
                            rows[n]["ref"].get("truth_be")), 3, True),
                    fmt(get(rows[n], "otg_ref", "be_signed_ms"), 3, True)])
           for n in names])

    def amp_ref(n):
        r = rows[n]["ref"]
        return r.get("truth_amp") if r.get("truth_amp") is not None else r.get("cp_amp")

    table("Amplitude (deg): reference = truth, else ChronoPulse onset",
          ["file", "ref (T=truth, C=CP)", "firmware", "fw err", "otg_ref",
           "otg err"],
          [line(n, [("T " if rows[n]["ref"].get("truth_amp") is not None else "C ")
                    + fmt(amp_ref(n), 1),
                    fmt(get(rows[n], "firmware", "amp_deg"), 1),
                    fmt(err(get(rows[n], "firmware", "amp_deg"), amp_ref(n)),
                        1, True),
                    fmt(get(rows[n], "otg_ref", "amp_deg"), 1),
                    fmt(err(get(rows[n], "otg_ref", "amp_deg"), amp_ref(n)),
                        1, True)])
           for n in names])

    side_rows = [n for n in names
                 if rows[n]["ref"].get("truth_amp_tick") is not None
                 or get(rows[n], "otg_ref", "amp_tick_deg") is not None]
    if side_rows:
        table("Amplitude per side (deg); truth in the late-side labelling",
              ["file", "truth tick", "truth tock", "firmware tick", "err",
               "firmware tock", "err", "otg tick", "otg tock", "windows used/total"],
              [line(n, [fmt(rows[n]["ref"].get("truth_amp_tick_lab"), 1),
                        fmt(rows[n]["ref"].get("truth_amp_tock_lab"), 1),
                        fmt(get(rows[n], "firmware", "amp_tick_deg"), 1),
                        fmt(err(get(rows[n], "firmware", "amp_tick_deg"),
                                rows[n]["ref"].get("truth_amp_tick_lab")), 1, True),
                        fmt(get(rows[n], "firmware", "amp_tock_deg"), 1),
                        fmt(err(get(rows[n], "firmware", "amp_tock_deg"),
                                rows[n]["ref"].get("truth_amp_tock_lab")), 1, True),
                        fmt(get(rows[n], "otg_ref", "amp_tick_deg"), 1),
                        fmt(get(rows[n], "otg_ref", "amp_tock_deg"), 1),
                        "{}/{}".format(get(rows[n], "firmware", "windows_used") or "-",
                                       get(rows[n], "firmware", "windows_total") or "-")])
               for n in side_rows])

    signed_rows = [n for n in names
                   if rows[n]["ref"].get("truth_be_signed") is not None]
    if signed_rows:
        table("Signed beat error (ms, + = Tick late; Tick = late side by drop BE)",
              ["file", "truth", "firmware", "fw err", "source", "otg_ref",
               "otg err"],
              [line(n, [fmt(rows[n]["ref"].get("truth_be_signed"), 3, True),
                        fmt(get(rows[n], "firmware", "be_signed_ms"), 3, True),
                        fmt(err(get(rows[n], "firmware", "be_signed_ms"),
                                rows[n]["ref"].get("truth_be_signed")), 3, True),
                        str(get(rows[n], "firmware", "be_source") or "-"),
                        fmt(get(rows[n], "otg_ref", "be_signed_ms"), 3, True),
                        fmt(err(get(rows[n], "otg_ref", "be_signed_ms"),
                                rows[n]["ref"].get("truth_be_signed")), 3, True)])
               for n in signed_rows])

    table("Jitter (us RMS about the fit)",
          ["file", "firmware", "otg_ref"],
          [line(n, [fmt(get(rows[n], "firmware", "jitter_us"), 1),
                    fmt(get(rows[n], "otg_ref", "jitter_us"), 1)])
           for n in names])

    table("Lock, valid fractions, first valid reading",
          ["file", "bph (fw)", "locked", "lock frac", "amp valid frac",
           "first valid (s)", "otg lock frac", "otg valid frac"],
          [line(n, [str(get(rows[n], "firmware", "bph") or "-"),
                    str(get(rows[n], "firmware", "locked")),
                    fmt(get(rows[n], "firmware", "lock_fraction"), 3),
                    fmt(get(rows[n], "firmware", "amp_valid_fraction"), 3),
                    fmt(get(rows[n], "firmware", "first_valid_s"), 2),
                    fmt(get(rows[n], "otg_ref", "lock_fraction"), 3),
                    fmt(get(rows[n], "otg_ref", "amp_valid_fraction"), 3)])
           for n in names])

    table("CPU proxy (host ns per 240-sample block, wall clock)",
          ["file", "firmware ns/block"],
          [line(n, [fmt(get(rows[n], "firmware", "host_ns_per_block"), 0)])
           for n in names])


def print_deltas(rows, base):
    """Prints before/after deltas of the firmware rows against a baseline."""
    base_rows = {r["file"]: r for r in base.get("rows", [])
                 if r.get("method") == "firmware"}
    keys = ["rate_s_day", "be_ms", "amp_deg", "jitter_us", "lock_fraction",
            "amp_valid_fraction", "first_valid_s", "host_ns_per_block"]
    out = []
    for n in sorted(rows):
        new = rows[n].get("firmware")
        old = base_rows.get(n + ".wav") or base_rows.get(n)
        if new is None or old is None:
            out.append([n] + ["-"] * len(keys))
            continue
        cells = []
        for k in keys:
            a, b = old.get(k), new.get(k)
            if a is None or b is None:
                cells.append("-")
            else:
                d = 3 if k.endswith("fraction") or k == "be_ms" else 1
                cells.append(f"{fmt(a, d)} -> {fmt(b, d)} ({fmt(b - a, d, True)})")
        out.append([n] + cells)
    table("Firmware before -> after (delta) against the baseline",
          ["file"] + keys, out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("harness", nargs="?", help="path to the dsp_wav binary")
    ap.add_argument("--json", help="write every row to this JSON file")
    ap.add_argument("--baseline", help="JSON written by an earlier --json run")
    ap.add_argument("--only", help="only files whose name contains this text")
    ap.add_argument("--harness-args", default="",
                    help="extra harness options, one quoted string "
                         "(for example '--timing drop')")
    ap.add_argument("--no-otg", action="store_true",
                    help="skip reference_otg.py even if present")
    ap.add_argument("--otg-python", default=sys.executable,
                    help="interpreter for reference_otg.py (needs numpy)")
    args = ap.parse_args()

    harness_args = shlex.split(args.harness_args)
    tmp = None
    if args.harness:
        harness = Path(args.harness)
    else:
        tmp = tempfile.TemporaryDirectory()
        harness = build_harness(tmp.name)

    wavs = sorted(CLIPS.glob("*.wav")) + sorted(GENERATED.glob("*.wav"))
    if args.only:
        wavs = [w for w in wavs if args.only in w.name]
    if not wavs:
        print("no wav files found", file=sys.stderr)
        return 1
    clips = load_clips_json()
    use_otg = OTG.exists() and not args.no_otg

    rows = {}
    flat = []
    for wav in wavs:
        name = wav.stem
        row = {"ref": reference_for(wav, clips)}
        fw = run_firmware(harness, wav, harness_args)
        if fw is not None:
            row["firmware"] = fw
            flat.append(fw)
        if use_otg:
            otg = run_otg(args.otg_python, wav)
            if otg is not None:
                row["otg_ref"] = otg
                flat.append(otg)
        rows[name] = row

    print(f"{len(rows)} files, lift {LIFT_DEG} deg, avg {AVG_S} s, "
          f"otg_ref {'on' if use_otg else 'off'}")
    print_tables(rows)
    if args.baseline:
        print_deltas(rows, json.loads(Path(args.baseline).read_text()))
    if args.json:
        Path(args.json).write_text(json.dumps(
            {"lift_deg": LIFT_DEG, "avg_s": AVG_S, "rows": flat}, indent=1) + "\n")
    if tmp is not None:
        tmp.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
