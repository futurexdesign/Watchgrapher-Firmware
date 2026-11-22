#!/usr/bin/env python3
"""Run the reference algorithms on every test vector and compare with the ground truth.
python3 run_tests.py VECTOR_DIR  -> prints a table and writes VECTOR_DIR/../results.json"""
import sys, os, json
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gen"))
import channel as ch
import timing_ref as R
import signals as S

VD = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "vectors")
results = {}


def drive_end(width, duty):
    segs = S.motor_segments(width, duty)
    return max(s[0] for s in segs if not s[1])


for item in json.load(open(os.path.join(VD, "manifest.json"))):
    name = item["file"][:-4]
    truth = json.load(open(os.path.join(VD, item["truth"])))
    fs, L, Rr = ch.read_wav24(os.path.join(VD, item["file"]))
    r = {"description": truth["description"]}
    if name.startswith("ind_") and "events" in truth:
        pulses, y, sig = R.detect_motor_pulses(L, fs)
        ev = truth["events"]
        r["pulses_found"] = len(pulses)
        r["pulses_true"] = len(ev)
        r["noise_sigma_codes"] = round(float(sig))
        r["clipped"] = bool(np.abs(L).max() >= ch.CODE_FS - 1)
        if len(pulses) == len(ev):
            true_t = np.array([e["t_onset_s"] for e in ev])
            r["polarity_ok"] = all(p["polarity"] * pulses[0]["polarity"] == e["polarity"] * ev[0]["polarity"]
                                   for p, e in zip(pulses, ev))
            for method in ("xcorr", "threshold"):
                tt = R.timestamp_pulses(pulses, y, fs, method=method)
                d = tt - true_t
                d -= np.mean(d)                       # constant delay removed
                r[f"timing_error_rms_us_{method}"] = round(float(np.sqrt(np.mean(d ** 2)) * 1e6), 3)
                r[f"timing_error_max_us_{method}"] = round(float(np.max(np.abs(d)) * 1e6), 3)
                if len(ev) >= 3:
                    mr = R.motor_rate(tt, [p["polarity"] for p in pulses])
                    r[f"rate_{method}"] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in mr.items()
                                           if k in ("step_s", "rate_without_inhibition_s_per_day",
                                                    "rate_with_inhibition_s_per_day", "inhibition_period_s",
                                                    "inhibition_jumps_us", "residual_rms_us", "warning")}
            shapes = [R.drive_shape(p, y, fs) for p in pulses]
            r["duty_found"] = [s["duty_quantised"] if s else None for s in shapes]
            r["duty_true"] = [e["duty"] for e in ev]
            r["drive_width_found_ms"] = [round(s["drive_width_s"] * 1e3, 3) if s and s["drive_width_s"] else None for s in shapes]
            r["drive_width_true_ms"] = [round(drive_end(e["pulse_width_s"], e["duty"]) * 1e3, 3) for e in ev]
        r["expected"] = truth["expected"]
    elif name.startswith("ind_tuning"):
        out = R.tone_rate(L, fs, truth["nominal_frequency_hz"], block_s=0.25)
        r["rate_s_per_day"] = round(out["rate_s_per_day"], 4)
        r["expected"] = truth["expected"]
    elif name.startswith("lcd_"):
        out = R.lcd_rate(Rr, fs)
        r.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in out.items()})
        r["expected"] = truth["expected"]
    elif name.startswith("xtal_"):
        out = R.tone_rate(Rr, fs, 32768.0, block_s=0.05)
        r["rate_s_per_day"] = round(out["rate_s_per_day"], 4)
        r["expected"] = truth["expected"]
    results[name] = r
    print(name)
    for k, v in r.items():
        if k not in ("description",):
            print("   ", k, v)

json.dump(results, open(os.path.join(VD, "..", "results.json"), "w"), indent=1)
