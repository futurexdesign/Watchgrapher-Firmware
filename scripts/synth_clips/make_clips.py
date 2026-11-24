"""Cuts seamless synthetic-source loops from watch recordings.

Each loop holds an even number N of beats (tick + tock pairs, so the beat
error survives the wrap) and is exactly round(N * fitted_half_period * 48000)
samples long, so a timegrapher reads the recording's rate from the loop. N is
chosen near 4 s so that the rounding is as small as possible. Cuts fall
halfway between ticks; the last 2 ms crossfade into the audio just before the
start, so the wrap is continuous.

Usage: python3 scripts/synth_clips/make_clips.py <src_dir> <out_dir>
(src_dir holds the ChronoPulse recordings and ChronoPulse_Results.txt;
out_dir is normally assets/synth_clips). Needs numpy, scipy, soundfile.
Writes <id>.wav (mono S16, 48 kHz) per clip and clips.json.
"""
import json
import os
import re
import sys

import numpy as np
import soundfile as sf

from analyze import FS, analyze, detect, envelope, fit, load

TARGET_S = (3.5, 4.5)
XFADE = 96  # 2 ms

IDS = {
    "18000BPH_Watham.wav": "waltham_18000",
    "21600BPH_8215_InCase .wav": "m8215_incase_21600",
    "21600BPH_8215_jubilee .wav": "m8215_jubilee_21600",
    "21600BPH_NH35.wav": "nh35_21600",
    "21600BPH_NH39A.wav": "nh39a_21600",
    "21600BPH_ST3600.wav": "st3600_21600",
    "28800BPH_3135_hulk.wav": "r3135_hulk_28800",
    "28800BPH_3235_FreeSprung.wav": "r3235_freesprung_28800",
    "28800BPH_3235_Starbucks.wav": "r3235_starbucks_28800",
    "Switch21600To28800.wav": "switch_21600_28800",
}


def chronopulse(path):
    """Reference values from ChronoPulse_Results.txt, keyed by file name."""
    out = {}
    pat = re.compile(r"^(\S.*?\.wav)\s+s/d\s+([+-]?[\d.]+)\s+BE\s+([\d.]+)\s+AMP Onset\s+(\d+)\s+Peak\s+(\d+)")
    for line in open(path):
        m = pat.match(line.strip())
        if m:
            out[m.group(1).strip()] = {"rate_s_day": float(m.group(2)), "beat_error_ms": float(m.group(3)),
                                       "amp_onset_deg": int(m.group(4)), "amp_peak_deg": int(m.group(5))}
    return out


def write_loop(x, start, length, path):
    seg = x[start:start + length].copy()
    w = np.linspace(0.0, 1.0, XFADE)
    pre = x[start - XFADE:start]
    seg[-XFADE:] = seg[-XFADE:] * (1.0 - w) + pre * w
    pcm = np.clip(np.round(seg * 32767.0), -32768, 32767).astype(np.int16)
    sf.write(path, pcm, FS, subtype="PCM_16")
    return pcm


def verify(pcm, bph):
    """Re-analyses 10 loops back to back: rate and beat error seen by a fit."""
    x = np.tile(pcm.astype(np.float64) / 32768.0, 10)
    env, _ = envelope(x)
    hp = 3600.0 / bph
    on = detect(env, hp) / FS
    (a, b, c), k, resid, keep = fit(on, hp)
    return round((hp / b - 1.0) * 86400.0, 2), round(abs(2 * c) * 1000, 3)


def make_regular(path, out_dir, ref):
    name = os.path.basename(path)
    r, x, d = analyze(path)
    on, k, keep, hp = d["onsets"], d["k"], d["keep"], d["hp"]
    b = d["coef"][1]
    # N: even, loop near 4 s, smallest rounding to whole samples
    cands = [n for n in range(2, 400, 2) if TARGET_S[0] <= n * b <= TARGET_S[1]]
    n_beats = min(cands, key=lambda n: abs(n * b * FS - round(n * b * FS)))
    length = int(round(n_beats * b * FS))
    # window: consecutive grid indices, no outliers, local rate closest to global
    best = None
    idx = {int(kk): i for i, kk in enumerate(k)}
    for i0 in range(1, len(on) - n_beats - 1):
        if k[i0] % 2:
            continue
        ks = list(range(int(k[i0]), int(k[i0]) + n_beats + 1))
        if not all(kk in idx and keep[idx[kk]] for kk in ks):
            continue
        sel = [idx[kk] for kk in ks]
        A = np.column_stack([np.ones(len(sel)), k[sel], (-1.0) ** k[sel]])
        cw, *_ = np.linalg.lstsq(A, on[sel], rcond=None)
        local = (hp / cw[1] - 1.0) * 86400.0
        score = abs(local - r["rate_s_day"])
        if best is None or score < best[0]:
            best = (score, i0, local)
    _, i0, local = best
    start = int(round((on[i0 - 1] + on[i0]) / 2 * FS))
    cid = IDS[name]
    pcm = write_loop(x, start, length, os.path.join(out_dir, cid + ".wav"))
    loop_rate = (n_beats * hp / (length / FS) - 1.0) * 86400.0
    v_rate, v_be = verify(pcm, r["bph"])
    return {"id": cid, "source": name, "bph": r["bph"], "loop_beats": n_beats, "loop_samples": length,
            "loop_seconds": round(length / FS, 4), "source_start_s": round(start / FS, 3),
            "expected": {"rate_s_day": round(loop_rate, 2), "beat_error_ms_est": r["beat_error_ms"]},
            "recording_fit": {"rate_s_day": r["rate_s_day"], "window_rate_s_day": round(local, 2),
                              "beat_error_ms": r["beat_error_ms"], "jitter_us_rms": r["jitter_us_rms"],
                              "rate_20s_windows": r["rate_20s_windows"]},
            "loop_check": {"rate_s_day": v_rate, "beat_error_ms": v_be},
            "chronopulse": ref.get(name)}


def make_switch(path, out_dir, ref):
    """21600 (12 s) -> changeover -> 28800 (12 s); cut between ticks."""
    name = os.path.basename(path)
    x = load(path)
    env, _ = envelope(x)
    parts = {}
    for tag, bph, a, b in (("a", 21600, 0.0, 25.3), ("b", 28800, 30.5, 58.5)):
        hp = 3600.0 / bph
        s, e = int(a * FS), int(b * FS)
        on = (detect(env[s:e], hp) + s) / FS
        (c0, b1, c2), k, res, keep = fit(on, hp)
        parts[tag] = {"onsets": on, "rate": round((hp / b1 - 1) * 86400, 2), "be": round(abs(2 * c2) * 1000, 3)}
    on_a, on_b = parts["a"]["onsets"], parts["b"]["onsets"]
    i = int(np.searchsorted(on_a, on_a[-1] - 12.0))
    start = int(round((on_a[i - 1] + on_a[i]) / 2 * FS))
    j = int(np.searchsorted(on_b, on_b[0] + 12.0))
    end = int(round((on_b[j - 1] + on_b[j]) / 2 * FS))
    cid = IDS[name]
    write_loop(x, start, end - start, os.path.join(out_dir, cid + ".wav"))
    return {"id": cid, "source": name, "bph": None, "loop_samples": end - start,
            "loop_seconds": round((end - start) / FS, 3), "source_start_s": round(start / FS, 3),
            "segments": [
                {"bph": 21600, "from_s": 0.0, "to_s": round(on_a[-1] - start / FS, 2), "rate_s_day": parts["a"]["rate"], "beat_error_ms_est": parts["a"]["be"]},
                {"note": "watch changed over, noise", "from_s": round(on_a[-1] - start / FS, 2), "to_s": round(on_b[0] - start / FS, 2)},
                {"bph": 28800, "from_s": round(on_b[0] - start / FS, 2), "to_s": round((end - start) / FS, 2), "rate_s_day": parts["b"]["rate"], "beat_error_ms_est": parts["b"]["be"]}],
            "note": "the loop wraps from 28800 straight back to 21600",
            "chronopulse": ref.get(name)}


def main():
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    ref = chronopulse(os.path.join(src, "ChronoPulse_Results.txt"))
    clips = []
    for name in sorted(IDS):
        path = os.path.join(src, name)
        c = make_switch(path, out, ref) if name.startswith("Switch") else make_regular(path, out, ref)
        clips.append(c)
        print(json.dumps(c))
    json.dump({"sample_rate_hz": FS, "format": "mono S16LE WAV", "lift_angle_deg_reference": 52,
               "reference": "ChronoPulse, lift angle 52 deg, integration 20 s", "clips": clips},
              open(os.path.join(out, "clips.json"), "w"), indent=2)


main()
