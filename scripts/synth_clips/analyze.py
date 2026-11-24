"""Tick timing analysis of watch recordings (reference for synthetic clips).

For each WAV: resample to 48 kHz, band-pass 1-12 kHz, detect each tick's
onset (first crossing of 25 % of the tick's envelope peak), then least-squares
fit  t_k = a + b*k + c*(-1)^k  over all ticks:
  rate s/d   = (nominal_half_period / b - 1) * 86400
  beat error = 2|c| (ms), the firmware's definition (dsp_types.h)
Also reports the rate over sliding 20 s windows (ChronoPulse integration).

Usage: python analyze.py <dir> [out.json]
"""
import glob
import json
import os
import re
import sys

import numpy as np
import soundfile as sf
from scipy.signal import butter, resample_poly, sosfiltfilt

FS = 48000


def load(path):
    x, fs = sf.read(path, dtype="float64", always_2d=True)
    x = x[:, 0]
    if fs != FS:
        g = np.gcd(int(fs), FS)
        x = resample_poly(x, FS // g, int(fs) // g)
    return x


def envelope(x):
    sos = butter(4, [1000, 12000], btype="band", fs=FS, output="sos")
    y = sosfiltfilt(sos, x)
    e = np.abs(y)
    k = int(0.0002 * FS)  # 0.2 ms smoothing
    return np.convolve(e, np.ones(k) / k, mode="same"), y


def detect(env, half_period_s):
    """Onsets (fractional samples) of successive ticks."""
    hp = int(half_period_s * FS)
    win = int(0.6 * hp)
    noise = np.median(env)
    onsets = []
    # coarse: strongest point in every half-period-ish window, walking forward
    i = int(np.argmax(env[:hp]))
    while i < len(env) - win:
        lo = max(0, i - win // 2)
        hi = min(len(env), i + win // 2)
        p = lo + int(np.argmax(env[lo:hi]))
        peak = env[p]
        if peak > 6 * noise:
            thr = noise + 0.25 * (peak - noise)
            j = p
            floor = max(lo, p - int(0.012 * FS))
            while j > floor and env[j] > thr:
                j -= 1
            # linear interpolation of the crossing
            a, b = env[j], env[j + 1]
            frac = (thr - a) / (b - a) if b != a else 0.0
            onsets.append(j + frac)
            i = p + hp
        else:
            i = p + hp
    return np.array(onsets)


def fit(onsets_s, half_period_s):
    """Fits a + b*k + c*(-1)^k with k from the nominal grid (handles misses)."""
    k = np.round((onsets_s - onsets_s[0]) / half_period_s).astype(int)
    A = np.column_stack([np.ones_like(k), k, (-1.0) ** k])
    coef, *_ = np.linalg.lstsq(A, onsets_s, rcond=None)
    resid = onsets_s - A @ coef
    keep = np.abs(resid) < 0.002  # drop >2 ms outliers, refit
    coef, *_ = np.linalg.lstsq(A[keep], onsets_s[keep], rcond=None)
    resid = onsets_s - A @ coef
    return coef, k, resid, keep


def analyze(path):
    name = os.path.basename(path)
    m = re.match(r"(\d+)BPH", name)
    bph = int(m.group(1)) if m else None
    x = load(path)
    env, _ = envelope(x)
    out = {"file": name, "seconds": round(len(x) / FS, 2), "peak_fs": round(float(np.max(np.abs(x))), 4),
           "rms_fs": round(float(np.sqrt(np.mean(x ** 2))), 5)}
    if bph is None:
        out["note"] = "no nominal rate in the name"
        return out, x, None
    hp = 3600.0 / bph
    on = detect(env, hp) / FS
    (a, b, c), k, resid, keep = fit(on, hp)
    rate = (hp / b - 1.0) * 86400.0
    win = []
    for t0 in np.arange(0, on[-1] - 20, 5):
        sel = keep & (on >= t0) & (on < t0 + 20)
        if sel.sum() > 20:
            A = np.column_stack([np.ones(sel.sum()), k[sel], (-1.0) ** k[sel]])
            cw, *_ = np.linalg.lstsq(A, on[sel], rcond=None)
            win.append(round((hp / cw[1] - 1.0) * 86400.0, 1))
    out.update({"bph": bph, "ticks": int(len(on)), "outliers": int((~keep).sum()),
                "rate_s_day": round(rate, 2), "beat_error_ms": round(abs(2 * c) * 1000, 3),
                "jitter_us_rms": round(float(np.std(resid[keep])) * 1e6, 1),
                "rate_20s_windows": win, "half_period_fit_s": b})
    return out, x, {"onsets": on, "k": k, "keep": keep, "coef": (a, b, c), "hp": hp}


if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(sys.argv[1], "*.wav")))
    res = []
    for f in files:
        r, _, _ = analyze(f)
        res.append(r)
        print(json.dumps({k: v for k, v in r.items() if k != "half_period_fit_s"}))
    if len(sys.argv) > 2:
        json.dump(res, open(sys.argv[2], "w"), indent=1)
