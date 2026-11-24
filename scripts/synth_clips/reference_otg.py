#!/usr/bin/env python3
"""Offline reference of the opentimegrapher (OTG) method (TIMING_ENGINE_PLAN.md, T0c).

Reproduces OTG's published math (opentimegrapher.org, pages M1, M2, M4, M5, M6)
as far as the pages specify it, in plain numpy/scipy, not real time. It is a
yardstick for the firmware timing engine: every step can be switched so the
contribution of each change to amplitude, beat error and rate can be measured.

Pipeline
  1. Front end: 4th-order Butterworth high-pass at 1.5 kHz, rectify, 0.2 ms
     smoothing. Knock gate: bursts shorter than 0.1 s with more than 2x the
     median tick energy are zeroed.
  2. Beat rate: autocorrelation of the envelope at the standard rates.
  3. Beat times: two-pass template matching. Pass 1 builds a median beat shape
     (-20 ms ... +8 ms around the drop) from envelope peaks and matches it with
     a +-12 ms search; pass 2 builds one template per side (k parity) from the
     aligned beats and matches again (+-3 ms, parabolic sub-sample). Beats with
     quality (Pearson correlation) below 0.4 are ignored. Beat numbers k count
     whole beats and keep gaps.
  4. Rate: least squares t = t0 + P k + c s on the drop landmark, two refits
     dropping residuals above 5 robust SD; rate = (nominal/P - 1) * 86400.
  5. Amplitude: per side, in 2 s windows, averaged shapes (trimmed mean of the
     middle half); drop = half-height crossing of the rising edge walking back
     at most 2 ms from the peak; unlock = first steady rise above
     max(2 % of drop height, 4x background noise), timed at its own half height;
     A = L / (2 sin(pi t / T)), T = 7200 / bph; window amplitude = mean of both
     sides (each side needs >= 3 beats); reading = median of windows.
  6. Beat error: signed, positive when the Tick comes late; per window
     BE = BE_drop - lift_tick + lift_tock; reading = median of windows; drop BE
     is the fallback (no unlock, or windows disagree in sign beyond their spread).
  7. Cuts: more than 2 s without a beat, or 4 consecutive off-grid beats, ends a
     stretch; nothing is fitted or averaged across a cut. A stretch shorter than
     10 s gives no rate.

Final reading: only the last stretch with enough beats is used, and only audio
after the last change of beat rate (rates are estimated on 6 s windows, hop 2 s).
Files shorter than --min-stretch-s (the 4 s watch loops, seamless whole-beat
loops by construction) are tiled to >= 20 s first (--tile 1 disables; the 10 s
rule is then waived and the result carries "rate_waived": true).

Switches (each isolates one step):
  --timing unlock|drop       beat error from the unlock (corrected) or from the drop
  --landmarks half|peak      half-height crossings or burst peaks (also for the fit time)
  --windows on|off           2 s shape windows or one window per beat pair
                             (2P wide, 1 beat per side, i.e. single-beat landmarks)
  --knock on|off             knock gate
  --frontend hp|raw          1.5 kHz high-pass + rectify + smooth, or |x| only
  --trim F                   fraction cut from each end in the window mean (0.25)

Usage
  reference_otg.py <wav> [flags]         one Result JSON line (method "otg_ref")
  reference_otg.py <wav> --beats         per-beat JSON lines first
  reference_otg.py --table <wav>...      one row per file vs ChronoPulse / truth
  reference_otg.py --contrib <wav>...    landmarks x windows contribution table
Truth JSON (<name>.truth.json beside the WAV) is used by --table when present.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import soundfile as sf
from scipy.ndimage import uniform_filter1d
from scipy.signal import butter, find_peaks, sosfiltfilt
from scipy.stats import trim_mean

STANDARD_BPH = [12000, 14400, 18000, 19800, 21600, 25200, 28800, 36000, 43200, 72000]
PRE_S = 0.020
POST_S = 0.008


# ---------------------------------------------------------------- front end
def envelope(x, fs, frontend):
    """Rectified, smoothed envelope. hp: >1.5 kHz then 0.2 ms boxcar."""
    if frontend == "hp":
        sos = butter(4, 1500.0, btype="highpass", fs=fs, output="sos")
        x = sosfiltfilt(sos, x)
    env = np.abs(x)
    n = max(1, int(round(0.0002 * fs)))
    return uniform_filter1d(env, n, mode="nearest")


def knock_gate(env, fs, bph):
    """Zero bursts shorter than 0.1 s whose energy exceeds 2x the median tick energy.

    Returns (gated envelope, list of knock start times in s).
    """
    e5 = uniform_filter1d(env, max(1, int(0.005 * fs)), mode="nearest")
    noise = float(np.median(e5))
    ref = float(np.percentile(e5, 99.5))
    thr = noise + 0.15 * (ref - noise)
    on = e5 > thr
    # merge gaps shorter than 10 ms
    idx = np.flatnonzero(on)
    if idx.size == 0:
        return env, []
    brk = np.flatnonzero(np.diff(idx) > int(0.010 * fs))
    starts = np.r_[idx[0], idx[brk + 1]]
    ends = np.r_[idx[brk], idx[-1]]
    energy = np.array([env[s:e + 1].sum() for s, e in zip(starts, ends)]) / fs
    dur = (ends - starts + 1) / fs
    # typical tick energy: median over bursts of tick-like length (< 40 ms)
    tl = dur < 0.04
    if tl.sum() < 3:
        return env, []
    typical = float(np.median(energy[tl]))
    out = env.copy()
    knocks = []
    m = int(0.005 * fs)
    for s, e, en, d in zip(starts, ends, energy, dur):
        if d < 0.1 and en > 2.0 * typical:
            out[max(0, s - m):e + 1 + m] = noise
            knocks.append(s / fs)
    return out, knocks


# ----------------------------------------------------------------- beat rate
def autocorr_bph(env, fs):
    """Beat rate by envelope autocorrelation at the standard rates; None if hopeless."""
    dec = max(1, int(fs // 2000))
    n = (len(env) // dec) * dec
    e = env[:n].reshape(-1, dec).mean(axis=1)
    fs2 = fs / dec
    e = e - e.mean()
    if len(e) < 8 * fs2 * 0.125 or e.std() == 0:
        return None
    nfft = 1 << int(math.ceil(math.log2(2 * len(e))))
    f = np.fft.rfft(e, nfft)
    ac = np.fft.irfft(f * np.conj(f))[:len(e)]
    ac = ac / (len(e) - np.arange(len(e)))
    ac = ac / ac[0]
    score = {}
    for bph in STANDARD_BPH:
        lag = 3600.0 / bph * fs2
        lo, hi = int(lag * 0.97), int(lag * 1.03) + 1
        if hi >= len(ac):
            continue
        score[bph] = float(ac[lo:hi + 1].max())
    if not score:
        return None
    best = max(score.values())
    if best < 0.05:
        return None
    # prefer the fastest rate that is nearly as good: slower rates see the same ticks at 2x lag
    for bph in sorted(score, reverse=True):
        if score[bph] >= 0.5 * best:
            return bph
    return None


def stable_start(env, fs, win_s=6.0, hop_s=2.0):
    """Returns (bph of the last window, start sample of the last stable-rate region)."""
    n = len(env)
    w, h = int(win_s * fs), int(hop_s * fs)
    if n <= w + h:
        return autocorr_bph(env, fs), 0
    starts = list(range(0, n - w + 1, h))
    if starts[-1] != n - w:
        starts.append(n - w)
    rates = [autocorr_bph(env[s:s + w], fs) for s in starts]
    last = rates[-1]
    i = len(rates) - 1
    while i > 0 and rates[i - 1] == last:
        i -= 1
    return last, starts[i]


# ------------------------------------------------------------ template match
def extract(env, pos, n_pre, n_post):
    """Envelope around fractional sample position pos (linear interpolation)."""
    p = pos + np.arange(-n_pre, n_post)
    return np.interp(p, np.arange(len(env)), env, left=np.nan, right=np.nan)


def parabolic(y, i):
    if i <= 0 or i >= len(y) - 1:
        return 0.0
    a, b, c = y[i - 1], y[i], y[i + 1]
    den = a - 2 * b + c
    return 0.0 if den == 0 else 0.5 * (a - c) / den


def match(env, pos, tpl, n_pre, n_post, max_lag):
    """Slide tpl over env around sample pos. Returns (refined pos, quality) or None."""
    ip = int(round(pos))
    lo, hi = ip - n_pre - max_lag, ip + n_post + max_lag
    if lo < 0 or hi > len(env):
        return None
    seg = env[lo:hi]
    L = n_pre + n_post
    win = np.lib.stride_tricks.sliding_window_view(seg, L)
    win = win - win.mean(axis=1, keepdims=True)
    t = tpl - tpl.mean()
    nt = np.linalg.norm(t)
    nw = np.linalg.norm(win, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        r = (win @ t) / (nw * nt)
    r = np.nan_to_num(r)
    i = int(np.argmax(r))
    lag = i - max_lag + parabolic(r, i)
    return ip + lag, float(max(0.0, r[i]))


def median_template(env, positions, n_pre, n_post):
    shapes = [extract(env, p, n_pre, n_post) for p in positions]
    shapes = [s for s in shapes if not np.isnan(s).any()]
    if len(shapes) < 3:
        return None
    return np.median(np.array(shapes), axis=0)


def drop_peak_index(shape, fs, n_pre):
    """Index of the drop peak in a template (0.5 ms mean strength).

    The drop is the last strong burst of the beat: the last local peak reaching
    at least 70 % of the strongest. Taking the plain maximum jumps between the
    unlock and the drop when both are loud (clipped recordings).
    """
    sm = uniform_filter1d(shape, max(1, int(0.0005 * fs)), mode="nearest")
    top = float(sm.max())
    pk, _ = find_peaks(sm, height=0.7 * top, distance=max(1, int(0.002 * fs)))
    return int(pk[-1]) if len(pk) else int(np.argmax(sm))


def anchored_drop_index(shape, fs, n_pre, half_window_s=0.0025):
    """Drop peak of a shape already aligned on the drop: strongest 0.5 ms mean
    within +-2.5 ms of the anchor (time 0)."""
    sm = uniform_filter1d(shape, max(1, int(0.0005 * fs)), mode="nearest")
    w = int(half_window_s * fs)
    lo = max(0, n_pre - w)
    return lo + int(np.argmax(sm[lo:n_pre + w + 1]))


# ------------------------------------------------------------------ landmarks
def crossing_up(shape, level, i_lo, i_hi):
    """First upward crossing of level in [i_lo, i_hi], linear interpolation; None if none."""
    for i in range(max(i_lo, 0), min(i_hi, len(shape) - 1)):
        if shape[i] < level <= shape[i + 1]:
            d = shape[i + 1] - shape[i]
            return i + (level - shape[i]) / d if d > 0 else float(i)
    return None


def drop_landmark(shape, fs, i_peak, mode):
    """Drop time (samples, fractional) and height-above-background inputs.

    half: walk back from the peak (at most 2 ms) to the half-height crossing of
    the rising edge. peak: the peak itself, refined parabolically.
    """
    bg = float(np.median(shape[:int(0.004 * fs)]))
    sm = uniform_filter1d(shape, max(1, int(0.0004 * fs)), mode="nearest")
    if mode == "peak":
        return i_peak + parabolic(sm, i_peak), bg
    H = shape[i_peak] - bg
    level = bg + 0.5 * H
    lim = max(0, i_peak - int(0.002 * fs))
    i = i_peak
    while i > lim and shape[i] > level:
        i -= 1
    if shape[i] >= level:
        return float(i), bg
    d = shape[i + 1] - shape[i]
    return i + (level - shape[i]) / d, bg


def unlock_landmark(shape, fs, i_drop_peak, t_drop, mode):
    """Unlock time (samples) or None.

    Onset = first steady (0.2 ms) rise above bg + max(2 % of drop height, 4 SD
    of the background); timed at its own half height (or its peak in peak mode).
    """
    n_bg = int(0.004 * fs)
    bg = float(np.median(shape[:n_bg]))
    sd = 1.4826 * float(np.median(np.abs(shape[:n_bg] - bg)))
    H = shape[i_drop_peak] - bg
    if H <= 0:
        return None
    thr = bg + max(0.02 * H, 4.0 * sd)
    stop = int(max(n_bg + 2, t_drop - 1))
    steady = max(2, int(0.0002 * fs))
    above = shape[:stop] > thr
    i_on = None
    for i in range(n_bg, stop - steady):
        if above[i:i + steady].all():
            i_on = i
            break
    if i_on is None:
        return None
    sm = uniform_filter1d(shape, max(1, int(0.0004 * fs)), mode="nearest")
    seg = sm[i_on:stop]
    prom = max(0.5 * (thr - bg), 1e-12)
    pk, _ = find_peaks(seg, prominence=prom)
    j = i_on + (int(pk[0]) if len(pk) else int(np.argmax(seg)))
    if mode == "peak":
        return j + parabolic(sm, j)
    level = bg + 0.5 * (sm[j] - bg)
    c = crossing_up(shape, level, i_on - 1, j)
    return c if c is not None else float(i_on)


def shape_lift(shape, fs, n_pre, mode):
    """(lift seconds or None, drop landmark samples, unlock landmark samples or None)."""
    ip = anchored_drop_index(shape, fs, n_pre)
    td, _ = drop_landmark(shape, fs, ip, mode)
    tu = unlock_landmark(shape, fs, ip, td, mode)
    if tu is None:
        return None, td, None
    return (td - tu) / fs, td, tu


def amplitude(lift_s, bph, L):
    T = 7200.0 / bph
    if lift_s is None or lift_s <= 0 or lift_s >= T / 2:
        return None
    return L / (2.0 * math.sin(math.pi * lift_s / T))


# ----------------------------------------------------------------- beat times
def find_candidates(env, fs, P):
    env1 = uniform_filter1d(env, max(1, int(0.0005 * fs)), mode="nearest")
    noise = float(np.median(env1))
    dist = max(1, int(0.4 * P * fs))
    pk, pr = find_peaks(env1, height=noise * 3.0, distance=dist)
    if len(pk) < 6:
        return np.array([], dtype=int)
    h = env1[pk]
    ref = float(np.median(h))
    keep = h > noise + 0.3 * (ref - noise)
    return pk[keep]


def number_beats(times, P, gap_s=2.0, tol=0.15, max_off=4):
    """Assigns k with gaps; splits into stretches at gaps > gap_s or after
    max_off consecutive off-grid beats. Returns list of stretches, each a list
    of (index into times, k)."""
    stretches = []
    cur = []
    i = 0
    off = 0
    n = len(times)
    first_off = None
    while i < n:
        if not cur:
            cur = [(i, 0)]
            i += 1
            off, first_off = 0, None
            continue
        ti, ki = cur[-1]
        dt = times[i] - times[ti]
        if dt > gap_s:
            stretches.append(cur)
            cur = []
            continue
        m = int(round(dt / P))
        if m >= 1 and abs(dt - m * P) <= tol * P:
            cur.append((i, ki + m))
            off, first_off = 0, None
            i += 1
        else:
            off += 1
            if first_off is None:
                first_off = i
            if off >= max_off:
                stretches.append(cur)
                cur = []
                i = first_off
                off, first_off = 0, None
            else:
                i += 1
    if cur:
        stretches.append(cur)
    return stretches


# ------------------------------------------------------------------------ fit
def lsq_fit(t, k, s, nsd=5.0, refits=2):
    """t = t0 + P k + c s with two refits dropping residuals above nsd robust SD.

    Returns (t0, P, c, keep mask, residuals in s)."""
    A = np.column_stack([np.ones_like(t), k, s])
    keep = np.ones(len(t), bool)
    for it in range(refits + 1):
        x, *_ = np.linalg.lstsq(A[keep], t[keep], rcond=None)
        res = t - A @ x
        if it == refits:
            break
        sd = 1.4826 * np.median(np.abs(res[keep] - np.median(res[keep])))
        sd = max(sd, 5e-6)
        keep = np.abs(res) <= nsd * sd
        if keep.sum() < 4:
            break
    return x[0], x[1], x[2], keep, res


# ----------------------------------------------------------------- main engine
def analyse(x, fs, name, opt):
    """Runs the OTG pipeline. Returns (Result dict, per-beat list)."""
    L = opt.lift
    env = envelope(x, fs, opt.frontend)
    res = {"file": name, "method": "otg_ref", "locked": False, "bph": None,
           "rate_s_day": None, "be_ms": None, "be_signed_ms": None,
           "amp_deg": None, "amp_tick_deg": None, "amp_tock_deg": None,
           "jitter_us": None, "lock_fraction": None, "amp_valid_fraction": None,
           "first_valid_s": None, "host_ns_per_block": None}
    beats_out = []
    bph = opt.bph
    start = 0
    if bph is None:
        bph, start = stable_start(env, fs)
    if bph is None:
        return res, beats_out
    res["bph"] = bph
    t_off = start / fs
    env = env[start:]
    P = 3600.0 / bph
    knocks = []
    if opt.knock == "on":
        env, knocks = knock_gate(env, fs, bph)
    n_pre, n_post = int(PRE_S * fs), int(POST_S * fs)

    cand = find_candidates(env, fs, P)
    if len(cand) < 8:
        return res, beats_out
    # Pass 1: global template. Candidate anchors may sit on the unlock, impulse or
    # drop (clipped recordings tie), so align iteratively on a wide window
    # (+-30 ms), re-anchor on the median shape's drop peak, then crop.
    wpre = wpost = int(0.030 * fs)
    pos1 = cand.astype(float)
    for _ in range(3):
        tplw = median_template(env, pos1, wpre, wpost)
        if tplw is None:
            return res, beats_out
        ipk = drop_peak_index(tplw, fs, wpre)
        nxt = []
        for c in pos1 + (ipk - wpre):
            m = match(env, float(c), tplw[ipk - n_pre:ipk + n_post], n_pre, n_post, int(0.012 * fs))
            if m is not None and m[1] >= opt.quality:
                nxt.append(m[0])
        if len(nxt) < 8:
            return res, beats_out
        pos1 = np.array(nxt)
    tpl1 = median_template(env, pos1, n_pre, n_post)
    ip = drop_peak_index(tpl1, fs, n_pre)
    pos1 = pos1 + (ip - n_pre)
    tpl1 = median_template(env, pos1, n_pre, n_post)
    t1 = np.sort(pos1) / fs
    stretches = number_beats(t1, P)
    # Pass 2: per-side templates from k parity
    order = np.argsort(pos1)
    pos1s = pos1[order]
    k_of = {}
    for st in stretches:
        for i, k in st:
            k_of[i] = k
    best = None
    out_stretches = []
    for st in stretches:
        idx = [i for i, _ in st]
        ks = np.array([k for _, k in st])
        pp = pos1s[idx]
        side_tpl = {}
        for par in (0, 1):
            sel = pp[(ks % 2) == par]
            tp = median_template(env, sel, n_pre, n_post) if len(sel) >= 3 else None
            side_tpl[par] = tp if tp is not None else tpl1
        pos2, q2, k2 = [], [], []
        for p, k in zip(pp, ks):
            tp = side_tpl[int(k % 2)]
            m = match(env, float(p), tp, n_pre, n_post, int(0.003 * fs))
            if m is not None and m[1] >= opt.quality:
                pos2.append(m[0])
                q2.append(m[1])
                k2.append(int(k))
        if len(pos2) < 4:
            continue
        # physical landmark offset of each side's template (samples from its drop peak)
        off = {}
        for par in (0, 1):
            tp = side_tpl[par]
            ipk = drop_peak_index(tp, fs, n_pre)
            td, _ = drop_landmark(tp, fs, ipk, opt.landmarks)
            off[par] = td - ipk
        out_stretches.append({"pos": np.array(pos2), "q": np.array(q2), "k": np.array(k2),
                              "off": off, "tpl": side_tpl, "n_cand": len(st)})
    if not out_stretches:
        return res, beats_out

    # choose last stretch with enough beats
    chosen = None
    for st in reversed(out_stretches):
        if len(st["pos"]) >= 8:
            chosen = st
            break
    if chosen is None:
        return res, beats_out

    pos, q, k = chosen["pos"], chosen["q"], chosen["k"]
    # re-anchor: drop landmark time per beat
    tt = np.array([(p + chosen["off"][int(kk % 2)]) / fs for p, kk in zip(pos, k)])
    span = tt[-1] - tt[0]
    total_audio = len(env) / fs
    waived = total_audio < opt.min_stretch_s
    # label parity: tick = side coming late over the first 5 s
    s_even = np.where(k % 2 == 0, 1.0, -1.0)
    first = tt - tt[0] <= 5.0
    if first.sum() >= 6:
        _, _, c5, _, _ = lsq_fit(tt[first], k[first].astype(float), s_even[first])
    else:
        _, _, c5, _, _ = lsq_fit(tt, k.astype(float), s_even)
    if abs(2 * c5 * 1000) < 0.1:
        tick_par = int(k[0] % 2)
    else:
        tick_par = 0 if c5 > 0 else 1
    s_tick = np.where(k % 2 == tick_par, 1.0, -1.0)
    t0, Pfit, c, keep, resid = lsq_fit(tt, k.astype(float), s_tick)
    be_drop_ms = 2.0 * c * 1000.0
    rate = (P / Pfit - 1.0) * 86400.0
    jitter_us = float(np.sqrt(np.mean(resid[keep] ** 2)) * 1e6)
    n_exp = span / P + 1
    res["locked"] = True
    res["jitter_us"] = jitter_us
    res["lock_fraction"] = float(keep.sum() / n_exp)
    res["first_valid_s"] = t_off + tt[0] + (min(opt.min_stretch_s, span) if not waived else span)
    if span >= opt.min_stretch_s or waived:
        res["rate_s_day"] = float(rate)
        res["rate_waived"] = bool(waived and span < opt.min_stretch_s)
    res["beats_used"] = int(keep.sum())
    res["stretch_span_s"] = float(span)
    res["knocks_gated"] = len(knocks)
    res["be_drop_ms"] = float(be_drop_ms)

    for p, kk, qq, ti, kp, rr in zip(pos, k, q, tt, keep, resid):
        beats_out.append({"type": "beat", "k": int(kk), "side": "tick" if kk % 2 == tick_par else "tock",
                          "t_s": float(t_off + ti), "quality": float(qq),
                          "resid_us": float(rr * 1e6), "used": bool(kp)})

    # ------------------------------------------------- windows (per side shapes)
    if opt.windows == "on":
        win_s, min_beats = opt.window_s, 3
    else:
        win_s, min_beats = 2.0 * P, 1
    good = keep
    wins = {}
    for p, kk, ti, g in zip(pos, k, tt, good):
        if not g:
            continue
        w = int((ti - tt[0]) // win_s)
        wins.setdefault(w, {0: [], 1: []})
        wins[w][0 if kk % 2 == tick_par else 1].append(p)
    wlist = []
    for w in sorted(wins):
        lifts = {}
        for side in (0, 1):
            ps = wins[w][side]
            if len(ps) < min_beats:
                continue
            shapes = np.array([extract(env, p, n_pre, n_post) for p in ps])
            shapes = shapes[~np.isnan(shapes).any(axis=1)]
            if len(shapes) < min_beats:
                continue
            sh = trim_mean(shapes, opt.trim, axis=0) if len(shapes) >= 4 and opt.trim > 0 else shapes.mean(axis=0)
            lift, _, tu = shape_lift(sh, fs, n_pre, opt.landmarks)
            if lift is not None:
                lifts[side] = lift
        wlist.append(lifts)
    n_win = len(wlist)
    amp_t = [amplitude(w[0], bph, L) for w in wlist if 0 in w]
    amp_k = [amplitude(w[1], bph, L) for w in wlist if 1 in w]
    amp_t = [a for a in amp_t if a is not None]
    amp_k = [a for a in amp_k if a is not None]
    both = []
    for w in wlist:
        if 0 in w and 1 in w:
            a0, a1 = amplitude(w[0], bph, L), amplitude(w[1], bph, L)
            if a0 is not None and a1 is not None:
                both.append(0.5 * (a0 + a1))
    res["amp_valid_fraction"] = float(len(both) / n_win) if n_win else 0.0
    res["windows_used"] = len(both)
    res["windows_total"] = n_win
    if amp_t:
        res["amp_tick_deg"] = float(np.median(amp_t))
    if amp_k:
        res["amp_tock_deg"] = float(np.median(amp_k))
    if both:
        res["amp_deg"] = float(np.median(both))

    # --------------------------------------------------------------- beat error
    be_win = [be_drop_ms - 1000 * w[0] + 1000 * w[1] for w in wlist if 0 in w and 1 in w]
    source = "drop"
    be_ms = be_drop_ms
    if opt.timing == "unlock" and be_win:
        m = float(np.median(be_win))
        ok = True
        if len(be_win) >= 3:
            sd = float(np.std(be_win))
            same = float(np.mean(np.sign(be_win) == np.sign(m)))
            expect = 0.5 * (1 + math.erf(abs(m) / (sd * math.sqrt(2)))) if sd > 0 else 1.0
            ok = same >= expect - 0.2
        if ok:
            be_ms, source = m, "unlock"
    res["be_signed_ms"] = float(be_ms)
    res["be_ms"] = float(abs(be_ms))
    res["be_source"] = source
    return res, beats_out


# ----------------------------------------------------------------- CLI helpers
def run_file(path, opt):
    x, fs = sf.read(path, dtype="float64", always_2d=True)
    x = x[:, 0]
    tile = opt.tile
    if tile == 0:  # auto: loops shorter than the 10 s rule are repeated to >= 20 s
        tile = int(math.ceil(20.0 / (len(x) / fs))) if len(x) / fs < opt.min_stretch_s else 1
    if tile > 1:
        x = np.tile(x, tile)
    r, b = analyse(x, fs, os.path.basename(path), opt)
    r["tiled"] = tile
    return r, b


def load_chrono(clips_json):
    try:
        d = json.load(open(clips_json))
        out = {}
        for c in d["clips"]:
            if "chronopulse" in c:
                out[c["id"]] = dict(c["chronopulse"], loop_rate=c.get("expected", {}).get("rate_s_day"))
        return out
    except (OSError, KeyError, ValueError):
        return {}


def load_truth(path):
    tp = os.path.splitext(path)[0] + ".truth.json"
    if not os.path.exists(tp):
        return None
    return json.load(open(tp))["segments"][-1]


def fmt(v, w=7, d=1):
    return f"{'--':>{w}}" if v is None else f"{v:{w}.{d}f}"


def table(paths, opt):
    here = os.path.dirname(os.path.abspath(__file__))
    chrono = load_chrono(os.path.join(here, "..", "..", "assets", "synth_clips", "clips.json"))
    print(f"{'file':34s} {'bph':>6s} {'rate':>7s} {'cp':>6s} {'loop':>6s} {'BE':>6s} {'cp':>5s} {'amp':>6s} {'cpOn':>5s} {'tick':>6s} {'tock':>6s} {'jit_us':>7s} win")
    for p in paths:
        r, _ = run_file(p, opt)
        stem = os.path.splitext(os.path.basename(p))[0]
        cp = chrono.get(stem)
        tr = load_truth(p)
        line = (f"{stem:34s} {r['bph'] or 0:6d} {fmt(r['rate_s_day'])} "
                f"{fmt(cp['rate_s_day'] if cp else None, 6)} {fmt(cp['loop_rate'] if cp else None, 6, 2)} {fmt(r['be_ms'], 6, 2)} "
                f"{fmt(cp['beat_error_ms'] if cp else None, 5, 2)} {fmt(r['amp_deg'], 6, 0)} "
                f"{fmt(cp['amp_onset_deg'] if cp else None, 5, 0)} {fmt(r['amp_tick_deg'], 6, 0)} "
                f"{fmt(r['amp_tock_deg'], 6, 0)} {fmt(r['jitter_us'], 7, 0)} "
                f"{r.get('windows_used', 0)}/{r.get('windows_total', 0)}")
        print(line)
        if tr:
            # the reference labels the late side by the drop BE: be_drop = be + lift_tick - lift_tock
            late_tick = tr["be_ms"] + tr["lift_ms_tick"] - tr["lift_ms_tock"] >= 0
            tick_truth = tr["amp_tick_deg"] if late_tick else tr["amp_tock_deg"]
            tock_truth = tr["amp_tock_deg"] if late_tick else tr["amp_tick_deg"]

            def err(a, b):
                return None if a is None else a - b
            print(f"{'  vs truth (err)':34s} {'':6s} {fmt(err(r['rate_s_day'], tr['rate_s_day']), 7, 2)} "
                  f"{'':6s} {'':6s} {fmt(err(r['be_ms'], abs(tr['be_ms'])), 6, 3)} {'':5s} "
                  f"{fmt(err(r['amp_deg'], tr['amp_deg']), 6, 1)} {'':5s} "
                  f"{fmt(err(r['amp_tick_deg'], tick_truth), 6, 1)} "
                  f"{fmt(err(r['amp_tock_deg'], tock_truth), 6, 1)}")


def contrib(paths, opt):
    here = os.path.dirname(os.path.abspath(__file__))
    chrono = load_chrono(os.path.join(here, "..", "..", "assets", "synth_clips", "clips.json"))
    combos = [("peak", "off"), ("half", "off"), ("peak", "on"), ("half", "on")]
    print(f"{'file':28s} {'cpAmp':>6s} {'cpBE':>5s} | " + " | ".join(f"{lm}/{w:3s} amp   BE" for lm, w in combos))
    sums = {c: [[], []] for c in combos}
    for p in paths:
        stem = os.path.splitext(os.path.basename(p))[0]
        cp = chrono.get(stem)
        row = f"{stem:28s} {fmt(cp['amp_onset_deg'] if cp else None, 6, 0)} {fmt(cp['beat_error_ms'] if cp else None, 5, 2)} |"
        for c in combos:
            o = argparse.Namespace(**vars(opt))
            o.landmarks, o.windows = c
            r, _ = run_file(p, o)
            row += f" {fmt(r['amp_deg'], 9, 0)} {fmt(r['be_ms'], 5, 2)} |"
            if cp and r["amp_deg"] is not None:
                sums[c][0].append(r["amp_deg"] - cp["amp_onset_deg"])
            if cp and r["be_ms"] is not None:
                sums[c][1].append(r["be_ms"] - cp["beat_error_ms"])
        print(row)
    print("\nvs ChronoPulse: mean error / mean |error| / n")
    for c in combos:
        a, b = sums[c]
        fa = f"amp {np.mean(a):+6.1f} / {np.mean(np.abs(a)):5.1f} / {len(a)}" if a else "amp --"
        fb = f"BE {np.mean(b):+6.3f} / {np.mean(np.abs(b)):5.3f} / {len(b)}" if b else "BE --"
        print(f"  landmarks={c[0]:4s} windows={c[1]:3s}: {fa}   {fb}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav", nargs="*")
    ap.add_argument("--timing", choices=["unlock", "drop"], default="unlock")
    ap.add_argument("--landmarks", choices=["half", "peak"], default="half")
    ap.add_argument("--windows", choices=["on", "off"], default="on")
    ap.add_argument("--knock", choices=["on", "off"], default="on")
    ap.add_argument("--frontend", choices=["hp", "raw"], default="hp")
    ap.add_argument("--trim", type=float, default=0.25)
    ap.add_argument("--window-s", type=float, default=2.0)
    ap.add_argument("--min-stretch-s", type=float, default=10.0)
    ap.add_argument("--tile", type=int, default=0,
                    help="repeat the file N times (0 = auto: files under --min-stretch-s are tiled to >= 20 s; "
                         "the synthetic clips are seamless whole-beat loops)")
    ap.add_argument("--quality", type=float, default=0.4)
    ap.add_argument("--lift", type=float, default=52.0)
    ap.add_argument("--bph", type=int, default=None, help="override auto rate")
    ap.add_argument("--beats", action="store_true", help="print per-beat JSON lines first")
    ap.add_argument("--table", action="store_true")
    ap.add_argument("--contrib", action="store_true")
    opt = ap.parse_args()
    if not opt.wav:
        ap.error("no input")
    if opt.table:
        table(opt.wav, opt)
        return
    if opt.contrib:
        contrib(opt.wav, opt)
        return
    for p in opt.wav:
        r, beats = run_file(p, opt)
        if opt.beats:
            for b in beats:
                print(json.dumps(b))
        print(json.dumps(r))


if __name__ == "__main__":
    main()
