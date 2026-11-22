"""Reference timing algorithms for the quartz / LCD pickup head.

This is a REFERENCE, written for clarity, not for a microcontroller: floating point, whole-record
processing, numpy/scipy.  Each function documents the steps a firmware port must reproduce.
Input everywhere: x = ADC codes (int or float) for one channel, fs = sample rate in Hz.
Times are in seconds from the first sample.  The sample clock is GPS-disciplined, so
sample index / fs is the time base; never resample or re-clock the data.
"""
import math
import numpy as np
from scipy import signal

SPD = 86400.0
ALLOWED_STEP_S = (1.0, 2.0, 2.5, 5.0, 10.0, 20.0, 30.0, 60.0)   # motor step intervals seen on watches
TCHOP = 1 / 1024.0


def rate_s_per_day(nominal_period, measured_period):
    """positive = watch gains"""
    return SPD * (nominal_period / measured_period - 1.0)


def highpass(x, fs, fc=400.0, order=4):
    """removes mains hum (60/180 Hz) and any DC. Causal: adds a constant delay, which cancels in rates."""
    sos = signal.butter(order, fc, "highpass", fs=fs, output="sos")
    return signal.sosfilt(sos, np.asarray(x, float))


def robust_sigma(y):
    return 1.4826 * np.median(np.abs(y - np.median(y)))


# =========================================================================== motor pulses
def detect_motor_pulses(x, fs, k_sigma=8.0, gap_s=0.05):
    """Find drive pulses.  Returns list of dicts: i_first (first sample over threshold),
    i_last, i_peak (leading spike), polarity, peak (codes, after high-pass), y (the filtered record)."""
    y = highpass(x, fs)
    sig = robust_sigma(y)
    thr = k_sigma * sig
    idx = np.where(np.abs(y) > thr)[0]
    pulses = []
    if len(idx) == 0:
        return pulses, y, sig
    groups = np.split(idx, np.where(np.diff(idx) > gap_s * fs)[0] + 1)
    for g in groups:
        i0 = g[0]
        w = y[i0: i0 + int(0.3e-3 * fs) + 2]
        ip = i0 + int(np.argmax(np.abs(w)))
        pulses.append({"i_first": int(i0), "i_last": int(g[-1]), "i_peak": ip,
                       "polarity": int(np.sign(y[ip])), "peak": float(abs(y[ip]))})
    return pulses, y, sig


def _frac_xcorr(seg, tmpl, up=64):
    """delay (in samples, fractional) of seg relative to tmpl by band-limited cross-correlation"""
    n = len(seg)
    m = 1 << int(math.ceil(math.log2(2 * n)))
    S = np.fft.rfft(seg, m)
    T = np.fft.rfft(tmpl, m)
    R = S * np.conj(T)
    r = np.fft.irfft(R, m * up)                 # zero-padded spectrum = sinc-interpolated correlation
    k = int(np.argmax(r))
    # parabolic refinement
    a, b, c = r[k - 1], r[k], r[(k + 1) % len(r)]
    d = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
    lag = (k + d) / up
    if lag > m / 2:
        lag -= m
    return lag


def timestamp_pulses(pulses, y, fs, method="xcorr", pre_s=0.2e-3, post_s=0.3e-3):
    """Leading-edge time of each pulse (seconds).

    method 'xcorr' (recommended): cross-correlate a short window around the leading edge with a
      template (the polarity-normalised mean of all pulses, built in two passes), with
      band-limited fractional interpolation.  Window -0.2..+0.3 ms keeps it inside the first
      chopping on-time even at 37.5 % duty (0.366 ms), so pulses of different drive level match.
    method 'threshold': 50 % crossing of the leading spike, linear interpolation between samples.
      Simple, but at 48 kHz the edge is shorter than one sample, so it is biased by the sub-sample
      phase.  Kept for comparison."""
    pre, post = int(round(pre_s * fs)), int(round(post_s * fs))
    if method == "threshold":
        out = []
        for p in pulses:
            ip, s = p["i_peak"], p["polarity"]
            half = 0.5 * s * y[ip]
            k = ip
            while k > 0 and s * y[k - 1] > half:
                k -= 1
            y0, y1 = s * y[k - 1], s * y[k]
            frac = (half - y0) / (y1 - y0) if y1 != y0 else 0.0
            out.append((k - 1 + frac) / fs)
        return np.array(out)
    segs = []
    for p in pulses:
        a = p["i_peak"] - pre
        segs.append(p["polarity"] * y[a: a + pre + post])
    segs = np.array(segs)
    tmpl = segs.mean(axis=0)
    for _ in range(2):                           # re-align and re-average
        lags = np.array([_frac_xcorr(s, tmpl) for s in segs])
        aligned = []
        for s, l in zip(segs, lags):
            n = len(s)
            f = np.fft.rfftfreq(n)
            aligned.append(np.fft.irfft(np.fft.rfft(s) * np.exp(2j * np.pi * f * l), n))
        tmpl = np.mean(aligned, axis=0)
    lags = np.array([_frac_xcorr(s, tmpl) for s in segs])
    # absolute time: segment start + lag + the template's own edge position (constant)
    return np.array([(p["i_peak"] - pre + l) / fs for p, l in zip(pulses, lags)])


def drive_shape(pulse, y, fs, rel=0.3):
    """Pulse width and chopping duty from the edge pattern inside one pulse.
    on-edges have the pulse's polarity, off-edges the opposite; duty = median(off - on) / 976.5625 us."""
    a, b = pulse["i_first"], min(pulse["i_last"] + int(0.5e-3 * fs), len(y) - 1)
    s = pulse["polarity"]
    w = s * y[a:b + 1]
    lim = rel * pulse["peak"]
    pk_on, _ = signal.find_peaks(w, height=lim, distance=max(1, int(0.15e-3 * fs)))
    pk_off, _ = signal.find_peaks(-w, height=lim, distance=max(1, int(0.15e-3 * fs)))

    def interp(i, v):
        if 0 < i < len(v) - 1:
            al, be, ga = v[i - 1], v[i], v[i + 1]
            den = al - 2 * be + ga
            return i + (0.5 * (al - ga) / den if den else 0.0)
        return float(i)
    t_on = np.array([interp(i, w) for i in pk_on]) / fs
    t_off = np.array([interp(i, -w) for i in pk_off]) / fs
    if len(t_on) == 0:
        return None
    dts = []
    for ton in t_on:
        later = t_off[t_off > ton]
        if len(later) and later[0] - ton < TCHOP * 0.999:
            dts.append(later[0] - ton)
    duty = float(np.median(dts) / TCHOP) if dts else 1.0
    duty_q = min(1.0, round(duty / 0.0625) * 0.0625)        # drive levels come in 6.25 % steps
    width = float(t_off[-1] - t_on[0]) if len(t_off) else None
    return {"duty": duty, "duty_quantised": duty_q, "drive_width_s": width,
            "n_chops": int(len(t_on))}


def motor_rate(t, pol, step_s=None, jump_us=20.0):
    """Rate from leading-edge timestamps t (s) and polarities pol (+1/-1).

    1. nominal step: snap the median interval to the allowed list.
    2. step numbers n_k = round((t_k - t_0) / step).
    3. inhibition events: an interval that is longer than the median by more than jump_us
       (true inhibition jumps are >= 61 us; timing noise is ~1 us).
    4. rate WITHOUT inhibition (the raw oscillator rate): least squares t = a_seg + b*n + c*pol,
       with a separate intercept a_seg for each stretch between inhibition events, one common
       slope b and one polarity offset c (alternate pulses can differ slightly in shape).
    5. rate WITH inhibition (what the wearer sees): mean period over a whole number of
       inhibition cycles, from the first pulse after the first event to the first same-polarity
       pulse after the last event.  Needs at least two events, i.e. a gate covering one full cycle.
       With no events: same as 4, flagged as 'inhibition not seen'."""
    t = np.asarray(t, float)
    pol = np.asarray(pol, float)
    iv = np.diff(t)
    if step_s is None:
        med = float(np.median(iv))
        step_s = min(ALLOWED_STEP_S, key=lambda s: abs(s - med))
    n = np.round((t - t[0]) / step_s).astype(int)
    dn = np.diff(n)
    dev = iv - dn * np.median(iv / np.maximum(dn, 1))
    ev = np.where(dev > jump_us * 1e-6)[0]               # event between k and k+1
    seg = np.zeros(len(t), int)
    for e in ev:
        seg[e + 1:] += 1
    nseg = seg.max() + 1
    A = np.zeros((len(t), nseg + 2))
    A[np.arange(len(t)), seg] = 1
    A[:, nseg] = n
    A[:, nseg + 1] = pol
    coef, res, *_ = np.linalg.lstsq(A, t, rcond=None)
    b = coef[nseg]
    resid = t - A @ coef
    out = {"step_s": step_s, "pulses": int(len(t)), "inhibition_events": [int(e) for e in ev],
           "inhibition_jumps_us": [float(dev[e] * 1e6) for e in ev],
           "rate_without_inhibition_s_per_day": rate_s_per_day(step_s, b),
           "residual_rms_us": float(np.sqrt(np.mean(resid ** 2)) * 1e6),
           "polarity_offset_us": float(coef[nseg + 1] * 1e6)}
    if len(ev) >= 2:
        i = ev[0] + 1
        cands = [j for j in range(ev[-1] + 1, len(t)) if pol[j] == pol[i]]
        j = cands[0]
        T_avg = (t[j] - t[i]) / (n[j] - n[i])
        per = float(np.median(np.diff(n[ev + 1])) * step_s)
        out.update({"rate_with_inhibition_s_per_day": rate_s_per_day(step_s, T_avg),
                    "inhibition_period_s": per, "inhibition_cycles_used": int(len(ev) - 1)})
    else:
        out.update({"rate_with_inhibition_s_per_day": out["rate_without_inhibition_s_per_day"],
                    "inhibition_period_s": None,
                    "warning": "fewer than two inhibition events in the record: either the watch has no "
                               "inhibition or the gate is shorter than its cycle (up to 960 s)"})
    return out


# =========================================================================== LCD
def lcd_period(y, fs, candidates_hz=(128.0, 64.0, 32.0, 16.0), seconds=2.0):
    """Pattern repetition rate: the highest candidate whose period gives a strong positive autocorrelation."""
    seg = y[: int(seconds * fs)]
    seg = seg - seg.mean()
    r0 = float(np.dot(seg, seg))
    best = None
    for f in sorted(candidates_hz, reverse=True):
        L = int(round(fs / f))
        r = float(np.dot(seg[:-L], seg[L:])) / r0
        if r > 0.8:
            best = f
            break
    return best


def harmonic_phase_rate(y, fs, f_nom, block_s=0.5, f_max=2500.0, harmonics=None):
    """Rate of a periodic signal with nominal repetition f_nom.

    For each block, the complex amplitude of the strongest harmonics k*f_nom (k*f_nom <= f_max, so
    that +-100 ppm moves the phase by well under half a cycle per block) is taken with a
    single-bin DFT (a Goertzel filter in firmware).  Phases are unwrapped block to block,
    divided by k, and fitted with a straight line; slope / (2*pi*f_nom) is the fractional
    frequency error."""
    nb = int(block_s * fs)
    nblk = len(y) // nb
    t = np.arange(nb) / fs
    ks = harmonics
    if ks is None:
        kmax = int(f_max // f_nom)
        X = np.fft.rfft(y[:nb] * np.hanning(nb))
        mags = [abs(X[int(round(k * f_nom * nb / fs))]) for k in range(1, kmax + 1)]
        order = np.argsort(mags)[::-1]
        ks = [int(o + 1) for o in order[:4]]
    tc = []
    ph = {k: [] for k in ks}
    w = np.hanning(nb)
    for b in range(nblk):
        seg = y[b * nb:(b + 1) * nb] * w
        tc.append((b * nb + nb / 2) / fs)
        for k in ks:
            z = np.sum(seg * np.exp(-2j * np.pi * k * f_nom * (t + b * nb / fs)))
            ph[k].append(np.angle(z))
    tc = np.array(tc)
    slopes, weights = [], []
    for k in ks:
        p = np.unwrap(np.array(ph[k])) / k
        A = np.vstack([tc, np.ones_like(tc)]).T
        (sl, _), res, *_ = np.linalg.lstsq(A, p, rcond=None)
        slopes.append(sl)
        weights.append(k)
    eps = float(np.average(slopes, weights=weights)) / (2 * np.pi * f_nom)
    return {"f_nominal_hz": f_nom, "harmonics_used": ks, "fractional_error": eps,
            "rate_s_per_day": SPD * eps}


def lcd_rate(x, fs):
    y = highpass(x, fs)
    f = lcd_period(y, fs)
    if f is None:
        return {"error": "no periodic LCD signal found"}
    out = harmonic_phase_rate(y, fs, f)
    out["pattern_repetition_hz"] = f
    return out


# =========================================================================== tones
def tone_rate(x, fs, f_nom, block_s=0.1, hp=True):
    """Crystal (32768 Hz, needs fs = 96 kHz) or tuning fork (300-720 Hz): complex demodulation at the
    nominal frequency, phase per block, straight-line fit.  Same maths as harmonic_phase_rate with k = 1."""
    y = highpass(x, fs, fc=min(200.0, f_nom / 2)) if hp else np.asarray(x, float)
    return harmonic_phase_rate(y, fs, f_nom, block_s=block_s, harmonics=[1])
