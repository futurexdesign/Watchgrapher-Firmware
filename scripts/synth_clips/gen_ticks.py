#!/usr/bin/env python3
"""Parametric mechanical-watch tick generator with exact ground truth.

Implements task T0a of docs/plans/TIMING_ENGINE_PLAN.md (section 2, "0a").
Each scenario becomes a mono 16-bit 48 kHz WAV plus a <name>.truth.json that
records the rate, signed beat error, lift time per side, impairments and the
unlock/drop time of every beat. The WAVs measure the timing chain against
numbers we control, instead of another instrument's opinion.

Tick model (per beat): three damped-sinusoid bursts
  unlock   small, ~5 kHz, ~1 ms decay, at t_unlock
  impulse  medium, ~4 kHz, half-way through the lift
  drop     largest, ~3.8 kHz, ~1.5 ms decay, at t_drop = t_unlock + lift(side)
Beat times: t_unlock(k) = t0 + k*P + (+e/2 tick, -e/2 tock) + jitter,
P = (3600/bph) / (1 + rate/86400), e = be_ms/1000. Tick = even k, Tock = odd k.
Amplitude truth: A = L / (2 sin(pi t / T)), T = 7200/bph s, L = 52 deg.

Usage:
  gen_ticks.py [--out DIR] [--only NAME ...]   write WAVs, truth JSON, README
  gen_ticks.py --check NAME ...                refit rate and BE from the
                                               written beat times and compare
                                               with the truth (needs a prior run)
Needs numpy and soundfile. All randomness uses fixed seeds: output is
byte-for-byte reproducible for a given numpy version.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import soundfile as sf

SR = 48000
LIFT_ANGLE_DEG = 52.0
DROP_PEAK = 0.08  # full-scale fraction; real clips peak at 0.04-0.11
DEFAULT_SNR_DB = 40.0  # "clean": a quiet noise floor, not silence
HUM_HARMONICS = ((1, 1.0), (2, 0.5), (3, 0.3))  # (multiple, relative amplitude)

# (frequency Hz, decay time constant s, level relative to the drop peak)
UNLOCK = (5000.0, 0.40e-3, 0.25)
IMPULSE = (4000.0, 0.60e-3, 0.50)
DROP = (3800.0, 0.50e-3, 1.00)
ATTACK_S = 0.03e-3  # onset ramp: keeps the burst start sample-accurate but smooth
AMP_SPREAD = 0.05  # per-burst random level spread (relative)
FREQ_SPREAD = 0.03  # per-burst random frequency spread (relative)


def default_lift_ms(bph):
    """Lift time that gives about 296 deg: it scales with the beat period T."""
    return 9.3 * (7200.0 / bph) / (7200.0 / 21600.0)


def amplitude_deg(lift_ms, bph):
    """A = L / (2 sin(pi t / T)), T = 7200/bph s (two beats)."""
    period_t = 7200.0 / bph
    return LIFT_ANGLE_DEG / (2.0 * math.sin(math.pi * (lift_ms / 1000.0) / period_t))


def unit_peak(freq, tau):
    """Peak of the unit burst, to scale the drop to exactly DROP_PEAK."""
    t = np.arange(0, 12 * tau, 1.0 / (SR * 8))
    return float(np.max(np.abs(burst_shape(t, freq, tau))))


def burst_shape(dt, freq, tau):
    """Damped sinusoid with a short onset ramp, zero for dt < 0."""
    dtp = np.maximum(dt, 0.0)
    return np.where(
        dt >= 0.0,
        (1.0 - np.exp(-dtp / ATTACK_S)) * np.exp(-dtp / tau) * np.sin(2.0 * math.pi * freq * dtp),
        0.0,
    )


def add_burst(x, t0, spec, level, rng):
    """Add one burst starting at time t0 (s, fractional-sample exact)."""
    freq, tau, rel = spec
    freq *= 1.0 + FREQ_SPREAD * rng.standard_normal()
    amp = level * rel * (1.0 + AMP_SPREAD * rng.standard_normal())
    first = int(math.floor(t0 * SR))
    last = min(first + int(10 * tau * SR) + 2, len(x))
    if first >= len(x) or last <= 0:
        return
    first = max(first, 0)
    dt = np.arange(first, last) / SR - t0
    x[first:last] += amp * burst_shape(dt, freq, tau) / NORM[spec]


NORM = {spec: unit_peak(spec[0], spec[1]) for spec in (UNLOCK, IMPULSE, DROP)}


def beat_grid(seg, rng_unused=None):
    """Nominal beat times g_k (before beat error and jitter) for one segment.

    A constant rate uses the closed form; a drifting rate integrates the period
    beat by beat so that the instantaneous rate follows the linear ramp.
    """
    nominal = 3600.0 / seg["bph"]
    t_start = seg["from_s"] + seg.get("lead_s", 0.25)
    grid = []
    t = t_start
    while t < seg["to_s"] - 0.05:
        grid.append(t)
        frac = (t - seg["from_s"]) / (seg["to_s"] - seg["from_s"])
        rate = seg["rate"] + (seg.get("rate_end", seg["rate"]) - seg["rate"]) * frac
        t += nominal / (1.0 + rate / 86400.0)
    return np.array(grid)


def knock(x, t0, dur_s, energy, rng):
    """Add a loud broadband burst of given duration and energy (sum of squares)."""
    first = int(t0 * SR)
    n = int(dur_s * SR)
    if first + n > len(x):
        return
    env = np.hanning(n) ** 2
    noise = rng.standard_normal(n) * env
    noise *= math.sqrt(energy / float(np.sum(noise**2)))
    x[first : first + n] += noise


def handling_noise(x, t_from, t_to, rng):
    """Watch-change handling: rumble, scrapes and a few thumps, no ticks."""
    a, b = int(t_from * SR), int(t_to * SR)
    n = b - a
    base = rng.standard_normal(n)
    # slow random envelope (about 5 Hz) so the noise comes in gusts
    knots = rng.random(max(2, int((t_to - t_from) * 8)))
    env = np.interp(np.linspace(0, len(knots) - 1, n), np.arange(len(knots)), knots) ** 2
    x[a:b] += base * env * DROP_PEAK * 0.35
    for _ in range(int((t_to - t_from) * 2)):
        t0 = t_from + rng.random() * (t_to - t_from - 0.05)
        knock(x, t0, 0.005 + 0.015 * rng.random(), 3.0 * drop_energy(), rng)


def drop_energy():
    """Sum of squares of one drop burst at DROP_PEAK."""
    n = int(10 * DROP[1] * SR)
    dt = np.arange(n) / SR
    s = DROP_PEAK * burst_shape(dt, DROP[0], DROP[1]) / NORM[DROP]
    return float(np.sum(s**2))


def synthesize(sc):
    """Render a scenario dict. Returns (samples float64, truth dict)."""
    rng = np.random.default_rng(sc["seed"])
    seconds = sc["seconds"]
    x = np.zeros(int(round(seconds * SR)))
    beats = []
    segments = []
    jitter = sc.get("jitter_us", 0.0) * 1e-6
    louder = sc.get("unlock_louder", False)
    for seg in sc["segments"]:
        bph = seg["bph"]
        lift_tick = seg.get("lift_ms_tick", default_lift_ms(bph))
        lift_tock = seg.get("lift_ms_tock", seg.get("lift_ms_tick", default_lift_ms(bph)))
        e = seg.get("be_ms", 0.0) / 1000.0
        grid = beat_grid(seg)
        seg_beats = []
        for k, g in enumerate(grid):
            side = "tick" if k % 2 == 0 else "tock"
            t_unlock = g + (e / 2.0 if side == "tick" else -e / 2.0)
            t_unlock += jitter * rng.standard_normal()
            lift = (lift_tick if side == "tick" else lift_tock) / 1000.0
            t_drop = t_unlock + lift
            unlock_level = DROP_PEAK * (1.5 if (louder and k % 2 == 1) else 1.0)
            add_burst(x, t_unlock, UNLOCK, unlock_level, rng)
            add_burst(x, t_unlock + 0.5 * lift, IMPULSE, DROP_PEAK, rng)
            add_burst(x, t_drop, DROP, DROP_PEAK, rng)
            seg_beats.append(
                {"k": k, "side": side, "t_unlock_s": round(t_unlock, 7), "t_drop_s": round(t_drop, 7)}
            )
        beats.extend(seg_beats)
        t_a, t_b = grid[0], grid[-1]
        mean_p = (t_b - t_a) / (len(grid) - 1)
        rate_mean = (3600.0 / bph / mean_p - 1.0) * 86400.0
        amp_tick = amplitude_deg(lift_tick, bph)
        amp_tock = amplitude_deg(lift_tock, bph)
        entry = {
            "from_s": seg["from_s"],
            "to_s": seg["to_s"],
            "bph": bph,
            "rate_s_day": round(rate_mean, 4),
            "be_ms": seg.get("be_ms", 0.0),
            "lift_ms_tick": round(lift_tick, 4),
            "lift_ms_tock": round(lift_tock, 4),
            "amp_tick_deg": round(amp_tick, 3),
            "amp_tock_deg": round(amp_tock, 3),
            "amp_deg": round(0.5 * (amp_tick + amp_tock), 3),
        }
        if "rate_end" in seg:
            entry["rate_start_s_day"] = seg["rate"]
            entry["rate_end_s_day"] = seg["rate_end"]
        segments.append(entry)

    for t_from, t_to in sc.get("handling", []):
        handling_noise(x, t_from, t_to, rng)

    knocks = []
    for t0 in sc.get("knocks", []):
        knock(x, t0, 0.005 + 0.015 * rng.random(), (3.0 + 2.0 * rng.random()) * drop_energy(), rng)
        knocks.append(round(t0, 4))

    if sc.get("hum_hz", 0):
        t = np.arange(len(x)) / SR
        hum = sum(a * np.sin(2.0 * math.pi * sc["hum_hz"] * m * t) for m, a in HUM_HARMONICS)
        x += hum * (DROP_PEAK / 3.0)

    snr_db = sc.get("snr_db", DEFAULT_SNR_DB)
    x += rng.standard_normal(len(x)) * DROP_PEAK * 10.0 ** (-snr_db / 20.0)

    clip = sc.get("clip_rel")
    clip_level = None
    if clip is not None:
        clip_level = round(clip * DROP_PEAK, 5)
        x = np.clip(x, -clip_level, clip_level)

    for t_from, t_to in sc.get("dropouts", []):
        x[int(t_from * SR) : int(t_to * SR)] = 0.0
    for b in beats:  # a beat whose unlock or drop falls inside a dropout is inaudible
        for t_from, t_to in sc.get("dropouts", []):
            if t_from - 0.002 <= b["t_unlock_s"] <= t_to or t_from <= b["t_drop_s"] <= t_to:
                b["audible"] = False

    truth = {
        "file": sc["name"] + ".wav",
        "sample_rate_hz": SR,
        "seconds": seconds,
        "lift_angle_deg": int(LIFT_ANGLE_DEG),
        "segments": segments,
        "impairments": {
            "snr_db": snr_db,
            "jitter_us_rms": sc.get("jitter_us", 0.0),
            "clip_level": clip_level,
            "hum_hz": sc.get("hum_hz", 0),
            "knocks_s": knocks,
            "dropouts_s": [list(d) for d in sc.get("dropouts", [])],
            "unlock_louder_than_drop": louder,
        },
        "beats": beats,
    }
    if sc.get("handling"):
        truth["impairments"]["handling_s"] = [list(h) for h in sc["handling"]]
    return x, truth


def seg(bph, rate=0.0, be_ms=0.0, from_s=0.0, to_s=20.0, **kw):
    """Segment spec helper."""
    return dict(bph=bph, rate=rate, be_ms=be_ms, from_s=from_s, to_s=to_s, **kw)


def scenarios():
    """The scenario table. Seeds are fixed per row."""
    s = []

    def add(name, desc, seconds=20.0, segs=None, **kw):
        s.append(dict(name=name, desc=desc, seconds=seconds, seed=1000 + len(s),
                      segments=segs or [seg(21600, to_s=seconds)], **kw))

    for bph in (18000, 21600, 28800, 36000):
        add(f"g_{bph}_clean", f"{bph} bph, +6 s/d, BE 0, 40 dB SNR",
            segs=[seg(bph, rate=6.0)])
    add("g_21600_rate_p25", "rate +25 s/d", segs=[seg(21600, rate=25.0)])
    add("g_21600_rate_m30", "rate -30 s/d", segs=[seg(21600, rate=-30.0)])
    add("g_21600_be_p0.3", "BE +0.3 ms (tick late), +4 s/d", segs=[seg(21600, 4.0, 0.3)])
    add("g_21600_be_m0.8", "BE -0.8 ms (tock late), +4 s/d", segs=[seg(21600, 4.0, -0.8)])
    add("g_21600_lift_unequal", "lift tick 9.0 ms, tock 9.6 ms, BE +0.2 ms",
        segs=[seg(21600, 4.0, 0.2, lift_ms_tick=9.0, lift_ms_tock=9.6)])
    add("g_21600_snr20", "20 dB SNR (re drop peak), +5 s/d", segs=[seg(21600, 5.0)], snr_db=20.0)
    add("g_21600_snr12", "12 dB SNR (re drop peak), +5 s/d", segs=[seg(21600, 5.0)], snr_db=12.0)
    add("g_21600_jitter50", "50 us RMS unlock jitter, +5 s/d", segs=[seg(21600, 5.0)], jitter_us=50.0)
    add("g_21600_jitter300", "300 us RMS unlock jitter, +5 s/d", segs=[seg(21600, 5.0)], jitter_us=300.0)
    add("g_21600_clip30", "hard clipped at 30 % of drop peak", segs=[seg(21600, 5.0)], clip_rel=0.3)
    add("g_21600_hum60", "60 Hz hum + harmonics, fundamental = drop peak / 3",
        segs=[seg(21600, 5.0)], hum_hz=60)
    add("g_21600_knocks", "8 off-grid knocks (5-20 ms, 3-5x drop energy)",
        segs=[seg(21600, 5.0)],
        knocks=[2.13, 4.71, 7.36, 9.92, 12.58, 14.27, 16.84, 18.55])
    add("g_21600_dropouts", "3 silence intervals (0.3 s, 0.6 s, 1.2 s)",
        segs=[seg(21600, 5.0)], dropouts=[(4.0, 4.3), (9.0, 9.6), (14.0, 15.2)])
    add("g_21600_unlock_louder", "unlock 1.5x drop level on every tock",
        segs=[seg(21600, 5.0)], unlock_louder=True)
    add("g_change_21600_28800", "watch change: 21600 bph (12 s), handling noise (5 s), 28800 bph (12 s)",
        seconds=29.0,
        segs=[seg(21600, 8.0, 0.2, 0.0, 12.0),
              seg(28800, -5.0, -0.4, 17.0, 29.0, lead_s=0.3)],
        handling=[(12.0, 17.0)])
    add("g_21600_drift", "rate drifts linearly +10 -> +14 s/d over the file",
        seconds=30.0, segs=[seg(21600, 10.0, 0.0, 0.0, 30.0, rate_end=14.0)])
    return s


def write_scenario(sc, out):
    x, truth = synthesize(sc)
    peak = float(np.max(np.abs(x)))
    if peak >= 0.99:
        raise RuntimeError(f"{sc['name']}: peak {peak:.3f} would clip")
    sf.write(out / (sc["name"] + ".wav"), x, SR, subtype="PCM_16")
    with open(out / (sc["name"] + ".truth.json"), "w") as f:
        json.dump(truth, f, indent=1)
        f.write("\n")
    return truth


def fmt_highlights(truth):
    parts = []
    for sg in truth["segments"]:
        rate = (f"{sg['rate_start_s_day']:+.0f}..{sg['rate_end_s_day']:+.0f}"
                if "rate_start_s_day" in sg else f"{sg['rate_s_day']:+.2f}")
        parts.append(f"{sg['bph']} bph, rate {rate} s/d, BE {sg['be_ms']:+.2f} ms, "
                     f"lift {sg['lift_ms_tick']:.1f}/{sg['lift_ms_tock']:.1f} ms, "
                     f"amp {sg['amp_tick_deg']:.0f}/{sg['amp_tock_deg']:.0f} deg")
    return "; ".join(parts)


README_HEAD = """# Generated tick scenarios

Synthetic mechanical-watch recordings with exact ground truth, for measuring the
timing chain (`docs/plans/TIMING_ENGINE_PLAN.md`, Phase 0, task 0a). Each
`<name>.wav` (mono, 16-bit, 48 kHz) has a `<name>.truth.json` beside it.

## Regenerate

```bash
python3 scripts/synth_clips/gen_ticks.py            # all scenarios into assets/synth_clips/generated/
python3 scripts/synth_clips/gen_ticks.py --only g_21600_clean g_21600_snr20
python3 scripts/synth_clips/gen_ticks.py --check g_21600_be_p0.3   # refit rate and BE from the written beats
```

Needs Python 3 with numpy and soundfile. Seeds are fixed, so the output is
reproducible. This README is rewritten by a full run.

## Model

- Three damped-sinusoid bursts per beat: unlock (5 kHz, 0.25 of the drop level),
  impulse (4 kHz, 0.5, mid-lift), drop (3.8 kHz, 1.0, peak {drop_peak} FS).
- `t_unlock(k) = t0 + k*P + (+e/2 tick, -e/2 tock) + jitter`,
  `P = (3600/bph) / (1 + rate/86400)`, `t_drop = t_unlock + lift(side)`.
  Tick = even `k`, Tock = odd `k`; `k` restarts at 0 in each segment.
- Amplitude truth: `A = L / (2 sin(pi t / T))`, `T = 7200/bph` s, `L = 52` deg.
  The default lift scales with the beat period (9.3 ms at 21,600 bph, about 296 deg).
- Noise is white; SNR is relative to the drop peak. Scenarios without an SNR
  impairment carry {snr} dB. Hum is 50/60 Hz plus 2nd and 3rd harmonics (0.5, 0.3).
- Knocks and handling noise are broadband bursts of 5-20 ms.
- Beats whose unlock or drop falls in a dropout carry `"audible": false` (an
  addition to the shared truth format; absent otherwise).
- A drifting rate is integrated beat by beat; the segment's `rate_s_day` is the
  mean over the segment and `rate_start_s_day` / `rate_end_s_day` give the ramp.
- The watch-change scenario has no beats during the handling interval
  (`impairments.handling_s`).

## Scenarios

| File | Length | Size | Truth highlights |
|---|---|---|---|
"""


def write_readme(out, rows):
    text = README_HEAD.format(drop_peak=DROP_PEAK, snr=int(DEFAULT_SNR_DB))
    for name, seconds, size, sc, truth in rows:
        imp = truth["impairments"]
        extra = []
        if sc["name"].endswith("snr20") or sc["name"].endswith("snr12"):
            extra.append(f"SNR {imp['snr_db']:.0f} dB")
        if imp["jitter_us_rms"]:
            extra.append(f"jitter {imp['jitter_us_rms']:.0f} us RMS")
        if imp["clip_level"]:
            extra.append(f"clip at {imp['clip_level']:.3f} FS")
        if imp["hum_hz"]:
            extra.append(f"hum {imp['hum_hz']} Hz")
        if imp["knocks_s"]:
            extra.append(f"{len(imp['knocks_s'])} knocks")
        if imp["dropouts_s"]:
            extra.append(f"{len(imp['dropouts_s'])} dropouts")
        if imp["unlock_louder_than_drop"]:
            extra.append("unlock louder on tocks")
        if imp.get("handling_s"):
            extra.append("handling noise " + ", ".join(f"{a:.0f}-{b:.0f} s" for a, b in imp["handling_s"]))
        note = fmt_highlights(truth) + ("; " + ", ".join(extra) if extra else "")
        text += f"| `{name}.wav` | {seconds:.0f} s | {size / 1e6:.2f} MB | {note} |\n"
    text += f"\nTotal: {sum(r[2] for r in rows) / 1e6:.1f} MB.\n"
    (out / "README.md").write_text(text)


def check(out, names):
    """Refit t = a + P k + c s to the written unlock times and compare with truth."""
    ok = True
    for name in names:
        truth = json.loads((out / (name + ".truth.json")).read_text())
        for i, sg in enumerate(truth["segments"]):
            beats = [b for b in truth["beats"] if sg["from_s"] <= b["t_unlock_s"] <= sg["to_s"]
                     and b.get("audible", True)]
            k = np.array([b["k"] for b in beats], float)
            sgn = np.array([1.0 if b["side"] == "tick" else -1.0 for b in beats])
            t = np.array([b["t_unlock_s"] for b in beats])
            a_mat = np.column_stack([np.ones_like(k), k, sgn])
            (a, p, c), *_ = np.linalg.lstsq(a_mat, t, rcond=None)
            rate = (3600.0 / sg["bph"] / p - 1.0) * 86400.0
            be_ms = 2.0 * c * 1000.0
            d_rate, d_be = rate - sg["rate_s_day"], be_ms - sg["be_ms"]
            jit = truth["impairments"]["jitter_us_rms"] * 1e-6
            n = len(beats)
            period = 3600.0 / sg["bph"]
            # one-sigma slope and offset errors of the fit, with jitter as the only noise
            sd_rate = jit / (period * math.sqrt(n**3 / 12.0)) * 86400.0 if n > 2 else 0.0
            sd_be = jit * 2.0 / math.sqrt(n) * 1000.0
            tol_rate = 0.01 + 4.0 * sd_rate
            if "rate_start_s_day" in sg:
                tol_rate = 0.3  # a linear drift fitted by a constant rate
            good = abs(d_rate) < tol_rate and abs(d_be) < 0.01 + 4.0 * sd_be
            ok &= good
            print(f"{name} seg{i}: beats {len(beats)}  rate fit {rate:+.4f} truth {sg['rate_s_day']:+.4f} "
                  f"(d {d_rate:+.4f} s/d)  BE fit {be_ms:+.4f} truth {sg['be_ms']:+.4f} "
                  f"(d {d_be:+.4f} ms)  {'OK' if good else 'MISMATCH'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).resolve().parents[2] / "assets" / "synth_clips" / "generated")
    ap.add_argument("--only", nargs="*", help="generate only these scenario names")
    ap.add_argument("--check", nargs="*", help="refit written beat times and compare with truth")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.check is not None:
        return 0 if check(args.out, args.check) else 1
    rows = []
    for sc in scenarios():
        if args.only and sc["name"] not in args.only:
            continue
        truth = write_scenario(sc, args.out)
        size = (args.out / (sc["name"] + ".wav")).stat().st_size
        rows.append((sc["name"], sc["seconds"], size, sc, truth))
        print(f"{sc['name']:28s} {sc['seconds']:5.1f} s {size / 1e6:6.2f} MB  {sc['desc']}")
    if not args.only:
        write_readme(args.out, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
