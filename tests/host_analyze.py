#!/usr/bin/env python3
"""Analyze a mono S16LE WAV from the timegrapher USB mic."""

from __future__ import annotations

import argparse
import math
import struct
import sys
import wave


def load_mono_s16(path: str):
    with wave.open(path, "rb") as w:
        ch = w.getnchannels()
        sw = w.getsampwidth()
        rate = w.getframerate()
        n = w.getnframes()
        raw = w.readframes(n)
    if sw != 2:
        raise SystemExit(f"expected 16-bit samples, got {sw * 8}-bit")
    samples = struct.unpack("<" + "h" * (len(raw) // 2), raw)
    if ch == 2:
        samples = samples[0::2]
    elif ch != 1:
        raise SystemExit(f"unsupported channel count {ch}")
    return rate, list(samples)


def stats(samples):
    n = len(samples)
    if n == 0:
        return {}
    mean = sum(samples) / n
    var = sum((x - mean) ** 2 for x in samples) / n
    rms = math.sqrt(sum(x * x for x in samples) / n)
    peak = max(abs(x) for x in samples)
    mn = min(samples)
    mx = max(samples)
    clipped = sum(1 for x in samples if x <= -32768 or x >= 32767)
    dbfs = 20 * math.log10(rms / 32768.0) if rms > 0 else float("-inf")
    return {
        "n": n,
        "mean": mean,
        "stdev": math.sqrt(var),
        "rms": rms,
        "rms_dbfs": dbfs,
        "min": mn,
        "max": mx,
        "peak": peak,
        "peak_to_peak": mx - mn,
        "clipped_pct": 100.0 * clipped / n,
    }


def estimate_tone_hz(samples, rate: int) -> float | None:
    """Zero-crossing estimate — good enough for 997 Hz synthetic check."""
    if len(samples) < rate // 10:
        return None
    # High-pass-ish: remove mean
    mean = sum(samples) / len(samples)
    s = [x - mean for x in samples]
    crossings = 0
    for i in range(1, len(s)):
        if (s[i - 1] < 0 and s[i] >= 0) or (s[i - 1] >= 0 and s[i] < 0):
            crossings += 1
    duration = len(s) / rate
    if duration <= 0:
        return None
    return (crossings / 2.0) / duration


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("wav")
    p.add_argument("--expect-hz", type=float, default=None)
    args = p.parse_args()

    rate, samples = load_mono_s16(args.wav)
    st = stats(samples)
    tone = estimate_tone_hz(samples, rate)

    print(f"file: {args.wav}")
    print(f"rate: {rate} Hz  samples: {st['n']}  duration: {st['n']/rate:.3f}s")
    print(f"mean: {st['mean']:.2f}  stdev: {st['stdev']:.2f}")
    print(f"rms: {st['rms']:.2f} ({st['rms_dbfs']:.1f} dBFS)")
    print(f"min/max: {st['min']} / {st['max']}  peak-to-peak: {st['peak_to_peak']}")
    print(f"clipped: {st['clipped_pct']:.4f}%")
    if tone is not None:
        print(f"approx tone (ZCR): {tone:.2f} Hz")
        if args.expect_hz is not None:
            err = abs(tone - args.expect_hz)
            print(f"expect {args.expect_hz} Hz  abs err {err:.2f} Hz")
            if err > 20:
                print("WARNING: tone far from expected (check format/resampling)")
                return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
