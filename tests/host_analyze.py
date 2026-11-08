#!/usr/bin/env python3
"""Analyze a mono S24LE or S16LE WAV from the timegrapher USB mic."""

from __future__ import annotations

import argparse
import math
import struct
import sys
import wave


def load_mono(path: str):
    with wave.open(path, "rb") as w:
        ch = w.getnchannels()
        sw = w.getsampwidth()
        rate = w.getframerate()
        n = w.getnframes()
        raw = w.readframes(n)
    if sw == 3:
        # Pack 3-byte LE into signed int32
        samples = []
        for i in range(0, len(raw), 3):
            b0, b1, b2 = raw[i], raw[i + 1], raw[i + 2]
            v = b0 | (b1 << 8) | (b2 << 16)
            if v & 0x800000:
                v |= ~0xFFFFFF
            samples.append(v)
        full_scale = 8388608.0
        clip_lo, clip_hi = -8388608, 8388607
    elif sw == 2:
        samples = list(struct.unpack("<" + "h" * (len(raw) // 2), raw))
        full_scale = 32768.0
        clip_lo, clip_hi = -32768, 32767
    else:
        raise SystemExit(f"expected 16- or 24-bit samples, got {sw * 8}-bit")
    if ch == 2:
        samples = samples[0::2]
    elif ch != 1:
        raise SystemExit(f"unsupported channel count {ch}")
    return rate, samples, full_scale, clip_lo, clip_hi


def stats(samples, full_scale: float, clip_lo: int, clip_hi: int):
    n = len(samples)
    if n == 0:
        return {}
    mean = sum(samples) / n
    var = sum((x - mean) ** 2 for x in samples) / n
    rms = math.sqrt(sum(x * x for x in samples) / n)
    peak = max(abs(x) for x in samples)
    mn = min(samples)
    mx = max(samples)
    clipped = sum(1 for x in samples if x <= clip_lo or x >= clip_hi)
    dbfs = 20 * math.log10(rms / full_scale) if rms > 0 else float("-inf")
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

    rate, samples, full_scale, clip_lo, clip_hi = load_mono(args.wav)
    st = stats(samples, full_scale, clip_lo, clip_hi)
    print(f"rate={rate} n={st.get('n')}")
    for k in ("mean", "stdev", "rms", "rms_dbfs", "min", "max", "peak",
              "peak_to_peak", "clipped_pct"):
        if k in st:
            print(f"  {k}={st[k]}")
    hz = estimate_tone_hz(samples, rate)
    if hz is not None:
        print(f"  tone_est_hz={hz:.2f}")
    if args.expect_hz is not None and hz is not None:
        err = abs(hz - args.expect_hz) / args.expect_hz
        if err > 0.02:
            print(f"tone mismatch: got {hz:.2f} expect {args.expect_hz}",
                  file=sys.stderr)
            sys.exit(1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
