#!/usr/bin/env python3
"""Host test for scripts/calibrate_timebase.py fitting (no device needed).

Run: python3 tests/test_calibrate_timebase.py
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from calibrate_timebase import (  # noqa: E402
    Sample,
    fit_ppm,
    is_clock_step,
    select_min_rtt,
)


def simulate(ppm: float, duration_s: float, interval_s: float, seed: int = 1,
             spike_rate: float = 0.05):
    """Device clock ``ppm`` fast; each reply's RTT 0.2-2 ms, the device sample
    landing anywhere inside the round trip (unknown latency asymmetry). A
    fraction ``spike_rate`` of replies hit a 5-50 ms OS / USB scheduling
    hiccup on the return path (the device sampled early in the round trip)."""
    rng = random.Random(seed)
    host0 = 1_763_000_000.0  # 2025-11-13, large like real UNIX time
    dev0 = 3_600_000_000     # device up for an hour
    out = []
    t = 0.0
    while t < duration_s:
        h0 = host0 + t
        if rng.random() < spike_rate:
            rtt = rng.uniform(0.005, 0.050)
            true_at = h0 + rng.uniform(0.0, 0.001)
        else:
            rtt = rng.uniform(0.0002, 0.002)
            true_at = h0 + rng.uniform(0.0, rtt)
        dev = dev0 + int((true_at - host0) * 1e6 * (1.0 + ppm * 1e-6))
        out.append(Sample(host_s=h0 + rtt / 2.0, dev_us=dev, rtt_s=rtt))
        t += interval_s
    return out


def check(name: str, got: float, want: float, tol: float) -> None:
    print(f"  {name:28s} got {got:+9.4f} want {want:+9.4f}")
    assert abs(got - want) <= tol, f"{name}: {got} vs {want} (tol {tol})"


def main() -> int:
    # 30 min, 1 ms-class jitter: recovers 23.4 ppm well inside 0.1 ppm.
    samples = simulate(23.4, 1800.0, 2.0)
    kept = select_min_rtt(samples, 60.0)
    assert len(kept) == 30
    assert all(k.rtt_s <= s.rtt_s for k in kept for s in samples
               if int((s.host_s - samples[0].host_s) // 60) ==
               int((k.host_s - samples[0].host_s) // 60))
    fit = fit_ppm(kept)
    assert fit is not None
    check("30 min, +23.4 ppm", fit.ppm, 23.4, 0.1)
    assert fit.stderr_ppm < 0.1

    # Negative error, shorter run: still sub-ppm.
    fit = fit_ppm(select_min_rtt(simulate(-17.25, 600.0, 2.0, seed=7), 30.0))
    check("10 min, -17.25 ppm", fit.ppm, -17.25, 0.5)

    # With latency spikes, min-RTT selection beats fitting every sample.
    errs_raw, errs_sel = [], []
    for seed in range(20):
        data = simulate(5.0, 600.0, 2.0, seed=seed, spike_rate=0.1)
        errs_raw.append(abs(fit_ppm(data).ppm - 5.0))
        errs_sel.append(abs(fit_ppm(select_min_rtt(data, 30.0)).ppm - 5.0))
    worst_raw, worst_sel = max(errs_raw), max(errs_sel)
    check("worst |err|, all samples", worst_raw, 0.0, 10.0)
    check("worst |err|, min-RTT", worst_sel, 0.0, 0.5)
    assert worst_sel < worst_raw

    # Degenerate input.
    assert fit_ppm([]) is None
    assert fit_ppm(simulate(1.0, 4.0, 2.0)) is None  # 2 points

    # Clock-step detector.
    assert not is_clock_step(None, 5.0)
    assert not is_clock_step(5.0, 5.001)
    assert is_clock_step(5.0, 5.2)

    print("test_calibrate_timebase: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
