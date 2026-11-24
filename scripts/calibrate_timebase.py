#!/usr/bin/env python3
"""Measure the Timegrapher Stand's crystal error against NTP time.

Legacy: this script drives the serial CLI, which is built only into the
``*_cli`` envs (``TG_SERIAL_CLI=1``, e.g. ``pico2_pcm1808_cli``). Shipped
builds have no CLI on the CDC port; use the web app's PC-assisted calibration
there (docs/build_and_flash.md, docs/TIMEBASE_CALIBRATION.md).

Every clock on the board (µs timer, PCM1808 MCLK, ADC clock) comes from the
same 12 MHz crystal, so the error of the µs timer is the error of every
capture path. This script polls the device's ``time`` CLI command over the
CDC serial port for a while, timestamps each reply with the host's
NTP-disciplined wall clock, and fits a line:

    device_us = a + b * host_us        ppm = (b - 1) * 1e6   (+ = board fast)

Per-sample USB / OS latency (~0.1-2 ms) is cut down NTP-style: each sample's
host time is the midpoint of the request/reply round trip, and only the
lowest-RTT sample per bucket is kept. A 30 min run typically resolves
0.05-0.3 ppm, limited mainly by how well the host's NTP daemon holds its
frequency.

Usage:
    python3 scripts/calibrate_timebase.py --port /dev/cu.usbmodem101
    python3 scripts/calibrate_timebase.py --port COM5 --duration 3600 --apply --save
    python3 scripts/calibrate_timebase.py --port ... --ntp-server pool.ntp.org

Let the board warm up (~10 min powered, ideally streaming) before measuring:
the crystal drifts with temperature. ``--save`` needs the audio stream closed
(flash writes stall the CPU). Requires pyserial; ``--ntp-server`` also needs
ntplib. See docs/TIMEBASE_CALIBRATION.md.
"""

from __future__ import annotations

import argparse
import math
import platform
import re
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence

TIME_RE = re.compile(r"CLI: time t_us=(\d+)")
# 1 ppm = 0.0864 s/day.
S_PER_DAY_PER_PPM = 0.0864
# A host clock step larger than this between samples restarts the run.
STEP_LIMIT_S = 0.005


@dataclass
class Sample:
    host_s: float  # midpoint of the round trip, host wall clock (s)
    dev_us: int    # device time_us_64()
    rtt_s: float   # round-trip time (s)


@dataclass
class Fit:
    ppm: float
    stderr_ppm: float
    resid_rms_us: float
    points: int
    span_s: float


def select_min_rtt(samples: Sequence[Sample], bucket_s: float) -> List[Sample]:
    """Keeps the lowest-RTT sample in each ``bucket_s`` slice of host time."""
    if not samples:
        return []
    t0 = samples[0].host_s
    best: dict[int, Sample] = {}
    for s in samples:
        k = int((s.host_s - t0) // bucket_s)
        if k not in best or s.rtt_s < best[k].rtt_s:
            best[k] = s
    return [best[k] for k in sorted(best)]


def fit_ppm(points: Sequence[Sample]) -> Optional[Fit]:
    """Least-squares slope of device time against host time.

    Values are centred before the fit; host seconds since 1970 and device
    microseconds since boot are both far too large to square directly.
    """
    n = len(points)
    if n < 3:
        return None
    h0 = points[0].host_s
    d0 = points[0].dev_us
    xs = [(p.host_s - h0) * 1e6 for p in points]  # host µs
    ys = [float(p.dev_us - d0) for p in points]   # device µs
    xbar = sum(xs) / n
    ybar = sum(ys) / n
    sxx = sum((x - xbar) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    sxy = sum((x - xbar) * (y - ybar) for x, y in zip(xs, ys))
    b = sxy / sxx
    a = ybar - b * xbar
    resid = [y - (a + b * x) for x, y in zip(xs, ys)]
    rss = sum(r * r for r in resid)
    sigma = math.sqrt(rss / (n - 2)) if n > 2 else 0.0
    stderr_b = sigma / math.sqrt(sxx)
    return Fit(
        ppm=(b - 1.0) * 1e6,
        stderr_ppm=stderr_b * 1e6,
        resid_rms_us=math.sqrt(rss / n),
        points=n,
        span_s=(xs[-1] - xs[0]) / 1e6,
    )


def is_clock_step(prev_offset: Optional[float], offset: float) -> bool:
    """True if ``time.time() - time.monotonic()`` jumped (an NTP step)."""
    return prev_offset is not None and abs(offset - prev_offset) > STEP_LIMIT_S


def check_host_ntp() -> str:
    """Best-effort report of whether the host clock is NTP-synchronised."""
    system = platform.system()
    try:
        if system == "Linux":
            out = subprocess.run(
                ["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                capture_output=True, text=True, timeout=5,
            ).stdout.strip()
            return "synchronised" if out == "yes" else f"NOT synchronised ({out or '?'})"
        if system == "Darwin":
            out = subprocess.run(
                ["systemsetup", "-getusingnetworktime"],
                capture_output=True, text=True, timeout=5,
            ).stdout.strip()
            return out or "unknown (macOS timed)"
        if system == "Windows":
            out = subprocess.run(
                ["w32tm", "/query", "/status"],
                capture_output=True, text=True, timeout=5,
            ).stdout
            m = re.search(r"Source:\s*(.+)", out)
            return f"source {m.group(1).strip()}" if m else "unknown"
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"


class Device:
    """Minimal line-oriented client for the CDC CLI."""

    def __init__(self, port: str, timeout_s: float = 0.5) -> None:
        import serial  # pyserial; imported here so tests do not need it

        self.ser = serial.Serial(port, 115200, timeout=timeout_s)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def _send(self, line: str) -> None:
        self.ser.write((line + "\r").encode("ascii"))
        self.ser.flush()

    def command(self, line: str, expect: str, timeout_s: float = 2.0) -> str:
        """Sends ``line`` and returns the first reply line containing ``expect``."""
        self.ser.reset_input_buffer()
        self._send(line)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.ser.readline().decode("ascii", "replace").strip()
            if expect in raw:
                return raw
        raise TimeoutError(f"no '{expect}' reply to '{line}'")

    def sample(self) -> Sample:
        self.ser.reset_input_buffer()
        h0 = time.time()
        self._send("time")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            h1 = time.time()
            m = TIME_RE.search(raw.decode("ascii", "replace"))
            if m:
                return Sample(host_s=0.5 * (h0 + h1), dev_us=int(m.group(1)),
                              rtt_s=h1 - h0)
        raise TimeoutError("no reply to 'time'")


def ntp_time(server: str) -> float:
    """UNIX time from ``server`` (offset applied to the local clock)."""
    import ntplib  # optional dependency

    resp = ntplib.NTPClient().request(server, version=3, timeout=5)
    return time.time() + resp.offset


def best_of(dev: Device, n: int = 10) -> Sample:
    return min((dev.sample() for _ in range(n)), key=lambda s: s.rtt_s)


def run(args: argparse.Namespace) -> int:
    print(f"host clock: {check_host_ntp()}")
    dev = Device(args.port)
    print(dev.command("timebase", "CLI: timebase"))

    ntp_start = None
    if args.ntp_server:
        ref = best_of(dev)
        ntp_start = (ntp_time(args.ntp_server), ref)

    samples: List[Sample] = []
    prev_offset: Optional[float] = None
    t_end = time.monotonic() + args.duration
    next_report = time.monotonic() + 60.0
    print(f"sampling every {args.interval:g} s for {args.duration:g} s ...")
    while time.monotonic() < t_end:
        offset = time.time() - time.monotonic()
        if is_clock_step(prev_offset, offset):
            print(f"host clock stepped by {offset - prev_offset:+.3f} s; restarting")
            samples.clear()
            t_end = time.monotonic() + args.duration
        prev_offset = offset
        try:
            samples.append(dev.sample())
        except TimeoutError as e:
            print(f"warning: {e}")
        if time.monotonic() >= next_report:
            next_report += 60.0
            fit = fit_ppm(select_min_rtt(samples, args.bucket))
            if fit:
                print(f"  {fit.span_s:6.0f} s  ppm {fit.ppm:+8.3f} ± {fit.stderr_ppm:.3f}")
        time.sleep(args.interval)

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("host_s,dev_us,rtt_s\n")
            for s in samples:
                f.write(f"{s.host_s:.6f},{s.dev_us},{s.rtt_s:.6f}\n")

    kept = select_min_rtt(samples, args.bucket)
    fit = fit_ppm(kept)
    if not fit:
        print("error: not enough samples")
        return 1
    rtts = [s.rtt_s * 1e3 for s in samples]
    print()
    print(f"samples      {len(samples)} ({fit.points} kept, min-RTT per {args.bucket:g} s)")
    print(f"rtt ms       min {min(rtts):.2f}  median {statistics.median(rtts):.2f}"
          f"  max {max(rtts):.2f}")
    print(f"span         {fit.span_s:.0f} s")
    print(f"residual     {fit.resid_rms_us:.0f} µs rms")
    print(f"clock error  {fit.ppm:+.3f} ppm ± {fit.stderr_ppm:.3f}  "
          f"({fit.ppm * S_PER_DAY_PER_PPM:+.3f} s/d on a watch reading)")
    if fit.stderr_ppm > 0.5:
        print("warning: uncertainty > 0.5 ppm; run longer or check the USB link")

    if ntp_start:
        ref = best_of(dev)
        ntp_end = ntp_time(args.ntp_server)
        (t0, r0) = ntp_start
        ppm = ((ref.dev_us - r0.dev_us) / 1e6 / (ntp_end - t0) - 1.0) * 1e6
        print(f"ntp check    {ppm:+.3f} ppm vs {args.ntp_server} "
              f"(each query ±~5 ms; agreement within "
              f"{0.01 / max(fit.span_s, 1) * 1e6:.1f} ppm is expected)")

    if args.apply:
        line = (f"timebase set {fit.ppm:.3f} ntp {int(time.time())} "
                f"{int(round(fit.span_s))}")
        print(dev.command(line, "CLI: timebase"))
        if args.save:
            print(dev.command("timebase save", "CLI:"))
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    p = argparse.ArgumentParser(description="\n\n".join(__doc__.split("\n\n")[0:2]))
    p.add_argument("--port", required=True, help="CDC serial port of the device")
    p.add_argument("--duration", type=float, default=1800.0, help="seconds (default 1800)")
    p.add_argument("--interval", type=float, default=2.0, help="seconds between samples")
    p.add_argument("--bucket", type=float, default=60.0,
                   help="keep the lowest-RTT sample per this many seconds")
    p.add_argument("--ntp-server", help="cross-check against this NTP server (ntplib)")
    p.add_argument("--csv", help="write raw samples to this file")
    p.add_argument("--apply", action="store_true", help="send the result to the device")
    p.add_argument("--save", action="store_true", help="also persist it in flash")
    args = p.parse_args(argv)
    if args.save and not args.apply:
        p.error("--save requires --apply")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
