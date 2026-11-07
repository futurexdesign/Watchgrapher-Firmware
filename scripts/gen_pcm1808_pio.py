#!/usr/bin/env python3
"""Regenerate include/pcm1808_i2s.pio.h from src/pcm1808_i2s.pio via pioasm."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src" / "pcm1808_i2s.pio"
OUT = ROOT / "include" / "pcm1808_i2s.pio.h"

CANDIDATES = [
    os.environ.get("PIOASM"),
    str(Path.home() / ".platformio/packages/tool-pioasm-rp2040-earlephilhower/pioasm"),
    "pioasm",
]


def find_pioasm() -> str:
    for c in CANDIDATES:
        if not c:
            continue
        p = Path(c)
        if p.is_file() and os.access(p, os.X_OK):
            return str(p)
        # bare name on PATH
        from shutil import which

        w = which(c)
        if w:
            return w
    raise SystemExit(
        "pioasm not found. Set PIOASM=... or install PlatformIO package "
        "tool-pioasm-rp2040-earlephilhower"
    )


def main() -> int:
    pioasm = find_pioasm()
    if not SRC.is_file():
        raise SystemExit(f"missing {SRC}")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    # PIO version 0: compatible with RP2040 and RP2350 (no v1-only ops).
    cmd = [pioasm, "-v", "0", "-o", "c-sdk", str(SRC), str(OUT)]
    print(" ".join(cmd))
    subprocess.check_call(cmd)
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
