#!/usr/bin/env python3
"""Regenerate board-specific pcm1808_i2s_*.pio.h from src/pcm1808_i2s.pio."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# Keep the template outside src/ so PlatformIO does not auto-pioasm it.
SRC = ROOT / "scripts" / "pcm1808_i2s.pio.in"
INCLUDE = ROOT / "include"

# (filename stem, BCK gpio, LRCK gpio)
BOARDS = (
    ("pcm1808_i2s_legacy", 2, 3),   # Kiwi / previous Pico 2 wiring
    ("pcm1808_i2s_pico2", 18, 20),  # Pico 2: BCK GP18, LRCK GP20 (DOUT GP19)
)

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
        from shutil import which

        w = which(c)
        if w:
            return w
    raise SystemExit(
        "pioasm not found. Set PIOASM=... or install PlatformIO package "
        "tool-pioasm-rp2040-earlephilhower"
    )


def generate_one(pioasm: str, bck: int, lrck: int, out: Path) -> None:
    text = SRC.read_text()
    if "@BCK_GPIO@" not in text or "@LRCK_GPIO@" not in text:
        raise SystemExit(f"{SRC} missing @BCK_GPIO@ / @LRCK_GPIO@ placeholders")
    text = text.replace("@BCK_GPIO@", str(bck)).replace("@LRCK_GPIO@", str(lrck))
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td) / "pcm1808_i2s.pio"
        tmp.write_text(text)
        cmd = [pioasm, "-v", "0", "-o", "c-sdk", str(tmp), str(out)]
        print(" ".join(cmd))
        subprocess.check_call(cmd)
    print(f"wrote {out} (BCK={bck} LRCK={lrck})")


def write_selector() -> None:
    selector = INCLUDE / "pcm1808_i2s.pio.h"
    selector.write_text(
        """\
/**
 * @file pcm1808_i2s.pio.h
 * @brief Board-selecting wrapper for PCM1808 I²S PIO (autogen companions).
 *
 * wait gpio immediates are baked per board — do not share one program blob.
 * Regenerate with: {@code python3 scripts/gen_pcm1808_pio.py}
 */
#pragma once

#if defined(BOARD_PICO2)
#include "pcm1808_i2s_pico2.pio.h"
#else
#include "pcm1808_i2s_legacy.pio.h"
#endif
"""
    )
    print(f"wrote {selector}")


def main() -> int:
    pioasm = find_pioasm()
    if not SRC.is_file():
        raise SystemExit(f"missing {SRC}")
    INCLUDE.mkdir(parents=True, exist_ok=True)
    for stem, bck, lrck in BOARDS:
        generate_one(pioasm, bck, lrck, INCLUDE / f"{stem}.pio.h")
    write_selector()
    # Do not write src/pcm1808_i2s.pio.h — PlatformIO puts src/ on the include
    # path and a full blob there would shadow include/pcm1808_i2s.pio.h.
    stale = ROOT / "src" / "pcm1808_i2s.pio.h"
    if stale.exists():
        stale.unlink()
        print(f"removed stale {stale}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
