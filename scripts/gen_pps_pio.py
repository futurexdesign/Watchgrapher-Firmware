#!/usr/bin/env python3
"""Regenerate include/pps_counter.pio.h from scripts/pps_counter.pio.in.

The template lives outside src/ (with a .in suffix) so PlatformIO does not
assemble it on its own; the generated header is committed like the PCM1808
I2S headers.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gen_pcm1808_pio import ROOT, find_pioasm  # noqa: E402

SRC = ROOT / "scripts" / "pps_counter.pio.in"
OUT = ROOT / "include" / "pps_counter.pio.h"


def main() -> int:
    pioasm = find_pioasm()
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td) / "pps_counter.pio"
        shutil.copyfile(SRC, tmp)
        cmd = [pioasm, "-v", "0", "-o", "c-sdk", str(tmp), str(OUT)]
        print(" ".join(cmd))
        subprocess.check_call(cmd)
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
