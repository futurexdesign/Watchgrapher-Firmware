#!/usr/bin/env python3
"""Regenerate include/tick_tock_pcm.h from tick-tock.wav (exact copy, no crossfade)."""
from __future__ import annotations

import pathlib
import struct
import wave

root = pathlib.Path(__file__).resolve().parents[1]
wav_path = root / "tick-tock.wav"
hdr_path = root / "include" / "tick_tock_pcm.h"


def main() -> None:
    with wave.open(str(wav_path), "rb") as w:
        assert w.getnchannels() == 1 and w.getsampwidth() == 2
        assert w.getframerate() == 48000
        n = w.getnframes()
        samples = list(struct.unpack("<" + "h" * n, w.readframes(n)))

    with hdr_path.open("w") as f:
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n\n")
        f.write("/* Auto-generated from tick-tock.wav (mono S16LE @ 48 kHz).\n")
        f.write(" * Exact sample copy — no crossfade, no rotation. Looped from index 0.\n")
        f.write(" * Regenerate: python3 scripts/gen_tick_tock_pcm.py\n")
        f.write(" */\n")
        f.write(f"#define TICK_TOCK_PCM_LEN {len(samples)}u\n")
        f.write("#define TICK_TOCK_PCM_RATE_HZ 48000u\n\n")
        f.write("static const int16_t kTickTockPcm[TICK_TOCK_PCM_LEN] = {\n")
        for i in range(0, len(samples), 12):
            chunk = ", ".join(str(x) for x in samples[i : i + 12])
            f.write(f"  {chunk},\n")
        f.write("};\n")

    print(f"wrote {hdr_path} len={len(samples)}")


if __name__ == "__main__":
    main()
