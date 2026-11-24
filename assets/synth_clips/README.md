# Recorded watch clips for the synthetic source

Trimmed loops of real watches, for testing the timing chain without a pickup. A synthetic build plays one in place of the default tick-tock loop:

```bash
PLATFORMIO_BUILD_FLAGS=-DTG_SYNTH_CLIP=nh35_21600 pio run -e pico2w_synthetic_core1 -t upload
```

`hello.source` then reads `SYNTHETIC:nh35_21600`. See [`docs/build_and_flash.md`](../../docs/build_and_flash.md) ("Recorded watch clips").

Hardware validation with these clips: [`docs/handoff/TIMING_VALIDATION_HANDOFF.md`](../../docs/handoff/TIMING_VALIDATION_HANDOFF.md) (`scripts/bench/validate_clips_hw.py`).

## How the loops are cut

- **Source:** the ChronoPulse test recordings (`ChronoPulse-Test-Audio`, 32-bit float; the 96 kHz and 192 kHz files resampled to 48 kHz). The recordings themselves are not in the repo.
- **Rate is set by the loop length.** Each loop holds an even number of beats N (tick + tock pairs, so the beat error survives the wrap) and is exactly `round(N × fitted half-period × 48000)` samples long. A timegrapher therefore reads the recording's rate from the loop. N is chosen near 4 s so that the rounding to whole samples is as small as possible (≤ 0.03 s/d).
- **Window:** a run of N + 1 consecutive, clean ticks whose own rate is closest to the whole recording's.
- **Cuts** fall halfway between ticks; the last 2 ms crossfade into the audio just before the start, so the wrap is continuous.
- **Level** is the recording's own (no normalisation), stored as mono 16-bit at 48 kHz.
- **Beat error and amplitude** are whatever the real ticks in the window carry.

`clips.json` holds every number: loop beats and samples, the expected rate, the fit over the whole recording and 20 s windows, and the ChronoPulse reference. To regenerate (needs Python 3 with numpy, scipy, soundfile):

```bash
python3 scripts/synth_clips/make_clips.py <ChronoPulse-Test-Audio dir> assets/synth_clips
```

`scripts/synth_clips/analyze.py <dir>` prints the tick analysis alone.

## Clips and expected values

Reference: ChronoPulse at lift angle 52°, integration 20 s, on the full recordings. Board: `pico2w_synthetic_core1` with the clip, standalone, lift angle 52°, 20 s average (`getBeat.avgLong`), measured 2026-11-23 (amplitude after the pulse-strength fix below).

| Id | bph | Loop | Loop rate (s/d) | Board rate | ChronoPulse rate | Board BE (ms) | ChronoPulse BE | Board amplitude (°) | ChronoPulse amplitude onset / peak |
|---|---|---|---|---|---|---|---|---|---|
| `waltham_18000` | 18,000 | 22 beats, 4.40 s | +20.05 | +20.06 | +20.9 | 0.67 | 0.6 | 307 | 309 / 287 |
| `m8215_incase_21600` | 21,600 | 22 beats, 3.67 s | +3.93 | +3.99 | +4.1 | 0.02 | 0.3 | 293 | 287 / 278 |
| `m8215_jubilee_21600` | 21,600 | 26 beats, 4.33 s | −15.78 | −15.79 | −15.5 | 0.04 | 0.0 | 311 | 300 / 279 |
| `nh35_21600` | 21,600 | 24 beats, 4.00 s | +4.05 | +4.29 | +5.5 | 0.03 | 0.1 | 267 | 261 / 254 |
| `nh39a_21600` | 21,600 | 22 beats, 3.67 s | +17.19 | +17.19 | +17.3 | 0.22 | 0.4 | 335 | 328 / 318 |
| `st3600_21600` | 21,600 | 26 beats, 4.33 s | +12.05 | +12.04 | +11.9 | 0.21 | 0.3 | 328 | 327 / 292 |
| `r3135_hulk_28800` | 28,800 | 32 beats, 4.00 s | +15.30 | +15.32 | +16.1 | 0.21 | 0.0 | 297 | 301 / 276 |
| `r3235_freesprung_28800` | 28,800 | 30 beats, 3.75 s | +5.28 | +5.39 | +5.2 | 0.10 | 0.2 | 256 | 240 / 232 |
| `r3235_starbucks_28800` | 28,800 | 32 beats, 4.00 s | +4.50 | +4.57 | +2.3 | 0.02 | 0.2 | 228 | 214 / 206 |
| `switch_21600_28800` | 21,600 → 28,800 | 29.4 s, not a whole-beat loop | no designed value (12 s segments; firmware and the OTG reference read ≈ +3 on the 28,800 part; the whole 28,800 stretch of the recording fits +6.8) | see below | +7.1 (28,800 part of the whole recording) | | 0.0 | | 302 / 264 |

Notes:

- **Rate differences to ChronoPulse** (NH35, Starbucks, hulk) come from the recordings, not the loops: those watches drift by 1–3 s/d within their 45 s, so the value depends on the stretch measured. A loop reproduces the rate fitted over the whole recording.
- **Amplitude** on the board reads −4 to +16° from ChronoPulse's onset figure and above its peak figure, so its lift-time landmarks sit closest to ChronoPulse's onset method.
- **Clipped recordings:** `waltham_18000`, `st3600_21600`, `r3135_hulk_28800`, `r3235_freesprung_28800` and `switch_21600_28800` peak at exactly 0.100: the recordings were limited, and the unlock and drop pulses hit the same ceiling. The firmware used to pick the drop as the loudest single sample, so it took the unlock (lift 0, amplitude invalid on every ST3600 tick, and on 60 % of Waltham's). It now ranks pulses by their 0.5 ms mean level, and keeps the strongest when a busy tick has more than 8 pulses (`src/mechanical/tick_detect.cpp`). Amplitude-valid ticks: Waltham 81 → 199 of 202, ST3600 0 → 256 of 256, hulk 237 → 316 of 316.
- **`switch_21600_28800`:** 12 s at 21,600 bph, a 5 s changeover (noise while the watch was swapped), 12 s at 28,800 bph; the loop then wraps straight back to 21,600. The board keeps lock through the changeover and re-locks to the new rate within a few seconds. The rolling means and the amplitude history restart on a new nominal rate, and rate and beat error enter the means only once the fit spans ¾ of its window (`src/mechanical/beat_analysis.cpp`): after each switch the average is empty for about 5 s, then reads the new watch directly (28,800 part: +3.2 to +3.8 s/d; 21,600 part: +7.0 to +7.6). Before that fix it overshot to about +34 / +26 s/d for a whole averaging window. Remaining: during the 5 s of changeover noise the lock holds and the average drifts by about 3 s/d. Best used with a 10 s average (the segments are shorter than 20 s).
- **Flash:** a 4 s clip adds about 345 KB; `switch_21600_28800` about 2.8 MB.
