# CHANGELOG — PCM1808 backend

## Pico 2 PCB pin map (2026-11-08)

- `pico2_pcm1808`: MCLK GP21, LRCK GP20, DOUT GP19, BCK GP18 (header 27/26/25/24) — no-cross wiring.
- Board-specific PIO blobs: `pcm1808_i2s_pico2.pio.h` vs `pcm1808_i2s_legacy.pio.h` (Kiwi unchanged GP2/3/4).
- PIO template moved to `scripts/pcm1808_i2s.pio.in` (not under `src/`, so PlatformIO won’t auto-pioasm placeholders).
- Regen: `python3 scripts/gen_pcm1808_pio.py`.

## Added

- `PCM1808` audio source (`AUDIO_SOURCE=3`) with PlatformIO envs `pico_pcm1808` (RP2040 Pico @ 150 MHz), `pico2_pcm1808`, and `kiwi_pcm1808`.
- RP2040 ADC/SYNTHETIC envs: `pico`, `pico_synthetic` (default 133 MHz).
- `pcm1808_clock` — GPOUT0 on GP21 from PLL_SYS, div `12 + 0x3500`, started at `audio_source_init`.
- `pcm1808_i2s.pio` + checked-in `include/pcm1808_i2s.pio.h`; regen via `scripts/gen_pcm1808_pio.py`.
- `pcm1808_capture` — PIO RX + DMA ping-pong `2 × 240` stereo frames; left → mono PCM16 into `audio_ring`.
- `pcm1808_format.h` — host-testable left-slot conversion; `tests/test_pcm1808_pcm_conversion.c`.
- Diagnostics counters for clock/MCLK/DMA/frames/resync/peak/saturation (P3.0 §7).
- `docs/PCM1808_WIRING_AND_CLOCKS.md` — wiring, straps, fractional divider, Gate B checklist.

## Preserved

- Composite CDC + UAC2 descriptors and TinyUSB feed of **48 samples/ms** with flow control.
- Existing `pico2`, `pico2_synthetic`, `kiwi`, `kiwi_synthetic` envs and ADC/SYNTHETIC code paths.
- Ring capacity 8192, prefill 480, stream FSM, status LED GP25, optional `DIAG_UART`.
- No runtime PLL_SYS reprogramming; no stereo USB presentation; no CMake migration.

## Not hardware-verified in this change

- Oscilloscope confirmation of SCKI/BCK/LRCK.
- USB enumeration / recording soak with PCM1808 attached.
