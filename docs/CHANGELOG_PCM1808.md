# CHANGELOG — PCM1808 backend

This file started as the PCM1808 backend log and now records later firmware rounds as well. Entries are newest first; the current behavior is in [AS_BUILT_ARCHITECTURE.md](AS_BUILT_ARCHITECTURE.md).

## Device mode, flash store, beats, GPS detail, calibration log (2026-11-18)

- **Device modes.** `usbMic` (default) and `standalone`, persisted (DEVICE region, magic `DEV1`). A run latch (`run start|stop|status`, HID `start`/`stop`) lets capture and analysis run without a USB stream; standalone starts it at boot, skips the audio ring and the sample repeat, and sends silence on the USB IN endpoint. CLI `device`, `run`; OLED tag `SA` / `USB`; header and quartz status texts `no host stream`, `stopped`, `clock check...`.
- **`flash_store`.** One owner for the EEPROM-emulation sector, now a 1 KB image: TIMEBASE [0,256), DEVICE [256,512), TBLOG [512,1024). `flash_store_commit_paused()` refuses while USB audio streams, pauses and restarts capture, verifies by read-back, and skips when nothing changed. Init order in `setup()`: `control_api_init`, `flash_store_init`, `timebase_init`, `device_mode_init`, `audio_source_init`, `device_boot_run`.
- **Calibration log.** 24-entry ring (`timebase_log.h`, record `TBL1`), GPS-PPS policy (64-pulse window, 10 min continuous lock, UTC known, `logDeltaPpm` 0.05..10 default 0.2, 1 day / 30 days), deferred write (never while streaming or while a quartz measurement is acquiring / measuring; retry 10 s / 60 s). CLI `timebase log [clear]`, `timebase logdelta [ppm]`.
- **GPS detail.** `nmea.h` parses RMC, GGA (+ position), GSA (union per epoch) and GSV (up to 32 satellites, 5 s ageing, PRN normalisation per system); `timebase gps` and HID `getGps`. Needs `-DGPS_UART_RX=N` (no shipped env sets it).
- **Per-beat ring.** 64 `dsp_beat_t` records in `beat_analysis` (grid id, k, parity, residual `u_us` against the nominal grid, corrected by the timebase ppm); CLI `dsp beats [n]`, HID `getBeats` / `beats` notify.
- **Pipeline table.** `dsp_pipeline_nodes()` (block `group` / `order`, data-flow edges, quartz node) in `hello.caps.pipeline`; CLI `dsp graph`.
- **Quartz engine.** Motor / LCD / fork timing on the raw stereo codes (PCM1808 builds only), `quartz` CLI, HID `getQuartz`, OLED quartz screens; library in `src/quartz/`, mechanical analysis moved to `src/mechanical/`.
- **Tooling.** `tests/run_all.sh` (15 rows), `.github/workflows/ci.yml`, `codeql.yml`, `dependabot.yml`.
- Docs refreshed to match: README, AS_BUILT_ARCHITECTURE (new §4.3, §8.7 to §8.10, §12.1), TIMEBASE_CALIBRATION, DISPLAY, QUARTZ_TIMING, build_and_flash, acceptance_results.

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
