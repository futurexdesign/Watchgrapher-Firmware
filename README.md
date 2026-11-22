# Timegrapher Stand — USB Microphone Firmware

PlatformIO firmware for an RP2350/RP2354 board (RP2040 / original Pico envs removed) that enumerates as a **USB Audio Class 2 mono microphone** and streams 48 kHz signed **24-bit** PCM (3-byte LE) from one of:

- **ADC** — onboard SAR ADC via FIFO + DMA (prototype path for watch-tick capture)
- **PCM1808** — external 24-bit I²S ADC via PIO + DMA, through the float DSP graph (event detect, beat rate / beat error / amplitude)
- **SYNTHETIC** — looped `tick-tock.wav` clip (USB stack validation)

Only the PCM1808 path runs the DSP graph, the per-beat ring and the quartz engine. On the ADC and SYNTHETIC builds the device is a plain USB microphone (the CLI, HID and OLED still work, but there is nothing to measure).

**Quartz timing** (PCM1808 builds): the stand also times quartz watches from the raw stereo codes. It times stepper-motor pulses on LEFT, the inductive coil, giving the rate with and without inhibition. It times LCD drive on RIGHT, the capacitive electrode, and tuning forks on LEFT. The USB microphone stays mono LEFT. Select the mode with the CLI `quartz` command or over HID; the OLED switches to a quartz screen. See `docs/QUARTZ_TIMING.md`.

**Two modalities.** The same hardware works in two persisted modes, chosen with the CLI `device mode usbmic|standalone` or HID `setDevice`:

- **USB microphone** (default): a PC timegrapher program records the audio and does the analysis, and capture runs while the host streams. The CLI `run start|stop` (HID `start` / `stop`) can also run capture and analysis without a stream.
- **Standalone timegrapher**: the device is the timegrapher. Capture and analysis start at boot, results appear on the OLED (header tag `SA`), the CLI and WebHID, and the USB audio feed is silent if a host opens it.

The mode is saved to flash (a write that is refused while the audio stream is open). `device` shows the mode, whether capture is running, whether a run is requested and whether a USB stream is open; `run start|stop|status` sets or clears the run latch. After `run stop`, a standalone device stays stopped until `run start` or the next boot. The quartz mode is not saved: a standalone device always boots in `mechanical` and needs the CLI or HID to select a quartz mode. See `docs/AS_BUILT_ARCHITECTURE.md` §4.3 and `docs/HID_CONTROL_PLANE.md` (Device modes).

**Web app.** The end-user web front end is the sibling repository `timegrapher-web` (`../timegrapher-web`): a Vue app that talks to this firmware over **WebHID** and takes its protocol types from `docs/hid/timegrapher_hid.d.ts` (`npm run sync-types` there). It includes a **virtual device** (simulator), so the UI can be used and developed without hardware. `docs/hid/example.html` remains the minimal dev smoke page.

Alongside the microphone the device exposes a USB CDC serial CLI and a vendor HID (WebHID) JSON control plane — see `docs/HID_CONTROL_PLANE.md`.

Timing accuracy depends on the board's 12 MHz crystal (1 ppm = 0.0864 s/d). Calibrate it once against NTP with `scripts/calibrate_timebase.py`, or use a GPS PPS input. See `docs/TIMEBASE_CALIBRATION.md`.

The device also keeps a 24-entry calibration history in flash (CLI `timebase log`, HID `getTimebaseLog`). A GPS PPS measurement is logged only when it is worth a flash write (full 64 s average, 10 min of continuous lock, UTC known, and a change of at least `logDeltaPpm` after a day or 30 days since the last entry). GPS detail (satellite table, DOPs, position) needs `-DGPS_UART_RX=N`, which no shipped env sets; `pico2_pcm1808` sets `-DPPS_GPIO=3` only. See `docs/TIMEBASE_CALIBRATION.md`.

An optional 128×128 SH1107 I²C OLED shows the averaged rate, amplitude and beat error (integration time configurable, default 10 s), the nominal bph, the lift angle, the mode tag (`SA` / `USB`), a GPS timebase badge and a 2-minute rate trend. It runs on core 1, so the display can't disturb the audio stream. See `docs/DISPLAY.md`.

Product string: `Watch Timing Microphone` (`USB_PRODUCT_STRING` in `include/fw_identity.h`, with the VID/PID and `FW_VERSION`, default `0.1.0-dev`).

## Analog safety

**Do not connect the raw TL071 stand SIG output directly to an ADC GPIO.** The preamp may have an unknown DC offset and must be AC-coupled and mid-rail biased into 0…ADC_AVDD. See `IMPLEMENTATION_SPEC.md` (repository root, the original P1 spec) §3 for the provisional frontend.

## Build targets

| Env | Board | ADC pin | Source |
|---|---|---|---|
| `pico2` | Raspberry Pi Pico 2 (RP2350A) | GPIO26 | ADC |
| `pico2_synthetic` | Pico 2 | GPIO26 | Synthetic |
| `kiwi` | KiwiBoard / RP2354 (RP2350B, 2MB) | GPIO40 | ADC |
| `kiwi_synthetic` | KiwiBoard | GPIO40 | Synthetic |
| `pico2_pcm1808` | Pico 2 | — | PCM1808 (BCK GP18, LRCK GP20, DOUT GP19, MCLK GP21); GPS 1 PPS input on GP3 (`-DPPS_GPIO=3`) |
| `kiwi_pcm1808` | KiwiBoard | — | PCM1808 (BCK GP2, LRCK GP3, DOUT GP4, MCLK GP21); no PPS input |

All six envs have the OLED on core 1 (`-DDISPLAY_SH1107=1`): SDA/SCL on GP4/GP5 (Pico 2) or GP6/GP7 (Kiwi). CI builds `pico2_pcm1808`, `kiwi_pcm1808`, `pico2` and `pico2_synthetic`.

```bash
pio run -e pico2_synthetic          # validate USB first
pio run -e pico2                    # ADC capture (Pico 2)
pio run -e kiwi                     # ADC capture (KiwiBoard)
pio run -e pico2_pcm1808            # PCM1808 + DSP (Pico 2)
```

UF2 output: `.pio/build/<env>/firmware.uf2`

Firmware size (`pio run -e pico2_pcm1808 -e kiwi_pcm1808 -e pico2 -e pico2_synthetic`, RP2350, `-Os`, 2026-11-18):

| Env | RAM used / 512 KiB (524,288 B) | Flash used / program maximum |
|---|---|---|
| `pico2` | 92,352 B (17.6 %) | 205,936 B of 4,190,208 (4.9 %) |
| `pico2_synthetic` | 79,856 B (15.2 %) | 242,792 B of 4,190,208 (5.8 %) |
| `pico2_pcm1808` | 164,600 B (31.4 %) | 255,208 B of 4,190,208 (6.1 %) |
| `kiwi_pcm1808` | 164,308 B (31.3 %) | 254,344 B of 2,093,056 (12.2 %) |

Persistent settings live in the last 4 KiB flash sector, which Arduino-Pico emulates as EEPROM (the program image is smaller than the reported maximum by exactly that sector). `flash_store` owns it as a 1 KB image: calibration (0..255), device mode and log threshold (256..511), calibration log (512..1023). A write pauses capture and is refused while the USB audio stream is open; see `docs/AS_BUILT_ARCHITECTURE.md` §4.3.

## Flash

- **Pico 2:** hold BOOTSEL, plug USB, copy the UF2 to the mass-storage drive.
- **Kiwi:** `pio run -e kiwi -t upload` (picoprobe), or copy UF2 the same way if bootsel is available.

## CLI quick reference

The CDC serial console (`help`, `help <command>`) has: `version`, `status`, `time`, `timebase` (`set`, `save`, `clear`, `gps`, `log [clear]`, `logdelta [ppm]`), `device` (`status`, `mode [usbmic|standalone]`), `run [start|stop|status]`, `display`, `diag`, `dsp` (`list`, `on`/`off`, `get`/`set`, `events`, `beat`, `beats [n]`, `graph`, `metrics`, `timing`, `spectrum`, `match`, `errors`, `help`) and `quartz` / `qz`. The same operations are available over WebHID (`docs/HID_CONTROL_PLANE.md`).

## Host tests and CI

```bash
tests/run_all.sh [BUILD_DIR]      # builds and runs every host test, prints a PASS/FAIL/SKIP table
SANITIZE=1 tests/run_all.sh       # same with ASan + UBSan on the C/C++ tests
```

The table has 15 rows: the C and C++ unit tests (PCM conversion, ring buffer, PCM1808 rate, beat fit, PPS filter, NMEA, UI format, DSP block table, timebase log, device CLI formatters), the quartz group built by `tests/build_quartz_tests.sh` (units, HID JSON, quartz CLI formatters, and the engine and acceptance-vector runs), and the Python NTP-fit test. The acceptance vectors `docs/handoff/vectors/*.wav` (97 MB) are git-ignored; without them the vector and engine rows are reported as SKIP, not FAIL.

`.github/workflows/ci.yml` runs the host tests (gcc, clang, and gcc with sanitizers), builds the four firmware envs listed above, and checks clang-format on changed lines and cppcheck. `codeql.yml` analyses the host code weekly and on push; `dependabot.yml` keeps the Actions versions current.

## Host capture

```bash
arecord -l
arecord -D hw:<CARD>,<DEVICE> -c 1 -f S24_3LE -r 48000 -d 60 capture.wav
python3 tests/host_analyze.py capture.wav
```

Disable OS mic enhancements / AGC while evaluating tick SNR.

## Spec divergences (intentional)

| Spec | This project |
|---|---|
| CMake + Pico SDK, no Arduino | PlatformIO + Arduino-Pico (earlephilhower) |
| No USB CDC | Adafruit TinyUSB Arduino requires CDC to link; device also exposes Serial CDC alongside the mic |
| Audio-only USB function set | Also a vendor-defined HID interface (Usage Page `0xFFA0`) for the WebHID JSON control plane |
| PCM16 USB audio | PCM24 (S24LE) USB audio; the PCM1808 left slot is passed through at full width |
| Project name `watchtiming-rp2350-adc/` | `timegrapher-stand/` |

Functional goals (UAC2 mono 48 kHz, DMA capture, ring + 47/48/49 packet sizing, synthetic mode) remain the acceptance bar. The as-built reference is `docs/AS_BUILT_ARCHITECTURE.md`. See `docs/acceptance_results.md`.

## Layout

```
include/     app_config, board_config, fw_identity, module headers
src/         main, usb_audio/usb_descriptors, audio_source/audio_ring, device_mode, flash_store,
             adc_capture, pcm1808_capture/pcm1808_clock, dsp_graph/dsp_blocks,
             control_api, hid_control/hid_json, timebase (calibration, PPS, GPS, log),
             display_ui/ui_snapshot (OLED, core 1), power_control,
             serial_cli + cli/, diagnostics, perf_timing
src/mechanical/  mechanical-watch analysis: tick_detect, beat_analysis, beat_fit
             (headers in include/mechanical/)
src/quartz/  quartz-watch timing: quartz_engine, motor / lcd / tone timing, quartz_dsp
             (headers in include/quartz/; C view in include/dsp_quartz_types.h)
scripts/     calibrate_timebase.py (NTP calibration), header generators (tick-tock table,
             PCM1808 and PPS PIO programs)
third_party/ vendored CMSIS-DSP
docs/        AS_BUILT_ARCHITECTURE, HID_CONTROL_PLANE (+ hid/ TypeScript types and example page),
             QUARTZ_TIMING, TIMEBASE_CALIBRATION, DISPLAY, build_and_flash, PCM1808 notes,
             handoff/ (quartz pickup spec, reference and acceptance vectors)
tests/       run_all.sh, host unit tests (test_*.cpp/.c/.py), build_quartz_tests.sh,
             host_analyze.py, host_capture.md
.github/     ci.yml, codeql.yml, dependabot.yml

TinyUSB is configured only by the -DCFG_TUD_* flags in platformio.ini.
```

## Diagnostics

Build with `-DDIAG_UART=1` to print counters on Serial1 (GPIO0/1 by default). Do not use USB CDC logging in performance-critical paths.

## License

Application code: project license. TinyUSB / Pico SDK / Arduino-Pico: see `LICENSES.md`.
