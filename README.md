# Timegrapher Stand — USB Microphone Firmware

PlatformIO firmware for an RP2350/RP2354 board (RP2040 / original Pico envs removed) that enumerates as a **USB Audio Class 2 mono microphone** and streams 48 kHz signed **24-bit** PCM (3-byte LE) from one of:

- **ADC** — onboard SAR ADC via FIFO + DMA (prototype path for watch-tick capture)
- **PCM1808** — external 24-bit I²S ADC via PIO + DMA, through the float DSP graph (event detect, beat rate / beat error / amplitude)
- **SYNTHETIC** — looped `tick-tock.wav` clip (USB stack validation)

Alongside the microphone the device exposes a USB CDC serial CLI and a vendor HID (WebHID) JSON control plane — see `docs/HID_CONTROL_PLANE.md`.

Timing accuracy depends on the board's 12 MHz crystal (1 ppm = 0.0864 s/d). Calibrate it once against NTP with `scripts/calibrate_timebase.py`, or use a GPS PPS input. See `docs/TIMEBASE_CALIBRATION.md`.

An optional 128×128 SH1107 I²C OLED shows the averaged rate, amplitude and beat error (integration time configurable, default 10 s), the nominal bph, the lift angle, a GPS timebase badge and a 2-minute rate trend. It runs on core 1, so the display can't disturb the audio stream. See `docs/DISPLAY.md`.

Product string: `Watch Timing Microphone (Prototype)`.

## Analog safety

**Do not connect the raw TL071 stand SIG output directly to an ADC GPIO.** The preamp may have an unknown DC offset and must be AC-coupled and mid-rail biased into 0…ADC_AVDD. See `IMPLEMENTATION_SPEC.md` §3 for the provisional frontend.

## Build targets

| Env | Board | ADC pin | Source |
|---|---|---|---|
| `pico2` | Raspberry Pi Pico 2 (RP2350A) | GPIO26 | ADC |
| `pico2_synthetic` | Pico 2 | GPIO26 | Synthetic |
| `kiwi` | KiwiBoard / RP2354 (RP2350B, 2MB) | GPIO40 | ADC |
| `kiwi_synthetic` | KiwiBoard | GPIO40 | Synthetic |
| `pico2_pcm1808` | Pico 2 | — | PCM1808 (BCK GP18, LRCK GP20, DOUT GP19, MCLK GP21) |
| `kiwi_pcm1808` | KiwiBoard | — | PCM1808 (BCK GP2, LRCK GP3, DOUT GP4, MCLK GP21) |

```bash
pio run -e pico2_synthetic          # validate USB first
pio run -e pico2                    # ADC capture (Pico 2)
pio run -e kiwi                     # ADC capture (KiwiBoard)
pio run -e pico2_pcm1808            # PCM1808 + DSP (Pico 2)
```

UF2 output: `.pio/build/<env>/firmware.uf2`

## Flash

- **Pico 2:** hold BOOTSEL, plug USB, copy the UF2 to the mass-storage drive.
- **Kiwi:** `pio run -e kiwi -t upload` (picoprobe), or copy UF2 the same way if bootsel is available.

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
| Project name `watchtiming-rp2350-adc/` | `timegrapher-stand/` |

Functional goals (UAC2 mono 48 kHz, DMA capture, ring + 47/48/49 packet sizing, synthetic mode) remain the acceptance bar. The as-built reference is `docs/AS_BUILT_ARCHITECTURE.md`. See `docs/acceptance_results.md`.

## Layout

```
include/     app_config, board_config, fw_identity, module headers
src/         main, usb_audio/usb_descriptors, audio_source/audio_ring,
             adc_capture, pcm1808_capture/pcm1808_clock, dsp_graph/dsp_blocks,
             control_api, hid_control, serial_cli + cli/, diagnostics, perf_timing
scripts/     header generators (tick-tock table, PCM1808 PIO programs)
third_party/ vendored CMSIS-DSP
docs/        AS_BUILT_ARCHITECTURE, HID_CONTROL_PLANE, TIMEBASE_CALIBRATION, DISPLAY, build_and_flash, PCM1808 notes
tests/       host_analyze.py, host unit tests, host_capture.md

TinyUSB is configured only by the -DCFG_TUD_* flags in platformio.ini.
```

## Diagnostics

Build with `-DDIAG_UART=1` to print counters on Serial1 (GPIO0/1 by default). Do not use USB CDC logging in performance-critical paths.

## License

Application code: project license. TinyUSB / Pico SDK / Arduino-Pico: see `LICENSES.md`.
