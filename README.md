# Timegrapher Stand — USB Microphone Firmware

PlatformIO firmware for an RP2350/RP2354 board (RP2040 / original Pico envs removed) that enumerates as a **USB Audio Class 2 mono microphone** and streams 48 kHz signed 16-bit PCM from either:

- **ADC** — onboard SAR ADC via FIFO + DMA (prototype path for watch-tick capture)
- **SYNTHETIC** — deterministic 997 Hz sine at −12 dBFS (USB stack validation)

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

```bash
pio run -e pico2_synthetic          # validate USB first
pio run -e pico2                    # ADC capture (Pico 2)
pio run -e kiwi                     # ADC capture (KiwiBoard)
```

UF2 output: `.pio/build/<env>/firmware.uf2`

## Flash

- **Pico 2:** hold BOOTSEL, plug USB, copy the UF2 to the mass-storage drive.
- **Kiwi:** `pio run -e kiwi -t upload` (picoprobe), or copy UF2 the same way if bootsel is available.

## Host capture

```bash
arecord -l
arecord -D hw:<CARD>,<DEVICE> -c 1 -f S16_LE -r 48000 -d 60 capture.wav
python3 tests/host_analyze.py capture.wav
```

Disable OS mic enhancements / AGC while evaluating tick SNR.

## Spec divergences (intentional)

| Spec | This project |
|---|---|
| CMake + Pico SDK, no Arduino | PlatformIO + Arduino-Pico (earlephilhower) |
| No USB CDC | Adafruit TinyUSB Arduino requires CDC to link; device also exposes Serial CDC alongside the mic |
| Project name `watchtiming-rp2350-adc/` | `timegrapher-stand/` |

Functional goals (UAC2 mono 48 kHz S16, DMA ADC, ring + 47/48/49 packet sizing, synthetic mode) remain the acceptance bar. See `docs/acceptance_results.md`.

## Layout

```
include/   app_config, board_config, module headers, reference tusb_config.h
src/       main, usb_audio, usb_descriptors, audio_ring, audio_source, adc_capture, diagnostics
docs/      build_and_flash, PORTING_TO_PCM1809, acceptance_results
tests/     host_analyze.py, host unit tests, host_capture.md
```

## Diagnostics

Build with `-DDIAG_UART=1` to print counters on Serial1 (GPIO0/1 by default). Do not use USB CDC logging in performance-critical paths.

## License

Application code: project license. TinyUSB / Pico SDK / Arduino-Pico: see `LICENSES.md`.
