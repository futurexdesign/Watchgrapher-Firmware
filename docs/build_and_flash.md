# Build and flash

## Prerequisites

- PlatformIO Core (`pio`)
- For Kiwi upload: CMSIS-DAP / picoprobe connected as configured in `platformio.ini`

## Build

```bash
cd timegrapher-stand
pio run -e pico2_synthetic   # Stage A — USB only
pio run -e pico2             # Stage B — ADC on Pico 2
pio run -e kiwi              # ADC on KiwiBoard (GPIO40)
pio run -e pico2_pcm1808     # PCM1808 + DSP + quartz timing (Pico 2)
pio run -e kiwi_pcm1808      # PCM1808 + DSP + quartz timing (KiwiBoard)
pio run                      # all six envs (pico2, pico2_synthetic, pico2_pcm1808, kiwi, kiwi_synthetic, kiwi_pcm1808)
```

CI builds `pico2_pcm1808`, `kiwi_pcm1808`, `pico2` and `pico2_synthetic`:

```bash
pio run -e pico2_pcm1808 -e kiwi_pcm1808 -e pico2 -e pico2_synthetic
```

Size reported by that command (RP2350, `-Os`):

| Env | RAM used / 512 KiB (524,288 B) | Flash used / program maximum |
|---|---|---|
| `pico2` | 92,352 B (17.6 %) | 205,936 B of 4,190,208 (4.9 %) |
| `pico2_synthetic` | 79,856 B (15.2 %) | 242,792 B of 4,190,208 (5.8 %) |
| `pico2_pcm1808` | 164,600 B (31.4 %) | 255,208 B of 4,190,208 (6.1 %) |
| `kiwi_pcm1808` | 164,308 B (31.3 %) | 254,344 B of 2,093,056 (12.2 %) |

Build flags worth knowing (all optional, add to an env's `build_flags`; see `platformio.ini`):

| Flag | Effect |
|---|---|
| `-DDISPLAY_SH1107=0` | No OLED; core 1 stays in reset (the ini sets it to 1 in every env) |
| `-DPPS_GPIO=N` | GPS 1 PPS input on GPIO N (already `3` in `pico2_pcm1808`) |
| `-DGPS_UART_RX=N` | GPS NMEA input on UART RX pin N (off in every shipped env); also `GPS_UART_BAUD`, `GPS_MIN_SATS` |
| `-DDIAG_UART=1` | Counters on Serial1 (GP0/GP1) |
| `-DBOOST_SWITCH_PRESENT=1 -DBOOST_EN_GPIO=N` | Preamp rail switch with warm-up |
| `-DPCM1808_OUTPUT_UPSAMPLE=1` | Skip the 48 kHz-vs-24 kHz guess before the first rate confirmation |

## Host tests

```bash
tests/run_all.sh                  # 15 rows: PASS / FAIL / SKIP table, exit 1 on any FAIL
SANITIZE=1 tests/run_all.sh       # ASan + UBSan on the C/C++ tests
tests/build_quartz_tests.sh       # only the quartz group (run_all.sh calls it)
```

Output goes to `build/host-tests` (git-ignored). The quartz vector and engine tests need `docs/handoff/vectors/*.wav` (git-ignored, 97 MB); without them those two rows are SKIP. Compilers come from `CC` / `CXX` (default `cc` / `c++`).

Artifacts per env:

- `.pio/build/<env>/firmware.uf2`
- `.pio/build/<env>/firmware.elf`
- `.pio/build/<env>/firmware.bin`

## Flash UF2 (BOOTSEL)

1. Hold BOOTSEL, connect USB.
2. Copy `firmware.uf2` to the presented drive.
3. Board reboots into the application.

## Flash Kiwi via picoprobe

```bash
pio run -e kiwi -t upload              # or -e kiwi_pcm1808 / kiwi_synthetic
```

## Saved settings

The calibration, device mode, `logDeltaPpm` and the calibration log live in the last 4 KiB flash sector (Arduino-Pico EEPROM emulation, one 1 KB image owned by `flash_store`). That sector is outside the program image, so an ordinary firmware upload should leave it in place (not verified on hardware here; a full flash erase clears it, and the device then reads defaults). A firmware that grows the image keeps the old 256-byte calibration record. A write is refused while the USB audio stream is open; on a standalone device it pauses capture for a few tens of ms. See `docs/AS_BUILT_ARCHITECTURE.md` §4.3.

## Recommended bring-up order

1. Flash `pico2_synthetic` or `kiwi_synthetic`.
2. Confirm host sees **Watch Timing Microphone** (and a CDC serial port — Arduino TinyUSB side effect).
3. Record 60 s WAV; run `tests/host_analyze.py` — expect the looped `tick-tock.wav` clip (~400 ms period).
4. Verify analog bias on ADC pin **before** connecting TL071 SIG.
5. Flash ADC env; capture silence + watch ticks.
6. For the PCM1808 envs: check CLI `status` for `pcm1808 rate=ok` and `timebase=ok`, then record; see `docs/PCM1808_WIRING_AND_CLOCKS.md`. Try `device mode standalone` (results on the OLED, header tag `SA`) and `quartz mode motor|lcd|fork`.

## Changing the ADC pin

Edit `ADC_GPIO` / `ADC_CHANNEL` in `platformio.ini` for the relevant env, or add a new env. Defaults:

| Board | Package | ADC0 GPIO |
|---|---|---|
| Pico 2 | RP2350A | 26 |
| Kiwi / RP2354 | RP2350B | 40 |
