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
pio run                      # all envs
```

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
pio run -e kiwi -t upload
```

## Recommended bring-up order

1. Flash `pico2_synthetic` or `kiwi_synthetic`.
2. Confirm host sees **Watch Timing Microphone (Prototype)** (and a CDC serial port — Arduino TinyUSB side effect).
3. Record 60 s WAV; run `tests/host_analyze.py` — expect ~997 Hz tone.
4. Verify analog bias on ADC pin **before** connecting TL071 SIG.
5. Flash ADC env; capture silence + watch ticks.

## Changing the ADC pin

Edit `ADC_GPIO` / `ADC_CHANNEL` in `platformio.ini` for the relevant env, or add a new env. Defaults:

| Board | Package | ADC0 GPIO |
|---|---|---|
| Pico 2 | RP2350A | 26 |
| Kiwi / RP2354 | RP2350B | 40 |
