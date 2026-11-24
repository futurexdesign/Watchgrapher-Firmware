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
pio run -e pico2_pcm1808_cli # same, with the legacy serial CLI (see "Serial CLI")
pio run                      # every env in platformio.ini
```

The serial CLI is not built by default. Only the `*_cli` envs include it (see "Serial CLI").

CI builds `pico2_pcm1808`, `kiwi_pcm1808`, `pico2` and `pico2_synthetic`:

```bash
pio run -e pico2_pcm1808 -e kiwi_pcm1808 -e pico2 -e pico2_synthetic
```

Size reported by that command (RP2350, `-Os`):

| Env | RAM used / 512 KiB (524,288 B) | Flash used / program maximum |
|---|---|---|
| `pico2` | 92,352 B (17.6 %) | 205,936 B of 4,190,208 (4.9 %) |
| `pico2_synthetic` | 79,856 B (15.2 %) | 242,792 B of 4,190,208 (5.8 %) |
| `pico2_pcm1808` (no CLI) | 162,728 B (31.0 %) | 202,792 B of 4,190,208 (4.8 %) |
| `pico2_pcm1808_cli` | 164,604 B (31.4 %) | 255,648 B of 4,190,208 (6.1 %) |
| `kiwi_pcm1808` | 164,308 B (31.3 %) | 254,344 B of 2,093,056 (12.2 %) |

Build flags worth knowing (all optional, add to an env's `build_flags`; see `platformio.ini`):

| Flag | Effect |
|---|---|
| `-DDISPLAY_SH1107=0` | No OLED; core 1 stays in reset (the ini sets it to 1 in every env) |
| `-DPPS_GPIO=N` | GPS 1 PPS input on GPIO N (already `3` in `pico2_pcm1808`) |
| `-DGPS_UART_RX=N` | GPS NMEA input on UART RX pin N (off in every shipped env); also `GPS_UART_BAUD`, `GPS_MIN_SATS` |
| `-DDIAG_UART=1` | Counters on Serial1 (GP0/GP1); the periodic print needs a `*_cli` build (`diag on`) |
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

## Pico 2 W (BLE)

The Raspberry Pi Pico 2 W (RP2350A with the CYW43439 radio) adds a Bluetooth LE control plane next to USB HID. The protocol is in [`BLE_CONTROL_PLANE.md`](BLE_CONTROL_PLANE.md). The envs below are in `platformio.ini` (sections `[ble]`, `[board_pico2w]`, `[cli]`).

```bash
pio run -e pico2w                    # ADC, BLE
pio run -e pico2w_synthetic          # synthetic source, BLE (CI)
pio run -e pico2w_pcm1808            # PCM1808, BLE (CI)
pio run -e pico2w_synthetic_noble    # W board, BLE not built: hardware A/B diagnostics only
```

CI builds `pico2w_pcm1808`, `pico2w_synthetic` and `pico2_pcm1808_cli` (the last keeps the CLI compiling). `pico2w_synthetic_noble` also includes `[cli]`, so it is the reference for a BLE A/B comparison with `pico2w_synthetic_cli`.

**GPIO25 warning.** On the Pico 2 W, GPIO25 is the CYW43 chip select (`WL_CS`). Do not drive it, and do not wire anything to it. The status LED is the CYW43 LED, not GPIO25: the W build sets `STATUS_LED_GPIO -1` and drives the LED through the radio. GPIO29 is the radio clock and the VSYS sense input, so do not use it as an ADC input on a W build. The PCM1808 and ADC pins are the same as on the Pico 2 (see `docs/PCM1808_WIRING_AND_CLOCKS.md`).

**Serial CLI (opt-in).** No shipped `pico2w_*` env has the serial CLI. See "Serial CLI" below for the `*_cli` envs. Use `getDiag` and `getLink` over HID or BLE for the counters that the CLI `diag` and `ble` commands used to show.

**Uploads still reset the board.** The CDC serial port is kept in every build, also without the CLI. `pio run -e <env> -t upload` therefore still works: the 1200-baud reset puts a running board into BOOTSEL. A terminal on the port shows nothing, because nothing is written to it except one line at boot, the BLE service UUID that the library prints once (`c137e7c0-92b4-4d9e-87e0-9ba757169016`). Input on the port is ignored.

**`ble` CLI command (CLI builds only).** The `ble` command prints the BLE status: enabled, name (`TG-XXXX`), address, advertising, connected, MTU, connection interval, frame and drop counters, disconnects and the core that runs the radio. `ble off` and `ble on` turn BLE off and on. The same fields are in `getLink.ble` for every build with BLE. Use `getLink` for hardware tests that run without the CLI.

**Echo build (override, G1 bring-up only).** The echo build replaces the control plane with a loop that returns every received frame as a notification. It is for checking the radio and the link with a generic BLE client (`docs/BLE_CONTROL_PLANE.md` §5). Enable it with the `TG_BLE_ECHO` build flag through `PLATFORMIO_BUILD_FLAGS`, for example:

```bash
PLATFORMIO_BUILD_FLAGS=-DTG_BLE_ECHO=1 pio run -e pico2w_synthetic_cli
```

The override needs a W env (`TG_BLE=1`). Do not flash the echo build to a unit you use for control: it has no control plane.

### Double-speed synthetic loop (HW-20, test only)

`TG_SYNTH_SPEED` (default `1`) makes the synthetic source skip loop samples: with `2` it advances the recording by two samples per output sample, so the loop plays at double speed and 18,000 bph becomes 36,000 bph (10 beats per second). The sample rate stays 48 kHz and the pitch rises an octave. It is a test-only flag for the 36,000 bph throughput case (plan section 14.6, HW-20); no shipped env sets it. Valid values are `1` and `2`. It applies to `AUDIO_SOURCE=SYNTHETIC` builds only. Example:

```bash
PLATFORMIO_BUILD_FLAGS=-DTG_SYNTH_SPEED=2 pio run -e pico2w_synthetic -t upload
```

### Recorded watch clips for the synthetic source

`TG_SYNTH_CLIP=<id>` makes a synthetic build loop a trimmed recording of a real watch from `assets/synth_clips/` instead of the default tick-tock loop. Each loop holds a whole number of tick-tock pairs, cut so that a timegrapher reads the watch's rate from it; beat error and amplitude come from the real ticks. The clips, their expected values and how they were cut are listed in [`assets/synth_clips/README.md`](../assets/synth_clips/README.md). `hello.source` reports the clip (`SYNTHETIC:<id>`). The pre-build script `scripts/synth_clip_header.py` generates the table; an unknown id stops the build and lists the known ones. Example:

```bash
PLATFORMIO_BUILD_FLAGS=-DTG_SYNTH_CLIP=nh35_21600 pio run -e pico2w_synthetic -t upload
```

It combines with `TG_SYNTH_SPEED=2` (double speed). A 4 s clip adds about 345 KB of flash; `switch_21600_28800` adds about 2.8 MB.

### Radio core

The radio (CYW43 driver plus BTstack) runs as low-priority interrupt work on the core that initialised it. Every W env with BLE runs it on **core 1**, next to the display, so BT bursts pre-empt only the display and never USB, capture or DSP on core 0. HW-19 decided this on 2026-11-23: core 1 had 0 audio discontinuities and a longest main-loop pass of 6.5 ms, against 5 and 8.7 ms on core 0 (`docs/acceptance_results.md`).

Two settings make it so, and both are in `platformio.ini`:

- `[ble]` passes `-DTG_BLE_CORE=1`.
- `[board_pico2w]` selects the project variant `variants/rpipico2w_tg` (`board_build.variants_dir = variants`, `board_build.variant = rpipico2w_tg`). Its `initVariant()` does not start the radio when `TG_BLE_CORE` is 1; `src/ble_core.cpp` starts it from core 1. Envs without `[ble]` (`pico2w_synthetic_noble`) build the same code as the stock variant.

`getLink.ble.core` reports `0` or `1`, and `getLink.ble.core1StackFree` the free bytes of core 1's 8 KB stack (core 1 only; 5.4 to 6.7 KB in HW-19).

**Override: radio on core 0.** No shipped env builds this placement. Add a custom env that uses the stock framework variant and `-DTG_BLE_CORE=0`. Spell out the `[ble]` flags instead of inheriting them: PlatformIO moves `-U` options behind `-D` ones, so `-UTG_BLE_CORE` cannot cancel the inherited `-DTG_BLE_CORE=1`. Example, in a local `platformio.ini` section or an extra config:

```ini
[env:pico2w_synthetic_core0]
extends = env:pico2w_synthetic
; stock framework variant: initVariant() starts the radio on core 0
board_build.variants_dir = ~/.platformio/packages/framework-arduinopico/variants
board_build.variant = rpipico2w
build_flags =
	${board_pico2w.build_flags}
	-DPIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH
	-DTG_BLE=1
	-DTG_BLE_CORE=0
	-DAUDIO_SOURCE=SYNTHETIC
```

Expand `~` to the absolute path of your PlatformIO home. The variant and the flag must agree: the project variant with `-DTG_BLE_CORE=0` is also correct (it starts the radio on core 0), but the stock variant with `-DTG_BLE_CORE=1` starts the radio twice. `tests/check_ble_core.sh --build` builds exactly this env from a temporary config and checks both ELFs (CORE-01, BLD-06).

## Serial CLI

The serial CLI (`help`, `status`, `diag`, `dsp`, `quartz` and the others) is retired from the shipped builds. USB HID and BLE are the control planes; the CLI to HID mapping is in [`HID_CONTROL_PLANE.md`](HID_CONTROL_PLANE.md) Appendix A. The CLI stays available for bench work behind the `TG_SERIAL_CLI` flag.

| Env | Contents |
|---|---|
| `pico2_pcm1808_cli` | `pico2_pcm1808` plus the CLI (CI builds it so the CLI keeps compiling) |
| `pico2w_pcm1808_cli` | `pico2w_pcm1808` (BLE) plus the CLI |
| `pico2w_synthetic_cli` | `pico2w_synthetic` (BLE) plus the CLI |
| `pico2w_synthetic_noble` | W board without BLE, with the CLI (hardware A/B) |

What the flag does:

- `[cli]` in `platformio.ini` sets `-DTG_SERIAL_CLI=1` and a `build_src_filter` that compiles `src/cli/` and `src/serial_cli.cpp`. `[base]` excludes both. An env's own `build_src_filter` replaces the inherited one, so a new CLI env needs both `${cli.build_flags}` in `build_flags` and `build_src_filter = ${cli.build_src_filter}`.
- Without the flag the CLI sources are not compiled, `serial_cli_init()` and `serial_cli_tick()` are inline no-ops, and the periodic `diag` print and `diagnostics_set_print_enabled()` are compiled out. The counters still accumulate (`getDiag`). Nothing writes to `Serial`.
- Saving: `pico2_pcm1808` without the CLI is 1,876 B of RAM and 52,856 B of flash smaller than `pico2_pcm1808_cli` (162,728 B / 202,792 B against 164,604 B / 255,648 B).
- The CDC port stays in every build, with or without the CLI. `pio run -e <env> -t upload` auto-resets a running board into BOOTSEL as before (the library handles the 1200-baud touch). A terminal on the port shows nothing in a build without the CLI, and input is ignored.
- `scripts/calibrate_timebase.py` drives the CLI and needs a `*_cli` build. It is legacy; the web app's PC-assisted calibration is the supported path.
- The window extrema (ring occupancy min/max, ADC min/max and peaks) were cleared by each periodic print. Without the CLI nothing clears them, so they cover the time since boot (or since a stream open, where that already cleared them).
- Hardware-day diagnostics use `getDiag`, `getTiming` and `getLink` over HID or BLE, or a `*_cli` env.

## Saved settings

The calibration, device mode, `logDeltaPpm` and the calibration log live in the last 4 KiB flash sector (Arduino-Pico EEPROM emulation, one 1 KB image owned by `flash_store`). That sector is outside the program image, so an ordinary firmware upload should leave it in place (not verified on hardware here; a full flash erase clears it, and the device then reads defaults). A firmware that grows the image keeps the old 256-byte calibration record. A write is refused while the USB audio stream is open; on a standalone device it pauses capture for a few tens of ms. See `docs/AS_BUILT_ARCHITECTURE.md` §4.3.

## Recommended bring-up order

1. Flash `pico2_synthetic` or `kiwi_synthetic`.
2. Confirm host sees **Watch Timing Microphone** (and a CDC serial port, kept for the upload reset; it carries no CLI unless the env is a `*_cli` one).
3. Record 60 s WAV; run `tests/host_analyze.py` — expect the looped `tick-tock.wav` clip (~400 ms period).
4. Verify analog bias on ADC pin **before** connecting TL071 SIG.
5. Flash ADC env; capture silence + watch ticks.
6. For the PCM1808 envs: check `getDiag` and `getTimebase` over HID (or `status` on a `*_cli` build) for the PCM1808 rate status `ok` and `timebase=ok`, then record; see `docs/PCM1808_WIRING_AND_CLOCKS.md`. Try standalone mode (`setDevice`, or `device mode standalone` on a `*_cli` build; results on the OLED, header tag `SA`) and quartz modes (`patchState`, or `quartz mode motor|lcd|fork` on a `*_cli` build).

## Changing the ADC pin

Edit `ADC_GPIO` / `ADC_CHANNEL` in `platformio.ini` for the relevant env, or add a new env. Defaults:

| Board | Package | ADC0 GPIO |
|---|---|---|
| Pico 2 | RP2350A | 26 |
| Kiwi / RP2354 | RP2350B | 40 |
