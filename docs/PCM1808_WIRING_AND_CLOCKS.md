# PCM1808 wiring and clocks (Gate B)

Comimark LY451-style **PCM1808** module in master I²S mode, clocked by RP2350 GPOUT0.

Supported PlatformIO envs (RP2350-only; RP2040 `pico_pcm1808` removed): `pico2_pcm1808`, `kiwi_pcm1808`.

## Expected clocks (software-configured)

| Signal | Pico 2 GPIO | Kiwi / legacy GPIO | Expected | Notes |
|---|---|---|---|---|
| SCKI (MCLK) | **GP21** | **GP21** | **~12.288 MHz average** | `clock_gpio_init_int_frac16(21, PLL_SYS, 12, 0x3500)` |
| BCK | **GP18** | GP2 | ~3.072 MHz | Driven by PCM1808 after power + straps |
| LRCK / LRC | **GP20** | GP3 | ~48 kHz | Driven by PCM1808 |
| DOUT / OUT | **GP19** | GP4 | I²S data | Sampled by PIO |

Fractional GPOUT divider math:

```text
# RP2350 (16-bit frac field):
f_out = 150_000_000 / (12 + 0x3500/65536)   # 0x3500/65536 = 53/256
      ≈ 12_288_000 Hz average
```

**Edge jitter is expected** with a fractional divider. A frequency counter averaging many periods should read ~12.288 MHz. Oscilloscope period-to-period variation does **not** by itself mean misconfiguration.

Software checks `clock_get_hz(clk_sys)==150e6` and `clock_get_hz(clk_usb)==48e6` at boot. That validates configured rates only — **not** a substitute for probing GP21.

MCLK starts in `audio_source_init()` on PCM1808 builds so GP21 toggles **without** opening a USB stream.

## Strap levels (before module power)

MD1/MD0 select master or slave mode and the SCKI ratio (TI SLES177). With 12.288 MHz SCKI:

| MD1 | MD0 | Mode | LRCK (fs) | Firmware |
|---|---|---|---|---|
| L | L | Slave | — | Not supported (the RP2350 does not drive BCK/LRCK) |
| L | H | Master, 512×fs | 24 kHz | Supported: each frame repeated ×2 to USB |
| H | L | Master, 384×fs | 32 kHz | Detected and reported, timing refused (48/32 is not an integer repeat) |
| H | H | Master, 256×fs | 48 kHz | Supported: frames passed 1:1 to USB |

FMT = L selects I²S, which the PIO program expects. Check the module silkscreen too, since many modules have their own pull resistors on these pins.

The intended straps are MD1=H, MD0=H (48 kHz). The bench module measured about 24 k frames/s, which is consistent with 512×fs. The firmware no longer depends on which is true. About 1 s after capture starts, it measures the frame rate from DMA block timestamps, snaps it to one of the ratios above, and sets both the DSP analysis rate and the USB sample repeat to match. Until a rate is confirmed, or if it matches no supported ratio, `beat_rate` refuses to lock, so no rate, beat error or amplitude is ever computed against a guessed clock. Check the result with the CLI `status` command:

```
CLI: pcm1808 rate=ok measured_hz=24000.012 mclk_ratio=512 upsample=2 dsp_hz=24000 timebase=ok
```

The measurement counts frames as the PIO delivers them. If it reports 24 kHz while a scope shows LRCK at 48 kHz, the PIO is dropping frames: that is a capture bug, not a strap setting.

SCKI and the timer both come from the same crystal, so the measurement identifies the ratio but cannot see crystal error. To correct for that, see [TIMEBASE_CALIBRATION.md](TIMEBASE_CALIBRATION.md).

## Wiring tables

### Pico 2 (`pico2_pcm1808`) — PCB alignment (no-cross harness)

| Module | Pico 2 GPIO | Header pin | Direction |
|---|---|---|---|
| SCKI | GP21 | 27 | Pico → PCM1808, 12.288 MHz |
| LRC / LRCK | GP20 | 26 | PCM1808 → Pico, 48 kHz |
| OUT / DOUT | GP19 | 25 | PCM1808 → Pico, serial data |
| BCK | GP18 | 24 | PCM1808 → Pico, 3.072 MHz |
| 3.3 V dig | 3V3 | — | Do not backfeed |
| 5 V analog | VSYS/5V as appropriate | — | Verify module regulator scheme |
| GND | GND | — | Common ground |

### Kiwi (`kiwi_pcm1808`) — unchanged legacy map

| Module | RP2350 | Notes |
|---|---|---|
| SCKI | GP21 | 3.3 V logic from GPOUT0 |
| BCK | GP2 | Input to MCU |
| LRCK / WS | GP3 | Input to MCU |
| DOUT | GP4 | Input to MCU |
| 3.3 V dig | 3V3 | Do not backfeed |
| 5 V analog | VSYS/5V as appropriate | Verify module regulator scheme |
| GND | GND | Common ground |

**Channels.** Both I²S slots are decoded. LEFT carries the inductive pickup and is the only channel the USB microphone and the mechanical DSP graph use; RIGHT carries the capacitive (LCD) pickup and is read only by the quartz engine (`pcm1808_raw_left/right_peak_abs` in the diagnostics). See [QUARTZ_TIMING.md](QUARTZ_TIMING.md).

**GPS PPS (optional).** `pico2_pcm1808` also builds with `-DPPS_GPIO=3`: a GPS 1 PPS output goes to GP3, which is free on the Pico 2 harness above. On `kiwi_pcm1808` GP3 is LRCK, so that env has no PPS input unless you choose another pin. The NMEA input (`-DGPS_UART_RX`) is off in every env. See [TIMEBASE_CALIBRATION.md](TIMEBASE_CALIBRATION.md).

If GP21 is unavailable, RP2350 also routes GPOUT0 to **GP13** — change `PCM1808_MCLK_GPIO` and rewire together.

BCK/LRCK wait immediates are **baked into board-specific PIO headers** (`pcm1808_i2s_pico2.pio.h` / `pcm1808_i2s_legacy.pio.h`). Changing those pins requires regenerating with `python3 scripts/gen_pcm1808_pio.py` (edit `BOARDS` in that script).

## Gate B scope checklist

1. Straps MD1=H, MD0=H, FMT=L **before** applying module power.
2. Power module; verify 5 V analog / 3.3 V digital rails; no MCU pin backfeed.
3. Flash `pico2_pcm1808` or `kiwi_pcm1808`; measure **GP21** ≈ 12.288 MHz average.
4. With module clocked: **BCK ≈ 3.072 MHz**, **LRCK ≈ 48 kHz** (Pico 2: probe GP18 / GP20). At 512×fs straps expect half of both. Then confirm CLI `status` reports the same rate with `rate=ok`.
5. Only then proceed to USB recording / capture soak.

BCK/LRCK appear only after the module is powered with correct straps and receives SCKI.

## Measured results

| Check | Board / env | Result |
|---|---|---|
| GP21 SCKI average | Historical RP2040 `pico_pcm1808` (env removed) | **12.288199 MHz** (2026-11-04); stable over a **3.4 s** counter gate |

BCK/LRCK with module powered and USB capture remain unchecked.
