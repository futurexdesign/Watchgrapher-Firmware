# PCM1808 wiring and clocks (Gate B)

Comimark LY451-style **PCM1808** module in master I²S mode, clocked by RP2350 GPOUT0.

Supported PlatformIO envs (RP2350-only; RP2040 `pico_pcm1808` removed): `pico2_pcm1808`, `kiwi_pcm1808`.

## Expected clocks (software-configured)

| Signal | Pin | Expected | Notes |
|---|---|---|---|
| SCKI (MCLK) | **GP21** | **~12.288 MHz average** | `clock_gpio_init_int_frac16(21, PLL_SYS, 12, 0x3500)` |
| BCK | GP2 | ~3.072 MHz | Driven by PCM1808 after power + straps |
| LRCK | GP3 | ~48 kHz | Driven by PCM1808 |
| DOUT | GP4 | I²S data | Sampled by PIO |

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

| Strap | Level | Meaning |
|---|---|---|
| MD1 | High | Master mode |
| MD0 | High | 256×fs system clock (with FMT) |
| FMT | Low | I²S format |

Confirm against the module silkscreen / TI PCM1808 datasheet (SLES177B) for the exact MD1/MD0/FMT encoding on your board revision.

## Wiring table

| Module | RP2350 | Notes |
|---|---|---|
| SCKI | GP21 | 3.3 V logic from GPOUT0 |
| BCK | GP2 | Input to MCU |
| LRCK / WS | GP3 | Input to MCU |
| DOUT | GP4 | Input to MCU |
| 3.3 V dig | 3V3 | Do not backfeed |
| 5 V analog | VSYS/5V as appropriate | Verify module regulator scheme |
| GND | GND | Common ground |

If GP21 is unavailable, RP2350 also routes GPOUT0 to **GP13** — change `PCM1808_MCLK_GPIO` and rewire together. BCK/LRCK wait immediates in `pcm1808_i2s.pio` are fixed to GPIO 2/3; changing those pins requires regenerating the PIO header (`scripts/gen_pcm1808_pio.py`).

## Gate B scope checklist

1. Straps MD1=H, MD0=H, FMT=L **before** applying module power.
2. Power module; verify 5 V analog / 3.3 V digital rails; no MCU pin backfeed.
3. Flash `pico2_pcm1808` or `kiwi_pcm1808`; measure **GP21** ≈ 12.288 MHz average.
4. With module clocked: **BCK ≈ 3.072 MHz**, **LRCK ≈ 48 kHz**.
5. Only then proceed to USB recording / capture soak.

BCK/LRCK appear only after the module is powered with correct straps and receives SCKI.

## Measured results

| Check | Board / env | Result |
|---|---|---|
| GP21 SCKI average | Historical RP2040 `pico_pcm1808` (env removed) | **12.288199 MHz** (2026-11-04); stable over a **3.4 s** counter gate |

BCK/LRCK with module powered and USB capture remain unchecked.
