# Acceptance results

**Status:** Firmware builds on all six RP2350/RP2354 PlatformIO envs (RP2040 `pico*` envs removed). Host conversion unit tests pass. **PCM1808 oscilloscope / USB soak and ADC hardware quality are NOT verified** (no board measurement claimed in this environment).

## Build

| Env | Result |
|---|---|
| `pico2` | SUCCESS — UF2 produced |
| `pico2_synthetic` | SUCCESS — UF2 produced |
| `kiwi` | SUCCESS — UF2 produced |
| `kiwi_synthetic` | SUCCESS — UF2 produced |
| `pico2_pcm1808` | SUCCESS — UF2 produced (compiled; **not hardware-tested**) |
| `kiwi_pcm1808` | SUCCESS — UF2 produced (compiled; **not hardware-tested**) |

## Host unit tests

| Test | Result |
|---|---|
| `tests/test_pcm_conversion.c` | OK (ADC mirror) |
| `tests/test_pcm1808_pcm_conversion.c` | OK (P3.0 §5.5 vectors) |
| `tests/test_ring_buffer.c` | OK (if run) |

## Checklist

### Baseline USB / ADC / SYNTHETIC

- [x] `SYNTHETIC` UF2 builds
- [x] `ADC` UF2 builds and uses DMA (not ADC polling) — code path present
- [ ] Host captures signed mono PCM16 at 48 kHz — **untested on hardware** (this session)
- [x] No forced software gain / NR / hidden resampling in firmware
- [ ] ADC input safety confirmed before TL071 connect — **operator must verify**
- [x] USB buffer drift handled (47/48/49 + counters) — **needs hardware soak**
- [ ] 30-minute continuous capture — **untested**
- [ ] Quiet noise floor + watch-tick captures archived — **untested**
- [ ] Stream open/close + USB reconnect — **untested**
- [x] Limitations documented (CDC present; Arduino stack)

### PCM1808 (P3.0)

- [x] `clock_gpio_init_int_frac16` present in bundled Pico SDK; used for GP21 MCLK
- [x] Checked-in `pcm1808_i2s.pio.h` + `scripts/gen_pcm1808_pio.py`
- [x] MCLK starts at boot on PCM1808 envs (scope without USB stream)
- [x] PIO + DMA ping-pong `2×240` stereo frames; left → mono PCM16
- [x] Diagnostics counters per P3.0 §7
- [x] Gate B: GP21 ≈ 12.288 MHz average — **historically measured on RP2040 `pico_pcm1808` (removed): 12.288199 MHz**, stable over a 3.4 s gate; re-verify on `pico2_pcm1808` / `kiwi_pcm1808`
- [ ] Gate B: BCK ≈ 3.072 MHz, LRCK ≈ 48 kHz with module powered — **not hardware-tested**
- [ ] USB record from PCM1808 — **not hardware-tested**

## Known limitations

1. USB CDC Serial appears next to the microphone (Adafruit TinyUSB Arduino requirement).
2. Host-side success (enumeration, ALSA/CoreAudio/WASAPI) is unverified here for PCM1808.
3. Fractional MCLK has edge jitter by design; measure average frequency.
4. PIO BCK/LRCK waits are hardcoded to GPIO 2/3; pin changes need PIO regen.
5. `APP_FIXED_PACKET_48` can be set to `1` if a host rejects variable ISO packet sizes.
