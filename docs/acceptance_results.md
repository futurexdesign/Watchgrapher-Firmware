# Acceptance results

**Status:** Firmware builds on all six RP2350/RP2354 PlatformIO envs (RP2040 `pico*` envs removed); the four CI envs were rebuilt on 2026-11-18 (sizes below). `tests/run_all.sh` passes 15 of 15 rows, including the quartz acceptance vectors. **PCM1808 oscilloscope / USB soak, ADC hardware quality, and the newer features on a real board (standalone mode and run latch, flash store, calibration log, GPS, OLED quartz screens, quartz timing on real watches) are NOT verified** (no board measurement claimed in this environment).

## Build

| Env | Result | RAM / Flash used (2026-11-18) |
|---|---|---|
| `pico2` | SUCCESS — UF2 produced | 92,352 B / 205,936 B |
| `pico2_synthetic` | SUCCESS — UF2 produced | 79,856 B / 242,792 B |
| `kiwi` | SUCCESS (earlier build; not rebuilt in the 2026-11-18 run) | — |
| `kiwi_synthetic` | SUCCESS (earlier build; not rebuilt in the 2026-11-18 run) | — |
| `pico2_pcm1808` | SUCCESS — UF2 produced (compiled; **not hardware-tested**) | 164,600 B / 255,208 B |
| `kiwi_pcm1808` | SUCCESS — UF2 produced (compiled; **not hardware-tested**) | 164,308 B / 254,344 B |

## Host unit tests

| Test | Result |
|---|---|
| `tests/test_pcm_conversion.c` | OK (ADC mirror) |
| `tests/test_pcm1808_pcm_conversion.cpp` | OK (P3.0 §5.5 vectors) |
| `tests/test_pcm1808_rate.cpp` | OK (256/384/512 fs classification) |
| `tests/test_beat_fit.cpp` | OK (≈0.1 s/d on whole-sample ticks; dropouts, spurious events, long runs) |
| `tests/test_pps_filter.cpp` | OK (23.4 ppm ± 0.001; glitch / missed-pulse rejection) |
| `tests/test_ui_format.cpp` | OK (rate / amplitude / BE / bph formatting, widths, trend scale) |
| `tests/test_calibrate_timebase.py` | OK (30 min synthetic: 23.4 ppm ± 0.02; min-RTT 6× better with latency spikes) |
| `tests/test_ring_buffer.c` | OK |
| `tests/test_nmea.cpp` | OK (RMC / GGA / GSA / GSV, multi-GNSS union, PRN normalisation, ageing) |
| `tests/test_dsp_blocks.cpp` | OK (block table and pipeline graph agree) |
| `tests/test_timebase_log.cpp` | OK (ring, flash record, PPS policy) |
| `tests/test_device_cli_format.cpp` | OK (device / run / beats / graph / GPS / log formatters) |
| `tests/build_quartz_tests.sh` | OK: `test_quartz_units`, `test_hid_json`, `test_quartz_cli_format`, and with the git-ignored vectors present `test_quartz_vectors` (handoff §9 table) and `test_quartz_engine` |

Run everything with `tests/run_all.sh`: 15 rows, 15 PASS, 0 FAIL, 0 SKIP on 2026-11-18 (the quartz vector and engine rows are SKIP when `docs/handoff/vectors/*.wav` is absent, as in CI). On hardware nothing here replaces the soak items below.

## Checklist

### Baseline USB / ADC / SYNTHETIC

- [x] `SYNTHETIC` UF2 builds
- [x] `ADC` UF2 builds and uses DMA (not ADC polling) — code path present
- [ ] Host captures signed mono PCM24 (S24LE) at 48 kHz — **untested on hardware** (this session)
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
- [x] PIO + DMA ping-pong `2×240` stereo frames; left → mono PCM24 (right kept raw for the quartz engine)
- [x] Diagnostics counters per P3.0 §7
- [x] Gate B: GP21 ≈ 12.288 MHz average — **historically measured on RP2040 `pico_pcm1808` (removed): 12.288199 MHz**, stable over a 3.4 s gate; re-verify on `pico2_pcm1808` / `kiwi_pcm1808`
- [ ] Gate B: BCK ≈ 3.072 MHz, LRCK ≈ 48 kHz with module powered — **not hardware-tested**
- [ ] USB record from PCM1808 — **not hardware-tested**

## Known limitations

1. USB CDC Serial appears next to the microphone (Adafruit TinyUSB Arduino requirement).
2. Host-side success (enumeration, ALSA/CoreAudio/WASAPI) is unverified here for PCM1808.
3. Fractional MCLK has edge jitter by design; measure average frequency.
4. PIO BCK/LRCK waits are baked into board-specific headers (Pico 2: GP18/GP20, Kiwi/legacy: GP2/GP3); pin changes need PIO regen (`scripts/gen_pcm1808_pio.py`).
5. ISO packet sizing (47/48/49) is handled by TinyUSB flow control (`CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL=1`); the former `APP_FIXED_PACKET_48` knob was unused and has been removed.
