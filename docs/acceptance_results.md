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

## BLE transport: hardware day 2026-11-23 (Pico 2 W, synthetic source)

Plan: `docs/plans/BLE_TRANSPORT_PLAN.md` (rev. 4); test IDs from `docs/plans/BLE_TRANSPORT_TESTS.md`. Board: Pico 2 W, `TG-F7B4`, USB to a MacBook Air (macOS, Chrome 157). No PCM1808 attached, so every audio test uses the synthetic source, which since rev. 4 also feeds the DSP graph (it reads 18,000 bph, +36.0 s/d, as expected from its 19,192-sample loop).

**Bench tools** (manager scratch, not in the repo): a Python client for the framed protocol over USB HID (`hidapi`) and BLE (`bleak`, the Mac as central); USB audio recorded with PortAudio (`sounddevice`, 48 kHz S24). Every recording is checked **sample by sample** against `include/tick_tock_pcm.h` (±2 LSB for CoreAudio's float path): any dropped, repeated or corrupted sample counts as a discontinuity. ffmpeg's AVFoundation capture was rejected as a recorder: it dropped 512-sample buffers on the host (22 % of a 15 s take) while the device counters were clean.

### Defects found on hardware and fixed (all in the working tree)

| # | Symptom | Cause | Fix | Regression test |
|---|---|---|---|---|
| 1 | Web app: "timeout waiting for hello" | A3-M2 (BLE session) not yet wired; frames were counted and dropped | `BleLink` + `Session` + BLE `ArbiterPort` (`src/ble_control.cpp`, `src/control/arbiter_port.cpp`) | HW-05 |
| 2 | Hello still timed out; `getLink`: one notification per connection | BTstack can deliver the can-send callback **inside** the registration call; the pacer then marked it pending after it had already run, so it never asked again | `BlePacer::request()` sets `pending_` before the call (`include/control/ble_pacer.h`) | PACE-11 |
| 3 | With every stream subscribed at its maximum, `unsubscribe` never got a reply | `Session::task()` started the next notify in the same pass that finished the previous one, so on a link busy at the start of every task a waiting request was never dispatched | A waiting request now goes before the next notify (`src/control/session.cpp`) | SES-15 |
| 4 | Audio gaps in the synthetic build | A ring refill could run up to six DSP blocks in one main-loop pass | At most one DSP block per pass (`src/audio_source.cpp`) | sample check |
| 5 | Audio gaps whenever BLE carried traffic (both radio cores) | USB IN FIFO of 4 packets: flow control keeps it half full, ~2 ms of slack; BLE load stretches main-loop passes to 4–9 ms (on core 1 through slower core-0 code, probably XIP cache contention) | `CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ` 588 → 2352 (16 packets, ~8 ms), catch-up 5 → 12 frames (`platformio.ini`, `src/usb_audio.cpp`); +1,764 B RAM in every env | sample check |

### Build and static (BLD, CLI, CORE)

| ID | Result | Notes |
|---|---|---|
| BLD-01 | **Fail (marginal, explained)** | All non-W envs build. `pico2_pcm1808_cli`: 166,396 B RAM / 261,476 B flash vs limits 165,624 / 261,360. Flash +116 B over (M2 wiring); RAM over by the deliberate +1,764 B USB FIFO (defect 5). Limits need re-baselining |
| BLD-02 | Pass | `pico2w_pcm1808` 251,928 B RAM (≤ 262,144), 618,880 B flash (≤ 720 KB) |
| BLD-03 | Pass | `ci.yml` parses; matrix has `pico2w_pcm1808`, `pico2w_synthetic` |
| BLD-04 | Pass | cppcheck (CI flags) exit 0; clang-format clean on changed lines |
| BLD-05 | Pass | No `setValue` on TX; no heap calls in `ble_control.cpp` / `src/control/*.cpp` outside setup |
| BLD-06 | Pass | `pico2w_synthetic` (core 1 default) links `initVariant` from `variants/rpipico2w_tg` |
| CLI-01 | Pass | `pico2_pcm1808` vs `_cli`: flash −53,312 B, RAM −1,880 B |
| CLI-02 | Pass | No `cli/` or `serial_cli` objects in the no-CLI build |
| CLI-04 | Pass | Every `Serial.` use outside CLI sources is inside `#if TG_SERIAL_CLI`, except `Serial.begin()` (CDC kept, F-18) |
| CLI-05 | Pass | No `serial_cli_*` symbols in the no-CLI ELF |
| CLI-07 | Pass | `tud_cdc_line_state_cb` and the descriptor callback present; `usb_audio.cpp` untouched by A8 |
| CORE-01 | Pass | Core-0 ELF: `initVariant` calls `init_cyw43_wifi`, no `ble_core_setup1`; core-1 ELF: `initVariant` calls nothing |
| CORE-02 / 04 | Pass | `tests/check_ble_core.sh` |
| CORE-05 | Pass | Core 1: `core == 1`, `core1StackFree` 5,456–6,752 B (≥ 2,048) under load |
| Host tests | Pass | `tests/run_all.sh`: 22 / 22 (adds PACE-11, SES-15) |

### Hardware

| ID | Result | Measurements |
|---|---|---|
| HW-04 | Pass (macOS Chrome / bleak) | ATT MTU 527 (INFO), bleak reports 515; ≥ 66. iPad/Bluefy, Windows, Android: **not run** |
| HW-05 | Pass | `hello` over BLE: 32 frames, link `{kind: ble, mtu: 527}` |
| HW-07 | Pass, with a finding | `ping` ×100: mean 63–65 ms, p95 89 ms, max 120–147 ms (30 ms connection interval). Each stream alone reaches its cap (metrics 20 Hz, beat/timing 5 Hz, events/beats 5/s). **Link capacity ≈ 100 frames/s (≈ 6 KB/s, macOS).** Frames per message: beat 16, timing 8, event 6, beats 5, metrics 4. The current `LINK_RATE_CAPS.webble` (10/2/2/2 + events + beats) asks for ≈ 143 frames/s, so streams share the link and events fall to ~1/s. iPad/Bluefy figures still needed to set the caps |
| HW-10 | Pass (Mac USB HID + Mac BLE) | BLE request while USB owns: `linkBusy`; BLE `claim {takeover}` → USB gets `controlReleased {to: ble}`; reverse works. The iPad dialog part: **not run** |
| HW-11 | Pass (Mac variant) | 20 connect / hello / disconnect cycles: 0 failures, find+connect mean 1.49 s, max 2.84 s; `disconnects` +20; advertising resumes. iPad cycles: **not run** |
| HW-14 | Pass | 10 × `saveTimebase` over BLE: all ok, 121–237 ms each, no disconnect |
| HW-18 | Pass | `pio run -t upload` on the no-CLI `pico2w_synthetic`: 1200-baud reset into BOOTSEL over the kept CDC port, upload 17 s |
| HW-19 | **Decided: core 1 default (2026-11-23)** | 30 s, BLE subscribed at caps, after defect 5's fix: core 0: 5 discontinuities, loop max 8.7 ms, DSP block max 3.1 ms; core 1: 0 discontinuities, loop max 6.5 ms, DSP block max 1.8 ms. Without BLE traffic both are clean. 10 min soak on core 1 clean; `core1StackFree` 5.4 to 6.7 KB. Decision: `TG_BLE_CORE=1` and the project variant are the default of every W env; core 0 is an override. iPad part: **not run** |
| HW-02 (core 1, BLE advertising) | **Open: rare gap, source unconfirmed** | Run 1 (14-env compile on the same Mac): 2 gaps of 836 / 944 samples at 531.7 s. Run 2 (idle Mac): 1 gap of 676 samples (14 ms) at 150.7 s. Both: device counters clean (0 underrun / overrun / short write, samples sent = 600 s), loop max 1.4 / 3.6 ms (well inside the FIFO slack), PortAudio no overflow. The BLE-loaded 10 min run had 0 gaps, so the gaps don't follow device load. Most likely host-side (CoreAudio dropping buffers PortAudio doesn't flag); to confirm, record on a second host (Linux, `arecord`) |
| HW-06 / HW-13 short (core 1) | Pass | 10 min, USB audio + BLE subscribed at `LINK_RATE_CAPS.webble`: **0 discontinuities in 28.8 M samples**, 0 underruns / overruns / short writes, 0 BLE disconnects / retries / drops, ≈ 102 frames/s. Loop max 8.3 ms: at the edge of the ~8 ms FIFO slack |
| HW-03, 08, 09, 12, 15, 16, 17 | Not run | Need nRF Connect / iPad / Android / power bank / OLED observation by the user |
| HW-01 (PCM1808), HW-00 | Not run | No PCM1808 attached; no plain Pico 2 baseline run |
| HW-13 (1 h soak) | Not run yet | |

### Follow-ups from the measurements

- **Loop-time margin.** Under BLE load the core-1 build still reaches 8.3 ms main-loop passes against ~8 ms of USB slack. Candidates: put the hot DSP and USB-feed code in RAM (`__not_in_flash_func`) to remove the XIP contention, move the large `beat` JSON off the per-poll path, or raise the IN FIFO further (latency is irrelevant here).
- **BLE rate caps.** Reduce `LINK_RATE_CAPS.webble` to fit ≈ 80 frames/s (for example metrics 4 Hz, beat 1 Hz, timing 1 Hz, with events and beats), or shrink the `beat` notify, after the iPad measurement.
- **`TG_BLE_CORE`.** Decided 2026-11-23: core 1 is the default for every W env (`[ble]` flag, project variant in `[board_pico2w]`); env `pico2w_synthetic_core1` is removed. Core 0 remains a documented override (`docs/build_and_flash.md`, "Radio core").

### Revision 5: BLE throughput (packed beats, large frames, 15 ms interval), 2026-11-23

Plan §14; tests §12. Board on `pico2w_synthetic` (radio on core 1, the default since 2026-11-23).

| ID | Result | Measurements |
|---|---|---|
| Host BP-01..04, BP-06 (encoder), LF-01..05 | Pass | `tests/run_all.sh` 23 / 23 (adds `beats_packed`); web 43 files / 564 tests (PK-, LF-W- ids) |
| 15 ms interval (§14.4) | Pass (macOS) | Accepted first time: `connIntervalUs` 15000, `connParamStep` 1, 0 rejects. Ping 32 ms mean / 44 ms p95 (was 65 / 89 ms at 30 ms) |
| Large frames (§14.3) | Pass after a fix | **Finding:** a notification value above 248 bytes (HCI ACL payload > 255) gets no answer and leaves the radio unable to advertise until reset (bisected: 245 and 248 work, 249 / 251 / 255 fail). `txMax` is therefore **244** on the Pico 2 W (`kTxFrameMax`, `src/ble_control.cpp`); a `frameMax` of 255 negotiates to 244 and runs cleanly |
| HW-07b | Pass | All streams at their maximum together (metrics 20 Hz, beat / timing / events / packed beats 5 Hz): 63-byte frames: 203 notifications/s, 11.8 KB/s, metrics only reaches 13 Hz (saturated); **244-byte frames: every stream at its maximum, 65 notifications/s, 13.1 KB/s**. At that load the link is byte-limited (≈ 13 KB/s, data length extension off) and ping rises to ≈ 350 ms. Packed beats: 5 beats/s cost 630 B/s (JSON `beats`: 1,365 B/s) |
| HW-20 (36,000 bph) | **Pass (link)** | `TG_SYNTH_SPEED=2`, 591 s, packed beats + `beat` 2 Hz + `metrics` 5 Hz, 244-byte frames: locked 36,000 bph; **5,907 / 5,907 beats delivered (seq 93–5999), no gaps, no duplicates**; arrival lag above the fastest beat p50 15 ms, p95 79 ms, max 237 ms; 23 notifications/s, 4.1 KB/s ≈ 31 % of capacity; 0 retries, no disconnect. USB audio: 3 clean skips (616–800 samples), the same signature as the open HW-02 item (device counters clean, loop max 5.2 ms) |

**Budget note (superseded the same day: BLD-02 raised to 314,572 B, TIMING_ENGINE_PLAN.md §6):** `pico2w_pcm1808` RAM was 256,336 B against the then BLD-02 limit of 262,144 B (5.8 KB left) after the 16-packet USB FIFO, the 244-byte TX ring and the 32-beat scratch. The legacy JSON `beats` encoder is the first candidate to drop if space runs out.
| HW-04 / §14.4 (iPhone, web app Link panel) | Pass | Reported by the user, 2026-11-23: connection interval **15.00 ms** (the 15 ms request accepted), ATT MTU **527**, so 244-byte frames fit (`txMax`). Throughput and the HW-08 view walk-through on iOS are still to do |

### Recorded watch clips (synthetic source), 2026-11-23

`assets/synth_clips/` (README has the full table): nine whole-beat loops and one watch-change clip cut from the ChronoPulse recordings; `-DTG_SYNTH_CLIP=<id>` plays one. On the board (52°, 20 s): every clip locks at its nominal rate and reads its loop's designed rate within 0.25 s/d. Two firmware defects found and fixed, both covered by host rows that run the real DSP chain over the clips (`tests/dsp_host`, `tests/run_all.sh`: `dsp-host-nh35`, `dsp-host-st3600-amp`, `dsp-host-switch`; each fails on the previous code):

| Defect | Fix | Result |
|---|---|---|
| Amplitude invalid on clipped recordings (ST3600: 0 of 256 ticks; Waltham: 81 of 202) | Drop = strongest pulse by 0.5 ms mean \|x\|, not peak sample; a full pulse list keeps the strongest (`tick_detect.cpp`) | ST3600 328° (ChronoPulse 327°), Waltham 307° (309°); valid on 256/256 and 199/202 ticks |
| Rolling mean overshot to 26–34 s/d for a whole window after a watch change | Means and amplitude history restart on a new nominal rate; rate / beat error enter the means only once the fit spans ¾ of its window (`beat_analysis.cpp`, `beat_fit_span()`) | Switch clip: empty ~5 s, then +3.2 to +3.8 s/d (28,800) and +7.0 to +7.6 (21,600) |

### OLED orientation, 2026-11-24

**Pass (user-confirmed).** The OLED is correctly oriented with the current
`DISPLAY_ROTATION` setting of `3`.
