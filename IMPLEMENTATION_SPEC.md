a---
title: RP2350 USB Mono Microphone — Internal ADC Prototype
subtitle: Firmware implementation specification for code generation
version: P1.0
status: Engineering specification — code not yet implemented
created: 2026-11-03
---

# RP2350 USB Mono Microphone — Internal ADC Prototype

**Implementation specification | Revision P1.0 | 2026-11-03**  
**Status:** Code-generator handoff; NOT implemented or hardware-validated.  
**Primary target:** Existing custom RP2350 board, using the microcontroller's integrated ADC.  
**Future target:** PCM1809 external I²S ADC using the same USB audio transport.

## 0. Instructions for the code-generation agent

Implement an **actual, buildable, working firmware project**, not pseudocode. First create a known-good single-channel USB Audio Class 2.0 (UAC2) microphone using TinyUSB's `examples/device/audio_test` as reference, then replace its generated samples with real RP2350 ADC data transferred by DMA. Preserve an explicit synthetic-test mode for diagnostics.

Follow the *MUST* requirements and acceptance tests in this document. Mark any incomplete items clearly and do not claim hardware tests you cannot perform. Use **C/C++ Pico SDK and TinyUSB**; do not require an RTOS or Arduino framework. Use a pinned dependency version/commit and document the exact versions tested. Build for RP2350 (not silently for RP2040).

**If the exact custom board details are unavailable:** use Raspberry Pi Pico 2 / RP2350A as the default compile target, put the analog GPIO and LED selection in `board_config.h`, and document how to change them for the user's board. Do not invent the actual custom-board pinout.

**Do not:** add USB CDC serial, mass storage, HID, a playback interface, a speaker endpoint, Wi-Fi, gain control, automatic gain control (AGC), noise reduction, DSP, resampling, or a boot-time calibration routine that depends on user intervention. Do not use `stdio_usb` because that adds a USB interface; use SWD/RTT or an optional physical UART for debugging.

## 1. Purpose and success definition

Build a proof-of-concept USB recording device that captures watch-tick transients from an existing TL071 preamplifier and appears to a host computer as **a single mono microphone**. The prototype should provide objective data about whether the RP2350's ~9.2 effective-bit ADC is adequate for watch-timing software before migrating to an external PCM1809 I²S ADC.

### Explicit goals

1. Connect a supported RP2350 board to a computer via its native USB port (USB-C on the eventual custom board).
2. Enumerate as one class-compliant UAC2 **recording-only** function, named `Watch Timing Microphone (Prototype)`.
3. Advertise a single continuous recording format: **48,000 samples/second, signed 16-bit little-endian, one channel**.
4. Acquire a live analog input from the RP2350's integrated ADC using free-running clocked conversion, FIFO, and DMA.
5. Stream audio continuously without deliberate sample drops or timing artifacts.
6. Capture unprocessed transient waveforms suitable for comparative analysis, including silence/noise, watch ticks, and a sine reference.
7. Allow test builds with a deterministic digital test signal to isolate USB stack behavior from ADC behavior.
8. Preserve an abstraction that can later accept signed samples from PCM1809 rather than from the internal ADC.

### Non-goals for this prototype

- A production PCB, custom 9 V boost converter, or USB power certification.
- A functioning external PCM1809 I²S interface (design should facilitate it later).
- Sub-second-per-day timebase calibration; measure and document timing behavior first.
- DSP watch rate or beat-error algorithms within the RP2350; the computer's existing watch-timing application analyzes the audio.
- Microphone bias or an additional analog preamplifier; the external TL071 circuit already amplifies the pickup.
- Persistent user settings or on-device audio gain controls.

## 2. Existing system and hardware assumptions

```
Existing watch microphone stand                       Prototype RP2350 board
+-----------------------------+                      +----------------------------------+
| Mechanical pickup           |                      | Analog input conditioning        |
|    -> existing TL071        |-- SIG (analog) ----->| AC coupling / DC bias / LPF      |
|    -> amplified ticks       |                      |   -> RP2350 ADC0                  |
|                             |                      |   -> ADC FIFO -> DMA -> ring      |
| Power input: 8 to 9 V       |<-- 8/9 V supply ----|   -> PCM16 -> TinyUSB UAC2       |---- USB ----> PC
| GND                         |<---------------------| GND                              |
+-----------------------------+                      +----------------------------------+
```

- Source: existing TL071 preamplifier inside a watch microphone stand.
- Measured on an oscilloscope at the stand's output with **8 V** powering the preamp.
- Observed transient peaks were approximately **1.2–1.3 V peak-to-peak**. This is an estimate from a 200 mV/div, AC-coupled capture using a 10:1 probe.
- Actual output DC offset is **unknown** because the scope capture was AC coupled.
- The future external connector has `9V+`, `GND`, and `SIG`. For this proof of concept, an external regulated 8 V or 9 V bench supply may power the preamp independently. All grounds must be common.
- The RP2350 ADC pin MUST NOT receive a negative voltage or a voltage above its ADC supply. The TL071's output MUST NOT be wired to the ADC GPIO directly without proper AC coupling and bias.
- The existing RP2350 board uses a 12 MHz Abracon `ABM8-272-T3` crystal, LCSC **C20625731**. Preserve its normal board clock configuration; do not add a second clock source for this test.
- The RP2350 integrated ADC is 12-bit nominal, approximately 9.2 effective bits, with a 48 MHz ADC clock and maximum conversion rate of 500 ksample/s per official SDK documentation.

### ADC pin selection

| MCU package | ADC input 0 | Default implementation |
|---|---|---|
| RP2350A / QFN-60 / Pico 2 | GPIO26 | `ADC_CHANNEL=0`, `ADC_GPIO=26` |
| RP2350B / QFN-80 | GPIO40 | `ADC_CHANNEL=0`, `ADC_GPIO=40` |

Use only one ADC channel. Disable round-robin conversion. Put pin mappings behind build-time configuration. Confirm that the selected ADC GPIO is physically accessible and not already assigned on the actual board.

## 3. Suggested temporary analog frontend (bench prototype)

**This is a proposed test circuit, not a validated finished analog design.** Prefer using the user's existing analog input board circuitry if it already provides equivalent bias and protection. Build it on a small daughterboard if the RP2350 board does not have an ADC-compatible analog input.

```
Existing SIG --- R_ATT_SER (0R) ---o--- C_IN 4.7 uF --- R_SER 1 kΩ ---o--- ADC0
                                  |                                |
                               R_ATT_SHUNT                         +--- C_LP 10 nF --- AGND
                                 (DNP)                             |
                                  |                                +--- R_BIAS_TOP 22 kΩ --- 3V3_A
                                 GND                               |
                                                                    +--- R_BIAS_BOT 22 kΩ --- AGND

Existing GND --------------------------------------------------------------- AGND
Existing 8/9 V supply is separate from the ADC input.
```

**Parts/function:**

- `R_ATT_SER`: **0 Ω populated initially**; can be substituted with a series resistor if attenuation is required.
- `R_ATT_SHUNT`: **DNP** footprint to ground on the *source side of the AC-coupling capacitor*, permitting a passive divider without disturbing ADC DC bias. Leave absent initially. This preserves the user's preferred optional-DNP approach; **do not use a trimmer**.
- `C_IN`: 4.7 µF nonpolar ceramic, >=6.3 V, preferably >=10 V rating; actual capacitance under DC bias needs verification. Blocks unknown TL071 DC offset.
- `R_SER`: 1 kΩ, limits transients and works with the input capacitor as a preliminary low-pass stage.
- `C_LP`: 10 nF to analog ground near the ADC input; approximate 15.9 kHz single-pole corner for a low-impedance source through 1 kΩ. This is **only a provisional anti-aliasing filter**; it is not sufficient for production-grade Nyquist rejection.
- `R_BIAS_TOP` and `R_BIAS_BOT`: 22 kΩ each to an adequately clean 3.3 V analog supply and analog ground. Nominal DC bias: ~1.65 V before ADC loading and bias errors. The 22k//22k bias network with nominal 4.7 µF coupling gives an approximate 3 Hz first-order high-pass corner, subject to real component conditions.
- Place one small decoupling capacitor near the ADC analog supply connection if not already present on the board; maintain a clean `ADC_AVDD` reference and ground.
- Provide oscilloscope access to `SIG`, `ADC0`, `3V3_A`, and `GND`.
- For this prototype, do not add microphone bias voltage to SIG.

**Voltage safety:** Verify the conditioned ADC voltage is within 0 V to ADC_AVDD under all expected startup, shutdown, and signal conditions. Apply external input clamp protection as appropriate for hot-plugging. A 1 kΩ series resistor alone is not proof of absolute-max compliance. Verify with a scope before connecting to a powered RP2350 ADC input. In particular, confirm the TL071 supply ground and RP2350 ground are connected.

**Measured waveform expectation:** ±0.65 V around a nominal 1.65 V bias becomes approximately 1.0–2.3 V. That fits within a nominal 0–3.3 V range and leaves headroom, assuming the amplifier output and real bias remain as characterized. Start without additional attenuation. If clipping occurs, populate R_ATT_SER and R_ATT_SHUNT appropriately; document final values and the divider loading.

## 4. Firmware platform and toolchain

- Language: **C11**, plus CMake. C++ optional only when necessary for board-specific SDK integrations.
- Official Raspberry Pi **Pico SDK**, minimum RP2350-capable release. Record the exact pinned version in `README.md` and lock dependency revision in the project configuration.
- Official **TinyUSB** device stack, UAC2 example `examples/device/audio_test`. Use the TinyUSB revision actually bundled or explicitly checked out with the SDK; do not mix headers from one revision with source files from another.
- No RTOS; a single `main()` cooperative loop and DMA interrupts are adequate. No busy-wait ADC polling in production streaming mode.
- Build with `PICO_BOARD=pico2` for the reference board if a custom board definition has not yet been supplied. The final user board may require a custom board file, GPIO mapping, flash configuration, and power settings.
- Use native RP2350 USB device peripheral. No external USB-to-serial adapters or USB audio codecs.
- Build an `.elf`, `.bin`, and `.uf2`. Keep diagnostics separate from the USB microphone function.

### Mandatory source tree

```text
watchtiming-rp2350-adc/
  CMakeLists.txt
  pico_sdk_import.cmake            # if needed by selected project structure
  README.md
  LICENSES.md                       # third-party licensing / attribution
  include/
    app_config.h
    board_config.h
    audio_source.h
    audio_ring.h
    adc_capture.h
    usb_audio.h
    diagnostics.h
    tusb_config.h
  src/
    main.c
    adc_capture.c
    audio_ring.c
    audio_source.c
    usb_audio.c
    usb_descriptors.c
    diagnostics.c
  tests/
    test_pcm_conversion.c
    test_ring_buffer.c
    host_capture.md
    host_analyze.py
  docs/
    build_and_flash.md
    acceptance_results.md
```

Do not create placeholder modules that are never integrated. Simplify the structure if necessary but maintain separation between ADC capture, sample conversion, buffering, and USB transport.

## 5. USB functionality and descriptors — REQUIRED

Use **TinyUSB's current `audio_test` example** as the functional starting point. Verify that the example enumerates and records its synthetic ramp before integrating any ADC code.

| Parameter | Required value |
|---|---|
| USB mode | Device, full-speed |
| USB audio class | UAC2 |
| USB functions | **One recording-only audio function** |
| Audio channels | **1 (mono)** |
| Audio stream | Isochronous IN only |
| Formats | Signed PCM, 16 bits/sample, little-endian |
| Sample rates | **48,000 Hz only** |
| USB playback | **Not enumerated** |
| Additional CDC/HID/MSC | **Disabled** |
| Product string | `Watch Timing Microphone (Prototype)` |
| Serial number | Board-unique identifier if supported, otherwise documented stable prototype serial |
| Default host gain | Unity gain; host-adjustable volume may be disabled if supported by descriptors |
| Device identity (VID/PID) | **Bench-only IDs for prototype; production IDs require authorization** |

Use exactly the audio control and streaming interfaces required by UAC2. These are not two separate sound devices; they are standard pieces of one recording device. No speaker terminal, playback endpoint, or headphone interface should appear in the descriptors.

**Prototyping VID/PID:** TinyUSB's example includes an example VID/PID for development. Using them for a private bench test is not a substitute for obtaining appropriately authorized IDs for distributed products. Do not silently ship with the example IDs.

**Host control requests:** Implement or preserve the sample-frequency and clock-validity replies needed for enumeration; advertise 48 kHz as the only rate. Handle open/close streaming alternate settings. If a mute or volume control is advertised, implement it correctly; otherwise remove the control from descriptors rather than returning fake success. Do not silently apply software gain to the ADC data.

**Nominal bandwidth:** 48,000 frames/s × 2 bytes/frame = **96,000 bytes/s**, or nominally **96 bytes per 1 ms USB frame**. The software must cope with rate mismatch between the locally clocked ADC and the USB host. See Section 7.

### Suggested reference defines (verify in pinned TinyUSB version)

```c
#define CFG_TUD_AUDIO                              1
#define CFG_TUD_CDC                                0
#define CFG_TUD_MSC                                0
#define CFG_TUD_HID                                0
#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE           48000
#define CFG_TUD_AUDIO_ENABLE_EP_IN                 1
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX         1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX 2
```

The actual configuration and descriptor macro definitions must come from the pinned TinyUSB checkout. Ensure the isochronous IN endpoint's `wMaxPacketSize`, descriptors, and software buffers permit **occasional 49-sample packets (98 bytes)** if the selected clock synchronization design uses variable packet sizes. Do not hardcode a 96-byte maximum packet in that case.

## 6. ADC acquisition — REQUIRED

Use only hardware-paced ADC conversion with FIFO/DMA. The CPU must not determine sample timing by calling `adc_read()` at intervals in the main loop.

### Required setup procedure

1. Initialize the ADC with `adc_init()`.
2. Configure analog GPIO with `adc_gpio_init(ADC_GPIO)`.
3. Select only channel 0 using `adc_select_input(ADC_CHANNEL)`; disable round robin.
4. Read/confirm `clock_get_hz(clk_adc)` equals **48,000,000 Hz** before claiming a 48 kHz sample rate.
5. With 48 MHz ADC clock, call `adc_set_clkdiv(999.0f)` since the period is approximately `(1 + divider)` cycles: 48 MHz / 1000 = **48,000 samples/s**. **Verify actual hardware behavior and exact ADC clock; do not assume this setting if the clock tree is modified.**
6. Configure the ADC FIFO with DMA requests enabled and DREQ threshold=1; **do not byte-shift** because 12-bit data must be preserved.
7. Enable sample error flags in FIFO if supported by the selected SDK/hardware; DMA transfer size should be **16 bits**. An ADC FIFO error flag in bit 15 must be counted and not treated as valid signal.
8. Configure DMA read from `adc_hw->fifo` to memory; read address does not increment, write address increments, DREQ = `DREQ_ADC`.
9. Implement ping-pong/chained DMA buffers (for example **2 buffers × 240 samples** = 5 ms per buffer at 48 kHz). Arrange rearming/ownership so the FIFO cannot overflow while the consumer is processing the other buffer.
10. Start DMA **before** starting continuous ADC conversions with `adc_run(true)`.
11. Stop and drain the FIFO cleanly on streaming shutdown. Reinitialize buffer indices on reopening the USB streaming interface.

### Required DMA data-flow rules

- DMA ISR must do minimal work: mark completed buffer ready, enqueue ownership, increment stats, arm/rearm other buffer, and exit. Do not call potentially blocking TinyUSB write functions in the DMA ISR.
- Never overwrite a buffer still owned by the processing task.
- Each capture buffer carries **sample count** and **monotonic sequence number**.
- A hardware ADC conversion error must increment `adc_error_count`; define a deterministic strategy (e.g., mark the missing sample and insert the previous valid value for transport) and report it in test results. Do not silently shift the entire timeline by dropping conversion-error samples.
- Track `dma_overrun_count`, `fifo_overrun_count` (when detectable), capture-block discontinuities, and total acquired samples.
- Avoid use of `sleep_ms()` or USB log printing in real-time capture functions.

### Sample conversion

For each ADC FIFO word:

```c
uint16_t code = adc_fifo_word & 0x0FFF;        // 0..4095
int32_t centered = (int32_t)code - 2048;       // starting DC midpoint
int32_t pcm = centered * 16;                   // 12-bit signed scale -> 16-bit
// Clamp pcm to [-32768, 32767], cast to int16_t, emit little endian.
```

- Keep original raw **12-bit samples** internally, at least for optional diagnostic comparison.
- Default midpoint `2048` is an initial assumption; the actual DC midpoint can differ with analog bias. Expose `ADC_DC_OFFSET_COUNTS` as a compile-time constant and log raw DC mean in test builds.
- No AGC, compression, limiting, dynamic normalization, digital low-pass, or digital high-pass processing in the baseline capture. Those operations can change tick transient shape.
- Do not truncate samples to 8 bits and then pad to 16 bits; this test is expressly about the RP2350's actual 12-bit conversion quality and ~9.2 ENOB.
- Convert to `int16_t` with saturation and no undefined integer overflow. All audio samples are little-endian on USB.

## 7. Buffering and sample-clock/USB synchronization — CRITICAL

ADC conversion is paced by the local 12 MHz crystal-derived clock. USB frame events are paced by the host. Even when both nominally equal 48 kHz, **their actual timebases will differ by ppm-scale amounts**, accumulating samples in or draining samples from a finite buffer.

### Architecture

```
ADC hardware -> FIFO -> DMA ping/pong -> RAW12 conversion -> PCM16 ring -> TinyUSB IN endpoint -> host
                                        (producer)        (consumer)
```

- PCM ring buffer initial capacity: **8,192 samples** (~171 ms). Single writer on capture-processing side and single reader on USB side; protect indices with an appropriate synchronization policy.
- Initial prefill target: **480 samples (~10 ms)** before sending normal live packets. On stream open, explicitly handle prefill without unbounded blocking (e.g., output short initial silence or defer as permitted by the USB stream protocol).
- No task may busy-wait on a full or empty ring. Track every underrun, overrun, and discontinuity.
- Default packet scheduling should use native captured samples; **do not insert or delete samples as the routine rate-control mechanism** if the USB transport can accept variable packet counts.
- Design a **variable 47/48/49 sample per 1 ms frame mechanism** (94, 96, 98 bytes) with a buffer-occupancy controller or producer-time accounting; 48 is the nominal packet size. This permits long-term correction of ppm-scale differences without changing the captured signal's underlying spacing. Verify that TinyUSB and the selected host accept the configured maximum packet size and varying payload length.
- Do not implement USB OUT audio or feedback endpoints for a recording-only IN device unless the UAC2 specification and selected synchronization topology actually require them. Document the endpoint synchronization type.
- If the selected TinyUSB port cannot support needed variable-sized IN payloads, implement a **clearly labeled interim 48-sample fixed-packet test mode**, instrument buffer drift, and report the limitation. Do not claim long-term lossless capture in that mode.
- On USB suspend/disconnect/stream-close, stop or idle acquisition safely, clear/flush stale queue data, and restart cleanly when streaming resumes.

**Important distinction:** A stable local sample rate is valuable for watch timing, even though the USB transfer rate may have small packet-to-packet variations. Host applications should receive real, sequential ADC samples with no hidden resampling, dropped blocks, or duplicate frames.

## 8. Firmware application states

| State | Entry | Action |
|---|---|---|
| `USB_UNCONFIGURED` | boot, disconnect | Initialize clocks and peripherals; no audio transfer |
| `USB_CONFIGURED_IDLE` | host enumerated, stream closed | Answer controls; ADC stopped or idle, buffer empty |
| `STREAM_PREFILL` | recording stream opened | Start DMA/ADC, fill initial samples |
| `STREAM_ACTIVE` | buffer is ready | Deliver mono audio to USB; maintain queue and counters |
| `USB_SUSPENDED` | suspend callback | Stop sampling or enter low-power state as appropriate; stop streaming |
| `STREAM_ERROR` | persistent DMA/USB error | Increment diagnostic counter, attempt clean restart; never lock device indefinitely |

The USB device should resume capture after an application closes and reopens its recording stream **without a power cycle**. Device disconnection and reconnection should also be recoverable.

## 9. Build-time test modes

Build both modes without modifying the code by hand:

| Mode | Build flag | Source | Purpose |
|---|---|---|---|
| `SYNTHETIC` | `AUDIO_SOURCE=SYNTHETIC` | Generated 997 Hz sine or monotonic ramp | Validate USB enumeration, data order, stream reliability |
| `ADC` | `AUDIO_SOURCE=ADC` | RP2350 ADC0 using DMA | Validate analog capture, noise floor, watch ticks |

- Default normal build: `AUDIO_SOURCE=ADC` after USB synthetic stage has passed.
- Synthetic samples must be generated at 48 kHz using a sample-count phase accumulator, not `sleep_ms()` timing.
- Use deterministic sample amplitudes; a 997 Hz sine at -12 dBFS is suitable for host spectral analysis.
- The USB descriptors must be **identical** in both modes, apart from optional version/build strings that do not add functions or endpoints.
- Provide debug statistics over SWD/RTT/UART only (or a firmware debug counter accessible in a debugger). **Do not add a USB diagnostic interface.**

## 10. Diagnostics and instrumentation

At minimum, track and expose in a debug build:

```text
usb_enumeration_count
usb_audio_stream_open_count
usb_audio_stream_close_count
usb_suspend_count
adc_total_sample_count
adc_conversion_error_count
dma_complete_count
dma_overrun_count
adc_fifo_overrun_or_discontinuity_count
pcm_ring_underrun_count
pcm_ring_overrun_count
usb_in_packet_count
usb_in_sample_count
usb_short_write_count
ring_min_occupancy
ring_max_occupancy
adc_min_code
adc_max_code
adc_mean_code_over_interval
```

- The debug mechanism must not modify the USB descriptor interfaces.
- Debug instrumentation must not block a DMA interrupt, USB SOF handler, or audio endpoint service.
- Provide a diagnostic report on demand in the debugger or via optional UART, and document capture/test timing.

## 11. USB and analog test plan

### Stage A — USB stack only

1. Build and flash `SYNTHETIC` mode.
2. Confirm host enumerates the device under its product name as **one microphone recording device** and **no playback device**.
3. Confirm format: mono, signed 16-bit, 48 kHz. Host native class driver only; no special application-specific driver.
4. Record 60 seconds of 997 Hz synthetic sine to uncompressed WAV; check channel count, sample rate, expected frequency, stable amplitude, and absence of periodic zeros or discontinuities.
5. Record continuously for **30 minutes** and inspect counters and buffer trend. For an initial pass, zero underruns, overruns, and DMA errors is the goal.
6. Stop/start capture 10 times; unplug/replug USB 5 times; ensure the device recovers without reboot loops.

### Stage B — ADC frontend safety and sampling

1. With **preamp disconnected**, verify ADC pin DC bias using a DC-coupled oscilloscope: near 1.65 V and always within 0–ADC_AVDD.
2. With TL071 stand powered from separate external 8 or 9 V supply, confirm common ground before connecting SIG.
3. Measure conditioned ADC pin under representative watch ticks. Target nonclipping peaks (initially expected roughly 1.0–2.3 V at ADC pin); photograph or log the scope measurement.
4. Flash `ADC` mode; confirm 48 kHz ADC capture via `clk_adc` measurement and DMA counts. Compare total samples acquired versus elapsed time.
5. Record **30 seconds of baseline noise** with no tick stimulus and **60 seconds of normal watch ticks**.
6. Record a 1 kHz reference sine at known voltage (where signal-generator equipment is available) to measure effective waveform distortion and check the sample conversion/scaling.
7. Repeat with USB-powered supply and external bench supply separately to compare rail noise and ground coupling.

### Stage C — is the internal ADC good enough?

Record baseline and watch-tick samples without operating-system microphone enhancements. For each capture, report:

- `mean`, `standard deviation`, `RMS`, `min/max`, and `peak-to-peak` of raw converted samples.
- Noise RMS during tick-free windows (in ADC codes and dBFS).
- Tick peak amplitude relative to adjacent baseline noise in dB (define and document window selection).
- Percentage of clipped/saturated samples and any ADC FIFO conversion errors.
- Representative waveforms: first 10–50 ms around each detected watch tick, plus a few seconds overview.
- Spectrum of the quiet baseline, checking specifically for 50/60 Hz mains pickup and USB-related 1 kHz components.
- Detection performance: success rate and false-positive rate with the user's target watch-timing application, at several watch positions and microphone coupling strengths.
- If a reference recording interface is available, simultaneously record the same stand output with the reference and compare detection rate, noise floor, and detected inter-tick times.

**Prototype decision criterion (to be confirmed experimentally):** If watch ticks have a robust peak-to-noise margin (initial goal >=20 dB), show no ADC clipping, and the target timegrapher application detects ticks reliably, the integrated ADC may be sufficient. A ~9.2 ENOB spec alone is not an automatic failure; mechanical pickup SNR and detection consistency are the deciding factors. The 20 dB figure is an exploratory test threshold, not a guaranteed product specification.

### Host recording guidance

On Linux, after discovering the real ALSA device name:

```bash
arecord -l
arecord -D hw:<CARD>,<DEVICE> -c 1 -f S16_LE -r 48000 -d 60 watch_test.wav
```

On Windows, record to **uncompressed 16-bit mono 48 kHz WAV** using the default USB Audio Class 2 driver where supported and disable audio enhancements, noise suppression, and automatic microphone level control when possible. Similar advice applies to macOS Core Audio. Host recording software may resample or alter gain; inspect host input settings and use a bit-transparent capture path if possible.

### Acceptance checklist

- [ ] `SYNTHETIC` UF2 builds and enumerates with exactly one recording function and no playback.
- [ ] `ADC` UF2 builds and uses DMA (not ADC polling).
- [ ] Host captures signed mono PCM16 at 48 kHz.
- [ ] No forced software gain, noise reduction, sample interpolation, or hidden resampling.
- [ ] ADC input safety confirmed before connecting the external TL071 output.
- [ ] USB buffer drift is handled and measured; no unreported sample discontinuities.
- [ ] 30-minute continuous capture tested (provide actual results and environment).
- [ ] Actual quiet noise floor and representative watch-tick captures archived.
- [ ] Repeated stream open/close and USB reconnect tested.
- [ ] Limitations documented honestly; comparison with external I²S ADC is straightforward.

## 12. Risks and engineering notes

1. **Input voltage and DC offset:** The scope capture is AC-coupled. Proper input conditioning is essential; **never** attach raw TL071 output to the ADC pin directly.
2. **ADC accuracy:** RP2350 ADC resolves 12 bits but is specified at ~9.2 effective bits; USB PCM16 formatting cannot restore missing ADC resolution.
3. **Input reference:** ADC_AVDD rail noise directly affects conversion. Keep the analog reference clean and inspect USB-related interference.
4. **Hardware revision:** Raspberry Pi has published RP2350 silicon errata and a later A4 stepping. Log the prototype stepping and check applicable errata before investigating unexplained GPIO/ADC behavior.
5. **USB synchronization:** A crystal accurate to ±30 ppm is adequate for evaluating audio quality but not automatically absolute timegrapher accuracy better than 1 second/day. No calibration is required for phase 1; capture rate measurements for future comparison.
6. **Host effects:** Windows/macOS/Linux audio drivers or third-party apps can apply gain/filters without device intervention. Disable them during controlled comparisons.
7. **USB power and suspend:** For the first firmware experiment, the TL071 may use an external supply. Production USB 9 V module switching, USB suspend power behavior, and total USB-C compliance remain separate design tasks.
8. **Analog anti-aliasing:** One RC low-pass near the ADC is an adequate prototype starting point; assess aliasing if high-frequency mechanical harmonics or interference influence the timegrapher.
9. **Future PCM1809:** Keep the USB streamer and PCM conversion independent of the source; later add a `PCM1809_I2S` source with PIO+DMA while leaving descriptors unchanged.

## 13. Expected deliverables from the code-generation agent

The agent must deliver:

1. A complete CMake project with all implementation source and header files.
2. Two compiled RP2350 UF2 firmware files when a build toolchain is available: `watchtiming_synthetic.uf2` and `watchtiming_adc.uf2`.
3. Explicit commands for building, flashing, connecting, capturing audio, and obtaining diagnostics.
4. Host-side analysis tool/script that loads WAV and produces baseline-noise, clipping, level, and transient statistics (and preferably a plot).
5. Automated unit tests for ADC conversion, ring-buffer behavior, sample-order preservation, and saturation.
6. A report of **actual** build, test, and hardware results, with all untested items clearly marked. Do not claim successful enumeration, ADC capture, or audio quality without observing real hardware.
7. README describing any required analog conditioning and a warning not to connect raw SIG directly to the ADC.
8. A short `PORTING_TO_PCM1809.md` explaining how to replace `adc_capture.c` with a PCM1809 I²S sample producer in a future version.

**Agent completion condition:** Do not mark the task complete solely because USB descriptors were generated or placeholder PCM was streamed. Completion requires a compiling firmware with both source modes and a documented test procedure. Hardware success can remain unverified if no board is available, but must be identified as such.

## 14. Manufacturer and software references

- Raspberry Pi, Pico C/C++ SDK hardware ADC API: https://www.raspberrypi.com/documentation/pico-sdk/hardware.html
- Raspberry Pi, RP2350 datasheet and hardware reference: https://pip.raspberrypi.com/categories/1214-rp2350
- Raspberry Pi, RP2350 ADC header implementation: https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_adc/include/hardware/adc.h
- TinyUSB, UAC2 test microphone example: https://docs.tinyusb.org/en/stable/examples/device/audio_test.html
- TinyUSB, USB Audio device reference: https://docs.tinyusb.org/en/latest/reference/class/audio.html
- TinyUSB source example: https://github.com/hathach/tinyusb/tree/master/examples/device/audio_test
- Raspberry Pi Pico SDK overview: https://www.raspberrypi.com/documentation/pico-sdk/
- Abracon crystal ABM8-272-T3, LCSC C20625731: https://www.lcsc.com/datasheet/C20625731.pdf

---

### Engineering change log

- **P1.0 (2026-11-03):** First code-generation specification for USB mono microphone with internal RP2350 ADC. Intentionally excludes external PCM1809 hardware and 9 V converter design. Retains optional unpopulated attenuation footprints, eliminates trimmer, specifies DMA, buffering, USB synchronization, and objective noise evaluation.
