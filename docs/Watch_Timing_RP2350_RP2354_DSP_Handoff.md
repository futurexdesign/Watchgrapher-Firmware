# RP2350 / RP2354 DSP Evaluation and Implementation Handoff

**Project:** Timegrapher Stand / Kiwi USB watch-timing microphone  
**Purpose:** Consolidate processor DSP capabilities, candidate audio-processing features, architecture, and implementation priorities for a follow-on engineering/code agent.  
**Status:** Engineering assessment and proposed DSP roadmap; **not a claim that the DSP features below have been implemented or benchmarked**.  
**Scope:** RP2350 and RP2354 **only**. Exclude RP2040. External audio is acquired using PCM1808 (bench) or PCM1809 (candidate production) over I²S, then presented as 48 kHz mono USB audio.

## 1. Executive conclusion

**The RP2350/RP2354 has sufficient compute and SRAM for substantial real-time DSP on a single 48 kHz audio channel.** Adjustable digital gain, high-pass/low-pass/band-pass and notch filtering, running noise estimates, transient-aware gain, clipping management, watch-tick detection, correlation, moderate FIR processing, and FFT-based noise analysis are all technically feasible. No separate DSP chip is justified for this workload without first benchmarking a concrete implementation.

The more significant product risks are **analog overload**, preserving the timing and weaker components of watch-tick events, and maintaining the already-working USB stream behavior. Digital DSP cannot repair a signal clipped inside the ADC or protect its analog input from overvoltage.

**Recommended strategy:** Preserve a fully bypassable, near-raw recording mode; add processing in small, measurable stages; measure CPU usage and clock-domain/buffer integrity; use core 1 only if core 0 processing interferes with USB deadlines.

## 2. Chip DSP capabilities and available budget

| Capability | RP2350 / RP2354 assessment |
|---|---|
| Processor | Dual Arm Cortex-M33 cores, commonly clocked at **150 MHz each**, sharing `clk_sys` |
| Per-core arithmetic | Arm DSP extension (including multiply-accumulate and packed integer DSP instructions) and **hardware single-precision floating-point unit** |
| On-chip SRAM | **520 KB** across the processor family |
| Audio serial interface | I²S through **PIO + DMA**; do not describe this as a dedicated built-in I²S peripheral |
| Software options | Plain C++/C, Pico SDK hardware APIs embedded in Arduino-Pico, and optionally **Arm CMSIS-DSP** |
| RP2354 distinction | RP2354 packages include integrated flash; **no meaningful difference in CPU DSP throughput** versus comparable RP2350 silicon |
| Core choice | Use Arm Cortex-M33 for hardware FPU and Arm DSP support; do not assume equivalent toolchain/DSP characteristics for the optional Hazard3 RISC-V mode |

At a **48,000-sample/s** rate:

- One 150 MHz core has **3,125 nominal clock cycles per mono sample**.
- One **240-sample / 5 ms block** represents **750,000 nominal cycles per core**.
- Both cores together have a theoretical aggregate throughput of **300 million cycles/s**, but this is **not** an available-DSP-cycle benchmark: USB, PIO/DMA service, interrupts, memory stalls, and application tasks consume time, and the cores share memory resources.
- The raw mono PCM16 USB data rate is only **96,000 bytes/s**.

**Initial performance target:** Aim for complete DSP processing of a 240-sample block in **less than 1 ms (150,000 cycles at 150 MHz)**; ideally substantially less when running on the same core as USB. This 20%-of-a-block budget is a *proposed acceptance target*, not measured utilization. Because the current USB feed runs every 1 ms, avoid a single long blocking DSP call in the main loop; process smaller chunks or use core 1 if needed.

### Illustrative operation counts — not cycle benchmarks

| Algorithm | Approximate operations at 48 kHz mono |
|---|---:|
| Per-sample scalar gain | 48,000 multiplies/s |
| Four biquad stages | ~960,000 coefficient multiplies/s (5/stage/sample) |
| 32-tap FIR | 1.54 million MACs/s |
| 64-tap FIR | 3.07 million MACs/s |
| 128-tap FIR | 6.14 million MACs/s |
| 256-tap FIR | 12.29 million MACs/s |
| 1024-point FFT with 50% overlap | ~94 transforms/s |

These indicate that typical filtering is inexpensive relative to the available CPU capacity; actual wall time depends on implementation, RAM/cache behavior, conversion, and compiler options. A 1024-sample window at 48 kHz spans ~21.3 ms; 50% overlap yields a new analysis window every ~10.7 ms and can add buffering latency. Do not infer event-timing precision from FFT display resolution.

## 3. Existing as-built firmware: preserve this, do not rewrite

The user's **`AS_BUILT_ARCHITECTURE.md`** describes the original firmware as built. The DSP agent must inspect the current repository as well, because its PCM1808 integration may have progressed beyond that document.

| Area | As-built behavior to retain |
|---|---|
| Project/toolchain | `timegrapher-stand`, **PlatformIO + Arduino-Pico (Earle Philhower)**, C++ `setup()` / `loop()` |
| USB library | **Adafruit TinyUSB**, `-DUSE_TINYUSB`, `lib_archive = no` |
| Host-visible USB | Composite **CDC + one UAC2 microphone**, capture-only, mono **48 kHz signed PCM16 LE**; **no audio playback** |
| USB feed | `usb_audio_task()` gives TinyUSB **48 PCM samples once per millisecond**; `CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL=1` handles on-wire 47/48/49-sample packets |
| Producer/ring | `audio_source_process()` writes into `audio_ring`: **8192 signed 16-bit samples**, 480-sample prefill |
| Underruns | Existing hold-last-sample USB behavior, with counters |
| Processing location | Application loop: `TinyUSBDevice.task()` -> `audio_source_process()` -> `usb_audio_task()` -> diagnostics -> LED |
| Sources in original architecture | Built-in ADC and looping synthetic `tick-tock.wav`; PCM1808 is an added bench path. PCM1809 is prospective |
| Configuration | `platformio.ini` has effective `-DCFG_TUD_*` definitions; `include/tusb_config.h` is a mirror, not authoritative |
| Boards | Pico 2 (RP2350A), Kiwi (RP2350B); Kiwi code uses board-specific GPIO/flash settings. Verify current target for RP2354. |
| Diagnostics | Existing optional `Serial1` diagnostic UART; do not add new USB functions solely for DSP diagnostics |

**Non-regression rule:** DSP is a producer-side transformation before the current PCM16 ring. It must **not** rewrite USB descriptors, alter the 48-samples/ms feed, introduce a second USB 47/48/49 packet controller, replace TinyUSB, or silently remove CDC. Existing capture modes, diagnostics, synthetic test signal, open/close/suspend state machine, and USB timing must continue working.

Current system diagram:

```text
PCM1808 (24-bit stereo) / PCM1809 (serial ADC output)
       -> RP2350 PIO + DMA -> full-resolution acquired frames
       -> [NEW OPTIONAL DSP: filters / gain / level / limiter]
       -> saturation + quantization -> mono int16 PCM
       -> EXISTING audio_ring (8192 samples, 480 prefill)
       -> EXISTING usb_audio_task (48 samples every 1 ms)
       -> EXISTING Adafruit TinyUSB UAC2 IN + CDC composite
       -> computer watch-timing application
```

The **current** pipeline may quantize to PCM16 before applying `PCM1808_PCM_GAIN=4` (~+12.04 dB). The DSP design should instead retain the ADC's full word precision **until after DSP and gain**, then convert to PCM16 **once**. Verify the current source before changing it.

## 4. Data representation and safe gain

- **PCM1808:** 24-bit two's-complement samples in I²S slots. Capture correct left/right alignment, extract **left only**, and sign-extend correctly before processing. The right ADC channel is not a second USB channel.
- **PCM1809:** Keep the converter's serial word format/source-specific alignment separate from DSP. A nominal 32-bit sample word does **not** mean 32 noise-free bits.
- **Internal DSP sample format:** Recommend `float32_t` for the first implementation. The M33 has hardware float support. A signed Q31 fixed-point path is an alternative; use wide accumulators and explicit saturation.
- A 24-bit integer sample is exactly representable in a standard 32-bit float's integer precision for its relevant range. Float32 does **not** preserve every low bit of an arbitrary signed 32-bit integer, which is unlikely to matter at these ADCs' real noise floors; validate if applying specialized high-dynamic-range algorithms.
- Apply **digital gain before final 16-bit conversion** to avoid losing small sample bits prematurely. Example: +12 dB is approximately ×4 linear amplitude. Always use saturation/limiting *before* casting to `int16_t` to avoid signed overflow/wrap.
- Gain improves recorded level/quantization utilization, **not the SNR of the analog input or ADC**.
- Record both **raw/pre-DSP peak** and **post-DSP peak** levels, plus clipping counts. Do not represent a digitally clipped waveform as recovered information.
- Consider optional quantization dither only after core functionality and measurements justify it; it is not a phase-1 requirement.

## 5. Recommended operations, ordered by value

These are **candidate features**, not confirmed implemented functions. Processing must be bypassable and parameter changes must not silently distort tick timing.

| Priority | Feature | Complexity | Why it is useful | Main caveat |
|---|---|---|---|---|
| P0 | Raw/bypass + diagnostic levels | Very low | Reference for every comparison | Must preserve real capture, not fake gain |
| P1 | User/configurable digital gain | Very low | Standardizes recorded level | Preexisting clipping/noise remains |
| P1 | 16-bit saturating output limiter | Very low | Prevents PCM16 wraparound | Cannot recover analog ADC clipping |
| P1 | Configurable high-pass/low-pass/band-pass IIR | Very low | Reduces rumble and out-of-band noise | Filtering can move/reshape tick onsets |
| P1 | Rolling noise-floor estimator | Very low | Detects changing ambient noise; thresholding | Exclude actual ticks from noise estimate |
| P2 | 50/60 Hz notch (optional) | Very low | Rejects mains pickup when present | Avoid unnecessary narrowband phase effects |
| P2 | Slow adaptive gain | Low | Helps different watch positions/microphone coupling | Never track/envelope-compress each tick |
| P2 | Transient-aware preservation/denoising | Low–moderate | Retains quiet unlocking/impulse/drop structure | Standard voice noise gates can erase important events |
| P2 | Onset detector and short-window metrics | Low–moderate | Measures event timing, signal-to-noise margin | Avoid false locks on strongest impulse alone |
| P3 | Matched filtering / correlation | Low–moderate | Detects repeated tick patterns in noise | Template may vary by watch/mount/position |
| P3 | 64–256-tap FIR / alternative phase responses | Moderate | Frequency response with more predictable phase | Compute cost, buffering and delay |
| P3 | 512/1024-point FFT noise analysis | Moderate | Identifies hum, tonal interference, converter switching bands | Window latency and spectral leakage |
| P4 | STFT spectral denoising | Moderate–high | Addresses persistent broadband background noise | Musical noise, transient smearing, added latency |
| Defer | Neural-network speech-style denoising | High/uncertain | Low priority for impulse/timing signal | Likely counterproductive; higher validation burden |

### 5.1 Initial filter presets (experiments, not normative truth)

A sensible first set of passbands, inspired by watch-timing software practices:

- **WIDE:** **160 Hz–16 kHz** — recommended default *processed* mode; preserves broad transient content.
- **MEDIUM:** **400 Hz–12 kHz** — more environmental noise rejection.
- **NARROW:** **800 Hz–8 kHz** — use only if measured tick detection is better.
- **BYPASS:** unfiltered audio (except ADC's unavoidable own filtering), used for validation and compatibility.

Implement with cascaded second-order IIR (biquad) sections, ideally with selectable 2nd/4th-order slopes. Do **not** reflexively implement a 24 dB/octave slope without testing its timing consequences. At 48 kHz, Nyquist is 24 kHz, so 16 kHz is a meaningful but relatively high low-pass cutoff.

**Timing caution:** IIR filters can introduce frequency-dependent group delay and shift an event's detected threshold crossing. Test **absolute onset offset**, **inter-tick interval error**, and **relative spacing of subevents** using raw-versus-processed paired recordings. Constant delay is generally easier to compensate than amplitude-dependent or frequency-dependent onset shifts.

### 5.2 Noise-floor tracking

Watch-tick events occupy a small portion of the stream, permitting background-noise estimates during quiet windows:

1. Compute running RMS/energy and optionally short-term spectrum in fixed windows.
2. Flag candidate transient intervals using an adaptive threshold and time hysteresis.
3. Update the background noise estimate mostly **outside** those transient intervals.
4. Report noise floor, peak-to-noise margin and event confidence as diagnostics.
5. Prefer **measurement and optional mild suppression** first, not aggressive muting.

For gated/suppressed modes, preserve audio before and after the primary impulse (pre-roll and tail) so small unlocking/impulse/drop events are not removed. Abrupt hard gating can introduce artificial edges and false watch ticks; fade if gating is eventually necessary. An estimator should not be built on an assumed exact tick repetition frequency—watches and acoustic pickups vary.

### 5.3 Gain and peak control

- **Manual gain:** initial profiles 0, +6, +12 dB (or user-adjustable via configuration) with saturating PCM16 output. Keep any existing `PCM1808_PCM_GAIN=4` support compatible while changing the implementation to apply it before quantization when appropriate.
- **Slow AGC:** estimate level across many tick events and adjust at most on a **hundreds-of-milliseconds to seconds** scale, not on the rise/fall of an individual tick. Prefer stable gain across the full tick event, including its weak components.
- **Limiter:** protect the exported PCM16 format with saturation or a controllable soft knee. Record clipping/limiter activity and preserve a raw mode. A digital limiter **cannot prevent PCM1808/PCM1809 silicon input overload**.
- **Timegrapher timing:** Gain changes, nonlinear limiting, and noise gates can change the shape of the initial transient. The success criterion is **timing accuracy and secondary-event retention**, not subjective audio loudness.

### 5.4 Watch-specific algorithms

The valuable acoustic information is not necessarily the largest peak. An escapement event can contain smaller phases associated with unlocking, impulse and drop. Candidate measurements:

- Tick onset time and confidence with noise-adaptive thresholding.
- Peak and integrated energy in a defined window around the event.
- Spacing and relative strengths of major/subsequent peaks.
- Cross-correlation between consecutive events or between an event and learned templates.
- Event statistics by alternating tick/tock phase, where meaningful.
- Optional narrowband interference rejection **only** for spectral lines observed in the user's environment.

Do not hard-code timing values or claim reliable watch-rate/beat-error determination without validation against a calibrated reference and watch-timing software.

## 6. Suggested deployment architecture

### Stage A — Single-core, minimal risk (start here)

Within the current `audio_source_process()` processing stage, before committing PCM16 samples to `audio_ring`:

```text
DMA buffer (240 frames @ 48 kHz, 5 ms)
 -> source-specific unpack to signed samples
 -> [BYPASS / DSP processing in small cooperative chunks]
 -> saturate/quantize to mono PCM16
 -> existing ring
 -> existing USB feed
```

Do not call TinyUSB or log Serial from DMA interrupt context. Processing 240 frames once per 5 ms may still block 1 ms USB servicing if executed in one long operation, so **instrument latency and consider splitting into ~48-sample sub-blocks** while keeping filter state continuous. The DMA producer, CPU DSP stage, and USB consumer need explicit ownership and overrun diagnostics.

### Stage B — Optional core 1 DSP worker (only if measured need)

Use Arduino-Pico-supported `setup1()` / `loop1()` or a verified framework-compatible multicore arrangement:

- **Core 0:** existing TinyUSB polling, stream FSM, diagnostics, USB output and ring consumer.
- **Core 1:** completed DMA buffer processing, DSP algorithms, and PCM ring producer.
- Exchange buffers via clearly owned SPSC queues/atomics, memory barriers, or suitable Pico SDK synchronization. **Never share a mutable DMA block concurrently** without an ownership protocol.
- Do not move TinyUSB service across cores casually. Preserve Arduino/TinyUSB affinity and assess interaction with existing ISR handlers.
- Both cores remain **150 MHz shared**; there is no independent per-core clock-domain setting.

The second core is **optional headroom**. Introducing it before profiling risks making a stable stream less deterministic.

## 7. SRAM and implementation constraints

Approximate reference storage costs:

| Item | RAM cost |
|---|---:|
| Existing 8192-sample mono PCM16 ring | ~16 KB |
| Two 240-frame stereo I²S buffers (2 × 240 × 2 × 4 bytes) | ~3.75 KB |
| One 240-sample float32 DSP working buffer | ~0.94 KB |
| Two float32 working blocks | ~1.88 KB |
| 1024 complex float32 FFT working buffer | ~8 KB (plus tables/windows / scratch depending on API) |

These are approximate **data arrays**, not full linked/stack memory use. They fit comfortably within 520 KB in isolation. The actual build must inspect the memory map and core stacks; FFT and CMSIS-DSP implementations may need extra temporary storage.

**CMSIS-DSP:** Useful for optimized `arm_biquad_cascade_df1_f32`, FIR, FFT, RMS/statistics, etc., but not a mandatory dependency for phase 1. An ordinary few-biquad native C++ float32 implementation may be easier to integrate. If using CMSIS-DSP, confirm **Cortex-M33 FPU instructions, floating-point ABI, compile flags, library version and linker configuration** match the PlatformIO/Arduino-Pico toolchain. Do not drop a precompiled incompatible archive into the project.

## 8. Analog limitations and input-protection boundary

The system has previously shown **large, very brief watch-tick peaks** (up to ~7.32 Vpp reported at the PCM1808 module connector in one scope capture) while the converter nominal full-scale range is around **3 Vpp**. Those peaks are not necessarily the voltage directly at the silicon ADC pin because the breakout may contain coupling/filter components.

- **DSP gain, noise reduction and limiting cannot restore data clipped before conversion** or protect the IC's absolute-maximum input rating.
- An analog transient-limiting or clamp experiment is a **separate hardware problem**. Verify the actual PCM1808/PCM1809 silicon pin bias and input electrical limits; do not connect ground-referenced limiter diodes across an internally biased analog input without the required isolation/coupling.
- The bench input has used **0.1 µF AC coupling and 2.2 kΩ series resistance**; a 1 µF film capacitor has been considered. Treat analog changes as separate A/B tests rather than combining them with DSP algorithm changes.
- A clean 1 kHz sine-wave capture was subsequently achieved after fixing PCM1808 stream-buffer/synchronization issues. **Do not reopen those resolved issues without evidence.** Preserve the corrected USB/I²S path.

A useful combined solution may be **minimal analog peak protection** followed by digital filtering and controlled gain, but evaluate it with raw recordings and a true signal-to-noise metric, not perceived volume alone.

## 9. DSP test modes and acceptance criteria

### Required modes

1. **BYPASS / RAW REFERENCE:** no new software EQ, gain or gating; retains native ADC path and existing known conversion semantics (except necessary 24/32-to-16 scaling). This is the reference against which every DSP result must be judged.
2. **GAIN_ONLY:** configurable linear gain with safe saturation.
3. **WIDE_FILTER:** wide-band IIR filter with stable gain.
4. **FILTER_PLUS_LEVEL:** wide filter, noise-floor estimator and optional slow gain; no aggressive suppression by default.
5. **EXPERIMENTAL:** notch, transient-aware gating, matched filter or FFT analysis, initially behind feature flags.

Prefer compile-time selection or configuration compiled into firmware first; runtime controls can follow, without changing the established USB descriptors. Existing watch-timing software must continue to see the same one-channel UAC2 input.

### Measurement approach

- Capture the **same raw audio** and process it both offline and on device where practical; if comparing sequential recordings, note changes in watch position, coupling force and ambient noise.
- Retain actual uncompressed mono WAV captures. Disable host-side automatic enhancements and normalize only when explicitly indicated.
- Use a known sine test (e.g., **1 kHz, ~1.1 Vpp measured at the module header**) to check gain linearity, distortion, saturation and frequency response.
- Use actual watch ticks for transient shape/onset measurements, not only sine waves.
- Include quiet baseline and noisy conditions (USB power supply noise, nearby fans/mains noise, different stand coupling).

### Metrics to log

| Category | Required checks |
|---|---|
| CPU | DSP **min/mean/max cycles** per 240-sample block; worst-case blocking time; core load where measurable |
| Transport | DMA overruns, missed frames, ring min/max/trend, USB underrun/overrun, held-last-sample counts, resyncs |
| Level | Input/pre-DSP peak, output/post-DSP peak, RMS, clipping/limiter activations, effective gain |
| Noise | RMS noise during non-event windows, spectral interference, tick peak-to-noise ratio |
| Watch timing | Onset-time offset vs raw; interval bias/jitter; preservation of weaker secondary event spacing |
| Functional | 60-s capture; **30-minute soak**; repeated start/stop, suspend/resume and reconnection |
| Compatibility | All existing PlatformIO environments and USB interfaces still work; no extra playback endpoint or added USB audio channel |

**Proposed pass criteria (not measured facts):** Typical DSP block processing well below 1 ms; no added USB or DMA loss; no clipping except intentionally limited signal; improved noise/detection metrics **without unacceptable onset shift or deletion of weaker subevents**. Set a numerical timing-error threshold after establishing a raw baseline and the target timegrapher application's tolerance.

**Instrumentation:** A Cortex-M33 cycle counter or a verified RP timer can support benchmarking. Ensure the selected compiler/core enables the profiling source and handles wraparound; the mere availability of a cycle counter is not evidence of measurement results. Never print or allocate memory inside a DMA ISR.

## 10. Recommended implementation milestones

| Phase | Work | Exit gate |
|---|---|---|
| 0 | Freeze current working PCM1808/I²S + USB behavior; add DSP bypass switch, profiles and pre/post metrics | Bit-identical or predictably equivalent bypass captures |
| 1 | Move gain into full-resolution processing before PCM16 conversion; add saturating output, clipping diagnostics | Known sine level, no wraparound, unchanged host format |
| 2 | Implement 2–4 biquad wide/medium/narrow filter profiles (default bypass) | Response measured; tick onset/secondary peaks compared |
| 3 | Add robust running noise-floor estimate and event-window detection diagnostics | Detect useful ticks without raising false positives |
| 4 | Slow adaptive gain and optional transient-aware gate/limiter | Weak events retained, no per-tick gain pumping |
| 5 | Optimize/profile: optional CMSIS-DSP and/or core 1, only if measured deadlines justify | No new underruns, stable long-duration stream |
| 6 | Evaluate FFT/spectral noise reduction / matched filtering for measured interference | Objective improvement in watch-timing metrics |

**Make one processing change at a time** so a regression can be traced to the specific algorithm. Avoid simultaneous modifications to ADC mode pins, MCLK generation, input analog circuitry, I²S DMA, and USB packet scheduling.

## 11. Instructions to the next agent

1. Read the **actual current repository** and `docs/AS_BUILT_ARCHITECTURE.md` before editing; firmware has evolved beyond early design specs. The as-built baseline above is based on the user-supplied architecture summary, not a complete source code audit.
2. Identify the source-specific **full-resolution-to-PCM16 conversion** point. Insert an optional DSP stage **before** the PCM16 ring. Preserve USB source timing, descriptors, and flow control.
3. Implement **bypass and gain/saturation first**, with host-testable pure DSP functions; do not introduce CMSIS-DSP or multicore immediately unless the existing code already uses them.
4. Add explicit diagnostic and benchmarking counters without blocking TinyUSB/DMA.
5. Validate a reference 1 kHz sine and actual watch ticks; report raw vs processed levels, timing shifts, and noise-floor change.
6. Keep algorithm parameters grouped in an easily auditable configuration file and provide documented profile defaults. Do not imply all watches will benefit from one filter profile.
7. Treat analog peak protection as **separate and safety-critical**. No firmware feature can prevent ADC pin overvoltage.
8. State exactly which tasks were **implemented, compiled, host-tested, hardware-tested, or remain unverified**.

## 12. Reference material

These are background reference links and should be checked against the installed framework and actual hardware revision:

- [Raspberry Pi RP2350 datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) — Cortex-M33, clocks, DMA, PIO, SRAM.
- [Raspberry Pi Pico SDK hardware documentation](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html) — clock, PIO, DMA and multicore interfaces.
- [Arduino-Pico multicore documentation](https://arduino-pico.readthedocs.io/en/latest/multicore.html) — `setup1()`/`loop1()`, core behavior, stack concerns.
- [Arm CMSIS-DSP](https://github.com/ARM-software/CMSIS-DSP) — optimized signal-processing primitives and examples.
- [TI PCM1808 datasheet](https://www.ti.com/lit/ds/symlink/pcm1808.pdf) — 24-bit audio ADC characteristics and analog range.
- [TI PCM1809 datasheet](https://www.ti.com/lit/ds/symlink/pcm1809.pdf) — production-candidate digital audio ADC.
- **Local source document:** `docs/AS_BUILT_ARCHITECTURE.md` or the user-supplied `AS_BUILT_ARCHITECTURE.md`.

---

**Bottom line:** The RP2350/RP2354 is easily powerful enough to implement meaningful watch-timing DSP at 48 kHz mono. The first engineering objective is **preserve the raw tick structure and stable USB streaming while measurably improving capture quality**, not maximize audio processing complexity.
