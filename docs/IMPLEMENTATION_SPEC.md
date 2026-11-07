# Timegrapher Stand Firmware — As-Built Baseline + PCM1808 Integration

**Implementation specification P3.0**  
**Date:** 2026-11-04  
**Project:** `timegrapher-stand`  
**Status:** Code-generation / implementation handoff; PCM1808 firmware **not represented as implemented or hardware-validated**  
**Authoritative existing-behavior source:** User-supplied `AS_BUILT_ARCHITECTURE.md`, dated 2026-11-04  
**Target external ADC:** PCM1808, Comimark LY451-style 24-bit stereo ADC bench module

> **Priority rule:** This P3.0 document supersedes P1.0/P2.0/P2.1 *implementation requirements* for this project. The accompanying `AS_BUILT_ARCHITECTURE_SOURCE.md` is authoritative on **what already exists**. Where the as-built document and earlier intentions disagree, preserve the as-built behavior unless a change is explicitly required by P3.0. This specification is based on the supplied architecture document—not a source-code audit of the full repository. Check the actual project APIs, names, and dependencies before editing.

## 1. Goal and acceptance boundary

Extend the **existing** PlatformIO / Arduino-Pico / Adafruit TinyUSB firmware with a third compile-time acquisition backend, `PCM1808`. Retain both currently implemented sources (`ADC`, `SYNTHETIC`) and the host-visible USB audio behavior. The PCM1808 is a **bench validation tool** for comparing the onboard RP2350 ADC against a higher-performance converter; it is not the committed production codec.

When selected, the PCM1808 backend:

1. Outputs **12.288 MHz nominal-average SCKI** from RP2350 **GPIO21 GPOUT0**, derived through a 16-bit fractional divider from existing **150 MHz PLL_SYS**.
2. Keeps the **shared RP2350 `clk_sys` at 150 MHz**; both cores run on that same clock. Does **not** downclock to 61.44 MHz, reprogram PLL_SYS, or disturb the 48 MHz USB clock.
3. Configures the PCM1808 breakout for **I²S master mode, 256 fS, 24-bit I²S** using static straps (`MD1=1`, `MD0=1`, `FMT=0`) present before power-up.
4. Receives **BCK, LRCK and DOUT** through an RP2350 **PIO input state machine and DMA**. The ADC, not the RP2350, generates BCK/LRCK.
5. Converts **left-channel signed 24-bit audio** to mono signed **16-bit PCM** and feeds the existing `audio_ring` through the existing `audio_source` state machine.
6. Exposes the **unchanged composite USB CDC + UAC2 mono microphone** to the host. Preserve the existing USB sample pacing and TinyUSB flow-control strategy.
7. Adds board builds, source-specific diagnostics, hardware tests, and documentation without refactoring working USB code or the internal-ADC/synthetic pathways.

**Explicitly accepted:** deterministic edge jitter produced by fractional GPOUT division. Measure it only enough to confirm valid PCM1808 clocking; do not redesign around an external oscillator or integer PLL scheme for this proof of concept. An accurate *average* divide ratio is not a guarantee of crystal ppm accuracy, jitter-free sampling, or lossless USB clock-domain adaptation.

**Out of scope:** production PCM1809 board, watch-rate estimation algorithms, audio filters/AGC, host playback, runtime codec switching, mandatory CDC removal, unrelated Pico SDK/CMake migration.

## 2. Existing architecture — MUST preserve

### 2.1 Build system and framework

| Area | Actual implemented baseline / requirement |
|---|---|
| Project root | `timegrapher-stand/` |
| Build | **PlatformIO**, `platformio.ini` |
| Platform | `https://github.com/maxgerhardt/platform-raspberrypi.git` |
| Framework | **Arduino**, Earle Philhower core (`board_build.core = earlephilhower`) |
| Language | C++ modules and C-style APIs; Arduino `setup()` / `loop()` |
| USB | **Adafruit TinyUSB** under `-DUSE_TINYUSB`; `lib_archive = no` |
| RTOS | None |
| Pico hardware APIs | Existing core's bundled Pico SDK `hardware_adc`, `hardware_dma`; add `hardware_pio`, `hardware_clocks` as needed |
| TinyUSB flags | Effective `-DCFG_TUD_*` flags live in `platformio.ini` |
| `include/tusb_config.h` | Reference mirror only; do not use as primary build configuration |

**Do not generate a new standalone Pico SDK CMake project** and do not swap TinyUSB stacks merely to match older specifications. Preserve the project libraries, pinning/version choices, and compiler flags unless a specific compatibility fix is demonstrated.

### 2.2 Current PlatformIO environments

| Env | Board ID | Chip package | ADC input | Flash | Mode |
|---|---|---|---|---|---|
| `pico2` | `rpipico2` | RP2350A | GP26 / ADC ch0 | Board default | `ADC` |
| `pico2_synthetic` | `rpipico2` | RP2350A | GP26 | Board default | `SYNTHETIC` |
| `kiwi` | `generic_rp2350` | RP2350B (`__PICO_RP2350A=0`) | GP40 / ADC ch0 | **2 MB** | `ADC` |
| `kiwi_synthetic` | `generic_rp2350` | RP2350B | GP40 | **2 MB** | `SYNTHETIC` |

Add **`pico2_pcm1808`** and **`kiwi_pcm1808`** by inheriting as much as possible from their respective existing environments. Preserve Kiwi's `picoprobe` upload/debug settings, package/flash defines, existing USB compile flags, and LED **GP25**. For new modes `ADC_GPIO` may remain defined but must not trigger ADC capture initialization. **Do not change default environments.**

### 2.3 Source selection

The existing build uses `AUDIO_SOURCE` identifiers `SYNTHETIC = 1`, `ADC = 2` as defined in `include/app_config.h` (exact macro details to be checked in repository). Add `PCM1808 = 3` without renumbering existing choices; set `-DAUDIO_SOURCE=PCM1808` on both new environments.

**Source exclusivity:** Only code and IRQ handlers for the selected acquisition backend must be compiled/activated. Internal ADC DMA and PCM1808 DMA must not run simultaneously. An optional future build to capture both would require a separate design; it is not required here.

### 2.4 Existing files and responsibilities

| Path | Preserve / integrate |
|---|---|
| `src/TimegrapherStand_main.cpp` | Retain Arduino `setup()/loop()`, TinyUSB task ordering, LED behavior |
| `src/usb_audio.cpp` | Preserve UAC callbacks, **48 PCM samples per millisecond** application feed and TinyUSB flow control |
| `src/usb_descriptors.cpp` | Preserve `Adafruit_USBD_Interface` microphone descriptor injection and USB composite configuration |
| `src/audio_source.cpp` | Extend FSM compile-time source dispatch and PCM producer path, no rewrite |
| `src/audio_ring.cpp` | Reuse PCM16 ring as-is; no new ring required |
| `src/adc_capture.cpp` | Keep existing onboard ADC, FIFO + ping-pong DMA behavior unchanged |
| `src/diagnostics.cpp` | Extend counters/output without breaking existing fields |
| `include/app_config.h` | Add PCM1808 source constant and source-specific settings |
| `include/board_config.h` | Put new pin mapping and board checks here; keep ADC and LED pins |
| `include/tick_tock_pcm.h` | Preserve exact loop of embedded `tick-tock.wav` |
| `tests/host_analyze.py` | Extend optional comparison analysis; preserve existing CLI behavior |

### 2.5 Runtime contract

Current main loop sequence stays:

1. `TinyUSBDevice.task()`
2. `audio_source_process()`
3. `usb_audio_task()`
4. `diagnostics_poll()`
5. Status LED update

Current `setup()` order stays logically **diagnostics → audio_source → `usb_audio_init()`**, with any PCM1808-only clock initialization performed within/under `audio_source_init()` before PCM1808 capture. A non-blocking start-up wait may be needed because TinyUSB must continue to be serviced.

Retain FSM states: `USB_UNCONFIGURED`, `USB_CONFIGURED_IDLE`, `STREAM_PREFILL`, `STREAM_ACTIVE`, `USB_SUSPENDED`, and `STREAM_ERROR`. On stream open, enter prefill; on close/unmount/suspend, stop acquisition and reset the ring. Add ADC readiness/fault behavior within this FSM without introducing a separate competing USB state machine.

### 2.6 Host-visible USB — critical non-regression

**As implemented**, the device is a **composite USB CDC + UAC2 microphone**. Earlier specs calling for no CDC are **superseded**: Adafruit's Arduino TinyUSB integration requires at least one CDC instance in this codebase. Do not remove CDC as part of PCM1808 work.

| Property | Keep exactly |
|---|---|
| VID / PID | Bench `0xCafe` / `0x4003` (development only; obtain allocated IDs for distributed product) |
| Manufacturer | `TimegrapherStand` |
| Product | `Watch Timing Microphone (Prototype)` |
| Audio interface string | `UAC2` |
| Function | One UAC2 **microphone**, mono 48,000 Hz, PCM S16LE; **no playback** |
| USB composite | **CDC + UAC2**; CDC is not an additional audio device |
| Descriptor macro | Existing `TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR` |
| Isochronous EP IN packet ceiling | **98 bytes** (up to 49 mono samples), per existing configuration |
| UAC Feature Unit | Preserve mute; volume range **0…0 dB**, not applied to PCM |
| Stream open | `tud_audio_set_itf_cb`: alternate-setting in **low byte of `wValue`** |
| Stream close | `tud_audio_set_itf_close_ep_cb` |
| USB app pacing | **Write 48 mono samples exactly once per millisecond** using existing `usb_audio_feed_frame` |
| On-wire packet adjustment | **`CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL=1`** lets TinyUSB select 47/48/49 samples |
| Missed loop iterations | Existing catch-up ceiling of 5 one-millisecond feed periods |
| Ring underrun | **Hold last PCM sample**, not forced digital zero |

**Do not burst multiple app frames at once** except through the existing bounded catch-up mechanism; this has previously produced a near-continuous sequence of 49-sample packets and audible pitch distortion. Do not implement a second app-level 47/48/49 packet controller: older ring sizing helpers exist but are unused in the working USB stream. Keep TinyUSB's current flow-control path.

### 2.7 PCM buffer and test source

- `audio_ring`: **8192 `int16_t` samples**, ring with one slot left empty, **480-sample** prefill (~10 ms). Producer = `audio_source`; consumer = `usb_audio`.
- Existing `ADC`: GP26 (Pico2) / GP40 (Kiwi); ADC FIFO + DMA **2 × 240 ADC words (5 ms)**; error bit handling, scale 16, offset 2048; leave logic unchanged.
- Existing `SYNTHETIC`: loop embedded `tick-tock.wav`, 19,192 samples at 48 kHz; **exact copy**, no crossfade/rotation. Do not replace with a 997 Hz sine from older specifications.
- Existing diagnostics UART: `DIAG_UART=1` uses `Serial1`, nominal GPIO0/1; default off. LED GP25 existing blink patterns remain.

## 3. Physical breakout module — wiring (proposed)

**Pins are proposed harness assignments, not proof GP2/GP3/GP4/GP21 are free on the user's boards.** Before wiring, check the received Comimark PCB silkscreen and continuity, actual installed power circuitry, and existing PCB pin allocation. The bare PCM1808 IC is not identical to an arbitrary module breakout.

| PCM1808 module label | RP2350 / supply | Direction | Function / caution |
|---|---|---|---|
| `SCK`, `SCKI` | **GP21** | RP2350 → PCM1808 | 12.288 MHz nominal-average system clock, **GPOUT0** |
| `BCK`, `BCLK` | **GP2** | PCM1808 → RP2350 | 3.072 MHz I²S bit clock |
| `LRC`, `LRCK`, `WS` | **GP3** | PCM1808 → RP2350 | 48 kHz I²S frame select |
| `OUT`, `DOUT`, `SDOUT` | **GP4** | PCM1808 → RP2350 | Serial stereo I²S data |
| `MD1` | **3.3 V high** | Strap before power | Master mode select |
| `MD0` | **3.3 V high** | Strap before power | 256× system-clock master mode |
| `FMT` | **GND low** | Strap before power | Standard 24-bit I²S |
| `5V` / `VCC` | **Regulated 5 V**, if board requires it | Supply | PCM1808 analog supply, verify module wiring |
| `3V3` / `VDD` | **Regulated 3.3 V**, if board requires it | Supply | PCM1808 digital supply, verify module wiring |
| `GND` | Common digital/analog ground | Supply | Common RP2350, module and TL071 preamp reference |
| `LIN`, `L` | TL071 `SIG` via module's verified AC-coupled input | Analog in | Measure DC level and peak voltage first |
| `RIN`, `R` | Leave unconnected for first test or quiet-terminate **ahead of input capacitor** if appropriate | Analog in | Do not hard-ground raw IC VINR pin and defeat its bias |

The PCM1808 bare IC needs **5 V analog VCC** and **3.3 V digital VDD**. **Do not connect a 5 V signal to RP2350 pins or PCM1808 VDD.** A module may have internal regulators or tied pads: continuity-check before feeding both rails to avoid regulator backfeed. Confirm input coupling capacitors; do **not** assume every PCM1808 module includes them. The observed watch-tick transient at the TL071 preamp output was ~**1.2–1.3 Vpp**, below PCM1808's nominal ~**3 Vpp full-scale**; higher real-world spikes or wrong DC coupling can still clip. Preserve any optional DNP attenuation footprint unpopulated for this bench test.

### 3.1 PCM1808 hardware mode and interface

Static mode straps must be valid **before power-up**:

| Control | Level | Selection |
|---|---|---|
| `MD1` | HIGH | Master mode |
| `MD0` | HIGH | 256 × sample-rate system clock |
| `FMT` | LOW | I²S, 24-bit data |

This yields:

| Signal | Direction | Nominal frequency |
|---|---|---:|
| SCKI/MCLK | GP21 → PCM1808 | **12,288,000 Hz** |
| BCK | PCM1808 → GP2 | **3,072,000 Hz** (64 BCK/frame) |
| LRCK | PCM1808 → GP3 | **48,000 Hz** |
| DOUT | PCM1808 → GP4 | Signed 24-bit stereo I²S, 32 BCK slots per channel |

This is **PCM1808 master** for BCK/LRCK, yet it **still requires RP2350-generated SCKI**. The ADC is not internally self-clocking from USB. I²S has a **one-bit delay** after LRCK transition. Left and right each occupy 32 bit-clock periods even though each converter word is 24 bits. Use the TI PCM1808 serial-format timing drawing to choose capture edge and word alignment. No I²C configuration bus is needed.

### 3.2 Power and warmup behavior

- Static MD1/MD0/FMT straps first; establish stable 5 V analog, 3.3 V digital, and SCKI. If the module is already powered at RP2350 reset, straps must already be correct.
- Ignore initial conversion data for **at least 250 ms after stable supplies and clock** (PCM1808 reset/soft-mute recovery nominally ~8960/fS plus fade-in). Use a **non-blocking readiness deadline**: do not stall the USB task loop for that interval.
- If the host opens before readiness, keep the existing stream FSM in `STREAM_PREFILL` or another well-defined waiting state; hold/silence host output through the existing USB policy until real capture fills the ring. Do not claim valid PCM1808 samples until readiness.
- On USB stream close, suspend or unplug, stop the PIO/DMA producer and reset the ring per existing source contract. The master clock may remain running as needed; do **not** assume GPIO clock-off meets USB suspend-current compliance—measure module and preamp consumption separately.

## 4. RP2350 clocks and fractional-divider policy — REQUIRED

**Neither CPU core is separately clockable.** Both share `clk_sys`; keep the normal Arduino-Pico board system-clock configuration at **150 MHz**. The SCKI clock is generated in the **hardware GPOUT0** peripheral, independently of the CPU instruction rate.

| Node / setting | Required normal PCM1808 build |
|---|---|
| Board crystal | Existing **12 MHz XOSC** |
| PLL_SYS / GPOUT0 source | **150,000,000 Hz** nominal |
| `clk_sys` | **150,000,000 Hz**, shared both cores |
| `clk_usb` | Existing independent **48 MHz** (do not modify PLL_USB) |
| GPOUT | **GPOUT0 on GP21** (GP13 may be alternate pin on RP2350, but default is GP21) |
| Divider integer | **12** |
| Divider 16-bit fraction | **`0x3500`**, equal to 13568/65536 = 53/256 |
| Divider total | **12.20703125** |
| SCKI nominal average | **150,000,000 / 12.20703125 = 12,288,000 Hz** |
| PCM1808 LRCK | **12,288,000 / 256 = 48,000 Hz** |
| PCM1808 BCK | **48,000 × 64 = 3,072,000 Hz** |

**Mandatory API:** `clock_gpio_init_int_frac16()` from the Pico SDK included in the selected Arduino-Pico toolchain. **Do not use the older** `clock_gpio_init_int_frac()` overload with only an 8-bit fractional field. If the toolchain lacks the 16-bit API, pin or upgrade the bundled framework/SDK deliberately and document the change. Do not silently approximate the divider or replace the stack.

```cpp
#include "hardware/clocks.h"

static constexpr unsigned PCM1808_MCLK_GPIO = 21u;
static constexpr uint32_t PCM1808_PLL_HZ = 150000000u;
static constexpr uint32_t PCM1808_USB_HZ = 48000000u;

bool pcm1808_clock_start() {
    // Read/verify the board's actual PLL_SYS configuration in addition to
    // checking the nominal SDK clk_sys and clk_usb frequencies.
    if (clock_get_hz(clk_sys) != PCM1808_PLL_HZ) return false;
    if (clock_get_hz(clk_usb) != PCM1808_USB_HZ) return false;

    clock_gpio_init_int_frac16(
        PCM1808_MCLK_GPIO,
        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        12u,
        0x3500u
    );
    return true;
}
```

**Important:** `clock_get_hz(clk_sys)` does not by itself prove PLL_SYS is exactly 150 MHz, or that `clk_sys` is sourced from PLL_SYS rather than another clock source. Confirm the clock-tree configuration in the actual board/core build, and independently verify GP21 output using an oscilloscope/frequency counter. The above is *illustrative code*, not proof that the unexamined project compiles it.

**Do not change PLL_SYS at runtime after Arduino/USB initialization.** If this board's actual `PLL_SYS` is not 150 MHz, treat that as a **configuration blocker to resolve early**, preferably by adjusting the supported PlatformIO board CPU frequency setting at build/boot, not by uncoordinated late PLL changes. `board_build.f_cpu` and platform-specific clock settings must be validated against the installed Arduino-Pico version; do not assume any particular setting works without checking. Provide a fail-fast diagnostic instead of silently emitting the wrong MCLK.

### 4.1 Jitter is accepted; accuracy is not fabricated

Because the GPOUT divider is fractional, individual MCLK periods exhibit **deterministic cycle quantization** (~6.67 ns base 150 MHz clock period). The divider ratio is exactly representable, so there is no divider *rounding* error in the **nominal average** frequency. Actual absolute frequency still depends on the **12 MHz crystal's ppm error**. Do not call the variation random jitter or long-term clock drift. Report noise/tick timing as results obtained with this **fractional-clock bench arrangement**, not as definitive PCM1808 silicon limits.

The old 61.44 MHz / integer-divider configuration is **not part of the required build**, **must not be activated**, and is mentioned only as an optional future isolated lab experiment. No additional MCLK oscillator is required.

## 5. New PCM1808 acquisition backend

### 5.1 Add files in existing repository style

New (recommended paths; adapt naming to the real checked-out source where needed):

```text
include/pcm1808_capture.h        # init/start/stop/process/readiness/stats
include/pcm1808_clock.h          # GPOUT clock init, shutdown, nominal clock checks
include/pcm1808_format.h         # pure signed PCM conversion helpers (host-testable)
src/pcm1808_capture.cpp          # PIO + DMA setup, ISR, raw-block processing
src/pcm1808_clock.cpp            # hardware_clocks API integration
src/pcm1808_i2s.pio              # PIO RX assembler source
include/pcm1808_i2s.pio.h        # checked-in generated header for PlatformIO build
scripts/gen_pcm1808_pio.py       # or PlatformIO extra_script prebuild; repeatable generation

tests/test_pcm1808_pcm_conversion.cpp
```

**PlatformIO specificity:** Do **not** assume a `.pio` source automatically generates `.pio.h` under Arduino + PlatformIO. Either add a verified `extra_scripts` prebuild step invoking `pioasm` from the installed Pico SDK toolchain **or** check in the generated header plus a reproducible documented regeneration command. Integrate through `platformio.ini` rather than adding Pico SDK `CMakeLists.txt` as the primary build path.

### 5.2 `audio_source` integration

- Preserve the existing `audio_source` API and FSM; inspect `include/audio_source.h` before designing any new public API.
- Add compile-time branch selecting `PCM1808` and initialize its clock/acquisition backend on that path only.
- `audio_source_process()` drains **completed PCM1808 DMA blocks**, converts them to `int16_t`, pushes samples into the existing ring, and updates source diagnostics. Do **not** write USB data here.
- Stream-opening event starts (or permits start of) capture only after clock/ADC readiness. Stream-closing event stops DMA/PIO, clears pending completion flags, and resets ring via existing logic.
- When no stream is open, avoid filling the PCM ring indefinitely; either stop PIO/DMA or explicitly discard capture data. State which choice is implemented.
- Keep the synthetic and internal ADC module code and conditional compilation unchanged except for source selection additions.

### 5.3 PIO requirements (PCM1808 is BCK/LRCK **master**)

**Functional contract:** PIO samples DOUT on the correct BCK edge and uses LRCK channel boundaries to align an I²S stream. No RP2350-generated BCK or LRCK, software-bitbanged polling, or CPU-timed sample collection. A suitable approach uses PIO `WAIT` on GP2/GP3 with `IN PINS, 1` from GP4, then autopush or explicit push of 32-bit slot words to the RX FIFO. **The resulting PIO assembly and pin mapping must be reviewed against the current SDK `pioasm` and tested on hardware.**

Requirements:

1. Detect a valid LRCK transition and discard the standard **one-bit I²S delay** before collecting the first MSB. Verify channel ordering with a test signal applied only to `LIN`.
2. Produce **one 32-bit word per channel slot**, with the converter's 24-bit signed sample in **bits 31..8** and the 8 unused clocks in **bits 7..0** (do not assume their physical data values; discard them). The PIO design must prove/verify this alignment; conversion helper assumes it.
3. Collect **64 BCK periods per stereo frame**, nominally **48,000 frames/s**; prevent cumulative framing slips from inserting/dropping bits.
4. Select stable PIO sampling edge per TI PCM1808 timing table and verify with a waveform or known-input test. At 150 MHz system clock, there are ~48.8 PIO cycles per 3.072 MHz BCK period, sufficient for a small WAIT/IN loop if written correctly.
5. Handle loss of BCK/LRCK, late DMA service or broken alignment: increment explicit fault/resync counters, disable/flush stream as appropriate, then restart at a known LRCK boundary. **Do not silently report perfect continuous capture after a resynchronization.**
6. Do not bind any of PIO's output side-set pins to GP21; the MCLK signal is controlled only by GPOUT0.

The PIO decoder may optionally capture both channel words for debug. Only `LIN`/left becomes USB audio. No forced downmix of two channels (averaging can distort single-channel amplitude).

### 5.4 DMA buffers and resource ownership

Keep the existing internal ADC implementation's convenient **5 ms block cadence** for the new backend:

| Parameter | PCM1808 bench backend |
|---|---|
| `APP_PCM1808_DMA_FRAMES` | **240 complete stereo frames** = 5 ms |
| Raw words per block | **480 × 32-bit words** (L,R interleaved) |
| Bytes per raw DMA block | **1,920 bytes** |
| Buffer count | **2** (ping-pong, separate from `audio_ring`) |
| PCM output per block | **240 mono `int16_t` samples** |
| DMA trigger | PIO RX DREQ (not CPU timer) |
| IRQ work | Set ready flags / increment counters / rearm as required; no DSP, Serial prints, or USB calls |
| Conversion location | `audio_source_process()`/backend process in main loop |

Use DMA channels claimed/allocated through the SDK, don't hardcode fixed channel indices if the existing app or libraries might use them. Explicitly handle ping-pong buffer ownership; never overwrite a block before it is consumed. Define behavior when USB/main loop is too slow: increment overrun counter and drop an **entire stale stereo-frame block** rather than corrupting word alignment; ring underruns follow the existing **hold-last-sample** policy.

If ping-pong chaining/rearm implementation changes the number of channels required, explain and test it. ADC and PCM1808 IRQ handlers must not conflict; both backends are compile-time mutually exclusive.

### 5.5 24-bit stereo -> mono PCM16 conversion

The PCM1808 outputs signed **24-bit two's-complement** samples in two 32-bit I²S slots. For the above verified left-justified 32-bit raw-slot representation:

```text
raw stereo frame = [ left_slot32, right_slot32 ]
left24 = signed-extend(bits 31..8 of left_slot32)      // range -8388608..8388607
pcm16  = arithmetic_shift_right(left24, 8)            // range -32768..32767
append exactly ONE pcm16 sample per stereo frame to audio_ring
```

Make a small host-testable conversion helper without relying on the main firmware/Arduino runtime. Use explicit sign extension or well-documented compiler behavior for arithmetic negative right shifts, and test it; no `float`, software normalization, AGC, filtering, clipping-for-no-reason, or host-channel changes.

Mandatory test vectors for *MSB-aligned* raw left slots:

| `left_slot32` | Expected PCM16 |
|---|---:|
| `0x00000000` | 0 |
| `0x7FFFFF00` | +32767 |
| `0x80000000` | -32768 |
| `0xFFFFFF00` | -1 |
| `0x00000100` | 0 |
| `0xFFFFFE00` | -1 |

Also test that changing right-slot data cannot affect mono output, that sign and amplitude are correct with 1 kHz input, and that an I²S one-bit-shift error is detected in a known-waveform test.

## 6. USB streaming, clock-domain drift, and life-cycle

### 6.1 Preserve the working endpoint strategy

The USB stack already writes **48 PCM samples per millisecond** into TinyUSB, which owns actual 47/48/49-sample ISO packet flow control. This **remains unchanged** for the first PCM1808 implementation. Do not switch to an imagined native USB stack, alter UAC descriptors, add playback, or replace the existing app pacing with new packet-burst logic.

The USB 48 MHz clock, host frame clock, and analog conversion timebase are not identical in ppm. **A fractional divider with an exact mathematical ratio does not prevent cumulative producer/consumer drift relative to the USB host.** The existing 8192-sample ring is a temporary elastic buffer, not infinite drift compensation. Do not claim continuous lossless hours without measurement.

New diagnostics must record **ring occupancy trend**, min/max occupancy, underruns/overruns, actual samples produced/consumed, real USB packet-size statistics if available, and PIO resync events. If an extended soak reveals unbounded drift, isolate it as a separately measured **follow-on clock-domain-adaptation defect**; do not silently change both TinyUSB flow control and the app feeding algorithm in the same PCM1808 integration patch.

### 6.2 Start, stop, resume and failure handling

- On boot, initialize MCLK once, verify config and mark warmup deadline. During no-stream idle leave capture stopped.
- On host `SET_INTERFACE` to streaming alternative, start PIO/DMA capture after readiness; enter existing `STREAM_PREFILL` until ring reaches 480 samples.
- On close/alt0, USB unplug, USB suspend: stop capture, abort DMA channels safely, disable PIO state machine, clear PIO FIFOs, discard stale completed blocks, and reset ring and prefill status. Avoid IRQ callbacks accessing freed/reinitialized buffers.
- On resume/reopen, resynchronize at a fresh LRCK boundary, clear old partial frames, prefill anew, and continue existing USB behavior.
- On missing MCLK, BCK or LRCK: transition to `STREAM_ERROR` if appropriate, increment count, allow bounded reinitialization or require stream close/reopen; do not hang the Arduino main loop.
- Use fixed static buffers suitable for IRQ/DMA; do not allocate/free DMA capture blocks on every stream start. Avoid serial logging inside ISR.

## 7. Diagnostics additions, no existing counters removed

Add to `diag_counters_t` (or a subsidiary PCM1808 stats struct merged into the current diagnostic dump), preserving all current USB/ADC/DMA/ring counters:

| Counter / value | Required meaning |
|---|---|
| `pcm1808_clock_config_ok` | Clock tree and GPOUT setup passed software validation |
| `pcm1808_mclk_nominal_hz` | **Configured** 12,288,000; not a measured hardware claim |
| `pcm1808_startup_wait_count` | Starts delayed by warmup |
| `pcm1808_dma_blocks` | Complete raw blocks captured |
| `pcm1808_stereo_frames_received` | Total full stereo frames |
| `pcm1808_mono_samples_emitted` | Samples pushed into ring |
| `pcm1808_dma_block_overruns` | Full DMA block could not be consumed safely |
| `pcm1808_pio_fifo_stall_or_overflow` | Actual detection if possible; otherwise report unavailable |
| `pcm1808_frame_sync_errors` | Detected frame alignment faults |
| `pcm1808_resync_count` | Deliberate frame decoder restarts |
| `pcm1808_sample_peak_abs` | Maximum PCM16 abs, avoid `abs(INT16_MIN)` overflow |
| `pcm1808_saturated_count` | Exact 24-bit full-scale hits (if observed) |
| Ring min/mean/max | Recorded throughout stream; existing underrun/overrun counts retained |
| USB packets / samples / 47-48-49 histograms | Record actual values only where TinyUSB exposes them, not guessed from application writes |

Keep `DIAG_UART=1` via Serial1 GPIO0/1 and no periodic logging when disabled. Preserve status LED behavior (GP25); existing fault blink remains.

## 8. Implementation sequence and gates

### Gate A — Baseline freeze and build verification

- [ ] Inspect `platformio.ini`, actual `include/` APIs, `src/audio_source.cpp`, `usb_audio.cpp`, and underlying framework version before coding.
- [ ] Record known working **four original envs**, their compilation status and exact PlatformIO/Arduino-Pico/TinyUSB dependencies.
- [ ] Confirm GP2, GP3, GP4, GP21 availability and 5 V/3.3 V physical supply pins for both Pico2 and Kiwi boards. If GP21 unavailable, choose **GP13 (also GPOUT0 on RP2350)** only after checking compatibility/wiring; update pin table and firmware constants together.
- [ ] Verify bundled Pico SDK exposes `clock_gpio_init_int_frac16`, and `pioasm` or a checked-in generated PIO header is available.

### Gate B — Clocks and breakout electrical validation

- [ ] With mode straps fixed *before module power*, verify actual PCM1808 supplies, no regulator backfeed, and TL071 signal AC-coupling.
- [ ] Both cores retain nominal shared 150 MHz clock, USB remains 48 MHz.
- [ ] GP21 SCKI measured at ~**12.288 MHz average**; divider fields **12** and **0x3500**; deterministic jitter accepted. Verify signal high/low timing is acceptable to module.
- [ ] PCM1808 BCK measured near **3.072 MHz**, LRCK near **48.000 kHz** at steady power.
- [ ] Wait ≥250 ms after stable module power/MCLK before evaluating samples.

### Gate C — I²S capture and PCM conversion

- [ ] PIO RX + DMA captures **10,000+ consecutive stereo frames** with correct boundaries and no FIFO/DMA errors.
- [ ] 1 kHz sine into `LIN` identifies the left slot and verifies 24-bit signed alignment and sample amplitude; R channel ignored.
- [ ] All host conversion tests pass, including negative full-scale, zero, right-channel isolation, and deliberate one-bit misalignment detection.
- [ ] No USB APIs or Serial calls inside PIO DMA IRQ handler.

### Gate D — USB non-regression and soak

- [ ] All **six** PlatformIO envs build (`pico2`, `pico2_synthetic`, `kiwi`, `kiwi_synthetic`, plus two PCM1808 envs).
- [ ] USB enumeration for PCM1808 remains **CDC + one UAC2 mono 48 kHz S16LE input**, no audio playback.
- [ ] Synthetic `tick-tock.wav` unchanged; internal ADC behavior unchanged.
- [ ] Record PCM1808 stream for 60 s and 30 minutes; record underruns, overruns, resyncs, ring trend, USB packet stats where observable.
- [ ] USB stream open/close ten cycles, USB replug five cycles, suspend/resume five cycles; no stuck DMA or stale samples.
- [ ] Existing `tests/host_analyze.py` continues to work; bench tests marked **not run** if no hardware available.

### Gate E — Watch timing value comparison

- [ ] Capture matched conditions: powered TL071 source, same watch/mount and approximate source amplitude, comparable room noise, no undocumented software gain differences.
- [ ] Compare ~30 s quiet baseline and ~60 s watch ticks for both RP2350 internal ADC and PCM1808 (sequential recording acceptable; note non-simultaneity).
- [ ] Report RMS noise, tick peak-to-noise ratio, clipping, transient shape, tick detection recall / false positives, and sample continuity. Distinguish ADC benefit from hardware gain/coupling differences.
- [ ] Label fractional-divider clock as bench-test configuration; avoid extrapolating PCM1808 results to final PCM1809 production board without testing.

## 9. Required code-generator output

Make the smallest safe incremental patch **to the existing repo**, not a clean-room rewrite:

1. Updated `platformio.ini` with two PCM1808 envs, preserving current four envs, PlatformIO platform and Arduino core, Adafruit TinyUSB and its compile flags.
2. Updated `include/app_config.h`, `include/board_config.h`, existing `audio_source.*` and diagnostics as necessary; unchanged USB descriptors, main-loop scheduler and `audio_ring` except source-specific bug fixes proven necessary.
3. `pcm1808_clock.*`, `pcm1808_capture.*`, `.pio` and generated `.pio.h`/repeatable code-generation integration, and pure PCM conversion helper.
4. Working PIO/DMA code with clear buffer ownership, correct I²S bit alignment and signed 24-to-16 conversion; no pseudocode in final firmware files.
5. Host conversion/unit tests and PIO timing sanity tests; extend host WAV analysis only without breaking existing entry points.
6. `docs/PCM1808_WIRING_AND_CLOCKS.md` with both Pico2/Kiwi connections, actual verified pin conflicts/changes, fractional divider rationale, electrical safety notes and manual oscilloscope checks.
7. Updated `docs/acceptance_results.md` distinguishing **compiled**, **unit-tested**, **bench-validated**, and **not tested**. Supply real build logs/results or explicit blockers.
8. A short `CHANGELOG_PCM1808.md` identifying exact affected files and listing preserved as-built behaviors.

**The code generator must not claim hardware validation without hardware, must not drop CDC to satisfy an obsolete goal, must not switch the project to CMake, and must not silently downclock to 61.44 MHz.**

## 10. Source-of-truth and external references

1. **`AS_BUILT_ARCHITECTURE_SOURCE.md`** (user-supplied; exact local copy): highest priority for current implementation facts. This document was provided, but underlying repository source was not supplied for a compile audit.
2. [Texas Instruments PCM1808 datasheet SLES177B](https://www.ti.com/lit/ds/symlink/pcm1808.pdf): pin function, master-mode straps, SCKI/BCK/LRCK timing, 24-bit I²S, startup and electrical constraints.
3. [Raspberry Pi Pico SDK hardware clocks API](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_clocks/include/hardware/clocks.h): RP2350 GPOUT pins and `clock_gpio_init_int_frac16()`.
4. [Raspberry Pi Pico SDK documentation](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html): clock, PIO, DMA APIs.
5. [Comimark PCM1808 module listing](https://www.amazon.com/dp/B07X3QLWR8): module identity only, **not** a verified breakout schematic.

## 11. Traceability: what was corrected from previous spec P2.1

| P2.1 instruction | P3.0 correction |
|---|---|
| Standalone Pico SDK/CMake architecture | **PlatformIO + Arduino-Pico/Earle Philhower** |
| USB mic only, no CDC | Preserve existing **CDC + UAC2 mono mic** composite |
| TinyUSB reference `audio_test` behavior as starting point | **Actual `usb_audio.cpp` 48 samples/ms + flow-control** retained |
| Generic source API possibly added | **Extend existing `audio_source` API/FSM**, no parallel framework |
| New 256-frame stereo DMA blocks | **240-frame blocks (5 ms)**, matching existing ADC capture cadence |
| New C/CMake-style source files | **C++ modules, PlatformIO build, PIO generated header** |
| Synthetic sine/test generator assumption | **Embedded looped `tick-tock.wav`** preserved |
| USB no CDC and no serial diagnostics | USB CDC remains; physical optional **Serial1 diagnostic UART** retained |
| Both cores 150 MHz with fractional GPOUT | **Unchanged and mandatory**; no need to downclock |
| New external ADC needs new USB descriptor path | **No new USB descriptor implementation**; existing endpoint unchanged |

**End of P3.0 implementation specification.**
