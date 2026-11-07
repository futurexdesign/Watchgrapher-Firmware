# USB Suspend and 8 V Preamp Power Control — Firmware Implementation Summary

**Project:** `timegrapher-stand`  
**Hardware:** USB Power Revision B (TPS22918 load switch + TPS61041 boost)  
**Firmware baseline:** User-supplied `AS_BUILT_ARCHITECTURE.md` (2026-11-04)  
**Status:** Implementation handoff; not yet implemented or bench-validated

## Objective

Extend the **existing** PlatformIO + Arduino-Pico (Earle Philhower core) + Adafruit TinyUSB firmware to control the Revision B 8 V preamplifier rail and respond correctly to USB suspend/resume. **Do not** rebuild the application using standalone Pico SDK/CMake, alter audio descriptors, remove CDC, or change the established audio packet scheduling.

### Hardware contract

- USB 5 V continues to supply the AP2112K and the **always-on 3.3 V** rail for the RP2350/RP2354 and PCM1809.
- The **TPS22918DBVR** (U3) connects USB 5 V to `BOOST_5V_SW`. Its output supplies **both** TPS61041 VIN/EN and the boost inductor. The 8 V regulator and external TL071 are disconnected when this switch is off.
- `TPS22918 ON` (pin 3) is driven from a **spare MCU GPIO**, name `BOOST_EN_GPIO` (**pin not yet assigned**). **HIGH = enable; LOW = disable.** Revision B provides an external **100 kΩ pull-down**, so it is normally off at reset/boot.
- `QOD` (pin 5) is connected to switched 5 V (`VOUT`, pin 6). **It discharges only `BOOST_5V_SW`, not `8V_RAW`/`8V_CLEAN`.** The 8 V capacitors may remain charged for some time after switch-off.
- The TPS22918 CT network soft-starts the switched rail. Keep the regulator and preamp warm-up **nonblocking**; verify actual settling time on the scope.
- **Never disable the AP2112K or the MCU's USB-resume/wake power path** through `BOOST_EN`.

## 1. Configuration and GPIO lifecycle

Add to the existing `include/board_config.h` / `include/app_config.h` (adapt naming to current project conventions):

```cpp
#define BOOST_EN_GPIO              /* spare validated GPIO: assign per board */
#define BOOST_SWITCH_PRESENT      1
#define PREAMP_SETTLE_MS          50  // provisional starting value; verify on hardware
```

For existing development boards without the TPS22918, support `BOOST_SWITCH_PRESENT=0` or a board-specific build flag to avoid driving an unrelated GPIO. Do not silently choose GPIO13/21/25; these may be allocated to other functions. Select and verify the real pin on the Kiwi board first.

**At the earliest safe point in `setup()`**, set the output latch LOW, configure the selected GPIO as output, and confirm it remains LOW until a permitted stream starts. Configure the pin once; don't repeatedly change its direction. Provide `boost_power_enable(bool)` as an idempotent helper and track requested and actual state for diagnostics. An ordinary MCU reset, firmware crash, or high-Z boot period must default the switch to OFF via the board's pull-down.

## 2. Required behavior by USB state

| Event / state | 8 V boost | I²S / ADC / DMA | Required action |
|---|---|---|---|
| Reset / boot | **OFF** | Stopped | Configure output LOW, initialize USB and FSM |
| USB not mounted / disconnected | **OFF** | Stopped | Clear pending warm-up and ring buffers |
| USB configured, recording closed (alternate 0) | **OFF** | Stopped | Stay idle; keep USB responsive |
| Host opens **real microphone** stream | **ON** | Start **after** preamp settles | Request switch HIGH; wait for warm-up; reset/prefill ring; begin capture |
| Recording active | **ON** | Running | Preserve existing audio scheduler |
| Recording stream closed | **OFF** | Stopped | Stop source safely, reset buffers, switch LOW |
| USB bus suspended (from any state) | **OFF immediately** | Stopped | Stop source, disable boost, enter `USB_SUSPENDED`; preserve USB wake |
| USB resumed | **OFF initially** | Stopped | Restore ordinary processing; restart **only if the host's recording alternate setting is still active**, or if it subsequently opens the stream |
| Persistent capture error | **OFF** | Stopped | Enter/report error state; no powered preamp left behind unnecessarily |

**Synthetic mode exception:** The embedded `tick-tock.wav` test source does not require the external preamp. Keep `BOOST_EN` **OFF** for `AUDIO_SOURCE=SYNTHETIC`, even if its USB stream is open. Real ADC and PCM1809 capture modes that use the microphone stand require the preamp rail.

**Resume nuance:** USB suspend normally does **not** reset the configured interface/alternate setting. Do **not** assume resume always produces a new `SET_INTERFACE` event. Retain or query whether the audio streaming interface was active before suspend and re-arm it following the nonblocking power-up/warm-up sequence when appropriate. Conversely, do not restart audio merely because the bus resumed if alternate setting 0 is selected.

## 3. TinyUSB integration with the *existing* callbacks

Use the present PlatformIO/Arduino-Pico/Adafruit TinyUSB stack and the existing `audio_source` FSM. Extend the already-owned callbacks rather than introducing duplicate symbol definitions:

```cpp
void tud_suspend_cb(bool remote_wakeup_en); // bus suspend notification
void tud_resume_cb(void);                    // bus resume notification
void tud_umount_cb(void);                    // disconnect/unmount
// existing: tud_audio_set_itf_cb(...) and tud_audio_set_itf_close_ep_cb(...)
```

First inspect the actual code and Adafruit wrapper for ownership of these callbacks; if the wrapper implements them, use its supported callback/handler mechanism. Keep callbacks/interrupt paths **short and nonblocking**. For suspend, lower `BOOST_EN` as promptly as safe using a previously initialized GPIO and set a lightweight event flag. Perform DMA cancellation, source teardown, buffer resets and warm-up state transitions in the normal application context (unless existing code explicitly guarantees safe operations in callbacks). No `delay()`, serial logging, memory allocation, or recursive TinyUSB tasks in callbacks.

Preserve the existing main-loop order:

1. `TinyUSBDevice.task()`
2. `audio_source_process()`
3. `usb_audio_task()`
4. `diagnostics_poll()`
5. LED update

Preserve **CDC + UAC2 mono microphone** enumeration, `TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR`, 48 kHz / signed PCM16, and the existing **48 samples per millisecond application feed with `CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL=1`**. The suspend change must not add a new packet-size controller or revert to burst feeding.

## 4. Nonblocking preamp turn-on sequence

On a legitimate microphone-stream request, and only if the device is mounted/configured and not suspended:

1. Cancel any stale suspend/power-off events.
2. Set `BOOST_EN` HIGH, record `boost_enabled_at_ms` and mark source as **POWER_SETTLING** (internal substate/flag).
3. Return to the main loop immediately and continue USB servicing. Do not block for rail rise or call `delay(PREAMP_SETTLE_MS)`.
4. After `millis() - boost_enabled_at_ms >= PREAMP_SETTLE_MS`, start the selected ADC/PIO/DMA source, discard any known invalid startup samples, clear the PCM ring, and enter the existing `STREAM_PREFILL` state.
5. On reaching the existing **480-sample prefill**, enter `STREAM_ACTIVE` and stream using the existing `usb_audio` logic.
6. If suspend, disconnect, stream-close, or error occurs at any point, **abort warm-up**, turn boost off, and leave no asynchronous restart pending.

The **50 ms** value is an initial bench-test assumption, not a validated electrical startup time. Tune after measuring `BOOST_5V_SW`, `8V_RAW`, `8V_CLEAN` and actual TL071 output settling. Avoid blocking or starving the USB endpoint while waiting: maintain the stack's currently working behavior (e.g., its existing hold-last-sample/silence path when capture has not yet filled the ring), and verify host compatibility.

## 5. Low-power MCU behavior: two implementation stages

**Stage 1 — required for this change:** Switch off the 8 V stage, stop audio acquisition clocks/PIO/DMA, disable unnecessary LED activity while suspended, and keep USB clocks/interrupts operational. Leave the 3.3 V rail powered. This is **functional USB-suspend support**, not yet evidence of USB suspend-current compliance.

**Stage 2 — measured optimization only:** Measure actual current drawn from USB VBUS during suspend. If above the **applicable USB 2.0 suspend-current limit for the device's configuration**, add USB-resume-capable RP2350 idle/sleep techniques. For instance, `WFI` may help if USB interrupts and the TinyUSB/Arduino core can wake reliably; validate on both core contexts if both cores are in use. Keep USB PHY/controller, its clock and interrupt wake functionality enabled. **Do not enter dormant mode or disable PLL_USB as an assumed optimization.** Consider PCM1809 clock-stop standby and flash/LED leakage as part of the total budget; gate further rails only if measurements justify it.

Do not assume the device qualifies for an elevated suspend allowance; verify the USB descriptors, remote-wakeup configuration and applicable spec. Remote wakeup is **not** a requirement of this patch; do not change descriptors just to claim a larger current budget.

## 6. Files to change (minimal incremental patch)

| File | Change |
|---|---|
| `include/board_config.h`, `include/app_config.h` | Board-specific `BOOST_EN_GPIO`, availability flag, nonblocking settle interval |
| New `include/power_control.h`, `src/power_control.cpp` (recommended) | Default-OFF GPIO initialization, idempotent enable/disable, warm-up timer, diagnostics |
| `src/audio_source.cpp` | Gate preamp power based on source type, stream open/close, USB state; integrate warm-up and safe source stop |
| `src/usb_audio.cpp` / callback owner | Extend suspend/resume/unmount handling without duplicate callback definitions; communicate events to source FSM |
| `src/TimegrapherStand_main.cpp` | Initialize power control early; preserve normal TinyUSB task servicing and LED behavior except suspend LED-off |
| `src/diagnostics.cpp` | Record suspend count/duration, boost ON/OFF events, startup waits, aborted starts, resumes, and error shutdown |
| `platformio.ini` | Add board-specific pin/enable build flags only where appropriate; preserve current Adafruit TinyUSB flags |
| `docs/acceptance_results.md` | Actual USB-current measurements and suspend/resume behavior, with untested items indicated |

Do **not** refactor the PCM ring, source enum, PCM conversion, descriptors, TinyUSB flow-control settings or existing ADC DMA logic as part of suspend work.

## 7. Acceptance tests

### Functional tests (must pass)

- [ ] Power-up and USB enumeration: `BOOST_EN` remains LOW until a **real microphone** stream is requested; verify default OFF through reset/high-Z boot.
- [ ] USB configured but idle: boost OFF and no unnecessary ADC/PIO clocks.
- [ ] Real microphone recording: GPIO goes HIGH; after observed rail/TL071 stabilization, capture begins; ring prefill completes and 48 kHz mono audio is recorded without added glitches.
- [ ] Synthetic recording: boost stays OFF and the existing looped `tick-tock.wav` stream still works.
- [ ] Stream close: GPIO LOW, capture stops, ring cleared; no stale re-enable event.
- [ ] Host suspends while streaming: GPIO LOW promptly; acquisition and LED stopped; USB wakes correctly on host resume without disconnect/re-enumeration.
- [ ] Resume while recording alternate setting remains active: preamp is powered and capture restarts **after warm-up** without waiting for a new stream-open callback.
- [ ] Resume when not recording: boost stays OFF.
- [ ] Repeated suspend/resume (at least 10 cycles), stream open/close (10), and USB unplug/replug (5): no stuck state, unexpected GPIO ON, or crashes.
- [ ] The `ADC`/PCM1809 real-mic backend and synthetic backend retain their existing USB audio format, CDC behavior, and packet pacing.

### Electrical / compliance tests (must report actual measurements)

- [ ] Observe `USB_5V`, `BOOST_5V_SW`, `8V_RAW`, `8V_CLEAN` at stream-open, suspend, resume, and removal; verify no overshoot, brownout, or problematic transients.
- [ ] Confirm `BOOST_5V_SW` discharges when disabled. **Do not require immediate 8 V discharge**—QOD does not directly discharge the 8 V capacitors.
- [ ] With actual ~2 mA TL071 load, measure 8 V settling time and preamp output startup artifact; tune `PREAMP_SETTLE_MS` based on results.
- [ ] Measure **total USB VBUS current** under active, configured-idle, and suspended conditions, not merely the boost current; record USB configuration/remote-wakeup state and compare to the applicable limit.
- [ ] If Stage 2 sleep optimizations are added, verify enumeration, CDC operation, and reliable host resume on each supported board.

## 8. Code-generator deliverables

Produce a **small patch to the existing `timegrapher-stand` repository**, not a new firmware application. Supply source changes, build logs for all supported PlatformIO environments, and a short changelog identifying affected modules. Hardware status must be labeled **NOT TESTED** until measured. Explicitly report the selected `BOOST_EN_GPIO` and any pin conflict, startup-delay measurement, total suspend-current results, resume behavior, and unresolved USB-compliance limits.

**Definition of done:** functional default-off boost control; nonblocking start/warm-up and correct streaming source transitions; suspend/close/unmount boost shutdown; reliable resume if the recording alternate setting remains active; **no regression** to existing CDC + UAC2 audio behavior. **USB current compliance is a separate measured gate**, not something to claim from firmware changes alone.
