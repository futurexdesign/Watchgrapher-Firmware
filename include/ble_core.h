/**
 * @file ble_core.h
 * @brief Radio core selection ({@code TG_BLE_CORE}), the core-0 hold, the
 *        CYW43 status LED proxy and the core 1 stack watermark.
 *
 * Implements docs/plans/BLE_TRANSPORT_PLAN.md section 5.8 (task A9). The
 * implementation is {@code src/ble_core.cpp}; its file comment lists the boot
 * sequence step by step.
 *
 * {@code TG_BLE_CORE=1} (default for every BLE build, decided by HW-19 on
 * 2026-11-23) runs the CYW43 init and {@link ble_control_begin} on core 1
 * ({@link ble_core_setup1}) and needs the project variant
 * {@code variants/rpipico2w_tg} (set in {@code [board_pico2w]}; the flag
 * itself comes from {@code [ble]}). {@code TG_BLE_CORE=0} is an override that
 * leaves the radio where the framework puts it: the variant's
 * {@code initVariant()} initialises the CYW43 on core 0 and
 * {@link ble_control_begin} runs from {@code setup()}. It is built with a
 * custom env (docs/build_and_flash.md, "Radio core"), and the platformio.ini
 * flag keeps the build and the variant in agreement: the variant reads
 * {@code -DTG_BLE_CORE} from the command line, not this header.
 *
 * The header is host-compilable: {@link ble_core::wait_radio_ready} is a
 * header-only template that tests/test_ble_core.cpp exercises (CORE-03).
 */

#pragma once

#include <atomic>
#include <cstdint>

#ifndef TG_BLE
#define TG_BLE 0
#endif

#ifndef TG_BLE_CORE
/** Core that runs the radio (BT context): 0 or 1. Meaningful with TG_BLE=1.
 *  Defaults to 1 (HW-19); platformio.ini passes it explicitly. */
#define TG_BLE_CORE TG_BLE
#endif

#if TG_BLE_CORE != 0 && TG_BLE_CORE != 1
#error "TG_BLE_CORE must be 0 or 1"
#endif
#if TG_BLE_CORE == 1 && !TG_BLE
#error "TG_BLE_CORE=1 needs TG_BLE=1"
#endif

/** 1 when the radio runs on core 1 (BLE build with TG_BLE_CORE=1), else 0. */
#if TG_BLE && TG_BLE_CORE == 1
#define BLE_CORE_RADIO_ON_CORE1 1
#else
#define BLE_CORE_RADIO_ON_CORE1 0
#endif

namespace ble_core {

/** Longest time core 0 waits for the radio in {@link ble_core_hold}, in ms. */
constexpr std::uint32_t kHoldCapMs = 3000;

/**
 * Spins until {@code flag} is set or {@code cap_ms} have passed.
 *
 * This is the core-0 hold of plan section 5.8 rule 6. It must not run the
 * USB stack or yield, so the loop touches nothing but the flag and the
 * injected clock: no {@code delay()}, no {@code yield()}, no
 * {@code TinyUSBDevice.task()}. The clock is injected so the helper can be
 * tested on the host (CORE-03).
 *
 * @tparam NowMsFn callable returning the current time in ms as
 *         {@code std::uint32_t}; wrap-around is handled
 * @param flag set (release) by the thread that finishes the radio start
 * @param cap_ms give up after this many ms; 0 only samples the flag
 * @param now_ms clock, for example {@code millis}
 * @return true if the flag was set, false if the cap was reached first
 */
template <typename NowMsFn>
[[nodiscard]] bool wait_radio_ready(const std::atomic<bool> &flag,
                                    std::uint32_t cap_ms, NowMsFn now_ms) {
  const std::uint32_t start = now_ms();
  while (!flag.load(std::memory_order_acquire)) {
    if (static_cast<std::uint32_t>(now_ms() - start) >= cap_ms) {
      return flag.load(std::memory_order_acquire);
    }
  }
  return true;
}

} // namespace ble_core

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Core 1 entry of the radio (call first from {@code setup1()}), only built
 * with {@code TG_BLE_CORE=1}. Paints the core 1 stack, initialises the CYW43
 * on core 1, starts the LED worker, waits until core 0 is in
 * {@link ble_core_hold}, runs {@link ble_control_begin} and then publishes
 * "radio ready". Does not return before BLE is up or the CYW43 failed to
 * initialise (in which case core 0 times out in {@link ble_core_hold}).
 */
void ble_core_setup1(void);

/**
 * Core 0 side of the hold (plan section 5.8 rule 6). Only built with
 * {@code TG_BLE_CORE=1}; callers guard the call with
 * {@code BLE_CORE_RADIO_ON_CORE1}. Call from {@code setup()} right after
 * {@code usb_audio_init()}. Tells core 1 that core 0 is quiescent, then spins
 * (no {@code delay}, no {@code yield}, no USB task) until
 * {@link ble_core_setup1} reports ready, for at most
 * {@link ble_core::kHoldCapMs}.
 *
 * @return true if the radio is ready; false on timeout (core 0 continues and
 *         {@code ble_status_t::started} stays false until core 1 finishes)
 */
bool ble_core_hold(void);

/**
 * Prepares the status LED proxy on boards where the LED is a CYW43 GPIO.
 * Call once from {@code setup()} before the first {@link ble_core_led_set}.
 * With the radio on core 0 (or no BLE) the CYW43 already exists and the LED
 * worker is registered here; with {@code TG_BLE_CORE=1} core 1 does it in
 * {@link ble_core_setup1} and this call does nothing. No-op on other boards.
 */
void ble_core_led_init(void);

/**
 * Sets the wanted LED state (plan section 5.8 rule 4, F-25). Safe from
 * {@code loop()} on either core: stores the state in an atomic and signals
 * the LED worker in BT context with {@code async_context_set_work_pending()}.
 * Takes no async-context lock and does no SPI transfer. Until the worker is
 * registered the state is only stored and applied when it is. Only built on
 * CYW43-LED boards.
 *
 * @param on true to light the LED
 */
void ble_core_led_set(bool on);

/**
 * Free bytes of core 1's 8 KB stack: the part below the deepest point ever
 * reached (including BT IRQ work on core 1), measured by a paint pattern.
 * Feeds {@code getLink.ble.core1StackFree} (CORE-05 needs >= 2048). Safe from
 * any core; reads the other core's stack without synchronisation, which can
 * only make the result stale by one word.
 *
 * @return free bytes, or 0 when not measured (radio on core 0, or the stack
 *         could not be painted)
 */
std::uint32_t ble_core_core1_stack_free(void);

#ifdef __cplusplus
}
#endif
