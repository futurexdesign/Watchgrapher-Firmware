/**
 * @file power_control.h
 * @brief TPS22918 8 V boost rail enable and nonblocking preamp warm-up.
 *
 * Controls {@code BOOST_EN_GPIO} (active HIGH) when {@code BOOST_SWITCH_PRESENT}
 * is 1. Defaults OFF at init; the board's 100 kΩ pull-down keeps the switch
 * off through reset / high-Z boot. Does not touch the always-on 3.3 V path
 * or USB PHY clocks.
 *
 * Warm-up is polled from the main loop — never {@code delay()} in TinyUSB
 * callbacks. Stage-1 suspend support only; MCU sleep / USB current compliance
 * is a measured follow-on.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Latches BOOST_EN LOW (if the switch is present), configures the GPIO as
 * output once, and clears warm-up state. Call at the earliest safe point in
 * {@code setup()} before USB / audio init.
 */
void power_control_init(void);

/**
 * Idempotent request to enable or disable the boost load switch.
 *
 * Safe from TinyUSB callbacks when {@code enable} is false (GPIO write only).
 * When the switch is not present, this is a no-op that still tracks the
 * requested logical state for diagnostics.
 *
 * @param enable {@code true} to drive BOOST_EN HIGH; {@code false} for LOW.
 */
void boost_power_enable(bool enable);

/**
 * @return {@code true} if the last {@link boost_power_enable} request was ON.
 */
bool boost_power_requested(void);

/**
 * @return {@code true} if the GPIO (when present) is believed driven HIGH.
 */
bool boost_power_is_on(void);

/**
 * Begins nonblocking preamp warm-up: enables the boost (if present) and
 * arms the settle deadline. Idempotent if already settling or ready.
 *
 * @param now_ms {@code millis()} sample from the caller.
 */
void power_control_begin_preamp_warmup(uint32_t now_ms);

/**
 * Aborts warm-up, clears the ready latch, and forces the boost OFF.
 * Call on stream close, suspend, unmount, or capture error.
 *
 * @param now_ms {@code millis()} (unused today; reserved for duration stats).
 */
void power_control_abort_preamp(uint32_t now_ms);

/**
 * @return {@code true} while a warm-up is in progress and the settle time
 *         has not yet elapsed.
 */
bool power_control_is_settling(void);

/**
 * @return {@code true} when warm-up has completed (or was unnecessary because
 *         {@code BOOST_SWITCH_PRESENT=0}). Remains true until abort/disable.
 */
bool power_control_preamp_ready(void);

/**
 * Advances the nonblocking settle timer. Call from the main loop.
 *
 * @param now_ms {@code millis()} sample.
 * @return {@code true} on the iteration when settle first completes.
 */
bool power_control_poll(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
