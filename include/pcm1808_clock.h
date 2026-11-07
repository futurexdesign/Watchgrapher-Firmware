/**
 * @file pcm1808_clock.h
 * @brief PCM1808 system clock (SCKI / MCLK) via clk_gpout from PLL_SYS.
 *
 * Requires {@code clk_sys == 150 MHz} and {@code clk_usb == 48 MHz} (see
 * {@code PCM1808_EXPECT_* } in board_config). Does not reprogram PLLs at
 * runtime — wrong clocks fail fast with a diagnostic flag.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Nominal SCKI frequency produced by div 12 + frac 0x3500 at 150 MHz sys. */
#define PCM1808_MCLK_NOMINAL_HZ 12288000u

/**
 * Validates sys/USB clocks, drives {@code PCM1808_MCLK_GPIO} as GPOUT0 SCKI,
 * and records a warmup deadline (≥250 ms) for {@link pcm1808_clock_ready_deadline_ms}.
 *
 * @return {@code false} if sys/USB Hz do not match expectations (capture should
 *         treat the backend as errored).
 */
bool pcm1808_clock_start(void);

/**
 * Stops GPOUT0 and returns the MCLK pin to high-Z SIO input.
 * Optional for bench — MCLK may be left running across capture stop.
 */
void pcm1808_clock_stop(void);

/**
 * @return {@code true} if clocks matched at start and GPOUT is still running.
 */
bool pcm1808_clock_is_ok(void);

/**
 * @return {@link PCM1808_MCLK_NOMINAL_HZ} (design target, not a measured value).
 */
uint32_t pcm1808_clock_nominal_hz(void);

/**
 * @return {@code millis()} value after which MCLK is considered warm enough
 *         to start I²S capture.
 */
uint32_t pcm1808_clock_ready_deadline_ms(void);

#ifdef __cplusplus
}
#endif
