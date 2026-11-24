/**
 * @file Arduino.h
 * @brief Host stand-in for the Arduino/pico-sdk time functions the DSP uses.
 *
 * Time is simulated ({@link host_clock.h}); there is no wall clock.
 */

#pragma once

#include "host_clock.h"

#include <cstdint>

/** @return Simulated milliseconds since the start of the run (wraps at 2^32).
 */
inline uint32_t millis() {
  return static_cast<uint32_t>(host_clock::now_us() / 1000u);
}

/** @return Simulated microseconds since the start of the run (wraps at 2^32).
 */
inline uint32_t time_us_32() {
  return static_cast<uint32_t>(host_clock::now_us());
}
