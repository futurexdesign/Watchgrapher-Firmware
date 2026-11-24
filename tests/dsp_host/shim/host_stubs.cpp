/**
 * @file host_stubs.cpp
 * @brief Host definitions of the firmware services the DSP chain links to.
 *
 * Provides the simulated clock ({@link host_clock.h}), the timebase calibration
 * accessors that beat_analysis.cpp reads (an uncalibrated clock: 0 ppm, source
 * "none"; {@code timebase_set} keeps the value in RAM only), and the
 * diagnostics counter block that perf_timing.cpp reads.
 */

#include "host_clock.h"

#include "diagnostics.h"
#include "timebase.h"

#include <cmath>

namespace host_clock {
namespace {
uint64_t g_now_us = 0;
} // namespace

void set_samples(uint64_t samples, uint32_t rate_hz) {
  g_now_us = samples * 1000000ull / rate_hz;
}

uint64_t now_us() { return g_now_us; }

} // namespace host_clock

namespace {
float g_ppm = 0.f;
timebase_source_t g_source = TIMEBASE_SRC_NONE;
diag_counters_t g_diag = {};
} // namespace

float timebase_ppm(void) { return g_ppm; }

timebase_source_t timebase_source(void) { return g_source; }

bool timebase_set(float ppm, timebase_source_t source, uint32_t, uint32_t) {
  if (!std::isfinite(ppm) || std::fabs(ppm) > 1000.f ||
      source == TIMEBASE_SRC_PPS || source >= TIMEBASE_SRC_COUNT) {
    return false;
  }
  g_ppm = ppm;
  g_source = source;
  return true;
}

const char *timebase_source_name(timebase_source_t s) {
  switch (s) {
  case TIMEBASE_SRC_MANUAL:
    return "manual";
  case TIMEBASE_SRC_NTP:
    return "ntp";
  case TIMEBASE_SRC_PPS:
    return "pps";
  default:
    return "none";
  }
}

diag_counters_t *diagnostics_get(void) { return &g_diag; }
