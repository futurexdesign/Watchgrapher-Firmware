/**
 * @file host_clock.h
 * @brief Simulated clock of the host DSP harness (tests/dsp_host).
 *
 * The firmware reads {@code millis()} and {@code time_us_32()}. On the host
 * both derive from the number of samples the driver has fed, so a run is
 * deterministic and independent of wall time.
 */

#pragma once

#include <cstdint>

namespace host_clock {

/**
 * Sets the simulated time from the number of samples fed so far.
 * @param samples Samples fed to the graph since the start of the run.
 * @param rate_hz Sample rate of the signal (Hz).
 */
void set_samples(uint64_t samples, uint32_t rate_hz);

/** @return Simulated time in µs. */
uint64_t now_us();

} // namespace host_clock
