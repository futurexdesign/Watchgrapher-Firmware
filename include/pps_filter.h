/**
 * @file pps_filter.h
 * @brief Averages GPS 1 PPS period measurements into a clock error (ppm).
 *
 * Each input is the number of clk_sys cycles between two PPS rising edges.
 * Periods far from nominal (missed pulse, glitch, GPS without fix) are
 * rejected; accepted periods are averaged over a sliding window of up to
 * {@link PPS_FILTER_CAP} seconds. Edge-sampling error is a few cycles per
 * edge and does not accumulate, so 64 s at 150 MHz resolves ~0.001 ppm —
 * far below the GPS module's own PPS jitter (typically 10–50 ns).
 *
 * Header-only and SDK-free so it is host-testable.
 *
 * @see timebase.cpp, tests/test_pps_filter.cpp
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Seconds averaged. */
#define PPS_FILTER_CAP 64u
/** Accepted periods before the result is trusted. */
#define PPS_FILTER_MIN 16u
/** Periods further than this from nominal (ppm) are rejected. */
#define PPS_FILTER_REJECT_PPM 200.0

typedef struct {
  uint32_t nominal; /**< Expected cycles per second (clk_sys Hz). */
  uint32_t ring[PPS_FILTER_CAP];
  uint32_t head;
  uint32_t n;
  uint64_t sum;
  uint32_t good;    /**< Accepted since reset of counters (not of window). */
  uint32_t rejects; /**< Rejected since reset of counters. */
} pps_filter_t;

static inline void pps_filter_init(pps_filter_t *f, uint32_t nominal_hz) {
  f->nominal = nominal_hz;
  f->head = 0;
  f->n = 0;
  f->sum = 0;
  f->good = 0;
  f->rejects = 0;
}

/** Empties the averaging window (keeps the good / reject counters). */
static inline void pps_filter_clear(pps_filter_t *f) {
  f->head = 0;
  f->n = 0;
  f->sum = 0;
}

/** @return {@code true} if {@code cycles} was accepted into the window. */
static inline bool pps_filter_push(pps_filter_t *f, uint32_t cycles) {
  const double err_ppm =
      ((double)cycles - (double)f->nominal) / (double)f->nominal * 1e6;
  if (f->nominal == 0u || err_ppm > PPS_FILTER_REJECT_PPM ||
      err_ppm < -PPS_FILTER_REJECT_PPM) {
    f->rejects++;
    return false;
  }
  if (f->n == PPS_FILTER_CAP) {
    f->sum -= f->ring[f->head];
    f->ring[f->head] = cycles;
    f->head = (f->head + 1u) % PPS_FILTER_CAP;
  } else {
    f->ring[(f->head + f->n) % PPS_FILTER_CAP] = cycles;
    f->n++;
  }
  f->sum += cycles;
  f->good++;
  return true;
}

/** @return {@code true} once at least {@link PPS_FILTER_MIN} periods are held. */
static inline bool pps_filter_ready(const pps_filter_t *f) {
  return f->n >= PPS_FILTER_MIN;
}

/** @return Board clock error in ppm (+ = fast), or 0 if the window is empty. */
static inline double pps_filter_ppm(const pps_filter_t *f) {
  if (f->n == 0u || f->nominal == 0u) {
    return 0.0;
  }
  const double mean = (double)f->sum / (double)f->n;
  return (mean / (double)f->nominal - 1.0) * 1e6;
}
