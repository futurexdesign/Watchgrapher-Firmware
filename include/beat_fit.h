/**
 * @file beat_fit.h
 * @brief Least-squares beat timing: rate and beat error from tick timestamps.
 *
 * Each accepted tick is assigned an integer half-beat number {@code k} on the
 * nominal grid (so missed ticks leave gaps rather than doubling an interval)
 * and the window is fitted with
 *
 *   t_k = a + P * k + c * (-1)^k
 *
 * where {@code P} is the measured half-period and {@code c} the TIC/TAC
 * offset. Rate follows from {@code P} against the nominal half-period, and
 * beat error is {@code 2|c|}. Fitting many beats instead of taking medians of
 * whole-sample intervals gives sub-sample resolution: a 1-sample step at
 * 24 kHz on one 28,800 A/h period is ~167 ppm (~14 s/d), while the fit over a
 * few seconds resolves well under 1 s/d.
 *
 * Timestamps are stored as small float offsets from the nominal grid (rebased
 * on the oldest point), so the solve runs in single precision on the M33 FPU
 * without losing sub-sample resolution on long runs.
 *
 * Pure C++ (no Arduino / Pico SDK) so it is host-testable.
 *
 * @see dsp_graph.cpp, tests/test_beat_fit.cpp
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Max ticks held; at 57,600 A/h (16 Hz) this is 16 s of history. */
#define BEAT_FIT_CAP 256u
/** Fewest ticks for a rate solution. */
#define BEAT_FIT_MIN_POINTS 8u
/** A gap longer than this many half-beats restarts the fit. */
#define BEAT_FIT_MAX_GAP 8
/** Ticks further than this fraction of a half-beat off-grid are rejected. */
#define BEAT_FIT_MAX_GRID_ERR 0.3f
/** This many consecutive off-grid rejects restarts the fit at the new tick. */
#define BEAT_FIT_MAX_REJECTS 4u

typedef struct {
  double origin;        /**< Absolute sample time of k = 0 on the grid. */
  double nominal;       /**< Nominal half-period (samples) the grid uses. */
  int32_t k[BEAT_FIT_CAP];
  float u[BEAT_FIT_CAP]; /**< t_k - (origin + k * nominal), samples. */
  uint32_t head;         /**< Index of the oldest point. */
  uint32_t n;
  int32_t last_k;
  double last_t;
  uint32_t rejects; /**< Consecutive off-grid rejects. */
} beat_fit_t;

typedef enum {
  BEAT_FIT_ACCEPTED = 0,
  BEAT_FIT_STARTED,  /**< First point, or fit restarted at this point. */
  BEAT_FIT_REJECTED, /**< Off-grid or non-increasing; fit unchanged. */
} beat_fit_push_result_t;

typedef struct {
  double half_period;      /**< Fitted half-period P (samples). */
  float half_period_delta; /**< P minus the nominal half-period (samples). */
  float alt_offset;   /**< TIC/TAC offset c (samples); BE = 2|c|. */
  float resid_rms;    /**< RMS residual of inliers (samples). */
  uint32_t points;    /**< Inliers used in the final solve. */
  bool alt_valid;     /**< False when the window holds only one parity. */
} beat_fit_result_t;

/** Clears all points; the next push starts a new grid. */
void beat_fit_reset(beat_fit_t *f);

/**
 * Adds one tick timestamp.
 *
 * @param t Absolute tick time in samples (may carry a sub-sample fraction).
 * @param nominal_half_period Nominal half-period in samples. A change from
 *        the value the current grid was started with restarts the fit.
 * @param max_span Points older than {@code t - max_span} samples are dropped.
 */
beat_fit_push_result_t beat_fit_push(beat_fit_t *f, double t,
                                     double nominal_half_period,
                                     double max_span);

/**
 * Solves the current window. Two passes: a full fit, then a refit without
 * points more than 3.5 sigma (and more than 1 sample) from the first line.
 *
 * @return {@code false} if fewer than {@link BEAT_FIT_MIN_POINTS} points.
 */
bool beat_fit_solve(const beat_fit_t *f, beat_fit_result_t *out);
