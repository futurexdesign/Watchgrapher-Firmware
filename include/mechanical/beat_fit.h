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
 * @see mechanical/beat_analysis.cpp, tests/test_beat_fit.cpp
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Max ticks held; at 57,600 A/h (16 Hz) this is 16 s of history. */
#define BEAT_FIT_CAP          256u
/** Fewest ticks for a rate solution. */
#define BEAT_FIT_MIN_POINTS   8u
/**
 * A gap longer than this many half-beats restarts the fit. 32 half-beats is
 * 2 s at the fastest rate in the lock table (57,600 A/h), the gap beyond which
 * {@code beat_analysis} cuts the stretch itself; at slower rates the analysis
 * cut comes first.
 */
#define BEAT_FIT_MAX_GAP      32
/** Ticks further than this fraction of a half-beat off-grid are rejected. */
#define BEAT_FIT_MAX_GRID_ERR 0.3f
/** This many consecutive off-grid rejects restarts the fit at the new tick. */
#define BEAT_FIT_MAX_REJECTS  4u

typedef struct {
  double origin; /**< Absolute sample time of k = 0 on the (rebased) window. */
  double grid_origin; /**< Origin of the grid when it STARTED; never rebased. */
  int32_t k_base;     /**< Added to window k to get the grid-absolute k. */
  double nominal;     /**< Nominal half-period (samples) the grid uses. */
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

/**
 * Where the most recent push landed on the grid, whatever its outcome.
 * k and rel are grid-absolute: they do not change when the window slides.
 */
typedef struct {
  bool valid;     /**< False only if the push was refused (nominal <= 0). */
  int32_t k;      /**< Grid index since {@code grid_origin} (nearest slot). */
  double rel;     /**< t - grid_origin, samples. */
  double u;       /**< rel - k * nominal, samples (raw, uncorrected). */
  double nominal; /**< Nominal half-period of the grid (samples). */
} beat_fit_point_t;

typedef struct {
  double half_period;      /**< Fitted half-period P (samples). */
  float half_period_delta; /**< P minus the nominal half-period (samples). */
  float alt_offset;        /**< TIC/TAC offset c (samples); BE = 2|c|. */
  float intercept;         /**< Intercept a (samples, at the window origin). */
  float resid_rms;         /**< RMS residual of inliers (samples). */
  uint32_t points;         /**< Inliers used in the final solve. */
  bool alt_valid;          /**< False when the window holds only one parity. */
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
 * @param pt Optional; receives the grid index and residual of this point
 *        for every outcome. For STARTED, k = 0 and u = 0. For REJECTED, k is
 *        the nearest slot to the last accepted point and u the miss.
 */
beat_fit_push_result_t beat_fit_push(beat_fit_t *f, double t,
                                     double nominal_half_period,
                                     double max_span,
                                     beat_fit_point_t *pt = nullptr);

/**
 * Predicts the grid slot {@link beat_fit_push} would report for a tick at
 * {@code t}, without changing the fit. Callers that need the half-beat parity
 * of a tick before choosing the timestamp to push (the drop-timing stage keeps
 * one template per parity) use it; it applies the same gap, grid-error and
 * reject rules as the push.
 *
 * @param t Absolute tick time in samples.
 * @param nominal_half_period Nominal half-period in samples.
 * @param new_grid Receives {@code true} when the push would start a new grid
 *        (empty fit, new nominal period, long gap, or the reject limit); the
 *        returned index is then 0. Must not be {@code nullptr}.
 * @return Grid-absolute half-beat index, as {@link beat_fit_point_t#k}.
 */
int32_t beat_fit_predict_k(const beat_fit_t *f, double t,
                           double nominal_half_period, bool *new_grid);

/**
 * Solves the current window. Two passes: a full fit, then a refit without
 * points more than 3.5 sigma (and more than 1 sample) from the first line.
 *
 * @return {@code false} if fewer than {@link BEAT_FIT_MIN_POINTS} points.
 */
bool beat_fit_solve(const beat_fit_t *f, beat_fit_result_t *out);

/**
 * Moves every stored point onto another time base by adding a per-side
 * offset to its time. The drop-timing stage uses it at the handover from
 * unlock times to drop times: each side's running mean of (drop - unlock) is
 * added, so the fit continues without restarting and the rate stays
 * continuous.
 *
 * The side of a point is the parity of its grid-absolute slot
 * ({@code k + k_base}, as {@link beat_fit_point_t#k} reports it), not of the
 * window index, which changes parity whenever the window slides by an odd
 * number of points. The oldest point stays at {@code origin} with u = 0
 * (origin moves by its own offset), {@code grid_origin} moves by the even
 * side's offset so that slot 0 keeps its place, and the newest time moves by
 * its side's offset. Does nothing for an empty window.
 *
 * @param offset_even Samples added to points on even grid-absolute slots.
 * @param offset_odd Samples added to points on odd grid-absolute slots.
 */
void beat_fit_shift_points(beat_fit_t *f, double offset_even,
                           double offset_odd);

/**
 * Residual of a tick time against the solved line, whatever the tick's
 * outcome in {@link beat_fit_push}. The tick is placed on the grid slot
 * nearest to the newest accepted point (the slot a reject reports) and the
 * line's value there is subtracted.
 *
 * @param r Result of {@link beat_fit_solve} on {@code f}, taken after the
 *        push of {@code t} (accepted points are then inside the line).
 * @param t Absolute tick time in samples.
 * @return Signed residual in samples (+ = later than the line); 0 for an
 *         empty window.
 */
double beat_fit_residual(const beat_fit_t *f, const beat_fit_result_t &r,
                         double t);

/**
 * Time the current window covers: newest minus oldest point.
 *
 * @return Span in samples; 0 for an empty or one-point window.
 */
double beat_fit_span(const beat_fit_t *f);
