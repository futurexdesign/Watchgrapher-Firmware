/**
 * @file tick_shape.h
 * @brief Stage B of the mechanical timing chain: envelope front end and
 *        beat-template timing of the drop landmark.
 *
 * Implements item 1 of {@code docs/plans/TIMING_ENGINE_PLAN.md} after the
 * method of opentimegrapher (OTG, pages M1 and M6). Stage A
 * ({@code tick_detect.cpp}) still finds tick windows and the unlock / drop
 * estimates from raw samples. Stage B times the <em>drop</em> from the shape of
 * the whole beat, which does not jump between the unlock and the drop when
 * their loudness changes.
 *
 * Two parts, both internal to the DSP graph:
 *
 *   - Front end, per source sample ({@link tick_shape_push}): 4th-order
 *     Butterworth high-pass at 1.5 kHz (two biquads), rectify, 0.2 ms boxcar,
 *     decimate to at most 24.5 kHz into an {@code int16} ring. It runs only
 *     while enabled ({@link tick_shape_set_enabled}), so the default
 *     {@code timingLandmark = unlock} costs nothing.
 *   - Per event ({@link tick_shape_measure}): locate the drop in the ring,
 *     learn one template per beat side (mean of the first
 *     {@link TICK_SHAPE_LEARN_BEATS} accepted beats, then an exponential
 *     update at 1/32), and slide it over the beat with a normalised
 *     cross-correlation over +-2 ms. The timestamp is the template's own drop
 *     landmark (half-height crossing of the rising edge into the drop peak)
 *     moved by the lag, so it is a physical landmark and not an arbitrary
 *     alignment point.
 *
 * <b>Interface to {@code beat_analysis}.</b> A template belongs to a side of
 * the escapement (even / odd half-beat of the fit grid), but the grid slot is
 * known only after the timestamp has been pushed to the fit, and the
 * timestamp is what this module produces. {@code beat_analysis} therefore
 * asks the fit for the slot the unlock time would take
 * ({@link beat_fit_predict_k}), derives the side from its parity and passes it
 * in. The drop lies a lift time (about 10 ms) after the unlock, far less than
 * the 0.3 half-beat grid tolerance, so both times map to the same slot. On a
 * new grid the parity-to-side mapping is arbitrary; the caller then sets
 * {@code detect_side} and the module picks the side whose template matches
 * best.
 *
 * <b>Ring length.</b> The ring is read when the detector closes an event, from
 * inside the block that contains the close. Looking back from the newest ring
 * sample to the earliest sample the correlation needs:
 *
 * | Part | ms |
 * |---|---|
 * | rest of the block the event closed in (the ring is advanced by the whole
 * block first) | 5 (240 samples at 48 kHz) | | drop to close, quiet close:
 * {@code tail_ms} (25) plus loud decay after the drop (allowance 12) | 37 | |
 * drop to close, forced close of a noisy event: 80 ms window, onset 25 ms into
 * it, drop not before the onset | 55 | | template pre-roll before the drop | 20
 * | | correlation search before that | 2 | | total, with the larger
 * drop-to-close case | 82 |
 *
 * That is 1,968 samples at 24 kHz (3.9 KB). The sum is checked at compile time
 * against the ring capacity ({@code static_assert}). The forced close is not a
 * corner case: on the 20 dB SNR test clip most events are closed by the 80 ms
 * limit because the noise keeps the window above the close threshold. An
 * event whose drop lies outside what the ring still holds (a larger
 * {@code tail_ms} or a shorter {@code preroll_ms}, or a slower source rate with
 * the same block size) is skipped and counted in
 * {@link tick_shape_stats_t#out_of_ring}.
 *
 * Not thread-safe: call only from the core that runs the DSP graph.
 *
 * @see mechanical/beat_analysis.h, mechanical/tick_detect.h,
 *      mechanical/beat_fit.h, scripts/synth_clips/reference_otg.py (offline
 *      reference of the same method)
 */

#pragma once

#include "dsp_types.h"

#include <cstdint>

/**
 * Beats folded into a side's template, as a plain mean, before it is used for
 * timing. Later beats update it with weight 1/32.
 */
inline constexpr uint32_t TICK_SHAPE_LEARN_BEATS = 16u;

/**
 * Lowest normalised correlation (0..1) at which a beat is timed and may update
 * its template. OTG ignores beats below 0.4.
 */
inline constexpr float TICK_SHAPE_QUALITY_MIN = 0.4f;

/** Result of timing one event. */
struct tick_shape_result_t {
  /**
   * True when {@code drop_sample} is usable: the side's template is learned
   * and the correlation reached {@link TICK_SHAPE_QUALITY_MIN}.
   */
  bool valid = false;
  uint8_t side = 0;    /**< Side (0 or 1) whose template was used. */
  float quality = 0.f; /**< Peak normalised correlation, 0..1; 0 if none. */
  /**
   * Time of the drop landmark in source samples, with the sub-sample fraction,
   * in the same units as {@code dsp_event_t::drop_sample}. Valid only when
   * {@link valid} is true.
   */
  double drop_sample = 0.0;
};

/** Fewest beats a side needs in a window for its shape to be used. */
inline constexpr uint32_t TICK_SHAPE_WINDOW_MIN_BEATS = 3u;

/** Landmarks of one closed window, per side (template side 0 and 1). */
struct tick_shape_window_t {
  bool valid[2] = {false, false}; /**< Lift found on the averaged shape. */
  uint32_t beats[2] = {0, 0};     /**< Beats in the side's average. */
  /**
   * Unlock-to-drop time on the averaged shape in source samples; meaningful
   * only where {@link valid} is true.
   */
  double lift_samples[2] = {0.0, 0.0};
};

/** Event counters since {@link tick_shape_reset}, for diagnostics. */
struct tick_shape_stats_t {
  uint32_t events = 0;   /**< Events passed to {@link tick_shape_measure}. */
  uint32_t timed = 0;    /**< Events that returned a valid time. */
  uint32_t learning = 0; /**< Events folded into a template still learning. */
  uint32_t low_quality = 0; /**< Events below the quality threshold. */
  uint32_t out_of_ring = 0; /**< Events whose window the ring no longer held. */
  uint32_t restarts = 0;    /**< Templates restarted after repeated misfits. */
};

/**
 * Designs the front end for a source rate: the high-pass coefficients, the
 * boxcar length, the decimation factor
 * {@code ceil(source_hz / 24500)} (so the envelope rate never exceeds
 * 24.5 kHz and the ring capacity holds) and the template geometry. Clears the
 * front end and the templates. A repeat call with the current rate does
 * nothing. A rate below 6 kHz leaves the module inactive.
 *
 * @param source_hz Analysis sample rate (Hz).
 * @see dsp_graph_set_source_rate
 */
void tick_shape_configure(uint32_t source_hz);

/**
 * Clears the front end (filters, ring), the templates and the counters.
 * Configuration and the enable flag stay.
 *
 * @see dsp_graph_reset_stream
 */
void tick_shape_reset();

/**
 * Discards the learned templates (another watch, another nominal rate, or a
 * changed clock) and keeps the front end running.
 */
void tick_shape_forget_templates();

/**
 * Enables or disables the per-sample front end. Enabling starts it from an
 * empty ring.
 *
 * @param on {@code true} while the drop landmark is in use.
 */
void tick_shape_set_enabled(bool on);

/** @return {@code true} while the front end runs ({@link tick_shape_push}). */
[[nodiscard]] bool tick_shape_enabled();

/**
 * Feeds one block of source samples to the front end. Does nothing when
 * disabled or unconfigured. A block that does not follow the previous one
 * ({@code base} jumps) restarts the front end. Per sample: 10 multiply-adds
 * for the two biquads, an absolute value, a running-sum update and, once per
 * decimation step, a conversion and a store.
 *
 * @param x Float full-scale samples (read-only).
 * @param n Sample count.
 * @param base Absolute source index of {@code x[0]}.
 */
void tick_shape_push(const float *x, uint32_t n, uint64_t base);

/**
 * Times one closed event. Call it from the event callback, while the ring
 * still holds the event (see the ring length table above). Bounded work: one
 * pass over about 440 ring samples to find the drop, one over about 780 to
 * copy the window, a coarse correlation (49 lags of 336 products), a fine one
 * (5 lags of 672 products) and, once per accepted beat, one template update
 * (672 samples).
 *
 * @param ev Closed event from the detector.
 * @param side Side (0 or 1) of the beat on the fit grid.
 * @param detect_side When true, choose the side from the templates already
 *        held (those with at least 4 beats) and use {@code side} only if none
 *        qualifies. Used on the first beat of a grid, where the parity of the
 *        grid slot says nothing about the side.
 * @return The timing result; {@code side} holds the side used.
 */
[[nodiscard]] tick_shape_result_t
tick_shape_measure(const dsp_event_t &ev, uint8_t side, bool detect_side);

/** @return Counters since the last {@link tick_shape_reset}. */
[[nodiscard]] tick_shape_stats_t tick_shape_stats();

/**
 * Adds the beat that the last {@link tick_shape_measure} timed (valid result)
 * to its side's accumulator of the open window, aligned on its fractional
 * drop time: the envelope window of that beat is shifted by the match lag and
 * its sub-sample part (linear interpolation, Q8) and summed. Call it right
 * after {@code tick_shape_measure}, only for a beat the caller accepts (the
 * fit took it); the scratch window the measure gathered is overwritten by the
 * next measure, and the call consumes it. Plain mean: OTG uses a per-sample
 * trimmed mean of the middle half, which needs every beat of the window
 * stored; the offline reference shows what that costs
 * ({@code reference_otg.py --trim 0}).
 *
 * Cost: one pass over the template span (686 samples at most, int32 only).
 *
 * @return {@code false} if there is no such beat (the last measure was not
 *         valid, or its beat was added already).
 */
bool tick_shape_window_add();

/**
 * Closes the open window without analysing it: new beats go to a fresh window
 * and the closed one waits for {@link tick_shape_window_step}. O(1). Does
 * nothing while a closed window is still pending (the caller finishes it
 * first, see {@link tick_shape_window_pending}).
 */
void tick_shape_window_seal();

/** @return {@code true} while a sealed window has not been fully analysed. */
[[nodiscard]] bool tick_shape_window_pending();

/**
 * Analyses one side of the sealed window: mean shape (one division per
 * sample), drop landmark, unlock landmark, lift. Called twice per window, so
 * each call is bounded: about 686 divisions, 2 passes of 686 for the drop
 * peak, 686 for the rise test and, for the peak search, at most about 240
 * smoothings of 9 samples plus the prominence of the first few maxima, a few
 * thousand integer operations in all. The caller runs it outside the event
 * path (one call per DSP block with no event) so that no landmark work shares
 * a call with a stage B match.
 *
 * @param out Receives the window when the call analysed the last side.
 * @return {@code true} when {@code out} is filled (window complete);
 *         {@code false} when the other side is still to do or nothing is
 *         pending.
 */
bool tick_shape_window_step(tick_shape_window_t *out);

/** Drops the open and the sealed window (a cut, or a new template set). */
void tick_shape_window_clear();
