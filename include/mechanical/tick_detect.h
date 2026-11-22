/**
 * @file tick_detect.h
 * @brief Mechanical tick detector: event windows and Witschi unlock / drop
 *        landmarks ({@code event_detect} block).
 *
 * Internal to the DSP graph. {@code dsp_graph.cpp} computes the open
 * threshold from its noise floor and pre-RMS meter (those are shared with the
 * soft gate and noise-floor blocks, so they stay in the graph), then calls
 * {@link tick_detect_block} once per analysis block. The detector owns the
 * per-sample state machine:
 *
 *   - open when {@code |x| > thr_open} (unless vetoed by the matched filter),
 *     with the window start {@code preroll_ms} before the onset,
 *   - track peak, energy, and three-point local maxima ("pulses") with a
 *     ~0.75 ms refractory and a parabolic sub-sample vertex,
 *   - close after {@code tail_ms} below the hysteresis threshold
 *     ({@code 0.35 × thr_open}) or after 80 ms, dropping windows whose peak is
 *     below {@code max(thr_open, 0.25 × tick peak EMA)},
 *   - on close: unlock = first pulse, drop = loudest pulse, lift = drop −
 *     unlock, then report the event through the callback.
 *
 * The first {@link TICK_DETECT_WAVE_LEN} raw samples of the open window are
 * kept so the graph's matched filter can learn a template from them inside
 * the close callback ({@link tick_detect_wave}).
 *
 * Not thread-safe: call only from the core that runs the DSP graph.
 *
 * @see dsp_graph.cpp, mechanical/beat_analysis.h, dsp_types.h
 *      ({@code dsp_event_t})
 */

#pragma once

#include "dsp_types.h"

#include <cstdint>

/** Raw samples of each open window kept for matched-filter learning. */
inline constexpr uint32_t TICK_DETECT_WAVE_LEN = 64u;

/**
 * Per-block inputs from the graph. Every field is constant for the duration
 * of one {@link tick_detect_block} call.
 */
struct tick_detect_block_params_t {
  float thr_open;     /**< Open threshold, float full-scale. */
  float noise_floor;  /**< Noise EMA, copied into each event (and its SNR). */
  float preroll_ms;   /**< Window start before the onset sample (ms). */
  float tail_ms;      /**< Time below the close threshold that ends it (ms). */
  uint32_t source_hz; /**< Analysis sample rate (Hz). */
  /**
   * Matched-filter veto: when {@code true} no new window may open in this
   * block (filter enabled, template learned, block score below threshold).
   */
  bool open_veto;
};

/**
 * Close callback. Called once per accepted event, from inside
 * {@link tick_detect_block}, while {@link tick_detect_wave} still holds that
 * event's samples. {@code ev.confidence} is the detector's own
 * {@code min(1, peak / thr_open / 4)}; the graph blends in the matched-filter
 * score before logging.
 *
 * @param ev Closed event with unlock / drop landmarks filled in.
 */
using tick_detect_event_fn = void (*)(const dsp_event_t &ev);

/**
 * Clears the detector state machine, captured wave, pulse list and the tick
 * peak EMA (back to 0.05 FS). Parameters live in the graph and are not
 * touched.
 *
 * @see dsp_graph_init, dsp_graph_reset_stream
 */
void tick_detect_reset();

/**
 * @return {@code true} while an event window is open. The graph's noise
 *         floor only adapts when this is {@code false}.
 */
[[nodiscard]] bool tick_detect_in_event();

/**
 * Runs the detector over one analysis block.
 *
 * @param x Float full-scale samples (read-only).
 * @param n Sample count.
 * @param base Absolute source index of {@code x[0]}.
 * @param params Thresholds and timing for this block.
 * @param on_event Called for each accepted event, in sample order; must not
 *        be {@code nullptr}.
 */
void tick_detect_block(const float *x, uint32_t n, uint64_t base,
                       const tick_detect_block_params_t &params,
                       tick_detect_event_fn on_event);

/**
 * Raw samples captured from the start of the current window (onset sample
 * first). Valid inside the {@link tick_detect_event_fn} callback; reset to
 * zero length when the window closes.
 *
 * @param n Receives the sample count (0..{@link TICK_DETECT_WAVE_LEN}); must
 *        not be {@code nullptr}.
 * @return Pointer to the internal buffer of {@link TICK_DETECT_WAVE_LEN}
 *         floats.
 */
[[nodiscard]] const float *tick_detect_wave(uint32_t *n);
