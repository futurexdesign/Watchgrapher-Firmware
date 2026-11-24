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
 *   - open when {@code |x| > thr_open},
 *     with the window start {@code preroll_ms} before the onset,
 *   - track peak, energy, and three-point local maxima ("pulses") with a
 *     ~0.75 ms refractory and a parabolic sub-sample vertex,
 *   - close after {@code tail_ms} below the hysteresis threshold
 *     ({@code 0.35 × thr_open}) or after 80 ms, dropping windows whose peak is
 *     below {@code max(thr_open, 0.25 × tick peak EMA)},
 *   - on close: unlock = first pulse, drop = loudest pulse, lift = drop −
 *     unlock, then report the event through the callback.
 *
 * Not thread-safe: call only from the core that runs the DSP graph.
 *
 * @see dsp_graph.cpp, mechanical/beat_analysis.h, dsp_types.h
 *      ({@code dsp_event_t})
 */

#pragma once

#include "dsp_types.h"

#include <cstdint>

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
};

/**
 * Close callback. Called once per accepted event, from inside
 * {@link tick_detect_block}. {@code ev.confidence} is
 * {@code min(1, peak / thr_open / 4)}.
 *
 * @param ev Closed event with unlock / drop landmarks filled in.
 */
using tick_detect_event_fn = void (*)(const dsp_event_t &ev);

/**
 * Clears the detector state machine, pulse list and the tick peak EMA (back
 * to 0.05 FS). Parameters live in the graph and are not touched.
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
