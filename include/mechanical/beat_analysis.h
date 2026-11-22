/**
 * @file beat_analysis.h
 * @brief Mechanical escapement timing: lock, rate, beat error, amplitude and
 *        rolling averages from detected tick events ({@code beat_rate} block).
 *
 * Internal to the DSP graph: {@code dsp_graph.cpp} logs each closed event
 * from {@link tick_detect_block} and then hands it to
 * {@link beat_analysis_on_event}. Everything after that is owned here:
 *
 *   - unlock-to-unlock half-interval history (median → {@code vph_raw} and
 *     the nominal A/h lock, auto common table or manual {@code vph}),
 *   - the least-squares rate / beat-error window ({@link beat_fit_push}),
 *   - unlock→drop lift time → amplitude (Witschi {@code A = λ / (2 sin(π f
 *     t))}),
 *   - the rolling {@code avg_1s} / {@code avg_long} means,
 *   - the {@code beat_rate} block parameters ({@code liftAngle}, {@code vph},
 *     {@code calPpm}, {@code windowS}, {@code avgS}).
 *
 * The graph keeps ownership of the source sample rate, the block enable
 * flags and the event log, and passes what is needed per call. The module
 * reads {@code millis()} for the rolling-average clock and
 * {@link timebase_ppm} for the sample-clock correction, exactly as the code
 * did when it lived in {@code dsp_graph.cpp}.
 *
 * Not thread-safe: call only from the core that runs the DSP graph.
 *
 * @see dsp_graph.cpp, mechanical/tick_detect.h, mechanical/beat_fit.h,
 *      dsp_types.h ({@code dsp_beat_rate_t})
 */

#pragma once

#include "dsp_types.h"

#include <cstdint>

/**
 * Restores default {@code beat_rate} parameters (lift angle 51°, auto A/h,
 * 6 s fit window, 10 s long average), marks the timebase verified, and clears
 * all timing state via {@link beat_analysis_reset}.
 *
 * @see dsp_graph_init
 */
void beat_analysis_init();

/**
 * Clears per-stream timing state (interval and amplitude history, rate fit,
 * last unlock, published estimate, rolling averages) while keeping
 * parameters and the timebase flag.
 *
 * @see dsp_graph_reset_stream
 */
void beat_analysis_reset();

/**
 * Records whether the capture sample rate is verified. A change in either
 * direction discards intervals, the rate fit, the lock and the rolling
 * averages, because they were timed against a clock of unknown accuracy.
 * Setting the current value again is a no-op.
 *
 * @param ok {@code true} when the source rate is verified.
 * @see dsp_graph_set_timebase
 */
void beat_analysis_set_timebase(bool ok);

/**
 * Updates the timing estimate from one closed tick event.
 *
 * Intervals are unlock-to-unlock. With at least 4 half-intervals, the block
 * enabled and a verified timebase, the median interval gives {@code vph_raw}
 * and the nominal A/h lock. Once locked, the sub-sample unlock time joins the
 * least-squares window ({@link beat_fit_push}) whose slope gives rate (s/d)
 * and whose TIC/TAC term gives beat error (ms). Amplitude comes from the
 * unlock→drop lift time. Instant values then feed the rolling averages.
 *
 * @param ev Closed event window from the detector (already logged).
 * @param source_hz Analysis sample rate of {@code ev}'s sample indices (Hz).
 *        On the PCM1808 path this is the pre-upsample codec rate, not the
 *        48 kHz USB rate.
 * @param beat_rate_enabled State of the {@code beat_rate} block. When
 *        {@code false} intervals are still collected but no lock or timing
 *        result is produced.
 */
void beat_analysis_on_event(const dsp_event_t &ev, uint32_t source_hz,
                            bool beat_rate_enabled);

/**
 * Copies the current estimate into {@code out}, including the fields this
 * module owns: {@code intervals_held}, {@code avg_s}, {@code cal_ppm},
 * {@code cal_source} and {@code timebase_ok}. The caller fills the graph-owned
 * fields ({@code events_seen}, {@code source_hz}, {@code source_hz_measured}).
 *
 * @param out Destination snapshot; must not be {@code nullptr}.
 * @see dsp_graph_get_beat
 */
void beat_analysis_get(dsp_beat_rate_t *out);

/**
 * Sets one {@code beat_rate} parameter. Accepted names (camelCase and
 * snake_case aliases): {@code liftAngle} 10..90°, {@code vph} 0 (auto) or
 * 3600..72000, {@code calPpm} (forwarded to {@link timebase_set} as a manual
 * calibration), {@code windowS} 2..30 s, {@code avgS} 1..60 s.
 *
 * @param param Parameter name; {@code nullptr} is rejected.
 * @param value New value (caller has already rejected non-finite values).
 * @return {@code true} if the name is known and the value in range.
 * @see dsp_graph_set_param
 */
[[nodiscard]] bool beat_analysis_set_param(const char *param, float value);

/**
 * Reads one {@code beat_rate} parameter (same names as
 * {@link beat_analysis_set_param}).
 *
 * @param param Parameter name; {@code nullptr} is rejected.
 * @param out Receives the value; {@code nullptr} is rejected.
 * @return {@code true} if the name is known.
 * @see dsp_graph_get_param
 */
[[nodiscard]] bool beat_analysis_get_param(const char *param, float *out);

/**
 * Seq the next recorded beat will get; equals the number of beats recorded
 * since the last stream reset (never decreases except at reset). A HID
 * subscriber stores this as its baseline and later asks for beats with
 * {@code seq >=} baseline.
 *
 * Beat records are produced one per tick handed to {@link beat_fit_push}
 * (i.e. only while locked); see {@link dsp_beat_t} for the field semantics
 * (u sign, timebase correction, REJECTED handling). The 64-entry ring is
 * owned here rather than by the graph because this module owns everything
 * that determines a record (grid id, fit point, amplitude, timebase) and its
 * reset; the graph merely forwards, as it does for beat_rate.
 *
 * Thread safety: single writer (DSP core); readers on the other core are
 * lock-free: the record is written before the counter is published with
 * release semantics, and the copy functions discard entries the writer may
 * have lapped mid-copy.
 *
 * @return Next seq.
 */
uint32_t beat_analysis_beat_seq();

/**
 * Copies recorded beats with {@code seq >= seq}, oldest first.
 *
 * @param seq Baseline; older beats than the ring still holds are skipped.
 * @param out Destination; {@code nullptr} yields 0.
 * @param max_n Capacity of {@code out}.
 * @return Number written (the oldest {@code max_n} qualifying beats).
 */
uint32_t beat_analysis_copy_beats_since(uint32_t seq, dsp_beat_t *out,
                                        uint32_t max_n);

/**
 * Copies up to {@code max_n} most recent beats, newest first.
 *
 * @return Number written.
 */
uint32_t beat_analysis_copy_beats(dsp_beat_t *out, uint32_t max_n);
