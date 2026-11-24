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
 *   - cuts and the no-watch rule (item 4 of
 *     {@code docs/plans/TIMING_ENGINE_PLAN.md}),
 *   - the {@code beat_rate} block parameters ({@code liftAngle}, {@code vph},
 *     {@code calPpm}, {@code windowS}, {@code avgS}, {@code timingLandmark},
 *     {@code settleS}).
 *
 * <b>Stretches and cuts.</b> A stretch is a run of accepted beats. A cut ends
 * it, and then the fit, the rolling means, the amplitude history and the
 * no-watch window start empty (the stage-B templates only when the nominal
 * rate changed). The reasons are {@link dsp_cut_reason_t}: more than 2 s
 * without an accepted beat, a timing jump (the fit's consecutive-reject
 * restart), a nominal rate change, a change of the timebase correction of more
 * than 0.5 ppm (or of its verification), a lost lock, the no-watch rule, and
 * the stream reset. A stretch gives no rate or beat error until it spans 5 s;
 * the readings are provisional until it spans 10 s ({@code settled}).
 *
 * <b>No watch.</b> Over the last 10 s of a stretch of at least 5 s the rule
 * fires if fewer than half the expected beats were accepted or the RMS scatter
 * of the events about the fit (the worst 1 in 8 left out) is at least 2 % of
 * the half-period. It drops the lock and cuts the stretch; the state is then
 * {@link DSP_BEAT_STATE_NO_WATCH} until a lock holds a stretch of 2 s.
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
 * Daily rate from the fitted and the nominal half-period, both in seconds.
 * The watch is fast when the measured half-period is shorter than nominal,
 * which gives a positive result. Form: {@code (P_nom / P_true − 1)·86400}
 * (opentimegrapher M2). Compared with {@code (1 − P_true / P_nom)·86400}
 * it differs by about 0.005 s/d at ±20 s/d.
 *
 * @param nominal_half_s Nominal half-period (s), must be greater than 0.
 * @param true_half_s Fitted half-period (s), must be greater than 0.
 * @return Rate in seconds per day, + = gaining.
 */
[[nodiscard]] inline double beat_rate_s_day(double nominal_half_s,
                                            double true_half_s) {
  return (nominal_half_s / true_half_s - 1.0) * 86400.0;
}

/**
 * Beat-error angle in degrees: {@code amp·π·vph / 7200 · be_s}, where
 * {@code vph} is the nominal A/h and {@code be_s} the beat error in seconds
 * (opentimegrapher M5). It puts beat errors of different amplitudes on one
 * scale. The result is valid only when both {@code amp_deg} and {@code be_ms}
 * are valid; the caller checks that. Returns 0 when {@code vph} is 0.
 *
 * @param amp_deg Balance amplitude (degrees).
 * @param vph Nominal vibrations per hour (A/h) the amplitude was measured at.
 * @param be_ms Beat error (ms), as in {@code beatErrorMs}.
 * @return Beat-error angle in degrees.
 */
[[nodiscard]] inline float beat_error_angle_deg(float amp_deg, uint32_t vph,
                                                float be_ms) {
  constexpr double kPiD = 3.14159265358979323846;
  return static_cast<float>(static_cast<double>(amp_deg) * kPiD *
                            static_cast<double>(vph) / 7200.0 *
                            (static_cast<double>(be_ms) / 1000.0));
}

/**
 * Restores default {@code beat_rate} parameters (lift angle 51°, auto A/h,
 * 6 s fit window, 10 s long average, unlock landmark, no settling), marks the
 * timebase verified, and clears all timing state via
 * {@link beat_analysis_reset}.
 *
 * @see dsp_graph_init
 */
void beat_analysis_init();

/**
 * Switches the timing engine on or off (USB microphone mode is off). Off, an
 * event only feeds the interval history and the A/h lock
 * ({@code locked}, {@code vph_raw}, {@code vph_nearest}, {@code confidence}):
 * no fit, stretch, cut, rolling mean, amplitude, no-watch rule or beat
 * record, and stage B stays off whatever {@code timingLandmark} says. A
 * change clears all timing state ({@link beat_analysis_reset}); the stage B
 * front end follows {@code timingLandmark} again when the engine is switched
 * on. Setting the current value does nothing.
 *
 * @param on {@code true} to run the engine (default after
 *        {@link beat_analysis_init}).
 * @see dsp_graph_set_analysis
 */
void beat_analysis_set_enabled(bool on);

/** @return {@code true} while the timing engine runs (default). */
[[nodiscard]] bool beat_analysis_enabled();

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
 * Idle step of the shape windows, called by the graph after each analysed
 * block. A block in which an event was handled does nothing, so landmark work
 * never shares a block with a stage B match. Otherwise one bounded step: the
 * landmarks of one side of a sealed window ({@link tick_shape_window_step}),
 * the storing of the finished window, or the refresh of the window readings
 * (medians over at most 32 windows). A window needs four such blocks (about
 * 20 ms at 48 kHz), well inside the 60 ms minimum spacing of events.
 */
void beat_analysis_on_block_end();

/**
 * Copies the current estimate into {@code out}, including the fields this
 * module owns: {@code intervals_held}, {@code avg_s}, {@code cal_ppm},
 * {@code cal_source}, {@code timebase_ok} and {@code state} (the other cut
 * fields {@code cuts}, {@code last_cut} and {@code stretch_s} are kept as
 * part of the estimate). The caller fills the graph-owned
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
 * calibration), {@code windowS} 2..30 s, {@code avgS} 1..60 s,
 * {@code timingLandmark} 0 or 1 and {@code settleS} 0..60 s (seconds after a
 * cut before samples enter the rolling means).
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
