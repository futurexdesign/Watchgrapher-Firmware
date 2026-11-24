/**
 * @file quartz_engine.h
 * @brief Firmware-side owner of quartz-watch timing: mode, gating,
 *        parameters, the one active pipeline and the control-plane snapshot.
 *
 * The quartz library (motor_timing.h, lcd_timing.h, tone_timing.h) holds the
 * algorithms; this module decides which of them runs, on which channel, when
 * it may run and what the control plane sees:
 *
 *   - mode {@code mechanical | motor | lcd | fork}. One static
 *     {@code std::variant<std::monostate, MotorTiming, LcdTiming, ToneTiming>}
 *     holds the active pipeline and is switched with {@code emplace<>()};
 *     there is no heap. {@code mechanical} keeps the monostate and leaves the
 *     beat_rate path untouched;
 *   - channels: motor and fork read LEFT (inductive), LCD reads RIGHT
 *     (capacitive), as raw sign-extended 24-bit codes;
 *   - analysis switch ({@link set_analysis}): off in USB microphone mode, where
 *     the host does the timing; nothing is measured and the snapshot says so;
 *   - gating: nothing is measured until the capture rate is verified AND is
 *     {@link quartz::engine::kRateHz}; the snapshot says why it waits;
 *   - resets: stream reset, source-rate change, timebase-verified change,
 *     mode change, a parameter change that affects the active mode, an
 *     explicit {@link quartz::engine::restart}, captured sample loss, and a
 *     gap in the source index all start a new measurement;
 *   - motor gate: a time in seconds, or 0 = "until two inhibition events";
 *     reaching it freezes the result ({@code DSP_QUARTZ_STATE_DONE}), but
 *     never while the incremental fit is still rebuilding;
 *   - every (re)start gets a new {@code measurement_id}, which clients use
 *     to detect a new measurement;
 *   - calibration: rates are produced against the board clock and the
 *     sample-clock correction is removed with
 *     {@link quartz::apply_timebase_ppm}; results are flagged uncalibrated
 *     when no calibration exists.
 *
 * Host-testable: no Arduino / Pico SDK. The firmware injects the
 * calibration source, a microsecond clock and the per-block timing sink
 * through {@link quartz::engine::Hooks}; a host test injects fakes.
 *
 * Threading: every function must be called from the core that runs the DSP
 * graph (core 0). Core 1 reads only {@code ui_snapshot}.
 *
 * @see quartz_engine.cpp, dsp_quartz_types.h, control_api.h,
 *      tests/test_quartz_engine.cpp, docs/handoff/FIRMWARE_HANDOFF.md
 */

#pragma once

#include "dsp_quartz_types.h"

#include <cstddef>
#include <cstdint>

namespace quartz::engine {

/**
 * Only capture rate the engine measures at (Hz). The library supports up to
 * 96 kHz, but every acceptance vector of the handoff (§9) except the crystal
 * one is 48 kHz, and the board is strapped for 48 kHz, so that is the rate
 * the motor geometry, LCD lags and fork folds are validated at.
 */
inline constexpr uint32_t kRateHz = 48000u;

/** Motor gate value meaning "until two inhibition events" (handoff §4). */
inline constexpr float kGateUntilEvents = 0.f;

/** Shortest gate in seconds: three pulses at the common 1 s step, the
 * minimum for a provisional rate (handoff §4). */
inline constexpr float kGateMinS = 3.f;

/** Longest gate in seconds: the longest inhibition period in the
 * specification ({@code signal_spec.yaml}, handoff §3.2). */
inline constexpr float kGateMaxS = 960.f;

/** Smallest accepted {@code kSigma}: at 4 sigma, Gaussian noise alone
 * already crosses the threshold about 3 times per second at 48 kHz
 * (P(|x| > 4σ) ≈ 6.3e-5); lower values only produce false pulses. */
inline constexpr float kKSigmaMin = 4.f;

/** Largest accepted {@code kSigma}: at about 0.52 sigma per µV of EMF (see
 * {@link kWeakSnr}), 20 sigma still detects pulses from about 40 µV up,
 * which covers the typical 50-300 µV range (handoff §3.1). A higher
 * threshold would reject typical pulses that time well. */
inline constexpr float kKSigmaMax = 20.f;

/**
 * Peak SNR below which the motor result carries the weak-signal hint.
 *
 * Handoff §8 sets the limit at about 30 µV pickup EMF (per-pulse jitter
 * above 2 µs). Analytic upper bound: §2 / §3.1 give, with JP1 bridged
 * (×11), 6080 codes at the ADC per µV of EMF and 10 215 codes rms of LEFT
 * noise, so a 30 µV pulse peaks at 30 · 6080 / 10 215 ≈ 17.9 sigmas; the
 * input-referred noise gives the same (30 µV / 1.7 µV ≈ 17.6), so the
 * ratio also holds with JP1 open, where gain and noise drop together.
 *
 * What the detector actually reports is lower, because the leading spike
 * rises in under one sample: the largest sample after the 400 Hz high-pass
 * falls 12-14 % below the simulated analog peak, while sigma (measured after
 * the same high-pass, ~9 900 codes) barely moves. On the handoff vectors the
 * median {@code peak / sigma} is 0.52 per µV of EMF: 9.6 at 20 µV, 15.8 at
 * 30 µV (ind_inhibition_60s, residual 1.65 µs rms), 25.3 at 50 µV and 51.5
 * at 100 µV. The limit therefore sits at 16 in the detector's own units
 * (tests/test_quartz_engine.cpp checks 20 µV is flagged and 100 µV is not).
 */
inline constexpr float kWeakSnr = 16.f;

/**
 * Longest inhibition period the user is expected to have (s): the motor fit's
 * {@code longest_inhibition_period_s}. 60 s covers the common 10 s and 60 s
 * trims; the specification allows up to {@link kGateMaxS}, which the "until
 * two events" gate still catches. {@code motor_config()} configures exactly
 * this value, and it equals the library default (the reference's).
 */
inline constexpr double kLongestInhibitionPeriodS = 60.0;

/** Step-grid span (s) with no jump after which the motor status becomes
 * "no inhibition detected": twice {@link kLongestInhibitionPeriodS}
 * (handoff §5). The fit's rule is
 * {@code (n_last - n_first + 1) · step >= 2 · longest_inhibition_period_s},
 * so it counts steps covered (missed pulses included), not the
 * first-to-last timestamp span, which is one step shorter. */
inline constexpr double kNoJumpSpanS = 2.0 * kLongestInhibitionPeriodS;

/**
 * Interval excess (µs) that marks an inhibition event: the motor fit's
 * {@code jump_threshold_us}. The smallest real jump is 61 µs and timing noise
 * is at most ~2.6 µs rms (handoff §4). {@code motor_config()} configures
 * exactly this value, and it equals the library default.
 */
inline constexpr double kJumpThresholdUs = 20.0;

/** Default fork frequency (Hz): 360 Hz, the Accutron 214 / most common
 * tuning-fork movement; one of {@link quartz::kForkFrequencies}. */
inline constexpr float kDefaultForkHz = 360.f;

/** Sample-clock correction in effect, as reported by the injected source. */
struct Calibration {
  float ppm = 0.f;             /**< Correction (ppm, + = board fast). */
  const char *source = "none"; /**< Static source name, never null. */
  bool calibrated = false;     /**< False when the source is "none". */
};

/**
 * Platform services the engine needs. Every member may be null: a null
 * calibration means "none, 0 ppm"; a null clock or sink disables the
 * per-block timing report.
 */
struct Hooks {
  /** @return The correction in effect now. Called when results are read
   *          and once when a motor gate freezes its result. */
  Calibration (*calibration)() noexcept = nullptr;
  /** @return A free-running microsecond counter (wraps at 2^32). */
  uint32_t (*now_us)() noexcept = nullptr;
  /** Receives the wall time of one {@link process} call (µs). */
  void (*block_timed)(uint32_t us) noexcept = nullptr;
};

/**
 * Installs the hooks, restores default parameters (gate 0, fork 360 Hz,
 * threshold timestamps, kSigma 8), selects mechanical mode, marks the
 * timebase unverified and the source rate unknown.
 *
 * Unlike beat_analysis, the timebase defaults to unverified: the engine only
 * runs on the PCM1808 path, whose capture init always reports the state, and
 * failing closed means a quartz rate can never come from a guessed clock.
 *
 * @param hooks Platform services (copied).
 */
void init(const Hooks &hooks) noexcept;

/**
 * Starts a new measurement at the next block (capture restart).
 *
 * @see dsp_graph_reset_stream
 */
void reset_stream() noexcept;

/**
 * Records the analysis sample rate. A change restarts the measurement;
 * setting the current value again is a no-op.
 *
 * @param hz Source rate (Hz); 0 = unknown.
 * @see dsp_graph_set_source_rate
 */
void set_source_rate(uint32_t hz) noexcept;

/**
 * Records whether the capture rate is verified. A change in either
 * direction restarts the measurement; setting the current value again is a
 * no-op (the capture reports it every measurement window).
 *
 * @param ok {@code true} when the source rate is verified.
 * @see dsp_graph_set_timebase
 */
void set_timebase(bool ok) noexcept;

/**
 * Reports that the capture lost samples (a DMA buffer was overwritten
 * before it was read). Source indices then no longer match signal time, so
 * a running quartz measurement restarts; a frozen one is kept.
 */
void notify_sample_loss() noexcept;

/**
 * Turns the engine on or off. Off (USB microphone mode): {@link process}
 * returns at once, {@link get} reports {@code analysis} false with state
 * {@code IDLE}, and the mode and parameters are kept. A change in either
 * direction starts a new measurement ({@code measurement_id} increases), so
 * a result from before the switch is never shown afterwards. Default on.
 *
 * @param on {@code false} to stop measuring.
 * @see dsp_graph_set_analysis
 */
void set_analysis(bool on) noexcept;

/** @return {@code true} while the engine measures (default). */
[[nodiscard]] bool analysis() noexcept;

/**
 * Feeds one capture block. Call once per DMA block, after the USB samples
 * have been queued, with the same absolute index the DSP graph receives.
 *
 * Does nothing in mechanical mode, while analysis is off
 * ({@link set_analysis}), while gated, or once a motor result is
 * frozen. After a timed gate is reached no more samples are fed, but the
 * motor fit may still finish a rebuild over the next blocks before the
 * result freezes. A source index that does not follow the previous block
 * restarts the measurement. Reports its wall time through {@link
 * Hooks::block_timed} on every call except the one skipped while analysis is
 * off.
 *
 * @param left LEFT codes, sign-extended 24-bit (motor, fork); may be null
 *        in LCD mode.
 * @param right RIGHT codes, sign-extended 24-bit (LCD); may be null in
 *        motor and fork modes.
 * @param n Frames in the block.
 * @param source_index Absolute source index of frame 0.
 */
void process(const int32_t *left, const int32_t *right, std::size_t n,
             uint64_t source_index) noexcept;

/**
 * Selects the mode. A different mode emplaces its pipeline (or the
 * monostate) and starts a new measurement; the current mode again is a
 * no-op (use {@link restart}).
 *
 * @param mode New mode.
 * @return {@code false} if {@code mode} is out of range (nothing changes).
 */
[[nodiscard]] bool set_mode(dsp_quartz_mode_t mode) noexcept;

/** @return The active mode. */
[[nodiscard]] dsp_quartz_mode_t mode() noexcept;

/**
 * Starts a new measurement in the active mode (also leaves
 * {@code DSP_QUARTZ_STATE_DONE}).
 *
 * @return {@code false} in mechanical mode (nothing to restart).
 */
[[nodiscard]] bool restart() noexcept;

/**
 * Sets one parameter. Names (camelCase, with snake_case aliases):
 *  - {@code gate} (motor): 0 = until two inhibition events, or seconds in
 *    [{@link kGateMinS}, {@link kGateMaxS}] (10 and 60 are the usual);
 *  - {@code forkHz} / {@code fork_hz}: one of
 *    {@link quartz::kForkFrequencies} within 0.01 Hz (341.33 selects the
 *    exact 1024/3 Hz);
 *  - {@code tsMethod} / {@code ts_method}: 0 = threshold, 1 = xcorr
 *    ({@code dsp_quartz_ts_method_t});
 *  - {@code kSigma} / {@code k_sigma}: [{@link kKSigmaMin},
 *    {@link kKSigmaMax}].
 *
 * A change to a parameter the active mode uses restarts its measurement; a
 * parameter of another mode is only stored.
 *
 * @param param Name; {@code nullptr} is rejected.
 * @param value New value; non-finite values are rejected.
 * @return {@code true} if the name is known and the value accepted.
 */
[[nodiscard]] bool set_param(const char *param, float value) noexcept;

/**
 * {@link set_param} with the reason for a refusal.
 *
 * @param param Name; {@code nullptr} gives {@code UNKNOWN_NAME}.
 * @param value New value; a non-finite value of a known name gives
 *        {@code OUT_OF_RANGE}.
 * @return {@code DSP_QUARTZ_PARAM_OK}, {@code UNKNOWN_NAME} or
 *         {@code OUT_OF_RANGE} (never {@code NO_ENGINE}).
 */
[[nodiscard]] dsp_quartz_param_result_t set_param_ex(const char *param,
                                                     float value) noexcept;

/**
 * Reads one parameter (same names as {@link set_param}).
 *
 * @param param Name; {@code nullptr} is rejected.
 * @param out Receives the value; {@code nullptr} is rejected.
 * @return {@code true} if the name is known.
 */
[[nodiscard]] bool get_param(const char *param, float *out) noexcept;

/**
 * Fills the control-plane snapshot. Cheap: the motor fit is solved when
 * pulses arrive, not here.
 *
 * @param out Destination; {@code nullptr} is ignored.
 */
void get(dsp_quartz_t *out) noexcept;

/**
 * Copies recent motor pulses, newest first. Empty outside motor mode.
 *
 * @param out Destination array of at least {@code max_n} entries.
 * @param max_n Capacity of {@code out}.
 * @return Entries written (at most {@code DSP_QUARTZ_PULSE_LOG_CAP}).
 */
[[nodiscard]] uint32_t copy_pulses(dsp_quartz_pulse_t *out,
                                   uint32_t max_n) noexcept;

/**
 * @param mode Mode.
 * @return Its stable name, or {@code "unknown"} out of range.
 */
[[nodiscard]] const char *mode_name(dsp_quartz_mode_t mode) noexcept;

/**
 * Case-insensitive lookup of a mode name.
 *
 * @param name Name; {@code nullptr} gives {@code DSP_QUARTZ_MODE_COUNT}.
 * @return The mode, or {@code DSP_QUARTZ_MODE_COUNT} if unknown.
 */
[[nodiscard]] dsp_quartz_mode_t mode_from_name(const char *name) noexcept;

/** @param s State. @return Its stable camelCase name, or "unknown". */
[[nodiscard]] const char *state_name(dsp_quartz_state_t s) noexcept;

/** @param s Status. @return Its stable snake_case name, or "unknown". */
[[nodiscard]] const char *inhibition_name(dsp_quartz_inhibition_t s) noexcept;

/** @param m Method. @return "threshold" / "xcorr", or "unknown". */
[[nodiscard]] const char *ts_method_name(dsp_quartz_ts_method_t m) noexcept;

/**
 * Case-insensitive lookup of a timestamp-method name.
 *
 * @param name Name; {@code nullptr} gives {@code DSP_QUARTZ_TS_COUNT}.
 * @return The method, or {@code DSP_QUARTZ_TS_COUNT} if unknown.
 */
[[nodiscard]] dsp_quartz_ts_method_t
ts_method_from_name(const char *name) noexcept;

/** @param r Result. @return Its stable camelCase name, or "unknown". */
[[nodiscard]] const char *
param_result_name(dsp_quartz_param_result_t r) noexcept;

/** @param s Status. @return The library's label, or "unknown". */
[[nodiscard]] const char *lcd_status_name(dsp_quartz_lcd_status_t s) noexcept;

} // namespace quartz::engine
