/**
 * @file quartz_types.h
 * @brief Result structs, status enums and rate helpers shared by the quartz
 *        timing library (stepper motor, LCD, tuning fork, crystal).
 *
 * Everything here is plain data plus a few {@code constexpr} helpers, so the
 * firmware glue, the CLI/HID formatters and the host tests can all include it
 * without pulling in any DSP code.
 *
 * Units used throughout the quartz library:
 *  - sample positions are absolute indices counted from the stream origin
 *    given to {@code reset()}, held as {@code uint64_t} (whole samples) or
 *    {@code double} (sub-sample positions; a double resolves 1e-6 sample up
 *    to ~2^33 samples, i.e. ~2 days at 48 kHz, which is far finer than the
 *    1e-3 sample the timestamps carry);
 *  - rates are seconds per day, positive = watch gains (runs fast), exactly
 *    as {@code rate_s_per_day()} in {@code docs/handoff/ref/timing_ref.py};
 *  - all rates are in the board-clock frame. The caller removes the board
 *    clock error with {@link quartz::apply_timebase_ppm}.
 *
 * Pure C++17, no Arduino / Pico SDK, so it is host-testable.
 *
 * @see motor_timing.h, lcd_timing.h, tone_timing.h,
 *      docs/handoff/FIRMWARE_HANDOFF.md
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace quartz {

/** Seconds per day: the scale factor from fractional error to s/d. */
inline constexpr double kSecondsPerDay = 86400.0;

/**
 * Raw code magnitude that counts as clipped. The reference harness
 * ({@code run_tests.py}) flags {@code |x| >= CODE_FS - 1} with
 * {@code CODE_FS = 2^23}, i.e. both the positive rail (8 388 607) and the
 * negative rail (-8 388 608) count.
 */
inline constexpr int32_t kClipCode = 8388607;

/** Highest sample rate any fixed-capacity buffer in the library is sized for
 * (the 96 kHz crystal mode). Configuring a higher rate is refused. */
inline constexpr uint32_t kMaxSampleRateHz = 96000u;

/** Samples converted and filtered per internal step. Callers may pass any
 * block length; the firmware passes 240-frame DMA blocks. */
inline constexpr std::size_t kChunk = 256u;

/** Delay added to the next motor step by one oscillator pulse removed at the
 * 8192 Hz divider stage (µs): 1e6 / 8192. */
inline constexpr double kInhibitionQuantumUs = 1e6 / 8192.0;

/**
 * A frequency in Hz as an exact fraction {@code num / den}.
 *
 * The phase-slope methods reference every block to the absolute sample index
 * with integer modular arithmetic, which needs the ratio {@code f / fs} as an
 * exact fraction. Most nominal frequencies are integers ({@code den = 1}),
 * but the 341.33 Hz tuning fork is 32768 / 96 = 1024 / 3 Hz and must not be
 * rounded: 341.33 instead of 1024/3 would read as -9.8 ppm (-0.84 s/d).
 */
struct Rational {
  uint32_t num = 0; /**< Numerator (Hz times {@code den}). */
  uint32_t den = 1; /**< Denominator, at least 1. */

  /** @return The frequency in Hz as a double (for display and for the final
   *          scaling of a phase slope, never for phase bookkeeping). */
  [[nodiscard]] constexpr double hz() const noexcept {
    return static_cast<double>(num) / static_cast<double>(den);
  }
};

/** Tuning-fork nominal frequencies (Hz) from {@code signal_spec.yaml}
 * ({@code signals.tuning_fork.frequencies_hz}); 341.33 is exactly 1024/3. */
inline constexpr std::array<Rational, 5> kForkFrequencies = {
    {{300u, 1u}, {1024u, 3u}, {360u, 1u}, {480u, 1u}, {720u, 1u}}};

/** Crystal nominal frequency (Hz). Needs fs = 96 kHz. */
inline constexpr Rational kCrystalFrequency{32768u, 1u};

/**
 * Rate from a nominal and a measured period, ported from
 * {@code rate_s_per_day()} in {@code timing_ref.py}.
 *
 * @param nominal_period Nominal period (any unit).
 * @param measured_period Measured period (same unit), non-zero.
 * @return Rate in s/d, positive = watch gains (shorter measured period).
 */
[[nodiscard]] constexpr double rate_s_per_day(double nominal_period,
                                              double measured_period) noexcept {
  return kSecondsPerDay * (nominal_period / measured_period - 1.0);
}

/**
 * Removes the board sample-clock error from a rate measured in board-clock
 * units.
 *
 * Sign convention (same as {@code timebase_ppm()} and the beat_rate block in
 * {@code dsp_graph.cpp}): {@code +ppm} means the board clock is fast, i.e. it
 * counts {@code (1 + ppm·1e-6)·fs} samples per true second. A period measured
 * in board seconds is then {@code (1 + ppm·1e-6)} times too long and a
 * frequency the same factor too low, so both rate forms correct the same way:
 *
 *   rate_true = (rate_board + 86400) · (1 + ppm·1e-6) - 86400
 *
 * A board fast by 1 ppm makes an exact watch read -0.0864 s/d; this adds it
 * back.
 *
 * @param rate_board_s_per_day Rate measured against the board clock (s/d).
 * @param ppm Board clock error, + = fast.
 * @return Rate against true time (s/d).
 */
[[nodiscard]] constexpr double apply_timebase_ppm(double rate_board_s_per_day,
                                                  double ppm) noexcept {
  return (rate_board_s_per_day + kSecondsPerDay) * (1.0 + 1e-6 * ppm) -
         kSecondsPerDay;
}

/** How a motor pulse's leading edge is timestamped (handoff §4). */
enum class TimestampMethod : uint8_t {
  /** 50 % crossing of the leading spike, linearly interpolated: handoff
   * method (b), {@code timestamp_pulses(method="threshold")}. Cheap; the
   * default. */
  kThreshold = 0,
  /** Band-limited cross-correlation against a template fixed after the first
   * four pulses: handoff method (a),
   * {@code timestamp_pulses(method="xcorr")}. */
  kXcorr,
};

/**
 * What the motor result can claim about inhibition, the four rows of the
 * handoff §5 table plus one transient state.
 */
enum class InhibitionStatus : uint8_t {
  /** No jump seen, and the pulses cover at least twice the longest
   * expected inhibition period, counted as (n_last - n_first + 1) step
   * intervals: report the rate as "no inhibition detected". */
  kNoJumpLongGate = 0,
  /** No jump seen in a short gate: raw rate, flagged "may include
   * inhibition"; offer a longer gate. */
  kMayIncludeInhibition,
  /** One jump seen: raw rate plus "need one more cycle". */
  kNeedOneMoreCycle,
  /** Two or more jumps, but no pulse yet after the last jump with the same
   * polarity as the first pulse of the first cycle (the reference would
   * raise IndexError here). Resolves on the next pulse. Not in §5; it is a
   * streaming-only state. */
  kCorrectionPending,
  /** Two or more jumps: the corrected rate (with inhibition) is the headline;
   * raw rate, period and jump size are details. */
  kCorrected,
};

/**
 * @param s Status.
 * @return A short, stable, lower-case label for CLI / HID / logs.
 */
[[nodiscard]] constexpr const char *to_string(InhibitionStatus s) noexcept {
  switch (s) {
  case InhibitionStatus::kNoJumpLongGate:
    return "no_jump_long_gate";
  case InhibitionStatus::kMayIncludeInhibition:
    return "may_include_inhibition";
  case InhibitionStatus::kNeedOneMoreCycle:
    return "need_one_more_cycle";
  case InhibitionStatus::kCorrectionPending:
    return "correction_pending";
  case InhibitionStatus::kCorrected:
    return "corrected";
  }
  return "?";
}

/**
 * Output of the segmented least-squares motor fit, the port of
 * {@code motor_rate()} in {@code timing_ref.py}. All rates are in the
 * board-clock frame.
 */
struct MotorRate {
  double step_s = 0.0; /**< Nominal step interval, snapped to the allowed
                            list (or the configured override). */
  uint32_t pulses = 0; /**< Pulses in the fit. */
  double span_s = 0.0; /**< Time from the first to the last pulse (s). */
  /** Raw oscillator rate: the common slope {@code b} of
   * {@code t = a[seg] + b·n + c·pol} (s/d). */
  double rate_without_inhibition_s_per_day = 0.0;
  /** Rate over whole inhibition cycles when {@link status} is
   * {@code kCorrected}; otherwise equal to the raw rate (s/d). */
  double rate_with_inhibition_s_per_day = 0.0;
  /** Median event spacing times the step (s), when two or more events. */
  std::optional<double> inhibition_period_s;
  uint32_t inhibition_events = 0;      /**< Jumps above the threshold. */
  uint32_t inhibition_cycles_used = 0; /**< Events - 1 when corrected. */
  double jump_mean_us = 0.0;           /**< Mean jump size (µs), 0 if none. */
  double jump_min_us = 0.0;            /**< Smallest jump (µs), 0 if none. */
  double jump_max_us = 0.0;            /**< Largest jump (µs), 0 if none. */
  /** Mean jump in units of one 8192 Hz pulse ({@code jump / 122.07 µs}),
   * rounded: watchmakers read this as "n" of the trimming. 0 if none. */
  uint32_t n_at_8192 = 0;
  double residual_rms_us = 0.0;    /**< RMS fit residual (µs). */
  double polarity_offset_us = 0.0; /**< Polarity term {@code c} (µs). */
  InhibitionStatus status = InhibitionStatus::kMayIncludeInhibition;
  /** Detection/correction bursts logged inside pulse groups (handoff §3.1,
   * §8: time the first pulse of a group only, log the extras). */
  uint32_t extra_pulses = 0;
  uint32_t clipped_pulses = 0; /**< Pulses whose group hit the rail. */
  /** Any raw code reached the rail since reset (whole-stream flag, as the
   * reference harness computes it). Suggest opening JP1. */
  bool clipped = false;
};

/** One detected motor pulse, kept in a short history for the CLI. */
struct PulseInfo {
  uint32_t sequence = 0;  /**< 0-based pulse number since reset. */
  uint64_t i_first = 0;   /**< First sample over threshold (absolute). */
  uint64_t i_peak = 0;    /**< Leading-spike peak sample (absolute). */
  uint64_t i_last = 0;    /**< Last sample over threshold, once closed. */
  double t_samples = 0.0; /**< Leading-edge timestamp (absolute samples). */
  float peak = 0.f;       /**< |y| at the peak after the high-pass (codes). */
  int8_t polarity = 0;    /**< Sign of the leading spike, +1 / -1. */
  uint8_t extras = 0;     /**< Correction/detection bursts in the group. */
  bool clipped = false;   /**< A raw code reached the rail in the group. */
  bool closed = false;    /**< The 50 ms quiet gap after it has passed. */
};

/**
 * Output of the harmonic phase-slope method, the port of
 * {@code harmonic_phase_rate()} (and of {@code tone_rate()} with k = 1).
 */
struct PhaseRate {
  double f_nominal_hz = 0.0; /**< Fundamental the harmonics are taken of. */
  std::array<uint16_t, 4> harmonics{}; /**< Harmonic numbers used. */
  uint8_t harmonic_count = 0;          /**< Valid entries in harmonics. */
  uint32_t blocks = 0;                 /**< Blocks in the phase fit. */
  double fractional_error = 0.0;       /**< f_meas / f_nom - 1. */
  double rate_s_per_day = 0.0;         /**< 86400 · fractional_error. */
};

/** Progress of the LCD measurement. */
enum class LcdStatus : uint8_t {
  kDetectingPeriod = 0, /**< Collecting the 2 s autocorrelation record. */
  kNoPattern,           /**< No candidate passed (reference "error"). */
  kUnsupportedRate,     /**< fs / f0 period does not fit the buffer. */
  kTracking,            /**< f0 known; phase blocks accumulating. */
};

/** @param s Status. @return A short, stable, lower-case label. */
[[nodiscard]] constexpr const char *to_string(LcdStatus s) noexcept {
  switch (s) {
  case LcdStatus::kDetectingPeriod:
    return "detecting_period";
  case LcdStatus::kNoPattern:
    return "no_pattern";
  case LcdStatus::kUnsupportedRate:
    return "unsupported_rate";
  case LcdStatus::kTracking:
    return "tracking";
  }
  return "?";
}

/** Snapshot of the LCD measurement ({@code lcd_rate()} in the reference). */
struct LcdResult {
  LcdStatus status = LcdStatus::kDetectingPeriod;
  /** Pattern repetition rate (Hz) chosen by the autocorrelation, 0 until
   * known or when none passed. */
  uint32_t pattern_hz = 0;
  /** Normalised autocorrelation at lag fs/f for f = 128, 64, 32, 16 Hz (in
   * that order), valid once the period is decided. */
  std::array<float, 4> autocorr{};
  /** Phase-slope rate, once at least two blocks are in the fit. */
  std::optional<PhaseRate> rate;
};

} // namespace quartz
