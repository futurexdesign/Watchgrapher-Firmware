/**
 * @file quartz_dsp.h
 * @brief DSP building blocks for the quartz timing library: the 4th-order
 *        Butterworth high-pass, exact-phase single-bin DFT and Goertzel
 *        helpers, phase unwrapping and an online straight-line fit.
 *
 * Ports the {@code highpass()} helper and the single-bin DFT inside
 * {@code harmonic_phase_rate()} of {@code docs/handoff/ref/timing_ref.py}.
 *
 * Precision policy (the M33 has a single-precision FPU only):
 *  - per-sample work is float32 and goes through CMSIS-DSP kernels
 *    ({@code arm_biquad_cascade_df1_f32}, {@code arm_dot_prod_f32},
 *    {@code arm_mult_f32}, {@code arm_power_f32});
 *  - anything that accumulates across chunks (block DFT sums, fit sums) is
 *    double, touched once per chunk or per block, not per sample;
 *  - oscillator phases are never accumulated: each chunk's carrier is
 *    re-seeded from an exact integer phase index, so no phase error builds up
 *    over a long block or a long run.
 *
 * Pure C++17 plus CMSIS-DSP C kernels (which also build on the host), no
 * Arduino / Pico SDK.
 *
 * @see quartz_dsp.cpp, tests/test_quartz_units.cpp
 */

#pragma once

#include "quartz/quartz_types.h"

#include "arm_math.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace quartz {

/**
 * One second-order section in scipy's sign convention:
 * {@code H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)}.
 */
struct Biquad {
  double b0 = 1.0; /**< Feed-forward coefficient of x[n]. */
  double b1 = 0.0; /**< Feed-forward coefficient of x[n-1]. */
  double b2 = 0.0; /**< Feed-forward coefficient of x[n-2]. */
  double a1 = 0.0; /**< Feedback coefficient of y[n-1] (scipy sign). */
  double a2 = 0.0; /**< Feedback coefficient of y[n-2] (scipy sign). */
};

/**
 * Designs {@code scipy.signal.butter(4, fc, "highpass", fs=fs,
 * output="sos")} without scipy.
 *
 * Steps (as scipy does them): analog Butterworth prototype poles
 * {@code exp(iπ(2k+5)/8)}; low-pass to high-pass {@code s -> ωc/s} with the
 * cut-off pre-warped to {@code ωc = 2·fs·tan(π·fc/fs)} so the -3 dB point
 * lands exactly on {@code fc} after the bilinear transform
 * {@code z = (2fs + s) / (2fs - s)}; all four zeros map to z = 1. The
 * conjugate pole pairs are ordered as scipy's {@code zpk2sos} ("nearest"
 * pairing) orders them: the pair closest to the unit circle goes last, and
 * the overall gain (unity at Nyquist) is folded into the first section's
 * numerator. The cascade therefore matches scipy coefficient for coefficient
 * (tests compare to 1e-12), not just in transfer function.
 *
 * @param fs_hz Sample rate (Hz), > 0.
 * @param fc_hz Cut-off (Hz), 0 < fc < fs/2.
 * @return The two sections, or {@code std::nullopt} for invalid arguments.
 */
[[nodiscard]] std::optional<std::array<Biquad, 2>>
design_butterworth_highpass4(double fs_hz, double fc_hz) noexcept;

/**
 * Causal 4th-order Butterworth high-pass, the port of
 * {@code highpass(x, fs, fc, order=4)} ({@code scipy.signal.sosfilt}).
 *
 * Its job in every quartz path is to remove DC and the 60/180 Hz hum before
 * any threshold or phase is measured. The filter is time-invariant and
 * causal, so it adds the same group delay to every pulse or block, and that
 * delay cancels out of every rate (handoff rule 4).
 *
 * Runs on {@code arm_biquad_cascade_df1_f32} (direct form I is the vendored
 * biquad; for poles at |z| ≈ 0.98 its float32 round-off is lower than DF2T's
 * anyway because the state holds input/output samples, not internal gains).
 *
 * The CMSIS instance holds raw pointers into this object's coefficient and
 * state arrays, so the object is neither copyable nor movable.
 */
class Highpass4 {
public:
  /** Constructs a pass-through (unconfigured) filter. */
  Highpass4() noexcept;
  Highpass4(const Highpass4 &) = delete;
  Highpass4 &operator=(const Highpass4 &) = delete;
  Highpass4(Highpass4 &&) = delete;
  Highpass4 &operator=(Highpass4 &&) = delete;
  ~Highpass4() = default;

  /**
   * Designs the filter for a sample rate and clears its state.
   *
   * @param fs_hz Sample rate (Hz).
   * @param fc_hz Cut-off (Hz).
   * @return {@code false} (filter left unchanged) for invalid arguments.
   * @see design_butterworth_highpass4
   */
  [[nodiscard]] bool configure(double fs_hz, double fc_hz) noexcept;

  /** Zeroes the filter state (as at stream start; the reference filters
   * from a zero state too). */
  void reset() noexcept;

  /**
   * Filters a block. {@code in} and {@code out} may alias.
   *
   * @param in Input samples (codes as float).
   * @param out Output samples.
   * @param n Number of samples.
   */
  void process(const float *in, float *out, std::size_t n) noexcept;

  /** @return The designed sections in scipy sign convention, double. */
  [[nodiscard]] const std::array<Biquad, 2> &sections() const noexcept {
    return sections_;
  }

private:
  /** Copies {@link sections_} into the CMSIS coefficient layout. */
  void load_cmsis_coeffs() noexcept;

  /** Sections as designed (double, scipy sign convention). */
  std::array<Biquad, 2> sections_{};
  /** CMSIS DF1 layout {b0, b1, b2, -a1, -a2} per stage. */
  std::array<float, 10> coeffs_{};
  /** CMSIS DF1 state {x[n-1], x[n-2], y[n-1], y[n-2]} per stage. */
  std::array<float, 8> state_{};
  /** CMSIS instance pointing at {@link coeffs_} and {@link state_}. */
  arm_biquad_casd_df1_inst_f32 inst_{};
};

/**
 * An exact frequency-to-sample-rate ratio {@code f / fs = a / b} in lowest
 * terms. The phase of {@code exp(-2πi·f·n/fs)} at absolute sample {@code n}
 * is {@code 2π·((a·n) mod b) / b}: computed with integers it is exact at any
 * {@code n}, so phases measured hours apart are referenced to the same
 * origin without any accumulated rounding.
 *
 * The sequence {@code exp(-2πi·a·n/b)} is periodic in {@code n} with period
 * {@code b}; the phase trackers exploit that to fold a block into {@code b}
 * bins before taking its DFT.
 */
struct FrequencyRatio {
  uint32_t a = 0; /**< Cycles per {@code b} samples (reduced numerator). */
  uint32_t b = 1; /**< Period of the carrier in samples (reduced). */
};

/**
 * Reduces {@code f / fs} to lowest terms.
 *
 * @param f Frequency (exact fraction, Hz).
 * @param fs_hz Sample rate (Hz).
 * @return The ratio, or {@code std::nullopt} if {@code f} is zero, the
 *         denominator overflows 32 bits, or {@code f >= fs}.
 */
[[nodiscard]] std::optional<FrequencyRatio>
reduce_ratio(Rational f, uint32_t fs_hz) noexcept;

/**
 * Exact single-bin DFT:
 * {@code Σ_{j<n} x[j]·exp(-2πi·a·(m0 + j)/b)}.
 *
 * Processes {@link kChunk}-sample chunks: each chunk's carrier starts from
 * the exact phase {@code 2π·((a·m) mod b)/b} (computed in double from the
 * integer index) and is advanced in float32 by complex rotation for at most
 * {@link kChunk} steps (error under 2e-5 rad), then
 * {@code arm_dot_prod_f32} forms the chunk's real and imaginary sums, which
 * are added in double.
 *
 * This replaces the classic float32 Goertzel recursion wherever the phase
 * matters: at the low normalised frequencies used here (a/b down to 1/6000)
 * a float32 Goertzel's round-off is amplified by ~1/ω² and would cost more
 * phase accuracy than the 1e-3 rad the rate tolerance allows.
 *
 * @param x Samples.
 * @param n Number of samples.
 * @param ratio Carrier frequency as an exact ratio to fs.
 * @param m0 Absolute index of {@code x[0]} (phase origin is index 0).
 * @return The complex sum.
 */
[[nodiscard]] std::complex<double> single_bin_dft(const float *x, std::size_t n,
                                                  FrequencyRatio ratio,
                                                  uint64_t m0) noexcept;

/**
 * Goertzel filter evaluated to a complex DFT value in the same convention
 * as {@link single_bin_dft} with {@code m0 = 0}:
 * {@code Σ_{j<n} x[j]·exp(-2πi·a·j/b)}.
 *
 * The recursion {@code s[j] = x[j] + 2cos(ω)·s[j-1] - s[j-2]} is float32
 * (one multiply, two adds per sample, the cheapest way to rank many bins);
 * the final step {@code exp(-iω(n-1))·(s[n-1] - exp(-iω)·s[n-2])} is double.
 * Used to rank candidate harmonics by magnitude, where float32 is ample.
 *
 * @param x Samples.
 * @param n Number of samples.
 * @param ratio Bin frequency as an exact ratio to fs.
 * @return The complex DFT value.
 */
[[nodiscard]] std::complex<double> goertzel(const float *x, std::size_t n,
                                            FrequencyRatio ratio) noexcept;

/**
 * Fills {@code c[j] = cos(phase0 + j·step)} and {@code s[j] = sin(...)} for
 * {@code j < n} by float32 complex rotation, seeded in double. Keep
 * {@code n} at or below {@link kChunk} so the rotation error stays below
 * ~2e-5 rad.
 *
 * @param c Output cosines.
 * @param s Output sines.
 * @param n Count.
 * @param phase0 Phase of element 0 (rad).
 * @param step Phase increment per element (rad).
 */
void fill_phasor(float *c, float *s, std::size_t n, double phase0,
                 double step) noexcept;

/**
 * Wraps an angle to {@code [-π, π)} and maps an exact {@code -π} to
 * {@code +π} when the input was positive, as {@code numpy.unwrap} does.
 *
 * @param d Angle difference (rad).
 * @return The wrapped difference.
 */
[[nodiscard]] double wrap_phase_difference(double d) noexcept;

/**
 * Online {@code numpy.unwrap}: removes 2π jumps from a sequence of phases
 * that arrive one block at a time.
 */
class PhaseUnwrapper {
public:
  /** Forgets the history; the next phase is taken as is. */
  void reset() noexcept { have_ = false; }

  /**
   * @param phase Wrapped phase in (-π, π] (e.g. from {@code std::arg}).
   * @return The unwrapped phase: the first input unchanged, later inputs
   *         continued from the previous output by the wrapped difference.
   */
  [[nodiscard]] double push(double phase) noexcept;

private:
  bool have_ = false;      /**< A previous phase exists. */
  double last_raw_ = 0.0;  /**< Previous wrapped input. */
  double unwrapped_ = 0.0; /**< Previous unwrapped output. */
};

/**
 * Online least-squares straight line {@code y = slope·x + c} (Welford-style
 * centred updates, double). Replaces the {@code np.linalg.lstsq} on
 * {@code [tc, 1]} in {@code harmonic_phase_rate()} without storing points.
 */
class LineFit {
public:
  /** Clears all points. */
  void reset() noexcept { *this = LineFit{}; }

  /**
   * Adds one point.
   *
   * @param x Abscissa (e.g. block centre time, s).
   * @param y Ordinate (e.g. unwrapped phase / k, rad).
   */
  void add(double x, double y) noexcept;

  /** @return Points added. */
  [[nodiscard]] uint32_t count() const noexcept { return n_; }

  /** @return The slope, or {@code std::nullopt} with fewer than two
   *          distinct abscissae. */
  [[nodiscard]] std::optional<double> slope() const noexcept;

private:
  uint32_t n_ = 0;      /**< Points. */
  double mean_x_ = 0.0; /**< Running mean of x. */
  double mean_y_ = 0.0; /**< Running mean of y. */
  double cxx_ = 0.0;    /**< Σ (x - mean_x)². */
  double cxy_ = 0.0;    /**< Σ (x - mean_x)(y - mean_y). */
};

/**
 * Converts raw 24-bit codes (sign-extended in int32) to float32 and reports
 * whether any code reached the rail.
 *
 * float32 holds every 24-bit code exactly, so the conversion is lossless.
 *
 * @param codes Raw codes.
 * @param out Output floats (codes, unscaled).
 * @param n Count.
 * @return {@code true} if any {@code |code| >= }{@link kClipCode}.
 */
bool codes_to_float(const int32_t *codes, float *out, std::size_t n) noexcept;

/**
 * {@code round()} that is usable in constant expressions (std::round is not
 * constexpr in C++17). Rounds half away from zero; the geometry values it is
 * used for (0.2 ms and 0.3 ms at 44.1–96 kHz) never land on .5.
 *
 * @param v Value.
 * @return The nearest integer as int64.
 */
[[nodiscard]] constexpr int64_t round_to_int(double v) noexcept {
  return v >= 0.0 ? static_cast<int64_t>(v + 0.5)
                  : -static_cast<int64_t>(-v + 0.5);
}

} // namespace quartz
