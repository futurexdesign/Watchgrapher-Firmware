/**
 * @file quartz_dsp.cpp
 * @brief High-pass design and filtering, exact-phase DFT helpers, phase
 *        unwrapping and the online line fit.
 *
 * @see quartz_dsp.h
 */

#include "quartz/quartz_dsp.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace quartz {

namespace {

/** π in double. */
constexpr double kPi = 3.14159265358979323846;
/** 2π in double. */
constexpr double kTwoPi = 2.0 * kPi;

/**
 * Phase {@code 2π·((a·m) mod b)/b} of an exact-ratio carrier at index m,
 * computed with integers so it does not depend on how large m is.
 *
 * @param ratio Carrier ratio.
 * @param m Absolute sample index.
 * @return Phase in [0, 2π).
 */
double exact_phase(FrequencyRatio ratio, uint64_t m) noexcept {
  const uint64_t b = ratio.b;
  const uint64_t r = (static_cast<uint64_t>(ratio.a) % b) * (m % b) % b;
  return kTwoPi * static_cast<double>(r) / static_cast<double>(b);
}

} // namespace

/*
 * scipy's butter(4, fc, "highpass", fs=fs, output="sos") in four steps.
 * timing_ref.highpass() calls exactly this, followed by sosfilt.
 *  1. buttap(4): poles exp(iπ(2k+5)/8), k = 0..3, unit gain. Only the
 *     upper-half-plane pair (k = 0, 1: 112.5° and 157.5°) is generated; the
 *     other two are their conjugates;
 *  2. pre-warp: scipy normalises Wn = fc/(fs/2) and warps it to
 *     4·tan(π·Wn/2) at its internal fs = 2. With the real fs this is
 *     ωc = 2·fs·tan(π·fc/fs), which puts the bilinear -3 dB point exactly
 *     at fc;
 *  3. lp2hp_zpk: p -> ωc/p, and four zeros appear at s = 0. The gain is
 *     unchanged (Π(-p) = 1 for Butterworth);
 *  4. bilinear_zpk: z = (2fs + s)/(2fs - s). Zeros go to z = 1, and the gain
 *     becomes Π(2fs - z_s)/Π(2fs - p_s) = (2fs)^4/Π(2fs - p_s). Over
 *     conjugate pairs that product is Π|2fs - p|², which is real.
 * Each section is then b = g·[1, -2, 1] (a double zero at z = 1) and
 * a = [1, -2·Re p, |p|²]. zpk2sos(pairing="nearest") puts the pole pair
 * nearest the unit circle in the last section and folds the gain into the
 * first section's numerator, so the ordering is chosen by |z|.
 */
std::optional<std::array<Biquad, 2>>
design_butterworth_highpass4(double fs_hz, double fc_hz) noexcept {
  if (!(fs_hz > 0.0) || !(fc_hz > 0.0) || !(fc_hz < 0.5 * fs_hz)) {
    return std::nullopt;
  }
  const double two_fs = 2.0 * fs_hz;
  const double wc = two_fs * std::tan(kPi * fc_hz / fs_hz); // pre-warped
  // Real arithmetic only (std::complex operators pull libgcc's
  // __muldc3/__divdc3 into the target image). For each upper-half-plane
  // prototype pole e^{iθ}: p = wc/e^{iθ} = wc·(cos θ - i·sin θ). Its
  // bilinear image z = (2fs + p)/(2fs - p) is divided out by hand as
  // A·conj(B)/|B|². The conjugate pole contributes the same |2fs - p|² to the
  // gain denominator.
  std::array<double, 2> zr{};
  std::array<double, 2> zi{};
  double denom_prod = 1.0; // Π over all four poles of (2fs - p), real
  for (std::size_t k = 0; k < 2; k++) {
    const double theta = kPi * (2.0 * static_cast<double>(k) + 5.0) / 8.0;
    const double pr = wc * std::cos(theta);
    const double pim = -wc * std::sin(theta);
    const double ar = two_fs + pr; // A = 2fs + p (imaginary part pim)
    const double br = two_fs - pr; // B = 2fs - p (imaginary part -pim)
    const double bi = -pim;
    const double b2 = br * br + bi * bi;
    zr[k] = (ar * br + pim * bi) / b2;
    zi[k] = (pim * br - ar * bi) / b2;
    denom_prod *= b2;
  }
  // Four zeros at s = 0 map to z = 1; gain = (2fs)^4 / Π(2fs - p): unity at
  // Nyquist (z = -1).
  const double gain = std::pow(two_fs, 4) / denom_prod;
  // zpk2sos("nearest"): the pole pair nearest the unit circle goes last.
  if (zr[0] * zr[0] + zi[0] * zi[0] > zr[1] * zr[1] + zi[1] * zi[1]) {
    std::swap(zr[0], zr[1]);
    std::swap(zi[0], zi[1]);
  }
  std::array<Biquad, 2> out{};
  for (std::size_t i = 0; i < 2; i++) {
    const double g = (i == 0) ? gain : 1.0;
    out[i].b0 = g;
    out[i].b1 = -2.0 * g;
    out[i].b2 = g;
    out[i].a1 = -2.0 * zr[i];
    out[i].a2 = zr[i] * zr[i] + zi[i] * zi[i];
  }
  return out;
}

/* Pass-through until configured: b0 = 1 and all other coefficients 0 in both
 * stages, so an unconfigured filter copies its input unchanged. */
Highpass4::Highpass4() noexcept {
  // Pass-through until configured: b0 = 1, everything else 0.
  sections_[0] = Biquad{};
  sections_[1] = Biquad{};
  load_cmsis_coeffs();
}

/* Designs in double, then converts once to the float CMSIS layout. The
 * state is cleared by arm_biquad_cascade_df1_init_f32, which
 * load_cmsis_coeffs() calls. */
bool Highpass4::configure(double fs_hz, double fc_hz) noexcept {
  const auto sos = design_butterworth_highpass4(fs_hz, fc_hz);
  if (!sos) {
    return false;
  }
  sections_ = *sos;
  load_cmsis_coeffs();
  return true;
}

/*
 * CMSIS DF1 stage layout: {b0, b1, b2, a1, a2} with
 *   y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] + a1·y[n-1] + a2·y[n-2],
 * so the feedback terms are the negated scipy a1, a2. Rounding the
 * coefficients to float moves the poles by about 1e-7, which shifts the
 * corner by well under 0.01 Hz (the measured |H| test bounds it). The
 * same constant shift applies to every pulse, so it cancels in rates.
 */
void Highpass4::load_cmsis_coeffs() noexcept {
  for (std::size_t i = 0; i < sections_.size(); i++) {
    const Biquad &s = sections_[i];
    // CMSIS DF1: y = b0 x + b1 x1 + b2 x2 + a1 y1 + a2 y2, i.e. the feedback
    // coefficients enter with the opposite sign to scipy's.
    coeffs_[i * 5 + 0] = static_cast<float>(s.b0);
    coeffs_[i * 5 + 1] = static_cast<float>(s.b1);
    coeffs_[i * 5 + 2] = static_cast<float>(s.b2);
    coeffs_[i * 5 + 3] = static_cast<float>(-s.a1);
    coeffs_[i * 5 + 4] = static_cast<float>(-s.a2);
  }
  arm_biquad_cascade_df1_init_f32(&inst_, static_cast<uint8_t>(2),
                                  coeffs_.data(), state_.data());
}

void Highpass4::reset() noexcept { state_.fill(0.f); }

/* One CMSIS call for the whole block. DF1 keeps x[n-1], x[n-2], y[n-1] and
 * y[n-2] per stage, so splitting a stream into blocks of any size gives
 * bit-identical output (the block-invariance unit test relies on this). */
void Highpass4::process(const float *in, float *out, std::size_t n) noexcept {
  if (n == 0u) {
    return;
  }
  // Re-seat the instance on our own arrays every call (two stores), so the
  // filter never depends on the address it was initialised at.
  inst_.pCoeffs = coeffs_.data();
  inst_.pState = state_.data();
  arm_biquad_cascade_df1_f32(&inst_, in, out, static_cast<uint32_t>(n));
}

/*
 * f/fs = num/(den·fs) reduced by the gcd in 64-bit (den·fs <= 2^32·2^17).
 * Examples: 64 Hz at 48 kHz is 1/750; 1024/3 Hz at 48 kHz is 1024/144000 =
 * 8/1125; 32768 Hz at 96 kHz is 128/375. f >= fs is refused: as a carrier
 * it is indistinguishable from f - fs, so it is not a meaningful request.
 */
std::optional<FrequencyRatio> reduce_ratio(Rational f,
                                           uint32_t fs_hz) noexcept {
  if (f.num == 0u || f.den == 0u || fs_hz == 0u) {
    return std::nullopt;
  }
  const uint64_t num = f.num;
  const uint64_t den = static_cast<uint64_t>(f.den) * fs_hz;
  if (num >= den) {
    return std::nullopt; // at or above fs: not a usable carrier
  }
  const uint64_t g = std::gcd(num, den);
  const uint64_t b = den / g;
  if (b > UINT32_MAX) {
    return std::nullopt;
  }
  return FrequencyRatio{static_cast<uint32_t>(num / g),
                        static_cast<uint32_t>(b)};
}

/*
 * Complex rotation (c + is)·(cos δ + i sin δ) per step in float. The seed
 * and the step angle are computed in double. Each step adds ~1 ulp of
 * angle and magnitude error, so after kChunk = 256 steps the phase is
 * within ~256·6e-8 = 1.5e-5 rad and the magnitude within the same relative
 * amount. Callers re-seed every chunk, so the error never grows past that.
 */
void fill_phasor(float *c, float *s, std::size_t n, double phase0,
                 double step) noexcept {
  if (n == 0u) {
    return;
  }
  const auto rc = static_cast<float>(std::cos(step));
  const auto rs = static_cast<float>(std::sin(step));
  auto pc = static_cast<float>(std::cos(phase0));
  auto ps = static_cast<float>(std::sin(phase0));
  for (std::size_t j = 0; j < n; j++) {
    c[j] = pc;
    s[j] = ps;
    const float nc = pc * rc - ps * rs;
    ps = pc * rs + ps * rc;
    pc = nc;
  }
}

/*
 * Per chunk of up to kChunk samples:
 *  1. the seed phase is exact_phase(ratio, m0 + j0), the carrier angle at
 *     the chunk's first absolute index, from integers only. This is the
 *     rational phase reference that makes a phase measured at sample 1e12
 *     as exact as one at sample 0;
 *  2. cos/sin arrays come from fill_phasor() at step ω = 2π·a/b;
 *  3. two arm_dot_prod_f32 calls give Σx·cos and Σx·sin. Each float
 *     partial sum has at most 256 terms; they are added in double, so
 *     rounding does not grow with the block length (24 000 samples for an
 *     LCD block, 3000 for a fold);
 *  4. Z = Σ x·e^{-iθ} = Σx·cos θ - i·Σx·sin θ.
 * The scratch arrays live on the stack (2 KB), which keeps the function
 * reentrant and allocation-free.
 */
std::complex<double> single_bin_dft(const float *x, std::size_t n,
                                    FrequencyRatio ratio,
                                    uint64_t m0) noexcept {
  std::array<float, kChunk> cbuf{};
  std::array<float, kChunk> sbuf{};
  const double step =
      kTwoPi * static_cast<double>(ratio.a) / static_cast<double>(ratio.b);
  double re = 0.0;
  double im = 0.0;
  for (std::size_t j0 = 0; j0 < n; j0 += kChunk) {
    const std::size_t len = std::min(kChunk, n - j0);
    fill_phasor(cbuf.data(), sbuf.data(), len, exact_phase(ratio, m0 + j0),
                step);
    float dr = 0.f;
    float di = 0.f;
    arm_dot_prod_f32(x + j0, cbuf.data(), static_cast<uint32_t>(len), &dr);
    arm_dot_prod_f32(x + j0, sbuf.data(), static_cast<uint32_t>(len), &di);
    re += static_cast<double>(dr);
    im -= static_cast<double>(di); // exp(-iθ) = cos θ - i sin θ
  }
  return {re, im};
}

/*
 * Standard second-order Goertzel: s[j] = x[j] + 2cos ω·s[j-1] - s[j-2]
 * (float). At the end, y = s[n-1] - e^{-iω}·s[n-2] = Σ x[j]·e^{iω(n-1-j)}.
 * Multiplying by e^{-iω(n-1)} gives the DFT-convention value
 * Σ x[j]·e^{-iωj}, so it can be compared with single_bin_dft. The phase of
 * index n-1 comes from exact_phase(), so the final rotation is exact even
 * when n·ω is large. Precision is only as good as the float recursion
 * (~1e-5 rad here), which is enough for ranking but not for timing.
 */
std::complex<double> goertzel(const float *x, std::size_t n,
                              FrequencyRatio ratio) noexcept {
  if (n == 0u) {
    return {0.0, 0.0};
  }
  const double w =
      kTwoPi * static_cast<double>(ratio.a) / static_cast<double>(ratio.b);
  const auto coeff = static_cast<float>(2.0 * std::cos(w));
  float s1 = 0.f;
  float s2 = 0.f;
  for (std::size_t j = 0; j < n; j++) {
    const float s0 = x[j] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  // y = s[n-1] - e^{-iω}·s[n-2] = Σ x[j]·e^{iω(n-1-j)}, then rotate by
  // e^{-iφ}, φ = ω(n-1) taken exactly, to the DFT convention
  // Σ x[j]·e^{-iωj}. Real arithmetic, no complex-multiply helpers.
  const double yr =
      static_cast<double>(s1) - std::cos(w) * static_cast<double>(s2);
  const double yi = std::sin(w) * static_cast<double>(s2);
  const double phi = exact_phase(ratio, n - 1u);
  const double cr = std::cos(phi);
  const double ci = -std::sin(phi);
  return {yr * cr - yi * ci, yr * ci + yi * cr};
}

/*
 * numpy.unwrap's ddmod = mod(dd + π, 2π) - π, using Python's floored mod
 * (std::fmod truncates, so negative results are shifted up by 2π). Then
 * numpy's tie rule: an exact -π coming from a positive dd becomes +π.
 */
double wrap_phase_difference(double d) noexcept {
  double m = std::fmod(d + kPi, kTwoPi);
  if (m < 0.0) {
    m += kTwoPi; // Python-style modulo: result in [0, 2π)
  }
  m -= kPi;
  if (m == -kPi && d > 0.0) {
    m = kPi;
  }
  return m;
}

/*
 * numpy.unwrap applies a correction only where |dd| >= π (discont = π), and
 * leaves smaller steps exactly as they are. Accumulating the corrected
 * step onto the previous unwrapped value is the running form of numpy's
 * cumsum(ph_correct).
 */
double PhaseUnwrapper::push(double phase) noexcept {
  if (!have_) {
    have_ = true;
    last_raw_ = phase;
    unwrapped_ = phase;
    return unwrapped_;
  }
  const double d = phase - last_raw_;
  // numpy.unwrap leaves differences below π (the default discont) untouched.
  unwrapped_ += (std::fabs(d) < kPi) ? d : wrap_phase_difference(d);
  last_raw_ = phase;
  return unwrapped_;
}

/*
 * Welford co-moment updates. With dx = x - mean_x(old):
 *   mean_x += dx/n,  mean_y += (y - mean_y)/n,
 *   Cxx += dx·(x - mean_x(new)),  Cxy += dx·(y - mean_y(new)).
 * This keeps only centred sums, so no Σx² - (Σx)²/n cancellation occurs
 * even when the unwrapped phases reach thousands of radians. The slope
 * Cxy/Cxx is the lstsq([tc, 1]) slope of harmonic_phase_rate().
 */
void LineFit::add(double x, double y) noexcept {
  n_++;
  const double inv_n = 1.0 / static_cast<double>(n_);
  const double dx = x - mean_x_;
  mean_x_ += dx * inv_n;
  mean_y_ += (y - mean_y_) * inv_n;
  // Welford co-moment updates: (x - old mean)·(y - new mean).
  cxx_ += dx * (x - mean_x_);
  cxy_ += dx * (y - mean_y_);
}

/* Cxy/Cxx; undefined with fewer than two points or a single abscissa. */
std::optional<double> LineFit::slope() const noexcept {
  if (n_ < 2u || !(cxx_ > 0.0)) {
    return std::nullopt;
  }
  return cxy_ / cxx_;
}

/* The rail test is |c| >= 2^23 - 1, i.e. +8 388 607 and both negative
 * extremes, the same as run_tests.py's |L| >= CODE_FS - 1. */
bool codes_to_float(const int32_t *codes, float *out, std::size_t n) noexcept {
  bool clipped = false;
  for (std::size_t i = 0; i < n; i++) {
    const int32_t c = codes[i];
    clipped = clipped || c >= kClipCode || c <= -kClipCode;
    out[i] = static_cast<float>(c);
  }
  return clipped;
}

} // namespace quartz
