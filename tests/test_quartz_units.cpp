/**
 * @file test_quartz_units.cpp
 * @brief Host unit tests for the quartz timing library: high-pass design and
 *        response, exact-phase DFT helpers, phase unwrapping, the threshold
 *        and xcorr timestamps, the segmented motor fit, the tone and LCD
 *        trackers on synthetic signals, and the calibration helper.
 *
 * Every test runs the same code (and the same CMSIS-DSP kernels) as the
 * firmware. Exits non-zero on any failure.
 *
 * Build and run (from the repository root): {@code
 * tests/build_quartz_tests.sh [BUILD_DIR]} compiles the vendored CMSIS-DSP
 * kernels for the host, builds this test and its sibling with
 * {@code -std=c++17 -Wall -Wextra}, and runs both. The binary is
 * {@code BUILD_DIR/test_quartz_units}.
 */

#include "quartz/lcd_timing.h"
#include "quartz/motor_timing.h"
#include "quartz/quartz_dsp.h"
#include "quartz/tone_timing.h"

#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <type_traits>
#include <variant>
#include <vector>

namespace {

/** π in double. */
constexpr double kPi = 3.14159265358979323846;

/** Failed checks. */
int g_failures = 0;

/**
 * Records one check.
 *
 * @param ok Condition.
 * @param what Description printed on failure.
 */
void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("  FAIL: %s\n", what);
    g_failures++;
  }
}

/**
 * Records a numeric check {@code |got - want| <= tol} and prints both.
 *
 * @param got Measured value.
 * @param want Expected value.
 * @param tol Tolerance.
 * @param what Description.
 */
void near(double got, double want, double tol, const char *what) {
  const bool ok = std::fabs(got - want) <= tol;
  std::printf("  %-4s %-48s got %.10g want %.10g (tol %.1e)\n",
              ok ? "ok" : "FAIL", what, got, want, tol);
  if (!ok) {
    g_failures++;
  }
}

/** Deterministic Gaussian noise (sum of 4 uniforms, unit variance). */
class Noise {
public:
  /** @param seed LCG seed. */
  explicit Noise(uint32_t seed) : state_(seed) {}

  /** @return One N(0, 1)-ish sample. */
  double next() {
    double s = 0.0;
    for (int i = 0; i < 4; i++) {
      state_ = state_ * 1664525u + 1013904223u;
      s += static_cast<double>(state_ >> 8) / 16777216.0 - 0.5;
    }
    return s * std::sqrt(3.0); // var(sum of 4 U(-.5,.5)) = 1/3
  }

private:
  uint32_t state_;
};

/** @return |H(e^{jω})| of a biquad cascade at f (Hz). */
double cascade_gain(const std::array<quartz::Biquad, 2> &sos, double f,
                    double fs) {
  const std::complex<double> z1 = std::polar(1.0, -2.0 * kPi * f / fs);
  std::complex<double> h(1.0, 0.0);
  for (const auto &s : sos) {
    h *= (s.b0 + s.b1 * z1 + s.b2 * z1 * z1) /
         (1.0 + s.a1 * z1 + s.a2 * z1 * z1);
  }
  return std::abs(h);
}

/** Analytic bilinear-Butterworth high-pass magnitude (order 4). */
double butterworth_gain(double f, double fc, double fs) {
  const double r = std::tan(kPi * fc / fs) / std::tan(kPi * f / fs);
  return 1.0 / std::sqrt(1.0 + std::pow(r, 8));
}

// ------------------------------------------------------------- high-pass

void test_hpf_coefficients() {
  std::printf("HPF coefficients vs scipy.signal.butter (scipy 1.18.1)\n");
  struct Case {
    double fc, fs;
    std::array<std::array<double, 5>, 2> sos; // b0 b1 b2 a1 a2
  };
  const std::array<Case, 3> cases = {{
      {400.0,
       48000.0,
       {{{0.9338675736364777, -1.8677351472729553, 0.9338675736364777,
          -1.9051414440976044, 0.9077559573338853},
         {1.0, -2.0, 1.0, -1.9580431783283938, 0.9607302910479292}}}},
      {180.0,
       48000.0,
       {{{0.9696830640821985, -1.939366128164397, 0.9696830640821985,
          -1.956851282961526, 0.9573945965529782},
         {1.0, -2.0, 1.0, -1.981579078857877, 0.9821292580439271}}}},
      {200.0,
       96000.0,
       {{{0.9830424139842885, -1.966084827968577, 0.9830424139842885,
          -1.97593328015711, 0.9761025776589489},
         {1.0, -2.0, 1.0, -1.9898610999129416, 0.9900315907471235}}}},
  }};
  for (const Case &c : cases) {
    const auto sos = quartz::design_butterworth_highpass4(c.fs, c.fc);
    check(sos.has_value(), "design succeeds");
    if (!sos) {
      continue;
    }
    double worst = 0.0;
    for (std::size_t i = 0; i < 2; i++) {
      const auto &s = (*sos)[i];
      const std::array<double, 5> got = {s.b0, s.b1, s.b2, s.a1, s.a2};
      for (std::size_t k = 0; k < 5; k++) {
        worst = std::max(worst, std::fabs(got[k] - c.sos[i][k]));
      }
    }
    char what[64];
    std::snprintf(what, sizeof what, "max |coef - scipy| fc=%g fs=%g", c.fc,
                  c.fs);
    near(worst, 0.0, 1e-12, what);
  }
  check(!quartz::design_butterworth_highpass4(48000.0, 24000.0),
        "fc at Nyquist rejected");
  check(!quartz::design_butterworth_highpass4(48000.0, 0.0), "fc = 0 rejected");
}

void test_hpf_magnitude() {
  std::printf("HPF magnitude vs analytic Butterworth (400 Hz, 48 kHz)\n");
  constexpr double kFs = 48000.0;
  constexpr double kFc = 400.0;
  static quartz::Highpass4 hpf;
  check(hpf.configure(kFs, kFc), "configure");
  for (double f : {60.0, 180.0, 400.0, 2000.0}) {
    const double want = butterworth_gain(f, kFc, kFs);
    char what[80];
    std::snprintf(what, sizeof what, "designed |H| at %4.0f Hz", f);
    near(cascade_gain(hpf.sections(), f, kFs), want, 1e-9 * want + 1e-15, what);

    // Measured through the float32 CMSIS cascade: 1 s settle, then the
    // amplitude over exactly 0.5 s (an integer number of periods).
    hpf.reset();
    const auto ratio =
        quartz::reduce_ratio({static_cast<uint32_t>(f), 1u}, 48000u);
    constexpr std::size_t kSettle = 48000u;
    constexpr std::size_t kMeasure = 24000u;
    std::vector<float> x(kSettle + kMeasure);
    constexpr double kAmp = 1e6;
    for (std::size_t n = 0; n < x.size(); n++) {
      x[n] = static_cast<float>(kAmp * std::cos(2.0 * kPi * f * n / kFs));
    }
    hpf.process(x.data(), x.data(), x.size());
    const auto z =
        quartz::single_bin_dft(x.data() + kSettle, kMeasure, *ratio, kSettle);
    const double got = 2.0 * std::abs(z) / kMeasure / kAmp;
    std::snprintf(what, sizeof what, "measured |H| at %4.0f Hz (float32)", f);
    near(got, want, 2e-5 * want + 1e-7, what);
  }
}

// ----------------------------------------------------------- DFT helpers

void test_reduce_ratio() {
  std::printf("Exact frequency ratios\n");
  const auto a = quartz::reduce_ratio({1024u, 3u}, 48000u);
  check(a && a->a == 8u && a->b == 1125u, "1024/3 Hz @ 48k = 8/1125");
  const auto a96 = quartz::reduce_ratio({1024u, 3u}, 96000u);
  check(a96 && a96->a == 4u && a96->b == 1125u, "1024/3 Hz @ 96k = 4/1125");
  const auto b = quartz::reduce_ratio({32768u, 1u}, 96000u);
  check(b && b->a == 128u && b->b == 375u, "32768 Hz @ 96k = 128/375");
  const auto c = quartz::reduce_ratio({360u, 1u}, 48000u);
  check(c && c->a == 3u && c->b == 400u, "360 Hz @ 48k = 3/400");
  check(!quartz::reduce_ratio({48000u, 1u}, 48000u), "f = fs rejected");
}

/**
 * Closed-form {@code Σ_{j<n} A·cos(ω(m0+j) + φ)·e^{-iω(m0+j)}} for
 * {@code ω = 2π·a/b}: the positive-frequency term is {@code n·e^{iφ}} for
 * any m0; the image term carries {@code e^{-2iω·m0}}, whose phase is taken
 * exactly from {@code (2a·m0) mod b}.
 */
std::complex<double> closed_form(double amp, quartz::FrequencyRatio r,
                                 double phi, std::size_t n, uint64_t m0 = 0) {
  using cd = std::complex<double>;
  const double w = 2.0 * kPi * r.a / r.b;
  const cd e2 = std::polar(1.0, -2.0 * w);
  const cd geo = (1.0 - std::pow(e2, static_cast<double>(n))) / (1.0 - e2);
  const uint64_t k = (2u * static_cast<uint64_t>(r.a) * (m0 % r.b)) % r.b;
  const cd image = std::polar(1.0, -2.0 * kPi * static_cast<double>(k) / r.b);
  return 0.5 * amp *
         (static_cast<double>(n) * std::polar(1.0, phi) +
          std::polar(1.0, -phi) * image * geo);
}

void test_goertzel_phase() {
  std::printf("Goertzel / single-bin DFT phase vs closed form\n");
  const quartz::FrequencyRatio r{37u, 480u}; // not a whole number of cycles
  const double w = 2.0 * kPi * r.a / r.b;
  constexpr std::size_t kN = 1000u;
  constexpr double kAmp = 1e5;
  for (double phi : {0.3, -2.0, 3.0}) {
    std::vector<float> x(kN);
    for (std::size_t j = 0; j < kN; j++) {
      x[j] = static_cast<float>(kAmp * std::cos(w * j + phi));
    }
    const auto want = closed_form(kAmp, r, phi, kN);
    const auto g = quartz::goertzel(x.data(), kN, r);
    const auto d = quartz::single_bin_dft(x.data(), kN, r, 0u);
    char what[80];
    std::snprintf(what, sizeof what, "goertzel phase, phi=%+.1f", phi);
    near(std::arg(g / want), 0.0, 2e-5, what);
    std::snprintf(what, sizeof what, "single-bin DFT phase, phi=%+.1f", phi);
    near(std::arg(d / want), 0.0, 2e-6, what);
    std::snprintf(what, sizeof what, "single-bin DFT |Z| ratio, phi=%+.1f",
                  phi);
    near(std::abs(d) / std::abs(want), 1.0, 1e-5, what);
  }

  // Absolute phase reference at a huge index: x[j] = cos(ω(m0+j) + φ) has
  // the same closed form when the carrier is referenced to index 0.
  constexpr uint64_t kM0 = 1000000000123ull; // ~240 days at 48 kHz
  constexpr double kPhi = 0.7;
  std::vector<float> x(kN);
  for (std::size_t j = 0; j < kN; j++) {
    const uint64_t idx = (r.a * ((kM0 + j) % r.b)) % r.b;
    x[j] = static_cast<float>(kAmp * std::cos(2.0 * kPi * idx / r.b + kPhi));
  }
  const auto d = quartz::single_bin_dft(x.data(), kN, r, kM0);
  near(std::arg(d / closed_form(kAmp, r, kPhi, kN, kM0)), 0.0, 2e-6,
       "single-bin DFT phase at index 1e12 (exact ref)");
}

void test_unwrap() {
  std::printf("Phase unwrap\n");
  quartz::PhaseUnwrapper u;
  double worst = 0.0;
  for (int b = 0; b < 200; b++) {
    const double truth = 0.1 + 2.5 * b; // wraps every ~2.5 blocks
    const double wrapped = std::arg(std::polar(1.0, truth));
    worst = std::max(worst, std::fabs(u.push(wrapped) - truth));
  }
  near(worst, 0.0, 1e-9, "unwrap of +2.5 rad/block over 200 blocks");
  u.reset();
  worst = 0.0;
  for (int b = 0; b < 200; b++) {
    const double truth = -0.4 - 3.0 * b;
    worst = std::max(
        worst, std::fabs(u.push(std::arg(std::polar(1.0, truth))) - truth));
  }
  near(worst, 0.0, 1e-9, "unwrap of -3.0 rad/block over 200 blocks");
  near(quartz::wrap_phase_difference(kPi), kPi, 0.0, "wrap(+pi) = +pi");
  near(quartz::wrap_phase_difference(-kPi), -kPi, 0.0, "wrap(-pi) = -pi");
  near(quartz::wrap_phase_difference(3.0 * kPi / 2.0), -kPi / 2.0, 1e-15,
       "wrap(3pi/2) = -pi/2");
}

// ----------------------------------------------------------- timestamps

void test_threshold_timestamp() {
  std::printf("Threshold timestamp\n");
  quartz::PulseWindow w{};
  w.start = 1000u;
  w.len = 64u;
  // A negative pulse: quiet, then a linear edge to -1000 at index 40.
  for (std::size_t j = 0; j < w.len; j++) {
    w.y[j] = 0.f;
  }
  w.y[38] = -100.f;
  w.y[39] = -400.f;
  w.y[40] = -1000.f;
  w.y[41] = -600.f;
  w.i_peak = w.start + 40u;
  w.polarity = -1;
  w.peak = 1000.f;
  // Half = 500, crossed between 39 (400) and 40 (1000): 39 + 100/600.
  near(quartz::threshold_timestamp(w), 1000.0 + 39.0 + 100.0 / 600.0, 1e-12,
       "50 % crossing, negative pulse");
}

void test_xcorr() {
  std::printf("Cross-correlation timestamps\n");
  constexpr std::size_t kN = 24u;
  constexpr double kCentre = 9.0;
  constexpr double kWidth = 2.0;
  const std::array<double, 5> delay = {0.0, 0.31, -0.22, 0.47, 0.123};
  // Smooth, near band-limited pulse delayed by d samples.
  const auto make = [&](double d, std::array<float, kN> &s) {
    for (std::size_t t = 0; t < kN; t++) {
      const double u = (static_cast<double>(t) - kCentre - d) / kWidth;
      s[t] = static_cast<float>(1e5 * std::exp(-u * u));
    }
  };
  static quartz::XcorrTimestamper x;
  check(x.configure(kN), "configure 24-sample segments");
  std::array<float, kN> seg{};
  for (std::size_t i = 0; i < quartz::XcorrTimestamper::kTemplatePulses; i++) {
    make(delay[i], seg);
    x.add_template_pulse(seg.data(), 0.0);
  }
  check(x.fixed(), "template fixed after four pulses");
  double worst = 0.0;
  for (std::size_t i = 1; i < 4; i++) {
    const double d = x.pending_timestamp(i) - x.pending_timestamp(0);
    worst = std::max(worst, std::fabs(d - (delay[i] - delay[0])));
  }
  near(worst, 0.0, 2e-3, "pending pulses: relative delay error (samples)");
  make(delay[4], seg);
  const double t5 = x.timestamp(seg.data(), 100.0);
  near(t5 - 100.0 - x.pending_timestamp(0), delay[4] - delay[0], 2e-3,
       "fifth pulse against the fixed template");

  // Fractional shift round trip: shifting by +d then -d restores the input.
  make(0.0, seg);
  std::array<float, kN> a{};
  std::array<float, kN> b{};
  x.fractional_shift(seg.data(), 0.37, a.data());
  x.fractional_shift(a.data(), -0.37, b.data());
  double err = 0.0;
  for (std::size_t t = 0; t < kN; t++) {
    err = std::max(err, std::fabs(static_cast<double>(b[t] - seg[t])));
  }
  // Not exact: numpy's irfft drops the imaginary part of the Nyquist bin,
  // and this Gaussian still has ~5e-5 of its peak spectrum there.
  near(err / 1e5, 0.0, 1e-4, "fractional shift +d then -d (relative)");
}

// ------------------------------------------------------------ motor fit

/** Builds a fit from synthetic pulses and solves it. */
std::optional<quartz::MotorRate>
fit_synthetic(quartz::MotorRateFit &fit, std::size_t pulses, double period_s,
              double pol_offset_s, double jump_s, std::size_t jump_every,
              const std::vector<std::size_t> &missing, double noise_s = 0.0) {
  constexpr uint32_t kFs = 48000u;
  fit.configure(kFs, {});
  Noise rng(7u);
  double delay = 0.0;
  for (std::size_t k = 0; k < pulses; k++) {
    if (jump_every != 0u && k > 0u && k % jump_every == 0u) {
      delay += jump_s; // inhibition: the next steps all come later
    }
    bool skip = false;
    for (std::size_t m : missing) {
      skip = skip || m == k;
    }
    const int8_t pol = (k & 1u) ? -1 : 1;
    const double t =
        0.29 + k * period_s + delay + pol_offset_s * pol + noise_s * rng.next();
    if (!skip) {
      fit.push(t * kFs + 1e9, pol); // large absolute origin
    }
  }
  return fit.solve();
}

void test_motor_fit() {
  std::printf("Segmented motor fit\n");
  static quartz::MotorRateFit fit;
  // Odd count, polarity offset, watch +0.5 s/d.
  const double p = 1.0 / (1.0 + 0.5 / 86400.0);
  auto r = fit_synthetic(fit, 31u, p, 3e-6, 0.0, 0u, {});
  check(r.has_value(), "solve");
  if (r) {
    near(r->rate_without_inhibition_s_per_day, 0.5, 1e-7,
         "rate, 31 pulses + 3 us polarity offset");
    near(r->polarity_offset_us, 3.0, 1e-6, "polarity offset c (us)");
    near(r->residual_rms_us, 0.0, 1e-6, "residual (us)");
    check(r->status == quartz::InhibitionStatus::kMayIncludeInhibition,
          "short gate, no jump: may_include_inhibition");
  }
  // Missed pulses cost nothing.
  r = fit_synthetic(fit, 31u, p, 3e-6, 0.0, 0u, {5u, 6u, 17u});
  if (r) {
    near(r->rate_without_inhibition_s_per_day, 0.5, 1e-7,
         "rate with pulses 5, 6, 17 missing");
    check(r->pulses == 28u && r->step_s == 1.0, "28 pulses, step 1 s");
  }
  // Inhibition: crystal +14 ppm, one 8192 Hz pulse removed every 10 s.
  const double jump = 1.0 / 8192.0;
  const double pc = 1.0 / (1.0 + 14e-6);
  r = fit_synthetic(fit, 35u, pc, 2e-6, jump, 10u, {});
  if (r) {
    const double raw = quartz::rate_s_per_day(1.0, pc);
    const double corr = quartz::rate_s_per_day(1.0, pc + jump / 10.0);
    near(r->rate_without_inhibition_s_per_day, raw, 1e-6,
         "raw rate with 3 inhibition events");
    near(r->rate_with_inhibition_s_per_day, corr, 1e-6,
         "corrected rate over whole cycles");
    check(r->inhibition_period_s && *r->inhibition_period_s == 10.0,
          "inhibition period exactly 10 s");
    check(r->inhibition_events == 3u, "three events");
    // dev_k (as in motor_rate()) does not remove the polarity term, so a
    // jump across an odd-to-even interval also carries 2c = +4 us here.
    near(r->jump_mean_us, 122.0703125 + 4.0, 1e-4,
         "jump size (us), incl. 2c polarity term");
    check(r->n_at_8192 == 1u, "n at 8192 Hz = 1");
    check(r->status == quartz::InhibitionStatus::kCorrected,
          "status corrected");
  }
  // Same with noise and a missed pulse straddling nothing in particular.
  r = fit_synthetic(fit, 130u, 1.0 / (1.0 + 10.453e-6), 0.0, 4.0 * jump, 60u,
                    {33u}, 1.5e-6);
  if (r) {
    check(r->inhibition_period_s && *r->inhibition_period_s == 60.0,
          "60 s period exact with noise and a missed pulse");
    check(r->n_at_8192 == 4u, "n at 8192 Hz = 4");
  }
  // One event: need one more cycle.
  r = fit_synthetic(fit, 15u, pc, 0.0, jump, 10u, {});
  check(r && r->status == quartz::InhibitionStatus::kNeedOneMoreCycle,
        "one event: need_one_more_cycle");
  // Two events, but the only pulse after the last has the other polarity.
  // Events after pulses 8 and 17; i = 9 is odd, and the only pulse after
  // the last event (18) is even.
  r = fit_synthetic(fit, 19u, pc, 0.0, jump, 9u, {});
  check(r && r->status == quartz::InhibitionStatus::kCorrectionPending,
        "two events, no same-polarity pulse after: correction_pending");
  // Long gate, no events.
  r = fit_synthetic(fit, 130u, p, 0.0, 0.0, 0u, {});
  check(r && r->status == quartz::InhibitionStatus::kNoJumpLongGate,
        "130 s without a jump: no_jump_long_gate");
  // A 20 s stepper snaps to 20.
  r = fit_synthetic(fit, 4u, 20.0 / (1.0 + 0.5 / 86400.0), 0.0, 0.0, 0u, {});
  check(r && r->step_s == 20.0, "20 s step snapped");
  if (r) {
    near(r->rate_without_inhibition_s_per_day, 0.5, 1e-7, "20 s step rate");
  }
  // Too few pulses.
  fit.configure(48000u, {});
  fit.push(1.0, 1);
  fit.push(48001.0, -1);
  check(!fit.solve(), "two pulses: no rate");
}

// ------------------------------------------------ incremental motor fit

/** Parameters of one synthetic pulse train. */
struct Train {
  std::size_t pulses = 0;    /**< Steps generated (before misses). */
  double step = 1.0;         /**< Nominal step (s). */
  double ppm = 0.0;          /**< Oscillator error (+ = fast). */
  double pol_offset = 0.0;   /**< Polarity term c (s). */
  double noise = 0.0;        /**< Timing noise sigma (s). */
  double miss = 0.0;         /**< Probability of a missed pulse. */
  std::size_t inh_every = 0; /**< Steps per inhibition cycle (0 = none). */
  double jump = 0.0;         /**< Delay per inhibition event (s). */
  std::size_t inh_phase = 0; /**< Step of the first event. */
};

/** Uniform [0, 1) from an LCG. */
double uniform(uint32_t &state) {
  state = state * 1664525u + 1013904223u;
  return static_cast<double>(state >> 8) / 16777216.0;
}

/**
 * Pushes a train into a fit, calling solve_incremental() after each pulse
 * (and occasionally skipping calls, as a caller that solves once a second
 * would), then drains any rebuild.
 *
 * @return Number of inhibition events between pushed pulses (the truth).
 */
uint32_t push_train(quartz::MotorRateFit &fit, const Train &tr, uint32_t seed,
                    bool skip_some) {
  fit.configure(48000u, {});
  Noise rng(seed);
  uint32_t u = seed * 7u + 1u;
  double delay = 0.0;
  const double period = tr.step / (1.0 + 1e-6 * tr.ppm);
  uint32_t events = 0;
  bool pending_event = false; // a jump since the last pushed pulse
  bool pushed_any = false;
  for (std::size_t k = 0; k < tr.pulses; k++) {
    if (tr.inh_every != 0u && k >= tr.inh_phase &&
        (k - tr.inh_phase) % tr.inh_every == 0u && k > 0u) {
      delay += tr.jump;
      pending_event = true;
    }
    const bool missed = uniform(u) < tr.miss && k > 0u;
    const int8_t pol = (k & 1u) ? -1 : 1;
    const double t =
        0.3 + k * period + delay + tr.pol_offset * pol + tr.noise * rng.next();
    if (missed) {
      continue;
    }
    fit.push(t * 48000.0 + 1e7, pol);
    if (pending_event && pushed_any) {
      events++;
    }
    pending_event = false;
    pushed_any = true;
    if (!skip_some || uniform(u) < 0.6) {
      static_cast<void>(fit.solve_incremental());
    }
  }
  do {
    static_cast<void>(fit.solve_incremental());
  } while (fit.rebuilding());
  return events;
}

/** @return {@code true} if the incremental and batch results agree. */
bool same_result(const std::optional<quartz::MotorRate> &inc,
                 const std::optional<quartz::MotorRate> &bat, double tol,
                 double *worst) {
  if (inc.has_value() != bat.has_value()) {
    return false;
  }
  if (!inc) {
    return true;
  }
  const double d1 = std::fabs(inc->rate_without_inhibition_s_per_day -
                              bat->rate_without_inhibition_s_per_day);
  const double d2 = std::fabs(inc->rate_with_inhibition_s_per_day -
                              bat->rate_with_inhibition_s_per_day);
  *worst = std::max(*worst, std::max(d1, d2));
  return d1 <= tol && d2 <= tol && inc->step_s == bat->step_s &&
         inc->pulses == bat->pulses &&
         inc->inhibition_events == bat->inhibition_events &&
         inc->inhibition_period_s == bat->inhibition_period_s &&
         inc->status == bat->status && inc->n_at_8192 == bat->n_at_8192 &&
         std::fabs(inc->residual_rms_us - bat->residual_rms_us) <= 1e-3 &&
         std::fabs(inc->polarity_offset_us - bat->polarity_offset_us) <= 1e-3;
}

/**
 * Random trains: incremental vs batch.
 *
 * @param label Sweep name.
 * @param c_max Largest |polarity offset| (s).
 * @param noise_max Largest timing noise sigma (s).
 * @param strict Every train must match; otherwise only trains where a
 *        solver's event count departs from the truth may differ.
 */
void incremental_sweep(const char *label, double c_max, double noise_max,
                       bool strict) {
  static quartz::MotorRateFit fit;
  constexpr std::array<double, 8> kSteps = {1.0, 1.0,  1.0,  2.0,
                                            5.0, 10.0, 20.0, 60.0};
  constexpr std::array<std::size_t, 5> kCycles = {10u, 20u, 30u, 60u, 0u};
  uint32_t u = strict ? 12345u : 777u;
  int mismatches = 0;
  int explained = 0;
  int with_events = 0;
  double worst = 0.0;
  constexpr int kTrials = 400;
  for (int trial = 0; trial < kTrials; trial++) {
    Train tr{};
    // Log-uniform length 3..2048 (never overflows the store).
    tr.pulses = static_cast<std::size_t>(
        std::lround(3.0 * std::pow(2048.0 / 3.0, uniform(u))));
    tr.step = kSteps[static_cast<std::size_t>(uniform(u) * kSteps.size())];
    tr.ppm = 100.0 * (uniform(u) - 0.5);
    tr.pol_offset = 2.0 * c_max * (uniform(u) - 0.5);
    tr.noise = noise_max * uniform(u);
    tr.miss = 0.05 * uniform(u);
    tr.inh_every = kCycles[static_cast<std::size_t>(uniform(u) * 5.0)];
    tr.jump = (1.0 + std::floor(8.0 * uniform(u))) / 8192.0;
    tr.inh_phase = tr.inh_every
                       ? static_cast<std::size_t>(
                             uniform(u) * static_cast<double>(tr.inh_every))
                       : 0u;
    const uint32_t truth =
        push_train(fit, tr, static_cast<uint32_t>(trial + 1), (trial & 1) != 0);
    const auto inc = fit.solve_incremental();
    const auto bat = fit.solve();
    if (bat && bat->inhibition_events >= 2u) {
      with_events++;
    }
    double w = 0.0;
    if (same_result(inc, bat, 1e-4, &w)) {
      worst = std::max(worst, w);
      continue;
    }
    mismatches++;
    // A legitimate difference: the 20 µs event test misfired (or missed)
    // in at least one solver. The global and windowed medians then sit on
    // different sides of a borderline interval.
    const bool legit =
        inc && bat &&
        (inc->inhibition_events != truth || bat->inhibition_events != truth);
    explained += legit ? 1 : 0;
    std::printf("  %s trial %d: N=%zu step=%g c=%+.2f us noise=%.2f us "
                "events truth %u inc %u batch %u, raw %+.6f / %+.6f\n",
                legit ? "explained" : "UNEXPLAINED", trial, tr.pulses, tr.step,
                tr.pol_offset * 1e6, tr.noise * 1e6, truth,
                inc ? inc->inhibition_events : 0u,
                bat ? bat->inhibition_events : 0u,
                inc ? inc->rate_without_inhibition_s_per_day : 0.0,
                bat ? bat->rate_without_inhibition_s_per_day : 0.0);
  }
  std::printf("  %s: %d trials (%d with >= 2 events), %d differ (%d with a "
              "misfired event test), worst |d rate| of the rest %.2e s/d\n",
              label, kTrials, with_events, mismatches, explained, worst);
  if (strict) {
    check(mismatches == 0, "incremental equals batch on every train");
  } else {
    check(explained == mismatches,
          "every difference comes from a misfired event test");
  }
  near(worst, 0.0, 1e-4, "worst rate difference of matching trains (s/d)");
}

void test_incremental_random() {
  std::printf("Incremental motor fit vs batch solve(): randomised trains\n");
  // Realistic: |c| <= 1 us (vectors: <= 0.74 us), sigma <= 2 us (handoff:
  // 0.01-2.6 us). The 20 us event test then has >= 5 sigma of margin.
  incremental_sweep("realistic", 1e-6, 2e-6, true);
  // Stress: |c| up to 5 us and sigma up to 3 us. Per-step intervals then
  // split into two clusters 4c apart, so the median sits in one of them
  // and the other cluster's intervals reach the 20 us threshold. The
  // batch reference misfires too; the two medians just misfire on
  // different intervals.
  incremental_sweep("stress", 5e-6, 3e-6, false);
}

void test_long_gate_boundary() {
  std::printf("§5 long gate: 120 s boundary (P_max = 60 s)\n");
  static quartz::MotorRateFit fit;
  struct Case {
    double step;
    std::size_t pulses;
    bool long_gate;
  };
  // 120 pulses at 1 s span 119 s but cover 120 step intervals.
  const std::array<Case, 4> cases = {{{1.0, 119u, false},
                                      {1.0, 120u, true},
                                      {20.0, 5u, false},
                                      {20.0, 6u, true}}};
  for (const Case &c : cases) {
    fit.configure(48000u, {});
    for (std::size_t k = 0; k < c.pulses; k++) {
      fit.push((0.4 + c.step * static_cast<double>(k)) * 48000.0,
               (k & 1u) ? -1 : 1);
      static_cast<void>(fit.solve_incremental());
    }
    const auto bat = fit.solve();
    const auto inc = fit.solve_incremental();
    const auto want = c.long_gate
                          ? quartz::InhibitionStatus::kNoJumpLongGate
                          : quartz::InhibitionStatus::kMayIncludeInhibition;
    char what[96];
    std::snprintf(what, sizeof what,
                  "%2zu pulses at %2.0f s: %s (batch and "
                  "incremental)",
                  c.pulses, c.step, quartz::to_string(want));
    check(bat && inc && bat->status == want && inc->status == want, what);
    std::printf("  %-4s %s\n",
                (bat && inc && bat->status == want && inc->status == want)
                    ? "ok"
                    : "FAIL",
                what);
  }
}

/**
 * Past the 2048-pulse store: 5000 pulses at 1 Hz. The incremental state is
 * cumulative, so it must stay finite and keep using the first event as its
 * anchor (held as a scalar, not a store index). A later run of missed
 * pulses would re-snap the step to 2 s, but the overflow policy suppresses
 * the rebuild.
 */
void test_incremental_overflow() {
  std::printf("Incremental motor fit past the store (5000 pulses)\n");
  static quartz::MotorRateFit fit;
  constexpr std::size_t kPulses = 5000u;
  const double jump = 4.0 / 8192.0;
  const double pc = 1.0 / (1.0 + 10.453e-6);
  struct Case {
    bool inhibition;
    const char *what;
  };
  for (const Case c :
       {Case{true, "60 s inhibition"}, Case{false, "no inhibition"}}) {
    fit.configure(48000u, {});
    Noise rng(11u);
    double delay = 0.0;
    double last_t = 0.0;
    bool finite = true;
    for (std::size_t k = 0; k < kPulses; k++) {
      if (c.inhibition && k > 0u && k % 60u == 17u) {
        delay += jump;
      }
      last_t = 0.44 + static_cast<double>(k) * pc + delay + 1e-6 * rng.next();
      fit.push(last_t * 48000.0, (k & 1u) ? -1 : 1);
      const auto r = fit.solve_incremental();
      if (r) {
        finite = finite &&
                 std::isfinite(r->rate_without_inhibition_s_per_day) &&
                 std::isfinite(r->rate_with_inhibition_s_per_day) &&
                 std::isfinite(r->residual_rms_us);
      }
    }
    // A run of 40 missed-every-other pulses: the window snaps to 2 s.
    for (int k = 0; k < 40; k++) {
      last_t += 2.0 * pc;
      fit.push(last_t * 48000.0, 1);
      static_cast<void>(fit.solve_incremental());
    }
    const auto r = fit.solve_incremental();
    check(finite, "every intermediate result finite");
    check(r.has_value() && !fit.rebuilding(), "result, no rebuild");
    if (!r) {
      continue;
    }
    char what[96];
    std::snprintf(what, sizeof what, "%s: pulses cumulative (5040)", c.what);
    check(r->pulses == kPulses + 40u && fit.size() == 2048u, what);
    std::snprintf(what, sizeof what, "%s: step stays 1 s", c.what);
    check(r->step_s == 1.0, what);
    std::snprintf(what, sizeof what, "%s: raw rate (s/d)", c.what);
    near(r->rate_without_inhibition_s_per_day, quartz::rate_s_per_day(1.0, pc),
         2e-4, what);
    if (c.inhibition) {
      const double corr = quartz::rate_s_per_day(1.0, pc + jump / 60.0);
      near(r->rate_with_inhibition_s_per_day, corr, 2e-4,
           "60 s inhibition: corrected rate (s/d)");
      check(r->status == quartz::InhibitionStatus::kCorrected &&
                r->inhibition_period_s && *r->inhibition_period_s == 60.0 &&
                r->inhibition_events == 84u && r->n_at_8192 == 4u,
            "60 s inhibition: 84 events, period 60 s, n = 4, corrected");
      check(fit.stream_jump_count() == 84u, "84 jumps kept");
    } else {
      check(r->status == quartz::InhibitionStatus::kNoJumpLongGate &&
                r->inhibition_events == 0u,
            "no inhibition: no_jump_long_gate");
    }
  }
}

void test_incremental_step_change() {
  std::printf("Incremental motor fit: step re-snap and spread rebuild\n");
  static quartz::MotorRateFit fit;
  fit.configure(48000u, {});
  // 40 pulses two steps apart, then 300 consecutive 1 s steps: the early
  // snap says 2 s; the window later says 1 s, which forces a rebuild.
  const double p = 1.0 / (1.0 + 8e-6);
  std::size_t calls_rebuilding = 0;
  double t = 0.5;
  for (int k = 0; k < 340; k++) {
    t += (k < 40) ? 2.0 * p : p;
    fit.push(t * 48000.0, (k & 1) ? -1 : 1);
    static_cast<void>(fit.solve_incremental());
    calls_rebuilding += fit.rebuilding() ? 1u : 0u;
  }
  while (fit.rebuilding()) {
    static_cast<void>(fit.solve_incremental());
    calls_rebuilding++;
  }
  const auto inc = fit.solve_incremental();
  const auto bat = fit.solve();
  double worst = 0.0;
  std::printf("  calls spent rebuilding: %zu\n", calls_rebuilding);
  check(calls_rebuilding > 0u, "a rebuild happened");
  check(inc && inc->step_s == 1.0, "re-snapped to 1 s");
  check(same_result(inc, bat, 1e-6, &worst), "equal to batch after rebuild");
  if (inc) {
    near(inc->rate_without_inhibition_s_per_day, 0.6912, 1e-6,
         "rate +8 ppm after rebuild (s/d)");
  }
}

void test_incremental_timing() {
  std::printf("Host time per call, 2048 pulses (10 s inhibition)\n");
  static quartz::MotorRateFit fit;
  Train tr{};
  tr.pulses = 2048u;
  tr.ppm = 14.0;
  tr.noise = 1e-6;
  tr.inh_every = 10u;
  tr.jump = 1.0 / 8192.0;
  using clock = std::chrono::steady_clock;
  fit.configure(48000u, {});
  Noise rng(3u);
  double delay = 0.0;
  double inc_total = 0.0;
  double inc_max = 0.0;
  double inc_max_steady = 0.0;
  for (std::size_t k = 0; k < tr.pulses; k++) {
    if (k > 0u && k % tr.inh_every == 0u) {
      delay += tr.jump;
    }
    const double t = 0.3 + k / (1.0 + 14e-6) + delay + 1e-6 * rng.next();
    const auto t0 = clock::now();
    fit.push(t * 48000.0, (k & 1u) ? -1 : 1);
    static_cast<void>(fit.solve_incremental());
    const double us =
        std::chrono::duration<double, std::micro>(clock::now() - t0).count();
    inc_total += us;
    inc_max = std::max(inc_max, us);
    if (k > 40u) {
      inc_max_steady = std::max(inc_max_steady, us);
    }
  }
  const auto t1 = clock::now();
  constexpr int kBatchRuns = 50;
  for (int i = 0; i < kBatchRuns; i++) {
    static_cast<void>(fit.solve());
  }
  const double batch_us =
      std::chrono::duration<double, std::micro>(clock::now() - t1).count() /
      kBatchRuns;
  std::printf("  incremental push+solve: mean %.2f us, max %.2f us (max %.2f "
              "us after warm-up)\n  batch solve() at N = 2048: %.1f us\n",
              inc_total / static_cast<double>(tr.pulses), inc_max,
              inc_max_steady, batch_us);
  double worst = 0.0;
  check(same_result(fit.solve_incremental(), fit.solve(), 1e-4, &worst),
        "2048-pulse train: incremental equals batch");
}

// --------------------------------------------------- streaming pipelines

/** Synthetic LEFT signal: noise, hum, and motor-like pulses. */
std::vector<int32_t> synthetic_motor(double seconds) {
  constexpr double kFs = 48000.0;
  // Onsets on exact integer samples (0.3 s + k s), so an exact watch.
  constexpr std::size_t kOnset = 14400u;
  constexpr std::size_t kStep = 48000u;
  constexpr std::size_t kWidth = 192u; // 4 ms
  std::vector<int32_t> x(static_cast<std::size_t>(seconds * kFs));
  Noise rng(42u);
  for (std::size_t n = 0; n < x.size(); n++) {
    double v =
        10000.0 * rng.next() + 16000.0 * std::sin(2 * kPi * 60 * n / kFs);
    if (n >= kOnset && (n - kOnset) % kStep < kWidth) {
      // Leading spike (decaying) plus a 1024 Hz chopped tail.
      const std::size_t k = (n - kOnset) / kStep;
      const double tp = static_cast<double>((n - kOnset) % kStep) / kFs;
      const double s = (k & 1u) ? -1.0 : 1.0;
      v += s * 600000.0 * std::exp(-tp / 60e-6);
      if (std::fmod(tp, 1.0 / 1024.0) < 0.5 / 1024.0) {
        v += s * 50000.0;
      }
    }
    x[n] = static_cast<int32_t>(std::lround(v));
  }
  return x;
}

void test_motor_block_invariance() {
  std::printf("Motor pipeline: block size does not change the result\n");
  const auto x = synthetic_motor(12.0);
  static quartz::MotorTiming a;
  static quartz::MotorTiming b;
  check(a.configure(48000u, {}) && b.configure(48000u, {}), "configure");
  for (std::size_t i = 0; i < x.size(); i++) {
    a.process(&x[i], 1u); // one sample at a time
  }
  for (std::size_t i = 0; i < x.size(); i += 240u) {
    b.process(&x[i], std::min<std::size_t>(240u, x.size() - i));
  }
  check(a.pulse_count() == 12u && b.pulse_count() == 12u, "12 pulses");
  bool same = a.fit().size() == b.fit().size();
  for (std::size_t i = 0; same && i < a.fit().size(); i++) {
    same = a.fit().time_samples(i) == b.fit().time_samples(i);
  }
  check(same, "bit-identical timestamps for 1- and 240-sample blocks");
  const auto r = b.solve();
  check(r && r->extra_pulses == 0u && !r->clipped, "no extras, no clip");
  if (r) {
    near(r->rate_without_inhibition_s_per_day, 0.0, 0.01,
         "rate of an exact synthetic watch");
  }
}

void test_tone_unwrap() {
  std::printf("Tone tracker: unwrap across blocks, exact rational f\n");
  static quartz::ToneTiming tt;
  struct Case {
    quartz::Rational f;
    double ppm;
    const char *what;
  };
  const std::array<Case, 3> cases = {{
      {{480u, 1u}, 2000.0, "480 Hz +2000 ppm (1.5 rad/block, wraps)"},
      {{1024u, 3u}, 5.0, "1024/3 Hz +5 ppm (fold b = 1125)"},
      {{300u, 1u}, -40.0, "300 Hz -40 ppm"},
  }};
  constexpr uint32_t kFs = 48000u;
  for (const Case &c : cases) {
    check(tt.configure(kFs, {c.f, quartz::ToneTiming::kForkBlockSeconds}),
          "configure");
    const double f = c.f.hz() * (1.0 + 1e-6 * c.ppm);
    std::array<int32_t, 240> blk{};
    std::size_t n = 0;
    for (std::size_t b = 0; b < 10u * kFs / 240u; b++) {
      for (auto &v : blk) {
        v = static_cast<int32_t>(
            std::lround(1e6 * std::sin(2.0 * kPi * f * n / kFs + 0.4)));
        n++;
      }
      tt.process(blk.data(), blk.size());
    }
    const auto r = tt.result();
    check(r.has_value(), "tone result");
    if (r) {
      // f_meas/f_nom - 1 = ppm·1e-6 exactly.
      near(r->rate_s_per_day, 0.0864 * c.ppm, 2e-4, c.what);
    }
  }
}

/**
 * LCD-like spike train: one spike per transition of a square wave, sign
 * alternating. Each spike is a Gaussian (σ = 50 µs), i.e. band-limited, so
 * its sampled harmonics carry the transition time exactly; a spike that
 * starts abruptly on a sample would alias and bias the phase as the edges
 * drift across sample boundaries.
 */
std::vector<int32_t> synthetic_lcd(uint32_t pattern_hz, double ppm,
                                   double seconds) {
  constexpr double kFs = 48000.0;
  std::vector<double> y(static_cast<std::size_t>(seconds * kFs), 0.0);
  const double half_period = 1.0 / (2.0 * pattern_hz * (1.0 + 1e-6 * ppm));
  constexpr double kSigma = 50e-6;
  for (int e = 0;; e++) {
    const double te = 0.0123 + e * half_period;
    if (te >= seconds) {
      break;
    }
    const double s = (e & 1) ? 1.0 : -1.0;
    const auto nc = static_cast<long>(std::lround(te * kFs));
    for (long n = nc - 24; n <= nc + 24; n++) {
      if (n >= 0 && static_cast<std::size_t>(n) < y.size()) {
        const double u = (static_cast<double>(n) / kFs - te) / kSigma;
        y[static_cast<std::size_t>(n)] += s * 1e6 * std::exp(-0.5 * u * u);
      }
    }
  }
  Noise rng(9u);
  std::vector<int32_t> x(y.size());
  for (std::size_t n = 0; n < y.size(); n++) {
    x[n] = static_cast<int32_t>(std::lround(
        y[n] + 300.0 * rng.next() + 5000.0 * std::sin(2 * kPi * 60 * n / kFs)));
  }
  return x;
}

void test_lcd_candidates() {
  std::printf("LCD pipeline: 128 and 16 Hz patterns (not in the vectors)\n");
  static quartz::LcdTiming lt;
  for (uint32_t hz : {128u, 16u}) {
    // 20 s at 0.5 pF-like spikes: noise-limited sigma ~0.0003 s/d at 16 Hz.
    const auto x = synthetic_lcd(hz, 3.0, 20.0);
    check(lt.configure(48000u), "configure");
    for (std::size_t i = 0; i < x.size(); i += 240u) {
      lt.process(&x[i], std::min<std::size_t>(240u, x.size() - i));
    }
    const auto r = lt.result();
    char what[64];
    std::snprintf(what, sizeof what, "pattern %u Hz detected", hz);
    check(r.pattern_hz == hz && r.status == quartz::LcdStatus::kTracking, what);
    if (r.rate) {
      std::snprintf(what, sizeof what, "%u Hz pattern, +3 ppm (s/d)", hz);
      near(r.rate->rate_s_per_day, 0.2592, 0.002, what);
    } else {
      check(false, "LCD rate available");
    }
  }
}

// --------------------------------------------------------- variant reuse

static_assert(std::is_nothrow_default_constructible_v<quartz::MotorTiming>,
              "MotorTiming must be noexcept default-constructible");
static_assert(std::is_nothrow_default_constructible_v<quartz::LcdTiming>,
              "LcdTiming must be noexcept default-constructible");
static_assert(std::is_nothrow_default_constructible_v<quartz::ToneTiming>,
              "ToneTiming must be noexcept default-constructible");

/** The Phase 3 storage: one pipeline at a time, in place. */
using Engine = std::variant<std::monostate, quartz::MotorTiming,
                            quartz::LcdTiming, quartz::ToneTiming>;

/** Feeds a signal in 240-frame blocks. */
template <typename Pipeline>
void feed(Pipeline &p, const std::vector<int32_t> &x) {
  for (std::size_t i = 0; i < x.size(); i += 240u) {
    p.process(&x[i], std::min<std::size_t>(240u, x.size() - i));
  }
}

/** A clean sine tone as codes. */
std::vector<int32_t> synthetic_tone(double hz, double seconds) {
  std::vector<int32_t> x(static_cast<std::size_t>(seconds * 48000.0));
  for (std::size_t n = 0; n < x.size(); n++) {
    x[n] = static_cast<int32_t>(
        std::lround(1e6 * std::sin(2.0 * kPi * hz * n / 48000.0 + 0.4)));
  }
  return x;
}

void test_variant_cycle() {
  std::printf("std::variant reuse: Motor -> LCD -> Tone, twice\n");
  const auto motor_x = synthetic_motor(12.0);
  const auto lcd_x = synthetic_lcd(64u, 3.0, 20.0);
  const auto tone_x = synthetic_tone(480.0 * (1.0 + 5e-6), 10.0);
  const quartz::ToneTiming::Config tone_cfg{
      {480u, 1u}, quartz::ToneTiming::kForkBlockSeconds};

  // Reference results from stand-alone instances.
  static quartz::MotorTiming ref_m;
  static quartz::LcdTiming ref_l;
  static quartz::ToneTiming ref_t;
  check(ref_m.configure(48000u, {}) && ref_l.configure(48000u) &&
            ref_t.configure(48000u, tone_cfg),
        "configure stand-alone pipelines");
  feed(ref_m, motor_x);
  feed(ref_l, lcd_x);
  feed(ref_t, tone_x);
  const auto want_m = ref_m.solve();
  const auto want_l = ref_l.result().rate;
  const auto want_t = ref_t.result();
  check(want_m && want_l && want_t, "stand-alone results exist");
  if (!want_m || !want_l || !want_t) {
    return;
  }

  static Engine engine; // starts as std::monostate
  std::printf("  sizeof(Engine) = %zu bytes\n", sizeof(Engine));
  for (int round = 0; round < 2; round++) {
    auto &m = engine.emplace<quartz::MotorTiming>();
    check(m.configure(48000u, {}), "motor configure in variant");
    feed(m, motor_x);
    const auto got_m = m.solve();
    check(got_m && m.pulse_count() == 12u &&
              got_m->rate_without_inhibition_s_per_day ==
                  want_m->rate_without_inhibition_s_per_day,
          "motor in variant: 12 pulses, identical rate");

    auto &l = engine.emplace<quartz::LcdTiming>();
    check(l.configure(48000u), "LCD configure in variant");
    feed(l, lcd_x);
    const auto got_l = l.result();
    check(got_l.pattern_hz == 64u && got_l.rate &&
              got_l.rate->rate_s_per_day == want_l->rate_s_per_day,
          "LCD in variant: 64 Hz, identical rate");

    auto &t = engine.emplace<quartz::ToneTiming>();
    check(t.configure(48000u, tone_cfg), "tone configure in variant");
    feed(t, tone_x);
    const auto got_t = t.result();
    check(got_t && got_t->rate_s_per_day == want_t->rate_s_per_day,
          "tone in variant: identical rate");
    if (got_l.rate && got_t) {
      std::printf("  round %d: motor %+.5f  lcd %+.5f  tone %+.5f s/d\n", round,
                  got_m ? got_m->rate_without_inhibition_s_per_day : 0.0,
                  got_l.rate->rate_s_per_day, got_t->rate_s_per_day);
    }
  }
  near(want_l->rate_s_per_day, 0.2592, 0.002, "LCD 64 Hz +3 ppm (s/d)");
  near(want_t->rate_s_per_day, 0.432, 2e-4, "tone 480 Hz +5 ppm (s/d)");
  engine.emplace<std::monostate>();
}

// ---------------------------------------------------------- calibration

void test_ppm() {
  std::printf("Calibration helper\n");
  // Exact watch, board clock fast by 1 ppm: the watch's 1 s period lasts
  // 1 + 1e-6 board seconds and reads as losing.
  const double board = quartz::rate_s_per_day(1.0, 1.0 + 1e-6);
  near(board, -0.0864, 1e-6, "exact watch on a +1 ppm board reads -0.0864");
  near(quartz::apply_timebase_ppm(board, 1.0), 0.0, 1e-9,
       "apply_timebase_ppm(+1 ppm) restores 0");
  near(quartz::apply_timebase_ppm(0.5, 0.0), 0.5, 0.0, "0 ppm is identity");
  near(quartz::apply_timebase_ppm(0.0, -2.0), -0.1728, 1e-9,
       "board slow by 2 ppm: subtract 0.1728");
}

} // namespace

int main() {
  test_hpf_coefficients();
  test_hpf_magnitude();
  test_reduce_ratio();
  test_goertzel_phase();
  test_unwrap();
  test_threshold_timestamp();
  test_xcorr();
  test_motor_fit();
  test_incremental_random();
  test_long_gate_boundary();
  test_incremental_overflow();
  test_incremental_step_change();
  test_incremental_timing();
  test_motor_block_invariance();
  test_tone_unwrap();
  test_lcd_candidates();
  test_variant_cycle();
  test_ppm();
  std::printf("\n%s: %d failure(s)\n", g_failures == 0 ? "PASS" : "FAIL",
              g_failures);
  return g_failures == 0 ? 0 : 1;
}
