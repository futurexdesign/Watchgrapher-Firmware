/**
 * @file test_beat_fit.cpp
 * @brief Host test for the least-squares beat timing fit.
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_beat_fit.cpp
 * src/mechanical/beat_fit.cpp -o /tmp/t && /tmp/t
 */

#include "mechanical/beat_fit.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

namespace {

constexpr double kFs = 24000.0;

/** Deterministic uniform noise in [-1, 1). */
double noise(uint32_t &state) {
  state = state * 1664525u + 1013904223u;
  return static_cast<double>(state >> 8) / 8388608.0 - 1.0;
}

struct Sim {
  uint32_t vph;
  double rate_s_day;    /* true watch rate */
  double be_ms;         /* true beat error */
  double jitter_samp;   /* +- uniform landmark jitter */
  bool quantize;        /* round to whole samples (old resolution) */
  uint32_t drop_every;  /* drop every Nth tick (0 = none) */
  uint32_t spur_every;  /* add a spurious mid-beat event every Nth (0 = none) */
  double seconds;
  double t0;            /* absolute start (samples) */
};

struct Out {
  double rate_s_day;
  double be_ms;
  double jitter_us;
  uint32_t points;
  bool ok;
};

Out run(const Sim &s, double window_s = 8.0) {
  static beat_fit_t f;
  beat_fit_reset(&f);
  const double nominal = kFs * 3600.0 / static_cast<double>(s.vph);
  const double true_half = nominal * (1.0 - s.rate_s_day / 86400.0);
  const double c = 0.5e-3 * s.be_ms * kFs;
  uint32_t rng = 12345u;
  const auto beats = static_cast<uint32_t>(s.seconds * kFs / true_half);
  for (uint32_t k = 0; k < beats; k++) {
    double t = s.t0 + true_half * k + ((k & 1u) ? -c : c) +
               s.jitter_samp * noise(rng);
    if (s.quantize) {
      t = std::floor(t + 0.5);
    }
    if (s.drop_every && k % s.drop_every == s.drop_every - 1u) {
      continue;
    }
    beat_fit_push(&f, t, nominal, window_s * kFs);
    if (s.spur_every && k % s.spur_every == 3u) {
      beat_fit_push(&f, t + 0.45 * true_half, nominal, window_s * kFs);
    }
  }
  beat_fit_result_t r{};
  Out o{};
  o.ok = beat_fit_solve(&f, &r);
  o.rate_s_day = -static_cast<double>(r.half_period_delta) / nominal * 86400.0;
  o.be_ms = r.alt_valid ? 2.0 * std::fabs(r.alt_offset) / kFs * 1000.0 : -1.0;
  o.jitter_us = r.resid_rms / kFs * 1e6;
  o.points = r.points;
  return o;
}

void expect_near(const char *what, double got, double want, double tol) {
  std::printf("  %-16s got %9.4f want %9.4f\n", what, got, want);
  if (std::fabs(got - want) > tol) {
    std::printf("FAIL %s: got %.4f want %.4f (tol %.4f)\n", what, got, want,
                tol);
    assert(false);
  }
}

} // namespace

int main() {
  /* Clean, sub-sample timestamps: exact recovery. */
  {
    Out o = run({28800, 12.5, 0.8, 0.0, false, 0, 0, 10.0, 1000.0});
    assert(o.ok);
    expect_near("clean rate", o.rate_s_day, 12.5, 0.05);
    expect_near("clean be", o.be_ms, 0.8, 0.005);
  }

  /*
   * Whole-sample timestamps (what the old median estimator saw). One sample
   * on a 28,800 A/h period at 24 kHz is ~14 s/d; the fit must do far better.
   */
  for (double rate : {-37.0, -3.2, 0.0, 4.4, 21.7}) {
    Out o = run({28800, rate, 0.4, 0.0, true, 0, 0, 10.0, 7.0});
    assert(o.ok);
    expect_near("quantized rate", o.rate_s_day, rate, 0.5);
    expect_near("quantized be", o.be_ms, 0.4, 0.05);
  }

  /* +-0.5 ms uniform landmark jitter, several beat rates. */
  for (uint32_t vph : {18000u, 21600u, 28800u, 36000u}) {
    Out o = run({vph, -8.0, 1.5, 0.0005 * kFs, true, 0, 0, 12.0, 0.0});
    assert(o.ok);
    /* ~0.29 ms RMS over 40..80 ticks: sigma ~1.7 s/d at 18,000 A/h. */
    expect_near("jitter rate", o.rate_s_day, -8.0, 5.0);
    expect_near("jitter be", o.be_ms, 1.5, 0.15);
  }

  /* Missed ticks and spurious mid-beat detections. */
  {
    Out o = run({28800, 5.0, 0.6, 0.0, true, 7, 5, 10.0, 0.0});
    assert(o.ok);
    expect_near("dropouts rate", o.rate_s_day, 5.0, 0.5);
    expect_near("dropouts be", o.be_ms, 0.6, 0.05);
  }

  /* Large absolute timestamps (hours of streaming) keep sub-sample math. */
  {
    Out o = run({28800, 2.0, 0.3, 0.0, false, 0, 0, 10.0, 24000.0 * 3600 * 9});
    assert(o.ok);
    expect_near("late rate", o.rate_s_day, 2.0, 0.05);
    expect_near("late be", o.be_ms, 0.3, 0.005);
  }

  /* Long run: window slides and rebases many times. */
  {
    Out o = run({36000, -15.0, 0.5, 0.0, false, 0, 0, 600.0, 0.0}, 6.0);
    assert(o.ok);
    expect_near("long rate", o.rate_s_day, -15.0, 0.05);
    assert(o.points <= static_cast<uint32_t>(6.0 * 10.0) + 1u);
  }

  /* Too few points. */
  {
    Out o = run({28800, 0.0, 0.0, 0.0, false, 0, 0, 0.5, 0.0});
    assert(!o.ok);
  }

  /* Single parity only (every other tick missed): rate still valid. */
  {
    Out o = run({28800, 6.0, 1.0, 0.0, true, 2, 0, 10.0, 0.0});
    assert(o.ok);
    expect_near("one-parity rate", o.rate_s_day, 6.0, 0.5);
    assert(o.be_ms < 0.0);
  }

  /*
   * Per-point report: grid-absolute k and raw residual vs the nominal grid,
   * stable while the window slides; a fast watch gives decreasing u; a
   * rejected point still reports its off-grid miss.
   */
  {
    static beat_fit_t f;
    beat_fit_reset(&f);
    const double nominal = kFs * 3600.0 / 28800.0;
    const double rate = 20.0; /* s/d, gaining */
    const double true_half = nominal * (1.0 - rate / 86400.0);
    const double t0 = 5000.0;
    beat_fit_point_t pt{};
    for (int32_t k = 0; k < 400; k++) {
      const double t = t0 + true_half * k;
      auto r = beat_fit_push(&f, t, nominal, 2.0 * kFs, &pt);
      assert(r == (k == 0 ? BEAT_FIT_STARTED : BEAT_FIT_ACCEPTED));
      assert(pt.valid && pt.k == k);
      assert(std::fabs(pt.u - (t - t0 - k * nominal)) < 1e-6);
      assert(std::fabs(pt.nominal - nominal) < 1e-12);
    }
    /* Window slid (400 > 2 s of beats): many rebases, k still absolute. */
    assert(f.n < 400u);
    assert(pt.u < 0.0); /* gaining watch: ticks early, u falls */
    expect_near("chart slope us/beat", pt.u / 399.0 / kFs * 1e6,
                -rate / 86400.0 * nominal / kFs * 1e6, 1e-6);

    /* Off-grid point (0.45 beat late): rejected, k = nearest slot. */
    const double tl = t0 + true_half * 399 + 1.45 * true_half;
    auto r = beat_fit_push(&f, tl, nominal, 2.0 * kFs, &pt);
    assert(r == BEAT_FIT_REJECTED);
    assert(pt.valid && pt.k == 400);
    assert(pt.u > 0.0);
    /* Gap longer than max restarts the grid: k and u restart at 0. */
    r = beat_fit_push(&f, tl + 100.0 * nominal, nominal, 2.0 * kFs, &pt);
    assert(r == BEAT_FIT_STARTED && pt.k == 0 && pt.u == 0.0);
  }

  std::printf("test_beat_fit: OK\n");
  return 0;
}
