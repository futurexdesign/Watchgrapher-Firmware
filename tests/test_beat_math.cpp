/**
 * @file test_beat_math.cpp
 * @brief Host test for the pure beat-rate and beat-error-angle formulas in
 *        {@code mechanical/beat_analysis.h} (docs/plans/TIMING_ENGINE_PLAN.md,
 *        item 8).
 *
 * Built and run by tests/run_all.sh (entry {@code beat_math}).
 */

#include "mechanical/beat_analysis.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace {

/** Absolute tolerance for float results (degrees, s/d). */
constexpr double kTol = 1e-4;

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

/* A watch 20 s/d fast reads +20 s/d with the M2 form. */
void test_rate_fast_watch() {
  const double nominal = 3600.0 / 28800.0;
  const double measured = nominal / (1.0 + 20.0 / 86400.0);
  const double rate = beat_rate_s_day(nominal, measured);
  std::printf("rate fast 20 s/d: %.9f s/d\n", rate);
  assert(near(rate, 20.0, 1e-9));
}

/* The M2 form is exact at +-20 s/d. The old (1 - P/Pn) form differs by
 * about 0.005 s/d there. */
void test_rate_form_difference() {
  const double nominal = 3600.0 / 21600.0;
  for (const double r : {20.0, -20.0}) {
    const double p = nominal / (1.0 + r / 86400.0);
    const double m2 = beat_rate_s_day(nominal, p);
    const double old_form = (1.0 - p / nominal) * 86400.0;
    std::printf("rate %+5.1f s/d: M2 %.9f  old %.9f  diff %.6f\n", r, m2,
                old_form, m2 - old_form);
    assert(near(m2, r, 1e-9));
    assert(std::fabs(m2 - old_form) > 0.001 && std::fabs(m2 - old_form) < 0.01);
  }
}

/* Equal half-periods give a zero rate. */
void test_rate_zero() { assert(near(beat_rate_s_day(0.1, 0.1), 0.0, 1e-12)); }

/* 280 deg at 28800 A/h with 0.5 ms BE: 280 * pi * 4 * 0.0005 deg. */
void test_angle_value() {
  const float a = beat_error_angle_deg(280.f, 28800u, 0.5f);
  const double expect = 280.0 * 3.14159265358979323846 * 4.0 * 0.0005;
  std::printf("BE angle 280 deg, 28800 A/h, 0.5 ms: %.6f deg (expect %.6f)\n",
              static_cast<double>(a), expect);
  assert(near(static_cast<double>(a), expect, kTol));
}

/* The angle is linear in amplitude and in beat error, and 0 for zero inputs. */
void test_angle_linearity() {
  const float base = beat_error_angle_deg(250.f, 21600u, 0.3f);
  assert(near(static_cast<double>(beat_error_angle_deg(500.f, 21600u, 0.3f)),
              2.0 * static_cast<double>(base), kTol));
  assert(near(static_cast<double>(beat_error_angle_deg(250.f, 21600u, 0.6f)),
              2.0 * static_cast<double>(base), kTol));
  assert(beat_error_angle_deg(0.f, 21600u, 0.3f) == 0.f);
  assert(beat_error_angle_deg(250.f, 21600u, 0.f) == 0.f);
  assert(beat_error_angle_deg(250.f, 0u, 0.3f) == 0.f);
}

} // namespace

int main() {
  test_rate_fast_watch();
  test_rate_form_difference();
  test_rate_zero();
  test_angle_value();
  test_angle_linearity();
  std::printf("beat_math: all checks passed\n");
  return 0;
}
