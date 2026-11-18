/**
 * @file test_pps_filter.cpp
 * @brief Host test for GPS PPS period averaging.
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_pps_filter.cpp -o /tmp/t && /tmp/t
 */

#include "pps_filter.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

constexpr uint32_t kNominal = 150000000u;

/** Cycles counted in one true second by a clock {@code ppm} fast, ±jitter. */
uint32_t period(double ppm, int jitter) {
  return static_cast<uint32_t>(std::llround(kNominal * (1.0 + ppm * 1e-6))) +
         static_cast<uint32_t>(jitter);
}

} // namespace

int main() {
  static pps_filter_t f;
  pps_filter_init(&f, kNominal);
  assert(!pps_filter_ready(&f) && pps_filter_ppm(&f) == 0.0);

  /* Crystal 23.4 ppm fast, ±8 cycles (~50 ns) of PPS / edge jitter. */
  for (int i = 0; i < 100; i++) {
    const int jitter = (i * 7919) % 17 - 8;
    assert(pps_filter_push(&f, period(23.4, jitter)));
  }
  assert(pps_filter_ready(&f) && f.n == PPS_FILTER_CAP);
  std::printf("  ppm %.5f (want 23.4)\n", pps_filter_ppm(&f));
  assert(std::fabs(pps_filter_ppm(&f) - 23.4) < 0.01);

  /* Missed pulse (2 s period) and a glitch (0.3 s) are rejected. */
  const uint32_t good_before = f.good;
  assert(!pps_filter_push(&f, 2u * kNominal));
  assert(!pps_filter_push(&f, kNominal / 3u));
  assert(f.rejects == 2u && f.good == good_before);
  assert(std::fabs(pps_filter_ppm(&f) - 23.4) < 0.01);

  /* Window slides: a new error replaces the old one after 64 s. */
  for (int i = 0; i < 64; i++) {
    pps_filter_push(&f, period(-12.0, 0));
  }
  assert(std::fabs(pps_filter_ppm(&f) + 12.0) < 0.01);

  /* Beyond ±200 ppm is rejected outright. */
  assert(!pps_filter_push(&f, period(250.0, 0)));

  pps_filter_clear(&f);
  assert(!pps_filter_ready(&f) && f.n == 0u);

  std::printf("test_pps_filter: OK\n");
  return 0;
}
