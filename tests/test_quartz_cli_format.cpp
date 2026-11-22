/**
 * @file test_quartz_cli_format.cpp
 * @brief Host test for the quartz CLI parsing / formatting helpers.
 *
 * Header-only, no objects to link. Built and run by
 * tests/build_quartz_tests.sh, or alone:
 *   c++ -std=c++17 -Wall -Wextra -Iinclude tests/test_quartz_cli_format.cpp \
 *     -o /tmp/t && /tmp/t
 */

#include "cli/quartz_format.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

char buf[64];

void eq(const char *got, const char *want) {
  if (std::strcmp(got, want) != 0) {
    std::printf("FAIL: got '%s' want '%s'\n", got, want);
    assert(false);
  }
}

} // namespace

int main() {
  float v = -1.f;

  // Gate: auto (any case) = until two events; numbers pass through (the
  // engine owns the 3..960 s range and reports outOfRange itself).
  assert(Cli::quartzParseGate("auto", v) && v == 0.f);
  v = -1.f;
  assert(Cli::quartzParseGate("AUTO", v) && v == 0.f);
  assert(Cli::quartzParseGate("0", v) && v == 0.f);
  assert(Cli::quartzParseGate("60", v) && v == 60.f);
  assert(Cli::quartzParseGate("2.5", v) && v == 2.5f); // engine refuses it
  v = 7.f;
  assert(!Cli::quartzParseGate("10s", v));
  assert(!Cli::quartzParseGate("", v));
  assert(!Cli::quartzParseGate(nullptr, v));
  assert(!Cli::quartzParseGate("nan", v));
  assert(!Cli::quartzParseGate("inf", v));
  assert(v == 7.f); // untouched on failure

  // Plain numbers (fork Hz, kSigma).
  assert(Cli::quartzParseNumber("341.33", v) && v == 341.33f);
  assert(Cli::quartzParseNumber("-4", v) && v == -4.f);
  assert(!Cli::quartzParseNumber("8x", v));
  assert(!Cli::quartzParseNumber("auto", v));

  // Formatting.
  eq(Cli::quartzFormatGate(0.f, buf, sizeof(buf)), "auto");
  eq(Cli::quartzFormatGate(60.f, buf, sizeof(buf)), "60s");
  eq(Cli::quartzFormatGate(12.5f, buf, sizeof(buf)), "12.5s");
  eq(Cli::quartzFormatHz(360.0, buf, sizeof(buf)), "360");
  eq(Cli::quartzFormatHz(1024.0 / 3.0, buf, sizeof(buf)), "341.33");
  eq(Cli::quartzFormatForkList(buf, sizeof(buf)), "300|341.33|360|480|720");
  char small[8];
  Cli::quartzFormatForkList(small, sizeof(small)); // truncates, stays NUL'd
  assert(std::strlen(small) < sizeof(small));
  eq(Cli::quartzFormatRate(true, -0.4123f, buf, sizeof(buf)), "-0.412");
  eq(Cli::quartzFormatRate(true, 3.1f, buf, sizeof(buf)), "3.100");
  eq(Cli::quartzFormatRate(false, 3.1f, buf, sizeof(buf)), "n/a");
  eq(Cli::quartzFormatRate(true, NAN, buf, sizeof(buf)), "n/a");

  // Every §5 status has a distinct human message.
  for (int i = 0; i < DSP_QUARTZ_INHIB_COUNT; i++) {
    const char *a =
        Cli::quartzInhibitionMessage(static_cast<dsp_quartz_inhibition_t>(i));
    assert(std::strcmp(a, "unknown") != 0);
    for (int j = 0; j < i; j++) {
      assert(std::strcmp(a, Cli::quartzInhibitionMessage(
                                static_cast<dsp_quartz_inhibition_t>(j))) != 0);
    }
  }
  eq(Cli::quartzInhibitionMessage(DSP_QUARTZ_INHIB_COUNT), "unknown");
  eq(Cli::quartzInhibitionMessage(DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE),
     "no inhibition detected");

  // Engine constants the help text quotes.
  static_assert(quartz::engine::kNoJumpSpanS ==
                    2.0 * quartz::engine::kLongestInhibitionPeriodS,
                "help says 2x the longest period");

  std::printf("test_quartz_cli_format: OK\n");
  return 0;
}
