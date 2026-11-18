/**
 * @file test_ui_format.cpp
 * @brief Host test for OLED readout formatting.
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_ui_format.cpp -o /tmp/t && /tmp/t
 */

#include "ui_format.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

char buf[16];

const char *thousands(uint32_t v) {
  ui_format_thousands(v, buf, sizeof(buf));
  return buf;
}
const char *rate(bool ok, float v) {
  ui_format_rate(ok, v, buf, sizeof(buf));
  return buf;
}
const char *amp(bool ok, float v) {
  ui_format_amp(ok, v, buf, sizeof(buf));
  return buf;
}
const char *lift(float v) {
  ui_format_lift(v, buf, sizeof(buf));
  return buf;
}
const char *be(bool ok, float v) {
  ui_format_be(ok, v, buf, sizeof(buf));
  return buf;
}

void eq(const char *got, const char *want) {
  if (std::strcmp(got, want) != 0) {
    std::printf("FAIL: got '%s' want '%s'\n", got, want);
    assert(false);
  }
}

} // namespace

int main() {
  eq(thousands(0), "0");
  eq(thousands(999), "999");
  eq(thousands(18000), "18,000");
  eq(thousands(28800), "28,800");
  eq(thousands(57600), "57,600");
  eq(thousands(1234567), "1,234,567");

  eq(rate(false, 3.f), "---");
  eq(rate(true, NAN), "---");
  eq(rate(true, 12.34f), "+12.3");
  eq(rate(true, -4.06f), "-4.1");
  eq(rate(true, -0.04f), "+0.0");
  eq(rate(true, 99.94f), "+99.9");
  eq(rate(true, 99.96f), "+100");
  eq(rate(true, -456.4f), "-456");
  eq(rate(true, 1500.f), ">+999");
  eq(rate(true, -2000.f), "<-999");
  for (float v = -1500.f; v < 1500.f; v += 0.37f) {
    assert(std::strlen(rate(true, v)) <= 5u);
  }

  eq(amp(true, 284.6f), "285");
  eq(amp(false, 284.6f), "---");

  eq(be(true, 0.42f), "0.4");
  eq(be(true, 9.96f), "10");
  eq(be(false, 1.f), "-.-");

  eq(lift(52.f), "52");
  eq(lift(51.98f), "52");
  eq(lift(52.5f), "52.5");
  eq(lift(NAN), "--");
  eq(lift(0.f), "--");

  {
    const float empty[] = {NAN, NAN};
    assert(ui_trend_scale(empty, 0) == 2.f);
    assert(ui_trend_scale(empty, 2) == 2.f);
    const float small[] = {0.5f, -1.9f, NAN};
    assert(ui_trend_scale(small, 3) == 2.f);
    const float mid[] = {3.f, -4.99f, NAN, 1.f};
    assert(ui_trend_scale(mid, 4) == 5.f);
    const float edge[] = {-20.f};
    assert(ui_trend_scale(edge, 1) == 20.f);
    const float big[] = {20.1f};
    assert(ui_trend_scale(big, 1) == 50.f);
    const float huge[] = {5000.f};
    assert(ui_trend_scale(huge, 1) == 1000.f);
  }

  std::printf("test_ui_format: OK\n");
  return 0;
}
