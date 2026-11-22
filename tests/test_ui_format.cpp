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

const char *fixed(bool ok, float v, int d) {
  ui_format_signed_fixed(ok, v, d, buf, sizeof(buf));
  return buf;
}
const char *qrate(bool ok, float v) {
  ui_format_quartz_rate(ok, v, buf, sizeof(buf));
  return buf;
}
const char *secs(bool ok, float v) {
  ui_format_seconds(ok, v, buf, sizeof(buf));
  return buf;
}
const char *hz(float v) {
  ui_format_hz(v, buf, sizeof(buf));
  return buf;
}
const char *jump(bool ok, float us, uint32_t count) {
  ui_format_jump(ok, us, count, 'u', buf, sizeof(buf));
  return buf;
}
const char *gate(float e, float g, bool done) {
  ui_format_gate(e, g, done, buf, sizeof(buf));
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

  /* ---- quartz ---- */
  eq(fixed(true, 0.2731f, 3), "+0.273");
  eq(fixed(true, -0.0004f, 3), "+0.000");
  eq(fixed(true, -0.0006f, 3), "-0.001");
  eq(fixed(true, -12.4f, 0), "-12");
  eq(fixed(true, 1.f, 9), "+1.000000"); /* decimals clamped to 6 */
  eq(fixed(false, 1.f, 2), "---");
  eq(fixed(true, INFINITY, 2), "---");

  eq(qrate(false, 0.1f), "---");
  eq(qrate(true, NAN), "---");
  eq(qrate(true, 0.2731f), "+0.273");
  eq(qrate(true, -0.0002f), "+0.000");
  eq(qrate(true, -3.1234f), "-3.123");
  eq(qrate(true, 9.9994f), "+9.999");
  eq(qrate(true, 9.9996f), "+10.00");
  eq(qrate(true, -45.678f), "-45.68");
  eq(qrate(true, 99.996f), "+100.0");
  eq(qrate(true, -456.44f), "-456.4");
  eq(qrate(true, 999.96f), ">+999");
  eq(qrate(true, -5000.f), "<-999");
  for (float v = -1200.f; v < 1200.f; v += 0.0137f) {
    assert(std::strlen(qrate(true, v)) <= kUiQuartzRateMaxChars);
  }
  for (float v = -12.f; v < 12.f; v += 0.000071f) {
    assert(std::strlen(qrate(true, v)) <= kUiQuartzRateMaxChars);
  }

  eq(secs(true, 60.f), "60.0");
  eq(secs(true, 9.96f), "10.0");
  eq(secs(true, 99.96f), "100");
  eq(secs(true, 960.f), "960");
  eq(secs(false, 60.f), "--");
  eq(secs(true, 0.f), "--");
  eq(secs(true, NAN), "--");

  eq(hz(64.f), "64");
  eq(hz(32768.f), "32768");
  eq(hz(440.5f), "440.5");
  eq(hz(0.f), "--");
  eq(hz(NAN), "--");

  eq(jump(true, 244.14f, 2), "244.1us n=2");
  eq(jump(true, 999.94f, 8), "999.9us n=8");
  eq(jump(true, 7324.2f, 60), "7324us n=60");
  eq(jump(true, 0.f, 0), "--");
  eq(jump(false, 244.f, 2), "--");
  {
    char b[24];
    ui_format_jump(true, 122.07f, 1, static_cast<char>(230), b, sizeof(b));
    assert(std::strcmp(b, "122.1\xe6s n=1") == 0);
  }

  eq(gate(23.7f, 60.f, false), "23/60s");
  eq(gate(0.f, 60.f, false), "0/60s");
  eq(gate(60.4f, 60.f, false), "60/60s");
  eq(gate(61.2f, 60.f, false), "60/60s"); /* capped until DONE */
  eq(gate(123.9f, 0.f, false), "123s");   /* open-ended gate */
  eq(gate(5.f, NAN, false), "5s");
  eq(gate(60.f, 60.f, true), "done");
  eq(gate(NAN, 60.f, false), "--");
  eq(gate(-1.f, 60.f, false), "--");

  eq(ui_quartz_mode_tag(DSP_QUARTZ_MODE_MOTOR), "MOTOR");
  eq(ui_quartz_mode_tag(DSP_QUARTZ_MODE_LCD), "LCD");
  eq(ui_quartz_mode_tag(DSP_QUARTZ_MODE_FORK), "FORK");
  eq(ui_quartz_mode_tag(DSP_QUARTZ_MODE_COUNT), "QUARTZ");

  eq(ui_quartz_clip_hint(DSP_QUARTZ_MODE_MOTOR), "CLIP: open JP1");
  eq(ui_quartz_clip_hint(DSP_QUARTZ_MODE_FORK), "CLIP: open JP1");
  eq(ui_quartz_clip_hint(DSP_QUARTZ_MODE_LCD), "CLIP: open JP2");
  for (int m = 0; m < DSP_QUARTZ_MODE_COUNT; m++) {
    assert(std::strlen(ui_quartz_clip_hint(static_cast<dsp_quartz_mode_t>(m))) <= kUiStatusMaxChars);
  }

  {
    const auto M = DSP_QUARTZ_MODE_MOTOR;
    const auto MEAS = DSP_QUARTZ_STATE_MEASURING;
    eq(ui_quartz_status_text(false, false, M, MEAS, DSP_QUARTZ_INHIB_CORRECTED), "no host stream");
    eq(ui_quartz_status_text(false, true, M, MEAS, DSP_QUARTZ_INHIB_CORRECTED), "stopped");
    eq(ui_quartz_status_text(false, false, M, DSP_QUARTZ_STATE_DONE, DSP_QUARTZ_INHIB_CORRECTED),
       "corrected");
    eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_WAIT_TIMEBASE, DSP_QUARTZ_INHIB_NONE),
       "clock check...");
    eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_UNSUPPORTED_RATE, DSP_QUARTZ_INHIB_NONE),
       "unsupported rate");
    eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_ACQUIRING, DSP_QUARTZ_INHIB_NONE),
       "acquiring...");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_NONE), "acquiring...");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE), "no inhibition");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION),
       "may incl. inhib.");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE),
       "need 1 more cycle");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_CORRECTION_PENDING), "correcting...");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_CORRECTED), "corrected");
    eq(ui_quartz_status_text(true, false, DSP_QUARTZ_MODE_LCD, MEAS, DSP_QUARTZ_INHIB_CORRECTED),
       "measuring");
    /* Every combination fits one size-1 row. */
    for (int st = 0; st <= 3; st++) {
      for (int m = 0; m < DSP_QUARTZ_MODE_COUNT; m++) {
        for (int s = 0; s < DSP_QUARTZ_STATE_COUNT; s++) {
          for (int i = 0; i < DSP_QUARTZ_INHIB_COUNT; i++) {
            const char *t = ui_quartz_status_text((st & 1) != 0, (st & 2) != 0,
                                                  static_cast<dsp_quartz_mode_t>(m),
                                                  static_cast<dsp_quartz_state_t>(s),
                                                  static_cast<dsp_quartz_inhibition_t>(i));
            assert(t != nullptr && std::strlen(t) <= kUiStatusMaxChars);
          }
        }
      }
    }
  }

  std::printf("test_ui_format: OK\n");
  return 0;
}
