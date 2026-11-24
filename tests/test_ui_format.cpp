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
    assert(std::strlen(ui_quartz_clip_hint(
               static_cast<dsp_quartz_mode_t>(m))) <= kUiStatusMaxChars);
  }

  {
    const auto M = DSP_QUARTZ_MODE_MOTOR;
    const auto MEAS = DSP_QUARTZ_STATE_MEASURING;
    eq(ui_quartz_status_text(false, false, M, MEAS, DSP_QUARTZ_INHIB_CORRECTED),
       "no stream");
    eq(ui_quartz_status_text(false, true, M, MEAS, DSP_QUARTZ_INHIB_CORRECTED),
       "stopped");
    eq(ui_quartz_status_text(false, false, M, DSP_QUARTZ_STATE_DONE,
                             DSP_QUARTZ_INHIB_CORRECTED),
       "corrected");
    eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_WAIT_TIMEBASE,
                             DSP_QUARTZ_INHIB_NONE),
       "clock check");
    eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_UNSUPPORTED_RATE,
                             DSP_QUARTZ_INHIB_NONE),
       "unsupported rate");
    eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_ACQUIRING,
                             DSP_QUARTZ_INHIB_NONE),
       "acquiring...");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_NONE),
       "acquiring...");
    eq(ui_quartz_status_text(true, false, M, MEAS,
                             DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE),
       "no inhibition");
    eq(ui_quartz_status_text(true, false, M, MEAS,
                             DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION),
       "may incl. inhib.");
    eq(ui_quartz_status_text(true, false, M, MEAS,
                             DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE),
       "need 1 more cycle");
    eq(ui_quartz_status_text(true, false, M, MEAS,
                             DSP_QUARTZ_INHIB_CORRECTION_PENDING),
       "correcting...");
    eq(ui_quartz_status_text(true, false, M, MEAS, DSP_QUARTZ_INHIB_CORRECTED),
       "corrected");
    eq(ui_quartz_status_text(true, false, DSP_QUARTZ_MODE_LCD, MEAS,
                             DSP_QUARTZ_INHIB_CORRECTED),
       "measuring");
    /* Every combination fits one size-1 row. */
    for (int st = 0; st <= 3; st++) {
      for (int m = 0; m < DSP_QUARTZ_MODE_COUNT; m++) {
        for (int s = 0; s < DSP_QUARTZ_STATE_COUNT; s++) {
          for (int i = 0; i < DSP_QUARTZ_INHIB_COUNT; i++) {
            const char *t = ui_quartz_status_text(
                (st & 1) != 0, (st & 2) != 0, static_cast<dsp_quartz_mode_t>(m),
                static_cast<dsp_quartz_state_t>(s),
                static_cast<dsp_quartz_inhibition_t>(i));
            assert(t != nullptr && std::strlen(t) <= kUiStatusMaxChars);
          }
        }
      }
    }
  }

  /* ---- Header layout (UIH-01..08) ---- */
  {
    struct Span {
      int x0, x1; /* [x0, x1) */
    };
    for (int c = 0; c < 8; c++) {
      const bool pps = (c & 1) != 0, ble = (c & 2) != 0, sa = (c & 4) != 0;
      const UiHeaderLayout l = ui_header_layout(pps, ble, sa);
      Span sp[3];
      int n = 0;
      sp[n++] = {l.tag_x, l.tag_end};
      if (l.ble_x >= 0) {
        sp[n++] = {l.ble_x, l.ble_x + kUiBleW};
      }
      if (l.gps_x >= 0) {
        sp[n++] = {l.gps_x, l.gps_x + kUiGpsW};
      }
      for (int i = 0; i < n; i++) { /* UIH-01 */
        assert(sp[i].x0 >= 0 && sp[i].x1 <= kUiPanelW);
        for (int j = i + 1; j < n; j++) {
          const bool disjoint = sp[i].x1 + kUiHeaderGap <= sp[j].x0 ||
                                sp[j].x1 + kUiHeaderGap <= sp[i].x0;
          assert(disjoint);
        }
      }
      assert(l.max_left_px == l.tag_x - kUiHeaderGap);
      assert((l.gps_x >= 0) == pps && (l.ble_x >= 0) == ble);
      if (!ble) { /* UIH-02 */
        assert(l.gps_x == (pps ? 105 : -1));
        assert(l.tag_end == (pps ? 103 : 127));
      }
    }
    /* UIH-03 */
    assert(ui_header_layout(true, true, false).ble_x == 92);
    assert(ui_header_layout(false, true, false).ble_x == 117);
    /* UIH-04 */
    assert(ui_header_layout(true, true, false).max_left_px == 70);

    /* UIH-05: every mechanical header string, worst-case layout. */
    const int worst = ui_header_layout(true, true, false).max_left_px;
    const char *mech[] = {"starting...", "no stream", "stopped",
                          "clock check", "listening", "locking..."};
    for (const char *s : mech) {
      assert(static_cast<int>(std::strlen(s)) * kUiCharW <= worst);
    }
    for (uint32_t vph = 0; vph <= 432000u; vph += 3600u) {
      ui_format_thousands(vph, buf, sizeof(buf));
      assert((static_cast<int>(std::strlen(buf)) + 4) * kUiCharW <= worst);
    }
    ui_format_thousands(432000u, buf, sizeof(buf));
    eq(buf, "432,000");

    /* UIH-06 */
    {
      const auto M = DSP_QUARTZ_MODE_MOTOR;
      eq(ui_quartz_status_text(false, false, M, DSP_QUARTZ_STATE_MEASURING,
                               DSP_QUARTZ_INHIB_NONE),
         "no stream");
      eq(ui_quartz_status_text(true, false, M, DSP_QUARTZ_STATE_WAIT_TIMEBASE,
                               DSP_QUARTZ_INHIB_NONE),
         "clock check");
    }

    /* UIH-07 */
    for (int m = 0; m < 4; m++) {
      const char *tag = m == 0   ? "MOTOR"
                        : m == 1 ? "LCD"
                        : m == 2 ? "FORK"
                                 : "QUARTZ";
      if (m < 3) {
        eq(ui_quartz_mode_tag(static_cast<dsp_quartz_mode_t>(
               m == 0 ? DSP_QUARTZ_MODE_MOTOR
                      : (m == 1 ? DSP_QUARTZ_MODE_LCD : DSP_QUARTZ_MODE_FORK))),
           tag);
      }
      const int tag_w = static_cast<int>(std::strlen(tag)) * kUiCharW;
      for (int c = 0; c < 8; c++) {
        const UiHeaderLayout l =
            ui_header_layout((c & 1) != 0, (c & 2) != 0, (c & 4) != 0);
        const UiUncalBadge b = ui_quartz_uncal_badge(tag, l.max_left_px);
        const int w = static_cast<int>(std::strlen(b.text)) * kUiCharW + 6;
        assert(b.x + w <= l.max_left_px);
        assert(b.x >= tag_w + 3);
        const bool full_fits = tag_w + 6 + 5 * kUiCharW + 6 <= l.max_left_px;
        assert((std::strcmp(b.text, "UNC") == 0) == !full_fits);
        if (l.ble_x < 0) {
          assert(full_fits); /* non-BLE layouts keep UNCAL */
        }
      }
    }

    /* UIH-08 */
    const char *rows[9] = {"..X..", "..XX.", "X.X.X", ".XXX.", "..X..",
                           ".XXX.", "X.X.X", "..XX.", "..X.."};
    static_assert(sizeof(kUiBleRune) == 9, "rune has 9 rows");
    for (int r = 0; r < 9; r++) {
      assert((kUiBleRune[r] & 0x07) == 0); /* only the top 5 bits */
      for (int col = 0; col < 5; col++) {
        const bool on = (kUiBleRune[r] & (0x80 >> col)) != 0;
        assert(on == (rows[r][col] == 'X'));
      }
    }
  }

  std::printf("test_ui_format: OK\n");
  return 0;
}
