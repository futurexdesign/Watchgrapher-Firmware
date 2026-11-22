/**
 * @file ui_format.h
 * @brief Text formatting for the OLED readout (fixed widths, placeholders).
 *
 * Header-only and SDK-free so it is host-testable. The quartz helpers
 * depend only on the plain-C enums in dsp_quartz_types.h.
 *
 * @see display_ui.cpp, tests/test_ui_format.cpp
 */

#pragma once

#include "dsp_quartz_types.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/** "28,800" style thousands grouping. */
static inline void ui_format_thousands(uint32_t v, char *out, size_t n) {
  char digits[12];
  const int len = snprintf(digits, sizeof(digits), "%lu", (unsigned long)v);
  size_t o = 0;
  for (int i = 0; i < len && o + 1 < n; i++) {
    if (i > 0 && (len - i) % 3 == 0 && o + 2 < n) {
      out[o++] = ',';
    }
    out[o++] = digits[i];
  }
  if (n > 0) {
    out[o < n ? o : n - 1] = '\0';
  }
}

/**
 * Daily rate, at most 5 characters: "+12.3" below 100 s/d, "+123" below
 * 1000, then ">+999" / "<-999". "---" when invalid.
 */
static inline void ui_format_rate(bool valid, float s_day, char *out, size_t n) {
  if (!valid || !isfinite(s_day)) {
    snprintf(out, n, "---");
  } else if (fabsf(s_day) < 99.95f) {
    /* Avoid "-0.0": a value that rounds to zero shows as "+0.0". */
    snprintf(out, n, "%+.1f", fabsf(s_day) < 0.05f ? 0.0 : (double)s_day);
  } else if (fabsf(s_day) < 999.5f) {
    snprintf(out, n, "%+.0f", (double)s_day);
  } else {
    snprintf(out, n, s_day > 0.f ? ">+999" : "<-999");
  }
}

/** Amplitude in whole degrees, "---" when invalid (degree sign drawn separately). */
static inline void ui_format_amp(bool valid, float deg, char *out, size_t n) {
  if (!valid || !isfinite(deg)) {
    snprintf(out, n, "---");
  } else {
    snprintf(out, n, "%.0f", (double)deg);
  }
}

/** Beat error in ms: "0.4" below 10 ms, "12" above; "-.-" when invalid. */
static inline void ui_format_be(bool valid, float ms, char *out, size_t n) {
  if (!valid || !isfinite(ms)) {
    snprintf(out, n, "-.-");
  } else if (ms < 9.95f) {
    snprintf(out, n, "%.1f", (double)ms);
  } else {
    snprintf(out, n, "%.0f", (double)ms);
  }
}

/** Lift angle: "52", or "52.5" when not a whole degree; "--" when invalid. */
static inline void ui_format_lift(float deg, char *out, size_t n) {
  if (!isfinite(deg) || deg < 1.f || deg >= 99.95f) {
    snprintf(out, n, "--");
  } else if (fabsf(deg - roundf(deg)) < 0.05f) {
    snprintf(out, n, "%.0f", (double)deg);
  } else {
    snprintf(out, n, "%.1f", (double)deg);
  }
}

/**
 * Symmetric trend-graph scale (s/d): the smallest step of 2, 5, 10, 20 … 1000
 * that holds every finite value in {@code v}. Non-finite entries (gaps) are
 * ignored; an empty or all-gap history gives the minimum, 2.
 */
static inline float ui_trend_scale(const float *v, size_t n) {
  static const float kSteps[] = {2.f, 5.f, 10.f, 20.f, 50.f, 100.f, 200.f, 500.f, 1000.f};
  float peak = 0.f;
  for (size_t i = 0; i < n; i++) {
    if (isfinite(v[i]) && fabsf(v[i]) > peak) {
      peak = fabsf(v[i]);
    }
  }
  for (float s : kSteps) {
    if (peak <= s) {
      return s;
    }
  }
  return kSteps[sizeof(kSteps) / sizeof(kSteps[0]) - 1];
}

/* ---- Quartz readout ----------------------------------------------------- */

/**
 * Signed fixed-point value, e.g. "+0.273" for {@code decimals} 3. A value
 * that rounds to zero shows as "+0.000", never "-0.000". "---" when invalid.
 *
 * @param valid    {@code false} prints the placeholder.
 * @param v        Value to print.
 * @param decimals Digits after the point, clamped to 0..6.
 * @param out      Destination buffer.
 * @param n        Size of {@code out} in bytes (output is truncated to fit).
 */
static inline void ui_format_signed_fixed(bool valid, float v, int decimals, char *out, size_t n) {
  if (!valid || !isfinite(v)) {
    snprintf(out, n, "---");
    return;
  }
  const int d = decimals < 0 ? 0 : (decimals > 6 ? 6 : decimals);
  /* Half of the last printed digit: anything smaller would print as -0. */
  const double half = 0.5 * pow(10.0, -d);
  const double x = fabs((double)v) < half ? 0.0 : (double)v;
  snprintf(out, n, "%+.*f", d, x);
}

/**
 * Most characters {@link ui_format_quartz_rate} produces. The quartz headline
 * is drawn at text size 3 (18 px per character) with "s/d" beside it:
 * 6 × 18 + 2 + 3 × 6 = 128 px, the full panel width.
 */
static constexpr size_t kUiQuartzRateMaxChars = 6;

/**
 * Quartz daily rate with as many decimals as fit in
 * {@link kUiQuartzRateMaxChars} characters: "+0.273" (3 dp) below 10 s/d,
 * "+12.35" below 100, "+123.4" below 1000, then ">+999" / "<-999". "---"
 * when invalid.
 *
 * Quartz rates are small (a good watch is within ±0.5 s/d, about ±6 ppm),
 * so the mechanical one-decimal format would hide everything interesting;
 * 0.001 s/d (12 ppb) is about the resolution of a 60 s motor gate.
 */
static inline void ui_format_quartz_rate(bool valid, float s_day, char *out, size_t n) {
  if (!valid || !isfinite(s_day)) {
    snprintf(out, n, "---");
    return;
  }
  /* Thresholds sit half a last digit below the next power of ten, so a
   * value that rounds up (9.9996 -> "10.000") moves to the shorter format. */
  const float a = fabsf(s_day);
  if (a < 9.9995f) {
    ui_format_signed_fixed(true, s_day, 3, out, n);
  } else if (a < 99.995f) {
    ui_format_signed_fixed(true, s_day, 2, out, n);
  } else if (a < 999.95f) {
    ui_format_signed_fixed(true, s_day, 1, out, n);
  } else {
    snprintf(out, n, s_day > 0.f ? ">+999" : "<-999");
  }
}

/**
 * A duration in seconds: "60.0" below 100 s, "960" above; "--" when invalid
 * or not positive. Used for the inhibition period (10 / 60 / up to 960 s).
 */
static inline void ui_format_seconds(bool valid, float s, char *out, size_t n) {
  if (!valid || !isfinite(s) || s <= 0.f) {
    snprintf(out, n, "--");
  } else if (s < 99.95f) {
    snprintf(out, n, "%.1f", (double)s);
  } else {
    snprintf(out, n, "%.0f", (double)s);
  }
}

/**
 * A frequency in Hz: "64", "32768", or "440.5" when not a whole number;
 * "--" when unknown (not finite or not positive).
 */
static inline void ui_format_hz(float hz, char *out, size_t n) {
  if (!isfinite(hz) || hz <= 0.f) {
    snprintf(out, n, "--");
  } else if (fabsf(hz - roundf(hz)) < 0.05f) {
    snprintf(out, n, "%.0f", (double)hz);
  } else {
    snprintf(out, n, "%.1f", (double)hz);
  }
}

/**
 * Inhibition jump with its pulse count: "244.1µs n=2" (one decimal below
 * 1000 µs, whole µs above). The count is the number of 8192 Hz pulses
 * deleted (jump / 122.07 µs); the caller adds the "@8192" reference when it
 * has room. "--" when invalid or no jump has been seen ({@code us} ≤ 0).
 *
 * @param valid  {@code false} prints the placeholder.
 * @param us     Jump size in µs.
 * @param count  Pulses deleted at 8192 Hz ("n").
 * @param micro  Character for 'µ' in the target font (CP437: 230; 'u' for
 *               plain ASCII).
 * @param out    Destination buffer.
 * @param n      Size of {@code out} in bytes.
 */
static inline void ui_format_jump(bool valid, float us, uint32_t count, char micro, char *out,
                                  size_t n) {
  if (!valid || !isfinite(us) || us <= 0.f) {
    snprintf(out, n, "--");
  } else if (us < 999.95f) {
    snprintf(out, n, "%.1f%cs n=%lu", (double)us, micro, (unsigned long)count);
  } else {
    snprintf(out, n, "%.0f%cs n=%lu", (double)us, micro, (unsigned long)count);
  }
}

/**
 * Measurement progress against a gate: "23/60s" while running, "23s" when
 * there is no fixed gate ({@code gate_s} ≤ 0 or not finite, e.g. the motor
 * "until two inhibition events" gate), "done" once the result is frozen,
 * "--" when the elapsed time is unknown. Whole seconds, rounded down, and
 * capped at the gate so the last frame before "done" never reads "61/60s".
 *
 * @param elapsed_s Signal time since the measurement start (s).
 * @param gate_s    Gate length (s); ≤ 0 means open-ended.
 * @param done      The gate was reached and the result is frozen.
 * @param out       Destination buffer.
 * @param n         Size of {@code out} in bytes.
 */
static inline void ui_format_gate(float elapsed_s, float gate_s, bool done, char *out, size_t n) {
  if (done) {
    snprintf(out, n, "done");
    return;
  }
  if (!isfinite(elapsed_s) || elapsed_s < 0.f) {
    snprintf(out, n, "--");
    return;
  }
  const double e = floor((double)elapsed_s);
  if (isfinite(gate_s) && gate_s > 0.f) {
    const double g = floor((double)gate_s + 0.5);
    snprintf(out, n, "%.0f/%.0fs", e < g ? e : g, g);
  } else {
    snprintf(out, n, "%.0fs", e);
  }
}

/**
 * Header tag for a quartz mode: "MOTOR", "LCD", "FORK"; "QUARTZ" for any
 * other value (the display only calls it for non-mechanical modes).
 */
[[nodiscard]] static inline const char *ui_quartz_mode_tag(dsp_quartz_mode_t mode) {
  switch (mode) {
  case DSP_QUARTZ_MODE_MOTOR:
    return "MOTOR";
  case DSP_QUARTZ_MODE_LCD:
    return "LCD";
  case DSP_QUARTZ_MODE_FORK:
    return "FORK";
  default:
    return "QUARTZ";
  }
}

/**
 * Clip hint for the channel the mode listens on (handoff §8): the pickup
 * hit the ADC rail, so open the attenuator jumper of that channel. LCD uses
 * the capacitive pickup on RIGHT ("CLIP: open JP2"); motor and fork use the
 * inductive pickup on LEFT ("CLIP: open JP1"), which is also the default.
 */
[[nodiscard]] static inline const char *ui_quartz_clip_hint(dsp_quartz_mode_t mode) {
  return mode == DSP_QUARTZ_MODE_LCD ? "CLIP: open JP2" : "CLIP: open JP1";
}

/**
 * Most characters any {@link ui_quartz_status_text} string has: one full
 * text-size-1 row of the 128 px panel (21 × 6 px = 126 px).
 */
static constexpr size_t kUiStatusMaxChars = 21;

/**
 * The quartz status line under the header: what the device is waiting for,
 * or, once a rate exists, what the motor rate can claim about inhibition
 * (handoff §5). In order of precedence:
 *
 * | Condition                               | Text                |
 * |-----------------------------------------|---------------------|
 * | not capturing, USB mic mode (no frozen) | "no host stream"    |
 * | not capturing, standalone (no frozen)   | "stopped"           |
 * | capture rate not verified               | "clock check..."    |
 * | verified rate is not 48 kHz             | "unsupported rate"  |
 * | running, no rate yet                    | "acquiring..."      |
 * | motor: no jump, long gate               | "no inhibition"     |
 * | motor: no jump, short gate              | "may incl. inhib."  |
 * | motor: one jump                         | "need 1 more cycle" |
 * | motor: two jumps, next pulse pending    | "correcting..."     |
 * | motor: two or more jumps                | "corrected"         |
 * | LCD / fork with a rate                  | "measuring"         |
 *
 * A {@code DONE} result is frozen, so it keeps its §5 text after the host
 * stops recording. Every string fits {@link kUiStatusMaxChars}. The idle and
 * clock-check texts match the mechanical header so both layouts read alike.
 *
 * @param streaming  Capture and DSP are running (host stream or run latch).
 * @param standalone The device is in standalone mode (no host stream needed).
 * @param mode      Active quartz mode.
 * @param state     Measurement state.
 * @param inhib     Motor inhibition status (ignored for LCD / fork).
 * @return A static string, never null.
 */
[[nodiscard]] static inline const char *ui_quartz_status_text(bool streaming,
                                                              bool standalone,
                                                              dsp_quartz_mode_t mode,
                                                              dsp_quartz_state_t state,
                                                              dsp_quartz_inhibition_t inhib) {
  if (!streaming && state != DSP_QUARTZ_STATE_DONE) {
    return standalone ? "stopped" : "no host stream";
  }
  switch (state) {
  case DSP_QUARTZ_STATE_WAIT_TIMEBASE:
    return "clock check...";
  case DSP_QUARTZ_STATE_UNSUPPORTED_RATE:
    return "unsupported rate";
  case DSP_QUARTZ_STATE_MEASURING:
  case DSP_QUARTZ_STATE_DONE:
    break;
  default: /* IDLE (transient on a mode switch), ACQUIRING */
    return "acquiring...";
  }
  if (mode != DSP_QUARTZ_MODE_MOTOR) {
    return "measuring";
  }
  switch (inhib) {
  case DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE:
    return "no inhibition";
  case DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION:
    return "may incl. inhib.";
  case DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE:
    return "need 1 more cycle";
  case DSP_QUARTZ_INHIB_CORRECTION_PENDING:
    return "correcting...";
  case DSP_QUARTZ_INHIB_CORRECTED:
    return "corrected";
  default: /* NONE: fewer than three pulses */
    return "acquiring...";
  }
}
