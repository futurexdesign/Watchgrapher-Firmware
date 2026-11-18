/**
 * @file ui_format.h
 * @brief Text formatting for the OLED readout (fixed widths, placeholders).
 *
 * Header-only and SDK-free so it is host-testable.
 *
 * @see display_ui.cpp, tests/test_ui_format.cpp
 */

#pragma once

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
