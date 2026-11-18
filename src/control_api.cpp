/**
 * @file control_api.cpp
 * @brief Thin transport-agnostic façade over {@link dsp_graph}.
 *
 * Keeps CDC / HID formatting out of the graph. Every call here is a straight
 * pass-through except name ↔ id helpers (backed by {@link dsp_block_info})
 * used by the CLI parser.
 *
 * @see control_api.h, dsp_graph.cpp
 */

#include "control_api.h"

#include "dsp_blocks.h"
#include "dsp_graph.h"
#include "perf_timing.h"

#include <cctype>
#include <cstring>

/**
 * Case-insensitive C-string equality (ASCII).
 * @return {@code true} when both strings match ignoring case.
 */
static bool name_eq(const char *a, const char *b) {
  if (!a || !b) {
    return false;
  }
  while (*a && *b) {
    if (tolower(static_cast<unsigned char>(*a)) !=
        tolower(static_cast<unsigned char>(*b))) {
      return false;
    }
    a++;
    b++;
  }
  return *a == *b;
}

void control_api_init(void) {
  dsp_graph_init();
  perf_timing_init();
}

void control_api_watchdog_poll(void) { dsp_graph_watchdog_poll(); }

bool control_api_get_bypass(void) { return dsp_graph_get_bypass(); }

void control_api_set_bypass(bool on) { dsp_graph_set_bypass(on); }

bool control_api_block_enabled(dsp_block_id_t id) {
  return dsp_graph_block_enabled(id);
}

bool control_api_set_block_enabled(dsp_block_id_t id, bool on) {
  return dsp_graph_set_block_enabled(id, on);
}

const char *control_api_block_name(dsp_block_id_t id) {
  return dsp_block_name(id);
}

dsp_block_id_t control_api_block_from_name(const char *name) {
  if (!name) {
    return DSP_BLOCK_COUNT;
  }
  for (int i = 0; i < static_cast<int>(DSP_BLOCK_COUNT); i++) {
    if (name_eq(name, control_api_block_name(static_cast<dsp_block_id_t>(i)))) {
      return static_cast<dsp_block_id_t>(i);
    }
  }
  /* Short aliases for CLI convenience. */
  if (name_eq(name, "meter") || name_eq(name, "level")) {
    return DSP_BLOCK_LEVEL_METER;
  }
  if (name_eq(name, "filter") || name_eq(name, "band")) {
    return DSP_BLOCK_FILTER_BANK;
  }
  if (name_eq(name, "event") || name_eq(name, "detect")) {
    return DSP_BLOCK_EVENT_DETECT;
  }
  if (name_eq(name, "beat") || name_eq(name, "rate") || name_eq(name, "vph")) {
    return DSP_BLOCK_BEAT_RATE;
  }
  if (name_eq(name, "noise") || name_eq(name, "floor")) {
    return DSP_BLOCK_NOISE_FLOOR;
  }
  if (name_eq(name, "gate")) {
    return DSP_BLOCK_SOFT_GATE;
  }
  if (name_eq(name, "limit")) {
    return DSP_BLOCK_LIMITER;
  }
  if (name_eq(name, "match") || name_eq(name, "matched")) {
    return DSP_BLOCK_MATCHED_FILTER;
  }
  if (name_eq(name, "fir")) {
    return DSP_BLOCK_FIR_BANK;
  }
  if (name_eq(name, "fft") || name_eq(name, "spec")) {
    return DSP_BLOCK_SPECTRUM;
  }
  return DSP_BLOCK_COUNT;
}

bool control_api_set_param(dsp_block_id_t id, const char *param, float value) {
  return dsp_graph_set_param(id, param, value);
}

bool control_api_get_param(dsp_block_id_t id, const char *param, float *out) {
  return dsp_graph_get_param(id, param, out);
}

void control_api_get_metrics(dsp_metrics_t *out) { dsp_graph_get_metrics(out); }

void control_api_get_timing(dsp_timing_t *out) { perf_timing_get(out); }

void control_api_reset_timing(void) { perf_timing_reset(); }

void control_api_get_beat_rate(dsp_beat_rate_t *out) {
  dsp_graph_get_beat(out);
}

void control_api_get_spectrum(dsp_spectrum_t *out) {
  dsp_graph_get_spectrum(out);
}

void control_api_get_matched(dsp_matched_t *out) {
  dsp_graph_get_matched(out);
}

uint32_t control_api_copy_events(dsp_event_t *out, uint32_t max_n) {
  return dsp_graph_copy_events(out, max_n);
}

uint32_t control_api_copy_errors(dsp_error_t *out, uint32_t max_n) {
  return dsp_graph_copy_errors(out, max_n);
}

void control_api_clear_errors(void) { dsp_graph_clear_errors(); }

bool control_api_pop_async_message(char *buf, size_t buflen) {
  return dsp_graph_pop_async_message(buf, buflen);
}
