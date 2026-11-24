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

#include "app_config.h"
#include "device_mode.h"
#include "dsp_blocks.h"
#include "dsp_graph.h"
#include "perf_timing.h"
#include "quartz/quartz_engine.h"
#include "timebase.h"

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

/* Device mode / run control. */

void control_api_get_device(device_status_t *out) { device_status_get(out); }

device_mode_set_result_t control_api_set_device_mode(device_mode_t mode) {
  return device_mode_set(mode);
}

device_run_result_t control_api_run_start(void) { return device_run_start(); }

void control_api_run_stop(void) { device_run_stop(); }

const char *control_api_device_mode_name(device_mode_t mode) {
  return device_mode_name(mode);
}

bool control_api_device_mode_from_name(const char *name, device_mode_t *out) {
  return device_mode_from_name(name, out);
}

/* Per-beat records. */

uint32_t control_api_copy_beats(dsp_beat_t *out, uint32_t max_n) {
  return dsp_graph_copy_beats(out, max_n);
}

uint32_t control_api_beat_seq(void) { return dsp_graph_beat_seq(); }

uint32_t control_api_copy_beats_since(uint32_t seq, dsp_beat_t *out,
                                      uint32_t max_n) {
  return dsp_graph_copy_beats_since(seq, out, max_n);
}

/* Pipeline graph. */

const dsp_pipeline_node_t *control_api_pipeline(uint32_t *count) {
  return dsp_pipeline_nodes(count);
}

/* GPS and timebase log. */

bool control_api_gps_built(void) { return timebase_gps_built(); }

bool control_api_pps_built(void) { return timebase_pps_built(); }

bool control_api_get_gps(timebase_gps_t *out) { return timebase_gps_get(out); }

void control_api_timebase_log(timebase_log_t *out) {
  timebase_log_snapshot(out);
}

bool control_api_timebase_log_pending(void) { return timebase_log_pending(); }

uint32_t control_api_timebase_log_last_write_unix(void) {
  return timebase_log_last_write_unix();
}

timebase_save_result_t control_api_timebase_log_clear(void) {
  return timebase_log_clear_saved();
}

float control_api_timebase_log_delta_ppm(void) {
  return timebase_log_delta_ppm();
}

timebase_log_delta_result_t control_api_set_timebase_log_delta_ppm(float ppm) {
  return timebase_set_log_delta_ppm(ppm);
}

/*
 * Quartz timing. The name helpers are plain tables and work everywhere;
 * the rest exists only where the capture feeds the engine (PCM1808), so
 * other builds never reference the engine's ~41 KB pipeline store.
 */

const char *control_api_quartz_mode_name(dsp_quartz_mode_t mode) {
  return quartz::engine::mode_name(mode);
}

dsp_quartz_mode_t control_api_quartz_mode_from_name(const char *name) {
  return quartz::engine::mode_from_name(name);
}

const char *control_api_quartz_state_name(dsp_quartz_state_t state) {
  return quartz::engine::state_name(state);
}

const char *control_api_quartz_inhibition_name(dsp_quartz_inhibition_t status) {
  return quartz::engine::inhibition_name(status);
}

const char *control_api_quartz_ts_method_name(dsp_quartz_ts_method_t method) {
  return quartz::engine::ts_method_name(method);
}

dsp_quartz_ts_method_t
control_api_quartz_ts_method_from_name(const char *name) {
  return quartz::engine::ts_method_from_name(name);
}

const char *
control_api_quartz_param_result_name(dsp_quartz_param_result_t result) {
  return quartz::engine::param_result_name(result);
}

const char *control_api_quartz_lcd_status_name(dsp_quartz_lcd_status_t status) {
  return quartz::engine::lcd_status_name(status);
}

#if AUDIO_SOURCE == PCM1808

bool control_api_quartz_supported(void) { return true; }

void control_api_get_quartz(dsp_quartz_t *out) { quartz::engine::get(out); }

uint32_t control_api_copy_quartz_pulses(dsp_quartz_pulse_t *out,
                                        uint32_t max_n) {
  return quartz::engine::copy_pulses(out, max_n);
}

bool control_api_set_quartz_mode(dsp_quartz_mode_t mode) {
  return quartz::engine::set_mode(mode);
}

dsp_quartz_mode_t control_api_get_quartz_mode(void) {
  return quartz::engine::mode();
}

bool control_api_set_quartz_param(const char *param, float value) {
  return quartz::engine::set_param(param, value);
}

dsp_quartz_param_result_t control_api_set_quartz_param_ex(const char *param,
                                                          float value) {
  return quartz::engine::set_param_ex(param, value);
}

bool control_api_get_quartz_param(const char *param, float *out) {
  return quartz::engine::get_param(param, out);
}

bool control_api_quartz_restart(void) { return quartz::engine::restart(); }

#else /* AUDIO_SOURCE != PCM1808: no quartz engine */

bool control_api_quartz_supported(void) { return false; }

void control_api_get_quartz(dsp_quartz_t *out) {
  if (out == nullptr) {
    return;
  }
  *out = dsp_quartz_t{};
  out->mode = DSP_QUARTZ_MODE_MECHANICAL;
  out->state = DSP_QUARTZ_STATE_IDLE;
  out->reason = "quartz timing needs the PCM1808 source";
  out->cal_source = "none";
}

uint32_t control_api_copy_quartz_pulses(dsp_quartz_pulse_t *, uint32_t) {
  return 0u;
}

bool control_api_set_quartz_mode(dsp_quartz_mode_t mode) {
  return mode == DSP_QUARTZ_MODE_MECHANICAL;
}

dsp_quartz_mode_t control_api_get_quartz_mode(void) {
  return DSP_QUARTZ_MODE_MECHANICAL;
}

bool control_api_set_quartz_param(const char *, float) { return false; }

dsp_quartz_param_result_t control_api_set_quartz_param_ex(const char *, float) {
  return DSP_QUARTZ_PARAM_NO_ENGINE;
}

bool control_api_get_quartz_param(const char *, float *) { return false; }

bool control_api_quartz_restart(void) { return false; }

#endif
