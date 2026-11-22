/**
 * @file hid_control.cpp
 * @brief Vendor HID framing + JSON RPC → control_api (WebHID).
 *
 * @see docs/HID_CONTROL_PLANE.md, hid_control.h, control_api.h
 */

#include "hid_control.h"

#include "board_config.h"
#include "control_api.h"
#include "dsp_blocks.h"
#include "dsp_types.h"
#include "fw_identity.h"
#include "hid_json.h"
#include "timebase.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <hardware/timer.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

constexpr uint8_t kReportId = 0x01;
constexpr uint8_t kFlagMore = 0x01;
constexpr uint8_t kFlagNotifyHint = 0x02;
constexpr uint16_t kMaxJson = 2048;
constexpr uint8_t kHeaderLen = 5; /* flags..payloadLen (no report ID in buffer) */
constexpr uint8_t kMaxChunk = 58;
constexpr uint8_t kPayloadBytes = 63; /* HID report payload after report ID */
constexpr uint32_t kReasmTimeoutMs = 500;

constexpr uint8_t kMetricsHzMax = 20;
constexpr uint8_t kBeatHzMax = 5;
constexpr uint8_t kTimingHzMax = 5;
/** Same cap as {@code beatHz}: a snapshot is 0.8-1.3 KB (14-22 reports). */
constexpr uint8_t kQuartzHzMax = 5;

/* Report descriptor: Usage Page 0xFFA0, Usage 0x0001, Report ID 1, 63-byte IO. */
uint8_t const kHidReportDesc[] = {
    HID_USAGE_PAGE_N(0xFFA0, 2),
    HID_USAGE(0x01),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    HID_REPORT_ID(1) HID_USAGE(0x02),
    HID_LOGICAL_MIN(0x00),
    HID_LOGICAL_MAX_N(0xff, 2),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(63),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_USAGE(0x03),
    HID_LOGICAL_MIN(0x00),
    HID_LOGICAL_MAX_N(0xff, 2),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(63),
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END};

Adafruit_USBD_HID g_hid;

char g_rx_json[kMaxJson + 1];
uint16_t g_rx_len = 0;
uint8_t g_rx_seq = 0;
uint8_t g_rx_frag_count = 0;
uint8_t g_rx_got = 0;
uint8_t g_rx_seen[64]; /* bitmask for frag indices; fragCount <= 36 for 2048 */
uint32_t g_rx_deadline_ms = 0;
volatile bool g_rx_ready = false;

char g_tx_json[kMaxJson + 1];
uint16_t g_tx_len = 0;
uint16_t g_tx_off = 0;
uint8_t g_tx_seq = 0;
uint8_t g_tx_frag_index = 0;
uint8_t g_tx_frag_count = 0;
bool g_tx_active = false;
bool g_tx_notify = false;

/*
 * Response / notify assembly buffer and request scratch. File-scope (not
 * stack): core 0 has only a 4 KB stack. Main-loop only; a response or notify
 * is assembled here only while no TX is in flight, then copied by tx_begin.
 */
char g_out[kMaxJson];
char g_patch_obj[1200];
char g_blocks_obj[1024];
dsp_event_t g_ev_scratch[DSP_EVENT_LOG_CAP];
dsp_error_t g_err_scratch[DSP_ERROR_LOG_CAP];
/** Quartz snapshot / pulse scratch (dsp_quartz_t is ~200 B; keep it off
 * the 4 KB core-0 stack like the other scratch buffers). */
dsp_quartz_t g_quartz;
dsp_quartz_pulse_t g_quartz_pulses[DSP_QUARTZ_PULSE_LOG_CAP];
/** The {@code quartz} member of a patchState patch. */
char g_quartz_obj[256];
/** Beats scratch ({@code getBeats} up to hid_json::kBeatsMax, notify 8). */
dsp_beat_t g_beats[hid_json::kBeatsMax];
/** GPS snapshot (~330 B) and timebase-log snapshot (~400 B). */
timebase_gps_t g_gps;
timebase_log_t g_tblog;

uint8_t g_sub_metrics_hz = 0;
uint8_t g_sub_beat_hz = 0;
uint8_t g_sub_timing_hz = 0;
uint8_t g_sub_quartz_hz = 0;
bool g_sub_quartz_pulses = false;
bool g_sub_events = false;
bool g_sub_errors = false;
bool g_sub_beats = false;
uint32_t g_last_metrics_ms = 0;
uint32_t g_last_beat_ms = 0;
uint32_t g_last_timing_ms = 0;
uint32_t g_last_quartz_ms = 0;
/** Measurement the {@code quartzPulse} cursor belongs to. */
uint32_t g_pulse_measurement = 0;
/** First pulse sequence of {@link g_pulse_measurement} not yet pushed (or
 * already present when the subscription was seeded). */
uint32_t g_pulse_next_seq = 0;
/** First beat seq not yet pushed (the {@code beats} baseline). */
uint32_t g_beat_next_seq = 0;
/**
 * {@code event} notify cursor: {@code peak_sample} of the newest event
 * already sent (or already present when the subscription was seeded, 0 when
 * the log was empty). Events are sent oldest first and only those with a
 * larger {@code peak_sample} are new; event windows close in order, so the
 * peak sample is strictly increasing along the log. dsp_event_t has no
 * sequence number (it lives in dsp_types.h / dsp_graph.cpp), so the peak
 * sample is the key.
 */
uint64_t g_last_event_peak = 0;
uint32_t g_last_error_uptime = 0;
bool g_have_last_error = false;

const char *skip_ws(const char *p) {
  while (p && *p && isspace(static_cast<unsigned char>(*p))) {
    p++;
  }
  return p;
}

/** Pointer to value after `"key":` or nullptr. */
const char *json_find_key(const char *json, const char *key) {
  if (!json || !key) {
    return nullptr;
  }
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = json;
  while ((p = strstr(p, pat)) != nullptr) {
    const char *after = p + strlen(pat);
    after = skip_ws(after);
    if (*after == ':') {
      return skip_ws(after + 1);
    }
    p++;
  }
  return nullptr;
}

bool json_get_string(const char *json, const char *key, char *out, size_t outlen) {
  const auto *v = json_find_key(json, key);
  if (!v || *v != '"') {
    return false;
  }
  v++;
  size_t i = 0;
  while (*v && *v != '"' && i + 1 < outlen) {
    if (*v == '\\' && v[1]) {
      v++;
    }
    out[i++] = *v++;
  }
  out[i] = '\0';
  return true;
}

/**
 * Reads an integer member as a full 64-bit value ({@code long} is 32-bit on
 * the RP2350). Used for {@code id} and {@code ping.t}, which carry a client
 * timestamp / counter that exceeds 2^31.
 *
 * @param json Object text.
 * @param key Member name.
 * @param out Receives the value.
 * @return true if present and numeric (and within int64).
 */
bool json_get_i64(const char *json, const char *key, int64_t *out) {
  const auto *v = json_find_key(json, key);
  return v && hid_json::parse_i64(v, out);
}

/**
 * Reads an integer member that is really a 32-bit quantity (limits, rates,
 * offsets): the 64-bit value saturates to the range of {@code long}, and
 * callers range-check the result.
 */
bool json_get_int(const char *json, const char *key, long *out) {
  int64_t n = 0;
  if (!json_get_i64(json, key, &n)) {
    return false;
  }
  if (n > static_cast<int64_t>(LONG_MAX)) {
    n = LONG_MAX;
  } else if (n < static_cast<int64_t>(LONG_MIN)) {
    n = LONG_MIN;
  }
  *out = static_cast<long>(n);
  return true;
}

bool json_get_double(const char *json, const char *key, double *out) {
  const auto *v = json_find_key(json, key);
  if (!v) {
    return false;
  }
  char *end = nullptr;
  auto n = strtod(v, &end);
  if (end == v) {
    return false;
  }
  *out = n;
  return true;
}

bool json_get_bool(const char *json, const char *key, bool *out) {
  const auto *v = json_find_key(json, key);
  if (!v) {
    return false;
  }
  if (strncmp(v, "true", 4) == 0) {
    *out = true;
    return true;
  }
  if (strncmp(v, "false", 5) == 0) {
    *out = false;
    return true;
  }
  return false;
}

/** Extract object spanning from `{` at start into dst (NUL-terminated). */
bool json_extract_object(const char *start, char *dst, size_t dstlen) {
  start = skip_ws(start);
  if (!start || *start != '{') {
    return false;
  }
  int depth = 0;
  size_t i = 0;
  bool in_str = false;
  for (const char *p = start; *p; p++) {
    char c = *p;
    if (in_str) {
      if (c == '\\' && p[1]) {
        if (i + 2 >= dstlen) {
          return false;
        }
        dst[i++] = *p++;
        dst[i++] = *p;
        continue;
      }
      if (c == '"') {
        in_str = false;
      }
    } else {
      if (c == '"') {
        in_str = true;
      } else if (c == '{') {
        depth++;
      } else if (c == '}') {
        depth--;
        if (depth == 0) {
          if (i + 2 >= dstlen) {
            return false;
          }
          dst[i++] = c;
          dst[i] = '\0';
          return true;
        }
      }
    }
    if (i + 1 >= dstlen) {
      return false;
    }
    dst[i++] = c;
  }
  return false;
}

void u64_to_dec(uint64_t v, char *out, size_t outlen) {
  if (outlen == 0) {
    return;
  }
  char tmp[24];
  int n = 0;
  if (v == 0) {
    tmp[n++] = '0';
  } else {
    while (v > 0 && n < static_cast<int>(sizeof(tmp))) {
      tmp[n++] = static_cast<char>('0' + (v % 10ull));
      v /= 10ull;
    }
  }
  if (static_cast<size_t>(n) + 1 > outlen) {
    out[0] = '\0';
    return;
  }
  for (int i = 0; i < n; i++) {
    out[i] = tmp[n - 1 - i];
  }
  out[n] = '\0';
}

void rx_reset(void) {
  g_rx_len = 0;
  g_rx_frag_count = 0;
  g_rx_got = 0;
  memset(g_rx_seen, 0, sizeof(g_rx_seen));
  g_rx_deadline_ms = 0;
}

void tx_begin(const char *json, bool notify) {
  auto n = strlen(json);
  if (n > kMaxJson) {
    n = kMaxJson;
  }
  memcpy(g_tx_json, json, n);
  g_tx_json[n] = '\0';
  g_tx_len = static_cast<uint16_t>(n);
  g_tx_off = 0;
  g_tx_frag_index = 0;
  g_tx_frag_count = static_cast<uint8_t>((n + kMaxChunk - 1) / kMaxChunk);
  if (g_tx_frag_count == 0) {
    g_tx_frag_count = 1;
  }
  g_tx_seq = static_cast<uint8_t>(g_tx_seq + 1u);
  g_tx_notify = notify;
  g_tx_active = true;
}

void tx_pump(void) {
  if (!g_tx_active || !g_hid.ready()) {
    return;
  }
  uint8_t report[kPayloadBytes];
  memset(report, 0, sizeof(report));
  auto remain = (g_tx_off < g_tx_len) ? static_cast<uint16_t>(g_tx_len - g_tx_off) : static_cast<uint16_t>(0);
  auto chunk = (remain > kMaxChunk) ? kMaxChunk : static_cast<uint8_t>(remain);
  bool more = (g_tx_frag_index + 1u) < g_tx_frag_count;
  report[0] = (more ? kFlagMore : 0) | (g_tx_notify ? kFlagNotifyHint : 0);
  report[1] = g_tx_seq;
  report[2] = g_tx_frag_index;
  report[3] = g_tx_frag_count;
  report[4] = chunk;
  if (chunk) {
    memcpy(&report[5], &g_tx_json[g_tx_off], chunk);
    g_tx_off = static_cast<uint16_t>(g_tx_off + chunk);
  }
  if (!g_hid.sendReport(kReportId, report, kPayloadBytes)) {
    return;
  }
  g_tx_frag_index++;
  if (g_tx_frag_index >= g_tx_frag_count) {
    g_tx_active = false;
  }
}

using hid_json::appendf;
using hid_json::json_sanitize;

void send_error(int64_t id, const char *code, const char *message) {
  char safe[128];
  json_sanitize(message, safe, sizeof(safe));
  char ids[24];
  hid_json::i64_dec(id, ids);
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"v\":1,\"id\":%s,\"ok\":false,\"error\":{\"code\":\"%s\","
           "\"message\":\"%s\"}}",
           ids, code, safe);
  tx_begin(buf, false);
}

int append_params_for_block(char *dst, size_t dstlen, int off, dsp_block_id_t id) {
  const auto *info = dsp_block_info(id);
  if (!info) {
    return off;
  }
  bool first = true;
  for (uint8_t i = 0; i < info->param_count && off >= 0; i++) {
    const auto &prm = info->params[i];
    float v = 0.f;
    if (!control_api_get_param(id, prm.name, &v)) {
      continue;
    }
    off = appendf(dst, dstlen, off, "%s\"%s\":", first ? "" : ",", prm.json_name);
    first = false;
    if (off < 0) {
      break;
    }
    /* Integers for preset/hz when whole. */
    if (fabsf(v - roundf(v)) < 1e-4f && fabsf(v) < 1e6f) {
      off = appendf(dst, dstlen, off, "%d", static_cast<int>(lroundf(v)));
    } else {
      off = appendf(dst, dstlen, off, "%.6g", static_cast<double>(v));
    }
  }
  return off;
}

/**
 * @return {@code true} when this build runs the quartz engine, i.e. quartz
 *         modes other than mechanical exist ({@link
 *         control_api_quartz_supported}).
 */
[[nodiscard]] bool quartz_supported() { return control_api_quartz_supported(); }

int encode_state(char *dst, size_t dstlen, int off) {
  off = appendf(dst, dstlen, off, "{\"bypass\":%s,\"blocks\":{",
                control_api_get_bypass() ? "true" : "false");
  if (off < 0) {
    return -1;
  }
  /* Skip DSP_BLOCK_BYPASS (index 0): reported as the top-level "bypass". */
  for (int i = 1; i < static_cast<int>(DSP_BLOCK_COUNT); i++) {
    const auto id = static_cast<dsp_block_id_t>(i);
    off = appendf(dst, dstlen, off, "%s\"%s\":{\"enabled\":%s,\"params\":{",
                  i > 1 ? "," : "", dsp_block_json_name(id),
                  control_api_block_enabled(id) ? "true" : "false");
    if (off < 0) {
      return -1;
    }
    off = append_params_for_block(dst, dstlen, off, id);
    if (off < 0) {
      return -1;
    }
    off = appendf(dst, dstlen, off, "}}");
    if (off < 0) {
      return -1;
    }
  }
  off = appendf(dst, dstlen, off, "}");
  if (off >= 0 && quartz_supported()) {
    control_api_get_quartz(&g_quartz);
    off = appendf(dst, dstlen, off, ",\"quartz\":");
    off = hid_json::encode_quartz_settings(dst, dstlen, off, g_quartz);
  }
  return appendf(dst, dstlen, off, "}");
}

int encode_metrics(char *dst, size_t dstlen, int off) {
  dsp_metrics_t m{};
  control_api_get_metrics(&m);
  return appendf(dst, dstlen, off,
                 "{\"prePeak\":%.6g,\"postPeak\":%.6g,\"preRms\":%.6g,"
                 "\"postRms\":%.6g,\"noiseFloor\":%.6g,\"agcGainDb\":%.6g,"
                 "\"matchScore\":%.6g,\"clipCount\":%lu,\"limiterCount\":%lu}",
                 static_cast<double>(m.pre_peak), static_cast<double>(m.post_peak), static_cast<double>(m.pre_rms),
                 static_cast<double>(m.post_rms), static_cast<double>(m.noise_floor),
                 static_cast<double>(m.agc_gain_db), static_cast<double>(m.match_score),
                 static_cast<unsigned long>(m.clip_count), static_cast<unsigned long>(m.limiter_count));
}

int encode_timing(char *dst, size_t dstlen, int off) {
  dsp_timing_t t{};
  control_api_get_timing(&t);
  return appendf(
      dst, dstlen, off,
      "{\"dspBlockUsLast\":%lu,\"dspBlockUsMean\":%lu,\"dspBlockUsMax\":%lu,"
      "\"dspBlockBudgetUs\":%lu,\"dspBlockSamples\":%lu,\"dspBlocks\":%lu,"
      "\"loopUsLast\":%lu,\"loopUsMean\":%lu,\"loopUsMax\":%lu,"
      "\"loopCount\":%lu,\"idleUsPerS\":%lu,\"loopBusyPct\":%u,"
      "\"underruns\":%lu,\"shortWrites\":%lu,\"ringMin\":%lu,\"ringMax\":%lu,"
      "\"quartzBlockUsLast\":%lu,\"quartzBlockUsMean\":%lu,"
      "\"quartzBlockUsMax\":%lu,\"quartzBlocks\":%lu}",
      static_cast<unsigned long>(t.dsp_block_us_last), static_cast<unsigned long>(t.dsp_block_us_mean),
      static_cast<unsigned long>(t.dsp_block_us_max), static_cast<unsigned long>(t.dsp_block_budget_us),
      static_cast<unsigned long>(t.dsp_block_samples), static_cast<unsigned long>(t.dsp_blocks),
      static_cast<unsigned long>(t.loop_us_last), static_cast<unsigned long>(t.loop_us_mean),
      static_cast<unsigned long>(t.loop_us_max), static_cast<unsigned long>(t.loop_count),
      static_cast<unsigned long>(t.idle_us_per_s), static_cast<unsigned>(t.loop_busy_pct),
      static_cast<unsigned long>(t.underruns), static_cast<unsigned long>(t.short_writes),
      static_cast<unsigned long>(t.ring_min), static_cast<unsigned long>(t.ring_max),
      static_cast<unsigned long>(t.quartz_block_us_last),
      static_cast<unsigned long>(t.quartz_block_us_mean),
      static_cast<unsigned long>(t.quartz_block_us_max),
      static_cast<unsigned long>(t.quartz_blocks));
}

int encode_spectrum(char *dst, size_t dstlen, int off) {
  dsp_spectrum_t s{};
  control_api_get_spectrum(&s);
  off = appendf(dst, dstlen, off,
                "{\"valid\":%s,\"fftN\":%lu,\"sourceHz\":%lu,\"nPeaks\":%lu,"
                "\"peaks\":[",
                s.valid ? "true" : "false", static_cast<unsigned long>(s.fft_n),
                static_cast<unsigned long>(s.source_hz), static_cast<unsigned long>(s.n_peaks));
  for (uint32_t i = 0; i < s.n_peaks && off >= 0; i++) {
    off = appendf(dst, dstlen, off, "%s{\"hz\":%.6g,\"mag\":%.6g}", i ? "," : "",
                  static_cast<double>(s.peak_hz[i]), static_cast<double>(s.peak_mag[i]));
  }
  if (off >= 0) {
    off = appendf(dst, dstlen, off, "]}");
  }
  return off;
}

int encode_matched(char *dst, size_t dstlen, int off) {
  dsp_matched_t m{};
  control_api_get_matched(&m);
  return appendf(dst, dstlen, off,
                 "{\"score\":%.6g,\"templatePeak\":%.6g,\"templateLen\":%lu,"
                 "\"learned\":%s}",
                 static_cast<double>(m.score), static_cast<double>(m.template_peak),
                 static_cast<unsigned long>(m.template_len), m.learned ? "true" : "false");
}

int encode_timing_window(char *dst, size_t dstlen, int off,
                         const dsp_beat_timing_window_t &w) {
  return appendf(dst, dstlen, off,
                 "{\"rateSDay\":%.6g,\"beatErrorMs\":%.6g,\"amplitudeDeg\":%.6g,"
                 "\"samples\":%lu,\"rateValid\":%s,\"beatErrorValid\":%s,"
                 "\"amplitudeValid\":%s}",
                 static_cast<double>(w.rate_s_day), static_cast<double>(w.beat_error_ms),
                 static_cast<double>(w.amplitude_deg), static_cast<unsigned long>(w.samples),
                 w.rate_valid ? "true" : "false",
                 w.beat_error_valid ? "true" : "false",
                 w.amplitude_valid ? "true" : "false");
}

int encode_beat(char *dst, size_t dstlen, int off) {
  dsp_beat_rate_t b{};
  control_api_get_beat_rate(&b);
  off = appendf(
      dst, dstlen, off,
      "{\"vphRaw\":%.6g,\"vphNearest\":%lu,\"confidence\":%.6g,"
      "\"locked\":%s,\"intervalSamples\":%lu,\"eventsSeen\":%lu,"
      "\"intervalsHeld\":%lu,\"sourceHz\":%lu,\"sourceHzMeasured\":%.9g,"
      "\"timebaseOk\":%s,\"ratePoints\":%lu,\"jitterUs\":%.6g,"
      "\"calPpm\":%.6g,\"calSource\":\"%s\","
      "\"rateSDay\":%.6g,\"beatErrorMs\":%.6g,\"amplitudeDeg\":%.6g,"
      "\"liftAngleDeg\":%.6g,\"liftTimeMs\":%.6g,"
      "\"rateValid\":%s,\"beatErrorValid\":%s,\"amplitudeValid\":%s,"
      "\"instant\":",
      static_cast<double>(b.vph_raw), static_cast<unsigned long>(b.vph_nearest), static_cast<double>(b.confidence),
      b.locked ? "true" : "false", static_cast<unsigned long>(b.interval_samples),
      static_cast<unsigned long>(b.events_seen), static_cast<unsigned long>(b.intervals_held),
      static_cast<unsigned long>(b.source_hz),
      static_cast<double>(b.source_hz_measured), b.timebase_ok ? "true" : "false",
      static_cast<unsigned long>(b.rate_points), static_cast<double>(b.jitter_us),
      static_cast<double>(b.cal_ppm), b.cal_source ? b.cal_source : "none",
      static_cast<double>(b.rate_s_day),
      static_cast<double>(b.beat_error_ms), static_cast<double>(b.amplitude_deg),
      static_cast<double>(b.lift_angle_deg), static_cast<double>(b.lift_time_ms),
      b.rate_valid ? "true" : "false", b.beat_error_valid ? "true" : "false",
      b.amplitude_valid ? "true" : "false");
  if (off < 0) {
    return -1;
  }
  off = encode_timing_window(dst, dstlen, off, b.instant);
  if (off < 0) {
    return -1;
  }
  off = appendf(dst, dstlen, off, ",\"avg1s\":");
  if (off < 0) {
    return -1;
  }
  off = encode_timing_window(dst, dstlen, off, b.avg_1s);
  if (off < 0) {
    return -1;
  }
  off = appendf(dst, dstlen, off, ",\"avgS\":%.6g,\"avgLong\":",
                static_cast<double>(b.avg_s));
  if (off < 0) {
    return -1;
  }
  off = encode_timing_window(dst, dstlen, off, b.avg_long);
  if (off < 0) {
    return -1;
  }
  return appendf(dst, dstlen, off, "}");
}

const char *reason_str(dsp_error_reason_t r) {
  switch (r) {
  case DSP_ERR_UNDERRUN:
    return "underrun";
  case DSP_ERR_USB_STALL:
    return "usbStall";
  case DSP_ERR_OVERLOAD:
    return "overload";
  default:
    return "none";
  }
}

int encode_error(char *dst, size_t dstlen, int off, const dsp_error_t &e) {
  char si[24];
  u64_to_dec(e.sample_index, si, sizeof(si));
  return appendf(dst, dstlen, off,
                 "{\"block\":\"%s\",\"reason\":\"%s\",\"uptimeMs\":%lu,"
                 "\"sampleIndex\":\"%s\"}",
                 dsp_block_json_name(e.block), reason_str(e.reason),
                 static_cast<unsigned long>(e.uptime_ms), si);
}

bool apply_block_params(dsp_block_id_t id, const char *obj) {
  const auto *info = dsp_block_info(id);
  if (!info) {
    return false;
  }
  bool ok = true;
  for (uint8_t i = 0; i < info->param_count; i++) {
    const auto &prm = info->params[i];
    double d = 0.0;
    if (!prm.writable || !json_get_double(obj, prm.json_name, &d)) {
      continue;
    }
    if (!control_api_set_param(id, prm.name, static_cast<float>(d))) {
      ok = false;
    }
  }
  return ok;
}

/** {@code badParam} message for a rejected {@code blocks} patch. */
constexpr const char *kPatchRejected = "patch rejected";

/** One {@code quartz} patch key and the control_api parameter it sets. */
struct QuartzPatchParam {
  const char *json_name; /**< Key inside the {@code quartz} object. */
  const char *api_name;  /**< {@link control_api_set_quartz_param} name. */
  const char *range;     /**< Accepted values, for the error message. */
};

/** Numeric quartz settings accepted by patchState (tsMethod is separate). */
constexpr std::array<QuartzPatchParam, 3> kQuartzPatchParams = {{
    {"gateS", "gate", "quartz.gateS must be 0 or 3..960"},
    {"forkHz", "forkHz", "quartz.forkHz must be 300, 341.33, 360, 480 or 720"},
    {"kSigma", "kSigma", "quartz.kSigma must be 4..20"},
}};

/**
 * Applies the {@code quartz} member of a patch: the parameters first, then
 * the mode, so a mode change starts its measurement with the new values.
 * {@code mode} and {@code tsMethod} names are checked before anything is
 * applied; a range error from control_api leaves earlier fields of the
 * same object applied (as for blocks, §6.3).
 *
 * @param obj The extracted {@code quartz} object.
 * @return {@code nullptr} on success, else the {@code badParam} message.
 */
[[nodiscard]] const char *apply_quartz_patch(const char *obj) {
  char name[16];
  auto mode = DSP_QUARTZ_MODE_COUNT;
  const bool has_mode = json_get_string(obj, "mode", name, sizeof(name));
  if (has_mode) {
    mode = control_api_quartz_mode_from_name(name);
    if (mode == DSP_QUARTZ_MODE_COUNT) {
      return "quartz.mode must be mechanical, motor, lcd or fork";
    }
  } else if (json_find_key(obj, "mode") != nullptr) {
    return "quartz.mode must be a string";
  }
  constexpr const char *kBadTs = "quartz.tsMethod must be threshold or xcorr";
  auto ts = DSP_QUARTZ_TS_COUNT;
  const bool has_ts = json_find_key(obj, "tsMethod") != nullptr;
  if (has_ts) {
    double v = -1.0;
    if (json_get_string(obj, "tsMethod", name, sizeof(name))) {
      ts = control_api_quartz_ts_method_from_name(name);
    } else if (json_get_double(obj, "tsMethod", &v) && (v == 0.0 || v == 1.0)) {
      ts = static_cast<dsp_quartz_ts_method_t>(static_cast<int>(v));
    }
    if (ts == DSP_QUARTZ_TS_COUNT) {
      return kBadTs;
    }
  }
  const bool supported = quartz_supported();
  if (!supported &&
      (has_ts || (has_mode && mode != DSP_QUARTZ_MODE_MECHANICAL))) {
    return "quartz timing needs the PCM1808 source";
  }
  for (const auto &prm : kQuartzPatchParams) {
    double v = 0.0;
    if (!json_get_double(obj, prm.json_name, &v)) {
      continue;
    }
    if (!supported) {
      return "quartz timing needs the PCM1808 source";
    }
    if (!control_api_set_quartz_param(prm.api_name, static_cast<float>(v))) {
      return prm.range;
    }
  }
  if (has_ts &&
      !control_api_set_quartz_param("tsMethod", static_cast<float>(ts))) {
    return kBadTs;
  }
  if (has_mode && !control_api_set_quartz_mode(mode)) {
    return "quartz timing needs the PCM1808 source";
  }
  return nullptr;
}

/**
 * Applies a merge patch: {@code bypass}, then {@code quartz}, then
 * {@code blocks}.
 *
 * @param patch_json The extracted {@code patch} object.
 * @return {@code nullptr} on success, else the {@code badParam} message.
 */
[[nodiscard]] const char *apply_patch(const char *patch_json) {
  bool bypass;
  if (json_get_bool(patch_json, "bypass", &bypass)) {
    control_api_set_bypass(bypass);
  }
  if (const char *quartz = json_find_key(patch_json, "quartz")) {
    if (!json_extract_object(quartz, g_quartz_obj, sizeof(g_quartz_obj))) {
      return "quartz must be an object";
    }
    if (const char *why = apply_quartz_patch(g_quartz_obj)) {
      return why;
    }
  }
  const char *blocks = json_find_key(patch_json, "blocks");
  if (!blocks) {
    return nullptr;
  }
  char *blocks_obj = g_blocks_obj;
  if (!json_extract_object(blocks, blocks_obj, sizeof(g_blocks_obj))) {
    return kPatchRejected;
  }
  for (int i = 1; i < static_cast<int>(DSP_BLOCK_COUNT); i++) {
    const auto id = static_cast<dsp_block_id_t>(i);
    const auto *bv = json_find_key(blocks_obj, dsp_block_json_name(id));
    if (!bv) {
      continue;
    }
    char bobj[384];
    if (!json_extract_object(bv, bobj, sizeof(bobj))) {
      return kPatchRejected;
    }
    bool en;
    if (json_get_bool(bobj, "enabled", &en)) {
      if (!control_api_set_block_enabled(id, en)) {
        return kPatchRejected;
      }
    }
    const char *params = json_find_key(bobj, "params");
    if (params) {
      char pobj[256];
      if (json_extract_object(params, pobj, sizeof(pobj))) {
        if (!apply_block_params(id, pobj)) {
          return kPatchRejected;
        }
      }
    }
  }
  return nullptr;
}

/* --- Request handlers: each appends the "result" value at {@code off}. --- */

using ResultEncoder = int (*)(const char *req, char *dst, size_t dstlen, int off);

/** {@code ping}: echoes {@code t} as a full int64 plus the uptime. */
int result_ping(const char *req, char *dst, size_t dstlen, int off) {
  int64_t t = 0;
  json_get_i64(req, "t", &t);
  return hid_json::encode_ping(dst, dstlen, off, t, millis());
}

/**
 * {@code hello}: identity and capabilities, including the pipeline graph
 * ({@link hid_json::encode_hello}).
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int result_hello(const char *, char *dst, size_t dstlen, int off) {
  hid_json::HelloParams p{};
  p.fw_version = FW_VERSION;
  p.board = TG_BOARD_NAME;
  p.source = AUDIO_SOURCE_NAME;
  p.vid = static_cast<unsigned>(USB_VENDOR_ID);
  p.pid = static_cast<unsigned>(USB_PRODUCT_ID);
  p.max_json = static_cast<unsigned>(kMaxJson);
  p.metrics_hz_max = kMetricsHzMax;
  p.beat_hz_max = kBeatHzMax;
  p.timing_hz_max = kTimingHzMax;
  p.quartz_hz_max = kQuartzHzMax;
  p.quartz_pulses_max = static_cast<unsigned>(hid_json::kQuartzPulsesMax);
  p.quartz = quartz_supported();
  p.gps = control_api_gps_built();
  p.pps = control_api_pps_built();
  return hid_json::encode_hello(dst, dstlen, off, p);
}

int result_get_state(const char *, char *dst, size_t dstlen, int off) {
  return encode_state(dst, dstlen, off);
}

int result_metrics(const char *, char *dst, size_t dstlen, int off) {
  return encode_metrics(dst, dstlen, off);
}

int result_timing(const char *, char *dst, size_t dstlen, int off) {
  return encode_timing(dst, dstlen, off);
}

int result_reset_timing(const char *, char *dst, size_t dstlen, int off) {
  control_api_reset_timing();
  return appendf(dst, dstlen, off, "{\"cleared\":true}");
}

int result_beat(const char *, char *dst, size_t dstlen, int off) {
  return encode_beat(dst, dstlen, off);
}

int result_spectrum(const char *, char *dst, size_t dstlen, int off) {
  return encode_spectrum(dst, dstlen, off);
}

int result_matched(const char *, char *dst, size_t dstlen, int off) {
  return encode_matched(dst, dstlen, off);
}

/**
 * {@code getQuartz} result and {@code quartz} notify data: the full quartz
 * snapshot ({@link hid_json::encode_quartz}).
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int encode_quartz(char *dst, size_t dstlen, int off) {
  control_api_get_quartz(&g_quartz);
  return hid_json::encode_quartz(dst, dstlen, off, g_quartz);
}

/**
 * {@code getQuartz} / {@code quartzRestart}: the quartz snapshot.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int result_quartz(const char *, char *dst, size_t dstlen, int off) {
  return encode_quartz(dst, dstlen, off);
}

/**
 * {@code getQuartzPulses}: up to {@code max} recent motor pulses (1..
 * {@link hid_json::kQuartzPulsesMax}, default the maximum), newest first.
 * Empty outside motor mode.
 *
 * @param req Request JSON (optional {@code max}).
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int result_quartz_pulses(const char *req, char *dst, size_t dstlen, int off) {
  long max = static_cast<long>(hid_json::kQuartzPulsesMax);
  json_get_int(req, "max", &max);
  if (max < 1) {
    max = 1;
  }
  if (max > static_cast<long>(hid_json::kQuartzPulsesMax)) {
    max = static_cast<long>(hid_json::kQuartzPulsesMax);
  }
  const auto n = control_api_copy_quartz_pulses(g_quartz_pulses,
                                                static_cast<uint32_t>(max));
  return hid_json::encode_quartz_pulses(dst, dstlen, off, g_quartz_pulses, n);
}

/**
 * {@code getDevice} / {@code setDevice} / {@code start} / {@code stop}:
 * the {@code DeviceStatus} object.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int result_device(const char *, char *dst, size_t dstlen, int off) {
  device_status_t st{};
  control_api_get_device(&st);
  return hid_json::encode_device_status(dst, dstlen, off, st);
}

/**
 * {@code getBeats}: the newest {@code max} beats (1..
 * {@link hid_json::kBeatsMax}, default 8), newest first. The reply holds as
 * many of them as fit the message limit.
 *
 * @param req Request JSON (optional {@code max}).
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int result_beats(const char *req, char *dst, size_t dstlen, int off) {
  long max = 8;
  json_get_int(req, "max", &max);
  if (max < 1) {
    max = 1;
  }
  if (max > static_cast<long>(hid_json::kBeatsMax)) {
    max = static_cast<long>(hid_json::kBeatsMax);
  }
  const auto n = control_api_copy_beats(g_beats, static_cast<uint32_t>(max));
  return hid_json::encode_beats(dst, dstlen, off, g_beats, n, nullptr);
}

/** {@code getGps}: the {@code GpsInfo} object ({@link hid_json::encode_gps}). */
int result_gps(const char *, char *dst, size_t dstlen, int off) {
  control_api_get_gps(&g_gps);
  return hid_json::encode_gps(dst, dstlen, off, g_gps);
}

/**
 * {@code getTimebaseLog}: one page of the {@code TimebaseLog} object
 * (optional {@code offset}, default 0; see hid_json::encode_timebase_log).
 */
int result_timebase_log(const char *req, char *dst, size_t dstlen, int off) {
  long offset = 0;
  json_get_int(req, "offset", &offset);
  if (offset < 0) {
    offset = 0;
  }
  control_api_timebase_log(&g_tblog);
  return hid_json::encode_timebase_log(
      dst, dstlen, off, g_tblog, static_cast<uint32_t>(offset),
      control_api_timebase_log_pending(),
      control_api_timebase_log_delta_ppm(),
      control_api_timebase_log_last_write_unix(), nullptr);
}

/** {@code clearTimebaseLog} reply: the (empty) log as offset 0. */
int result_timebase_log_first(const char *, char *dst, size_t dstlen,
                              int off) {
  return result_timebase_log("", dst, dstlen, off);
}

/**
 * {@code getEvents}: the newest {@code limit} events (1..DSP_EVENT_LOG_CAP,
 * default 8), newest first. The reply holds as many of them as fit the
 * message limit ({@link hid_json::encode_events}), always the newest ones.
 *
 * @param req Request JSON (optional {@code limit}).
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @return New length, or -1 if it does not fit.
 */
int result_events(const char *req, char *dst, size_t dstlen, int off) {
  long limit = 8;
  json_get_int(req, "limit", &limit);
  if (limit < 1) {
    limit = 1;
  }
  if (limit > static_cast<long>(DSP_EVENT_LOG_CAP)) {
    limit = static_cast<long>(DSP_EVENT_LOG_CAP);
  }
  const auto n = control_api_copy_events(g_ev_scratch, static_cast<uint32_t>(limit));
  return hid_json::encode_events(dst, dstlen, off, g_ev_scratch, n, nullptr);
}

int result_errors(const char *, char *dst, size_t dstlen, int off) {
  const auto n = control_api_copy_errors(g_err_scratch, DSP_ERROR_LOG_CAP);
  off = appendf(dst, dstlen, off, "{\"errors\":[");
  for (uint32_t i = 0; i < n && off >= 0; i++) {
    if (i) {
      off = appendf(dst, dstlen, off, ",");
    }
    if (off >= 0) {
      off = encode_error(dst, dstlen, off, g_err_scratch[i]);
    }
  }
  return (off < 0) ? off : appendf(dst, dstlen, off, "]}");
}

int result_clear_errors(const char *, char *dst, size_t dstlen, int off) {
  control_api_clear_errors();
  g_have_last_error = false;
  g_last_error_uptime = 0;
  return appendf(dst, dstlen, off, "{\"cleared\":true}");
}

/** Reads {@code key} as an integer rate clamped to {@code [0, max_hz]}. */
void read_rate(const char *req, const char *key, uint8_t max_hz, uint8_t *out) {
  long hz = 0;
  if (!json_get_int(req, key, &hz)) {
    return;
  }
  if (hz < 0) {
    hz = 0;
  }
  if (hz > max_hz) {
    hz = max_hz;
  }
  *out = static_cast<uint8_t>(hz);
}

/**
 * Seeds the {@code quartzPulse} cursor past the pulses already logged, so
 * subscribing does not replay the pulse history. With no pulse yet, the
 * cursor points at sequence 0 of the current measurement.
 */
void seed_quartz_pulse_baseline() {
  dsp_quartz_pulse_t newest{};
  if (control_api_copy_quartz_pulses(&newest, 1) > 0) {
    g_pulse_measurement = newest.measurement_id;
    g_pulse_next_seq = newest.sequence + 1u;
    return;
  }
  control_api_get_quartz(&g_quartz);
  g_pulse_measurement = g_quartz.measurement_id;
  g_pulse_next_seq = 0;
}

int result_subscribe(const char *req, char *dst, size_t dstlen, int off) {
  read_rate(req, "metricsHz", kMetricsHzMax, &g_sub_metrics_hz);
  read_rate(req, "beatHz", kBeatHzMax, &g_sub_beat_hz);
  read_rate(req, "timingHz", kTimingHzMax, &g_sub_timing_hz);
  read_rate(req, "quartzHz", kQuartzHzMax, &g_sub_quartz_hz);
  bool flag;
  if (json_get_bool(req, "quartzPulses", &flag)) {
    g_sub_quartz_pulses = flag;
  }
  if (!quartz_supported()) {
    g_sub_quartz_hz = 0;
    g_sub_quartz_pulses = false;
  }
  if (json_get_bool(req, "events", &flag)) {
    g_sub_events = flag;
  }
  if (json_get_bool(req, "errors", &flag)) {
    g_sub_errors = flag;
  }
  if (json_get_bool(req, "beats", &flag)) {
    g_sub_beats = flag;
  }
  /* Seed baselines so we don't dump history on subscribe. */
  dsp_event_t last_ev;
  g_last_event_peak =
      control_api_copy_events(&last_ev, 1) > 0 ? last_ev.peak_sample : 0u;
  dsp_error_t last_err;
  g_have_last_error = control_api_copy_errors(&last_err, 1) > 0;
  if (g_have_last_error) {
    g_last_error_uptime = last_err.uptime_ms;
  }
  seed_quartz_pulse_baseline();
  g_beat_next_seq = control_api_beat_seq();
  const auto now = millis();
  g_last_metrics_ms = now;
  g_last_beat_ms = now;
  g_last_timing_ms = now;
  g_last_quartz_ms = now;
  return appendf(
      dst, dstlen, off,
      "{\"metricsHz\":%u,\"beatHz\":%u,\"timingHz\":%u,"
      "\"quartzHz\":%u,\"events\":%s,\"errors\":%s,"
      "\"quartzPulses\":%s,\"beats\":%s}",
      static_cast<unsigned>(g_sub_metrics_hz),
      static_cast<unsigned>(g_sub_beat_hz),
      static_cast<unsigned>(g_sub_timing_hz),
      static_cast<unsigned>(g_sub_quartz_hz), g_sub_events ? "true" : "false",
      g_sub_errors ? "true" : "false", g_sub_quartz_pulses ? "true" : "false",
      g_sub_beats ? "true" : "false");
}

/** {@code getTime}: device µs timer, sampled first to minimise latency. */
int result_time(const char *, char *dst, size_t dstlen, int off) {
  const uint64_t t = time_us_64();
  return appendf(dst, dstlen, off, "{\"tUs\":%llu,\"uptimeMs\":%lu}",
                 static_cast<unsigned long long>(t),
                 static_cast<unsigned long>(millis()));
}

/** {@code getTimebase} (also the reply to set / save / clear). */
int result_timebase(const char *, char *dst, size_t dstlen, int off) {
  timebase_info_t tb{};
  timebase_get(&tb);
  return appendf(
      dst, dstlen, off,
      "{\"ppm\":%.6g,\"source\":\"%s\",\"calPpm\":%.6g,"
      "\"calSource\":\"%s\",\"dateUnix\":%lu,\"durationS\":%lu,"
      "\"saved\":%s,\"logDeltaPpm\":%.6g,\"logPending\":%s,"
      "\"pps\":{\"enabled\":%s,\"locked\":%s,\"ppm\":%.6g,"
      "\"window\":%lu,\"good\":%lu,\"rejects\":%lu,\"noFix\":%lu},"
      "\"gps\":{\"enabled\":%s,\"fix\":%s,\"quality\":%u,\"sats\":%u,"
      "\"hdop\":%.3g,\"utcUnix\":%lu,\"ageMs\":%ld,\"sentences\":%lu,"
      "\"errors\":%lu,\"overruns\":%lu}}",
      static_cast<double>(tb.ppm), timebase_source_name(tb.source),
      static_cast<double>(tb.cal_ppm), timebase_source_name(tb.cal_source),
      static_cast<unsigned long>(tb.cal_date_unix),
      static_cast<unsigned long>(tb.cal_duration_s), tb.cal_saved ? "true" : "false",
      static_cast<double>(tb.log_delta_ppm), tb.log_pending ? "true" : "false",
      tb.pps_enabled ? "true" : "false", tb.pps_locked ? "true" : "false",
      static_cast<double>(tb.pps_ppm), static_cast<unsigned long>(tb.pps_pulses),
      static_cast<unsigned long>(tb.pps_good), static_cast<unsigned long>(tb.pps_rejects),
      static_cast<unsigned long>(tb.pps_no_fix), tb.gps_enabled ? "true" : "false",
      tb.gps_fix ? "true" : "false", tb.gps_quality, tb.gps_sats,
      static_cast<double>(tb.gps_hdop), static_cast<unsigned long>(tb.gps_utc_unix),
      tb.gps_age_ms == 0xFFFFFFFFu ? -1L : static_cast<long>(tb.gps_age_ms),
      static_cast<unsigned long>(tb.gps_sentences), static_cast<unsigned long>(tb.gps_errors),
      static_cast<unsigned long>(tb.gps_overruns));
}

/**
 * Optional pre-step for an op: validates / applies the request and returns
 * {@code false} after sending its own error reply.
 */
using RequestApplier = bool (*)(int64_t id, const char *req);

bool apply_set_timebase(int64_t id, const char *req) {
  double ppm = 0.0;
  double delta = 0.0;
  const bool has_ppm = json_get_double(req, "ppm", &ppm);
  const bool has_delta = json_get_double(req, "logDeltaPpm", &delta);
  if (!has_ppm && !has_delta) {
    send_error(id, "badRequest", "missing ppm");
    return false;
  }
  /* Validate everything before applying anything. */
  if (has_delta && !timebase_log_delta_valid(static_cast<float>(delta))) {
    send_error(id, "badParam", "logDeltaPpm must be 0.05..10");
    return false;
  }
  timebase_source_t src = TIMEBASE_SRC_MANUAL;
  double date = 0.0, dur = 0.0;
  if (has_ppm) {
    char name[16];
    if (json_get_string(req, "source", name, sizeof(name)) &&
        !timebase_source_from_name(name, &src)) {
      send_error(id, "badParam", "source must be none, manual or ntp");
      return false;
    }
    json_get_double(req, "dateUnix", &date);
    json_get_double(req, "durationS", &dur);
    if (!(std::fabs(ppm) <= static_cast<double>(TIMEBASE_PPM_MAX)) ||
        date < 0.0 || date > 4294967295.0 || dur < 0.0 ||
        dur > 4294967295.0) {
      send_error(id, "badParam", "ppm must be -1000..1000");
      return false;
    }
  }
  /* The flash write goes first: if it is refused nothing is applied. It is
   * skipped when the value is unchanged, so a client that always sends it
   * neither wears flash nor gets 'busy'. */
  if (has_delta &&
      static_cast<float>(delta) != control_api_timebase_log_delta_ppm()) {
    switch (control_api_set_timebase_log_delta_ppm(static_cast<float>(delta))) {
    case TIMEBASE_LOGDELTA_OK:
      break;
    case TIMEBASE_LOGDELTA_BAD_PARAM:
      send_error(id, "badParam", "logDeltaPpm must be 0.05..10");
      return false;
    case TIMEBASE_LOGDELTA_BUSY:
      send_error(id, "busy", "close the audio stream before saving");
      return false;
    case TIMEBASE_LOGDELTA_FAILED:
      send_error(id, "flashError", "flash write failed");
      return false;
    }
  }
  /* With logDeltaPpm alone the stored calibration is left untouched. */
  if (has_ppm && !timebase_set(static_cast<float>(ppm), src,
                               static_cast<uint32_t>(date),
                               static_cast<uint32_t>(dur))) {
    send_error(id, "badParam", "ppm must be -1000..1000");
    return false;
  }
  return true;
}

bool apply_save_timebase(int64_t id, const char *) {
  switch (timebase_save()) {
  case TIMEBASE_SAVE_OK:
    return true;
  case TIMEBASE_SAVE_BUSY:
    send_error(id, "busy", "close the audio stream before saving");
    return false;
  case TIMEBASE_SAVE_FAILED:
    break;
  }
  send_error(id, "flashError", "flash write failed");
  return false;
}

bool apply_clear_timebase(int64_t, const char *) {
  timebase_clear();
  return true;
}

/**
 * {@code setDevice}: selects and persists the device mode.
 *
 * @param id Request id, for the error reply.
 * @param req Request JSON ({@code mode}).
 * @return {@code false} after replying {@code badParam} (missing or unknown
 *         mode), {@code busy} (a USB stream is open) or {@code flashError}.
 */
bool apply_set_device(int64_t id, const char *req) {
  constexpr const char *kBadMode = "device.mode must be usbMic or standalone";
  char name[16];
  device_mode_t mode;
  if (!json_get_string(req, "mode", name, sizeof(name)) ||
      !control_api_device_mode_from_name(name, &mode)) {
    send_error(id, "badParam", kBadMode);
    return false;
  }
  switch (control_api_set_device_mode(mode)) {
  case DEVICE_SET_OK:
    return true;
  case DEVICE_SET_BUSY:
    send_error(id, "busy", "close the audio stream before changing mode");
    return false;
  case DEVICE_SET_BAD_PARAM:
    send_error(id, "badParam", kBadMode);
    return false;
  case DEVICE_SET_FAILED:
    break;
  }
  send_error(id, "flashError", "flash write failed");
  return false;
}

/**
 * {@code start}: sets the run latch.
 *
 * @param id Request id, for the error reply.
 * @return {@code false} after replying {@code badRequest} when the capture
 *         backend is unhealthy.
 */
bool apply_start(int64_t id, const char *) {
  if (control_api_run_start() == DEVICE_RUN_OK) {
    return true;
  }
  send_error(id, "badRequest", "capture backend error (stream state: error)");
  return false;
}

/**
 * {@code stop}: clears the run latch. Always succeeds; while a USB stream is
 * open capture keeps running and the reply still says {@code running}.
 */
bool apply_stop(int64_t, const char *) {
  control_api_run_stop();
  return true;
}

/**
 * {@code getGps} pre-step: refuses with {@code badRequest} on a build
 * without GPS_UART_RX.
 */
bool apply_require_gps(int64_t id, const char *) {
  if (control_api_gps_built()) {
    return true;
  }
  send_error(id, "badRequest", "GPS not built in");
  return false;
}

/**
 * {@code clearTimebaseLog}: clears the log in RAM and flash.
 *
 * @return {@code false} after replying {@code busy} while a USB stream is
 *         open, or {@code flashError}.
 */
bool apply_clear_timebase_log(int64_t id, const char *) {
  switch (control_api_timebase_log_clear()) {
  case TIMEBASE_SAVE_OK:
    return true;
  case TIMEBASE_SAVE_BUSY:
    send_error(id, "busy", "close the audio stream before saving");
    return false;
  case TIMEBASE_SAVE_FAILED:
    break;
  }
  send_error(id, "flashError", "flash write failed");
  return false;
}

/**
 * {@code quartzRestart}: starts a new measurement in the active quartz mode
 * (also after a motor gate froze the result).
 *
 * @param id Request id, for the error reply.
 * @return {@code false} after replying {@code badRequest} in mechanical
 *         mode or on a build without the quartz engine.
 */
bool apply_quartz_restart(int64_t id, const char *) {
  if (control_api_quartz_restart()) {
    return true;
  }
  send_error(id, "badRequest",
             quartz_supported() ? "quartzRestart needs motor, lcd or fork mode"
                                : "quartz timing needs the PCM1808 source");
  return false;
}

struct OpHandler {
  const char *op;
  ResultEncoder encode;
  RequestApplier apply = nullptr; /**< Set only for ops that change state. */
};

const OpHandler kOps[] = {
    {"ping", result_ping},
    {"hello", result_hello},
    {"getState", result_get_state},
    {"getMetrics", result_metrics},
    {"getTiming", result_timing},
    {"resetTiming", result_reset_timing},
    {"getBeat", result_beat},
    {"getSpectrum", result_spectrum},
    {"getMatched", result_matched},
    {"getEvents", result_events},
    {"getErrors", result_errors},
    {"clearErrors", result_clear_errors},
    {"subscribe", result_subscribe},
    {"getTime", result_time},
    {"getTimebase", result_timebase},
    {"setTimebase", result_timebase, apply_set_timebase},
    {"saveTimebase", result_timebase, apply_save_timebase},
    {"clearTimebase", result_timebase, apply_clear_timebase},
    {"getQuartz", result_quartz},
    {"getQuartzPulses", result_quartz_pulses},
    {"quartzRestart", result_quartz, apply_quartz_restart},
    {"getDevice", result_device},
    {"setDevice", result_device, apply_set_device},
    {"start", result_device, apply_start},
    {"stop", result_device, apply_stop},
    {"getBeats", result_beats},
    {"getGps", result_gps, apply_require_gps},
    {"getTimebaseLog", result_timebase_log},
    {"clearTimebaseLog", result_timebase_log_first, apply_clear_timebase_log},
};

/** Applies a patchState request; replies with an error and returns false on failure. */
bool apply_patch_request(int64_t id, const char *json) {
  const char *patch = json_find_key(json, "patch");
  if (!patch || !json_extract_object(patch, g_patch_obj, sizeof(g_patch_obj))) {
    send_error(id, "badRequest", "missing patch object");
    return false;
  }
  if (const char *why = apply_patch(g_patch_obj)) {
    send_error(id, "badParam", why);
    return false;
  }
  return true;
}

void handle_request(const char *json) {
  int64_t id = 0;
  if (!json_get_i64(json, "id", &id)) {
    send_error(0, "badRequest", "missing id");
    return;
  }
  long v = 0;
  if (!json_get_int(json, "v", &v) || v != 1) {
    send_error(id, "badRequest", "unsupported protocol version");
    return;
  }
  char op[32];
  if (!json_get_string(json, "op", op, sizeof(op))) {
    send_error(id, "badRequest", "missing op");
    return;
  }

  ResultEncoder encode = nullptr;
  if (strcmp(op, "patchState") == 0) {
    if (!apply_patch_request(id, json)) {
      return;
    }
    encode = result_get_state; /* patchState replies with the new state */
  } else {
    for (const auto &h : kOps) {
      if (strcmp(op, h.op) == 0) {
        if (h.apply && !h.apply(id, json)) {
          return; /* applier already replied with an error */
        }
        encode = h.encode;
        break;
      }
    }
  }
  if (!encode) {
    send_error(id, "unknownOp", op);
    return;
  }

  char ids[24];
  hid_json::i64_dec(id, ids);
  int off = appendf(g_out, sizeof(g_out), 0,
                    "{\"v\":1,\"id\":%s,\"ok\":true,\"result\":", ids);
  if (off >= 0) {
    off = encode(json, g_out, sizeof(g_out), off);
  }
  if (off >= 0) {
    off = appendf(g_out, sizeof(g_out), off, "}");
  }
  if (off < 0) {
    send_error(id, "tooLarge", "response exceeds buffer");
    return;
  }
  tx_begin(g_out, false);
}

/**
 * Wraps {@code encode_data} in a notify envelope for {@code op} and starts TX.
 * No-op while a previous transfer is still in flight.
 */
void push_notify(const char *op, int (*encode_data)(char *, size_t, int)) {
  if (g_tx_active) {
    return;
  }
  int off = appendf(g_out, sizeof(g_out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"%s\",\"data\":", op);
  if (off >= 0) {
    off = encode_data(g_out, sizeof(g_out), off);
  }
  if (off >= 0) {
    off = appendf(g_out, sizeof(g_out), off, "}");
  }
  if (off >= 0) {
    tx_begin(g_out, true);
  }
}

/**
 * Pushes the events not yet sent as one {@code event} notify, oldest first
 * (consistent with {@code beats} and {@code quartzPulse}). The reply holds as
 * many as fit one message; the cursor advances only past the events actually
 * encoded, so the rest follow on the next polls and none is lost.
 *
 * New events are those resident in the log with {@code peak_sample} above
 * {@link g_last_event_peak}. If the log lapped the cursor (more than
 * DSP_EVENT_LOG_CAP events since the last push) the evicted ones are gone
 * and the push starts at the oldest resident event. If the newest resident
 * event is older than the cursor the sample clock restarted, so the cursor is
 * reset and every resident event is new.
 */
void push_notify_events(void) {
  if (g_tx_active || !g_sub_events) {
    return;
  }
  dsp_event_t *ev = g_ev_scratch;
  const auto n = control_api_copy_events(ev, DSP_EVENT_LOG_CAP); /* newest first */
  if (n == 0) {
    return;
  }
  if (ev[0].peak_sample < g_last_event_peak) {
    g_last_event_peak = 0;
  }
  uint32_t neu = 0; /* newest-first prefix that is newer than the cursor */
  while (neu < n && ev[neu].peak_sample > g_last_event_peak) {
    neu++;
  }
  if (neu == 0) {
    return;
  }
  std::reverse(ev, ev + neu); /* oldest first */
  int off = appendf(g_out, sizeof(g_out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"event\",\"data\":");
  uint32_t sent = 0;
  if (off >= 0) {
    off = hid_json::encode_events(g_out, sizeof(g_out), off, ev, neu, &sent);
  }
  if (off >= 0) {
    off = appendf(g_out, sizeof(g_out), off, "}");
  }
  if (off >= 0 && sent > 0) {
    g_last_event_peak = ev[sent - 1].peak_sample;
    tx_begin(g_out, true);
  }
}

/** Latest fault, captured by push_notify_errors for encode_latest_error. */
dsp_error_t g_notify_error;

int encode_latest_error(char *dst, size_t dstlen, int off) {
  return encode_error(dst, dstlen, off, g_notify_error);
}

void push_notify_errors(void) {
  if (g_tx_active || !g_sub_errors) {
    return;
  }
  if (control_api_copy_errors(&g_notify_error, 1) == 0) {
    return;
  }
  const bool is_new =
      g_have_last_error && g_notify_error.uptime_ms != g_last_error_uptime;
  g_last_error_uptime = g_notify_error.uptime_ms;
  g_have_last_error = true;
  if (is_new) {
    push_notify("error", encode_latest_error);
  }
}

/** Pulse captured by push_notify_quartz_pulse for encode_notify_pulse. */
dsp_quartz_pulse_t g_notify_pulse;

/** push_notify adapter for {@link g_notify_pulse}. */
int encode_notify_pulse(char *dst, size_t dstlen, int off) {
  return hid_json::encode_quartz_pulse(dst, dstlen, off, g_notify_pulse);
}

/**
 * Pushes the oldest closed motor pulse not yet sent as one
 * {@code quartzPulse} notify (one per call, since one TX is in flight at a
 * time; at most one pulse per second arrives at the shortest step, and the
 * subscribe loop runs every main-loop pass).
 *
 * A pulse is pushed once its 50 ms group has closed, so {@code extras} and
 * {@code clipped} are final. The cursor is (measurement_id, sequence): the
 * log only holds pulses of the current measurement, so a different
 * measurement_id means a restart, and every pulse of the new measurement
 * is new (sequences restart at 0).
 */
void push_notify_quartz_pulse(void) {
  if (g_tx_active || !g_sub_quartz_pulses) {
    return;
  }
  const auto n =
      control_api_copy_quartz_pulses(g_quartz_pulses, DSP_QUARTZ_PULSE_LOG_CAP);
  if (n == 0) {
    return;
  }
  if (g_quartz_pulses[0].measurement_id != g_pulse_measurement) {
    g_pulse_measurement = g_quartz_pulses[0].measurement_id;
    g_pulse_next_seq = 0;
  }
  /* Newest first: walk from the oldest to the first unsent pulse. */
  for (auto i = n; i-- > 0;) {
    const auto &p = g_quartz_pulses[i];
    if (p.sequence < g_pulse_next_seq) {
      continue;
    }
    if (!p.closed) {
      return; /* later pulses cannot be closed before this one */
    }
    g_notify_pulse = p;
    g_pulse_next_seq = p.sequence + 1u;
    push_notify("quartzPulse", encode_notify_pulse);
    return;
  }
}

/**
 * Pushes the oldest unsent beats (at most 8) as one {@code beats} notify,
 * oldest first. The cursor is the beat {@code seq}: seeded by subscribe, and
 * re-seeded to 0 when it runs backwards (a stream reset
 * restarts the numbering, so the host must treat it as a new stream). If the
 * ring wrapped past the cursor the lost beats are skipped. A notify that is
 * too large for one message carries fewer beats; the rest follow in the next
 * notify.
 */
void push_notify_beats(void) {
  if (g_tx_active || !g_sub_beats) {
    return;
  }
  const auto seq = control_api_beat_seq();
  if (seq < g_beat_next_seq) {
    /* Stream reset: numbering restarted, every beat since is new. */
    g_beat_next_seq = 0;
  }
  if (seq == g_beat_next_seq) {
    return;
  }
  const auto n = control_api_copy_beats_since(g_beat_next_seq, g_beats,
                                              hid_json::kBeatsPerNotifyMax);
  if (n == 0) {
    g_beat_next_seq = seq; /* all evicted from the ring */
    return;
  }
  int off = appendf(g_out, sizeof(g_out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"beats\",\"data\":");
  uint32_t emitted = 0;
  off = hid_json::encode_beats(g_out, sizeof(g_out), off, g_beats, n, &emitted);
  if (off >= 0) {
    off = appendf(g_out, sizeof(g_out), off, "}");
  }
  if (off >= 0 && emitted > 0) {
    g_beat_next_seq = g_beats[emitted - 1u].seq + 1u;
    tx_begin(g_out, true);
  }
}

void subscribe_poll(void) {
  auto now = millis();
  if (g_sub_metrics_hz > 0) {
    uint32_t period = 1000u / g_sub_metrics_hz;
    if ((now - g_last_metrics_ms) >= period) {
      g_last_metrics_ms = now;
      push_notify("metrics", encode_metrics);
    }
  }
  if (g_sub_beat_hz > 0 && !g_tx_active) {
    uint32_t period = 1000u / g_sub_beat_hz;
    if ((now - g_last_beat_ms) >= period) {
      g_last_beat_ms = now;
      push_notify("beat", encode_beat);
    }
  }
  if (g_sub_timing_hz > 0 && !g_tx_active) {
    uint32_t period = 1000u / g_sub_timing_hz;
    if ((now - g_last_timing_ms) >= period) {
      g_last_timing_ms = now;
      push_notify("timing", encode_timing);
    }
  }
  if (g_sub_quartz_hz > 0 && !g_tx_active) {
    uint32_t period = 1000u / g_sub_quartz_hz;
    if ((now - g_last_quartz_ms) >= period) {
      g_last_quartz_ms = now;
      push_notify("quartz", encode_quartz);
    }
  }
  push_notify_quartz_pulse();
  push_notify_beats();
  push_notify_events();
  push_notify_errors();
}

void on_host_report(uint8_t report_id, hid_report_type_t report_type,
                    uint8_t const *buffer, uint16_t bufsize) {
  (void)report_type;
  if (g_rx_ready) {
    return; /* drop while prior message pending */
  }
  const auto *p = buffer;
  auto n = bufsize;
  /* Some stacks leave report ID as first byte. */
  if (n > 0 && report_id == 0 && p[0] == kReportId) {
    p++;
    n--;
    report_id = kReportId;
  }
  if (report_id != 0 && report_id != kReportId) {
    return;
  }
  if (n < kHeaderLen) {
    return;
  }

  uint8_t flags = p[0];
  uint8_t seq = p[1];
  uint8_t frag_index = p[2];
  uint8_t frag_count = p[3];
  uint8_t payload_len = p[4];
  (void)flags;
  if (frag_count == 0 || frag_index >= frag_count || payload_len > kMaxChunk) {
    rx_reset();
    return;
  }
  if (static_cast<uint16_t>(frag_count) * static_cast<uint16_t>(kMaxChunk) > kMaxJson) {
    rx_reset();
    return;
  }
  if (n < static_cast<uint16_t>(kHeaderLen + payload_len)) {
    return;
  }

  auto now = millis();
  if (g_rx_deadline_ms != 0 && static_cast<int32_t>(now - g_rx_deadline_ms) > 0) {
    rx_reset();
  }

  if (g_rx_frag_count == 0 || seq != g_rx_seq) {
    rx_reset();
    g_rx_seq = seq;
    g_rx_frag_count = frag_count;
    g_rx_deadline_ms = now + kReasmTimeoutMs;
  }
  if (frag_count != g_rx_frag_count) {
    rx_reset();
    return;
  }
  if (frag_index < sizeof(g_rx_seen) * 8 &&
      (g_rx_seen[frag_index / 8] & (1u << (frag_index % 8)))) {
    return; /* duplicate */
  }
  if (frag_index < sizeof(g_rx_seen) * 8) {
    g_rx_seen[frag_index / 8] |= static_cast<uint8_t>(1u << (frag_index % 8));
  }

  auto offset = static_cast<uint16_t>(frag_index) * kMaxChunk;
  if (static_cast<uint32_t>(offset) + payload_len > kMaxJson) {
    rx_reset();
    return;
  }
  memcpy(&g_rx_json[offset], &p[kHeaderLen], payload_len);
  if (offset + payload_len > g_rx_len) {
    g_rx_len = static_cast<uint16_t>(offset + payload_len);
  }
  g_rx_got++;
  if (g_rx_got >= g_rx_frag_count) {
    g_rx_json[g_rx_len] = '\0';
    g_rx_ready = true;
  }
}

uint16_t on_get_report(uint8_t, hid_report_type_t, uint8_t *, uint16_t) {
  return 0;
}

} // namespace

bool hid_control_usb_begin(void) {
  g_hid.setPollInterval(2);
  g_hid.enableOutEndpoint(true);
  g_hid.setReportDescriptor(kHidReportDesc, sizeof(kHidReportDesc));
  g_hid.setStringDescriptor("Timegrapher Control");
  g_hid.setReportCallback(on_get_report, on_host_report);
  if (!g_hid.begin()) {
    return false;
  }
  return true;
}

void hid_control_task(void) {
  if (g_rx_deadline_ms != 0 && !g_rx_ready && g_rx_frag_count != 0) {
    if (static_cast<int32_t>(millis() - g_rx_deadline_ms) > 0) {
      rx_reset();
    }
  }

  if (g_rx_ready && !g_tx_active) {
    g_rx_ready = false;
    handle_request(g_rx_json);
    rx_reset();
  }

  tx_pump();
  if (!g_tx_active) {
    subscribe_poll();
    tx_pump();
  }
}
