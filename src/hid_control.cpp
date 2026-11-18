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
#include "timebase.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <hardware/timer.h>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdarg>
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

uint8_t g_sub_metrics_hz = 0;
uint8_t g_sub_beat_hz = 0;
uint8_t g_sub_timing_hz = 0;
bool g_sub_events = false;
bool g_sub_errors = false;
uint32_t g_last_metrics_ms = 0;
uint32_t g_last_beat_ms = 0;
uint32_t g_last_timing_ms = 0;
uint64_t g_last_event_peak = 0;
bool g_have_last_event_peak = false;
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

bool json_get_int(const char *json, const char *key, long *out) {
  const auto *v = json_find_key(json, key);
  if (!v) {
    return false;
  }
  char *end = nullptr;
  auto n = strtol(v, &end, 10);
  if (end == v) {
    return false;
  }
  *out = n;
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

/** Copies {@code in} to {@code out}, dropping characters unsafe in a JSON string. */
void json_sanitize(const char *in, char *out, size_t outlen) {
  size_t i = 0;
  for (; in && *in && i + 1 < outlen; in++) {
    const auto c = static_cast<unsigned char>(*in);
    if (c >= 0x20 && c != '"' && c != '\\') {
      out[i++] = static_cast<char>(c);
    }
  }
  out[i] = '\0';
}

void send_error(long id, const char *code, const char *message) {
  char safe[128];
  json_sanitize(message, safe, sizeof(safe));
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"v\":1,\"id\":%ld,\"ok\":false,\"error\":{\"code\":\"%s\","
           "\"message\":\"%s\"}}",
           id, code, safe);
  tx_begin(buf, false);
}

int appendf(char *dst, size_t dstlen, int off, const char *fmt, ...) {
  if (off < 0 || static_cast<size_t>(off) >= dstlen) {
    return -1;
  }
  va_list ap;
  va_start(ap, fmt);
  auto n = vsnprintf(dst + off, dstlen - static_cast<size_t>(off), fmt, ap);
  va_end(ap);
  if (n < 0 || static_cast<size_t>(off) + static_cast<size_t>(n) >= dstlen) {
    return -1;
  }
  return off + n;
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
  return appendf(dst, dstlen, off, "}}");
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
      "\"underruns\":%lu,\"shortWrites\":%lu,\"ringMin\":%lu,\"ringMax\":%lu}",
      static_cast<unsigned long>(t.dsp_block_us_last), static_cast<unsigned long>(t.dsp_block_us_mean),
      static_cast<unsigned long>(t.dsp_block_us_max), static_cast<unsigned long>(t.dsp_block_budget_us),
      static_cast<unsigned long>(t.dsp_block_samples), static_cast<unsigned long>(t.dsp_blocks),
      static_cast<unsigned long>(t.loop_us_last), static_cast<unsigned long>(t.loop_us_mean),
      static_cast<unsigned long>(t.loop_us_max), static_cast<unsigned long>(t.loop_count),
      static_cast<unsigned long>(t.idle_us_per_s), static_cast<unsigned>(t.loop_busy_pct),
      static_cast<unsigned long>(t.underruns), static_cast<unsigned long>(t.short_writes),
      static_cast<unsigned long>(t.ring_min), static_cast<unsigned long>(t.ring_max));
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

int encode_event(char *dst, size_t dstlen, int off, const dsp_event_t &ev) {
  char s0[24], s1[24], s2[24], s3[24];
  u64_to_dec(ev.start_sample, s0, sizeof(s0));
  u64_to_dec(ev.peak_sample, s1, sizeof(s1));
  u64_to_dec(ev.unlock_sample, s2, sizeof(s2));
  u64_to_dec(ev.drop_sample, s3, sizeof(s3));
  return appendf(dst, dstlen, off,
                 "{\"startSample\":\"%s\",\"peakSample\":\"%s\","
                 "\"unlockSample\":\"%s\",\"dropSample\":\"%s\","
                 "\"durationSamples\":%lu,\"riseSamples\":%lu,"
                 "\"liftSamples\":%lu,"
                 "\"peakAbs\":%.6g,\"noiseFloor\":%.6g,\"snrDb\":%.6g,"
                 "\"energy\":%.6g,\"confidence\":%.6g}",
                 s0, s1, s2, s3, static_cast<unsigned long>(ev.duration_samples),
                 static_cast<unsigned long>(ev.rise_samples), static_cast<unsigned long>(ev.lift_samples),
                 static_cast<double>(ev.peak_abs), static_cast<double>(ev.noise_floor),
                 static_cast<double>(ev.snr_db), static_cast<double>(ev.energy), static_cast<double>(ev.confidence));
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

bool apply_patch(const char *patch_json) {
  bool bypass;
  if (json_get_bool(patch_json, "bypass", &bypass)) {
    control_api_set_bypass(bypass);
  }
  const char *blocks = json_find_key(patch_json, "blocks");
  if (!blocks) {
    return true;
  }
  char *blocks_obj = g_blocks_obj;
  if (!json_extract_object(blocks, blocks_obj, sizeof(g_blocks_obj))) {
    return false;
  }
  for (int i = 1; i < static_cast<int>(DSP_BLOCK_COUNT); i++) {
    const auto id = static_cast<dsp_block_id_t>(i);
    const auto *bv = json_find_key(blocks_obj, dsp_block_json_name(id));
    if (!bv) {
      continue;
    }
    char bobj[384];
    if (!json_extract_object(bv, bobj, sizeof(bobj))) {
      return false;
    }
    bool en;
    if (json_get_bool(bobj, "enabled", &en)) {
      if (!control_api_set_block_enabled(id, en)) {
        return false;
      }
    }
    const char *params = json_find_key(bobj, "params");
    if (params) {
      char pobj[256];
      if (json_extract_object(params, pobj, sizeof(pobj))) {
        if (!apply_block_params(id, pobj)) {
          return false;
        }
      }
    }
  }
  return true;
}

/* --- Request handlers: each appends the "result" value at {@code off}. --- */

using ResultEncoder = int (*)(const char *req, char *dst, size_t dstlen, int off);

int result_ping(const char *req, char *dst, size_t dstlen, int off) {
  long t = 0;
  json_get_int(req, "t", &t);
  return appendf(dst, dstlen, off, "{\"t\":%ld,\"uptimeMs\":%lu}", t,
                 static_cast<unsigned long>(millis()));
}

int result_hello(const char *, char *dst, size_t dstlen, int off) {
  off = appendf(dst, dstlen, off,
                "{\"protocolVersion\":1,\"fwVersion\":\"%s\",\"board\":\"%s\","
                "\"source\":\"%s\",\"vid\":%u,\"pid\":%u,\"caps\":{"
                "\"maxJsonBytes\":%u,\"metricsHzMax\":%u,\"beatHzMax\":%u,"
                "\"timingHzMax\":%u,\"timing\":true,\"blocks\":[",
                FW_VERSION, TG_BOARD_NAME, AUDIO_SOURCE_NAME,
                static_cast<unsigned>(USB_VENDOR_ID),
                static_cast<unsigned>(USB_PRODUCT_ID),
                static_cast<unsigned>(kMaxJson), static_cast<unsigned>(kMetricsHzMax),
                static_cast<unsigned>(kBeatHzMax), static_cast<unsigned>(kTimingHzMax));
  for (int i = 1; i < static_cast<int>(DSP_BLOCK_COUNT) && off >= 0; i++) {
    off = appendf(dst, dstlen, off, "%s\"%s\"", i > 1 ? "," : "",
                  dsp_block_json_name(static_cast<dsp_block_id_t>(i)));
  }
  return (off < 0) ? off : appendf(dst, dstlen, off, "]}}");
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
  off = appendf(dst, dstlen, off, "{\"events\":[");
  for (uint32_t i = 0; i < n && off >= 0; i++) {
    if (i) {
      off = appendf(dst, dstlen, off, ",");
    }
    if (off >= 0) {
      off = encode_event(dst, dstlen, off, g_ev_scratch[i]);
    }
  }
  return (off < 0) ? off : appendf(dst, dstlen, off, "]}");
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

int result_subscribe(const char *req, char *dst, size_t dstlen, int off) {
  read_rate(req, "metricsHz", kMetricsHzMax, &g_sub_metrics_hz);
  read_rate(req, "beatHz", kBeatHzMax, &g_sub_beat_hz);
  read_rate(req, "timingHz", kTimingHzMax, &g_sub_timing_hz);
  bool flag;
  if (json_get_bool(req, "events", &flag)) {
    g_sub_events = flag;
  }
  if (json_get_bool(req, "errors", &flag)) {
    g_sub_errors = flag;
  }
  /* Seed baselines so we don't dump history on subscribe. */
  dsp_event_t last_ev;
  g_have_last_event_peak = control_api_copy_events(&last_ev, 1) > 0;
  if (g_have_last_event_peak) {
    g_last_event_peak = last_ev.peak_sample;
  }
  dsp_error_t last_err;
  g_have_last_error = control_api_copy_errors(&last_err, 1) > 0;
  if (g_have_last_error) {
    g_last_error_uptime = last_err.uptime_ms;
  }
  const auto now = millis();
  g_last_metrics_ms = now;
  g_last_beat_ms = now;
  g_last_timing_ms = now;
  return appendf(dst, dstlen, off,
                 "{\"metricsHz\":%u,\"beatHz\":%u,\"timingHz\":%u,"
                 "\"events\":%s,\"errors\":%s}",
                 static_cast<unsigned>(g_sub_metrics_hz),
                 static_cast<unsigned>(g_sub_beat_hz),
                 static_cast<unsigned>(g_sub_timing_hz),
                 g_sub_events ? "true" : "false", g_sub_errors ? "true" : "false");
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
      "\"saved\":%s,\"pps\":{\"enabled\":%s,\"locked\":%s,\"ppm\":%.6g,"
      "\"window\":%lu,\"good\":%lu,\"rejects\":%lu,\"noFix\":%lu},"
      "\"gps\":{\"enabled\":%s,\"fix\":%s,\"quality\":%u,\"sats\":%u,"
      "\"hdop\":%.3g,\"utcUnix\":%lu,\"ageMs\":%ld,\"sentences\":%lu,"
      "\"errors\":%lu,\"overruns\":%lu}}",
      static_cast<double>(tb.ppm), timebase_source_name(tb.source),
      static_cast<double>(tb.cal_ppm), timebase_source_name(tb.cal_source),
      static_cast<unsigned long>(tb.cal_date_unix),
      static_cast<unsigned long>(tb.cal_duration_s), tb.cal_saved ? "true" : "false",
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
using RequestApplier = bool (*)(long id, const char *req);

bool apply_set_timebase(long id, const char *req) {
  double ppm = 0.0;
  if (!json_get_double(req, "ppm", &ppm)) {
    send_error(id, "badRequest", "missing ppm");
    return false;
  }
  timebase_source_t src = TIMEBASE_SRC_MANUAL;
  char name[16];
  if (json_get_string(req, "source", name, sizeof(name)) &&
      !timebase_source_from_name(name, &src)) {
    send_error(id, "badParam", "source must be manual or ntp");
    return false;
  }
  double date = 0.0, dur = 0.0;
  json_get_double(req, "dateUnix", &date);
  json_get_double(req, "durationS", &dur);
  if (date < 0.0 || date > 4294967295.0 || dur < 0.0 || dur > 4294967295.0 ||
      !timebase_set(static_cast<float>(ppm), src, static_cast<uint32_t>(date),
                    static_cast<uint32_t>(dur))) {
    send_error(id, "badParam", "ppm must be -1000..1000");
    return false;
  }
  return true;
}

bool apply_save_timebase(long id, const char *) {
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

bool apply_clear_timebase(long, const char *) {
  timebase_clear();
  return true;
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
};

/** Applies a patchState request; replies with an error and returns false on failure. */
bool apply_patch_request(long id, const char *json) {
  const char *patch = json_find_key(json, "patch");
  if (!patch || !json_extract_object(patch, g_patch_obj, sizeof(g_patch_obj))) {
    send_error(id, "badRequest", "missing patch object");
    return false;
  }
  if (!apply_patch(g_patch_obj)) {
    send_error(id, "badParam", "patch rejected");
    return false;
  }
  return true;
}

void handle_request(const char *json) {
  long id = 0;
  if (!json_get_int(json, "id", &id)) {
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

  int off = appendf(g_out, sizeof(g_out), 0,
                    "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":", id);
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

void push_notify_events(void) {
  if (g_tx_active || !g_sub_events) {
    return;
  }
  dsp_event_t *ev = g_ev_scratch;
  const auto n = control_api_copy_events(ev, 8);
  if (n == 0) {
    return;
  }
  if (!g_have_last_event_peak) {
    /* First observation after subscribe already seeded; nothing to push. */
    g_last_event_peak = ev[0].peak_sample;
    g_have_last_event_peak = true;
    return;
  }
  uint32_t neu = 0;
  while (neu < n && ev[neu].peak_sample != g_last_event_peak) {
    neu++;
  }
  if (neu == 0) {
    return;
  }
  g_last_event_peak = ev[0].peak_sample;
  int off = appendf(g_out, sizeof(g_out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"event\",\"data\":{"
                    "\"events\":[");
  for (uint32_t i = 0; i < neu && off >= 0; i++) {
    if (i) {
      off = appendf(g_out, sizeof(g_out), off, ",");
    }
    if (off >= 0) {
      off = encode_event(g_out, sizeof(g_out), off, ev[i]);
    }
  }
  if (off >= 0) {
    off = appendf(g_out, sizeof(g_out), off, "]}}");
  }
  if (off >= 0) {
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
