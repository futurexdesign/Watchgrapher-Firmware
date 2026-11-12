/**
 * @file hid_control.cpp
 * @brief Vendor HID framing + JSON RPC → control_api (WebHID).
 *
 * @see docs/HID_CONTROL_PLANE.md, hid_control.h, control_api.h
 */

#include "hid_control.h"

#include "board_config.h"
#include "control_api.h"
#include "dsp_types.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef FW_VERSION
#define FW_VERSION "0.1.0-dev"
#endif

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

struct BlockMap {
  const char *json;
  const char *cli;
  dsp_block_id_t id;
};

const BlockMap kBlocks[] = {
    {"levelMeter", "level_meter", DSP_BLOCK_LEVEL_METER},
    {"gain", "gain", DSP_BLOCK_GAIN},
    {"limiter", "limiter", DSP_BLOCK_LIMITER},
    {"filterBank", "filter_bank", DSP_BLOCK_FILTER_BANK},
    {"eventDetect", "event_detect", DSP_BLOCK_EVENT_DETECT},
    {"beatRate", "beat_rate", DSP_BLOCK_BEAT_RATE},
    {"noiseFloor", "noise_floor", DSP_BLOCK_NOISE_FLOOR},
    {"eventLog", "event_log", DSP_BLOCK_EVENT_LOG},
    {"softGate", "soft_gate", DSP_BLOCK_SOFT_GATE},
    {"notch", "notch", DSP_BLOCK_NOTCH},
    {"agc", "agc", DSP_BLOCK_AGC},
    {"matchedFilter", "matched_filter", DSP_BLOCK_MATCHED_FILTER},
    {"firBank", "fir_bank", DSP_BLOCK_FIR_BANK},
    {"spectrum", "spectrum", DSP_BLOCK_SPECTRUM},
};

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

void send_error(long id, const char *code, const char *message) {
  char buf[384];
  snprintf(buf, sizeof(buf),
           "{\"v\":1,\"id\":%ld,\"ok\":false,\"error\":{\"code\":\"%s\","
           "\"message\":\"%s\"}}",
           id, code, message);
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
  float v = 0.f;
  bool first = true;
  auto add = [&](const char *json_key, const char *cli_key) -> bool {
    if (!control_api_get_param(id, cli_key, &v)) {
      return true;
    }
    off = appendf(dst, dstlen, off, "%s\"%s\":", first ? "" : ",", json_key);
    first = false;
    if (off < 0) {
      return false;
    }
    /* Integers for preset/hz when whole. */
    if (fabsf(v - roundf(v)) < 1e-4f && fabsf(v) < 1e6f) {
      off = appendf(dst, dstlen, off, "%d", static_cast<int>(lroundf(v)));
    } else {
      off = appendf(dst, dstlen, off, "%.6g", static_cast<double>(v));
    }
    return off >= 0;
  };

  switch (id) {
  case DSP_BLOCK_GAIN:
    if (!add("db", "db")) {
      return -1;
    }
    break;
  case DSP_BLOCK_FILTER_BANK:
    if (!add("preset", "preset")) {
      return -1;
    }
    break;
  case DSP_BLOCK_EVENT_DETECT:
    if (!add("threshold", "threshold") || !add("prerollMs", "preroll_ms") ||
        !add("tailMs", "tail_ms")) {
      return -1;
    }
    break;
  case DSP_BLOCK_NOTCH:
    if (!add("hz", "hz")) {
      return -1;
    }
    break;
  case DSP_BLOCK_SOFT_GATE:
    if (!add("floorDb", "floor_db") || !add("attackMs", "attack_ms") ||
        !add("releaseMs", "release_ms")) {
      return -1;
    }
    break;
  case DSP_BLOCK_AGC:
    if (!add("target", "target") || !add("tauS", "tau_s") ||
        !add("gainDb", "gain_db")) {
      return -1;
    }
    break;
  case DSP_BLOCK_FIR_BANK:
    if (!add("preset", "preset")) {
      return -1;
    }
    break;
  case DSP_BLOCK_MATCHED_FILTER:
    if (!add("threshold", "threshold") || !add("learn", "learn") ||
        !add("score", "score")) {
      return -1;
    }
    break;
  case DSP_BLOCK_SPECTRUM:
    if (!add("stride", "stride")) {
      return -1;
    }
    break;
  case DSP_BLOCK_BEAT_RATE:
    if (!add("liftAngle", "liftAngle") || !add("vph", "vph")) {
      return -1;
    }
    break;
  default:
    break;
  }
  return off;
}

int encode_state(char *dst, size_t dstlen, int off) {
  off = appendf(dst, dstlen, off, "{\"bypass\":%s,\"blocks\":{",
                control_api_get_bypass() ? "true" : "false");
  if (off < 0) {
    return -1;
  }
  for (size_t i = 0; i < sizeof(kBlocks) / sizeof(kBlocks[0]); i++) {
    const auto &b = kBlocks[i];
    off = appendf(dst, dstlen, off, "%s\"%s\":{\"enabled\":%s,\"params\":{",
                  i ? "," : "", b.json,
                  control_api_block_enabled(b.id) ? "true" : "false");
    if (off < 0) {
      return -1;
    }
    off = append_params_for_block(dst, dstlen, off, b.id);
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
      "\"intervalsHeld\":%lu,\"sourceHz\":%lu,"
      "\"rateSDay\":%.6g,\"beatErrorMs\":%.6g,\"amplitudeDeg\":%.6g,"
      "\"liftAngleDeg\":%.6g,\"liftTimeMs\":%.6g,"
      "\"rateValid\":%s,\"beatErrorValid\":%s,\"amplitudeValid\":%s,"
      "\"instant\":",
      static_cast<double>(b.vph_raw), static_cast<unsigned long>(b.vph_nearest), static_cast<double>(b.confidence),
      b.locked ? "true" : "false", static_cast<unsigned long>(b.interval_samples),
      static_cast<unsigned long>(b.events_seen), static_cast<unsigned long>(b.intervals_held),
      static_cast<unsigned long>(b.source_hz), static_cast<double>(b.rate_s_day),
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
  off = appendf(dst, dstlen, off, ",\"avg10s\":");
  if (off < 0) {
    return -1;
  }
  off = encode_timing_window(dst, dstlen, off, b.avg_10s);
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

const char *block_json_name(dsp_block_id_t id) {
  for (const auto &b : kBlocks) {
    if (b.id == id) {
      return b.json;
    }
  }
  if (id == DSP_BLOCK_BYPASS) {
    return "bypass";
  }
  return "unknown";
}

int encode_error(char *dst, size_t dstlen, int off, const dsp_error_t &e) {
  char si[24];
  u64_to_dec(e.sample_index, si, sizeof(si));
  return appendf(dst, dstlen, off,
                 "{\"block\":\"%s\",\"reason\":\"%s\",\"uptimeMs\":%lu,"
                 "\"sampleIndex\":\"%s\"}",
                 block_json_name(e.block), reason_str(e.reason),
                 static_cast<unsigned long>(e.uptime_ms), si);
}

bool apply_block_params(dsp_block_id_t id, const char *obj) {
  double d;
  bool ok = true;
  auto setf = [&](const char *json_key, const char *cli_key) {
    if (json_get_double(obj, json_key, &d)) {
      if (!control_api_set_param(id, cli_key, static_cast<float>(d))) {
        ok = false;
      }
    }
  };
  switch (id) {
  case DSP_BLOCK_GAIN:
    setf("db", "db");
    break;
  case DSP_BLOCK_FILTER_BANK:
    setf("preset", "preset");
    break;
  case DSP_BLOCK_EVENT_DETECT:
    setf("threshold", "threshold");
    setf("prerollMs", "preroll_ms");
    setf("tailMs", "tail_ms");
    break;
  case DSP_BLOCK_NOTCH:
    setf("hz", "hz");
    break;
  case DSP_BLOCK_SOFT_GATE:
    setf("floorDb", "floor_db");
    setf("attackMs", "attack_ms");
    setf("releaseMs", "release_ms");
    break;
  case DSP_BLOCK_AGC:
    setf("target", "target");
    setf("tauS", "tau_s");
    /* gainDb is read-only */
    break;
  case DSP_BLOCK_FIR_BANK:
    setf("preset", "preset");
    break;
  case DSP_BLOCK_MATCHED_FILTER:
    setf("threshold", "threshold");
    setf("learn", "learn");
    break;
  case DSP_BLOCK_SPECTRUM:
    setf("stride", "stride");
    break;
  case DSP_BLOCK_BEAT_RATE:
    setf("liftAngle", "liftAngle");
    setf("vph", "vph");
    break;
  default:
    break;
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
  char blocks_obj[1024];
  if (!json_extract_object(blocks, blocks_obj, sizeof(blocks_obj))) {
    /* blocks value may already be object start */
    return false;
  }
  for (const auto &b : kBlocks) {
    const auto *bv = json_find_key(blocks_obj, b.json);
    if (!bv) {
      continue;
    }
    char bobj[384];
    if (!json_extract_object(bv, bobj, sizeof(bobj))) {
      return false;
    }
    bool en;
    if (json_get_bool(bobj, "enabled", &en)) {
      if (!control_api_set_block_enabled(b.id, en)) {
        return false;
      }
    }
    const char *params = json_find_key(bobj, "params");
    if (params) {
      char pobj[256];
      if (json_extract_object(params, pobj, sizeof(pobj))) {
        if (!apply_block_params(b.id, pobj)) {
          return false;
        }
      }
    }
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

  char out[kMaxJson];
  int off = 0;

  if (strcmp(op, "ping") == 0) {
    long t = 0;
    json_get_int(json, "t", &t);
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{\"t\":%ld,"
                  "\"uptimeMs\":%lu}}",
                  id, t, static_cast<unsigned long>(millis()));
  } else if (strcmp(op, "hello") == 0) {
    off = appendf(
        out, sizeof(out), 0,
        "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{"
        "\"protocolVersion\":1,\"fwVersion\":\"%s\",\"board\":\"%s\","
        "\"source\":\"%s\",\"vid\":%u,\"pid\":%u,\"caps\":{"
        "\"maxJsonBytes\":%u,\"metricsHzMax\":%u,\"beatHzMax\":%u,"
        "\"timingHzMax\":%u,\"timing\":true,"
        "\"blocks\":[\"levelMeter\",\"gain\",\"limiter\",\"filterBank\","
        "\"eventDetect\",\"beatRate\",\"noiseFloor\",\"eventLog\","
        "\"softGate\",\"notch\",\"agc\",\"matchedFilter\",\"firBank\","
        "\"spectrum\"]}}}",
        id, FW_VERSION, BOARD_NAME, AUDIO_SOURCE_NAME, 0xCafeu, 0x4003u,
        static_cast<unsigned>(kMaxJson), static_cast<unsigned>(kMetricsHzMax), static_cast<unsigned>(kBeatHzMax),
        static_cast<unsigned>(kTimingHzMax));
  } else if (strcmp(op, "getState") == 0) {
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_state(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "patchState") == 0) {
    const char *patch = json_find_key(json, "patch");
    char patch_obj[1200];
    if (!patch || !json_extract_object(patch, patch_obj, sizeof(patch_obj))) {
      send_error(id, "badRequest", "missing patch object");
      return;
    }
    if (!apply_patch(patch_obj)) {
      send_error(id, "badParam", "patch rejected");
      return;
    }
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_state(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "getMetrics") == 0) {
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_metrics(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "getTiming") == 0) {
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_timing(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "resetTiming") == 0) {
    control_api_reset_timing();
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{\"cleared\":true}}",
                  id);
  } else if (strcmp(op, "getBeat") == 0) {
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_beat(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "getSpectrum") == 0) {
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_spectrum(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "getMatched") == 0) {
    off = appendf(out, sizeof(out), 0, "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":",
                  id);
    if (off >= 0) {
      off = encode_matched(out, sizeof(out), off);
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "}");
    }
  } else if (strcmp(op, "getEvents") == 0) {
    long limit = 8;
    json_get_int(json, "limit", &limit);
    if (limit < 1) {
      limit = 1;
    }
    if (limit > 32) {
      limit = 32;
    }
    dsp_event_t ev[32];
    auto n = control_api_copy_events(ev, static_cast<uint32_t>(limit));
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{\"events\":[", id);
    for (uint32_t i = 0; i < n && off >= 0; i++) {
      if (i) {
        off = appendf(out, sizeof(out), off, ",");
      }
      if (off >= 0) {
        off = encode_event(out, sizeof(out), off, ev[i]);
      }
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "]}}");
    }
  } else if (strcmp(op, "getErrors") == 0) {
    dsp_error_t err[8];
    auto n = control_api_copy_errors(err, 8);
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{\"errors\":[", id);
    for (uint32_t i = 0; i < n && off >= 0; i++) {
      if (i) {
        off = appendf(out, sizeof(out), off, ",");
      }
      if (off >= 0) {
        off = encode_error(out, sizeof(out), off, err[i]);
      }
    }
    if (off >= 0) {
      off = appendf(out, sizeof(out), off, "]}}");
    }
  } else if (strcmp(op, "clearErrors") == 0) {
    control_api_clear_errors();
    g_have_last_error = false;
    g_last_error_uptime = 0;
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{\"cleared\":true}}",
                  id);
  } else if (strcmp(op, "subscribe") == 0) {
    long hz;
    bool flag;
    if (json_get_int(json, "metricsHz", &hz)) {
      if (hz < 0) {
        hz = 0;
      }
      if (hz > kMetricsHzMax) {
        hz = kMetricsHzMax;
      }
      g_sub_metrics_hz = static_cast<uint8_t>(hz);
    }
    if (json_get_int(json, "beatHz", &hz)) {
      if (hz < 0) {
        hz = 0;
      }
      if (hz > kBeatHzMax) {
        hz = kBeatHzMax;
      }
      g_sub_beat_hz = static_cast<uint8_t>(hz);
    }
    if (json_get_int(json, "timingHz", &hz)) {
      if (hz < 0) {
        hz = 0;
      }
      if (hz > kTimingHzMax) {
        hz = kTimingHzMax;
      }
      g_sub_timing_hz = static_cast<uint8_t>(hz);
    }
    if (json_get_bool(json, "events", &flag)) {
      g_sub_events = flag;
    }
    if (json_get_bool(json, "errors", &flag)) {
      g_sub_errors = flag;
    }
    /* Seed baselines so we don't dump history on subscribe. */
    {
      dsp_event_t dump[1];
      if (control_api_copy_events(dump, 1) > 0) {
        g_last_event_peak = dump[0].peak_sample;
        g_have_last_event_peak = true;
      } else {
        g_have_last_event_peak = false;
      }
    }
    {
      dsp_error_t dump[1];
      if (control_api_copy_errors(dump, 1) > 0) {
        g_last_error_uptime = dump[0].uptime_ms;
        g_have_last_error = true;
      } else {
        g_have_last_error = false;
      }
    }
    g_last_metrics_ms = millis();
    g_last_beat_ms = millis();
    g_last_timing_ms = millis();
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{"
                  "\"metricsHz\":%u,\"beatHz\":%u,\"timingHz\":%u,"
                  "\"events\":%s,\"errors\":%s}}",
                  id, static_cast<unsigned>(g_sub_metrics_hz), static_cast<unsigned>(g_sub_beat_hz),
                  static_cast<unsigned>(g_sub_timing_hz), g_sub_events ? "true" : "false",
                  g_sub_errors ? "true" : "false");
  } else {
    send_error(id, "unknownOp", op);
    return;
  }

  if (off < 0) {
    send_error(id, "tooLarge", "response exceeds buffer");
    return;
  }
  tx_begin(out, false);
}

void push_notify_metrics(void) {
  if (g_tx_active) {
    return;
  }
  char out[512];
  int off = appendf(out, sizeof(out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"metrics\",\"data\":");
  if (off >= 0) {
    off = encode_metrics(out, sizeof(out), off);
  }
  if (off >= 0) {
    off = appendf(out, sizeof(out), off, "}");
  }
  if (off >= 0) {
    tx_begin(out, true);
  }
}

void push_notify_timing(void) {
  if (g_tx_active) {
    return;
  }
  char out[640];
  int off = appendf(out, sizeof(out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"timing\",\"data\":");
  if (off >= 0) {
    off = encode_timing(out, sizeof(out), off);
  }
  if (off >= 0) {
    off = appendf(out, sizeof(out), off, "}");
  }
  if (off >= 0) {
    tx_begin(out, true);
  }
}

void push_notify_beat(void) {
  if (g_tx_active) {
    return;
  }
  char out[512];
  int off = appendf(out, sizeof(out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"beat\",\"data\":");
  if (off >= 0) {
    off = encode_beat(out, sizeof(out), off);
  }
  if (off >= 0) {
    off = appendf(out, sizeof(out), off, "}");
  }
  if (off >= 0) {
    tx_begin(out, true);
  }
}

void push_notify_events(void) {
  if (g_tx_active || !g_sub_events) {
    return;
  }
  dsp_event_t ev[8];
  auto n = control_api_copy_events(ev, 8);
  if (n == 0) {
    return;
  }
  uint32_t neu = 0;
  if (!g_have_last_event_peak) {
    /* First observation after subscribe already seeded; nothing to push. */
    g_last_event_peak = ev[0].peak_sample;
    g_have_last_event_peak = true;
    return;
  }
  for (uint32_t i = 0; i < n; i++) {
    if (ev[i].peak_sample == g_last_event_peak) {
      break;
    }
    neu++;
  }
  if (neu == 0) {
    return;
  }
  g_last_event_peak = ev[0].peak_sample;
  char out[kMaxJson];
  int off = appendf(out, sizeof(out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"event\",\"data\":{"
                    "\"events\":[");
  for (uint32_t i = 0; i < neu && off >= 0; i++) {
    if (i) {
      off = appendf(out, sizeof(out), off, ",");
    }
    if (off >= 0) {
      off = encode_event(out, sizeof(out), off, ev[i]);
    }
  }
  if (off >= 0) {
    off = appendf(out, sizeof(out), off, "]}}");
  }
  if (off >= 0) {
    tx_begin(out, true);
  }
}

void push_notify_errors(void) {
  if (g_tx_active || !g_sub_errors) {
    return;
  }
  dsp_error_t err[8];
  auto n = control_api_copy_errors(err, 8);
  if (n == 0) {
    return;
  }
  if (g_have_last_error && err[0].uptime_ms == g_last_error_uptime) {
    return;
  }
  if (!g_have_last_error) {
    g_last_error_uptime = err[0].uptime_ms;
    g_have_last_error = true;
    return;
  }
  g_last_error_uptime = err[0].uptime_ms;
  char out[384];
  int off = appendf(out, sizeof(out), 0,
                    "{\"v\":1,\"notify\":true,\"op\":\"error\",\"data\":");
  if (off >= 0) {
    off = encode_error(out, sizeof(out), off, err[0]);
  }
  if (off >= 0) {
    off = appendf(out, sizeof(out), off, "}");
  }
  if (off >= 0) {
    tx_begin(out, true);
  }
}

void subscribe_poll(void) {
  auto now = millis();
  if (g_sub_metrics_hz > 0) {
    uint32_t period = 1000u / g_sub_metrics_hz;
    if ((now - g_last_metrics_ms) >= period) {
      g_last_metrics_ms = now;
      push_notify_metrics();
    }
  }
  if (g_sub_beat_hz > 0 && !g_tx_active) {
    uint32_t period = 1000u / g_sub_beat_hz;
    if ((now - g_last_beat_ms) >= period) {
      g_last_beat_ms = now;
      push_notify_beat();
    }
  }
  if (g_sub_timing_hz > 0 && !g_tx_active) {
    uint32_t period = 1000u / g_sub_timing_hz;
    if ((now - g_last_timing_ms) >= period) {
      g_last_timing_ms = now;
      push_notify_timing();
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
