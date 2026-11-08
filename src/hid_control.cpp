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
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
bool g_sub_events = false;
bool g_sub_errors = false;
uint32_t g_last_metrics_ms = 0;
uint32_t g_last_beat_ms = 0;
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
  while (p && *p && isspace((unsigned char)*p)) {
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
  const char *v = json_find_key(json, key);
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
  const char *v = json_find_key(json, key);
  if (!v) {
    return false;
  }
  char *end = nullptr;
  long n = strtol(v, &end, 10);
  if (end == v) {
    return false;
  }
  *out = n;
  return true;
}

bool json_get_double(const char *json, const char *key, double *out) {
  const char *v = json_find_key(json, key);
  if (!v) {
    return false;
  }
  char *end = nullptr;
  double n = strtod(v, &end);
  if (end == v) {
    return false;
  }
  *out = n;
  return true;
}

bool json_get_bool(const char *json, const char *key, bool *out) {
  const char *v = json_find_key(json, key);
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
    while (v > 0 && n < (int)sizeof(tmp)) {
      tmp[n++] = (char)('0' + (v % 10ull));
      v /= 10ull;
    }
  }
  if ((size_t)n + 1 > outlen) {
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
  size_t n = strlen(json);
  if (n > kMaxJson) {
    n = kMaxJson;
  }
  memcpy(g_tx_json, json, n);
  g_tx_json[n] = '\0';
  g_tx_len = (uint16_t)n;
  g_tx_off = 0;
  g_tx_frag_index = 0;
  g_tx_frag_count = (uint8_t)((n + kMaxChunk - 1) / kMaxChunk);
  if (g_tx_frag_count == 0) {
    g_tx_frag_count = 1;
  }
  g_tx_seq = (uint8_t)(g_tx_seq + 1u);
  g_tx_notify = notify;
  g_tx_active = true;
}

void tx_pump(void) {
  if (!g_tx_active || !g_hid.ready()) {
    return;
  }
  uint8_t report[kPayloadBytes];
  memset(report, 0, sizeof(report));
  uint16_t remain = (g_tx_off < g_tx_len) ? (uint16_t)(g_tx_len - g_tx_off) : 0;
  uint8_t chunk = (remain > kMaxChunk) ? kMaxChunk : (uint8_t)remain;
  bool more = (g_tx_frag_index + 1u) < g_tx_frag_count;
  report[0] = (more ? kFlagMore : 0) | (g_tx_notify ? kFlagNotifyHint : 0);
  report[1] = g_tx_seq;
  report[2] = g_tx_frag_index;
  report[3] = g_tx_frag_count;
  report[4] = chunk;
  if (chunk) {
    memcpy(&report[5], &g_tx_json[g_tx_off], chunk);
    g_tx_off = (uint16_t)(g_tx_off + chunk);
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
  if (off < 0 || (size_t)off >= dstlen) {
    return -1;
  }
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(dst + off, dstlen - (size_t)off, fmt, ap);
  va_end(ap);
  if (n < 0 || (size_t)off + (size_t)n >= dstlen) {
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
      off = appendf(dst, dstlen, off, "%d", (int)lroundf(v));
    } else {
      off = appendf(dst, dstlen, off, "%.6g", (double)v);
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
    const BlockMap &b = kBlocks[i];
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
                 (double)m.pre_peak, (double)m.post_peak, (double)m.pre_rms,
                 (double)m.post_rms, (double)m.noise_floor,
                 (double)m.agc_gain_db, (double)m.match_score,
                 (unsigned long)m.clip_count, (unsigned long)m.limiter_count);
}

int encode_spectrum(char *dst, size_t dstlen, int off) {
  dsp_spectrum_t s{};
  control_api_get_spectrum(&s);
  off = appendf(dst, dstlen, off,
                "{\"valid\":%s,\"fftN\":%lu,\"sourceHz\":%lu,\"nPeaks\":%lu,"
                "\"peaks\":[",
                s.valid ? "true" : "false", (unsigned long)s.fft_n,
                (unsigned long)s.source_hz, (unsigned long)s.n_peaks);
  for (uint32_t i = 0; i < s.n_peaks && off >= 0; i++) {
    off = appendf(dst, dstlen, off, "%s{\"hz\":%.6g,\"mag\":%.6g}", i ? "," : "",
                  (double)s.peak_hz[i], (double)s.peak_mag[i]);
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
                 (double)m.score, (double)m.template_peak,
                 (unsigned long)m.template_len, m.learned ? "true" : "false");
}

int encode_beat(char *dst, size_t dstlen, int off) {
  dsp_beat_rate_t b{};
  control_api_get_beat_rate(&b);
  return appendf(dst, dstlen, off,
                 "{\"vphRaw\":%.6g,\"vphNearest\":%lu,\"confidence\":%.6g,"
                 "\"locked\":%s,\"intervalSamples\":%lu,\"eventsSeen\":%lu,"
                 "\"intervalsHeld\":%lu,\"sourceHz\":%lu}",
                 (double)b.vph_raw, (unsigned long)b.vph_nearest,
                 (double)b.confidence, b.locked ? "true" : "false",
                 (unsigned long)b.interval_samples,
                 (unsigned long)b.events_seen,
                 (unsigned long)b.intervals_held, (unsigned long)b.source_hz);
}

int encode_event(char *dst, size_t dstlen, int off, const dsp_event_t &ev) {
  char s0[24], s1[24];
  u64_to_dec(ev.start_sample, s0, sizeof(s0));
  u64_to_dec(ev.peak_sample, s1, sizeof(s1));
  return appendf(dst, dstlen, off,
                 "{\"startSample\":\"%s\",\"peakSample\":\"%s\","
                 "\"durationSamples\":%lu,\"riseSamples\":%lu,"
                 "\"peakAbs\":%.6g,\"noiseFloor\":%.6g,\"snrDb\":%.6g,"
                 "\"energy\":%.6g,\"confidence\":%.6g}",
                 s0, s1, (unsigned long)ev.duration_samples,
                 (unsigned long)ev.rise_samples, (double)ev.peak_abs,
                 (double)ev.noise_floor, (double)ev.snr_db, (double)ev.energy,
                 (double)ev.confidence);
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
  for (const BlockMap &b : kBlocks) {
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
                 (unsigned long)e.uptime_ms, si);
}

bool apply_block_params(dsp_block_id_t id, const char *obj) {
  double d;
  bool ok = true;
  auto setf = [&](const char *json_key, const char *cli_key) {
    if (json_get_double(obj, json_key, &d)) {
      if (!control_api_set_param(id, cli_key, (float)d)) {
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
  for (const BlockMap &b : kBlocks) {
    const char *bv = json_find_key(blocks_obj, b.json);
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
                  id, t, (unsigned long)millis());
  } else if (strcmp(op, "hello") == 0) {
    off = appendf(
        out, sizeof(out), 0,
        "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{"
        "\"protocolVersion\":1,\"fwVersion\":\"%s\",\"board\":\"%s\","
        "\"source\":\"%s\",\"vid\":%u,\"pid\":%u,\"caps\":{"
        "\"maxJsonBytes\":%u,\"metricsHzMax\":%u,\"beatHzMax\":%u,"
        "\"blocks\":[\"levelMeter\",\"gain\",\"limiter\",\"filterBank\","
        "\"eventDetect\",\"beatRate\",\"noiseFloor\",\"eventLog\","
        "\"softGate\",\"notch\",\"agc\",\"matchedFilter\",\"firBank\","
        "\"spectrum\"]}}}",
        id, FW_VERSION, BOARD_NAME, AUDIO_SOURCE_NAME, 0xCafeu, 0x4003u,
        (unsigned)kMaxJson, (unsigned)kMetricsHzMax, (unsigned)kBeatHzMax);
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
    uint32_t n = control_api_copy_events(ev, (uint32_t)limit);
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
    uint32_t n = control_api_copy_errors(err, 8);
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
      g_sub_metrics_hz = (uint8_t)hz;
    }
    if (json_get_int(json, "beatHz", &hz)) {
      if (hz < 0) {
        hz = 0;
      }
      if (hz > kBeatHzMax) {
        hz = kBeatHzMax;
      }
      g_sub_beat_hz = (uint8_t)hz;
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
    off = appendf(out, sizeof(out), 0,
                  "{\"v\":1,\"id\":%ld,\"ok\":true,\"result\":{"
                  "\"metricsHz\":%u,\"beatHz\":%u,\"events\":%s,\"errors\":%s}}",
                  id, (unsigned)g_sub_metrics_hz, (unsigned)g_sub_beat_hz,
                  g_sub_events ? "true" : "false",
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
  uint32_t n = control_api_copy_events(ev, 8);
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
  uint32_t n = control_api_copy_errors(err, 8);
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
  uint32_t now = millis();
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
  push_notify_events();
  push_notify_errors();
}

void on_host_report(uint8_t report_id, hid_report_type_t report_type,
                    uint8_t const *buffer, uint16_t bufsize) {
  (void)report_type;
  if (g_rx_ready) {
    return; /* drop while prior message pending */
  }
  const uint8_t *p = buffer;
  uint16_t n = bufsize;
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
  if ((uint16_t)frag_count * (uint16_t)kMaxChunk > kMaxJson) {
    rx_reset();
    return;
  }
  if (n < (uint16_t)(kHeaderLen + payload_len)) {
    return;
  }

  uint32_t now = millis();
  if (g_rx_deadline_ms != 0 && (int32_t)(now - g_rx_deadline_ms) > 0) {
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
    g_rx_seen[frag_index / 8] |= (uint8_t)(1u << (frag_index % 8));
  }

  uint16_t offset = (uint16_t)frag_index * kMaxChunk;
  if ((uint32_t)offset + payload_len > kMaxJson) {
    rx_reset();
    return;
  }
  memcpy(&g_rx_json[offset], &p[kHeaderLen], payload_len);
  if (offset + payload_len > g_rx_len) {
    g_rx_len = (uint16_t)(offset + payload_len);
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
    if ((int32_t)(millis() - g_rx_deadline_ms) > 0) {
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
