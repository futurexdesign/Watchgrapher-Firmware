/**
 * @file test_hid_json.cpp
 * @brief Host test for the HID JSON encoders ({@link hid_json}): the
 *        device / beats / pipeline / GPS / timebase-log payloads and the
 *        hello size, then the quartz ones:
 *        a populated {@code dsp_quartz_t} in every mode, and the pulse list,
 *        must encode to valid JSON that fits the 2048-byte HID message
 *        limit together with the response / notify envelope, also with
 *        every field at its longest printable value.
 *
 * The quartz name helpers of control_api forward to the real
 * {@code quartz::engine::*_name()} (see the shim below), so the test links
 * src/quartz/quartz_engine.cpp, the quartz library and CMSIS-DSP.
 *
 * Build and run (from the repository root), with the other quartz tests:
 *   tests/build_quartz_tests.sh [BUILD_DIR]
 */

#include "control_api.h"
#include "hid_json.h"
#include "quartz/quartz_engine.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

/* --- control_api name helpers -------------------------------------------
 * The real control_api.cpp pulls in dsp_graph and other firmware-only code,
 * so these forward to the quartz engine exactly as control_api.cpp does.
 * The names therefore come from quartz_engine.cpp itself, not from copies.
 * --------------------------------------------------------------------- */

extern "C" {

const char *control_api_quartz_mode_name(dsp_quartz_mode_t mode) {
  return quartz::engine::mode_name(mode);
}

const char *control_api_quartz_state_name(dsp_quartz_state_t state) {
  return quartz::engine::state_name(state);
}

const char *control_api_quartz_inhibition_name(dsp_quartz_inhibition_t s) {
  return quartz::engine::inhibition_name(s);
}

const char *control_api_quartz_ts_method_name(dsp_quartz_ts_method_t m) {
  return quartz::engine::ts_method_name(m);
}

const char *control_api_quartz_lcd_status_name(dsp_quartz_lcd_status_t s) {
  return quartz::engine::lcd_status_name(s);
}

} // extern "C"

namespace {

/** HID message limit (kMaxJson in hid_control.cpp, §3.1). */
constexpr std::size_t kMaxJson = 2048u;

/** Scratch for one encoded beat. */
constexpr std::size_t kBeatJsonMaxBuf = 512u;

/** Longest response envelope around a result: id is a signed long. */
constexpr const char *kResponseHead =
    "{\"v\":1,\"id\":-9223372036854775807,\"ok\":true,\"result\":";
/** Longest notify envelope around quartz data. */
constexpr const char *kNotifyHead =
    "{\"v\":1,\"notify\":true,\"op\":\"quartzPulse\",\"data\":";

int g_failures = 0; /**< Failed checks. */
int g_checks = 0;   /**< Checks run. */

/** Records one check and prints it when it fails. */
void check(bool ok, const char *what) {
  g_checks++;
  if (!ok) {
    g_failures++;
    std::printf("FAIL: %s\n", what);
  }
}

/** Notify envelope around beats data. */
constexpr const char *kBeatsNotifyHead =
    "{\"v\":1,\"notify\":true,\"op\":\"beats\",\"data\":";

/**
 * Minimal strict JSON (RFC 8259) recogniser: enough to prove the encoder
 * output parses. No numbers like NaN / inf, no trailing commas, no
 * unescaped control characters.
 */
class JsonChecker {
public:
  /** @param s NUL-terminated text. */
  explicit JsonChecker(const char *s) : p_(s) {}

  /** @return {@code true} if the whole text is exactly one JSON value. */
  bool valid() {
    ws();
    if (!value()) {
      return false;
    }
    ws();
    return *p_ == '\0';
  }

private:
  const char *p_;

  void ws() {
    while (*p_ == ' ' || *p_ == '\t' || *p_ == '\n' || *p_ == '\r') {
      p_++;
    }
  }
  bool lit(const char *w) {
    const auto n = std::strlen(w);
    if (std::strncmp(p_, w, n) != 0) {
      return false;
    }
    p_ += n;
    return true;
  }
  bool digits() {
    if (!std::isdigit(static_cast<unsigned char>(*p_))) {
      return false;
    }
    while (std::isdigit(static_cast<unsigned char>(*p_))) {
      p_++;
    }
    return true;
  }
  bool number() {
    if (*p_ == '-') {
      p_++;
    }
    if (*p_ == '0') {
      p_++;
    } else if (!digits()) {
      return false;
    }
    if (*p_ == '.') {
      p_++;
      if (!digits()) {
        return false;
      }
    }
    if (*p_ == 'e' || *p_ == 'E') {
      p_++;
      if (*p_ == '+' || *p_ == '-') {
        p_++;
      }
      if (!digits()) {
        return false;
      }
    }
    return true;
  }
  bool string() {
    if (*p_ != '"') {
      return false;
    }
    p_++;
    while (*p_ != '"') {
      const auto c = static_cast<unsigned char>(*p_);
      if (c == '\0' || c < 0x20) {
        return false;
      }
      if (c == '\\') {
        p_++;
        if (std::strchr("\"\\/bfnrtu", *p_) == nullptr || *p_ == '\0') {
          return false;
        }
      }
      p_++;
    }
    p_++;
    return true;
  }
  bool members(char close, bool keyed) {
    p_++;
    ws();
    if (*p_ == close) {
      p_++;
      return true;
    }
    for (;;) {
      ws();
      if (keyed) {
        if (!string()) {
          return false;
        }
        ws();
        if (*p_++ != ':') {
          return false;
        }
        ws();
      }
      if (!value()) {
        return false;
      }
      ws();
      if (*p_ == ',') {
        p_++;
        continue;
      }
      if (*p_ == close) {
        p_++;
        return true;
      }
      return false;
    }
  }
  bool value() {
    switch (*p_) {
    case '{':
      return members('}', true);
    case '[':
      return members(']', false);
    case '"':
      return string();
    case 't':
      return lit("true");
    case 'f':
      return lit("false");
    case 'n':
      return lit("null");
    default:
      return number();
    }
  }
};

/** @return Whether {@code s} is valid JSON. */
bool is_json(const std::string &s) { return JsonChecker(s.c_str()).valid(); }

/** A 95-character reason: the longest text the encoder copies. */
const std::string kLongReason(hid_json::kQuartzTextMax, 'r');

/**
 * @return A snapshot with every field at its longest printable value in
 *         {@code mode}: largest counters, 12-character floats, the longest
 *         names and the longest reason.
 */
dsp_quartz_t worst_case(dsp_quartz_mode_t mode) {
  constexpr float kF = -1.23456789e-30f; /* prints as "-1.23457e-30" */
  constexpr uint32_t kU = UINT32_MAX;
  dsp_quartz_t q{};
  q.mode = mode;
  q.state = DSP_QUARTZ_STATE_UNSUPPORTED_RATE;
  q.analysis = false; /* "off" is the longer literal */
  q.reason = kLongReason.c_str();
  q.source_hz = kU;
  q.timebase_ok = false;
  q.elapsed_s = kF;
  q.gate_s = kF;
  q.fork_hz = kF;
  q.ts_method = DSP_QUARTZ_TS_THRESHOLD;
  q.k_sigma = kF;
  q.sample_loss_restarts = kU;
  q.rate_s_day = kF;
  q.rate_valid = false;
  q.cal_ppm = kF;
  q.cal_source = "manual";
  q.calibrated = false;
  q.measurement_id = kU;
  /* Named fields (booleans false = the longer literal), so fields appended
   * to dsp_quartz_motor_t later cannot shift these values. */
  auto &m = q.motor;
  m.valid = false;
  m.pulses = m.fit_pulses = kU;
  m.step_s = m.span_s = kF;
  m.rate_with_s_day = m.rate_with_uncal_s_day = kF;
  m.rate_without_s_day = m.rate_without_uncal_s_day = kF;
  m.inhibition = DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION;
  m.inhibition_period_valid = false;
  m.inhibition_period_s = kF;
  m.events = m.cycles_used = kU;
  m.jump_last_us = m.jump_mean_us = kF;
  m.jump_last_n = m.jump_mean_n = kU;
  m.residual_rms_us = m.polarity_offset_us = kF;
  m.clipped = false;
  m.clipped_pulses = m.extra_pulses = kU;
  m.noise_sigma = m.threshold = m.peak_snr = kF;
  m.hint_weak_signal = m.hint_clipped = false;
  m.no_jump_span_s = m.jump_threshold_us = kF;
  m.rebuilding = false;
  auto &l = q.lcd;
  l.valid = false;
  l.status = DSP_QUARTZ_LCD_DETECTING_PERIOD;
  l.f0_hz = kU;
  l.harmonics[0] = l.harmonics[1] = l.harmonics[2] = l.harmonics[3] =
      UINT16_MAX;
  l.harmonic_count = 255; /* clamped to DSP_QUARTZ_MAX_HARMONICS */
  l.blocks = kU;
  l.rate_s_day = l.rate_uncal_s_day = kF;
  l.retries = kU;
  auto &f = q.fork;
  f.valid = false;
  f.f_nom_hz = kF;
  f.blocks = kU;
  f.rate_s_day = f.rate_uncal_s_day = kF;
  f.clipped = false;
  return q;
}

/** @return A pulse with every field at its longest printable value. */
dsp_quartz_pulse_t worst_pulse() {
  dsp_quartz_pulse_t p{};
  p.sequence = UINT32_MAX;
  p.sample_index = UINT64_MAX;
  /* Largest possible t_s: the whole uint64 index range at 48 kHz. */
  p.t_s = -static_cast<double>(UINT64_MAX) / 48000.0;
  p.peak = -1.23456789e-30f;
  p.snr = -1.23456789e-30f;
  p.polarity = -1;
  p.extras = 255;
  p.clipped = false;
  p.closed = false;
  p.measurement_id = UINT32_MAX;
  return p;
}

/** Encodes {@code q} and checks validity, the size bound and the mode
 * object layout. @return The encoded length. */
std::size_t check_quartz(const dsp_quartz_t &q, const char *what) {
  char buf[kMaxJson];
  const int n = hid_json::encode_quartz(buf, sizeof(buf), 0, q);
  check(n > 0, what);
  if (n <= 0) {
    return 0;
  }
  const std::string s(buf, static_cast<std::size_t>(n));
  check(is_json(s), what);
  const auto full = std::strlen(kResponseHead) + s.size() + 1u;
  check(full < kMaxJson, what);
  const char *names[] = {"\"motor\":", "\"lcd\":", "\"fork\":"};
  for (int i = 0; i < 3; i++) {
    const auto at = s.find(names[i]);
    const bool active = q.mode == static_cast<dsp_quartz_mode_t>(i + 1);
    check(at != std::string::npos, what);
    if (at != std::string::npos) {
      const char next = s[at + std::strlen(names[i])];
      check(active ? next == '{' : next == 'n', what);
    }
  }
  return full;
}

/** A realistic motor snapshot (ind_inhibition_60s, corrected). */
dsp_quartz_t realistic_motor() {
  dsp_quartz_t q{};
  q.mode = DSP_QUARTZ_MODE_MOTOR;
  q.state = DSP_QUARTZ_STATE_DONE;
  q.reason = "two inhibition events seen";
  q.source_hz = 48000;
  q.timebase_ok = true;
  q.elapsed_s = 121.5f;
  q.gate_s = 0.f;
  q.fork_hz = 360.f;
  q.k_sigma = 8.f;
  q.rate_s_day = 0.2059f;
  q.rate_valid = true;
  q.cal_ppm = -1.25f;
  q.cal_source = "pps";
  q.calibrated = true;
  auto &m = q.motor;
  m.valid = true;
  m.pulses = 121;
  m.fit_pulses = 121;
  m.step_s = 1.f;
  m.span_s = 120.f;
  m.rate_with_s_day = 0.2059f;
  m.rate_without_s_day = 0.9022f;
  m.inhibition = DSP_QUARTZ_INHIB_CORRECTED;
  m.inhibition_period_valid = true;
  m.inhibition_period_s = 60.f;
  m.events = 2;
  m.cycles_used = 1;
  m.jump_last_us = 488.3f;
  m.jump_last_n = 4;
  m.peak_snr = 52.f;
  m.no_jump_span_s = 120.f;
  m.jump_threshold_us = 20.f;
  m.rebuilding = true;
  q.measurement_id = 7;
  q.analysis = true;
  return q;
}

} // namespace

/**
 * Writes {@code head}, then what {@code encode} appends, then the closing
 * {@code "}"} of the envelope, into a 2048-byte buffer (the real
 * {@code g_out}), exactly as hid_control.cpp assembles a message.
 *
 * @param head Envelope prefix.
 * @param encode Callable {@code int(char *dst, size_t dstlen, int off)}.
 * @param out Receives the message (NUL-terminated).
 * @return Message length, or -1 if it did not fit.
 */
template <typename F>
int enveloped(const char *head, F encode, std::array<char, kMaxJson> &out) {
  int off = hid_json::appendf(out.data(), out.size(), 0, "%s", head);
  off = encode(out.data(), out.size(), off);
  return hid_json::appendf(out.data(), out.size(), off, "}");
}

/** @return Whether {@code s} contains {@code needle}. */
bool has(const std::array<char, kMaxJson> &s, const char *needle) {
  return std::strstr(s.data(), needle) != nullptr;
}

/** @return {@code n} occurrences of {@code needle} in {@code s}. */
int count_of(const std::array<char, kMaxJson> &s, const char *needle) {
  int n = 0;
  for (const char *p = s.data(); (p = std::strstr(p, needle)) != nullptr;
       p += std::strlen(needle)) {
    n++;
  }
  return n;
}

/** @return A beat with realistic values, number {@code i}. */
dsp_beat_t realistic_beat(uint32_t i) {
  dsp_beat_t b{};
  b.seq = i;
  b.grid_id = 1;
  b.sample_index = 1234567ull + i * 8333ull;
  b.t_s = 25.7 + 0.1736 * i;
  b.k = static_cast<int32_t>(i);
  b.parity = static_cast<uint8_t>(i & 1u);
  b.u_us = -12.5f + static_cast<float>(i);
  b.nominal_us = 69444.4f;
  b.amplitude_deg = 287.4f;
  b.amplitude_valid = true;
  b.lift_ms = 2.345f;
  b.status = DSP_BEAT_ACCEPTED;
  return b;
}

/** @return A beat with every field at its longest printable value. */
dsp_beat_t worst_beat(uint32_t i) {
  dsp_beat_t b{};
  b.seq = UINT32_MAX - i;
  b.grid_id = UINT32_MAX;
  b.sample_index = UINT64_MAX;
  b.t_s = -1e300; /* clamped to -1e12 */
  b.k = INT32_MIN;
  b.parity = 1;
  b.u_us = -FLT_MAX;
  b.nominal_us = -FLT_MAX;
  b.amplitude_deg = -FLT_MAX;
  b.amplitude_valid = false; /* "false" is the longer literal */
  b.lift_ms = -FLT_MAX;
  b.status = DSP_BEAT_ACCEPTED; /* "accepted" is the longest name */
  return b;
}

void test_device_status() {
  device_status_t s{};
  s.mode = DEVICE_MODE_STANDALONE;
  s.running = true;
  s.run_requested = true;
  s.usb_streaming = false;
  s.stream_state = STREAM_STATE_USB_CONFIGURED_IDLE;
  s.saved = true;
  char buf[256];
  int n = hid_json::encode_device_status(buf, sizeof(buf), 0, s);
  check(n > 0 && is_json(buf), "device status json");
  check(std::strcmp(buf, "{\"mode\":\"standalone\",\"running\":true,"
                         "\"runRequested\":true,\"usbStreaming\":false,"
                         "\"streamState\":\"idle\",\"saved\":true}") == 0,
        "device status fields");
  const char *names[] = {"unconfigured", "idle",      "prefill",
                         "active",       "suspended", "error"};
  for (int i = 0; i <= static_cast<int>(STREAM_STATE_STREAM_ERROR); i++) {
    s.stream_state = static_cast<stream_state_t>(i);
    s.mode = DEVICE_MODE_USB_MIC;
    n = hid_json::encode_device_status(buf, sizeof(buf), 0, s);
    const std::string want =
        std::string("\"streamState\":\"") + names[i] + "\",";
    check(n > 0 && std::strstr(buf, want.c_str()) != nullptr &&
              std::strstr(buf, "\"mode\":\"usbMic\"") != nullptr,
          "device stream state names");
  }
}

void test_beats() {
  char buf[kBeatJsonMaxBuf];
  const auto r = realistic_beat(3);
  int n = hid_json::encode_beat(buf, sizeof(buf), 0, r);
  check(n > 0 && is_json(buf), "realistic beat json");
  check(std::strcmp(buf, "{\"seq\":3,\"gridId\":1,\"sampleIndex\":\"1259566\","
                         "\"tS\":26.2208000,\"k\":3,\"parity\":1,\"uUs\":-9.5,"
                         "\"nominalUs\":69444.4,\"amplitudeDeg\":287.4,"
                         "\"amplitudeValid\":true,\"liftMs\":2.345,"
                         "\"status\":\"accepted\"}") == 0,
        "realistic beat fields");
  std::printf("beat realistic: %s\n", buf);

  const auto w = worst_beat(0);
  n = hid_json::encode_beat(buf, sizeof(buf), 0, w);
  check(n > 0 && is_json(buf), "worst beat json");
  check(static_cast<std::size_t>(n) <= hid_json::kBeatJsonMax,
        "beat within kBeatJsonMax");
  std::printf("beat worst case: %d bytes (bound %zu)\n", n,
              hid_json::kBeatJsonMax);
  check(std::strstr(buf, "\"tS\":-1000000000000.0000000") != nullptr,
        "tS clamped");

  /* Non-finite floats are 0. */
  dsp_beat_t nf = r;
  nf.u_us = NAN;
  nf.nominal_us = INFINITY;
  nf.t_s = -INFINITY;
  n = hid_json::encode_beat(buf, sizeof(buf), 0, nf);
  check(n > 0 && is_json(buf) && std::strstr(buf, "nan") == nullptr &&
            std::strstr(buf, "inf") == nullptr,
        "beat non-finite floats");

  /* Notify: 8 worst-case beats, oldest first. */
  std::array<char, kMaxJson> msg{};
  std::array<dsp_beat_t, hid_json::kBeatsMax> list{};
  for (uint32_t i = 0; i < list.size(); i++) {
    list[i] = worst_beat(15u - i);
  }
  uint32_t emitted = 0;
  int len = enveloped(
      kBeatsNotifyHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_beats(d, l, off, list.data(),
                                      hid_json::kBeatsPerNotifyMax, &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()), "worst beats notify json");
  check(emitted >= 1u && emitted <= hid_json::kBeatsPerNotifyMax,
        "worst beats notify count");
  std::printf("beats notify worst case: %u of %u beats, %d bytes\n", emitted,
              hid_json::kBeatsPerNotifyMax, len);

  /* Notify with realistic beats: all 8 fit. */
  for (uint32_t i = 0; i < list.size(); i++) {
    list[i] = realistic_beat(100000u + i);
  }
  len = enveloped(
      kBeatsNotifyHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_beats(d, l, off, list.data(),
                                      hid_json::kBeatsPerNotifyMax, &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()) && emitted == 8u, "8 beats notify");
  check(count_of(msg, "\"seq\":") == 8, "8 seq keys");
  std::printf("beats notify realistic: 8 beats, %d bytes\n", len);

  /* getBeats max 16: a prefix (newest first) that fits. */
  len = enveloped(
      kResponseHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_beats(d, l, off, list.data(),
                                      hid_json::kBeatsMax, &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()) && emitted == 8u,
        "getBeats realistic json / count");
  check(hid_json::kBeatsMax == 8u, "caps.beatsMax is 8");
  check(static_cast<std::size_t>(len) < kMaxJson, "getBeats fits 2048");
  check(has(msg, "\"seq\":100000,"), "getBeats keeps the leading beat");
  std::printf("getBeats realistic (8 requested): %u beats, %d bytes\n", emitted,
              len);
  for (uint32_t i = 0; i < list.size(); i++) {
    list[i] = worst_beat(i);
  }
  len = enveloped(
      kResponseHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_beats(d, l, off, list.data(),
                                      hid_json::kBeatsMax, &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()) && emitted >= 7u, "getBeats worst");
  std::printf("getBeats worst case (8 requested): %u beats, %d bytes\n",
              emitted, len);

  /* Empty list. */
  n = hid_json::encode_beats(buf, sizeof(buf), 0, nullptr, 0, &emitted);
  check(n > 0 && std::strcmp(buf, "{\"beats\":[]}") == 0 && emitted == 0u,
        "empty beats");
}

/** Notify envelope around event data. */
constexpr const char *kEventNotifyHead =
    "{\"v\":1,\"notify\":true,\"op\":\"event\",\"data\":";

/** Event with realistic small values; {@code peak} identifies it. */
dsp_event_t realistic_event(uint64_t peak) {
  dsp_event_t e{};
  e.start_sample = peak - 100u;
  e.peak_sample = peak;
  e.unlock_sample = peak - 20u;
  e.drop_sample = peak + 5u;
  e.duration_samples = 480u;
  e.rise_samples = 100u;
  e.lift_samples = 25u;
  e.peak_abs = 0.5f;
  e.noise_floor = 0.01f;
  e.snr_db = 34.0f;
  e.energy = 1.5f;
  e.confidence = 0.9f;
  return e;
}

/** Longest event: 20-digit indices, 12-character floats, maximal counts.
 * {@code id} is stamped into the low digits of {@code peak_sample}. */
dsp_event_t worst_event(uint64_t id) {
  dsp_event_t e{};
  e.start_sample = UINT64_MAX - 1000u;
  e.peak_sample = UINT64_MAX - 100u + id;
  e.unlock_sample = UINT64_MAX - 2000u;
  e.drop_sample = UINT64_MAX - 3000u;
  e.duration_samples = UINT32_MAX;
  e.rise_samples = UINT32_MAX;
  e.lift_samples = UINT32_MAX;
  e.peak_abs = -3.4028235e+38f;
  e.noise_floor = -3.4028235e+38f;
  e.snr_db = -1.17549435e-38f;
  e.energy = -3.4028235e+38f;
  e.confidence = -3.4028235e+38f;
  return e;
}

void test_events() {
  char buf[kBeatJsonMaxBuf];
  int n = hid_json::encode_event(buf, sizeof(buf), 0, realistic_event(1100u));
  check(n > 0 && is_json(buf), "realistic event json");
  check(n >= 200 && n < 260, "realistic event size");
  std::printf("event realistic: %d bytes\n", n);
  n = hid_json::encode_event(buf, sizeof(buf), 0, worst_event(0));
  check(n > 0 && is_json(buf), "worst event json");
  check(static_cast<std::size_t>(n) <= hid_json::kEventJsonMax,
        "event within kEventJsonMax");
  std::printf("event worst case: %d bytes (bound %zu)\n", n,
              hid_json::kEventJsonMax);

  /* getEvents: 32 extreme events requested, newest first. */
  std::array<char, kMaxJson> msg{};
  std::array<dsp_event_t, 32> list{};
  for (uint32_t i = 0; i < list.size(); i++) {
    list[i] = worst_event(31u - i); /* index 0 is the newest */
  }
  uint32_t emitted = 0;
  int len = enveloped(
      kResponseHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_events(d, l, off, list.data(),
                                       static_cast<uint32_t>(list.size()),
                                       &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()), "worst getEvents json");
  check(static_cast<std::size_t>(len) <= kMaxJson, "worst getEvents <= 2048");
  check(emitted >= 5u && emitted < 32u, "worst getEvents count");
  check(count_of(msg, "\"peakSample\"") == static_cast<int>(emitted),
        "worst getEvents count matches");
  check(has(msg, "\"peakSample\":\"18446744073709551546\""),
        "worst getEvents keeps the newest");
  std::printf("getEvents worst case (32 requested): %u events, %d bytes\n",
              emitted, len);

  /* Realistic events: a prefix fits, newest kept. */
  for (uint32_t i = 0; i < list.size(); i++) {
    list[i] = realistic_event(100000u - i);
  }
  len = enveloped(
      kResponseHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_events(d, l, off, list.data(), 32u, &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()) && emitted >= 8u && emitted < 32u,
        "realistic getEvents");
  check(has(msg, "\"peakSample\":\"100000\""), "realistic keeps newest");
  std::printf("getEvents realistic (32 requested): %u events, %d bytes\n",
              emitted, len);

  /* Notify: oldest-first prefix; the caller advances by `emitted`. */
  for (uint32_t i = 0; i < list.size(); i++) {
    list[i] = worst_event(i); /* oldest first */
  }
  len = enveloped(
      kEventNotifyHead,
      [&](char *d, std::size_t l, int off) {
        return hid_json::encode_events(d, l, off, list.data(), 32u, &emitted);
      },
      msg);
  check(len > 0 && is_json(msg.data()) &&
            static_cast<std::size_t>(len) <= kMaxJson,
        "worst event notify json");
  check(emitted >= 5u && emitted < 32u, "worst event notify count");
  check(has(msg, "\"peakSample\":\"18446744073709551515\""),
        "event notify starts with the oldest");
  check(!has(msg, "\"peakSample\":\"18446744073709551546\""),
        "event notify leaves the newest for the next poll");
  std::printf("event notify worst case: %u of 32 events, %d bytes\n", emitted,
              len);

  /* Non-finite floats are 0. */
  auto nf = realistic_event(10u);
  nf.peak_abs = NAN;
  nf.energy = INFINITY;
  n = hid_json::encode_event(buf, sizeof(buf), 0, nf);
  check(n > 0 && is_json(buf) && std::strstr(buf, "nan") == nullptr &&
            std::strstr(buf, "inf") == nullptr,
        "event non-finite floats");

  /* Empty list. */
  n = hid_json::encode_events(buf, sizeof(buf), 0, nullptr, 0, &emitted);
  check(n > 0 && std::strcmp(buf, "{\"events\":[]}") == 0 && emitted == 0u,
        "empty events");
}

void test_int64() {
  int64_t v = 0;
  check(hid_json::parse_i64("1731916800123,\"x\":1}", &v) &&
            v == 1731916800123LL,
        "parse Date.now()");
  check(hid_json::parse_i64("  -42}", &v) && v == -42, "parse negative");
  check(hid_json::parse_i64("9223372036854775807", &v) && v == INT64_MAX,
        "parse int64 max");
  check(!hid_json::parse_i64("9223372036854775808", &v), "reject overflow");
  check(!hid_json::parse_i64("abc", &v) && !hid_json::parse_i64("", &v) &&
            !hid_json::parse_i64(nullptr, &v),
        "reject non-numbers");

  char s[24];
  hid_json::i64_dec(INT64_MIN, s);
  check(std::strcmp(s, "-9223372036854775808") == 0, "i64_dec min");
  hid_json::i64_dec(0, s);
  check(std::strcmp(s, "0") == 0, "i64_dec zero");
  hid_json::i64_dec(1731916800123LL, s);
  check(std::strcmp(s, "1731916800123") == 0, "i64_dec Date.now()");

  char buf[64];
  const int n =
      hid_json::encode_ping(buf, sizeof(buf), 0, 1731916800123LL, 5000u);
  check(n > 0 &&
            std::strcmp(buf, "{\"t\":1731916800123,\"uptimeMs\":5000}") == 0,
        "ping echoes a 64-bit t");
}

timebase_gps_t realistic_gps(uint32_t n_sats) {
  timebase_gps_t g{};
  g.fix = true;
  g.quality = 1;
  g.utc_unix = 1700000000u;
  g.age_ms = 123;
  g.sats_used = 12;
  g.hdop = 0.9f;
  g.pdop = 1.5f;
  g.vdop = 1.2f;
  g.lat = 47.6062095;
  g.lon = -122.3320708;
  g.alt_m = 56.5f;
  g.pos_valid = true;
  g.n_sats = static_cast<uint8_t>(n_sats);
  for (uint32_t i = 0; i < n_sats; i++) {
    g.sats[i].sys = static_cast<uint8_t>(1 + i % 5);
    g.sats[i].prn = static_cast<uint16_t>(1 + i);
    g.sats[i].el = static_cast<int8_t>(10 + i);
    g.sats[i].az = static_cast<int16_t>(100 + 7 * i);
    g.sats[i].snr = static_cast<int8_t>(30 + i % 15);
    g.sats[i].used = i % 3 != 2;
  }
  return g;
}

void test_gps() {
  std::array<char, kMaxJson> msg{};
  auto enc = [](const timebase_gps_t &g) {
    return [&g](char *d, std::size_t l, int off) {
      return hid_json::encode_gps(d, l, off, g);
    };
  };

  auto g = realistic_gps(8);
  int len = enveloped(kResponseHead, enc(g), msg);
  check(len > 0 && is_json(msg.data()) && count_of(msg, "[\"") == 8 + 1,
        "gps 8 sats (+ satCols)");
  check(has(msg, "\"satCols\":[\"sys\",\"prn\",\"el\",\"az\",\"snr\","
                 "\"used\"],\"sats\":[[\"GP\",1,10,100,30,1],"),
        "gps sat tuples");
  check(has(msg, "\"lat\":47.6062095,\"lon\":-122.3320708,\"altM\":56.5,"),
        "gps position");
  std::printf("getGps 8 sats: %d bytes\n", len);

  /* Worst case: 32 satellites, every field at its longest. */
  timebase_gps_t w{};
  w.fix = false;
  w.quality = 255;
  w.utc_unix = UINT32_MAX;
  w.age_ms = INT32_MIN;
  w.sats_used = 255;
  w.hdop = -FLT_MAX;
  w.pdop = -FLT_MAX;
  w.vdop = -FLT_MAX;
  w.lat = -1e300;
  w.lon = -1e300;
  w.alt_m = -FLT_MAX;
  w.pos_valid = false;
  w.n_sats = 32;
  for (auto &s : w.sats) {
    s.sys = 4;
    s.prn = 65535;
    s.el = -128;
    s.az = -32768;
    s.snr = -128;
    s.used = false;
  }
  len = enveloped(kResponseHead, enc(w), msg);
  check(len > 0 && is_json(msg.data()), "gps worst json");
  check(len > 0 && static_cast<std::size_t>(len) <= kMaxJson,
        "gps worst case (32 sats) fits 2048 with envelope");
  check(count_of(msg, "[\"BD\"") == 32, "gps worst case keeps all 32 sats");
  std::printf("getGps worst case (32 sats): %d bytes with envelope\n", len);

  /* 32 realistic satellites all fit too. */
  g = realistic_gps(32);
  len = enveloped(kResponseHead, enc(g), msg);
  check(len > 0 && is_json(msg.data()) &&
            count_of(msg, "[\"G") + count_of(msg, "[\"B") +
                    count_of(msg, "[\"Q") ==
                32,
        "gps 32 realistic all kept");
  std::printf("getGps 32 realistic: %d bytes\n", len);

  /* No satellites, non-finite DOP. */
  g = realistic_gps(0);
  g.hdop = NAN;
  g.pdop = INFINITY;
  len = enveloped(kResponseHead, enc(g), msg);
  check(len > 0 && is_json(msg.data()) && has(msg, "\"sats\":[]") &&
            !has(msg, "nan") && !has(msg, "inf"),
        "gps empty / non-finite");

  /* An unknown system and a too-large n_sats are tolerated. */
  g = realistic_gps(2);
  g.sats[0].sys = 0;
  g.n_sats = 200;
  len = enveloped(kResponseHead, enc(g), msg);
  check(len > 0 && is_json(msg.data()) && has(msg, "[\"??\","),
        "gps unknown sys / clamped n_sats");
}

timebase_log_t make_log(uint32_t n, bool worst) {
  timebase_log_t log;
  timebase_log_init(&log);
  for (uint32_t i = 0; i < n; i++) {
    timebase_log_entry_t e{};
    if (worst) {
      e.date_unix = UINT32_MAX - (n - 1u - i);
      e.ppm = -1.23456789e-30f;
      e.source = TIMEBASE_SRC_MANUAL;
      e.duration_s = UINT32_MAX;
      e.reason = TIMEBASE_LOG_REASON_MONTHLY;
    } else {
      e.date_unix = 1700000000u + i * 86400u;
      e.ppm = -12.345f + 0.01f * static_cast<float>(i);
      e.source = TIMEBASE_SRC_PPS;
      e.duration_s = 3600u + i;
      e.reason = TIMEBASE_LOG_REASON_DELTA;
    }
    timebase_log_append(&log, &e);
  }
  return log;
}

void test_timebase_log() {
  std::array<char, kMaxJson> msg{};
  uint32_t emitted = 0;
  auto enc = [&emitted](const timebase_log_t &l, uint32_t offset, bool pending,
                        float delta, uint32_t last) {
    return [&l, offset, pending, delta, last, &emitted](char *d, std::size_t n,
                                                        int off) {
      return hid_json::encode_timebase_log(d, n, off, l, offset, pending, delta,
                                           last, &emitted);
    };
  };

  auto log = make_log(3, false);
  int len = enveloped(kResponseHead, enc(log, 0, true, 0.2f, 1700172800u), msg);
  check(len > 0 && is_json(msg.data()) && emitted == 3u &&
            count_of(msg, "\"dateUnix\":") == 3,
        "log 3 entries");
  check(has(msg,
            "\"entries\":[{\"dateUnix\":1700000000,\"ppm\":-12.345,"
            "\"source\":\"pps\",\"durationS\":3600,\"reason\":\"delta\"},"),
        "log entry fields, oldest first");
  check(has(msg, "],\"total\":3,\"offset\":0,\"pending\":true,"
                 "\"logDeltaPpm\":0.2,\"lastWriteUnix\":1700172800}"),
        "log trailer");

  /* Offset pages. */
  len = enveloped(kResponseHead, enc(log, 2, false, 0.2f, 0), msg);
  check(len > 0 && is_json(msg.data()) && emitted == 1u &&
            has(msg, "\"total\":3,\"offset\":2,"),
        "log page at offset 2");
  len = enveloped(kResponseHead, enc(log, 9, false, 0.2f, 0), msg);
  check(len > 0 && is_json(msg.data()) && emitted == 0u &&
            has(msg, "\"entries\":[],\"total\":3,\"offset\":3,"),
        "log offset past the end is an empty page");

  /* Empty log (clearTimebaseLog reply). */
  timebase_log_t empty;
  timebase_log_init(&empty);
  len = enveloped(kResponseHead, enc(empty, 0, false, 0.2f, 0), msg);
  check(
      len > 0 && is_json(msg.data()) &&
          has(msg, "\"entries\":[],\"total\":0,\"offset\":0,\"pending\":false"),
      "empty log");

  /* Source / reason names. */
  timebase_log_t two;
  timebase_log_init(&two);
  timebase_log_entry_t e{};
  e.source = TIMEBASE_SRC_NTP;
  e.reason = TIMEBASE_LOG_REASON_SAVE;
  timebase_log_append(&two, &e);
  e.source = TIMEBASE_SRC_MANUAL;
  e.reason = TIMEBASE_LOG_REASON_MONTHLY;
  timebase_log_append(&two, &e);
  len = enveloped(kResponseHead, enc(two, 0, false, 0.2f, 0), msg);
  check(len > 0 && has(msg, "\"source\":\"ntp\"") &&
            has(msg, "\"reason\":\"save\"") &&
            has(msg, "\"source\":\"manual\"") &&
            has(msg, "\"reason\":\"monthly\""),
        "log names");

  /* A full ring (worst case and realistic): paged, <= 2 pages, and the
   * pages concatenate to the ring. */
  for (const bool worst : {true, false}) {
    log = make_log(24, worst);
    uint32_t offset = 0;
    int pages = 0;
    std::string got;
    while (offset < 24u && pages < 8) {
      len = enveloped(kResponseHead,
                      enc(log, offset, false, worst ? -FLT_MAX : 0.2f,
                          worst ? UINT32_MAX : 1702000000u),
                      msg);
      check(len > 0 && is_json(msg.data()) &&
                static_cast<std::size_t>(len) <= kMaxJson,
            "log page fits 2048");
      check(emitted >= 1u, "log page makes progress");
      std::printf("getTimebaseLog %s page %d: offset %u, %u entries, %d "
                  "bytes\n",
                  worst ? "worst" : "realistic", pages, offset, emitted, len);
      /* Append this page's entries text (between '[' and '],"total"'). */
      const char *b = std::strstr(msg.data(), "\"entries\":[") + 11;
      const char *t = std::strstr(msg.data(), "],\"total\"");
      got += (got.empty() ? "" : ",") + std::string(b, t);
      check(has(msg, ("\"offset\":" + std::to_string(offset) + ",").c_str()),
            "log page echoes offset");
      offset += emitted;
      pages++;
    }
    check(offset == 24u, "log pages cover the ring");
    check(pages <= 2, "full ring needs <= 2 pages");
    /* The concatenation equals the entries of one unlimited encoding. */
    std::string want;
    for (uint32_t i = 0; i < 24u; i++) {
      char one[160];
      const auto *en = timebase_log_get(&log, i);
      std::snprintf(one, sizeof(one),
                    "{\"dateUnix\":%lu,\"ppm\":%.5g,\"source\":\"%s\","
                    "\"durationS\":%lu,\"reason\":\"%s\"}",
                    static_cast<unsigned long>(en->date_unix),
                    static_cast<double>(en->ppm),
                    en->source == TIMEBASE_SRC_PPS ? "pps" : "manual",
                    static_cast<unsigned long>(en->duration_s),
                    timebase_log_reason_name(en->reason));
      want += (i ? "," : "") + std::string(one);
    }
    check(got == want, "concatenated pages reproduce the ring");
  }
}

void test_pipeline_and_hello() {
  uint32_t count = 0;
  const auto *nodes = dsp_pipeline_nodes(&count);
  std::array<char, kMaxJson> msg{};
  auto pipe = [&](bool quartz) {
    return [&, quartz](char *d, std::size_t l, int off) {
      return hid_json::encode_pipeline(d, l, off, nodes, count, quartz);
    };
  };
  /* The array alone (no envelope): the encoder output is the JSON value. */
  int len = pipe(true)(msg.data(), msg.size(), 0);
  check(len > 0 && is_json(msg.data()), "pipeline json");
  check(has(msg, "{\"key\":\"input\",\"group\":\"source\",\"order\":0,"
                 "\"inputs\":[]}"),
        "pipeline input node");
  check(has(msg, "{\"key\":\"quartz\",\"group\":\"quartz\",\"order\":0,"
                 "\"inputs\":[\"input\"]}"),
        "pipeline quartz node");
  check(has(msg, "{\"key\":\"usb\",\"group\":\"sink\",\"order\":9,"
                 "\"inputs\":[\"pcm24\"]}"),
        "pipeline usb sink");
  check(has(msg, "\"key\":\"levelMeter\",\"group\":\"analysis\",\"order\":0,"
                 "\"inputs\":[\"input\",\"limiter\"]"),
        "pipeline levelMeter inputs");
  check(count_of(msg, "\"key\":") == static_cast<int>(count),
        "pipeline lists every node");
  std::printf("pipeline (quartz): %d bytes\n", len);

  len = pipe(false)(msg.data(), msg.size(), 0);
  check(len > 0 && is_json(msg.data()) && !has(msg, "quartz"),
        "pipeline without quartz has no quartz node or input");
  check(count_of(msg, "\"key\":") == static_cast<int>(count) - 1,
        "pipeline drops exactly the quartz node");
  /* A node whose only input is quartz would show an empty list; the real
   * table has none, so just check the shape of a synthetic one. */
  static const char *const kIn[] = {"input", "quartz"};
  const dsp_pipeline_node_t synth[] = {
      {"input", DSP_NODE_SOURCE, 0, nullptr, 0},
      {"quartz", DSP_NODE_QUARTZ, 0, kIn, 1},
      {"x", DSP_NODE_ANALYSIS, 0, kIn, 2},
  };
  char buf[256];
  len = hid_json::encode_pipeline(buf, sizeof(buf), 0, synth, 3, false);
  check(len > 0 && is_json(buf) &&
            std::strcmp(buf,
                        "[{\"key\":\"input\",\"group\":\"source\",\"order\":0,"
                        "\"inputs\":[]},{\"key\":\"x\",\"group\":\"analysis\","
                        "\"order\":0,\"inputs\":[\"input\"]}]") == 0,
        "synthetic pipeline drops inputs naming an omitted node");

  /* hello, with the longest identity strings the firmware can have. */
  hid_json::HelloParams p{};
  p.fw_version = "1.2.3-123-gabcdef0-dirty_______"; /* 31 chars */
  p.board = "uberkiwi_rp2350_____";
  p.source = "pcm1808___";
  p.vid = 65535;
  p.pid = 65535;
  p.max_json = 2048;
  p.metrics_hz_max = 20;
  p.beat_hz_max = 5;
  p.timing_hz_max = 5;
  p.quartz_hz_max = 5;
  p.quartz_pulses_max = 8;
  p.quartz = true;
  p.gps = true;
  p.pps = true;
  auto hello = [&](char *d, std::size_t l, int off) {
    return hid_json::encode_hello(d, l, off, p);
  };
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()), "hello json");
  check(len > 0 && static_cast<std::size_t>(len) < kMaxJson,
        "hello (quartz, worst identity) fits 2048");
  std::printf("hello worst case (quartz + gps + pps): %d bytes with envelope"
              "\n",
              len);
  check(has(msg,
            "\"deviceMode\":true,\"beats\":true,\"beatsMax\":8,"
            "\"beatsPacked\":1,\"gps\":true,\"pps\":true,\"timebaseLog\":true,"
            "\"timebaseLogMax\":24,"),
        "hello caps");
  check(has(msg, "\"pipeline\":[{\"key\":\"input\""), "hello pipeline");
  check(has(msg, "\"key\":\"quartz\""), "hello keeps the quartz node");

  p.quartz = false;
  p.gps = false;
  p.pps = false;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()) && has(msg, "\"quartz\":false,") &&
            has(msg, "\"gps\":false,\"pps\":false,"),
        "hello without quartz / gps / pps");
  check(!has(msg, "\"key\":\"quartz\"") && !has(msg, "\"inputs\":[\"quartz\""),
        "hello without quartz has no quartz pipeline node");
  std::printf("hello without quartz: %d bytes with envelope\n", len);

  /* Block keys still match the table (no 'bypass'). */
  check(has(msg, "\"blocks\":[\"levelMeter\",") && !has(msg, "\"bypass\""),
        "hello blocks");
}

/** Parameters of the longest {@code hello} the firmware can send. */
hid_json::HelloParams worst_hello_params() {
  hid_json::HelloParams p{};
  p.fw_version = "1.2.3-123-gabcdef0-dirty_______"; /* 31 chars */
  p.board = "uberkiwi_rp2350_____";
  p.source = "pcm1808___";
  p.vid = 65535;
  p.pid = 65535;
  p.max_json = 2048;
  p.metrics_hz_max = 20;
  p.beat_hz_max = 5;
  p.timing_hz_max = 5;
  p.quartz_hz_max = 5;
  p.quartz_pulses_max = 8;
  p.quartz = true;
  p.gps = true;
  p.pps = true;
  return p;
}

/** HJ-01..04: {@code hello.link}, {@code hello.control}, {@code caps.ble},
 * {@code caps.cli}. */
void test_hello_link_control() {
  std::array<char, kMaxJson> msg{};
  auto p = worst_hello_params();
  auto hello = [&](char *d, std::size_t l, int off) {
    return hid_json::encode_hello(d, l, off, p);
  };

  /* Without link_kind the shape is the one before H7. */
  int len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && !has(msg, "\"link\"") && !has(msg, "\"control\"") &&
            !has(msg, "\"ble\"") && !has(msg, "\"cli\""),
        "hello without link / ble / cli keeps the old shape");

  /* HJ-01 */
  p.link_kind = "usbHid";
  p.control_owner = nullptr;
  p.lease_ms = 10000;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()), "HJ-01 hello json");
  check(has(msg, ",\"link\":{\"kind\":\"usbHid\"},\"control\":"),
        "HJ-01 link kind usbHid, no mtu key");
  check(!has(msg, "\"mtu\""), "HJ-01 no mtu for usbHid");
  check(has(msg, "\"control\":{\"owner\":null,\"self\":false,"
                 "\"leaseMs\":10000}}}"),
        "HJ-04 owner none gives null; control closes the result");

  /* HJ-02 */
  p.link_kind = "ble";
  p.link_mtu = 185;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()) &&
            has(msg, "\"link\":{\"kind\":\"ble\",\"mtu\":185}"),
        "HJ-02 link kind ble with mtu 185");
  p.link_frame_max = 255;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()) &&
            has(msg, "\"link\":{\"kind\":\"ble\",\"mtu\":185,"
                     "\"frameMax\":255}"),
        "LF-02 hello.link.frameMax follows mtu");
  p.link_frame_max = 0;

  /* HJ-04 */
  p.control_owner = "ble";
  p.control_self = false;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()) &&
            has(msg, "\"control\":{\"owner\":\"ble\",\"self\":false,"
                     "\"leaseMs\":10000}"),
        "HJ-04 owner ble, self false");
  p.control_owner = "usbHid";
  p.control_self = true;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && has(msg, "\"control\":{\"owner\":\"usbHid\",\"self\":true,"),
        "HJ-04 owner usbHid, self true");

  /* HJ-03: caps keys only when built; the longest hello still fits. */
  check(!has(msg, "\"ble\":true") && !has(msg, "\"cli\":true"),
        "HJ-03 caps.ble / caps.cli absent when not built");
  p.ble = true;
  p.cli = true;
  p.link_mtu = 517;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && is_json(msg.data()), "HJ-03 hello with ble and cli is json");
  check(
      has(msg, "\"timebaseLogMax\":24,\"ble\":true,\"cli\":true,\"blocks\":["),
      "HJ-03 caps.ble and caps.cli are true when built");
  check(len > 0 && static_cast<std::size_t>(len) < kMaxJson,
        "HJ-03 the longest hello (ble, cli, link, control) fits 2048");
  std::printf("hello worst case with link + control + ble + cli: %d bytes\n",
              len);
  p.ble = false;
  len = enveloped(kResponseHead, hello, msg);
  check(len > 0 && !has(msg, "\"ble\":true") && has(msg, "\"cli\":true"),
        "HJ-03 caps.cli alone");
}

/** Reads the member names of {@code export interface name { ... }} from
 * the TypeScript contract; empty when the file is not found. */
std::vector<std::string> ts_interface_keys(const char *name) {
  std::vector<std::string> keys;
  /* Relative to this source file, so the test runs from any directory. */
  std::string path = __FILE__;
  path = path.substr(0, path.find_last_of('/') + 1) +
         "../docs/hid/timegrapher_hid.d.ts";
  std::ifstream in(path);
  std::string line;
  const std::string head = std::string("export interface ") + name + " ";
  bool inside = false;
  while (std::getline(in, line)) {
    if (!inside) {
      inside = line.rfind(head, 0) == 0;
      continue;
    }
    if (line == "}") {
      break;
    }
    if (line.rfind("  ", 0) == 0 && line.rfind("   ", 0) != 0 &&
        line.rfind("  /", 0) != 0) {
      std::size_t end = 2;
      while (end < line.size() &&
             (std::isalnum(static_cast<unsigned char>(line[end])) != 0)) {
        end++;
      }
      if (end > 2 && (line[end] == ':' || line[end] == '?')) {
        keys.push_back(line.substr(2, end - 2));
      }
    }
  }
  return keys;
}

/** A fully populated BLE status with the longest text values. */
hid_json::BleLinkParams worst_ble() {
  hid_json::BleLinkParams b;
  b.enabled = true;
  b.name = "TG-FFFF";
  b.address = "FF:FF:FF:FF:FF:FF";
  b.advertising = true;
  b.connected = true;
  b.mtu = UINT32_MAX;
  b.conn_interval_us = UINT32_MAX;
  b.conn_param_step = 3;
  b.conn_param_rejects = UINT32_MAX;
  b.rx_frames = UINT32_MAX;
  b.tx_frames = UINT32_MAX;
  b.rx_dropped = UINT32_MAX;
  b.rx_invalid = UINT32_MAX;
  b.tx_retries = UINT32_MAX;
  b.disconnects = UINT32_MAX;
  b.last_disconnect_reason = 255;
  b.core = 1;
  b.core1_stack_free = 8192;
  return b;
}

/** HJ-05: the {@code getLink} result. */
void test_link_status() {
  std::array<char, kMaxJson> msg{};
  hid_json::LinkStatusParams s;
  s.usb_mounted = true;
  s.control = {"usbHid", true, 10000};
  auto enc = [&](char *d, std::size_t l, int off) {
    return hid_json::encode_link_status(d, l, off, s);
  };
  int len = enveloped(kResponseHead, enc, msg);
  check(len > 0 && is_json(msg.data()), "HJ-05 getLink without ble is json");
  check(has(msg, "\"result\":{\"usbHid\":{\"mounted\":true},\"control\":"
                 "{\"owner\":\"usbHid\",\"self\":true,\"leaseMs\":10000}}}"),
        "HJ-05 usbHid and control present; no ble object");
  check(!has(msg, "\"ble\""), "HJ-05 no ble key without BLE");

  auto ble = worst_ble();
  s.ble = &ble;
  s.usb_mounted = false;
  s.control = {nullptr, false, 10000};
  len = enveloped(kResponseHead, enc, msg);
  check(len > 0 && is_json(msg.data()), "HJ-05 getLink with ble is json");
  check(len > 0 && static_cast<std::size_t>(len) < kMaxJson,
        "HJ-05 the longest getLink fits 2048 with the envelope");
  std::printf("getLink worst case: %d bytes with envelope\n", len);
  check(has(msg, "\"usbHid\":{\"mounted\":false}") &&
            has(msg, "\"owner\":null,\"self\":false"),
        "HJ-05 mounted false, owner null");
  const char *keys[] = {"enabled",
                        "name",
                        "address",
                        "advertising",
                        "connected",
                        "mtu",
                        "connIntervalUs",
                        "connParamStep",
                        "connParamRejects",
                        "rxFrames",
                        "txFrames",
                        "rxDropped",
                        "rxInvalid",
                        "txRetries",
                        "disconnects",
                        "lastDisconnectReason",
                        "core",
                        "core1StackFree"};
  bool all = true;
  for (const char *k : keys) {
    const std::string pat = std::string("\"") + k + "\":";
    all = all && std::strstr(msg.data(), pat.c_str()) != nullptr;
  }
  check(all, "HJ-05 every key of HidBleLinkStatus is present");
  check(has(msg, "\"name\":\"TG-FFFF\",\"address\":\"FF:FF:FF:FF:FF:FF\"") &&
            has(msg, "\"rxFrames\":4294967295") &&
            has(msg, "\"core\":1,\"core1StackFree\":8192}"),
        "HJ-05 values and the core 1 stack key");
  const auto ts_keys = ts_interface_keys("HidBleLinkStatus");
  if (ts_keys.empty()) {
    std::printf("note: docs/hid/timegrapher_hid.d.ts not found; skipped the "
                "HidBleLinkStatus key cross-check\n");
  } else {
    bool match = ts_keys.size() == sizeof(keys) / sizeof(keys[0]);
    for (const auto &k : ts_keys) {
      const std::string pat = std::string("\"") + k + "\":";
      match = match && std::strstr(msg.data(), pat.c_str()) != nullptr;
    }
    check(match, "HJ-05 the keys equal the HidBleLinkStatus interface");
  }
  ble.core = 0;
  len = enveloped(kResponseHead, enc, msg);
  check(len > 0 && has(msg, "\"core\":0}") && !has(msg, "core1StackFree"),
        "HJ-05 no core1StackFree on core 0");
  ble.core = 1;
  ble.core1_stack_free = -1;
  len = enveloped(kResponseHead, enc, msg);
  check(len > 0 && !has(msg, "core1StackFree"),
        "HJ-05 no core1StackFree when unknown");
}

/** X-macro of every {@code diag_counters_t} field and its camelCase key, in
 * struct order. */
#define DIAG_FIELDS(X)                                                         \
  X(usb_enumeration_count, "usbEnumerationCount")                              \
  X(usb_audio_stream_open_count, "usbAudioStreamOpenCount")                    \
  X(usb_audio_stream_close_count, "usbAudioStreamCloseCount")                  \
  X(usb_suspend_count, "usbSuspendCount")                                      \
  X(usb_resume_count, "usbResumeCount")                                        \
  X(boost_on_count, "boostOnCount")                                            \
  X(boost_off_count, "boostOffCount")                                          \
  X(preamp_settle_start_count, "preampSettleStartCount")                       \
  X(preamp_settle_done_count, "preampSettleDoneCount")                         \
  X(preamp_start_aborted_count, "preampStartAbortedCount")                     \
  X(usb_suspend_duration_ms, "usbSuspendDurationMs")                           \
  X(adc_total_sample_count, "adcTotalSampleCount")                             \
  X(adc_conversion_error_count, "adcConversionErrorCount")                     \
  X(dma_complete_count, "dmaCompleteCount")                                    \
  X(dma_overrun_count, "dmaOverrunCount")                                      \
  X(pcm_ring_underrun_count, "pcmRingUnderrunCount")                           \
  X(pcm_ring_overrun_count, "pcmRingOverrunCount")                             \
  X(usb_in_packet_count, "usbInPacketCount")                                   \
  X(usb_in_sample_count, "usbInSampleCount")                                   \
  X(usb_short_write_count, "usbShortWriteCount")                               \
  X(ring_min_occupancy, "ringMinOccupancy")                                    \
  X(ring_max_occupancy, "ringMaxOccupancy")                                    \
  X(adc_min_code, "adcMinCode")                                                \
  X(adc_max_code, "adcMaxCode")                                                \
  X(adc_mean_code_accum, "adcMeanCodeAccum")                                   \
  X(adc_mean_code_count, "adcMeanCodeCount")                                   \
  X(adc_dc_counts, "adcDcCounts")                                              \
  X(adc_ac_peak_counts, "adcAcPeakCounts")                                     \
  X(adc_pcm_peak_abs, "adcPcmPeakAbs")                                         \
  X(pcm1808_clock_config_ok, "pcm1808ClockConfigOk")                           \
  X(pcm1808_clk_sys_hz, "pcm1808ClkSysHz")                                     \
  X(pcm1808_clk_usb_hz, "pcm1808ClkUsbHz")                                     \
  X(pcm1808_mclk_nominal_hz, "pcm1808MclkNominalHz")                           \
  X(pcm1808_startup_wait_count, "pcm1808StartupWaitCount")                     \
  X(pcm1808_dma_blocks, "pcm1808DmaBlocks")                                    \
  X(pcm1808_stereo_frames_received, "pcm1808StereoFramesReceived")             \
  X(pcm1808_mono_samples_emitted, "pcm1808MonoSamplesEmitted")                 \
  X(pcm1808_dma_block_overruns, "pcm1808DmaBlockOverruns")                     \
  X(pcm1808_pio_fifo_stall_or_overflow, "pcm1808PioFifoStallOrOverflow")       \
  X(pcm1808_sample_peak_abs, "pcm1808SamplePeakAbs")                           \
  X(pcm1808_saturated_count, "pcm1808SaturatedCount")                          \
  X(pcm1808_raw_left_peak_abs, "pcm1808RawLeftPeakAbs")                        \
  X(pcm1808_raw_right_peak_abs, "pcm1808RawRightPeakAbs")                      \
  X(pcm1808_frame_rate_mhz, "pcm1808FrameRateMhz")                             \
  X(pcm1808_rate_status, "pcm1808RateStatus")                                  \
  X(pcm1808_mclk_ratio, "pcm1808MclkRatio")                                    \
  X(pcm1808_upsample, "pcm1808Upsample")                                       \
  X(pcm1808_blocks_lost, "pcm1808BlocksLost")

#define X(field, key) +1
/** Number of fields in {@link DIAG_FIELDS}. */
constexpr int kDiagFieldCount = 0 DIAG_FIELDS(X);
#undef X

/** HJ-06: the {@code getDiag} result. */
void test_diag() {
  std::array<char, kMaxJson> msg{};
  /* Distinct values: a swapped or missing field shows up as a mismatch. */
  diag_counters_t d{};
  std::uint32_t seed = 1000;
#define X(field, key) d.field = static_cast<decltype(d.field)>(seed++);
  DIAG_FIELDS(X)
#undef X
  auto enc = [&](char *dst, std::size_t l, int off) {
    return hid_json::encode_diag(dst, l, off, d);
  };
  int len = enveloped(kResponseHead, enc, msg);
  check(len > 0 && is_json(msg.data()), "HJ-06 getDiag is json");
  bool all = true;
  int fields = 0;
  seed = 1000;
#define X(field, key)                                                          \
  {                                                                            \
    const std::string pat = std::string("\"") + key + "\":" +                  \
                            std::to_string(static_cast<unsigned long>(         \
                                static_cast<decltype(d.field)>(seed++))) +     \
                            (fields == kDiagFieldCount - 1 ? "}" : ",");       \
    all = all && std::strstr(msg.data(), pat.c_str()) != nullptr;              \
    fields++;                                                                  \
  }
  DIAG_FIELDS(X)
#undef X
  check(all, "HJ-06 every field present under its camelCase key and value");
  check(fields == kDiagFieldCount, "HJ-06 the field list is complete");
  /* All members are 32-bit except the adjacent adc_min_code / adc_max_code
   * pair of 16-bit values (4 bytes together): a member missing from the table
   * changes the size. */
  check(sizeof(diag_counters_t) ==
            static_cast<std::size_t>(kDiagFieldCount) * 4u - 4u,
        "HJ-06 the table covers every member of diag_counters_t");

  const auto ts_keys = ts_interface_keys("HidDiag");
  if (ts_keys.empty()) {
    std::printf("note: docs/hid/timegrapher_hid.d.ts not found; skipped the "
                "HidDiag key cross-check\n");
  } else {
    bool match = static_cast<int>(ts_keys.size()) == fields;
    for (const auto &k : ts_keys) {
      const std::string pat = std::string("\"") + k + "\":";
      match = match && std::strstr(msg.data(), pat.c_str()) != nullptr;
    }
    check(match, "HJ-06 the keys equal the HidDiag interface");
  }

  /* The maxima must fit. */
  std::memset(&d, 0xFF, sizeof(d));
  len = enveloped(kResponseHead, enc, msg);
  check(len > 0 && is_json(msg.data()), "HJ-06 the maxima encode");
  check(len > 0 && static_cast<std::size_t>(len) < kMaxJson,
        "HJ-06 the maxima fit 2048 with the envelope");
  std::printf("getDiag worst case: %d bytes with envelope\n", len);
}

int main() {
  test_device_status();
  test_beats();
  test_events();
  test_int64();
  test_gps();
  test_timebase_log();
  test_pipeline_and_hello();
  test_hello_link_control();
  test_link_status();
  test_diag();

  /* Every mode at its longest. */
  std::size_t worst = 0;
  for (int mode = 0; mode < static_cast<int>(DSP_QUARTZ_MODE_COUNT); mode++) {
    const auto q = worst_case(static_cast<dsp_quartz_mode_t>(mode));
    const auto n = check_quartz(q, "worst-case getQuartz");
    std::printf("getQuartz worst case, mode %-10s: %zu bytes with envelope\n",
                control_api_quartz_mode_name(q.mode), n);
    worst = n > worst ? n : worst;
  }
  check(worst < kMaxJson, "getQuartz worst case fits 2048");

  /* Non-finite floats must not leak as nan / inf. */
  auto nf = realistic_motor();
  nf.rate_s_day = NAN;
  nf.motor.rate_without_s_day = INFINITY;
  nf.elapsed_s = -INFINITY;
  check_quartz(nf, "non-finite floats");

  /* Realistic snapshot, printed for the docs. */
  {
    char buf[kMaxJson];
    const int n =
        hid_json::encode_quartz(buf, sizeof(buf), 0, realistic_motor());
    check(n > 0 && is_json(buf), "realistic motor");
    check(std::strstr(buf, "\"measurementId\":7,") != nullptr,
          "measurementId encoded");
    check(std::strstr(buf, "\"rebuilding\":true") != nullptr,
          "rebuilding encoded");
    check(std::strstr(buf, "\"noJumpSpanS\":120,") != nullptr &&
              std::strstr(buf, "\"jumpThresholdUs\":20,") != nullptr,
          "motor constants encoded");
    std::printf("getQuartz realistic motor: %d bytes\n%s\n", n, buf);
  }

  /* USB microphone mode: analysis is reported off (TIMING_ENGINE_PLAN.md
   * section 8); a snapshot with the engine on says "on". */
  {
    char buf[kMaxJson];
    auto q = realistic_motor();
    int n = hid_json::encode_quartz(buf, sizeof(buf), 0, q);
    check(n > 0 && std::strstr(buf, "\"analysis\":\"on\",") != nullptr,
          "analysis on encoded");
    q.analysis = false;
    q.state = DSP_QUARTZ_STATE_IDLE;
    q.reason = "USB Audio mode";
    n = hid_json::encode_quartz(buf, sizeof(buf), 0, q);
    check(n > 0 && is_json(buf) &&
              std::strstr(buf, "\"state\":\"idle\",\"analysis\":\"off\",") !=
                  nullptr,
          "analysis off encoded");
  }

  /* A null reason / cal source must still give valid JSON. */
  {
    dsp_quartz_t q{};
    q.reason = nullptr;
    q.cal_source = nullptr;
    check_quartz(q, "null strings");
  }

  /* A reason with characters that need escaping is sanitised. */
  {
    auto q = realistic_motor();
    q.reason = "bad \"quote\" \\ back\nslash";
    check_quartz(q, "sanitised reason");
  }

  /* getState quartz object. */
  {
    char buf[256];
    const auto q = worst_case(DSP_QUARTZ_MODE_MECHANICAL);
    const int n = hid_json::encode_quartz_settings(buf, sizeof(buf), 0, q);
    check(n > 0 && is_json(buf), "settings object");
    std::printf("getState quartz object worst case: %d bytes\n", n);
  }

  /* One pulse, and the largest getQuartzPulses reply. */
  {
    char buf[kMaxJson];
    const auto p = worst_pulse();
    const int n = hid_json::encode_quartz_pulse(buf, sizeof(buf), 0, p);
    check(n > 0 && is_json(buf), "worst pulse");
    check(static_cast<std::size_t>(n) <= hid_json::kQuartzPulseJsonMax,
          "pulse within kQuartzPulseJsonMax");
    std::printf("pulse worst case: %d bytes (bound %zu)\n", n,
                hid_json::kQuartzPulseJsonMax);
    check(std::strlen(kNotifyHead) + static_cast<std::size_t>(n) + 1u <
              kMaxJson,
          "quartzPulse notify fits");

    std::array<dsp_quartz_pulse_t, hid_json::kQuartzPulsesMax> list{};
    list.fill(p);
    const int m = hid_json::encode_quartz_pulses(buf, sizeof(buf), 0,
                                                 list.data(), list.size());
    check(m > 0 && is_json(buf), "worst pulse list");
    const auto full =
        std::strlen(kResponseHead) + static_cast<std::size_t>(m) + 1u;
    check(full < kMaxJson, "getQuartzPulses max fits 2048");
    std::printf(
        "getQuartzPulses worst case (%u pulses): %zu bytes with envelope\n",
        static_cast<unsigned>(list.size()), full);

    const int e =
        hid_json::encode_quartz_pulses(buf, sizeof(buf), 0, nullptr, 0);
    check(e > 0 && std::strcmp(buf, "{\"pulses\":[]}") == 0, "empty list");

    dsp_quartz_pulse_t real{};
    real.sequence = 41;
    real.sample_index = 123456789012ull;
    real.t_s = 41.0000123;
    real.peak = 512345.f;
    real.snr = 51.7f;
    real.polarity = -1;
    real.closed = true;
    real.measurement_id = 7;
    const int r = hid_json::encode_quartz_pulse(buf, sizeof(buf), 0, real);
    check(r > 0 && is_json(buf) &&
              std::strstr(buf, "\"sampleIndex\":\"123456789012\"") != nullptr &&
              std::strstr(buf, "\"tS\":41.0000123") != nullptr &&
              std::strstr(buf, "{\"measurementId\":7,\"sequence\":41,") !=
                  nullptr,
          "realistic pulse");
    std::printf("pulse realistic: %s\n", buf);
  }

  /* Overflow is reported, never written past the buffer. */
  {
    char small[64];
    std::memset(small, 'x', sizeof(small));
    const int n = hid_json::encode_quartz(small, 40, 0, realistic_motor());
    check(n < 0, "overflow reported");
    check(small[40] == 'x', "no write past dstlen");
    check(hid_json::appendf(small, sizeof(small), -1, "x") == -1,
          "negative offset passes through");
  }

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
