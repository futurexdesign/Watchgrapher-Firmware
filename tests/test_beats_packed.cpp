/**
 * @file test_beats_packed.cpp
 * @brief Host test for the packed beats encoder
 *        ({@link hid_json::encode_beats_packed}): BP-01..04 and the encoder
 *        parts of BP-06 of docs/plans/BLE_TRANSPORT_TESTS.md section 12.
 *
 * A reference decoder (base64 and the layout of docs/plans/
 * BLE_TRANSPORT_PLAN.md section 14.2) lives in this file; it shares no code
 * with the encoder. BP-05 and the {@code getBeats} part of BP-06 run inside
 * the firmware-only control handler and are covered by the hardware and web
 * tests.
 *
 * Build and run (from the repository root):
 *   c++ -std=c++17 -Iinclude tests/test_beats_packed.cpp src/hid_json.cpp \
 *       src/dsp_blocks.cpp -o /tmp/t && /tmp/t
 */

#include "control_api.h"
#include "hid_json.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

/* hid_json.cpp names the quartz enums through control_api; this test never
 * encodes a quartz payload. */
extern "C" {
const char *control_api_quartz_mode_name(dsp_quartz_mode_t) { return ""; }
const char *control_api_quartz_state_name(dsp_quartz_state_t) { return ""; }
const char *control_api_quartz_inhibition_name(dsp_quartz_inhibition_t) {
  return "";
}
const char *control_api_quartz_ts_method_name(dsp_quartz_ts_method_t) {
  return "";
}
const char *control_api_quartz_lcd_status_name(dsp_quartz_lcd_status_t) {
  return "";
}
}

namespace {

constexpr std::size_t kMaxJson = 2048;
constexpr uint32_t kHz = 48000;

int g_failures = 0;

/** Records a failed check; the run exits non-zero at the end. */
void check(bool cond, const char *label) {
  if (!cond) {
    std::printf("FAIL: %s\n", label);
    g_failures++;
  }
}

/** One decoded beat, with the absolute values restored from the deltas. */
struct Decoded {
  uint32_t seq = 0;
  uint32_t grid_id = 0;
  float nominal_us = 0;
  uint64_t sample = 0;
  int32_t k = 0;
  uint8_t parity = 0;
  bool amp_valid = false;
  uint8_t status = 0;
  uint8_t frac = 0;
  double u_us = 0;
  double amp_deg = 0;
  double lift_us = 0;
};

/** Result of {@link decode}. */
struct Batch {
  bool ok = false;
  std::vector<Decoded> beats;
};

/** @return Value of a base64 character, or -1. */
int b64_val(char c) {
  if (c >= 'A' && c <= 'Z') {
    return c - 'A';
  }
  if (c >= 'a' && c <= 'z') {
    return c - 'a' + 26;
  }
  if (c >= '0' && c <= '9') {
    return c - '0' + 52;
  }
  return c == '+' ? 62 : (c == '/' ? 63 : -1);
}

/**
 * Decodes padded base64; fails on a bad character, bad padding or length.
 *
 * @param s Text.
 * @param out Receives the bytes.
 * @return true on success.
 */
bool b64_decode(const std::string &s, std::vector<uint8_t> &out) {
  out.clear();
  if (s.size() % 4u != 0u) {
    return false;
  }
  for (std::size_t i = 0; i < s.size(); i += 4) {
    int v[4];
    int pad = 0;
    for (int j = 0; j < 4; j++) {
      const char c = s[i + static_cast<std::size_t>(j)];
      if (c == '=') {
        if (i + 4 != s.size() || j < 2) {
          return false;
        }
        pad++;
        v[j] = 0;
      } else if (pad != 0 || (v[j] = b64_val(c)) < 0) {
        return false;
      }
    }
    const uint32_t w = (static_cast<uint32_t>(v[0]) << 18) |
                       (static_cast<uint32_t>(v[1]) << 12) |
                       (static_cast<uint32_t>(v[2]) << 6) |
                       static_cast<uint32_t>(v[3]);
    out.push_back(static_cast<uint8_t>(w >> 16));
    if (pad < 2) {
      out.push_back(static_cast<uint8_t>(w >> 8));
    }
    if (pad < 1) {
      out.push_back(static_cast<uint8_t>(w));
    }
  }
  return true;
}

/** @return Little-endian unsigned value of {@code n} bytes at {@code p}. */
uint64_t le(const uint8_t *p, int n) {
  uint64_t v = 0;
  for (int i = n - 1; i >= 0; i--) {
    v = (v << 8) | p[i];
  }
  return v;
}

/**
 * Extracts {@code p} from {@code {"p":"..."}} and decodes the layout.
 *
 * @param json Encoder output, with or without the envelope's closing brace.
 * @param bytes Receives the binary block size.
 * @return Decoded batch ({@code ok} false on any structural error).
 */
Batch decode(const char *json, std::size_t *bytes = nullptr) {
  Batch r;
  std::string s(json);
  if (s.size() > 3 && s.compare(s.size() - 3, 3, "\"}}") == 0) {
    s.pop_back(); /* the envelope's closing brace */
  }
  if (s.rfind("{\"p\":\"", 0) != 0 || s.size() < 8 ||
      s.compare(s.size() - 2, 2, "\"}") != 0) {
    return r;
  }
  std::vector<uint8_t> b;
  if (!b64_decode(s.substr(6, s.size() - 8), b)) {
    return r;
  }
  if (bytes != nullptr) {
    *bytes = b.size();
  }
  if (b.empty()) {
    r.ok = true;
    return r;
  }
  if (b.size() < 26 || b[0] != 1 || b[1] < 1 || b[1] > 32 ||
      b.size() != 26u + 17u * b[1]) {
    return r;
  }
  const auto seq0 = static_cast<uint32_t>(le(&b[2], 4));
  const auto grid = static_cast<uint32_t>(le(&b[6], 4));
  const auto nom_bits = static_cast<uint32_t>(le(&b[10], 4));
  float nominal = 0;
  std::memcpy(&nominal, &nom_bits, 4);
  uint64_t sample = le(&b[14], 8);
  auto k = static_cast<int32_t>(le(&b[22], 4));
  uint32_t seq = seq0;
  for (unsigned i = 0; i < b[1]; i++) {
    const uint8_t *p = &b[26 + 17u * i];
    Decoded d;
    seq += p[0];
    sample += le(p + 2, 4);
    k += static_cast<int16_t>(le(p + 7, 2));
    d.seq = seq;
    d.grid_id = grid;
    d.nominal_us = nominal;
    d.sample = sample;
    d.k = k;
    d.parity = p[1] & 1u;
    d.amp_valid = (p[1] & 2u) != 0;
    d.status = (p[1] >> 2) & 3u;
    d.frac = p[6];
    d.u_us = static_cast<int32_t>(le(p + 9, 4)) / 10.0;
    d.amp_deg = static_cast<double>(le(p + 13, 2)) / 100.0;
    d.lift_us = static_cast<double>(le(p + 15, 2));
    r.beats.push_back(d);
  }
  r.ok = true;
  return r;
}

/**
 * Builds a plausible beat stream: 10 beats per second at 48 kHz, k and seq
 * increasing by 1, alternating parity, mixed status and amplitude validity.
 *
 * @param n Number of beats.
 * @param first_seq Seq of the first beat.
 * @return Beats in ascending seq order, all on grid 3.
 */
std::vector<dsp_beat_t> make_beats(uint32_t n, uint32_t first_seq = 1000) {
  std::vector<dsp_beat_t> v(n);
  uint64_t sample = 5'000'000'000ull; /* beyond 32 bits: only deltas are */
  for (uint32_t i = 0; i < n; i++) {
    dsp_beat_t &b = v[i];
    b.seq = first_seq + i;
    b.grid_id = 3;
    sample += 4800u + (i * 37u) % 11u;
    b.sample_index = sample;
    const double frac = static_cast<double>((i * 53u) % 256u) / 256.0;
    b.t_s = (static_cast<double>(sample) + frac) / kHz;
    b.k = static_cast<int32_t>(500 + i);
    b.parity = static_cast<uint8_t>(b.k & 1);
    b.u_us = (static_cast<float>(i % 17u) - 8.0f) * 3.37f + 0.04f;
    b.nominal_us = 50000.0f;
    b.amplitude_valid = (i % 3u) != 1u;
    b.amplitude_deg = b.amplitude_valid ? 250.0f + 1.37f * i : 0.0f;
    b.lift_ms = 2.0f + 0.011f * i;
    b.status = (i % 7u == 3u)    ? DSP_BEAT_REJECTED
               : (i % 11u == 5u) ? DSP_BEAT_STARTED
                                 : DSP_BEAT_ACCEPTED;
  }
  return v;
}

/**
 * Encodes into a fresh message-sized buffer behind an envelope head.
 *
 * @param beats Input.
 * @param n Count.
 * @param emitted Receives the emitted count.
 * @param json Receives the full message text (head included).
 * @return Length of the message including the closing brace, or -1.
 */
int encode_msg(const dsp_beat_t *beats, uint32_t n, uint32_t *emitted,
               std::array<char, kMaxJson> &json) {
  int off = std::snprintf(
      json.data(), json.size(),
      "{\"v\":1,\"notify\":true,\"op\":\"beatsPacked\",\"data\":");
  off = hid_json::encode_beats_packed(json.data(), json.size(), off, beats, n,
                                      kHz, emitted);
  if (off < 0 || static_cast<std::size_t>(off) + 1u >= json.size()) {
    return -1;
  }
  json[static_cast<std::size_t>(off)] = '}';
  json[static_cast<std::size_t>(off) + 1u] = '\0';
  return off + 1;
}

/** @return The {@code data} object of a message from {@link encode_msg}. */
const char *data_of(const std::array<char, kMaxJson> &json) {
  return std::strstr(json.data(), "\"data\":") + 7;
}

/** Compares one decoded beat with its source within the format quanta. */
bool beat_matches(const Decoded &d, const dsp_beat_t &b) {
  const double frac_in = b.t_s * kHz - static_cast<double>(b.sample_index);
  const double frac_out = d.frac / 256.0;
  const double lift_us = b.lift_ms * 1000.0;
  return d.seq == b.seq && d.grid_id == b.grid_id &&
         d.sample == b.sample_index && d.k == b.k &&
         d.parity == (b.parity & 1u) && d.amp_valid == b.amplitude_valid &&
         d.nominal_us == b.nominal_us &&
         std::fabs(d.u_us - b.u_us) <= 0.05 + 1e-4 &&
         std::fabs(d.amp_deg - (b.amplitude_valid ? b.amplitude_deg : 0.0f)) <=
             0.005 + 1e-4 &&
         std::fabs(d.lift_us - lift_us) <= 0.5 + 1e-4 &&
         std::fabs(frac_out - frac_in) <= 1.0 / 256.0 &&
         d.status == (b.status == DSP_BEAT_ACCEPTED  ? 0u
                      : b.status == DSP_BEAT_STARTED ? 1u
                                                     : 2u);
}

/** BP-01: round trip of 1, 8 and 32 beats. */
void test_round_trip() {
  for (const uint32_t n : {1u, 8u, 32u}) {
    const auto in = make_beats(n);
    std::array<char, kMaxJson> msg{};
    uint32_t emitted = 0;
    const int len = encode_msg(in.data(), n, &emitted, msg);
    const auto out = decode(data_of(msg));
    check(len > 0 && emitted == n, "BP-01 all beats emitted");
    check(out.ok && out.beats.size() == n, "BP-01 decodes");
    bool all = out.ok && out.beats.size() == n;
    for (uint32_t i = 0; all && i < n; i++) {
      all = beat_matches(out.beats[i], in[i]);
    }
    check(all, "BP-01 fields within quanta");
    std::printf("BP-01: %u beats, %d bytes with envelope\n", n, len);
  }

  /* Every status and parity, amplitude valid and invalid, explicitly. */
  auto in = make_beats(6);
  const dsp_beat_status_t st[] = {DSP_BEAT_ACCEPTED, DSP_BEAT_STARTED,
                                  DSP_BEAT_REJECTED};
  for (uint32_t i = 0; i < in.size(); i++) {
    in[i].status = st[i % 3u];
    in[i].parity = static_cast<uint8_t>(i & 1u);
    in[i].amplitude_valid = (i / 3u) == 0u;
    in[i].amplitude_deg = in[i].amplitude_valid ? 300.0f : 123.0f;
  }
  std::array<char, kMaxJson> msg{};
  uint32_t emitted = 0;
  check(encode_msg(in.data(), 6, &emitted, msg) > 0 && emitted == 6u,
        "BP-01 mixed encodes");
  const auto out = decode(data_of(msg));
  bool all = out.ok && out.beats.size() == 6u;
  for (uint32_t i = 0; all && i < 6u; i++) {
    all =
        out.beats[i].status == i % 3u && out.beats[i].parity == (i & 1u) &&
        out.beats[i].amp_valid == (i < 3u) &&
        (i < 3u ? out.beats[i].amp_deg == 300.0 : out.beats[i].amp_deg == 0.0);
  }
  check(all, "BP-01 status / parity / amplitude flags");

  /* Saturation and signs: u negative, lift above 65,535 us, huge amplitude,
   * non-finite floats. */
  auto x = make_beats(3);
  x[0].u_us = -123456.7f;
  x[1].lift_ms = 90.0f;
  x[1].amplitude_deg = 9999.0f;
  x[1].amplitude_valid = true;
  x[2].u_us = NAN;
  x[2].lift_ms = INFINITY;
  x[2].t_s = NAN;
  check(encode_msg(x.data(), 3, &emitted, msg) > 0 && emitted == 3u,
        "BP-01 extremes encode");
  const auto ex = decode(data_of(msg));
  check(ex.ok && ex.beats.size() == 3u && ex.beats[0].u_us < -123456.0 &&
            ex.beats[1].lift_us == 65535.0 && ex.beats[1].amp_deg == 655.35 &&
            ex.beats[2].u_us == 0.0 && ex.beats[2].lift_us == 0.0 &&
            ex.beats[2].frac == 0u,
        "BP-01 saturation and non-finite values");
}

/** BP-02: exact size of 8 beats. */
void test_size() {
  const auto in = make_beats(8);
  std::array<char, kMaxJson> msg{};
  uint32_t emitted = 0;
  check(encode_msg(in.data(), 8, &emitted, msg) > 0, "BP-02 encodes");
  std::size_t bytes = 0;
  const auto out = decode(data_of(msg), &bytes);
  const std::string text(data_of(msg));
  /* data is {"p":"<b64>"}} : 6 + chars + 2 + the envelope's closing brace */
  const std::size_t chars = text.size() - 6 - 2 - 1;
  check(out.ok && bytes == 162u, "BP-02 binary block is 162 bytes");
  check(chars == 216u, "BP-02 base64 is 216 characters");
  std::printf("BP-02: %zu bytes, %zu characters, message %zu bytes\n", bytes,
              chars, std::strlen(msg.data()));
}

/**
 * Encodes {@code in} repeatedly like the notify loop and returns the batch
 * sizes.
 *
 * @param in Beats in ascending order.
 * @return Number of beats of each successive batch (empty on a failure).
 */
std::vector<uint32_t> batch_sizes(const std::vector<dsp_beat_t> &in) {
  std::vector<uint32_t> sizes;
  uint32_t at = 0;
  while (at < in.size()) {
    std::array<char, kMaxJson> msg{};
    uint32_t emitted = 0;
    if (encode_msg(in.data() + at, static_cast<uint32_t>(in.size()) - at,
                   &emitted, msg) < 0 ||
        emitted == 0u) {
      return {};
    }
    const auto d = decode(data_of(msg));
    if (!d.ok || d.beats.size() != emitted || d.beats[0].seq != in[at].seq) {
      return {};
    }
    sizes.push_back(emitted);
    at += emitted;
  }
  return sizes;
}

/** BP-03: grid, nominal, dSeq and dK breaks. */
void test_breaks() {
  using Sizes = std::vector<uint32_t>;
  auto in = make_beats(10);
  check(batch_sizes(in) == Sizes{10}, "BP-03 no break without a cause");

  in = make_beats(10);
  for (uint32_t i = 4; i < 10; i++) {
    in[i].grid_id = 4;
  }
  check(batch_sizes(in) == (Sizes{4, 6}), "BP-03 grid change");

  in = make_beats(10);
  for (uint32_t i = 6; i < 10; i++) {
    in[i].nominal_us = 50000.5f;
  }
  check(batch_sizes(in) == (Sizes{6, 4}), "BP-03 nominal change");

  in = make_beats(10);
  for (uint32_t i = 5; i < 10; i++) {
    in[i].seq += 255; /* dSeq = 256 at beat 5 */
  }
  check(batch_sizes(in) == (Sizes{5, 5}), "BP-03 dSeq of 256 breaks");

  in = make_beats(10);
  for (uint32_t i = 5; i < 10; i++) {
    in[i].seq += 254; /* dSeq = 255 still fits */
  }
  check(batch_sizes(in) == Sizes{10}, "BP-03 dSeq of 255 stays");

  in = make_beats(10);
  for (uint32_t i = 3; i < 10; i++) {
    in[i].k += 40000; /* dK outside i16 at beat 3 */
  }
  check(batch_sizes(in) == (Sizes{3, 7}), "BP-03 dK outside i16 breaks");

  in = make_beats(10);
  for (uint32_t i = 3; i < 10; i++) {
    in[i].k += 32767 - 1; /* dK = 32767 at beat 3 fits */
  }
  check(batch_sizes(in) == Sizes{10}, "BP-03 dK of 32766 stays");

  in = make_beats(10);
  for (uint32_t i = 7; i < 10; i++) {
    in[i].sample_index += 0x1'0000'0000ull; /* dSample >= 2^32 */
  }
  check(batch_sizes(in) == (Sizes{7, 3}), "BP-03 dSample of 2^32 breaks");

  in = make_beats(10);
  in[6].seq = in[5].seq; /* seq not increasing */
  check(batch_sizes(in).size() >= 2u, "BP-03 non-increasing seq breaks");

  /* More than 32 beats: batches of 32. */
  in = make_beats(70);
  check(batch_sizes(in) == (Sizes{32, 32, 6}), "BP-03 at most 32 per batch");

  /* A break at beat 0 of a later batch starts it with a zero delta header. */
  in = make_beats(4);
  in[2].grid_id = 9;
  in[3].grid_id = 9;
  std::array<char, kMaxJson> msg{};
  uint32_t emitted = 0;
  check(encode_msg(&in[2], 2, &emitted, msg) > 0 && emitted == 2u,
        "BP-03 next batch starts at the break");
  const auto d = decode(data_of(msg));
  check(d.ok && d.beats.size() == 2u && d.beats[0].seq == in[2].seq &&
            d.beats[0].grid_id == 9u,
        "BP-03 next batch header carries the break beat");
}

/** BP-04: the message never exceeds 2048 bytes; emitted says how many. */
void test_fit() {
  const auto in = make_beats(32);
  std::array<char, kMaxJson> msg{};
  uint32_t emitted = 0;
  int len = encode_msg(in.data(), 32, &emitted, msg);
  check(len > 0 && len < static_cast<int>(kMaxJson) && emitted == 32u,
        "BP-04 32 beats fit 2048");

  /* Shrink the buffer until fewer beats go; the count matches the decode. */
  bool monotonic = true;
  uint32_t prev = 0;
  for (std::size_t cap = 60; cap <= kMaxJson; cap += 7) {
    std::array<char, kMaxJson + 16> big{};
    big.fill('#');
    const int head = std::snprintf(
        big.data(), cap,
        "{\"v\":1,\"notify\":true,\"op\":\"beatsPacked\",\"data\":");
    uint32_t e = 0;
    int off = hid_json::encode_beats_packed(big.data(), cap, head, in.data(),
                                            32, kHz, &e);
    if (off < 0 || static_cast<std::size_t>(off) + 1u >= cap) {
      /* No room even for the empty string. */
      continue;
    }
    big[static_cast<std::size_t>(off)] = '}';
    const std::size_t total = static_cast<std::size_t>(off) + 1u;
    big[total] = '\0';
    const char *data = std::strstr(big.data(), "\"data\":") + 7;
    const auto d = decode(data);
    monotonic = monotonic && e >= prev && total < cap && d.ok &&
                d.beats.size() == e && big[cap] == '#';
    prev = e;
  }
  check(monotonic, "BP-04 emitted matches the decode at every capacity");
  check(prev == 32u, "BP-04 a full buffer takes all 32");

  /* A long head leaves room for only some beats (2048 - head). */
  std::array<char, kMaxJson> m2{};
  const int head = 1500;
  std::memset(m2.data(), 'x', static_cast<std::size_t>(head));
  uint32_t e = 0;
  const int off = hid_json::encode_beats_packed(m2.data(), m2.size(), head,
                                                in.data(), 32, kHz, &e);
  check(off > 0 && static_cast<std::size_t>(off) + 1u < kMaxJson && e > 0u &&
            e < 32u,
        "BP-04 a long head limits the batch");
  std::printf("BP-04: head 1500 bytes -> %u beats, %d bytes\n", e, off);

  /* No room for one beat: empty string, emitted 0. */
  std::array<char, 64> tiny{};
  e = 99;
  const int t = hid_json::encode_beats_packed(tiny.data(), tiny.size(), 0,
                                              in.data(), 32, kHz, &e);
  check(t > 0 && e == 0u && std::strcmp(tiny.data(), "{\"p\":\"\"}") == 0,
        "BP-04 no room for one beat gives an empty string");
  e = 99;
  const int z = hid_json::encode_beats_packed(m2.data(), m2.size(), 0, nullptr,
                                              0, kHz, &e);
  check(z > 0 && e == 0u && std::strcmp(m2.data(), "{\"p\":\"\"}") == 0,
        "BP-04 n == 0 gives an empty string");
  check(hid_json::encode_beats_packed(m2.data(), m2.size(), -1, in.data(), 1,
                                      kHz, nullptr) < 0,
        "BP-04 negative offset passes through");
}

/** BP-06 (encoder part): {@code hello.caps.beatsPacked} is 1. */
void test_caps() {
  hid_json::HelloParams p{};
  p.fw_version = "1.0";
  p.board = "b";
  p.source = "s";
  p.max_json = 2048;
  std::array<char, 4096> buf{};
  const int n = hid_json::encode_hello(buf.data(), buf.size(), 0, p);
  check(n > 0 && std::strstr(buf.data(), "\"beatsPacked\":1,") != nullptr,
        "BP-06 hello.caps.beatsPacked is 1");
  check(hid_json::kBeatsPackedFormat == 1u, "BP-06 format version is 1");
}

} // namespace

int main() {
  test_round_trip();
  test_size();
  test_breaks();
  test_fit();
  test_caps();
  if (g_failures != 0) {
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
  }
  std::printf("test_beats_packed: all checks passed\n");
  return 0;
}
