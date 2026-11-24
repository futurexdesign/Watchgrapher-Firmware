/**
 * @file test_control_frame.cpp
 * @brief Host test for the control-plane framing (FrameReassembler,
 *        FrameFragmenter, normalize_out_report).
 *
 * Implements FR-RX-01..20, FR-TX-01..10, FR-RT-01..03 and FR-HID-01..03 of
 * docs/plans/BLE_TRANSPORT_TESTS.md. Time is passed explicitly; nothing sleeps.
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_control_frame.cpp src/control/frame.cpp
 * -o /tmp/t && /tmp/t
 */

#include "control/frame.h"
#include "control/hid_report.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace control;

namespace {

int g_failures = 0;

/** Records a failed check with its spec ID; the run exits non-zero at the end.
 */
void check(bool cond, const char *label) {
  if (!cond) {
    std::printf("FAIL: %s\n", label);
    g_failures++;
  }
}

using Bytes = std::vector<std::uint8_t>;

/** Builds {@code [flags, seq, idx, cnt, len, text...]}; payloadLen =
 * text.size(). */
Bytes F(std::uint8_t flags, std::uint8_t seq, std::uint8_t idx,
        std::uint8_t cnt, const std::string &text) {
  Bytes b = {flags, seq, idx, cnt, static_cast<std::uint8_t>(text.size())};
  b.insert(b.end(), text.begin(), text.end());
  return b;
}

/** Same as F, zero-padded to kFrameMaxBytes. */
Bytes Fpad(std::uint8_t flags, std::uint8_t seq, std::uint8_t idx,
           std::uint8_t cnt, const std::string &text) {
  Bytes b = F(flags, seq, idx, cnt, text);
  b.resize(kFrameMaxBytes, 0);
  return b;
}

RxResult push(FrameReassembler &r, const Bytes &b, std::uint32_t t) {
  return r.push(b.data(), b.size(), t);
}

std::string msg(const FrameReassembler &r) {
  return std::string(r.message(), r.length());
}

/** Upper bound on fragments of one message, plus slack. */
constexpr std::size_t kDrainLimit = kMaxJson / kFrameMaxChunk + 2;

/** Reads every fragment of a begun message with the given pad policy. */
std::vector<Bytes> drain(FrameFragmenter &f, bool pad) {
  std::vector<Bytes> out;
  std::size_t guard = 0;
  while (f.active()) {
    if (++guard > kDrainLimit) {
      check(false, "drain: fragmenter still active after the iteration bound "
                   "(commit() does not advance)");
      break;
    }
    std::uint8_t buf[kFrameMaxBytes];
    const std::size_t n = f.peek(buf, pad);
    out.emplace_back(buf, buf + n);
    f.commit();
  }
  return out;
}

/** Splits text into three fragments of 58, 58 and the rest. */
std::vector<std::string> split3(const std::string &t) {
  return {t.substr(0, 58), t.substr(58, 58), t.substr(116)};
}

std::string text_of(std::size_t n, char c) { return std::string(n, c); }

void test_reassembler() {
  { // FR-RX-01
    FrameReassembler r;
    check(push(r, F(0, 7, 0, 1, "{\"a\":1}"), 0) == RxResult::kComplete,
          "FR-RX-01: complete");
    check(r.ready(), "FR-RX-01: ready");
    check(std::strcmp(r.message(), "{\"a\":1}") == 0, "FR-RX-01: message");
    check(r.length() == 7, "FR-RX-01: length");
  }
  { // FR-RX-02
    FrameReassembler r;
    const auto p = split3(text_of(126, 'a'));
    check(push(r, F(1, 3, 0, 3, p[0]), 0) == RxResult::kAccepted,
          "FR-RX-02: first stored");
    const Bytes any = Fpad(1, 3, 1, 3, p[1]);
    check(r.push(any.data(), 4, 1) == RxResult::kShort, "FR-RX-02: kShort");
    check(push(r, F(1, 3, 1, 3, p[1]), 2) == RxResult::kAccepted,
          "FR-RX-02: fragment 1");
    check(push(r, F(0, 3, 2, 3, p[2]), 3) == RxResult::kComplete,
          "FR-RX-02: partial survived");
  }
  { // FR-RX-03
    FrameReassembler r;
    check(push(r, F(1, 1, 0, 2, "a"), 0) == RxResult::kAccepted,
          "FR-RX-03: partial stored");
    check(push(r, F(0, 1, 0, 0, "x"), 1) == RxResult::kInvalid,
          "FR-RX-03: kInvalid");
    check(push(r, F(0, 1, 1, 2, "b"), 2) == RxResult::kAccepted,
          "FR-RX-03: old seq restarts");
    check(!r.ready(), "FR-RX-03: partial discarded");
  }
  { // FR-RX-04
    FrameReassembler r;
    check(push(r, F(1, 1, 0, 2, "a"), 0) == RxResult::kAccepted,
          "FR-RX-04: partial stored");
    check(push(r, F(0, 1, 3, 3, "x"), 1) == RxResult::kInvalid,
          "FR-RX-04: kInvalid");
    check(push(r, F(0, 1, 1, 2, "b"), 2) == RxResult::kAccepted,
          "FR-RX-04: state cleared");
    check(!r.ready(), "FR-RX-04: not complete");
  }
  { // FR-RX-05
    FrameReassembler r;
    Bytes b(64, 'x');
    b[0] = 0;
    b[1] = 1;
    b[2] = 0;
    b[3] = 1;
    b[4] = 59;
    check(r.push(b.data(), b.size(), 0) == RxResult::kInvalid,
          "FR-RX-05: payloadLen 59");
  }
  { // FR-RX-06
    FrameReassembler r;
    check(push(r, F(1, 1, 0, 36, text_of(58, 'a')), 0) == RxResult::kInvalid,
          "FR-RX-06: fragCount 36 rejected");
    check(push(r, F(1, 2, 0, 35, text_of(58, 'a')), 0) == RxResult::kAccepted,
          "FR-RX-06: fragCount 35 accepted");
  }
  { // FR-RX-07
    FrameReassembler r;
    check(push(r, F(1, 4, 0, 2, "ab"), 0) == RxResult::kAccepted,
          "FR-RX-07: partial stored");
    Bytes b = F(1, 4, 1, 2, "0123456789");
    b.pop_back();
    check(r.push(b.data(), b.size(), 1) == RxResult::kShort,
          "FR-RX-07: kShort");
    check(push(r, F(0, 4, 1, 2, "cd"), 2) == RxResult::kComplete,
          "FR-RX-07: state untouched");
  }
  const std::string t126 = [] {
    std::string s;
    for (int i = 0; i < 126; i++) {
      s.push_back(static_cast<char>('A' + (i % 26)));
    }
    return s;
  }();
  const auto p126 = split3(t126);
  { // FR-RX-08
    FrameReassembler r;
    check(push(r, F(1, 2, 0, 3, p126[0]), 0) == RxResult::kAccepted,
          "FR-RX-08: 0");
    check(push(r, F(1, 2, 1, 3, p126[1]), 1) == RxResult::kAccepted,
          "FR-RX-08: 1");
    check(push(r, F(0, 2, 2, 3, p126[2]), 2) == RxResult::kComplete,
          "FR-RX-08: 2");
    check(r.length() == 126, "FR-RX-08: length");
    check(msg(r) == t126, "FR-RX-08: content");
  }
  { // FR-RX-09
    FrameReassembler r;
    check(push(r, F(0, 2, 2, 3, p126[2]), 0) == RxResult::kAccepted,
          "FR-RX-09: 2");
    check(push(r, F(1, 2, 0, 3, p126[0]), 1) == RxResult::kAccepted,
          "FR-RX-09: 0");
    check(push(r, F(1, 2, 1, 3, p126[1]), 2) == RxResult::kComplete,
          "FR-RX-09: 1");
    check(msg(r) == t126, "FR-RX-09: content");
  }
  { // FR-RX-10
    FrameReassembler r;
    check(push(r, F(1, 2, 0, 2, text_of(58, 'A')), 0) == RxResult::kAccepted,
          "FR-RX-10: 0");
    check(push(r, F(1, 2, 0, 2, text_of(58, 'B')), 1) == RxResult::kDuplicate,
          "FR-RX-10: kDuplicate");
    check(push(r, F(0, 2, 1, 2, "end"), 2) == RxResult::kComplete,
          "FR-RX-10: complete");
    check(msg(r) == text_of(58, 'A') + "end", "FR-RX-10: first copy kept");
  }
  { // FR-RX-11
    FrameReassembler r;
    check(push(r, F(1, 5, 0, 2, "old"), 0) == RxResult::kAccepted,
          "FR-RX-11: seq 5");
    check(push(r, F(0, 6, 0, 1, "new"), 1) == RxResult::kComplete,
          "FR-RX-11: seq 6");
    check(msg(r) == "new", "FR-RX-11: seq 6 text");
  }
  { // FR-RX-12
    FrameReassembler r;
    check(push(r, F(1, 5, 0, 3, "a"), 0) == RxResult::kAccepted,
          "FR-RX-12: first");
    check(push(r, F(1, 5, 1, 2, "b"), 1) == RxResult::kCountMismatch,
          "FR-RX-12: mismatch");
    check(push(r, F(0, 5, 2, 3, "c"), 2) == RxResult::kAccepted,
          "FR-RX-12: new reassembly");
    check(!r.ready(), "FR-RX-12: cannot complete");
  }
  { // FR-RX-13
    FrameReassembler r;
    check(push(r, F(1, 1, 0, 2, "a"), 1000) == RxResult::kAccepted,
          "FR-RX-13: 0");
    check(push(r, F(0, 1, 1, 2, "b"), 1500) == RxResult::kComplete,
          "FR-RX-13: 500 ms accepted");
  }
  { // FR-RX-14
    FrameReassembler r;
    check(push(r, F(1, 1, 0, 2, "a"), 1000) == RxResult::kAccepted,
          "FR-RX-14: 0");
    check(push(r, F(0, 1, 1, 2, "b"), 1501) == RxResult::kAccepted,
          "FR-RX-14: 501 ms restarts");
    check(!r.ready(), "FR-RX-14: not complete");
  }
  { // FR-RX-15
    FrameReassembler r;
    check(push(r, F(1, 1, 0, 2, "a"), 0) == RxResult::kAccepted, "FR-RX-15: 0");
    r.poll_timeout(500);
    check(push(r, F(0, 1, 1, 2, "b"), 500) == RxResult::kComplete,
          "FR-RX-15: 500 ms keeps");
    r.reset();
    check(push(r, F(1, 1, 0, 2, "a"), 0) == RxResult::kAccepted,
          "FR-RX-15: 0 again");
    r.poll_timeout(501);
    check(push(r, F(0, 1, 1, 2, "b"), 502) == RxResult::kAccepted,
          "FR-RX-15: alone accepted");
    check(!r.ready(), "FR-RX-15: not complete");
  }
  { // FR-RX-16
    FrameReassembler r;
    check(push(r, F(0, 1, 0, 1, "x"), 0) == RxResult::kComplete,
          "FR-RX-16: complete");
    r.poll_timeout(10000);
    check(r.ready(), "FR-RX-16: still ready");
    check(msg(r) == "x", "FR-RX-16: message intact");
  }
  { // FR-RX-17
    FrameReassembler r;
    check(push(r, F(0, 1, 0, 1, "keep"), 0) == RxResult::kComplete,
          "FR-RX-17: complete");
    check(push(r, F(0, 2, 0, 1, "other"), 1) == RxResult::kBusy,
          "FR-RX-17: kBusy");
    check(push(r, F(0, 1, 0, 0, "bad"), 1) == RxResult::kBusy,
          "FR-RX-17: busy before validation");
    check(r.ready() && msg(r) == "keep", "FR-RX-17: unchanged");
  }
  { // FR-RX-18
    FrameReassembler r;
    const Bytes f = F(0, 9, 0, 1, "again");
    check(push(r, f, 0) == RxResult::kComplete, "FR-RX-18: first");
    r.release();
    check(!r.ready(), "FR-RX-18: released");
    check(push(r, f, 1) == RxResult::kComplete, "FR-RX-18: same seq is new");
  }
  { // FR-RX-19
    FrameReassembler r;
    std::string all;
    for (int i = 0; i < 35; i++) {
      const std::string c = text_of(58, static_cast<char>('a' + (i % 26)));
      all += c;
      const RxResult res =
          push(r, F(i < 34 ? 1 : 0, 1, static_cast<std::uint8_t>(i), 35, c), 0);
      check(res == (i < 34 ? RxResult::kAccepted : RxResult::kComplete),
            "FR-RX-19: result");
    }
    check(r.length() == 2030, "FR-RX-19: length");
    check(r.message()[2030] == '\0', "FR-RX-19: NUL");
    check(msg(r) == all, "FR-RX-19: content");
  }
  { // FR-RX-20
    FrameReassembler a;
    FrameReassembler b;
    for (std::uint8_t i = 0; i < 3; i++) {
      const std::uint8_t flags = i < 2 ? 1 : 0;
      (void)push(a, F(flags, 4, i, 3, p126[i]), i);
      Bytes padded = Fpad(flags, 4, i, 3, p126[i]);
      for (std::size_t k = kFrameHeaderBytes + p126[i].size();
           k < padded.size(); k++) {
        padded[k] = 0xEE;
      }
      (void)push(b, padded, i);
    }
    check(a.ready() && b.ready(), "FR-RX-20: both ready");
    check(a.length() == b.length() && msg(a) == msg(b), "FR-RX-20: identical");
    check(msg(a) == t126, "FR-RX-20: content");
  }
}

void test_fragmenter() {
  { // FR-TX-01
    FrameFragmenter f;
    check(f.begin("", 0, false), "FR-TX-01: begin");
    std::uint8_t out[kFrameMaxBytes];
    check(f.peek(out, false) == 5, "FR-TX-01: size 5");
    const std::uint8_t want[5] = {0, 1, 0, 1, 0};
    check(std::memcmp(out, want, 5) == 0, "FR-TX-01: bytes");
    f.commit();
    check(!f.active(), "FR-TX-01: inactive");
  }
  { // FR-TX-02
    FrameFragmenter f;
    for (std::uint8_t want = 1; want <= 3; want++) {
      check(f.begin("x", 1, false), "FR-TX-02: begin");
      check(f.seq() == want, "FR-TX-02: seq");
      f.abort();
    }
  }
  { // FR-TX-03
    FrameFragmenter f;
    for (int i = 1; i <= 257; i++) {
      check(f.begin("x", 1, false), "FR-TX-03: begin");
      if (i == 256) {
        check(f.seq() == 0, "FR-TX-03: message 256 wraps to 0");
      }
      if (i == 257) {
        check(f.seq() == 1, "FR-TX-03: message 257 is 1");
      }
      f.abort();
    }
  }
  { // FR-TX-04
    const std::size_t lens[4] = {58, 59, 116, 117};
    const std::size_t counts[4] = {1, 2, 2, 3};
    for (int k = 0; k < 4; k++) {
      FrameFragmenter f;
      const std::string s = text_of(lens[k], 'q');
      check(f.begin(s.data(), s.size(), false), "FR-TX-04: begin");
      const auto frames = drain(f, false);
      check(frames.size() == counts[k], "FR-TX-04: fragment count");
      for (std::size_t i = 0; i < frames.size(); i++) {
        const bool last = i + 1 == frames.size();
        check(frames[i][3] == counts[k], "FR-TX-04: fragCount field");
        check(last || frames[i][4] == 58,
              "FR-TX-04: payloadLen 58 on non-last");
        check(((frames[i][0] & kFlagMore) != 0) == !last,
              "FR-TX-04: more flag");
      }
    }
  }
  { // FR-TX-05
    FrameFragmenter f;
    const std::string s = text_of(3000, 'z');
    check(f.begin(s.data(), s.size(), false), "FR-TX-05: begin");
    const auto frames = drain(f, false);
    std::size_t total = 0;
    for (const auto &fr : frames) {
      total += fr[4];
    }
    check(frames.size() == 36, "FR-TX-05: 36 fragments");
    check(total == 2048, "FR-TX-05: 2048 bytes");
    check(frames.back()[4] == 18, "FR-TX-05: last payloadLen 18");
  }
  { // FR-TX-06
    FrameFragmenter f;
    const std::string s = text_of(130, 'n');
    check(f.begin(s.data(), s.size(), true), "FR-TX-06: begin");
    const auto frames = drain(f, false);
    check(frames.size() == 3, "FR-TX-06: 3 fragments");
    for (const auto &fr : frames) {
      check((fr[0] & kFlagNotifyHint) != 0, "FR-TX-06: hint on every fragment");
    }
    check(frames.back()[0] == kFlagNotifyHint, "FR-TX-06: last flags == 0x02");
  }
  { // FR-TX-07
    FrameFragmenter f;
    const std::string s = text_of(126, 'p');
    check(f.begin(s.data(), s.size(), false), "FR-TX-07: begin");
    std::uint8_t a[kFrameMaxBytes];
    std::uint8_t b[kFrameMaxBytes];
    const std::size_t na = f.peek(a, false);
    const std::size_t nb = f.peek(b, false);
    check(na == nb && std::memcmp(a, b, na) == 0, "FR-TX-07: peek idempotent");
    check(a[2] == 0, "FR-TX-07: fragment 0");
    f.commit();
    (void)f.peek(a, false);
    check(a[2] == 1, "FR-TX-07: fragment 1 after commit");
  }
  { // FR-TX-08
    FrameFragmenter f;
    const std::string s = text_of(10, 'k');
    check(f.begin(s.data(), s.size(), false), "FR-TX-08: begin");
    std::uint8_t padded[kFrameMaxBytes];
    std::uint8_t plain[kFrameMaxBytes];
    std::memset(padded, 0xFF, sizeof(padded));
    check(f.peek(padded, true) == 63, "FR-TX-08: padded 63");
    check(f.peek(plain, false) == 15, "FR-TX-08: unpadded 15");
    bool zero = true;
    for (std::size_t i = 15; i < 63; i++) {
      zero = zero && padded[i] == 0;
    }
    check(zero, "FR-TX-08: padding zeroed");
    check(std::memcmp(padded, plain, 15) == 0, "FR-TX-08: same leading bytes");
  }
  { // FR-TX-09
    FrameFragmenter f;
    check(f.begin("AAA", 3, false), "FR-TX-09: begin A");
    check(!f.begin("BBB", 3, false), "FR-TX-09: second begin refused");
    std::uint8_t out[kFrameMaxBytes];
    const std::size_t n = f.peek(out, false);
    check(n == 8 && std::memcmp(&out[5], "AAA", 3) == 0 && out[1] == 1,
          "FR-TX-09: A intact");
  }
  { // FR-TX-10
    FrameFragmenter f;
    check(f.begin("A", 1, false), "FR-TX-10: begin");
    f.abort();
    check(!f.active(), "FR-TX-10: inactive");
    check(f.begin("B", 1, false), "FR-TX-10: begin again");
    check(f.seq() == 2, "FR-TX-10: seq continues");
  }
}

std::string random_text(std::size_t n, std::uint32_t &state) {
  std::string s;
  for (std::size_t i = 0; i < n; i++) {
    state = state * 1664525u + 1013904223u;
    s.push_back(static_cast<char>(0x20 + ((state >> 16) % 0x5F)));
  }
  return s;
}

/** Sends text through the fragmenter and reassembler; returns the received
 * text. */
bool round_trip(const std::string &text, bool pad, std::string &received) {
  FrameFragmenter f;
  FrameReassembler r;
  if (!f.begin(text.data(), text.size(), false)) {
    return false;
  }
  std::uint32_t t = 0;
  std::size_t guard = 0;
  while (f.active()) {
    if (++guard > kDrainLimit) {
      check(false, "round_trip: fragmenter still active after the iteration "
                   "bound (commit() does not advance)");
      return false;
    }
    std::uint8_t buf[kFrameMaxBytes];
    const std::size_t n = f.peek(buf, pad);
    (void)r.push(buf, n, t++);
    f.commit();
  }
  if (!r.ready()) {
    return false;
  }
  received = msg(r);
  return true;
}

void test_round_trip() {
  { // FR-RT-01
    std::uint32_t state = 12345;
    bool all = true;
    for (std::size_t n = 0; n <= 2030; n++) {
      const std::string s = random_text(n, state);
      std::string got;
      all = all && round_trip(s, false, got) && got == s;
    }
    check(all, "FR-RT-01: every length 0..2030 byte-exact");
  }
  { // FR-RT-02
    bool all = true;
    for (std::size_t n = 1; n <= 400; n++) {
      std::string s;
      while (s.size() < n) {
        s += "\xC2\xB0\xC2\xB1\xC2\xB5";
      }
      s.resize(n);
      std::string got;
      all = all && round_trip(s, false, got) && got == s;
    }
    check(all, "FR-RT-02: UTF-8 split across chunks byte-exact");
  }
  { // FR-RT-03
    std::uint32_t state = 99;
    const std::size_t lens[6] = {0, 1, 57, 58, 59, 2030};
    bool all = true;
    for (std::size_t n : lens) {
      const std::string s = random_text(n, state);
      std::string got;
      all = all && round_trip(s, true, got) && got == s;
    }
    check(all, "FR-RT-03: padded fragments byte-exact");
  }
}

void test_hid() {
  const std::uint8_t buf[4] = {1, 2, 3, 4};
  { // FR-HID-01
    const auto r = normalize_out_report(1, buf, 4);
    check(r.ok && r.data == buf && r.len == 4, "FR-HID-01: unchanged");
  }
  { // FR-HID-02
    const auto r = normalize_out_report(0, buf, 4);
    check(r.ok && r.data == buf + 1 && r.len == 3,
          "FR-HID-02: report ID skipped");
    const std::uint8_t other[2] = {7, 8};
    const auto u = normalize_out_report(0, other, 2);
    check(u.ok && u.data == other && u.len == 2,
          "FR-HID-02: no skip when buf[0] != 1");
  }
  { // FR-HID-03
    const auto r = normalize_out_report(2, buf, 4);
    check(!r.ok, "FR-HID-03: report ID 2 ignored");
  }
}

} // namespace

int main() {
  test_reassembler();
  test_fragmenter();
  test_round_trip();
  test_hid();
  if (g_failures != 0) {
    std::printf("test_control_frame: %d FAILED\n", g_failures);
    return 1;
  }
  std::printf("test_control_frame: OK\n");
  return 0;
}
