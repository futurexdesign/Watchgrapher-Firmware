/**
 * @file test_control_session.cpp
 * @brief Host test for {@link control::Session}: request sequencing, reply
 *        and notify fragmentation, backpressure, link-down handling.
 *
 * Implements SES-01..14 of docs/plans/BLE_TRANSPORT_TESTS.md with a fake
 * {@link control::Link} and a fake {@link control::Handler}; no control_api,
 * no Arduino. Time is passed explicitly; nothing sleeps.
 *
 * Build and run (from the repository root):
 *   c++ -std=c++17 -Iinclude tests/test_control_session.cpp \
 *       src/control/frame.cpp src/control/session.cpp -o /tmp/t && /tmp/t
 */

#include "control/frame.h"
#include "control/handler.h"
#include "control/link.h"
#include "control/session.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
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

/** One frame offered to the fake link, with the link's verdict. */
struct Attempt {
  Bytes bytes;
  bool accepted;
};

/** Fake transport: scriptable can_send and send_frame results. */
class FakeLink final : public Link {
public:
  FakeLink(const char *kind, bool pad, std::uint16_t mtu = 0)
      : kind_(kind), pad_(pad), mtu_(mtu) {}

  const char *kind() const noexcept override { return kind_; }
  bool connected() const noexcept override { return connected_; }
  bool can_send() const noexcept override { return can_send_; }
  bool send_frame(const std::uint8_t *f, std::size_t n) noexcept override {
    bool ok = true;
    if (!script_.empty()) {
      ok = script_.front();
      script_.pop_front();
    }
    attempts.push_back({Bytes(f, f + n), ok});
    return ok;
  }
  bool pad_frames() const noexcept override { return pad_; }
  std::uint16_t mtu() const noexcept override { return mtu_; }
  std::size_t max_tx_frame() const noexcept override { return max_tx_; }

  /** Accepted frames only. */
  std::vector<Bytes> delivered() const {
    std::vector<Bytes> out;
    for (const auto &a : attempts) {
      if (a.accepted) {
        out.push_back(a.bytes);
      }
    }
    return out;
  }

  bool connected_ = true;
  bool can_send_ = true;
  /** Reported by max_tx_frame(); 63 like HID, tests raise it for BLE. */
  std::size_t max_tx_ = kFrameMaxBytes;
  std::deque<bool> script_;
  std::vector<Attempt> attempts;

private:
  const char *kind_;
  bool pad_;
  std::uint16_t mtu_;
};

/** Fake handler: records calls; the reply text is chosen by a callback. */
class FakeHandler final : public Handler {
public:
  void on_request(const char *json, Reply &out) noexcept override {
    requests.emplace_back(json);
    link_kind_seen = out.link().kind();
    link_mtu_seen = out.link().mtu();
    frame_max_seen = out.tx_frame_max();
    const std::string reply = reply_for ? reply_for(json) : std::string("{}");
    out.send(reply.c_str(), reply.size(), false);
  }
  void poll(Reply &out, std::uint32_t) noexcept override {
    poll_calls++;
    if (on_poll) {
      on_poll(out);
    }
  }
  void reset() noexcept override { reset_calls++; }

  std::vector<std::string> requests;
  std::function<std::string(const std::string &)> reply_for;
  std::function<void(Reply &)> on_poll;
  int poll_calls = 0;
  int reset_calls = 0;
  std::string link_kind_seen;
  unsigned link_mtu_seen = 0;
  std::size_t frame_max_seen = 0;
};

/** Frames of one request: fragments of 58 bytes, optionally zero-padded. */
std::vector<Bytes> make_frames(const std::string &json, std::uint8_t seq,
                               bool pad = true) {
  std::vector<Bytes> out;
  const std::size_t count =
      json.empty() ? 1 : (json.size() + kFrameMaxChunk - 1) / kFrameMaxChunk;
  for (std::size_t i = 0; i < count; i++) {
    const std::string part = json.substr(i * kFrameMaxChunk, kFrameMaxChunk);
    Bytes b = {static_cast<std::uint8_t>(i + 1 < count ? kFlagMore : 0), seq,
               static_cast<std::uint8_t>(i), static_cast<std::uint8_t>(count),
               static_cast<std::uint8_t>(part.size())};
    b.insert(b.end(), part.begin(), part.end());
    if (pad) {
      b.resize(kFrameMaxBytes, 0);
    }
    out.push_back(b);
  }
  return out;
}

/** Delivers every frame of {@code json} to the session at time {@code t}. */
void send_request(Session &s, const std::string &json, std::uint8_t seq,
                  std::uint32_t t = 0) {
  for (const auto &f : make_frames(json, seq)) {
    s.on_frame(f.data(), f.size(), t);
  }
}

/** Runs {@code n} task passes at {@code t}. */
void run(Session &s, int n, std::uint32_t t = 0) {
  for (int i = 0; i < n; i++) {
    s.task(t);
  }
}

/** Decodes the messages carried by the device-to-host frames. */
std::vector<std::string> decode(const std::vector<Bytes> &frames) {
  std::vector<std::string> out;
  FrameReassembler r;
  for (const auto &f : frames) {
    if (r.push(f.data(), f.size(), 0) == RxResult::kComplete) {
      out.emplace_back(r.message(), r.length());
      r.release();
    }
  }
  return out;
}

/** A reply text of {@code n} characters that starts with {@code tag}. */
std::string text_of(const std::string &tag, std::size_t n) {
  std::string s = tag;
  while (s.size() < n) {
    s += 'x';
  }
  return s;
}

/** A request that carries a marker; the reply echoes the marker. */
std::string req(const char *mark) {
  return std::string("{\"v\":1,\"id\":1,\"op\":\"") + mark + "\"}";
}

void test_ses_01() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 130); };
  Session s(link, h);
  send_request(s, req("a"), 5);
  s.task(0);
  check(h.requests.size() == 1 && h.requests[0] == req("a"),
        "SES-01 handler sees the exact JSON once");
  check(link.attempts.size() == 1, "SES-01 one frame in the first task");
  run(s, 1);
  check(link.attempts.size() == 2, "SES-01 one frame per task (2)");
  run(s, 1);
  check(link.attempts.size() == 3, "SES-01 one frame per task (3)");
  run(s, 3);
  check(link.attempts.size() == 3 && h.requests.size() == 1,
        "SES-01 nothing more is sent or dispatched");
  const auto frames = link.delivered();
  check(frames.size() == 3 && frames[0][2] == 0 && frames[1][2] == 1 &&
            frames[2][2] == 2 && frames[0][3] == 3 && frames[2][3] == 3,
        "SES-01 consecutive fragIndex of one 3-fragment reply");
  const auto msgs = decode(frames);
  check(msgs.size() == 1 && msgs[0] == text_of("R", 130),
        "SES-01 reply text intact");
}

void test_ses_02_03() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &j) {
    return text_of(j.find("\"a\"") != std::string::npos ? "A" : "B", 130);
  };
  Session s(link, h);
  send_request(s, req("a"), 1);
  s.task(0); /* A dispatched, frame 0 of its reply sent */
  send_request(s, req("b"), 2);
  s.task(0); /* B is complete but A is still sending */
  check(h.requests.size() == 1, "SES-02 B waits while A is sending");
  /* C arrives while B is complete and not dispatched. */
  send_request(s, req("c"), 3);
  run(s, 10);
  check(h.requests.size() == 2 && h.requests[1] == req("b"),
        "SES-02 B is dispatched after A (not dropped)");
  bool saw_c = false;
  for (const auto &r : h.requests) {
    saw_c = saw_c || r == req("c");
  }
  check(!saw_c, "SES-03 C arrived while B was pending: dropped, never seen");
  const auto msgs = decode(link.delivered());
  check(msgs.size() == 2 && msgs[0] == text_of("A", 130) &&
            msgs[1] == text_of("B", 130),
        "SES-02 A is sent completely first, then B");
}

void test_ses_04() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 130); };
  Session s(link, h);
  send_request(s, req("a"), 1);
  s.task(0);
  check(link.delivered().size() == 1, "SES-04 first frame sent");
  link.can_send_ = false;
  run(s, 5);
  check(link.delivered().size() == 1 && link.attempts.size() == 1,
        "SES-04 no frames while can_send is false");
  link.can_send_ = true;
  run(s, 4);
  const auto f = link.delivered();
  check(f.size() == 3 && f[0][2] == 0 && f[1][2] == 1 && f[2][2] == 2,
        "SES-04 resumes with the next fragIndex, nothing skipped or repeated");
  const auto msgs = decode(f);
  check(msgs.size() == 1 && msgs[0] == text_of("R", 130),
        "SES-04 the message is intact");
}

void test_ses_05() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 130); };
  link.script_ = {true, false, true, true};
  Session s(link, h);
  send_request(s, req("a"), 1);
  run(s, 6);
  check(link.attempts.size() == 4, "SES-05 four send attempts");
  if (link.attempts.size() == 4) {
    const auto &a = link.attempts;
    check(a[0].bytes[2] == 0 && a[0].accepted && a[1].bytes[2] == 1 &&
              !a[1].accepted && a[2].bytes[2] == 1 && a[2].accepted &&
              a[3].bytes[2] == 2 && a[3].accepted,
          "SES-05 indexes 0, 1 (failed), 1, 2");
    check(a[1].bytes == a[2].bytes,
          "SES-05 the retried frame is byte-identical (F-15)");
  }
  const auto msgs = decode(link.delivered());
  check(msgs.size() == 1 && msgs[0] == text_of("R", 130),
        "SES-05 the delivered message is intact");
}

void test_ses_06() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 130); };
  bool pending = false;
  h.on_poll = [&](Reply &out) {
    if (pending && out.idle()) {
      pending = false;
      const char *n = "{\"v\":1,\"notify\":true,\"op\":\"x\",\"data\":{}}";
      out.send(n, std::strlen(n), true);
    }
  };
  Session s(link, h);
  send_request(s, req("a"), 1);
  s.task(0);
  pending = true; /* the notify is queued while the reply is in flight */
  run(s, 10);
  const auto f = link.delivered();
  check(f.size() == 4, "SES-06 three reply frames and one notify frame");
  if (f.size() == 4) {
    check(f[0][2] == 0 && f[1][2] == 1 && f[2][2] == 2 && f[3][2] == 0 &&
              f[3][3] == 1,
          "SES-06 all reply fragments come before the notify");
    check(f[0][1] == 1 && f[3][1] == 2,
          "SES-06 the notify has the next msgSeq");
    check((f[3][0] & kFlagNotifyHint) != 0 && (f[0][0] & kFlagNotifyHint) == 0,
          "SES-06 only the notify carries the notify hint");
  }
}

void test_ses_07() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 130); };
  Session s(link, h);
  send_request(s, req("a"), 1);
  s.task(0);
  s.task(0);
  check(h.poll_calls == 0, "SES-07 no poll while the TX is busy");
  run(s, 3);
  check(h.poll_calls >= 1, "SES-07 poll runs once the TX is idle");
  FakeLink down("ble", false);
  down.connected_ = false;
  FakeHandler h2;
  Session s2(down, h2, 4);
  run(s2, 5);
  check(h2.poll_calls == 0, "SES-07 no poll while the link is disconnected");
}

void test_ses_08() {
  FakeLink link("ble", false);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 130); };
  Session s(link, h);
  s.task(0); /* connected */
  send_request(s, req("a"), 1);
  s.task(0); /* reply 1 mid-message */
  /* A partial request (fragment 0 of 2) is stored. */
  const auto part =
      make_frames(text_of("{\"v\":1,\"id\":2,\"op\":\"p", 100), 9);
  s.on_frame(part[0].data(), part[0].size(), 0);
  const std::size_t sent_before = link.attempts.size();
  s.on_link_down();
  check(h.reset_calls == 1, "SES-08 reset() called once");
  run(s, 3);
  check(link.attempts.size() == sent_before,
        "SES-08 TX inactive after link-down");
  s.on_frame(part[1].data(), part[1].size(), 0);
  run(s, 2);
  check(h.requests.size() == 1, "SES-08 the stored partial RX was cleared");
  send_request(s, req("n"), 1);
  run(s, 10);
  check(h.requests.size() == 2, "SES-08 a new request works after reconnect");
  const auto f = link.delivered();
  check(!f.empty() && f.back()[1] == 2,
        "SES-08 the reply msgSeq continues incrementing");
  check(h.reset_calls == 1, "SES-08 explicit link-down is not reported twice");
}

void test_ses_09() {
  FakeLink link("ble", false);
  FakeHandler h;
  Session s(link, h, 4);
  s.task(0);
  check(h.reset_calls == 0, "SES-09 no reset while connected");
  link.connected_ = false;
  s.task(1);
  check(h.reset_calls == 1, "SES-09 true to false calls on_link_down");
  run(s, 3, 2);
  check(h.reset_calls == 1, "SES-09 once per transition");
  link.connected_ = true;
  s.task(3);
  link.connected_ = false;
  s.task(4);
  check(h.reset_calls == 2, "SES-09 a second transition counts again");
}

void test_ses_10() {
  FakeLink l1("usbHid", true);
  FakeLink l2("ble", false);
  FakeHandler h1;
  FakeHandler h2;
  h1.reply_for = [](const std::string &) { return std::string("{\"r\":1}"); };
  bool pending = true;
  h2.on_poll = [&](Reply &out) {
    if (pending && out.idle()) {
      pending = false;
      const char *n = "{\"v\":1,\"notify\":true,\"op\":\"y\",\"data\":{}}";
      out.send(n, std::strlen(n), true);
    }
  };
  Session s1(l1, h1);
  Session s2(l2, h2, 4);
  send_request(s1, req("a"), 7);
  run(s1, 3);
  run(s2, 3);
  const auto m1 = decode(l1.delivered());
  const auto m2 = decode(l2.delivered());
  check(m1.size() == 1 && m1[0] == "{\"r\":1}" && h2.requests.empty(),
        "SES-10 the reply appears only on L1");
  check(m2.size() == 1 && m2[0].find("\"op\":\"y\"") != std::string::npos &&
            h1.requests.size() == 1,
        "SES-10 the notify appears only on L2");
  check(l1.delivered()[0][1] == 1 && l2.delivered()[0][1] == 1,
        "SES-10 the msgSeq counters are independent");
}

void test_ses_11() {
  FakeLink link("ble", false);
  FakeHandler h;
  Session s(link, h, 4);
  const std::string json = text_of("{\"v\":1,\"id\":3,\"op\":\"q", 100);
  const auto fr = make_frames(json, 4);
  s.on_frame(fr[0].data(), fr[0].size(), 0);
  s.task(501);
  s.on_frame(fr[1].data(), fr[1].size(), 502);
  run(s, 3, 503);
  check(h.requests.empty(),
        "SES-11 the handler is never called after a timeout");
}

void test_ses_12() {
  FakeLink hid("usbHid", true);
  FakeLink ble("ble", false);
  FakeHandler h1;
  FakeHandler h2;
  h1.reply_for = h2.reply_for = [](const std::string &) {
    return std::string("0123456789");
  };
  Session s1(hid, h1);
  Session s2(ble, h2);
  send_request(s1, req("a"), 1);
  send_request(s2, req("a"), 1);
  s1.task(0);
  s2.task(0);
  check(hid.attempts.size() == 1 && hid.attempts[0].bytes.size() == 63,
        "SES-12 HID-like link: 63-byte frames");
  check(ble.attempts.size() == 1 && ble.attempts[0].bytes.size() == 5 + 10,
        "SES-12 BLE-like link: 5 + payloadLen bytes");
}

void test_ses_13() {
  FakeLink link("ble", false);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 580); };
  Session s(link, h, 4);
  send_request(s, req("a"), 1);
  s.task(0);
  check(link.attempts.size() == 4, "SES-13 first task: 4 frames");
  s.task(0);
  check(link.attempts.size() == 8, "SES-13 second task: 4 frames");
  s.task(0);
  check(link.attempts.size() == 10, "SES-13 third task: 2 frames");
  const auto msgs = decode(link.delivered());
  check(msgs.size() == 1 && msgs[0] == text_of("R", 580),
        "SES-13 the 10-fragment message is intact");
}

void test_ses_14() {
  FakeLink usb("usbHid", true);
  FakeLink ble("ble", false, 185);
  FakeHandler h1;
  FakeHandler h2;
  Session s1(usb, h1);
  Session s2(ble, h2, 4);
  send_request(s1, req("a"), 1);
  send_request(s2, req("a"), 1);
  s1.task(0);
  s2.task(0);
  check(h1.link_kind_seen == "usbHid" && h1.link_mtu_seen == 0,
        "SES-14 handler sees the usbHid link");
  check(h2.link_kind_seen == "ble" && h2.link_mtu_seen == 185,
        "SES-14 handler sees the ble link and its MTU");
}

/**
 * SES-15: a request is not starved by back-to-back notifies. Found on
 * hardware (2026-11-23): with every stream subscribed at its maximum rate the
 * BLE link was always busy at the start of a task, so the handler's next
 * notify always started first and an `unsubscribe` never got a reply.
 */
void test_ses_15() {
  FakeLink link("ble", false);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return std::string("{\"ok\":1}"); };
  /* Always has a 10-fragment notify due (580 bytes, 4 frames per task). */
  h.on_poll = [](Reply &out) {
    const std::string n = text_of("N", 580);
    out.send(n.c_str(), n.size(), true);
  };
  Session s(link, h, 4);
  for (int i = 0; i < 3; i++) {
    s.task(0);
  }
  send_request(s, req("a"), 1);
  int tasks = 0;
  while (h.requests.empty() && tasks < 20) {
    s.task(0);
    tasks++;
  }
  check(h.requests.size() == 1, "SES-15 request dispatched under notify load");
  check(tasks <= 3, "SES-15 dispatched as soon as the message in flight ends");
}

} // namespace

/** A hello request that asks for {@code frame_max}. */
std::string hello_req(long frame_max) {
  return "{\"v\":1,\"id\":1,\"op\":\"hello\",\"frameMax\":" +
         std::to_string(frame_max) + "}";
}

/** Largest frame among the delivered ones from index {@code from} on. */
std::size_t max_frame_from(const FakeLink &link, std::size_t from) {
  std::size_t m = 0;
  const auto d = link.delivered();
  for (std::size_t i = from; i < d.size(); i++) {
    m = d[i].size() > m ? d[i].size() : m;
  }
  return m;
}

void test_lf_02() {
  FakeLink link("ble", false, 517);
  link.max_tx_ = 255;
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 600); };
  Session s(link, h, 8);
  send_request(s, req("a"), 1);
  run(s, 4);
  check(max_frame_from(link, 0) <= kFrameMaxBytes &&
            link.delivered().size() > 4,
        "LF-02 before frameMax frames are <= 63 bytes");
  const std::size_t before = link.delivered().size();
  send_request(s, hello_req(255), 2);
  run(s, 4);
  const auto hello_frames = link.delivered();
  check(h.frame_max_seen == 255,
        "LF-02 hello reports frameMax 255 to the handler");
  check(max_frame_from(link, before) <= kFrameMaxBytes &&
            hello_frames.size() > before,
        "LF-02 the hello reply itself uses <= 63-byte frames");
  const std::size_t after = link.delivered().size();
  send_request(s, req("b"), 3);
  run(s, 4);
  const auto d = link.delivered();
  check(d.size() == after + 3 && d[after].size() == kFrameMaxTxBytes &&
            d[after][4] == kFrameMaxTxChunk,
        "LF-02 later replies use 255-byte frames (chunk 250)");
  /* The device-side reassembler keeps its 58-byte RX limit, so join by hand. */
  std::string joined;
  for (std::size_t i = after; i < d.size(); i++) {
    joined.append(d[i].begin() + kFrameHeaderBytes, d[i].end());
  }
  check(joined == text_of("R", 600),
        "LF-02 a large-frame reply reassembles intact");
}

void test_lf_03() {
  auto negotiated = [](std::size_t link_max, long ask) {
    FakeLink link("ble", false, 517);
    link.max_tx_ = link_max;
    FakeHandler h;
    Session s(link, h);
    send_request(s, hello_req(ask), 1);
    run(s, 2);
    return h.frame_max_seen;
  };
  check(negotiated(255, 62) == 63, "LF-03 frameMax below 63 is ignored");
  check(negotiated(255, 0) == 63, "LF-03 frameMax 0 is ignored");
  check(negotiated(255, -5) == 63, "LF-03 negative frameMax is ignored");
  check(negotiated(255, 63) == 63, "LF-03 frameMax 63 keeps 63");
  check(negotiated(255, 1000) == 255, "LF-03 frameMax 1000 gives 255");
  check(negotiated(120, 255) == 120,
        "LF-03 bounded by the link's max_tx_frame");
  check(negotiated(20, 255) == 63, "LF-03 a link limit below 63 keeps 63");

  /* A hello without frameMax keeps the value; a later low value is ignored. */
  FakeLink link("ble", false, 517);
  link.max_tx_ = 255;
  FakeHandler h;
  Session s(link, h);
  send_request(s, hello_req(200), 1);
  run(s, 2);
  send_request(s, req("hello"), 2);
  run(s, 2);
  check(h.frame_max_seen == 200, "LF-03 absent frameMax keeps the value");
  send_request(s, hello_req(10), 3);
  run(s, 2);
  check(h.frame_max_seen == 200, "LF-03 a low frameMax keeps the value");

  /* A later MTU drop bounds the size at the next message. */
  link.max_tx_ = 100;
  const std::size_t before = link.delivered().size();
  h.reply_for = [](const std::string &) { return text_of("R", 300); };
  send_request(s, req("a"), 4);
  run(s, 4);
  check(max_frame_from(link, before) == 100,
        "LF-03 a shrunk link limit bounds new messages");

  /* frameMax in a non-hello request has no effect. */
  FakeLink l2("ble", false, 517);
  l2.max_tx_ = 255;
  FakeHandler h2;
  Session s2(l2, h2);
  send_request(s2, "{\"v\":1,\"id\":1,\"op\":\"ping\",\"frameMax\":255}", 1);
  run(s2, 2);
  check(h2.frame_max_seen == 63, "LF-03 only hello negotiates frameMax");
}

void test_lf_04() {
  FakeLink link("ble", false, 517);
  link.max_tx_ = 255;
  FakeHandler h;
  Session s(link, h);
  send_request(s, hello_req(255), 1);
  run(s, 2);
  s.on_link_down();
  check(h.frame_max_seen == 255, "LF-04 negotiated before the drop");
  send_request(s, req("a"), 2);
  run(s, 2);
  check(h.frame_max_seen == 63, "LF-04 63 again after on_link_down");
  h.reply_for = [](const std::string &) { return text_of("R", 300); };
  const std::size_t before = link.delivered().size();
  send_request(s, req("b"), 3);
  run(s, 8);
  check(max_frame_from(link, before) <= kFrameMaxBytes,
        "LF-04 frames are <= 63 bytes after the reset");
}

void test_lf_05() {
  FakeLink link("usbHid", true);
  FakeHandler h;
  h.reply_for = [](const std::string &) { return text_of("R", 300); };
  Session s(link, h);
  send_request(s, hello_req(255), 1);
  run(s, 12);
  check(h.frame_max_seen == 63, "LF-05 HID keeps frameMax 63");
  const auto d = link.delivered();
  bool all63 = !d.empty();
  for (const auto &f : d) {
    all63 = all63 && f.size() == kFrameMaxBytes;
  }
  check(all63, "LF-05 every HID frame is 63 bytes, padded");
  send_request(s, req("a"), 2);
  run(s, 12);
  bool still63 = true;
  for (const auto &f : link.delivered()) {
    still63 = still63 && f.size() == kFrameMaxBytes;
  }
  check(still63, "LF-05 later HID frames stay 63 padded");
}

int main() {
  test_ses_01();
  test_ses_02_03();
  test_ses_04();
  test_ses_05();
  test_ses_06();
  test_ses_07();
  test_ses_08();
  test_ses_09();
  test_ses_10();
  test_ses_11();
  test_ses_12();
  test_ses_13();
  test_ses_14();
  test_ses_15();
  test_lf_02();
  test_lf_03();
  test_lf_04();
  test_lf_05();
  if (g_failures == 0) {
    std::printf("test_control_session: all checks passed\n");
    return 0;
  }
  std::printf("test_control_session: %d failure(s)\n", g_failures);
  return 1;
}
