/**
 * @file test_control_arbiter.cpp
 * @brief Host test for control ownership: the pure {@link control::Arbiter}
 *        (ARB-01..15) and the per-link {@link control::ArbiterPort} with two
 *        sessions (ARB-16).
 *
 * Implements ARB-01..16 of docs/plans/BLE_TRANSPORT_TESTS.md. Time is passed
 * explicitly; nothing sleeps.
 *
 * Build and run (from the repository root):
 *   c++ -std=c++17 -Iinclude tests/test_control_arbiter.cpp \
 *       src/control/frame.cpp src/control/session.cpp \
 *       src/control/arbiter.cpp -o /tmp/t && /tmp/t
 */

#include "control/arbiter.h"
#include "control/frame.h"
#include "control/handler.h"
#include "control/link.h"
#include "control/session.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace control;
using Decision = Arbiter::Decision;

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

constexpr LinkKind U = LinkKind::kUsbHid;
constexpr LinkKind B = LinkKind::kBle;
constexpr std::uint32_t kLease = Arbiter::kLeaseMs;

void test_arb_01() {
  Arbiter a;
  std::uint32_t t = 0;
  std::uint32_t rng = 12345;
  const char *ops[] = {"getState",  "hello",   "ping",       "claim",
                       "subscribe", "getDiag", "patchState", "getLink"};
  bool ok = true;
  for (int i = 0; i < 1000; i++) {
    rng = rng * 1664525u + 1013904223u;
    t += (rng >> 8) % 3600u; /* mean 1.8 s: about 1 h in total */
    if ((rng >> 4) % 50u == 0u) {
      t += 60000u; /* gaps of 60 s, past the lease */
    }
    const char *op = ops[(rng >> 16) % 8u];
    const bool local = !std::strcmp(op, "hello") || !std::strcmp(op, "ping") ||
                       !std::strcmp(op, "claim");
    const auto o = a.on_request(U, op, false, t);
    ok = ok &&
         o.decision == (local ? Decision::kAnswerLocally : Decision::kDispatch);
    ok = ok && !o.reset_handler && o.notify_released == LinkKind::kNone;
  }
  check(ok, "ARB-01 single link: always dispatched or answered locally");
  check(a.owner() == U, "ARB-01 owner stays usbHid");
}

void test_arb_02_to_08() {
  {
    Arbiter a;
    const auto o = a.on_request(U, "getState", false, 0);
    check(o.decision == Decision::kDispatch && a.owner() == U &&
              !o.reset_handler,
          "ARB-02 auto-claim from none");
    const auto b = a.on_request(B, "getState", false, 1000);
    check(b.decision == Decision::kBusy && a.owner() == U,
          "ARB-03 non-owner refused");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    const auto h = a.on_request(B, "hello", false, 1000);
    const auto p = a.on_request(B, "ping", false, 2000);
    check(h.decision == Decision::kAnswerLocally &&
              p.decision == Decision::kAnswerLocally && a.owner() == U,
          "ARB-04 hello and ping always answered; owner unchanged");
    check(a.lease_valid(kLease) && !a.lease_valid(kLease + 1),
          "ARB-04 the owner's lease is not extended by the non-owner");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    (void)a.on_request(U, "ping", false, 9000);
    const auto o = a.on_request(B, "getState", false, 15000);
    check(o.decision == Decision::kBusy,
          "ARB-05 owner ping refreshes the lease");
    check(a.lease_valid(19000) && !a.lease_valid(19001),
          "ARB-05 the lease runs to 19000");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    (void)a.on_request(B, "ping", false, 9000);
    const auto o = a.on_request(B, "getState", false, 10001);
    check(o.decision == Decision::kDispatch && o.reset_handler &&
              o.notify_released == U && a.owner() == B,
          "ARB-06 non-owner ping does not refresh; expiry hands over");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    check(a.on_request(B, "getState", false, 10000).decision == Decision::kBusy,
          "ARB-07 exactly at the deadline: busy");
    check(a.on_request(B, "getState", false, 10001).decision ==
              Decision::kDispatch,
          "ARB-07 one ms later: dispatch");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    const auto o = a.on_request(U, "getState", false, 60000);
    check(o.decision == Decision::kDispatch && !o.reset_handler &&
              o.notify_released == LinkKind::kNone && a.owner() == U,
          "ARB-08 expiry alone resets nothing");
  }
}

void test_arb_09_to_14() {
  {
    Arbiter a;
    const auto o = a.on_request(B, "claim", false, 0);
    check(o.decision == Decision::kAnswerLocally && a.owner() == B &&
              !o.reset_handler,
          "ARB-09 claim when free");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    check(a.on_request(B, "claim", false, 100).decision == Decision::kBusy &&
              a.owner() == U,
          "ARB-10 claim without takeover while busy");
    check(a.on_request(B, "claim", true, 100).decision ==
              Decision::kAnswerLocally,
          "ARB-11 takeover is answered locally");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    const auto o = a.on_request(B, "claim", true, 100);
    check(a.owner() == B && o.reset_handler && o.notify_released == U,
          "ARB-11 takeover: owner B, reset, release notify to U");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    const auto o = a.on_request(U, "claim", true, 100);
    check(o.decision == Decision::kAnswerLocally && a.owner() == U &&
              !o.reset_handler && o.notify_released == LinkKind::kNone,
          "ARB-12 takeover by the owner itself changes nothing");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    check(a.on_link_down(U) && a.owner() == LinkKind::kNone,
          "ARB-13 owner link down: reset requested, owner none");
    const auto o = a.on_request(B, "getState", false, 100);
    check(o.decision == Decision::kDispatch && !o.reset_handler &&
              a.owner() == B,
          "ARB-13 the next op dispatches without reset");
  }
  {
    Arbiter a;
    (void)a.on_request(U, "getState", false, 0);
    check(!a.on_link_down(B) && a.owner() == U,
          "ARB-14 non-owner link down changes nothing");
  }
}

void test_arb_15() {
  Arbiter a;
  (void)a.on_request(U, "getState", false, 0xFFFFF000u);
  check(a.on_request(B, "getState", false, 0x00000800u).decision ==
            Decision::kBusy,
        "ARB-15 lease still valid across the millis() wrap");
  check(a.on_request(B, "getState", false, 0x00002000u).decision ==
            Decision::kDispatch,
        "ARB-15 expired after the wrap (deadline 0x1710)");
}

/* --- ARB-16: ports and sessions ----------------------------------------- */

using Bytes = std::vector<std::uint8_t>;

/** Fake transport recording the accepted frames. */
class FakeLink final : public Link {
public:
  FakeLink(const char *kind, bool pad) : kind_(kind), pad_(pad) {}
  const char *kind() const noexcept override { return kind_; }
  bool connected() const noexcept override { return connected_; }
  bool can_send() const noexcept override { return true; }
  bool send_frame(const std::uint8_t *f, std::size_t n) noexcept override {
    sent.emplace_back(f, f + n);
    return true;
  }
  bool pad_frames() const noexcept override { return pad_; }

  bool connected_ = true;
  std::vector<Bytes> sent;

private:
  const char *kind_;
  bool pad_;
};

/** Fake control handler: answers every request with an ok envelope, pushes
 * one notify per poll, and counts resets. */
class FakeControl final : public Handler {
public:
  void on_request(const char *json, Reply &out) noexcept override {
    requests.emplace_back(json);
    const char *r = "{\"v\":1,\"id\":1,\"ok\":true,\"result\":{}}";
    out.send(r, std::strlen(r), false);
  }
  void poll(Reply &out, std::uint32_t) noexcept override {
    if (out.idle()) {
      const char *n = "{\"v\":1,\"notify\":true,\"op\":\"push\",\"data\":{}}";
      out.send(n, std::strlen(n), true);
    }
  }
  void reset() noexcept override { resets++; }

  std::vector<std::string> requests;
  int resets = 0;
};

std::uint32_t g_now = 0;
std::uint32_t fake_clock() noexcept { return g_now; }

/** Messages carried by the frames a link accepted. */
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

/** Counts the messages that contain {@code needle}. */
int count_with(const std::vector<std::string> &msgs, const char *needle) {
  int n = 0;
  for (const auto &m : msgs) {
    n += m.find(needle) != std::string::npos ? 1 : 0;
  }
  return n;
}

void feed(Session &s, const std::string &json, std::uint8_t seq) {
  Bytes b = {0, seq, 0, 1, static_cast<std::uint8_t>(json.size())};
  b.insert(b.end(), json.begin(), json.end());
  s.on_frame(b.data(), b.size(), g_now);
}

void pass(Session &su, Session &sb, int n) {
  for (int i = 0; i < n; i++) {
    su.task(g_now);
    sb.task(g_now);
  }
}

void test_arb_16() {
  Arbiter arb;
  FakeControl ctl;
  ArbiterPort pu(U, arb, ctl, fake_clock);
  ArbiterPort pb(B, arb, ctl, fake_clock);
  FakeLink lu("usbHid", true);
  FakeLink lb("ble", false);
  pu.attach_link(lu);
  pb.attach_link(lb);
  pu.set_peer(&pb);
  pb.set_peer(&pu);
  Session su(lu, pu, 1);
  Session sb(lb, pb, 4);

  /* U owns after its first op; pushes go to U only. */
  g_now = 100;
  feed(su, "{\"v\":1,\"id\":1,\"op\":\"getState\"}", 1);
  pass(su, sb, 3);
  check(ctl.requests.size() == 1 && arb.owner() == U,
        "ARB-16 U's request is dispatched and U owns control");
  check(count_with(decode(lu.sent), "\"op\":\"push\"") >= 1 &&
            decode(lb.sent).empty(),
        "ARB-16 pushes go to the owner session only");

  /* B is refused with its own request id (a 64-bit value). */
  g_now = 200;
  feed(sb, "{\"v\":1,\"id\":9007199254740993,\"op\":\"getState\"}", 1);
  pass(su, sb, 3);
  const auto busy = decode(lb.sent);
  check(ctl.requests.size() == 1 && busy.size() == 1 &&
            busy[0] == "{\"v\":1,\"id\":9007199254740993,\"ok\":false,"
                       "\"error\":{\"code\":\"linkBusy\",\"message\":"
                       "\"control held by usbHid\"}}",
        "ARB-16 non-owner gets linkBusy with the request id; handler unseen");

  /* B takes over. */
  g_now = 300;
  const std::size_t u_before = lu.sent.size();
  feed(sb, "{\"v\":1,\"id\":10,\"op\":\"claim\",\"takeover\":true}", 2);
  pass(su, sb, 6);
  const auto claim_reply = decode(lb.sent);
  check(count_with(claim_reply,
                   "{\"v\":1,\"id\":10,\"ok\":true,\"result\":{\"owner\":"
                   "\"ble\",\"previous\":\"usbHid\"}}") == 1 &&
            arb.owner() == B,
        "ARB-16 takeover claim result names owner and previous");
  check(ctl.resets == 1, "ARB-16 takeover resets the handler once");
  const auto u_msgs = decode(std::vector<Bytes>(
      lu.sent.begin() + static_cast<std::ptrdiff_t>(u_before), lu.sent.end()));
  check(count_with(u_msgs,
                   "{\"v\":1,\"notify\":true,\"op\":\"controlReleased\","
                   "\"data\":{\"to\":\"ble\"}}") == 1,
        "ARB-16 the previous owner gets exactly one controlReleased");
  /* A push may already be in flight when B's claim is processed; none may
   * follow the controlReleased notify. */
  bool released_seen = false;
  bool push_after = false;
  for (const auto &m : u_msgs) {
    if (m.find("controlReleased") != std::string::npos) {
      released_seen = true;
    } else if (released_seen &&
               m.find("\"op\":\"push\"") != std::string::npos) {
      push_after = true;
    }
  }
  check(released_seen && !push_after,
        "ARB-16 U gets no pushes after losing control");
  pass(su, sb, 6);
  const auto u_all = decode(lu.sent);
  check(count_with(u_all, "controlReleased") == 1 &&
            count_with(std::vector<std::string>(u_all.end() - 1, u_all.end()),
                       "controlReleased") == 1,
        "ARB-16 controlReleased is not repeated");
  check(count_with(decode(lb.sent), "\"op\":\"push\"") >= 1,
        "ARB-16 pushes now go to B");

  /* Link-down handling through the ports. */
  const int resets = ctl.resets;
  lu.connected_ = false;
  pass(su, sb, 2);
  check(ctl.resets == resets && arb.owner() == B,
        "ARB-16 a non-owner link going down does not reset the handler");
  lb.connected_ = false;
  pass(su, sb, 2);
  check(ctl.resets == resets + 1 && arb.owner() == LinkKind::kNone,
        "ARB-16 the owner link going down resets the handler");

  /* A malformed request from a non-owner changes nothing. */
  g_now = 400;
  lu.connected_ = true;
  lb.connected_ = true;
  pass(su, sb, 2);
  feed(su, "{\"v\":1,\"op\":\"getState\"}", 3); /* no id */
  pass(su, sb, 2);
  check(arb.owner() == LinkKind::kNone && ctl.requests.size() == 2,
        "ARB-16 a request without id goes to the handler and claims nothing");
}

} // namespace

int main() {
  test_arb_01();
  test_arb_02_to_08();
  test_arb_09_to_14();
  test_arb_15();
  test_arb_16();
  if (g_failures == 0) {
    std::printf("test_control_arbiter: all checks passed\n");
    return 0;
  }
  std::printf("test_control_arbiter: %d failure(s)\n", g_failures);
  return 1;
}
