/**
 * @file test_ble_pacer.cpp
 * @brief Host test for control::BlePacer (BLE notify pacing state machine).
 *
 * Implements PACE-01..08 of docs/plans/BLE_TRANSPORT_TESTS.md, plus PACE-09
 * (frame longer than the ATT payload limit), PACE-10 (backpressure) and
 * PACE-11 (can-send callback delivered inside the registration call). BTstack
 * is replaced by a fake that counts can-send requests and records
 * notifications.
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_ble_pacer.cpp -o /tmp/t && /tmp/t
 */

#include "control/ble_pacer.h"

#include <cstdint>
#include <cstdio>
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

/** Fake BTstack: counts requests, records notifications, scripted status. */
struct Fake {
  int requests = 0;
  int request_status = 0;
  std::vector<std::vector<std::uint8_t>> sent;
  std::vector<std::uint16_t> handles;
  /** Statuses returned by successive notify calls; 0 once exhausted. */
  std::vector<int> notify_status;
  std::size_t calls = 0;
  /**
   * When set, {@link request} delivers the can-send callback before it
   * returns, as BTstack does when the controller has a free buffer
   * (att_server_request_to_send_notification -> l2cap -> can-send-now).
   */
  BlePacer<16> *sync_pacer = nullptr;

  static int request(void *ctx, std::uint16_t) {
    auto *f = static_cast<Fake *>(ctx);
    f->requests++;
    if (f->request_status == 0 && f->sync_pacer != nullptr) {
      f->sync_pacer->on_can_send();
    }
    return f->request_status;
  }
  static int notify(void *ctx, std::uint16_t con, const std::uint8_t *data,
                    std::uint16_t len) {
    auto *f = static_cast<Fake *>(ctx);
    const int st =
        f->calls < f->notify_status.size() ? f->notify_status[f->calls] : 0;
    f->calls++;
    f->handles.push_back(con);
    f->sent.emplace_back(data, data + len);
    return st;
  }
};

/** Pacer + ring + fake wired together, connected with CCCD on. */
struct Rig {
  Fake fake;
  FrameRing<16> ring;
  BlePacer<16> pacer;
  Rig() : pacer(ring, BlePacerOps{&Fake::request, &Fake::notify, &fake}) {
    pacer.set_max_value_len(63);
  }
  void connect(std::uint16_t handle = 0x0040) {
    pacer.on_connect(handle);
    pacer.set_cccd(true);
  }
  void push(std::uint8_t tag) {
    const std::uint8_t f[3] = {tag, 2, 3};
    check(ring.push(f, 3), "setup push");
  }
};

/** PACE-01 */
void pace_01() {
  Rig r;
  r.connect();
  r.push(1);
  r.pacer.kick();
  check(r.fake.requests == 1, "PACE-01 exactly one request");
  check(r.pacer.pending(), "PACE-01 pending");
}

/** PACE-02 */
void pace_02() {
  Rig r;
  r.connect();
  r.push(1);
  r.pacer.kick();
  r.push(2);
  r.pacer.kick();
  check(r.fake.requests == 1, "PACE-02 no duplicate registration");
}

/** PACE-03 */
void pace_03() {
  Rig r;
  r.connect();
  r.push(1);
  r.push(2);
  r.pacer.kick();
  r.pacer.on_can_send();
  check(r.fake.sent.size() == 1 && r.fake.sent[0][0] == 1,
        "PACE-03 frame 1 sent");
  check(r.ring.size() == 1, "PACE-03 frame 1 popped");
  check(r.fake.requests == 2, "PACE-03 re-registered");
  r.pacer.on_can_send();
  check(r.fake.sent.size() == 2 && r.fake.sent[1][0] == 2,
        "PACE-03 frame 2 sent");
  check(r.fake.requests == 2, "PACE-03 not re-registered when empty");
  check(!r.pacer.pending() && r.ring.empty(), "PACE-03 idle");
  check(r.pacer.tx_frames() == 2, "PACE-03 tx_frames");
}

/** PACE-04 */
void pace_04() {
  Rig r;
  r.fake.notify_status = {-1};
  r.connect();
  r.push(9);
  r.pacer.kick();
  r.pacer.on_can_send();
  check(r.ring.size() == 1, "PACE-04 frame stays queued");
  check(r.fake.requests == 2, "PACE-04 re-registered");
  check(r.pacer.tx_retries() == 1, "PACE-04 tx_retries == 1");
  r.pacer.on_can_send();
  check(r.fake.sent.size() == 2 && r.fake.sent[1] == r.fake.sent[0] &&
            r.fake.sent[1][0] == 9,
        "PACE-04 same frame sent again");
  check(r.ring.empty() && r.pacer.tx_frames() == 1,
        "PACE-04 popped on success");
  r.pacer.reset_counters();
  check(r.pacer.tx_retries() == 0 && r.pacer.tx_frames() == 0,
        "PACE-04 reset_counters");
}

/** PACE-05 */
void pace_05() {
  Rig r;
  r.connect();
  r.push(1);
  r.pacer.kick();
  r.ring.clear();
  r.pacer.on_can_send();
  check(r.fake.sent.empty(), "PACE-05 no notify");
  check(r.fake.requests == 1, "PACE-05 no re-register");
  check(!r.pacer.pending(), "PACE-05 pending cleared");
}

/** PACE-06 */
void pace_06() {
  Rig r;
  r.connect();
  r.push(1);
  r.push(2);
  r.push(3);
  r.pacer.kick();
  r.pacer.on_disconnect();
  check(r.ring.empty(), "PACE-06 ring empty");
  check(!r.pacer.pending(), "PACE-06 pending false");
  check(!r.pacer.connected(), "PACE-06 not connected");
  const int before = r.fake.requests;
  const std::uint8_t f[1] = {4};
  check(r.ring.push(f, 1), "PACE-06 push while down");
  r.pacer.kick();
  check(r.fake.requests == before,
        "PACE-06 kick without connection is a no-op");
}

/** PACE-07 */
void pace_07() {
  Rig r;
  r.connect(0x0123);
  r.push(1);
  r.pacer.kick();
  r.pacer.on_can_send();
  check(r.fake.handles.size() == 1 && r.fake.handles[0] == 0x0123,
        "PACE-07 connection handle passed to notify");
}

/** PACE-08 */
void pace_08() {
  Rig r;
  r.pacer.on_connect(0x0040); // CCCD still off
  check(!r.pacer.can_send(), "PACE-08 can_send false with CCCD off");
  r.push(1);
  r.pacer.kick();
  check(r.fake.requests == 0, "PACE-08 kick does not register");
  r.pacer.set_cccd(true);
  check(r.pacer.can_send(), "PACE-08 can_send true with CCCD on");
  r.pacer.set_cccd(false);
  check(!r.pacer.can_send(), "PACE-08 can_send false after CCCD off");
}

/** PACE-09 (extra): a frame above the ATT payload limit is dropped, not sent.
 */
void pace_09() {
  Rig r;
  r.pacer.set_max_value_len(2);
  r.connect();
  r.push(1); // 3 bytes
  r.pacer.kick();
  r.pacer.on_can_send();
  check(r.fake.sent.empty(), "PACE-09 oversize frame not notified");
  check(r.ring.empty() && r.pacer.tx_oversize() == 1,
        "PACE-09 oversize frame dropped and counted");
}

/** PACE-10 (extra): can_send is false when the TX ring is full; a refused
 *  registration leaves nothing pending. */
void pace_10() {
  Rig r;
  r.connect();
  for (int i = 0; i < 16; i++) {
    r.push(static_cast<std::uint8_t>(i));
  }
  check(!r.pacer.can_send(), "PACE-10 can_send false with a full ring");
  r.fake.request_status = 1;
  r.pacer.kick();
  check(!r.pacer.pending(), "PACE-10 refused registration is not pending");
  r.fake.request_status = 0;
  r.pacer.kick();
  check(r.pacer.pending(), "PACE-10 kick registers again");
}

/** PACE-11: the callback runs inside the registration call (hardware bug). */
void pace_11() {
  Rig r;
  r.connect();
  r.fake.sync_pacer = &r.pacer;
  r.push(1);
  r.pacer.kick();
  check(r.fake.sent.size() == 1, "PACE-11 first frame sent");
  check(!r.pacer.pending(),
        "PACE-11 not pending after a callback delivered during registration");
  r.push(2);
  r.push(3);
  r.pacer.kick();
  check(r.fake.sent.size() == 3, "PACE-11 later frames sent");
  check(r.ring.empty() && !r.pacer.pending(), "PACE-11 idle");
}

} // namespace

int main() {
  pace_01();
  pace_02();
  pace_03();
  pace_04();
  pace_05();
  pace_06();
  pace_07();
  pace_08();
  pace_09();
  pace_10();
  pace_11();
  if (g_failures != 0) {
    std::printf("test_ble_pacer: %d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_ble_pacer: all passed\n");
  return 0;
}
