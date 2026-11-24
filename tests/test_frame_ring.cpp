/**
 * @file test_frame_ring.cpp
 * @brief Host test for control::FrameRing (SPSC frame queue).
 *
 * Implements RING-01..08 of docs/plans/BLE_TRANSPORT_TESTS.md. RING-08 runs a
 * producer and a consumer thread and needs {@code -pthread}.
 *
 * Build and run:
 *   c++ -std=c++17 -pthread -Iinclude tests/test_frame_ring.cpp -o /tmp/t &&
 * /tmp/t
 */

#include "control/frame_ring.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace control;

/** Scratch size for frames of up to 63 bytes. */
constexpr std::size_t kFrameBuf = 63;

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

/** Fills {@code buf} with a pattern derived from {@code seed}. */
void fill(std::uint8_t *buf, std::size_t len, std::uint32_t seed) {
  for (std::size_t i = 0; i < len; i++) {
    buf[i] = static_cast<std::uint8_t>((seed * 31U + i * 7U) & 0xFFU);
  }
}

/** Builds a frame of {@code len} bytes (len >= 4) carrying {@code seq}. */
void make_frame(std::uint8_t *buf, std::size_t len, std::uint32_t seq) {
  fill(buf, len, seq);
  std::memcpy(buf, &seq, sizeof(seq));
}

/** RING-01 */
void ring_01_empty() {
  FrameRing<8> r;
  std::uint8_t out[kRingSlotBytes];
  std::size_t len = 99;
  check(!r.pop(out, len), "RING-01 pop on empty returns false");
  check(len == 99, "RING-01 pop on empty leaves len untouched");
  check(r.size() == 0 && r.empty() && !r.full(), "RING-01 size 0");
  const std::uint8_t *d = nullptr;
  check(!r.front(d, len), "RING-01 front on empty returns false");
  r.drop_front();
  check(r.size() == 0, "RING-01 drop_front on empty is a no-op");
}

/** RING-02 */
void ring_02_roundtrip() {
  FrameRing<8> r;
  std::uint8_t in[31];
  fill(in, sizeof(in), 5);
  check(r.push(in, sizeof(in)), "RING-02 push");
  check(r.size() == 1, "RING-02 size 1");
  std::uint8_t out[kRingSlotBytes] = {};
  std::size_t len = 0;
  check(r.pop(out, len), "RING-02 pop");
  check(len == sizeof(in) && std::memcmp(in, out, len) == 0,
        "RING-02 identical bytes and length");
  check(r.empty(), "RING-02 empty after pop");
}

/** RING-03 */
void ring_03_capacity() {
  FrameRing<8> r;
  for (std::uint8_t i = 0; i < 8; i++) {
    const std::uint8_t f[2] = {i, static_cast<std::uint8_t>(i + 100)};
    check(r.push(f, 2), "RING-03 push within capacity");
  }
  check(r.full(), "RING-03 full");
  const std::uint8_t extra[1] = {0xEE};
  check(!r.push(extra, 1), "RING-03 9th push fails");
  check(r.overflows() == 1, "RING-03 overflows == 1");
  check(r.size() == 8, "RING-03 size unchanged by the rejected push");
  for (std::uint8_t i = 0; i < 8; i++) {
    std::uint8_t out[kRingSlotBytes];
    std::size_t len = 0;
    check(r.pop(out, len) && len == 2 && out[0] == i && out[1] == i + 100,
          "RING-03 FIFO order");
  }
  r.reset_overflows();
  check(r.overflows() == 0, "RING-03 reset_overflows");
}

/** RING-04 */
void ring_04_oversize() {
  FrameRing<8> r;
  std::uint8_t big[65] = {};
  const std::uint8_t one[1] = {7};
  check(r.push(one, 1), "RING-04 setup push");
  check(!r.push(big, 65), "RING-04 len 65 rejected");
  check(r.overflows() == 1, "RING-04 counted as overflow");
  check(r.size() == 1, "RING-04 ring unchanged");
  check(r.push(big, 64), "RING-04 len 64 accepted");
  std::uint8_t out[kRingSlotBytes];
  std::size_t len = 0;
  check(r.pop(out, len) && len == 1 && out[0] == 7,
        "RING-04 first frame intact");
  check(r.pop(out, len) && len == 64, "RING-04 64-byte frame intact");
}

/** RING-05 */
void ring_05_zero_length() {
  FrameRing<8> r;
  check(r.push(nullptr, 0), "RING-05 zero-length push accepted");
  std::uint8_t out[kRingSlotBytes];
  std::size_t len = 5;
  check(r.pop(out, len) && len == 0, "RING-05 popped with len 0");
}

/** RING-06 */
void ring_06_wraparound() {
  constexpr std::size_t kCap = 5; // not a power of two on purpose
  FrameRing<kCap> r;
  const auto total = static_cast<std::uint32_t>(10 * kCap * 2000);
  std::uint32_t pushed = 0;
  std::uint32_t popped = 0;
  bool ok = true;
  while (popped < total && ok) {
    // Push a burst of 1..kCap frames, then pop them all (varies the fill).
    const std::uint32_t burst = 1 + (pushed % kCap);
    for (std::uint32_t i = 0; i < burst && pushed < total; i++) {
      std::uint8_t f[40];
      const std::size_t len = 8 + (pushed % 33);
      make_frame(f, len, pushed);
      ok = ok && r.push(f, len);
      pushed++;
    }
    std::uint8_t out[kRingSlotBytes];
    std::size_t len = 0;
    while (r.pop(out, len)) {
      std::uint8_t expect[40];
      const std::size_t elen = 8 + (popped % 33);
      make_frame(expect, elen, popped);
      ok = ok && len == elen && std::memcmp(out, expect, len) == 0;
      popped++;
    }
  }
  check(ok, "RING-06 order and content preserved across index wraps");
  check(popped == total && total >= 70000, "RING-06 >= 70000 operations");
  check(r.overflows() == 0, "RING-06 no overflow");
}

/** RING-07 */
void ring_07_clear() {
  FrameRing<8> r;
  const std::uint8_t f[3] = {1, 2, 3};
  for (int i = 0; i < 3; i++) {
    check(r.push(f, 3), "RING-07 push");
  }
  r.clear();
  check(r.size() == 0, "RING-07 size 0 after clear");
  std::uint8_t out[kRingSlotBytes];
  std::size_t len = 0;
  check(!r.pop(out, len), "RING-07 pop after clear is empty");
  check(r.push(f, 3) && r.pop(out, len) && len == 3 && out[2] == 3,
        "RING-07 next push/pop works");
}

/** RING-08 */
void ring_08_threaded() {
  constexpr std::uint32_t kFrames = 1000000;
  static FrameRing<40> ring; // 40 = the RX ring size used by the firmware
  std::atomic<bool> producer_done{false};

  const auto start = std::chrono::steady_clock::now();
  std::thread producer([&] {
    std::uint8_t f[kFrameBuf];
    for (std::uint32_t seq = 0; seq < kFrames; seq++) {
      const std::size_t len = 4 + (seq % 60); // 4..63
      make_frame(f, len, seq);
      while (!ring.push(f, len)) {
        std::this_thread::yield();
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::uint32_t expect = 0;
  bool ok = true;
  std::uint8_t out[kRingSlotBytes];
  std::size_t len = 0;
  while (expect < kFrames && ok) {
    if (ring.pop(out, len)) {
      std::uint32_t seq = 0;
      std::memcpy(&seq, out, sizeof(seq));
      std::uint8_t want[kFrameBuf];
      const std::size_t wlen = 4 + (expect % 60);
      make_frame(want, wlen, expect);
      ok = seq == expect && len == wlen && std::memcmp(out, want, len) == 0;
      expect++;
    } else if (producer_done.load(std::memory_order_acquire) && ring.empty()) {
      break;
    }
  }
  producer.join();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - start)
                      .count();
  check(ok, "RING-08 every frame in order with correct content");
  check(expect == kFrames, "RING-08 all frames received exactly once");
  check(ring.empty(), "RING-08 ring drained");
  check(ms < 10000, "RING-08 finishes within 10 s");
  std::printf("RING-08: %u frames in %lld ms\n", static_cast<unsigned>(expect),
              static_cast<long long>(ms));
}

/** LF-01: {@code FrameRing<16, 255>} (256-byte slots) stores 255-byte frames;
 * the default ring still stores 64. */
void lf_01_large_slot() {
  static FrameRing<16, 255> big;
  static_assert(sizeof(FrameRing<16, 255>) >= 16 * 256,
                "slots are 256 bytes each");
  std::uint8_t in[255];
  std::uint8_t out[255] = {};
  std::size_t len = 0;
  for (std::uint32_t i = 0; i < 40; i++) { /* wraps the 2N counter */
    make_frame(in, sizeof(in), i);
    check(big.push(in, sizeof(in)), "LF-01 push of 255 bytes");
    check(big.pop(out, len) && len == sizeof(in) &&
              std::memcmp(in, out, len) == 0,
          "LF-01 255-byte frame returned intact");
  }
  std::uint8_t too_big[256] = {};
  check(!big.push(too_big, 256) && big.overflows() == 1,
        "LF-01 256 bytes exceed the u8 length and are rejected");
  for (int i = 0; i < 16; i++) {
    check(big.push(in, 255), "LF-01 fills 16 slots");
  }
  check(!big.push(in, 1) && big.full(), "LF-01 17th push is rejected");
  FrameRing<4> small;
  check(!small.push(in, 65), "LF-01 default ring still rejects 65 bytes");
}

} // namespace

int main() {
  ring_01_empty();
  ring_02_roundtrip();
  ring_03_capacity();
  ring_04_oversize();
  ring_05_zero_length();
  ring_06_wraparound();
  ring_07_clear();
  ring_08_threaded();
  lf_01_large_slot();
  if (g_failures != 0) {
    std::printf("test_frame_ring: %d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_frame_ring: all passed\n");
  return 0;
}
