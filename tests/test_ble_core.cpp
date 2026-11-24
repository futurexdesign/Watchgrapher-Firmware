/**
 * @file test_ble_core.cpp
 * @brief Host test for the core-0 hold helper (CORE-03, plan section 5.8
 *        rule 6). The helper takes an injected clock, so the test proves it
 *        uses nothing else: no delay, no yield, only flag and clock.
 */

#include "ble_core.h"

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>

namespace {

/** Fake clock: every read advances time by {@code step_ms}. */
struct FakeClock {
  std::uint32_t now_ms = 0;
  std::uint32_t step_ms = 1;
  /** Time at which {@code flag} is set; UINT32_MAX = never. */
  std::uint32_t set_at_ms = UINT32_MAX;
  std::atomic<bool> *flag = nullptr;

  std::uint32_t operator()() {
    const std::uint32_t t = now_ms;
    now_ms += step_ms;
    if (flag != nullptr && now_ms >= set_at_ms) {
      flag->store(true, std::memory_order_release);
    }
    return t;
  }
};

/** Wraps the fake clock by reference: the helper takes its clock by value. */
auto clock_ref(FakeClock &clk) {
  return [&clk] { return clk(); };
}

/** CORE-03: flag already set at 0 ms returns true at once. */
void core03_flag_set_at_0ms() {
  std::atomic<bool> flag{true};
  FakeClock clk;
  assert(
      ble_core::wait_radio_ready(flag, ble_core::kHoldCapMs, clock_ref(clk)));
  assert(clk.now_ms < 10);
}

/** CORE-03: flag set after 500 ms returns true before the cap. */
void core03_flag_set_at_500ms() {
  std::atomic<bool> flag{false};
  FakeClock clk;
  clk.flag = &flag;
  clk.set_at_ms = 500;
  assert(
      ble_core::wait_radio_ready(flag, ble_core::kHoldCapMs, clock_ref(clk)));
  assert(clk.now_ms >= 500 && clk.now_ms < 600);
}

/** CORE-03: flag never set returns false at the 3000 ms cap, not before. */
void core03_flag_never_set() {
  std::atomic<bool> flag{false};
  FakeClock clk;
  assert(
      !ble_core::wait_radio_ready(flag, ble_core::kHoldCapMs, clock_ref(clk)));
  assert(clk.now_ms >= 3000 && clk.now_ms <= 3002);
}

/** CORE-03: the cap is measured with wrap-safe arithmetic. */
void core03_clock_wraps() {
  std::atomic<bool> flag{false};
  FakeClock clk;
  clk.now_ms = UINT32_MAX - 1000;
  assert(
      !ble_core::wait_radio_ready(flag, ble_core::kHoldCapMs, clock_ref(clk)));
  const std::uint32_t elapsed = clk.now_ms - (UINT32_MAX - 1000);
  assert(elapsed >= 3000 && elapsed <= 3002);
}

/** CORE-03: a zero cap only samples the flag. */
void core03_zero_cap() {
  std::atomic<bool> flag{false};
  FakeClock clk;
  assert(!ble_core::wait_radio_ready(flag, 0, clock_ref(clk)));
  flag.store(true);
  assert(ble_core::wait_radio_ready(flag, 0, clock_ref(clk)));
}

/** CORE-03: the flag set on the final tick still counts as ready. */
void core03_flag_set_at_cap() {
  std::atomic<bool> flag{false};
  FakeClock clk;
  clk.flag = &flag;
  clk.set_at_ms = 3000;
  assert(
      ble_core::wait_radio_ready(flag, ble_core::kHoldCapMs, clock_ref(clk)));
}

} // namespace

int main() {
  core03_flag_set_at_0ms();
  core03_flag_set_at_500ms();
  core03_flag_never_set();
  core03_clock_wraps();
  core03_zero_cap();
  core03_flag_set_at_cap();
  std::puts("test_ble_core: all passed");
  return 0;
}
