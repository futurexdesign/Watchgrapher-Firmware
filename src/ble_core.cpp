/**
 * @file ble_core.cpp
 * @brief Radio core selection ({@code TG_BLE_CORE}), the core-0 hold, the
 *        CYW43 status LED proxy and the core 1 stack watermark.
 *
 * Implements docs/plans/BLE_TRANSPORT_PLAN.md section 5.8 (task A9).
 *
 * <b>Boot sequence with {@code TG_BLE_CORE=0}</b> (override, stock framework
 * variant; the default build uses core 1, decided by HW-19 on 2026-11-23):
 * 1. {@code main()} on core 0 calls the variant's {@code initVariant()}, which
 *    calls {@code init_cyw43_wifi()}. The threadsafe-background async context
 *    of the CYW43 is bound to core 0.
 * 2. {@code setup()} calls {@link ble_core_led_init} (registers the LED
 *    worker) and later {@link ble_control_begin}; BT work runs as low-priority
 *    IRQ work on core 0.
 *
 * <b>Boot sequence with {@code TG_BLE_CORE=1}</b> (default; {@code [ble]} and
 * {@code [board_pico2w]}; variant {@code variants/rpipico2w_tg} has an empty
 * {@code initVariant()}).
 * Every step was checked against pico-sdk and arduino-pico 5.5.1 sources
 * ({@code cores/rp2040/main.cpp}, {@code cyw43_wrappers.cpp},
 * {@code async_context_threadsafe_background.c},
 * {@code cyw43_arch_threadsafe_background.c}, {@code cyw43_driver.c},
 * {@code btstack_cyw43.c}); nothing was run on hardware (plan HW-19, CORE-05).
 * 1. {@code main()} (core 0): {@code initVariant()} does nothing, so the
 *    CYW43 does not exist yet and {@code cyw43_arch_async_context()} is null.
 *    TinyUSB starts, then {@code core1_separate_stack_address} is
 *    {@code malloc(0x2000)}-ed and core 1 is launched into {@code main1()}
 *    (the project defines {@code setup1()}, so this happens), then
 *    {@code setup()} runs. Core 1 therefore starts before core 0's
 *    {@code setup()}.
 * 2. {@code setup1()} (core 1) calls {@link ble_core_setup1}. The stack is
 *    painted.
 * 3. {@code init_cyw43_wifi()} runs on core 1: {@code CheckPicoW()} is true on
 *    RP2350, then {@code cyw43_arch_init_with_country()} ->
 *    {@code cyw43_arch_init()}. There is no context yet, so
 *    {@code async_context_threadsafe_background_init()} records
 *    {@code core_num = get_core_num() = 1}. The default alarm pool belongs to
 *    core 0, so it takes the {@code
 * alarm_pool_create_with_unused_hardware_alarm()} branch and creates its own
 * pool on a free hardware alarm (F-22; it needs one free alarm, otherwise the
 * init fails and the null check below triggers). {@code low_prio_irq_init()}
 * claims a user IRQ and enables it in core 1's NVIC. The init-time asserts in
 * {@code cyw43_driver.c} compare
 *    {@code get_core_num()} with the context's core and hold because
 *    {@code cyw43_driver_init()}, {@code cyw43_irq_init()} (through
 *    {@code async_context_execute_sync()}, which short-circuits on the
 *    context's own core) and {@code btstack_cyw43_init()} all run here.
 *    {@code cyw43_irq_init()} installs the WL_D host-wake GPIO handler and
 *    enables {@code IO_IRQ_BANK0} on core 1. The gSPI driver claims its PIO
 *    state machine and two DMA channels, before PCM1808 capture on core 0
 *    claims its own (F-23); PCM1808 uses "claim free" and takes what is left.
 * 4. If {@code cyw43_arch_async_context()} is null the init failed: the LED
 *    stays off, BLE is not started, core 0 times out in {@link ble_core_hold}.
 * 5. The LED worker is registered ({@code
 * async_context_add_when_pending_worker()} takes the context lock, uncontended
 * here) and marked pending. In BT context on core 1 it brings the chip up on
 * first use
 *    ({@code cyw43_ensure_up()}: firmware download, using busy waits because
 *    it runs in an IRQ) and applies the initial LED state.
 * 6. Core 1 waits until core 0 has entered {@link ble_core_hold} (a flag set
 *    after {@code usb_audio_init()}). This closes a gap in plan section 5.8
 *    rule 6: without it {@code BLE.startAdvertising()} could print the UUID
 *    on the CDC port (F-12) from core 1 while core 0 is still inside
 *    {@code usb_audio_init()}. Core 1 runs steps 3 to 5 meanwhile, which do
 *    not touch USB.
 * 7. {@link ble_control_begin} runs on core 1: {@code BLE.begin()} powers the
 *    controller (the context's IRQ on core 1 services the boot), the
 *    {@code BluetoothLock} blocks (it is acquired on the context's own core,
 *    so releasing it runs pending BT work inline on core 1), advertising
 *    starts once and the TX worker is registered on core 1's context.
 *    {@code ble_status_t::core} records 1.
 * 8. {@link ble_core_setup1} sets "radio ready"; core 0 leaves the hold and
 *    resumes {@code setup()} (serial CLI init, LED on, then {@code loop()}).
 *    {@code setup1()} then starts the display (when built) and
 *    {@code loop1()} runs it or idles in {@code __wfe()}; BT IRQs still run
 *    on core 1.
 *
 * <b>Cross-core rules relied on.</b> {@code async_context_set_work_pending()}
 * from core 0 is lock-free: it arms an alarm on core 1's alarm pool, whose
 * handler wakes the context (F-22). {@code loop()} never takes the context
 * lock. The only cross-core lock use is in setup: step 5 (core 1 only) and
 * {@link ble_core_led_init}.
 *
 * <b>Contexts.</b>
 * - Main loop (core 0): {@link ble_core_led_set}, {@link ble_core_hold},
 *   {@link ble_core_core1_stack_free}. No BTstack call, no lock.
 * - BT context (core of the radio): the LED worker, the only caller of
 *   {@code cyw43_arch_gpio_put()} (CORE-02 allow-list).
 * - Core 1 thread: {@link ble_core_setup1}.
 */

#include "ble_core.h"

#include "board_config.h"

#if BLE_CORE_RADIO_ON_CORE1
#include "ble_control.h"
#endif

#if STATUS_LED_CYW43 || BLE_CORE_RADIO_ON_CORE1
#include <Arduino.h>
#include <pico/async_context.h>
#include <pico/cyw43_arch.h>
#endif

#if BLE_CORE_RADIO_ON_CORE1
#include <cyw43_wrappers.h>
#include <pico/stdlib.h>
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>

#if BLE_CORE_RADIO_ON_CORE1
/** Base of core 1's heap stack, set by {@code main()} before core 1 starts. */
extern "C" std::uint32_t *core1_separate_stack_address;
#endif

namespace {

#if STATUS_LED_CYW43

/**
 * CYW43 GPIO number of the LED. {@code LED_BUILTIN} is a pseudo pin (64 + GPIO)
 * that {@code cyw43_digitalWrite()} maps with {@code pin - 64}.
 */
constexpr std::uint32_t kCyw43LedGpio =
    static_cast<std::uint32_t>(LED_BUILTIN) - 64u;

/** Wanted LED state; written by {@link ble_core_led_set}, read by the worker.
 */
std::atomic<bool> g_led_want{false};
/** True once the worker is registered on the context (release/acquire). */
std::atomic<bool> g_led_ready{false};
/** Async context of the radio; written before {@code g_led_ready} is set. */
async_context_t *g_led_ctx = nullptr;

/**
 * LED worker. Runs in BT context (core of the radio), where taking the
 * context lock again is a recursive acquire and the SPI transfer is in the
 * right place. Reads the latest wanted state, so several requests coalesce.
 *
 * @param context unused
 * @param worker unused
 */
void led_work(async_context_t *context, async_when_pending_worker_t *worker) {
  static_cast<void>(context);
  static_cast<void>(worker);
  cyw43_arch_gpio_put(kCyw43LedGpio,
                      g_led_want.load(std::memory_order_relaxed));
}

/** LED worker registration record. */
async_when_pending_worker_t g_led_worker = {nullptr, led_work, false, nullptr};

/**
 * Registers the LED worker on the CYW43 context and applies the wanted state.
 * Setup time only: the registration takes the context lock.
 *
 * @return false if the CYW43 context does not exist
 */
bool led_start() {
  async_context_t *ctx = cyw43_arch_async_context();
  if (ctx == nullptr) {
    return false;
  }
  g_led_ctx = ctx;
  if (!async_context_add_when_pending_worker(ctx, &g_led_worker)) {
    return false;
  }
  g_led_ready.store(true, std::memory_order_release);
  async_context_set_work_pending(ctx, &g_led_worker);
  return true;
}

#endif // STATUS_LED_CYW43

#if BLE_CORE_RADIO_ON_CORE1

/** Paint word for the core 1 stack watermark. */
constexpr std::uint32_t kStackPaint = 0xC0DE5AFEu;
/** Core 1 stack size: {@code malloc(0x2000)} in {@code cores/rp2040/main.cpp}.
 */
constexpr std::size_t kStackBytes = 0x2000;
/** Bytes below the current frame left unpainted (this function's callees). */
constexpr std::size_t kPaintGuardBytes = 256;

/** Set by core 0 when it is inside {@link ble_core_hold}. */
std::atomic<bool> g_core0_holding{false};
/** Set by core 1 when {@link ble_control_begin} has returned. */
std::atomic<bool> g_radio_ready{false};
/** True when the stack was painted and the watermark is valid. */
std::atomic<bool> g_stack_painted{false};

/**
 * Paints the unused part of core 1's heap stack with {@link kStackPaint}.
 * Call early on core 1. Does nothing if the thread does not run on the
 * separate stack ({@code core1_separate_stack} false).
 */
__attribute__((noinline)) void paint_core1_stack() {
  std::uint32_t *const base = core1_separate_stack_address;
  if (base == nullptr) {
    return;
  }
  const auto lo = reinterpret_cast<std::uintptr_t>(base);
  const auto hi = lo + kStackBytes;
  const auto sp = reinterpret_cast<std::uintptr_t>(__builtin_frame_address(0));
  if (sp <= lo + kPaintGuardBytes || sp > hi) {
    return;
  }
  volatile std::uint32_t *p = base;
  const volatile std::uint32_t *const end =
      reinterpret_cast<volatile std::uint32_t *>(sp - kPaintGuardBytes);
  while (p < end) {
    *p++ = kStackPaint;
  }
  g_stack_painted.store(true, std::memory_order_release);
}

#endif // BLE_CORE_RADIO_ON_CORE1

} // namespace

#if BLE_CORE_RADIO_ON_CORE1

extern "C" void ble_core_setup1(void) {
  paint_core1_stack();
  init_cyw43_wifi();
  if (cyw43_arch_async_context() == nullptr) {
    return;
  }
#if STATUS_LED_CYW43
  static_cast<void>(led_start());
#endif
  while (!g_core0_holding.load(std::memory_order_acquire)) {
    sleep_ms(1);
  }
  ble_control_begin();
  g_radio_ready.store(true, std::memory_order_release);
}

extern "C" bool ble_core_hold(void) {
  g_core0_holding.store(true, std::memory_order_release);
  return ble_core::wait_radio_ready(g_radio_ready, ble_core::kHoldCapMs, [] {
    return static_cast<std::uint32_t>(millis());
  });
}

extern "C" std::uint32_t ble_core_core1_stack_free(void) {
  if (!g_stack_painted.load(std::memory_order_acquire)) {
    return 0;
  }
  const volatile std::uint32_t *p = core1_separate_stack_address;
  std::size_t words = 0;
  while (words < kStackBytes / sizeof(std::uint32_t) &&
         p[words] == kStackPaint) {
    ++words;
  }
  return static_cast<std::uint32_t>(words * sizeof(std::uint32_t));
}

#else

extern "C" std::uint32_t ble_core_core1_stack_free(void) { return 0; }

#endif // BLE_CORE_RADIO_ON_CORE1

#if STATUS_LED_CYW43

extern "C" void ble_core_led_init(void) {
#if !BLE_CORE_RADIO_ON_CORE1
  static_cast<void>(led_start());
#endif
}

extern "C" void ble_core_led_set(bool on) {
  g_led_want.store(on, std::memory_order_relaxed);
  if (g_led_ready.load(std::memory_order_acquire)) {
    async_context_set_work_pending(g_led_ctx, &g_led_worker);
  }
}

#else

extern "C" void ble_core_led_init(void) {}

#endif // STATUS_LED_CYW43
