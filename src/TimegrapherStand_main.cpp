/**
 * @file TimegrapherStand_main.cpp
 * @brief Arduino entry: power default-OFF, USB audio loop, CDC CLI.
 *
 * Core 0 (setup / loop) owns everything time-critical: TinyUSB, capture, the
 * DSP graph, CLI, HID. Loop order: TinyUSB → audio_source → usb_audio → CLI
 * → HID → diagnostics → timebase → UI snapshot → LED.
 *
 * Core 1 (setup1 / loop1) runs only the OLED readout when DISPLAY_SH1107=1,
 * reading values core 0 publishes through ui_snapshot. Without a display
 * core 1 is held in reset. With TG_BLE_CORE=1 core 1 also brings up the radio
 * (ble_core.cpp) and runs the BT work, with or without a display.
 */

#include "app_config.h"
#include "audio_source.h"
#include "ble_core.h"
#if TG_BLE
#include "ble_control.h"
#endif
#include "board_config.h"
#include "control_api.h"
#include "device_mode.h"
#include "diagnostics.h"
#include "display_ui.h"
#include "flash_store.h"
#include "hid_control.h"
#include "perf_timing.h"
#include "power_control.h"
#include "serial_cli.h"
#include "timebase.h"
#include "ui_snapshot.h"
#include "usb_audio.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <pico/multicore.h>

/**
 * Drives the status LED. On Pico 2 W the LED is a CYW43439 GPIO. A direct
 * write would take the CYW43 async-context lock from loop() (F-25), so the
 * request goes to an LED worker in BT context instead (ble_core.cpp, plan
 * section 5.8 rule 4).
 *
 * @param on true to light the LED
 */
#if STATUS_LED_CYW43
static void led_set(bool on) { ble_core_led_set(on); }
#elif STATUS_LED_GPIO >= 0
static void led_set(bool on) { digitalWrite(STATUS_LED_GPIO, on ? HIGH : LOW); }
#else
static void led_set(bool) {}
#endif

/** Display redraws at 2 Hz; publishing at 10 Hz keeps it current. */
constexpr uint32_t kUiPublishMs = 100u;

/**
 * Fills the Bluetooth badge fields of the UI snapshot (plan section 5.6) from
 * {@code ble_control_status()}. Core 0 only; core 1 reads just the snapshot.
 * The badge shows only once the GATT service is up ({@code started}): a
 * core-1 bring-up that timed out (plan 5.8 rule 6) shows no badge.
 * "Connected" means a central subscribed to TX, as in {@code getLink}.
 */
static void publish_ble_badge(ui_snapshot_t &s) {
  s.ble_built = TG_BLE != 0;
#if TG_BLE
  ble_status_t st;
  ble_control_status(&st);
  s.ble_enabled = st.started && st.enabled;
  s.ble_connected = s.ble_enabled && st.connected && st.cccd;
#endif
}

/** Copies the values the display shows into the core 0 → core 1 snapshot. */
static void publish_ui_snapshot(void) {
  static uint32_t last_ms;
  const uint32_t now = millis();
  if (now - last_ms < kUiPublishMs) {
    return;
  }
  last_ms = now;
  dsp_beat_rate_t b{};
  control_api_get_beat_rate(&b);
  ui_snapshot_t s{};
  s.streaming = usb_audio_streaming() || audio_source_capture_running();
  s.standalone = device_mode_get() == DEVICE_MODE_STANDALONE;
  s.timebase_ok = b.timebase_ok;
  s.locked = b.locked;
  s.vph = b.vph_nearest;
  s.events_seen = b.events_seen;
  s.avg = b.avg_long;
  s.avg_s = b.avg_s;
  s.lift_angle_deg = b.lift_angle_deg;
  timebase_info_t tb{};
  timebase_get(&tb);
  s.pps_enabled = tb.pps_enabled;
  s.pps_locked = tb.pps_locked;
  publish_ble_badge(s);
  dsp_quartz_t q{};
  control_api_get_quartz(&q);
  s.quartz.mode = q.mode;
  s.quartz.state = q.state;
  s.quartz.rate_s_day = q.rate_s_day;
  s.quartz.rate_valid = q.rate_valid;
  s.quartz.raw_rate_s_day = q.motor.rate_without_s_day;
  s.quartz.raw_rate_valid = q.motor.valid;
  s.quartz.inhibition_period_s = q.motor.inhibition_period_s;
  s.quartz.inhibition_period_valid = q.motor.inhibition_period_valid;
  s.quartz.jump_us = q.motor.jump_last_us;
  s.quartz.jump_n = q.motor.jump_last_n;
  s.quartz.inhibition = q.motor.inhibition;
  s.quartz.hint_weak_signal = q.motor.hint_weak_signal;
  s.quartz.hint_clipped = q.motor.hint_clipped;
  s.quartz.calibrated = q.calibrated;
  s.quartz.elapsed_s = q.elapsed_s;
  s.quartz.gate_s = q.gate_s;
  /* Only the active mode's detail struct is filled, so these pick by mode. */
  if (q.mode == DSP_QUARTZ_MODE_LCD) {
    s.quartz.ref_hz = static_cast<float>(q.lcd.f0_hz);
    s.quartz.blocks = q.lcd.blocks;
    s.quartz.blocks_valid = q.lcd.f0_hz > 0u;
    s.quartz.hint_clipped = q.lcd.clipped;
  } else if (q.mode == DSP_QUARTZ_MODE_FORK) {
    s.quartz.ref_hz = q.fork.f_nom_hz;
    s.quartz.blocks = q.fork.blocks;
    s.quartz.blocks_valid = q.fork.f_nom_hz > 0.f;
    s.quartz.hint_clipped = q.fork.clipped;
  }
  s.published_ms = now;
  ui_snapshot_publish(&s);
}

#if DISPLAY_SH1107 || BLE_CORE_RADIO_ON_CORE1
/* Core 1: 8 KB heap stack instead of the 4 KB SCRATCH_X default (GFX + printf,
 * and BT IRQ work when the radio runs here). */
bool core1_separate_stack = true;

void setup1() {
#if BLE_CORE_RADIO_ON_CORE1
  ble_core_setup1(); /* CYW43 + BLE on core 1; before the display (plan 5.8) */
#endif
#if DISPLAY_SH1107
  display_ui_setup();
#endif
}

void loop1() {
#if DISPLAY_SH1107
  display_ui_loop();
#else
  __wfe(); /* no display: idle; BT IRQs still run on this core */
#endif
}
#endif

void setup() {
#if !DISPLAY_SH1107 && !BLE_CORE_RADIO_ON_CORE1
  /* No display and no radio on core 1: nothing runs there, keep it in reset.
   * (With setup1() defined, Arduino-Pico has already started core 1 before
   * setup() runs.) */
  multicore_reset_core1();
#endif
  power_control_init();

#if STATUS_LED_CYW43
  /* pinMode(LED_BUILTIN) does nothing for a CYW43 pin; the LED worker is set
   * up here (or on core 1, TG_BLE_CORE=1) and applies the initial state. */
  ble_core_led_init();
  led_set(false);
#elif STATUS_LED_GPIO >= 0
  pinMode(STATUS_LED_GPIO, OUTPUT);
  led_set(false);
#endif

  diagnostics_init();
  /* DSP graph before capture: capture init sets the graph's source rate. */
  control_api_init();
  flash_store_init(); /* flash image first: timebase and device mode read it */
  timebase_init();    /* stored clock calibration (+ PPS if built in) */
  device_mode_init(); /* persisted mode (USB mic / standalone) */
  audio_source_init();
  device_boot_run(); /* STANDALONE: start measuring without a host stream */

  if (!usb_audio_init()) {
    while (true) {
      led_set(true);
      delay(50);
      led_set(false);
      delay(50);
    }
  }

#if BLE_CORE_RADIO_ON_CORE1
  /* Core 1 runs ble_control_begin(); wait for it (plan 5.8 rule 6). On
   * timeout setup() continues and ble_status_t::started stays false. */
  static_cast<void>(ble_core_hold());
#elif TG_BLE
  ble_control_begin();
#endif
  serial_cli_init();
  led_set(true);
}

void loop() {
  const uint32_t t0 = time_us_32();

  TinyUSBDevice.task();
  audio_source_process();
  usb_audio_task();
  serial_cli_tick();
  hid_control_task();
#if TG_BLE
  ble_control_task();
#endif
  diagnostics_poll();
  timebase_poll();
  publish_ui_snapshot();

#if STATUS_LED_CYW43 || STATUS_LED_GPIO >= 0
  static uint32_t last;
  static bool on;
  uint32_t now = millis();

  if (audio_source_state() == STREAM_STATE_USB_SUSPENDED) {
    if (on) {
      on = false;
      led_set(false);
    }
    perf_timing_note_loop(time_us_32() - t0);
    return;
  }

  uint32_t period_ms = 1000;
  if (usb_audio_streaming()) {
    period_ms = 100;
  } else if (TinyUSBDevice.mounted()) {
    period_ms = 500;
  }
  if ((now - last) >= period_ms) {
    last = now;
    on = !on;
    led_set(on);
  }
#endif
  perf_timing_note_loop(time_us_32() - t0);
}
