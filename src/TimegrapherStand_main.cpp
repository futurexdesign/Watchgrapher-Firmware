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
 * core 1 is held in reset.
 */

#include "app_config.h"
#include "audio_source.h"
#include "board_config.h"
#include "control_api.h"
#include "diagnostics.h"
#include "display_ui.h"
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

#if STATUS_LED_GPIO >= 0
static void led_set(bool on) { digitalWrite(STATUS_LED_GPIO, on ? HIGH : LOW); }
#else
static void led_set(bool) {}
#endif

/** Display redraws at 2 Hz; publishing at 10 Hz keeps it current. */
constexpr uint32_t kUiPublishMs = 100u;

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
  s.streaming = usb_audio_streaming();
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
  s.published_ms = now;
  ui_snapshot_publish(&s);
}

#if DISPLAY_SH1107
/* Core 1: 8 KB heap stack instead of the 4 KB SCRATCH_X default (GFX + printf). */
bool core1_separate_stack = true;

void setup1() { display_ui_setup(); }

void loop1() { display_ui_loop(); }
#endif

void setup() {
#if !DISPLAY_SH1107
  /* No display: nothing runs on core 1, keep it in reset. (With a display,
   * Arduino-Pico has already started core 1 before setup() runs.) */
  multicore_reset_core1();
#endif
  power_control_init();

#if STATUS_LED_GPIO >= 0
  pinMode(STATUS_LED_GPIO, OUTPUT);
  led_set(false);
#endif

  diagnostics_init();
  /* DSP graph before capture: capture init sets the graph's source rate. */
  control_api_init();
  timebase_init(); /* stored clock calibration (+ PPS if built in) */
  audio_source_init();

  if (!usb_audio_init()) {
    while (true) {
      led_set(true);
      delay(50);
      led_set(false);
      delay(50);
    }
  }

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
  diagnostics_poll();
  timebase_poll();
  publish_ui_snapshot();

#if STATUS_LED_GPIO >= 0
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
