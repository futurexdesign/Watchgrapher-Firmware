/**
 * @file TimegrapherStand_main.cpp
 * @brief Arduino entry: power default-OFF, USB audio loop, CDC CLI.
 *
 * Loop order: TinyUSB → audio_source → usb_audio → CLI → diagnostics → LED.
 */

#include "app_config.h"
#include "audio_source.h"
#include "board_config.h"
#include "diagnostics.h"
#include "dsp_graph.h"
#include "power_control.h"
#include "serial_cli.h"
#include "usb_audio.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <pico/multicore.h>

#if STATUS_LED_GPIO >= 0
static void led_set(bool on) { digitalWrite(STATUS_LED_GPIO, on ? HIGH : LOW); }
#else
static void led_set(bool) {}
#endif

void setup() {
  multicore_reset_core1();
  power_control_init();

#if STATUS_LED_GPIO >= 0
  pinMode(STATUS_LED_GPIO, OUTPUT);
  led_set(false);
#endif

  diagnostics_init();
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
  /*
   * serial_cli_init → control_api_init → dsp_graph_init resets analysis rate.
   * Re-assert PCM1808 pre-upsample timing if that backend is active.
   */
#if AUDIO_SOURCE == PCM1808
  dsp_graph_set_source_rate(APP_SAMPLE_RATE_HZ / (uint32_t)PCM1808_OUTPUT_UPSAMPLE);
#endif
  led_set(true);
}

void loop() {
  TinyUSBDevice.task();
  audio_source_process();
  usb_audio_task();
  serial_cli_tick();
  diagnostics_poll();

#if STATUS_LED_GPIO >= 0
  static uint32_t last;
  static bool on;
  uint32_t now = millis();

  if (audio_source_state() == STREAM_STATE_USB_SUSPENDED) {
    if (on) {
      on = false;
      led_set(false);
    }
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
}
