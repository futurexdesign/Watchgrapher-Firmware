/**
 * @file TimegrapherStand_main.cpp
 * @brief Arduino entry points: bring-up order and main cooperative loop.
 *
 * Init order matters: diagnostics → audio source (backend + ring) → USB audio
 * (descriptors + re-enumerate). The loop services TinyUSB, fills the PCM ring,
 * feeds ISO IN, and emits diagnostic lines.
 */

#include "adc_capture.h"
#include "audio_source.h"
#include "board_config.h"
#include "diagnostics.h"
#include "usb_audio.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <pico/multicore.h>

#if STATUS_LED_GPIO >= 0
/**
 * Drives the board status LED.
 *
 * @param on {@code true} for illuminated (active level depends on wiring;
 *           Pico LED is active-high via {@code digitalWrite HIGH}).
 */
static void led_set(bool on) { digitalWrite(STATUS_LED_GPIO, on ? HIGH : LOW); }
#else
static void led_set(bool) {}
#endif

/**
 * One-time firmware setup: LED, diagnostics, capture backend, UAC2 mic.
 *
 * TinyUSB device init already ran in the core; we register the mic interface
 * and re-attach so the host sees the composite CDC + audio configuration.
 * Failure to register the interface enters a rapid LED blink fault loop.
 */
void setup() {
  /*
   * Hold core 1 in reset. Arduino-Pico only launches it if setup1()/loop1()
   * exist (we do not); this makes the low-power park explicit. Do not use
   * rp2040.idleOtherCore() here — that busy-waits the peer in RAM and is for
   * flash-write safety, not minimum power. Do not add a WFI loop1() either;
   * starting core 1 costs more than leaving it reset.
   */
  multicore_reset_core1();

#if STATUS_LED_GPIO >= 0
  pinMode(STATUS_LED_GPIO, OUTPUT);
  led_set(false);
#endif

  diagnostics_init();
  audio_source_init();

  /*
   * TinyUSB_Device_Init() already ran from the core with CFG_TUD_CDC=0
   * (empty config). Register the UAC2 mic interface and re-enumerate.
   */
  if (!usb_audio_init()) {
    /* Hard fault indicator: rapid blink if LED present. */
    while (true) {
      led_set(true);
      delay(50);
      led_set(false);
      delay(50);
    }
  }

  led_set(true);
}

/**
 * Cooperative main loop. Heartbeat LED rate encodes USB state:
 * unconfigured ~1 Hz, mounted ~2 Hz, streaming ~10 Hz.
 */
void loop() {
  TinyUSBDevice.task();
  audio_source_process();
  usb_audio_task();
  diagnostics_poll();

#if STATUS_LED_GPIO >= 0
  static uint32_t last;
  static bool on;
  uint32_t now = millis();
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
