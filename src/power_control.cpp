/**
 * @file power_control.cpp
 * @brief TPS22918 BOOST_EN GPIO and nonblocking preamp settle timer.
 *
 * @see power_control.h
 */

#include "power_control.h"

#include "app_config.h"
#include "board_config.h"
#include "diagnostics.h"

#include <Arduino.h>

static bool g_gpio_ready;
static bool g_requested;
static bool g_actual_on;
static bool g_settling;
static bool g_ready;
static uint32_t g_enabled_at_ms;

void power_control_init(void) {
  g_requested = false;
  g_actual_on = false;
  g_settling = false;
  g_ready = false;
  g_enabled_at_ms = 0;
  g_gpio_ready = false;

#if BOOST_SWITCH_PRESENT
  /* Drive LOW before switching to output so the pin never glitches HIGH. */
  digitalWrite(BOOST_EN_GPIO, LOW);
  pinMode(BOOST_EN_GPIO, OUTPUT);
  digitalWrite(BOOST_EN_GPIO, LOW);
  g_gpio_ready = true;
#endif
}

void boost_power_enable(bool enable) {
  if (enable == g_requested && enable == g_actual_on) {
    return;
  }

  diag_counters_t *d = diagnostics_get();
  g_requested = enable;

#if BOOST_SWITCH_PRESENT
  if (g_gpio_ready) {
    digitalWrite(BOOST_EN_GPIO, enable ? HIGH : LOW);
  }
#endif

  if (enable != g_actual_on) {
    if (enable) {
      d->boost_on_count++;
    } else {
      d->boost_off_count++;
    }
  }
  g_actual_on = enable;

  if (!enable) {
    g_settling = false;
    g_ready = false;
    g_enabled_at_ms = 0;
  }
}

bool boost_power_requested(void) { return g_requested; }

bool boost_power_is_on(void) { return g_actual_on; }

void power_control_begin_preamp_warmup(uint32_t now_ms) {
#if !BOOST_SWITCH_PRESENT
  (void)now_ms;
  /* No load switch on this build — treat preamp rail as already available. */
  g_settling = false;
  g_ready = true;
  return;
#else
  if (g_ready) {
    return;
  }
  if (g_settling) {
    return;
  }

  diagnostics_get()->preamp_settle_start_count++;
  boost_power_enable(true);
  g_enabled_at_ms = now_ms;
  g_settling = true;
  g_ready = false;
#endif
}

void power_control_abort_preamp(uint32_t now_ms) {
  (void)now_ms;
  if (g_settling || g_ready || g_requested || g_actual_on) {
    if (g_settling) {
      diagnostics_get()->preamp_start_aborted_count++;
    }
  }
  g_settling = false;
  g_ready = false;
  g_enabled_at_ms = 0;
  boost_power_enable(false);
}

bool power_control_is_settling(void) { return g_settling; }

bool power_control_preamp_ready(void) {
#if !BOOST_SWITCH_PRESENT
  return true;
#else
  return g_ready;
#endif
}

bool power_control_poll(uint32_t now_ms) {
  if (!g_settling) {
    return false;
  }

  if ((now_ms - g_enabled_at_ms) < (uint32_t)PREAMP_SETTLE_MS) {
    return false;
  }

  g_settling = false;
  g_ready = true;
  diagnostics_get()->preamp_settle_done_count++;
  return true;
}
