/**
 * @file pcm1808_clock.cpp
 * @brief PCM1808 SCKI (MCLK) generation from PLL_SYS via GPOUT0.
 *
 * @see pcm1808_clock.h
 */

#include "pcm1808_clock.h"

#include "app_config.h"
#include "board_config.h"
#include "diagnostics.h"

#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include <Arduino.h>

#if AUDIO_SOURCE == PCM1808

static bool g_started;
static bool g_ok;
static uint32_t g_ready_ms;

bool pcm1808_clock_start(void) {
  diag_counters_t *d = diagnostics_get();

  uint32_t sys_hz = clock_get_hz(clk_sys);
  uint32_t usb_hz = clock_get_hz(clk_usb);
  g_ok = (sys_hz == PCM1808_EXPECT_SYS_HZ) && (usb_hz == PCM1808_EXPECT_USB_HZ);
  d->pcm1808_clock_config_ok = g_ok ? 1u : 0u;
  d->pcm1808_mclk_nominal_hz = PCM1808_MCLK_NOMINAL_HZ;

  if (!g_ok) {
    /* Fail-fast diagnostic: do not reprogram PLL_SYS at runtime. */
    g_started = false;
    return false;
  }

  /*
   * GPOUT0 from PLL_SYS: div_int=12, frac16=0x3500.
   * RP2350 uses 16-bit frac (0x3500/65536); RP2040 hardware keeps 8-bit frac
   * and the SDK applies (frac16>>8)=0x35 → 53/256 — same 12.20703125 ratio.
   * 150e6 / 12.20703125 ≈ 12.288000 MHz average. Edge jitter is expected.
   */
  clock_gpio_init_int_frac16(
      (uint)PCM1808_MCLK_GPIO,
      CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
      12u,
      0x3500u);

  g_started = true;
  /* Warmup deadline: ≥250 ms of stable MCLK before claiming capture ready. */
  g_ready_ms = millis() + 250u;
  return true;
}

void pcm1808_clock_stop(void) {
  if (!g_started) {
    return;
  }
  clock_stop(clk_gpout0);
  gpio_set_function((uint)PCM1808_MCLK_GPIO, GPIO_FUNC_SIO);
  gpio_set_dir((uint)PCM1808_MCLK_GPIO, GPIO_IN);
  g_started = false;
}

bool pcm1808_clock_is_ok(void) { return g_ok && g_started; }

uint32_t pcm1808_clock_nominal_hz(void) { return PCM1808_MCLK_NOMINAL_HZ; }

uint32_t pcm1808_clock_ready_deadline_ms(void) { return g_ready_ms; }

#else

bool pcm1808_clock_start(void) { return true; }
void pcm1808_clock_stop(void) {}
bool pcm1808_clock_is_ok(void) { return false; }
uint32_t pcm1808_clock_nominal_hz(void) { return 0; }
uint32_t pcm1808_clock_ready_deadline_ms(void) { return 0; }

#endif
