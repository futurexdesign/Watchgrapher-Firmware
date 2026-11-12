/**
 * @file diagnostics.cpp
 * @brief Counter storage and optional CDC/UART status dumps.
 *
 * Periodic lines stay off until {@link diagnostics_set_print_enabled} /
 * CLI {@code diag on}. Suspend timing always runs.
 */

#include "diagnostics.h"

#include "app_config.h"
#include "audio_source.h"
#include "board_config.h"
#include "perf_timing.h"
#include "power_control.h"

#include <Arduino.h>
#include <climits>
#include <cstring>

static diag_counters_t g_diag;
static uint32_t g_suspend_entered_ms;
static bool g_suspend_timing_active;
static bool g_print_enabled;

void diagnostics_init(void) {
  memset(&g_diag, 0, sizeof(g_diag));
  g_diag.ring_min_occupancy = UINT32_MAX;
  g_diag.adc_min_code = UINT16_MAX;
  g_diag.adc_max_code = 0;
  g_print_enabled = false;

  /* CDC Serial is shared with the CLI; open early, do not spam a diag banner. */
  Serial.begin(115200);

#if DIAG_UART
  Serial1.setTX(DIAG_UART_TX_GPIO);
  Serial1.setRX(DIAG_UART_RX_GPIO);
  Serial1.begin(DIAG_UART_BAUD);
#endif
}

void diagnostics_set_print_enabled(bool on) { g_print_enabled = on; }

bool diagnostics_print_enabled(void) { return g_print_enabled; }

diag_counters_t *diagnostics_get(void) { return &g_diag; }

void diagnostics_note_ring_occupancy(uint32_t occ) {
  if (occ < g_diag.ring_min_occupancy) {
    g_diag.ring_min_occupancy = occ;
  }
  if (occ > g_diag.ring_max_occupancy) {
    g_diag.ring_max_occupancy = occ;
  }
}

void diagnostics_note_adc_code(uint16_t code) {
  if (code < g_diag.adc_min_code) {
    g_diag.adc_min_code = code;
  }
  if (code > g_diag.adc_max_code) {
    g_diag.adc_max_code = code;
  }
  g_diag.adc_mean_code_accum += code;
  g_diag.adc_mean_code_count++;
}

void diagnostics_reset_adc_stats(void) {
  g_diag.adc_min_code = UINT16_MAX;
  g_diag.adc_max_code = 0;
  g_diag.adc_mean_code_accum = 0;
  g_diag.adc_mean_code_count = 0;
  g_diag.adc_dc_counts = 0;
  g_diag.adc_ac_peak_counts = 0;
  g_diag.adc_pcm_peak_abs = 0;
}

#if DIAG_UART || DIAG_CDC
static void diagnostics_print_line(Stream &out) {
  uint32_t mean = 0;
  if (g_diag.adc_mean_code_count > 0) {
    mean = static_cast<uint32_t>(g_diag.adc_mean_code_accum /
                                 g_diag.adc_mean_code_count);
  }
  uint16_t amin =
      g_diag.adc_min_code == UINT16_MAX ? static_cast<uint16_t>(0)
                                        : g_diag.adc_min_code;
  uint32_t ring_min =
      g_diag.ring_min_occupancy == UINT32_MAX ? 0u : g_diag.ring_min_occupancy;

  out.print(F("enum="));
  out.print(g_diag.usb_enumeration_count);
  out.print(F(" open="));
  out.print(g_diag.usb_audio_stream_open_count);
  out.print(F(" close="));
  out.print(g_diag.usb_audio_stream_close_count);
  out.print(F(" sus="));
  out.print(g_diag.usb_suspend_count);
  out.print(F(" res="));
  out.print(g_diag.usb_resume_count);
  out.print(F(" boost="));
  out.print(boost_power_is_on() ? 1 : 0);
  out.print(F(" pkt="));
  out.print(g_diag.usb_in_packet_count);
  out.print(F(" und="));
  out.print(g_diag.pcm_ring_underrun_count);
  out.print(F(" ring["));
  out.print(ring_min);
  out.print(',');
  out.print(g_diag.ring_max_occupancy);
  out.print(F("] adc["));
  out.print(amin);
  out.print(',');
  out.print(g_diag.adc_max_code);
  out.print(F("] mean="));
  out.print(mean);
  out.print(F(" dc="));
  out.print(g_diag.adc_dc_counts);
#if AUDIO_SOURCE == PCM1808
  out.print(F(" pcm clk_ok="));
  out.print(g_diag.pcm1808_clock_config_ok);
  out.print(F(" sys="));
  out.print(g_diag.pcm1808_clk_sys_hz);
  out.print(F(" blocks="));
  out.print(g_diag.pcm1808_dma_blocks);
  out.print(F(" mono="));
  out.print(g_diag.pcm1808_mono_samples_emitted);
  out.print(F(" resync="));
  out.print(g_diag.pcm1808_resync_count);
  out.print(F(" rawL="));
  out.print(g_diag.pcm1808_raw_left_peak_abs);
  out.print(F(" rawR="));
  out.print(g_diag.pcm1808_raw_right_peak_abs);
  out.print(F(" pk="));
  out.print(g_diag.pcm1808_sample_peak_abs);
  out.print(F(" sat="));
  out.print(g_diag.pcm1808_saturated_count);
#endif
  {
    dsp_timing_t t{};
    perf_timing_get(&t);
    out.print(F(" dsp_us="));
    out.print(t.dsp_block_us_last);
    out.print('/');
    out.print(t.dsp_block_us_mean);
    out.print('/');
    out.print(t.dsp_block_us_max);
    out.print(F(" bud="));
    out.print(t.dsp_block_budget_us);
    out.print(F(" busy="));
    out.print(t.loop_busy_pct);
    out.print('%');
    out.print(F(" short="));
    out.print(t.short_writes);
  }
  out.println();

  g_diag.adc_min_code = UINT16_MAX;
  g_diag.adc_max_code = 0;
  g_diag.adc_mean_code_accum = 0;
  g_diag.adc_mean_code_count = 0;
  g_diag.adc_ac_peak_counts = 0;
  g_diag.adc_pcm_peak_abs = 0;
  g_diag.ring_min_occupancy = UINT32_MAX;
  g_diag.ring_max_occupancy = 0;
  g_diag.pcm1808_raw_left_peak_abs = 0;
  g_diag.pcm1808_raw_right_peak_abs = 0;
  g_diag.pcm1808_sample_peak_abs = 0;
  perf_timing_clear_window_max();
}
#endif

void diagnostics_poll(void) {
  uint32_t now = millis();
  if (audio_source_state() == STREAM_STATE_USB_SUSPENDED) {
    if (!g_suspend_timing_active) {
      g_suspend_entered_ms = now;
      g_suspend_timing_active = true;
    }
  } else if (g_suspend_timing_active) {
    g_diag.usb_suspend_duration_ms += (now - g_suspend_entered_ms);
    g_suspend_timing_active = false;
  }

  if (!g_print_enabled) {
    return;
  }

#if DIAG_UART || DIAG_CDC
  constexpr uint32_t kPrintIntervalMs = 2000u;
  static uint32_t last_ms = 0;
  if ((now - last_ms) < kPrintIntervalMs) {
    return;
  }
#if DIAG_CDC
  if (Serial.availableForWrite() < 128) {
    return;
  }
#endif
  last_ms = now;
#if DIAG_UART
  diagnostics_print_line(Serial1);
#endif
#if DIAG_CDC
  diagnostics_print_line(Serial);
#endif
#endif
}
