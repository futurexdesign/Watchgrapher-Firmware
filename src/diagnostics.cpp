/**
 * @file diagnostics.cpp
 * @brief Counter storage and periodic CDC/UART status dumps.
 *
 * @see diagnostics.h for field meanings and windowing rules.
 */

#include "diagnostics.h"

#include "app_config.h"
#include "board_config.h"

#include <limits.h>
#include <string.h>

#if DIAG_UART || DIAG_CDC
#include <Arduino.h>
#endif

static diag_counters_t g_diag;

void diagnostics_init(void) {
  memset(&g_diag, 0, sizeof(g_diag));
  g_diag.ring_min_occupancy = UINT32_MAX;
  g_diag.adc_min_code = UINT16_MAX;
  g_diag.adc_max_code = 0;

#if DIAG_UART
  Serial1.setTX(DIAG_UART_TX_GPIO);
  Serial1.setRX(DIAG_UART_RX_GPIO);
  Serial1.begin(DIAG_UART_BAUD);
  Serial1.println(F("diag: uart ready"));
  Serial1.print(F("board="));
  Serial1.print(BOARD_NAME);
  Serial1.print(F(" source="));
  Serial1.println(AUDIO_SOURCE_NAME);
#endif
#if DIAG_CDC
  /* CDC is brought up by the Adafruit TinyUSB core; just tag the stream. */
  Serial.begin(115200);
  Serial.print(F("diag: cdc board="));
  Serial.print(BOARD_NAME);
  Serial.print(F(" source="));
  Serial.println(AUDIO_SOURCE_NAME);
#endif
}

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
/**
 * Formats one status line and clears windowed ADC/ring fields afterward
 * so the next poll reflects only the subsequent ~2 s. {@code dc=} is kept
 * so the learned midrail remains visible across windows.
 *
 * @param out CDC ({@code Serial}) or UART ({@code Serial1}) stream.
 */
static void diagnostics_print_line(Stream &out) {
  /* Windowed mean — accum is cleared each print so uint32 cannot wrap. */
  uint32_t mean = 0;
  if (g_diag.adc_mean_code_count > 0) {
    mean = (uint32_t)(g_diag.adc_mean_code_accum / g_diag.adc_mean_code_count);
  }

  uint16_t amin =
      g_diag.adc_min_code == UINT16_MAX ? (uint16_t)0 : g_diag.adc_min_code;
  uint16_t amax = g_diag.adc_max_code;
  uint32_t ac_pk = g_diag.adc_ac_peak_counts;
  uint32_t pcm_pk = g_diag.adc_pcm_peak_abs;
  uint32_t ring_min =
      g_diag.ring_min_occupancy == UINT32_MAX ? 0u : g_diag.ring_min_occupancy;
  uint32_t ring_max = g_diag.ring_max_occupancy;

  out.print(F("enum="));
  out.print(g_diag.usb_enumeration_count);
  out.print(F(" open="));
  out.print(g_diag.usb_audio_stream_open_count);
  out.print(F(" close="));
  out.print(g_diag.usb_audio_stream_close_count);
  out.print(F(" pkt="));
  out.print(g_diag.usb_in_packet_count);
  out.print(F(" dma="));
  out.print(g_diag.dma_complete_count);
  out.print(F(" und="));
  out.print(g_diag.pcm_ring_underrun_count);
  out.print(F(" ovr="));
  out.print(g_diag.pcm_ring_overrun_count);
  out.print(F(" dma_ovr="));
  out.print(g_diag.dma_overrun_count);
  out.print(F(" adc_err="));
  out.print(g_diag.adc_conversion_error_count);
  out.print(F(" ring["));
  out.print(ring_min);
  out.print(',');
  out.print(ring_max);
  out.print(F("] adc["));
  out.print(amin);
  out.print(',');
  out.print(amax);
  out.print(F("] mean="));
  out.print(mean);
  out.print(F(" dc="));
  out.print(g_diag.adc_dc_counts);
  out.print(F(" ac_pk="));
  out.print(ac_pk);
  out.print(F(" pcm_pk="));
  out.print(pcm_pk);
#if AUDIO_SOURCE == PCM1808
  out.print(F(" pcm clk_ok="));
  out.print(g_diag.pcm1808_clock_config_ok);
  out.print(F(" mclk="));
  out.print(g_diag.pcm1808_mclk_nominal_hz);
  out.print(F(" blocks="));
  out.print(g_diag.pcm1808_dma_blocks);
  out.print(F(" mono="));
  out.print(g_diag.pcm1808_mono_samples_emitted);
  out.print(F(" dma_ovr="));
  out.print(g_diag.pcm1808_dma_block_overruns);
  out.print(F(" resync="));
  out.print(g_diag.pcm1808_resync_count);
  out.print(F(" peak="));
  out.print(g_diag.pcm1808_sample_peak_abs);
  out.print(F(" sat="));
  out.print(g_diag.pcm1808_saturated_count);
#endif
  out.println();

  /*
   * adc[*]/mean/ac_pk/pcm_pk/ring[*] are last-window stats. Clear them so the
   * next line reflects the next ~2 s (dc= stays as learned midrail).
   */
  g_diag.adc_min_code = UINT16_MAX;
  g_diag.adc_max_code = 0;
  g_diag.adc_mean_code_accum = 0;
  g_diag.adc_mean_code_count = 0;
  g_diag.adc_ac_peak_counts = 0;
  g_diag.adc_pcm_peak_abs = 0;
  g_diag.ring_min_occupancy = UINT32_MAX;
  g_diag.ring_max_occupancy = 0;
}
#endif

void diagnostics_poll(void) {
#if DIAG_UART || DIAG_CDC
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if ((now - last_ms) < 2000u) {
    return;
  }

  /*
   * Skip rather than block if the host isn't draining CDC. A blocking
   * Serial.print stalls the main loop (no DMA drain / USB feed) and was a
   * contributor to PCM1808 underrun plateaus.
   */
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
