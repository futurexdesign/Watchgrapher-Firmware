/**
 * @file adc_capture.cpp
 * @brief DMA ping-pong SAR ADC capture with oversampling and PCM conversion.
 *
 * @see adc_capture.h for the public API contract.
 */

#include "adc_capture.h"

#include "app_config.h"
#include "audio_ring.h"
#include "audio_source.h"
#include "board_config.h"
#include "diagnostics.h"

#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/sync.h>

#include <cstring>

/**
 * Scales DC-centered ADC counts into a clamped PCM24 sample.
 *
 * {@code ADC_PCM_SCALE} is applied in the historical 16-bit domain, then the
 * result is multiplied by 256 (an 8-bit left shift, written as a multiply so
 * negative values are well-defined) into the signed 24-bit container.
 */
static int32_t adc_scale_centered(int32_t centered) {
  int32_t pcm = centered * static_cast<int32_t>(ADC_PCM_SCALE) * 256;
  if (pcm > APP_PCM24_FULL_SCALE) {
    pcm = APP_PCM24_FULL_SCALE;
  }
  if (pcm < APP_PCM24_MIN) {
    pcm = APP_PCM24_MIN;
  }
  return pcm;
}

#if AUDIO_SOURCE == ADC

static uint16_t g_dma_buf[APP_ADC_DMA_BUF_COUNT][APP_ADC_DMA_BUF_RAW_SAMPLES]
    __attribute__((aligned(4)));
static volatile bool g_buf_ready[APP_ADC_DMA_BUF_COUNT];
static volatile bool g_running;
static volatile uint32_t g_active;
static int g_dma_chan = -1;
static bool g_inited;
static uint16_t g_last_valid_code = static_cast<uint16_t>(ADC_DC_OFFSET_COUNTS);

/**
 * Frozen midrail for DC removal ({@code ADC_DC_REMOVE=1}): learned as the
 * mean of the first raw DMA block, then held for the rest of the stream.
 */
static int32_t g_dc_counts;
static bool g_dc_ready;
static int32_t g_lpf_y;
static bool g_lpf_primed;

/*
 * Per-block scratch for adc_capture_process (~4.8 KB). File-scope rather than
 * stack: core 0 only has a 4 KB stack (SCRATCH_Y). Main loop only.
 */
static uint16_t g_block_raw[APP_ADC_DMA_BUF_RAW_SAMPLES];
static audio_sample_t g_block_pcm[APP_ADC_DMA_BUF_SAMPLES];

/**
 * Pulls a 12-bit code from a FIFO word; on ERR (bit 15) reuses the last
 * valid code and bumps {@code adc_conversion_error_count}.
 *
 * @param fifo_word Raw ADC FIFO entry.
 * @return Code in 0..4095.
 */
static uint16_t adc_extract_code(uint16_t fifo_word) {
  auto *d = diagnostics_get();
  if (fifo_word & 0x8000u) {
    d->adc_conversion_error_count++;
    return g_last_valid_code;
  }
  auto code = static_cast<uint16_t>(fifo_word & 0x0FFFu);
  g_last_valid_code = code;
  return code;
}

/**
 * Centers, scales, optionally low-passes, and clamps one averaged ADC code.
 *
 * {@code ADC_PCM_SCALE} is applied in the historical 16-bit domain, then the
 * result is left-shifted 8 bits into a signed 24-bit container so USB keeps
 * full native width without changing relative loudness.
 *
 * Updates {@code adc_*} diagnostic peaks. LPF state is per-stream (reset on
 * start). Peak stats use the pre-LPF magnitude so CDC {@code pcm_pk} still
 * reflects converter headroom when a filter is enabled.
 *
 * @param code 12-bit ADC code (typically an oversample average).
 * @return PCM24 sample (int32 container) for the ring.
 */
static int32_t adc_code_to_pcm(uint16_t code) {
  auto *d = diagnostics_get();
  diagnostics_note_adc_code(code);

#if ADC_DC_REMOVE
  int32_t dc = g_dc_ready ? g_dc_counts : static_cast<int32_t>(ADC_DC_OFFSET_COUNTS);
#else
  int32_t dc = 0;
#endif
  const int32_t centered = static_cast<int32_t>(code) - dc;
  int32_t pcm = adc_scale_centered(centered);

  const uint32_t abs_pcm = audio_pcm24_abs(pcm);
  if (abs_pcm > d->adc_pcm_peak_abs) {
    d->adc_pcm_peak_abs = abs_pcm;
  }

  auto abs_c = static_cast<uint32_t>(centered < 0 ? -centered : centered);
  if (abs_c > d->adc_ac_peak_counts) {
    d->adc_ac_peak_counts = abs_c;
  }

#if ADC_LPF_SHIFT > 0
  if (!g_lpf_primed) {
    g_lpf_y = pcm;
    g_lpf_primed = true;
  } else {
    g_lpf_y += (pcm - g_lpf_y) >> ADC_LPF_SHIFT;
  }
  pcm = g_lpf_y;
  if (pcm > APP_PCM24_FULL_SCALE) {
    pcm = APP_PCM24_FULL_SCALE;
  }
  if (pcm < APP_PCM24_MIN) {
    pcm = APP_PCM24_MIN;
  }
#endif

  return pcm;
}

int32_t adc_convert_sample(uint16_t fifo_word) {
  return adc_code_to_pcm(adc_extract_code(fifo_word));
}

/**
 * DMA IRQ0: mark the completed ping-pong half ready and re-arm the other.
 * Increments {@code dma_overrun_count} if the main loop has not yet drained
 * the completed buffer (producer overrun).
 */
static void __isr dma_handler(void) {
  if (!(dma_hw->ints0 & (1u << static_cast<uint32_t>(g_dma_chan)))) {
    return;
  }
  dma_hw->ints0 = 1u << static_cast<uint32_t>(g_dma_chan);

  uint32_t done = g_active;

  if (g_running) {
    uint32_t next = done ^ 1u;
    g_active = next;
    dma_channel_set_write_addr(g_dma_chan, g_dma_buf[next], false);
    dma_channel_set_trans_count(g_dma_chan, APP_ADC_DMA_BUF_RAW_SAMPLES, true);
  }

  if (g_buf_ready[done]) {
    diagnostics_get()->dma_overrun_count++;
  }
  g_buf_ready[done] = true;
  diagnostics_get()->dma_complete_count++;
}

/** Clears ping-pong flags, DC learning, and LPF state for a fresh stream. */
static void reset_stream_state(void) {
  for (auto &ready : g_buf_ready) {
    ready = false;
  }
  g_active = 0;
#if ADC_DC_REMOVE
  g_dc_ready = false;
  g_dc_counts = static_cast<int32_t>(ADC_DC_OFFSET_COUNTS);
#else
  g_dc_ready = true;
  g_dc_counts = 0;
#endif
  g_last_valid_code = static_cast<uint16_t>(ADC_DC_OFFSET_COUNTS);
  g_lpf_y = 0;
  g_lpf_primed = false;
}

bool adc_capture_init(void) {
  g_running = false;
  reset_stream_state();

  adc_init();
  adc_gpio_init(ADC_GPIO);
  adc_select_input(ADC_CHANNEL);
  adc_set_round_robin(0);

  uint32_t adc_hz = clock_get_hz(clk_adc);
  float div = (static_cast<float>(adc_hz) / static_cast<float>(APP_ADC_RAW_RATE_HZ)) - 1.0f;
  if (div < 0.0f) {
    div = 0.0f;
  }
  adc_set_clkdiv(div);

  adc_fifo_setup(true, true, 1, true, false);

  g_dma_chan = dma_claim_unused_channel(false);
  if (g_dma_chan < 0) {
    return false;
  }
  dma_channel_config cfg = dma_channel_get_default_config(g_dma_chan);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(g_dma_chan, &cfg, g_dma_buf[0], &adc_hw->fifo,
                        APP_ADC_DMA_BUF_RAW_SAMPLES, false);

  dma_channel_set_irq0_enabled(g_dma_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  g_inited = true;
  return true;
}

void adc_capture_start(void) {
  if (!g_inited) {
    return;
  }
  uint32_t irq = save_and_disable_interrupts();
  if (g_running) {
    restore_interrupts(irq);
    return;
  }

  reset_stream_state();
  diagnostics_get()->adc_pcm_peak_abs = 0;
  diagnostics_get()->adc_ac_peak_counts = 0;
  diagnostics_get()->adc_dc_counts = static_cast<uint32_t>(g_dc_counts);

  adc_run(false);
  adc_fifo_drain();

  dma_channel_abort(g_dma_chan);
  dma_channel_acknowledge_irq0(g_dma_chan);

  g_running = true;
  dma_channel_set_write_addr(g_dma_chan, g_dma_buf[0], false);
  dma_channel_set_trans_count(g_dma_chan, APP_ADC_DMA_BUF_RAW_SAMPLES, true);
  adc_run(true);
  restore_interrupts(irq);
}

void adc_capture_stop(void) {
  uint32_t irq = save_and_disable_interrupts();
  if (!g_running) {
    restore_interrupts(irq);
    return;
  }

  g_running = false;
  adc_run(false);
  dma_channel_abort(g_dma_chan);
  dma_channel_acknowledge_irq0(g_dma_chan);
  adc_fifo_drain();
  for (auto &ready : g_buf_ready) {
    ready = false;
  }
  restore_interrupts(irq);
}

bool adc_capture_is_running(void) { return g_running; }

void adc_capture_process(void) {
  if (!g_running) {
    return;
  }

  for (uint32_t i = 0; i < APP_ADC_DMA_BUF_COUNT; i++) {
    if (!g_buf_ready[i]) {
      continue;
    }

    uint16_t *local = g_block_raw;
    uint32_t irq = save_and_disable_interrupts();
    memcpy(local, g_dma_buf[i], sizeof(g_block_raw));
    g_buf_ready[i] = false;
    restore_interrupts(irq);

#if ADC_DC_REMOVE
    if (!g_dc_ready) {
      /* Learn bias from first raw block; do not push the DC step into the ring. */
      uint32_t sum = 0;
      for (uint32_t s = 0; s < APP_ADC_DMA_BUF_RAW_SAMPLES; s++) {
        uint16_t code = adc_extract_code(local[s]);
        sum += code;
        diagnostics_note_adc_code(code);
      }
      g_dc_counts = static_cast<int32_t>(sum / APP_ADC_DMA_BUF_RAW_SAMPLES);
      g_dc_ready = true;
      diagnostics_get()->adc_dc_counts = static_cast<uint32_t>(g_dc_counts);
      diagnostics_get()->adc_total_sample_count += APP_ADC_DMA_BUF_SAMPLES;
      continue;
    }
#endif

    /* Nothing consumes the ring (STANDALONE, or no USB stream): skip the
     * conversion and the push, keep the sample counter. */
    if (!audio_source_feeds_ring()) {
      diagnostics_get()->adc_total_sample_count += APP_ADC_DMA_BUF_SAMPLES;
      continue;
    }

    audio_sample_t *pcm = g_block_pcm;
    for (uint32_t s = 0; s < APP_ADC_DMA_BUF_SAMPLES; s++) {
      uint32_t sum = 0;
      uint32_t base = s * static_cast<uint32_t>(ADC_OVERSAMPLE);
      for (uint32_t k = 0; k < static_cast<uint32_t>(ADC_OVERSAMPLE); k++) {
        sum += adc_extract_code(local[base + k]);
      }
      auto avg = static_cast<uint16_t>(sum / static_cast<uint32_t>(ADC_OVERSAMPLE));
      pcm[s] = adc_code_to_pcm(avg);
    }
    diagnostics_get()->adc_total_sample_count += APP_ADC_DMA_BUF_SAMPLES;
    audio_ring_push(pcm, APP_ADC_DMA_BUF_SAMPLES);
  }
}

#else /* AUDIO_SOURCE != ADC */

int32_t adc_convert_sample(uint16_t fifo_word) {
  const auto code = static_cast<uint16_t>(fifo_word & 0x0FFFu);
  return adc_scale_centered(static_cast<int32_t>(code) -
                            static_cast<int32_t>(ADC_DC_OFFSET_COUNTS));
}

bool adc_capture_init(void) { return true; }
void adc_capture_start(void) {}
void adc_capture_stop(void) {}
bool adc_capture_is_running(void) { return false; }
void adc_capture_process(void) {}

#endif
