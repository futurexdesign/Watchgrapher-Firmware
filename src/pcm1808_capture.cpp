/**
 * @file pcm1808_capture.cpp
 * @brief PIO I²S RX + DMA ping-pong capture for the PCM1808 codec.
 *
 * Uses a single DMA channel that switches write buffers in the IRQ (same
 * pattern as the onboard ADC path). Dual-channel {@code chain_to} ping-pong
 * previously left the second descriptor dead after the first block, which
 * filled the PIO RX FIFO, stalled the SM, and produced USB underrun holds.
 *
 * @see pcm1808_capture.h
 */

#include "pcm1808_capture.h"

#include "app_config.h"
#include "audio_ring.h"
#include "board_config.h"
#include "diagnostics.h"
#include "dsp_graph.h"
#include "pcm1808_clock.h"
#include "pcm1808_i2s.pio.h"

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>

#include <Arduino.h>
#include <string.h>

#if AUDIO_SOURCE == PCM1808

/* PIO wait gpio immediates are fixed to GP2/GP3 — must match board_config. */
static_assert(PCM1808_BCK_GPIO == pcm1808_i2s_rx_BCK_GPIO, "BCK pin mismatch");
static_assert(PCM1808_LRCK_GPIO == pcm1808_i2s_rx_LRCK_GPIO, "LRCK pin mismatch");

#define PCM1808_WORDS_PER_BLOCK (APP_PCM1808_DMA_FRAMES * 2u)

static uint32_t g_dma_buf[APP_PCM1808_DMA_BUF_COUNT][PCM1808_WORDS_PER_BLOCK]
    __attribute__((aligned(4)));
static volatile bool g_buf_ready[APP_PCM1808_DMA_BUF_COUNT];
static volatile bool g_running;
static volatile bool g_start_pending;
static volatile uint32_t g_active;
static int g_dma_chan;
static PIO g_pio = pio0;
static uint g_sm;
static uint g_offset;
static bool g_inited;
static uint64_t g_source_index;

/**
 * Tracks absolute PCM peak for {@code pcm1808_sample_peak_abs}.
 *
 * @param s One mono PCM24 sample (int32 container) after DSP.
 */
static void pcm1808_note_peak(int32_t s) {
  diag_counters_t *d = diagnostics_get();
  uint32_t abs_v;
  if (s == (int32_t)0xFF800000) {
    abs_v = 8388608u;
  } else if (s < 0) {
    abs_v = (uint32_t)(-s);
  } else {
    abs_v = (uint32_t)s;
  }
  if (abs_v > d->pcm1808_sample_peak_abs) {
    d->pcm1808_sample_peak_abs = abs_v;
  }
}

/**
 * DMA IRQ0: swap to the other ping-pong buffer and restart immediately so the
 * PIO RX FIFO never backs up.
 */
static void __isr pcm1808_dma_handler(void) {
  if (!(dma_hw->ints0 & (1u << (uint32_t)g_dma_chan))) {
    return;
  }
  dma_hw->ints0 = 1u << (uint32_t)g_dma_chan;

  uint32_t done = g_active;

  if (g_running) {
    uint32_t next = done ^ 1u;
    g_active = next;
    dma_channel_set_write_addr(g_dma_chan, g_dma_buf[next], false);
    dma_channel_set_trans_count(g_dma_chan, PCM1808_WORDS_PER_BLOCK, true);
  }

  if (g_buf_ready[done]) {
    diagnostics_get()->pcm1808_dma_block_overruns++;
    diagnostics_get()->dma_overrun_count++;
  }
  g_buf_ready[done] = true;
  diagnostics_get()->pcm1808_dma_blocks++;
  diagnostics_get()->dma_complete_count++;
}

/**
 * Arms DMA into buffer 0, resets the PIO SM to the I²S entry point, and
 * enables the state machine.
 */
static void capture_hw_start(void) {
  memset((void *)g_buf_ready, 0, sizeof(g_buf_ready));
  g_active = 0;

  /* Re-apply after control_api/dsp_graph_init (runs later in setup than capture_init). */
  dsp_graph_set_source_rate(APP_SAMPLE_RATE_HZ / (uint32_t)PCM1808_OUTPUT_UPSAMPLE);

  g_pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_sm));

  dma_channel_abort(g_dma_chan);
  dma_hw->ints0 = 1u << (uint32_t)g_dma_chan;

  pio_sm_set_enabled(g_pio, g_sm, false);
  pio_sm_clear_fifos(g_pio, g_sm);
  pio_sm_restart(g_pio, g_sm);
  pio_sm_clkdiv_restart(g_pio, g_sm);
  pio_sm_exec(g_pio, g_sm,
              pio_encode_jmp(g_offset + pcm1808_i2s_rx_offset_entry_point));

  g_running = true;
  dma_channel_set_write_addr(g_dma_chan, g_dma_buf[0], false);
  dma_channel_set_trans_count(g_dma_chan, PCM1808_WORDS_PER_BLOCK, true);
  pio_sm_set_enabled(g_pio, g_sm, true);
  g_start_pending = false;
}

/**
 * Disables the PIO SM, aborts DMA, and clears FIFOs / ready flags.
 */
static void capture_hw_stop(void) {
  if (!g_running && !g_start_pending) {
    return;
  }
  g_start_pending = false;
  g_running = false;
  pio_sm_set_enabled(g_pio, g_sm, false);
  dma_channel_abort(g_dma_chan);
  dma_hw->ints0 = 1u << (uint32_t)g_dma_chan;
  pio_sm_clear_fifos(g_pio, g_sm);
  g_pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_sm));
  memset((void *)g_buf_ready, 0, sizeof(g_buf_ready));
}

bool pcm1808_capture_init(void) {
  memset((void *)g_buf_ready, 0, sizeof(g_buf_ready));
  g_running = false;
  g_start_pending = false;
  g_active = 0;

  gpio_init(PCM1808_BCK_GPIO);
  gpio_set_dir(PCM1808_BCK_GPIO, GPIO_IN);
  gpio_init(PCM1808_LRCK_GPIO);
  gpio_set_dir(PCM1808_LRCK_GPIO, GPIO_IN);
  gpio_init(PCM1808_DOUT_GPIO);
  gpio_set_dir(PCM1808_DOUT_GPIO, GPIO_IN);

  if (!pio_claim_free_sm_and_add_program(&pcm1808_i2s_rx_program, &g_pio, &g_sm,
                                         &g_offset)) {
    return false;
  }

  pcm1808_i2s_rx_program_init(g_pio, g_sm, g_offset, (uint)PCM1808_DOUT_GPIO);

  g_dma_chan = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(g_dma_chan);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, pio_get_dreq(g_pio, g_sm, false));

  dma_channel_configure(g_dma_chan, &cfg, g_dma_buf[0], &g_pio->rxf[g_sm],
                        PCM1808_WORDS_PER_BLOCK, false);

  dma_channel_set_irq0_enabled(g_dma_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, pcm1808_dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  /*
   * DSP sees pre-upsample mono frames. With UPSAMPLE=2 the codec LRCK is
   * typically ~24 kHz — beat/event timing must use that rate, not USB 48 kHz.
   */
  dsp_graph_set_source_rate(APP_SAMPLE_RATE_HZ / (uint32_t)PCM1808_OUTPUT_UPSAMPLE);

  g_inited = true;
  return true;
}

bool pcm1808_capture_is_ready(void) {
  if (!pcm1808_clock_is_ok()) {
    return false;
  }
  return (int32_t)(millis() - pcm1808_clock_ready_deadline_ms()) >= 0;
}

void pcm1808_capture_start(void) {
  if (!g_inited || g_running) {
    return;
  }
  if (!pcm1808_capture_is_ready()) {
    diagnostics_get()->pcm1808_startup_wait_count++;
    g_start_pending = true;
    return;
  }
  capture_hw_start();
}

void pcm1808_capture_stop(void) { capture_hw_stop(); }

bool pcm1808_capture_is_running(void) { return g_running; }

void pcm1808_capture_process(void) {
  if (g_start_pending && pcm1808_capture_is_ready() && !g_running) {
    capture_hw_start();
  }

  if (!g_running) {
    return;
  }

  /* Count sticky RXSTALL but do not tear down the stream. */
  uint32_t stall_mask = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_sm));
  if (g_pio->fdebug & stall_mask) {
    diagnostics_get()->pcm1808_pio_fifo_stall_or_overflow++;
    g_pio->fdebug = stall_mask;
  }

  for (uint32_t i = 0; i < APP_PCM1808_DMA_BUF_COUNT; i++) {
    if (!g_buf_ready[i]) {
      continue;
    }

    uint32_t local[PCM1808_WORDS_PER_BLOCK];
    uint32_t irq = save_and_disable_interrupts();
    memcpy(local, g_dma_buf[i], sizeof(local));
    g_buf_ready[i] = false;
    restore_interrupts(irq);

    audio_sample_t pcm[APP_PCM1808_DMA_FRAMES * (uint32_t)PCM1808_OUTPUT_UPSAMPLE];
    uint32_t out_n = 0;
    float in_fs[APP_PCM1808_DMA_FRAMES];
    audio_sample_t mono[APP_PCM1808_DMA_FRAMES];

    diag_counters_t *d = diagnostics_get();
    for (uint32_t f = 0; f < APP_PCM1808_DMA_FRAMES; f++) {
      uint32_t left = local[f * 2u];
      uint32_t right = local[f * 2u + 1u];
      int32_t left24 = (int32_t)left >> 8;
      int32_t right24 = (int32_t)right >> 8;
      uint32_t labs = (left24 < 0) ? (uint32_t)(-left24) : (uint32_t)left24;
      uint32_t rabs = (right24 < 0) ? (uint32_t)(-right24) : (uint32_t)right24;
      if (labs > d->pcm1808_raw_left_peak_abs) {
        d->pcm1808_raw_left_peak_abs = labs;
      }
      if (rabs > d->pcm1808_raw_right_peak_abs) {
        d->pcm1808_raw_right_peak_abs = rabs;
      }
      if (left24 == 0x7FFFFF || left24 == (int32_t)0xFF800000) {
        d->pcm1808_saturated_count++;
      }
      in_fs[f] = dsp_sample_from_pcm1808_left24(left24);
    }
    uint64_t base = g_source_index;
    dsp_graph_process(in_fs, APP_PCM1808_DMA_FRAMES, base, mono);
    g_source_index += APP_PCM1808_DMA_FRAMES;
    for (uint32_t f = 0; f < APP_PCM1808_DMA_FRAMES; f++) {
      pcm1808_note_peak(mono[f]);
      for (uint32_t u = 0; u < (uint32_t)PCM1808_OUTPUT_UPSAMPLE; u++) {
        pcm[out_n++] = mono[f];
      }
    }

    diagnostics_get()->pcm1808_stereo_frames_received += APP_PCM1808_DMA_FRAMES;
    audio_ring_push(pcm, out_n);
    diagnostics_get()->pcm1808_mono_samples_emitted += out_n;
    diagnostics_get()->adc_total_sample_count += out_n;
  }
}

#else /* AUDIO_SOURCE != PCM1808 */

bool pcm1808_capture_init(void) { return true; }
void pcm1808_capture_start(void) {}
void pcm1808_capture_stop(void) {}
bool pcm1808_capture_is_running(void) { return false; }
bool pcm1808_capture_is_ready(void) { return false; }
void pcm1808_capture_process(void) {}

#endif
