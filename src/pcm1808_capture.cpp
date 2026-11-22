/**
 * @file pcm1808_capture.cpp
 * @brief PIO I²S RX + DMA ping-pong capture for the PCM1808 codec.
 *
 * Uses a single DMA channel that switches write buffers in the IRQ (same
 * pattern as the onboard ADC path). Dual-channel {@code chain_to} ping-pong
 * previously left the second descriptor dead after the first block, which
 * filled the PIO RX FIFO, stalled the SM, and produced USB underrun holds.
 *
 * The codec's frame rate depends on its MD straps (256/384/512 × fs), so it
 * is measured rather than assumed: the DMA IRQ timestamps block completions,
 * and every {@code kRateWindowBlocks} blocks the main loop computes frames/s,
 * snaps it to an MCLK ratio ({@link pcm1808_classify_rate}), and sets the
 * DSP source rate and the USB sample repeat to match. Until a rate is
 * confirmed the DSP timebase is marked unverified, so the beat_rate block
 * will not lock on a guessed clock.
 *
 * @see pcm1808_capture.h
 */

#include "pcm1808_capture.h"

#include "app_config.h"
#include "audio_ring.h"
#include "audio_source.h"
#include "board_config.h"
#include "diagnostics.h"
#include "dsp_graph.h"
#include "pcm1808_clock.h"
#include "pcm1808_format.h"
#include "pcm1808_i2s.pio.h"
#include "pcm1808_rate.h"
#include "quartz/quartz_engine.h"

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <hardware/timer.h>

#include <Arduino.h>
#include <cstring>

#if AUDIO_SOURCE == PCM1808

/* PIO wait gpio immediates are board-baked — must match PCM1808_* build flags. */
static_assert(PCM1808_BCK_GPIO == pcm1808_i2s_rx_BCK_GPIO, "BCK pin mismatch");
static_assert(PCM1808_LRCK_GPIO == pcm1808_i2s_rx_LRCK_GPIO, "LRCK pin mismatch");

#define PCM1808_WORDS_PER_BLOCK (APP_PCM1808_DMA_FRAMES * 2u)

static uint32_t g_dma_buf[APP_PCM1808_DMA_BUF_COUNT][PCM1808_WORDS_PER_BLOCK]
    __attribute__((aligned(4)));
static volatile bool g_buf_ready[APP_PCM1808_DMA_BUF_COUNT];
/**
 * Completion sequence number of each buffer's current contents: the value of
 * {@code g_irq_blocks} after the IRQ that completed it (1 for the first block
 * after start). Written by the IRQ together with {@code g_buf_ready}; read
 * by the main loop with interrupts disabled. See
 * {@link pcm1808_capture_process} for why blocks are taken oldest first.
 */
static volatile uint32_t g_buf_seq[APP_PCM1808_DMA_BUF_COUNT];
static volatile bool g_running;
static volatile bool g_start_pending;
static volatile uint32_t g_active;
static int g_dma_chan = -1;
static PIO g_pio = pio0;
static uint g_sm;
static uint g_offset;
static bool g_inited;
static uint64_t g_source_index;

/* Frame-rate measurement (see file comment). */
/** Blocks ignored after start while the PIO aligns to LRCK. */
constexpr uint32_t kRateSkipBlocks = 2u;
/** Blocks per measurement window (0.5 s at 24 kHz, 0.25 s at 48 kHz). */
constexpr uint32_t kRateWindowBlocks = 50u;
/** Consecutive agreeing windows needed to change the accepted rate. */
constexpr uint32_t kRateConfirmWindows = 2u;
static volatile uint32_t g_irq_blocks;
static volatile uint64_t g_irq_last_us;
static bool g_have_anchor;
static uint32_t g_anchor_blocks;
static uint64_t g_anchor_us;
static pcm1808_rate_match_t g_rate;      /* accepted */
static pcm1808_rate_match_t g_candidate; /* awaiting confirmation */
static uint32_t g_candidate_windows;
/** Current sample repeat into the 48 kHz ring (measured; build flag is the
 *  guess used until the first confirmation). */
static uint32_t g_upsample = PCM1808_OUTPUT_UPSAMPLE;

/*
 * Per-block scratch for pcm1808_capture_process. File-scope rather than stack:
 * together these are ~5.8 KB, and core 0 only has a 4 KB stack (SCRATCH_Y).
 * Only the main loop touches them.
 */
static uint32_t g_block_words[PCM1808_WORDS_PER_BLOCK];
static float g_block_fs[APP_PCM1808_DMA_FRAMES];
static audio_sample_t g_block_mono[APP_PCM1808_DMA_FRAMES];
static audio_sample_t g_block_pcm[APP_PCM1808_DMA_FRAMES * PCM1808_MAX_UPSAMPLE];
/** Raw sign-extended codes per channel for the quartz engine (LEFT =
 *  inductive pickup, RIGHT = capacitive pickup); 1.9 KB more scratch, kept
 *  file-scope for the same stack reason. */
static int32_t g_block_left24[APP_PCM1808_DMA_FRAMES];
static int32_t g_block_right24[APP_PCM1808_DMA_FRAMES];
/** DMA block overruns already reported to the quartz engine. */
static uint32_t g_quartz_overruns_seen;
/** Sequence number of the last block processed (0 at start, so the first
 *  block, sequence 1, is contiguous). */
static uint32_t g_last_seq;

static_assert(PCM1808_OUTPUT_UPSAMPLE <= PCM1808_MAX_UPSAMPLE,
              "PCM1808_OUTPUT_UPSAMPLE exceeds the block buffer");

/**
 * Tracks absolute PCM peak for {@code pcm1808_sample_peak_abs}.
 *
 * @param s One mono PCM24 sample (int32 container) after DSP.
 */
static void pcm1808_note_peak(int32_t s) {
  auto *d = diagnostics_get();
  const uint32_t abs_v = audio_pcm24_abs(s);
  if (abs_v > d->pcm1808_sample_peak_abs) {
    d->pcm1808_sample_peak_abs = abs_v;
  }
}

/**
 * Finds the ready buffer holding the oldest block. Call with interrupts
 * disabled so the ready flags and sequence numbers are consistent.
 *
 * Sequence numbers are compared by signed difference, so the comparison
 * stays correct across the 2^32 wrap (about 248 days at 48 kHz).
 *
 * @return Buffer index, or -1 if none is ready.
 */
static int32_t oldest_ready_buffer(void) {
  int32_t best = -1;
  for (uint32_t i = 0; i < APP_PCM1808_DMA_BUF_COUNT; i++) {
    if (!g_buf_ready[i]) {
      continue;
    }
    if (best < 0 || static_cast<int32_t>(g_buf_seq[i] - g_buf_seq[best]) < 0) {
      best = static_cast<int32_t>(i);
    }
  }
  return best;
}

/**
 * DMA IRQ0: swap to the other ping-pong buffer and restart immediately so the
 * PIO RX FIFO never backs up, then stamp the completed buffer with its
 * completion sequence number so the main loop can take blocks oldest first.
 */
static void __isr pcm1808_dma_handler(void) {
  if (!(dma_hw->ints0 & (1u << static_cast<uint32_t>(g_dma_chan)))) {
    return;
  }
  dma_hw->ints0 = 1u << static_cast<uint32_t>(g_dma_chan);

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
  g_irq_last_us = time_us_64();
  const uint32_t seq = g_irq_blocks + 1u;
  g_irq_blocks = seq;
  g_buf_seq[done] = seq; /* one store: the only IRQ cost of age ordering */
  g_buf_ready[done] = true;
  diagnostics_get()->pcm1808_dma_blocks++;
  diagnostics_get()->dma_complete_count++;
}

/** Mirrors the accepted rate into the diagnostic counters. */
static void publish_rate_diag(double measured_hz) {
  auto *d = diagnostics_get();
  d->pcm1808_frame_rate_mhz = static_cast<uint32_t>(measured_hz * 1000.0 + 0.5);
  d->pcm1808_rate_status = static_cast<uint32_t>(g_rate.status);
  d->pcm1808_mclk_ratio = g_rate.mclk_ratio;
  d->pcm1808_upsample = g_upsample;
}

/**
 * Makes {@code m} the accepted rate: on a supported ratio, retimes the DSP
 * graph and the sample repeat (resetting the stream if either changed) and
 * marks the timebase verified; otherwise marks it unverified and leaves the
 * repeat alone so audio keeps flowing.
 */
static void apply_rate(const pcm1808_rate_match_t &m, double measured_hz) {
  g_rate = m;
  if (m.status == PCM1808_RATE_OK) {
    if (m.upsample != g_upsample || m.nominal_hz != dsp_graph_source_rate()) {
      g_upsample = m.upsample;
      dsp_graph_set_source_rate(m.nominal_hz);
      dsp_graph_reset_stream();
    }
    dsp_graph_set_timebase(true, static_cast<float>(measured_hz));
  } else {
    dsp_graph_set_timebase(false, static_cast<float>(measured_hz));
  }
}

/**
 * Closes a measurement window once {@code kRateWindowBlocks} more blocks have
 * completed since the last one. IRQ timestamps make the result independent
 * of main-loop latency; jitter of a few µs over ≥ 0.25 s is ~10 ppm, far
 * finer than the 0.5 % needed to tell 256 / 384 / 512 fs apart.
 */
static void update_rate_measurement(void) {
  const uint32_t irq = save_and_disable_interrupts();
  const uint32_t blocks = g_irq_blocks;
  const uint64_t t_us = g_irq_last_us;
  restore_interrupts(irq);

  if (blocks < kRateSkipBlocks) {
    return;
  }
  if (!g_have_anchor) {
    g_have_anchor = true;
    g_anchor_blocks = blocks;
    g_anchor_us = t_us;
    return;
  }
  const uint32_t n_blocks = blocks - g_anchor_blocks;
  if (n_blocks < kRateWindowBlocks) {
    return;
  }
  const uint64_t dt_us = t_us - g_anchor_us;
  g_anchor_blocks = blocks;
  g_anchor_us = t_us;
  if (dt_us == 0u) {
    return;
  }

  const double hz = static_cast<double>(n_blocks) * APP_PCM1808_DMA_FRAMES *
                    1e6 / static_cast<double>(dt_us);
  const pcm1808_rate_match_t m =
      pcm1808_classify_rate(hz, PCM1808_MCLK_NOMINAL_HZ, APP_SAMPLE_RATE_HZ);

  if (m.status == g_rate.status && m.nominal_hz == g_rate.nominal_hz) {
    g_candidate_windows = 0;
    dsp_graph_set_timebase(m.status == PCM1808_RATE_OK, static_cast<float>(hz));
  } else {
    if (m.status == g_candidate.status && m.nominal_hz == g_candidate.nominal_hz) {
      g_candidate_windows++;
    } else {
      g_candidate = m;
      g_candidate_windows = 1u;
    }
    if (g_candidate_windows >= kRateConfirmWindows) {
      g_candidate_windows = 0;
      apply_rate(m, hz);
    }
  }
  publish_rate_diag(hz);
}

/**
 * Arms DMA into buffer 0, resets the PIO SM to the I²S entry point, and
 * enables the state machine.
 */
static void capture_hw_start(void) {
  for (auto &ready : g_buf_ready) {
    ready = false;
  }
  g_active = 0;
  g_source_index = 0;
  g_irq_blocks = 0;
  g_last_seq = 0;
  g_have_anchor = false;
  g_candidate_windows = 0;
  g_quartz_overruns_seen = diagnostics_get()->pcm1808_dma_block_overruns;
  dsp_graph_reset_stream();

  g_pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_sm));

  dma_channel_abort(g_dma_chan);
  dma_hw->ints0 = 1u << static_cast<uint32_t>(g_dma_chan);

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
  dma_hw->ints0 = 1u << static_cast<uint32_t>(g_dma_chan);
  pio_sm_clear_fifos(g_pio, g_sm);
  g_pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_sm));
  for (auto &ready : g_buf_ready) {
    ready = false;
  }
}

bool pcm1808_capture_init(void) {
  for (auto &ready : g_buf_ready) {
    ready = false;
  }
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

  pcm1808_i2s_rx_program_init(g_pio, g_sm, g_offset, static_cast<uint>(PCM1808_DOUT_GPIO));

  g_dma_chan = dma_claim_unused_channel(false);
  if (g_dma_chan < 0) {
    return false;
  }
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
   * DSP sees codec frames before the sample repeat. Start from the build-flag
   * guess, but mark the timebase unverified until the rate is measured.
   */
  g_upsample = PCM1808_OUTPUT_UPSAMPLE;
  g_rate = {PCM1808_RATE_PENDING, 0u, 0u, 0u};
  g_candidate = g_rate;
  dsp_graph_set_source_rate(APP_SAMPLE_RATE_HZ / g_upsample);
  dsp_graph_set_timebase(false, 0.f);
  publish_rate_diag(0.0);

  g_inited = true;
  return true;
}

bool pcm1808_capture_is_ready(void) {
  if (!pcm1808_clock_is_ok()) {
    return false;
  }
  return static_cast<int32_t>(millis() - pcm1808_clock_ready_deadline_ms()) >= 0;
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

  update_rate_measurement();

  /* Count sticky RXSTALL but do not tear down the stream. */
  uint32_t stall_mask = (1u << (PIO_FDEBUG_RXSTALL_LSB + g_sm));
  if (g_pio->fdebug & stall_mask) {
    diagnostics_get()->pcm1808_pio_fifo_stall_or_overflow++;
    g_pio->fdebug = stall_mask;
  }

  /*
   * Take ready blocks oldest first. The IRQ completes the buffers
   * alternately, but when the loop falls behind both can be ready, and the
   * older one is buffer 1 half of the time; after an overrun the overwritten
   * buffer holds the NEWEST block although its index is the lower one.
   * Taking them in index order (as this loop once did) then swapped two
   * blocks: each was given the other's source index, so anything detected
   * in them was timed 5 ms off and the waveform itself jumped back and
   * forth. That is a fake timing step for both the beat_rate fit and the
   * quartz engine (a motor pulse there reads as a 5 ms inhibition jump).
   * In the normal case only one buffer is ready per pass and the order is
   * the same either way.
   *
   * At most one pass per buffer per call, as before, so a stall cannot
   * starve the rest of the main loop.
   */
  for (uint32_t pass = 0; pass < APP_PCM1808_DMA_BUF_COUNT; pass++) {
    uint32_t *local = g_block_words;
    uint32_t irq = save_and_disable_interrupts();
    const int32_t i = oldest_ready_buffer();
    if (i < 0) {
      restore_interrupts(irq);
      break;
    }
    const uint32_t seq = g_buf_seq[i];
    memcpy(local, g_dma_buf[i], sizeof(g_block_words));
    g_buf_ready[i] = false;
    restore_interrupts(irq);
    /*
     * Blocks between the last one processed and this one were overwritten
     * before they could be read (the IRQ also counted them as overruns).
     * Their samples are gone, but their time is not: advancing the source
     * index by the lost frames keeps every later sample at its true absolute
     * index. Without this, everything after the loss would be timed
     * 240 frames (5 ms at 48 kHz) early. The beat_rate fit would see that as
     * a step in its unlock times and bias its slope. With it, the fit sees an
     * honest gap, which it already handles (missed-tick grid numbers, or a
     * restart past BEAT_FIT_MAX_GAP). Unsigned arithmetic keeps this exact
     * across the sequence wrap.
     *
     * The quartz engine restarts twice over here: once from
     * notify_sample_loss() below (counted once in sample_loss_restarts), and
     * once from the index gap at its next process() call, which only re-arms
     * a pipeline that is already disarmed. Both restarts land on the same
     * block, so nothing is lost twice or counted twice.
     *
     * An overrun-counter change without a sequence gap keeps the old
     * handling: only the quartz restart below.
     */
    const uint32_t lost = seq - g_last_seq - 1u;
    const bool skipped = lost != 0u;
    g_last_seq = seq;
    if (skipped) {
      g_source_index += static_cast<uint64_t>(lost) * APP_PCM1808_DMA_FRAMES;
      diagnostics_get()->pcm1808_blocks_lost += lost;
    }

    audio_sample_t *pcm = g_block_pcm;
    uint32_t out_n = 0;
    float *in_fs = g_block_fs;
    audio_sample_t *mono = g_block_mono;

    auto *d = diagnostics_get();
    for (uint32_t f = 0; f < APP_PCM1808_DMA_FRAMES; f++) {
      uint32_t left = local[f * 2u];
      uint32_t right = local[f * 2u + 1u];
      const int32_t left24 = pcm1808_slot_to_pcm24(left);
      const int32_t right24 = pcm1808_slot_to_pcm24(right);
      const uint32_t labs = audio_pcm24_abs(left24);
      const uint32_t rabs = audio_pcm24_abs(right24);
      if (labs > d->pcm1808_raw_left_peak_abs) {
        d->pcm1808_raw_left_peak_abs = labs;
      }
      if (rabs > d->pcm1808_raw_right_peak_abs) {
        d->pcm1808_raw_right_peak_abs = rabs;
      }
      if (left24 == APP_PCM24_FULL_SCALE || left24 == APP_PCM24_MIN) {
        d->pcm1808_saturated_count++;
      }
      in_fs[f] = dsp_sample_from_pcm1808_left24(left24);
      g_block_left24[f] = left24;
      g_block_right24[f] = right24;
    }
    uint64_t base = g_source_index;
    dsp_graph_process(in_fs, APP_PCM1808_DMA_FRAMES, base, mono);
    g_source_index += APP_PCM1808_DMA_FRAMES;
    /* No ring consumer (STANDALONE, or no USB stream): skip the sample
     * repeat and the push, but keep the diagnostics counting what would
     * have been emitted. */
    const bool feed_ring = audio_source_feeds_ring();
    for (uint32_t f = 0; f < APP_PCM1808_DMA_FRAMES; f++) {
      pcm1808_note_peak(mono[f]);
      if (feed_ring) {
        for (uint32_t u = 0; u < g_upsample; u++) {
          pcm[out_n++] = mono[f];
        }
      }
    }
    if (!feed_ring) {
      out_n = APP_PCM1808_DMA_FRAMES * g_upsample;
    }

    diagnostics_get()->pcm1808_stereo_frames_received += APP_PCM1808_DMA_FRAMES;
    if (feed_ring) {
      audio_ring_push(pcm, out_n);
    }
    diagnostics_get()->pcm1808_mono_samples_emitted += out_n;
    diagnostics_get()->adc_total_sample_count += out_n;

    /*
     * Quartz timing runs after the block is queued for USB, so a slow block
     * (the LCD harmonic selection) delays analysis, never the audio. An
     * overwritten DMA buffer means a block is missing although the source
     * index advanced as if it were not, so a running measurement restarts.
     */
    const uint32_t overruns = d->pcm1808_dma_block_overruns;
    if (skipped || overruns != g_quartz_overruns_seen) {
      g_quartz_overruns_seen = overruns;
      quartz::engine::notify_sample_loss();
    }
    quartz::engine::process(g_block_left24, g_block_right24,
                            APP_PCM1808_DMA_FRAMES, base);
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
