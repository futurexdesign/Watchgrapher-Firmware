/**
 * @file perf_timing.cpp
 * @brief DSP block + main-loop timing accumulators.
 */

#include "perf_timing.h"

#include "diagnostics.h"

#include <Arduino.h>
#include <climits>
#include <cstring>

namespace {

constexpr uint32_t kUsPerSecond = 1000000u;

uint32_t g_dsp_last = 0;
uint32_t g_dsp_mean = 0;
uint32_t g_dsp_max = 0;
uint32_t g_dsp_samples = 0;
uint32_t g_dsp_budget = 0;
uint32_t g_dsp_blocks = 0;

uint32_t g_loop_last = 0;
uint32_t g_loop_mean = 0;
uint32_t g_loop_max = 0;
uint32_t g_loop_count = 0;
uint8_t g_busy_pct = 0;
uint32_t g_idle_us_per_s = kUsPerSecond;

uint32_t g_win_start_us = 0;
uint64_t g_win_busy_us = 0;

/** Integer EMA: mean += (sample - mean) / 8. */
uint32_t ema_update(uint32_t mean, uint32_t sample) {
  if (mean == 0u) {
    return sample;
  }
  return mean - (mean >> 3) + (sample >> 3);
}

void roll_busy_window(uint32_t now_us) {
  if (g_win_start_us == 0u) {
    g_win_start_us = now_us;
    g_win_busy_us = 0;
    return;
  }
  uint32_t elapsed = now_us - g_win_start_us;
  if (elapsed < kUsPerSecond) {
    return;
  }
  /* Close one or more full seconds (usually exactly one). */
  while (elapsed >= kUsPerSecond) {
    uint64_t busy = g_win_busy_us;
    if (busy > kUsPerSecond) {
      busy = kUsPerSecond;
    }
    g_busy_pct = static_cast<uint8_t>((busy * 100u) / kUsPerSecond);
    g_idle_us_per_s = kUsPerSecond - static_cast<uint32_t>(busy);
    g_win_busy_us = 0;
    g_win_start_us += kUsPerSecond;
    elapsed = now_us - g_win_start_us;
  }
}

} // namespace

void perf_timing_init(void) { perf_timing_reset(); }

void perf_timing_note_dsp_block(uint32_t us, uint32_t n, uint32_t source_hz) {
  g_dsp_last = us;
  g_dsp_mean = ema_update(g_dsp_mean, us);
  if (us > g_dsp_max) {
    g_dsp_max = us;
  }
  g_dsp_samples = n;
  if (source_hz == 0u) {
    source_hz = DSP_SAMPLE_RATE_HZ;
  }
  g_dsp_budget = static_cast<uint32_t>(
      (static_cast<uint64_t>(n) * 1000000ull) / static_cast<uint64_t>(source_hz));
  g_dsp_blocks++;
}

void perf_timing_note_loop(uint32_t us) {
  g_loop_last = us;
  g_loop_mean = ema_update(g_loop_mean, us);
  if (us > g_loop_max) {
    g_loop_max = us;
  }
  g_loop_count++;

  uint32_t now = time_us_32();
  roll_busy_window(now);
  g_win_busy_us += us;
}

void perf_timing_get(dsp_timing_t *out) {
  if (!out) {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->dsp_block_us_last = g_dsp_last;
  out->dsp_block_us_mean = g_dsp_mean;
  out->dsp_block_us_max = g_dsp_max;
  out->dsp_block_samples = g_dsp_samples;
  out->dsp_block_budget_us = g_dsp_budget;
  out->dsp_blocks = g_dsp_blocks;
  out->loop_us_last = g_loop_last;
  out->loop_us_mean = g_loop_mean;
  out->loop_us_max = g_loop_max;
  out->loop_count = g_loop_count;
  out->idle_us_per_s = g_idle_us_per_s;
  out->loop_busy_pct = g_busy_pct;

  auto *d = diagnostics_get();
  out->underruns = d->pcm_ring_underrun_count;
  out->short_writes = d->usb_short_write_count;
  out->ring_min =
      d->ring_min_occupancy == UINT32_MAX ? 0u : d->ring_min_occupancy;
  out->ring_max = d->ring_max_occupancy;
}

void perf_timing_reset(void) {
  g_dsp_last = 0;
  g_dsp_mean = 0;
  g_dsp_max = 0;
  g_dsp_samples = 0;
  g_dsp_budget = 0;
  g_dsp_blocks = 0;
  g_loop_last = 0;
  g_loop_mean = 0;
  g_loop_max = 0;
  g_loop_count = 0;
  g_busy_pct = 0;
  g_idle_us_per_s = kUsPerSecond;
  g_win_start_us = time_us_32();
  g_win_busy_us = 0;
}

void perf_timing_clear_window_max(void) {
  g_dsp_max = g_dsp_last;
  g_loop_max = g_loop_last;
}
