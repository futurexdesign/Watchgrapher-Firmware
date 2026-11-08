/**
 * @file diagnostics.h
 * @brief Runtime counters and optional CDC/UART status lines for bench bring-up.
 *
 * Counters always accumulate. Periodic status lines print only when the
 * runtime print flag is on ({@link diagnostics_set_print_enabled}, CLI
 * {@code diag on}) and {@code DIAG_CDC} / {@code DIAG_UART} compile flags
 * allow the transport. ADC min/max/mean/peaks and ring occupancy extrema are
 * cleared after each print so each line reflects the last window.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Aggregate diagnostic counters. All fields are monotonically increasing
 * unless explicitly reset (ADC window fields via {@link diagnostics_reset_adc_stats}
 * or the periodic print clear).
 */
typedef struct {
  uint32_t usb_enumeration_count;
  uint32_t usb_audio_stream_open_count;
  uint32_t usb_audio_stream_close_count;
  uint32_t usb_suspend_count;
  uint32_t usb_resume_count;
  uint32_t boost_on_count;
  uint32_t boost_off_count;
  uint32_t preamp_settle_start_count;
  uint32_t preamp_settle_done_count;
  uint32_t preamp_start_aborted_count;
  uint32_t usb_suspend_duration_ms;
  uint32_t adc_total_sample_count;
  uint32_t adc_conversion_error_count;
  uint32_t dma_complete_count;
  uint32_t dma_overrun_count;
  uint32_t adc_fifo_overrun_or_discontinuity_count;
  uint32_t pcm_ring_underrun_count;
  uint32_t pcm_ring_overrun_count;
  uint32_t usb_in_packet_count;
  uint32_t usb_in_sample_count;
  uint32_t usb_short_write_count;
  /** Lowest ring occupancy observed in the current diag window. */
  uint32_t ring_min_occupancy;
  /** Highest ring occupancy observed in the current diag window. */
  uint32_t ring_max_occupancy;
  uint16_t adc_min_code;
  uint16_t adc_max_code;
  uint32_t adc_mean_code_accum;
  uint32_t adc_mean_code_count;
  /** Learned or forced midrail (ADC counts) used for DC removal. */
  uint32_t adc_dc_counts;
  /** Max |code − dc| in the current window (pre-scale). */
  uint32_t adc_ac_peak_counts;
  /** Max |PCM24| after scale (and before optional LPF) in the window. */
  uint32_t adc_pcm_peak_abs;

  /* PCM1808 backend (compile-time exclusive with ADC capture). */
  uint32_t pcm1808_clock_config_ok;
  uint32_t pcm1808_clk_sys_hz;
  uint32_t pcm1808_clk_usb_hz;
  uint32_t pcm1808_mclk_nominal_hz;
  uint32_t pcm1808_startup_wait_count;
  uint32_t pcm1808_dma_blocks;
  uint32_t pcm1808_stereo_frames_received;
  uint32_t pcm1808_mono_samples_emitted;
  uint32_t pcm1808_dma_block_overruns;
  uint32_t pcm1808_pio_fifo_stall_or_overflow;
  uint32_t pcm1808_frame_sync_errors;
  uint32_t pcm1808_resync_count;
  uint32_t pcm1808_sample_peak_abs;
  uint32_t pcm1808_saturated_count;
  /** Max |left24| / |right24| in the current diag window (raw I²S, pre-DSP). */
  uint32_t pcm1808_raw_left_peak_abs;
  uint32_t pcm1808_raw_right_peak_abs;
} diag_counters_t;

/**
 * Zeroes counters, opens {@code Serial} for the CDC CLI, and optionally
 * opens Serial1 when {@code DIAG_UART=1}. Does not enable periodic prints.
 */
void diagnostics_init(void);

/**
 * Enables or disables the ~2 s periodic status dump (CDC and/or UART).
 * Counters keep updating either way. Default after init is off.
 */
void diagnostics_set_print_enabled(bool on);

/** @return {@code true} when periodic diagnostic prints are enabled. */
bool diagnostics_print_enabled(void);

/**
 * @return Pointer to the global counter block (never null). Callers may
 *         increment fields directly from ISR or main context.
 */
diag_counters_t *diagnostics_get(void);

/**
 * Updates ring min/max occupancy for the current diagnostic window.
 *
 * @param occ Current {@link audio_ring_occupancy} value.
 */
void diagnostics_note_ring_occupancy(uint32_t occ);

/**
 * Updates ADC code min/max and running mean accumulators for the window.
 *
 * @param code 12-bit ADC result (0..4095).
 */
void diagnostics_note_adc_code(uint16_t code);

/**
 * Clears ADC window stats (min/max/mean/dc/peaks). Called on stream open so
 * a new recording starts with a clean slate; {@code dc} is filled again when
 * the capture path learns bias.
 */
void diagnostics_reset_adc_stats(void);

/**
 * Tracks suspend duration. When prints are enabled and ≥2 s since the last
 * dump, prints one status line to CDC and/or UART and clears windowed
 * ADC/ring fields. Safe to call every main-loop iteration.
 */
void diagnostics_poll(void);

#ifdef __cplusplus
}
#endif
