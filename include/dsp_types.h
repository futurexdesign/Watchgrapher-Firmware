/**
 * @file dsp_types.h
 * @brief Shared enums and structs for the float32 DSP graph.
 *
 * Two paths share one sample stream:
 *   - Analysis blocks observe only (meters, event detect, beat rate). They
 *     never mutate PCM delivered to USB.
 *   - Process blocks may mutate PCM (notch, filter/fir, soft_gate, gain, AGC,
 *     limiter), and only when enabled with master bypass off.
 *
 * Event / beat timestamps use absolute source sample indices from capture,
 * not USB packet time.
 *
 * @see dsp_graph.h, control_api.h
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Nominal rate for DSP timing (matches USB / capture). */
#ifndef DSP_SAMPLE_RATE_HZ
#define DSP_SAMPLE_RATE_HZ 48000u
#endif

/**
 * Max mono samples accepted by one {@link dsp_graph_process} call.
 * Sized to one PCM1808 / ADC DMA block (240 @ 48 kHz ≈ 5 ms).
 */
#ifndef DSP_MAX_BLOCK
#define DSP_MAX_BLOCK 240u
#endif

/** Ring capacity for recent watch-tick events (CLI / future HID). */
#ifndef DSP_EVENT_LOG_CAP
#define DSP_EVENT_LOG_CAP 32u
#endif

/** Ring capacity for auto-eject / continuity faults. */
#ifndef DSP_ERROR_LOG_CAP
#define DSP_ERROR_LOG_CAP 8u
#endif

/**
 * Named DSP blocks. IDs are stable for CLI strings, {@link control_api}, and
 * a future HID control plane — append new IDs before {@code COUNT} only.
 *
 * {@code DSP_BLOCK_BYPASS} is the master process-path gate (not a real block).
 */
typedef enum {
  DSP_BLOCK_BYPASS = 0,
  DSP_BLOCK_LEVEL_METER,
  DSP_BLOCK_GAIN,
  DSP_BLOCK_LIMITER,
  DSP_BLOCK_FILTER_BANK,
  DSP_BLOCK_EVENT_DETECT,
  DSP_BLOCK_BEAT_RATE,
  DSP_BLOCK_NOISE_FLOOR,
  DSP_BLOCK_EVENT_LOG,
  DSP_BLOCK_SOFT_GATE,
  DSP_BLOCK_NOTCH,
  DSP_BLOCK_AGC,
  DSP_BLOCK_MATCHED_FILTER,
  DSP_BLOCK_FIR_BANK,
  DSP_BLOCK_SPECTRUM,
  DSP_BLOCK_COUNT
} dsp_block_id_t;

/**
 * Band-limit presets for {@link DSP_BLOCK_FILTER_BANK}.
 * Implemented as a cascaded one-pole HPF + LPF (CMSIS DF1 biquad pair).
 */
typedef enum {
  DSP_FILTER_BYPASS = 0, /**< Process path leaves spectrum unchanged. */
  DSP_FILTER_WIDE,       /**< ~160 Hz – 16 kHz */
  DSP_FILTER_MEDIUM,     /**< ~400 Hz – 12 kHz */
  DSP_FILTER_NARROW      /**< ~800 Hz – 8 kHz */
} dsp_filter_preset_t;

/**
 * One acoustic event from the analysis path (watch tick / impulse window).
 * Written when the detector leaves an active window after the tail timeout
 * or max-event duration.
 */
typedef struct {
  uint64_t start_sample; /**< Source index at window open (includes pre-roll). */
  uint64_t peak_sample;  /**< Source index of peak |sample| in the window. */
  uint64_t unlock_sample; /**< First pulse (unlock); used for rate / beat error. */
  uint64_t drop_sample;   /**< Loudest pulse (drop); used for amplitude lift. */
  uint32_t duration_samples;
  uint32_t rise_samples; /**< Samples from onset to peak. */
  uint32_t lift_samples; /**< drop - unlock when both valid; else 0. */
  float peak_abs;        /**< Peak magnitude in float full-scale (~[-1,1]). */
  float noise_floor;     /**< Noise EMA at close time. */
  float snr_db;          /**< 20*log10(peak / noise). */
  float energy;          /**< Sum of x^2 over the open window. */
  float confidence;      /**< Heuristic 0..1 from peak / threshold. */
} dsp_event_t;

/**
 * One timing view: instant snapshot or a rolling time-average window.
 * Invalid metrics are zeroed with the matching {@code *_valid} flag false.
 */
typedef struct {
  float rate_s_day;    /**< Daily rate deviation (s/d). */
  float beat_error_ms; /**< Beat error / repère (ms). */
  float amplitude_deg; /**< Balance amplitude (degrees). */
  uint32_t samples;    /**< Instant: 0/1. Rolling: count contributing ticks. */
  bool rate_valid;
  bool beat_error_valid;
  bool amplitude_valid;
} dsp_beat_timing_window_t;

/**
 * Escapement timing estimate from unlock-based intervals (Witschi-style).
 * Updated when enough recent intervals exist and beat_rate is enabled.
 *
 * Top-level rate/beat-error/amplitude fields are the <em>instant</em> view
 * (last computed tick set). {@code avg_1s} / {@code avg_10s} are arithmetic
 * means of those instant values over rolling wall-clock windows.
 */
typedef struct {
  float vph_raw;             /**< Vibrations per hour from median half-interval. */
  uint32_t vph_nearest;      /**< Nominal A/h when locked (auto or manual), else 0. */
  float confidence;          /**< 0..1 proximity to nominal rate. */
  bool locked;               /**< True when within tolerance of a nominal rate. */
  uint32_t interval_samples; /**< Median unlock-to-unlock half-interval (samples). */
  uint32_t events_seen;      /**< Events currently held in the event log. */
  uint32_t intervals_held;   /**< Accepted half-intervals in beat history. */
  uint32_t source_hz;        /**< Analysis sample rate used for timing math. */

  float rate_s_day;     /**< Instant daily rate (s/d); alias of instant.rate_s_day. */
  float beat_error_ms;  /**< Instant beat error (ms). */
  float amplitude_deg;  /**< Instant amplitude (degrees). */
  float lift_angle_deg; /**< Configured lift angle λ used for amplitude. */
  float lift_time_ms;   /**< Latest unlock→drop lift time (ms). */
  bool rate_valid;
  bool beat_error_valid;
  bool amplitude_valid;

  dsp_beat_timing_window_t instant; /**< Last computed tick set (same as top-level). */
  dsp_beat_timing_window_t avg_1s;  /**< Rolling ~1 s mean of instant values. */
  dsp_beat_timing_window_t avg_10s; /**< Rolling ~10 s mean of instant values. */
} dsp_beat_rate_t;

/**
 * Running level meters: pre = before process path, post = after process /
 * before PCM24 quantize. Peak values are lightly decayed; RMS is an EMA.
 */
typedef struct {
  float pre_peak;
  float post_peak;
  float pre_rms;
  float post_rms;
  float noise_floor;      /**< Event-detect noise EMA (float full-scale). */
  float agc_gain_db;      /**< Current slow-AGC gain in dB (0 if AGC off). */
  float match_score;      /**< Latest matched-filter peak score (0 if unused). */
  uint32_t clip_count;    /**< Hard clamps at PCM24 full scale. */
  uint32_t limiter_count; /**< Soft clamps at ±1.0 in the limiter block. */
} dsp_metrics_t;

/**
 * Core-0 CPU / DSP timing snapshot for headroom diagnostics.
 * Block stats update on each {@code dsp_graph_process}; loop stats on each
 * main {@code loop()} pass. Continuity counters are copied from diagnostics.
 */
typedef struct {
  uint32_t dsp_block_us_last;
  uint32_t dsp_block_us_mean;   /**< EMA of block wall time. */
  uint32_t dsp_block_us_max;    /**< Max since last timing reset / window clear. */
  uint32_t dsp_block_samples;   /**< Sample count of last process call. */
  uint32_t dsp_block_budget_us; /**< n * 1e6 / source_hz for last block. */
  uint32_t dsp_blocks;          /**< Process calls since reset. */

  uint32_t loop_us_last;
  uint32_t loop_us_mean; /**< EMA of main-loop wall time. */
  uint32_t loop_us_max;
  uint32_t loop_count;   /**< Loop iterations since reset. */
  uint32_t idle_us_per_s; /**< Approx idle µs in the last completed 1 s window. */
  uint8_t loop_busy_pct;  /**< 0..100 over the last completed 1 s window. */

  uint32_t underruns;    /**< {@code pcm_ring_underrun_count} snapshot. */
  uint32_t short_writes; /**< {@code usb_short_write_count} snapshot. */
  uint32_t ring_min;     /**< Current diag window ring min occupancy. */
  uint32_t ring_max;     /**< Current diag window ring max occupancy. */
} dsp_timing_t;

/** Last 512-point real-FFT peak list ({@link DSP_BLOCK_SPECTRUM}). */
typedef struct {
  float peak_hz[8];
  float peak_mag[8];
  uint32_t n_peaks;
  uint32_t fft_n; /**< Always 512 when valid. */
  uint32_t source_hz;
  bool valid;
} dsp_spectrum_t;

/** Matched-filter learning / score state. */
typedef struct {
  float score;         /**< Last block peak correlation (normalized). */
  float template_peak; /**< Peak |sample| in learned template. */
  uint32_t template_len;
  bool learned;
} dsp_matched_t;

/** Why a process block was auto-ejected (or a fault was logged). */
typedef enum {
  DSP_ERR_NONE = 0,
  DSP_ERR_UNDERRUN,  /**< Ring underrun rose after the block was enabled. */
  DSP_ERR_USB_STALL, /**< USB short-write count rose sharply after enable. */
  DSP_ERR_OVERLOAD
} dsp_error_reason_t;

/** One fault log entry for CLI {@code dsp errors}. */
typedef struct {
  dsp_block_id_t block;
  dsp_error_reason_t reason;
  uint32_t uptime_ms;
  uint64_t sample_index;
} dsp_error_t;

#ifdef __cplusplus
}
#endif
