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
  float unlock_frac;     /**< Sub-sample offset of unlock (-0.5..0.5). */
  float drop_frac;       /**< Sub-sample offset of drop (-0.5..0.5). */
  float peak_abs;        /**< Peak magnitude in float full-scale (~[-1,1]). */
  float noise_floor;     /**< Noise EMA at close time. */
  float snr_db;          /**< 20*log10(peak / noise). */
  float energy;          /**< Sum of x^2 over the open window. */
  float confidence;      /**< Heuristic 0..1 from peak / threshold. */
} dsp_event_t;

/** Capacity of the per-beat ring ({@link dsp_graph_copy_beats}). */
#define DSP_BEAT_LOG_CAP 64u

/** Outcome of feeding one tick to the beat-timing fit (see beat_fit.h). */
typedef enum {
  DSP_BEAT_ACCEPTED = 0, /**< Point joined the current grid / fit. */
  DSP_BEAT_STARTED,      /**< First point, or the grid restarted here. */
  DSP_BEAT_REJECTED      /**< Off-grid or non-increasing; not in the fit. */
} dsp_beat_status_t;

/**
 * One tick as seen by the beat-timing fit; the raw material of a tick-tock
 * (paper-strip) chart. One record per tick handed to the fit once the A/h
 * lock is established; none while unlocked.
 *
 * Grid: each {@code grid_id} has a fixed origin (the STARTED tick, k = 0,
 * u = 0) and the nominal half-period {@code nominal_us}. The fit window
 * sliding does not move the origin or renumber k.
 *
 * Residual sign and correction: {@code u_us} is the arrival time of the tick
 * minus the nominal-grid time, {@code u = T - k * nominal}, where
 * {@code T} is the unlock time since the grid origin scaled by
 * {@code 1 / (1 + timebase_ppm * 1e-6)} (the same sample-clock correction the
 * rate uses). It is NOT relative to the fitted line. A watch that gains
 * (positive s/d) has ticks arriving early, so u DECREASES over time with
 * slope {@code -rate_s_day / 86400 * nominal_us} per beat; the separation of
 * the even-k and odd-k clouds is the beat error (2|c|). If the timebase ppm
 * changes mid-grid, u steps; hosts should treat that as a new grid.
 *
 * REJECTED ticks are recorded too: k is the nearest grid index to the last
 * accepted point (last_k + round(dt / nominal), possibly the same or lower
 * than an earlier k) and u is the true distance to that grid slot, so it
 * shows the off-grid miss (|u| up to about half a beat). They are not in
 * the fit. If no grid exists (cannot happen once locked) k = u = 0.
 */
typedef struct {
  uint32_t seq;          /**< Per-stream counter, 0 at stream reset. */
  uint32_t grid_id;      /**< Bumps on every STARTED (first grid is 1). */
  uint64_t sample_index; /**< Integer part of the unlock sample time. */
  double t_s;            /**< Unlock time, seconds of signal (t / source_hz). */
  int32_t k;             /**< Half-beat grid index since the grid origin. */
  uint8_t parity;        /**< {@code k & 1}; arbitrary but stable per grid. */
  float u_us;            /**< Residual vs nominal grid, microseconds. */
  float nominal_us;      /**< Nominal half-period, microseconds. */
  float amplitude_deg;   /**< Per-tick amplitude; 0 when invalid. */
  bool amplitude_valid;  /**< Lift time gave 50..400 degrees. */
  float lift_ms;         /**< Unlock-to-drop lift time; 0 if unavailable. */
  dsp_beat_status_t status;
} dsp_beat_t;

/**
 * One timing view: instant snapshot or a rolling time-average window.
 * Invalid metrics are zeroed with the matching {@code *_valid} flag false.
 */
typedef struct {
  float rate_s_day;    /**< Daily rate deviation (s/d). */
  float beat_error_ms; /**< Beat error / repère (ms). */
  float amplitude_deg; /**< Balance amplitude (degrees). */
  /**
   * Beat-error angle (degrees): amplitude × π × vph / 7200 × BE (s), so beat
   * errors of different amplitudes compare on one scale (opentimegrapher M5).
   * Rolling: mean of the per-tick angles where both BE and amplitude were
   * valid.
   */
  float beat_error_angle_deg;
  uint32_t samples;    /**< Instant: 0/1. Rolling: count contributing ticks. */
  bool rate_valid;
  bool beat_error_valid;
  bool amplitude_valid;
  /** True when beat error and amplitude are valid (and a lock exists). */
  bool beat_error_angle_valid;
} dsp_beat_timing_window_t;

/**
 * Why the beat analysis started a new stretch (a "cut"). Nothing is fitted or
 * averaged across a cut: the rate fit, the rolling means, the amplitude
 * history and, when the nominal rate changed, the stage-B templates start
 * empty. See {@code beat_analysis.h}.
 */
typedef enum {
  /** No cut since the stream reset. */
  DSP_CUT_NONE = 0,
  /** Stream reset (capture start); not counted in {@code cuts}. */
  DSP_CUT_RESET,
  /** More than 2 s without an accepted beat. */
  DSP_CUT_GAP,
  /** Timing jump: consecutive beats off the current grid. */
  DSP_CUT_JUMP,
  /** The nominal A/h changed (another watch or rate). */
  DSP_CUT_RATE,
  /** Timebase correction or its verification changed. */
  DSP_CUT_CLOCK,
  /** The lock on a nominal rate was lost. */
  DSP_CUT_LOCK_LOST,
  /** The no-watch rule fired. */
  DSP_CUT_NO_WATCH,
} dsp_cut_reason_t;

/**
 * Coarse state of the beat analysis, for hosts and displays.
 *
 *   - {@code LISTENING}: no tick since the stream reset, or the capture rate
 *     is unverified.
 *   - {@code LOCKING}: ticks seen, but either no lock on a nominal rate or the
 *     current stretch is shorter than 5 s (no rate or beat-error reading yet).
 *   - {@code LOCKED}: locked and the current stretch spans at least 5 s
 *     (readings exist; {@code settled} says whether they are final).
 *   - {@code NO_WATCH}: the no-watch rule fired and no lock has returned.
 *
 * With the timing engine off (USB microphone mode) the state follows the lock
 * alone: {@code LOCKED} when locked, {@code LOCKING} after the first tick
 * without a lock, {@code LISTENING} before it. No stretch exists then, so the
 * 5 s rule and {@code NO_WATCH} do not apply.
 */
typedef enum {
  DSP_BEAT_STATE_LISTENING = 0,
  DSP_BEAT_STATE_LOCKING,
  DSP_BEAT_STATE_LOCKED,
  DSP_BEAT_STATE_NO_WATCH,
} dsp_beat_state_t;

/**
 * Where the reported beat error comes from.
 *
 *   - {@code UNLOCK}: the unlock landmark: the unlock-timed fit, or, in drop
 *     mode, the median over 2 s shape windows of the drop beat error corrected
 *     by each side's unlock-to-drop time.
 *   - {@code DROP}: the drop-timed fit (drop mode with fewer than 3 usable
 *     windows, or windows that disagree in sign beyond their spread).
 */
typedef enum {
  DSP_BE_SOURCE_UNLOCK = 0,
  DSP_BE_SOURCE_DROP,
} dsp_be_source_t;

/**
 * Escapement timing estimate from unlock-based intervals (Witschi-style).
 * Updated when enough recent intervals exist and beat_rate is enabled.
 *
 * Lock and {@code vph_raw} come from the median unlock interval. Once locked,
 * rate and beat error come from a least-squares fit of sub-sample unlock
 * times over the last {@code window_s} seconds (see mechanical/beat_fit.h),
 * so they resolve well below one sample period.
 *
 * Top-level rate/beat-error/amplitude fields are the <em>instant</em> view
 * (last computed tick set). {@code avg_1s} / {@code avg_long} are arithmetic
 * means of those instant values over rolling wall-clock windows: 1 s, and
 * the configurable integration time {@code avg_s} (beat param, default 10 s).
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
  /**
   * Measured frame rate of the capture clock (Hz), or 0 where the source
   * rate is fixed by design (ADC, synthetic).
   */
  float source_hz_measured;
  /**
   * False while the capture rate is unverified or does not match
   * {@code source_hz}; no lock or timing results are produced then.
   */
  bool timebase_ok;
  uint32_t rate_points; /**< Ticks in the rate fit window (0 = no fit). */
  float jitter_us;      /**< RMS residual about the fit (µs). */
  /**
   * Beats timed at the drop by the beat-template stage since the last stream
   * reset ({@code timingLandmark = drop}); 0 with the unlock landmark.
   */
  uint32_t drop_timed_beats;
  /**
   * Beats that were to be timed at the drop but fell back to the unlock
   * (template still learning, correlation below 0.4, or the beat outside the
   * envelope ring) since the last stream reset. Such a beat keeps the unlock
   * plus the side's mean lift once the fit runs on drop times.
   */
  uint32_t drop_fallbacks;
  /** Analysis state (see {@link dsp_beat_state_t}). */
  dsp_beat_state_t state;
  /**
   * The timing engine runs. False in USB microphone mode
   * ({@link dsp_graph_set_analysis}): only tick detection and the A/h lock are
   * live then ({@code locked}, {@code vph_raw}, {@code vph_nearest},
   * {@code confidence}, {@code interval_samples}, {@code state}). Rate, beat
   * error, amplitude and the rolling means are invalid and no beat records
   * are produced. Reported as {@code analysis: "on" | "off"}.
   */
  bool analysis;
  /**
   * Cuts since the stream reset (the reset itself is not counted). A cut is
   * counted only when it ended a stretch that had beats.
   */
  uint32_t cuts;
  /** Reason of the latest cut; {@code DSP_CUT_RESET} after a stream reset. */
  dsp_cut_reason_t last_cut;
  /**
   * Span of the current stretch (s): first to latest accepted beat since the
   * last cut, on the corrected clock; 0 when no stretch is open. Rate and beat
   * error are reported from 5 s on (provisional) and are {@code settled} from
   * 10 s on.
   */
  float stretch_s;
  /**
   * The stretch spans at least 10 s: rate and beat error are no longer
   * provisional. False before that and while unlocked. Hosts may grey out
   * unsettled readings.
   */
  bool settled;
  /**
   * True once the fit runs on drop times ({@code timingLandmark = drop} and the
   * handover has happened); false while it runs on unlock times. Reported as
   * {@code timingSource}.
   */
  bool timing_drop;
  /**
   * Handovers from unlock to drop times since the stream reset (the fit
   * continues across one; only the rolling beat-error samples are cleared).
   */
  uint32_t handovers;
  float cal_ppm;          /**< Sample-clock correction applied (ppm, + = fast). */
  const char *cal_source; /**< "none" / "manual" / "ntp" / "pps" (static string). */

  float rate_s_day;     /**< Instant daily rate (s/d); alias of instant.rate_s_day. */
  float beat_error_ms;  /**< Instant beat error (ms). */
  float amplitude_deg;  /**< Instant amplitude (degrees). */
  /**
   * Amplitude of the Tick and Tock side (degrees): median over the 2 s shape
   * windows of the averaging span (drop mode only, {@code timingLandmark} 1).
   * Tick is the side that comes late; see {@code beat_error_signed_ms}.
   */
  float amplitude_tick_deg;
  float amplitude_tock_deg;
  bool amplitude_tick_valid; /**< {@code amplitude_tick_deg} is a reading. */
  bool amplitude_tock_valid; /**< {@code amplitude_tock_deg} is a reading. */
  /** Windows of the span that gave an amplitude on both sides. */
  uint32_t amplitude_windows_used;
  /** Windows of the span that held any beat. */
  uint32_t amplitude_windows_total;
  /**
   * Signed beat error (ms): positive when the Tick comes late. Tick is the side
   * that came late over the first seconds of the watch and keeps that label
   * until the nominal rate changes, so the value starts positive and can turn
   * negative. Valid with {@code beat_error_valid}; its magnitude is
   * {@code beat_error_ms}. Unlock mode: always the magnitude.
   */
  float beat_error_signed_ms;
  dsp_be_source_t beat_error_source; /**< Where the beat error comes from. */
  float lift_angle_deg; /**< Configured lift angle λ used for amplitude. */
  float lift_time_ms;   /**< Latest unlock→drop lift time (ms). */
  /** Instant beat-error angle (deg); same as instant.beat_error_angle_deg. */
  float beat_error_angle_deg;
  bool rate_valid;
  bool beat_error_valid;
  bool amplitude_valid;
  /** True when beat error and amplitude are valid and locked. */
  bool beat_error_angle_valid;

  dsp_beat_timing_window_t instant; /**< Last computed tick set (same as top-level). */
  dsp_beat_timing_window_t avg_1s;  /**< Rolling ~1 s mean of instant values. */
  dsp_beat_timing_window_t avg_long; /**< Rolling mean over {@code avg_s} seconds. */
  float avg_s;                       /**< Integration time of {@code avg_long} (s). */
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

  /* Quartz engine (appended; PCM1808 builds only, else all zero). One
   * quartz::engine::process() call per capture block, timed separately
   * from dsp_block_* because it runs after the USB samples are queued. */
  uint32_t quartz_block_us_last; /**< Last call wall time (µs). */
  uint32_t quartz_block_us_mean; /**< EMA of call wall time (µs). */
  uint32_t quartz_block_us_max;  /**< Max since last reset / window clear. */
  uint32_t quartz_blocks;        /**< Calls since reset. */
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
