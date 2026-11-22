/**
 * @file dsp_quartz_types.h
 * @brief C-compatible enums and POD structs for quartz-watch timing: the
 *        control-plane view of {@code quartz::engine} (stepper motor, LCD,
 *        tuning fork).
 *
 * These are the only quartz types that cross the {@code extern "C"}
 * control plane ({@link control_api_get_quartz},
 * {@link control_api_copy_quartz_pulses}) and the core 0 → core 1
 * {@code ui_snapshot}. Plain enums, fixed-width integers, floats and static
 * strings only, so the CLI, the HID JSON codec and the display can use them
 * without seeing any C++.
 *
 * Units and conventions:
 *  - rates are seconds per day (s/d), positive = the watch gains, as in the
 *    handoff ({@code rate_s_per_day()});
 *  - a field named {@code *_uncal_*} is measured against the board clock
 *    with no correction; the matching field without the suffix has the
 *    sample-clock correction ({@code cal_ppm}) removed. When no calibration
 *    exists ({@code calibrated} false) the correction is 0 ppm and both are
 *    equal, and the values must be shown as uncalibrated (an uncorrected
 *    board can be off by ±1.7 s/d at ±20 ppm);
 *  - "raw" in the handoff §5 sense means the rate WITHOUT inhibition (the
 *    oscillator), not "uncalibrated";
 *  - times are seconds of signal (sample count / source rate), not wall
 *    clock, so they do not depend on main-loop latency.
 *
 * @see quartz/quartz_engine.h, control_api.h, ui_snapshot.h,
 *      docs/handoff/FIRMWARE_HANDOFF.md
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Entries in the recent motor-pulse log ({@link dsp_quartz_pulse_t}). */
#define DSP_QUARTZ_PULSE_LOG_CAP 16u

/** Most LCD harmonics tracked (the reference keeps the four strongest). */
#define DSP_QUARTZ_MAX_HARMONICS 4u

/**
 * Measurement mode. Stable CLI / HID names: {@code "mechanical"},
 * {@code "motor"}, {@code "lcd"}, {@code "fork"}. Append before
 * {@code COUNT} only.
 */
typedef enum {
  /** Quartz analysis idle; the mechanical beat_rate path works as before. */
  DSP_QUARTZ_MODE_MECHANICAL = 0,
  /** Stepper-motor pulses on LEFT (inductive pickup), handoff §4/§5. */
  DSP_QUARTZ_MODE_MOTOR,
  /** LCD drive on RIGHT (capacitive pickup), handoff §6. */
  DSP_QUARTZ_MODE_LCD,
  /** Tuning-fork tone on LEFT, handoff §7. */
  DSP_QUARTZ_MODE_FORK,
  DSP_QUARTZ_MODE_COUNT
} dsp_quartz_mode_t;

/**
 * Measurement state. Stable names: {@code "idle"}, {@code "waitTimebase"},
 * {@code "unsupportedRate"}, {@code "acquiring"}, {@code "measuring"},
 * {@code "done"}.
 */
typedef enum {
  DSP_QUARTZ_STATE_IDLE = 0,         /**< Mechanical mode: nothing measured. */
  DSP_QUARTZ_STATE_WAIT_TIMEBASE,    /**< Capture rate not verified yet. */
  DSP_QUARTZ_STATE_UNSUPPORTED_RATE, /**< Verified rate is not 48 kHz. */
  DSP_QUARTZ_STATE_ACQUIRING,        /**< Running, no rate yet (see reason). */
  DSP_QUARTZ_STATE_MEASURING,        /**< A rate exists and is being refined. */
  DSP_QUARTZ_STATE_DONE,             /**< Motor gate reached; result frozen. */
  DSP_QUARTZ_STATE_COUNT
} dsp_quartz_state_t;

/**
 * What the motor result can claim about inhibition (handoff §5). Mirrors
 * {@code quartz::InhibitionStatus} plus {@code NONE} for "no fit yet".
 * Stable names: {@code "none"} and the library's {@code to_string()} labels.
 */
typedef enum {
  DSP_QUARTZ_INHIB_NONE = 0, /**< Fewer than three pulses: no fit. */
  /** No jump, gate ≥ 2 × the longest expected period: rate is final. */
  DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE,
  /** No jump in a short gate: raw rate, "may include inhibition". */
  DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION,
  /** One jump: raw rate, "need one more cycle". */
  DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE,
  /** Two jumps, waiting for the next same-polarity pulse (transient). */
  DSP_QUARTZ_INHIB_CORRECTION_PENDING,
  /** Two or more jumps: the rate with inhibition is the headline. */
  DSP_QUARTZ_INHIB_CORRECTED,
  DSP_QUARTZ_INHIB_COUNT
} dsp_quartz_inhibition_t;

/**
 * Motor-pulse leading-edge timestamp method (handoff §4). The numeric value
 * is the {@code tsMethod} parameter value. Names: {@code "threshold"},
 * {@code "xcorr"}.
 */
typedef enum {
  DSP_QUARTZ_TS_THRESHOLD = 0, /**< 50 % crossing, interpolated (default). */
  DSP_QUARTZ_TS_XCORR,         /**< Template cross-correlation. */
  DSP_QUARTZ_TS_COUNT
} dsp_quartz_ts_method_t;

/**
 * LCD measurement progress. Mirrors {@code quartz::LcdStatus}; names are
 * the library's {@code to_string()} labels.
 */
typedef enum {
  DSP_QUARTZ_LCD_DETECTING_PERIOD = 0, /**< Collecting the 2 s record. */
  DSP_QUARTZ_LCD_NO_PATTERN,           /**< No candidate passed; will retry. */
  DSP_QUARTZ_LCD_UNSUPPORTED_RATE,     /**< fs / f0 does not fit the buffer. */
  DSP_QUARTZ_LCD_TRACKING,             /**< f0 known; phase blocks running. */
  DSP_QUARTZ_LCD_COUNT
} dsp_quartz_lcd_status_t;

/**
 * Stepper-motor result (LEFT). Valid only in {@code DSP_QUARTZ_MODE_MOTOR};
 * zero otherwise. Rate fields are 0 while {@code valid} is false.
 */
typedef struct {
  bool valid;          /**< A fit exists (three or more pulses). */
  uint32_t pulses;     /**< Pulses detected since the measurement start. */
  uint32_t fit_pulses; /**< Pulses in the fit (≤ 2048; sliding window). */
  float step_s;        /**< Step interval snapped to the allowed list (s). */
  float span_s;        /**< First to last fitted pulse (s). */
  /** Rate with inhibition, calibrated (s/d): the motor headline. Equals the
   * raw rate unless {@code inhibition} is {@code CORRECTED}. */
  float rate_with_s_day;
  float rate_with_uncal_s_day;        /**< Same, board clock (s/d). */
  float rate_without_s_day;           /**< Raw oscillator rate, calibrated. */
  float rate_without_uncal_s_day;     /**< Same, board clock (s/d). */
  dsp_quartz_inhibition_t inhibition; /**< Handoff §5 status. */
  bool inhibition_period_valid;       /**< Two or more events seen. */
  float inhibition_period_s;          /**< Median event spacing (s), else 0. */
  uint32_t events;                    /**< Inhibition jumps (> 20 µs). */
  uint32_t cycles_used;  /**< Whole cycles in the corrected rate. */
  float jump_last_us;    /**< Size of the most recent jump (µs), 0 if none. */
  uint32_t jump_last_n;  /**< {@code jump_last_us} / 122.07 µs, rounded: the
                              "n" deleted at 8192 Hz. */
  float jump_mean_us;    /**< Mean jump size (µs), 0 if none. */
  uint32_t jump_mean_n;  /**< Mean jump in 8192 Hz pulses, rounded. */
  float residual_rms_us; /**< RMS fit residual (µs): per-pulse jitter. */
  float polarity_offset_us; /**< Edge offset between coil halves (µs). */
  bool clipped;             /**< A LEFT code hit the rail since start. */
  uint32_t clipped_pulses;  /**< Pulses whose group hit the rail. */
  uint32_t extra_pulses;    /**< Detection / correction bursts logged. */
  float noise_sigma;        /**< Noise sigma after the 400 Hz high-pass (codes);
                                 0 until the detector has calibrated. */
  float threshold;          /**< Detection threshold, kSigma × sigma (codes). */
  /** Median of {@code peak / noise_sigma} over the recent pulses (up to
   * {@link DSP_QUARTZ_PULSE_LOG_CAP}); 0 with no pulse. */
  float peak_snr;
  /** {@code peak_snr} is below the weak-signal limit (about 30 µV pickup
   * EMF, handoff §8): tell the user to reposition the watch. */
  bool hint_weak_signal;
  /** {@code clipped}: tell the user to open JP1 (handoff §8). */
  bool hint_clipped;
  /** Step-grid span (s) a measurement with no jump must cover before it
   * reports {@code NO_JUMP_LONG_GATE} ("no inhibition detected"): the
   * condition is {@code (n_last - n_first + 1) · step_s >= no_jump_span_s},
   * i.e. the number of steps the fitted pulses cover (missed pulses
   * included) times the step. It is twice the longest expected inhibition
   * period (handoff §5). Note it is not {@code span_s}, which is one step
   * shorter. Constant, the value the engine configures; filled in motor mode
   * in every state. */
  float no_jump_span_s;
  /** Interval excess (µs) that counts as an inhibition jump (handoff §4).
   * Constant, the value the engine configures; filled in motor mode in every
   * state. */
  float jump_threshold_us;
  /** The fit is replaying its pulse store (after a step change or a
   * re-timed pulse) and the rate fields are the last good result, not yet
   * updated with the newest pulses. Transient (well under a second at one
   * replay per capture block); a motor gate never freezes during it. */
  bool rebuilding;
} dsp_quartz_motor_t;

/** LCD result (RIGHT). Valid only in {@code DSP_QUARTZ_MODE_LCD}. */
typedef struct {
  bool valid;                     /**< A phase-slope rate exists. */
  dsp_quartz_lcd_status_t status; /**< Progress. */
  uint32_t f0_hz; /**< Pattern rate 16/32/64/128 Hz, 0 until known. */
  uint16_t harmonics[DSP_QUARTZ_MAX_HARMONICS]; /**< Harmonic numbers k. */
  uint8_t harmonic_count; /**< Valid entries in {@code harmonics}. */
  uint32_t blocks;        /**< 0.5 s phase blocks in the fit. */
  float rate_s_day;       /**< Rate, calibrated (s/d). */
  float rate_uncal_s_day; /**< Rate, board clock (s/d). */
  uint32_t retries;       /**< Pattern searches restarted after
                               {@code NO_PATTERN}. */
  bool clipped;           /**< A RIGHT code hit the rail since start. */
} dsp_quartz_lcd_t;

/** Tuning-fork result (LEFT). Valid only in {@code DSP_QUARTZ_MODE_FORK}. */
typedef struct {
  bool valid;             /**< A phase-slope rate exists (≥ 2 blocks). */
  float f_nom_hz;         /**< Nominal fork frequency (Hz). */
  uint32_t blocks;        /**< 0.25 s phase blocks in the fit. */
  float rate_s_day;       /**< Rate, calibrated (s/d). */
  float rate_uncal_s_day; /**< Rate, board clock (s/d). */
  bool clipped;           /**< A LEFT code hit the rail since start. */
} dsp_quartz_fork_t;

/**
 * Complete quartz snapshot ({@link control_api_get_quartz}). Only the
 * sub-struct of the active mode is filled; the others are zero.
 */
typedef struct {
  dsp_quartz_mode_t mode;   /**< Active mode. */
  dsp_quartz_state_t state; /**< Measurement state. */
  /** Why the state is what it is ("capture rate not verified", "waiting
   * for 3 pulses", ...); static string, never null, "" when nothing to
   * add. For display and logs only: do not parse. */
  const char *reason;
  uint32_t source_hz; /**< Analysis sample rate (Hz). */
  bool timebase_ok;   /**< Capture rate verified. */
  float elapsed_s;    /**< Signal time since the measurement start (s);
                           frozen in {@code DONE}. */
  float gate_s;       /**< Motor gate (s); 0 = until two inhibition events. */
  float fork_hz;      /**< Selected fork frequency (Hz). */
  dsp_quartz_ts_method_t ts_method; /**< Motor timestamp method. */
  float k_sigma; /**< Motor detection threshold (noise sigmas). */
  /** Capture sample-loss events (DMA overruns) that forced a restart since
   * the mode was selected. */
  uint32_t sample_loss_restarts;

  /** Headline rate of the active mode, calibrated (s/d): motor = rate with
   * inhibition, LCD and fork = the phase-slope rate. */
  float rate_s_day;
  bool rate_valid; /**< {@code rate_s_day} is meaningful. */

  float cal_ppm;          /**< Correction applied (ppm, + = board fast). */
  const char *cal_source; /**< "none" / "manual" / "ntp" / "pps"; static,
                               never null. */
  bool calibrated;        /**< False when the source is none: every rate
                               above is uncalibrated. */

  dsp_quartz_motor_t motor; /**< Motor detail. */
  dsp_quartz_lcd_t lcd;     /**< LCD detail. */
  dsp_quartz_fork_t fork;   /**< Fork detail. */
  /**
   * Identifies the current measurement. It increases by one on every
   * (re)start (mode change, restart, a parameter change that restarts the
   * active mode, capture stream reset, source-rate or timebase-verified
   * change, sample loss, a gap in the source index) and never repeats while
   * the firmware runs (it wraps after 2^32). Clients detect a new
   * measurement by a change of this value, and drop the history (plots,
   * pulse lists) tied to the old one. Re-reading the same measurement,
   * including a frozen {@code DONE} one, never changes it.
   */
  uint32_t measurement_id;
} dsp_quartz_t;

/**
 * Outcome of a quartz parameter write
 * ({@link control_api_set_quartz_param_ex}). Stable names: {@code "ok"},
 * {@code "unknownName"}, {@code "outOfRange"}, {@code "noEngine"}.
 */
typedef enum {
  DSP_QUARTZ_PARAM_OK = 0,       /**< Accepted (possibly unchanged). */
  DSP_QUARTZ_PARAM_UNKNOWN_NAME, /**< No such parameter (or a null name). */
  DSP_QUARTZ_PARAM_OUT_OF_RANGE, /**< Known name; value rejected (range,
                                      not one of the allowed values, or not
                                      finite). */
  DSP_QUARTZ_PARAM_NO_ENGINE,    /**< Build without the quartz engine. */
  DSP_QUARTZ_PARAM_COUNT
} dsp_quartz_param_result_t;

/**
 * One recent motor pulse ({@link control_api_copy_quartz_pulses}), for a
 * CLI / HID "pulses" listing.
 */
typedef struct {
  uint32_t sequence;     /**< 0-based pulse number since the start. */
  uint64_t sample_index; /**< First sample over threshold (absolute source
                              index). */
  double t_s;      /**< Leading-edge timestamp, s since the measurement start
                        (sub-sample; xcorr timestamps carry a constant
                        template offset that cancels in rates). */
  float peak;      /**< |y| at the leading-spike peak after the high-pass
                        (codes). */
  float snr;       /**< {@code peak} / noise sigma at detection. */
  int8_t polarity; /**< Sign of the leading spike, +1 / -1. */
  uint8_t extras;  /**< Correction / detection bursts in its group. */
  bool clipped;    /**< A raw code reached the rail in its group. */
  bool closed;     /**< Its 50 ms quiet gap has passed. */
  /** {@code dsp_quartz_t.measurement_id} of the measurement the pulse
   * belongs to. Sequence numbers restart at 0 with each measurement, so a
   * client that merges pulse lists keys them by (measurement_id, sequence). */
  uint32_t measurement_id;
} dsp_quartz_pulse_t;

#ifdef __cplusplus
}
#endif
