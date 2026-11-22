/**
 * @file ui_snapshot.h
 * @brief Core 0 → core 1 hand-off of the values the display shows.
 *
 * Core 0 owns the DSP graph and USB; core 1 only renders. Core 0 publishes a
 * small snapshot from its main loop and core 1 copies the latest one. A
 * sequence lock makes the copy consistent without either core blocking:
 * the writer never waits, and the reader retries if it raced a write.
 *
 * Single writer (core 0 main loop), any number of readers (core 1).
 */

#pragma once

#include "dsp_quartz_types.h"
#include "dsp_types.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * Quartz values for the OLED (a subset of {@link dsp_quartz_t}). Rates are
 * calibrated s/d; {@code calibrated} false means no correction exists and
 * they must be marked uncalibrated.
 */
typedef struct {
  dsp_quartz_mode_t mode;       /**< Active mode (mechanical = show beat). */
  dsp_quartz_state_t state;     /**< Measurement state. */
  float rate_s_day;             /**< Headline rate (s/d): motor with inhibition,
                                     LCD / fork phase-slope rate. */
  bool rate_valid;              /**< {@code rate_s_day} is meaningful. */
  float raw_rate_s_day;         /**< Motor rate WITHOUT inhibition (§5 "raw"),
                                     s/d. */
  bool raw_rate_valid;          /**< Motor fit exists. */
  float inhibition_period_s;    /**< Motor inhibition period (s). */
  bool inhibition_period_valid; /**< Two or more events seen. */
  float jump_us;                /**< Last inhibition jump (µs), 0 if none. */
  uint32_t jump_n;              /**< Last jump in 8192 Hz pulses ("n"). */
  dsp_quartz_inhibition_t inhibition; /**< Handoff §5 status. */
  bool hint_weak_signal;              /**< Reposition the watch (handoff §8). */
  /** A raw code on the active channel hit the rail since the start: LEFT in
   * motor / fork mode (suggest opening JP1, handoff §8), RIGHT in LCD mode
   * (suggest opening JP2 or moving the watch away, handoff §2). */
  bool hint_clipped;
  bool calibrated; /**< A timebase correction is applied. */
  /** Signal time since the measurement started (s): samples fed / source
   * rate, so it does not depend on loop latency. Frozen at the gate in
   * {@code DSP_QUARTZ_STATE_DONE}; 0 while gated or before the first block. */
  float elapsed_s;
  /** Motor gate (s). A value ≤ 0 means "until two inhibition events";
   * otherwise 3..960 s. Published in every mode (it is a parameter), but it
   * only applies to motor mode. */
  float gate_s;
  /** Reference frequency (Hz): the detected LCD pattern rate f0 (16, 32, 64
   * or 128) in LCD mode, the nominal fork frequency in fork mode, otherwise
   * 0. In LCD mode it stays 0 until the 2 s pattern search succeeds. */
  float ref_hz;
  /** Phase blocks in the LCD (0.5 s) or fork (0.25 s) rate fit; 0 in the
   * other modes. A rate needs at least two. */
  uint32_t blocks;
  /** {@code blocks} is meaningful: LCD or fork mode with {@code ref_hz}
   * known (> 0). */
  bool blocks_valid;
} ui_quartz_t;

typedef struct {
  bool streaming;  /**< Host is recording or the device runs standalone (capture
                      + DSP). */
  bool standalone; /**< Device mode is STANDALONE (else USB mic). */
  bool timebase_ok; /**< Capture rate verified (see dsp_graph_set_timebase). */
  bool locked;      /**< Beat locked to a nominal rate. */
  uint32_t vph;     /**< Nominal beats per hour when locked, else 0. */
  uint32_t events_seen;
  float lift_angle_deg; /**< Configured lift angle λ (°). */
  bool pps_enabled;     /**< Built with PPS_GPIO. */
  bool pps_locked;      /**< Timebase disciplined by GPS 1 PPS right now. */
  /** Rolling means of rate (s/d), beat error (ms), amplitude (°) over avg_s. */
  dsp_beat_timing_window_t avg;
  float avg_s; /**< Integration time of {@code avg} (beat param avg_s). */
  ui_quartz_t quartz;    /**< Quartz mode and headline values. */
  uint32_t published_ms; /**< millis() on core 0 at publish time. */
} ui_snapshot_t;

/** Publishes {@code s}. Core 0 main loop only. */
void ui_snapshot_publish(const ui_snapshot_t *s);

/**
 * Copies the latest snapshot into {@code out}.
 *
 * @return {@code false} if no consistent copy was obtained (a write kept
 *         racing; retry later) or nothing has been published yet.
 */
bool ui_snapshot_read(ui_snapshot_t *out);
