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

#include "dsp_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool streaming;   /**< Host is recording (capture + DSP running). */
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
