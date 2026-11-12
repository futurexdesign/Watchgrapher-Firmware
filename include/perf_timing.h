/**
 * @file perf_timing.h
 * @brief Core-0 DSP block + main-loop wall-time accounting for headroom.
 *
 * Source of truth for {@link dsp_timing_t}; control_api / CLI / HID / diag
 * all read through here. Uses {@code time_us_32()} (cheap µs timer).
 */

#pragma once

#include "dsp_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Clears all timing accumulators (call from {@link control_api_init}). */
void perf_timing_init(void);

/**
 * Records one {@link dsp_graph_process} wall time.
 *
 * @param us Elapsed microseconds for the call.
 * @param n Sample count processed.
 * @param source_hz Analysis sample rate (for budget_us = n * 1e6 / hz).
 */
void perf_timing_note_dsp_block(uint32_t us, uint32_t n, uint32_t source_hz);

/**
 * Records one main {@code loop()} iteration wall time and updates the
 * rolling 1 s busy / idle window.
 *
 * @param us Elapsed microseconds for the iteration.
 */
void perf_timing_note_loop(uint32_t us);

/** Copies a full timing snapshot (includes continuity proxies from diag). */
void perf_timing_get(dsp_timing_t *out);

/** Clears block/loop stats and the 1 s busy window (maxima and EMAs). */
void perf_timing_reset(void);

/**
 * Clears window maxima only (keeps EMA / last / busy_pct). Used after a
 * periodic diag print so the next window shows fresh peaks.
 */
void perf_timing_clear_window_max(void);

#ifdef __cplusplus
}
#endif
