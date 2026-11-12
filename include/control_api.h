/**
 * @file control_api.h
 * @brief Transport-agnostic DSP control plane (CDC CLI + WebHID).
 *
 * All DSP enable/param/meter/event access from hosts should go through this
 * façade rather than calling {@link dsp_graph} directly. Handlers must not
 * assume CDC {@code Serial} — formatting belongs in the CLI or HID JSON
 * codec. This API only mutates graph state and returns structured results.
 *
 * HID / WebHID protocol: {@code docs/HID_CONTROL_PLANE.md}.
 *
 * @see dsp_graph.h, serial_cli.cpp, cli/cli_commands.cpp
 */

#pragma once

#include "dsp_types.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initializes the underlying DSP graph ({@link dsp_graph_init}). */
void control_api_init(void);

/**
 * Polls USB continuity / auto-eject. Call once per CLI tick (or future HID
 * task); safe when idle.
 */
void control_api_watchdog_poll(void);

/**
 * @return {@code true} when master process bypass is on (analysis may still
 *         run; PCM process path is identity).
 */
bool control_api_get_bypass(void);

/** Sets master process bypass without changing per-block enable flags. */
void control_api_set_bypass(bool on);

/** @return Whether {@code id} is enabled in the graph. */
bool control_api_block_enabled(dsp_block_id_t id);

/**
 * Enables or disables {@code id}. Same rules as
 * {@link dsp_graph_set_block_enabled} (process enables clear bypass).
 */
bool control_api_set_block_enabled(dsp_block_id_t id, bool on);

/**
 * Stable CLI / HID name for {@code id} (e.g. {@code "gain"}).
 * @return Non-null C string; {@code "unknown"} for out-of-range ids.
 */
const char *control_api_block_name(dsp_block_id_t id);

/**
 * Case-insensitive lookup of a block name.
 * @return Matching id, or {@code DSP_BLOCK_COUNT} if unknown / null.
 */
dsp_block_id_t control_api_block_from_name(const char *name);

/**
 * Writes a block parameter (see {@link dsp_graph_set_param} for names).
 * @return {@code false} on unknown id/param or out-of-range value.
 */
bool control_api_set_param(dsp_block_id_t id, const char *param, float value);

/**
 * Reads a block parameter into {@code *out}.
 * @return {@code false} on unknown id/param or null {@code out}.
 */
bool control_api_get_param(dsp_block_id_t id, const char *param, float *out);

/** Copies current level / clip meters. */
void control_api_get_metrics(dsp_metrics_t *out);

/** Copies DSP block + main-loop CPU timing / headroom snapshot. */
void control_api_get_timing(dsp_timing_t *out);

/** Clears timing maxima, EMAs, and the 1 s busy window. */
void control_api_reset_timing(void);

/** Copies the current beat-rate estimate. */
void control_api_get_beat_rate(dsp_beat_rate_t *out);

/** Copies last spectrum peak list. */
void control_api_get_spectrum(dsp_spectrum_t *out);

/** Copies matched-filter state. */
void control_api_get_matched(dsp_matched_t *out);

/**
 * Copies up to {@code max_n} recent analysis events (newest first).
 * @return Number copied.
 */
uint32_t control_api_copy_events(dsp_event_t *out, uint32_t max_n);

/**
 * Copies up to {@code max_n} recent auto-eject / fault entries (newest first).
 * @return Number copied.
 */
uint32_t control_api_copy_errors(dsp_error_t *out, uint32_t max_n);

/** Clears the fault log. */
void control_api_clear_errors(void);

/**
 * Pops one pending async error string for the CLI (NUL-terminated).
 * @return {@code true} if a message was written to {@code buf}.
 */
bool control_api_pop_async_message(char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif
