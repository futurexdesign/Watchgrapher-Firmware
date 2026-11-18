/**
 * @file dsp_blocks.h
 * @brief Single source of truth for DSP block names and parameter keys.
 *
 * Every transport (CDC CLI, WebHID JSON) and the DSP graph's own fault
 * messages resolve block / parameter names through this table, so adding a
 * block or parameter is a one-place change here plus its implementation in
 * {@code dsp_graph_set_param} / {@code dsp_graph_get_param}.
 *
 * @see dsp_types.h, control_api.h, docs/HID_CONTROL_PLANE.md
 */

#pragma once

#include "dsp_types.h"

#include <stddef.h>
#include <stdint.h>

/** One block parameter: CLI / graph key and WebHID JSON key. */
typedef struct {
  const char *name;      /**< snake_case key used by the CLI and dsp_graph. */
  const char *json_name; /**< camelCase key used by the WebHID JSON protocol. */
  /**
   * False when patchState must not write it: read-only telemetry (agc
   * gain_db), or a value owned by another store (beat_rate cal_ppm, which
   * has its own timebase commands).
   */
  bool writable;
} dsp_param_info_t;

/** Static description of one DSP block. */
typedef struct {
  dsp_block_id_t id;
  const char *name;      /**< snake_case CLI name (e.g. "filter_bank"). */
  const char *json_name; /**< camelCase WebHID name (e.g. "filterBank"). */
  bool is_process;       /**< True if the block mutates PCM (process path). */
  const dsp_param_info_t *params;
  uint8_t param_count;
} dsp_block_info_t;

/**
 * @return Metadata for {@code id}, or {@code nullptr} if out of range.
 *         Entries exist for every id in {@code [0, DSP_BLOCK_COUNT)}.
 */
const dsp_block_info_t *dsp_block_info(dsp_block_id_t id);

/** @return CLI name for {@code id}, or {@code "unknown"} if out of range. */
const char *dsp_block_name(dsp_block_id_t id);

/** @return WebHID JSON name for {@code id}, or {@code "unknown"}. */
const char *dsp_block_json_name(dsp_block_id_t id);

/** @return {@code true} if {@code id} mutates PCM (process path). */
bool dsp_block_is_process(dsp_block_id_t id);
