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

/**
 * Which half of {@code dsp_graph_process} a block runs in.
 *
 * The graph runs the analysis group first (read-only taps on the raw input
 * block) and then the process chain (mutates the block in place).
 */
typedef enum {
  DSP_GROUP_ANALYSIS = 0, /**< Read-only tap of the raw input block. */
  DSP_GROUP_PROCESS,      /**< Sequential in-place chain before PCM24. */
} dsp_group_t;

/** Static description of one DSP block. */
typedef struct {
  dsp_block_id_t id;
  const char *name;      /**< snake_case CLI name (e.g. "filter_bank"). */
  const char *json_name; /**< camelCase WebHID name (e.g. "filterBank"). */
  bool is_process;       /**< True if the block mutates PCM (process path). */
  const dsp_param_info_t *params;
  uint8_t param_count;
  /**
   * Execution group. {@code DSP_BLOCK_BYPASS} (the master gate, not a real
   * block) is reported as {@code DSP_GROUP_PROCESS} with order 0: it gates
   * the process chain, which occupies orders 1..7.
   */
  dsp_group_t group;
  /**
   * Position within {@link group} in execution order (ascending = earlier).
   * Analysis: 0..6. Process: bypass 0, then 1..7.
   */
  uint8_t order;
} dsp_block_info_t;

/** Role of a {@link dsp_pipeline_node_t}. */
typedef enum {
  DSP_NODE_SOURCE = 0, /**< Pseudo-node: raw capture input ("input"). */
  DSP_NODE_ANALYSIS,   /**< Analysis block (read-only tap). */
  DSP_NODE_PROCESS,    /**< Process-chain block. */
  DSP_NODE_SINK,       /**< Pseudo-node: "pcm24" or "usb". */
  DSP_NODE_QUARTZ,     /**< The quartz engine (PCM1808 builds only). */
} dsp_node_kind_t;

/**
 * One node of the pipeline graph reported to hosts (HID caps.pipeline).
 *
 * {@code key} is the block's json_name for real blocks, or one of the
 * pseudo-nodes "input", "pcm24", "usb", "quartz". {@code inputs} lists the
 * keys this node reads from (data-flow edges), all of which are nodes too.
 */
typedef struct {
  const char *key;
  uint8_t kind;  /**< A {@link dsp_node_kind_t} value. */
  uint8_t order; /**< Execution order within the node's kind/group. */
  const char *const *inputs;
  uint8_t n_inputs;
} dsp_pipeline_node_t;

/**
 * Pipeline graph, mirroring the real data flow of
 * {@code dsp_graph_process} (see dsp_blocks.cpp for the derivation).
 * Includes the "quartz" node unconditionally; callers omit it when the
 * build has no quartz engine.
 *
 * @param count Receives the node count (may be null).
 * @return Static node array.
 */
const dsp_pipeline_node_t *dsp_pipeline_nodes(uint32_t *count);

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
