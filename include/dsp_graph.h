/**
 * @file dsp_graph.h
 * @brief Fixed-order float32 analysis + process graph before PCM24 output.
 *
 * Capture backends convert native samples to float full-scale (~[-1,1]), call
 * {@link dsp_graph_process}, and push the returned signed PCM24 samples into
 * {@code audio_ring}. USB never sees float; it only packs PCM24.
 *
 * Per-block order inside {@link dsp_graph_process}:
 *   1. Analysis (meters / noise / event detect / beat rate) — no PCM mutate
 *   2. Process (filter → gain → limiter) — skipped while master bypass is on
 *   3. Quantize to signed 24-bit (int32 container)
 *
 * Soft gate is reserved and cannot be enabled yet.
 *
 * @see dsp_types.h, control_api.h, pcm1808_capture.cpp
 */

#pragma once

#include "dsp_types.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Resets enable flags, meters, event/beat/error logs, and filter state.
 * Defaults: master bypass on; level_meter, event_detect, beat_rate, and
 * limiter enabled (limiter is sat-safety for when process path is armed).
 */
void dsp_graph_init(void);

/**
 * Sets the sample rate of samples fed to {@link dsp_graph_process} for timing
 * math (event pre-roll/tail, beat intervals → vph).
 *
 * USB / ring rate is always {@code APP_SAMPLE_RATE_HZ} (48 kHz). On the
 * PCM1808 path with {@code PCM1808_OUTPUT_UPSAMPLE=2}, analysis runs on
 * pre-upsample frames (~24 kHz) — call this with {@code 48000/upsample}.
 * Default after init is {@code DSP_SAMPLE_RATE_HZ} (48000).
 */
void dsp_graph_set_source_rate(uint32_t hz);

/** @return Rate last set by {@link dsp_graph_set_source_rate}. */
uint32_t dsp_graph_source_rate(void);

/**
 * Runs one DSP block on mono float full-scale input and writes PCM24 out.
 *
 * Copies {@code in_fs} into an internal work buffer so analysis and process
 * share one contiguous frame. {@code source_index} is the absolute capture
 * index of {@code in_fs[0]} (used for event timestamps).
 *
 * When master bypass is on, process blocks are skipped and optional
 * {@code PCM1808_PCM_GAIN} is applied at quantize for loudness parity.
 * When bypass is off, gain lives in the gain block instead.
 *
 * @param in_fs Float samples already scaled to ~[-1, 1] (length ≥ n).
 * @param n Sample count; must be in 1..{@code DSP_MAX_BLOCK}.
 * @param source_index Absolute source index of {@code in_fs[0]}.
 * @param out_pcm Destination PCM24 buffer (int32 containers, length ≥ n).
 * @return Samples written ({@code n}), or 0 if arguments are invalid.
 */
uint32_t dsp_graph_process(const float *in_fs, uint32_t n, uint64_t source_index,
                           int32_t *out_pcm);

/**
 * USB continuity watchdog. If ring underruns or USB short-writes spike within
 * a few seconds of enabling a process block, that block is disabled, a fault
 * is logged, and an async CLI message is queued. Call from the CLI tick
 * (via {@link control_api_watchdog_poll}).
 */
void dsp_graph_watchdog_poll(void);

/**
 * Records that {@code id} was just enabled so the watchdog can attribute a
 * near-term USB continuity failure to that block.
 */
void dsp_graph_note_enable(dsp_block_id_t id);

/** Copies current pre/post peak, RMS EMA, clip and limiter counters. */
void dsp_graph_get_metrics(dsp_metrics_t *out);

/** Copies the latest beat-rate estimate (may be unlocked / zeroed). */
void dsp_graph_get_beat(dsp_beat_rate_t *out);

/**
 * Copies up to {@code max_n} recent events, newest first.
 * @return Number of events written to {@code out}.
 */
uint32_t dsp_graph_copy_events(dsp_event_t *out, uint32_t max_n);

/**
 * @return {@code true} when the process path is identity (analysis may still
 *         run).
 */
bool dsp_graph_get_bypass(void);

/** Sets master process-path bypass without changing per-block enable flags. */
void dsp_graph_set_bypass(bool on);

/**
 * @return {@code true} if {@code id} is currently enabled.
 */
bool dsp_graph_block_enabled(dsp_block_id_t id);

/**
 * Enables or disables one block. Enabling gain / limiter / filter_bank clears
 * master bypass and arms the continuity watchdog. Soft gate cannot be enabled.
 *
 * @return {@code false} for invalid / reserved ids.
 */
bool dsp_graph_set_block_enabled(dsp_block_id_t id, bool on);

/**
 * Writes a named parameter on {@code id}.
 * Supported today: gain {@code db}; filter_bank {@code preset} (0..3);
 * event_detect {@code threshold}, {@code preroll_ms}, {@code tail_ms}.
 * Gain / filter writes also enable that block and clear bypass.
 *
 * @return {@code false} if the id/param pair is unknown or out of range.
 */
bool dsp_graph_set_param(dsp_block_id_t id, const char *param, float value);

/**
 * Reads a named parameter into {@code *out} (same names as set_param).
 * @return {@code false} if unknown or {@code out} is null.
 */
bool dsp_graph_get_param(dsp_block_id_t id, const char *param, float *out);

/**
 * Copies up to {@code max_n} recent faults, newest first.
 * @return Number written to {@code out}.
 */
uint32_t dsp_graph_copy_errors(dsp_error_t *out, uint32_t max_n);

/** Clears the fault log (does not re-enable ejected blocks). */
void dsp_graph_clear_errors(void);

/**
 * If the watchdog queued a one-shot CLI string, copies it to {@code buf} and
 * clears the pending flag.
 * @return {@code true} when a message was consumed.
 */
bool dsp_graph_pop_async_message(char *buf, size_t buflen);

/**
 * Converts a sign-extended PCM1808 left 24-bit sample to float full-scale.
 * Scale is {@code 1/8388608} so ±full-scale maps to ~±1.0.
 */
float dsp_sample_from_pcm1808_left24(int32_t left24);

/**
 * Converts DC-centered onboard ADC counts to approximate float full-scale.
 * Scale is {@code 1/2048} (±2048 counts ≈ ±1.0).
 */
float dsp_sample_from_adc_centered(int32_t centered_counts);

#ifdef __cplusplus
}
#endif
