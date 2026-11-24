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

#include "device_mode.h"
#include "dsp_blocks.h"
#include "dsp_quartz_types.h"
#include "dsp_types.h"
#include "timebase.h"

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

/* ------------------------------------------------------------------------
 * Quartz-watch timing (stepper motor, LCD, tuning fork).
 *
 * Backed by quartz::engine (quartz/quartz_engine.h) on PCM1808 builds. On
 * other audio sources there is no quartz engine: the mode stays
 * mechanical, setters refuse anything else, and getters report idle. The
 * name helpers work on every build.
 * --------------------------------------------------------------------- */

/**
 * @return {@code true} when this build has the quartz engine (PCM1808
 *         builds); {@code false} where every quartz setter refuses and the
 *         snapshot is always idle. Lets a client hide the quartz UI instead
 *         of probing with a mode change.
 */
bool control_api_quartz_supported(void);

/**
 * Copies the quartz measurement snapshot: mode, state and reason, gating
 * inputs, parameters, the headline rate with its validity, the calibration
 * applied, and the detail of the active mode ({@code motor}, {@code lcd} or
 * {@code fork}; the others are zero). See {@link dsp_quartz_t} for every
 * field and its unit.
 *
 * @param out Destination; {@code NULL} is ignored.
 * @see control_api_copy_quartz_pulses
 */
void control_api_get_quartz(dsp_quartz_t *out);

/**
 * Copies up to {@code max_n} recent motor pulses, newest first (at most
 * {@link DSP_QUARTZ_PULSE_LOG_CAP}). Nothing outside motor mode or before a
 * measurement has started.
 *
 * @param out Destination array of at least {@code max_n} entries.
 * @param max_n Capacity of {@code out}.
 * @return Number of entries written.
 */
uint32_t control_api_copy_quartz_pulses(dsp_quartz_pulse_t *out,
                                        uint32_t max_n);

/**
 * Selects the quartz mode. A different mode starts a new measurement;
 * selecting the current mode again is a no-op (see
 * {@link control_api_quartz_restart}). {@code DSP_QUARTZ_MODE_MECHANICAL}
 * stops quartz analysis; the mechanical beat_rate path runs in every mode.
 *
 * @param mode New mode.
 * @return {@code false} if {@code mode} is out of range, or is not
 *         mechanical on a build without the quartz engine.
 */
bool control_api_set_quartz_mode(dsp_quartz_mode_t mode);

/** @return The active quartz mode (mechanical on builds without it). */
dsp_quartz_mode_t control_api_get_quartz_mode(void);

/**
 * @param mode Mode.
 * @return Its stable CLI / HID name ({@code "mechanical"}, {@code "motor"},
 *         {@code "lcd"}, {@code "fork"}), or {@code "unknown"}.
 */
const char *control_api_quartz_mode_name(dsp_quartz_mode_t mode);

/**
 * Case-insensitive lookup of a quartz mode name.
 *
 * @param name Name; {@code NULL} is treated as unknown.
 * @return The mode, or {@code DSP_QUARTZ_MODE_COUNT} if unknown.
 */
dsp_quartz_mode_t control_api_quartz_mode_from_name(const char *name);

/**
 * @param state State.
 * @return Its stable camelCase name ({@code "idle"}, {@code "waitTimebase"},
 *         {@code "unsupportedRate"}, {@code "acquiring"},
 *         {@code "measuring"}, {@code "done"}), or {@code "unknown"}.
 */
const char *control_api_quartz_state_name(dsp_quartz_state_t state);

/**
 * @param status Handoff §5 inhibition status.
 * @return Its stable snake_case name ({@code "none"},
 *         {@code "no_jump_long_gate"}, {@code "may_include_inhibition"},
 *         {@code "need_one_more_cycle"}, {@code "correction_pending"},
 *         {@code "corrected"}), or {@code "unknown"}.
 */
const char *control_api_quartz_inhibition_name(dsp_quartz_inhibition_t status);

/**
 * @param method Timestamp method.
 * @return {@code "threshold"} / {@code "xcorr"}, or {@code "unknown"}.
 */
const char *control_api_quartz_ts_method_name(dsp_quartz_ts_method_t method);

/**
 * Case-insensitive lookup of a timestamp-method name ({@code "threshold"},
 * {@code "xcorr"}). The result's numeric value is the {@code tsMethod}
 * parameter value.
 *
 * @param name Name; {@code NULL} is treated as unknown.
 * @return The method, or {@code DSP_QUARTZ_TS_COUNT} if unknown.
 */
dsp_quartz_ts_method_t control_api_quartz_ts_method_from_name(const char *name);

/**
 * @param status LCD progress.
 * @return {@code "detecting_period"}, {@code "no_pattern"},
 *         {@code "unsupported_rate"} or {@code "tracking"}, or
 *         {@code "unknown"}.
 */
const char *control_api_quartz_lcd_status_name(dsp_quartz_lcd_status_t status);

/**
 * Sets one quartz parameter. Names are camelCase with snake_case aliases,
 * as for the beat_rate block:
 *  - {@code gate} (motor, s): 0 = until two inhibition events, or 3..960
 *    (10 and 60 are the usual choices). Reaching it freezes the result
 *    ({@code done}).
 *  - {@code forkHz} / {@code fork_hz} (Hz): 300, 341.33 (= 1024/3), 360,
 *    480 or 720, within 0.01 Hz. Default 360.
 *  - {@code tsMethod} / {@code ts_method}: 0 = threshold (default),
 *    1 = xcorr ({@link dsp_quartz_ts_method_t}).
 *  - {@code kSigma} / {@code k_sigma}: motor detection threshold in noise
 *    sigmas, 4..20. Default 8.
 * A change to a parameter the active mode uses restarts its measurement.
 *
 * @param param Name.
 * @param value New value; non-finite values are rejected.
 * @return {@code false} for an unknown name, an out-of-range value, or a
 *         build without the quartz engine.
 */
bool control_api_set_quartz_param(const char *param, float value);

/**
 * {@link control_api_set_quartz_param} with the reason for a refusal, so a
 * CLI or HID client can say "unknown parameter" vs "value out of range".
 *
 * @param param Name.
 * @param value New value.
 * @return {@code DSP_QUARTZ_PARAM_OK}, {@code DSP_QUARTZ_PARAM_UNKNOWN_NAME}
 *         (also for {@code NULL}), {@code DSP_QUARTZ_PARAM_OUT_OF_RANGE}
 *         (also for a non-finite value), or {@code DSP_QUARTZ_PARAM_NO_ENGINE}
 *         on a build without the quartz engine.
 */
dsp_quartz_param_result_t control_api_set_quartz_param_ex(const char *param,
                                                          float value);

/**
 * @param result Parameter-write outcome.
 * @return Its stable name ({@code "ok"}, {@code "unknownName"},
 *         {@code "outOfRange"}, {@code "noEngine"}), or {@code "unknown"}.
 */
const char *
control_api_quartz_param_result_name(dsp_quartz_param_result_t result);

/**
 * Reads one quartz parameter (same names as
 * {@link control_api_set_quartz_param}).
 *
 * @param param Name.
 * @param out Receives the value; {@code NULL} is rejected.
 * @return {@code false} for an unknown name, a {@code NULL} argument, or a
 *         build without the quartz engine.
 */
bool control_api_get_quartz_param(const char *param, float *out);

/**
 * Starts a new measurement in the active quartz mode, also after a motor
 * gate has frozen the result.
 *
 * @return {@code false} in mechanical mode or on a build without the
 *         quartz engine.
 */
bool control_api_quartz_restart(void);

/* ------------------------------------------------------------------------
 * Device mode and run control (device_mode.h).
 *
 * Capture and analysis run while the USB audio stream is open or the run
 * latch is set. In USB_MIC mode the stream drives capture; STANDALONE
 * latches the run at boot and when selected.
 * --------------------------------------------------------------------- */

/** Fills {@code out} with the device status ({@link device_status_get}). */
void control_api_get_device(device_status_t *out);

/**
 * Selects and persists the device mode ({@link device_mode_set}); stops
 * capture around the flash write.
 *
 * @return OK, BUSY while a USB stream is open, FAILED, BAD_PARAM.
 */
device_mode_set_result_t control_api_set_device_mode(device_mode_t mode);

/** Sets the run latch ({@link device_run_start}). */
device_run_result_t control_api_run_start(void);

/**
 * Clears the run latch ({@link device_run_stop}). Capture keeps running
 * while the USB stream is open, so in USB_MIC mode with a host stream the
 * status still reads {@code running}.
 */
void control_api_run_stop(void);

/** @return {@code "usbMic"} or {@code "standalone"}. */
const char *control_api_device_mode_name(device_mode_t mode);

/**
 * Case-insensitive mode lookup ({@code usbMic}, {@code usb_mic}, {@code usb},
 * {@code standalone}, {@code sa}).
 *
 * @return {@code true} and sets {@code *out} on a match.
 */
bool control_api_device_mode_from_name(const char *name, device_mode_t *out);

/* ------------------------------------------------------------------------
 * Per-beat records (dsp_beat_t, 64-entry ring in the graph).
 * --------------------------------------------------------------------- */

/**
 * Copies up to {@code max_n} recent beats, newest first.
 * @return Number copied.
 */
uint32_t control_api_copy_beats(dsp_beat_t *out, uint32_t max_n);

/** @return Seq the next beat will receive (subscriber baseline). */
uint32_t control_api_beat_seq(void);

/**
 * Copies beats with {@code seq >= seq}, oldest first.
 * @return Number copied (at most {@code max_n}).
 */
uint32_t control_api_copy_beats_since(uint32_t seq, dsp_beat_t *out,
                                      uint32_t max_n);

/* ------------------------------------------------------------------------
 * Pipeline graph (dsp_blocks.h).
 * --------------------------------------------------------------------- */

/**
 * @param count Receives the node count (may be null).
 * @return The static pipeline node table. It always lists the "quartz"
 *         node; callers hide it when {@link control_api_quartz_supported}
 *         is false.
 */
const dsp_pipeline_node_t *control_api_pipeline(uint32_t *count);

/* ------------------------------------------------------------------------
 * GPS and the timebase calibration log (timebase.h).
 * --------------------------------------------------------------------- */

/** @return {@code true} if the build has GPS_UART_RX (NMEA available). */
bool control_api_gps_built(void);

/** @return {@code true} if the build has PPS_GPIO. */
bool control_api_pps_built(void);

/**
 * Snapshot of the detailed GPS state.
 * @return {@code false} (out zeroed) when GPS is not built in.
 */
bool control_api_get_gps(timebase_gps_t *out);

/** Copies the calibration log (ring, oldest first via timebase_log.h). */
void control_api_timebase_log(timebase_log_t *out);

/** @return A log entry is staged but not yet in flash. */
bool control_api_timebase_log_pending(void);

/** @return GPS UTC (UNIX s) of the last log flash write, 0 = none. */
uint32_t control_api_timebase_log_last_write_unix(void);

/**
 * Clears the log in RAM and flash (pause-and-write).
 * @return OK, BUSY (USB streaming), FAILED.
 */
timebase_save_result_t control_api_timebase_log_clear(void);

/** @return The PPS log delta threshold (ppm, default 0.2). */
float control_api_timebase_log_delta_ppm(void);

/**
 * Sets and persists the PPS log delta threshold.
 * @param ppm 0.05..10.
 * @return OK, BAD_PARAM, BUSY, FAILED.
 */
timebase_log_delta_result_t control_api_set_timebase_log_delta_ppm(float ppm);

#ifdef __cplusplus
}
#endif
