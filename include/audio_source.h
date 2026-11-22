/**
 * @file audio_source.h
 * @brief Stream state machine and capture backend selection.
 *
 * Bridges USB mount / UAC alternate-setting events to the compile-time audio
 * backend ({@code ADC}, {@code PCM1808}, or {@code SYNTHETIC}) and keeps the
 * PCM ring prefilled while the host is recording. A separate run latch
 * ({@link audio_source_set_run_requested}) lets capture and analysis run
 * without an open USB stream (STANDALONE mode, host/CLI start).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * High-level stream lifecycle. Transitions are driven by TinyUSB callbacks
 * ({@link audio_source_set_configured}, {@link audio_source_on_stream_open},
 * etc.) and by {@link audio_source_process} once the ring is prefilled.
 */
typedef enum {
  /** USB not mounted (or just detached). Capture must be idle. */
  STREAM_STATE_USB_UNCONFIGURED = 0,
  /** Configured but UAC alt-setting 0 (not streaming). */
  STREAM_STATE_USB_CONFIGURED_IDLE,
  /** Host opened the mic; filling the ring toward {@code APP_RING_PREFILL_SAMPLES}. */
  STREAM_STATE_STREAM_PREFILL,
  /** Prefill met; USB feeder is consuming and capture should keep up. */
  STREAM_STATE_STREAM_ACTIVE,
  /** Bus suspend — capture requested off. */
  STREAM_STATE_USB_SUSPENDED,
  /** Backend failed to initialize (e.g. PCM1808 MCLK clock check). */
  STREAM_STATE_STREAM_ERROR
} stream_state_t;

/**
 * Initializes the PCM ring and the selected capture backend (ADC / PCM1808 /
 * synthetic index). Call once from {@code setup()} before USB audio init.
 */
void audio_source_init(void);

/**
 * Host selected the streaming alternate setting (UAC mic open).
 * Resets the ring / synth cursor, requests capture start, enters PREFILL.
 *
 * @note Capture hardware is started later from {@link audio_source_process}
 *       so DMA abort/start never runs inside a TinyUSB control callback.
 */
void audio_source_on_stream_open(void);

/**
 * Host closed the streaming interface (back to alt 0). Requests capture stop.
 */
void audio_source_on_stream_close(void);

/**
 * USB suspend: drop capture request and mark SUSPENDED.
 */
void audio_source_on_suspend(void);

/**
 * USB resume: return to CONFIGURED_IDLE if we were suspended (does not
 * automatically reopen the stream).
 */
void audio_source_on_resume(void);

/**
 * Main-loop tick: apply deferred capture start/stop, pull samples from the
 * backend into the ring, and advance PREFILL → ACTIVE when occupancy allows.
 *
 * Call every iteration while the device is mounted.
 */
void audio_source_process(void);

/**
 * @return Current {@link stream_state_t}.
 */
stream_state_t audio_source_state(void);

/**
 * Reflects device configuration (mount / unmount).
 *
 * @param configured {@code true} on {@code tud_mount_cb}, {@code false} on umount.
 *        When false, capture is stopped and the ring is cleared immediately.
 */
void audio_source_set_configured(bool configured);

/**
 * Sets or clears the host/CLI run latch. Capture and analysis run while the
 * USB stream is open or this latch is set (see {@link device_mode.h}). A set
 * latch starts a warm-up + capture from the next {@link audio_source_process}
 * with no host stream; clearing it stops capture unless a stream still needs
 * it. Setting the latch on an unhealthy backend drops it again at the next
 * tick (check {@link audio_source_backend_ok} first).
 *
 * @param requested New latch value.
 */
void audio_source_set_run_requested(bool requested);

/** @return The run latch (not whether capture is actually running). */
bool audio_source_run_requested(void);

/**
 * @return {@code true} while capture is running and its output is being
 *         processed (preamp warm-up finished), for any reason.
 */
bool audio_source_capture_running(void);

/** @return {@code false} if the backend failed to initialize or its clock is
 * bad. */
bool audio_source_backend_ok(void);

/**
 * Whether the capture backends must feed the PCM ring: only while a USB
 * stream is armed and the device is not in STANDALONE mode. Backends query it
 * once per block and skip {@code audio_ring_push} and the 24 to 48 kHz sample
 * repeat when it is false.
 */
bool audio_source_feeds_ring(void);

/**
 * Stops the capture hardware (if running) so a flash write can stall XIP.
 * Run latch and stream state are untouched; call
 * {@link audio_source_capture_resume} afterwards. Main-loop context only.
 *
 * @return {@code true} if capture was running.
 */
bool audio_source_capture_pause(void);

/**
 * Restarts capture stopped by {@link audio_source_capture_pause} (a fresh
 * stream: the DSP / quartz stream reset happens in the backend start). No-op
 * if the pause found nothing running.
 */
void audio_source_capture_resume(void);

#ifdef __cplusplus
}
#endif
