/**
 * @file device_mode.h
 * @brief Persisted device mode (USB_MIC / STANDALONE) and capture run control.
 *
 * Capture and analysis run while {@code usb_streaming || run_requested}.
 * - {@link DEVICE_MODE_USB_MIC}: capture follows the USB audio stream (the
 *   classic behaviour); a host/CLI {@link device_run_start} also runs it
 *   without a stream.
 * - {@link DEVICE_MODE_STANDALONE}: the device measures on its own. The run
 *   latch is set at boot and whenever this mode is selected. The capture
 *   backends skip the PCM ring and the 24 to 48 kHz repeat, and USB IN
 *   packets carry silence if a host opens the stream anyway.
 *
 * The mode and the timebase-log delta threshold live in one record in the
 * DEVICE region of {@link flash_store.h}.
 */

#pragma once

#include "audio_source.h"
#include "timebase_log.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Device operating mode. Values are persisted; do not renumber. */
typedef enum {
  /** Capture runs while the USB host has the audio stream open. */
  DEVICE_MODE_USB_MIC = 0,
  /** Capture and analysis run without a host stream; started at boot. */
  DEVICE_MODE_STANDALONE
} device_mode_t;

/** Outcome of the setters that persist to flash. */
typedef enum {
  DEVICE_SET_OK = 0,
  /** USB audio is streaming; close the stream first. */
  DEVICE_SET_BUSY,
  /** The flash write failed; the previous value is kept. */
  DEVICE_SET_FAILED,
  /** Argument out of range. */
  DEVICE_SET_BAD_PARAM
} device_mode_set_result_t;

/** Outcome of {@link device_run_start}. */
typedef enum {
  DEVICE_RUN_OK = 0,
  /** The capture backend is unhealthy (init or clock failure). */
  DEVICE_RUN_BACKEND_ERROR
} device_run_result_t;

/** Snapshot for the control plane (DeviceStatus). */
typedef struct {
  device_mode_t mode;
  /** Capture and analysis currently running, for any reason. */
  bool running;
  /** Host/CLI start latch. */
  bool run_requested;
  /** USB host has the audio stream open. */
  bool usb_streaming;
  stream_state_t stream_state;
  /** Flash holds the current mode. */
  bool saved;
} device_status_t;

/** Default timebase-log delta threshold (ppm). */
#define DEVICE_LOG_DELTA_PPM_DEFAULT TIMEBASE_LOG_DELTA_PPM_DEFAULT

/**
 * Loads the settings record (default: USB_MIC, 0.2 ppm). Call after
 * {@code flash_store_init}; the boot auto-run is {@link device_boot_run}.
 */
void device_mode_init(void);

/**
 * Starts the run latch if the loaded mode is STANDALONE. Call once from
 * {@code setup()} after {@code audio_source_init}.
 */
void device_boot_run(void);

/** @return The current mode (cheap; safe to poll once per capture block). */
device_mode_t device_mode_get(void);

/**
 * Switches mode and persists it. Setting the current mode is a no-op that
 * returns OK. Stops capture around the flash write and re-arms afterwards.
 * Entering STANDALONE sets the run latch; entering USB_MIC clears it.
 *
 * @return OK, BUSY while the USB stream is open, FAILED, BAD_PARAM.
 */
device_mode_set_result_t device_mode_set(device_mode_t mode);

/** Sets the run latch (no-op if set). @return BACKEND_ERROR if unhealthy. */
device_run_result_t device_run_start(void);

/** Clears the run latch; capture stops unless the USB stream needs it. */
void device_run_stop(void);

/** Fills {@code out} with the current status. Null-safe. */
void device_status_get(device_status_t *out);

/** @return "usbMic" or "standalone". */
const char *device_mode_name(device_mode_t mode);

/**
 * Parses a mode name, case-insensitively: "usbMic", "usb_mic", "usb" or
 * "standalone", "sa".
 *
 * @return {@code true} and sets {@code *out} on a match.
 */
bool device_mode_from_name(const char *name, device_mode_t *out);

/** @return The timebase-log delta threshold (ppm, default 0.2). */
float device_log_delta_ppm_get(void);

/**
 * Sets and persists the timebase-log delta threshold.
 *
 * @param ppm Valid per {@link timebase_log_delta_valid} (0.05..10, finite).
 * @return OK, BUSY (USB streaming), FAILED, BAD_PARAM (out of range).
 */
device_mode_set_result_t device_log_delta_ppm_set(float ppm);

#ifdef __cplusplus
}
#endif
