/**
 * @file usb_audio.h
 * @brief UAC2 mono microphone device side (TinyUSB + Adafruit Arduino glue).
 *
 * Enumerates as a USB Audio Class 2 recording-only function (plus CDC for
 * diagnostics on the Arduino path). Descriptors live in
 * {@code usb_descriptors.cpp}; control / data callbacks in {@code usb_audio.cpp}.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sets USB VID/PID and string descriptors, registers the mic interface, and
 * forces a re-enumeration if the device was already mounted by the core.
 *
 * @return {@code false} if the interface could not be added (fatal — main
 *         blinks the status LED forever).
 */
bool usb_audio_init(void);

/**
 * Feeds one or more 1 ms PCM frames into TinyUSB while streaming is active.
 *
 * Call from the main loop after {@link audio_source_process}. Uses
 * {@code millis()} to pace ~1 frame/ms and catches up a few frames if the
 * loop stalled briefly (capped) to limit underruns without bursting.
 */
void usb_audio_task(void);

/**
 * @return {@code true} while the host has selected a non-zero UAC alternate
 *         setting (ISO IN streaming open).
 */
bool usb_audio_streaming(void);

#ifdef __cplusplus
}
#endif
