/**
 * @file hid_control.h
 * @brief Vendor HID / WebHID JSON control plane over {@link control_api}.
 *
 * Protocol: {@code docs/HID_CONTROL_PLANE.md}. Registers an Adafruit HID
 * interface (Usage Page 0xFFA0) alongside UAC2 + CDC.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configures and begins the HID interface. Call from {@link usb_audio_init}
 * before USB re-enumeration so the composite descriptor includes HID.
 *
 * @return {@code false} if the HID interface could not be added.
 */
bool hid_control_usb_begin(void);

/**
 * Runs one pass of the USB HID session: reassembly timeout, request
 * dispatch, TX fragments and subscription pushes (see control/session.h).
 */
void hid_control_task(void);

#ifdef __cplusplus
}
#endif
