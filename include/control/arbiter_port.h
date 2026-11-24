/**
 * @file arbiter_port.h
 * @brief Firmware wiring of the control plane: the single
 *        {@link control::Arbiter}, the single
 *        {@link control::ControlHandler} and one
 *        {@link control::ArbiterPort} per link.
 *
 * Each transport adapter owns its {@link control::Link} and
 * {@link control::Session}; it takes its port from here and binds its link.
 * A non-W build has only the USB HID port, so the arbiter always dispatches.
 *
 * A BLE build ({@code TG_BLE=1}) adds the BLE port (plan A3-M2). It shares
 * the arbiter and the handler with the USB port, and {@link attach_ble_link}
 * wires the two ports as peers so a takeover on one link queues
 * {@code controlReleased} on the other.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md section 5.3
 */

#ifndef CONTROL_ARBITER_PORT_H
#define CONTROL_ARBITER_PORT_H

#include "board_config.h"
#include "control/arbiter.h"
#include "control/link.h"

namespace control {

/**
 * Binds the USB HID link: it is the link {@code getLink.usbHid.mounted}
 * reports and the one that may receive {@code controlReleased}.
 *
 * @param link The USB HID link; must outlive the program (a static object).
 */
void attach_usb_hid_link(const Link &link) noexcept;

/** @return The port a USB HID {@link Session} uses as its handler. */
[[nodiscard]] ArbiterPort &usb_hid_port() noexcept;

#if TG_BLE
/**
 * Binds the BLE link to the BLE port and makes the USB and BLE ports peers.
 * Main loop only; call once, before the BLE session handles a frame.
 *
 * @param link The BLE link; must outlive the program (a static object).
 */
void attach_ble_link(const Link &link) noexcept;

/** @return The port the BLE {@link Session} uses as its handler. */
[[nodiscard]] ArbiterPort &ble_port() noexcept;
#endif

} // namespace control

#endif // CONTROL_ARBITER_PORT_H
