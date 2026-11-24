/**
 * @file arbiter_port.cpp
 * @brief Firmware singletons of the control plane: arbiter, control handler
 *        and the per-link ports; see arbiter_port.h.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md section 5.3
 */

#include "control/arbiter_port.h"

#include "control/control_handler.h"

#include <Arduino.h>

namespace control {

namespace {

/** The shared ownership state of all links. */
Arbiter g_arbiter;
/** The one protocol handler. */
ControlHandler g_handler(g_arbiter);

/** Clock of the lease decisions. */
std::uint32_t clock_ms() noexcept { return millis(); }

/** Port of the USB HID link. */
ArbiterPort g_usb_port(LinkKind::kUsbHid, g_arbiter, g_handler, clock_ms);

#if TG_BLE
/** Port of the BLE link; a peer of {@code g_usb_port}. */
ArbiterPort g_ble_port(LinkKind::kBle, g_arbiter, g_handler, clock_ms);
#endif

} // namespace

void attach_usb_hid_link(const Link &link) noexcept {
  g_usb_port.attach_link(link);
  g_handler.set_usb_link(&link);
}

ArbiterPort &usb_hid_port() noexcept { return g_usb_port; }

#if TG_BLE
void attach_ble_link(const Link &link) noexcept {
  g_ble_port.attach_link(link);
  g_ble_port.set_peer(&g_usb_port);
  g_usb_port.set_peer(&g_ble_port);
}

ArbiterPort &ble_port() noexcept { return g_ble_port; }
#endif

} // namespace control
