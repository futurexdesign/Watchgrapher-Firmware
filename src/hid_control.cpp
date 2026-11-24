/**
 * @file hid_control.cpp
 * @brief USB HID adapter of the control plane: the vendor report descriptor,
 *        the {@link control::Link} over Adafruit TinyUSB, and the session
 *        that carries the protocol.
 *
 * The framing, request dispatch, ownership and JSON encoders live in
 * {@code control::} (frame.h, session.h, arbiter.h, control_handler.cpp).
 * This file only moves 63-byte reports between TinyUSB and a
 * {@link control::Session}.
 *
 * @see docs/HID_CONTROL_PLANE.md, hid_control.h, control_api.h,
 *      docs/plans/BLE_TRANSPORT_PLAN.md section 5.3
 */

#include "hid_control.h"

#include "control/arbiter_port.h"
#include "control/hid_report.h"
#include "control/link.h"
#include "control/session.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <cstddef>
#include <cstdint>

namespace {

/* Report descriptor: Usage Page 0xFFA0, Usage 0x0001, Report ID 1, 63-byte IO.
 */
// clang-format off
uint8_t const kHidReportDesc[] = {
    HID_USAGE_PAGE_N(0xFFA0, 2),
    HID_USAGE(0x01),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    /* TinyUSB's HID_* macros are not on cppcheck's include path. */
    // cppcheck-suppress unknownMacro
    HID_REPORT_ID(1) HID_USAGE(0x02),
    HID_LOGICAL_MIN(0x00),
    HID_LOGICAL_MAX_N(0xff, 2),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(63),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_USAGE(0x03),
    HID_LOGICAL_MIN(0x00),
    HID_LOGICAL_MAX_N(0xff, 2),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(63),
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END};
// clang-format on

Adafruit_USBD_HID g_hid;

/** Report ID of the control-plane report. */
constexpr uint8_t kReportId = control::kHidReportId;

/**
 * The USB HID link: 63-byte reports through {@code g_hid}. {@code connected}
 * is the USB mount state, so unplugging the cable resets the subscriptions
 * (the former code kept them across a re-plug).
 */
class UsbHidLink final : public control::Link {
public:
  [[nodiscard]] const char *kind() const noexcept override { return "usbHid"; }
  [[nodiscard]] bool connected() const noexcept override {
    return TinyUSBDevice.mounted();
  }
  [[nodiscard]] bool can_send() const noexcept override {
    return g_hid.ready();
  }
  [[nodiscard]] bool send_frame(const uint8_t *frame,
                                size_t len) noexcept override {
    return g_hid.sendReport(kReportId, frame, static_cast<uint8_t>(len));
  }
  [[nodiscard]] bool pad_frames() const noexcept override { return true; }
};

UsbHidLink g_link;

/**
 * The USB session: one frame per {@code task()} call keeps the USB timing of
 * the former {@code tx_pump()}.
 */
control::Session g_session(g_link, control::usb_hid_port(), 1);

/** TinyUSB OUT-report callback: strips the report ID and queues the frame. */
void on_host_report(uint8_t report_id, hid_report_type_t, uint8_t const *buffer,
                    uint16_t bufsize) {
  const auto r = control::normalize_out_report(report_id, buffer, bufsize);
  if (!r.ok) {
    return;
  }
  g_session.on_frame(r.data, r.len, millis());
}

uint16_t on_get_report(uint8_t, hid_report_type_t, uint8_t *, uint16_t) {
  return 0;
}

} // namespace

bool hid_control_usb_begin(void) {
  control::attach_usb_hid_link(g_link);
  g_hid.setPollInterval(2);
  g_hid.enableOutEndpoint(true);
  g_hid.setReportDescriptor(kHidReportDesc, sizeof(kHidReportDesc));
  g_hid.setStringDescriptor("Timegrapher Control");
  g_hid.setReportCallback(on_get_report, on_host_report);
  if (!g_hid.begin()) {
    return false;
  }
  return true;
}

void hid_control_task(void) { g_session.task(millis()); }
