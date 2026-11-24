/**
 * @file hid_report.h
 * @brief Normalises the report-ID handling of an incoming HID OUT report.
 *
 * Moved from {@code on_host_report()} in {@code hid_control.cpp}: some USB
 * stacks leave the report ID as the first buffer byte while passing a report
 * ID of 0. Header-only and free of Arduino dependencies.
 *
 * @see docs/HID_CONTROL_PLANE.md section 3.2, docs/plans/BLE_TRANSPORT_TESTS.md
 * FR-HID
 */

#ifndef CONTROL_HID_REPORT_H
#define CONTROL_HID_REPORT_H

#include <cstddef>
#include <cstdint>

namespace control {

/** Report ID of the control-plane report. */
constexpr std::uint8_t kHidReportId = 0x01;

/** Result of {@link normalize_out_report}. */
struct NormalizedReport {
  /** First frame byte (after any stripped report ID). */
  const std::uint8_t *data = nullptr;
  /** Frame length in bytes. */
  std::size_t len = 0;
  /** false: the report is not for this interface; ignore it. */
  bool ok = false;
};

/**
 * Selects the frame inside an OUT report.
 *
 * @param report_id Report ID given by the USB stack (0 when unknown).
 * @param buf       Report bytes as delivered by the stack; must be non-null
 *                  when {@code len > 0}.
 * @param len       Number of bytes at {@code buf}.
 * @return {@code ok == false} when {@code report_id} is neither 0 nor
 *         {@link kHidReportId}. When {@code report_id} is 0 and
 *         {@code buf[0]} equals {@link kHidReportId}, that byte is skipped.
 */
[[nodiscard]] constexpr NormalizedReport
normalize_out_report(std::uint8_t report_id, const std::uint8_t *buf,
                     std::size_t len) noexcept {
  if (len > 0 && report_id == 0 && buf[0] == kHidReportId) {
    return {buf + 1, len - 1, true};
  }
  if (report_id != 0 && report_id != kHidReportId) {
    return {buf, len, false};
  }
  return {buf, len, true};
}

} // namespace control

#endif // CONTROL_HID_REPORT_H
