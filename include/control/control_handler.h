/**
 * @file control_handler.h
 * @brief The one control-plane handler: op dispatch, JSON encoders and the
 *        subscription (push) engine over {@code control_api}.
 *
 * Moved out of {@code hid_control.cpp} so that USB HID and BLE sessions share
 * it. Firmware only (it calls {@code control_api} and Arduino); the pure
 * parts are {@link control::Arbiter}, {@link control::Session} and the
 * encoders in hid_json.cpp.
 *
 * @see docs/HID_CONTROL_PLANE.md, docs/plans/BLE_TRANSPORT_PLAN.md
 *      section 5.3
 */

#ifndef CONTROL_CONTROL_HANDLER_H
#define CONTROL_CONTROL_HANDLER_H

#include "control/arbiter.h"
#include "control/handler.h"

#include <cstdint>

namespace control {

/**
 * Protocol brain of the device. Exactly one instance exists (its state is
 * file-scope in control_handler.cpp, as are the large scratch buffers, which
 * are used by one message at a time); sessions reach it through an
 * {@link ArbiterPort}, which applies control ownership first. Main loop only.
 *
 * Answers every op except {@code claim}, which the port answers.
 * {@code hello} and {@code ping} are answered for any link; every other op,
 * {@code getLink} and {@code getDiag} included, reaches the handler only from
 * the owner (the arbiter auto-claims or refuses with {@code linkBusy}).
 */
class ControlHandler final : public Handler {
public:
  /**
   * @param arbiter Ownership state, read for {@code hello.control} and
   *                {@code getLink.control}; must outlive the handler.
   */
  explicit ControlHandler(const Arbiter &arbiter) noexcept;

  /**
   * Binds the USB HID link whose {@code connected()} is reported as
   * {@code getLink.usbHid.mounted}.
   *
   * @param link The link, or null to unbind; must outlive the handler.
   */
  void set_usb_link(const Link *link) noexcept;

  /** Decodes and executes one request; sends exactly one reply. */
  void on_request(const char *json, Reply &out) noexcept override;

  /**
   * Pushes at most one due notification in the fixed order metrics, beat,
   * timing, quartz, quartzPulse, beats, event, error (rates and baselines of
   * HID_CONTROL_PLANE.md section 7).
   */
  void poll(Reply &out, std::uint32_t now_ms) noexcept override;

  /**
   * Stops every subscription and re-seeds the push cursors, as a
   * {@code subscribe} with everything off. Called when the owner link went
   * down or control moved.
   */
  void reset() noexcept override;
};

} // namespace control

#endif // CONTROL_CONTROL_HANDLER_H
