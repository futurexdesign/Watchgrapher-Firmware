/**
 * @file link.h
 * @brief One transport endpoint of the control plane (USB HID, BLE).
 *
 * A {@link control::Link} moves whole frames ({@code [flags, msgSeq,
 * fragIndex, fragCount, payloadLen, chunk...]}, see frame.h) between the
 * device and one host. The framing, request sequencing and ownership rules
 * live above it in {@link control::Session} and {@link control::Arbiter}, so
 * a link stays a thin adapter around its driver.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md section 5.3, session.h
 */

#ifndef CONTROL_LINK_H
#define CONTROL_LINK_H

#include "control/frame.h"

#include <cstddef>
#include <cstdint>

namespace control {

/**
 * One transport endpoint. All methods are called from the main loop only;
 * an implementation that receives data in interrupt or BT context hands it to
 * the main loop itself (ring, atomics) before it calls
 * {@link Session::on_frame}.
 */
class Link {
public:
  Link() = default;
  Link(const Link &) = delete;
  Link &operator=(const Link &) = delete;
  virtual ~Link() = default;

  /**
   * @return The link name used in {@code hello.link.kind}, {@code claim} and
   *         {@code controlReleased}: {@code "usbHid"} or {@code "ble"}. A
   *         string literal; never null.
   */
  [[nodiscard]] virtual const char *kind() const noexcept = 0;

  /**
   * @return true while a host is present and can receive frames (USB
   *         mounted; BLE connected with notifications on). A true to false
   *         transition tells the session that the link went down.
   */
  [[nodiscard]] virtual bool connected() const noexcept = 0;

  /** @return true when the link can accept one more frame right now. */
  [[nodiscard]] virtual bool can_send() const noexcept = 0;

  /**
   * Sends one frame. The bytes are only valid during the call.
   *
   * @param frame Frame bytes (no HID report ID).
   * @param len   Frame length: 63 with {@link pad_frames}, else
   *              {@code 5 + payloadLen}.
   * @return true when the link accepted the frame. On false the session
   *         offers the same frame again on a later call.
   */
  [[nodiscard]] virtual bool send_frame(const std::uint8_t *frame,
                                        std::size_t len) noexcept = 0;

  /**
   * @return true when frames are zero-padded to 63 bytes (a HID report has a
   *         fixed size); false for {@code 5 + payloadLen} bytes (BLE).
   */
  [[nodiscard]] virtual bool pad_frames() const noexcept = 0;

  /**
   * @return The negotiated ATT MTU in bytes for BLE; 0 where it does not
   *         apply. Reported in {@code hello.link.mtu}.
   */
  [[nodiscard]] virtual std::uint16_t mtu() const noexcept { return 0; }

  /**
   * @return The largest frame, in bytes, this link can carry to the host
   *         right now. {@link kFrameMaxBytes} (63) by default and for HID
   *         (fixed-size reports); BLE returns {@code min(255, mtu - 3)}. The
   *         session offers frames above 63 bytes only when the host asked for
   *         them with {@code hello.frameMax} (plan section 14.3), and never
   *         above this value.
   */
  [[nodiscard]] virtual std::size_t max_tx_frame() const noexcept {
    return kFrameMaxBytes;
  }
};

} // namespace control

#endif // CONTROL_LINK_H
