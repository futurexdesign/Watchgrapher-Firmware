/**
 * @file session.h
 * @brief Per-link protocol session: frame reassembly, one request at a time,
 *        response and notify fragmentation, link-down handling.
 *
 * A Session ties one {@link Link} to one {@link Handler}. It owns the
 * {@link FrameReassembler} and the {@link FrameFragmenter} of that link
 * (about 4.1 KB), and runs the loop that {@code hid_control_task()} used to
 * contain. Pure: no Arduino, no heap; time is passed in.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md section 5.3,
 *      docs/HID_CONTROL_PLANE.md section 3
 */

#ifndef CONTROL_SESSION_H
#define CONTROL_SESSION_H

#include "control/frame.h"
#include "control/handler.h"
#include "control/link.h"

#include <cstddef>
#include <cstdint>

namespace control {

/**
 * Protocol session of one link. Main loop only: {@link on_frame} must not
 * run concurrently with {@link task} (a link that receives in another
 * context queues frames and calls {@link on_frame} from the main loop).
 */
class Session final : private Reply {
public:
  /**
   * @param link   The transport endpoint; must outlive the session.
   * @param handler This link's handler (an {@link ArbiterPort} in the
   *                firmware); must outlive the session.
   * @param max_frames_per_task Most frames {@link task} sends per pump:
   *                1 for HID (keeps the USB timing of the former code), 4
   *                for BLE. 0 is treated as 1.
   */
  Session(Link &link, Handler &handler,
          std::uint8_t max_frames_per_task = 1) noexcept;

  /**
   * Feeds one received frame (no HID report ID) to the reassembler. A frame
   * that arrives while a complete request waits for dispatch is dropped.
   *
   * @param frame  Frame bytes.
   * @param len    Number of valid bytes.
   * @param now_ms Monotonic time in ms; wrap-safe.
   */
  void on_frame(const std::uint8_t *frame, std::size_t len,
                std::uint32_t now_ms) noexcept;

  /**
   * Runs one main-loop pass, in this order: link-down detection, reassembly
   * timeout, dispatch of a complete request (only while the TX is idle), TX
   * pump, and, when the TX is idle, the link connected and no request is
   * waiting, one {@link Handler::poll} followed by another pump. A waiting
   * request therefore always goes before the next notify.
   *
   * @param now_ms Monotonic time in ms; wrap-safe.
   */
  void task(std::uint32_t now_ms) noexcept;

  /**
   * The link went down: clears the partial request and the message in
   * flight, and calls {@link Handler::reset} once. The TX sequence counter
   * keeps counting. The TX frame size returns to 63. {@link task} calls this
   * itself when it sees {@link Link::connected} turn false.
   */
  void on_link_down() noexcept;

private:
  [[nodiscard]] bool idle() const noexcept override { return !tx_.active(); }
  void send(const char *json, std::size_t n, bool notify) noexcept override;
  [[nodiscard]] const Link &link() const noexcept override { return link_; }
  [[nodiscard]] std::size_t tx_frame_max() const noexcept override {
    return next_tx_frame_;
  }

  /**
   * Reads {@code frameMax} from a {@code hello} request and updates
   * {@code next_tx_frame_} (plan section 14.3). Other requests, a missing
   * key and values below 63 leave it unchanged.
   */
  void negotiate_frame_max(const char *json) noexcept;

  /** @return The frame size for a message begun now: the opted-in size,
   *          bounded by {@link Link::max_tx_frame}, never below 63. */
  [[nodiscard]] std::size_t current_tx_frame() const noexcept;

  /** Sends frames while the fragmenter is active and the link has room. */
  void pump() noexcept;

  Link &link_;
  Handler &handler_;
  FrameReassembler rx_;
  FrameFragmenter tx_;
  std::uint8_t max_frames_;
  /** Frame size in use for new messages (63 until the host opts in). */
  std::size_t tx_frame_ = kFrameMaxBytes;
  /** Size that {@code tx_frame_} takes once the current request's reply has
   * begun; what {@code hello.link.frameMax} reports. */
  std::size_t next_tx_frame_ = kFrameMaxBytes;
  bool link_was_connected_ = false;
};

} // namespace control

#endif // CONTROL_SESSION_H
