/**
 * @file handler.h
 * @brief The protocol brain behind a session: {@link control::Handler} and the
 *        {@link control::Reply} sink it answers through.
 *
 * A session owns the framing of one link. For every complete request it calls
 * a Handler, which writes at most one outgoing message into the session's
 * {@link Reply}. The handler never sees frames, fragments or the link driver.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md section 5.3, session.h, arbiter.h
 */

#ifndef CONTROL_HANDLER_H
#define CONTROL_HANDLER_H

#include "control/link.h"

#include <cstddef>
#include <cstdint>

namespace control {

/**
 * Where a handler puts its single outgoing message. Implemented by
 * {@link Session}; a handler never stores the reference past the call.
 */
class Reply {
public:
  Reply() = default;
  Reply(const Reply &) = delete;
  Reply &operator=(const Reply &) = delete;
  virtual ~Reply() = default;

  /** @return true when no message is being sent, so {@link send} is allowed. */
  [[nodiscard]] virtual bool idle() const noexcept = 0;

  /**
   * Starts sending one message. The text is copied; the caller may reuse its
   * buffer at once.
   *
   * @param json   Message text (not required to be NUL-terminated).
   * @param n      Length in bytes, at most 2048 (longer text is clamped).
   * @param notify true for a push (sets the notify hint flag), false for a
   *               response.
   * @pre {@link idle} returned true.
   */
  virtual void send(const char *json, std::size_t n, bool notify) noexcept = 0;

  /** @return The link this reply travels on (for {@code hello.link}). */
  [[nodiscard]] virtual const Link &link() const noexcept = 0;

  /**
   * @return The TX frame size, in bytes, that applies to messages begun
   *         after the current request: 63 unless the host opted in with
   *         {@code hello.frameMax} (plan section 14.3). During a {@code hello}
   *         that carries {@code frameMax} it already reports the new value,
   *         although the {@code hello} reply itself still goes out in 63-byte
   *         frames. Reported as {@code hello.link.frameMax}.
   */
  [[nodiscard]] virtual std::size_t tx_frame_max() const noexcept {
    return kFrameMaxBytes;
  }
};

/**
 * Protocol brain. A session reaches it through an {@link ArbiterPort}; tests
 * use fakes. All methods are called from the main loop only.
 */
class Handler {
public:
  Handler() = default;
  Handler(const Handler &) = delete;
  Handler &operator=(const Handler &) = delete;
  virtual ~Handler() = default;

  /**
   * Handles one complete request. Must send exactly one reply through
   * {@code out}, which is idle on entry.
   *
   * @param json NUL-terminated request, valid only during the call.
   * @param out  Reply sink of the requesting link.
   */
  virtual void on_request(const char *json, Reply &out) noexcept = 0;

  /**
   * Gives the handler a chance to push one notification. Called only while
   * the session's TX is idle and its link is connected; may send one message.
   *
   * @param out    Reply sink; send only if {@code out.idle()}.
   * @param now_ms Monotonic time in ms; wrap-safe.
   */
  virtual void poll(Reply &out, std::uint32_t now_ms) noexcept = 0;

  /**
   * The link behind this handler went down: drop what belongs to it
   * (subscriptions and push cursors for the control handler).
   */
  virtual void reset() noexcept = 0;
};

} // namespace control

#endif // CONTROL_HANDLER_H
