/**
 * @file arbiter.h
 * @brief Control ownership: at most one link owns the control plane at a
 *        time ({@link control::Arbiter}), and the per-link front end that
 *        applies it to requests ({@link control::ArbiterPort}).
 *
 * Both classes are pure: no Arduino, no {@code control_api}, no global state.
 * Time is passed in or read through an injected clock, so the whole state
 * machine runs on the host (tests/test_control_arbiter.cpp).
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md sections 4.7 and 5.3,
 *      docs/HID_CONTROL_PLANE.md section 4.6
 */

#ifndef CONTROL_ARBITER_H
#define CONTROL_ARBITER_H

#include "control/handler.h"
#include "control/link.h"

#include <cstdint>

namespace control {

/** Link kinds known to the arbiter. */
enum class LinkKind : std::uint8_t { kNone, kUsbHid, kBle };

/**
 * @param kind Link kind.
 * @return The protocol name ({@code "usbHid"}, {@code "ble"}), or
 *         {@code nullptr} for {@link LinkKind::kNone} (JSON null).
 */
[[nodiscard]] constexpr const char *link_kind_name(LinkKind kind) noexcept {
  switch (kind) {
  case LinkKind::kUsbHid:
    return "usbHid";
  case LinkKind::kBle:
    return "ble";
  case LinkKind::kNone:
    break;
  }
  return nullptr;
}

/**
 * Ownership state machine (plan section 4.7).
 *
 * State is {@code owner in {none, usbHid, ble}} and a lease deadline. A link
 * that sends nothing for {@link kLeaseMs} can be taken over by the next
 * request of another link; nothing happens by itself when the lease runs out.
 * Main loop only.
 */
class Arbiter {
public:
  /** What the caller does with a request. */
  enum class Decision : std::uint8_t {
    /** Hand the request to the control handler. */
    kDispatch,
    /** The port answers it: {@code claim}; {@code hello} and {@code ping}
     *  are answered by the handler without claiming. */
    kAnswerLocally,
    /** Reply {@code linkBusy}; the handler never sees the request. */
    kBusy,
  };

  /** Result of {@link on_request}. */
  struct Outcome {
    /** What to do with the request. */
    Decision decision = Decision::kDispatch;
    /** Ownership moved from another link: call {@link Handler::reset}. */
    bool reset_handler = false;
    /** kNone, or the previous owner to send {@code controlReleased} to. */
    LinkKind notify_released = LinkKind::kNone;
  };

  /** Lease length in ms ({@code hello.control.leaseMs}). */
  static constexpr std::uint32_t kLeaseMs = 10000;

  /**
   * Classifies one complete request.
   *
   * {@code hello} and {@code ping} are always answered; a {@code ping} from
   * the owner refreshes its lease. A {@code claim} without {@code takeover}
   * succeeds when the owner is none, the requester or expired. A claim with
   * {@code takeover} always succeeds. Any other op auto-claims under the same
   * condition as a plain claim and otherwise gets {@link Decision::kBusy}.
   * A change of owner from another link sets {@code reset_handler} and
   * {@code notify_released}.
   *
   * @param from     Link the request arrived on.
   * @param op       Operation name (never null).
   * @param takeover Value of {@code takeover} (only read for {@code claim}).
   * @param now_ms   Monotonic time in ms; wrap-safe.
   * @return The decision and its side effects for the caller.
   */
  Outcome on_request(LinkKind from, const char *op, bool takeover,
                     std::uint32_t now_ms) noexcept;

  /**
   * The link went down (BLE disconnect or CCCD off, USB unmount).
   *
   * @param kind The link.
   * @return true when it was the owner: owner is none now and the handler
   *         must reset.
   */
  [[nodiscard]] bool on_link_down(LinkKind kind) noexcept;

  /** @return The current owner (kNone when no link owns control). */
  [[nodiscard]] LinkKind owner() const noexcept { return owner_; }

  /**
   * @param now_ms Monotonic time in ms; wrap-safe.
   * @return true when there is an owner and its lease has not run out. The
   *         deadline itself is still valid; one ms later it is not.
   */
  [[nodiscard]] bool lease_valid(std::uint32_t now_ms) const noexcept;

private:
  /** Makes {@code from} the owner and refreshes the lease; fills the side
   * effects when another link loses control. */
  void take(LinkKind from, std::uint32_t now_ms, Outcome &out) noexcept;

  LinkKind owner_ = LinkKind::kNone;
  /** Time at which the owner's lease ends (valid while owner_ != kNone). */
  std::uint32_t deadline_ms_ = 0;
};

/**
 * Per-link front end of the arbiter: the {@link Handler} a {@link Session}
 * talks to. It reads {@code op} (and {@code takeover}) of each request, asks
 * the shared {@link Arbiter}, and then dispatches to the shared backend
 * handler, answers {@code claim} or {@code linkBusy} itself, resets the
 * backend when control moves, and queues {@code controlReleased} on the
 * previous owner's port.
 *
 * A request that fails the envelope checks (no {@code id}, {@code v} not 1,
 * no {@code op}) goes to the backend unchanged, so malformed requests get the
 * same error as before ownership existed and never change the owner.
 *
 * A two-link device wires its ports as peers (see {@link set_peer} and
 * arbiter_port.h). Main loop only.
 */
class ArbiterPort final : public Handler {
public:
  /** Monotonic millisecond clock; the firmware passes {@code millis()}. */
  using ClockFn = std::uint32_t (*)() noexcept;

  /**
   * @param kind    The link this port serves (not kNone).
   * @param arbiter The shared ownership state; must outlive the port.
   * @param backend The one control handler; must outlive the port.
   * @param now     Clock for lease decisions.
   */
  ArbiterPort(LinkKind kind, Arbiter &arbiter, Handler &backend,
              ClockFn now) noexcept;

  /**
   * Binds the link whose connection state decides whether the port may queue
   * a {@code controlReleased} for it. Before this call the link counts as not
   * connected.
   *
   * @param link The transport endpoint; must outlive the port.
   */
  void attach_link(const Link &link) noexcept { link_ = &link; }

  /**
   * Names the port of the other link, which receives {@code controlReleased}
   * when this port's link takes control from it. Wire both directions.
   *
   * @param peer The other port, or null on a one-link device.
   */
  void set_peer(ArbiterPort *peer) noexcept { peer_ = peer; }

  /** Applies ownership to one request; see the class comment. */
  void on_request(const char *json, Reply &out) noexcept override;

  /**
   * Sends a queued {@code controlReleased}, else forwards to the backend's
   * poll when this port's link owns control.
   */
  void poll(Reply &out, std::uint32_t now_ms) noexcept override;

  /**
   * This port's link went down: clears the queued notify and releases
   * ownership; the backend resets only if this link was the owner.
   */
  void reset() noexcept override;

private:
  /** Remembers that {@code to} took control; sent when the TX is idle, if
   * this port's link is still connected. */
  void queue_released(LinkKind to) noexcept;
  /** Answers {@code claim}. */
  void reply_claim(Reply &out, std::int64_t id, LinkKind previous) noexcept;
  /** Answers {@code linkBusy} with the request id. */
  void reply_busy(Reply &out, std::int64_t id) noexcept;

  LinkKind kind_;
  Arbiter &arbiter_;
  Handler &backend_;
  ClockFn now_;
  const Link *link_ = nullptr;
  ArbiterPort *peer_ = nullptr;
  /** New owner to announce with {@code controlReleased}; kNone = nothing. */
  LinkKind pending_to_ = LinkKind::kNone;
};

} // namespace control

#endif // CONTROL_ARBITER_H
