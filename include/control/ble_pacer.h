/**
 * @file ble_pacer.h
 * @brief Notification pacer: the "request can-send, then notify" state machine
 *        of the BLE TX path, with the BTstack calls injected.
 *
 * Header-only and free of BTstack headers so the logic runs in the host tests
 * (PACE-01..09 in tests/test_ble_pacer.cpp). Implements the {@code BlePacer}
 * of docs/plans/BLE_TRANSPORT_PLAN.md section 5.4 under the threading rules of
 * section 5.8.
 *
 * Why not {@code BLECharacteristic::setValue()}: it reallocates the one value
 * buffer that BTstack reads later and has no "sent" callback, so a second
 * call before the first notification leaves loses or corrupts data (plan
 * F-7). The pacer keeps its own frame queue and notifies from BTstack's
 * can-send-now callback.
 */

#ifndef CONTROL_BLE_PACER_H
#define CONTROL_BLE_PACER_H

#include "control/frame_ring.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace control {

/**
 * BTstack operations the pacer needs, injected so tests can fake them.
 *
 * Both functions run in BT context only.
 */
struct BlePacerOps {
  /**
   * Registers one can-send-now callback for the connection (BTstack
   * {@code att_server_register_can_send_now_callback}). The registration must
   * end in a call to {@link BlePacer::on_can_send}.
   *
   * @return 0 when registered, non-zero (BTstack error) when not
   */
  int (*request_can_send)(void *ctx, std::uint16_t con_handle);
  /**
   * Sends one notification (BTstack {@code att_server_notify}).
   *
   * @return 0 when BTstack accepted the value, non-zero (BTstack error such
   *         as an ACL buffer shortage) when the caller must retry later
   */
  int (*notify)(void *ctx, std::uint16_t con_handle, const std::uint8_t *data,
                std::uint16_t len);
  /** Opaque pointer handed back to both functions. */
  void *ctx;
};

/**
 * Drains a TX {@link FrameRing} into BLE notifications, one frame per
 * can-send-now callback.
 *
 * <b>Threading.</b> The main loop only pushes frames into the ring and reads
 * {@link connected} / {@link can_send}. Every other member runs in BT context
 * (the TX worker or a BTstack callback) and is never called from
 * {@code loop()}; BT context may be on either core (plan section 5.8), so
 * the members shared with the loop are atomics. The state that only BT
 * context touches ({@code pending_}) is a plain variable.
 *
 * <b>Wake-up protocol (no lost frame).</b> The producer pushes (release), then
 * wakes the worker with {@code async_context_set_work_pending()}. The worker
 * calls {@link kick}. If {@link kick} ran before the push became visible, the
 * wake-up still triggers another run. {@link on_can_send} clears
 * {@code pending_} before it looks at the ring and re-registers when frames
 * remain, so a push after its "ring empty" check is picked up by the next
 * {@link kick}.
 *
 * @tparam N         capacity of the TX ring the pacer drains
 * @tparam MaxFrame largest frame of that ring, bytes
 */
template <std::size_t N, std::size_t MaxFrame = kRingSlotBytes> class BlePacer {
public:
  /**
   * @param tx  TX ring; the pacer is its consumer
   * @param ops injected BTstack operations
   */
  BlePacer(FrameRing<N, MaxFrame> &tx, const BlePacerOps &ops)
      : tx_(tx), ops_(ops) {}
  BlePacer(const BlePacer &) = delete;
  BlePacer &operator=(const BlePacer &) = delete;

  /**
   * A central connected. BT context.
   *
   * Starts with notifications off, a clean ring (frames left by a previous
   * link must not reach this central) and the default ATT payload limit.
   *
   * @param con_handle HCI connection handle, non-zero
   */
  void on_connect(std::uint16_t con_handle) {
    cccd_.store(false, std::memory_order_relaxed);
    pending_ = false;
    tx_.clear();
    con_handle_.store(con_handle, std::memory_order_release);
  }

  /**
   * The central enabled or disabled notifications on the TX characteristic.
   * BT context.
   */
  void set_cccd(bool on) { cccd_.store(on, std::memory_order_release); }

  /**
   * Largest value {@link on_can_send} hands to notify, normally
   * {@code ATT MTU - 3}. BTstack silently truncates a longer value, so the
   * pacer drops such a frame instead (counted in {@link tx_oversize}).
   * BT context.
   */
  void set_max_value_len(std::size_t len) {
    max_len_.store(len, std::memory_order_relaxed);
  }

  /** Link is up and the central subscribed. Any context. */
  [[nodiscard]] bool connected() const {
    return con_handle_.load(std::memory_order_acquire) != 0 &&
           cccd_.load(std::memory_order_acquire);
  }

  /** A frame can be pushed: connected and the TX ring is not full. Loop. */
  [[nodiscard]] bool can_send() const { return connected() && !tx_.full(); }

  /**
   * Registers a can-send-now callback when frames wait and none is pending.
   * BT context (called from the TX worker).
   *
   * Registers at most once while a registration is outstanding.
   */
  void kick() {
    if (pending_ || !connected() || tx_.empty()) {
      return;
    }
    request();
  }

  /**
   * BTstack can-send-now callback. BT context.
   *
   * Sends the oldest frame. A frame leaves the ring only after notify
   * succeeded, so a failed send is retried with the same frame. Registers
   * again while frames remain.
   */
  void on_can_send() {
    pending_ = false;
    if (!connected()) {
      return;
    }
    const std::uint8_t *data = nullptr;
    std::size_t len = 0;
    if (!tx_.front(data, len)) {
      return;
    }
    if (len > max_len_.load(std::memory_order_relaxed)) {
      tx_.drop_front();
      tx_oversize_.fetch_add(1, std::memory_order_relaxed);
    } else if (ops_.notify(ops_.ctx,
                           con_handle_.load(std::memory_order_relaxed), data,
                           static_cast<std::uint16_t>(len)) == 0) {
      tx_.drop_front();
      tx_frames_.fetch_add(1, std::memory_order_relaxed);
    } else {
      tx_retries_.fetch_add(1, std::memory_order_relaxed);
    }
    if (!tx_.empty()) {
      request();
    }
  }

  /**
   * The link is gone: forget the connection and drop queued frames.
   * BT context.
   *
   * A registration still outstanding in BTstack is discarded by BTstack with
   * the connection; {@code pending_} is cleared so the next link starts
   * clean.
   */
  void on_disconnect() {
    cccd_.store(false, std::memory_order_release);
    con_handle_.store(0, std::memory_order_release);
    pending_ = false;
    tx_.clear();
  }

  /** Notifications accepted by BTstack. Any context. */
  [[nodiscard]] std::uint32_t tx_frames() const {
    return tx_frames_.load(std::memory_order_relaxed);
  }
  /** Failed notify calls that were retried. Any context. */
  [[nodiscard]] std::uint32_t tx_retries() const {
    return tx_retries_.load(std::memory_order_relaxed);
  }
  /** Frames dropped for exceeding the ATT payload limit. Any context. */
  [[nodiscard]] std::uint32_t tx_oversize() const {
    return tx_oversize_.load(std::memory_order_relaxed);
  }
  /** True while a can-send registration is outstanding. BT context. */
  [[nodiscard]] bool pending() const { return pending_; }

  /** Sets the counters to 0. BT context. */
  void reset_counters() {
    tx_frames_.store(0, std::memory_order_relaxed);
    tx_retries_.store(0, std::memory_order_relaxed);
    tx_oversize_.store(0, std::memory_order_relaxed);
  }

private:
  /**
   * Registers a callback; a refused registration leaves nothing pending.
   *
   * {@code pending_} is set before the call, not from its result: BTstack
   * delivers the callback inside the registration when the controller has a
   * free buffer, and {@link on_can_send} then clears {@code pending_} (and
   * may register again). Assigning the result afterwards would mark a
   * delivered callback as outstanding, and every later {@link kick} would
   * return early (found on hardware: one notification per connection).
   */
  void request() {
    pending_ = true;
    if (ops_.request_can_send(
            ops_.ctx, con_handle_.load(std::memory_order_relaxed)) != 0) {
      pending_ = false;
    }
  }

  /** TX ring; the pacer is its consumer. */
  FrameRing<N, MaxFrame> &tx_;
  /** Injected BTstack operations. */
  const BlePacerOps ops_;
  /** Connection handle, 0 when none. Written by BT context, read anywhere. */
  std::atomic<std::uint16_t> con_handle_{0};
  /** TX CCCD notify bit. Written by BT context, read anywhere. */
  std::atomic<bool> cccd_{false};
  /** Longest notifiable value. Written by BT context. */
  std::atomic<std::size_t> max_len_{20};
  /** Callback registered and not yet delivered. BT context only. */
  bool pending_ = false;
  /** Accepted notifications. Written by BT context. */
  std::atomic<std::uint32_t> tx_frames_{0};
  /** Retried notify failures. Written by BT context. */
  std::atomic<std::uint32_t> tx_retries_{0};
  /** Frames dropped as too long. Written by BT context. */
  std::atomic<std::uint32_t> tx_oversize_{0};
};

} // namespace control

#endif // CONTROL_BLE_PACER_H
