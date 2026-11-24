/**
 * @file frame_ring.h
 * @brief Lock-free single-producer / single-consumer queue of control-plane
 *        frames, used between BTstack context and the main loop.
 *
 * Header-only and free of Arduino and BTstack headers so it runs in the host
 * tests. Implements the {@code FrameRing} of docs/plans/BLE_TRANSPORT_PLAN.md
 * section 5.4; tests RING-01..08 in tests/test_frame_ring.cpp.
 *
 * Roles (the BLE link uses two rings):
 *
 * | Ring | Producer                          | Consumer                      |
 * |------|-----------------------------------|-------------------------------|
 * | RX   | BT context (GATT write callback)  | main loop ({@code loop()})    |
 * | TX   | main loop ({@code send_frame()})  | BT context (TX worker, pacer) |
 *
 * Either side may be on either core: the code uses no interrupt masking and
 * no assumption about which core runs BT context.
 */

#ifndef CONTROL_FRAME_RING_H
#define CONTROL_FRAME_RING_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace control {

/** Default largest frame of a {@link FrameRing}, in bytes (the RX ring). */
constexpr std::size_t kRingSlotBytes = 64;

/** Largest frame a {@link FrameRing} can store: the length is a u8. */
constexpr std::size_t kRingSlotBytesMax = 255;

/**
 * Fixed-capacity SPSC queue of frames of 0..{@code MaxFrame} bytes.
 *
 * <b>Protocol.</b> Exactly one thread (or interrupt context) calls the
 * producer functions ({@link push}, {@link reset_overflows}) and exactly one
 * calls the consumer functions ({@link front}, {@link drop_front},
 * {@link pop}, {@link clear}). {@link size}, {@link empty} and {@link full}
 * may be called from either side; the result is exact for the caller's own
 * role and a conservative snapshot for the other side (the producer sees a
 * count that can only shrink, the consumer one that can only grow).
 *
 * <b>Memory-ordering argument.</b> {@code tail_} is written only by the
 * producer and {@code head_} only by the consumer. Each side reads its own
 * index with {@code relaxed} order (no other writer exists) and the other
 * side's index with {@code acquire}.
 * - Producer: it writes the slot, then stores {@code tail_} with
 *   {@code release}. The consumer's {@code acquire} load of {@code tail_}
 *   that observes the new value therefore also observes every byte written
 *   to the slot. The consumer never reads a slot the producer has not
 *   published.
 * - Consumer: it reads the slot (through {@link front}), then stores
 *   {@code head_} with {@code release} ({@link drop_front}). The producer's
 *   {@code acquire} load of {@code head_} that observes the freed slot
 *   happens after the consumer's reads of that slot, so the producer never
 *   overwrites bytes the consumer is still reading.
 * - A slot is owned by the producer while it is outside {@code [head, tail)}
 *   and by the consumer while it is inside. Ownership changes only through
 *   the release/acquire pairs above, so slot bytes are never accessed
 *   concurrently and need no atomics. The pointer returned by {@link front}
 *   stays valid until {@link drop_front} because only the consumer frees
 *   slots.
 * - {@link clear} publishes {@code head_ = tail_} (consumer side, release).
 *   Frames pushed while it runs survive or are dropped atomically with
 *   respect to the snapshot; no slot is released twice.
 *
 * <b>Index scheme.</b> Both counters run through {@code [0, 2N)} and wrap
 * to 0, so {@code size = (tail - head) mod 2N} separates full ({@code N})
 * from empty ({@code 0}) without a spare slot, and the slot index
 * {@code counter mod N} stays continuous across the wrap for every
 * {@code N}, not only powers of two. (A free-running 16-bit counter would
 * jump to a wrong slot at 65536 for N = 40.) {@code 2N} fits in
 * {@code uint16_t} for {@code N <= 32767}.
 *
 * <b>Overflow counter.</b> {@link overflows} counts rejected pushes (ring
 * full, or length above {@code MaxFrame}). It is written only by the
 * producer and may be read from any side.
 *
 * @tparam N         slot count, 1..32767
 * @tparam MaxFrame largest frame a slot stores, 1..{@link kRingSlotBytesMax};
 *                   a slot takes {@code MaxFrame + 1} bytes (length byte
 *                   included). The RX ring uses the default, the BLE TX ring
 *                   255 (the large frames of plan section 14.3, 256-byte
 *                   slots)
 */
template <std::size_t N, std::size_t MaxFrame = kRingSlotBytes>
class FrameRing {
  static_assert(MaxFrame >= 1 && MaxFrame <= kRingSlotBytesMax,
                "slot length is stored in a u8");
  static_assert(N >= 1 && N <= 32767,
                "FrameRing capacity must fit 2N in 16 bits");
  static_assert(
      std::atomic<std::uint16_t>::is_always_lock_free,
      "ring indices must be lock-free to be safe in interrupt context");
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
                "counters must be lock-free to be safe in interrupt context");

public:
  FrameRing() = default;
  FrameRing(const FrameRing &) = delete;
  FrameRing &operator=(const FrameRing &) = delete;

  /** Slot count. */
  static constexpr std::size_t capacity() { return N; }

  /**
   * Appends a copy of a frame. Producer side.
   *
   * @param data frame bytes (may be null only when {@code len == 0})
   * @param len  frame length, 0..{@code MaxFrame}
   * @return false when the ring is full or {@code len} is too large; the
   *         rejection is counted in {@link overflows} and the ring is
   *         unchanged
   */
  [[nodiscard]] bool push(const std::uint8_t *data, std::size_t len) {
    if (len > MaxFrame) {
      overflows_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    const std::uint16_t tail = tail_.load(std::memory_order_relaxed);
    const std::uint16_t head = head_.load(std::memory_order_acquire);
    if (distance(head, tail) >= N) {
      overflows_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    Slot &slot = slots_[tail % N];
    slot.len = static_cast<std::uint8_t>(len);
    if (len != 0) {
      std::memcpy(slot.data, data, len);
    }
    tail_.store(advance(tail), std::memory_order_release);
    return true;
  }

  /**
   * Returns the oldest frame without removing it. Consumer side.
   *
   * The pointer stays valid until {@link drop_front} or {@link clear}.
   *
   * @param[out] data points to the stored bytes (read only)
   * @param[out] len  frame length
   * @return false when the ring is empty (outputs unchanged)
   */
  [[nodiscard]] bool front(const std::uint8_t *&data, std::size_t &len) const {
    const std::uint16_t head = head_.load(std::memory_order_relaxed);
    const std::uint16_t tail = tail_.load(std::memory_order_acquire);
    if (head == tail) {
      return false;
    }
    const Slot &slot = slots_[head % N];
    data = slot.data;
    len = slot.len;
    return true;
  }

  /**
   * Releases the frame returned by {@link front}. Consumer side. Does nothing
   * when the ring is empty.
   */
  void drop_front() {
    const std::uint16_t head = head_.load(std::memory_order_relaxed);
    const std::uint16_t tail = tail_.load(std::memory_order_acquire);
    if (head == tail) {
      return;
    }
    head_.store(advance(head), std::memory_order_release);
  }

  /**
   * Copies the oldest frame out and removes it. Consumer side.
   *
   * @param[out] out buffer of at least {@code MaxFrame} bytes
   * @param[out] len frame length
   * @return false when the ring is empty (outputs unchanged)
   */
  [[nodiscard]] bool pop(std::uint8_t *out, std::size_t &len) {
    const std::uint8_t *src = nullptr;
    std::size_t n = 0;
    if (!front(src, n)) {
      return false;
    }
    if (n != 0) {
      std::memcpy(out, src, n);
    }
    len = n;
    drop_front();
    return true;
  }

  /** Frames currently stored (exact for the caller's own role). */
  [[nodiscard]] std::size_t size() const {
    const std::uint16_t head = head_.load(std::memory_order_acquire);
    const std::uint16_t tail = tail_.load(std::memory_order_acquire);
    return distance(head, tail);
  }

  /** True when no frame is stored. */
  [[nodiscard]] bool empty() const { return size() == 0; }

  /** True when {@link push} would be rejected for lack of space. */
  [[nodiscard]] bool full() const { return size() >= N; }

  /** Drops every stored frame. Consumer side only. */
  void clear() {
    head_.store(tail_.load(std::memory_order_acquire),
                std::memory_order_release);
  }

  /** Pushes rejected so far (ring full or frame too long). */
  [[nodiscard]] std::uint32_t overflows() const {
    return overflows_.load(std::memory_order_relaxed);
  }

  /** Sets the overflow counter to 0. Producer side only. */
  void reset_overflows() { overflows_.store(0, std::memory_order_relaxed); }

private:
  /** One stored frame. */
  struct Slot {
    /** Frame length, 0..{@code MaxFrame}. */
    std::uint8_t len;
    /** Frame bytes; bytes past {@code len} are unspecified. */
    std::uint8_t data[MaxFrame];
  };

  /** Counter modulus: the counters count through [0, kWrap). */
  static constexpr std::size_t kWrap = 2 * N;

  /** Next counter value, wrapping at {@link kWrap}. */
  static std::uint16_t advance(std::uint16_t counter) {
    return static_cast<std::uint16_t>((counter + 1U) % kWrap);
  }

  /** Frames between {@code head} and {@code tail}, 0..N. */
  static std::size_t distance(std::uint16_t head, std::uint16_t tail) {
    return (static_cast<std::size_t>(tail) + kWrap - head) % kWrap;
  }

  /** Slot storage. Owned by the producer or the consumer, never both. */
  Slot slots_[N] = {};
  /** Consumer's read counter. Written by the consumer only. */
  std::atomic<std::uint16_t> head_{0};
  /** Producer's write counter. Written by the producer only. */
  std::atomic<std::uint16_t> tail_{0};
  /** Rejected pushes. Written by the producer only. */
  std::atomic<std::uint32_t> overflows_{0};
};

} // namespace control

#endif // CONTROL_FRAME_RING_H
