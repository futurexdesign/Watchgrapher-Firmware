/**
 * @file frame.h
 * @brief Transport-independent framing of the control-plane JSON messages:
 *        fragment reassembler (host to device) and fragmenter (device to host).
 *
 * The module is pure: no Arduino headers, no heap, no global state. Time is
 * passed in by the caller and every buffer is owned by the object with a
 * capacity taken from the constants below. It reproduces the framing that
 * {@code hid_control.cpp} implemented inline, byte for byte, so USB HID and
 * BLE share one implementation.
 *
 * A frame is the buffer without any HID report ID:
 * {@code [flags, msgSeq, fragIndex, fragCount, payloadLen, chunk...]}.
 *
 * @see docs/HID_CONTROL_PLANE.md section 3, docs/plans/BLE_TRANSPORT_PLAN.md
 *      section 5.2, docs/plans/BLE_TRANSPORT_TESTS.md section 1
 */

#ifndef CONTROL_FRAME_H
#define CONTROL_FRAME_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace control {

/** Header bytes of a frame: flags, msgSeq, fragIndex, fragCount, payloadLen. */
constexpr std::uint8_t kFrameHeaderBytes = 5;
/**
 * Maximum payload bytes per frame. Every non-final fragment carries exactly
 * this many.
 */
constexpr std::uint8_t kFrameMaxChunk = 58;
/** Maximum frame size: header plus a full chunk. */
constexpr std::uint8_t kFrameMaxBytes = kFrameHeaderBytes + kFrameMaxChunk;
/**
 * Largest chunk a transmitted frame may carry (plan section 14.3): the
 * {@code payloadLen} field is a u8, and 5 + 250 = 255 is the most a BLE
 * notification frame holds. Receive limits stay at {@link kFrameMaxChunk}.
 */
constexpr std::uint8_t kFrameMaxTxChunk = 250;
/** Largest transmitted frame: header plus {@link kFrameMaxTxChunk}. */
constexpr std::size_t kFrameMaxTxBytes = kFrameHeaderBytes + kFrameMaxTxChunk;
/**
 * Smallest chunk the fragmenter accepts. A message of {@link kMaxJson} bytes
 * must fit the u8 {@code fragCount}: 2048 / 9 = 228 fragments (8 would need
 * 256).
 */
constexpr std::uint8_t kFrameMinTxChunk = 9;
/** Maximum reassembled or fragmented JSON size in bytes (without the NUL). */
constexpr std::uint16_t kMaxJson = 2048;
/** Largest {@code fragCount} accepted in a request (35 x 58 = 2030 bytes). */
constexpr std::uint8_t kMaxRequestFragments = 35;
/** Time after the first fragment at which an incomplete message is dropped. */
constexpr std::uint32_t kReassemblyTimeoutMs = 500;
/** {@code flags} bit 0: more fragments follow. */
constexpr std::uint8_t kFlagMore = 0x01;
/** {@code flags} bit 1: advisory hint that the message is a notification. */
constexpr std::uint8_t kFlagNotifyHint = 0x02;

/** Outcome of {@link FrameReassembler::push} (for tests and counters). */
enum class RxResult : std::uint8_t {
  /** Stored; message not complete yet. */
  kAccepted,
  /** Stored; message complete, {@link FrameReassembler::ready} is true. */
  kComplete,
  /** fragIndex already stored for this message; ignored. */
  kDuplicate,
  /** A complete message awaits release(); frame ignored, state untouched. */
  kBusy,
  /** len < header or < header + payloadLen; ignored, state untouched. */
  kShort,
  /** Header rule broken; reassembly state cleared. */
  kInvalid,
  /** Same msgSeq, different fragCount; state cleared, frame dropped. */
  kCountMismatch,
};

/**
 * Reassembles fragments of one request into a NUL-terminated JSON string.
 *
 * One message is reassembled at a time. Fragments may arrive in any order;
 * fragment {@code i} lands at offset {@code i x 58}. Not thread-safe: the
 * owner serialises {@link push}, {@link poll_timeout} and the accessors.
 */
class FrameReassembler {
public:
  /**
   * Feeds one received frame.
   *
   * Order of checks (identical to the former HID receiver): busy, length
   * below the header, header rules, length below {@code 5 + payloadLen},
   * expired deadline (restarts), new {@code msgSeq}, {@code fragCount}
   * mismatch, duplicate fragment, store.
   *
   * @param frame  Frame bytes without a report ID; may be longer than the
   *               header plus payload (padding is ignored).
   * @param len    Number of valid bytes at {@code frame}.
   * @param now_ms Monotonic time in ms; wrap-safe.
   * @return What happened to the frame.
   */
  RxResult push(const std::uint8_t *frame, std::size_t len,
                std::uint32_t now_ms) noexcept;

  /**
   * Drops a partial message whose 500 ms window has passed. A complete message
   * awaiting {@link release} is never touched.
   *
   * @param now_ms Monotonic time in ms; wrap-safe.
   */
  void poll_timeout(std::uint32_t now_ms) noexcept;

  /** @return true when a complete message awaits {@link release}. */
  [[nodiscard]] bool ready() const noexcept { return ready_; }

  /** @return The message, NUL-terminated; valid while {@link ready} is true. */
  [[nodiscard]] const char *message() const noexcept { return buffer_.data(); }

  /** @return Message length in bytes without the NUL. */
  [[nodiscard]] std::uint16_t length() const noexcept { return length_; }

  /** Forgets the message after dispatch; the same {@code msgSeq} then starts a
   * new message. */
  void release() noexcept { reset(); }

  /** Clears all reassembly state, including a complete message. */
  void reset() noexcept;

private:
  /** Bytes of the seen-fragment bitmask. */
  static constexpr std::size_t kSeenBytes = (kMaxRequestFragments + 7u) / 8u;

  /** Message bytes plus the NUL. */
  std::array<char, kMaxJson + 1u> buffer_{};
  /** Bit per stored fragIndex. */
  std::array<std::uint8_t, kSeenBytes> seen_{};
  /** First fragment time + timeout. */
  std::uint32_t deadline_ms_ = 0;
  /** Highest stored byte offset + 1. */
  std::uint16_t length_ = 0;
  /** msgSeq of the message in progress. */
  std::uint8_t seq_ = 0;
  /** fragCount of the message; 0 = idle. */
  std::uint8_t frag_count_ = 0;
  /** Distinct fragments stored. */
  std::uint8_t got_ = 0;
  /** Message complete, not yet released. */
  bool ready_ = false;
};

/**
 * Splits one JSON message into frames.
 *
 * The caller reads the current frame with {@link peek} and calls
 * {@link commit} only after the transport accepted it. A failed send leaves
 * the state unchanged, so the retry carries the same fragment (fixes F-15:
 * the former HID code advanced the offset before the send result was known).
 */
class FrameFragmenter {
public:
  /**
   * Starts a message and copies it into the internal buffer. The first
   * message gets {@code msgSeq} 1; the 8-bit counter wraps.
   *
   * The clamp of {@code len} to {@link kMaxJson} reproduces the former
   * {@code tx_begin()}. Callers must never pass more: a clamped JSON text is
   * truncated and therefore invalid. Encoders check their own output size
   * against {@link kMaxJson} before calling.
   *
   * @param json   Message bytes; not required to be NUL-terminated.
   * @param len    Length in bytes; at most {@link kMaxJson} (clamped).
   * @param notify Sets {@link kFlagNotifyHint} on every fragment.
   * @param chunk  Payload bytes per frame, clamped to
   *               {@link kFrameMinTxChunk}..{@link kFrameMaxTxChunk}. The
   *               default is the 63-byte frame of HID and of BLE before the
   *               {@code frameMax} opt-in (plan section 14.3). Fixed for the
   *               whole message.
   * @return false (and no change) when a message is still active.
   */
  [[nodiscard]] bool begin(const char *json, std::size_t len, bool notify,
                           std::size_t chunk = kFrameMaxChunk) noexcept;

  /** @return true while fragments remain to be committed. */
  [[nodiscard]] bool active() const noexcept { return active_; }

  /**
   * Writes the current frame into {@code out} without advancing.
   *
   * @param out Buffer of at least {@link kFrameMaxBytes} bytes, or
   *            {@link kFrameMaxTxBytes} when {@link begin} got a chunk above
   *            {@link kFrameMaxChunk}.
   * @param pad true: write {@link kFrameMaxBytes} bytes, zero-padded after the
   *            payload (HID); false: write {@code 5 + payloadLen} bytes (BLE).
   *            Padding applies only to chunks up to {@link kFrameMaxChunk};
   *            a larger chunk is never padded.
   * @return Bytes written; 0 when no message is active.
   */
  [[nodiscard]] std::size_t peek(std::uint8_t *out, bool pad) const noexcept;

  /** Advances to the next fragment after a successful send; ends the message
   * after the last. */
  void commit() noexcept;

  /** Drops the active message. The sequence counter is kept. */
  void abort() noexcept { active_ = false; }

  /** @return {@code msgSeq} of the current or most recent message. */
  [[nodiscard]] std::uint8_t seq() const noexcept { return seq_; }

private:
  /** Message copy. */
  std::array<char, kMaxJson + 1u> buffer_{};
  /** Message length in bytes. */
  std::uint16_t len_ = 0;
  /** Offset of the current fragment's chunk. */
  std::uint16_t off_ = 0;
  /** Last assigned msgSeq (pre-incremented). */
  std::uint8_t seq_ = 0;
  /** Current fragment. */
  std::uint8_t frag_index_ = 0;
  /** Payload bytes per non-final fragment of the active message. */
  std::uint8_t chunk_ = kFrameMaxChunk;
  /** Fragments in the message (>= 1 when active). */
  std::uint8_t frag_count_ = 0;
  /** Set kFlagNotifyHint on every fragment. */
  bool notify_ = false;
  /** Fragments remain. */
  bool active_ = false;
};

} // namespace control

#endif // CONTROL_FRAME_H
