/**
 * @file frame.cpp
 * @brief Fragment reassembler and fragmenter of the control-plane framing.
 *
 * @see frame.h, docs/HID_CONTROL_PLANE.md section 3
 */

#include "control/frame.h"

#include <cstring>

namespace control {

namespace {

/** @return true when {@code now} is strictly after {@code deadline}. */
[[nodiscard]] bool expired(std::uint32_t now, std::uint32_t deadline) noexcept {
  return static_cast<std::int32_t>(now - deadline) > 0;
}

} // namespace

void FrameReassembler::reset() noexcept {
  length_ = 0;
  frag_count_ = 0;
  got_ = 0;
  seen_.fill(0);
  deadline_ms_ = 0;
  ready_ = false;
}

void FrameReassembler::poll_timeout(std::uint32_t now_ms) noexcept {
  if (!ready_ && frag_count_ != 0 && expired(now_ms, deadline_ms_)) {
    reset();
  }
}

RxResult FrameReassembler::push(const std::uint8_t *frame, std::size_t len,
                                std::uint32_t now_ms) noexcept {
  if (ready_) {
    return RxResult::kBusy;
  }
  if (frame == nullptr || len < kFrameHeaderBytes) {
    return RxResult::kShort;
  }

  const std::uint8_t seq = frame[1];
  const std::uint8_t frag_index = frame[2];
  const std::uint8_t frag_count = frame[3];
  const std::uint8_t payload_len = frame[4];
  if (frag_count == 0 || frag_index >= frag_count ||
      payload_len > kFrameMaxChunk ||
      static_cast<std::uint16_t>(frag_count) * kFrameMaxChunk > kMaxJson) {
    reset();
    return RxResult::kInvalid;
  }
  if (len < static_cast<std::size_t>(kFrameHeaderBytes) + payload_len) {
    return RxResult::kShort;
  }

  if (frag_count_ != 0 && expired(now_ms, deadline_ms_)) {
    reset();
  }
  if (frag_count_ == 0 || seq != seq_) {
    reset();
    seq_ = seq;
    frag_count_ = frag_count;
    deadline_ms_ = now_ms + kReassemblyTimeoutMs;
  }
  if (frag_count != frag_count_) {
    reset();
    return RxResult::kCountMismatch;
  }

  // frag_index < frag_count <= 35 here, so the bitmask index is in range.
  const std::size_t byte = frag_index / 8u;
  const auto bit = static_cast<std::uint8_t>(1u << (frag_index % 8u));
  if ((seen_[byte] & bit) != 0) {
    return RxResult::kDuplicate;
  }
  seen_[byte] = static_cast<std::uint8_t>(seen_[byte] | bit);

  const auto offset = static_cast<std::uint16_t>(frag_index * kFrameMaxChunk);
  if (payload_len > 0) {
    std::memcpy(&buffer_[offset], &frame[kFrameHeaderBytes], payload_len);
  }
  if (offset + payload_len > length_) {
    length_ = static_cast<std::uint16_t>(offset + payload_len);
  }
  got_++;
  if (got_ >= frag_count_) {
    buffer_[length_] = '\0';
    ready_ = true;
    return RxResult::kComplete;
  }
  return RxResult::kAccepted;
}

bool FrameFragmenter::begin(const char *json, std::size_t len, bool notify,
                            std::size_t chunk) noexcept {
  if (active_) {
    return false;
  }
  if (json == nullptr) {
    len = 0;
  }
  if (len > kMaxJson) {
    len = kMaxJson;
  }
  if (chunk < kFrameMinTxChunk) {
    chunk = kFrameMinTxChunk;
  } else if (chunk > kFrameMaxTxChunk) {
    chunk = kFrameMaxTxChunk;
  }
  chunk_ = static_cast<std::uint8_t>(chunk);
  if (len > 0) {
    std::memcpy(buffer_.data(), json, len);
  }
  buffer_[len] = '\0';
  len_ = static_cast<std::uint16_t>(len);
  off_ = 0;
  frag_index_ = 0;
  frag_count_ = static_cast<std::uint8_t>((len + chunk_ - 1u) / chunk_);
  if (frag_count_ == 0) {
    frag_count_ = 1;
  }
  seq_ = static_cast<std::uint8_t>(seq_ + 1u);
  notify_ = notify;
  active_ = true;
  return true;
}

std::size_t FrameFragmenter::peek(std::uint8_t *out, bool pad) const noexcept {
  if (!active_ || out == nullptr) {
    return 0;
  }
  const auto remain = static_cast<std::uint16_t>(off_ < len_ ? len_ - off_ : 0);
  const auto chunk =
      static_cast<std::uint8_t>(remain > chunk_ ? chunk_ : remain);
  const bool more = (frag_index_ + 1u) < frag_count_;
  const std::size_t total =
      (pad && chunk_ <= kFrameMaxChunk)
          ? kFrameMaxBytes
          : static_cast<std::size_t>(kFrameHeaderBytes) + chunk;
  std::memset(out, 0, total);
  out[0] = static_cast<std::uint8_t>((more ? kFlagMore : 0u) |
                                     (notify_ ? kFlagNotifyHint : 0u));
  out[1] = seq_;
  out[2] = frag_index_;
  out[3] = frag_count_;
  out[4] = chunk;
  if (chunk > 0) {
    std::memcpy(&out[kFrameHeaderBytes], &buffer_[off_], chunk);
  }
  return total;
}

void FrameFragmenter::commit() noexcept {
  if (!active_) {
    return;
  }
  const auto remain = static_cast<std::uint16_t>(off_ < len_ ? len_ - off_ : 0);
  const auto chunk =
      static_cast<std::uint16_t>(remain > chunk_ ? chunk_ : remain);
  off_ = static_cast<std::uint16_t>(off_ + chunk);
  frag_index_++;
  if (frag_index_ >= frag_count_) {
    active_ = false;
  }
}

} // namespace control
