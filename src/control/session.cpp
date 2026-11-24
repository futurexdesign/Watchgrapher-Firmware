/**
 * @file session.cpp
 * @brief Per-link protocol session; see session.h.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md section 5.3
 */

#include "control/session.h"

#include "control/json_scan.h"

#include <cstring>

namespace control {

Session::Session(Link &link, Handler &handler,
                 std::uint8_t max_frames_per_task) noexcept
    : link_(link), handler_(handler),
      max_frames_(max_frames_per_task == 0 ? std::uint8_t{1}
                                           : max_frames_per_task) {}

void Session::on_frame(const std::uint8_t *frame, std::size_t len,
                       std::uint32_t now_ms) noexcept {
  (void)rx_.push(frame, len, now_ms);
}

void Session::send(const char *json, std::size_t n, bool notify) noexcept {
  /* Handlers send only while idle(), so begin() cannot refuse. */
  const std::size_t frame = current_tx_frame();
  (void)tx_.begin(json, n, notify, frame - kFrameHeaderBytes);
}

std::size_t Session::current_tx_frame() const noexcept {
  if (tx_frame_ <= kFrameMaxBytes) {
    return kFrameMaxBytes;
  }
  /* The link's limit can fall (MTU) after the opt-in; 63 stays the floor. */
  std::size_t limit = link_.max_tx_frame();
  if (limit < kFrameMaxBytes) {
    limit = kFrameMaxBytes;
  }
  return tx_frame_ < limit ? tx_frame_ : limit;
}

void Session::negotiate_frame_max(const char *json) noexcept {
  char op[8];
  long value = 0;
  if (!json::get_string(json, "op", op, sizeof(op)) ||
      std::strcmp(op, "hello") != 0 ||
      !json::get_int(json, "frameMax", &value) ||
      value < static_cast<long>(kFrameMaxBytes)) {
    return;
  }
  std::size_t want = value > static_cast<long>(kFrameMaxTxBytes)
                         ? kFrameMaxTxBytes
                         : static_cast<std::size_t>(value);
  const std::size_t limit = link_.max_tx_frame();
  if (want > limit) {
    want = limit;
  }
  if (want < kFrameMaxBytes) {
    want = kFrameMaxBytes;
  }
  next_tx_frame_ = want;
}

void Session::pump() noexcept {
  for (std::uint8_t sent = 0;
       sent < max_frames_ && tx_.active() && link_.can_send(); ++sent) {
    std::uint8_t frame[kFrameMaxTxBytes]; /* 255 B: the largest frame */
    const std::size_t len = tx_.peek(frame, link_.pad_frames());
    if (len == 0 || !link_.send_frame(frame, len)) {
      return; /* the same fragment is offered again on the next call (F-15) */
    }
    tx_.commit();
  }
}

void Session::on_link_down() noexcept {
  link_was_connected_ = false;
  rx_.reset();
  tx_.abort();
  tx_frame_ = kFrameMaxBytes;
  next_tx_frame_ = kFrameMaxBytes;
  handler_.reset();
}

void Session::task(std::uint32_t now_ms) noexcept {
  const bool connected = link_.connected();
  if (link_was_connected_ && !connected) {
    on_link_down();
  }
  link_was_connected_ = connected;

  rx_.poll_timeout(now_ms);

  if (rx_.ready() && !tx_.active()) {
    negotiate_frame_max(rx_.message());
    handler_.on_request(rx_.message(), *this);
    /* The reply began with the old size; later messages use the new one. */
    tx_frame_ = next_tx_frame_;
    rx_.release();
  }

  pump();
  /* A waiting request goes before the next notify: otherwise, on a link
   * that is still busy at the start of every task (BLE at full subscription
   * load), each finished notify is followed at once by another and the
   * request is never dispatched (SES-15). It is dispatched on the next task. */
  if (!tx_.active() && connected && !rx_.ready()) {
    handler_.poll(*this, now_ms);
    pump();
  }
}

} // namespace control
