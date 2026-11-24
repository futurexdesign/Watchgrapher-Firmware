/**
 * @file arbiter.cpp
 * @brief Ownership state machine and its per-link port; see arbiter.h.
 *
 * @see docs/plans/BLE_TRANSPORT_PLAN.md sections 4.7 and 5.3
 */

#include "control/arbiter.h"

#include "control/json_scan.h"

#include <cstdio>
#include <cstring>

namespace control {

namespace {

/** @return true when {@code a} is the operation {@code b}. */
[[nodiscard]] bool is_op(const char *a, const char *b) noexcept {
  return std::strcmp(a, b) == 0;
}

} // namespace

bool Arbiter::lease_valid(std::uint32_t now_ms) const noexcept {
  return owner_ != LinkKind::kNone &&
         static_cast<std::int32_t>(deadline_ms_ - now_ms) >= 0;
}

void Arbiter::take(LinkKind from, std::uint32_t now_ms, Outcome &out) noexcept {
  if (owner_ != LinkKind::kNone && owner_ != from) {
    out.reset_handler = true;
    out.notify_released = owner_;
  }
  owner_ = from;
  deadline_ms_ = now_ms + kLeaseMs;
}

Arbiter::Outcome Arbiter::on_request(LinkKind from, const char *op,
                                     bool takeover,
                                     std::uint32_t now_ms) noexcept {
  Outcome out;
  if (is_op(op, "hello")) {
    out.decision = Decision::kAnswerLocally;
    return out;
  }
  if (is_op(op, "ping")) {
    if (from == owner_) {
      deadline_ms_ = now_ms + kLeaseMs;
    }
    out.decision = Decision::kAnswerLocally;
    return out;
  }
  const bool claim = is_op(op, "claim");
  out.decision = claim ? Decision::kAnswerLocally : Decision::kDispatch;
  if (claim && takeover) {
    take(from, now_ms, out);
    return out;
  }
  if (owner_ == LinkKind::kNone || owner_ == from || !lease_valid(now_ms)) {
    take(from, now_ms, out);
    return out;
  }
  out.decision = Decision::kBusy;
  return out;
}

bool Arbiter::on_link_down(LinkKind kind) noexcept {
  if (owner_ != kind || kind == LinkKind::kNone) {
    return false;
  }
  owner_ = LinkKind::kNone;
  return true;
}

ArbiterPort::ArbiterPort(LinkKind kind, Arbiter &arbiter, Handler &backend,
                         ClockFn now) noexcept
    : kind_(kind), arbiter_(arbiter), backend_(backend), now_(now) {}

void ArbiterPort::queue_released(LinkKind to) noexcept {
  if (link_ != nullptr && link_->connected()) {
    pending_to_ = to;
  }
}

void ArbiterPort::reply_claim(Reply &out, std::int64_t id,
                              LinkKind previous) noexcept {
  char ids[24];
  json::i64_dec(id, ids);
  const char *prev = link_kind_name(previous);
  char buf[160];
  const int n = std::snprintf(
      buf, sizeof(buf),
      "{\"v\":1,\"id\":%s,\"ok\":true,\"result\":{\"owner\":\"%s\","
      "\"previous\":%s%s%s}}",
      ids, link_kind_name(kind_), prev != nullptr ? "\"" : "",
      prev != nullptr ? prev : "null", prev != nullptr ? "\"" : "");
  if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
    out.send(buf, static_cast<std::size_t>(n), false);
  }
}

void ArbiterPort::reply_busy(Reply &out, std::int64_t id) noexcept {
  char ids[24];
  json::i64_dec(id, ids);
  char buf[192];
  const char *holder = link_kind_name(arbiter_.owner());
  const int n = std::snprintf(
      buf, sizeof(buf),
      "{\"v\":1,\"id\":%s,\"ok\":false,\"error\":{\"code\":\"linkBusy\","
      "\"message\":\"control held by %s\"}}",
      ids, holder != nullptr ? holder : "none");
  if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
    out.send(buf, static_cast<std::size_t>(n), false);
  }
}

void ArbiterPort::on_request(const char *json_text, Reply &out) noexcept {
  std::int64_t id = 0;
  long version = 0;
  char op[32];
  if (!json::get_i64(json_text, "id", &id) ||
      !json::get_int(json_text, "v", &version) || version != 1 ||
      !json::get_string(json_text, "op", op, sizeof(op))) {
    backend_.on_request(json_text, out);
    return;
  }
  bool takeover = false;
  const bool claim = is_op(op, "claim");
  if (claim) {
    (void)json::get_bool(json_text, "takeover", &takeover);
  }
  const LinkKind previous = arbiter_.owner();
  const Arbiter::Outcome r = arbiter_.on_request(kind_, op, takeover, now_());
  if (r.reset_handler) {
    backend_.reset();
  }
  if (r.notify_released != LinkKind::kNone && peer_ != nullptr) {
    peer_->queue_released(kind_);
  }
  if (r.decision == Arbiter::Decision::kBusy) {
    reply_busy(out, id);
  } else if (claim) {
    reply_claim(out, id, previous);
  } else {
    backend_.on_request(json_text, out);
  }
}

void ArbiterPort::poll(Reply &out, std::uint32_t now_ms) noexcept {
  if (pending_to_ != LinkKind::kNone) {
    if (!out.idle()) {
      return;
    }
    char buf[96];
    const int n =
        std::snprintf(buf, sizeof(buf),
                      "{\"v\":1,\"notify\":true,\"op\":\"controlReleased\","
                      "\"data\":{\"to\":\"%s\"}}",
                      link_kind_name(pending_to_));
    pending_to_ = LinkKind::kNone;
    if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
      out.send(buf, static_cast<std::size_t>(n), true);
    }
    return;
  }
  if (arbiter_.owner() == kind_) {
    backend_.poll(out, now_ms);
  }
}

void ArbiterPort::reset() noexcept {
  pending_to_ = LinkKind::kNone;
  if (arbiter_.on_link_down(kind_)) {
    backend_.reset();
  }
}

} // namespace control
