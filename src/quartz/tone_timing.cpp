/**
 * @file tone_timing.cpp
 * @brief Tuning-fork and crystal tone timing.
 *
 * tone_rate(x, fs, f_nom, block_s, hp=True) in timing_ref.py is
 *   y = highpass(x, fs, fc=min(200, f_nom/2))
 *   harmonic_phase_rate(y, fs, f_nom, block_s, harmonics=[1]).
 * Here that is a Highpass4 plus a HarmonicPhaseTracker fixed to k = 1, with
 * blocks from 0, so no stage has to wait for anything to be learned.
 *
 * @see tone_timing.h
 */

#include "quartz/tone_timing.h"

#include <algorithm>

namespace quartz {

/*
 * fc = min(200 Hz, f/2). Fork tones (300–720 Hz) get 150–200 Hz: 60 Hz hum
 * is cut, but the tone sits a factor of >= 2 above the corner, where the
 * 4th-order response is flat (|H| > 0.997). The crystal gets 200 Hz, which
 * is only DC removal. The tracker is configured last; on any failure the
 * object stays unconfigured (fs_hz_ == 0) and process() is a no-op.
 */
bool ToneTiming::configure(uint32_t fs_hz, const Config &cfg) noexcept {
  if (fs_hz == 0u || fs_hz > kMaxSampleRateHz || cfg.f_nominal.den == 0u ||
      !(cfg.block_s > 0.0)) {
    return false;
  }
  const double f = cfg.f_nominal.hz();
  if (!hpf_.configure(static_cast<double>(fs_hz),
                      std::min(kMaxHighpassHz, 0.5 * f))) {
    return false;
  }
  if (!tracker_.configure(fold_.data(), fold_.size(),
                          tracker_config(fs_hz, cfg, 0u))) {
    return false;
  }
  fs_hz_ = fs_hz;
  cfg_ = cfg;
  reset();
  return true;
}

/*
 * k = 1 only, so harmonic selection never runs and the tracker needs just
 * one fold of b samples (b = 400 for 360 Hz, 1125 for 1024/3 Hz at 48 kHz,
 * 375 for the crystal at 96 kHz). block_len = int(block_s·fs), as
 * nb = int(block_s*fs) (12 000 for a fork, 4 800 for the crystal).
 */
HarmonicPhaseTracker::Config
ToneTiming::tracker_config(uint32_t fs_hz, const Config &cfg,
                           uint64_t origin) noexcept {
  HarmonicPhaseTracker::Config tc{};
  tc.f0 = cfg.f_nominal;
  tc.fs_hz = fs_hz;
  // int(block_s·fs), as harmonic_phase_rate().
  tc.block_len =
      static_cast<uint32_t>(cfg.block_s * static_cast<double>(fs_hz));
  tc.harmonics = {1u, 0u, 0u, 0u};
  tc.harmonic_count = 1u;
  tc.origin = origin;
  tc.first_block = 0u;
  return tc;
}

/* Clears the filter and re-anchors block 0 (and the phase reference) at the
 * new origin. Reconfiguring the tracker with known-good parameters is the
 * simplest way to re-anchor it, and it zeroes the fold. */
void ToneTiming::reset(uint64_t first_index) noexcept {
  if (fs_hz_ == 0u) {
    return;
  }
  hpf_.reset();
  // Re-anchor block 0 at the new origin (cannot fail: the same parameters
  // were accepted by configure()); the tracker clears its fold.
  static_cast<void>(tracker_.configure(
      fold_.data(), fold_.size(), tracker_config(fs_hz_, cfg_, first_index)));
  next_index_ = first_index;
  clipped_ = false;
}

/* Convert, filter and fold, one kChunk at a time. next_index_ keeps the
 * absolute index, so the tracker's fold slots and block boundaries follow
 * the stream exactly, however the caller splits it. */
void ToneTiming::process(const int32_t *codes, std::size_t n) noexcept {
  if (fs_hz_ == 0u) {
    return;
  }
  while (n > 0u) {
    const std::size_t len = std::min(n, kChunk);
    clipped_ = codes_to_float(codes, chunk_.data(), len) || clipped_;
    hpf_.process(chunk_.data(), chunk_.data(), len);
    tracker_.process(chunk_.data(), len, next_index_);
    next_index_ += len;
    codes += len;
    n -= len;
  }
}

} // namespace quartz
