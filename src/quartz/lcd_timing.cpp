/**
 * @file lcd_timing.cpp
 * @brief Harmonic phase tracker and the LCD pipeline.
 *
 * Map to {@code docs/handoff/ref/timing_ref.py}:
 *  - {@link quartz::HarmonicPhaseTracker} is {@code harmonic_phase_rate()}:
 *    the {@code nb = int(block_s*fs)} blocks, {@code np.hanning(nb)}, the
 *    first-block harmonic ranking, the per-block
 *    {@code z = Σ seg·exp(-2πi·k·f·(t + b·nb/fs))}, {@code np.unwrap},
 *    {@code lstsq([tc, 1])} and {@code np.average(slopes, weights=ks)};
 *  - {@link quartz::LcdTiming} is {@code lcd_rate()}: {@code highpass()},
 *    {@code lcd_period()}, then the tracker.
 *
 * @see lcd_timing.h
 */

#include "quartz/lcd_timing.h"

#include <algorithm>
#include <cmath>

namespace quartz {

namespace {

/** π in double. */
constexpr double kPi = 3.14159265358979323846;

} // namespace

// ----------------------------------------------------------------- tracker

/*
 * Validation, then the derived constants:
 *  - ratio_ = f0/fs in lowest terms (a/b). Every harmonic k·f0 has the exact
 *    ratio (k·a mod b)/b, so one b-sample fold serves all harmonics;
 *  - auto-selection needs the buffer to hold two folds (active + parked);
 *  - kmax = floor(f_max/f0) is the reference's int(f_max // f_nom). The
 *    highest candidate must stay below Nyquist, otherwise its "harmonic"
 *    would alias onto a lower frequency;
 *  - hann_step_ = 2π/(nb-1): np.hanning(nb) is the symmetric window
 *    0.5 - 0.5·cos(2πj/(nb-1)), j = 0..nb-1 (both ends exactly zero).
 */
bool HarmonicPhaseTracker::configure(float *fold, std::size_t fold_cap,
                                     const Config &cfg) noexcept {
  const auto ratio = reduce_ratio(cfg.f0, cfg.fs_hz);
  if (fold == nullptr || !ratio || cfg.block_len < 2u ||
      cfg.harmonic_count > kMaxHarmonics) {
    return false;
  }
  const bool auto_select = cfg.harmonic_count == 0u;
  const std::size_t need =
      static_cast<std::size_t>(ratio->b) * (auto_select ? 2u : 1u);
  if (need > fold_cap) {
    return false;
  }
  const double f0 = cfg.f0.hz();
  const double nyquist = 0.5 * static_cast<double>(cfg.fs_hz);
  if (auto_select) {
    const auto kmax = static_cast<uint32_t>(std::floor(cfg.f_max_hz / f0));
    if (kmax == 0u || static_cast<double>(kmax) * f0 >= nyquist) {
      return false;
    }
  } else {
    for (std::size_t i = 0; i < cfg.harmonic_count; i++) {
      if (cfg.harmonics[i] == 0u ||
          static_cast<double>(cfg.harmonics[i]) * f0 >= nyquist) {
        return false;
      }
    }
  }
  base_ = fold;
  cfg_ = cfg;
  ratio_ = *ratio;
  hann_step_ = 2.0 * kPi / static_cast<double>(cfg.block_len - 1u);
  reset();
  return true;
}

/*
 * Restores the state at the first tracked block:
 *  - start_ = origin + first_block·nb. Blocks are aligned to the stream
 *    origin exactly as the reference aligns them to the start of the record
 *    (block b covers [b·nb, (b+1)·nb)), even when tracking starts late;
 *  - fold_pos_ = start_ mod b. The fold slot of absolute sample n is
 *    n mod b, and it is kept incrementally from here on. This is the
 *    "rational phase reference": slot r carries the phase 2π·k·a·r/b of
 *    every sample folded into it, which is exactly
 *    2π·k·f0·n/fs mod 2π, with no rounding however large n gets;
 *  - the active fold is the lower half and the parked fold the upper half.
 *    Only the first b floats of each are used.
 */
void HarmonicPhaseTracker::reset() noexcept {
  if (base_ == nullptr) {
    return;
  }
  fold_ = base_;
  parked_ = (cfg_.harmonic_count == 0u) ? base_ + ratio_.b : nullptr;
  std::fill(fold_, fold_ + ratio_.b, 0.f);
  if (parked_ != nullptr) {
    std::fill(parked_, parked_ + ratio_.b, 0.f);
  }
  start_ =
      cfg_.origin + static_cast<uint64_t>(cfg_.first_block) * cfg_.block_len;
  next_index_ = start_;
  pos_ = 0u;
  block_index_ = cfg_.first_block;
  fold_pos_ = static_cast<uint32_t>(start_ % ratio_.b);
  selected_ = cfg_.harmonic_count > 0u;
  ranking_ = false;
  rank_k_ = 1u;
  rank_kmax_ = static_cast<uint32_t>(std::floor(cfg_.f_max_hz / cfg_.f0.hz()));
  parked_block_ = 0u;
  best_mag_.fill(0.0);
  ks_ = cfg_.harmonics;
  k_count_ = cfg_.harmonic_count;
  for (auto &u : unwrap_) {
    u.reset();
  }
  for (auto &f : fits_) {
    f.reset();
  }
  blocks_ = 0u;
  tc0_ = 0.0;
}

/*
 * (k·a) mod b in 64-bit arithmetic: k <= 156 and a < b <= 2^32, so the
 * product fits easily. Reducing a harmonic's ratio this way keeps the
 * carrier exact. k·a/b may exceed 1 (several cycles per sample period of
 * the fold), and only its fractional part matters for the phase.
 */
FrequencyRatio HarmonicPhaseTracker::harmonic_ratio(uint32_t k) const noexcept {
  const uint64_t a = (static_cast<uint64_t>(k) * ratio_.a) % ratio_.b;
  return FrequencyRatio{static_cast<uint32_t>(a), ratio_.b};
}

/*
 * Per-sample path. Each pass of the loop handles one run that stays inside
 * one block and one kChunk of scratch:
 *  1. the Hann window for block positions [pos_, pos_+len) is produced by
 *     fill_phasor() seeded at the exact angle hann_step_·pos_ (computed in
 *     double from the integer position, so no window drift accumulates
 *     over the 24 000-sample block) and turned into 0.5 - 0.5·cos;
 *  2. u = y·w with arm_mult_f32;
 *  3. u is added into fold_[n mod b], with fold_pos_ walking the fold and
 *     wrapping at b. Folding is valid for any nb, not just multiples of b:
 *     Σ_n u[n]·e^{-iθn} = Σ_r F[r]·e^{-iθr} because e^{-iθn} has period b;
 *  4. at the end of a block, finish_block() parks it or pushes its phases.
 * After the samples come the ranking steps for this call, so ranking cost is
 * bounded per call (see the header for the worst case).
 */
void HarmonicPhaseTracker::process(const float *y, std::size_t n,
                                   uint64_t first_index) noexcept {
  if (base_ == nullptr) {
    return;
  }
  // Samples before the first tracked block (the LCD's 2 s period record
  // tail, for instance) are not part of any tracked block.
  if (first_index < start_) {
    const uint64_t skip = std::min<uint64_t>(start_ - first_index, n);
    y += skip;
    n -= static_cast<std::size_t>(skip);
    first_index += skip;
  }
  // A gap would break the fold-slot bookkeeping; the owner resets instead.
  if (n > 0u && first_index == next_index_) {
    while (n > 0u) {
      const std::size_t len = std::min<std::size_t>(
          {n, kChunk, static_cast<std::size_t>(cfg_.block_len - pos_)});
      fill_phasor(w_.data(), s_.data(), len,
                  hann_step_ * static_cast<double>(pos_), hann_step_);
      for (std::size_t j = 0; j < len; j++) {
        w_[j] = 0.5f - 0.5f * w_[j];
      }
      arm_mult_f32(y, w_.data(), u_.data(), static_cast<uint32_t>(len));
      for (std::size_t j = 0; j < len; j++) {
        fold_[fold_pos_] += u_[j];
        if (++fold_pos_ == ratio_.b) {
          fold_pos_ = 0u;
        }
      }
      y += len;
      n -= len;
      pos_ += static_cast<uint32_t>(len);
      next_index_ += len;
      if (pos_ == cfg_.block_len) {
        finish_block();
        pos_ = 0u;
        block_index_++;
      }
    }
  }
  if (ranking_) {
    rank_step(kRankStepsPerCall);
  }
}

/*
 * Ranking of candidate harmonics k = rank_k_ .. rank_kmax_ on the parked
 * fold, by Goertzel magnitude. This is the reference's
 *   X = rfft(y[:nb]·hanning(nb)); mags[k] = |X[round(k·f0·nb/fs)]|;
 *   ks = argsort(mags)[::-1][:4]
 * with the exact frequency k·f0 in place of the nearest bin. For the LCD
 * (f0 even, nb = fs/2) k·f0·nb/fs = k·f0/2 is an integer, so the bin is
 * exact and the two agree.
 *
 * Budget: a candidate costs b Goertzel steps; candidates are taken while
 * the call's spend stays within the budget, and at least one is always
 * taken so a budget smaller than b still makes progress. The top-four list
 * is kept sorted descending by insertion (ties keep the earlier k, which
 * matches a stable descending order; argsort's order on exact ties is
 * unspecified). When the last candidate is done the parked block's phases
 * are pushed, so it enters the fits before any later block.
 */
void HarmonicPhaseTracker::rank_step(uint32_t budget) noexcept {
  uint64_t spent = 0u;
  while (rank_k_ <= rank_kmax_ && (spent == 0u || spent + ratio_.b <= budget)) {
    const uint32_t k = rank_k_++;
    spent += ratio_.b;
    const double mag = std::abs(goertzel(parked_, ratio_.b, harmonic_ratio(k)));
    std::size_t pos = k_count_;
    while (pos > 0u && best_mag_[pos - 1u] < mag) {
      pos--;
    }
    if (pos >= kMaxHarmonics) {
      continue;
    }
    const std::size_t last =
        std::min<std::size_t>(k_count_, kMaxHarmonics - 1u);
    for (std::size_t i = last; i > pos; i--) {
      best_mag_[i] = best_mag_[i - 1u];
      ks_[i] = ks_[i - 1u];
    }
    best_mag_[pos] = mag;
    ks_[pos] = static_cast<uint16_t>(k);
    if (k_count_ < kMaxHarmonics) {
      k_count_++;
    }
  }
  if (rank_k_ > rank_kmax_) {
    ranking_ = false;
    selected_ = true;
    push_block(parked_, parked_block_);
    std::fill(parked_, parked_ + ratio_.b, 0.f);
  }
}

/*
 * Block end. Three cases:
 *  - first block with auto-selection: swap the halves. The finished fold
 *    becomes parked_ and is ranked over the following calls. Folding goes
 *    on into the other half, which reset() left zeroed. Nothing is pushed
 *    yet, because the harmonics are not known;
 *  - a block ends while ranking is still pending (calls far larger than
 *    one 240-frame block): finish the ranking now, which pushes the parked
 *    block first, then push this block, keeping the fits in block order;
 *  - normal: push this block's phases.
 * The active fold is then cleared for the next block.
 */
void HarmonicPhaseTracker::finish_block() noexcept {
  if (!selected_ && !ranking_) {
    std::swap(fold_, parked_);
    parked_block_ = block_index_;
    ranking_ = true;
    k_count_ = 0u;
    return; // fold_ (the former parked half) is already zero
  }
  if (ranking_) {
    rank_step(UINT32_MAX);
  }
  push_block(fold_, block_index_);
  std::fill(fold_, fold_ + ratio_.b, 0.f);
}

/*
 * One block into the fits, for each tracked k:
 *  - z = Σ_r F[r]·e^{-2πi·(k·a mod b)·r/b}, by single_bin_dft() with m0 = 0.
 *    Slot r holds samples with n ≡ r (mod b), so this equals the
 *    reference's z = Σ seg·exp(-2πi·k·f·(t + b·nb/fs)), whose phase is
 *    referenced to absolute sample 0;
 *  - phase = np.unwrap(angle(z)) continued online by PhaseUnwrapper. A jump
 *    of ±2π between blocks can only come from wrapping, because f_max keeps
 *    the true per-block change far below π;
 *  - the fit is of phase/k against tc, where tc = (b·nb + nb/2)/fs is the
 *    reference's block-centre time, rebased on the first pushed block so
 *    that the LineFit abscissae stay small (a slope does not depend on the
 *    origin).
 */
void HarmonicPhaseTracker::push_block(const float *fold,
                                      uint32_t block_index) noexcept {
  const double tc = (static_cast<double>(block_index) * cfg_.block_len +
                     0.5 * static_cast<double>(cfg_.block_len)) /
                    static_cast<double>(cfg_.fs_hz);
  if (blocks_ == 0u) {
    tc0_ = tc;
  }
  for (std::size_t i = 0; i < k_count_; i++) {
    const uint32_t k = ks_[i];
    const std::complex<double> z =
        single_bin_dft(fold, ratio_.b, harmonic_ratio(k), 0u);
    const double ph = unwrap_[i].push(std::arg(z));
    fits_[i].add(tc - tc0_, ph / static_cast<double>(k));
  }
  blocks_++;
}

/*
 * eps = Σ k·slope_k / Σ k / (2π·f0), the reference's
 * np.average(slopes, weights=ks) / (2π·f_nom). slope_k is in rad/s of phase
 * per unit harmonic, i.e. 2π·(f_meas - f_nom). Weighting by k favours the
 * higher harmonics, whose phase is k times more sensitive to the same
 * frequency error. Sign: z = Σ y·e^{-iωn} rotates forward when the signal
 * runs faster than ω, so positive eps means gaining, as in the reference.
 */
std::optional<PhaseRate> HarmonicPhaseTracker::result() const noexcept {
  if (blocks_ < 2u || k_count_ == 0u) {
    return std::nullopt;
  }
  double num = 0.0;
  double den = 0.0;
  for (std::size_t i = 0; i < k_count_; i++) {
    const auto sl = fits_[i].slope();
    if (!sl) {
      return std::nullopt;
    }
    num += static_cast<double>(ks_[i]) * *sl;
    den += static_cast<double>(ks_[i]);
  }
  PhaseRate r{};
  r.f_nominal_hz = cfg_.f0.hz();
  r.harmonics = ks_;
  r.harmonic_count = k_count_;
  r.blocks = blocks_;
  r.fractional_error = (num / den) / (2.0 * kPi * r.f_nominal_hz);
  r.rate_s_per_day = kSecondsPerDay * r.fractional_error;
  return r;
}

// --------------------------------------------------------------------- LCD

/* lcd_rate(): y = highpass(x, fs), with the default fc = 400 Hz. */
bool LcdTiming::configure(uint32_t fs_hz) noexcept {
  if (fs_hz == 0u || fs_hz > kMaxSampleRateHz ||
      !hpf_.configure(static_cast<double>(fs_hz), kLcdHighpassHz)) {
    return false;
  }
  fs_hz_ = fs_hz;
  reset();
  return true;
}

/*
 * Back to stage 1:
 *  - record_len_ = int(2·fs), as seg = y[:int(seconds*fs)] in lcd_period();
 *  - lag_[i] = round(fs/f_i), as L = int(round(fs / f)). At 48 kHz these are
 *    375, 750, 1500 and 3000 samples, and at 96 kHz twice that, which is why
 *    the delay line is sized fs_max/16 + one chunk.
 */
void LcdTiming::reset(uint64_t first_index) noexcept {
  hpf_.reset();
  buf_.fill(0.f);
  origin_ = first_index;
  seen_ = 0u;
  const auto fs = static_cast<double>(fs_hz_);
  record_len_ = static_cast<uint32_t>(kPeriodSeconds * fs);
  for (std::size_t i = 0; i < kCandidatesHz.size(); i++) {
    lag_[i] = static_cast<uint32_t>(
        round_to_int(fs / static_cast<double>(kCandidatesHz[i])));
  }
  cross_.fill(0.0);
  head_.fill(0.0);
  sum_ = 0.0;
  sumsq_ = 0.0;
  status_ = LcdStatus::kDetectingPeriod;
  pattern_hz_ = 0u;
  autocorr_.fill(0.f);
  clipped_ = false;
}

/*
 * The ring index of sample n is n mod kBufferCap. Both operands of a lag
 * product (n and n - L) may wrap independently, so the range is cut where
 * either one wraps, giving at most three contiguous pieces. Each piece is
 * one arm_dot_prod_f32 (float, at most a chunk long), and the pieces are
 * summed in double.
 */
double LcdTiming::ring_dot(std::size_t pa, std::size_t pb,
                           std::size_t len) const noexcept {
  double acc = 0.0;
  while (len > 0u) {
    const std::size_t run = std::min({len, kBufferCap - pa, kBufferCap - pb});
    float d = 0.f;
    arm_dot_prod_f32(buf_.data() + pa, buf_.data() + pb,
                     static_cast<uint32_t>(run), &d);
    acc += static_cast<double>(d);
    pa = (pa + run) % kBufferCap;
    pb = (pb + run) % kBufferCap;
    len -= run;
  }
  return acc;
}

/*
 * Streaming form of lcd_period()'s sums over seg = y[0..N):
 *  1. the chunk (sample numbers n0..n0+len, clipped to N) goes into the
 *     delay line first, so a lag partner n - L is always present: the ring
 *     holds L_max + one chunk, so writing this chunk cannot overwrite a
 *     partner of its own samples;
 *  2. S += Σy and Q += Σy² (arm_power_f32): per-chunk float sums of at most
 *     256 terms, accumulated in double;
 *  3. for each lag L: head_L collects Σ y[n] over n < L (needed for the
 *     mean correction of seg[L:]), and P_L += Σ y[n]·y[n-L] over the
 *     chunk's samples with n >= L. That is np.dot(seg[:-L], seg[L:]) term
 *     by term.
 * The mean is not known until the end, so it is removed afterwards with the
 * identity in decide_period() rather than by a second pass.
 */
std::size_t LcdTiming::accumulate_period(const float *y,
                                         std::size_t n) noexcept {
  const auto n0 = static_cast<std::size_t>(seen_);
  const std::size_t len = std::min<std::size_t>(n, record_len_ - n0);
  for (std::size_t j = 0; j < len; j++) {
    buf_[(n0 + j) % kBufferCap] = y[j];
  }
  float s = 0.f;
  for (std::size_t j = 0; j < len; j++) {
    s += y[j];
  }
  float q = 0.f;
  arm_power_f32(y, static_cast<uint32_t>(len), &q);
  sum_ += static_cast<double>(s);
  sumsq_ += static_cast<double>(q);
  for (std::size_t i = 0; i < lag_.size(); i++) {
    const std::size_t lag = lag_[i];
    if (n0 < lag) {
      const std::size_t h = std::min(len, lag - n0);
      float hs = 0.f;
      for (std::size_t j = 0; j < h; j++) {
        hs += y[j];
      }
      head_[i] += static_cast<double>(hs);
    }
    const std::size_t from = std::max(n0, lag);
    if (from < n0 + len) {
      cross_[i] += ring_dot(from % kBufferCap, (from - lag) % kBufferCap,
                            n0 + len - from);
    }
  }
  return len;
}

/*
 * Mean-removed correlation from raw sums, with m = S/N:
 *   r0 = Σ(y-m)² = Q - N·m²
 *   Σ_{n>=L} (y[n]-m)(y[n-L]-m)
 *      = P_L - m·(Σ_{n>=L} y[n] + Σ_{n<N-L} y[n]) + (N-L)·m²
 *      = P_L - m·((S - head_L) + (S - tail_L)) + (N-L)·m²,
 * where tail_L = Σ of the last L samples, read back from the delay line
 * (still intact, since only the record was written). The candidates are
 * tested highest first against 0.8, as lcd_period() does, so the first pass
 * wins. A 64 Hz pattern also correlates at 32 and 16 Hz, and taking the
 * highest picks its true period.
 *
 * Stage 2 then begins at the first block boundary at or after N (block 4
 * for 0.5 s blocks and a 2 s record). The delay line is cleared and lent to
 * the tracker as its fold buffer (two folds of b = fs/f0: 6000 floats at
 * 48 kHz and 16 Hz).
 */
void LcdTiming::decide_period() noexcept {
  const auto big_n = static_cast<double>(record_len_);
  const double m = sum_ / big_n;
  const double r0 = sumsq_ - big_n * m * m;
  for (std::size_t i = 0; i < lag_.size(); i++) {
    const std::size_t lag = lag_[i];
    double tail = 0.0;
    for (std::size_t j = record_len_ - lag; j < record_len_; j++) {
      tail += static_cast<double>(buf_[j % kBufferCap]);
    }
    const double cross = cross_[i] - m * (2.0 * sum_ - head_[i] - tail) +
                         (big_n - static_cast<double>(lag)) * m * m;
    const double r = (r0 > 0.0) ? cross / r0 : 0.0;
    autocorr_[i] = static_cast<float>(r);
    if (pattern_hz_ == 0u && r > kPeriodThreshold) {
      pattern_hz_ = kCandidatesHz[i];
    }
  }
  if (pattern_hz_ == 0u) {
    status_ = LcdStatus::kNoPattern;
    return;
  }
  const auto block_len =
      static_cast<uint32_t>(kBlockSeconds * static_cast<double>(fs_hz_));
  HarmonicPhaseTracker::Config cfg{};
  cfg.f0 = Rational{pattern_hz_, 1u};
  cfg.fs_hz = fs_hz_;
  cfg.block_len = block_len;
  cfg.f_max_hz = kFMaxHz;
  cfg.origin = origin_;
  cfg.first_block = (record_len_ + block_len - 1u) / block_len;
  buf_.fill(0.f);
  status_ = tracker_.configure(buf_.data(), buf_.size(), cfg)
                ? LcdStatus::kTracking
                : LcdStatus::kUnsupportedRate;
}

/*
 * Chunk loop: convert, high-pass (one continuous filter for the whole
 * stream, so stage 2 sees the same y the reference does), then route.
 * Within the chunk that completes the 2 s record, the samples up to N go to
 * stage 1, the period is decided, and the remainder of the same chunk goes
 * to the tracker. The tracker itself skips samples before its first block.
 * seen_ counts every sample since reset, so absolute index = origin_ +
 * seen_.
 */
void LcdTiming::process(const int32_t *codes, std::size_t n) noexcept {
  if (fs_hz_ == 0u) {
    return;
  }
  while (n > 0u) {
    const std::size_t len = std::min(n, kChunk);
    clipped_ = codes_to_float(codes, chunk_.data(), len) || clipped_;
    hpf_.process(chunk_.data(), chunk_.data(), len);
    std::size_t done = 0u;
    if (status_ == LcdStatus::kDetectingPeriod) {
      done = accumulate_period(chunk_.data(), len);
      if (seen_ + done == record_len_) {
        decide_period();
      }
    }
    if (status_ == LcdStatus::kTracking && done < len) {
      tracker_.process(chunk_.data() + done, len - done,
                       origin_ + seen_ + done);
    }
    seen_ += len;
    codes += len;
    n -= len;
  }
}

/* Snapshot; the rate exists only while tracking with two or more blocks. */
LcdResult LcdTiming::result() const noexcept {
  LcdResult r{};
  r.status = status_;
  r.pattern_hz = pattern_hz_;
  r.autocorr = autocorr_;
  if (status_ == LcdStatus::kTracking) {
    r.rate = tracker_.result();
  }
  return r;
}

} // namespace quartz
