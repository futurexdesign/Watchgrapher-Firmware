/**
 * @file motor_timing.cpp
 * @brief Stepper-motor pulse detection, timestamps and segmented rate fit.
 *
 * @see motor_timing.h
 */

#include "quartz/motor_timing.h"

#include <algorithm>
#include <cmath>

namespace quartz {

namespace {

/** π in double. */
constexpr double kPi = 3.14159265358979323846;
/** sqrt(π/2): sigma / mean|y| for Gaussian noise. */
constexpr float kSigmaPerMeanAbs = 1.2533141373155003f;
/** High-pass settling skipped before noise calibration (s). */
constexpr double kSettleS = 0.02;
/** Noise sub-block (s). */
constexpr double kNoiseBlockS = 0.01;
/** Quiet run that separates two bursts inside a group (s). */
constexpr double kBurstGapS = 2e-3;
/** Longest drive pulse (s), signal_spec.yaml drive_pulse_width_ms. */
constexpr double kMaxDriveS = 20e-3;

/**
 * The geometry at the highest supported rate, so the window capacity can be
 * checked at compile time.
 */
constexpr uint32_t max_window_len() {
  constexpr double fs = kMaxSampleRateHz;
  constexpr auto peak_len = static_cast<uint32_t>(0.3e-3 * fs) + 2u;
  constexpr auto post = static_cast<uint32_t>(round_to_int(0.3e-3 * fs));
  return kPulsePreHistory + (peak_len - 1u + post) + 1u;
}
static_assert(max_window_len() <= kPulseWindowCap,
              "pulse window does not fit at the highest sample rate");
static_assert((kPulseWindowCap & (kPulseWindowCap - 1u)) == 0u,
              "history ring must be a power of two");

/**
 * Median of a small float array with numpy semantics; reorders it.
 *
 * @param v Values.
 * @param n Count, > 0.
 * @return The median.
 */
double median_inplace(float *v, std::size_t n) noexcept {
  const std::size_t mid = n / 2u;
  std::nth_element(v, v + mid, v + n);
  const auto upper = static_cast<double>(v[mid]);
  if ((n & 1u) != 0u) {
    return upper;
  }
  const auto lower = static_cast<double>(*std::max_element(v, v + mid));
  return 0.5 * (lower + upper);
}

/**
 * The allowed step nearest a median interval; the first of two equal
 * distances wins, as Python's min(ALLOWED_STEP_S, key=...) does.
 *
 * @param med Median interval (s).
 * @return The snapped step (s).
 */
double snap_to_allowed(double med) noexcept {
  double step = kAllowedStepS[0];
  for (double s : kAllowedStepS) {
    if (std::fabs(s - med) < std::fabs(step - med)) {
      step = s;
    }
  }
  return step;
}

} // namespace

// ---------------------------------------------------------------- geometry

/*
 * Sample counts at 48 kHz (and 96 kHz):
 *  - gap = 0.05·fs = 2400 (4800): detect_motor_pulses(gap_s=0.05);
 *  - peak_len = int(0.3e-3·fs) + 2 = 16 (30): the slice
 *    y[i0 : i0 + int(0.3e-3*fs) + 2] searched for the leading spike. C++
 *    truncation of the same double product reproduces Python's int()
 *    (0.3e-3·48000 = 14.3999… -> 14);
 *  - pre = round(0.2e-3·fs) = 10 (19) and post = round(0.3e-3·fs) = 14 (29):
 *    timestamp_pulses(pre_s, post_s). The segment pol·y[ip-pre : ip+post]
 *    is 24 (48) samples and stays inside the first chopping on-time
 *    (0.366 ms at 37.5 % duty);
 *  - after = peak_len - 1 + post = 29 (58): the last window sample is
 *    i_first + after (the latest possible peak plus post), so a window
 *    can be captured that many samples after onset, i.e. 0.6 ms later;
 *  - settle 20 ms: the 400 Hz 4th-order high-pass has poles at about
 *    -960 s^-1 (time constant ~1 ms), so a start-up transient has decayed
 *    by e^-20;
 *  - noise_block 10 ms: long enough for a stable mean|y| (480 samples) and
 *    short enough that a pulse spoils few of them;
 *  - burst_gap 2 ms: two chopping periods (1/1024 s), so the gaps between
 *    chop spikes of one drive pulse never count as a new burst;
 *  - max_drive 20 ms: the longest drive pulse in signal_spec.yaml.
 */
std::optional<PulseGeometry> PulseGeometry::for_rate(uint32_t fs_hz,
                                                     double gap_s) noexcept {
  if (fs_hz == 0u || fs_hz > kMaxSampleRateHz || !(gap_s > 0.0)) {
    return std::nullopt;
  }
  const auto fs = static_cast<double>(fs_hz);
  PulseGeometry g{};
  g.fs_hz = fs_hz;
  g.gap_samples = gap_s * fs;
  // int() truncation and round() exactly as in detect_motor_pulses() and
  // timestamp_pulses(), so windows land on the same samples.
  g.peak_len = static_cast<uint32_t>(0.3e-3 * fs) + 2u;
  g.pre = static_cast<uint32_t>(round_to_int(0.2e-3 * fs));
  g.post = static_cast<uint32_t>(round_to_int(0.3e-3 * fs));
  g.after = g.peak_len - 1u + g.post;
  g.window_len = kPulsePreHistory + g.after + 1u;
  g.settle = static_cast<uint32_t>(round_to_int(kSettleS * fs));
  g.noise_block = static_cast<uint32_t>(round_to_int(kNoiseBlockS * fs));
  g.burst_gap = static_cast<uint32_t>(round_to_int(kBurstGapS * fs));
  g.max_drive = static_cast<uint32_t>(round_to_int(kMaxDriveS * fs));
  if (g.window_len > kPulseWindowCap || g.pre > kPulsePreHistory ||
      g.noise_block == 0u) {
    return std::nullopt;
  }
  return g;
}

// ------------------------------------------------------- threshold method

/*
 * timestamp_pulses(method="threshold") line by line:
 *   half = 0.5·s·y[ip]; k = ip
 *   while k > 0 and s·y[k-1] > half: k -= 1
 *   frac = (half - y0)/(y1 - y0), y0 = s·y[k-1], y1 = s·y[k]
 *   t = (k - 1 + frac)/fs
 * Here k is a window index and the result is in absolute samples (the
 * caller divides by fs). The reference stops at record index 0 and this
 * stops at window index 1; with 64 samples of pre-history before i_first,
 * every real edge is reached long before that bound. The arithmetic is in
 * double, so the only difference from the reference is float32 y.
 */
double threshold_timestamp(const PulseWindow &w) noexcept {
  const auto s = static_cast<double>(w.polarity);
  auto k = static_cast<std::size_t>(w.i_peak - w.start);
  const double half = 0.5 * s * static_cast<double>(w.y[k]);
  // Walk back down the leading edge (bounded by the window; the 64-sample
  // pre-history is far longer than any edge, so the bound never binds).
  while (k > 1u && s * static_cast<double>(w.y[k - 1u]) > half) {
    k--;
  }
  const double y0 = s * static_cast<double>(w.y[k - 1u]);
  const double y1 = s * static_cast<double>(w.y[k]);
  const double frac = (y1 != y0) ? (half - y0) / (y1 - y0) : 0.0;
  return static_cast<double>(w.start) + static_cast<double>(k - 1u) + frac;
}

// ---------------------------------------------------------------- detector

/* Geometry first (it validates fs), then the parameters, then a full reset.
 * A failed configure leaves the previous configuration untouched. */
bool PulseDetector::configure(uint32_t fs_hz, const Config &cfg) noexcept {
  const auto g = PulseGeometry::for_rate(fs_hz, cfg.gap_s);
  if (!g || !(cfg.k_sigma > 0.f)) {
    return false;
  }
  geo_ = *g;
  cfg_ = cfg;
  reset(0u);
  return true;
}

/* Start-up state: settling, then calibration. The history ring is zeroed so
 * a window captured very early cannot contain stale samples. */
void PulseDetector::reset(uint64_t first_index) noexcept {
  phase_ = Phase::kSettling;
  next_index_ = first_index;
  settle_left_ = geo_.settle;
  ring_.fill(0.f);
  have_clip_ = false;
  last_clip_ = 0u;
  calib_.fill(0.f);
  calib_count_ = 0u;
  blk_sum_ = 0.f;
  blk_quiet_ = 0u;
  blk_n_ = 0u;
  mean_abs_ = 0.f;
  thr_ = 0.f;
  group_open_ = false;
  captured_ = false;
  group_clipped_ = false;
  extras_ = 0u;
  i_first_ = 0u;
  last_over_ = 0u;
}

/* sigma = sqrt(π/2)·E|y| for zero-mean Gaussian noise. */
float PulseDetector::noise_sigma() const noexcept {
  return kSigmaPerMeanAbs * mean_abs_;
}

/* thr = k_sigma·sigma, as thr = k_sigma·robust_sigma(y) in the reference. */
void PulseDetector::update_threshold() noexcept {
  thr_ = cfg_.k_sigma * kSigmaPerMeanAbs * mean_abs_;
}

/*
 * The per-sample state machine. For every sample:
 *  1. it enters the 128-sample history ring at idx & 127. The ring always
 *     holds the latest samples, so a window can be copied out when it
 *     completes;
 *  2. a railed raw code updates last_clip_, and also the open group's clip
 *     flag;
 *  3. by phase:
 *     Settling    - count down geo_.settle samples (high-pass transient);
 *     Calibrating - sum |y| over kCalibBlocks sub-blocks of 10 ms. The
 *                   median of their means seeds mean|y|, so one pulse in
 *                   those 80 ms moves the median by at most one rank;
 *     Armed       - step_armed().
 * The loop returns right after any sample that produced an event, so an
 * event is never overwritten. The caller resumes with the rest of the
 * block, and no queue is needed.
 */
std::size_t PulseDetector::process(const float *y, const int32_t *raw,
                                   std::size_t n, Event &ev) noexcept {
  ev = Event{};
  for (std::size_t i = 0; i < n; i++) {
    const uint64_t idx = next_index_++;
    const float v = y[i];
    const float a = std::fabs(v);
    ring_[idx & (kRing - 1u)] = v;
    if (raw[i] >= kClipCode || raw[i] <= -kClipCode) {
      have_clip_ = true;
      last_clip_ = idx;
      group_clipped_ = group_clipped_ || group_open_;
    }
    switch (phase_) {
    case Phase::kSettling:
      if (settle_left_ > 0u) {
        settle_left_--;
      }
      if (settle_left_ == 0u) {
        phase_ = Phase::kCalibrating;
      }
      break;
    case Phase::kCalibrating:
      blk_sum_ += a;
      if (++blk_n_ == geo_.noise_block) {
        calib_[calib_count_++] = blk_sum_ / static_cast<float>(blk_n_);
        blk_sum_ = 0.f;
        blk_n_ = 0u;
        if (calib_count_ == kCalibBlocks) {
          std::array<float, kCalibBlocks> tmp = calib_;
          mean_abs_ =
              static_cast<float>(median_inplace(tmp.data(), tmp.size()));
          update_threshold();
          phase_ = Phase::kArmed;
        }
      }
      break;
    case Phase::kArmed:
      if (step_armed(idx, a, ev)) {
        return i + 1u;
      }
      break;
    }
  }
  return n;
}

/*
 * Armed phase: the grouping of detect_motor_pulses() run online.
 *   idx = where(|y| > thr); groups = split(idx, where(diff(idx) > gap) + 1)
 * becomes:
 *  - no group open and |y| > thr: open a group at i_first = idx;
 *  - group open and |y| > thr: extend it (last_over_ = idx). If this
 *    sample starts a new burst (at least burst_gap since the last
 *    over-threshold sample) more than max_drive after onset, count an
 *    "extra";
 *  - group open, quiet, and idx - last_over_ > gap: no later sample can
 *    join any more (the reference would start a new group), so the group
 *    closes and kGroupClosed reports i_last = last_over_;
 *  - when idx reaches i_first + after, every sample the timestamp methods
 *    need is in the ring, so the window is captured and kWindowReady is
 *    reported. This happens 0.6 ms after onset, long before the close.
 *    At most one event per sample: capture is at onset + 29 samples and
 *    close is no earlier than onset + 2401.
 * Noise: a sample counts as quiet when no group is open and it is under
 * threshold. A sub-block updates mean|y| only if at least half of it was
 * quiet (otherwise a pulse dominated it). The update is an exponential
 * average over kNoiseAvgBlocks = 32 sub-blocks (0.32 s), which follows a
 * slowly changing noise floor (hum residue, handling) without reacting to
 * single events. The threshold follows the estimate immediately.
 */
bool PulseDetector::step_armed(uint64_t idx, float a, Event &ev) noexcept {
  bool produced = false;
  const bool over = a > thr_;
  if (group_open_) {
    if (over) {
      // A burst that starts after a quiet run, beyond the longest drive
      // pulse, is a detection/correction pulse: log it, do not time it.
      if (idx - last_over_ >= geo_.burst_gap &&
          idx - i_first_ > geo_.max_drive && extras_ < UINT8_MAX) {
        extras_++;
      }
      last_over_ = idx;
    } else if (static_cast<double>(idx - last_over_) > geo_.gap_samples) {
      // No later over-threshold sample can join: the reference would start
      // a new group at any sample more than gap_s·fs after the last one.
      group_open_ = false;
      ev.kind = EventKind::kGroupClosed;
      ev.i_last = last_over_;
      ev.extras = extras_;
      ev.clipped = group_clipped_;
      produced = true;
    }
    if (group_open_ && !captured_ && idx == i_first_ + geo_.after) {
      capture_window();
      captured_ = true;
      ev.kind = EventKind::kWindowReady;
      ev.window = &window_;
      produced = true;
    }
  } else if (over) {
    group_open_ = true;
    captured_ = false;
    extras_ = 0u;
    i_first_ = idx;
    last_over_ = idx;
    group_clipped_ = have_clip_ && last_clip_ + geo_.pre >= idx;
  }

  // Noise estimate from quiet samples only.
  if (!group_open_ && !over) {
    blk_sum_ += a;
    blk_quiet_++;
  }
  if (++blk_n_ == geo_.noise_block) {
    if (2u * blk_quiet_ >= blk_n_) {
      const float m = blk_sum_ / static_cast<float>(blk_quiet_);
      mean_abs_ += (m - mean_abs_) / kNoiseAvgBlocks;
      update_threshold();
    }
    blk_sum_ = 0.f;
    blk_quiet_ = 0u;
    blk_n_ = 0u;
  }
  return produced;
}

/*
 * Copies ring samples [i_first - 64, i_first + after] into the window, then
 * finds the peak exactly as the reference does:
 *   w = y[i0 : i0 + peak_len]; ip = i0 + argmax(|w|) (first maximum, hence
 *   the strict >); polarity = sign(y[ip]).
 * The ring is 128 long and the window at most 123 (96 kHz), so nothing
 * has been overwritten.
 */
void PulseDetector::capture_window() noexcept {
  PulseWindow &w = window_;
  w.start = i_first_ - kPulsePreHistory;
  w.len = geo_.window_len;
  for (uint32_t j = 0; j < w.len; j++) {
    w.y[j] = ring_[(w.start + j) & (kRing - 1u)];
  }
  w.i_first = i_first_;
  // argmax |y| over y[i_first : i_first + peak_len] (first maximum).
  uint32_t best = kPulsePreHistory;
  float best_abs = -1.f;
  for (uint32_t j = 0; j < geo_.peak_len; j++) {
    const float a = std::fabs(w.y[kPulsePreHistory + j]);
    if (a > best_abs) {
      best_abs = a;
      best = kPulsePreHistory + j;
    }
  }
  w.i_peak = w.start + best;
  w.polarity = static_cast<int8_t>(w.y[best] > 0.f   ? 1
                                   : w.y[best] < 0.f ? -1
                                                     : 0);
  w.peak = best_abs;
}

// ------------------------------------------------------------------- xcorr

/* The 512-point instance points only at constant CMSIS tables, so it is
 * valid wherever the object lives. */
XcorrTimestamper::XcorrTimestamper() noexcept {
  arm_rfft_fast_init_f32(&rfft_, static_cast<uint16_t>(kFftLen));
}

/*
 * m = 1 << ceil(log2(2n)), as in _frac_xcorr: zero-padding to at least 2n
 * makes the circular correlation equal to the linear one (lags -(n-1)..n-1
 * do not wrap). n = 24 gives m = 64 and n = 48 gives m = 128. The
 * 512-point transform then needs a stride 512/m >= 4. The twiddles
 * e^{2πit/n} serve both the forward and inverse n-point DFTs in
 * fractional_shift().
 */
bool XcorrTimestamper::configure(std::size_t seg_len) noexcept {
  if (seg_len < 4u || seg_len > kSegCap) {
    return false;
  }
  // m = 1 << ceil(log2(2n)), as _frac_xcorr().
  std::size_t m = 1u;
  while (m < 2u * seg_len) {
    m <<= 1u;
  }
  if (m > 128u || m < 8u) {
    return false;
  }
  n_ = seg_len;
  m_ = m;
  stride_ = kFftLen / m;
  for (std::size_t t = 0; t < n_; t++) {
    const double a =
        2.0 * kPi * static_cast<double>(t) / static_cast<double>(n_);
    twiddle_[t] = {std::cos(a), std::sin(a)};
  }
  reset();
  return true;
}

/* Forget the template; the next pulse starts building a new one. */
void XcorrTimestamper::reset() noexcept {
  pending_count_ = 0u;
  fixed_ = false;
  tmpl_.fill(0.f);
  tmpl_spec_.fill({0.f, 0.f});
}

/* Store the segment and its absolute start, rebuild from all pending
 * segments, and fix the template on the fourth. */
std::size_t XcorrTimestamper::add_template_pulse(const float *seg,
                                                 double seg_start) noexcept {
  if (fixed_ || pending_count_ >= kTemplatePulses) {
    return pending_count_;
  }
  std::copy(seg, seg + n_, pending_[pending_count_].begin());
  pending_start_[pending_count_] = seg_start;
  pending_lag_[pending_count_] = 0.0;
  pending_count_++;
  rebuild();
  fixed_ = pending_count_ == kTemplatePulses;
  return pending_count_;
}

/*
 * timestamp_pulses(method="xcorr") on the pending segments:
 *   tmpl = segs.mean(axis=0)
 *   repeat 2: lags = [_frac_xcorr(s, tmpl)]
 *             aligned = [irfft(rfft(s)·exp(2πi·f·l), n)]; tmpl = mean
 *   lags = [_frac_xcorr(s, tmpl)]
 * Averaging after alignment sharpens the template: without it, the ±0.5
 * sample spread of the edges would blur the average. The final lags are
 * measured against the final template, and the pending timestamps
 * (segment start + lag) come from them.
 */
void XcorrTimestamper::rebuild() noexcept {
  const auto inv = 1.f / static_cast<float>(pending_count_);
  std::array<float, kSegCap> acc{};
  for (std::size_t p = 0; p < pending_count_; p++) {
    for (std::size_t t = 0; t < n_; t++) {
      acc[t] += pending_[p][t];
    }
  }
  for (std::size_t t = 0; t < n_; t++) {
    acc[t] *= inv;
  }
  std::array<float, kSegCap> shifted{};
  for (int pass = 0; pass < 2; pass++) { // re-align and re-average
    set_template(acc.data());
    std::array<float, kSegCap> next{};
    for (std::size_t p = 0; p < pending_count_; p++) {
      fractional_shift(pending_[p].data(), lag(pending_[p].data()),
                       shifted.data());
      for (std::size_t t = 0; t < n_; t++) {
        next[t] += shifted[t];
      }
    }
    for (std::size_t t = 0; t < n_; t++) {
      acc[t] = next[t] * inv;
    }
  }
  set_template(acc.data());
  for (std::size_t p = 0; p < pending_count_; p++) {
    pending_lag_[p] = lag(pending_[p].data());
  }
}

/* The template spectrum is cached because every lag() call needs it. */
void XcorrTimestamper::set_template(const float *tmpl) noexcept {
  std::copy(tmpl, tmpl + n_, tmpl_.begin());
  spectrum(tmpl_.data(), tmpl_spec_);
}

/*
 * Bin decimation. For x of length n <= m zero-padded to 512:
 *   X512[j·512/m] = Σ x[t]·e^{-2πi·(j·512/m)·t/512} = Σ x[t]·e^{-2πi·j·t/m}
 *                 = Xm[j],
 * so every (512/m)-th bin of the vendored 512-point CMSIS real FFT is the
 * reference's rfft(seg, m) bin. Bins j = 1..m/2-1 sit at packed slots
 * 2b and 2b+1. The m-point Nyquist bin j = m/2 always maps to b = 256, the
 * 512-point Nyquist, which CMSIS packs (real only) in slot 1. Slot 0 holds
 * the DC bin.
 */
void XcorrTimestamper::spectrum(const float *seg,
                                std::array<BinF, kMaxBins> &out) noexcept {
  fft_in_.fill(0.f);
  std::copy(seg, seg + n_, fft_in_.begin());
  arm_rfft_fast_f32(&rfft_, fft_in_.data(), fft_out_.data(), 0u);
  // Packed CMSIS output: [Re X0, Re X256, Re X1, Im X1, ...]. Bin j of the
  // m-point transform of the zero-padded segment is bin j·(512/m) here; its
  // Nyquist bin j = m/2 is therefore always the 512-point Nyquist (bin 256,
  // real, packed in slot 1).
  const std::size_t half = m_ / 2u;
  out[0] = {fft_out_[0], 0.f};
  for (std::size_t j = 1; j < half; j++) {
    const std::size_t b = j * stride_;
    out[j] = {fft_out_[2u * b], fft_out_[2u * b + 1u]};
  }
  out[half] = {fft_out_[1], 0.f};
}

/*
 * numpy.irfft(R, N) at sample q, for R of length m/2+1 zero-padded to
 * N/2+1 bins (N = m·64), without the 1/N factor:
 *   r(q) = Re R0 + 2·Σ_{j=1}^{m/2} Re(R_j·e^{2πi·j·q/N}).
 * Every j <= m/2 < N/2 is an interior bin of the N-point transform, so all
 * of them count twice. The imaginary part of R0 is dropped, as irfft drops
 * it. The phasor e^{2πiq/N} is computed once, and its powers come from
 * repeated multiplication (m/2 <= 64 steps, double).
 */
double XcorrTimestamper::fine_correlation(const std::array<BinD, kMaxBins> &r,
                                          int64_t q) const noexcept {
  const auto big_n = static_cast<int64_t>(m_ * kUpsample);
  q %= big_n;
  if (q < 0) {
    q += big_n;
  }
  const double ang =
      2.0 * kPi * static_cast<double>(q) / static_cast<double>(big_n);
  const double wr = std::cos(ang);
  const double wi = std::sin(ang);
  double zr = wr; // z = w^j, advanced by one complex multiply per bin
  double zi = wi;
  double acc = r[0].re;
  for (std::size_t j = 1; j <= m_ / 2u; j++) {
    acc += 2.0 * (r[j].re * zr - r[j].im * zi); // Re(R_j·z)
    const double nr = zr * wr - zi * wi;
    zi = zr * wi + zi * wr;
    zr = nr;
  }
  return acc;
}

/*
 * _frac_xcorr(seg, tmpl, up=64):
 *  1. R = S·conj(T) on bins 0..m/2 (S from spectrum(), T cached);
 *  2. coarse correlation: R placed in bins 0..m/2 of a 512-point inverse
 *     CMSIS rfft evaluates the same trigonometric polynomial at
 *     q = c·N/512, i.e. on a grid of m/512 sample (1/8 sample for m = 64),
 *     and arm_max_f32 locates its peak;
 *  3. fine search: the reference's argmax over all N = m·64 points of
 *     irfft(R, N). The correlation of these smooth segments has a single
 *     main lobe several samples wide, so its maximum lies within one coarse
 *     step of the coarse maximum. fine_correlation() is evaluated over ±2
 *     coarse steps (65 points at m = 64) rather than all 4096;
 *  4. parabolic refinement through r(q-1), r(q), r(q+1), as in the
 *     reference: d = ½(a - c)/(a - 2b + c), lag = (q + d)/up;
 *  5. lag > m/2 wraps to lag - m (the circular lag of a negative delay).
 * The result matches the reference to ~1e-6 sample on identical input. The
 * spectra are float32, but the correlation is evaluated in double.
 */
double XcorrTimestamper::lag(const float *seg) noexcept {
  std::array<BinF, kMaxBins> s{};
  spectrum(seg, s);
  // R = S·conj(T) in double: (sr + i·si)(tr - i·ti).
  std::array<BinD, kMaxBins> r{};
  const std::size_t bins = m_ / 2u + 1u;
  for (std::size_t j = 0; j < bins; j++) {
    const auto sr = static_cast<double>(s[j].re);
    const auto si = static_cast<double>(s[j].im);
    const auto tr = static_cast<double>(tmpl_spec_[j].re);
    const auto ti = static_cast<double>(tmpl_spec_[j].im);
    r[j] = {sr * tr + si * ti, si * tr - sr * ti};
  }
  // Coarse correlation on a (m/512)-sample grid: CMSIS inverse real FFT of
  // the cross-spectrum placed in bins 0..m/2 of a 512-point spectrum.
  fft_in_.fill(0.f);
  fft_in_[0] = static_cast<float>(r[0].re);
  for (std::size_t j = 1; j < bins; j++) {
    fft_in_[2u * j] = static_cast<float>(r[j].re);
    fft_in_[2u * j + 1u] = static_cast<float>(r[j].im);
  }
  arm_rfft_fast_f32(&rfft_, fft_in_.data(), fft_out_.data(), 1u);
  float coarse_max = 0.f;
  uint32_t coarse_idx = 0u;
  arm_max_f32(fft_out_.data(), static_cast<uint32_t>(kFftLen), &coarse_max,
              &coarse_idx);

  // Refine on the reference's fine grid (N = m·64 points): coarse point c is
  // fine point c·N/512; search two coarse steps either side.
  const auto big_n = static_cast<int64_t>(m_ * kUpsample);
  const int64_t coarse_step = big_n / static_cast<int64_t>(kFftLen);
  const int64_t qc = static_cast<int64_t>(coarse_idx) * coarse_step;
  int64_t best_q = qc;
  double best = fine_correlation(r, qc);
  for (int64_t q = qc - 2 * coarse_step; q <= qc + 2 * coarse_step; q++) {
    const double v = fine_correlation(r, q);
    if (v > best) {
      best = v;
      best_q = q;
    }
  }
  best_q = ((best_q % big_n) + big_n) % big_n;
  const double a = fine_correlation(r, best_q - 1);
  const double c = fine_correlation(r, best_q + 1);
  const double den = a - 2.0 * best + c;
  const double d = (den != 0.0) ? 0.5 * (a - c) / den : 0.0;
  double lag_samples =
      (static_cast<double>(best_q) + d) / static_cast<double>(kUpsample);
  if (lag_samples > 0.5 * static_cast<double>(m_)) {
    lag_samples -= static_cast<double>(m_);
  }
  return lag_samples;
}

/*
 * The reference's re-alignment, irfft(rfft(s)·exp(2πi·f·l), n) with
 * f = rfftfreq(n) = j/n. Delay l becomes a phase ramp e^{2πi·j·l/n}, which
 * shifts the band-limited interpolant of s earlier by l samples. n is 24
 * (or 48), not a power of two, so this is an O(n²) DFT with the twiddle
 * table: 13 x 24 multiplies forward and the same inverse, a few thousand
 * operations per pulse, only while the template is being built.
 */
void XcorrTimestamper::fractional_shift(const float *seg, double lag,
                                        float *out) const noexcept {
  // irfft(rfft(s, n) · exp(2πi·(j/n)·lag), n) by direct n-point DFT (n <=
  // 64), with numpy's irfft conventions: DC and (even n) Nyquist imaginary
  // parts are dropped, interior bins count twice.
  const std::size_t n = n_;
  const std::size_t half = n / 2u;
  std::array<BinD, kSegCap / 2u + 1u> y{};
  for (std::size_t j = 0; j <= half; j++) {
    // X_j = Σ s[t]·e^{-2πi·j·t/n} (conjugated twiddle).
    double xr = 0.0;
    double xi = 0.0;
    for (std::size_t t = 0; t < n; t++) {
      const BinD &w = twiddle_[(j * t) % n];
      xr += static_cast<double>(seg[t]) * w.re;
      xi -= static_cast<double>(seg[t]) * w.im;
    }
    // Y_j = X_j·e^{2πi·j·lag/n}.
    const double a =
        2.0 * kPi * static_cast<double>(j) * lag / static_cast<double>(n);
    const double cr = std::cos(a);
    const double ci = std::sin(a);
    y[j] = {xr * cr - xi * ci, xr * ci + xi * cr};
  }
  const bool even = (n & 1u) == 0u;
  const std::size_t last_interior = even ? half - 1u : half;
  const double inv_n = 1.0 / static_cast<double>(n);
  for (std::size_t t = 0; t < n; t++) {
    double acc = y[0].re;
    for (std::size_t j = 1; j <= last_interior; j++) {
      const BinD &w = twiddle_[(j * t) % n];
      acc += 2.0 * (y[j].re * w.re - y[j].im * w.im); // Re(Y_j·e^{iθ})
    }
    if (even) {
      acc += y[half].re * (((t & 1u) != 0u) ? -1.0 : 1.0);
    }
    out[t] = static_cast<float>(acc * inv_n);
  }
}

// --------------------------------------------------------------------- fit

/* fs only converts sample positions to seconds; the fit runs in seconds. */
void MotorRateFit::configure(uint32_t fs_hz, const Config &cfg) noexcept {
  fs_hz_ = fs_hz;
  cfg_ = cfg;
  reset();
}

void MotorRateFit::reset() noexcept {
  head_ = 0u;
  count_ = 0u;
  jump_count_ = 0u;
  inc_ = Incremental{};
}

/* Ring of kCapacity slots (a power of two, so masking is valid). When full,
 * head advances and the oldest pulse is dropped. Logical index 0 is always
 * the oldest held pulse. */
void MotorRateFit::push(double t_samples, int8_t polarity) noexcept {
  if (count_ == kCapacity) {
    head_ = (head_ + 1u) & (kCapacity - 1u); // drop the oldest
    count_--;
    // Store indices shift down by one. The incremental sums keep the dropped
    // pulse (they are cumulative); only the replay cursor moves.
    if (inc_.next > 0u) {
      inc_.next--;
      inc_.overflowed = true; // the state now outlives the store
    }
  }
  const std::size_t s = slot(count_);
  t_[s] = t_samples;
  pol_[s] = polarity;
  count_++;
}

/* Only used for the first pulses while the xcorr template forms. */
void MotorRateFit::set_time(std::size_t i, double t_samples) noexcept {
  if (i < count_) {
    t_[slot(i)] = t_samples;
    if (i < inc_.next && !inc_.overflowed) {
      inc_.dirty = true; // already ingested with the old time: rebuild
    }
  }
}

/*
 * Rebasing. Absolute positions are double samples (exact to 1e-6 sample up
 * to ~2^33). Subtracting the oldest pulse before dividing by fs keeps every
 * t_k in the fit below ~2048 steps, and at 60 s steps a double then
 * resolves 1e-11 s. This is the same idea as beat_fit's rebased offsets.
 */
double MotorRateFit::rel_s(std::size_t k) const noexcept {
  return (t_[slot(k)] - t_[slot(0u)]) / static_cast<double>(fs_hz_);
}

/* np.median semantics on the shared float scratch. */
double MotorRateFit::scratch_median(std::size_t n) noexcept {
  return median_inplace(scratch_.data(), n);
}

/*
 * motor_rate(), step by step. N = pulses held, all sums double.
 *
 * Step 1: med = median(diff(t)), then the allowed step nearest to it.
 *   Python's min(key=...) keeps the first of equal distances, which the
 *   strict < below also does.
 * Step 2: n_k = round(t_k/step). This is recomputed on demand by step_no()
 *   rather than stored (16 KB saved). It is a pure function of the stored
 *   times, so every pass sees the same value.
 * Step 3: dev_k = diff(t)_k - diff(n)_k·median(diff(t)/max(diff(n), 1)).
 *   The median is taken of offsets from the step, held as float, so ~1e-5 s
 *   values keep ~1e-12 s resolution. Events are dev_k > 20 µs, between
 *   pulses k and k+1, as ev = where(dev > jump_us·1e-6).
 * Step 4: lstsq on t_k = a[s_k] + b·n_k + c·pol_k with no design matrix.
 *   For fixed b, c the least-squares intercept of each segment is
 *   a_s = mean_s(t) - b·mean_s(n) - c·mean_s(pol). Substituting it leaves a
 *   two-parameter problem in the within-segment centred variables
 *   (t', n', p'), whose normal equations are
 *     [Σn'n' Σn'p'] [b]   [Σn't']
 *     [Σn'p' Σp'p'] [c] = [Σp't']
 *   summed over all segments. This is the Frisch–Waugh–Lovell result: it
 *   gives the same b and c as the full lstsq. Segments are contiguous runs,
 *   so two passes over each (means, then centred sums) suffice. The total
 *   is O(N), with no per-segment storage.
 *   Degenerate cases: Σn'n' = 0 (every segment a single pulse) gives no
 *   slope; Σp'p' = 0 or a singular 2x2 (polarity constant within segments)
 *   falls back to c = 0, the component lstsq's minimum-norm solution
 *   would also leave at zero.
 *   The residual RMS uses a third pass. Σt'² - b·Σn't' - c·Σp't' would be
 *   faster, but it cancels catastrophically (µs residuals against
 *   (1000 s)² sums).
 * Long gate (§5 "gate >= 2 x the longest expected period"): with no jump,
 *   kNoJumpLongGate needs (n_last - n_first + 1)·step >= 2·P_max. N pulses
 *   observe N step intervals of the watch (each pulse stands for the step
 *   it ends), while the first-to-last span is only (N-1)·step. A 120 s
 *   gate at 1 Hz holds 120 pulses spanning 119 s; the span test would read
 *   may_include_inhibition forever, but the interval count gives 120 s.
 *   Missed pulses inside the run do not shorten it (step numbers count).
 * Step 5: rate_with from i = ev[0]+1 to j = the first pulse at or after
 *   ev[-1]+1 with pol[j] == pol[i] (same polarity, so the polarity term
 *   cancels), T = (t_j - t_i)/(n_j - n_i). This spans whole inhibition
 *   cycles, so the jumps are averaged in exactly. The period is
 *   median(diff(n[ev+1]))·step.
 * Status per handoff §5; see InhibitionStatus.
 */
std::optional<MotorRate> MotorRateFit::solve() noexcept {
  const std::size_t big_k = count_;
  jump_count_ = 0u;
  if (big_k < 3u) {
    return std::nullopt;
  }

  // 1. Nominal step: the allowed interval nearest the median interval.
  double step = 0.0;
  if (cfg_.step_s && *cfg_.step_s > 0.0) {
    step = *cfg_.step_s;
  } else {
    for (std::size_t k = 0; k + 1u < big_k; k++) {
      scratch_[k] = static_cast<float>(rel_s(k + 1u) - rel_s(k));
    }
    step = snap_to_allowed(scratch_median(big_k - 1u));
  }

  // 2. Step numbers.
  const auto step_no = [&](std::size_t k) {
    return static_cast<double>(std::llround(rel_s(k) / step));
  };

  // 3. Median interval per step, held as a float offset from the step so
  //    the median keeps sub-ns resolution; then the event test.
  for (std::size_t k = 0; k + 1u < big_k; k++) {
    const double iv = rel_s(k + 1u) - rel_s(k);
    const double dn = step_no(k + 1u) - step_no(k);
    scratch_[k] = static_cast<float>(iv / std::max(dn, 1.0) - step);
  }
  const double med_iv = step + scratch_median(big_k - 1u);
  const double thr_s = cfg_.jump_threshold_us * 1e-6;
  const auto dev = [&](std::size_t k) {
    return (rel_s(k + 1u) - rel_s(k)) - (step_no(k + 1u) - step_no(k)) * med_iv;
  };
  const auto is_event = [&](std::size_t k) {
    return k + 1u < big_k && dev(k) > thr_s;
  };

  MotorRate out{};
  out.step_s = step;
  out.pulses = static_cast<uint32_t>(big_k);
  out.span_s = rel_s(big_k - 1u);

  std::size_t first_ev = 0u;
  std::size_t last_ev = 0u;
  double jump_sum = 0.0;
  for (std::size_t k = 0; k + 1u < big_k; k++) {
    if (!is_event(k)) {
      continue;
    }
    const double j_us = dev(k) * 1e6;
    if (out.inhibition_events == 0u) {
      first_ev = k;
      out.jump_min_us = j_us;
      out.jump_max_us = j_us;
    }
    last_ev = k;
    out.jump_min_us = std::min(out.jump_min_us, j_us);
    out.jump_max_us = std::max(out.jump_max_us, j_us);
    jump_sum += j_us;
    if (jump_count_ < kMaxEvents) {
      event_k_[jump_count_] = static_cast<uint16_t>(k);
      jump_us_[jump_count_] = static_cast<float>(j_us);
      jump_count_++;
    }
    out.inhibition_events++;
  }
  if (out.inhibition_events > 0u) {
    out.jump_mean_us = jump_sum / out.inhibition_events;
    out.n_at_8192 = static_cast<uint32_t>(
        std::llround(out.jump_mean_us / kInhibitionQuantumUs));
  }

  // 4. Segmented least squares. Segment s spans [s0, s1]; an event after
  //    pulse k ends a segment at k. Centring t, n and pol inside each
  //    segment eliminates its intercept a[s].
  const auto segment_end = [&](std::size_t s0) {
    std::size_t k = s0;
    while (k + 1u < big_k && !is_event(k)) {
      k++;
    }
    return k;
  };
  struct Means {
    double t, n, p;
  };
  const auto segment_means = [&](std::size_t s0, std::size_t s1) {
    Means m{0.0, 0.0, 0.0};
    for (std::size_t k = s0; k <= s1; k++) {
      m.t += rel_s(k);
      m.n += step_no(k);
      m.p += static_cast<double>(polarity(k));
    }
    const double inv = 1.0 / static_cast<double>(s1 - s0 + 1u);
    return Means{m.t * inv, m.n * inv, m.p * inv};
  };

  double snn = 0.0, snp = 0.0, spp = 0.0, snt = 0.0, spt = 0.0;
  for (std::size_t s0 = 0; s0 < big_k;) {
    const std::size_t s1 = segment_end(s0);
    const Means m = segment_means(s0, s1);
    for (std::size_t k = s0; k <= s1; k++) {
      const double dt = rel_s(k) - m.t;
      const double dn = step_no(k) - m.n;
      const double dp = static_cast<double>(polarity(k)) - m.p;
      snn += dn * dn;
      snp += dn * dp;
      spp += dp * dp;
      snt += dn * dt;
      spt += dp * dt;
    }
    s0 = s1 + 1u;
  }
  if (!(snn > 0.0)) {
    return std::nullopt;
  }
  double b = 0.0;
  double c = 0.0;
  const double det = snn * spp - snp * snp;
  if (spp > 0.0 && det > 1e-12 * snn * spp) {
    b = (snt * spp - snp * spt) / det;
    c = (snn * spt - snp * snt) / det;
  } else {
    // Polarity carries no information independent of the intercepts (e.g.
    // one polarity only): lstsq's minimum-norm answer has c = 0.
    b = snt / snn;
  }

  double rss = 0.0;
  for (std::size_t s0 = 0; s0 < big_k;) {
    const std::size_t s1 = segment_end(s0);
    const Means m = segment_means(s0, s1);
    for (std::size_t k = s0; k <= s1; k++) {
      const double r = (rel_s(k) - m.t) - b * (step_no(k) - m.n) -
                       c * (static_cast<double>(polarity(k)) - m.p);
      rss += r * r;
    }
    s0 = s1 + 1u;
  }
  out.rate_without_inhibition_s_per_day = rate_s_per_day(step, b);
  out.rate_with_inhibition_s_per_day = out.rate_without_inhibition_s_per_day;
  out.residual_rms_us = std::sqrt(rss / static_cast<double>(big_k)) * 1e6;
  out.polarity_offset_us = c * 1e6;

  // 5. Rate over whole inhibition cycles, and the §5 status.
  if (out.inhibition_events >= 2u) {
    const std::size_t i = first_ev + 1u;
    std::optional<std::size_t> j;
    for (std::size_t k = last_ev + 1u; k < big_k; k++) {
      if (polarity(k) == polarity(i)) {
        j = k;
        break;
      }
    }
    std::size_t gaps = 0u;
    for (std::size_t e = 1; e < jump_count_; e++) {
      scratch_[gaps++] = static_cast<float>(step_no(event_k_[e] + 1u) -
                                            step_no(event_k_[e - 1u] + 1u));
    }
    out.inhibition_period_s = scratch_median(gaps) * step;
    if (j && step_no(*j) != step_no(i)) {
      const double t_avg = (rel_s(*j) - rel_s(i)) / (step_no(*j) - step_no(i));
      out.rate_with_inhibition_s_per_day = rate_s_per_day(step, t_avg);
      out.inhibition_cycles_used = out.inhibition_events - 1u;
      out.status = InhibitionStatus::kCorrected;
    } else {
      out.status = InhibitionStatus::kCorrectionPending;
    }
  } else if (out.inhibition_events == 1u) {
    out.status = InhibitionStatus::kNeedOneMoreCycle;
  } else if ((step_no(big_k - 1u) - step_no(0u) + 1.0) * step >=
             2.0 * cfg_.longest_inhibition_period_s) {
    out.status = InhibitionStatus::kNoJumpLongGate;
  } else {
    out.status = InhibitionStatus::kMayIncludeInhibition;
  }
  return out;
}

// ------------------------------------------------------- incremental fit

/*
 * S_xy' = Σxy - Σx·Σy/m: the within-segment centring that eliminates
 * the segment's intercept a[s]. The raw sums are in the segment-local
 * frame: dn counts from the segment's first step (exact integers in
 * double) and u is the departure from a grid at the frame period, the
 * frozen median per-step interval, which is within ~1 µs/step of the
 * true period, so |u| stays at the ms level. The subtractions therefore
 * lose nothing that matters: at 2048 pulses Σdn² ≈ 3e9 is held exactly,
 * and Σu² ≈ 1e-3 s² keeps ~1e-19 s² of resolution against µs²
 * residuals. (Against the nominal step, a 60 s / 40 ppm / 1750-pulse
 * run gives Σu² ≈ 1e4 s² and loses 4 % of Σr²; the randomised test
 * found that case.)
 */
MotorRateFit::Centred MotorRateFit::centre(const Moments &mo) noexcept {
  Centred c{};
  if (mo.m <= 0.0) {
    return c;
  }
  const double inv = 1.0 / mo.m;
  c.nn = mo.snn - mo.sn * mo.sn * inv;
  c.np = mo.snp - mo.sn * mo.sp * inv;
  c.pp = mo.spp - mo.sp * mo.sp * inv;
  c.nu = mo.snu - mo.sn * mo.su * inv;
  c.pu = mo.spu - mo.sp * mo.su * inv;
  c.uu = mo.suu - mo.su * mo.su * inv;
  return c;
}

/*
 * The batch step rule on the first {@code count} raw intervals of the
 * store: snap(median(diff(t))). With count = all intervals this is exactly
 * motor_rate() step 1. Uses the shared scratch (never concurrently with
 * solve()).
 */
double MotorRateFit::snap_store_step(std::size_t count) noexcept {
  if (cfg_.step_s && *cfg_.step_s > 0.0) {
    return *cfg_.step_s;
  }
  for (std::size_t k = 0; k < count; k++) {
    scratch_[k] = static_cast<float>(rel_s(k + 1u) - rel_s(k));
  }
  return snap_to_allowed(scratch_median(count));
}

/*
 * Fresh state on the current store: the time origin is the oldest stored
 * pulse (as rel_s() uses it, so n_k is the same number the batch computes),
 * and the frozen median is the batch's median(diff(t)/max(diff(n), 1)) over
 * the first min(kEventWindow, N-1) intervals. While the store holds at most
 * kEventWindow+1 pulses, that is the batch's own median, which is why the
 * warm-up reproduces solve() exactly. The last complete result is kept so
 * that callers see a stable value during a spread-out rebuild.
 */
void MotorRateFit::inc_restart(double step) noexcept {
  std::optional<MotorRate> keep = inc_.cached;
  inc_ = Incremental{};
  inc_.cached = keep;
  inc_.dirty = false;
  inc_.step = step;
  inc_.t_ref = t_[slot(0u)];
  const std::size_t m = std::min(kEventWindow, count_ - 1u);
  for (std::size_t k = 0; k < m; k++) {
    const double iv = rel_s(k + 1u) - rel_s(k);
    const double dn = static_cast<double>(std::llround(rel_s(k + 1u) / step)) -
                      static_cast<double>(std::llround(rel_s(k) / step));
    scratch_[k] = static_cast<float>(iv / std::max(dn, 1.0) - step);
  }
  inc_.frozen_med = step + (m > 0u ? scratch_median(m) : 0.0);
  inc_.frame = inc_.frozen_med;
  inc_.frozen_count = m;
}

/*
 * One event between the previous pulse and this one (seg[e+1:] += 1 in the
 * reference):
 *  1. the open segment's centred sums are final, so they are folded into
 *     `closed` and the segment is forgotten (O(1) memory per segment);
 *  2. this pulse opens the next segment and becomes its local origin;
 *  3. jump statistics, the kept-event ring, and the first-after-first-event
 *     mark (pulse i of the rate with inhibition) are updated;
 *  4. the period, median(diff(n[ev+1]))·step over the kept events, is
 *     recomputed here and only here. Events are rare (one per 10–960
 *     steps), so the O(kMaxEvents) median never lands on an ordinary pulse.
 */
void MotorRateFit::inc_event(double t, double n, int8_t pol,
                             double jump_s) noexcept {
  const Centred c = centre(inc_.open);
  inc_.closed.nn += c.nn;
  inc_.closed.np += c.np;
  inc_.closed.pp += c.pp;
  inc_.closed.nu += c.nu;
  inc_.closed.pu += c.pu;
  inc_.closed.uu += c.uu;
  inc_.open = Moments{};
  inc_.open_t0 = t;
  inc_.open_n0 = n;
  inc_.open_first = {};

  const double j_us = jump_s * 1e6;
  if (inc_.events == 0u) {
    inc_.jump_min = j_us;
    inc_.jump_max = j_us;
    inc_.first_after = Mark{true, t, n, pol};
  }
  inc_.events++;
  inc_.jump_sum += j_us;
  inc_.jump_min = std::min(inc_.jump_min, j_us);
  inc_.jump_max = std::max(inc_.jump_max, j_us);
  inc_.ev_n[inc_.ev_head] = static_cast<int32_t>(n);
  inc_.ev_jump[inc_.ev_head] = static_cast<float>(j_us);
  inc_.ev_head = (inc_.ev_head + 1u) % kMaxEvents;
  inc_.ev_stored = std::min(inc_.ev_stored + 1u, kMaxEvents);
  if (inc_.ev_stored >= 2u) {
    const std::size_t first =
        (inc_.ev_head + kMaxEvents - inc_.ev_stored) % kMaxEvents;
    std::size_t gaps = 0u;
    for (std::size_t e = 1; e < inc_.ev_stored; e++) {
      const int32_t a = inc_.ev_n[(first + e - 1u) % kMaxEvents];
      const int32_t b = inc_.ev_n[(first + e) % kMaxEvents];
      scratch_[gaps++] = static_cast<float>(b - a);
    }
    inc_.period_s = scratch_median(gaps) * inc_.step;
  }
}

/*
 * One pulse, in arrival order:
 *  1. t (s from the origin), n = round(t/step), as motor_rate() steps 1-2;
 *  2. for every pulse but the first: iv, dn and the per-step interval
 *     iv/max(dn, 1) enter the 31-slot rings. The event median is the frozen
 *     one for the first kEventWindow intervals, and otherwise the median of
 *     the ring, which then holds exactly the last kEventWindow intervals
 *     including this one. dev = iv - dn·median; dev > threshold is an event
 *     (motor_rate() step 3, with a windowed median);
 *  3. the pulse joins the open segment's raw moments in the local frame
 *     dn = n - n0, u = t - t0 - step·dn, and is remembered if it is the
 *     segment's first pulse of its polarity (candidate j of step 5).
 */
void MotorRateFit::inc_ingest(std::size_t k) noexcept {
  const double t = (t_[slot(k)] - inc_.t_ref) / static_cast<double>(fs_hz_);
  const double n = static_cast<double>(std::llround(t / inc_.step));
  const int8_t pol = pol_[slot(k)];
  if (!inc_.have_prev) {
    inc_.open_t0 = t;
    inc_.open_n0 = n;
  } else {
    const double iv = t - inc_.prev_t;
    const double dn = n - inc_.prev_n;
    inc_.win_ratio[inc_.win_head] =
        static_cast<float>(iv / std::max(dn, 1.0) - inc_.step);
    inc_.win_iv[inc_.win_head] = static_cast<float>(iv);
    inc_.win_head = (inc_.win_head + 1u) % kEventWindow;
    inc_.win_n = std::min(inc_.win_n + 1u, kEventWindow);
    double med = inc_.frozen_med;
    if (inc_.intervals >= kEventWindow) {
      std::array<float, kEventWindow> tmp = inc_.win_ratio;
      med = inc_.step + median_inplace(tmp.data(), inc_.win_n);
    }
    inc_.intervals++;
    const double dev = iv - dn * med;
    if (dev > cfg_.jump_threshold_us * 1e-6) {
      inc_event(t, n, pol, dev);
    }
  }
  const double dn = n - inc_.open_n0;
  const double u = t - inc_.open_t0 - inc_.frame * dn;
  const auto p = static_cast<double>(pol);
  Moments &mo = inc_.open;
  mo.m += 1.0;
  mo.sn += dn;
  mo.sp += p;
  mo.su += u;
  mo.snn += dn * dn;
  mo.snp += dn * p;
  mo.spp += p * p;
  mo.snu += dn * u;
  mo.spu += p * u;
  mo.suu += u * u;
  Mark &first = inc_.open_first[pol > 0 ? 1u : 0u];
  if (!first.valid) {
    first = Mark{true, t, n, pol};
  }
  inc_.have_prev = true;
  inc_.prev_t = t;
  inc_.prev_n = n;
  inc_.t_last = t;
  inc_.pulses++;
}

/*
 * The batch solve's steps 4-5 from the running sums:
 *  - S = closed + centre(open); then the same 2x2 normal equations in
 *    (b', c) with u in place of t, where b = frame + b' (u removes frame·dn
 *    exactly), and the same c = 0 fallback;
 *  - Σr² = S_uu - b'·S_nu - c·S_pu, the least-squares residual identity
 *    (r ⟂ the fitted columns). Clamped at 0 against rounding;
 *  - with two or more events: i = first_after, and j = the open segment's
 *    first pulse with pol_i. The open segment starts at the last event, so
 *    this is the reference's j;
 *  - status as the batch.
 */
std::optional<MotorRate> MotorRateFit::inc_result() const noexcept {
  if (inc_.pulses < 3u) {
    return std::nullopt;
  }
  const Centred o = centre(inc_.open);
  const double snn = inc_.closed.nn + o.nn;
  const double snp = inc_.closed.np + o.np;
  const double spp = inc_.closed.pp + o.pp;
  const double snu = inc_.closed.nu + o.nu;
  const double spu = inc_.closed.pu + o.pu;
  const double suu = inc_.closed.uu + o.uu;
  if (!(snn > 0.0)) {
    return std::nullopt;
  }
  double bp = 0.0;
  double c = 0.0;
  const double det = snn * spp - snp * snp;
  if (spp > 0.0 && det > 1e-12 * snn * spp) {
    bp = (snu * spp - snp * spu) / det;
    c = (snn * spu - snp * snu) / det;
  } else {
    bp = snu / snn;
  }
  const double rss = std::max(0.0, suu - bp * snu - c * spu);
  const double step = inc_.step;
  const double b = inc_.frame + bp;

  MotorRate out{};
  out.step_s = step;
  out.pulses = inc_.pulses;
  out.span_s = inc_.t_last;
  out.rate_without_inhibition_s_per_day = rate_s_per_day(step, b);
  out.rate_with_inhibition_s_per_day = out.rate_without_inhibition_s_per_day;
  out.residual_rms_us = std::sqrt(rss / static_cast<double>(inc_.pulses)) * 1e6;
  out.polarity_offset_us = c * 1e6;
  out.inhibition_events = inc_.events;
  if (inc_.events > 0u) {
    out.jump_mean_us = inc_.jump_sum / inc_.events;
    out.jump_min_us = inc_.jump_min;
    out.jump_max_us = inc_.jump_max;
    out.n_at_8192 = static_cast<uint32_t>(
        std::llround(out.jump_mean_us / kInhibitionQuantumUs));
  }
  if (inc_.events >= 2u) {
    out.inhibition_period_s = inc_.period_s;
    const Mark &i = inc_.first_after;
    const Mark &j = inc_.open_first[i.pol > 0 ? 1u : 0u];
    if (j.valid && j.n != i.n) {
      out.rate_with_inhibition_s_per_day =
          rate_s_per_day(step, (j.t - i.t) / (j.n - i.n));
      out.inhibition_cycles_used = inc_.events - 1u;
      out.status = InhibitionStatus::kCorrected;
    } else {
      out.status = InhibitionStatus::kCorrectionPending;
    }
  } else if (inc_.events == 1u) {
    out.status = InhibitionStatus::kNeedOneMoreCycle;
  } else if ((std::round(inc_.t_last / step) + 1.0) * step >=
             2.0 * cfg_.longest_inhibition_period_s) {
    out.status = InhibitionStatus::kNoJumpLongGate;
  } else {
    out.status = InhibitionStatus::kMayIncludeInhibition;
  }
  return out;
}

/*
 * Scheduling:
 *  - fewer than 3 pulses: no rate, and the state is marked for a restart;
 *  - warm-up (N <= kEventWindow+1): each new pulse changes the batch rule's
 *    step and median, so the state is rebuilt from the store
 *    synchronously: at most 32 ingests that use only the frozen median;
 *  - otherwise: restart if dirty (step change or re-timed pulse), then
 *    ingest at most kReplayPulsesPerCall pulses. In steady state that is
 *    the one new pulse. Once caught up, the step is re-checked against
 *    the snap of the window's median raw interval; a mismatch schedules a
 *    rebuild instead of publishing a result built on the wrong grid.
 * The cached result changes only when the state is complete and current.
 */
std::optional<MotorRate> MotorRateFit::solve_incremental() noexcept {
  if (count_ < 3u) {
    inc_.dirty = true;
    inc_.next = 0u;
    inc_.cached.reset();
    return std::nullopt;
  }
  const bool fixed_step = cfg_.step_s && *cfg_.step_s > 0.0;
  if (count_ <= kEventWindow + 1u) {
    if (inc_.dirty || inc_.next < count_) {
      inc_restart(snap_store_step(count_ - 1u));
      while (inc_.next < count_) {
        inc_ingest(inc_.next++);
      }
    }
  } else {
    // A warm-up state built on fewer than kEventWindow intervals (the
    // caller skipped calls while the store grew) is not the replay
    // definition: restart it with the batch rule on the first window.
    if (!inc_.dirty && inc_.frozen_count < kEventWindow) {
      inc_.dirty = true;
      inc_.new_step = 0.0;
      inc_.step = 0.0;
    }
    if (inc_.dirty) {
      const double step = inc_.new_step > 0.0 ? inc_.new_step
                          : inc_.step > 0.0   ? inc_.step
                                              : snap_store_step(kEventWindow);
      inc_restart(step);
    }
    for (std::size_t b = 0; b < kReplayPulsesPerCall && inc_.next < count_;
         b++) {
      inc_ingest(inc_.next++);
    }
    if (inc_.next == count_ && !fixed_step && !inc_.overflowed &&
        inc_.win_n == kEventWindow) {
      std::array<float, kEventWindow> tmp = inc_.win_iv;
      const double snapped =
          snap_to_allowed(median_inplace(tmp.data(), kEventWindow));
      if (snapped != inc_.step) {
        inc_.dirty = true;
        inc_.new_step = snapped;
      }
    }
  }
  if (!rebuilding()) {
    inc_.cached = inc_result();
  }
  return inc_.cached;
}

// ---------------------------------------------------------------- pipeline

/* Detector first (it validates fs and supplies pre/post for the xcorr
 * segment), then the correlator, the high-pass and the fit. */
bool MotorTiming::configure(uint32_t fs_hz, const Config &cfg) noexcept {
  if (!det_.configure(fs_hz, cfg.detector)) {
    return false;
  }
  const PulseGeometry &g = det_.geometry();
  if (!xcorr_.configure(g.pre + g.post) ||
      !hpf_.configure(static_cast<double>(fs_hz), kMotorHighpassHz)) {
    return false;
  }
  fs_hz_ = fs_hz;
  cfg_ = cfg;
  fit_.configure(fs_hz, cfg.fit);
  reset();
  return true;
}

/* Everything restarts; the absolute index origin goes to the detector,
 * which numbers samples and therefore timestamps. */
void MotorTiming::reset(uint64_t first_index) noexcept {
  hpf_.reset();
  det_.reset(first_index);
  xcorr_.reset();
  fit_.reset();
  recent_.fill(PulseInfo{});
  pulses_ = 0u;
  extras_ = 0u;
  clipped_pulses_ = 0u;
  clipped_ = false;
}

/*
 * Chunk loop (kChunk = 256): convert codes (and note any rail hit), filter
 * in place, then let the detector consume the chunk. It stops after each
 * event, the event is handled, and detection resumes where it left off.
 * The high-pass runs on the whole chunk first; this is safe because the
 * detector only reads the filtered samples and the raw codes.
 */
void MotorTiming::process(const int32_t *codes, std::size_t n) noexcept {
  if (fs_hz_ == 0u) {
    return;
  }
  while (n > 0u) {
    const std::size_t len = std::min(n, kChunk);
    clipped_ = codes_to_float(codes, buf_.data(), len) || clipped_;
    hpf_.process(buf_.data(), buf_.data(), len);
    std::size_t done = 0u;
    while (done < len) {
      PulseDetector::Event ev{};
      done += det_.process(buf_.data() + done, codes + done, len - done, ev);
      if (ev.kind == PulseDetector::EventKind::kWindowReady) {
        on_window(*ev.window);
      } else if (ev.kind == PulseDetector::EventKind::kGroupClosed) {
        on_group_closed(ev);
      }
    }
    codes += len;
    n -= len;
  }
}

/*
 * One new pulse. The threshold timestamp is always computed (it is cheap
 * and the CLI shows it). With xcorr, the segment pol·y[ip-pre, ip+post) is
 * cut from the window:
 *  - while the template is forming, the pulse is pushed with a placeholder
 *    time and every pending pulse is re-timed from the rebuilt template.
 *    They are the first pulses since reset (store indices 0..3), so the
 *    fit is updated with set_time();
 *  - once the template is fixed, the pulse is timed directly.
 * The xcorr time is segment start + lag. That lies a constant offset (the
 * template's own edge position) before the edge, and the offset cancels in
 * every rate.
 */
void MotorTiming::on_window(const PulseWindow &w) noexcept {
  PulseInfo info{};
  info.sequence = pulses_;
  info.i_first = w.i_first;
  info.i_peak = w.i_peak;
  info.peak = w.peak;
  info.polarity = w.polarity;
  info.t_samples = threshold_timestamp(w);

  if (cfg_.method == TimestampMethod::kXcorr) {
    const PulseGeometry &g = det_.geometry();
    std::array<float, XcorrTimestamper::kSegCap> seg{};
    const auto off = static_cast<std::size_t>(w.i_peak - g.pre - w.start);
    const auto pol = static_cast<float>(w.polarity);
    for (std::size_t j = 0; j < g.pre + g.post; j++) {
      seg[j] = pol * w.y[off + j];
    }
    const auto seg_start = static_cast<double>(w.i_peak - g.pre);
    if (!xcorr_.fixed()) {
      // Still building the template: every pending pulse is re-timed. They
      // are the first pulses since reset, so their store index is their
      // sequence number.
      fit_.push(seg_start, w.polarity);
      const std::size_t m = xcorr_.add_template_pulse(seg.data(), seg_start);
      for (std::size_t i = 0; i < m; i++) {
        const double t = xcorr_.pending_timestamp(i);
        fit_.set_time(i, t);
        if (PulseInfo *p = find_recent(static_cast<uint32_t>(i))) {
          p->t_samples = t;
        }
      }
      info.t_samples = xcorr_.pending_timestamp(m - 1u);
    } else {
      info.t_samples = xcorr_.timestamp(seg.data(), seg_start);
      fit_.push(info.t_samples, w.polarity);
    }
  } else {
    fit_.push(info.t_samples, w.polarity);
  }
  recent_[pulses_ % kRecentPulses] = info;
  pulses_++;
}

/* A close always refers to the most recent pulse: groups never overlap. */
void MotorTiming::on_group_closed(const PulseDetector::Event &ev) noexcept {
  extras_ += ev.extras;
  if (ev.clipped) {
    clipped_pulses_++;
  }
  if (pulses_ == 0u) {
    return;
  }
  if (PulseInfo *p = find_recent(pulses_ - 1u)) {
    p->i_last = ev.i_last;
    p->extras = ev.extras;
    p->clipped = ev.clipped;
    p->closed = true;
  }
}

/* Entry seq lives at recent_[seq % 16] while it is among the last 16. */
PulseInfo *MotorTiming::find_recent(uint32_t seq) noexcept {
  if (seq >= pulses_ || pulses_ - seq > kRecentPulses) {
    return nullptr;
  }
  return &recent_[seq % kRecentPulses];
}

/* i = 0 is the newest entry. */
std::optional<PulseInfo> MotorTiming::recent(std::size_t i) const noexcept {
  if (i >= pulses_ || i >= kRecentPulses) {
    return std::nullopt;
  }
  return recent_[(pulses_ - 1u - i) % kRecentPulses];
}

/* The fit plus the detector-level counters the fit cannot know. */
std::optional<MotorRate> MotorTiming::solve_incremental() noexcept {
  auto r = fit_.solve_incremental();
  if (r) {
    r->extra_pulses = extras_;
    r->clipped_pulses = clipped_pulses_;
    r->clipped = clipped_;
  }
  return r;
}

std::optional<MotorRate> MotorTiming::solve() noexcept {
  auto r = fit_.solve();
  if (r) {
    r->extra_pulses = extras_;
    r->clipped_pulses = clipped_pulses_;
    r->clipped = clipped_;
  }
  return r;
}

} // namespace quartz
