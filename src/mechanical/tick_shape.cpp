/**
 * @file tick_shape.cpp
 * @brief Envelope front end and beat-template drop timing (stage B).
 *
 * @see mechanical/tick_shape.h for the method, the interface to
 *      {@code beat_analysis} and the ring-length derivation.
 */

#include "mechanical/tick_shape.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#ifdef TG_HOST_PROFILE_TICK_SHAPE
#include "tick_shape_profile.h"
#define TG_PROFILE_STAGE(name)                                                 \
  tick_shape_profile::Timer timer(tick_shape_profile::name)
#else
#define TG_PROFILE_STAGE(name) ((void)0)
#endif

namespace {

/* ---- Geometry ----------------------------------------------------------- */

/** Highest envelope rate (Hz); the decimation factor is chosen to respect it.
 */
constexpr uint32_t kEnvHzMax = 24500u;
/** Template span before the drop landmark anchor (ms), as in OTG. */
constexpr uint32_t kPreMs = 20u;
/** Template span after the anchor (ms), as in OTG. */
constexpr uint32_t kPostMs = 8u;
/** Correlation search either side of the anchor (ms). */
constexpr uint32_t kSearchMs = 2u;
/** Drop estimate region: before the detector's drop estimate (ms). */
constexpr uint32_t kRegionBeforeMs = 2u;
/**
 * Drop estimate region: after it (ms). The detector may name the unlock as
 * the strongest pulse; the drop then follows by one lift time (up to about
 * 14 ms at 14,400 A/h and 200 degrees).
 */
constexpr uint32_t kRegionAfterMs = 16u;
/** Window of the strength mean that ranks drop candidates (ms). */
constexpr float kStrengthMs = 0.5f;
/** Baseline of a template: mean of its first this many ms (quiet pre-roll). */
constexpr uint32_t kBaselineMs = 4u;
/** The half-height crossing is searched at most this far before the peak (ms).
 */
constexpr uint32_t kEdgeMs = 2u;
/** An unlock onset must stay above its threshold this long (ms). */
constexpr float kSteadyMs = 0.2f;
/** Smoothing used to find the unlock peak of an averaged shape (ms). */
constexpr float kSmoothMs = 0.4f;
/** The unlock threshold is at least this share of the drop height. */
constexpr float kOnsetShare = 0.02f;
/** The unlock threshold is at least this many times the background noise. */
constexpr float kOnsetNoiseMult = 4.f;
/**
 * Scale from the mean absolute deviation of Gaussian noise to its standard
 * deviation: {@code sqrt(pi / 2)}.
 */
constexpr float kMadToSigma = 1.2533141f;

/** Block time the ring is advanced by before an event is read (ms). */
constexpr uint32_t kBlockMs = 5u;
/** Default {@code tail_ms}: silence that closes an event (ms). */
constexpr uint32_t kTailMs = 25u;
/** Allowance for the loud decay between the drop and the last loud sample. */
constexpr uint32_t kDecayMs = 12u;
/** Longest event window before the detector closes it by force (ms). */
constexpr uint32_t kMaxEventMs = 80u;
/** Default {@code preroll_ms}: the window starts this long before the onset. */
constexpr uint32_t kPrerollMs = 25u;
/**
 * How far the drop can lie behind the close of its event (ms), the larger of
 * two cases. Quiet close: the {@link kTailMs} of silence plus the loud decay.
 * Forced close (noisy recording): the window is {@link kMaxEventMs} long and
 * the drop is not before the onset, which is {@link kPrerollMs} into it.
 */
constexpr uint32_t kDropAgeMs = kTailMs + kDecayMs > kMaxEventMs - kPrerollMs
                                    ? kTailMs + kDecayMs
                                    : kMaxEventMs - kPrerollMs;
/** Ring length (ms): the sum derived in tick_shape.h. */
constexpr uint32_t kRingMs = kBlockMs + kDropAgeMs + kPreMs + kSearchMs;

/**
 * Samples covering {@code ms} at {@code hz}, rounded up.
 *
 * @param ms Duration (ms).
 * @param hz Rate (Hz).
 */
constexpr uint32_t ms_to_samples(uint32_t ms, uint32_t hz) {
  return (ms * hz + 999u) / 1000u;
}

/** Ring capacity (envelope samples) at the highest envelope rate. */
constexpr uint32_t kRingCap = ms_to_samples(kRingMs, kEnvHzMax);
/** Template capacity (envelope samples) at the highest envelope rate. */
constexpr uint32_t kTplCap = ms_to_samples(kPreMs + kPostMs, kEnvHzMax);
/** Largest search half-width (lags) at the highest envelope rate. */
constexpr uint32_t kSearchCap = ms_to_samples(kSearchMs, kEnvHzMax);
/** Scratch window: template plus the search either side. */
constexpr uint32_t kWinCap = kTplCap + 2u * kSearchCap;

/* The 48 kHz worst case: 240-sample blocks, decimation 2, 24 kHz envelope. */
static_assert(DSP_MAX_BLOCK * 1000u / 48000u == kBlockMs,
              "kBlockMs is one DSP block at 48 kHz");
static_assert(kRingMs == 82u, "5 + 55 + 20 + 2 ms");
static_assert(ms_to_samples(kRingMs, 48000u / 2u) == 1968u, "82 ms at 24 kHz");
static_assert(ms_to_samples(kRingMs, 48000u / 2u) <= kRingCap,
              "the ring holds the 48 kHz worst case");
static_assert(ms_to_samples(kRegionBeforeMs + kRegionAfterMs, kEnvHzMax) +
                      2u * ms_to_samples(1u, kEnvHzMax) + 1u <=
                  kWinCap,
              "the drop-estimate region fits the scratch window");

/* ---- Front end ---------------------------------------------------------- */

/** Corner of the high-pass (Hz). */
constexpr float kHighPassHz = 1500.f;
/** Pole Q of the two sections of a 4th-order Butterworth. */
constexpr float kButterQ[2] = {0.5411961f, 1.3065630f};
/** Boxcar length (ms). */
constexpr float kBoxcarMs = 0.2f;
/** Largest boxcar (source samples): 0.2 ms up to 128 kHz. */
constexpr uint32_t kBoxcarMax = 32u;
/**
 * Envelope counts per unit of mean rectified full-scale signal. An
 * {@code int16} then holds up to 2.0 before it saturates; a tick of 0.003 FS
 * still spans about 50 counts.
 */
constexpr float kEnvScale = 16384.f;
/** Lowest source rate (Hz) at which the 1.5 kHz high-pass is well defined. */
constexpr uint32_t kMinSourceHz = 6000u;

/** One biquad section, transposed direct form II. */
struct Biquad {
  float b0 = 0.f, b1 = 0.f, b2 = 0.f, a1 = 0.f, a2 = 0.f;
  float s1 = 0.f, s2 = 0.f;

  /**
   * Filters one sample.
   *
   * @param x Input.
   * @return Output.
   */
  float step(float x) {
    const float y = b0 * x + s1;
    s1 = b1 * x - a1 * y + s2;
    s2 = b2 * x - a2 * y;
    return y;
  }
};

/** Resolved geometry for the configured source rate. */
struct Geometry {
  bool valid = false;
  uint32_t source_hz = 0;
  uint32_t decim = 1;       /**< Source samples per envelope sample. */
  uint32_t env_hz = 0;      /**< Envelope rate (Hz). */
  uint32_t boxcar = 1;      /**< Boxcar length (source samples). */
  double boxcar_delay = 0;  /**< Centre of the boxcar behind its last sample. */
  uint32_t n_pre = 0;       /**< Template samples before the anchor. */
  uint32_t n_tpl = 0;       /**< Template length. */
  uint32_t search = 0;      /**< Correlation search half-width (lags). */
  uint32_t reg_before = 0;  /**< Drop-estimate region before the estimate. */
  uint32_t reg_after = 0;   /**< Drop-estimate region after the estimate. */
  uint32_t half = 0;        /**< Half-width of the centred strength mean. */
  uint32_t baseline = 0;    /**< Template baseline length. */
  uint32_t edge = 0;        /**< Half-height search length. */
  uint32_t steady = 0;      /**< Samples a rise must hold to be an onset. */
  uint32_t smooth_half = 0; /**< Half-width of the 0.4 ms smoothing. */
};

Geometry g_geo;
std::array<Biquad, 2> g_hp;
bool g_enabled = false;

std::array<float, kBoxcarMax> g_box{};
float g_box_sum = 0.f;
uint32_t g_box_pos = 0;
uint32_t g_phase = 0;

std::array<int16_t, kRingCap> g_ring{};
/** Next ring slot to write; equals {@code g_total % kRingCap}. */
uint32_t g_head = 0;
/** Envelope samples written since the front end was cleared. */
uint64_t g_total = 0;
/** Source index of envelope sample 0. */
uint64_t g_src0 = 0;
/** Source index the next block must start at for the front end to continue. */
uint64_t g_next_src = 0;

/** Clears filters, boxcar and ring (not the templates). */
void clear_front_end() {
  for (Biquad &b : g_hp) {
    b.s1 = 0.f;
    b.s2 = 0.f;
  }
  g_box.fill(0.f);
  g_box_sum = 0.f;
  g_box_pos = 0;
  g_phase = 0;
  g_head = 0;
  g_total = 0;
  g_src0 = 0;
}

/**
 * Sets the biquads to a 4th-order Butterworth high-pass at
 * {@link kHighPassHz} (RBJ cookbook sections, bilinear transform).
 *
 * @param fs Source rate (Hz).
 */
void design_high_pass(float fs) {
  constexpr float kTwoPi = 6.28318530718f;
  const float w0 = kTwoPi * kHighPassHz / fs;
  const float c = std::cos(w0);
  for (size_t i = 0; i < g_hp.size(); i++) {
    const float alpha = std::sin(w0) / (2.f * kButterQ[i]);
    const float a0 = 1.f + alpha;
    Biquad &b = g_hp[i];
    b.b0 = 0.5f * (1.f + c) / a0;
    b.b1 = -(1.f + c) / a0;
    b.b2 = b.b0;
    b.a1 = -2.f * c / a0;
    b.a2 = (1.f - alpha) / a0;
  }
}

/* ---- Templates ---------------------------------------------------------- */

/** Divisor of the exponential template update (weight 1/32). */
constexpr int32_t kEmaDivisor = 32;
/** Beats a template needs before side detection may consider it. */
constexpr uint32_t kAutoMinBeats = 4u;
/**
 * Consecutive misfits while learning after which the template restarts from
 * the current beat, so a bad first beat cannot block learning for good.
 */
constexpr uint32_t kLearnStallRejects = 4u;
/** The drop peak is the last peak reaching this share of the strongest. */
constexpr int32_t kPeakShareNum = 7;
constexpr int32_t kPeakShareDen = 10;

/** Mean beat shape of one side and what is derived from it. */
struct Template {
  std::array<int16_t, kTplCap> v{};
  uint32_t beats = 0;   /**< Beats folded in since the last restart. */
  uint32_t rejects = 0; /**< Consecutive misfits while learning. */
  bool ready = false;   /**< Learned and the landmark found. */
  int64_t sum = 0;      /**< Sum of {@code v} over the template length. */
  int64_t sum_sq = 0;   /**< Sum of squares of {@code v}. */
  float landmark = 0.f; /**< Drop half-height crossing (template index). */
};

std::array<Template, 2> g_tpl;
/** Window gathered around the current event (ring copy). */
std::array<int16_t, kWinCap> g_win{};
tick_shape_stats_t g_stats;

/** Outcome of sliding one template over the window. */
struct Match {
  bool ok = false;      /**< A correlation was computed. */
  int32_t lag = 0;      /**< Best whole-sample lag (-search..search). */
  float lag_frac = 0.f; /**< Sub-sample refinement, -0.5..0.5. */
  float quality = 0.f;  /**< Peak normalised correlation. */
};

/**
 * Copies envelope samples {@code [first, first + len)} out of the ring.
 *
 * @param first Absolute envelope index of the first sample.
 * @param len Sample count.
 * @param dst Destination, {@code len} entries.
 * @return {@code false} if the ring does not hold the whole range.
 */
bool gather(int64_t first, uint32_t len, int16_t *dst) {
  const auto total = static_cast<int64_t>(g_total);
  const int64_t oldest =
      total > static_cast<int64_t>(kRingCap) ? total - kRingCap : 0;
  if (first < oldest || first + static_cast<int64_t>(len) > total) {
    return false;
  }
  auto slot = static_cast<uint32_t>(static_cast<uint64_t>(first) % kRingCap);
  for (uint32_t i = 0; i < len; i++) {
    dst[i] = g_ring[slot];
    slot = (slot + 1u == kRingCap) ? 0u : slot + 1u;
  }
  return true;
}

/**
 * Sub-sample vertex of three equally spaced correlation values around a
 * maximum.
 *
 * @return Offset from the middle point, clamped to -0.5..0.5.
 */
float parabolic_offset(float ym, float y0, float yp) {
  const float denom = ym - 2.f * y0 + yp;
  if (denom > -1e-12f) {
    return 0.f;
  }
  return std::min(0.5f, std::max(-0.5f, 0.5f * (ym - yp) / denom));
}

/**
 * Normalised cross-correlation (Pearson) from running sums.
 *
 * @param dot Sum of template times window.
 * @param sw Sum of the window.
 * @param sww Sum of squares of the window.
 * @param st Sum of the template.
 * @param stt Sum of squares of the template.
 * @param m Element count.
 * @return Correlation clamped to 0..1 (0 when either side is flat).
 */
float ncc(int64_t dot, int64_t sw, int64_t sww, int64_t st, int64_t stt,
          int64_t m) {
  const int64_t num = dot - st * sw / m;
  const int64_t var_t = stt - st * st / m;
  const int64_t var_w = sww - sw * sw / m;
  if (num <= 0 || var_t <= 0 || var_w <= 0) {
    return 0.f;
  }
  const float r =
      static_cast<float>(num) /
      std::sqrt(static_cast<float>(var_t) * static_cast<float>(var_w));
  return std::min(1.f, r);
}

/**
 * Correlation of the template with the window slice starting at
 * {@code s}, every sample.
 */
float ncc_full(const Template &t, uint32_t s) {
  const int16_t *w = &g_win[s];
  int64_t dot = 0, sw = 0, sww = 0;
  for (uint32_t i = 0; i < g_geo.n_tpl; i++) {
    const int32_t wi = w[i];
    dot += static_cast<int32_t>(t.v[i]) * wi;
    sw += wi;
    sww += wi * wi;
  }
  return ncc(dot, sw, sww, t.sum, t.sum_sq, g_geo.n_tpl);
}

/**
 * Slides {@code t} over the window: a coarse pass on every second lag and
 * every second sample (the envelope is smooth to about 5 kHz, so this finds
 * the peak lobe), then every lag within 2 of the coarse best at full
 * resolution, then a parabolic vertex.
 *
 * Cost: 49 lags of 336 products (window sums slide) plus 5 lags of 672.
 */
Match match_template(const Template &t) {
  TG_PROFILE_STAGE(match);
  Match m;
  const auto search = static_cast<int32_t>(g_geo.search);
  const uint32_t len = g_geo.n_tpl / 2u;
  const auto s0 = static_cast<uint32_t>(search); /* window index of lag 0 */
  int64_t st = 0, stt = 0;
  for (uint32_t i = 0; i < len; i++) {
    const int32_t ti = t.v[2u * i];
    st += ti;
    stt += ti * ti;
  }

  float best_r = -1.f;
  int32_t best_lag = 0;
  int64_t sw = 0, sww = 0;
  for (int32_t lag = -search; lag <= search; lag += 2) {
    const auto s = static_cast<uint32_t>(search + lag);
    const int16_t *w = &g_win[s];
    int64_t dot = 0;
    if (lag == -search) {
      for (uint32_t i = 0; i < len; i++) {
        const int32_t wi = w[2u * i];
        dot += static_cast<int32_t>(t.v[2u * i]) * wi;
        sw += wi;
        sww += wi * wi;
      }
    } else {
      /* The strided set shifts by one element: drop the first, add one. */
      const int32_t out = g_win[s - 2u];
      const int32_t in = w[2u * (len - 1u)];
      sw += in - out;
      sww += in * in - out * out;
      for (uint32_t i = 0; i < len; i++) {
        dot += static_cast<int32_t>(t.v[2u * i]) * w[2u * i];
      }
    }
    const float r = ncc(dot, sw, sww, st, stt, len);
    if (r > best_r) {
      best_r = r;
      best_lag = lag;
    }
  }

  std::array<float, 5> fine{};
  int best = -1;
  for (int j = 0; j < 5; j++) {
    const int32_t lag = best_lag - 2 + j;
    if (lag < -search || lag > search) {
      fine[static_cast<size_t>(j)] = -1.f;
      continue;
    }
    const float r = ncc_full(t, s0 + static_cast<uint32_t>(lag));
    fine[static_cast<size_t>(j)] = r;
    if (best < 0 || r > fine[static_cast<size_t>(best)]) {
      best = j;
    }
  }
  if (best < 0) {
    return m;
  }
  m.ok = true;
  m.lag = best_lag - 2 + best;
  m.quality = std::max(0.f, fine[static_cast<size_t>(best)]);
  if (best > 0 && best < 4) {
    m.lag_frac = parabolic_offset(fine[static_cast<size_t>(best) - 1u],
                                  fine[static_cast<size_t>(best)],
                                  fine[static_cast<size_t>(best) + 1u]);
  }
  return m;
}

/**
 * Index of the drop peak in {@code v[0..n)}: the highest point of the last run
 * where the centred strength mean stays at or above 70 % of its maximum. The
 * plain maximum jumps between the unlock and the drop when both are loud (a
 * clipped recording), the last strong run does not.
 *
 * @param v Envelope samples.
 * @param n Count; must exceed {@code 2 * half}.
 * @param half Half-width of the strength mean (samples).
 * @param peak Receives the index of the peak.
 * @return {@code false} if the strength is zero everywhere.
 */
bool last_strong_peak(const int16_t *v, uint32_t n, uint32_t half,
                      uint32_t *peak) {
  const uint32_t span = 2u * half + 1u;
  const uint32_t count = n - span + 1u;
  int32_t sum = 0;
  for (uint32_t i = 0; i < span; i++) {
    sum += v[i];
  }
  int32_t top = sum;
  for (uint32_t j = 1; j < count; j++) {
    sum += v[j + span - 1u] - v[j - 1u];
    top = std::max(top, sum);
  }
  if (top <= 0) {
    return false;
  }
  sum = 0;
  for (uint32_t i = 0; i < span; i++) {
    sum += v[i];
  }
  bool in_run = false;
  bool found = false;
  int32_t run_best = 0;
  uint32_t run_at = 0;
  for (uint32_t j = 0; j < count; j++) {
    if (j > 0) {
      sum += v[j + span - 1u] - v[j - 1u];
    }
    if (sum * kPeakShareDen >= top * kPeakShareNum) {
      if (!in_run || sum > run_best) {
        run_best = sum;
        run_at = j;
      }
      in_run = true;
      found = true;
    } else {
      in_run = false;
    }
  }
  *peak = run_at + half;
  return found;
}

/** Drop landmark of a beat shape. */
struct DropLandmark {
  uint32_t peak = 0;      /**< Index of the drop peak. */
  float time = 0.f;       /**< Half-height crossing of the rising edge. */
  float background = 0.f; /**< Mean of the quiet pre-roll. */
  float height = 0.f;     /**< Peak above the background. */
};

/**
 * Finds the drop landmark of a beat shape: the peak is the last strong peak
 * ({@link last_strong_peak}), the landmark is the half-height crossing of the
 * rising edge found by walking back at most {@link kEdgeMs} from it, with
 * linear interpolation. Templates and window means use this one rule, so the
 * timestamp of a beat and the lift of its window refer to the same landmark.
 *
 * @param v Shape (envelope units), {@code n} samples, aligned so that the
 *        first {@link Geometry#baseline} samples are quiet.
 * @param n Sample count.
 * @param out Receives the landmark.
 * @return {@code false} if there is no peak above the background.
 */
bool find_drop_landmark(const int16_t *v, uint32_t n, DropLandmark *out) {
  uint32_t ip = 0;
  if (!last_strong_peak(v, n, g_geo.half, &ip)) {
    return false;
  }
  int32_t bg = 0;
  for (uint32_t i = 0; i < g_geo.baseline; i++) {
    bg += v[i];
  }
  bg /= static_cast<int32_t>(g_geo.baseline);
  const auto height = static_cast<float>(v[ip] - bg);
  if (height <= 0.f) {
    return false;
  }
  const float level = static_cast<float>(bg) + 0.5f * height;
  const uint32_t lim = ip > g_geo.edge ? ip - g_geo.edge : 0u;
  uint32_t i = ip;
  while (i > lim && static_cast<float>(v[i]) > level) {
    i--;
  }
  auto lm = static_cast<float>(i);
  const auto vi = static_cast<float>(v[i]);
  if (vi < level) {
    lm += (level - vi) / (static_cast<float>(v[i + 1u]) - vi);
  }
  out->peak = ip;
  out->time = lm;
  out->background = static_cast<float>(bg);
  out->height = height;
  return true;
}

/**
 * Finds the drop half-height crossing of a template and its sums, and marks
 * it ready. Called once per template update.
 */
void finish_template(Template &t) {
  const uint32_t n = g_geo.n_tpl;
  t.sum = 0;
  t.sum_sq = 0;
  for (uint32_t i = 0; i < n; i++) {
    const int32_t vi = t.v[i];
    t.sum += vi;
    t.sum_sq += vi * vi;
  }
  t.ready = false;
  if (t.beats < TICK_SHAPE_LEARN_BEATS) {
    return;
  }
  DropLandmark d;
  if (!find_drop_landmark(t.v.data(), n, &d)) {
    return;
  }
  t.landmark = d.time;
  t.ready = true;
}

/**
 * Moves {@code a} toward {@code b} by {@code 1/div}, rounding to nearest.
 */
int32_t blend(int32_t a, int32_t b, int32_t div) {
  const int32_t d = b - a;
  return a + (d >= 0 ? (d + div / 2) / div : -((-d + div / 2) / div));
}

/**
 * Folds the window slice at {@code lag} into a template: a running mean while
 * it learns (weight 1/n), 1/32 afterwards.
 */
void fold(Template &t, int32_t lag) {
  TG_PROFILE_STAGE(fold);
  const int16_t *w =
      &g_win[static_cast<uint32_t>(static_cast<int32_t>(g_geo.search) + lag)];
  const int32_t div = t.beats < TICK_SHAPE_LEARN_BEATS
                          ? static_cast<int32_t>(t.beats) + 1
                          : kEmaDivisor;
  for (uint32_t i = 0; i < g_geo.n_tpl; i++) {
    t.v[i] = static_cast<int16_t>(blend(t.v[i], w[i], div));
  }
  t.beats++;
  finish_template(t);
}

/**
 * Restarts a template from the current window at lag 0.
 */
void restart_template(Template &t) {
  t.beats = 0;
  t.rejects = 0;
  fold(t, 0);
}

/**
 * Locates the drop estimate for an event in the ring: copies the region
 * around the detector's estimate and takes the last strong peak.
 *
 * @param e_seed Envelope index of the detector's drop estimate.
 * @param anchor Receives the envelope index of the drop peak.
 * @return {@code false} if the ring no longer holds the region.
 */
bool find_anchor(int64_t e_seed, int64_t *anchor) {
  const uint32_t reg = g_geo.reg_before + g_geo.reg_after + 1u;
  const uint32_t len = reg + 2u * g_geo.half;
  const int64_t first =
      e_seed - static_cast<int64_t>(g_geo.reg_before) - g_geo.half;
  if (!gather(first, len, g_win.data())) {
    return false;
  }
  uint32_t peak = 0;
  if (!last_strong_peak(g_win.data(), len, g_geo.half, &peak)) {
    return false;
  }
  *anchor = first + peak;
  return true;
}

/* ---- Shape windows ------------------------------------------------------ */

/** Fixed-point scale of the sub-sample shift when a beat is added (Q8). */
constexpr int32_t kShiftOne = 256;

/** Running sum of the beats of one side in the open window. */
struct Accum {
  std::array<int32_t, kTplCap> sum{}; /**< Per-sample sum over the beats. */
  uint32_t beats = 0;                 /**< Beats added. */
};

/** Both sides of one window. */
using Bank = std::array<Accum, 2>;

/**
 * Two banks: new beats go to the active one while the sealed one waits for
 * its landmarks ({@link tick_shape_window_step}), so sealing costs nothing.
 */
std::array<Bank, 2> g_bank;
uint32_t g_active = 0;
/** The other bank holds a sealed window not yet analysed. */
bool g_sealed = false;
/** Next side of the sealed window to analyse (0, 1 or 2 = done). */
uint32_t g_step = 0;
tick_shape_window_t g_sealed_res;
/** Scratch for the smoothed shape used by the unlock search. */
std::array<int16_t, kTplCap> g_smooth{};

/** The beat the last {@link tick_shape_measure} timed, for window adds. */
struct LastBeat {
  bool ok = false;      /**< Timed and not yet added. */
  uint8_t side = 0;     /**< Side of its template. */
  int32_t lag = 0;      /**< Whole-sample lag of the match. */
  float lag_frac = 0.f; /**< Sub-sample part of the lag. */
};
LastBeat g_last;

/** Empties both banks and the sealed state. */
void clear_windows() {
  for (Bank &b : g_bank) {
    for (Accum &a : b) {
      if (a.beats != 0u) { /* the sums are zero otherwise */
        a.sum.fill(0);
        a.beats = 0;
      }
    }
  }
  g_active = 0;
  g_sealed = false;
  g_step = 0;
  g_sealed_res = tick_shape_window_t{};
  g_last = LastBeat{};
}

/**
 * Smoothed value at {@code i}: mean of {@code 2 * smooth_half + 1} samples,
 * edges clamped (the same smoothing as the reference's 0.4 ms boxcar).
 */
int32_t smooth_at(const int16_t *v, uint32_t n, uint32_t i) {
  const auto last = static_cast<int32_t>(n) - 1;
  const auto h = static_cast<int32_t>(g_geo.smooth_half);
  int32_t sum = 0;
  for (int32_t j = -h; j <= h; j++) {
    sum += v[std::min<int32_t>(
        last, std::max<int32_t>(0, static_cast<int32_t>(i) + j))];
  }
  return sum / (2 * h + 1);
}

/**
 * Prominence of the local maximum at {@code j} in {@code y[0..n)}: its height
 * above the higher of the two lowest points reached before a sample above it
 * (or the end of the range) on either side.
 */
int32_t prominence(const int16_t *y, uint32_t n, uint32_t j) {
  int32_t left = y[j];
  for (uint32_t i = j; i-- > 0 && y[i] <= y[j];) {
    left = std::min<int32_t>(left, y[i]);
  }
  int32_t right = y[j];
  for (uint32_t i = j + 1u; i < n && y[i] <= y[j]; i++) {
    right = std::min<int32_t>(right, y[i]);
  }
  return y[j] - std::max<int32_t>(left, right);
}

/**
 * Time of the unlock on an averaged shape (envelope samples, fractional), or
 * {@code false}. The onset is the first rise that stays above
 * {@code bg + max(2 % of the drop height, 4 sigma)} for 0.2 ms, before the
 * drop. The unlock is timed at the half height of the first significant peak
 * after the onset (peaks are found on a 0.4 ms smoothing, as in the offline
 * reference), on the unsmoothed rising edge. Sigma is the mean absolute
 * deviation of the pre-roll times 1.2533, not the median-based estimate of
 * the reference: it needs no sort.
 *
 * Cost: one pass over the pre-roll, one over the span up to the drop (the
 * rise test), and for the peak search the smoothed span (9 samples each) plus
 * the prominence of each local maximum until one qualifies.
 *
 * @param v Shape, {@code n} samples.
 * @param d Drop landmark of the same shape.
 * @param out Receives the time (index units).
 */
bool find_unlock_time(const int16_t *v, uint32_t n, const DropLandmark &d,
                      float *out) {
  const uint32_t n_bg = g_geo.baseline;
  float dev = 0.f;
  for (uint32_t i = 0; i < n_bg; i++) {
    dev += std::fabs(static_cast<float>(v[i]) - d.background);
  }
  const float sigma = kMadToSigma * dev / static_cast<float>(n_bg);
  const float thr =
      d.background + std::max(kOnsetShare * d.height, kOnsetNoiseMult * sigma);
  const auto drop_i = static_cast<uint32_t>(d.time);
  const uint32_t stop =
      std::max<uint32_t>(n_bg + 2u, drop_i > 1u ? drop_i - 1u : 0u);
  if (stop <= n_bg + g_geo.steady) {
    return false;
  }
  uint32_t run = 0;
  uint32_t on = 0;
  bool found = false;
  for (uint32_t i = n_bg; i + 1u < stop; i++) {
    run = static_cast<float>(v[i]) > thr ? run + 1u : 0u;
    if (run == g_geo.steady) {
      on = i + 1u - g_geo.steady;
      found = true;
      break;
    }
  }
  if (!found) {
    return false;
  }
  const uint32_t len = stop - on;
  for (uint32_t i = 0; i < len; i++) {
    g_smooth[i] = static_cast<int16_t>(smooth_at(v, n, on + i));
  }
  const auto prom_min =
      static_cast<int32_t>(std::max(0.5f * (thr - d.background), 1.f));
  uint32_t pk = 0;
  bool have_pk = false;
  for (uint32_t j = 1; j + 1u < len && !have_pk; j++) {
    if (g_smooth[j - 1u] >= g_smooth[j]) {
      continue;
    }
    uint32_t m = j;
    while (m + 1u < len && g_smooth[m + 1u] == g_smooth[j]) {
      m++;
    }
    if (m + 1u >= len || g_smooth[m + 1u] > g_smooth[j]) {
      j = m;
      continue;
    }
    const uint32_t mid = (j + m) / 2u;
    if (prominence(g_smooth.data(), len, mid) >= prom_min) {
      pk = mid;
      have_pk = true;
    }
    j = m;
  }
  if (!have_pk) {
    for (uint32_t i = 1; i < len; i++) {
      if (g_smooth[i] > g_smooth[pk]) {
        pk = i;
      }
    }
  }
  const uint32_t jpk = on + pk;
  const float level =
      d.background + 0.5f * (static_cast<float>(g_smooth[pk]) - d.background);
  for (uint32_t i = on > 0u ? on - 1u : 0u; i < jpk && i + 1u < n; i++) {
    const auto a = static_cast<float>(v[i]);
    const auto b = static_cast<float>(v[i + 1u]);
    if (a < level && level <= b) {
      *out = static_cast<float>(i) + (level - a) / (b - a);
      return true;
    }
  }
  *out = static_cast<float>(on);
  return true;
}

/** Clears the front end and templates, and the counters. */
void clear_all() {
  clear_front_end();
  for (Template &t : g_tpl) {
    t = Template{};
  }
  g_stats = tick_shape_stats_t{};
  clear_windows();
}

} // namespace

void tick_shape_configure(uint32_t source_hz) {
  if (g_geo.source_hz == source_hz) {
    return;
  }
  g_geo = Geometry{};
  g_geo.source_hz = source_hz;
  clear_all();
  if (source_hz < kMinSourceHz) {
    return;
  }
  g_geo.decim = (source_hz + kEnvHzMax - 1u) / kEnvHzMax;
  g_geo.env_hz = source_hz / g_geo.decim;
  const auto fs = static_cast<float>(source_hz);
  g_geo.boxcar = std::min<uint32_t>(
      kBoxcarMax, std::max<uint32_t>(1u, static_cast<uint32_t>(
                                             kBoxcarMs * 0.001f * fs + 0.5f)));
  g_geo.boxcar_delay = 0.5 * static_cast<double>(g_geo.boxcar - 1u);
  const uint32_t env = g_geo.env_hz;
  g_geo.n_pre = ms_to_samples(kPreMs, env);
  g_geo.n_tpl = ms_to_samples(kPreMs + kPostMs, env);
  g_geo.search = ms_to_samples(kSearchMs, env);
  g_geo.reg_before = ms_to_samples(kRegionBeforeMs, env);
  g_geo.reg_after = ms_to_samples(kRegionAfterMs, env);
  g_geo.half = static_cast<uint32_t>(
      0.5f * kStrengthMs * 0.001f * static_cast<float>(env) + 0.5f);
  g_geo.baseline = ms_to_samples(kBaselineMs, env);
  g_geo.edge = ms_to_samples(kEdgeMs, env);
  g_geo.steady = std::max<uint32_t>(
      2u, static_cast<uint32_t>(kSteadyMs * 0.001f * static_cast<float>(env)));
  g_geo.smooth_half = std::max<uint32_t>(
      1u,
      static_cast<uint32_t>(kSmoothMs * 0.001f * static_cast<float>(env)) / 2u);
  design_high_pass(fs);
  g_geo.valid = true;
}

void tick_shape_reset() { clear_all(); }

void tick_shape_forget_templates() {
  for (Template &t : g_tpl) {
    t = Template{};
  }
  clear_windows();
}

void tick_shape_set_enabled(bool on) {
  if (on && !g_enabled) {
    clear_front_end();
  }
  g_enabled = on;
}

bool tick_shape_enabled() { return g_enabled; }

void tick_shape_push(const float *x, uint32_t n, uint64_t base) {
  if (!g_enabled || !g_geo.valid) {
    return;
  }
  TG_PROFILE_STAGE(push);
  if (base != g_next_src) {
    clear_front_end();
  }
  g_next_src = base + n;
  const uint32_t box = g_geo.boxcar;
  const uint32_t decim = g_geo.decim;
  const float scale = kEnvScale / static_cast<float>(box);
  for (uint32_t i = 0; i < n; i++) {
    const float a = std::fabs(g_hp[1].step(g_hp[0].step(x[i])));
    g_box_sum += a - g_box[g_box_pos];
    g_box[g_box_pos] = a;
    if (++g_box_pos == box) {
      g_box_pos = 0;
      /* Re-sum once per window so float rounding cannot accumulate. */
      float s = 0.f;
      for (uint32_t j = 0; j < box; j++) {
        s += g_box[j];
      }
      g_box_sum = s;
    }
    if (++g_phase == decim) {
      g_phase = 0;
      if (g_total == 0u) {
        g_src0 = base + i;
      }
      const float v = std::min(32767.f, g_box_sum * scale);
      g_ring[g_head] = static_cast<int16_t>(v + 0.5f);
      g_head = (g_head + 1u == kRingCap) ? 0u : g_head + 1u;
      g_total++;
    }
  }
}

tick_shape_result_t tick_shape_measure(const dsp_event_t &ev, uint8_t side,
                                       bool detect_side) {
  tick_shape_result_t res;
  res.side = static_cast<uint8_t>(side & 1u);
  g_last.ok = false;
  if (!g_enabled || !g_geo.valid) {
    return res;
  }
  g_stats.events++;

  /* Envelope index of the detector's drop estimate (source index plus the
   * boxcar's centre delay, relative to envelope sample 0). */
  const double p_seed =
      static_cast<double>(ev.drop_sample) + static_cast<double>(ev.drop_frac);
  const double e_seed_d =
      (p_seed + g_geo.boxcar_delay - static_cast<double>(g_src0)) /
      static_cast<double>(g_geo.decim);
  int64_t anchor = 0;
  if (e_seed_d < 0.0 || !find_anchor(std::llround(e_seed_d), &anchor)) {
    g_stats.out_of_ring++;
    return res;
  }

  /* Window: template span plus the search either side, around the anchor. */
  const uint32_t win_len = g_geo.n_tpl + 2u * g_geo.search;
  const int64_t first = anchor - static_cast<int64_t>(g_geo.n_pre) -
                        static_cast<int64_t>(g_geo.search);
  if (!gather(first, win_len, g_win.data())) {
    g_stats.out_of_ring++;
    return res;
  }

  Match m;
  if (detect_side) {
    /* Templates with enough beats compete; one that fits wins. With a single
     * candidate that does not fit, the beat belongs to the other side. */
    uint32_t usable = 0;
    float best = -1.f;
    uint8_t best_side = res.side;
    Match best_match;
    for (uint8_t s = 0; s < 2u; s++) {
      if (g_tpl[s].beats < kAutoMinBeats) {
        continue;
      }
      usable++;
      const Match cand = match_template(g_tpl[s]);
      if (cand.ok && cand.quality > best) {
        best = cand.quality;
        best_side = s;
        best_match = cand;
      }
    }
    if (best >= TICK_SHAPE_QUALITY_MIN) {
      res.side = best_side;
      m = best_match;
    } else if (usable == 1u) {
      res.side = static_cast<uint8_t>(best_side ^ 1u);
    }
  }

  Template &t = g_tpl[res.side];
  if (t.beats == 0u) {
    fold(t, 0);
    g_stats.learning++;
    return res;
  }
  if (!m.ok) {
    m = match_template(t);
  }
  res.quality = m.quality;
  if (!m.ok || m.quality < TICK_SHAPE_QUALITY_MIN) {
    g_stats.low_quality++;
    if (!t.ready && ++t.rejects >= kLearnStallRejects) {
      restart_template(t);
      g_stats.restarts++;
    }
    return res;
  }
  t.rejects = 0;
  if (t.ready) {
    const double e_drop =
        static_cast<double>(anchor) + static_cast<double>(m.lag) +
        static_cast<double>(m.lag_frac) + static_cast<double>(t.landmark) -
        static_cast<double>(g_geo.n_pre);
    res.drop_sample = static_cast<double>(g_src0) +
                      e_drop * static_cast<double>(g_geo.decim) -
                      g_geo.boxcar_delay;
    res.valid = true;
    g_stats.timed++;
    g_last = {true, res.side, m.lag, m.lag_frac};
  } else {
    g_stats.learning++;
  }
  fold(t, m.lag);
  return res;
}

tick_shape_stats_t tick_shape_stats() { return g_stats; }

bool tick_shape_window_add() {
  if (!g_last.ok) {
    return false;
  }
  TG_PROFILE_STAGE(window_add);
  g_last.ok = false;
  Accum &a = g_bank[g_active][g_last.side];
  const auto last = static_cast<int32_t>(g_geo.n_tpl + 2u * g_geo.search) - 1;
  const int32_t first = static_cast<int32_t>(g_geo.search) + g_last.lag;
  const auto q = static_cast<int32_t>(
      std::lround(static_cast<float>(kShiftOne) * g_last.lag_frac));
  const int32_t step = q >= 0 ? 1 : -1;
  const int32_t aq = q >= 0 ? q : -q;
  for (uint32_t i = 0; i < g_geo.n_tpl; i++) {
    const int32_t s = first + static_cast<int32_t>(i);
    const int32_t w0 = g_win[static_cast<uint32_t>(s)];
    const int32_t w1 = g_win[static_cast<uint32_t>(
        std::min<int32_t>(last, std::max<int32_t>(0, s + step)))];
    a.sum[i] += w0 + (w1 - w0) * aq / kShiftOne;
  }
  a.beats++;
  return true;
}

void tick_shape_window_seal() {
  if (g_sealed) {
    return;
  }
  g_sealed = true;
  g_step = 0;
  g_sealed_res = tick_shape_window_t{};
  g_active ^= 1u;
}

bool tick_shape_window_pending() { return g_sealed; }

bool tick_shape_window_step(tick_shape_window_t *out) {
  if (!g_sealed) {
    return false;
  }
  TG_PROFILE_STAGE(window_step);
  Accum &a = g_bank[g_active ^ 1u][g_step];
  g_sealed_res.beats[g_step] = a.beats;
  if (g_geo.valid && a.beats >= TICK_SHAPE_WINDOW_MIN_BEATS) {
    const uint32_t n = g_geo.n_tpl;
    const auto div = static_cast<int32_t>(a.beats);
    for (uint32_t i = 0; i < n; i++) {
      g_win[i] = static_cast<int16_t>((a.sum[i] + div / 2) / div);
    }
    DropLandmark d;
    float tu = 0.f;
    if (find_drop_landmark(g_win.data(), n, &d) &&
        find_unlock_time(g_win.data(), n, d, &tu) && d.time > tu) {
      g_sealed_res.valid[g_step] = true;
      g_sealed_res.lift_samples[g_step] =
          static_cast<double>(d.time - tu) * static_cast<double>(g_geo.decim);
    }
  }
  if (a.beats != 0u) {
    a.sum.fill(0);
    a.beats = 0;
  }
  if (++g_step < 2u) {
    return false;
  }
  g_sealed = false;
  *out = g_sealed_res;
  return true;
}

void tick_shape_window_clear() { clear_windows(); }
