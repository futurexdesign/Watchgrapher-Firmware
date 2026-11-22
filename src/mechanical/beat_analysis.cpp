/**
 * @file beat_analysis.cpp
 * @brief Mechanical escapement timing from tick events (beat_rate block).
 *
 * Moved verbatim from {@code dsp_graph.cpp}: interval history, A/h lock,
 * least-squares rate / beat error, lift-time amplitude, rolling averages and
 * the beat_rate parameters. The only changes are that the source rate and the
 * block enable flag arrive as arguments instead of being read from graph
 * globals.
 *
 * @see mechanical/beat_analysis.h, dsp_graph.cpp
 */

#include "mechanical/beat_analysis.h"

#include "mechanical/beat_fit.h"
#include "timebase.h"

#include <Arduino.h>
#include <cmath>
#include <cstring>

namespace {

constexpr float kPi = 3.14159265f;

/**
 * Common mechanical watch rates (vibrations / beats per hour) for
 * nearest-match. Includes vintage, modern, hi-beat, and ultra hi-beat
 * escapements.
 */
constexpr uint32_t kCommonVph[] = {
    14400u, /* 2.0 Hz  — vintage / marine chronometer era */
    16200u, /* 2.25 Hz — older pocket / early wrist */
    18000u, /* 2.5 Hz  — classic standard */
    19800u, /* 2.75 Hz — less common */
    21600u, /* 3.0 Hz  — common automatic */
    25200u, /* 3.5 Hz  — e.g. some Seiko */
    28800u, /* 4.0 Hz  — modern standard */
    36000u, /* 5.0 Hz  — hi-beat (e.g. El Primero) */
    43200u, /* 6.0 Hz  — hi-beat */
    57600u, /* 8.0 Hz  — ultra hi-beat (e.g. GS 9SA5) */
};
/** Sliding history of inter-event intervals used for median beat estimate. */
constexpr uint32_t kBeatHist = 16u;
/** Amplitude sample history for median lift-time / degree estimate. */
constexpr uint32_t kAmpHist = 8u;
/**
 * Ring of per-tick instant timing samples for the rolling 1 s mean and the
 * configurable long mean ({@code avg_s}, 1..60 s). Sized for 60 s at 16 Hz
 * (57600 A/h): 960 samples.
 */
constexpr uint32_t kTimingAvgCap = 1024u;
constexpr uint32_t kTimingAvg1sMs = 1000u;
constexpr float kAvgDefaultS = 10.f;
constexpr float kAvgMinS = 1.f;
constexpr float kAvgMaxS = 60.f;
constexpr uint8_t kTimingFlagRate = 1u;
constexpr uint8_t kTimingFlagBe = 2u;
constexpr uint8_t kTimingFlagAmp = 4u;

/** Accepted range for the beat_rate fit window. */
constexpr float kRateWindowMinS = 2.f;
constexpr float kRateWindowMaxS = 30.f;

struct TimingAvgSample {
  uint32_t t_ms;
  float rate;
  float be;
  float amp;
  uint8_t flags;
};

float g_lift_angle_deg = 51.0f; /* Witschi default lift angle λ */
uint32_t g_vph_manual = 0u;     /* 0 = auto nearest common A/h */
/** Rate / beat-error fit window (seconds of ticks). */
float g_rate_window_s = 6.0f;
/** Integration time of the long rolling mean (seconds). */
float g_avg_s = kAvgDefaultS;
/** False while the capture rate is unverified (PCM1808 before measurement). */
bool g_timebase_ok = true;

uint32_t g_intervals[kBeatHist];
beat_fit_t g_fit;
uint32_t g_interval_n = 0;
uint64_t g_last_event_unlock = 0;
bool g_have_last_event = false;
float g_amp_hist[kAmpHist];
uint32_t g_amp_n = 0;
TimingAvgSample g_timing_avg[kTimingAvgCap];
uint32_t g_timing_avg_n = 0;
uint32_t g_timing_avg_head = 0;
dsp_beat_rate_t g_beat = {};

/** Per-beat ring; slot = seq % DSP_BEAT_LOG_CAP. */
dsp_beat_t g_beats[DSP_BEAT_LOG_CAP];
/** Seq of the next beat to record == beats recorded since stream reset. */
uint32_t g_beat_seq = 0;
/** Current grid id (0 until the first STARTED). */
uint32_t g_grid_id = 0;

/** Insertion-sort median of {@code n} uint32 values (mutates {@code tmp}). */
uint32_t median_u32(uint32_t *tmp, uint32_t n) {
  if (n == 0) {
    return 0;
  }
  for (uint32_t i = 1; i < n; i++) {
    uint32_t key = tmp[i];
    auto j = static_cast<int32_t>(i) - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2u];
}

/** Insertion-sort median of {@code n} float values (mutates {@code tmp}). */
float median_f32(float *tmp, uint32_t n) {
  if (n == 0) {
    return 0.f;
  }
  for (uint32_t i = 1; i < n; i++) {
    float key = tmp[i];
    auto j = static_cast<int32_t>(i) - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2u];
}

/**
 * Witschi amplitude from lift time t and lift angle λ (degrees).
 * A = λ / (2 sin(π f t)) with f = S/(2·3600); falls back to small-angle form.
 */
float amplitude_from_lift(float lift_s, uint32_t vph, float lift_angle_deg) {
  if (lift_s < 5e-4f || vph == 0u || lift_angle_deg < 1.f) {
    return 0.f;
  }
  const auto f = static_cast<float>(vph) / 7200.0f; /* oscillation Hz */
  const float arg = kPi * f * lift_s;
  if (arg >= 1.0f) {
    return 0.f; /* outside sinusoidal model */
  }
  float s = sinf(arg);
  if (s < 1e-4f) {
    s = arg; /* ≈ π f t */
  }
  return lift_angle_deg / (2.0f * s);
}

void clear_timing_avg(void) {
  g_timing_avg_n = 0;
  g_timing_avg_head = 0;
  g_beat.avg_1s = {};
  g_beat.avg_long = {};
}

void push_timing_avg_sample(uint32_t t_ms, float rate, float be, float amp,
                            uint8_t flags) {
  if (flags == 0u) {
    return;
  }
  TimingAvgSample &s = g_timing_avg[g_timing_avg_head];
  s.t_ms = t_ms;
  s.rate = rate;
  s.be = be;
  s.amp = amp;
  s.flags = flags;
  g_timing_avg_head = (g_timing_avg_head + 1u) % kTimingAvgCap;
  if (g_timing_avg_n < kTimingAvgCap) {
    g_timing_avg_n++;
  }
}

/**
 * Arithmetic mean of instant samples whose timestamps fall in
 * {@code [now_ms - window_ms, now_ms]} (millis wrap-safe for spans << 24 d).
 */
void compute_timing_avg_window(uint32_t now_ms, uint32_t window_ms,
                               dsp_beat_timing_window_t *out) {
  if (!out) {
    return;
  }
  *out = {};
  if (g_timing_avg_n == 0u) {
    return;
  }
  float rate_sum = 0.f, be_sum = 0.f, amp_sum = 0.f;
  uint32_t rate_n = 0, be_n = 0, amp_n = 0;
  /* Newest first; samples are time-ordered, so stop at the first one older
   * than the window (a 1 s mean touches ~16 entries, not the whole ring). */
  for (uint32_t i = 1; i <= g_timing_avg_n; i++) {
    const uint32_t idx =
        (g_timing_avg_head + kTimingAvgCap - i) % kTimingAvgCap;
    const TimingAvgSample &s = g_timing_avg[idx];
    const uint32_t age = now_ms - s.t_ms; /* uint wrap-safe */
    if (age > window_ms) {
      break;
    }
    if (s.flags & kTimingFlagRate) {
      rate_sum += s.rate;
      rate_n++;
    }
    if (s.flags & kTimingFlagBe) {
      be_sum += s.be;
      be_n++;
    }
    if (s.flags & kTimingFlagAmp) {
      amp_sum += s.amp;
      amp_n++;
    }
  }
  out->samples = rate_n;
  if (be_n > out->samples) {
    out->samples = be_n;
  }
  if (amp_n > out->samples) {
    out->samples = amp_n;
  }
  if (rate_n >= 2u) {
    out->rate_s_day = rate_sum / static_cast<float>(rate_n);
    out->rate_valid = true;
  }
  if (be_n >= 2u) {
    out->beat_error_ms = be_sum / static_cast<float>(be_n);
    out->beat_error_valid = true;
  }
  if (amp_n >= 2u) {
    out->amplitude_deg = amp_sum / static_cast<float>(amp_n);
    out->amplitude_valid = true;
  }
}

void sync_instant_window(void) {
  g_beat.instant.rate_s_day = g_beat.rate_s_day;
  g_beat.instant.beat_error_ms = g_beat.beat_error_ms;
  g_beat.instant.amplitude_deg = g_beat.amplitude_deg;
  g_beat.instant.rate_valid = g_beat.rate_valid;
  g_beat.instant.beat_error_valid = g_beat.beat_error_valid;
  g_beat.instant.amplitude_valid = g_beat.amplitude_valid;
  g_beat.instant.samples =
      (g_beat.rate_valid || g_beat.beat_error_valid || g_beat.amplitude_valid)
          ? 1u
          : 0u;
}

} // namespace

void beat_analysis_init() {
  g_lift_angle_deg = 51.0f;
  g_vph_manual = 0u;
  g_rate_window_s = 6.0f;
  g_avg_s = kAvgDefaultS;
  g_timebase_ok = true;
  beat_analysis_reset();
}

void beat_analysis_reset() {
  g_interval_n = 0;
  beat_fit_reset(&g_fit);
  g_amp_n = 0;
  g_have_last_event = false;
  g_last_event_unlock = 0;
  g_beat = {};
  g_beat.lift_angle_deg = g_lift_angle_deg;
  clear_timing_avg();
  __atomic_store_n(&g_beat_seq, 0u, __ATOMIC_RELEASE);
  g_grid_id = 0;
}

void beat_analysis_set_timebase(bool ok) {
  if (ok == g_timebase_ok) {
    return;
  }
  g_timebase_ok = ok;
  /* Intervals timed against an unverified clock are not trusted either way. */
  g_interval_n = 0;
  g_have_last_event = false;
  beat_fit_reset(&g_fit);
  g_beat.locked = false;
  g_beat.vph_nearest = 0u;
  clear_timing_avg();
}

void beat_analysis_on_event(const dsp_event_t &ev, uint32_t source_hz,
                            bool beat_rate_enabled) {
  /* Prefer unlock landmark; fall back to peak only if unlock was never set. */
  const uint64_t unlock =
      (ev.unlock_sample != 0u || ev.drop_sample != 0u || ev.lift_samples != 0u)
          ? ev.unlock_sample
          : ev.peak_sample;
  if (g_have_last_event && unlock > g_last_event_unlock) {
    auto iv = static_cast<uint32_t>(unlock - g_last_event_unlock);
    /*
     * Accept ~25 ms .. 1 s in source-rate units (tick/tock pairs are often
     * half a beat). Using source_hz — not USB 48 kHz — is required when the
     * PCM1808 path analyzes pre-upsample frames.
     */
    const uint32_t iv_min = source_hz / 40u;
    const uint32_t iv_max = source_hz;
    if (iv > iv_min && iv < iv_max) {
      if (g_interval_n < kBeatHist) {
        g_intervals[g_interval_n++] = iv;
      } else {
        memmove(&g_intervals[0], &g_intervals[1],
                (kBeatHist - 1u) * sizeof(uint32_t));
        g_intervals[kBeatHist - 1u] = iv;
      }
    }
  }
  g_last_event_unlock = unlock;
  g_have_last_event = true;

  g_beat.lift_angle_deg = g_lift_angle_deg;
  g_beat.rate_valid = false;
  g_beat.beat_error_valid = false;
  g_beat.amplitude_valid = false;
  g_beat.rate_s_day = 0.f;
  g_beat.beat_error_ms = 0.f;
  g_beat.amplitude_deg = 0.f;
  g_beat.lift_time_ms = 0.f;
  g_beat.rate_points = 0;
  g_beat.jitter_us = 0.f;

  auto finalize_windows = [&](bool push_sample) {
    sync_instant_window();
    const auto now = millis();
    if (push_sample) {
      uint8_t flags = 0;
      if (g_beat.rate_valid) {
        flags |= kTimingFlagRate;
      }
      if (g_beat.beat_error_valid) {
        flags |= kTimingFlagBe;
      }
      if (g_beat.amplitude_valid) {
        flags |= kTimingFlagAmp;
      }
      push_timing_avg_sample(now, g_beat.rate_s_day, g_beat.beat_error_ms,
                             g_beat.amplitude_deg, flags);
    }
    compute_timing_avg_window(now, kTimingAvg1sMs, &g_beat.avg_1s);
    compute_timing_avg_window(now,
                              static_cast<uint32_t>(lroundf(g_avg_s * 1000.f)),
                              &g_beat.avg_long);
  };

  if (g_interval_n < 4u || !beat_rate_enabled || !g_timebase_ok) {
    if (!g_timebase_ok) {
      /* Unverified sample rate: refuse to lock rather than mis-time. */
      g_beat.locked = false;
      g_beat.vph_nearest = 0u;
    }
    beat_fit_reset(&g_fit);
    finalize_windows(false);
    return;
  }

  uint32_t tmp[kBeatHist];
  memcpy(tmp, g_intervals, g_interval_n * sizeof(uint32_t));
  uint32_t med = median_u32(tmp, g_interval_n);
  g_beat.interval_samples = med;
  auto period_s = static_cast<float>(med) / static_cast<float>(source_hz);
  g_beat.vph_raw = (period_s > 0.f) ? (3600.0f / period_s) : 0.f;

  uint32_t nearest = 0;
  float best = 1e9f;
  if (g_vph_manual != 0u) {
    nearest = g_vph_manual;
    best = fabsf(g_beat.vph_raw - static_cast<float>(nearest));
  } else {
    for (uint32_t c : kCommonVph) {
      auto d = fabsf(g_beat.vph_raw - static_cast<float>(c));
      if (d < best) {
        best = d;
        nearest = c;
      }
    }
  }
  auto tol = 0.04f * static_cast<float>(nearest);
  g_beat.locked = nearest != 0u && best <= tol;
  g_beat.vph_nearest = g_beat.locked ? nearest : 0u;
  g_beat.confidence =
      g_beat.locked
          ? fmaxf(0.f, 1.f - best / (tol + 1.f))
          : fmaxf(0.f, 1.f - best / (static_cast<float>(nearest) + 1.f));

  if (!g_beat.locked) {
    beat_fit_reset(&g_fit);
    finalize_windows(false);
    return;
  }

  /*
   * Rate + beat error from the fit. A change of nominal A/h restarts the
   * grid inside beat_fit_push. cal_ppm: a sample clock fast by p ppm counts
   * (1 + p·1e-6) samples per true second.
   */
  const double fs = static_cast<double>(source_hz);
  const double fs_true =
      fs * (1.0 + 1e-6 * static_cast<double>(timebase_ppm()));
  const double nominal_half =
      fs * 3600.0 / static_cast<double>(g_beat.vph_nearest);
  const double t_unlock =
      static_cast<double>(unlock) + static_cast<double>(ev.unlock_frac);
  beat_fit_point_t pt{};
  const beat_fit_push_result_t push_res =
      beat_fit_push(&g_fit, t_unlock, nominal_half,
                    static_cast<double>(g_rate_window_s) * fs, &pt);
  if (push_res == BEAT_FIT_STARTED) {
    g_grid_id++;
  }
  beat_fit_result_t fit{};
  if (beat_fit_solve(&g_fit, &fit)) {
    const double true_half_s = fit.half_period / fs_true;
    const double nominal_half_s =
        3600.0 / static_cast<double>(g_beat.vph_nearest);
    /* Watch fast ⇒ measured period shorter than nominal ⇒ positive s/d. */
    g_beat.rate_s_day =
        static_cast<float>((1.0 - true_half_s / nominal_half_s) * 86400.0);
    g_beat.rate_valid = true;
    g_beat.rate_points = fit.points;
    g_beat.jitter_us = static_cast<float>(fit.resid_rms / fs_true * 1e6);
    if (fit.alt_valid) {
      g_beat.beat_error_ms =
          static_cast<float>(2.0 * fabsf(fit.alt_offset) / fs_true * 1000.0);
      g_beat.beat_error_valid = true;
    }
  }

  /* Per-tick amplitude / lift for the beat record (not the median). */
  float beat_amp_deg = 0.f;
  bool beat_amp_valid = false;
  float beat_lift_ms = 0.f;

  /* Amplitude from unlock→drop lift time. */
  if (ev.lift_samples > 0u) {
    const float lift_samp =
        static_cast<float>(ev.lift_samples) + ev.drop_frac - ev.unlock_frac;
    const auto lift_s = lift_samp / static_cast<float>(fs_true);
    const float a_deg =
        amplitude_from_lift(lift_s, g_beat.vph_nearest, g_lift_angle_deg);
    beat_lift_ms = 1000.0f * lift_s;
    if (a_deg > 50.f && a_deg < 400.f) {
      beat_amp_deg = a_deg;
      beat_amp_valid = true;
      if (g_amp_n < kAmpHist) {
        g_amp_hist[g_amp_n++] = a_deg;
      } else {
        memmove(&g_amp_hist[0], &g_amp_hist[1],
                (kAmpHist - 1u) * sizeof(float));
        g_amp_hist[kAmpHist - 1u] = a_deg;
      }
      float amp_tmp[kAmpHist];
      memcpy(amp_tmp, g_amp_hist, g_amp_n * sizeof(float));
      g_beat.amplitude_deg = median_f32(amp_tmp, g_amp_n);
      g_beat.lift_time_ms = 1000.0f * lift_s;
      g_beat.amplitude_valid = g_amp_n >= 2u;
    }
  }

  /*
   * Record this tick. u is corrected like the rate: time since the grid
   * origin is scaled by 1/(1+ppm) before the nominal grid is subtracted, so
   * the chart slope equals -rate_s_day / 86400 * nominal.
   */
  if (pt.valid) {
    const double rel_true =
        pt.rel / (1.0 + 1e-6 * static_cast<double>(timebase_ppm()));
    const double u_samp = rel_true - static_cast<double>(pt.k) * pt.nominal;
    const uint32_t seq = g_beat_seq;
    dsp_beat_t &b = g_beats[seq % DSP_BEAT_LOG_CAP];
    b.seq = seq;
    b.grid_id = g_grid_id;
    b.sample_index = unlock;
    b.t_s = t_unlock / fs;
    b.k = pt.k;
    b.parity = static_cast<uint8_t>(pt.k & 1);
    b.u_us = static_cast<float>(u_samp / fs * 1e6);
    b.nominal_us = static_cast<float>(pt.nominal / fs * 1e6);
    b.amplitude_deg = beat_amp_valid ? beat_amp_deg : 0.f;
    b.amplitude_valid = beat_amp_valid;
    b.lift_ms = beat_lift_ms;
    b.status = push_res == BEAT_FIT_STARTED    ? DSP_BEAT_STARTED
               : push_res == BEAT_FIT_ACCEPTED ? DSP_BEAT_ACCEPTED
                                               : DSP_BEAT_REJECTED;
    __atomic_store_n(&g_beat_seq, seq + 1u, __ATOMIC_RELEASE);
  }

  finalize_windows(g_beat.rate_valid || g_beat.beat_error_valid ||
                   g_beat.amplitude_valid);
}

uint32_t beat_analysis_beat_seq() {
  return __atomic_load_n(&g_beat_seq, __ATOMIC_ACQUIRE);
}

uint32_t beat_analysis_copy_beats_since(uint32_t seq, dsp_beat_t *out,
                                        uint32_t max_n) {
  if (!out || max_n == 0u) {
    return 0;
  }
  uint32_t end = __atomic_load_n(&g_beat_seq, __ATOMIC_ACQUIRE);
  /* Oldest still-resident seq; anything older was overwritten. */
  uint32_t first = end > DSP_BEAT_LOG_CAP ? end - DSP_BEAT_LOG_CAP : 0u;
  if (seq > first) {
    first = seq;
  }
  if (first >= end) {
    return 0;
  }
  uint32_t n = 0;
  for (uint32_t s = first; s < end && n < max_n; s++) {
    out[n++] = g_beats[s % DSP_BEAT_LOG_CAP];
  }
  /*
   * The writer may have lapped us while copying: slot s is rewritten while
   * the counter reads s + CAP (record written before the counter advances).
   * Drop the leading entries that may be torn.
   */
  const uint32_t after = __atomic_load_n(&g_beat_seq, __ATOMIC_ACQUIRE);
  uint32_t safe_from =
      after >= DSP_BEAT_LOG_CAP ? after - DSP_BEAT_LOG_CAP + 1u : 0u;
  uint32_t drop = 0;
  while (drop < n && first + drop < safe_from) {
    drop++;
  }
  if (drop) {
    for (uint32_t i = drop; i < n; i++) {
      out[i - drop] = out[i];
    }
    n -= drop;
  }
  return n;
}

uint32_t beat_analysis_copy_beats(dsp_beat_t *out, uint32_t max_n) {
  if (!out || max_n == 0u) {
    return 0;
  }
  const uint32_t end = beat_analysis_beat_seq();
  const uint32_t avail = end < DSP_BEAT_LOG_CAP ? end : DSP_BEAT_LOG_CAP;
  const uint32_t want = avail < max_n ? avail : max_n;
  if (want == 0u) {
    return 0;
  }
  /* Oldest-first copy of the newest `want`, then reverse in place. */
  const uint32_t n = beat_analysis_copy_beats_since(end - want, out, want);
  for (uint32_t i = 0; i < n / 2u; i++) {
    const dsp_beat_t tmp = out[i];
    out[i] = out[n - 1u - i];
    out[n - 1u - i] = tmp;
  }
  return n;
}

void beat_analysis_get(dsp_beat_rate_t *out) {
  *out = g_beat;
  out->intervals_held = g_interval_n;
  out->avg_s = g_avg_s;
  out->cal_ppm = timebase_ppm();
  out->cal_source = timebase_source_name(timebase_source());
  out->timebase_ok = g_timebase_ok;
}

bool beat_analysis_set_param(const char *param, float value) {
  if (param == nullptr) {
    return false;
  }
  if (strcmp(param, "liftAngle") == 0 || strcmp(param, "lift_angle") == 0) {
    if (value < 10.f || value > 90.f) {
      return false;
    }
    g_lift_angle_deg = value;
    g_beat.lift_angle_deg = value;
    g_amp_n = 0; /* lift angle change invalidates prior amplitude hist */
    clear_timing_avg();
    return true;
  }
  if (strcmp(param, "vph") == 0) {
    if (value < 0.f) {
      return false;
    }
    auto v = static_cast<uint32_t>(lroundf(value));
    if (v == 0u) {
      g_vph_manual = 0u;
      clear_timing_avg();
      return true;
    }
    if (v < 3600u || v > 72000u) {
      return false;
    }
    g_vph_manual = v;
    clear_timing_avg();
    return true;
  }
  if (strcmp(param, "calPpm") == 0 || strcmp(param, "cal_ppm") == 0) {
    /* Same store as the CLI / HID timebase commands (RAM; save persists). */
    if (!timebase_set(value, TIMEBASE_SRC_MANUAL, 0u, 0u)) {
      return false;
    }
    clear_timing_avg();
    return true;
  }
  if (strcmp(param, "windowS") == 0 || strcmp(param, "window_s") == 0) {
    if (value < kRateWindowMinS || value > kRateWindowMaxS) {
      return false;
    }
    g_rate_window_s = value;
    return true;
  }
  if (strcmp(param, "avgS") == 0 || strcmp(param, "avg_s") == 0) {
    if (!(value >= kAvgMinS && value <= kAvgMaxS)) {
      return false;
    }
    /* History is kept: a longer window fills as ticks arrive. */
    g_avg_s = value;
    return true;
  }
  return false;
}

bool beat_analysis_get_param(const char *param, float *out) {
  if (param == nullptr || out == nullptr) {
    return false;
  }
  if (strcmp(param, "liftAngle") == 0 || strcmp(param, "lift_angle") == 0) {
    *out = g_lift_angle_deg;
    return true;
  }
  if (strcmp(param, "vph") == 0) {
    *out = static_cast<float>(g_vph_manual);
    return true;
  }
  if (strcmp(param, "calPpm") == 0 || strcmp(param, "cal_ppm") == 0) {
    *out = timebase_ppm();
    return true;
  }
  if (strcmp(param, "windowS") == 0 || strcmp(param, "window_s") == 0) {
    *out = g_rate_window_s;
    return true;
  }
  if (strcmp(param, "avgS") == 0 || strcmp(param, "avg_s") == 0) {
    *out = g_avg_s;
    return true;
  }
  return false;
}
