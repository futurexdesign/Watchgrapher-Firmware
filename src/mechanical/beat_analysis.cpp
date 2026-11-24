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
 * Cuts and the no-watch rule (item 4 of docs/plans/TIMING_ENGINE_PLAN.md) also
 * live here: a cut ends the current stretch of beats and nothing is fitted or
 * averaged across it.
 *
 * @see mechanical/beat_analysis.h, dsp_graph.cpp
 */

#include "mechanical/beat_analysis.h"

#include "mechanical/beat_fit.h"
#include "mechanical/tick_shape.h"
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
/**
 * A stretch must span this long (s, first to latest accepted beat since the
 * last cut) before it gives a rate or beat error. The reading is
 * provisional until the stretch spans {@link kMinStretchS}: a fit over a few
 * seconds of beats can be several s/d out (opentimegrapher M6). It replaces
 * the former gate on the fit's span (3/4 of its window, about 5 s): the fit
 * no longer restarts inside a stretch (the handover to drop times shifts its
 * points).
 */
constexpr double kProvisionalStretchS = 5.0;
/**
 * A stretch this long (s) gives settled readings: {@code settled} is set from
 * here on (opentimegrapher: a stretch under 10 s gives no reliable rate).
 */
constexpr double kMinStretchS = 10.0;
/** More than this many seconds without an accepted beat is a cut. */
constexpr double kGapCutS = 2.0;
/**
 * A change of {@link timebase_ppm} larger than this (ppm) since the last beat
 * is a cut. PPS discipline moves the correction in steps of about 0.01 ppm,
 * which must not cut; locking to PPS or setting a calibration moves it by
 * more.
 */
constexpr float kClockCutPpm = 0.5f;
/** The no-watch rule looks at this many seconds of beats. */
constexpr double kNoWatchWindowS = 10.0;
/**
 * State {@code noWatch} ends when a lock holds a stretch this long (s); the
 * lock flag itself may return sooner, on stray detections.
 */
constexpr double kNoWatchClearS = 2.0;
/** The rule starts once the stretch spans this long (s). */
constexpr double kNoWatchMinSpanS = 5.0;
/** RMS scatter about the fit, as a fraction of the half-period. */
constexpr float kNoWatchScatter = 0.02f;
/** Fewer than this fraction of the expected beats accepted is no watch. */
constexpr double kNoWatchMinAccepted = 0.5;
/**
 * The worst 1/kNoWatchTrimDiv (12.5 %) of the events in the window are left
 * out of the scatter, so that a knock or the stray detections of a real
 * recording (one clip shows 3 in 29 events) do not read as no watch, while the
 * events of handling noise (more than one in six within 2 s) do.
 */
constexpr uint32_t kNoWatchTrimDiv = 8u;
/** Events kept for the rule (10 s at 16 beats/s plus stray detections). */
constexpr uint32_t kWatchCap = 192u;
/** Largest number of trimmed events: {@code kWatchCap / kNoWatchTrimDiv}. */
constexpr uint32_t kTrimMax = kWatchCap / kNoWatchTrimDiv;
/** Residual scale of {@link WatchEvent#miss_q}: half-periods * 32768. */
constexpr float kMissScale = 32768.f;
/** Accepted range of the {@code settleS} parameter (s). */
constexpr float kSettleMaxS = 60.f;

/** Weight of a new beat in the per-side drop-minus-unlock estimate. */
constexpr double kLiftEstWeight = 0.125;

/** Length of a shape window (s): beats are averaged per side over this time. */
constexpr double kWindowS = 2.0;
/**
 * Window results kept: the longest averaging span (60 s) over the window
 * length, plus a margin.
 */
constexpr uint32_t kWindowCap = 32u;
/** Windows needed for a beat error from the unlock (else the drop figure). */
constexpr uint32_t kBeMinWindows = 3u;
/**
 * Slack of the sign test of the window beat errors (opentimegrapher M5, as in
 * {@code reference_otg.py}). With median {@code m} and standard deviation
 * {@code sd} of the window values, a share {@code Phi(|m| / sd)} of them is
 * expected on the side of {@code m}; the windows disagree when the observed
 * share falls more than this below it.
 */
constexpr float kBeSignSlack = 0.2f;
/**
 * The side that comes late is Tick when the drop beat error is at least this
 * large (ms); below it the side of the stretch's first beat is Tick.
 */
constexpr float kBeLabelMinMs = 0.1f;
/** Accepted amplitude range of a side or window (degrees), as per beat. */
constexpr float kAmpMinDeg = 50.f;
constexpr float kAmpMaxDeg = 400.f;

/** Result of one closed shape window. */
struct WindowResult {
  uint32_t t_ms;   /**< Time the window closed (ms, corrected clock). */
  float lift_s[2]; /**< Unlock-to-drop per template side (s). */
  bool lift_ok[2]; /**< The side's lift was found. */
  bool be_ok;      /**< {@code be_side0_ms} was taken from a drop-timed fit. */
  float be_side0_ms; /**< Drop beat error, positive when side 0 is late. */
};

/** What the closing event knew about the window it sealed. */
struct WindowMeta {
  bool keep;         /**< Past the settling time: store the result. */
  uint32_t t_ms;     /**< Closing time. */
  bool be_ok;        /**< Drop beat error was available. */
  float be_side0_ms; /**< Drop beat error, positive when side 0 is late. */
};

/** Readings derived from the stored windows over the averaging span. */
struct WindowReading {
  bool amp_valid = false;
  float amp_deg = 0.f;
  bool tick_valid = false;
  bool tock_valid = false;
  float tick_deg = 0.f;
  float tock_deg = 0.f;
  uint32_t used = 0;
  uint32_t total = 0;
  bool be_ok = false; /**< Window beat error usable (3 windows, same sign). */
  float be_signed_ms = 0.f;
};

/** Accepted range for the beat_rate fit window. */
constexpr float kRateWindowMinS = 2.f;
constexpr float kRateWindowMaxS = 30.f;

/** One tick event as the no-watch rule sees it. */
struct WatchEvent {
  uint32_t t_ms;  /**< Beat time on the corrected clock (ms, wraps). */
  int16_t miss_q; /**< Residual about the fit, half-periods * kMissScale. */
  bool accepted;  /**< The beat entered the fit. */
};

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
/**
 * {@code timingLandmark}: {@code true} times beats at the drop (stage B,
 * {@link tick_shape_measure}), {@code false} at the unlock.
 */
bool g_timing_drop = false;
/**
 * True once the fit has been restarted on drop times: from then on every
 * timestamp pushed to it is a drop time (measured, or the unlock plus the
 * side's lift estimate), never a raw unlock.
 */
bool g_drop_active = false;
/**
 * Seconds after a cut before samples enter the rolling means ({@code settleS}).
 */
float g_settle_s = 0.0f;
/**
 * The timing engine runs. False in USB microphone mode: events then only feed
 * the interval history and the A/h lock ({@link evaluate_lock}).
 */
bool g_analysis = true;
/** True while a stretch of accepted beats is open (since the last cut). */
bool g_stretch_open = false;
/** Time of the first accepted beat of the open stretch (source samples). */
double g_stretch_t0 = 0.0;
/** Time of the latest accepted beat (source samples). */
double g_last_accept_t = 0.0;
/** {@link timebase_ppm} at the latest accepted beat. */
float g_last_ppm = 0.f;
/** The no-watch rule fired and no lock has held a stretch since. */
bool g_no_watch = false;
/** The open stretch has already reached {@link kMinStretchS}. */
bool g_settled_seen = false;
/** Events of the open stretch, newest last; ring of {@link kWatchCap}. */
WatchEvent g_watch[kWatchCap];
uint32_t g_watch_n = 0;
uint32_t g_watch_head = 0;
/** Maps grid-slot parity to the template side: side = parity ^ xor. */
uint8_t g_side_xor = 0;
/**
 * Per side, running mean of drop minus unlock (source samples) from beats
 * timed at the drop; used to place a beat stage B could not time on the drop
 * time base.
 */
double g_lift_est[2] = {0.0, 0.0};
bool g_lift_est_valid[2] = {false, false};

/** Stored window results, oldest first in a ring. */
WindowResult g_windows[kWindowCap];
uint32_t g_win_n = 0;
uint32_t g_win_head = 0;
/** Meta of the sealed window waiting for its landmarks. */
WindowMeta g_win_meta = {};
/** The window clock runs (a stretch is open). */
bool g_win_open = false;
/** End of the open window (source samples on the stream clock). */
double g_win_end = 0.0;
/** Template side that is Tick (the late side), or -1 until decided. */
int32_t g_tick_side = -1;
/** The beat just handled was timed at the drop by stage B. */
bool g_beat_timed = false;
/** Drop beat error of the fit now (ms, + = side 0 late) and its validity. */
float g_drop_be_side0_ms = 0.f;
bool g_drop_be_ok = false;
/** Cached reading of the stored windows; refreshed outside the event path. */
WindowReading g_reading;
bool g_reading_dirty = false;
/** An event was handled in this block (the idle step then waits). */
bool g_event_in_block = false;
/** Corrected sample rate and time of the latest event, for the idle step. */
double g_fs_true = 48000.0;
uint32_t g_last_event_ms = 0;

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
/**
 * Nominal A/h the rolling means and the amplitude history belong to (0 =
 * none). A different lock (another watch on the stand) clears both: their
 * samples were timed, and the amplitude derived, against the old rate.
 */
uint32_t g_history_vph = 0;

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
 * Median of {@code n} float values that averages the two middle values when
 * {@code n} is even (as {@code numpy.median}, which the offline reference
 * uses); mutates {@code tmp}. {@link median_f32} takes the upper one.
 */
float median_mean_f32(float *tmp, uint32_t n) {
  const float upper = median_f32(tmp, n);
  return (n != 0u && n % 2u == 0u) ? 0.5f * (tmp[n / 2u - 1u] + upper) : upper;
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

/**
 * Drops the rate and beat-error contribution of every rolling-mean sample
 * (the amplitude stays). Called when a stretch becomes settled: the readings
 * taken before are provisional and, on a noisy recording, off by tens of s/d
 * (the 20 dB clip reads -87 s/d at 5 s), which would sit in a 20 s average
 * long after the fit is exact. They are also partly unlock-timed, and unlock-
 * and drop-based beat error differ by the lift asymmetry.
 */
void clear_provisional_samples(void) {
  for (uint32_t i = 0; i < g_timing_avg_n; i++) {
    g_timing_avg[i].flags &=
        static_cast<uint8_t>(~(kTimingFlagRate | kTimingFlagBe));
  }
  g_beat.avg_1s.rate_valid = false;
  g_beat.avg_1s.beat_error_valid = false;
  g_beat.avg_long.rate_valid = false;
  g_beat.avg_long.beat_error_valid = false;
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
 * The beat-error angle is averaged over the samples that carry both a beat
 * error and an amplitude, at nominal A/h {@code vph} (the A/h the history
 * belongs to; the history is cleared when it changes).
 */
void compute_timing_avg_window(uint32_t now_ms, uint32_t window_ms,
                               uint32_t vph, dsp_beat_timing_window_t *out) {
  if (!out) {
    return;
  }
  *out = {};
  if (g_timing_avg_n == 0u) {
    return;
  }
  float rate_sum = 0.f, be_sum = 0.f, amp_sum = 0.f, angle_sum = 0.f;
  uint32_t rate_n = 0, be_n = 0, amp_n = 0, angle_n = 0;
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
    if ((s.flags & kTimingFlagBe) && (s.flags & kTimingFlagAmp)) {
      angle_sum += beat_error_angle_deg(s.amp, vph, s.be);
      angle_n++;
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
  if (angle_n >= 2u) {
    out->beat_error_angle_deg = angle_sum / static_cast<float>(angle_n);
    out->beat_error_angle_valid = true;
  }
}

void sync_instant_window(void) {
  g_beat.instant.rate_s_day = g_beat.rate_s_day;
  g_beat.instant.beat_error_ms = g_beat.beat_error_ms;
  g_beat.instant.amplitude_deg = g_beat.amplitude_deg;
  g_beat.instant.beat_error_angle_deg = g_beat.beat_error_angle_deg;
  g_beat.instant.rate_valid = g_beat.rate_valid;
  g_beat.instant.beat_error_valid = g_beat.beat_error_valid;
  g_beat.instant.amplitude_valid = g_beat.amplitude_valid;
  g_beat.instant.beat_error_angle_valid = g_beat.beat_error_angle_valid;
  g_beat.instant.samples =
      (g_beat.rate_valid || g_beat.beat_error_valid || g_beat.amplitude_valid)
          ? 1u
          : 0u;
}

/**
 * Closes the open stretch without counting a cut: the next accepted beat
 * opens a new one and the no-watch window starts empty. For restarts that are
 * not a physical discontinuity (the fit changing its time base).
 */
void restart_stretch() {
  g_win_n = 0;
  g_win_head = 0;
  g_win_open = false;
  g_win_meta = {};
  g_reading = WindowReading{};
  g_reading_dirty = false;
  tick_shape_window_clear();
  g_stretch_open = false;
  g_settled_seen = false;
  g_watch_n = 0;
  g_watch_head = 0;
  g_beat.stretch_s = 0.f;
}

/**
 * Cuts the stream: ends the open stretch and discards what belongs to it, the
 * rolling means and the amplitude history. Counted only if a stretch was
 * open, so a lock after silence is not a cut. The caller resets the fit (a
 * restart inside {@link beat_fit_push} has already done so) and, when the
 * nominal rate changed, the templates ({@link restart_landmark_state}).
 *
 * @param reason Why; kept in {@code last_cut} when a stretch ended.
 */
void cut_stretch(dsp_cut_reason_t reason) {
  if (g_stretch_open) {
    g_beat.cuts++;
    g_beat.last_cut = reason;
    clear_timing_avg();
    g_amp_n = 0;
  }
  restart_stretch();
}

/**
 * Appends one tick event to the no-watch window, dropping the oldest when the
 * ring is full.
 *
 * @param t_ms Beat time (ms on the corrected clock).
 * @param miss Residual about the fit as a fraction of the half-period.
 * @param accepted The beat entered the fit.
 */
void watch_push(uint32_t t_ms, double miss, bool accepted) {
  const double q =
      fmax(-0.5, fmin(0.5, miss)) * static_cast<double>(kMissScale);
  const uint32_t slot = (g_watch_head + g_watch_n) % kWatchCap;
  if (g_watch_n == kWatchCap) {
    g_watch_head = (g_watch_head + 1u) % kWatchCap;
  } else {
    g_watch_n++;
  }
  g_watch[slot] = {t_ms, static_cast<int16_t>(lround(q)), accepted};
}

/**
 * The no-watch rule over the last {@link kNoWatchWindowS} s of the stretch:
 * true if fewer than {@link kNoWatchMinAccepted} of the expected beats were
 * accepted, or the RMS scatter of the events about the fit is at least
 * {@link kNoWatchScatter} of a half-period. The worst
 * 1/{@link kNoWatchTrimDiv} of the events are left out of the scatter.
 *
 * @param now_ms Time of the newest event, accepted or not (ms, corrected
 *        clock).
 * @param span_s Span of the open stretch (s).
 * @param vph Nominal A/h (beats per hour = expected events per hour).
 */
bool no_watch(uint32_t now_ms, double span_s, uint32_t vph) {
  const auto window_ms = static_cast<uint32_t>(kNoWatchWindowS * 1000.0);
  uint32_t n = 0, n_acc = 0;
  double sum_sq = 0.0;
  /* The m worst squares, descending; m is known once n is, so keep the most
   * kTrimMax and use the first n / kNoWatchTrimDiv of them. */
  float worst[kTrimMax] = {};
  for (uint32_t i = 1; i <= g_watch_n; i++) {
    const WatchEvent &e = g_watch[(g_watch_head + g_watch_n - i) % kWatchCap];
    if (now_ms - e.t_ms > window_ms) {
      break;
    }
    n++;
    n_acc += e.accepted ? 1u : 0u;
    const float m = static_cast<float>(e.miss_q) / kMissScale;
    const float sq = m * m;
    sum_sq += static_cast<double>(sq);
    float v = sq;
    for (uint32_t j = 0; j < kTrimMax && v > 0.f; j++) {
      if (v > worst[j]) {
        const float t = worst[j];
        worst[j] = v;
        v = t;
      }
    }
  }
  const double expected =
      fmin(span_s, kNoWatchWindowS) * static_cast<double>(vph) / 3600.0;
  if (static_cast<double>(n_acc) < kNoWatchMinAccepted * expected) {
    return true;
  }
  const uint32_t trim = n / kNoWatchTrimDiv;
  for (uint32_t j = 0; j < trim; j++) {
    sum_sq -= static_cast<double>(worst[j]);
  }
  if (n == trim) {
    return false;
  }
  const double rms = sqrt(fmax(0.0, sum_sq) / static_cast<double>(n - trim));
  return rms >= static_cast<double>(kNoWatchScatter);
}

/**
 * Drops everything that depends on the timestamp landmark: the fit (its
 * points are on the old time base), the drop state and the stage-B templates.
 * The rolling means are the caller's business: a change of landmark clears
 * them, a change of rate or clock already has.
 */
void restart_landmark_state() {
  restart_stretch();
  g_drop_active = false;
  g_tick_side = -1;
  g_side_xor = 0;
  g_lift_est_valid[0] = false;
  g_lift_est_valid[1] = false;
  beat_fit_reset(&g_fit);
  tick_shape_forget_templates();
}

/**
 * Drops the stored windows and the reading, for a parameter that changes what
 * they mean (lift angle, manual A/h). The open window keeps filling.
 */
void clear_window_readings() {
  g_win_n = 0;
  g_win_head = 0;
  g_reading = WindowReading{};
  g_reading_dirty = false;
}

/**
 * Stores the result of a closed window with the meta its closing event
 * captured. A window without any beat, or one that closed before the settling
 * time, is not stored.
 *
 * @param w Landmarks from {@link tick_shape_window_step}.
 */
void store_window(const tick_shape_window_t &w) {
  const WindowMeta meta = g_win_meta;
  g_win_meta = {};
  if (!meta.keep || w.beats[0] + w.beats[1] == 0u) {
    return;
  }
  const uint32_t slot = (g_win_head + g_win_n) % kWindowCap;
  if (g_win_n == kWindowCap) {
    g_win_head = (g_win_head + 1u) % kWindowCap;
  } else {
    g_win_n++;
  }
  WindowResult &r = g_windows[slot];
  r.t_ms = meta.t_ms;
  for (uint32_t i = 0; i < 2u; i++) {
    r.lift_ok[i] = w.valid[i];
    r.lift_s[i] =
        w.valid[i] ? static_cast<float>(w.lift_samples[i] / g_fs_true) : 0.f;
  }
  r.be_ok = meta.be_ok;
  r.be_side0_ms = meta.be_side0_ms;
  g_reading_dirty = true;
}

/**
 * Runs the pending window analysis to the end. Normally the idle steps
 * ({@link beat_analysis_on_block_end}) have done it long before the next
 * window closes; this is the bounded fallback for an overrun.
 */
void finish_pending_window() {
  tick_shape_window_t w;
  while (tick_shape_window_pending()) {
    if (tick_shape_window_step(&w)) {
      store_window(w);
    }
  }
}

/**
 * Recomputes the reading from the stored windows within the averaging span
 * (OTG M6): median of the window amplitudes, per-side medians, and the beat
 * error as the median of the windows' unlock beat error
 * ({@code BE_drop - lift_tick + lift_tock}, positive when Tick is late) with
 * the sign test of {@link kBeSignSlack}. A window closed within the last
 * {@code max(avg_s, kWindowS)} counts, so the newest window always does.
 * Bounded: at most {@link kWindowCap} windows and five medians of that many
 * values (about 2,500 comparisons in all).
 */
void update_reading() {
  WindowReading r;
  const uint32_t vph = g_history_vph;
  const auto span_ms = static_cast<uint32_t>(
      fmax(static_cast<double>(g_avg_s), kWindowS) * 1000.0);
  float amp[kWindowCap], tick[kWindowCap], tock[kWindowCap], be[kWindowCap];
  uint32_t n_amp = 0, n_tick = 0, n_tock = 0, n_be = 0;
  for (uint32_t i = 1; i <= g_win_n && vph != 0u; i++) {
    const WindowResult &w = g_windows[(g_win_head + g_win_n - i) % kWindowCap];
    if (g_last_event_ms - w.t_ms > span_ms) {
      break;
    }
    r.total++;
    float a[2] = {0.f, 0.f};
    bool ok[2] = {false, false};
    for (uint32_t s = 0; s < 2u; s++) {
      if (w.lift_ok[s]) {
        a[s] = amplitude_from_lift(w.lift_s[s], vph, g_lift_angle_deg);
        ok[s] = a[s] > kAmpMinDeg && a[s] < kAmpMaxDeg;
      }
    }
    if (ok[0] && ok[1]) {
      r.used++;
      amp[n_amp++] = 0.5f * (a[0] + a[1]);
    }
    if (g_tick_side < 0) {
      continue;
    }
    const auto ts = static_cast<uint32_t>(g_tick_side);
    const uint32_t ks = ts ^ 1u;
    if (ok[ts]) {
      tick[n_tick++] = a[ts];
    }
    if (ok[ks]) {
      tock[n_tock++] = a[ks];
    }
    if (w.be_ok && w.lift_ok[0] && w.lift_ok[1]) {
      const float drop_tick = ts == 0u ? w.be_side0_ms : -w.be_side0_ms;
      be[n_be++] = drop_tick - 1000.f * w.lift_s[ts] + 1000.f * w.lift_s[ks];
    }
  }
  if (n_amp > 0u) {
    r.amp_valid = true;
    r.amp_deg = median_mean_f32(amp, n_amp);
  }
  if (n_tick > 0u) {
    r.tick_valid = true;
    r.tick_deg = median_mean_f32(tick, n_tick);
  }
  if (n_tock > 0u) {
    r.tock_valid = true;
    r.tock_deg = median_mean_f32(tock, n_tock);
  }
  if (n_be >= kBeMinWindows) {
    float mean = 0.f;
    for (uint32_t i = 0; i < n_be; i++) {
      mean += be[i];
    }
    mean /= static_cast<float>(n_be);
    float var = 0.f;
    for (uint32_t i = 0; i < n_be; i++) {
      var += (be[i] - mean) * (be[i] - mean);
    }
    const float sd = sqrtf(var / static_cast<float>(n_be));
    const float m = median_mean_f32(be, n_be);
    uint32_t same = 0;
    for (uint32_t i = 0; i < n_be; i++) {
      same +=
          ((be[i] > 0.f) == (m > 0.f) && (be[i] < 0.f) == (m < 0.f)) ? 1u : 0u;
    }
    const float expect =
        sd > 0.f ? 0.5f * (1.f + erff(fabsf(m) / (sd * 1.41421356f))) : 1.f;
    if (static_cast<float>(same) / static_cast<float>(n_be) >=
        expect - kBeSignSlack) {
      r.be_ok = true;
      r.be_signed_ms = m;
    }
  }
  g_reading = r;
}

/**
 * Closes the open shape window at this event: seals the accumulators (O(1))
 * and keeps what the window needs from the fit now. The landmarks are found
 * later, one side per idle block.
 *
 * @param t_ms Time of this event (ms, corrected clock).
 * @param settled_for_windows The stretch is past {@code settleS}.
 */
void close_window(uint32_t t_ms, bool settled_for_windows) {
  finish_pending_window();
  g_win_meta = {settled_for_windows, t_ms, g_drop_active && g_drop_be_ok,
                g_drop_be_side0_ms};
  tick_shape_window_seal();
}

/**
 * Decides which template side is Tick: the side that comes late in the drop
 * beat error of the fit, once the fit runs on drop times and the stretch
 * spans {@link kProvisionalStretchS}. Within {@link kBeLabelMinMs} of zero
 * the side of the stretch's first beat is Tick. The label stays until the
 * nominal rate changes (templates are per physical side and survive grid
 * restarts).
 */
void decide_tick_side() {
  if (g_tick_side >= 0 || !g_drop_active || !g_drop_be_ok) {
    return;
  }
  if (fabsf(g_drop_be_side0_ms) >= kBeLabelMinMs) {
    g_tick_side = g_drop_be_side0_ms > 0.f ? 0 : 1;
  } else {
    g_tick_side = static_cast<int32_t>(g_side_xor & 1u);
  }
  g_reading_dirty = true;
}

/**
 * Chooses the timestamp that goes to the fit for this beat.
 *
 * With the drop landmark, stage B times the beat when its template is learned
 * and the match is good. A beat stage B cannot time is placed on the drop time
 * base by adding the side's running drop-minus-unlock estimate to its unlock;
 * before the fit has switched to drop times it keeps its unlock. Each such beat
 * is counted in {@code drop_fallbacks}.
 *
 * Handover: the fit starts on unlock times, so that rate and beat error are
 * published at once while the templates learn. When both sides have a
 * drop-minus-unlock estimate and stage B times a beat, the fit's stored points
 * are shifted by their side's estimate ({@link beat_fit_shift_points}) and
 * this and later beats use drop times. The handover is not a cut: the fit and
 * the stretch go on and the fit's rate is continuous. The provisional rate and
 * beat-error samples of the rolling means are dropped when the stretch becomes
 * settled, not here. {@code handovers} counts the handovers.
 *
 * @param ev Closed event.
 * @param t_unlock Sub-sample unlock time (source samples).
 * @param nominal_half Nominal half-period (source samples).
 * @return Timestamp for {@link beat_fit_push} (source samples).
 */
double beat_timestamp(const dsp_event_t &ev, double t_unlock,
                      double nominal_half) {
  g_beat_timed = false;
  if (!g_timing_drop) {
    return t_unlock;
  }
  /* Side of this beat from the grid slot its unlock time would take. */
  bool new_grid = false;
  const int32_t k_pred =
      beat_fit_predict_k(&g_fit, t_unlock, nominal_half, &new_grid);
  const auto side_guess =
      static_cast<uint8_t>((static_cast<uint32_t>(k_pred) ^ g_side_xor) & 1u);
  const tick_shape_result_t ts = tick_shape_measure(ev, side_guess, new_grid);
  if (new_grid) {
    g_side_xor = ts.side; /* a new grid starts at slot 0 (even parity) */
  }
  if (ts.valid) {
    g_beat_timed = true;
    g_beat.drop_timed_beats++;
    double &est = g_lift_est[ts.side];
    const double lift = ts.drop_sample - t_unlock;
    est =
        g_lift_est_valid[ts.side] ? est + kLiftEstWeight * (lift - est) : lift;
    g_lift_est_valid[ts.side] = true;
    if (!g_drop_active) {
      if (!g_lift_est_valid[0] || !g_lift_est_valid[1]) {
        return t_unlock; /* one side is still unknown: stay on unlock times */
      }
      /* Offsets by grid-absolute parity: side = parity ^ g_side_xor. */
      beat_fit_shift_points(&g_fit, g_lift_est[g_side_xor & 1u],
                            g_lift_est[(g_side_xor ^ 1u) & 1u]);
      g_drop_active = true;
      g_beat.handovers++;
    }
    return ts.drop_sample;
  }
  g_beat.drop_fallbacks++;
  if (g_drop_active && g_lift_est_valid[ts.side]) {
    return t_unlock + g_lift_est[ts.side];
  }
  return t_unlock;
}

/**
 * Evaluates the A/h lock from the interval history: the median interval gives
 * {@code interval_samples}, {@code vph_raw}, and the nearest common (or manual)
 * rate gives {@code locked}, {@code vph_nearest} and {@code confidence}.
 * Shared by the full engine and the lock-only mode (USB microphone).
 *
 * Preconditions: at least 4 intervals held, beat_rate enabled, timebase
 * verified (the callers check).
 *
 * @param source_hz Analysis sample rate of the interval history (Hz).
 */
void evaluate_lock(uint32_t source_hz) {
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
}

} // namespace

void beat_analysis_init() {
  g_analysis = true;
  g_lift_angle_deg = 51.0f;
  g_vph_manual = 0u;
  g_rate_window_s = 6.0f;
  g_avg_s = kAvgDefaultS;
  g_timebase_ok = true;
  g_timing_drop = false;
  g_settle_s = 0.0f;
  tick_shape_set_enabled(false);
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
  g_beat.last_cut = DSP_CUT_RESET;
  g_no_watch = false;
  g_last_accept_t = 0.0;
  g_last_ppm = 0.f;
  restart_stretch();
  clear_timing_avg();
  g_drop_active = false;
  g_tick_side = -1;
  g_event_in_block = false;
  g_side_xor = 0;
  g_lift_est_valid[0] = false;
  g_lift_est_valid[1] = false;
  __atomic_store_n(&g_beat_seq, 0u, __ATOMIC_RELEASE);
  g_grid_id = 0;
  g_history_vph = 0;
}

void beat_analysis_set_enabled(bool on) {
  if (on == g_analysis) {
    return;
  }
  g_analysis = on;
  beat_analysis_reset();
  tick_shape_set_enabled(on && g_timing_drop);
}

bool beat_analysis_enabled() { return g_analysis; }

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
  cut_stretch(DSP_CUT_CLOCK);
  clear_timing_avg();
  g_amp_n = 0;
  g_history_vph = 0;
  restart_landmark_state();
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
  if (!g_analysis) {
    /* USB microphone mode: the beat rate lock is all that runs. */
    if (!g_timebase_ok) {
      g_beat.locked = false;
      g_beat.vph_nearest = 0u;
    } else if (g_interval_n >= 4u && beat_rate_enabled) {
      evaluate_lock(source_hz);
    }
    return;
  }
  g_event_in_block = true;
  if (tick_shape_window_pending()) {
    finish_pending_window(); /* overrun: keeps the scratch free for the match */
  }

  g_beat.lift_angle_deg = g_lift_angle_deg;
  g_beat.rate_valid = false;
  g_beat.beat_error_valid = false;
  g_beat.amplitude_valid = false;
  g_beat.settled = false;
  g_beat.rate_s_day = 0.f;
  g_beat.beat_error_ms = 0.f;
  g_beat.amplitude_deg = 0.f;
  g_beat.lift_time_ms = 0.f;
  g_beat.rate_points = 0;
  g_beat.jitter_us = 0.f;
  g_beat.beat_error_angle_deg = 0.f;
  g_beat.beat_error_angle_valid = false;
  g_beat.beat_error_signed_ms = 0.f;
  g_beat.beat_error_source = DSP_BE_SOURCE_UNLOCK;

  auto finalize_windows = [&](bool push_sample) {
    /* Angle from this tick's BE and amplitude at the locked nominal A/h. */
    g_beat.beat_error_angle_valid = g_beat.beat_error_valid &&
                                    g_beat.amplitude_valid &&
                                    g_beat.vph_nearest != 0u;
    g_beat.beat_error_angle_deg =
        g_beat.beat_error_angle_valid
            ? beat_error_angle_deg(g_beat.amplitude_deg, g_beat.vph_nearest,
                                   g_beat.beat_error_ms)
            : 0.f;
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
    compute_timing_avg_window(now, kTimingAvg1sMs, g_history_vph,
                              &g_beat.avg_1s);
    compute_timing_avg_window(now,
                              static_cast<uint32_t>(lroundf(g_avg_s * 1000.f)),
                              g_history_vph, &g_beat.avg_long);
  };

  if (g_interval_n < 4u || !beat_rate_enabled || !g_timebase_ok) {
    if (!g_timebase_ok) {
      /* Unverified sample rate: refuse to lock rather than mis-time. */
      g_beat.locked = false;
      g_beat.vph_nearest = 0u;
    }
    beat_fit_reset(&g_fit);
    cut_stretch(DSP_CUT_LOCK_LOST);
    finalize_windows(false);
    return;
  }

  evaluate_lock(source_hz);

  if (!g_beat.locked) {
    beat_fit_reset(&g_fit);
    cut_stretch(DSP_CUT_LOCK_LOST);
    finalize_windows(false);
    return;
  }
  if (g_beat.vph_nearest != g_history_vph) {
    cut_stretch(DSP_CUT_RATE);
    clear_timing_avg();
    g_amp_n = 0;
    g_history_vph = g_beat.vph_nearest;
    restart_landmark_state();
  }

  /*
   * Rate + beat error from the fit. A change of nominal A/h restarts the
   * grid inside beat_fit_push. cal_ppm: a sample clock fast by p ppm counts
   * (1 + p·1e-6) samples per true second.
   */
  const double fs = static_cast<double>(source_hz);
  const double fs_true =
      fs * (1.0 + 1e-6 * static_cast<double>(timebase_ppm()));
  g_fs_true = fs_true;
  const double nominal_half =
      fs * 3600.0 / static_cast<double>(g_beat.vph_nearest);
  const double t_unlock =
      static_cast<double>(unlock) + static_cast<double>(ev.unlock_frac);
  const float ppm = timebase_ppm();
  if (g_stretch_open) {
    /* The push below then starts a new grid at this beat. */
    if (fabsf(ppm - g_last_ppm) > kClockCutPpm) {
      cut_stretch(DSP_CUT_CLOCK);
      beat_fit_reset(&g_fit);
    } else if (t_unlock - g_last_accept_t > kGapCutS * fs_true) {
      cut_stretch(DSP_CUT_GAP);
      beat_fit_reset(&g_fit);
    }
  }
  const double t_fit = beat_timestamp(ev, t_unlock, nominal_half);
  beat_fit_point_t pt{};
  const beat_fit_push_result_t push_res =
      beat_fit_push(&g_fit, t_fit, nominal_half,
                    static_cast<double>(g_rate_window_s) * fs, &pt);
  const bool beat_accepted = push_res != BEAT_FIT_REJECTED;
  if (push_res == BEAT_FIT_STARTED) {
    g_grid_id++;
    if (g_stretch_open) {
      /* Consecutive off-grid beats: the fit restarted at this one. */
      cut_stretch(DSP_CUT_JUMP);
    }
  }
  if (beat_accepted) {
    if (!g_stretch_open) {
      g_stretch_open = true;
      g_stretch_t0 = t_fit;
    }
    g_last_accept_t = t_fit;
    g_last_ppm = ppm;
  }
  const double stretch_samples = g_last_accept_t - g_stretch_t0;
  g_beat.stretch_s = static_cast<float>(stretch_samples / fs_true);
  const bool reading_ok = stretch_samples >= kProvisionalStretchS * fs_true;
  g_beat.settled = stretch_samples >= kMinStretchS * fs_true;
  if (g_beat.settled && !g_settled_seen) {
    g_settled_seen = true;
    clear_provisional_samples();
  }
  if (g_no_watch && stretch_samples >= kNoWatchClearS * fs_true) {
    g_no_watch = false;
  }
  beat_fit_result_t fit{};
  const bool solved = beat_fit_solve(&g_fit, &fit);
  if (solved) {
    const double true_half_s = fit.half_period / fs_true;
    const double nominal_half_s =
        3600.0 / static_cast<double>(g_beat.vph_nearest);
    /* Watch fast ⇒ measured period shorter than nominal ⇒ positive s/d. */
    g_beat.rate_points = fit.points;
    g_beat.jitter_us = static_cast<float>(fit.resid_rms / fs_true * 1e6);
    if (reading_ok) {
      g_beat.rate_s_day =
          static_cast<float>(beat_rate_s_day(nominal_half_s, true_half_s));
      g_beat.rate_valid = true;
      if (fit.alt_valid) {
        g_beat.beat_error_ms =
            static_cast<float>(2.0 * fabsf(fit.alt_offset) / fs_true * 1000.0);
        g_beat.beat_error_valid = true;
      }
    }
  }
  /*
   * Drop beat error of the fit in the sign convention of the template sides:
   * the fit's alternating term is relative to the window index parity, which
   * flips when the window slides by an odd number of beats ({@code k_base}),
   * and side = parity ^ g_side_xor.
   */
  g_drop_be_ok = solved && fit.alt_valid && g_drop_active;
  if (g_drop_be_ok) {
    const double c_even = (g_fit.k_base & 1) != 0
                              ? -static_cast<double>(fit.alt_offset)
                              : static_cast<double>(fit.alt_offset);
    const double c_side0 = (g_side_xor & 1u) != 0u ? -c_even : c_even;
    g_drop_be_side0_ms = static_cast<float>(2.0 * c_side0 / fs_true * 1000.0);
  }
  if (reading_ok) {
    decide_tick_side();
  }
  /* Beats the fit could not place count at the least a reject can miss by. */
  const double miss_frac =
      solved
          ? beat_fit_residual(&g_fit, fit, t_fit) / nominal_half
          : (beat_accepted ? 0.0 : static_cast<double>(BEAT_FIT_MAX_GRID_ERR));
  const auto t_fit_ms =
      static_cast<uint32_t>(static_cast<uint64_t>(t_fit / fs_true * 1000.0));
  watch_push(t_fit_ms, miss_frac, beat_accepted);
  g_last_event_ms = t_fit_ms;

  /*
   * Shape windows (drop mode): 2 s of beats from the stretch start, laid end
   * to end. The window that this beat falls beyond is sealed first; the beat
   * then joins the next one if stage B timed it.
   */
  if (g_timing_drop && beat_accepted) {
    if (!g_win_open) {
      g_win_open = true;
      g_win_end = g_stretch_t0 + kWindowS * fs_true;
    }
    if (t_fit >= g_win_end) {
      close_window(t_fit_ms, stretch_samples >=
                                 static_cast<double>(g_settle_s) * fs_true);
      do {
        g_win_end += kWindowS * fs_true;
      } while (t_fit >= g_win_end);
    }
    if (g_beat_timed) {
      (void)tick_shape_window_add();
    }
  }

  /* Signed beat error and its source (see dsp_be_source_t). */
  const bool drop_fit = g_timing_drop && g_drop_active;
  g_beat.beat_error_source = DSP_BE_SOURCE_UNLOCK;
  g_beat.beat_error_signed_ms = g_beat.beat_error_ms;
  if (drop_fit && g_beat.beat_error_valid) {
    if (g_reading.be_ok) {
      g_beat.beat_error_signed_ms = g_reading.be_signed_ms;
      g_beat.beat_error_ms = fabsf(g_reading.be_signed_ms);
    } else {
      g_beat.beat_error_source = DSP_BE_SOURCE_DROP;
      if (g_tick_side >= 0 && g_drop_be_ok) {
        const float tick_late =
            g_tick_side == 0 ? g_drop_be_side0_ms : -g_drop_be_side0_ms;
        g_beat.beat_error_signed_ms =
            tick_late < 0.f ? -g_beat.beat_error_ms : g_beat.beat_error_ms;
      }
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
    b.sample_index = g_timing_drop ? static_cast<uint64_t>(t_fit) : unlock;
    b.t_s = t_fit / fs;
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

  if (stretch_samples >= kNoWatchMinSpanS * fs_true &&
      no_watch(t_fit_ms, stretch_samples / fs_true, g_beat.vph_nearest)) {
    /* Not a watch: drop the lock; it may return once fresh intervals fit. */
    g_no_watch = true;
    cut_stretch(DSP_CUT_NO_WATCH);
    beat_fit_reset(&g_fit);
    g_interval_n = 0;
    g_beat.locked = false;
    g_beat.vph_nearest = 0u;
    g_beat.rate_valid = false;
    g_beat.beat_error_valid = false;
    g_beat.amplitude_valid = false;
    g_beat.settled = false;
    g_beat.rate_s_day = 0.f;
    g_beat.beat_error_ms = 0.f;
    g_beat.amplitude_deg = 0.f;
    g_beat.lift_time_ms = 0.f;
    finalize_windows(false);
    return;
  }

  finalize_windows((g_beat.rate_valid || g_beat.beat_error_valid ||
                    g_beat.amplitude_valid) &&
                   stretch_samples >=
                       static_cast<double>(g_settle_s) * fs_true);
}

void beat_analysis_on_block_end() {
  if (g_event_in_block) {
    g_event_in_block = false;
    return;
  }
  if (tick_shape_window_pending()) {
    tick_shape_window_t w;
    if (tick_shape_window_step(&w)) {
      store_window(w);
    }
    return;
  }
  if (g_reading_dirty) {
    g_reading_dirty = false;
    update_reading();
  }
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
  out->timing_drop = g_timing_drop && g_drop_active;
  out->analysis = g_analysis;
  /*
   * Shape-window readings (drop mode) replace the per-tick means in the
   * averaged view: amplitude from the window median, beat error from the
   * unlock windows when they gave one (the drop figure of the rolling mean
   * stays as the fallback). The instant view keeps single-beat values.
   */
  out->amplitude_tick_deg = g_reading.tick_valid ? g_reading.tick_deg : 0.f;
  out->amplitude_tock_deg = g_reading.tock_valid ? g_reading.tock_deg : 0.f;
  out->amplitude_tick_valid = g_reading.tick_valid;
  out->amplitude_tock_valid = g_reading.tock_valid;
  out->amplitude_windows_used = g_reading.used;
  out->amplitude_windows_total = g_reading.total;
  bool windowed = false;
  if (g_reading.amp_valid) {
    out->avg_long.amplitude_deg = g_reading.amp_deg;
    out->avg_long.amplitude_valid = true;
    windowed = true;
  }
  if (out->timing_drop && out->beat_error_valid &&
      out->beat_error_source == DSP_BE_SOURCE_UNLOCK) {
    out->avg_long.beat_error_ms = out->beat_error_ms;
    out->avg_long.beat_error_valid = true;
    windowed = true;
  }
  if (out->timing_drop && out->beat_error_valid &&
      out->beat_error_source == DSP_BE_SOURCE_DROP &&
      out->avg_long.beat_error_valid) {
    /* Fallback: the signed figure carries the magnitude of the averaged view,
     * so it agrees with avgLong.beatErrorMs. */
    out->beat_error_signed_ms =
        copysignf(out->avg_long.beat_error_ms, out->beat_error_signed_ms);
  }
  if (windowed) {
    out->avg_long.beat_error_angle_valid = out->avg_long.beat_error_valid &&
                                           out->avg_long.amplitude_valid &&
                                           g_history_vph != 0u;
    out->avg_long.beat_error_angle_deg =
        out->avg_long.beat_error_angle_valid
            ? beat_error_angle_deg(out->avg_long.amplitude_deg, g_history_vph,
                                   out->avg_long.beat_error_ms)
            : 0.f;
  }
  if (!g_timebase_ok || !g_have_last_event) {
    out->state = DSP_BEAT_STATE_LISTENING;
  } else if (!g_analysis) {
    out->state = g_beat.locked ? DSP_BEAT_STATE_LOCKED : DSP_BEAT_STATE_LOCKING;
  } else if (g_no_watch) {
    out->state = DSP_BEAT_STATE_NO_WATCH;
  } else if (g_beat.locked &&
             static_cast<double>(g_beat.stretch_s) >= kProvisionalStretchS) {
    out->state = DSP_BEAT_STATE_LOCKED;
  } else {
    out->state = DSP_BEAT_STATE_LOCKING;
  }
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
    clear_window_readings();
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
      clear_window_readings();
      return true;
    }
    if (v < 3600u || v > 72000u) {
      return false;
    }
    g_vph_manual = v;
    clear_timing_avg();
    clear_window_readings();
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
  if (strcmp(param, "timingLandmark") == 0 ||
      strcmp(param, "timing_landmark") == 0) {
    if (value != 0.f && value != 1.f) {
      return false;
    }
    const bool drop = value == 1.f;
    if (drop != g_timing_drop) {
      g_timing_drop = drop;
      tick_shape_set_enabled(drop && g_analysis);
      restart_landmark_state();
      clear_timing_avg();
    }
    return true;
  }
  if (strcmp(param, "settleS") == 0 || strcmp(param, "settle_s") == 0) {
    if (!(value >= 0.f && value <= kSettleMaxS)) {
      return false;
    }
    g_settle_s = value;
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
  if (strcmp(param, "settleS") == 0 || strcmp(param, "settle_s") == 0) {
    *out = g_settle_s;
    return true;
  }
  if (strcmp(param, "timingLandmark") == 0 ||
      strcmp(param, "timing_landmark") == 0) {
    *out = g_timing_drop ? 1.f : 0.f;
    return true;
  }
  return false;
}
