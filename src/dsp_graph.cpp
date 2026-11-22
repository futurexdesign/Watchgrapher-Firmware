/**
 * @file dsp_graph.cpp
 * @brief Float32 DSP graph: analysis, process path, PCM24 quantize, USB
 * watchdog.
 *
 * Capture pushes float full-scale frames here; we never touch the USB feeder
 * directly. Analysis runs on every block. Process (notch → filter → soft_gate →
 * gain → AGC → limiter) runs only when master bypass is off. Output is signed
 * PCM24 in int32 containers for {@code audio_ring}.
 *
 * Continuity: enabling a heavy process block arms a short attribution window.
 * If ring underruns or USB short-writes spike in that window, the block is
 * auto-ejected and an async CLI error string is queued.
 *
 * Mechanical escapement analysis lives in {@code src/mechanical/}: the tick
 * detector ({@code event_detect}, mechanical/tick_detect.h) and the timing
 * estimate ({@code beat_rate}, mechanical/beat_analysis.h). This file keeps
 * the generic parts they depend on: meters, noise floor, the event-detect
 * threshold (shared with soft_gate), the event log, and the matched filter.
 *
 * @see dsp_graph.h, control_api.cpp, mechanical/tick_detect.h,
 *      mechanical/beat_analysis.h
 */

#include "dsp_graph.h"

#include "app_config.h"
#include "diagnostics.h"
#include "dsp_blocks.h"
#include "mechanical/beat_analysis.h"
#include "mechanical/tick_detect.h"
#include "perf_timing.h"
#if AUDIO_SOURCE == PCM1808
#include "quartz/quartz_engine.h"
#include "timebase.h"
#endif

#include <Arduino.h>
#include <arm_math.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 2.f * kPi;

bool g_bypass = true;
bool g_en[DSP_BLOCK_COUNT];
/** Sample rate of graph input (may be < 48 kHz on PCM1808 pre-upsample path). */
uint32_t g_source_hz = DSP_SAMPLE_RATE_HZ;

float g_gain_lin = 1.0f; /* linear gain; 1.0 = 0 dB */
dsp_filter_preset_t g_filter = DSP_FILTER_BYPASS;
float g_event_threshold = 6.0f; /* detect when |x| > noise * threshold */
float g_preroll_ms = 25.0f;
float g_tail_ms = 25.0f;
float g_source_hz_measured = 0.f;

arm_biquad_casd_df1_inst_f32 g_bq;
float g_bq_state[8]; /* 2 stages × 4 DF1 state words */
float g_bq_coeffs[10];
bool g_bq_active = false;

/* Mains notch (1 biquad stage). */
arm_biquad_casd_df1_inst_f32 g_notch;
float g_notch_state[4];
float g_notch_coeffs[5];
bool g_notch_active = false;
float g_notch_hz = 60.f; /* 50 or 60 */

/* Slow AGC. */
float g_agc_lin = 1.0f;
float g_agc_target = 0.35f; /* desired tick peak FS */
float g_agc_tau_s = 1.5f;   /* time constant (seconds) */
float g_agc_min_lin = 0.25f; /* -12 dB */
float g_agc_max_lin = 4.0f;  /* +12 dB */

/* Soft gate (transient-aware). */
float g_gate_lin = 1.0f;
float g_gate_floor_lin = 0.25f; /* -12 dB between ticks */
float g_gate_attack_ms = 1.0f;
float g_gate_release_ms = 40.0f;

/* Matched filter (analysis). */
constexpr uint32_t kMatchLen = 64u;
static_assert(kMatchLen == TICK_DETECT_WAVE_LEN,
              "matched-filter template learns from the detector's wave");
float g_match_tmpl[kMatchLen];
float g_match_hist[kMatchLen];
float g_match_score = 0.f;
float g_match_threshold = 0.35f;
bool g_match_learned = false;
bool g_match_learn = true;
float g_match_tmpl_energy = 1.f;
float g_match_tmpl_peak = 0.f;

/* FIR bank (process). */
constexpr uint32_t kFirTaps = 64u;
arm_fir_instance_f32 g_fir;
float g_fir_coeffs[kFirTaps];
float g_fir_state[kFirTaps + DSP_MAX_BLOCK];
bool g_fir_active = false;
dsp_filter_preset_t g_fir_preset = DSP_FILTER_BYPASS;

/* Spectrum (analysis). */
constexpr uint32_t kFftN = 512u;
arm_rfft_fast_instance_f32 g_rfft;
float g_fft_acc[kFftN];
float g_fft_work[kFftN];
/* arm_rfft_fast_f32 does not support in-place: separate output buffer. */
float g_fft_out[kFftN];
float g_fft_mags[kFftN / 2u];
float g_fft_window[kFftN];
uint32_t g_fft_fill = 0;
uint32_t g_fft_stride = 1;
uint32_t g_fft_stride_i = 0;
bool g_rfft_ok = false;
dsp_spectrum_t g_spectrum = {};

/* Block work buffer for dsp_graph_process (kept off the 4 KB core-0 stack). */
float g_work[DSP_MAX_BLOCK];

float g_noise = 1e-4f;
float g_pre_peak = 0.f;
float g_post_peak = 0.f;
float g_pre_rms_ema = 0.f;
float g_post_rms_ema = 0.f;
uint32_t g_clip = 0;
uint32_t g_lim = 0;

dsp_event_t g_events[DSP_EVENT_LOG_CAP];
uint32_t g_event_count = 0;
uint32_t g_event_head = 0;

dsp_error_t g_errors[DSP_ERROR_LOG_CAP];
uint32_t g_err_count = 0;
uint32_t g_err_head = 0;
char g_async_msg[96];
bool g_async_pending = false;

/* Continuity watchdog: attributes USB faults to the most recent enable. */
constexpr uint32_t kWatchWindowMs = 5000u;
constexpr uint32_t kWatchCooldownMs = 2000u;
constexpr uint32_t kWatchShortWriteSlack = 50u;
dsp_block_id_t g_last_enabled = DSP_BLOCK_BYPASS;
bool g_watch_armed = false;
uint32_t g_enable_ms = 0;
uint32_t g_underrun_at_enable = 0;
uint32_t g_short_at_enable = 0;
bool g_watch_cooling = false;
uint32_t g_watch_cooldown_ms = 0;

/** Accepted range for the gain block (dB). */
constexpr float kGainDbMin = -40.f;
constexpr float kGainDbMax = 40.f;

/** Decibels → linear amplitude ({@code 10^(db/20)}). */
float db_to_lin(float db) { return powf(10.0f, db / 20.0f); }

/**
 * Appends a fault, advances the error ring, and queues a one-shot CLI string
 * ({@code CLI: ERR dsp eject ...}) for {@link dsp_graph_pop_async_message}.
 *
 * @param block Block blamed for the fault (usually the last enabled one).
 * @param reason Underrun / USB stall / overload.
 * @param sample Source index hint (0 when unknown).
 */
void push_error(dsp_block_id_t block, dsp_error_reason_t reason, uint64_t sample) {
  g_errors[g_err_head] = {block, reason, millis(), sample};
  g_err_head = (g_err_head + 1u) % DSP_ERROR_LOG_CAP;
  if (g_err_count < DSP_ERROR_LOG_CAP) {
    g_err_count++;
  }
  const char *why = reason == DSP_ERR_UNDERRUN     ? "underrun"
                    : reason == DSP_ERR_USB_STALL ? "usb_stall"
                                                  : "overload";
  snprintf(g_async_msg, sizeof(g_async_msg),
           "CLI: ERR dsp eject block=%s reason=%s", dsp_block_name(block), why);
  g_async_pending = true;
}

/**
 * Stores a completed analysis event in the event log and forwards it to the
 * mechanical timing analysis ({@link beat_analysis_on_event}).
 *
 * @param ev Closed event window from the detector.
 */
void push_event(const dsp_event_t &ev) {
  g_events[g_event_head] = ev;
  g_event_head = (g_event_head + 1u) % DSP_EVENT_LOG_CAP;
  if (g_event_count < DSP_EVENT_LOG_CAP) {
    g_event_count++;
  }
  beat_analysis_on_event(ev, g_source_hz, g_en[DSP_BLOCK_BEAT_RATE]);
}

/**
 * Designs the HPF+LPF biquad pair for {@code preset}, or deactivates the
 * filter when preset is bypass / bank disabled.
 *
 * Coeff layout matches CMSIS DF1: {@code b0,b1,b2,a1,a2} per stage. This is a
 * simple bilinear one-pole pair — good enough for tick isolation, not a
 * precision EQ.
 */
void design_biquad_pair(dsp_filter_preset_t preset) {
  g_bq_active = false;
  if (preset == DSP_FILTER_BYPASS) {
    return;
  }
  float hp = 160.f, lp = 16000.f;
  if (preset == DSP_FILTER_MEDIUM) {
    hp = 400.f;
    lp = 12000.f;
  } else if (preset == DSP_FILTER_NARROW) {
    hp = 800.f;
    lp = 8000.f;
  }
  const auto fs = static_cast<float>(g_source_hz);
  auto one_pole_hp = [&](float fc, float *c) {
    float x = expf(-kTwoPi * fc / fs);
    c[0] = (1.f + x) * 0.5f;
    c[1] = -(1.f + x) * 0.5f;
    c[2] = 0.f;
    c[3] = x;
    c[4] = 0.f;
  };
  auto one_pole_lp = [&](float fc, float *c) {
    float x = expf(-kTwoPi * fc / fs);
    c[0] = 1.f - x;
    c[1] = 0.f;
    c[2] = 0.f;
    c[3] = x;
    c[4] = 0.f;
  };
  one_pole_hp(hp, &g_bq_coeffs[0]);
  one_pole_lp(lp, &g_bq_coeffs[5]);
  memset(g_bq_state, 0, sizeof(g_bq_state));
  arm_biquad_cascade_df1_init_f32(&g_bq, 2, g_bq_coeffs, g_bq_state);
  g_bq_active = true;
}

/**
 * Designs a narrow RBJ notch at {@code g_notch_hz} using {@code g_source_hz}.
 * CMSIS DF1 stores feedback coeffs with opposite sign to the TF denominator.
 */
void design_notch(void) {
  g_notch_active = false;
  if (g_notch_hz < 40.f || g_notch_hz > 70.f) {
    return;
  }
  const auto fs = static_cast<float>(g_source_hz);
  const float w0 = kTwoPi * g_notch_hz / fs;
  const float cosw = cosf(w0);
  const float Q = 30.f;
  const float alpha = sinf(w0) / (2.f * Q);
  const float a0 = 1.f + alpha;
  g_notch_coeffs[0] = 1.f / a0;
  g_notch_coeffs[1] = (-2.f * cosw) / a0;
  g_notch_coeffs[2] = 1.f / a0;
  g_notch_coeffs[3] = (2.f * cosw) / a0;       /* CMSIS a1 = -a1_tf */
  g_notch_coeffs[4] = -(1.f - alpha) / a0;      /* CMSIS a2 = -a2_tf */
  memset(g_notch_state, 0, sizeof(g_notch_state));
  arm_biquad_cascade_df1_init_f32(&g_notch, 1, g_notch_coeffs, g_notch_state);
  g_notch_active = true;
}

/** Hamming-window FIR bandpass (linear phase), unity-ish midband gain. */
void design_fir(dsp_filter_preset_t preset) {
  g_fir_active = false;
  if (preset == DSP_FILTER_BYPASS) {
    return;
  }
  float hp = 160.f, lp = 16000.f;
  if (preset == DSP_FILTER_MEDIUM) {
    hp = 400.f;
    lp = 12000.f;
  } else if (preset == DSP_FILTER_NARROW) {
    hp = 800.f;
    lp = 8000.f;
  }
  const auto fs = static_cast<float>(g_source_hz);
  const float ny = 0.5f * fs;
  if (lp > ny * 0.98f) {
    lp = ny * 0.98f;
  }
  if (hp < 1.f) {
    hp = 1.f;
  }
  if (hp >= lp) {
    return;
  }
  const float fc1 = hp / fs;
  const float fc2 = lp / fs;
  const auto M = static_cast<int>(kFirTaps);
  const auto mid = 0.5f * static_cast<float>(M - 1);
  for (int n = 0; n < M; n++) {
    auto k = static_cast<float>(n) - mid;
    float ideal;
    if (fabsf(k) < 1e-6f) {
      ideal = 2.f * (fc2 - fc1);
    } else {
      ideal = (sinf(kTwoPi * fc2 * k) - sinf(kTwoPi * fc1 * k)) /
              (kPi * k);
    }
    const float w =
        0.54f - 0.46f * cosf(kTwoPi * static_cast<float>(n) / static_cast<float>(M - 1));
    g_fir_coeffs[n] = ideal * w;
  }
  /*
   * Normalize to unit coefficient energy (bandpass DC gain is ~0, so DC
   * normalization is meaningless) — keeps midband impulse energy moderate.
   */
  float energy = 0.f;
  for (uint32_t i = 0; i < kFirTaps; i++) {
    energy += g_fir_coeffs[i] * g_fir_coeffs[i];
  }
  if (energy > 1e-18f) {
    const float s = 1.f / sqrtf(energy);
    for (uint32_t i = 0; i < kFirTaps; i++) {
      g_fir_coeffs[i] *= s;
    }
  }
  memset(g_fir_state, 0, sizeof(g_fir_state));
  arm_fir_init_f32(&g_fir, kFirTaps, g_fir_coeffs, g_fir_state, DSP_MAX_BLOCK);
  g_fir_active = true;
}

void match_update_hist(const float *x, uint32_t n) {
  if (n >= kMatchLen) {
    memcpy(g_match_hist, x + (n - kMatchLen), kMatchLen * sizeof(float));
    return;
  }
  memmove(g_match_hist, g_match_hist + n, (kMatchLen - n) * sizeof(float));
  memcpy(g_match_hist + (kMatchLen - n), x, n * sizeof(float));
}

float match_score_block(const float *x, uint32_t n) {
  if (!g_match_learned || g_match_tmpl_energy < 1e-12f) {
    return 0.f;
  }
  float best = 0.f;
  float window[kMatchLen];
  /* Build contiguous view: hist tail + block (cap work). */
  const uint32_t total = kMatchLen + n;
  for (uint32_t i = 0; i + kMatchLen <= total; i++) {
    for (uint32_t k = 0; k < kMatchLen; k++) {
      uint32_t idx = i + k;
      window[k] = (idx < kMatchLen) ? g_match_hist[idx] : x[idx - kMatchLen];
    }
    float dot = 0.f;
    arm_dot_prod_f32(g_match_tmpl, window, kMatchLen, &dot);
    float sc = fabsf(dot) / g_match_tmpl_energy;
    if (sc > best) {
      best = sc;
    }
  }
  return best;
}

void match_learn_from_wave(void) {
  uint32_t wave_n = 0;
  const float *wave = tick_detect_wave(&wave_n);
  if (!g_match_learn || wave_n < (kMatchLen / 4u)) {
    return;
  }
  float tmp[kMatchLen];
  memset(tmp, 0, sizeof(tmp));
  uint32_t n = wave_n < kMatchLen ? wave_n : kMatchLen;
  /* Center on peak inside wave. */
  uint32_t peak_i = 0;
  float peak = 0.f;
  for (uint32_t i = 0; i < n; i++) {
    float a = fabsf(wave[i]);
    if (a > peak) {
      peak = a;
      peak_i = i;
    }
  }
  auto start = static_cast<int32_t>(peak_i) - static_cast<int32_t>(kMatchLen / 2u);
  for (uint32_t i = 0; i < kMatchLen; i++) {
    auto src = start + static_cast<int32_t>(i);
    tmp[i] = (src >= 0 && src < static_cast<int32_t>(n)) ? wave[src] : 0.f;
  }
  float energy = 0.f;
  arm_power_f32(tmp, kMatchLen, &energy);
  energy = sqrtf(fmaxf(energy, 1e-12f));
  for (uint32_t i = 0; i < kMatchLen; i++) {
    tmp[i] /= energy;
  }
  if (!g_match_learned) {
    memcpy(g_match_tmpl, tmp, sizeof(tmp));
    g_match_learned = true;
  } else {
    for (uint32_t i = 0; i < kMatchLen; i++) {
      g_match_tmpl[i] = 0.9f * g_match_tmpl[i] + 0.1f * tmp[i];
    }
  }
  arm_power_f32(g_match_tmpl, kMatchLen, &g_match_tmpl_energy);
  g_match_tmpl_energy = sqrtf(fmaxf(g_match_tmpl_energy, 1e-12f));
  g_match_tmpl_peak = 0.f;
  for (uint32_t i = 0; i < kMatchLen; i++) {
    float a = fabsf(g_match_tmpl[i]);
    if (a > g_match_tmpl_peak) {
      g_match_tmpl_peak = a;
    }
  }
}

void spectrum_push(const float *x, uint32_t n) {
  if (!g_en[DSP_BLOCK_SPECTRUM] || !g_rfft_ok) {
    return;
  }
  for (uint32_t i = 0; i < n; i++) {
    g_fft_acc[g_fft_fill++] = x[i];
    if (g_fft_fill < kFftN) {
      continue;
    }
    g_fft_fill = 0;
    g_fft_stride_i++;
    if ((g_fft_stride_i % g_fft_stride) != 0u) {
      continue;
    }
    /* Hann window into work buffer, then RFFT in-place. */
    for (uint32_t k = 0; k < kFftN; k++) {
      g_fft_work[k] = g_fft_acc[k] * g_fft_window[k];
    }
    /* g_fft_work is clobbered (CMSIS uses it as scratch); result in g_fft_out. */
    arm_rfft_fast_f32(&g_rfft, g_fft_work, g_fft_out, 0);
    /* Magnitudes: DC at [0], Nyquist at [1], then pairs. */
    float *mags = g_fft_mags;
    mags[0] = fabsf(g_fft_out[0]);
    mags[kFftN / 2u - 1u] = fabsf(g_fft_out[1]);
    for (uint32_t k = 1; k < kFftN / 2u - 1u; k++) {
      float re = g_fft_out[2u * k];
      float im = g_fft_out[2u * k + 1u];
      mags[k] = sqrtf(re * re + im * im);
    }
    g_spectrum = {};
    g_spectrum.fft_n = kFftN;
    g_spectrum.source_hz = g_source_hz;
    g_spectrum.valid = true;
    for (uint32_t p = 0; p < 8; p++) {
      uint32_t best = 1; /* skip DC */
      float bestm = -1.f;
      for (uint32_t k = 1; k < kFftN / 2u - 1u; k++) {
        if (mags[k] > bestm) {
          bestm = mags[k];
          best = k;
        }
      }
      if (bestm <= 0.f) {
        break;
      }
      g_spectrum.peak_hz[p] =
          static_cast<float>(best) * static_cast<float>(g_source_hz) / static_cast<float>(kFftN);
      g_spectrum.peak_mag[p] = bestm;
      g_spectrum.n_peaks = p + 1u;
      mags[best] = 0.f; /* suppress for next peak */
    }
  }
}

/**
 * Close callback for {@link tick_detect_block}: blends the matched-filter
 * score into the event confidence, logs the event and feeds the timing
 * analysis ({@link push_event}), then lets the matched filter learn from the
 * event's captured wave while {@link tick_detect_wave} still holds it.
 *
 * @param detected Closed event from the detector.
 */
void on_tick_event(const dsp_event_t &detected) {
  dsp_event_t ev = detected;
  if (g_en[DSP_BLOCK_MATCHED_FILTER] && g_match_learned) {
    ev.confidence = fminf(1.f, 0.5f * ev.confidence + 0.5f * g_match_score);
  }
  push_event(ev);
  if (g_en[DSP_BLOCK_MATCHED_FILTER]) {
    match_learn_from_wave();
  }
}

/**
 * Analysis path: meters, noise EMA, matched-filter score, spectrum, then the
 * mechanical tick detector ({@link tick_detect_block}), which feeds the event
 * log and beat analysis through {@link on_tick_event}.
 *
 * Does not modify {@code x}. Watch ticks are impulsive: open on a rise above
 * {@code noise * threshold}, close with hysteresis or a max event duration so
 * a too-low threshold cannot latch the detector open forever.
 *
 * @param x Float full-scale block (read-only).
 * @param n Sample count.
 * @param base Absolute source index of {@code x[0]}.
 */
void analyze(const float *x, uint32_t n, uint64_t base) {
  float peak = 0.f;
  float power = 0.f;
  arm_power_f32(const_cast<float *>(x), n, &power);
  for (uint32_t i = 0; i < n; i++) {
    float a = fabsf(x[i]);
    if (a > peak) {
      peak = a;
    }
  }
  float rms = sqrtf(power / static_cast<float>(n));
  g_pre_peak = fmaxf(g_pre_peak * 0.95f, peak);
  g_pre_rms_ema = 0.9f * g_pre_rms_ema + 0.1f * rms;

  if (g_en[DSP_BLOCK_MATCHED_FILTER]) {
    g_match_score = match_score_block(x, n);
  } else {
    g_match_score = 0.f;
  }
  spectrum_push(x, n);

  if (g_en[DSP_BLOCK_NOISE_FLOOR] || g_en[DSP_BLOCK_EVENT_DETECT] ||
      g_en[DSP_BLOCK_BEAT_RATE]) {
    /*
     * Only adapt noise on quiet blocks. Tick blocks would inflate the floor.
     * Floor open thr with a multiple of pre-RMS so background never latches.
     */
    const float open_guess =
        fmaxf(g_noise * g_event_threshold, fmaxf(4.f * g_pre_rms_ema, 1e-3f));
    if (!tick_detect_in_event() && peak < open_guess) {
      g_noise = 0.995f * g_noise + 0.005f * fmaxf(rms, 1e-7f);
      if (g_noise < 1e-6f) {
        g_noise = 1e-6f;
      }
    }
  }

  if (!g_en[DSP_BLOCK_EVENT_DETECT] && !g_en[DSP_BLOCK_BEAT_RATE]) {
    return;
  }

  const tick_detect_block_params_t params{
      fmaxf(g_noise * g_event_threshold, fmaxf(4.f * g_pre_rms_ema, 1e-3f)),
      g_noise,
      g_preroll_ms,
      g_tail_ms,
      g_source_hz,
      /* Matched-filter gate: a learned template must agree before opening. */
      g_en[DSP_BLOCK_MATCHED_FILTER] && g_match_learned &&
          g_match_score < g_match_threshold,
  };
  tick_detect_block(x, n, base, params, on_tick_event);
  match_update_hist(x, n);
}

/**
 * Process path: notch → filter → soft_gate → gain → AGC → limiter ±1.0.
 * No-op while master bypass is on. Mutates {@code x} in place.
 *
 * Soft gate opens quickly on impulsive peaks (transient preserve) and
 * releases toward a floor between ticks. AGC is a slow peak follower.
 *
 * @param x Float work buffer (length {@code n}).
 * @param n Sample count.
 */
void process_path(float *x, uint32_t n) {
  if (g_bypass) {
    return;
  }
  if (g_en[DSP_BLOCK_NOTCH] && g_notch_active) {
    arm_biquad_cascade_df1_f32(&g_notch, x, x, n);
  }
  if (g_en[DSP_BLOCK_FILTER_BANK] && g_bq_active) {
    arm_biquad_cascade_df1_f32(&g_bq, x, x, n);
  }
  if (g_en[DSP_BLOCK_FIR_BANK] && g_fir_active) {
    arm_fir_f32(&g_fir, x, x, n);
  }
  if (g_en[DSP_BLOCK_SOFT_GATE]) {
    const auto fs = static_cast<float>(g_source_hz);
    const float atk =
        1.f - expf(-1.f / fmaxf(g_gate_attack_ms * 0.001f * fs, 1.f));
    const float rel =
        1.f - expf(-1.f / fmaxf(g_gate_release_ms * 0.001f * fs, 1.f));
    const float open_thr =
        fmaxf(g_noise * g_event_threshold, fmaxf(4.f * g_pre_rms_ema, 1e-3f));
    for (uint32_t i = 0; i < n; i++) {
      const float a = fabsf(x[i]);
      const float target = (a > open_thr) ? 1.f : g_gate_floor_lin;
      const float coeff = (target > g_gate_lin) ? atk : rel;
      g_gate_lin += coeff * (target - g_gate_lin);
      x[i] *= g_gate_lin;
    }
  }
  if (g_en[DSP_BLOCK_GAIN] && g_gain_lin != 1.0f) {
    for (uint32_t i = 0; i < n; i++) {
      x[i] *= g_gain_lin;
    }
  }
  if (g_en[DSP_BLOCK_AGC]) {
    float peak = 0.f;
    for (uint32_t i = 0; i < n; i++) {
      float a = fabsf(x[i]);
      if (a > peak) {
        peak = a;
      }
    }
    /* Adapt only on energetic blocks (ticks), ignore quiet frames. */
    if (peak > fmaxf(4.f * g_pre_rms_ema, 1e-3f)) {
      const auto fs = static_cast<float>(g_source_hz);
      const float alpha =
          1.f - expf(-(static_cast<float>(n) / fs) / fmaxf(g_agc_tau_s, 0.05f));
      const float desired = g_agc_target / fmaxf(peak, 1e-6f);
      float next = g_agc_lin + alpha * (desired - g_agc_lin);
      if (next < g_agc_min_lin) {
        next = g_agc_min_lin;
      } else if (next > g_agc_max_lin) {
        next = g_agc_max_lin;
      }
      g_agc_lin = next;
    }
    for (uint32_t i = 0; i < n; i++) {
      x[i] *= g_agc_lin;
    }
  }
  if (g_en[DSP_BLOCK_LIMITER]) {
    for (uint32_t i = 0; i < n; i++) {
      if (x[i] > 1.f) {
        x[i] = 1.f;
        g_lim++;
      } else if (x[i] < -1.f) {
        x[i] = -1.f;
        g_lim++;
      }
    }
  }
}

} // namespace

/*
 * Quartz engine hooks. The engine (quartz/quartz_engine.h) follows the same
 * stream / rate / timebase events as the graph, so those events are
 * forwarded from the dsp_graph functions below; the graph's own logic is
 * unchanged. It exists only on the PCM1808 path (the only source with the
 * RIGHT channel and a measured, verified clock); elsewhere the hooks are
 * empty and the engine is never referenced, so --gc-sections drops it.
 */
namespace {
#if AUDIO_SOURCE == PCM1808

/** @return The timebase correction in effect, for the engine. */
quartz::engine::Calibration quartz_calibration() noexcept {
  const timebase_source_t src = timebase_source();
  return {timebase_ppm(), timebase_source_name(src), src != TIMEBASE_SRC_NONE};
}

/** @return The µs timer, for the engine's per-block timing. */
uint32_t quartz_now_us() noexcept { return time_us_32(); }

/** Forwards the engine's per-block time to perf_timing. */
void quartz_block_timed(uint32_t us) noexcept {
  perf_timing_note_quartz_block(us);
}

/** Installs the firmware hooks and resets the engine (from init). */
void quartz_hook_init() {
  quartz::engine::Hooks hooks{};
  hooks.calibration = quartz_calibration;
  hooks.now_us = quartz_now_us;
  hooks.block_timed = quartz_block_timed;
  quartz::engine::init(hooks);
}
/** Restarts the quartz measurement with the stream. */
void quartz_hook_reset_stream() { quartz::engine::reset_stream(); }
/** @param hz Effective source rate (Hz), after the graph's 0 → default. */
void quartz_hook_set_source_rate(uint32_t hz) {
  quartz::engine::set_source_rate(hz);
}
/** @param ok Capture rate verified. */
void quartz_hook_set_timebase(bool ok) { quartz::engine::set_timebase(ok); }

#else

/* No quartz engine on this source: the hooks compile to nothing. */
void quartz_hook_init() {}
void quartz_hook_reset_stream() {}
void quartz_hook_set_source_rate(uint32_t) {}
void quartz_hook_set_timebase(bool) {}

#endif
} // namespace

float dsp_sample_from_pcm1808_left24(int32_t left24) {
  return static_cast<float>(left24) * (1.0f / 8388608.0f);
}

void dsp_graph_reset_stream(void) {
  tick_detect_reset();
  g_event_count = 0;
  g_event_head = 0;
  beat_analysis_reset();

  g_noise = 1e-4f;
  g_pre_peak = 0.f;
  g_post_peak = 0.f;
  g_pre_rms_ema = 0.f;
  g_post_rms_ema = 0.f;
  g_agc_lin = 1.0f;
  g_gate_lin = 1.0f;
  g_match_score = 0.f;
  memset(g_match_hist, 0, sizeof(g_match_hist));
  g_fft_fill = 0;
  g_fft_stride_i = 0;
  g_spectrum = {};

  memset(g_bq_state, 0, sizeof(g_bq_state));
  memset(g_notch_state, 0, sizeof(g_notch_state));
  memset(g_fir_state, 0, sizeof(g_fir_state));
  quartz_hook_reset_stream();
}

void dsp_graph_init(void) {
  memset(g_en, 0, sizeof(g_en));
  g_en[DSP_BLOCK_LEVEL_METER] = true;
  g_en[DSP_BLOCK_EVENT_DETECT] = true;
  g_en[DSP_BLOCK_BEAT_RATE] = true;
  g_en[DSP_BLOCK_NOISE_FLOOR] = true;
  g_en[DSP_BLOCK_LIMITER] = true; /* sat safety when gain/filter on */
  g_bypass = true;
  g_source_hz = DSP_SAMPLE_RATE_HZ;
  g_gain_lin = 1.0f;
  g_agc_lin = 1.0f;
  g_gate_lin = 1.0f;
  g_notch_hz = 60.f;
  g_filter = DSP_FILTER_BYPASS;
  g_fir_preset = DSP_FILTER_BYPASS;
  design_biquad_pair(g_filter);
  design_notch();
  design_fir(g_fir_preset);
  g_rfft_ok = (arm_rfft_fast_init_f32(&g_rfft, kFftN) == ARM_MATH_SUCCESS);
  for (uint32_t k = 0; k < kFftN; k++) {
    g_fft_window[k] = 0.5f - 0.5f * cosf(kTwoPi * static_cast<float>(k) /
                                         static_cast<float>(kFftN - 1u));
  }
  g_fft_fill = 0;
  g_fft_stride = 1;
  g_fft_stride_i = 0;
  g_spectrum = {};
  memset(g_match_tmpl, 0, sizeof(g_match_tmpl));
  memset(g_match_hist, 0, sizeof(g_match_hist));
  g_match_learned = false;
  g_match_learn = true;
  g_match_score = 0.f;
  g_match_threshold = 0.35f;
  g_noise = 1e-4f;
  g_event_count = 0;
  g_event_head = 0;
  g_source_hz_measured = 0.f;
  tick_detect_reset();
  beat_analysis_init();
  g_err_count = 0;
  g_err_head = 0;
  g_async_pending = false;
  quartz_hook_init();
}

void dsp_graph_set_source_rate(uint32_t hz) {
  if (hz == 0) {
    hz = DSP_SAMPLE_RATE_HZ;
  }
  g_source_hz = hz;
  /* Coeffs depend on analysis rate (PCM1808 pre-upsample vs 48 kHz). */
  design_biquad_pair(g_en[DSP_BLOCK_FILTER_BANK] ? g_filter : DSP_FILTER_BYPASS);
  if (g_en[DSP_BLOCK_NOTCH]) {
    design_notch();
  } else {
    g_notch_active = false;
  }
  design_fir(g_en[DSP_BLOCK_FIR_BANK] ? g_fir_preset : DSP_FILTER_BYPASS);
  g_fft_fill = 0;
  quartz_hook_set_source_rate(hz);
}

uint32_t dsp_graph_source_rate(void) { return g_source_hz; }

void dsp_graph_set_timebase(bool ok, float measured_hz) {
  g_source_hz_measured = measured_hz;
  beat_analysis_set_timebase(ok);
  quartz_hook_set_timebase(ok);
}

uint32_t dsp_graph_process(const float *in_fs, uint32_t n, uint64_t source_index,
                           int32_t *out_pcm) {
  if (n == 0 || n > DSP_MAX_BLOCK || !in_fs || !out_pcm) {
    return 0;
  }
  const uint32_t t0 = time_us_32();
  float *buf = g_work;
  memcpy(buf, in_fs, n * sizeof(float));

  analyze(buf, n, source_index);
  process_path(buf, n);

  /*
   * Quantize to signed 24-bit. Bypass may apply PCM1808_PCM_GAIN for loudness
   * parity with older builds; when processing is on, gain is the gain block.
   */
  const auto qscale = g_bypass ? (static_cast<float>(APP_PCM24_FULL_SCALE) * static_cast<float>(PCM1808_PCM_GAIN))
                               : static_cast<float>(APP_PCM24_FULL_SCALE);

  float peak = 0.f;
  float power = 0.f;
  for (uint32_t i = 0; i < n; i++) {
    float y = buf[i];
    float a = fabsf(y);
    if (a > peak) {
      peak = a;
    }
    power += y * y;
    float s = y * qscale;
    if (s > static_cast<float>(APP_PCM24_FULL_SCALE)) {
      s = static_cast<float>(APP_PCM24_FULL_SCALE);
      g_clip++;
    } else if (s < static_cast<float>(APP_PCM24_MIN)) {
      s = static_cast<float>(APP_PCM24_MIN);
      g_clip++;
    }
    out_pcm[i] = static_cast<int32_t>(s);
  }
  g_post_peak = fmaxf(g_post_peak * 0.95f, peak);
  g_post_rms_ema = 0.9f * g_post_rms_ema + 0.1f * sqrtf(power / static_cast<float>(n));

  perf_timing_note_dsp_block(time_us_32() - t0, n, g_source_hz);
  return n;
}

void dsp_graph_watchdog_poll(void) {
  const auto now = millis();
  if (g_watch_cooling && (now - g_watch_cooldown_ms) < kWatchCooldownMs) {
    return;
  }
  g_watch_cooling = false;
  const auto *d = diagnostics_get();
  if (!g_watch_armed || g_last_enabled == DSP_BLOCK_BYPASS ||
      !g_en[g_last_enabled]) {
    return;
  }
  /* Only blame the most recently enabled block for a few seconds after arm. */
  if ((now - g_enable_ms) > kWatchWindowMs) {
    g_watch_armed = false;
    return;
  }

  const bool underrun = d->pcm_ring_underrun_count > g_underrun_at_enable;
  const bool short_w =
      d->usb_short_write_count > g_short_at_enable + kWatchShortWriteSlack;
  if (!underrun && !short_w) {
    return;
  }

  g_en[g_last_enabled] = false;
  if (g_last_enabled == DSP_BLOCK_FILTER_BANK) {
    g_bq_active = false;
  }
  if (g_last_enabled == DSP_BLOCK_FIR_BANK) {
    g_fir_active = false;
  }
  if (g_last_enabled == DSP_BLOCK_NOTCH) {
    g_notch_active = false;
  }
  push_error(g_last_enabled, underrun ? DSP_ERR_UNDERRUN : DSP_ERR_USB_STALL, 0);
  g_watch_cooling = true;
  g_watch_cooldown_ms = now;
  g_watch_armed = false;
}

void dsp_graph_note_enable(dsp_block_id_t id) {
  g_last_enabled = id;
  g_watch_armed = true;
  g_enable_ms = millis();
  auto *d = diagnostics_get();
  g_underrun_at_enable = d->pcm_ring_underrun_count;
  g_short_at_enable = d->usb_short_write_count;
}

void dsp_graph_get_metrics(dsp_metrics_t *out) {
  if (!out) {
    return;
  }
  out->pre_peak = g_pre_peak;
  out->post_peak = g_post_peak;
  out->pre_rms = g_pre_rms_ema;
  out->post_rms = g_post_rms_ema;
  out->noise_floor = g_noise;
  out->agc_gain_db =
      (g_en[DSP_BLOCK_AGC] && !g_bypass)
          ? (20.f * log10f(fmaxf(g_agc_lin, 1e-9f)))
          : 0.f;
  out->match_score = g_match_score;
  out->clip_count = g_clip;
  out->limiter_count = g_lim;
}

void dsp_graph_get_spectrum(dsp_spectrum_t *out) {
  if (!out) {
    return;
  }
  *out = g_spectrum;
}

void dsp_graph_get_matched(dsp_matched_t *out) {
  if (!out) {
    return;
  }
  out->score = g_match_score;
  out->template_peak = g_match_tmpl_peak;
  out->template_len = kMatchLen;
  out->learned = g_match_learned;
}

void dsp_graph_get_beat(dsp_beat_rate_t *out) {
  if (!out) {
    return;
  }
  beat_analysis_get(out);
  out->events_seen = g_event_count;
  out->source_hz = g_source_hz;
  out->source_hz_measured = g_source_hz_measured;
}

uint32_t dsp_graph_copy_events(dsp_event_t *out, uint32_t max_n) {
  if (!out || max_n == 0 || g_event_count == 0) {
    return 0;
  }
  uint32_t n = g_event_count < max_n ? g_event_count : max_n;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t idx =
        (g_event_head + DSP_EVENT_LOG_CAP - 1u - i) % DSP_EVENT_LOG_CAP;
    out[i] = g_events[idx];
  }
  return n;
}

uint32_t dsp_graph_copy_beats(dsp_beat_t *out, uint32_t max_n) {
  return beat_analysis_copy_beats(out, max_n);
}

uint32_t dsp_graph_beat_seq(void) { return beat_analysis_beat_seq(); }

uint32_t dsp_graph_copy_beats_since(uint32_t seq, dsp_beat_t *out,
                                    uint32_t max_n) {
  return beat_analysis_copy_beats_since(seq, out, max_n);
}

bool dsp_graph_get_bypass(void) { return g_bypass; }

void dsp_graph_set_bypass(bool on) { g_bypass = on; }

bool dsp_graph_block_enabled(dsp_block_id_t id) {
  if (id >= DSP_BLOCK_COUNT) {
    return false;
  }
  return g_en[id];
}

bool dsp_graph_set_block_enabled(dsp_block_id_t id, bool on) {
  if (id >= DSP_BLOCK_COUNT || id == DSP_BLOCK_BYPASS) {
    return false;
  }
  g_en[id] = on;
  if (id == DSP_BLOCK_FILTER_BANK) {
    design_biquad_pair(on ? g_filter : DSP_FILTER_BYPASS);
  }
  if (id == DSP_BLOCK_FIR_BANK) {
    design_fir(on ? g_fir_preset : DSP_FILTER_BYPASS);
  }
  if (id == DSP_BLOCK_NOTCH) {
    if (on) {
      design_notch();
    } else {
      g_notch_active = false;
    }
  }
  if (id == DSP_BLOCK_SOFT_GATE && on) {
    g_gate_lin = 1.0f;
  }
  if (id == DSP_BLOCK_AGC && on) {
    g_agc_lin = 1.0f;
  }
  if (id == DSP_BLOCK_SPECTRUM && on) {
    g_fft_fill = 0;
    g_spectrum = {};
  }
  if (on) {
    /* Process-path enables imply leaving master bypass. */
    if (dsp_block_is_process(id)) {
      g_bypass = false;
    }
    dsp_graph_note_enable(id);
  }
  return true;
}

bool dsp_graph_set_param(dsp_block_id_t id, const char *param, float value) {
  if (!param || !std::isfinite(value)) {
    return false;
  }
  if (id == DSP_BLOCK_GAIN && strcmp(param, "db") == 0) {
    if (value < kGainDbMin || value > kGainDbMax) {
      return false;
    }
    g_gain_lin = db_to_lin(value);
    g_en[DSP_BLOCK_GAIN] = true;
    g_bypass = false;
    dsp_graph_note_enable(DSP_BLOCK_GAIN);
    return true;
  }
  if (id == DSP_BLOCK_FILTER_BANK && strcmp(param, "preset") == 0) {
    auto p = static_cast<int>(value);
    if (p < 0 || p > static_cast<int>(DSP_FILTER_NARROW)) {
      return false;
    }
    g_filter = static_cast<dsp_filter_preset_t>(p);
    g_en[DSP_BLOCK_FILTER_BANK] = (g_filter != DSP_FILTER_BYPASS);
    design_biquad_pair(g_filter);
    g_bypass = false;
    dsp_graph_note_enable(DSP_BLOCK_FILTER_BANK);
    return true;
  }
  if (id == DSP_BLOCK_FIR_BANK && strcmp(param, "preset") == 0) {
    auto p = static_cast<int>(value);
    if (p < 0 || p > static_cast<int>(DSP_FILTER_NARROW)) {
      return false;
    }
    g_fir_preset = static_cast<dsp_filter_preset_t>(p);
    g_en[DSP_BLOCK_FIR_BANK] = (g_fir_preset != DSP_FILTER_BYPASS);
    design_fir(g_fir_preset);
    g_bypass = false;
    dsp_graph_note_enable(DSP_BLOCK_FIR_BANK);
    return true;
  }
  if (id == DSP_BLOCK_MATCHED_FILTER) {
    if (strcmp(param, "threshold") == 0) {
      if (value < 0.f || value > 2.f) {
        return false;
      }
      g_match_threshold = value;
      return true;
    }
    if (strcmp(param, "learn") == 0) {
      g_match_learn = value >= 0.5f;
      return true;
    }
  }
  if (id == DSP_BLOCK_SPECTRUM && strcmp(param, "stride") == 0) {
    auto s = static_cast<int>(value);
    if (s < 1 || s > 16) {
      return false;
    }
    g_fft_stride = static_cast<uint32_t>(s);
    return true;
  }
  if (id == DSP_BLOCK_EVENT_DETECT) {
    if (strcmp(param, "threshold") == 0) {
      if (value < 1.f || value > 100.f) {
        return false;
      }
      g_event_threshold = value;
      return true;
    }
    if (strcmp(param, "preroll_ms") == 0) {
      if (value < 0.f || value > 100.f) {
        return false;
      }
      g_preroll_ms = value;
      return true;
    }
    if (strcmp(param, "tail_ms") == 0) {
      if (value < 1.f || value > 200.f) {
        return false;
      }
      g_tail_ms = value;
      return true;
    }
  }
  if (id == DSP_BLOCK_BEAT_RATE) {
    return beat_analysis_set_param(param, value);
  }
  if (id == DSP_BLOCK_NOTCH) {
    if (strcmp(param, "hz") == 0 || strcmp(param, "freq") == 0) {
      if (value != 50.f && value != 60.f) {
        return false;
      }
      g_notch_hz = value;
      g_en[DSP_BLOCK_NOTCH] = true;
      design_notch();
      g_bypass = false;
      dsp_graph_note_enable(DSP_BLOCK_NOTCH);
      return true;
    }
  }
  if (id == DSP_BLOCK_AGC) {
    if (strcmp(param, "target") == 0) {
      if (value <= 0.f || value > 1.f) {
        return false;
      }
      g_agc_target = value;
      return true;
    }
    if (strcmp(param, "tau_s") == 0 || strcmp(param, "tau") == 0) {
      if (value < 0.05f || value > 30.f) {
        return false;
      }
      g_agc_tau_s = value;
      return true;
    }
  }
  if (id == DSP_BLOCK_SOFT_GATE) {
    if (strcmp(param, "floor_db") == 0) {
      if (value < -60.f || value > 0.f) {
        return false;
      }
      g_gate_floor_lin = db_to_lin(value);
      return true;
    }
    if (strcmp(param, "attack_ms") == 0) {
      if (value < 0.1f || value > 50.f) {
        return false;
      }
      g_gate_attack_ms = value;
      return true;
    }
    if (strcmp(param, "release_ms") == 0) {
      if (value < 1.f || value > 500.f) {
        return false;
      }
      g_gate_release_ms = value;
      return true;
    }
  }
  return false;
}

bool dsp_graph_get_param(dsp_block_id_t id, const char *param, float *out) {
  if (!param || !out) {
    return false;
  }
  if (id == DSP_BLOCK_GAIN && strcmp(param, "db") == 0) {
    *out = 20.f * log10f(fmaxf(g_gain_lin, 1e-9f));
    return true;
  }
  if (id == DSP_BLOCK_FILTER_BANK && strcmp(param, "preset") == 0) {
    *out = static_cast<float>(g_filter);
    return true;
  }
  if (id == DSP_BLOCK_FIR_BANK && strcmp(param, "preset") == 0) {
    *out = static_cast<float>(g_fir_preset);
    return true;
  }
  if (id == DSP_BLOCK_MATCHED_FILTER) {
    if (strcmp(param, "threshold") == 0) {
      *out = g_match_threshold;
      return true;
    }
    if (strcmp(param, "learn") == 0) {
      *out = g_match_learn ? 1.f : 0.f;
      return true;
    }
    if (strcmp(param, "score") == 0) {
      *out = g_match_score;
      return true;
    }
  }
  if (id == DSP_BLOCK_SPECTRUM && strcmp(param, "stride") == 0) {
    *out = static_cast<float>(g_fft_stride);
    return true;
  }
  if (id == DSP_BLOCK_EVENT_DETECT) {
    if (strcmp(param, "threshold") == 0) {
      *out = g_event_threshold;
      return true;
    }
    if (strcmp(param, "preroll_ms") == 0) {
      *out = g_preroll_ms;
      return true;
    }
    if (strcmp(param, "tail_ms") == 0) {
      *out = g_tail_ms;
      return true;
    }
  }
  if (id == DSP_BLOCK_BEAT_RATE) {
    return beat_analysis_get_param(param, out);
  }
  if (id == DSP_BLOCK_NOTCH &&
      (strcmp(param, "hz") == 0 || strcmp(param, "freq") == 0)) {
    *out = g_notch_hz;
    return true;
  }
  if (id == DSP_BLOCK_AGC) {
    if (strcmp(param, "target") == 0) {
      *out = g_agc_target;
      return true;
    }
    if (strcmp(param, "tau_s") == 0 || strcmp(param, "tau") == 0) {
      *out = g_agc_tau_s;
      return true;
    }
    if (strcmp(param, "gain_db") == 0) {
      *out = 20.f * log10f(fmaxf(g_agc_lin, 1e-9f));
      return true;
    }
  }
  if (id == DSP_BLOCK_SOFT_GATE) {
    if (strcmp(param, "floor_db") == 0) {
      *out = 20.f * log10f(fmaxf(g_gate_floor_lin, 1e-9f));
      return true;
    }
    if (strcmp(param, "attack_ms") == 0) {
      *out = g_gate_attack_ms;
      return true;
    }
    if (strcmp(param, "release_ms") == 0) {
      *out = g_gate_release_ms;
      return true;
    }
  }
  return false;
}

uint32_t dsp_graph_copy_errors(dsp_error_t *out, uint32_t max_n) {
  if (!out || max_n == 0 || g_err_count == 0) {
    return 0;
  }
  uint32_t n = g_err_count < max_n ? g_err_count : max_n;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t idx =
        (g_err_head + DSP_ERROR_LOG_CAP - 1u - i) % DSP_ERROR_LOG_CAP;
    out[i] = g_errors[idx];
  }
  return n;
}

void dsp_graph_clear_errors(void) {
  g_err_count = 0;
  g_err_head = 0;
}

bool dsp_graph_pop_async_message(char *buf, size_t buflen) {
  if (!g_async_pending || !buf || buflen == 0) {
    return false;
  }
  strncpy(buf, g_async_msg, buflen - 1);
  buf[buflen - 1] = '\0';
  g_async_pending = false;
  return true;
}
