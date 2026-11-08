/**
 * @file dsp_graph.cpp
 * @brief Float32 DSP graph: analysis, process path, PCM24 quantize, USB watchdog.
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
 * @see dsp_graph.h, control_api.cpp
 */

#include "dsp_graph.h"

#include "app_config.h"
#include "diagnostics.h"

#include <Arduino.h>
#include <arm_math.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

/**
 * Common mechanical watch rates (vibrations / beats per hour) for nearest-match.
 * Includes vintage, modern, hi-beat, and ultra hi-beat escapements.
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

bool g_bypass = true;
bool g_en[DSP_BLOCK_COUNT];
/** Sample rate of graph input (may be < 48 kHz on PCM1808 pre-upsample path). */
uint32_t g_source_hz = DSP_SAMPLE_RATE_HZ;

float g_gain_lin = 1.0f; /* linear gain; 1.0 = 0 dB */
dsp_filter_preset_t g_filter = DSP_FILTER_BYPASS;
float g_event_threshold = 6.0f; /* detect when |x| > noise * threshold */
float g_preroll_ms = 25.0f;
float g_tail_ms = 25.0f;

arm_biquad_casd_df1_inst_f32 g_bq;
float g_bq_state[8]; /* 2 stages × 4 DF1 state words */
float g_bq_coeffs[10];
bool g_bq_active = false;

/* Phase-2 mains notch (1 biquad stage). */
arm_biquad_casd_df1_inst_f32 g_notch;
float g_notch_state[4];
float g_notch_coeffs[5];
bool g_notch_active = false;
float g_notch_hz = 60.f; /* 50 or 60 */

/* Phase-2 slow AGC. */
float g_agc_lin = 1.0f;
float g_agc_target = 0.35f; /* desired tick peak FS */
float g_agc_tau_s = 1.5f;   /* time constant (seconds) */
float g_agc_min_lin = 0.25f; /* -12 dB */
float g_agc_max_lin = 4.0f;  /* +12 dB */

/* Phase-2 soft gate (transient-aware). */
float g_gate_lin = 1.0f;
float g_gate_floor_lin = 0.25f; /* -12 dB between ticks */
float g_gate_attack_ms = 1.0f;
float g_gate_release_ms = 40.0f;

/* Phase-3 matched filter (analysis). */
constexpr uint32_t kMatchLen = 64u;
float g_match_tmpl[kMatchLen];
float g_match_hist[kMatchLen];
float g_match_score = 0.f;
float g_match_threshold = 0.35f;
bool g_match_learned = false;
bool g_match_learn = true;
float g_match_tmpl_energy = 1.f;
float g_match_tmpl_peak = 0.f;
float g_event_wave[kMatchLen];
uint32_t g_event_wave_n = 0;

/* Phase-3 FIR bank (process). */
constexpr uint32_t kFirTaps = 64u;
arm_fir_instance_f32 g_fir;
float g_fir_coeffs[kFirTaps];
float g_fir_state[kFirTaps + DSP_MAX_BLOCK];
bool g_fir_active = false;
dsp_filter_preset_t g_fir_preset = DSP_FILTER_BYPASS;

/* Phase-3 spectrum (analysis). */
constexpr uint32_t kFftN = 512u;
arm_rfft_fast_instance_f32 g_rfft;
float g_fft_acc[kFftN];
float g_fft_work[kFftN];
uint32_t g_fft_fill = 0;
uint32_t g_fft_stride = 1;
uint32_t g_fft_stride_i = 0;
bool g_rfft_ok = false;
dsp_spectrum_t g_spectrum = {};

float g_noise = 1e-4f;
float g_tick_peak_ema = 0.05f; /* EMA of accepted event peaks (for min-peak gate) */
float g_pre_peak = 0.f;
float g_post_peak = 0.f;
float g_pre_rms_ema = 0.f;
float g_post_rms_ema = 0.f;
uint32_t g_clip = 0;
uint32_t g_lim = 0;
float g_event_energy = 0.f;

dsp_event_t g_events[DSP_EVENT_LOG_CAP];
uint32_t g_event_count = 0;
uint32_t g_event_head = 0;

uint32_t g_intervals[kBeatHist];
uint32_t g_interval_n = 0;
uint64_t g_last_event_peak = 0;
bool g_have_last_event = false;
dsp_beat_rate_t g_beat = {};

bool g_in_event = false;
uint64_t g_event_start = 0;
uint64_t g_event_peak_idx = 0;
float g_event_peak = 0.f;
uint32_t g_below_count = 0;

dsp_error_t g_errors[DSP_ERROR_LOG_CAP];
uint32_t g_err_count = 0;
uint32_t g_err_head = 0;
char g_async_msg[96];
bool g_async_pending = false;

dsp_block_id_t g_last_enabled = DSP_BLOCK_BYPASS;
uint32_t g_enable_ms = 0;
uint32_t g_underrun_at_enable = 0;
uint32_t g_short_at_enable = 0;
uint32_t g_watch_cooldown_ms = 0;

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
  const char *name = "block";
  switch (block) {
  case DSP_BLOCK_GAIN:
    name = "gain";
    break;
  case DSP_BLOCK_LIMITER:
    name = "limiter";
    break;
  case DSP_BLOCK_FILTER_BANK:
    name = "filter_bank";
    break;
  case DSP_BLOCK_EVENT_DETECT:
    name = "event_detect";
    break;
  case DSP_BLOCK_BEAT_RATE:
    name = "beat_rate";
    break;
  case DSP_BLOCK_SOFT_GATE:
    name = "soft_gate";
    break;
  case DSP_BLOCK_NOTCH:
    name = "notch";
    break;
  case DSP_BLOCK_AGC:
    name = "agc";
    break;
  case DSP_BLOCK_MATCHED_FILTER:
    name = "matched_filter";
    break;
  case DSP_BLOCK_FIR_BANK:
    name = "fir_bank";
    break;
  case DSP_BLOCK_SPECTRUM:
    name = "spectrum";
    break;
  default:
    break;
  }
  const char *why = reason == DSP_ERR_UNDERRUN     ? "underrun"
                    : reason == DSP_ERR_USB_STALL ? "usb_stall"
                                                  : "overload";
  snprintf(g_async_msg, sizeof(g_async_msg),
           "CLI: ERR dsp eject block=%s reason=%s", name, why);
  g_async_pending = true;
}

/**
 * Stores a completed analysis event and updates beat-rate state.
 *
 * When beat_rate is enabled and at least four recent inter-peak intervals are
 * available, recomputes median interval → vph_raw and nearest common rate.
 *
 * @param ev Closed event window from the detector.
 */
void push_event(const dsp_event_t &ev) {
  g_events[g_event_head] = ev;
  g_event_head = (g_event_head + 1u) % DSP_EVENT_LOG_CAP;
  if (g_event_count < DSP_EVENT_LOG_CAP) {
    g_event_count++;
  }

  if (g_have_last_event && ev.peak_sample > g_last_event_peak) {
    uint32_t iv = (uint32_t)(ev.peak_sample - g_last_event_peak);
    /*
     * Accept ~25 ms .. 1 s in source-rate units (tick/tock pairs are often
     * half a beat). Using g_source_hz — not USB 48 kHz — is required when the
     * PCM1808 path analyzes pre-upsample frames.
     */
    const uint32_t iv_min = g_source_hz / 40u;
    const uint32_t iv_max = g_source_hz;
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
  g_last_event_peak = ev.peak_sample;
  g_have_last_event = true;

  if (g_interval_n >= 4u && g_en[DSP_BLOCK_BEAT_RATE]) {
    uint32_t tmp[kBeatHist];
    memcpy(tmp, g_intervals, g_interval_n * sizeof(uint32_t));
    for (uint32_t i = 1; i < g_interval_n; i++) {
      uint32_t key = tmp[i];
      int32_t j = (int32_t)i - 1;
      while (j >= 0 && tmp[j] > key) {
        tmp[j + 1] = tmp[j];
        j--;
      }
      tmp[j + 1] = key;
    }
    uint32_t med = tmp[g_interval_n / 2u];
    g_beat.interval_samples = med;
    float period_s = (float)med / (float)g_source_hz;
    g_beat.vph_raw = (period_s > 0.f) ? (3600.0f / period_s) : 0.f;
    uint32_t nearest = 0;
    float best = 1e9f;
    for (uint32_t c : kCommonVph) {
      float d = fabsf(g_beat.vph_raw - (float)c);
      if (d < best) {
        best = d;
        nearest = c;
      }
    }
    float tol = 0.04f * (float)nearest;
    g_beat.locked = best <= tol;
    g_beat.vph_nearest = g_beat.locked ? nearest : 0u;
    g_beat.confidence = g_beat.locked ? fmaxf(0.f, 1.f - best / (tol + 1.f))
                                      : fmaxf(0.f, 1.f - best / ((float)nearest + 1.f));
  }
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
  const float fs = (float)g_source_hz;
  auto one_pole_hp = [&](float fc, float *c) {
    float x = expf(-2.f * 3.14159265f * fc / fs);
    c[0] = (1.f + x) * 0.5f;
    c[1] = -(1.f + x) * 0.5f;
    c[2] = 0.f;
    c[3] = x;
    c[4] = 0.f;
  };
  auto one_pole_lp = [&](float fc, float *c) {
    float x = expf(-2.f * 3.14159265f * fc / fs);
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
  const float fs = (float)g_source_hz;
  const float w0 = 2.f * 3.14159265f * g_notch_hz / fs;
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
  const float fs = (float)g_source_hz;
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
  const int M = (int)kFirTaps;
  const float mid = 0.5f * (float)(M - 1);
  float sum = 0.f;
  for (int n = 0; n < M; n++) {
    float k = (float)n - mid;
    float ideal;
    if (fabsf(k) < 1e-6f) {
      ideal = 2.f * (fc2 - fc1);
    } else {
      const float twopi = 2.f * 3.14159265f;
      ideal = (sinf(twopi * fc2 * k) - sinf(twopi * fc1 * k)) /
              (3.14159265f * k);
    }
    const float w =
        0.54f - 0.46f * cosf(2.f * 3.14159265f * (float)n / (float)(M - 1));
    g_fir_coeffs[n] = ideal * w;
    sum += g_fir_coeffs[n];
  }
  /* Prefer peak-normalize for bandpass (DC gain may be ~0). */
  float peak = 0.f;
  for (uint32_t i = 0; i < kFirTaps; i++) {
    float a = fabsf(g_fir_coeffs[i]);
    if (a > peak) {
      peak = a;
    }
  }
  if (peak > 1e-9f) {
    float s = 1.f / (peak * (float)kFirTaps * 0.25f + 1e-9f);
    /* Scale so midband impulse energy is moderate. */
    float energy = 0.f;
    for (uint32_t i = 0; i < kFirTaps; i++) {
      energy += g_fir_coeffs[i] * g_fir_coeffs[i];
    }
    s = 1.f / sqrtf(fmaxf(energy, 1e-12f));
    for (uint32_t i = 0; i < kFirTaps; i++) {
      g_fir_coeffs[i] *= s;
    }
  }
  (void)sum;
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
  if (!g_match_learn || g_event_wave_n < (kMatchLen / 4u)) {
    return;
  }
  float tmp[kMatchLen];
  memset(tmp, 0, sizeof(tmp));
  uint32_t n = g_event_wave_n < kMatchLen ? g_event_wave_n : kMatchLen;
  /* Center on peak inside wave. */
  uint32_t peak_i = 0;
  float peak = 0.f;
  for (uint32_t i = 0; i < n; i++) {
    float a = fabsf(g_event_wave[i]);
    if (a > peak) {
      peak = a;
      peak_i = i;
    }
  }
  int32_t start = (int32_t)peak_i - (int32_t)(kMatchLen / 2u);
  for (uint32_t i = 0; i < kMatchLen; i++) {
    int32_t src = start + (int32_t)i;
    tmp[i] = (src >= 0 && src < (int32_t)n) ? g_event_wave[src] : 0.f;
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
      float w = 0.5f - 0.5f * cosf(2.f * 3.14159265f * (float)k /
                                   (float)(kFftN - 1u));
      g_fft_work[k] = g_fft_acc[k] * w;
    }
    arm_rfft_fast_f32(&g_rfft, g_fft_work, g_fft_work, 0);
    /* Magnitudes: DC at [0], Nyquist at [1], then pairs. */
    float mags[kFftN / 2];
    mags[0] = fabsf(g_fft_work[0]);
    mags[kFftN / 2u - 1u] = fabsf(g_fft_work[1]);
    for (uint32_t k = 1; k < kFftN / 2u - 1u; k++) {
      float re = g_fft_work[2u * k];
      float im = g_fft_work[2u * k + 1u];
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
          (float)best * (float)g_source_hz / (float)kFftN;
      g_spectrum.peak_mag[p] = bestm;
      g_spectrum.n_peaks = p + 1u;
      mags[best] = 0.f; /* suppress for next peak */
    }
  }
}

/**
 * Analysis path: meters, noise EMA, event windows, beat-rate feed.
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
  float rms = sqrtf(power / (float)n);
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
    if (!g_in_event && peak < open_guess) {
      g_noise = 0.995f * g_noise + 0.005f * fmaxf(rms, 1e-7f);
      if (g_noise < 1e-6f) {
        g_noise = 1e-6f;
      }
    }
  }

  if (!g_en[DSP_BLOCK_EVENT_DETECT] && !g_en[DSP_BLOCK_BEAT_RATE]) {
    return;
  }

  const float thr_open =
      fmaxf(g_noise * g_event_threshold, fmaxf(4.f * g_pre_rms_ema, 1e-3f));
  const float thr_close = thr_open * 0.35f;
  const float min_peak = fmaxf(thr_open, 0.25f * g_tick_peak_ema);
  uint32_t preroll = (uint32_t)(g_preroll_ms * 0.001f * (float)g_source_hz);
  uint32_t tail = (uint32_t)(g_tail_ms * 0.001f * (float)g_source_hz);
  uint32_t max_event = (uint32_t)(0.080f * (float)g_source_hz); /* 80 ms */

  const bool match_gate = g_en[DSP_BLOCK_MATCHED_FILTER] && g_match_learned;
  for (uint32_t i = 0; i < n; i++) {
    float a = fabsf(x[i]);
    uint64_t idx = base + i;
    if (!g_in_event) {
      bool open_ok = a > thr_open;
      if (open_ok && match_gate && g_match_score < g_match_threshold) {
        open_ok = false;
      }
      if (open_ok) {
        g_in_event = true;
        g_event_start = (idx > preroll) ? (idx - preroll) : 0;
        g_event_peak_idx = idx;
        g_event_peak = a;
        g_event_energy = a * a;
        g_below_count = 0;
        g_event_wave_n = 0;
        if (g_event_wave_n < kMatchLen) {
          g_event_wave[g_event_wave_n++] = x[i];
        }
      }
    } else {
      if (g_event_wave_n < kMatchLen) {
        g_event_wave[g_event_wave_n++] = x[i];
      }
      g_event_energy += a * a;
      if (a > g_event_peak) {
        g_event_peak = a;
        g_event_peak_idx = idx;
      }
      if (a < thr_close) {
        g_below_count++;
      } else {
        g_below_count = 0;
      }
      const uint32_t dur = (uint32_t)(idx - g_event_start);
      if (g_below_count > tail || dur >= max_event) {
        if (g_event_peak >= min_peak) {
          dsp_event_t ev{};
          ev.start_sample = g_event_start;
          ev.peak_sample = g_event_peak_idx;
          ev.duration_samples = dur;
          ev.rise_samples =
              (uint32_t)(g_event_peak_idx > g_event_start
                             ? (g_event_peak_idx - g_event_start)
                             : 0u);
          ev.peak_abs = g_event_peak;
          ev.noise_floor = g_noise;
          ev.snr_db = 20.f * log10f(fmaxf(g_event_peak, 1e-9f) /
                                    fmaxf(g_noise, 1e-9f));
          ev.energy = g_event_energy;
          float conf = fminf(1.f, g_event_peak / (thr_open + 1e-9f) / 4.f);
          if (g_en[DSP_BLOCK_MATCHED_FILTER] && g_match_learned) {
            conf = fminf(1.f, 0.5f * conf + 0.5f * g_match_score);
          }
          ev.confidence = conf;
          push_event(ev);
          g_tick_peak_ema = 0.9f * g_tick_peak_ema + 0.1f * g_event_peak;
          if (g_en[DSP_BLOCK_MATCHED_FILTER]) {
            match_learn_from_wave();
          }
        }
        g_in_event = false;
        g_below_count = 0;
        g_event_energy = 0.f;
        g_event_wave_n = 0;
      }
    }
  }
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
    const float fs = (float)g_source_hz;
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
      const float fs = (float)g_source_hz;
      const float alpha =
          1.f - expf(-((float)n / fs) / fmaxf(g_agc_tau_s, 0.05f));
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

float dsp_sample_from_pcm1808_left24(int32_t left24) {
  return (float)left24 * (1.0f / 8388608.0f);
}

float dsp_sample_from_adc_centered(int32_t centered_counts) {
  return (float)centered_counts * (1.0f / 2048.0f);
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
  g_rfft_ok = (arm_rfft_fast_init_f32(&g_rfft, 512) == ARM_MATH_SUCCESS);
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
  g_tick_peak_ema = 0.05f;
  g_event_energy = 0.f;
  g_event_count = 0;
  g_event_head = 0;
  g_interval_n = 0;
  g_have_last_event = false;
  g_beat = {};
  g_beat.source_hz = g_source_hz;
  g_err_count = 0;
  g_err_head = 0;
  g_async_pending = false;
}

void dsp_graph_set_source_rate(uint32_t hz) {
  if (hz == 0) {
    hz = DSP_SAMPLE_RATE_HZ;
  }
  g_source_hz = hz;
  g_beat.source_hz = hz;
  /* Coeffs depend on analysis rate (PCM1808 pre-upsample vs 48 kHz). */
  design_biquad_pair(g_en[DSP_BLOCK_FILTER_BANK] ? g_filter : DSP_FILTER_BYPASS);
  if (g_en[DSP_BLOCK_NOTCH]) {
    design_notch();
  } else {
    g_notch_active = false;
  }
  design_fir(g_en[DSP_BLOCK_FIR_BANK] ? g_fir_preset : DSP_FILTER_BYPASS);
  g_rfft_ok = (arm_rfft_fast_init_f32(&g_rfft, 512) == ARM_MATH_SUCCESS);
  g_fft_fill = 0;
}

uint32_t dsp_graph_source_rate(void) { return g_source_hz; }

uint32_t dsp_graph_process(const float *in_fs, uint32_t n, uint64_t source_index,
                           int32_t *out_pcm) {
  if (n == 0 || n > DSP_MAX_BLOCK || !in_fs || !out_pcm) {
    return 0;
  }
  float buf[DSP_MAX_BLOCK];
  memcpy(buf, in_fs, n * sizeof(float));

  analyze(buf, n, source_index);
  process_path(buf, n);

  /*
   * Quantize to signed 24-bit. Bypass may apply PCM1808_PCM_GAIN for loudness
   * parity with older builds; when processing is on, gain is the gain block.
   */
  const float qscale = g_bypass ? ((float)APP_PCM24_FULL_SCALE * (float)PCM1808_PCM_GAIN)
                                : (float)APP_PCM24_FULL_SCALE;

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
    if (s > (float)APP_PCM24_FULL_SCALE) {
      s = (float)APP_PCM24_FULL_SCALE;
      g_clip++;
    } else if (s < (float)APP_PCM24_MIN) {
      s = (float)APP_PCM24_MIN;
      g_clip++;
    }
    out_pcm[i] = (int32_t)s;
  }
  g_post_peak = fmaxf(g_post_peak * 0.95f, peak);
  g_post_rms_ema = 0.9f * g_post_rms_ema + 0.1f * sqrtf(power / (float)n);
  return n;
}

void dsp_graph_watchdog_poll(void) {
  uint32_t now = millis();
  if (g_watch_cooldown_ms != 0 && (now - g_watch_cooldown_ms) < 2000u) {
    return;
  }
  diag_counters_t *d = diagnostics_get();
  if (g_enable_ms == 0 || g_last_enabled == DSP_BLOCK_BYPASS ||
      !g_en[g_last_enabled]) {
    return;
  }
  /* Only blame the most recently enabled block for a few seconds after arm. */
  if ((now - g_enable_ms) > 5000u) {
    g_enable_ms = 0;
    return;
  }

  const bool underrun = d->pcm_ring_underrun_count > g_underrun_at_enable;
  const bool short_w = d->usb_short_write_count > g_short_at_enable + 50u;
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
  g_watch_cooldown_ms = now;
  g_enable_ms = 0;
}

void dsp_graph_note_enable(dsp_block_id_t id) {
  g_last_enabled = id;
  g_enable_ms = millis();
  diag_counters_t *d = diagnostics_get();
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
  *out = g_beat;
  out->events_seen = g_event_count;
  out->intervals_held = g_interval_n;
  out->source_hz = g_source_hz;
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
    if (id == DSP_BLOCK_GAIN || id == DSP_BLOCK_LIMITER ||
        id == DSP_BLOCK_FILTER_BANK || id == DSP_BLOCK_SOFT_GATE ||
        id == DSP_BLOCK_NOTCH || id == DSP_BLOCK_AGC ||
        id == DSP_BLOCK_FIR_BANK) {
      g_bypass = false;
    }
    dsp_graph_note_enable(id);
  }
  return true;
}

bool dsp_graph_set_param(dsp_block_id_t id, const char *param, float value) {
  if (!param) {
    return false;
  }
  if (id == DSP_BLOCK_GAIN && strcmp(param, "db") == 0) {
    g_gain_lin = db_to_lin(value);
    g_en[DSP_BLOCK_GAIN] = true;
    g_bypass = false;
    dsp_graph_note_enable(DSP_BLOCK_GAIN);
    return true;
  }
  if (id == DSP_BLOCK_FILTER_BANK && strcmp(param, "preset") == 0) {
    int p = (int)value;
    if (p < 0 || p > (int)DSP_FILTER_NARROW) {
      return false;
    }
    g_filter = (dsp_filter_preset_t)p;
    g_en[DSP_BLOCK_FILTER_BANK] = (g_filter != DSP_FILTER_BYPASS);
    design_biquad_pair(g_filter);
    g_bypass = false;
    dsp_graph_note_enable(DSP_BLOCK_FILTER_BANK);
    return true;
  }
  if (id == DSP_BLOCK_FIR_BANK && strcmp(param, "preset") == 0) {
    int p = (int)value;
    if (p < 0 || p > (int)DSP_FILTER_NARROW) {
      return false;
    }
    g_fir_preset = (dsp_filter_preset_t)p;
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
    int s = (int)value;
    if (s < 1 || s > 16) {
      return false;
    }
    g_fft_stride = (uint32_t)s;
    return true;
  }
  if (id == DSP_BLOCK_EVENT_DETECT) {
    if (strcmp(param, "threshold") == 0) {
      g_event_threshold = value;
      return true;
    }
    if (strcmp(param, "preroll_ms") == 0) {
      g_preroll_ms = value;
      return true;
    }
    if (strcmp(param, "tail_ms") == 0) {
      g_tail_ms = value;
      return true;
    }
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
      g_gate_floor_lin = db_to_lin(value);
      if (g_gate_floor_lin > 1.f) {
        g_gate_floor_lin = 1.f;
      }
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
    *out = (float)g_filter;
    return true;
  }
  if (id == DSP_BLOCK_FIR_BANK && strcmp(param, "preset") == 0) {
    *out = (float)g_fir_preset;
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
    *out = (float)g_fft_stride;
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
