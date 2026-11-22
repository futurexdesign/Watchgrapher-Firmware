/**
 * @file tick_detect.cpp
 * @brief Mechanical tick detector (event_detect block).
 *
 * Moved verbatim from the event half of {@code analyze()} in
 * {@code dsp_graph.cpp}. The open threshold, noise floor and matched-filter
 * veto now arrive in {@link tick_detect_block_params_t}, and accepted events
 * leave through a callback instead of a direct call to the graph's
 * {@code push_event}. The matched-filter confidence blend and template
 * learning stay in the graph (inside that callback).
 *
 * @see mechanical/tick_detect.h, dsp_graph.cpp
 */

#include "mechanical/tick_detect.h"

#include <cmath>

namespace {

/** Intra-tick local maxima retained for unlock / drop landmarks. */
constexpr uint32_t kMaxPulses = 8u;

/** EMA of accepted event peaks (for the min-peak gate). */
float g_tick_peak_ema = 0.05f;
float g_event_energy = 0.f;

float g_event_wave[TICK_DETECT_WAVE_LEN];
uint32_t g_event_wave_n = 0;

bool g_in_event = false;
uint64_t g_event_start = 0;
uint64_t g_event_onset = 0; /* first sample that opened the window */
uint64_t g_event_peak_idx = 0;
float g_event_peak = 0.f;
uint32_t g_below_count = 0;

uint64_t g_pulse_idx[kMaxPulses];
float g_pulse_frac[kMaxPulses]; /* parabolic sub-sample offset, -0.5..0.5 */
float g_pulse_amp[kMaxPulses];
uint32_t g_pulse_n = 0;
uint64_t g_last_pulse_idx = 0;
float g_prev_a = 0.f;
float g_prev2_a = 0.f;
uint64_t g_prev_idx = 0;
bool g_have_prev = false;
bool g_have_prev2 = false;

/**
 * Sub-sample position of a three-point local maximum {@code y0} between
 * neighbours {@code ym} (before) and {@code yp} (after): vertex of the
 * fitted parabola, in samples relative to the middle point.
 */
float parabolic_offset(float ym, float y0, float yp) {
  const float denom = ym - 2.f * y0 + yp;
  if (denom > -1e-12f) {
    return 0.f; /* flat or not a maximum */
  }
  const float d = 0.5f * (ym - yp) / denom;
  return fminf(0.5f, fmaxf(-0.5f, d));
}

void push_pulse(uint64_t idx, float frac, float amp, uint32_t refractory) {
  if (g_pulse_n > 0 && idx < g_last_pulse_idx + refractory) {
    /* Same pulse cluster: keep the louder of the two. */
    if (amp > g_pulse_amp[g_pulse_n - 1u]) {
      g_pulse_idx[g_pulse_n - 1u] = idx;
      g_pulse_frac[g_pulse_n - 1u] = frac;
      g_pulse_amp[g_pulse_n - 1u] = amp;
      g_last_pulse_idx = idx;
    }
    return;
  }
  if (g_pulse_n < kMaxPulses) {
    g_pulse_idx[g_pulse_n] = idx;
    g_pulse_frac[g_pulse_n] = frac;
    g_pulse_amp[g_pulse_n] = amp;
    g_pulse_n++;
    g_last_pulse_idx = idx;
  }
}

} // namespace

void tick_detect_reset() {
  g_in_event = false;
  g_below_count = 0;
  g_event_energy = 0.f;
  g_event_wave_n = 0;
  g_pulse_n = 0;
  g_have_prev = false;
  g_have_prev2 = false;
  g_tick_peak_ema = 0.05f;
}

bool tick_detect_in_event() { return g_in_event; }

const float *tick_detect_wave(uint32_t *n) {
  *n = g_event_wave_n;
  return g_event_wave;
}

void tick_detect_block(const float *x, uint32_t n, uint64_t base,
                       const tick_detect_block_params_t &params,
                       tick_detect_event_fn on_event) {
  const float thr_open = params.thr_open;
  const float thr_close = thr_open * 0.35f;
  const float min_peak = fmaxf(thr_open, 0.25f * g_tick_peak_ema);
  const auto fs = static_cast<float>(params.source_hz);
  auto preroll = static_cast<uint32_t>(params.preroll_ms * 0.001f * fs);
  auto tail = static_cast<uint32_t>(params.tail_ms * 0.001f * fs);
  auto max_event = static_cast<uint32_t>(0.080f * fs); /* 80 ms */
  /* ~0.75 ms refractory between intra-tick pulses (unlock / impulse / drop). */
  auto pulse_refrac = static_cast<uint32_t>(0.00075f * fs);
  if (pulse_refrac < 1u) {
    pulse_refrac = 1u;
  }

  for (uint32_t i = 0; i < n; i++) {
    float a = fabsf(x[i]);
    uint64_t idx = base + i;
    if (!g_in_event) {
      bool open_ok = a > thr_open;
      if (open_ok && params.open_veto) {
        open_ok = false;
      }
      if (open_ok) {
        g_in_event = true;
        g_event_start = (idx > preroll) ? (idx - preroll) : 0;
        g_event_onset = idx;
        g_event_peak_idx = idx;
        g_event_peak = a;
        g_event_energy = a * a;
        g_below_count = 0;
        g_event_wave_n = 0;
        g_pulse_n = 0;
        g_last_pulse_idx = 0;
        g_prev_a = a;
        g_prev2_a = 0.f;
        g_prev_idx = idx;
        g_have_prev = true;
        g_have_prev2 = false;
        if (g_event_wave_n < TICK_DETECT_WAVE_LEN) {
          g_event_wave[g_event_wave_n++] = x[i];
        }
      }
    } else {
      if (g_event_wave_n < TICK_DETECT_WAVE_LEN) {
        g_event_wave[g_event_wave_n++] = x[i];
      }
      g_event_energy += a * a;
      if (a > g_event_peak) {
        g_event_peak = a;
        g_event_peak_idx = idx;
      }

      /* Three-point local max → candidate unlock / impulse / drop pulse. */
      if (g_have_prev2 && g_have_prev) {
        const float pulse_thr = fmaxf(thr_close, 0.25f * g_event_peak);
        if (g_prev_a >= g_prev2_a && g_prev_a >= a && g_prev_a >= pulse_thr) {
          push_pulse(g_prev_idx, parabolic_offset(g_prev2_a, g_prev_a, a),
                     g_prev_a, pulse_refrac);
        }
      }
      g_prev2_a = g_prev_a;
      g_have_prev2 = g_have_prev;
      g_prev_a = a;
      g_prev_idx = idx;
      g_have_prev = true;

      if (a < thr_close) {
        g_below_count++;
      } else {
        g_below_count = 0;
      }
      const auto dur = static_cast<uint32_t>(idx - g_event_start);
      if (g_below_count > tail || dur >= max_event) {
        if (g_event_peak >= min_peak) {
          /* Flush a trailing local max if still rising into silence. */
          if (g_have_prev) {
            const float pulse_thr = fmaxf(thr_close, 0.25f * g_event_peak);
            if (g_prev_a >= pulse_thr &&
                (!g_have_prev2 || g_prev_a >= g_prev2_a)) {
              push_pulse(g_prev_idx, 0.f, g_prev_a, pulse_refrac);
            }
          }

          dsp_event_t ev{};
          ev.start_sample = g_event_start;
          ev.peak_sample = g_event_peak_idx;
          ev.duration_samples = dur;
          ev.rise_samples =
              static_cast<uint32_t>(g_event_peak_idx > g_event_start
                                        ? (g_event_peak_idx - g_event_start)
                                        : 0u);
          ev.peak_abs = g_event_peak;
          ev.noise_floor = params.noise_floor;
          ev.snr_db = 20.f * log10f(fmaxf(g_event_peak, 1e-9f) /
                                    fmaxf(params.noise_floor, 1e-9f));
          ev.energy = g_event_energy;
          /* Matched-filter blend is applied by the graph's callback. */
          ev.confidence = fminf(1.f, g_event_peak / (thr_open + 1e-9f) / 4.f);

          /* Unlock = first pulse; drop = loudest pulse (Witschi landmarks). */
          if (g_pulse_n > 0u) {
            ev.unlock_sample = g_pulse_idx[0];
            ev.unlock_frac = g_pulse_frac[0];
            uint32_t drop_i = 0;
            for (uint32_t p = 1; p < g_pulse_n; p++) {
              if (g_pulse_amp[p] > g_pulse_amp[drop_i]) {
                drop_i = p;
              }
            }
            ev.drop_sample = g_pulse_idx[drop_i];
            ev.drop_frac = g_pulse_frac[drop_i];
          } else {
            ev.unlock_sample = g_event_onset;
            ev.drop_sample = g_event_peak_idx;
          }
          if (ev.drop_sample > ev.unlock_sample) {
            ev.lift_samples =
                static_cast<uint32_t>(ev.drop_sample - ev.unlock_sample);
          } else {
            ev.lift_samples = 0;
            /* Single-pulse tick: still time rate/BE from unlock landmark. */
            if (ev.unlock_sample == 0) {
              ev.unlock_sample =
                  g_event_onset ? g_event_onset : g_event_peak_idx;
            }
            if (ev.drop_sample == 0) {
              ev.drop_sample = g_event_peak_idx;
            }
          }

          on_event(ev);
          g_tick_peak_ema = 0.9f * g_tick_peak_ema + 0.1f * g_event_peak;
        }
        g_in_event = false;
        g_below_count = 0;
        g_event_energy = 0.f;
        g_event_wave_n = 0;
        g_pulse_n = 0;
        g_have_prev = false;
        g_have_prev2 = false;
      }
    }
  }
}
