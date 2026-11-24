/**
 * @file tick_detect.cpp
 * @brief Mechanical tick detector (event_detect block).
 *
 * Moved verbatim from the event half of {@code analyze()} in
 * {@code dsp_graph.cpp}. The open threshold and noise floor now arrive in
 * {@link tick_detect_block_params_t}, and accepted events leave through a
 * callback instead of a direct call to the graph's {@code push_event}.
 *
 * @see mechanical/tick_detect.h, dsp_graph.cpp
 */

#include "mechanical/tick_detect.h"

#include <cmath>

namespace {

/** Intra-tick local maxima retained for unlock / drop landmarks. */
constexpr uint32_t kMaxPulses = 8u;
/**
 * Longest pulse-strength window in samples (0.5 ms at up to 128 kHz). The
 * strength of a pulse is the mean |x| over the last 0.5 ms, not its single
 * peak sample: in a clipped recording every loud pulse hits the same ceiling,
 * and the unlock (first pulse) won the "loudest pulse" tie, so the lift time
 * came out 0 and the amplitude invalid (ST3600 clip, 2026-11-23).
 */
constexpr uint32_t kStrengthMax = 64u;

/** EMA of accepted event peaks (for the min-peak gate). */
float g_tick_peak_ema = 0.05f;
float g_event_energy = 0.f;

bool g_in_event = false;
uint64_t g_event_start = 0;
uint64_t g_event_onset = 0; /* first sample that opened the window */
uint64_t g_event_peak_idx = 0;
float g_event_peak = 0.f;
uint32_t g_below_count = 0;

uint64_t g_pulse_idx[kMaxPulses];
float g_pulse_frac[kMaxPulses]; /* parabolic sub-sample offset, -0.5..0.5 */
float g_pulse_amp[kMaxPulses];
/** 0.5 ms mean of |x| at each pulse: ranks the drop candidates. */
float g_pulse_strength[kMaxPulses];
uint32_t g_pulse_n = 0;
uint64_t g_last_pulse_idx = 0;
float g_prev_a = 0.f;
float g_prev2_a = 0.f;
uint64_t g_prev_idx = 0;
bool g_have_prev = false;
bool g_have_prev2 = false;

/** Ring of the last {@code g_strength_len} values of |x|, and their sum. */
float g_strength_ring[kStrengthMax];
float g_strength_sum = 0.f;
uint32_t g_strength_pos = 0;
uint32_t g_strength_len = 1;

/**
 * Adds one |x| to the running pulse-strength window.
 *
 * @return Mean |x| over the window, the strength of a pulse found now.
 */
float strength_update(float a) {
  g_strength_sum += a - g_strength_ring[g_strength_pos];
  g_strength_ring[g_strength_pos] = a;
  g_strength_pos++;
  if (g_strength_pos >= g_strength_len) {
    g_strength_pos = 0;
    /* Re-sum once per window so float rounding cannot accumulate. */
    float s = 0.f;
    for (uint32_t i = 0; i < g_strength_len; i++) {
      s += g_strength_ring[i];
    }
    g_strength_sum = s;
  }
  return g_strength_sum / static_cast<float>(g_strength_len);
}

/** Sets the window to 0.5 ms at {@code fs}; clears it when the length changes.
 */
void strength_configure(float fs) {
  auto len = static_cast<uint32_t>(0.0005f * fs + 0.5f);
  len = len < 1u ? 1u : (len > kStrengthMax ? kStrengthMax : len);
  if (len != g_strength_len) {
    g_strength_len = len;
    g_strength_pos = 0;
    g_strength_sum = 0.f;
    for (float &v : g_strength_ring) {
      v = 0.f;
    }
  }
}

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

/**
 * Records an intra-tick pulse.
 *
 * Pulses closer than {@code refractory} merge (the louder peak keeps the
 * position, the cluster keeps its highest strength). When the list is full,
 * the first pulse (the unlock) always stays and a new pulse replaces the
 * weakest later one if it is stronger, so a busy or clipped unlock group
 * cannot crowd the drop out of the list.
 */
void push_pulse(uint64_t idx, float frac, float amp, float strength,
                uint32_t refractory) {
  if (g_pulse_n > 0 && idx < g_last_pulse_idx + refractory) {
    /* Same pulse cluster. */
    const uint32_t last = g_pulse_n - 1u;
    if (amp > g_pulse_amp[last]) {
      g_pulse_idx[last] = idx;
      g_pulse_frac[last] = frac;
      g_pulse_amp[last] = amp;
      g_last_pulse_idx = idx;
    }
    g_pulse_strength[last] = fmaxf(g_pulse_strength[last], strength);
    return;
  }
  uint32_t slot = g_pulse_n;
  if (g_pulse_n == kMaxPulses) {
    slot = 1u;
    for (uint32_t p = 2; p < kMaxPulses; p++) {
      if (g_pulse_strength[p] < g_pulse_strength[slot]) {
        slot = p;
      }
    }
    if (strength <= g_pulse_strength[slot]) {
      return;
    }
  } else {
    g_pulse_n++;
  }
  g_pulse_idx[slot] = idx;
  g_pulse_frac[slot] = frac;
  g_pulse_amp[slot] = amp;
  g_pulse_strength[slot] = strength;
  g_last_pulse_idx = idx;
}

} // namespace

void tick_detect_reset() {
  g_in_event = false;
  g_below_count = 0;
  g_event_energy = 0.f;
  g_pulse_n = 0;
  g_have_prev = false;
  g_have_prev2 = false;
  g_tick_peak_ema = 0.05f;
  g_strength_sum = 0.f;
  g_strength_pos = 0;
  for (float &v : g_strength_ring) {
    v = 0.f;
  }
}

bool tick_detect_in_event() { return g_in_event; }

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
  strength_configure(fs);

  for (uint32_t i = 0; i < n; i++) {
    float a = fabsf(x[i]);
    uint64_t idx = base + i;
    const float strength = strength_update(a);
    if (!g_in_event) {
      if (a > thr_open) {
        g_in_event = true;
        g_event_start = (idx > preroll) ? (idx - preroll) : 0;
        g_event_onset = idx;
        g_event_peak_idx = idx;
        g_event_peak = a;
        g_event_energy = a * a;
        g_below_count = 0;
        g_pulse_n = 0;
        g_last_pulse_idx = 0;
        g_prev_a = a;
        g_prev2_a = 0.f;
        g_prev_idx = idx;
        g_have_prev = true;
        g_have_prev2 = false;
      }
    } else {
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
                     g_prev_a, strength, pulse_refrac);
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
              push_pulse(g_prev_idx, 0.f, g_prev_a, strength, pulse_refrac);
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
          ev.confidence = fminf(1.f, g_event_peak / (thr_open + 1e-9f) / 4.f);

          /* Unlock = first pulse; drop = strongest pulse (Witschi
           * landmarks). Strength is the 0.5 ms mean |x|, so a clipped
           * recording still tells the drop burst from the unlock. The
           * pulses are not in time order once the list has been full. */
          if (g_pulse_n > 0u) {
            ev.unlock_sample = g_pulse_idx[0];
            ev.unlock_frac = g_pulse_frac[0];
            uint32_t drop_i = 0;
            for (uint32_t p = 1; p < g_pulse_n; p++) {
              if (g_pulse_strength[p] > g_pulse_strength[drop_i]) {
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
        g_pulse_n = 0;
        g_have_prev = false;
        g_have_prev2 = false;
      }
    }
  }
}
