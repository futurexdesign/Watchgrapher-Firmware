/**
 * @file arm_math.h
 * @brief Host implementation of the CMSIS-DSP subset used by the DSP chain.
 *
 * Plain float C++ with the CMSIS contracts: DF1 biquad coefficients
 * {b0, b1, b2, a1, a2} with {@code y = b0 x0 + b1 x1 + b2 x2 + a1 y1 + a2 y2};
 * FIR coefficients in reversed time order; real FFT output packed as
 * {@code out[0] = DC, out[1] = Nyquist, then re/im pairs} for bins 1..N/2-1.
 * The FFT is a direct O(N^2) transform with the 1/N scaling omitted, as in
 * CMSIS {@code arm_rfft_fast_f32} (forward, unscaled).
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

/** CMSIS status codes (subset). */
typedef enum { ARM_MATH_SUCCESS = 0, ARM_MATH_ARGUMENT_ERROR = -1 } arm_status;

/** DF1 biquad cascade instance. */
typedef struct {
  uint32_t numStages;
  float *pState;
  const float *pCoeffs;
} arm_biquad_casd_df1_inst_f32;

/** FIR instance. */
typedef struct {
  uint16_t numTaps;
  float *pState;
  const float *pCoeffs;
} arm_fir_instance_f32;

/** Real FFT instance. */
typedef struct {
  uint32_t fftLen;
} arm_rfft_fast_instance_f32;

/** Sum of squares of {@code n} samples. */
inline void arm_power_f32(const float *src, uint32_t n, float *result) {
  float acc = 0.f;
  for (uint32_t i = 0; i < n; i++) {
    acc += src[i] * src[i];
  }
  *result = acc;
}

/** Dot product of two {@code n}-sample vectors. */
inline void arm_dot_prod_f32(const float *a, const float *b, uint32_t n,
                             float *result) {
  float acc = 0.f;
  for (uint32_t i = 0; i < n; i++) {
    acc += a[i] * b[i];
  }
  *result = acc;
}

/** Initialises a DF1 cascade; {@code state} holds 4 words per stage. */
inline void arm_biquad_cascade_df1_init_f32(arm_biquad_casd_df1_inst_f32 *s,
                                            uint8_t stages, const float *coeffs,
                                            float *state) {
  s->numStages = stages;
  s->pCoeffs = coeffs;
  s->pState = state;
  for (uint32_t i = 0; i < 4u * stages; i++) {
    state[i] = 0.f;
  }
}

/** Runs a DF1 cascade; state per stage is {x[n-1], x[n-2], y[n-1], y[n-2]}. */
inline void arm_biquad_cascade_df1_f32(const arm_biquad_casd_df1_inst_f32 *s,
                                       const float *src, float *dst,
                                       uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    float v = src[i];
    for (uint32_t st = 0; st < s->numStages; st++) {
      const float *c = s->pCoeffs + 5u * st;
      float *z = s->pState + 4u * st;
      const float y =
          c[0] * v + c[1] * z[0] + c[2] * z[1] + c[3] * z[2] + c[4] * z[3];
      z[1] = z[0];
      z[0] = v;
      z[3] = z[2];
      z[2] = y;
      v = y;
    }
    dst[i] = v;
  }
}

/**
 * Initialises a FIR; {@code state} has {@code taps + block - 1} words.
 * Coefficients are in reversed time order: {@code coeffs[0]} multiplies the
 * oldest sample of the window.
 */
inline arm_status arm_fir_init_f32(arm_fir_instance_f32 *s, uint16_t taps,
                                   const float *coeffs, float *state,
                                   uint32_t block) {
  s->numTaps = taps;
  s->pCoeffs = coeffs;
  s->pState = state;
  for (uint32_t i = 0; i < taps + block - 1u; i++) {
    state[i] = 0.f;
  }
  return ARM_MATH_SUCCESS;
}

/** Runs the FIR over {@code n} samples (in-place allowed). */
inline void arm_fir_f32(const arm_fir_instance_f32 *s, const float *src,
                        float *dst, uint32_t n) {
  float *st = s->pState;
  const uint32_t taps = s->numTaps;
  /* State holds the taps-1 previous samples; append the block after them. */
  for (uint32_t i = 0; i < n; i++) {
    st[taps - 1u + i] = src[i];
  }
  for (uint32_t i = 0; i < n; i++) {
    float acc = 0.f;
    for (uint32_t k = 0; k < taps; k++) {
      acc += s->pCoeffs[k] * st[i + k];
    }
    dst[i] = acc;
  }
  /* Keep the newest taps-1 samples for the next call. */
  for (uint32_t i = 0; i < taps - 1u; i++) {
    st[i] = st[n + i];
  }
}

/** Initialises a real FFT of length {@code len} (any even length). */
inline arm_status arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32 *s,
                                         uint16_t len) {
  if (len < 4u || (len & 1u) != 0u) {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  s->fftLen = len;
  return ARM_MATH_SUCCESS;
}

/**
 * Forward real FFT, CMSIS packing. {@code in} is not modified here (CMSIS
 * clobbers it; callers must not rely on either). Only the forward transform
 * ({@code flag == 0}) is implemented.
 */
inline void arm_rfft_fast_f32(const arm_rfft_fast_instance_f32 *s,
                              const float *in, float *out, uint8_t flag) {
  const uint32_t n = s->fftLen;
  if (flag != 0u) {
    return;
  }
  const double w = -2.0 * 3.14159265358979323846 / static_cast<double>(n);
  for (uint32_t k = 0; k <= n / 2u; k++) {
    double re = 0.0;
    double im = 0.0;
    for (uint32_t t = 0; t < n; t++) {
      const double a =
          w * static_cast<double>((static_cast<uint64_t>(k) * t) % n);
      re += static_cast<double>(in[t]) * std::cos(a);
      im += static_cast<double>(in[t]) * std::sin(a);
    }
    if (k == 0u) {
      out[0] = static_cast<float>(re);
    } else if (k == n / 2u) {
      out[1] = static_cast<float>(re);
    } else {
      out[2u * k] = static_cast<float>(re);
      out[2u * k + 1u] = static_cast<float>(im);
    }
  }
}
