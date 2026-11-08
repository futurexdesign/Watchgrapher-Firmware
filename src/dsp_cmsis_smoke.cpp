// CMSIS-DSP smoke test: references Phase-1..3 kernels so a missing/mismatched
// ABI CMSIS-DSP build fails at link time.
//
// CMSIS-DSP is vendored under third_party/CMSIS-DSP and built from source for
// RP2350 envs only (see [rp2350_dsp] in platformio.ini).

#include "dsp_cmsis_smoke.h"

#if defined(ARM_MATH_CM33)

#include <arm_math.h>
#include <math.h>
#include <string.h>

bool dsp_cmsis_smoke() {
  static const float32_t bq_coeffs[5] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  static float32_t bq_state[4];
  arm_biquad_casd_df1_inst_f32 biquad;
  arm_biquad_cascade_df1_init_f32(&biquad, 1, bq_coeffs, bq_state);

  float32_t in[8] = {1, -2, 3, -4, 5, -6, 7, -8};
  float32_t out[8] = {0};
  arm_biquad_cascade_df1_f32(&biquad, in, out, 8);

  float32_t rms = 0.0f, power = 0.0f, peak = 0.0f, dot = 0.0f;
  uint32_t peak_idx = 0;
  arm_rms_f32(out, 8, &rms);
  arm_power_f32(out, 8, &power);
  arm_max_f32(out, 8, &peak, &peak_idx);
  arm_dot_prod_f32(in, out, 8, &dot);

  static float32_t fir_coeffs[4] = {1.f, 0.f, 0.f, 0.f};
  static float32_t fir_state[4 + 8];
  arm_fir_instance_f32 fir;
  arm_fir_init_f32(&fir, 4, fir_coeffs, fir_state, 8);
  float32_t fir_out[8];
  arm_fir_f32(&fir, in, fir_out, 8);

  arm_rfft_fast_instance_f32 rfft;
  if (arm_rfft_fast_init_f32(&rfft, 512) != ARM_MATH_SUCCESS) {
    return false;
  }
  static float32_t fft_buf[512];
  memset(fft_buf, 0, sizeof(fft_buf));
  fft_buf[0] = 1.f;
  arm_rfft_fast_f32(&rfft, fft_buf, fft_buf, 0);

  const float32_t expected_rms = sqrtf(204.0f / 8.0f);
  return fabsf(out[3] + 4.0f) < 1e-6f && fabsf(power - 204.0f) < 1e-3f &&
         fabsf(rms - expected_rms) < 1e-3f && peak == 7.0f && peak_idx == 6 &&
         fabsf(fir_out[0] - 1.f) < 1e-5f;
}

#else

bool dsp_cmsis_smoke() { return false; }

#endif
