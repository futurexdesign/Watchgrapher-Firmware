// CMSIS-DSP smoke test: references biquad_df1_f32, rms_f32, power_f32, max_f32
// so a missing/mismatched-ABI CMSIS-DSP build fails at link time.
//
// CMSIS-DSP is vendored under third_party/CMSIS-DSP and built from source for
// RP2350 envs only (see [rp2350_dsp] in platformio.ini). It inherits the
// toolchain's float ABI/FPU flags from Arduino-Pico (currently
// -mfloat-abi=softfp -mfpu=fpv5-sp-d16 for RP2350: HW FPU instructions, soft-float
// calling convention). Forcing -mfloat-abi=hard would not link with the framework libs.

#include "dsp_cmsis_smoke.h"

#if defined(ARM_MATH_CM33)

#include <arm_math.h>
#include <math.h>

bool dsp_cmsis_smoke() {
    // Passthrough biquad (b0=1, rest 0) must reproduce the input.
    static const float32_t coeffs[5] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static float32_t state[4];
    arm_biquad_casd_df1_inst_f32 biquad;
    arm_biquad_cascade_df1_init_f32(&biquad, 1, coeffs, state);

    float32_t in[8] = {1, -2, 3, -4, 5, -6, 7, -8};
    float32_t out[8] = {0};
    arm_biquad_cascade_df1_f32(&biquad, in, out, 8);

    float32_t rms = 0.0f, power = 0.0f, peak = 0.0f;
    uint32_t peak_idx = 0;
    arm_rms_f32(out, 8, &rms);
    arm_power_f32(out, 8, &power);
    arm_max_f32(out, 8, &peak, &peak_idx);

    // power = sum(x^2) = 204, rms = sqrt(204/8), max = 7 at index 6.
    const float32_t expected_rms = sqrtf(204.0f / 8.0f);
    return fabsf(out[3] + 4.0f) < 1e-6f && fabsf(power - 204.0f) < 1e-3f &&
           fabsf(rms - expected_rms) < 1e-3f && peak == 7.0f && peak_idx == 6;
}

#else

bool dsp_cmsis_smoke() { return false; }

#endif
