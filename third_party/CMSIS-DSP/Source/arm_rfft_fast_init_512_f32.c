#include "dsp/transform_functions.h"
#include "arm_common_tables.h"

ARM_DSP_ATTRIBUTE arm_status arm_rfft_fast_init_512_f32(arm_rfft_fast_instance_f32 *S) {
  arm_status status;
  if (!S) return ARM_MATH_ARGUMENT_ERROR;
  status = arm_cfft_init_256_f32(&(S->Sint));
  if (status != ARM_MATH_SUCCESS) return status;
  S->fftLenRFFT = 512U;
  S->pTwiddleRFFT = (float32_t *)twiddleCoef_rfft_512;
  return ARM_MATH_SUCCESS;
}

ARM_DSP_ATTRIBUTE arm_status arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32 *S, uint16_t fftLen) {
  if (fftLen != 512U) return ARM_MATH_ARGUMENT_ERROR;
  return arm_rfft_fast_init_512_f32(S);
}
