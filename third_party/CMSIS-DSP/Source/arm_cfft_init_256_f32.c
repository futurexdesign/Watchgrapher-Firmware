#include "dsp/transform_functions.h"
#include "arm_const_structs.h"

ARM_DSP_ATTRIBUTE arm_status arm_cfft_init_256_f32(arm_cfft_instance_f32 *S) {
  if (!S) return ARM_MATH_ARGUMENT_ERROR;
  S->fftLen = 256;
  S->pTwiddle = arm_cfft_sR_f32_len256.pTwiddle;
  S->pBitRevTable = arm_cfft_sR_f32_len256.pBitRevTable;
  S->bitRevLength = arm_cfft_sR_f32_len256.bitRevLength;
  return ARM_MATH_SUCCESS;
}
