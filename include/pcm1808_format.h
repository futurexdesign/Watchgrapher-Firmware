/**
 * @file pcm1808_format.h
 * @brief PCM1808 left I²S slot → mono PCM16 conversion (header-only).
 *
 * Matches IMPLEMENTATION_SPEC P3.0 §5.5 vectors. Host unit tests mirror this
 * logic in {@code tests/test_pcm1808_pcm_conversion.c}.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Converts one left I²S slot word to mono PCM16.
 *
 * The PCM1808 presents a signed 24-bit sample left-justified in a 32-bit
 * slot (valid audio in bits 31..8). This shifts down to 16-bit and clamps.
 *
 * @param left_slot32 One stereo-frame left word as pushed by the PIO RX SM
 *                    (bit-endianness as captured on DOUT).
 * @return Clamped signed 16-bit PCM sample.
 */
static inline int16_t pcm1808_left_slot_to_pcm16(uint32_t left_slot32) {
  int32_t left24 = (int32_t)left_slot32 >> 8; /* arithmetic: sign-extend 24-bit */
  /* left24 is in -8388608..8388607 (top 24 of original); shift to 16-bit */
  int32_t pcm = left24 >> 8;
  if (pcm > 32767) {
    pcm = 32767;
  }
  if (pcm < -32768) {
    pcm = -32768;
  }
  return (int16_t)pcm;
}

#ifdef __cplusplus
}
#endif
