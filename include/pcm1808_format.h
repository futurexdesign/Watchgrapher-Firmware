/**
 * @file pcm1808_format.h
 * @brief PCM1808 left I²S slot → mono PCM24 conversion (header-only).
 *
 * Matches IMPLEMENTATION_SPEC P3.0 §5.5 vectors (updated for 24-bit USB).
 * Host unit tests mirror this logic in {@code tests/test_pcm1808_pcm_conversion.c}.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Converts one left I²S slot word to mono signed PCM24 (int32 container).
 *
 * The PCM1808 presents a signed 24-bit sample left-justified in a 32-bit
 * slot (valid audio in bits 31..8). This arithmetic-shifts to a sign-extended
 * 24-bit value with no further truncation.
 *
 * @param left_slot32 One stereo-frame left word as pushed by the PIO RX SM
 *                    (bit-endianness as captured on DOUT).
 * @return Sign-extended signed 24-bit PCM sample in an int32.
 */
static inline int32_t pcm1808_left_slot_to_pcm24(uint32_t left_slot32) {
  return static_cast<int32_t>(left_slot32) >> 8; /* arithmetic: sign-extend 24-bit */
}

#ifdef __cplusplus
}
#endif
