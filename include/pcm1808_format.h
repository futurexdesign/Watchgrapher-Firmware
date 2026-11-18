/**
 * @file pcm1808_format.h
 * @brief PCM1808 I²S slot word → signed PCM24 conversion (header-only).
 *
 * Matches IMPLEMENTATION_SPEC P3.0 §5.5 vectors (updated for 24-bit USB).
 * Covered by {@code tests/test_pcm1808_pcm_conversion.cpp}.
 */

#pragma once

#include <stdint.h>

/**
 * Converts one I²S slot word (left or right) to signed PCM24 (int32 container).
 *
 * The PCM1808 presents a signed 24-bit sample left-justified in a 32-bit
 * slot (valid audio in bits 31..8). This arithmetic-shifts to a sign-extended
 * 24-bit value with no further truncation.
 *
 * @param slot32 One stereo-frame slot word as pushed by the PIO RX SM
 *               (bit-endianness as captured on DOUT).
 * @return Sign-extended signed 24-bit PCM sample in an int32.
 */
static inline int32_t pcm1808_slot_to_pcm24(uint32_t slot32) {
  return static_cast<int32_t>(slot32) >> 8; /* arithmetic: sign-extend 24-bit */
}
