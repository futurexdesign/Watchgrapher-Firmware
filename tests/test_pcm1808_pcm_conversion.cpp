/**
 * @file test_pcm1808_pcm_conversion.cpp
 * @brief Host test for PCM1808 slot32 → PCM24 conversion (real header).
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_pcm1808_pcm_conversion.cpp -o /tmp/t && /tmp/t
 */

#include "pcm1808_format.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

/** Mono path keeps the left slot only; the right slot must not leak in. */
static int32_t convert_stereo_left_only(uint32_t left, uint32_t right) {
  (void)right;
  return pcm1808_slot_to_pcm24(left);
}

int main() {
  /* Mandatory vectors from IMPLEMENTATION_SPEC P3.0 §5.5 (24-bit native). */
  assert(pcm1808_slot_to_pcm24(0x00000000u) == 0);
  assert(pcm1808_slot_to_pcm24(0x7FFFFF00u) == 8388607);
  assert(pcm1808_slot_to_pcm24(0x80000000u) == -8388608);
  assert(pcm1808_slot_to_pcm24(0xFFFFFF00u) == -1);
  assert(pcm1808_slot_to_pcm24(0x00000100u) == 1);
  assert(pcm1808_slot_to_pcm24(0xFFFFFE00u) == -2);

  /* Right-slot data must not affect mono output. */
  assert(convert_stereo_left_only(0x01000000u, 0x7FFFFF00u) ==
         pcm1808_slot_to_pcm24(0x01000000u));
  assert(convert_stereo_left_only(0x01000000u, 0x80000000u) ==
         pcm1808_slot_to_pcm24(0x01000000u));
  assert(convert_stereo_left_only(0x01000000u, 0xFFFFFFFFu) == 65536);

  /* One-bit I²S shift error: MSB-aligned full-scale becomes half-scale-ish. */
  assert(pcm1808_slot_to_pcm24(0x3FFFFF80u) != 8388607);
  assert(pcm1808_slot_to_pcm24(0x3FFFFF80u) == 4194303);

  std::printf("test_pcm1808_pcm_conversion: OK\n");
  return 0;
}
