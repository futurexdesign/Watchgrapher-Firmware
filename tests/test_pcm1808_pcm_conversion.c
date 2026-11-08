/* Host unit test for PCM1808 left_slot32 → PCM24 (mirrors pcm1808_format.h). */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static int32_t pcm1808_left_slot_to_pcm24(uint32_t left_slot32) {
  return (int32_t)left_slot32 >> 8;
}

static int32_t convert_stereo_left_only(uint32_t left, uint32_t right) {
  (void)right;
  return pcm1808_left_slot_to_pcm24(left);
}

int main(void) {
  /* Mandatory vectors from IMPLEMENTATION_SPEC P3.0 §5.5 (24-bit native). */
  assert(pcm1808_left_slot_to_pcm24(0x00000000u) == 0);
  assert(pcm1808_left_slot_to_pcm24(0x7FFFFF00u) == 8388607);
  assert(pcm1808_left_slot_to_pcm24(0x80000000u) == (int32_t)0xFF800000);
  assert(pcm1808_left_slot_to_pcm24(0xFFFFFF00u) == (int32_t)-1);
  assert(pcm1808_left_slot_to_pcm24(0x00000100u) == 1);
  assert(pcm1808_left_slot_to_pcm24(0xFFFFFE00u) == (int32_t)-2);

  /* Right-slot data must not affect mono output. */
  assert(convert_stereo_left_only(0x01000000u, 0x7FFFFF00u) ==
         pcm1808_left_slot_to_pcm24(0x01000000u));
  assert(convert_stereo_left_only(0x01000000u, 0x80000000u) ==
         pcm1808_left_slot_to_pcm24(0x01000000u));
  assert(convert_stereo_left_only(0x01000000u, 0xFFFFFFFFu) == 65536);

  /* One-bit I²S shift error: MSB-aligned full-scale becomes half-scale-ish. */
  assert(pcm1808_left_slot_to_pcm24(0x3FFFFF80u) != 8388607);
  assert(pcm1808_left_slot_to_pcm24(0x3FFFFF80u) == 4194303);

  printf("test_pcm1808_pcm_conversion: OK\n");
  return 0;
}
