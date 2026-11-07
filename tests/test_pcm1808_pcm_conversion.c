/* Host unit test for PCM1808 left_slot32 → PCM16 (mirrors pcm1808_format.h). */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static int16_t pcm1808_left_slot_to_pcm16(uint32_t left_slot32) {
  int32_t left24 = (int32_t)left_slot32 >> 8;
  int32_t pcm = left24 >> 8;
  if (pcm > 32767) {
    pcm = 32767;
  }
  if (pcm < -32768) {
    pcm = -32768;
  }
  return (int16_t)pcm;
}

static int16_t convert_stereo_left_only(uint32_t left, uint32_t right) {
  (void)right;
  return pcm1808_left_slot_to_pcm16(left);
}

int main(void) {
  /* Mandatory vectors from IMPLEMENTATION_SPEC P3.0 §5.5 */
  assert(pcm1808_left_slot_to_pcm16(0x00000000u) == 0);
  assert(pcm1808_left_slot_to_pcm16(0x7FFFFF00u) == 32767);
  assert(pcm1808_left_slot_to_pcm16(0x80000000u) == (int16_t)-32768);
  assert(pcm1808_left_slot_to_pcm16(0xFFFFFF00u) == (int16_t)-1);
  assert(pcm1808_left_slot_to_pcm16(0x00000100u) == 0);
  assert(pcm1808_left_slot_to_pcm16(0xFFFFFE00u) == (int16_t)-1);

  /* Right-slot data must not affect mono output. */
  assert(convert_stereo_left_only(0x01000000u, 0x7FFFFF00u) ==
         pcm1808_left_slot_to_pcm16(0x01000000u));
  assert(convert_stereo_left_only(0x01000000u, 0x80000000u) ==
         pcm1808_left_slot_to_pcm16(0x01000000u));
  assert(convert_stereo_left_only(0x01000000u, 0xFFFFFFFFu) == 256);

  /* One-bit I²S shift error: MSB-aligned full-scale becomes half-scale-ish. */
  assert(pcm1808_left_slot_to_pcm16(0x3FFFFF80u) != 32767);
  assert(pcm1808_left_slot_to_pcm16(0x3FFFFF80u) == 16383);

  printf("test_pcm1808_pcm_conversion: OK\n");
  return 0;
}
