/* Host-side unit test for ADC → PCM16 conversion (mirrors adc_capture.cpp). */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#ifndef ADC_DC_OFFSET_COUNTS
#define ADC_DC_OFFSET_COUNTS 2048
#endif
#ifndef ADC_PCM_SCALE
#define ADC_PCM_SCALE 16
#endif

static int16_t adc_convert_sample(uint16_t fifo_word) {
  uint16_t code = (uint16_t)(fifo_word & 0x0FFFu);
  int32_t centered = (int32_t)code - (int32_t)ADC_DC_OFFSET_COUNTS;
  int32_t pcm = centered * (int32_t)ADC_PCM_SCALE;
  if (pcm > 32767) {
    pcm = 32767;
  }
  if (pcm < -32768) {
    pcm = -32768;
  }
  return (int16_t)pcm;
}

int main(void) {
  assert(adc_convert_sample(2048) == 0);
  assert(adc_convert_sample(2049) == 16);
  assert(adc_convert_sample(2047) == -16);
  assert(adc_convert_sample(0) == (int16_t)((0 - 2048) * 16));
  /* (4095-2048)*16 = 32752 — within int16, no saturate */
  assert(adc_convert_sample(4095) == 32752);
  /* Error flag ignored in this host mirror (bit15 cleared by mask). */
  assert(adc_convert_sample(0x8000u | 2048) == 0);
  printf("test_pcm_conversion: OK\n");
  return 0;
}
