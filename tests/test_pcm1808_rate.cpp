/**
 * @file test_pcm1808_rate.cpp
 * @brief Host test for PCM1808 frame-rate classification.
 *
 * Build and run:
 *   c++ -std=c++17 -Iinclude tests/test_pcm1808_rate.cpp -o /tmp/t && /tmp/t
 */

#include "pcm1808_rate.h"

#include <cassert>
#include <cstdio>

int main() {
  constexpr uint32_t kMclk = 12288000u;
  constexpr uint32_t kUsb = 48000u;

  /* 256 fs → 48 kHz, no repeat. */
  auto m = pcm1808_classify_rate(48000.4, kMclk, kUsb);
  assert(m.status == PCM1808_RATE_OK && m.mclk_ratio == 256u &&
         m.nominal_hz == 48000u && m.upsample == 1u);

  /* 512 fs → 24 kHz, repeat ×2 (the bench board today). */
  m = pcm1808_classify_rate(23998.7, kMclk, kUsb);
  assert(m.status == PCM1808_RATE_OK && m.mclk_ratio == 512u &&
         m.nominal_hz == 24000u && m.upsample == 2u);

  /* 384 fs → 32 kHz: recognised, but 48k/32k is not an integer repeat. */
  m = pcm1808_classify_rate(32001.0, kMclk, kUsb);
  assert(m.status == PCM1808_RATE_UNSUPPORTED && m.mclk_ratio == 384u &&
         m.nominal_hz == 32000u && m.upsample == 0u);

  /* Tolerance edges (0.5 %). */
  assert(pcm1808_classify_rate(24110.0, kMclk, kUsb).status == PCM1808_RATE_OK);
  assert(pcm1808_classify_rate(24130.0, kMclk, kUsb).status ==
         PCM1808_RATE_UNKNOWN);

  /* No clock / garbage. */
  m = pcm1808_classify_rate(0.0, kMclk, kUsb);
  assert(m.status == PCM1808_RATE_UNKNOWN && m.nominal_hz == 0u);
  m = pcm1808_classify_rate(44100.0, kMclk, kUsb);
  assert(m.status == PCM1808_RATE_UNKNOWN);

  std::printf("test_pcm1808_rate: OK\n");
  return 0;
}
