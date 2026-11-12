/**
 * @file audio_ring.h
 * @brief Lock-free-ish mono PCM24 ring between capture and USB IN feeder.
 *
 * Samples are signed 24-bit values stored in {@code int32_t} (sign-extended).
 * USB packs them to 3-byte little-endian on the wire. Single producer /
 * single consumer; capacity must be a power of two.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One mono sample: signed 24-bit in an int32 container. */
typedef int32_t audio_sample_t;

void audio_ring_init(void);
void audio_ring_reset(void);
uint32_t audio_ring_occupancy(void);
uint32_t audio_ring_free(void);
bool audio_ring_is_prefilled(void);

/**
 * Producer: append PCM24 samples (int32 containers).
 * @return Samples written (≤ {@code count}).
 */
uint32_t audio_ring_push(const audio_sample_t *samples, uint32_t count);

/**
 * Consumer: pop up to {@code count} samples.
 * @return Samples written to {@code out}.
 */
uint32_t audio_ring_pop(audio_sample_t *out, uint32_t count, bool pad_silence);

uint32_t audio_ring_packet_samples(void);

/**
 * Packs one signed 24-bit sample to 3-byte little-endian USB PCM.
 * Clamps to the 24-bit range before packing.
 */
static inline void audio_pcm24_pack_le(uint8_t dst[3], audio_sample_t s) {
  if (s > static_cast<audio_sample_t>(APP_PCM24_FULL_SCALE)) {
    s = static_cast<audio_sample_t>(APP_PCM24_FULL_SCALE);
  } else if (s < static_cast<audio_sample_t>(APP_PCM24_MIN)) {
    s = static_cast<audio_sample_t>(APP_PCM24_MIN);
  }
  dst[0] = static_cast<uint8_t>(static_cast<uint32_t>(s) & 0xFFu);
  dst[1] = static_cast<uint8_t>((static_cast<uint32_t>(s) >> 8) & 0xFFu);
  dst[2] = static_cast<uint8_t>((static_cast<uint32_t>(s) >> 16) & 0xFFu);
}

#ifdef __cplusplus
}
#endif
