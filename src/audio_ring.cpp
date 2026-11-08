/**
 * @file audio_ring.cpp
 * @brief Mono PCM24 (int32 container) ring buffer implementation.
 */

#include "audio_ring.h"

#include "diagnostics.h"

#include <limits.h>
#include <string.h>

static audio_sample_t g_ring[APP_RING_CAPACITY];
static volatile uint32_t g_write;
static volatile uint32_t g_read;

void audio_ring_init(void) { audio_ring_reset(); }

void audio_ring_reset(void) {
  g_write = 0;
  g_read = 0;
  memset(g_ring, 0, sizeof(g_ring));
  diag_counters_t *d = diagnostics_get();
  d->ring_min_occupancy = UINT32_MAX;
  d->ring_max_occupancy = 0;
}

static uint32_t occupancy_raw(void) {
  return (uint32_t)(g_write - g_read);
}

uint32_t audio_ring_occupancy(void) { return occupancy_raw(); }

uint32_t audio_ring_free(void) {
  uint32_t occ = occupancy_raw();
  if (occ >= APP_RING_CAPACITY) {
    return 0;
  }
  return APP_RING_CAPACITY - 1u - occ;
}

bool audio_ring_is_prefilled(void) {
  return occupancy_raw() >= APP_RING_PREFILL_SAMPLES;
}

uint32_t audio_ring_push(const audio_sample_t *samples, uint32_t count) {
  uint32_t written = 0;
  for (uint32_t i = 0; i < count; i++) {
    if (audio_ring_free() == 0) {
      diagnostics_get()->pcm_ring_overrun_count++;
      break;
    }
    g_ring[g_write % APP_RING_CAPACITY] = samples[i];
    g_write++;
    written++;
  }
  diagnostics_note_ring_occupancy(occupancy_raw());
  return written;
}

uint32_t audio_ring_pop(audio_sample_t *out, uint32_t count, bool pad_silence) {
  uint32_t read = 0;
  for (uint32_t i = 0; i < count; i++) {
    if (occupancy_raw() == 0) {
      diagnostics_get()->pcm_ring_underrun_count++;
      if (pad_silence) {
        out[i] = 0;
        read++;
        continue;
      }
      break;
    }
    out[i] = g_ring[g_read % APP_RING_CAPACITY];
    g_read++;
    read++;
  }
  diagnostics_note_ring_occupancy(occupancy_raw());
  return read;
}

uint32_t audio_ring_packet_samples(void) {
#if APP_FIXED_PACKET_48
  return APP_PKT_SAMPLES_NOM;
#else
  uint32_t occ = occupancy_raw();
  if (occ > (APP_RING_PREFILL_SAMPLES + APP_PKT_SAMPLES_NOM * 4u)) {
    return APP_PKT_SAMPLES_MAX;
  }
  if (occ < (APP_RING_PREFILL_SAMPLES / 2u)) {
    return APP_PKT_SAMPLES_MIN;
  }
  return APP_PKT_SAMPLES_NOM;
#endif
}
