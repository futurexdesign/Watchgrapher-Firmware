/**
 * @file audio_source.cpp
 * @brief Stream state machine and backend dispatch (ADC / PCM1808 / synthetic).
 *
 * @see audio_source.h
 */

#include "audio_source.h"

#include "adc_capture.h"
#include "app_config.h"
#include "audio_ring.h"
#include "diagnostics.h"
#include "pcm1808_capture.h"
#include "pcm1808_clock.h"

#include <stdint.h>

static stream_state_t g_state = STREAM_STATE_USB_UNCONFIGURED;

/**
 * Capture start/stop is deferred to the main loop. Calling
 * {@code dma_channel_abort} / {@code adc_run} from a TinyUSB control callback
 * can hang the USB stack on RP2040/RP2350.
 */
static volatile bool g_capture_request;

#if AUDIO_SOURCE == SYNTHETIC
#include "tick_tock_pcm.h"

static uint32_t g_pcm_index;

/**
 * Fills the ring with {@code count} samples from the looping tick-tock asset.
 *
 * @param count Number of PCM16 samples to produce (may be chunked internally).
 */
static void synthetic_generate(uint32_t count) {
  int16_t tmp[64];
  while (count > 0) {
    uint32_t n = count > 64 ? 64 : count;
    for (uint32_t i = 0; i < n; i++) {
      tmp[i] = kTickTockPcm[g_pcm_index];
      g_pcm_index++;
      if (g_pcm_index >= TICK_TOCK_PCM_LEN) {
        g_pcm_index = 0;
      }
    }
    audio_ring_push(tmp, n);
    count -= n;
  }
}
#endif

/**
 * Applies {@code g_capture_request}: start capture if requested and idle, or
 * stop + reset the ring if request cleared while running.
 */
static void capture_service_request(void) {
#if AUDIO_SOURCE == ADC
  if (g_capture_request) {
    if (!adc_capture_is_running()) {
      adc_capture_start();
    }
  } else if (adc_capture_is_running()) {
    adc_capture_stop();
    audio_ring_reset();
  }
#elif AUDIO_SOURCE == PCM1808
  if (g_capture_request) {
    if (!pcm1808_capture_is_running()) {
      pcm1808_capture_start();
    }
  } else if (pcm1808_capture_is_running()) {
    pcm1808_capture_stop();
    audio_ring_reset();
  }
#endif
}

void audio_source_init(void) {
  g_state = STREAM_STATE_USB_UNCONFIGURED;
  g_capture_request = false;
  audio_ring_init();
#if AUDIO_SOURCE == ADC
  adc_capture_init();
#elif AUDIO_SOURCE == PCM1808
  if (!pcm1808_clock_start()) {
    g_state = STREAM_STATE_STREAM_ERROR;
  }
  pcm1808_capture_init();
#elif AUDIO_SOURCE == SYNTHETIC
  g_pcm_index = 0;
#endif
}

void audio_source_set_configured(bool configured) {
  if (configured) {
    if (g_state == STREAM_STATE_USB_UNCONFIGURED ||
        g_state == STREAM_STATE_USB_SUSPENDED) {
      g_state = STREAM_STATE_USB_CONFIGURED_IDLE;
    }
  } else {
    g_capture_request = false;
#if AUDIO_SOURCE == ADC
    adc_capture_stop();
#elif AUDIO_SOURCE == PCM1808
    pcm1808_capture_stop();
#endif
    audio_ring_reset();
    g_state = STREAM_STATE_USB_UNCONFIGURED;
  }
}

void audio_source_on_stream_open(void) {
  audio_ring_reset();
  diagnostics_reset_adc_stats();
#if AUDIO_SOURCE == SYNTHETIC
  g_pcm_index = 0;
#endif
  g_capture_request = true;
  g_state = STREAM_STATE_STREAM_PREFILL;
}

void audio_source_on_stream_close(void) {
  g_capture_request = false;
  g_state = STREAM_STATE_USB_CONFIGURED_IDLE;
}

void audio_source_on_suspend(void) {
  g_capture_request = false;
  g_state = STREAM_STATE_USB_SUSPENDED;
}

void audio_source_on_resume(void) {
  if (g_state == STREAM_STATE_USB_SUSPENDED) {
    g_state = STREAM_STATE_USB_CONFIGURED_IDLE;
  }
}

void audio_source_process(void) {
  capture_service_request();

  if (g_state != STREAM_STATE_STREAM_PREFILL &&
      g_state != STREAM_STATE_STREAM_ACTIVE) {
    return;
  }

#if AUDIO_SOURCE == ADC
  adc_capture_process();
#elif AUDIO_SOURCE == PCM1808
  pcm1808_capture_process();
#elif AUDIO_SOURCE == SYNTHETIC
  const uint32_t target = APP_RING_PREFILL_SAMPLES + APP_PKT_SAMPLES_NOM * 10u;
  uint32_t occ = audio_ring_occupancy();
  if (occ < target) {
    synthetic_generate(target - occ);
  }
#endif

  if (g_state == STREAM_STATE_STREAM_PREFILL && audio_ring_is_prefilled()) {
    g_state = STREAM_STATE_STREAM_ACTIVE;
  }
}

stream_state_t audio_source_state(void) { return g_state; }
