/**
 * @file audio_source.cpp
 * @brief Stream FSM, capture deferral, and preamp warm-up gating.
 */

#include "audio_source.h"

#include "adc_capture.h"
#include "app_config.h"
#include "audio_ring.h"
#include "board_config.h"
#include "diagnostics.h"
#include "pcm1808_capture.h"
#include "pcm1808_clock.h"
#include "power_control.h"

#include <Arduino.h>
#include <stdint.h>

static stream_state_t g_state = STREAM_STATE_USB_UNCONFIGURED;
static volatile bool g_capture_request;
static volatile bool g_stream_armed;
static bool g_awaiting_preamp;

#if AUDIO_SOURCE == SYNTHETIC
#include "tick_tock_pcm.h"
static uint32_t g_pcm_index;

static void synthetic_generate(uint32_t count) {
  constexpr uint32_t kChunk = 64u;
  audio_sample_t tmp[kChunk];
  while (count > 0) {
    uint32_t n = count > kChunk ? kChunk : count;
    for (uint32_t i = 0; i < n; i++) {
      /* S16 source → MSB-aligned PCM24 for USB. */
      tmp[i] = (static_cast<audio_sample_t>(kTickTockPcm[g_pcm_index++])) << 8;
      if (g_pcm_index >= TICK_TOCK_PCM_LEN) {
        g_pcm_index = 0;
      }
    }
    audio_ring_push(tmp, n);
    count -= n;
  }
}
#endif

static bool uses_preamp(void) {
#if AUDIO_SOURCE == SYNTHETIC
  return false;
#else
  return BOOST_SWITCH_PRESENT != 0;
#endif
}

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

static void power_down(void) {
  g_capture_request = false;
  g_awaiting_preamp = false;
  power_control_abort_preamp(millis());
}

static void enter_prefill(void) {
  audio_ring_reset();
  diagnostics_reset_adc_stats();
#if AUDIO_SOURCE == SYNTHETIC
  g_pcm_index = 0;
#endif
  g_awaiting_preamp = false;
  g_capture_request = (AUDIO_SOURCE != SYNTHETIC);
  g_state = STREAM_STATE_STREAM_PREFILL;
}

static void arm_recording(void) {
#if AUDIO_SOURCE == PCM1808
  if (!pcm1808_clock_is_ok() || g_state == STREAM_STATE_STREAM_ERROR) {
    g_stream_armed = false;
    power_down();
    g_state = STREAM_STATE_STREAM_ERROR;
    return;
  }
#endif
  g_stream_armed = true;
  g_capture_request = false;
  audio_ring_reset();
  diagnostics_reset_adc_stats();
#if AUDIO_SOURCE == SYNTHETIC
  g_pcm_index = 0;
#endif
  if (uses_preamp()) {
    g_awaiting_preamp = true;
    g_state = STREAM_STATE_STREAM_PREFILL;
    power_control_begin_preamp_warmup(millis());
  } else {
    power_control_abort_preamp(millis());
    enter_prefill();
  }
}

void audio_source_init(void) {
  g_state = STREAM_STATE_USB_UNCONFIGURED;
  g_capture_request = false;
  g_stream_armed = false;
  g_awaiting_preamp = false;
  audio_ring_init();
#if AUDIO_SOURCE == ADC
  adc_capture_init();
#elif AUDIO_SOURCE == PCM1808
  if (!pcm1808_clock_start()) {
    g_state = STREAM_STATE_STREAM_ERROR;
    power_down();
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
    g_stream_armed = false;
    power_down();
#if AUDIO_SOURCE == ADC
    adc_capture_stop();
#elif AUDIO_SOURCE == PCM1808
    pcm1808_capture_stop();
#endif
    audio_ring_reset();
    g_state = STREAM_STATE_USB_UNCONFIGURED;
  }
}

void audio_source_on_stream_open(void) { arm_recording(); }

void audio_source_on_stream_close(void) {
  g_stream_armed = false;
  power_down();
  g_state = STREAM_STATE_USB_CONFIGURED_IDLE;
}

void audio_source_on_suspend(void) {
  power_down();
  g_state = STREAM_STATE_USB_SUSPENDED;
}

void audio_source_on_resume(void) {
  if (g_state != STREAM_STATE_USB_SUSPENDED) {
    return;
  }
  if (g_stream_armed) {
    arm_recording();
  } else {
    g_state = STREAM_STATE_USB_CONFIGURED_IDLE;
  }
}

void audio_source_process(void) {
  uint32_t now = millis();
  if (g_awaiting_preamp) {
    (void)power_control_poll(now);
    if (power_control_preamp_ready()) {
      enter_prefill();
    }
  }
  capture_service_request();

  if (g_state == STREAM_STATE_STREAM_ERROR) {
    power_down();
    return;
  }
  if (g_state != STREAM_STATE_STREAM_PREFILL &&
      g_state != STREAM_STATE_STREAM_ACTIVE) {
    return;
  }
  if (g_awaiting_preamp) {
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
