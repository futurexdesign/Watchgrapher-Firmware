/**
 * @file audio_source.cpp
 * @brief Stream FSM, capture deferral, and preamp warm-up gating.
 */

#include "audio_source.h"

#include "adc_capture.h"
#include "app_config.h"
#include "audio_ring.h"
#include "board_config.h"
#include "device_mode.h"
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
/** False when the capture backend failed to initialize (sticky until reset). */
static bool g_backend_ok = true;

/**
 * Progress of a run-without-stream (host/CLI {@code start} latch or
 * STANDALONE mode). Only meaningful while no USB stream owns the pipeline.
 */
enum class RunPhase : uint8_t {
  Off,    /**< Not running on behalf of the run latch. */
  Warmup, /**< Waiting for the preamp rail to settle. */
  On      /**< Capture requested and being processed. */
};
static RunPhase g_run_phase = RunPhase::Off;
/** Host/CLI start latch; capture runs while this or the USB stream is set. */
static volatile bool g_run_requested;
/** True between {@link audio_source_capture_pause} and its resume. */
static bool g_paused_was_running;

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
  g_run_phase = RunPhase::Off;
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

static bool backend_healthy(void) {
#if AUDIO_SOURCE == PCM1808
  return g_backend_ok && pcm1808_clock_is_ok();
#else
  return g_backend_ok;
#endif
}

static void arm_recording(void) {
  if (!backend_healthy()) {
    g_stream_armed = false;
    power_down();
    g_state = STREAM_STATE_STREAM_ERROR;
    return;
  }
  g_stream_armed = true;
  /* The USB stream takes over from any run-without-stream. */
  g_run_phase = RunPhase::Off;
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

/**
 * The run latch drives the pipeline when it is set and no USB stream does:
 * either no stream is armed, or the bus is suspended (the stream is torn
 * down, a standalone device keeps measuring).
 */
static bool run_owns_pipeline(void) {
  return g_run_requested &&
         (!g_stream_armed || g_state == STREAM_STATE_USB_SUSPENDED);
}

/** Requests capture for the run latch (the preamp, if any, is ready). */
static void run_begin_capture(void) {
  g_capture_request = (AUDIO_SOURCE != SYNTHETIC);
  g_run_phase = RunPhase::On;
}

/**
 * Starts a run without a USB stream: same health check and preamp warm-up as
 * {@link arm_recording}, but progress never waits for the ring prefill.
 * An unhealthy backend drops the latch (the status then shows the error).
 */
static void run_arm(uint32_t now) {
  if (!backend_healthy()) {
    g_run_requested = false;
    return;
  }
  audio_ring_reset();
  diagnostics_reset_adc_stats();
#if AUDIO_SOURCE == SYNTHETIC
  g_pcm_index = 0;
#endif
  if (uses_preamp()) {
    g_run_phase = RunPhase::Warmup;
    power_control_begin_preamp_warmup(now);
  } else {
    power_control_abort_preamp(now);
    run_begin_capture();
  }
}

/** Advances the run-without-stream state machine; called every tick. */
static void run_supervise(uint32_t now) {
  if (!run_owns_pipeline()) {
    if (g_run_phase != RunPhase::Off) {
      power_down(); /* latch cleared: capture off, preamp off, phase Off */
    }
    return;
  }
  if (g_run_phase == RunPhase::Off) {
    run_arm(now);
  } else if (g_run_phase == RunPhase::Warmup) {
    (void)power_control_poll(now);
    if (power_control_preamp_ready()) {
      run_begin_capture();
    }
  }
}

/** @return {@code true} if the hardware backend is capturing. */
static bool backend_running(void) {
#if AUDIO_SOURCE == ADC
  return adc_capture_is_running();
#elif AUDIO_SOURCE == PCM1808
  return pcm1808_capture_is_running();
#else
  return g_state == STREAM_STATE_STREAM_PREFILL ||
         g_state == STREAM_STATE_STREAM_ACTIVE || g_run_phase == RunPhase::On;
#endif
}

void audio_source_init(void) {
  g_state = STREAM_STATE_USB_UNCONFIGURED;
  g_capture_request = false;
  g_stream_armed = false;
  g_awaiting_preamp = false;
  g_run_phase = RunPhase::Off;
  g_run_requested = false;
  g_paused_was_running = false;
  audio_ring_init();
#if AUDIO_SOURCE == ADC
  g_backend_ok = adc_capture_init();
#elif AUDIO_SOURCE == PCM1808
  g_backend_ok = pcm1808_clock_start();
  g_backend_ok = pcm1808_capture_init() && g_backend_ok;
#elif AUDIO_SOURCE == SYNTHETIC
  g_pcm_index = 0;
#endif
  if (!g_backend_ok) {
    g_state = STREAM_STATE_STREAM_ERROR;
    power_down();
  }
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
  run_supervise(now);
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
  const bool stream_run = (g_state == STREAM_STATE_STREAM_PREFILL ||
                           g_state == STREAM_STATE_STREAM_ACTIVE) &&
                          !g_awaiting_preamp;
  if (!stream_run && g_run_phase != RunPhase::On) {
    return;
  }

#if AUDIO_SOURCE == ADC
  adc_capture_process();
#elif AUDIO_SOURCE == PCM1808
  pcm1808_capture_process();
#elif AUDIO_SOURCE == SYNTHETIC
  if (audio_source_feeds_ring()) {
    const uint32_t target =
        APP_RING_PREFILL_SAMPLES + APP_PKT_SAMPLES_NOM * 10u;
    uint32_t occ = audio_ring_occupancy();
    if (occ < target) {
      synthetic_generate(target - occ);
    }
  }
#endif

  /* STANDALONE never feeds the ring, so it must not wait for the prefill. */
  if (g_state == STREAM_STATE_STREAM_PREFILL &&
      (device_mode_get() == DEVICE_MODE_STANDALONE ||
       audio_ring_is_prefilled())) {
    g_state = STREAM_STATE_STREAM_ACTIVE;
  }
}

stream_state_t audio_source_state(void) { return g_state; }

void audio_source_set_run_requested(bool requested) {
  g_run_requested = requested;
}

bool audio_source_run_requested(void) { return g_run_requested; }

bool audio_source_capture_running(void) {
  return backend_running() && (g_run_phase == RunPhase::On || g_stream_armed);
}

bool audio_source_backend_ok(void) { return backend_healthy(); }

bool audio_source_feeds_ring(void) {
  return g_stream_armed && device_mode_get() != DEVICE_MODE_STANDALONE;
}

bool audio_source_capture_pause(void) {
  g_paused_was_running = false;
#if AUDIO_SOURCE == ADC
  if (adc_capture_is_running()) {
    g_paused_was_running = true;
    adc_capture_stop();
    audio_ring_reset();
  }
#elif AUDIO_SOURCE == PCM1808
  if (pcm1808_capture_is_running()) {
    g_paused_was_running = true;
    pcm1808_capture_stop();
    audio_ring_reset();
  }
#endif
  return g_paused_was_running;
}

void audio_source_capture_resume(void) {
  if (!g_paused_was_running) {
    return;
  }
  g_paused_was_running = false;
#if AUDIO_SOURCE == ADC
  adc_capture_start();
#elif AUDIO_SOURCE == PCM1808
  pcm1808_capture_start();
#endif
}
