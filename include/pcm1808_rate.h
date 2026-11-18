/**
 * @file pcm1808_rate.h
 * @brief Classifies a measured PCM1808 frame rate against its MCLK ratios.
 *
 * In master mode the PCM1808 divides SCKI (MCLK) by 256, 384 or 512 depending
 * on the MD1/MD0 straps (TI SLES177: 11 = 256 fs, 10 = 384 fs, 01 = 512 fs,
 * 00 = slave). With 12.288 MHz MCLK that is 48, 32 or 24 kHz. Every timing
 * result scales with this rate, so the firmware measures it instead of
 * trusting the straps, then snaps to the exact nominal value: MCLK and the
 * timer share the crystal, so the measurement identifies the ratio, and any
 * crystal error is left to the beat_rate {@code cal_ppm} parameter.
 *
 * Header-only and SDK-free so it is host-testable.
 *
 * @see pcm1808_capture.cpp, tests/test_pcm1808_rate.cpp
 */

#pragma once

#include <stdint.h>

/** Largest sample-repeat factor into the 48 kHz USB stream (24 kHz → 2). */
#define PCM1808_MAX_UPSAMPLE 2u
/** Measured rate must be within this fraction of a nominal rate to match. */
#define PCM1808_RATE_TOLERANCE 0.005

typedef enum {
  PCM1808_RATE_PENDING = 0, /**< Not measured yet. */
  PCM1808_RATE_OK,          /**< Matches a ratio the USB path can carry. */
  PCM1808_RATE_UNSUPPORTED, /**< Matches a ratio, but not an integer upsample. */
  PCM1808_RATE_UNKNOWN,     /**< Matches no ratio (straps, clock, wiring). */
} pcm1808_rate_status_t;

typedef struct {
  pcm1808_rate_status_t status;
  uint32_t mclk_ratio; /**< 256 / 384 / 512, or 0 when unknown. */
  uint32_t nominal_hz; /**< mclk_hz / mclk_ratio, or 0 when unknown. */
  uint32_t upsample;   /**< usb_hz / nominal_hz when OK, else 0. */
} pcm1808_rate_match_t;

/**
 * @param measured_hz Frames per second measured over a DMA window.
 * @param mclk_hz Nominal SCKI frequency.
 * @param usb_hz USB stream rate the frames are repeated into.
 */
static inline pcm1808_rate_match_t
pcm1808_classify_rate(double measured_hz, uint32_t mclk_hz, uint32_t usb_hz) {
  static const uint32_t kRatios[] = {256u, 384u, 512u};
  pcm1808_rate_match_t m = {PCM1808_RATE_UNKNOWN, 0u, 0u, 0u};
  for (uint32_t ratio : kRatios) {
    const uint32_t nominal = mclk_hz / ratio;
    const double err = (measured_hz - nominal) / nominal;
    if (err > -PCM1808_RATE_TOLERANCE && err < PCM1808_RATE_TOLERANCE) {
      m.mclk_ratio = ratio;
      m.nominal_hz = nominal;
      if (nominal != 0u && usb_hz % nominal == 0u &&
          usb_hz / nominal <= PCM1808_MAX_UPSAMPLE) {
        m.status = PCM1808_RATE_OK;
        m.upsample = usb_hz / nominal;
      } else {
        m.status = PCM1808_RATE_UNSUPPORTED;
      }
      return m;
    }
  }
  return m;
}

/** @return Short lowercase name for CLI / JSON. */
static inline const char *pcm1808_rate_status_name(pcm1808_rate_status_t s) {
  switch (s) {
  case PCM1808_RATE_PENDING:
    return "pending";
  case PCM1808_RATE_OK:
    return "ok";
  case PCM1808_RATE_UNSUPPORTED:
    return "unsupported";
  case PCM1808_RATE_UNKNOWN:
    return "unknown";
  }
  return "unknown";
}
