/**
 * @file timebase.h
 * @brief Sample-clock calibration: stored ppm correction and optional GPS PPS.
 *
 * Every clock on the board (µs timer, PLL_SYS → PCM1808 MCLK, PLL_USB → ADC)
 * comes from the one 12 MHz crystal, so a single ppm figure describes the
 * error of every capture path. This module owns that figure:
 *
 *  - a stored calibration (manual or measured by a host against NTP, see
 *    scripts/calibrate_timebase.py), persisted in the EEPROM-emulation flash
 *    sector and loaded at boot;
 *  - when built with {@code -DPPS_GPIO=N}, a live measurement against a GPS
 *    1 PPS signal (PIO cycle counter, no IRQ) that overrides the stored value
 *    while locked;
 *  - with {@code -DGPS_UART_RX=N} as well, the module's NMEA output: PPS is
 *    only trusted while it reports a valid fix, and its UTC date stamps
 *    calibrations set without one.
 *
 * Sign convention: positive ppm = board clock fast (counts more than 1 s of
 * samples per true second). The beat_rate block divides it out.
 *
 * @see docs/TIMEBASE_CALIBRATION.md
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Largest accepted correction magnitude (ppm). */
#define TIMEBASE_PPM_MAX 1000.0f

typedef enum {
  TIMEBASE_SRC_NONE = 0, /**< No calibration (0 ppm). */
  TIMEBASE_SRC_MANUAL,   /**< Set by hand (CLI / HID / dsp param). */
  TIMEBASE_SRC_NTP,      /**< Measured by a host against NTP time. */
  TIMEBASE_SRC_PPS,      /**< Measured on-board against GPS 1 PPS. */
  TIMEBASE_SRC_COUNT
} timebase_source_t;

typedef enum {
  TIMEBASE_SAVE_OK = 0,
  TIMEBASE_SAVE_BUSY,   /**< Refused: audio is streaming (flash stalls XIP). */
  TIMEBASE_SAVE_FAILED, /**< Flash write / verify failed. */
} timebase_save_result_t;

typedef struct {
  float ppm;                /**< Correction in effect (PPS if locked, else stored). */
  timebase_source_t source; /**< Source of {@code ppm}. */
  /* Stored (non-PPS) calibration. */
  float cal_ppm;
  timebase_source_t cal_source;
  uint32_t cal_date_unix;  /**< When it was measured (host-supplied), 0 = unknown. */
  uint32_t cal_duration_s; /**< Measurement length, 0 = unknown / manual. */
  bool cal_saved;          /**< True if flash holds exactly this calibration. */
  /* GPS PPS (all zero / false unless built with PPS_GPIO). */
  bool pps_enabled;
  bool pps_locked;
  float pps_ppm;
  uint32_t pps_pulses;  /**< Pulses in the current average (≤ 64). */
  uint32_t pps_good;    /**< Accepted pulses since boot. */
  uint32_t pps_rejects; /**< Rejected pulses since boot (glitch / gap). */
  uint32_t pps_no_fix;  /**< Pulses ignored because GPS reported no fix (GPS_UART_RX). */
  /* GPS NMEA (all zero / false unless built with GPS_UART_RX). */
  bool gps_enabled;
  bool gps_fix;          /**< Valid fix now (RMC 'A'; GGA quality > 0, ≥ GPS_MIN_SATS). */
  uint8_t gps_quality;   /**< GGA fix quality (0 = none / no recent GGA). */
  uint8_t gps_sats;      /**< Satellites in use (GGA). */
  float gps_hdop;        /**< GGA HDOP, 0 = unknown. */
  uint32_t gps_utc_unix; /**< Current UTC from GPS, 0 = never received. */
  uint32_t gps_age_ms;   /**< Since the last RMC; 0xFFFFFFFF = never. */
  uint32_t gps_sentences;
  uint32_t gps_errors;   /**< Bad checksum / overlong / UART framing errors. */
  uint32_t gps_overruns; /**< UART FIFO overruns (characters lost). */
} timebase_info_t;

/** Loads the stored calibration and starts PPS capture if built in. */
void timebase_init(void);

/** Drains PPS measurements and GPS NMEA, updates lock state. Call from loop(). */
void timebase_poll(void);

/** Current UTC (UNIX seconds) from GPS NMEA, or 0 if unknown / not built in. */
uint32_t timebase_utc_now(void);

/**
 * Sets the stored calibration in RAM (call {@link timebase_save} to persist).
 *
 * @return {@code false} if {@code ppm} is not finite or outside ±1000, or
 *         {@code source} is PPS / out of range.
 *
 * A {@code date_unix} of 0 is replaced by the GPS UTC date when known
 * ({@link timebase_utc_now}).
 */
bool timebase_set(float ppm, timebase_source_t source, uint32_t date_unix,
                  uint32_t duration_s);

/** Resets the stored calibration to 0 ppm / none (RAM only). */
void timebase_clear(void);

/** Writes the stored calibration to flash. Refused while streaming. */
timebase_save_result_t timebase_save(void);

/** @return Correction in effect (ppm, + = board clock fast). */
float timebase_ppm(void);

/** @return Source of {@link timebase_ppm}. */
timebase_source_t timebase_source(void);

void timebase_get(timebase_info_t *out);

/** @return Lowercase name ("none", "manual", "ntp", "pps"). */
const char *timebase_source_name(timebase_source_t s);

/** Parses a source name; PPS is not accepted (it is measured, not set). */
bool timebase_source_from_name(const char *name, timebase_source_t *out);

#ifdef __cplusplus
}
#endif
