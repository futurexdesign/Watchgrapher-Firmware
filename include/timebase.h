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
 * Calibration history: every successful {@link timebase_save} and every
 * noteworthy GPS PPS measurement (policy in timebase_log.h) is appended to a
 * 24-entry ring ({@link timebase_log_snapshot}) persisted in the TBLOG region
 * of the flash sector. PPS entries are staged in RAM at once and written to
 * flash later, when no USB stream is open and no quartz measurement is
 * running ({@link timebase_log_pending}).
 *
 * @see docs/TIMEBASE_CALIBRATION.md
 */

#pragma once

#include "timebase_log.h"

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

/** Outcome of {@link timebase_set_log_delta_ppm}. */
typedef enum {
  TIMEBASE_LOGDELTA_OK = 0,
  TIMEBASE_LOGDELTA_BAD_PARAM, /**< Not finite or outside 0.05..10 ppm. */
  TIMEBASE_LOGDELTA_BUSY,      /**< Audio is streaming; flash not written. */
  TIMEBASE_LOGDELTA_FAILED,    /**< Flash write / verify failed. */
} timebase_log_delta_result_t;

/** Satellites in {@link timebase_gps_t} (equals NMEA_MAX_SATS). */
#define TIMEBASE_GPS_MAX_SATS 32u

/** One satellite of {@link timebase_gps_t}. */
typedef struct {
  uint8_t sys;  /**< nmea_sys_t (1 GPS, 2 GLONASS, 3 Galileo, 4 BeiDou, 5 QZSS, 0 ?). */
  uint16_t prn; /**< Normalised PRN. */
  int8_t el;    /**< Elevation degrees, -1 unknown. */
  int16_t az;   /**< Azimuth degrees, -1 unknown. */
  int8_t snr;   /**< C/N0 dB-Hz, -1 = not tracked. */
  bool used;    /**< In the current position solution (GSA). */
} timebase_gps_sat_t;

/** Detailed GPS state for the control plane / CLI (see {@link timebase_gps_get}). */
typedef struct {
  bool fix;            /**< Same definition as timebase_info_t::gps_fix. */
  uint8_t quality;     /**< GGA fix quality, 0 = none / stale. */
  uint32_t utc_unix;   /**< UTC seconds, 0 = never received. */
  int32_t age_ms;      /**< Since the last RMC, -1 = never. */
  uint8_t sats_used;   /**< Satellites in use (GGA), 0 if stale. */
  float hdop;          /**< GGA HDOP (GSA HDOP if GGA is stale), 0 = unknown. */
  float pdop;          /**< GSA PDOP, 0 = unknown. */
  float vdop;          /**< GSA VDOP, 0 = unknown. */
  double lat;          /**< Degrees, + = N; valid if pos_valid. */
  double lon;          /**< Degrees, + = E; valid if pos_valid. */
  float alt_m;         /**< Altitude above MSL (m); valid if pos_valid. */
  bool pos_valid;      /**< lat / lon / alt are current and valid. */
  uint8_t n_sats;      /**< Valid entries of {@code sats}. */
  timebase_gps_sat_t sats[TIMEBASE_GPS_MAX_SATS];
} timebase_gps_t;

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
  /* Calibration log. */
  float log_delta_ppm; /**< PPS log threshold (ppm), persisted. */
  bool log_pending;    /**< A log entry is staged but not yet in flash. */
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

/**
 * Writes the stored calibration to flash and, on success, appends a
 * "save" entry (source / ppm / date / duration of the stored calibration;
 * the date falls back to GPS UTC, else 0) to the log. Any staged PPS log
 * entry is written in the same commit. Refused while streaming.
 *
 * Main-loop (core 0) context only; may pause capture for the flash write.
 */
timebase_save_result_t timebase_save(void);

/** @return Correction in effect (ppm, + = board clock fast). */
float timebase_ppm(void);

/** @return Source of {@link timebase_ppm}. */
timebase_source_t timebase_source(void);

void timebase_get(timebase_info_t *out);

/**
 * Copies the calibration log (ring, oldest-first via timebase_log_get() of
 * timebase_log.h).
 *
 * Named differently from the timebase_log.h accessor
 * {@code timebase_log_get(const timebase_log_t *, uint32_t)}, which has the
 * same C name.
 *
 * @param out Receives the log; null is ignored.
 */
void timebase_log_snapshot(timebase_log_t *out);

/** @return True if a log entry is staged in RAM but not yet committed to flash. */
bool timebase_log_pending(void);

/** @return GPS UTC (UNIX s) of the last successful log write, 0 = none / unknown. */
uint32_t timebase_log_last_write_unix(void);

/**
 * Clears the log in RAM and in flash (pause-and-write). On failure or BUSY
 * the previous log is kept.
 *
 * @return OK, BUSY (USB audio streaming), FAILED.
 */
timebase_save_result_t timebase_log_clear_saved(void);

/** @return The PPS log delta threshold (ppm, default 0.2). */
float timebase_log_delta_ppm(void);

/**
 * Sets and persists the PPS log delta threshold (device settings record;
 * pause-and-write).
 *
 * @param ppm 0.05..10, finite.
 * @return OK, BAD_PARAM, BUSY (USB audio streaming), FAILED.
 */
timebase_log_delta_result_t timebase_set_log_delta_ppm(float ppm);

/** @return True if built with GPS_UART_RX (NMEA available). */
bool timebase_gps_built(void);

/** @return True if built with PPS_GPIO. */
bool timebase_pps_built(void);

/**
 * Snapshot of the detailed GPS state (position, DOP, satellite table).
 *
 * NMEA is parsed by {@link timebase_poll} in the main loop; callers (HID
 * task, CLI) run in that same loop context, so the copy is consistent
 * without locking. Do not call from core 1 or an interrupt. Satellites not
 * seen for 5 s are pruned first.
 *
 * @param out Receives the snapshot (zeroed first).
 * @return false (out zeroed) when not built with GPS_UART_RX.
 */
bool timebase_gps_get(timebase_gps_t *out);

/** @return Lowercase name ("none", "manual", "ntp", "pps"). */
const char *timebase_source_name(timebase_source_t s);

/** Parses a source name; PPS is not accepted (it is measured, not set). */
bool timebase_source_from_name(const char *name, timebase_source_t *out);

#ifdef __cplusplus
}
#endif
