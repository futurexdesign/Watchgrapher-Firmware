/**
 * @file timebase.cpp
 * @brief Sample-clock calibration store and optional GPS PPS measurement.
 *
 * With {@code -DGPS_UART_RX=N} the GPS module's NMEA output is read too
 * (RX only, polled from the hardware UART FIFO). PPS pulses are then only
 * accepted while the receiver reports a valid fix, so a module that keeps
 * pulsing from its own oscillator without satellites cannot pose as a GPS
 * lock. The UTC date also stamps calibrations set without a date.
 *
 * @see timebase.h, docs/TIMEBASE_CALIBRATION.md
 */

#include "timebase.h"

#include "app_config.h"
#include "audio_source.h"
#include "board_config.h"
#include "device_mode.h"
#include "flash_store.h"
#if AUDIO_SOURCE == PCM1808
#include "quartz/quartz_engine.h"
#endif
#include "timebase_log.h"
#include "usb_audio.h"

#include <hardware/clocks.h>

#include <Arduino.h>
#include <cmath>
#include <cstring>

#ifdef PPS_GPIO
#include "pps_counter.pio.h"
#include "pps_filter.h"

#include <hardware/pio.h>
static_assert(PPS_GPIO >= 0 && PPS_GPIO < 32,
              "PPS_GPIO must be GPIO 0..31 (PIO GPIO base is left at 0)");
#endif

#ifdef GPS_UART_RX
#include "nmea.h"

#include <hardware/gpio.h>
#include <hardware/uart.h>
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD 9600
#endif
#ifndef GPS_MIN_SATS
#define GPS_MIN_SATS 4
#endif
/* UART RX pins: odd GPIOs; n % 4 == 1 is the UART function, n % 4 == 3 the
 * RP2350 UART_AUX function. The block is ((n + 4) / 8) % 2. */
static_assert(GPS_UART_RX >= 1 && GPS_UART_RX <= 47 && GPS_UART_RX % 2 == 1,
              "GPS_UART_RX must be a UART RX-capable (odd) GPIO");
#if defined(PICO_RP2350A) && PICO_RP2350A
static_assert(GPS_UART_RX <= 29, "RP2350A has GPIO 0..29");
#endif
#define GPS_UART_INDEX (((GPS_UART_RX + 4) / 8) % 2)
static_assert(!(DIAG_UART && GPS_UART_INDEX == 0),
              "GPS_UART_RX is on UART0, which DIAG_UART uses (Serial1); pick a UART1 pin "
              "such as GP9 or disable DIAG_UART");
#ifdef PPS_GPIO
static_assert(GPS_UART_RX != PPS_GPIO, "GPS_UART_RX and PPS_GPIO must differ");
#endif
#endif

namespace {

/*
 * Flash record in the EEPROM-emulation sector. Versioned and CRC-checked so
 * an erased (0xFF) or foreign sector reads as "no calibration".
 */
constexpr uint32_t kMagic = 0x31434254u; /* "TBC1" little-endian */
constexpr uint16_t kVersion = 1u;

struct Record {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float ppm;
  uint32_t source;
  uint32_t date_unix;
  uint32_t duration_s;
  uint32_t crc;
};
static_assert(sizeof(Record) <= FLASH_STORE_TIMEBASE_SIZE,
              "record exceeds TIMEBASE region");

struct Cal {
  float ppm;
  timebase_source_t source;
  uint32_t date_unix;
  uint32_t duration_s;
};

Cal g_cal = {0.f, TIMEBASE_SRC_NONE, 0u, 0u};
Cal g_saved = {0.f, TIMEBASE_SRC_NONE, 0u, 0u};
bool g_have_saved = false;

/** CRC-32 (IEEE, reflected) over {@code n} bytes. */
uint32_t crc32(const void *data, size_t n) {
  const auto *p = static_cast<const uint8_t *>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

uint32_t record_crc(const Record &r) {
  return crc32(&r, offsetof(Record, crc));
}

bool cal_equal(const Cal &a, const Cal &b) {
  return a.ppm == b.ppm && a.source == b.source &&
         a.date_unix == b.date_unix && a.duration_s == b.duration_s;
}

bool load_record(Cal *out) {
  Record r{};
  if (!flash_store_read(FLASH_STORE_TIMEBASE_OFFSET, &r, sizeof(r))) {
    return false;
  }
  if (r.magic != kMagic || r.version != kVersion ||
      r.size != sizeof(Record) || r.crc != record_crc(r)) {
    return false;
  }
  if (!std::isfinite(r.ppm) || fabsf(r.ppm) > TIMEBASE_PPM_MAX ||
      r.source >= TIMEBASE_SRC_COUNT || r.source == TIMEBASE_SRC_PPS) {
    return false;
  }
  *out = {r.ppm, static_cast<timebase_source_t>(r.source), r.date_unix,
          r.duration_s};
  return true;
}

/*
 * Calibration log. {@code g_log} is the RAM truth. An entry is "pending"
 * from the moment it is appended until a commit has carried it to flash; the
 * bytes are re-staged right before every commit attempt, so a failed stage
 * or an intervening unrelated commit cannot leave stale data behind.
 */
timebase_log_t g_log;
bool g_log_pending = false;
uint32_t g_log_last_write_unix = 0;
/** Deferred-write rate limit: millis() of the last attempt. */
uint32_t g_log_attempt_ms = 0;
uint32_t g_log_retry_ms = 0; /**< Minimum spacing for the next attempt. */
bool g_log_attempted = false;
constexpr uint32_t kLogAttemptMs =
    10000u;                              /**< After a deferral-free attempt. */
constexpr uint32_t kLogRetryMs = 60000u; /**< After a failed / busy attempt. */

/** Stages {@code log} in the TBLOG region (no commit). */
bool stage_log(const timebase_log_t &log) {
  timebase_log_record_t rec;
  timebase_log_encode(&log, &rec);
  return flash_store_write(FLASH_STORE_TBLOG_OFFSET, &rec, sizeof(rec));
}

/** Loads the log from flash; invalid or erased reads as empty. */
void load_log() {
  timebase_log_record_t rec{};
  timebase_log_init(&g_log);
  if (flash_store_read(FLASH_STORE_TBLOG_OFFSET, &rec, sizeof(rec))) {
    timebase_log_t l;
    if (timebase_log_decode(&rec, &l)) {
      g_log = l;
    }
  }
}

/** Marks the RAM log as written: clears pending, stamps the write time. */
void log_written() {
  g_log_pending = false;
  g_log_last_write_unix = timebase_utc_now();
}

/**
 * True while a quartz measurement is acquiring / measuring (a relock costs it).
 *
 * The quartz engine exists only on the PCM1808 source; elsewhere this is
 * always false. The guard matters for size, not only correctness: an
 * unconditional call would link the engine's ~41 KB static store into the
 * ADC / synthetic builds.
 */
bool quartz_measuring() {
#if AUDIO_SOURCE == PCM1808
  static dsp_quartz_t q; /* static: keeps the large snapshot off the stack */
  quartz::engine::get(&q);
  return q.mode != DSP_QUARTZ_MODE_MECHANICAL &&
         (q.state == DSP_QUARTZ_STATE_ACQUIRING ||
          q.state == DSP_QUARTZ_STATE_MEASURING);
#else
  return false;
#endif
}

/**
 * Writes a pending log entry when it is safe: no USB stream, no quartz
 * measurement in progress, at most one attempt per 10 s (60 s after a
 * failure or BUSY).
 */
void log_flush_poll() {
  if (!g_log_pending || usb_audio_streaming()) {
    return;
  }
  const uint32_t now = millis();
  if (g_log_attempted && now - g_log_attempt_ms < g_log_retry_ms) {
    return;
  }
  if (quartz_measuring()) {
    return;
  }
  g_log_attempted = true;
  g_log_attempt_ms = now;
  g_log_retry_ms = kLogRetryMs;
  if (!stage_log(g_log)) {
    return;
  }
  switch (flash_store_commit_paused()) {
  case FLASH_STORE_COMMIT_OK:
    log_written();
    g_log_retry_ms = kLogAttemptMs;
    break;
  default:
    break; /* keep pending, retry later */
  }
}

#ifdef GPS_UART_RX
/** No valid RMC for this long counts as "no fix". */
constexpr uint32_t kGpsStaleMs = 3000u;
/** GGA older than this is ignored for the satellite check. */
constexpr uint32_t kGgaStaleMs = 3000u;

nmea_t g_nmea;
uart_inst_t *const g_gps_uart = GPS_UART_INDEX == 0 ? uart0 : uart1;
uint32_t g_rmc_ms = 0;  /**< millis() of the last RMC, 0 = never. */
uint32_t g_gga_ms = 0;  /**< millis() of the last GGA, 0 = never. */
bool g_time_known = false;
uint32_t g_time_unix = 0; /**< UTC seconds at {@code g_time_ms}. */
uint32_t g_time_ms = 0;
uint32_t g_gps_overruns = 0;
uint32_t g_gps_line_errors = 0; /**< UART framing / parity / break. */

void gps_init() {
  nmea_init(&g_nmea);
  uart_init(g_gps_uart, GPS_UART_BAUD);
  gpio_set_function(GPS_UART_RX,
                    GPS_UART_RX % 4 == 1 ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX);
}

void gps_poll() {
  const uint32_t now = millis();
  /* The 32-byte hardware FIFO holds ~33 ms at 9600 baud; loop() is far faster. */
  while (uart_is_readable(g_gps_uart)) {
    const uint32_t dr = uart_get_hw(g_gps_uart)->dr;
    if (dr & UART_UARTDR_OE_BITS) {
      g_gps_overruns++;
    }
    if (dr & (UART_UARTDR_FE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_BE_BITS)) {
      g_gps_line_errors++;
      continue;
    }
    const unsigned got =
        nmea_feed_at(&g_nmea, static_cast<char>(dr & 0xFFu), now);
    if (got & NMEA_GOT_RMC) {
      g_rmc_ms = now ? now : 1u;
      if (g_nmea.time_valid) {
        g_time_known = true;
        g_time_unix = g_nmea.unix_s;
        g_time_ms = now;
      }
    }
    if (got & NMEA_GOT_GGA) {
      g_gga_ms = now ? now : 1u;
    }
  }
}

bool gps_fresh(uint32_t at_ms, uint32_t max_age) {
  return at_ms != 0u && millis() - at_ms <= max_age;
}

/** Receiver reports a usable fix (RMC valid, and GGA agrees when present). */
bool gps_fix_ok() {
  if (!gps_fresh(g_rmc_ms, kGpsStaleMs) || !g_nmea.rmc_valid) {
    return false;
  }
  if (gps_fresh(g_gga_ms, kGgaStaleMs)) {
    return g_nmea.gga_quality > 0u && g_nmea.sats >= GPS_MIN_SATS;
  }
  return true; /* module sends no GGA: trust RMC */
}

/** Current UTC seconds from the last RMC, or 0 if never known. */
uint32_t gps_unix_now() {
  return g_time_known ? g_time_unix + (millis() - g_time_ms) / 1000u : 0u;
}
#endif

#ifdef PPS_GPIO
/** No accepted pulse for this long drops PPS lock. */
constexpr uint32_t kPpsLossMs = 10000u;

PIO g_pps_pio = nullptr;
uint g_pps_sm = 0;
bool g_pps_ok = false;
bool g_pps_locked = false;
bool g_pps_lock_active = false;   /**< Lock streak running. */
uint32_t g_pps_lock_start_ms = 0; /**< millis() when the streak began. */
uint32_t g_pps_eval_ms = 0;       /**< Last log-policy evaluation. */
uint32_t g_pps_last_good_ms = 0;
uint32_t g_pps_no_fix = 0; /**< Pulses discarded because GPS had no fix. */
pps_filter_t g_pps;

void pps_init() {
  uint offset = 0;
  if (!pio_claim_free_sm_and_add_program(&pps_counter_program, &g_pps_pio,
                                         &g_pps_sm, &offset)) {
    return; /* no free PIO resources: PPS stays disabled */
  }
  pps_counter_program_init(g_pps_pio, g_pps_sm, offset,
                           static_cast<uint>(PPS_GPIO));
  pio_sm_set_enabled(g_pps_pio, g_pps_sm, true);
  pps_filter_init(&g_pps, clock_get_hz(clk_sys));
  g_pps_ok = true;
}

void pps_poll() {
  if (!g_pps_ok) {
    return;
  }
#ifdef GPS_UART_RX
  const bool fix = gps_fix_ok();
#else
  const bool fix = true;
#endif
  while (!pio_sm_is_rx_fifo_empty(g_pps_pio, g_pps_sm)) {
    const uint32_t x = pio_sm_get(g_pps_pio, g_pps_sm);
    const uint32_t cycles =
        2u * (0xFFFFFFFFu - x) + static_cast<uint32_t>(pps_counter_OVERHEAD_CYCLES);
    if (!fix) {
      /* Without a fix the module's PPS free-runs on its own oscillator. */
      g_pps_no_fix++;
      continue;
    }
    if (pps_filter_push(&g_pps, cycles)) {
      g_pps_last_good_ms = millis();
    }
  }
  if (g_pps.n > 0u && (!fix || millis() - g_pps_last_good_ms > kPpsLossMs)) {
    pps_filter_clear(&g_pps);
  }
  g_pps_locked = pps_filter_ready(&g_pps);
}

/**
 * Tracks the continuous-lock streak and runs the log policy (once per second,
 * and not while an entry is pending so each trigger yields one entry).
 */
void pps_log_poll() {
  const uint32_t now = millis();
  if (!g_pps_locked) {
    g_pps_lock_active = false;
    return;
  }
  if (!g_pps_lock_active) {
    g_pps_lock_active = true;
    g_pps_lock_start_ms = now;
  }
  if (g_log_pending || now - g_pps_eval_ms < 1000u) {
    return;
  }
  g_pps_eval_ms = now;
  const uint32_t locked_s = (now - g_pps_lock_start_ms) / 1000u;
  const float ppm = static_cast<float>(pps_filter_ppm(&g_pps));
  const uint32_t utc = timebase_utc_now();
  timebase_log_pps_input_t in{};
  in.pps_locked = g_pps_locked;
  in.pps_window = g_pps.n;
  in.locked_since_s = locked_s;
  in.pps_ppm = ppm;
  in.utc_now = utc;
  const timebase_log_decision_t d =
      timebase_log_pps_decision(&g_log, &in, device_log_delta_ppm_get());
  if (d == TIMEBASE_LOG_NONE) {
    return;
  }
  timebase_log_entry_t e{};
  e.date_unix = utc;
  e.ppm = ppm;
  e.source = static_cast<uint8_t>(TIMEBASE_SRC_PPS);
  e.reason = static_cast<uint8_t>(d == TIMEBASE_LOG_MONTHLY
                                      ? TIMEBASE_LOG_REASON_MONTHLY
                                      : TIMEBASE_LOG_REASON_DELTA);
  e.duration_s = locked_s;
  timebase_log_append(&g_log, &e);
  (void)stage_log(g_log); /* re-staged before every commit attempt anyway */
  g_log_pending = true;
  g_log_attempted = false; /* first attempt is not rate-limited */
}
#endif

} // namespace

void timebase_init(void) {
  Cal c{};
  if (load_record(&c)) {
    g_cal = c;
    g_saved = c;
    g_have_saved = true;
  }
  load_log();
#ifdef GPS_UART_RX
  gps_init();
#endif
#ifdef PPS_GPIO
  pps_init();
#endif
}

void timebase_poll(void) {
#ifdef GPS_UART_RX
  gps_poll();
#endif
#ifdef PPS_GPIO
  pps_poll();
  pps_log_poll();
#endif
  log_flush_poll();
}

uint32_t timebase_utc_now(void) {
#ifdef GPS_UART_RX
  return gps_unix_now();
#else
  return 0u;
#endif
}

bool timebase_set(float ppm, timebase_source_t source, uint32_t date_unix,
                  uint32_t duration_s) {
  if (!std::isfinite(ppm) || fabsf(ppm) > TIMEBASE_PPM_MAX ||
      source >= TIMEBASE_SRC_COUNT || source == TIMEBASE_SRC_PPS) {
    return false;
  }
  if (ppm == 0.f && source == TIMEBASE_SRC_MANUAL) {
    source = TIMEBASE_SRC_NONE;
  }
  if (date_unix == 0u) {
    date_unix = timebase_utc_now(); /* GPS date when known, else stays 0 */
  }
  g_cal = {ppm, source, date_unix, duration_s};
  return true;
}

void timebase_clear(void) { g_cal = {0.f, TIMEBASE_SRC_NONE, 0u, 0u}; }

timebase_save_result_t timebase_save(void) {
  /* Flash erase stalls XIP for tens of ms: never while a USB stream is open.
   * flash_store_commit_paused() pauses a running capture around the write. */
  if (usb_audio_streaming()) {
    return TIMEBASE_SAVE_BUSY;
  }
  Record r{};
  r.magic = kMagic;
  r.version = kVersion;
  r.size = sizeof(Record);
  r.ppm = g_cal.ppm;
  r.source = static_cast<uint32_t>(g_cal.source);
  r.date_unix = g_cal.date_unix;
  r.duration_s = g_cal.duration_s;
  r.crc = record_crc(r);
  /* The "save" entry goes into the same commit as the calibration, so the log
   * (and any staged PPS entry) is written together with it. */
  timebase_log_t next = g_log;
  timebase_log_entry_t e{};
  e.date_unix = g_cal.date_unix ? g_cal.date_unix : timebase_utc_now();
  e.ppm = g_cal.ppm;
  e.source = static_cast<uint8_t>(g_cal.source);
  e.reason = static_cast<uint8_t>(TIMEBASE_LOG_REASON_SAVE);
  e.duration_s = g_cal.duration_s;
  timebase_log_append(&next, &e);
  if (!flash_store_write(FLASH_STORE_TIMEBASE_OFFSET, &r, sizeof(r)) ||
      !stage_log(next)) {
    (void)stage_log(g_log);
    return TIMEBASE_SAVE_FAILED;
  }
  const flash_store_commit_result_t cr = flash_store_commit_paused();
  if (cr != FLASH_STORE_COMMIT_OK) {
    (void)stage_log(g_log); /* do not leave the rejected entry staged */
    return cr == FLASH_STORE_COMMIT_BUSY ? TIMEBASE_SAVE_BUSY
                                         : TIMEBASE_SAVE_FAILED;
  }
  Cal check{};
  if (!load_record(&check) || !cal_equal(check, g_cal)) {
    (void)stage_log(g_log);
    return TIMEBASE_SAVE_FAILED;
  }
  g_log = next;
  log_written();
  g_saved = g_cal;
  g_have_saved = true;
  return TIMEBASE_SAVE_OK;
}

float timebase_ppm(void) {
#ifdef PPS_GPIO
  if (g_pps_locked) {
    return static_cast<float>(pps_filter_ppm(&g_pps));
  }
#endif
  return g_cal.ppm;
}

timebase_source_t timebase_source(void) {
#ifdef PPS_GPIO
  if (g_pps_locked) {
    return TIMEBASE_SRC_PPS;
  }
#endif
  return g_cal.source;
}

void timebase_get(timebase_info_t *out) {
  if (!out) {
    return;
  }
  *out = {};
  out->ppm = timebase_ppm();
  out->source = timebase_source();
  out->cal_ppm = g_cal.ppm;
  out->cal_source = g_cal.source;
  out->cal_date_unix = g_cal.date_unix;
  out->cal_duration_s = g_cal.duration_s;
  out->cal_saved = g_have_saved && cal_equal(g_saved, g_cal);
#ifdef PPS_GPIO
  out->pps_enabled = g_pps_ok;
  out->pps_locked = g_pps_locked;
  out->pps_ppm = static_cast<float>(pps_filter_ppm(&g_pps));
  out->pps_pulses = g_pps.n;
  out->pps_good = g_pps.good;
  out->pps_rejects = g_pps.rejects;
  out->pps_no_fix = g_pps_no_fix;
#endif
#ifdef GPS_UART_RX
  out->gps_enabled = true;
  out->gps_fix = gps_fix_ok();
  out->gps_quality = gps_fresh(g_gga_ms, kGgaStaleMs) ? g_nmea.gga_quality : 0u;
  out->gps_sats = gps_fresh(g_gga_ms, kGgaStaleMs) ? g_nmea.sats : 0u;
  out->gps_hdop = gps_fresh(g_gga_ms, kGgaStaleMs) ? g_nmea.hdop : 0.f;
  out->gps_utc_unix = gps_unix_now();
  out->gps_age_ms = g_rmc_ms ? millis() - g_rmc_ms : 0xFFFFFFFFu;
  out->gps_sentences = g_nmea.sentences;
  out->gps_errors = g_nmea.checksum_errors + g_gps_line_errors;
  out->gps_overruns = g_gps_overruns;
#endif
  out->log_delta_ppm = device_log_delta_ppm_get();
  out->log_pending = g_log_pending;
}

void timebase_log_snapshot(timebase_log_t *out) {
  if (out) {
    *out = g_log;
  }
}

bool timebase_log_pending(void) { return g_log_pending; }

uint32_t timebase_log_last_write_unix(void) { return g_log_last_write_unix; }

timebase_save_result_t timebase_log_clear_saved(void) {
  if (usb_audio_streaming()) {
    return TIMEBASE_SAVE_BUSY;
  }
  timebase_log_t empty;
  timebase_log_init(&empty);
  if (!stage_log(empty)) {
    return TIMEBASE_SAVE_FAILED;
  }
  const flash_store_commit_result_t cr = flash_store_commit_paused();
  if (cr != FLASH_STORE_COMMIT_OK) {
    (void)stage_log(
        g_log); /* keep the old log; do not leave the clear staged */
    return cr == FLASH_STORE_COMMIT_BUSY ? TIMEBASE_SAVE_BUSY
                                         : TIMEBASE_SAVE_FAILED;
  }
  g_log = empty;
  log_written();
  return TIMEBASE_SAVE_OK;
}

float timebase_log_delta_ppm(void) { return device_log_delta_ppm_get(); }

timebase_log_delta_result_t timebase_set_log_delta_ppm(float ppm) {
  switch (device_log_delta_ppm_set(ppm)) {
  case DEVICE_SET_OK:
    return TIMEBASE_LOGDELTA_OK;
  case DEVICE_SET_BAD_PARAM:
    return TIMEBASE_LOGDELTA_BAD_PARAM;
  case DEVICE_SET_BUSY:
    return TIMEBASE_LOGDELTA_BUSY;
  default:
    return TIMEBASE_LOGDELTA_FAILED;
  }
}

bool timebase_gps_built(void) {
#ifdef GPS_UART_RX
  return true;
#else
  return false;
#endif
}

bool timebase_pps_built(void) {
#ifdef PPS_GPIO
  return true;
#else
  return false;
#endif
}

bool timebase_gps_get(timebase_gps_t *out) {
  if (!out) {
    return false;
  }
  *out = {};
#ifdef GPS_UART_RX
  static_assert(TIMEBASE_GPS_MAX_SATS == NMEA_MAX_SATS,
                "timebase_gps_t satellite capacity must match the NMEA table");
  const uint32_t now = millis();
  (void)nmea_sats_prune(&g_nmea, now);
  const bool gga = gps_fresh(g_gga_ms, kGgaStaleMs);
  out->fix = gps_fix_ok();
  out->quality = gga ? g_nmea.gga_quality : 0u;
  out->utc_unix = gps_unix_now();
  out->age_ms = g_rmc_ms ? static_cast<int32_t>(now - g_rmc_ms) : -1;
  out->sats_used = gga ? g_nmea.sats : 0u;
  out->hdop = gga && g_nmea.hdop > 0.f ? g_nmea.hdop : g_nmea.gsa_hdop;
  out->pdop = g_nmea.pdop;
  out->vdop = g_nmea.vdop;
  out->pos_valid = gga && g_nmea.pos_valid;
  if (out->pos_valid) {
    out->lat = g_nmea.lat;
    out->lon = g_nmea.lon;
    out->alt_m = g_nmea.alt_m;
  }
  out->n_sats = g_nmea.sat_count;
  for (uint8_t i = 0; i < g_nmea.sat_count; i++) {
    const nmea_sat_t &s = g_nmea.sat_tab[i];
    out->sats[i] = {s.sys, s.prn, s.el, s.az, s.snr, s.used};
  }
  return true;
#else
  return false;
#endif
}

const char *timebase_source_name(timebase_source_t s) {
  switch (s) {
  case TIMEBASE_SRC_NONE:
    return "none";
  case TIMEBASE_SRC_MANUAL:
    return "manual";
  case TIMEBASE_SRC_NTP:
    return "ntp";
  case TIMEBASE_SRC_PPS:
    return "pps";
  case TIMEBASE_SRC_COUNT:
    break;
  }
  return "unknown";
}

bool timebase_source_from_name(const char *name, timebase_source_t *out) {
  if (!name || !out) {
    return false;
  }
  for (int i = 0; i < static_cast<int>(TIMEBASE_SRC_COUNT); i++) {
    const auto s = static_cast<timebase_source_t>(i);
    if (s != TIMEBASE_SRC_PPS && strcasecmp(name, timebase_source_name(s)) == 0) {
      *out = s;
      return true;
    }
  }
  return false;
}
