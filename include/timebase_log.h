/**
 * @file timebase_log.h
 * @brief Timebase calibration history: POD ring, flash record and the pure
 *        policy that decides when a GPS PPS measurement is worth logging.
 *
 * Header-only, SDK-free and C ABI compatible (POD structs, {@code static
 * inline} functions) so timebase.cpp, the HID layer and host tests share one
 * definition. Nothing here touches flash: the owner of the flash sector
 * (flash_store) reads/writes the bytes produced by
 * {@link timebase_log_encode} and validated by {@link timebase_log_decode}.
 *
 * Ring: {@link TIMEBASE_LOG_CAP} entries, oldest dropped. {@code head} is the
 * slot the next entry is written to, so the oldest entry lives at
 * {@code (head - count) mod CAP}.
 *
 * Flash record (little-endian, 404 bytes, fits the 512 B region at
 * offset 512 of the EEPROM sector):
 * <pre>
 *   0  u32 magic   "TBL1" (bytes 'T','B','L','1')
 *   4  u16 version TIMEBASE_LOG_VERSION
 *   6  u16 size    sizeof(timebase_log_record_t)
 *   8  u32 count   \
 *  12  u32 head     } timebase_log_t
 *  16  entries[24] /  16 bytes each
 * 400  u32 crc32  IEEE reflected over bytes [0, 400)
 * </pre>
 * Erased flash (all 0xFF) fails magic and is therefore "no log".
 *
 * Policy (see addendum A5): {@link timebase_log_pps_decision}.
 *
 * @see timebase.h
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#define TIMEBASE_LOG_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define TIMEBASE_LOG_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif

/** Ring capacity (entries). */
#define TIMEBASE_LOG_CAP               24u
/** PPS average window that counts as "full" (pulses). */
#define TIMEBASE_LOG_PPS_WINDOW_FULL   64u
/** Minimum continuous PPS lock before a measurement is trusted (s). */
#define TIMEBASE_LOG_PPS_MIN_LOCK_S    600u
/** Minimum spacing between "delta" entries (s). */
#define TIMEBASE_LOG_MIN_DELTA_AGE_S   86400u
/** Spacing that forces a "monthly" entry (s): 30 days. */
#define TIMEBASE_LOG_MONTHLY_S         (30u * 86400u)
/** Accepted range of the user-set delta threshold (ppm). */
#define TIMEBASE_LOG_DELTA_PPM_MIN     0.05f
#define TIMEBASE_LOG_DELTA_PPM_MAX     10.0f
/** Default delta threshold (ppm). */
#define TIMEBASE_LOG_DELTA_PPM_DEFAULT 0.2f
/** Flash record magic: "TBL1" as little-endian bytes 'T','B','L','1'. */
#define TIMEBASE_LOG_MAGIC             0x314C4254u
/** Flash record layout version. */
#define TIMEBASE_LOG_VERSION           1u

/** Why an entry was logged. Stored as u8 in {@link timebase_log_entry_t}. */
typedef enum {
  TIMEBASE_LOG_REASON_SAVE = 0, /**< Successful saveTimebase (manual / ntp). */
  TIMEBASE_LOG_REASON_DELTA,    /**< PPS moved by >= logDeltaPpm (>= 1 day). */
  TIMEBASE_LOG_REASON_MONTHLY,  /**< >= 30 days since the previous entry. */
} timebase_log_reason_t;

/** Outcome of {@link timebase_log_pps_decision}. */
typedef enum {
  TIMEBASE_LOG_NONE = 0,
  TIMEBASE_LOG_DELTA,
  TIMEBASE_LOG_MONTHLY,
} timebase_log_decision_t;

/** One history entry (16 bytes, no padding). */
typedef struct {
  uint32_t date_unix;  /**< UTC seconds the calibration was dated. */
  float ppm;           /**< Correction (positive = board clock fast). */
  uint8_t source;      /**< A {@code timebase_source_t} value (timebase.h). */
  uint8_t reason;      /**< A {@link timebase_log_reason_t} value. */
  uint16_t reserved;   /**< Zero; keeps the layout explicit. */
  uint32_t duration_s; /**< Measurement length, 0 = unknown / manual. */
} timebase_log_entry_t;

/** Ring of the last {@link TIMEBASE_LOG_CAP} entries (392 bytes). */
typedef struct {
  uint32_t count; /**< Valid entries, 0..CAP. */
  uint32_t head;  /**< Next write slot, 0..CAP-1. */
  timebase_log_entry_t entries[TIMEBASE_LOG_CAP];
} timebase_log_t;

/** Serialised flash image of the log (404 bytes). */
typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  timebase_log_t log;
  uint32_t crc; /**< CRC-32 of every byte before this field. */
} timebase_log_record_t;

TIMEBASE_LOG_STATIC_ASSERT(sizeof(timebase_log_entry_t) == 16,
                           "entry must be 16 bytes");
TIMEBASE_LOG_STATIC_ASSERT(offsetof(timebase_log_entry_t, ppm) == 4 &&
                               offsetof(timebase_log_entry_t, source) == 8 &&
                               offsetof(timebase_log_entry_t, reason) == 9 &&
                               offsetof(timebase_log_entry_t, reserved) == 10 &&
                               offsetof(timebase_log_entry_t, duration_s) == 12,
                           "entry layout is part of the flash format");
TIMEBASE_LOG_STATIC_ASSERT(sizeof(timebase_log_t) == 8 + 16 * TIMEBASE_LOG_CAP,
                           "log must have no padding");
TIMEBASE_LOG_STATIC_ASSERT(sizeof(timebase_log_record_t) == 404 &&
                               offsetof(timebase_log_record_t, log) == 8 &&
                               offsetof(timebase_log_record_t, crc) == 400,
                           "record layout is part of the flash format");
TIMEBASE_LOG_STATIC_ASSERT(sizeof(timebase_log_record_t) <= 512,
                           "record must fit its 512 B flash region");

/** Input of {@link timebase_log_pps_decision}. */
typedef struct {
  bool pps_locked;         /**< PPS currently locked (and GPS fix valid). */
  uint32_t pps_window;     /**< Pulses in the current average. */
  uint32_t locked_since_s; /**< Continuous lock duration (s). */
  float pps_ppm;           /**< Measured correction. */
  uint32_t utc_now;        /**< GPS UTC seconds, 0 = unknown. */
} timebase_log_pps_input_t;

/**
 * CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF).
 * Bit-identical to the private crc32 in timebase.cpp.
 *
 * @param data Bytes to checksum.
 * @param n Byte count.
 * @return The CRC.
 */
static inline uint32_t timebase_log_crc32(const void *data, size_t n) {
  const uint8_t *p = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

/** Empties the log (all bytes zero, so flash images are deterministic). */
static inline void timebase_log_init(timebase_log_t *log) {
  memset(log, 0, sizeof(*log));
}

/** Alias of {@link timebase_log_init}. */
static inline void timebase_log_clear(timebase_log_t *log) {
  timebase_log_init(log);
}

/**
 * Appends {@code e}, dropping the oldest entry when full. The reserved field
 * is forced to zero.
 */
static inline void timebase_log_append(timebase_log_t *log,
                                       const timebase_log_entry_t *e) {
  log->entries[log->head] = *e;
  log->entries[log->head].reserved = 0;
  log->head = (log->head + 1u) % TIMEBASE_LOG_CAP;
  if (log->count < TIMEBASE_LOG_CAP) {
    log->count++;
  }
}

/**
 * @param i Index, 0 = oldest.
 * @return The i-th entry oldest-first, or NULL if {@code i >= count}.
 */
static inline const timebase_log_entry_t *
timebase_log_get(const timebase_log_t *log, uint32_t i) {
  if (i >= log->count) {
    return NULL;
  }
  const uint32_t slot =
      (log->head + TIMEBASE_LOG_CAP - log->count + i) % TIMEBASE_LOG_CAP;
  return &log->entries[slot];
}

/** @return The newest entry, or NULL when empty. */
static inline const timebase_log_entry_t *
timebase_log_last(const timebase_log_t *log) {
  return log->count ? timebase_log_get(log, log->count - 1u) : NULL;
}

/** @return "save", "delta", "monthly", or "unknown". */
static inline const char *timebase_log_reason_name(uint8_t reason) {
  switch (reason) {
  case TIMEBASE_LOG_REASON_SAVE:
    return "save";
  case TIMEBASE_LOG_REASON_DELTA:
    return "delta";
  case TIMEBASE_LOG_REASON_MONTHLY:
    return "monthly";
  default:
    return "unknown";
  }
}

/** @return True if {@code ppm} is finite and within 0.05..10. */
static inline bool timebase_log_delta_valid(float ppm) {
  return ppm >= TIMEBASE_LOG_DELTA_PPM_MIN &&
         ppm <= TIMEBASE_LOG_DELTA_PPM_MAX; /* false for NaN */
}

/**
 * Decides whether a PPS measurement should be appended to the log.
 *
 * Candidate only when locked, {@code pps_window == 64}, continuously locked
 * for >= 600 s and {@code utc_now != 0}; otherwise NONE. Then, against the
 * last entry of any source:
 *  - empty log: DELTA (the "first" entry; the reason set is only
 *    save/delta/monthly);
 *  - {@code utc_now - last.date >= 30 d}: MONTHLY (wins over DELTA);
 *  - {@code |ppm - last.ppm| >= log_delta_ppm} and age >= 1 d: DELTA;
 *  - clock went backwards ({@code utc_now < last.date}): NONE, unless the
 *    stored date is more than 1 day in the future (it was dated by a bad
 *    clock), in which case the age test is waived and only the ppm delta
 *    test applies (DELTA). A small backwards step is NONE.
 *
 * @param log Current log.
 * @param in Current PPS state.
 * @param log_delta_ppm Delta threshold (ppm); callers validate it with
 *        {@link timebase_log_delta_valid}.
 * @return NONE / DELTA / MONTHLY.
 */
static inline timebase_log_decision_t
timebase_log_pps_decision(const timebase_log_t *log,
                          const timebase_log_pps_input_t *in,
                          float log_delta_ppm) {
  if (!in->pps_locked || in->pps_window != TIMEBASE_LOG_PPS_WINDOW_FULL ||
      in->locked_since_s < TIMEBASE_LOG_PPS_MIN_LOCK_S || in->utc_now == 0u) {
    return TIMEBASE_LOG_NONE;
  }
  const timebase_log_entry_t *last = timebase_log_last(log);
  if (!last) {
    return TIMEBASE_LOG_DELTA;
  }
  const bool moved = fabsf(in->pps_ppm - last->ppm) >= log_delta_ppm;
  if (in->utc_now < last->date_unix) {
    const uint32_t ahead = last->date_unix - in->utc_now;
    return (ahead > TIMEBASE_LOG_MIN_DELTA_AGE_S && moved) ? TIMEBASE_LOG_DELTA
                                                           : TIMEBASE_LOG_NONE;
  }
  const uint32_t age = in->utc_now - last->date_unix;
  if (age >= TIMEBASE_LOG_MONTHLY_S) {
    return TIMEBASE_LOG_MONTHLY;
  }
  if (moved && age >= TIMEBASE_LOG_MIN_DELTA_AGE_S) {
    return TIMEBASE_LOG_DELTA;
  }
  return TIMEBASE_LOG_NONE;
}

/**
 * Serialises {@code log} into a flash record (magic, version, size, CRC).
 *
 * @param log Log to store.
 * @param out Receives the record; fully overwritten.
 */
static inline void timebase_log_encode(const timebase_log_t *log,
                                       timebase_log_record_t *out) {
  memset(out, 0, sizeof(*out));
  out->magic = TIMEBASE_LOG_MAGIC;
  out->version = TIMEBASE_LOG_VERSION;
  out->size = (uint16_t)sizeof(*out);
  out->log = *log;
  out->crc = timebase_log_crc32(out, offsetof(timebase_log_record_t, crc));
}

/**
 * Validates a flash record and extracts the log.
 *
 * Rejects bad magic (including erased 0xFF flash), version, size, CRC, and
 * out-of-range count/head. {@code out} is untouched on failure.
 *
 * @param rec Record read from flash.
 * @param out Receives the log on success.
 * @return True if valid.
 */
static inline bool timebase_log_decode(const timebase_log_record_t *rec,
                                       timebase_log_t *out) {
  if (rec->magic != TIMEBASE_LOG_MAGIC ||
      rec->version != TIMEBASE_LOG_VERSION || rec->size != sizeof(*rec) ||
      rec->crc !=
          timebase_log_crc32(rec, offsetof(timebase_log_record_t, crc)) ||
      rec->log.count > TIMEBASE_LOG_CAP || rec->log.head >= TIMEBASE_LOG_CAP) {
    return false;
  }
  *out = rec->log;
  return true;
}

#ifdef __cplusplus
}
#endif
