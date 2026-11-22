/**
 * @file nmea.h
 * @brief Minimal NMEA 0183 reader for GPS fix state and UTC date/time.
 *
 * Feeds one character at a time; parses the sentences the timebase and the GPS
 * status page need, from any talker (GP, GN, GL, GA, BD, ...):
 *  - RMC: fix status (A/V, and the NMEA 2.3+ mode field) and UTC date/time;
 *  - GGA: fix quality, satellites in use, HDOP, latitude / longitude /
 * altitude;
 *  - GSA: fix mode, PDOP / HDOP / VDOP and the PRNs used in the solution;
 *  - GSV: per-satellite elevation / azimuth / SNR into a small satellite table.
 * Sentences without a valid "*hh" checksum are counted and discarded.
 *
 * Satellite bookkeeping (see the nmea_sat_t table):
 *  - Satellites are keyed by (system, PRN). The system comes from the talker
 *    (GP=GPS, GL=GLONASS, GA=Galileo, GB/BD=BeiDou, GQ/QZ=QZSS). For the GN
 *    talker (GSA) the NMEA 4.1 system-id field is used if present, otherwise a
 *    PRN-range heuristic: 1..64 GPS (incl. SBAS 33..64), 65..96 GLONASS,
 *    193..202 QZSS, 301..336 Galileo, 401..437 BeiDou, else unknown.
 *  - PRNs are normalised per system so GSA and GSV agree whichever numbering
 *    the receiver uses: GLONASS 65..96 -> 1..32, Galileo 301..336 -> 1..36,
 *    BeiDou 401..437 -> 1..37. Other PRNs are kept as received.
 *  - GSV messages are applied one at a time (a missing message of a group only
 *    loses its satellites). The caller supplies a millisecond clock
 *    ({@link nmea_feed_at}); entries not seen for {@link NMEA_SAT_MAX_AGE_MS}
 *    are dropped by {@link nmea_sats_prune} (also run on every GSV).
 *  - Multi-GNSS receivers send one GSA per system per epoch. The used set is
 *    the union of all GSAs of an epoch. The epoch boundary is the first GSA
 *    after an RMC or GGA: that GSA clears the previous used set. The set stays
 *    valid until then (no clearing on RMC/GGA alone), so it works whether the
 *    receiver sends GSA before or after RMC/GGA.
 *  - With NMEA 4.1 GSV (trailing signal-id) the same satellite appears once per
 *    signal; within one timestamp the strongest SNR is kept.
 *
 * Header-only and SDK-free so it is host-testable (tests/test_nmea.cpp).
 *
 * @see timebase.cpp
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NMEA_MAX_LEN    96u /* NMEA limit is 82 incl. "$" and CRLF */
#define NMEA_MAX_FIELDS 24u /* GSV 4.1: 1 + 3 + 4*4 + 1 = 21 */

/** Bits returned by {@link nmea_feed}. */
#define NMEA_GOT_RMC 1u
#define NMEA_GOT_GGA 2u
#define NMEA_GOT_GSA                                                           \
  4u /**< One GSA parsed (used set may be partial for the epoch). */
#define NMEA_GOT_GSV                                                           \
  8u /**< One GSV message parsed (not necessarily the whole group). */

/** Capacity of the satellite table. */
#define NMEA_MAX_SATS       32u
/** Capacity of the per-epoch used-satellite set (12 per GSA, several systems).
 */
#define NMEA_MAX_USED       48u
/** Satellites not seen in a GSV for this long are dropped. */
#define NMEA_SAT_MAX_AGE_MS 5000u

/** GNSS system; values 1..5 equal the NMEA 4.1 GSA system-id field. */
typedef enum {
  NMEA_SYS_UNKNOWN = 0,
  NMEA_SYS_GPS = 1,
  NMEA_SYS_GLONASS = 2,
  NMEA_SYS_GALILEO = 3,
  NMEA_SYS_BEIDOU = 4,
  NMEA_SYS_QZSS = 5,
  NMEA_SYS_COUNT = 6
} nmea_sys_t;

/** One tracked satellite. */
typedef struct {
  uint16_t prn;          /**< Normalised PRN (see file header). */
  int16_t az;            /**< Azimuth degrees 0..359, -1 unknown. */
  uint32_t last_seen_ms; /**< Caller clock at the last GSV mentioning it. */
  uint8_t sys;           /**< {@link nmea_sys_t}. */
  int8_t el;             /**< Elevation degrees 0..90, -1 unknown. */
  int8_t snr;            /**< C/N0 dB-Hz, -1 = not tracked. */
  bool used;             /**< Listed in a GSA of the current epoch. */
} nmea_sat_t;

typedef struct {
  char buf[NMEA_MAX_LEN];
  uint8_t len;
  bool in_sentence;

  /* Latest RMC. */
  bool rmc_valid;  /**< Status 'A' and mode (if present) not 'N'. */
  bool time_valid; /**< UTC date and time parsed (needs a date >= 2020). */
  uint32_t unix_s; /**< UTC seconds of the last RMC with valid time. */

  /* Latest GGA. */
  uint8_t gga_quality; /**< 0 = no fix, 1 = GPS, 2 = DGPS, ... */
  uint8_t sats;        /**< Satellites used in the solution. */
  float hdop;          /**< 0 = unknown. */
  double lat;          /**< Signed degrees, + = N. Valid if pos_valid. */
  double lon;          /**< Signed degrees, + = E. Valid if pos_valid. */
  float alt_m;         /**< Altitude above MSL (GGA), 0 if absent. */
  bool pos_valid;      /**< GGA quality > 0 and lat/lon parsed. */

  /* Latest GSA. */
  uint8_t fix_mode; /**< 1 = none, 2 = 2D, 3 = 3D, 0 = no GSA yet. */
  float pdop;       /**< 0 = unknown. */
  float gsa_hdop;   /**< HDOP from GSA, 0 = unknown. */
  float vdop;       /**< 0 = unknown. */

  /* Satellites. */
  uint32_t now_ms; /**< Clock for nmea_feed (set by nmea_feed_at). */
  nmea_sat_t sat_tab[NMEA_MAX_SATS]; /**< Contiguous, first sat_count valid. */
  uint8_t sat_count;
  uint8_t in_view[NMEA_SYS_COUNT];  /**< "Sats in view" of the last GSV per
                                       system. */
  uint8_t gsv_next[NMEA_SYS_COUNT]; /**< Next expected GSV message number, 0 =
                                       broken. */
  uint32_t gsv_groups;              /**< Complete, in-order GSV groups seen. */
  uint16_t used[NMEA_MAX_USED];     /**< Epoch used set: (sys << 10) | prn. */
  uint8_t used_count;
  bool gsa_new_epoch; /**< RMC/GGA seen since the last GSA. */

  /* Counters. */
  uint32_t sentences;       /**< Sentences with a good checksum (any type). */
  uint32_t checksum_errors; /**< Missing / bad checksum or overlong line. */
} nmea_t;

static inline void nmea_init(nmea_t *n) { memset(n, 0, sizeof(*n)); }

static inline int nmea_hex(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

/** Unsigned decimal of exactly {@code len} digits at {@code s}, or -1. */
static inline int32_t nmea_digits(const char *s, size_t len) {
  int32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    if (s[i] < '0' || s[i] > '9')
      return -1;
    v = v * 10 + (s[i] - '0');
  }
  return v;
}

/** Leading unsigned integer of a field (stops at '.' or end), or -1 if empty.
 */
static inline int32_t nmea_uint(const char *s) {
  if (!s || *s < '0' || *s > '9')
    return -1;
  int32_t v = 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (*s - '0');
    s++;
  }
  return v;
}

/** Small decimal like "1.27" (no exponent), or -1 if empty / malformed. */
static inline float nmea_float(const char *s) {
  if (!s || !*s)
    return -1.f;
  float v = 0.f, scale = 0.f;
  for (; *s; s++) {
    if (*s == '.') {
      if (scale != 0.f)
        return -1.f;
      scale = 1.f;
    } else if (*s >= '0' && *s <= '9') {
      if (scale != 0.f) {
        scale *= 0.1f;
        v += (float)(*s - '0') * scale;
      } else {
        v = v * 10.f + (float)(*s - '0');
      }
    } else {
      return -1.f;
    }
  }
  return v;
}

/** Days since 1970-01-01 for a proleptic Gregorian date (H. Hinnant). */
static inline int32_t nmea_days_from_civil(int32_t y, uint32_t m, uint32_t d) {
  y -= m <= 2u;
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = (uint32_t)(y - era * 400);
  const uint32_t doy =
      (153u * (m + (m > 2u ? (uint32_t)-3 : 9u)) + 2u) / 5u + d - 1u;
  const uint32_t doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

/** Parses "hhmmss[.ss]" + "ddmmyy" into UNIX seconds; false if malformed. */
static inline bool nmea_parse_datetime(const char *t, const char *d,
                                       uint32_t *out) {
  if (strlen(t) < 6u || strlen(d) != 6u)
    return false;
  const int32_t hh = nmea_digits(t, 2), mi = nmea_digits(t + 2, 2),
                ss = nmea_digits(t + 4, 2);
  const int32_t dd = nmea_digits(d, 2), mo = nmea_digits(d + 2, 2),
                yy = nmea_digits(d + 4, 2);
  if (hh < 0 || hh > 23 || mi < 0 || mi > 59 || ss < 0 || ss > 60 || dd < 1 ||
      dd > 31 || mo < 1 || mo > 12 || yy < 0) {
    return false;
  }
  /* Two-digit year: 80..99 → 19xx (also what a GPS week-rollover bug looks
   * like). */
  const int32_t year = yy >= 80 ? 1900 + yy : 2000 + yy;
  if (year < 2020)
    return false;
  const int32_t days = nmea_days_from_civil(year, (uint32_t)mo, (uint32_t)dd);
  *out = (uint32_t)days * 86400u + (uint32_t)(hh * 3600 + mi * 60 + ss);
  return true;
}

/** Parses [-]digits[.digits] fully (no exponent) into {@code *out}; false if
 * malformed. */
static inline bool nmea_double(const char *s, double *out) {
  if (!s || !*s)
    return false;
  bool neg = false;
  if (*s == '-') {
    neg = true;
    s++;
  }
  double v = 0.0;
  unsigned digits = 0, frac = 0;
  bool dot = false;
  for (; *s; s++) {
    if (*s == '.') {
      if (dot)
        return false;
      dot = true;
    } else if (*s >= '0' && *s <= '9') {
      digits++;
      if (digits <= 17u) {
        v = v * 10.0 + (double)(*s - '0');
        if (dot)
          frac++;
      }
    } else {
      return false;
    }
  }
  if (digits == 0u)
    return false;
  double d = 1.0;
  while (frac--)
    d *= 10.0;
  v /= d;
  *out = neg ? -v : v;
  return true;
}

/**
 * Parses an NMEA coordinate "ddmm.mmmm" / "dddmm.mmmm" plus hemisphere.
 *
 * @param deg_digits 2 for latitude, 3 for longitude.
 * @param hemi       field with 'N'/'S' (latitude) or 'E'/'W' (longitude).
 * @return false if malformed or out of range.
 */
static inline bool nmea_coord(const char *s, const char *hemi,
                              unsigned deg_digits, double *out) {
  if (!s || !hemi)
    return false;
  unsigned whole = 0;
  while (s[whole] >= '0' && s[whole] <= '9')
    whole++;
  if (whole != deg_digits + 2u)
    return false;
  double v = 0.0;
  if (!nmea_double(s, &v))
    return false;
  const double deg = (double)(int)(v / 100.0);
  const double min = v - deg * 100.0;
  if (min >= 60.0)
    return false;
  double r = deg + min / 60.0;
  if (r > (deg_digits == 2u ? 90.0 : 180.0))
    return false;
  const char h = hemi[0];
  const bool lat = deg_digits == 2u;
  if (h == (lat ? 'S' : 'W')) {
    r = -r;
  } else if (h != (lat ? 'N' : 'E')) {
    return false;
  }
  *out = r;
  return true;
}

/** Short system name: "GP","GL","GA","BD","QZ" or "??". */
static inline const char *nmea_sys_name(unsigned sys) {
  switch (sys) {
  case NMEA_SYS_GPS:
    return "GP";
  case NMEA_SYS_GLONASS:
    return "GL";
  case NMEA_SYS_GALILEO:
    return "GA";
  case NMEA_SYS_BEIDOU:
    return "BD";
  case NMEA_SYS_QZSS:
    return "QZ";
  default:
    return "??";
  }
}

/** System from a two-letter talker id; NMEA_SYS_UNKNOWN for GN and others. */
static inline nmea_sys_t nmea_sys_from_talker(const char *t) {
  if (t[0] == 'G') {
    switch (t[1]) {
    case 'P':
      return NMEA_SYS_GPS;
    case 'L':
      return NMEA_SYS_GLONASS;
    case 'A':
      return NMEA_SYS_GALILEO;
    case 'B':
      return NMEA_SYS_BEIDOU;
    case 'Q':
      return NMEA_SYS_QZSS;
    default:
      break;
    }
  } else if (t[0] == 'B' && t[1] == 'D') {
    return NMEA_SYS_BEIDOU;
  } else if (t[0] == 'Q' && t[1] == 'Z') {
    return NMEA_SYS_QZSS;
  }
  return NMEA_SYS_UNKNOWN;
}

/** PRN-range heuristic for the GN talker without a system id (see file header).
 */
static inline nmea_sys_t nmea_sys_from_prn(uint32_t prn) {
  if (prn >= 1u && prn <= 64u)
    return NMEA_SYS_GPS;
  if (prn >= 65u && prn <= 96u)
    return NMEA_SYS_GLONASS;
  if (prn >= 193u && prn <= 202u)
    return NMEA_SYS_QZSS;
  if (prn >= 301u && prn <= 336u)
    return NMEA_SYS_GALILEO;
  if (prn >= 401u && prn <= 437u)
    return NMEA_SYS_BEIDOU;
  return NMEA_SYS_UNKNOWN;
}

/** Normalises a PRN within its system (see file header). */
static inline uint16_t nmea_norm_prn(unsigned sys, uint32_t prn) {
  if (sys == NMEA_SYS_GLONASS && prn >= 65u && prn <= 96u)
    prn -= 64u;
  if (sys == NMEA_SYS_GALILEO && prn >= 301u && prn <= 336u)
    prn -= 300u;
  if (sys == NMEA_SYS_BEIDOU && prn >= 401u && prn <= 437u)
    prn -= 400u;
  return (uint16_t)prn;
}

/** Field {@code i} of a split sentence, or "" if absent. */
static inline const char *nmea_fld(const char *const *f, size_t nf, size_t i) {
  return i < nf ? f[i] : "";
}

/** Recomputes nmea_sat_t::used for every satellite from the epoch used set. */
static inline void nmea_sats_mark_used(nmea_t *n) {
  for (uint8_t i = 0; i < n->sat_count; i++) {
    const uint16_t key =
        (uint16_t)((n->sat_tab[i].sys << 10) | n->sat_tab[i].prn);
    bool u = false;
    for (uint8_t j = 0; j < n->used_count && !u; j++)
      u = n->used[j] == key;
    n->sat_tab[i].used = u;
  }
}

/**
 * Drops satellites not seen for more than {@link NMEA_SAT_MAX_AGE_MS}.
 *
 * @param now_ms caller's millisecond clock (wrap-safe).
 * @return remaining satellite count.
 */
static inline uint8_t nmea_sats_prune(nmea_t *n, uint32_t now_ms) {
  uint8_t w = 0;
  for (uint8_t r = 0; r < n->sat_count; r++) {
    if ((uint32_t)(now_ms - n->sat_tab[r].last_seen_ms) <=
        NMEA_SAT_MAX_AGE_MS) {
      if (w != r)
        n->sat_tab[w] = n->sat_tab[r];
      w++;
    }
  }
  n->sat_count = w;
  return w;
}

/**
 * Finds or creates the entry for (sys, prn); when the table is full the
 * stalest entry is recycled.
 *
 * @param[out] existed true if the entry was already present.
 */
static inline nmea_sat_t *nmea_sat_slot(nmea_t *n, unsigned sys, uint16_t prn,
                                        bool *existed) {
  for (uint8_t i = 0; i < n->sat_count; i++) {
    if (n->sat_tab[i].sys == sys && n->sat_tab[i].prn == prn) {
      *existed = true;
      return &n->sat_tab[i];
    }
  }
  *existed = false;
  nmea_sat_t *s;
  if (n->sat_count < NMEA_MAX_SATS) {
    s = &n->sat_tab[n->sat_count++];
  } else {
    s = &n->sat_tab[0];
    for (uint8_t i = 1; i < NMEA_MAX_SATS; i++) {
      if ((uint32_t)(n->now_ms - n->sat_tab[i].last_seen_ms) >
          (uint32_t)(n->now_ms - s->last_seen_ms)) {
        s = &n->sat_tab[i];
      }
    }
  }
  memset(s, 0, sizeof(*s));
  s->sys = (uint8_t)sys;
  s->prn = prn;
  return s;
}

/** Applies one validated GSV message ({@code f}/{@code nf}: split fields). */
static inline void nmea_gsv(nmea_t *n, const char *const *f, size_t nf) {
  const int32_t total = nmea_uint(nmea_fld(f, nf, 1));
  const int32_t msg = nmea_uint(nmea_fld(f, nf, 2));
  const int32_t view = nmea_uint(nmea_fld(f, nf, 3));
  if (total < 1 || msg < 1 || msg > total)
    return;
  const nmea_sys_t tsys = nmea_sys_from_talker(f[0]);

  /* Group bookkeeping per talker system. */
  if (msg == 1) {
    n->gsv_next[tsys] = 2;
  } else if (n->gsv_next[tsys] == (uint8_t)msg) {
    n->gsv_next[tsys]++;
  } else {
    n->gsv_next[tsys] = 0;
  }
  if (msg == total && n->gsv_next[tsys] == (uint8_t)(msg + 1))
    n->gsv_groups++;
  n->in_view[tsys] = view > 0 && view < 256 ? (uint8_t)view : 0u;

  const size_t groups = nf > 4u ? (nf - 4u) / 4u : 0u;
  const bool has_sig = nf > 4u && (nf - 4u) % 4u == 1u;
  for (size_t g = 0; g < groups && g < 4u; g++) {
    const char *const *q = f + 4u + 4u * g;
    const int32_t prn = nmea_uint(q[0]);
    if (prn < 1 || prn > 1023)
      continue;
    nmea_sys_t sys =
        tsys != NMEA_SYS_UNKNOWN ? tsys : nmea_sys_from_prn((uint32_t)prn);
    const uint16_t np = nmea_norm_prn(sys, (uint32_t)prn);
    const int32_t el = nmea_uint(q[1]), az = nmea_uint(q[2]),
                  snr = nmea_uint(q[3]);
    bool existed = false;
    nmea_sat_t *s = nmea_sat_slot(n, sys, np, &existed);
    s->el = el >= 0 && el <= 90 ? (int8_t)el : (int8_t)-1;
    s->az = az >= 0 && az <= 359 ? (int16_t)az : (int16_t)-1;
    const int8_t ns = snr >= 0 && snr <= 127 ? (int8_t)snr : (int8_t)-1;
    if (!(has_sig && existed && s->last_seen_ms == n->now_ms && s->snr > ns))
      s->snr = ns;
    s->last_seen_ms = n->now_ms;
  }
  nmea_sats_prune(n, n->now_ms);
  nmea_sats_mark_used(n);
}

/** Applies one GSA sentence. */
static inline void nmea_gsa(nmea_t *n, const char *const *f, size_t nf) {
  const int32_t mode = nmea_uint(nmea_fld(f, nf, 2));
  n->fix_mode = mode >= 1 && mode <= 3 ? (uint8_t)mode : 0u;
  float v = nmea_float(nmea_fld(f, nf, 15));
  n->pdop = v > 0.f ? v : 0.f;
  v = nmea_float(nmea_fld(f, nf, 16));
  n->gsa_hdop = v > 0.f ? v : 0.f;
  v = nmea_float(nmea_fld(f, nf, 17));
  n->vdop = v > 0.f ? v : 0.f;

  if (n->gsa_new_epoch) { /* first GSA of a new epoch: drop the old union */
    n->used_count = 0;
    n->gsa_new_epoch = false;
  }
  const int32_t sid = nmea_uint(nmea_fld(f, nf, 18));
  const nmea_sys_t tsys = nmea_sys_from_talker(f[0]);
  for (size_t i = 3; i <= 14u; i++) {
    const int32_t prn = nmea_uint(nmea_fld(f, nf, i));
    if (prn < 1 || prn > 1023)
      continue;
    nmea_sys_t sys = sid >= 1 && sid <= 5 ? (nmea_sys_t)sid : tsys;
    if (sys == NMEA_SYS_UNKNOWN)
      sys = nmea_sys_from_prn((uint32_t)prn);
    const uint16_t key =
        (uint16_t)((sys << 10) | nmea_norm_prn(sys, (uint32_t)prn));
    bool dup = false;
    for (uint8_t j = 0; j < n->used_count && !dup; j++)
      dup = n->used[j] == key;
    if (!dup && n->used_count < NMEA_MAX_USED)
      n->used[n->used_count++] = key;
  }
  nmea_sats_mark_used(n);
}

/** Handles one complete line in {@code n->buf} (without '$' / CRLF). */
static inline unsigned nmea_line(nmea_t *n) {
  char *s = n->buf;
  char *star = strchr(s, '*');
  if (!star || nmea_hex(star[1]) < 0 || nmea_hex(star[2]) < 0) {
    n->checksum_errors++;
    return 0;
  }
  uint8_t sum = 0;
  for (char *p = s; p < star; p++)
    sum ^= (uint8_t)*p;
  if (sum != (uint8_t)(nmea_hex(star[1]) * 16 + nmea_hex(star[2]))) {
    n->checksum_errors++;
    return 0;
  }
  *star = '\0';
  n->sentences++;

  const char *f[NMEA_MAX_FIELDS];
  size_t nf = 0;
  f[nf++] = s;
  for (char *p = s; *p && nf < NMEA_MAX_FIELDS; p++) {
    if (*p == ',') {
      *p = '\0';
      f[nf++] = p + 1;
    }
  }
  if (strlen(f[0]) != 5u)
    return 0; /* "GNRMC": talker + type */
  const char *type = f[0] + 2;

  if (strcmp(type, "RMC") == 0 && nf >= 10u) {
    bool valid = f[2][0] == 'A';
    if (nf >= 13u && f[12][0] == 'N')
      valid = false; /* mode: not valid */
    n->gsa_new_epoch = true;
    n->rmc_valid = valid;
    uint32_t t = 0;
    n->time_valid = nmea_parse_datetime(f[1], f[9], &t);
    if (n->time_valid)
      n->unix_s = t;
    return NMEA_GOT_RMC;
  }
  if (strcmp(type, "GGA") == 0 && nf >= 9u) {
    const int32_t q = nmea_uint(f[6]);
    const int32_t sats = nmea_uint(f[7]);
    const float hdop = nmea_float(f[8]);
    n->gga_quality = q > 0 && q < 256 ? (uint8_t)q : 0u;
    n->sats = sats > 0 && sats < 256 ? (uint8_t)sats : 0u;
    n->hdop = hdop > 0.f ? hdop : 0.f;
    n->gsa_new_epoch = true;
    double la = 0.0, lo = 0.0, alt = 0.0;
    const bool ok = n->gga_quality > 0u &&
                    nmea_coord(f[2], nmea_fld(f, nf, 3), 2u, &la) &&
                    nmea_coord(nmea_fld(f, nf, 4), nmea_fld(f, nf, 5), 3u, &lo);
    n->pos_valid = ok;
    n->lat = ok ? la : 0.0;
    n->lon = ok ? lo : 0.0;
    n->alt_m = nmea_double(nmea_fld(f, nf, 9), &alt) ? (float)alt : 0.f;
    return NMEA_GOT_GGA;
  }
  if (strcmp(type, "GSA") == 0) {
    nmea_gsa(n, f, nf);
    return NMEA_GOT_GSA;
  }
  if (strcmp(type, "GSV") == 0) {
    nmea_gsv(n, f, nf);
    return NMEA_GOT_GSV;
  }
  return 0;
}

/**
 * Feeds one received character.
 *
 * Satellite ageing uses {@code n->now_ms}; set it (or use {@link
 * nmea_feed_at}).
 *
 * @return {@link NMEA_GOT_RMC} / {@link NMEA_GOT_GGA} / {@link NMEA_GOT_GSA} /
 *         {@link NMEA_GOT_GSV} when this character completed a valid sentence
 *         of that type, else 0.
 */
static inline unsigned nmea_feed(nmea_t *n, char c) {
  if (c == '$') {
    n->in_sentence = true;
    n->len = 0;
    return 0;
  }
  if (!n->in_sentence)
    return 0;
  if (c == '\r' || c == '\n') {
    n->in_sentence = false;
    n->buf[n->len] = '\0';
    return nmea_line(n);
  }
  if (n->len + 1u >= NMEA_MAX_LEN) {
    n->in_sentence = false; /* overlong: drop */
    n->checksum_errors++;
    return 0;
  }
  n->buf[n->len++] = c;
  return 0;
}

/**
 * {@link nmea_feed} with a millisecond timestamp for satellite ageing.
 *
 * @param now_ms caller's monotonic millisecond clock (wraps safely).
 */
static inline unsigned nmea_feed_at(nmea_t *n, char c, uint32_t now_ms) {
  n->now_ms = now_ms;
  return nmea_feed(n, c);
}
