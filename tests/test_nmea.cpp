/**
 * @file test_nmea.cpp
 * @brief Host test for the NMEA fix / time reader.
 *
 * Build and run:
 *   c++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/test_nmea.cpp -o
 * /tmp/t && /tmp/t
 */

#include "nmea.h"

#include <cassert>
#include <cstdio>
#include <cstring>

namespace {

/** Wraps {@code body} as "$body*hh\r\n" with a correct checksum. */
const char *frame(const char *body) {
  static char out[128];
  unsigned sum = 0;
  for (const char *p = body; *p; p++) {
    sum ^= static_cast<unsigned char>(*p);
  }
  std::snprintf(out, sizeof(out), "$%s*%02X\r\n", body, sum);
  return out;
}

unsigned feed(nmea_t *n, const char *s) {
  unsigned got = 0;
  for (; *s; s++) {
    got |= nmea_feed(n, *s);
  }
  return got;
}

/** Feeds a sentence at time {@code ms}. */
unsigned feed_at(nmea_t *n, const char *s, uint32_t ms) {
  unsigned got = 0;
  for (; *s; s++) {
    got |= nmea_feed_at(n, *s, ms);
  }
  return got;
}

bool near(double a, double b, double eps = 1e-6) {
  return a - b < eps && b - a < eps;
}

const nmea_sat_t *find_sat(const nmea_t &n, unsigned sys, unsigned prn) {
  for (unsigned i = 0; i < n.sat_count; i++) {
    if (n.sat_tab[i].sys == sys && n.sat_tab[i].prn == prn)
      return &n.sat_tab[i];
  }
  return nullptr;
}

unsigned used_count(const nmea_t &n) {
  unsigned c = 0;
  for (unsigned i = 0; i < n.sat_count; i++)
    c += n.sat_tab[i].used ? 1u : 0u;
  return c;
}

void test_position() {
  nmea_t n;
  nmea_init(&n);
  /* Northern / eastern, with altitude. */
  assert(feed(&n, frame("GNGGA,123519.00,4807.03800,N,01131.00000,E,1,08,0.9,"
                        "545.4,M,46.9,M,,")) == NMEA_GOT_GGA);
  assert(n.pos_valid && near(n.lat, 48.0 + 7.038 / 60.0) &&
         near(n.lon, 11.0 + 31.0 / 60.0));
  assert(n.alt_m > 545.3f && n.alt_m < 545.5f);
  assert(n.gga_quality == 1u && n.sats == 8u);
  /* Southern / western, negative altitude. */
  feed(&n,
       frame("GPGGA,123519,3352.1234,S,15112.5678,W,2,09,1.1,-12.5,M,0,M,,"));
  assert(n.pos_valid && near(n.lat, -(33.0 + 52.1234 / 60.0)) &&
         near(n.lon, -(151.0 + 12.5678 / 60.0)));
  assert(n.alt_m < -12.4f && n.alt_m > -12.6f);
  /* Western hemisphere, northern: US. */
  feed(&n,
       frame("GPGGA,123519,3746.4940,N,12225.1640,W,1,07,1.0,16.0,M,0,M,,"));
  assert(near(n.lat, 37.0 + 46.494 / 60.0) &&
         near(n.lon, -(122.0 + 25.164 / 60.0)));
  /* No fix: empty fields -> invalid position. */
  feed(&n, frame("GNGGA,123520,,,,,0,00,99.99,,,,,,"));
  assert(!n.pos_valid && n.lat == 0.0 && n.lon == 0.0 && n.alt_m == 0.f);
  /* Quality 0 with coordinates (last known): not valid. */
  feed(&n, frame("GPGGA,123520,4807.038,N,01131.000,E,0,00,99.99,,,,,,"));
  assert(!n.pos_valid);
  /* Malformed hemisphere / minutes >= 60 / wrong digit count. */
  feed(&n, frame("GPGGA,1,4807.038,X,01131.000,E,1,08,0.9,5,M,,,,"));
  assert(!n.pos_valid);
  feed(&n, frame("GPGGA,1,4875.038,N,01131.000,E,1,08,0.9,5,M,,,,"));
  assert(!n.pos_valid);
  feed(&n, frame("GPGGA,1,480.7038,N,01131.000,E,1,08,0.9,5,M,,,,"));
  assert(!n.pos_valid);
  /* Missing altitude field entirely (short sentence) still parses position. */
  feed(&n, frame("GPGGA,1,4807.038,N,01131.000,E,1,08,0.9"));
  assert(n.pos_valid && n.alt_m == 0.f);
  /* Bad checksum leaves position alone. */
  feed(&n, frame("GPGGA,1,1000.000,N,01000.000,E,1,08,0.9,5,M,,,,"));
  const double lat = n.lat;
  feed(&n, "$GPGGA,1,2000.000,N,02000.000,E,1,08,0.9,5,M,,,,*00\r\n");
  assert(n.lat == lat);
}

void test_gsa_gsv() {
  nmea_t n;
  nmea_init(&n);
  uint32_t t = 1000;
  /* u-blox M8/M10 style multi-GNSS epoch: RMC, GGA, GSA per system, GSV per
   * system. */
  feed_at(
      &n,
      frame(
          "GNRMC,092750.00,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A,V"),
      t);
  feed_at(
      &n,
      frame(
          "GNGGA,092750.00,5321.6802,N,00630.3372,W,1,10,1.01,61.7,M,55.2,M,,"),
      t);
  /* GPS-only GSA with 4.1 system id 1, then GLONASS (id 2) and Galileo (id 3).
   */
  assert(feed_at(&n, frame("GNGSA,A,3,04,05,09,,,,,,,,,,1.80,1.01,1.50,1"),
                 t) == NMEA_GOT_GSA);
  assert(n.fix_mode == 3u && near(n.pdop, 1.8, 1e-4) &&
         near(n.gsa_hdop, 1.01, 1e-4) && near(n.vdop, 1.5, 1e-4));
  feed_at(&n, frame("GNGSA,A,3,65,66,,,,,,,,,,,1.80,1.01,1.50,2"), t);
  feed_at(&n, frame("GNGSA,A,3,02,07,,,,,,,,,,,1.80,1.01,1.50,3"), t);
  /* No system id, GN: PRN heuristic (BeiDou 401+). */
  feed_at(&n, frame("GNGSA,A,3,405,,,,,,,,,,,,1.80,1.01,1.50"), t);

  assert(
      feed_at(
          &n,
          frame(
              "GPGSV,2,1,06,04,60,100,40,05,30,200,35,09,10,300,,02,05,010,20"),
          t) == NMEA_GOT_GSV);
  feed_at(&n, frame("GPGSV,2,2,06,07,40,050,,12,50,060,"), t);
  feed_at(&n, frame("GLGSV,1,1,02,65,20,120,30,66,10,250,25"), t);
  feed_at(&n, frame("GAGSV,1,1,02,02,33,044,28,07,45,222,31,7"),
          t); /* sig id, 2 sats */
  feed_at(&n, frame("GBGSV,1,1,01,05,70,001,41"), t);

  const nmea_sat_t *s = find_sat(n, NMEA_SYS_GPS, 4);
  assert(s && s->el == 60 && s->az == 100 && s->snr == 40 && s->used);
  s = find_sat(n, NMEA_SYS_GPS, 9);
  assert(s && s->snr == -1 && s->used); /* not tracked but used per GSA */
  s = find_sat(n, NMEA_SYS_GPS, 12);
  assert(s && s->snr == -1 && s->el == 50 && s->az == 60 && !s->used);
  /* Same PRN in different systems are distinct. GA 02 used, GPS 02 not. */
  s = find_sat(n, NMEA_SYS_GPS, 2);
  assert(s && !s->used && s->snr == 20);
  s = find_sat(n, NMEA_SYS_GALILEO, 2);
  assert(s && s->used && s->snr == 28);
  s = find_sat(n, NMEA_SYS_GALILEO, 7);
  assert(s && s->used && s->snr == 31);
  /* GLONASS 65/66 normalised to 1/2 in both GSA and GSV. */
  s = find_sat(n, NMEA_SYS_GLONASS, 1);
  assert(s && s->used && s->snr == 30);
  s = find_sat(n, NMEA_SYS_GLONASS, 2);
  assert(s && s->used);
  s = find_sat(n, NMEA_SYS_BEIDOU, 5);
  assert(s && s->used &&
         s->snr == 41); /* GN GSA PRN 405 -> BeiDou 5 by heuristic */
  assert(used_count(n) == 3u + 2u + 2u + 1u);
  assert(n.in_view[NMEA_SYS_GPS] == 6u && n.gsv_groups == 4u);
  assert(n.sat_count == 6u + 2u + 2u + 1u);

  /* Next epoch: first GSA clears the union; the used set persists until then.
   */
  t += 1000;
  feed_at(
      &n,
      frame(
          "GNRMC,092751.00,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A,V"),
      t);
  assert(used_count(n) == 8u);
  feed_at(
      &n,
      frame(
          "GNGGA,092751.00,5321.6802,N,00630.3372,W,1,10,1.01,61.7,M,55.2,M,,"),
      t);
  feed_at(&n, frame("GNGSA,A,3,05,,,,,,,,,,,,1.80,1.01,1.50,1"), t);
  assert(used_count(n) == 1u);
  feed_at(&n, frame("GNGSA,A,3,06,,,,,,,,,,,,1.80,1.01,1.50,1"),
          t);                  /* union within epoch */
  assert(used_count(n) == 1u); /* GPS 06 not in the table yet */
  feed_at(&n, frame("GPGSV,1,1,01,06,10,011,22"), t);
  assert(find_sat(n, NMEA_SYS_GPS, 6)->used &&
         find_sat(n, NMEA_SYS_GPS, 5)->used &&
         !find_sat(n, NMEA_SYS_GPS, 4)->used);

  /* GSA before RMC/GGA ordering also works; no-fix GSA clears used. */
  t += 1000;
  feed_at(&n, frame("GPRMC,092752.00,V,,,,,,,280511,,,N"), t);
  feed_at(&n, frame("GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99"), t);
  assert(n.fix_mode == 1u && used_count(n) == 0u);

  /* GPS-only GP output, 12 PRNs in one GSA, NMEA 2.x (no system id). */
  nmea_init(&n);
  feed_at(&n,
          frame("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,2.0,1.0,1.7"),
          t);
  assert(n.used_count == 12u && n.fix_mode == 3u);
  /* Empty GSA fields and short sentence tolerated. */
  nmea_init(&n);
  assert(feed_at(&n, frame("GPGSA,A,,,,,,,,,,,,,,,"), t) == NMEA_GOT_GSA);
  assert(n.fix_mode == 0u && n.used_count == 0u && n.pdop == 0.f);
  /* GSA with a bad checksum is ignored. */
  assert(feed_at(&n, "$GPGSA,A,3,01,02,,,,,,,,,,,2.0,1.0,1.7*00\r\n", t) == 0u);
  assert(n.used_count == 0u);
}

void test_aging_and_groups() {
  nmea_t n;
  nmea_init(&n);
  feed_at(&n, frame("GPGSV,1,1,02,01,10,010,20,02,20,020,30"), 1000);
  assert(n.sat_count == 2u);
  /* Refreshing only PRN 1 later: PRN 2 ages out after 5 s, not before. */
  feed_at(&n, frame("GPGSV,1,1,01,01,11,011,21"), 5500);
  assert(n.sat_count == 2u);
  feed_at(&n, frame("GPGSV,1,1,01,01,12,012,22"), 6001);
  assert(n.sat_count == 1u && find_sat(n, NMEA_SYS_GPS, 1) &&
         !find_sat(n, NMEA_SYS_GPS, 2));
  assert(find_sat(n, NMEA_SYS_GPS, 1)->el == 12);
  /* Explicit prune. */
  assert(nmea_sats_prune(&n, 6001 + 5000) == 1u);
  assert(nmea_sats_prune(&n, 6001 + 5001) == 0u);
  /* Clock wrap is safe. */
  feed_at(&n, frame("GPGSV,1,1,01,03,10,010,20"), 0xFFFFFF00u);
  assert(nmea_sats_prune(&n, 0x00000010u) == 1u);

  /* Group with a missing message: sats from msgs that arrived are kept, group
   * not counted. */
  nmea_init(&n);
  feed_at(
      &n,
      frame("GPGSV,3,1,09,01,10,010,20,02,20,020,30,03,30,030,25,04,40,040,22"),
      10);
  /* message 2 lost */
  feed_at(&n, frame("GPGSV,3,3,09,09,50,090,35"), 10);
  assert(n.sat_count == 5u && n.gsv_groups == 0u);
  feed_at(
      &n,
      frame("GPGSV,3,1,09,01,10,010,20,02,20,020,30,03,30,030,25,04,40,040,22"),
      20);
  feed_at(
      &n,
      frame("GPGSV,3,2,09,05,10,010,20,06,20,020,30,07,30,030,25,08,40,040,22"),
      20);
  feed_at(&n, frame("GPGSV,3,3,09,09,50,090,35"), 20);
  assert(n.sat_count == 9u && n.gsv_groups == 1u);
  /* Invalid group header is ignored. */
  feed_at(&n, frame("GPGSV,2,3,09,10,10,010,20"), 30);
  assert(!find_sat(n, NMEA_SYS_GPS, 10));
  /* Empty elevation / azimuth / snr and no sats at all. */
  feed_at(&n, frame("GLGSV,1,1,01,70,,,"), 30);
  const nmea_sat_t *s = find_sat(n, NMEA_SYS_GLONASS, 6);
  assert(s && s->el == -1 && s->az == -1 && s->snr == -1);
  assert(feed_at(&n, frame("GLGSV,1,1,00"), 30) == NMEA_GOT_GSV);
  /* Multi-signal 4.1 GSV: strongest SNR kept within a timestamp. */
  nmea_init(&n);
  feed_at(&n, frame("GAGSV,1,1,01,02,33,044,28,7"), 100);
  feed_at(&n, frame("GAGSV,1,1,01,02,33,044,20,1"), 100);
  assert(find_sat(n, NMEA_SYS_GALILEO, 2)->snr == 28);
  feed_at(&n, frame("GAGSV,1,1,01,02,33,044,20,7"), 1100);
  assert(find_sat(n, NMEA_SYS_GALILEO, 2)->snr == 20);
  /* Table overflow recycles the stalest entry. */
  nmea_init(&n);
  char body[64];
  for (unsigned i = 1; i <= NMEA_MAX_SATS + 3u; i++) {
    std::snprintf(body, sizeof(body), "GPGSV,1,1,01,%02u,10,010,20", i);
    feed_at(&n, frame(body), 100 + i);
  }
  assert(n.sat_count == NMEA_MAX_SATS && !find_sat(n, NMEA_SYS_GPS, 1) &&
         find_sat(n, NMEA_SYS_GPS, NMEA_MAX_SATS + 3u));
  /* Names. */
  assert(!std::strcmp(nmea_sys_name(NMEA_SYS_GPS), "GP") &&
         !std::strcmp(nmea_sys_name(NMEA_SYS_GLONASS), "GL") &&
         !std::strcmp(nmea_sys_name(NMEA_SYS_GALILEO), "GA") &&
         !std::strcmp(nmea_sys_name(NMEA_SYS_BEIDOU), "BD") &&
         !std::strcmp(nmea_sys_name(NMEA_SYS_QZSS), "QZ") &&
         !std::strcmp(nmea_sys_name(NMEA_SYS_UNKNOWN), "??"));
  assert(nmea_sys_from_talker("BD") == NMEA_SYS_BEIDOU &&
         nmea_sys_from_talker("GB") == NMEA_SYS_BEIDOU &&
         nmea_sys_from_talker("QZ") == NMEA_SYS_QZSS &&
         nmea_sys_from_talker("GQ") == NMEA_SYS_QZSS &&
         nmea_sys_from_talker("GN") == NMEA_SYS_UNKNOWN);
}

} // namespace

int main() {
  nmea_t n;
  nmea_init(&n);

  /* Valid RMC (NMEA 4.1 with mode / nav status), GN talker. */
  assert(
      feed(
          &n,
          frame(
              "GNRMC,123519.00,A,4807.038,N,01131.000,E,0.02,,151126,,,A,V")) ==
      NMEA_GOT_RMC);
  assert(n.rmc_valid && n.time_valid);
  /* 2026-11-15 12:35:19 UTC */
  assert(n.unix_s == 1763210119u + 365u * 86400u);

  /* Void status. */
  feed(&n, frame("GPRMC,123520.00,V,,,,,,,151126,,,N"));
  assert(!n.rmc_valid);
  assert(n.time_valid); /* time can still be known without a fix */

  /* 'A' status but mode 'N' (not valid) → invalid. */
  feed(&n, frame("GPRMC,123521.00,A,4807.038,N,01131.000,E,0.0,,151126,,,N"));
  assert(!n.rmc_valid);

  /* Old NMEA 2.0 RMC without mode field. */
  feed(&n, frame("GPRMC,000000,A,4807.038,N,01131.000,E,0.0,0.0,010120,,"));
  assert(n.rmc_valid && n.time_valid && n.unix_s == 1577836800u);

  /* Week-rollover style date (1999) rejected as time. */
  feed(&n, frame("GPRMC,000000,A,4807.038,N,01131.000,E,0.0,0.0,010199,,"));
  assert(!n.time_valid);

  /* GGA. */
  assert(feed(&n, frame("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,"
                        "46.9,M,,")) == NMEA_GOT_GGA);
  assert(n.gga_quality == 1u && n.sats == 8u && n.hdop > 0.89f &&
         n.hdop < 0.91f);
  feed(&n, frame("GNGGA,123520,,,,,0,00,99.99,,,,,,"));
  assert(n.gga_quality == 0u && n.sats == 0u && n.hdop > 99.9f);

  /* Bad checksum / no checksum / overlong are counted, state untouched. */
  const uint32_t errs = n.checksum_errors;
  assert(feed(&n, "$GPGGA,1,2,3,4,5,1,09,1.0,,,,,,*00\r\n") == 0u);
  assert(feed(&n, "$GPGGA,1,2,3,4,5,1,09,1.0,,,,,,\r\n") == 0u);
  char longline[200];
  std::memset(longline, 'A', sizeof(longline));
  longline[0] = '$';
  longline[sizeof(longline) - 1] = '\0';
  feed(&n, longline);
  assert(n.checksum_errors == errs + 3u);
  assert(n.sats == 0u);

  /* Garbage between sentences and other sentence types are ignored. */
  assert(feed(&n, "xx\r\n") == 0u);
  assert(feed(&n, frame("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,"
                        "13,06,292,00")) == NMEA_GOT_GSV);

  test_position();
  test_gsa_gsv();
  test_aging_and_groups();

  std::printf("sizeof(nmea_t) = %zu\n", sizeof(nmea_t));
  std::printf("test_nmea: OK\n");
  return 0;
}
