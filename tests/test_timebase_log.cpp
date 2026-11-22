/**
 * @file test_timebase_log.cpp
 * @brief Host test for the timebase log ring, flash record and PPS policy.
 *
 * Build and run:
 *   c++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/test_timebase_log.cpp
 * -o /tmp/t && /tmp/t
 */

#include "timebase_log.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

int g_fail = 0;

#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);                  \
      g_fail++;                                                                \
    }                                                                          \
  } while (0)

constexpr uint32_t kDay = 86400u;
constexpr uint32_t kT0 = 1700000000u;

timebase_log_entry_t entry(uint32_t date, float ppm, uint8_t src = 3,
                           uint8_t reason = TIMEBASE_LOG_REASON_DELTA,
                           uint32_t dur = 700) {
  return {date, ppm, src, reason, 0, dur};
}

timebase_log_pps_input_t good(uint32_t now, float ppm) {
  return {true, 64, 600, ppm, now};
}

void test_ring() {
  timebase_log_t log;
  timebase_log_init(&log);
  CHECK(log.count == 0 && timebase_log_last(&log) == nullptr);
  CHECK(timebase_log_get(&log, 0) == nullptr);
  for (uint32_t i = 0; i < 5; i++) {
    auto e = entry(kT0 + i, static_cast<float>(i));
    timebase_log_append(&log, &e);
  }
  CHECK(log.count == 5);
  CHECK(timebase_log_get(&log, 0)->date_unix == kT0);
  CHECK(timebase_log_last(&log)->date_unix == kT0 + 4);
  CHECK(timebase_log_get(&log, 5) == nullptr);
  /* Wrap: 30 total, keep last 24 (6..29). */
  for (uint32_t i = 5; i < 30; i++) {
    auto e = entry(kT0 + i, static_cast<float>(i));
    e.reserved = 0xBEEF;
    timebase_log_append(&log, &e);
  }
  CHECK(log.count == TIMEBASE_LOG_CAP);
  for (uint32_t i = 0; i < TIMEBASE_LOG_CAP; i++) {
    CHECK(timebase_log_get(&log, i)->date_unix == kT0 + 6 + i);
    CHECK(timebase_log_get(&log, i)->reserved == 0);
  }
  CHECK(timebase_log_last(&log)->date_unix == kT0 + 29);
  CHECK(timebase_log_get(&log, TIMEBASE_LOG_CAP) == nullptr);
  timebase_log_clear(&log);
  CHECK(log.count == 0 && log.head == 0);
}

void test_record() {
  timebase_log_t log;
  timebase_log_init(&log);
  for (uint32_t i = 0; i < 30; i++) {
    auto e = entry(kT0 + i * 10, 0.25f * static_cast<float>(i), 2, 1, i);
    timebase_log_append(&log, &e);
  }
  timebase_log_record_t rec;
  timebase_log_encode(&log, &rec);
  CHECK(rec.magic == TIMEBASE_LOG_MAGIC);
  unsigned char raw[sizeof(rec)];
  std::memcpy(raw, &rec, sizeof(rec));
  CHECK(raw[0] == 'T' && raw[1] == 'B' && raw[2] == 'L' && raw[3] == '1');

  timebase_log_t out;
  timebase_log_init(&out);
  CHECK(timebase_log_decode(&rec, &out));
  CHECK(std::memcmp(&out, &log, sizeof(log)) == 0);

  /* CRC matches the standard IEEE check value. */
  CHECK(timebase_log_crc32("123456789", 9) == 0xCBF43926u);

  auto bad = [&](auto mutate) {
    timebase_log_record_t r = rec;
    mutate(r);
    timebase_log_t o;
    timebase_log_init(&o);
    bool ok = timebase_log_decode(&r, &o);
    CHECK(!ok);
    CHECK(o.count == 0);
  };
  bad([](timebase_log_record_t &r) { r.magic ^= 1; });
  bad([](timebase_log_record_t &r) { r.version++; });
  bad([](timebase_log_record_t &r) { r.size++; });
  bad([](timebase_log_record_t &r) { r.crc ^= 1; });
  bad([](timebase_log_record_t &r) { r.log.entries[3].ppm += 1.f; });
  /* Out-of-range count/head with a self-consistent CRC. */
  bad([](timebase_log_record_t &r) {
    r.log.count = 25;
    r.crc = timebase_log_crc32(&r, offsetof(timebase_log_record_t, crc));
  });
  bad([](timebase_log_record_t &r) {
    r.log.head = 24;
    r.crc = timebase_log_crc32(&r, offsetof(timebase_log_record_t, crc));
  });
  /* Erased flash. */
  timebase_log_record_t erased;
  std::memset(&erased, 0xFF, sizeof(erased));
  CHECK(!timebase_log_decode(&erased, &out));
  /* Empty log round-trips. */
  timebase_log_t empty;
  timebase_log_init(&empty);
  timebase_log_encode(&empty, &rec);
  CHECK(timebase_log_decode(&rec, &out) && out.count == 0);
}

void test_names_and_validation() {
  CHECK(std::strcmp(timebase_log_reason_name(0), "save") == 0);
  CHECK(std::strcmp(timebase_log_reason_name(1), "delta") == 0);
  CHECK(std::strcmp(timebase_log_reason_name(2), "monthly") == 0);
  CHECK(std::strcmp(timebase_log_reason_name(9), "unknown") == 0);
  CHECK(timebase_log_delta_valid(0.05f) && timebase_log_delta_valid(10.f));
  CHECK(timebase_log_delta_valid(0.5f));
  CHECK(!timebase_log_delta_valid(0.049f) && !timebase_log_delta_valid(10.01f));
  CHECK(!timebase_log_delta_valid(NAN) && !timebase_log_delta_valid(INFINITY));
  CHECK(!timebase_log_delta_valid(-1.f) && !timebase_log_delta_valid(0.f));
}

void test_policy() {
  timebase_log_t log;
  timebase_log_init(&log);
  const float d = 0.5f;
  const uint32_t now = kT0 + 100 * kDay;

  /* Candidate gates. */
  auto in = good(now, 1.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_DELTA);
  in = good(now, 1.f);
  in.pps_locked = false;
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in = good(now, 1.f);
  in.pps_window = 63;
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in.pps_window = 65;
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in = good(now, 1.f);
  in.locked_since_s = 599;
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in.locked_since_s = 600;
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_DELTA);
  in = good(0, 1.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);

  /* Non-empty log, last at kT0 with ppm 1.0. */
  auto e = entry(kT0, 1.f);
  timebase_log_append(&log, &e);

  /* Delta exactly equal, age exactly one day. */
  in = good(kT0 + kDay, 1.5f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_DELTA);
  in = good(kT0 + kDay, 0.5f); /* negative direction, exactly equal */
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_DELTA);
  /* Just below delta. */
  in = good(kT0 + kDay, 1.49f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  /* Age one second short. */
  in = good(kT0 + kDay - 1, 5.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  /* Monthly boundary. */
  in = good(kT0 + 30 * kDay - 1, 1.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in = good(kT0 + 30 * kDay, 1.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_MONTHLY);
  /* Monthly wins over delta. */
  in = good(kT0 + 31 * kDay, 9.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_MONTHLY);
  /* Gates still apply with a non-empty log. */
  in = good(kT0 + 31 * kDay, 9.f);
  in.locked_since_s = 599;
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);

  /* Clock backwards: small step -> NONE even with a big delta. */
  in = good(kT0 - 3600, 9.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in = good(kT0 - kDay, 9.f); /* exactly one day ahead: not "more than" */
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  /* Stored date > 1 day in the future: DELTA only if ppm moved. */
  in = good(kT0 - kDay - 1, 9.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_DELTA);
  in = good(kT0 - kDay - 1, 1.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  /* Same-second repeat: age 0 -> NONE. */
  in = good(kT0, 9.f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);

  /* Compares against the last entry of any source (a "save" at t1). */
  auto s = entry(kT0 + 40 * kDay, 3.f, 1, TIMEBASE_LOG_REASON_SAVE, 0);
  timebase_log_append(&log, &s);
  in = good(kT0 + 40 * kDay + kDay, 3.1f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_NONE);
  in = good(kT0 + 40 * kDay + kDay, 3.5f);
  CHECK(timebase_log_pps_decision(&log, &in, d) == TIMEBASE_LOG_DELTA);
}

} // namespace

int main() {
  test_ring();
  test_record();
  test_names_and_validation();
  test_policy();
  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("test_timebase_log: all passed\n");
  return 0;
}
