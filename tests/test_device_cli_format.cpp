/**
 * @file test_device_cli_format.cpp
 * @brief Host test for the device / run / beats / gps / timebase-log /
 *        graph CLI formatters (include/cli/device_format.h).
 *
 * Header-only, links only dsp_blocks.cpp for the pipeline table. Run:
 *   c++ -std=c++17 -Wall -Wextra -Werror -Iinclude \
 *     tests/test_device_cli_format.cpp src/dsp_blocks.cpp -o /tmp/t && /tmp/t
 */

#include "cli/device_format.h"

#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

int g_fail = 0;

/** Compares {@code got} with {@code want} and records a failure. */
void eq(const char *got, const char *want) {
  if (std::strcmp(got, want) != 0) {
    std::printf("FAIL: got '%s' want '%s'\n", got, want);
    g_fail++;
  }
}

/** Records a failure when {@code ok} is false. */
void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

} // namespace

int main() {
  char buf[256];

  // Beat count.
  uint32_t n = 99;
  check(Cli::parseBeatsCount("1", n) && n == 1u, "count 1");
  check(Cli::parseBeatsCount("16", n) && n == 16u, "count 16");
  n = 99;
  check(!Cli::parseBeatsCount("0", n) && n == 99u, "count 0 rejected");
  check(!Cli::parseBeatsCount("17", n), "count 17 rejected");
  check(!Cli::parseBeatsCount("-3", n), "negative rejected");
  check(!Cli::parseBeatsCount("4x", n), "junk rejected");
  check(!Cli::parseBeatsCount("", n), "empty rejected");
  check(!Cli::parseBeatsCount(nullptr, n), "null rejected");

  // Device status.
  device_status_t s{};
  s.mode = DEVICE_MODE_STANDALONE;
  s.running = true;
  s.run_requested = true;
  s.stream_state = STREAM_STATE_USB_CONFIGURED_IDLE;
  s.saved = true;
  eq(Cli::formatDeviceStatus(s, "standalone", buf, sizeof(buf)),
     "device mode=standalone running=yes run_requested=yes usb_streaming=no "
     "stream=idle saved=yes");
  s.stream_state = STREAM_STATE_STREAM_ERROR;
  check(std::strstr(Cli::formatDeviceStatus(s, "usbMic", buf, sizeof(buf)),
                    "stream=error") != nullptr,
        "stream error text");

  // Beat rows.
  dsp_beat_t b{};
  b.seq = 1234;
  b.grid_id = 2;
  b.k = -17;
  b.parity = 1;
  b.u_us = -12.34f;
  b.amplitude_deg = 287.46f;
  b.amplitude_valid = true;
  b.lift_ms = 2.3456f;
  b.status = DSP_BEAT_STARTED;
  eq(Cli::formatBeatRow(b, buf, sizeof(buf)),
     "    1234    2     -17 1     -12.3  287.5  2.346 started");
  b.amplitude_valid = false;
  b.status = DSP_BEAT_REJECTED;
  b.u_us = NAN;
  check(std::strstr(Cli::formatBeatRow(b, buf, sizeof(buf)), " n/a ") !=
                nullptr &&
            std::strstr(buf, "nan") == nullptr &&
            std::strstr(buf, "rejected") != nullptr,
        "invalid amplitude / non-finite residual");
  check(std::strstr(Cli::kBeatsHeader, "status") - Cli::kBeatsHeader ==
            std::strstr(Cli::formatBeatRow(b, buf, sizeof(buf)), "rejected") -
                buf,
        "beats header aligns with a row");

  // UTC.
  char t[32];
  eq(Cli::formatUtc(0, t, sizeof(t)), "n/a");
  eq(Cli::formatUtc(1, t, sizeof(t)), "1970-01-01 00:00:01Z");
  eq(Cli::formatUtc(951782400u, t, sizeof(t)), "2000-02-29 00:00:00Z");
  eq(Cli::formatUtc(1700000000u, t, sizeof(t)), "2023-11-14 22:13:20Z");
  eq(Cli::formatUtc(4294967295u, t, sizeof(t)), "2106-02-07 06:28:15Z");

  // GPS lines.
  timebase_gps_t g{};
  g.fix = true;
  g.quality = 1;
  g.utc_unix = 1700000000u;
  g.age_ms = 240;
  g.sats_used = 11;
  g.hdop = 0.9f;
  g.pdop = 1.5f;
  g.vdop = 0.f;
  g.lat = 47.6062095;
  g.lon = -122.3320708;
  g.alt_m = 56.5f;
  g.pos_valid = true;
  eq(Cli::formatGpsFix(g, buf, sizeof(buf)),
     "gps fix=yes quality=1 sats_used=11 utc=2023-11-14 22:13:20Z (unix "
     "1700000000) last_rmc=240ms");
  g.age_ms = -1;
  g.utc_unix = 0;
  check(std::strstr(Cli::formatGpsFix(g, buf, sizeof(buf)),
                    "utc=n/a (unix 0) last_rmc=never") != nullptr,
        "gps never");
  eq(Cli::formatGpsDop(g, buf, sizeof(buf)),
     "gps hdop=0.90 pdop=1.50 vdop=n/a");
  eq(Cli::formatGpsPos(g, buf, sizeof(buf)),
     "gps pos lat=47.6062095 lon=-122.3320708 alt_m=56.5");
  g.pos_valid = false;
  eq(Cli::formatGpsPos(g, buf, sizeof(buf)), "gps pos=none");

  timebase_gps_sat_t sat{};
  sat.sys = 3;
  sat.prn = 12;
  sat.el = 45;
  sat.az = 123;
  sat.snr = 41;
  sat.used = true;
  eq(Cli::formatSatRow(sat, buf, sizeof(buf)),
     "  GA    12   45   123   41 yes");
  check(std::strstr(Cli::kSatHeader, "used") - Cli::kSatHeader ==
            std::strstr(Cli::formatSatRow(sat, buf, sizeof(buf)), "yes") - buf,
        "sat header aligns with a row");
  sat.sys = 9;
  sat.el = -1;
  sat.az = -1;
  sat.snr = -1;
  sat.used = false;
  eq(Cli::formatSatRow(sat, buf, sizeof(buf)), "  ??    12    -     -    - no");
  eq(Cli::gpsSysText(1), "GP");
  eq(Cli::gpsSysText(2), "GL");
  eq(Cli::gpsSysText(4), "BD");
  eq(Cli::gpsSysText(5), "QZ");

  // Timebase log rows.
  timebase_log_entry_t e{};
  e.date_unix = 1700000000u;
  e.ppm = -12.3456f;
  e.source = TIMEBASE_SRC_PPS;
  e.duration_s = 3600;
  e.reason = TIMEBASE_LOG_REASON_DELTA;
  eq(Cli::formatLogRow(3, e, buf, sizeof(buf)),
     "   3 2023-11-14 22:13:20Z   -12.346 pps           3600 delta");
  check(std::strstr(Cli::kLogHeader, "reason") - Cli::kLogHeader ==
            std::strstr(buf, "delta") - buf,
        "log header aligns with a row");
  e.source = TIMEBASE_SRC_NTP;
  e.reason = TIMEBASE_LOG_REASON_SAVE;
  check(std::strstr(Cli::formatLogRow(0, e, buf, sizeof(buf)), " ntp ") !=
                nullptr &&
            std::strstr(buf, "save") != nullptr,
        "ntp / save row");
  e.source = TIMEBASE_SRC_MANUAL;
  e.reason = TIMEBASE_LOG_REASON_MONTHLY;
  check(std::strstr(Cli::formatLogRow(0, e, buf, sizeof(buf)), " manual ") !=
            nullptr,
        "manual row");

  // logdelta parsing: 0.05..10.
  float v = 7.f;
  check(Cli::parseLogDelta("0.2", v) && v == 0.2f, "logdelta 0.2");
  check(Cli::parseLogDelta("0.05", v) && v == 0.05f, "logdelta min");
  check(Cli::parseLogDelta("10", v) && v == 10.f, "logdelta max");
  v = 7.f;
  check(!Cli::parseLogDelta("0.04", v) && v == 7.f, "logdelta too small");
  check(!Cli::parseLogDelta("10.1", v), "logdelta too large");
  check(!Cli::parseLogDelta("nan", v), "logdelta nan");
  check(!Cli::parseLogDelta("0.2x", v), "logdelta junk");
  check(!Cli::parseLogDelta("", v), "logdelta empty");
  check(!Cli::parseLogDelta(nullptr, v), "logdelta null");

  // Pipeline nodes from the real table.
  uint32_t count = 0;
  const auto *nodes = dsp_pipeline_nodes(&count);
  bool saw_level = false, saw_input = false, saw_gate = false;
  for (uint32_t i = 0; i < count; i++) {
    Cli::formatPipelineNode(nodes[i], buf, sizeof(buf));
    if (std::strcmp(nodes[i].key, "levelMeter") == 0) {
      eq(buf, "0 levelMeter <- input, limiter");
      saw_level = true;
    } else if (std::strcmp(nodes[i].key, "input") == 0) {
      eq(buf, "0 input");
      saw_input = true;
    } else if (std::strcmp(nodes[i].key, "softGate") == 0) {
      eq(buf, "4 softGate <- firBank");
      saw_gate = true;
    }
  }
  check(saw_level && saw_input && saw_gate, "pipeline nodes found");
  Cli::formatPipelineNode(nodes[1], buf, 6); // truncation is safe
  check(std::strlen(buf) < 6, "truncated");

  std::printf("%s\n", g_fail == 0 ? "ok" : "FAILED");
  return g_fail == 0 ? 0 : 1;
}
