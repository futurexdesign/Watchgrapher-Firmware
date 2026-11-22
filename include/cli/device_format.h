/**
 * @file device_format.h
 * @brief Pure parsing and formatting helpers for the {@code device},
 *        {@code run}, {@code dsp beats}, {@code dsp graph} and
 *        {@code timebase gps|log|logdelta} CLI commands (no Arduino, no
 *        Serial), so they can be host-tested.
 *
 * The CLI ({@code cli_commands.cpp}) owns all printing and every call into
 * {@code control_api}; this header only turns user tokens into values and
 * values into one printed line each. Every formatter writes into a caller
 * buffer and returns it, truncating safely.
 *
 * Host test: tests/test_device_cli_format.cpp.
 *
 * @see cli_commands.cpp, control_api.h, docs/HID_CONTROL_PLANE.md
 */

#pragma once

#include "audio_source.h"
#include "device_mode.h"
#include "dsp_blocks.h"
#include "dsp_types.h"
#include "timebase.h"
#include "timebase_log.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace Cli {

/** Beats {@code dsp beats} lists when no count is given. */
inline constexpr uint32_t kBeatsDefault = 8u;
/** Largest count {@code dsp beats} accepts (same as HID caps.beatsMax). */
inline constexpr uint32_t kBeatsCliMax = 16u;

/**
 * Parses the count of {@code dsp beats [n]}.
 *
 * @param s Token (null or empty is rejected).
 * @param out Receives 1..{@link kBeatsCliMax} on success only.
 * @return {@code true} if {@code s} is a whole number in range.
 */
[[nodiscard]] inline bool parseBeatsCount(const char *s,
                                          uint32_t &out) noexcept {
  if (s == nullptr || *s == '\0' || *s == '-' || *s == '+') {
    return false;
  }
  char *end = nullptr;
  const unsigned long v = std::strtoul(s, &end, 10);
  if (*end != '\0' || v < 1ul || v > kBeatsCliMax) {
    return false;
  }
  out = static_cast<uint32_t>(v);
  return true;
}

/**
 * @param s Stream state.
 * @return Its stable name ({@code unconfigured}, {@code idle},
 *         {@code prefill}, {@code active}, {@code suspended},
 *         {@code error}); the same names as the HID {@code streamState}.
 */
[[nodiscard]] inline const char *streamStateText(stream_state_t s) noexcept {
  switch (s) {
  case STREAM_STATE_USB_UNCONFIGURED:
    return "unconfigured";
  case STREAM_STATE_USB_CONFIGURED_IDLE:
    return "idle";
  case STREAM_STATE_STREAM_PREFILL:
    return "prefill";
  case STREAM_STATE_STREAM_ACTIVE:
    return "active";
  case STREAM_STATE_USB_SUSPENDED:
    return "suspended";
  case STREAM_STATE_STREAM_ERROR:
    return "error";
  }
  return "unknown";
}

/** @return {@code "yes"} / {@code "no"}. */
[[nodiscard]] constexpr const char *yesNo(bool b) noexcept {
  return b ? "yes" : "no";
}

/**
 * Formats the device status line, e.g.
 * {@code device mode=standalone running=yes run_requested=yes
 * usb_streaming=no stream=idle saved=yes}. {@code mode} is passed as text
 * because {@code device_mode_name} lives in the firmware.
 *
 * @param s Status.
 * @param mode_name Name of {@code s.mode} ({@code usbMic} / {@code
 *        standalone}).
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatDeviceStatus(const device_status_t &s,
                                      const char *mode_name, char *buf,
                                      std::size_t n) noexcept {
  std::snprintf(buf, n,
                "device mode=%s running=%s run_requested=%s usb_streaming=%s "
                "stream=%s saved=%s",
                mode_name, yesNo(s.running), yesNo(s.run_requested),
                yesNo(s.usb_streaming), streamStateText(s.stream_state),
                yesNo(s.saved));
  return buf;
}

/** Column header of the {@code dsp beats} table (matches
 * {@link formatBeatRow}). */
inline constexpr const char *kBeatsHeader =
    "     seq grid       k p      u_us    amp   lift status";

/**
 * Formats one row of the {@code dsp beats} table:
 * seq, grid id, k, parity (0 / 1), residual u (us), amplitude (deg, or
 * {@code n/a} when invalid), lift (ms) and status.
 *
 * @param b Beat.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatBeatRow(const dsp_beat_t &b, char *buf,
                                 std::size_t n) noexcept {
  const char *status = b.status == DSP_BEAT_ACCEPTED  ? "accepted"
                       : b.status == DSP_BEAT_STARTED ? "started"
                                                      : "rejected";
  char amp[12];
  if (b.amplitude_valid && std::isfinite(b.amplitude_deg)) {
    std::snprintf(amp, sizeof(amp), "%.1f",
                  static_cast<double>(b.amplitude_deg));
  } else {
    std::snprintf(amp, sizeof(amp), "n/a");
  }
  std::snprintf(
      buf, n, "%8lu %4lu %7ld %u %9.1f %6s %6.3f %s",
      static_cast<unsigned long>(b.seq), static_cast<unsigned long>(b.grid_id),
      static_cast<long>(b.k), static_cast<unsigned>(b.parity & 1u),
      std::isfinite(b.u_us) ? static_cast<double>(b.u_us) : 0.0, amp,
      std::isfinite(b.lift_ms) ? static_cast<double>(b.lift_ms) : 0.0, status);
  return buf;
}

/**
 * Formats UNIX seconds as {@code YYYY-MM-DD hh:mm:ssZ} (UTC, proleptic
 * Gregorian), or {@code n/a} for 0 (unknown).
 *
 * @param unix UTC seconds.
 * @param buf Destination (at least 21 bytes).
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatUtc(uint32_t unix, char *buf, std::size_t n) noexcept {
  if (unix == 0u) {
    std::snprintf(buf, n, "n/a");
    return buf;
  }
  /* Days since 1970-01-01 to a civil date (H. Hinnant's algorithm). */
  const int64_t days = static_cast<int64_t>(unix / 86400u);
  const uint32_t rem = unix % 86400u;
  const int64_t z = days + 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int64_t doe = z - era * 146097;
  const int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t y = yoe + era * 400;
  const int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int64_t mp = (5 * doy + 2) / 153;
  const int64_t d = doy - (153 * mp + 2) / 5 + 1;
  const int64_t m = mp < 10 ? mp + 3 : mp - 9;
  if (m <= 2) {
    y++;
  }
  /* y is 1970..2106 for a uint32_t input; m and d are calendar-bounded. The
   * no-op moduli let gcc's -Wformat-truncation prove the 20-char bound. */
  std::snprintf(buf, n, "%04u-%02u-%02u %02u:%02u:%02uZ",
                static_cast<unsigned>(y) % 10000u,
                static_cast<unsigned>(m) % 100u,
                static_cast<unsigned>(d) % 100u,
                static_cast<unsigned>(rem / 3600u) % 100u,
                static_cast<unsigned>(rem % 3600u / 60u),
                static_cast<unsigned>(rem % 60u));
  return buf;
}

/**
 * Formats the first GPS line: fix, quality, satellites used, UTC and the
 * age of the last RMC.
 *
 * @param g Snapshot.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatGpsFix(const timebase_gps_t &g, char *buf,
                                std::size_t n) noexcept {
  char utc[24];
  char age[16];
  if (g.age_ms < 0) {
    std::snprintf(age, sizeof(age), "never");
  } else {
    std::snprintf(age, sizeof(age), "%ldms", static_cast<long>(g.age_ms));
  }
  std::snprintf(buf, n,
                "gps fix=%s quality=%u sats_used=%u utc=%s (unix %lu) "
                "last_rmc=%s",
                yesNo(g.fix), static_cast<unsigned>(g.quality),
                static_cast<unsigned>(g.sats_used),
                formatUtc(g.utc_unix, utc, sizeof(utc)),
                static_cast<unsigned long>(g.utc_unix), age);
  return buf;
}

/**
 * Formats the dilution-of-precision line; an unknown value (0) prints
 * {@code n/a}.
 *
 * @param g Snapshot.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatGpsDop(const timebase_gps_t &g, char *buf,
                                std::size_t n) noexcept {
  char v[3][12];
  const float d[3] = {g.hdop, g.pdop, g.vdop};
  for (int i = 0; i < 3; i++) {
    if (std::isfinite(d[i]) && d[i] > 0.f) {
      std::snprintf(v[i], sizeof(v[i]), "%.2f", static_cast<double>(d[i]));
    } else {
      std::snprintf(v[i], sizeof(v[i]), "n/a");
    }
  }
  std::snprintf(buf, n, "gps hdop=%s pdop=%s vdop=%s", v[0], v[1], v[2]);
  return buf;
}

/**
 * Formats the position line: latitude / longitude (7 decimals, signed
 * degrees) and altitude, or {@code none} when no valid position.
 *
 * @param g Snapshot.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatGpsPos(const timebase_gps_t &g, char *buf,
                                std::size_t n) noexcept {
  if (!g.pos_valid) {
    std::snprintf(buf, n, "gps pos=none");
  } else {
    std::snprintf(buf, n, "gps pos lat=%.7f lon=%.7f alt_m=%.1f", g.lat, g.lon,
                  static_cast<double>(g.alt_m));
  }
  return buf;
}

/** Column header of the satellite table (matches {@link formatSatRow}). */
inline constexpr const char *kSatHeader = "  sys  prn   el    az  snr used";

/**
 * @param sys An {@code nmea_sys_t} value.
 * @return {@code GP}, {@code GL}, {@code GA}, {@code BD}, {@code QZ} or
 *         {@code ??}.
 */
[[nodiscard]] inline const char *gpsSysText(uint8_t sys) noexcept {
  static constexpr const char *kNames[] = {"??", "GP", "GL", "GA", "BD", "QZ"};
  return sys < sizeof(kNames) / sizeof(kNames[0]) ? kNames[sys] : "??";
}

/**
 * Formats one satellite row: system, PRN, elevation, azimuth, SNR and
 * whether it is in the fix. Unknown values (-1) print {@code -}; an SNR of
 * -1 means not tracked.
 *
 * @param s Satellite.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatSatRow(const timebase_gps_sat_t &s, char *buf,
                                std::size_t n) noexcept {
  char el[8], az[8], snr[8];
  auto field = [](char *out, std::size_t len, int v) {
    if (v < 0) {
      std::snprintf(out, len, "-");
    } else {
      std::snprintf(out, len, "%d", v);
    }
  };
  field(el, sizeof(el), s.el);
  field(az, sizeof(az), s.az);
  field(snr, sizeof(snr), s.snr);
  std::snprintf(buf, n, "  %-3s %4u %4s %5s %4s %s", gpsSysText(s.sys),
                static_cast<unsigned>(s.prn), el, az, snr,
                s.used ? "yes" : "no");
  return buf;
}

/** Column header of the timebase-log table (matches
 * {@link formatLogRow}). */
inline constexpr const char *kLogHeader =
    "   # date (UTC)                 ppm source  duration_s reason";

/**
 * Formats one timebase-log row (oldest = 0).
 *
 * @param index Row number.
 * @param e Entry.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatLogRow(uint32_t index, const timebase_log_entry_t &e,
                                char *buf, std::size_t n) noexcept {
  char date[24];
  const char *src = e.source == TIMEBASE_SRC_PPS   ? "pps"
                    : e.source == TIMEBASE_SRC_NTP ? "ntp"
                                                   : "manual";
  std::snprintf(buf, n, "%4lu %-20s %9.3f %-6s %11lu %s",
                static_cast<unsigned long>(index),
                formatUtc(e.date_unix, date, sizeof(date)),
                std::isfinite(e.ppm) ? static_cast<double>(e.ppm) : 0.0, src,
                static_cast<unsigned long>(e.duration_s),
                timebase_log_reason_name(e.reason));
  return buf;
}

/**
 * Parses the {@code timebase logdelta} argument. Range is
 * {@link timebase_log_delta_valid} (0.05..10 ppm).
 *
 * @param s Token.
 * @param out Receives the value on success only.
 * @return {@code true} if {@code s} is a finite number in range.
 */
[[nodiscard]] inline bool parseLogDelta(const char *s, float &out) noexcept {
  if (s == nullptr || *s == '\0') {
    return false;
  }
  char *end = nullptr;
  const float v = std::strtof(s, &end);
  if (end == s || *end != '\0' || !std::isfinite(v) ||
      !timebase_log_delta_valid(v)) {
    return false;
  }
  out = v;
  return true;
}

/**
 * Formats a pipeline node as {@code <key> <- <input>, <input>} (or just the
 * key for a node without inputs), with the order, e.g.
 * {@code 4 softGate <- firBank}.
 *
 * @param node Node from {@code dsp_pipeline_nodes}.
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *formatPipelineNode(const dsp_pipeline_node_t &node,
                                      char *buf, std::size_t n) noexcept {
  int w = std::snprintf(buf, n, "%u %s", static_cast<unsigned>(node.order),
                        node.key);
  for (uint8_t i = 0;
       i < node.n_inputs && w > 0 && static_cast<std::size_t>(w) < n; i++) {
    w += std::snprintf(buf + w, n - static_cast<std::size_t>(w), "%s%s",
                       i == 0 ? " <- " : ", ", node.inputs[i]);
  }
  return buf;
}

} // namespace Cli
