/**
 * @file quartz_format.h
 * @brief Pure parsing and formatting helpers for the {@code quartz} CLI
 *        command (no Arduino, no Serial), so they can be host-tested.
 *
 * The CLI ({@code cli_commands.cpp}) owns all printing; this header only
 * turns user tokens into values and values into short strings. It does not
 * validate ranges: the engine does, and reports why through
 * {@code control_api_set_quartz_param_ex}. Allowed values shown to the user
 * come from the engine and library constants ({@code quartz_engine.h},
 * {@code quartz_types.h}) so the CLI cannot drift from what the engine
 * accepts.
 *
 * Host test: tests/test_quartz_cli_format.cpp.
 *
 * @see cli_commands.cpp, quartz/quartz_engine.h, dsp_quartz_types.h,
 *      docs/QUARTZ_TIMING.md
 */

#pragma once

#include "dsp_quartz_types.h"
#include "quartz/quartz_engine.h"
#include "quartz/quartz_types.h"

#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace Cli {

/** Default number of pulses {@code quartz pulses} lists. */
inline constexpr uint32_t kQuartzPulsesDefault = 8u;

/**
 * Case-insensitive ASCII equality. A local copy of
 * {@code Cli::equalsIgnoreCase} because that one lives in cli_core.cpp,
 * which needs Arduino; this header must build on the host.
 *
 * @param a First string (may be null).
 * @param b Second string (may be null).
 * @return {@code true} if both are null or equal ignoring case.
 */
[[nodiscard]] inline bool quartzEqualsIgnoreCase(const char *a,
                                                 const char *b) noexcept {
  if (a == nullptr || b == nullptr) {
    return a == b;
  }
  for (; *a != '\0' && *b != '\0'; a++, b++) {
    if (std::tolower(static_cast<unsigned char>(*a)) !=
        std::tolower(static_cast<unsigned char>(*b))) {
      return false;
    }
  }
  return *a == *b;
}

/**
 * Parses a full-string finite decimal (no range check: the engine decides
 * what it accepts and says why it refuses).
 *
 * @param s Token (null or empty is rejected).
 * @param out Receives the value on success only.
 * @return {@code true} if {@code s} is a finite number.
 */
[[nodiscard]] inline bool quartzParseNumber(const char *s,
                                            float &out) noexcept {
  if (s == nullptr || *s == '\0') {
    return false;
  }
  char *end = nullptr;
  const float v = std::strtof(s, &end);
  if (end == s || *end != '\0' || !std::isfinite(v)) {
    return false;
  }
  out = v;
  return true;
}

/**
 * Parses a motor gate token: {@code auto} (any case) means "until two
 * inhibition events" ({@link quartz::engine::kGateUntilEvents}, also
 * accepted as {@code 0}); anything else must be a number of seconds, whose
 * range ({@link quartz::engine::kGateMinS} ..
 * {@link quartz::engine::kGateMaxS}) the engine checks.
 *
 * @param s Token.
 * @param out Receives the gate (s) on success only.
 * @return {@code true} for {@code auto} or a finite number.
 */
[[nodiscard]] inline bool quartzParseGate(const char *s, float &out) noexcept {
  if (quartzEqualsIgnoreCase(s, "auto")) {
    out = quartz::engine::kGateUntilEvents;
    return true;
  }
  return quartzParseNumber(s, out);
}

/**
 * The human message for a handoff §5 inhibition status: what the user may
 * conclude from the motor rate right now. The stable snake_case name
 * ({@code control_api_quartz_inhibition_name}) is printed next to it for
 * scripts; this text is for people.
 *
 * @param s Status.
 * @return A static message, never null.
 */
[[nodiscard]] inline const char *
quartzInhibitionMessage(dsp_quartz_inhibition_t s) noexcept {
  switch (s) {
  case DSP_QUARTZ_INHIB_NONE:
    return "no rate yet: waiting for 3 pulses";
  case DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE:
    return "no inhibition detected";
  case DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION:
    return "raw rate, may include inhibition: use a longer gate";
  case DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE:
    return "raw rate, inhibition seen once: need one more cycle";
  case DSP_QUARTZ_INHIB_CORRECTION_PENDING:
    return "raw rate, 2 inhibition events: corrected at the next pulse";
  case DSP_QUARTZ_INHIB_CORRECTED:
    return "corrected for inhibition";
  default:
    return "unknown";
  }
}

/**
 * Formats a gate value: {@code auto} for "until two events", else the
 * seconds without trailing zeros and with the unit ({@code 60s},
 * {@code 12.5s}), as the CLI prints it after {@code gate=}.
 *
 * @param gate_s Gate (s).
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *quartzFormatGate(float gate_s, char *buf,
                                    std::size_t n) noexcept {
  if (gate_s == quartz::engine::kGateUntilEvents) {
    std::snprintf(buf, n, "auto");
  } else {
    std::snprintf(buf, n, "%gs", static_cast<double>(gate_s));
  }
  return buf;
}

/**
 * Formats a frequency for display: a whole number without decimals,
 * otherwise two decimals (341.33 for the 1024/3 Hz fork, which is also the
 * value {@code quartz fork} accepts back).
 *
 * @param hz Frequency (Hz).
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *quartzFormatHz(double hz, char *buf,
                                  std::size_t n) noexcept {
  if (std::fabs(hz - std::round(hz)) < 0.005) {
    std::snprintf(buf, n, "%.0f", hz);
  } else {
    std::snprintf(buf, n, "%.2f", hz);
  }
  return buf;
}

/**
 * Formats the allowed tuning-fork frequencies from
 * {@link quartz::kForkFrequencies}, separated by {@code |}
 * ({@code 300|341.33|360|480|720}). Truncated safely if {@code n} is small.
 *
 * @param buf Destination.
 * @param n Capacity of {@code buf} (at least 1).
 * @return {@code buf}.
 */
inline const char *quartzFormatForkList(char *buf, std::size_t n) noexcept {
  std::size_t used = 0;
  buf[0] = '\0';
  for (std::size_t i = 0; i < quartz::kForkFrequencies.size() && used < n;
       i++) {
    char hz[16];
    quartzFormatHz(quartz::kForkFrequencies[i].hz(), hz, sizeof(hz));
    const int w = std::snprintf(buf + used, n - used, "%s%s", i ? "|" : "", hz);
    if (w < 0) {
      break;
    }
    used += static_cast<std::size_t>(w);
  }
  return buf;
}

/**
 * Formats a rate in s/d with three decimals (quartz rates are read to
 * milliseconds per day), or {@code n/a} when it is not valid.
 *
 * @param valid The value is meaningful.
 * @param rate_s_day Rate (s/d).
 * @param buf Destination.
 * @param n Capacity of {@code buf}.
 * @return {@code buf}.
 */
inline const char *quartzFormatRate(bool valid, float rate_s_day, char *buf,
                                    std::size_t n) noexcept {
  if (!valid || !std::isfinite(rate_s_day)) {
    std::snprintf(buf, n, "n/a");
  } else {
    std::snprintf(buf, n, "%.3f", static_cast<double>(rate_s_day));
  }
  return buf;
}

} // namespace Cli
