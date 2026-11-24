/**
 * @file json_scan.h
 * @brief Minimal scanners for the fixed-schema request JSON of the control
 *        plane: key lookup, scalar extraction, object slicing, int64 parse
 *        and format.
 *
 * Moved from {@code hid_control.cpp} so the control handler, the arbiter port
 * and the host tests share one implementation. Header-only, no Arduino
 * dependency, no heap. The scanners do not parse JSON in general: they find
 * {@code "key":} by text search, as the former code did, and rely on the
 * host sending the documented request shapes.
 *
 * @see docs/HID_CONTROL_PLANE.md, docs/plans/BLE_TRANSPORT_PLAN.md
 *      section 5.3
 */

#ifndef CONTROL_JSON_SCAN_H
#define CONTROL_JSON_SCAN_H

#include <cctype>
#include <cerrno>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace control::json {

/**
 * @param p Text, may be null.
 * @return {@code p} advanced past white space; null stays null.
 */
[[nodiscard]] inline const char *skip_ws(const char *p) noexcept {
  while (p != nullptr && *p != '\0' &&
         std::isspace(static_cast<unsigned char>(*p)) != 0) {
    p++;
  }
  return p;
}

/**
 * Finds a member of a flat request object.
 *
 * @param json Object text.
 * @param key  Member name (at most 45 characters).
 * @return Pointer to the value after {@code "key":}, or null.
 */
[[nodiscard]] inline const char *find_key(const char *json,
                                          const char *key) noexcept {
  if (json == nullptr || key == nullptr) {
    return nullptr;
  }
  char pat[48];
  std::snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = json;
  while ((p = std::strstr(p, pat)) != nullptr) {
    const char *after = skip_ws(p + std::strlen(pat));
    if (*after == ':') {
      return skip_ws(after + 1);
    }
    p++;
  }
  return nullptr;
}

/**
 * Copies a string member, resolving a backslash to the character after it.
 *
 * @param json   Object text.
 * @param key    Member name.
 * @param out    Destination; NUL-terminated on success.
 * @param outlen Capacity of {@code out}; longer values are truncated.
 * @return true if the member exists and is a string.
 */
[[nodiscard]] inline bool get_string(const char *json, const char *key,
                                     char *out, std::size_t outlen) noexcept {
  const char *v = find_key(json, key);
  if (v == nullptr || *v != '"' || outlen == 0) {
    return false;
  }
  v++;
  std::size_t i = 0;
  while (*v != '\0' && *v != '"' && i + 1 < outlen) {
    if (*v == '\\' && v[1] != '\0') {
      v++;
    }
    out[i++] = *v++;
  }
  out[i] = '\0';
  return true;
}

/**
 * Parses a signed decimal integer at the start of {@code s} (optional white
 * space and sign) as a full 64-bit value, independent of the width of
 * {@code long}. Trailing characters are ignored. A value outside the int64
 * range is rejected.
 *
 * @param s   Text starting at the number.
 * @param out Receives the value on success.
 * @return true if a number was parsed.
 */
[[nodiscard]] inline bool parse_i64(const char *s, std::int64_t *out) noexcept {
  if (s == nullptr || out == nullptr) {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const long long n = std::strtoll(s, &end, 10);
  if (end == s || errno == ERANGE) {
    return false;
  }
  *out = static_cast<std::int64_t>(n);
  return true;
}

/**
 * Formats an int64 as decimal without {@code %lld} (newlib-nano printf has no
 * long long support).
 *
 * @param v   Value.
 * @param out Destination; at most 21 bytes are used.
 */
inline void i64_dec(std::int64_t v, char (&out)[24]) noexcept {
  /* Magnitude as unsigned so INT64_MIN does not overflow. */
  std::uint64_t mag = v < 0 ? 0u - static_cast<std::uint64_t>(v)
                            : static_cast<std::uint64_t>(v);
  char tmp[24];
  std::size_t n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + static_cast<int>(mag % 10u));
    mag /= 10u;
  } while (mag != 0u && n < sizeof(tmp) - 1u);
  std::size_t o = 0;
  if (v < 0) {
    out[o++] = '-';
  }
  while (n > 0) {
    out[o++] = tmp[--n];
  }
  out[o] = '\0';
}

/**
 * Reads an integer member as a full 64-bit value ({@code long} is 32-bit on
 * the RP2350). Used for {@code id} and {@code ping.t}.
 *
 * @param json Object text.
 * @param key  Member name.
 * @param out  Receives the value.
 * @return true if present and numeric (and within int64).
 */
[[nodiscard]] inline bool get_i64(const char *json, const char *key,
                                  std::int64_t *out) noexcept {
  const char *v = find_key(json, key);
  return v != nullptr && parse_i64(v, out);
}

/**
 * Reads an integer member that is a 32-bit quantity (limits, rates,
 * offsets): the 64-bit value saturates to the range of {@code long}, and
 * callers range-check the result.
 *
 * @param json Object text.
 * @param key  Member name.
 * @param out  Receives the value.
 * @return true if present and numeric.
 */
[[nodiscard]] inline bool get_int(const char *json, const char *key,
                                  long *out) noexcept {
  std::int64_t n = 0;
  if (!get_i64(json, key, &n)) {
    return false;
  }
  if (n > static_cast<std::int64_t>(LONG_MAX)) {
    n = LONG_MAX;
  } else if (n < static_cast<std::int64_t>(LONG_MIN)) {
    n = LONG_MIN;
  }
  *out = static_cast<long>(n);
  return true;
}

/**
 * Reads a numeric member as {@code double}.
 *
 * @param json Object text.
 * @param key  Member name.
 * @param out  Receives the value.
 * @return true if present and numeric.
 */
[[nodiscard]] inline bool get_double(const char *json, const char *key,
                                     double *out) noexcept {
  const char *v = find_key(json, key);
  if (v == nullptr) {
    return false;
  }
  char *end = nullptr;
  const double n = std::strtod(v, &end);
  if (end == v) {
    return false;
  }
  *out = n;
  return true;
}

/**
 * Reads a {@code true} / {@code false} member.
 *
 * @param json Object text.
 * @param key  Member name.
 * @param out  Receives the value.
 * @return true if present and a boolean literal.
 */
[[nodiscard]] inline bool get_bool(const char *json, const char *key,
                                   bool *out) noexcept {
  const char *v = find_key(json, key);
  if (v == nullptr) {
    return false;
  }
  if (std::strncmp(v, "true", 4) == 0) {
    *out = true;
    return true;
  }
  if (std::strncmp(v, "false", 5) == 0) {
    *out = false;
    return true;
  }
  return false;
}

/**
 * Copies the object that starts at {@code start} (including its braces,
 * honouring strings and escapes) into {@code dst}.
 *
 * @param start  Text at or before the opening brace (white space is skipped).
 * @param dst    Destination; NUL-terminated on success.
 * @param dstlen Capacity of {@code dst}.
 * @return true if a complete object fit.
 */
[[nodiscard]] inline bool extract_object(const char *start, char *dst,
                                         std::size_t dstlen) noexcept {
  start = skip_ws(start);
  if (start == nullptr || *start != '{') {
    return false;
  }
  int depth = 0;
  std::size_t i = 0;
  bool in_str = false;
  for (const char *p = start; *p != '\0'; p++) {
    const char c = *p;
    if (in_str) {
      if (c == '\\' && p[1] != '\0') {
        if (i + 2 >= dstlen) {
          return false;
        }
        dst[i++] = *p++;
        dst[i++] = *p;
        continue;
      }
      if (c == '"') {
        in_str = false;
      }
    } else {
      if (c == '"') {
        in_str = true;
      } else if (c == '{') {
        depth++;
      } else if (c == '}') {
        depth--;
        if (depth == 0) {
          if (i + 2 >= dstlen) {
            return false;
          }
          dst[i++] = c;
          dst[i] = '\0';
          return true;
        }
      }
    }
    if (i + 1 >= dstlen) {
      return false;
    }
    dst[i++] = c;
  }
  return false;
}

} // namespace control::json

#endif // CONTROL_JSON_SCAN_H
