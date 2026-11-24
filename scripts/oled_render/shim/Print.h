/**
 * @file Print.h
 * @brief Host stand-in for the Arduino {@code Print} class: the overloads
 *        display_ui.cpp calls, all routed through {@code write(uint8_t)}.
 */
#ifndef OLED_RENDER_PRINT_H
#define OLED_RENDER_PRINT_H

#include <cstddef>
#include <cstdint>
#include <cstdio>

/** Byte sink with Arduino's text overloads. */
class Print {
public:
  virtual ~Print() = default;
  /** Writes one byte; returns the count written. */
  virtual std::size_t write(std::uint8_t c) = 0;
  /** Writes {@code n} bytes. */
  virtual std::size_t write(const std::uint8_t *buf, std::size_t n) {
    std::size_t out = 0;
    for (std::size_t i = 0; i < n; i++) {
      out += write(buf[i]);
    }
    return out;
  }
  std::size_t write(const char *s) {
    std::size_t out = 0;
    while (*s != '\0') {
      out += write(static_cast<std::uint8_t>(*s++));
    }
    return out;
  }
  std::size_t print(const char *s) { return write(s); }
  std::size_t print(char c) { return write(static_cast<std::uint8_t>(c)); }
  std::size_t print(int v) { return fmt("%d", v); }
  std::size_t print(unsigned v) { return fmt("%u", v); }
  std::size_t print(long v) { return fmt("%ld", v); }
  std::size_t print(unsigned long v) { return fmt("%lu", v); }
  std::size_t print(double v, int digits = 2) {
    char b[40];
    std::snprintf(b, sizeof(b), "%.*f", digits, v);
    return write(b);
  }

private:
  template <typename T> std::size_t fmt(const char *f, T v) {
    char b[24];
    std::snprintf(b, sizeof(b), f, v);
    return write(b);
  }
};

#endif // OLED_RENDER_PRINT_H
