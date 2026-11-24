/**
 * @file Arduino.h
 * @brief Host stand-in for the Arduino core: just what Adafruit GFX and
 *        display_ui.cpp use (scripts/oled_render only).
 */
#ifndef OLED_RENDER_ARDUINO_H
#define OLED_RENDER_ARDUINO_H

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "Print.h"

using boolean = bool;

/** Flash-string marker type; GFX only names it in overloads. */
class __FlashStringHelper;
/** Arduino String; GFX only names it in overloads. */
class String {
public:
  [[nodiscard]] const char *c_str() const { return ""; }
  [[nodiscard]] unsigned length() const { return 0; }
};

#ifndef radians
#define radians(deg) ((deg) * 0.017453292519943295)
#endif
using byte = std::uint8_t;

/** Simulated milliseconds since boot; {@link delay} advances it. */
std::uint32_t millis();
/** Advances the simulated clock by {@code ms}. */
void delay(std::uint32_t ms);

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#endif // OLED_RENDER_ARDUINO_H
