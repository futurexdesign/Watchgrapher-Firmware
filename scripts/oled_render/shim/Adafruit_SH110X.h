/**
 * @file Adafruit_SH110X.h
 * @brief Host stand-in for the SH1107 driver: a GFX frame buffer that the
 *        renderer reads back instead of sending it over I²C.
 *
 * The buffer uses the panel's page layout (bit {@code y % 8} of byte
 * {@code x + (y / 8) * W}), and {@link drawPixel} applies the rotation the
 * same way as Adafruit_GrayOLED, so the frame hash and every drawing call
 * behave as on the device.
 */
#ifndef OLED_RENDER_SH110X_H
#define OLED_RENDER_SH110X_H

#include <Adafruit_GFX.h>
#include <Wire.h>

#include <cstdint>
#include <vector>

#define SH110X_BLACK   0
#define SH110X_WHITE   1
#define SH110X_INVERSE 2

/** Frame-buffer-only SH1107. */
class Adafruit_SH1107 : public Adafruit_GFX {
public:
  Adafruit_SH1107(std::uint16_t w, std::uint16_t h, TwoWire * = &Wire,
                  std::int8_t = -1, std::uint32_t = 0, std::uint32_t = 0)
      : Adafruit_GFX(static_cast<int16_t>(w), static_cast<int16_t>(h)),
        buf_(static_cast<std::size_t>(w) * ((h + 7) / 8), 0) {
    last_ = this;
  }
  ~Adafruit_SH1107() override {
    if (last_ == this) {
      last_ = nullptr;
    }
  }

  /** The most recently created panel (display_ui.cpp keeps its own pointer
   * in an anonymous namespace; the renderer reads the frame through this). */
  [[nodiscard]] static Adafruit_SH1107 *last() { return last_; }

  /** Always succeeds: the simulated panel is present. */
  bool begin(std::uint8_t = 0x3D, bool = true) { return true; }
  void clearDisplay() { std::fill(buf_.begin(), buf_.end(), 0); }
  /** No bus: the renderer reads {@link getBuffer} after each frame. */
  void display() { frames_++; }
  void setContrast(std::uint8_t level) { contrast_ = level; }
  std::uint8_t *getBuffer() { return buf_.data(); }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || y < 0 || x >= width() || y >= height()) {
      return;
    }
    switch (getRotation()) {
    case 1:
      _swap_int16_t(x, y);
      x = static_cast<int16_t>(WIDTH - x - 1);
      break;
    case 2:
      x = static_cast<int16_t>(WIDTH - x - 1);
      y = static_cast<int16_t>(HEIGHT - y - 1);
      break;
    case 3:
      _swap_int16_t(x, y);
      y = static_cast<int16_t>(HEIGHT - y - 1);
      break;
    default:
      break;
    }
    std::uint8_t &b = buf_[static_cast<std::size_t>(x) + (y / 8) * WIDTH];
    const std::uint8_t bit = static_cast<std::uint8_t>(1u << (y & 7));
    if (color == SH110X_WHITE) {
      b |= bit;
    } else if (color == SH110X_BLACK) {
      b &= static_cast<std::uint8_t>(~bit);
    } else {
      b ^= bit;
    }
  }

  /** @return true when the physical pixel ({@code x}, {@code y}) is lit. */
  [[nodiscard]] bool lit(int x, int y) const {
    return (buf_[static_cast<std::size_t>(x) + (y / 8) * WIDTH] >> (y & 7)) &
           1u;
  }
  /** Last contrast sent (0 = never). */
  [[nodiscard]] std::uint8_t contrast() const { return contrast_; }

private:
  std::vector<std::uint8_t> buf_;
  std::uint8_t contrast_ = 0;
  std::uint32_t frames_ = 0;
  static inline Adafruit_SH1107 *last_ = nullptr;
};

#endif // OLED_RENDER_SH110X_H
