/**
 * @file Wire.h
 * @brief Host stand-in for the I²C bus: a display ACKs at
 *        {@code OLED_RENDER_ADDR} (0x3D), nothing else answers.
 */
#ifndef OLED_RENDER_WIRE_H
#define OLED_RENDER_WIRE_H

#include <cstdint>

#ifndef OLED_RENDER_ADDR
#define OLED_RENDER_ADDR 0x3D
#endif

/** I²C bus that only reports which address is present. */
class TwoWire {
public:
  void setSDA(int) {}
  void setSCL(int) {}
  void begin() {}
  void setClock(std::uint32_t) {}
  void beginTransmission(std::uint8_t addr) { addr_ = addr; }
  /** @return 0 (ACK) for the simulated display, 2 (address NACK) otherwise. */
  std::uint8_t endTransmission() const {
    return addr_ == OLED_RENDER_ADDR ? 0 : 2;
  }

private:
  std::uint8_t addr_ = 0;
};

inline TwoWire Wire;
inline TwoWire Wire1;

#endif // OLED_RENDER_WIRE_H
