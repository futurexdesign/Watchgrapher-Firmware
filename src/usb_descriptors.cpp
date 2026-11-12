/**
 * @file usb_descriptors.cpp
 * @brief Adafruit TinyUSB interface wrapper for the UAC2 mono microphone.
 *
 * Builds {@code TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR} into the composite
 * configuration (alongside CDC). The interface string is what many hosts
 * (e.g. macOS Core Audio) display as the input device name.
 */

#include "app_config.h"
#include "board_config.h"
#include "usb_audio.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <cstring>

/**
 * Adafruit_USBD_Interface that injects TinyUSB's UAC2 mono mic descriptor
 * into the configuration built by TinyUSBDevice.
 */
class TimegrapherUsbMic : public Adafruit_USBD_Interface {
public:
  /**
   * Sets the interface string and registers with {@code TinyUSBDevice}.
   *
   * @return {@code false} if {@code addInterface} fails.
   */
  bool begin(void) {
    setStringDescriptor("Watchgrapher Studio Interface");
    if (!TinyUSBDevice.addInterface(*this)) {
      return false;
    }
    return true;
  }

  /**
   * Emits the UAC2 mic function descriptor block, allocating 2 interface
   * numbers and one IN endpoint from the device allocator.
   *
   * @param itfnum_deprecated Unused (Adafruit API remnant).
   * @param buf               Output buffer, or null to query length only.
   * @param bufsize           Capacity of {@code buf} when non-null.
   * @return Descriptor length, or 0 if {@code buf} is too small.
   */
  uint16_t getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t *buf,
                                  uint16_t bufsize) override {
    (void)itfnum_deprecated;

    const uint16_t desc_len = TUD_AUDIO_MIC_ONE_CH_DESC_LEN;
    if (!buf) {
      return desc_len;
    }
    if (bufsize < desc_len) {
      return 0;
    }

    uint8_t itfnum = TinyUSBDevice.allocInterface(2);
    uint8_t ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);

    uint8_t desc[] = {
        TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR(
            itfnum, _strid,
            CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX,
            CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * 8,
            ep_in, CFG_TUD_AUDIO_EP_SZ_IN)};

    memcpy(buf, desc, sizeof(desc));
    return sizeof(desc);
  }
};

static TimegrapherUsbMic g_mic;

/**
 * @return Result of {@link TimegrapherUsbMic::begin}.
 */
bool usb_audio_register_interface(void) { return g_mic.begin(); }
