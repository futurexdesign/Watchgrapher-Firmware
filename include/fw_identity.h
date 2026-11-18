/**
 * @file fw_identity.h
 * @brief Firmware version and USB identity strings (single definition).
 *
 * {@code FW_VERSION} can be overridden at build time with
 * {@code -DFW_VERSION=\"x.y.z\"}.
 */

#pragma once

#ifndef FW_VERSION
#define FW_VERSION "0.1.0-dev"
#endif

/** TinyUSB example VID — prototype only; replace before any distribution. */
#define USB_VENDOR_ID  0xCAFEu
#define USB_PRODUCT_ID 0x4003u

#define USB_MANUFACTURER_STRING "TimegrapherStand"
#define USB_PRODUCT_STRING      "Watch Timing Microphone"
