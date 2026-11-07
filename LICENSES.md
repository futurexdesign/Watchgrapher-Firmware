# Third-party licenses / attribution

This firmware links against:

- **Adafruit TinyUSB Arduino** / TinyUSB — MIT License (Ha Thach / Adafruit)
- **Arduino-Pico (earlephilhower)** — LGPL-2.1 / additional notices in the core package
- **Raspberry Pi Pico SDK** — BSD-3-Clause (as bundled by Arduino-Pico)
- **PlatformIO platform-raspberrypi (maxgerhardt fork)** — Apache-2.0

Bench USB VID/PID `0xCafe` / `0x4003` follows TinyUSB example IDs and is **not** authorized for redistribution as a product. Obtain proper USB-IF IDs before shipping.

UAC2 descriptor macros and control-request patterns are adapted from TinyUSB `examples/device/audio_test` (MIT).
