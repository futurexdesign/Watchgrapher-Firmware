/**
 * @file board_config.h
 * @brief Board identity, pin map, and PCM1808 clock expectations.
 *
 * RP2350/RP2354 only (Pico 2 / Kiwi). RP2040 PlatformIO envs removed.
 * Pin selection is driven by {@code platformio.ini} build flags:
 * | Env family | MCU | Default ADC |
 * |------------|-----|-------------|
 * | pico2*     | RP2350A | GPIO26 / ch0 |
 * | pico2w*    | RP2350A + CYW43439 | GPIO26 / ch0 |
 * | kiwi*      | RP2350B | GPIO40 / ch0 |
 *
 * Override {@code ADC_GPIO}, {@code ADC_CHANNEL}, and {@code PCM1808_*} pins
 * via {@code -D} as needed. {@code TG_BOARD_NAME} / {@code AUDIO_SOURCE_NAME}
 * feed diagnostic banners.
 */

#pragma once

#include "app_config.h"

#ifndef ADC_GPIO
#error "ADC_GPIO must be defined (see platformio.ini)"
#endif

#ifndef ADC_CHANNEL
#error "ADC_CHANNEL must be defined (see platformio.ini)"
#endif

/*
 * BOARD_PICO2W is tested before BOARD_PICO2: W builds define both, because
 * [board_pico2w] extends [board_pico2].
 */
#if defined(BOARD_PICO2W)
#define TG_BOARD_NAME "Pico2W"
#elif defined(BOARD_PICO2)
#define TG_BOARD_NAME "Pico2"
#elif defined(BOARD_KIWI)
#define TG_BOARD_NAME "KiwiBoard"
#else
#define TG_BOARD_NAME "RP2350"
#endif

/**
 * BLE control plane switch. The [ble] section of platformio.ini sets it to 1
 * on the W envs that build the radio stack. Default 0 (no BLE code, no stack).
 */
#ifndef TG_BLE
#define TG_BLE 0
#endif

#if AUDIO_SOURCE == SYNTHETIC
/* A recorded clip (TG_SYNTH_CLIP) is named after the source, e.g.
 * "SYNTHETIC:nh35_21600"; TG_SYNTH_CLIP_NAME comes from
 * scripts/synth_clip_header.py. */
#if defined(TG_SYNTH_CLIP_NAME)
#define AUDIO_SOURCE_NAME "SYNTHETIC:" TG_SYNTH_CLIP_NAME
#else
#define AUDIO_SOURCE_NAME "SYNTHETIC"
#endif
#elif AUDIO_SOURCE == PCM1808
#define AUDIO_SOURCE_NAME "PCM1808"
#else
#define AUDIO_SOURCE_NAME "ADC"
#endif

/**
 * Status LED. {@code STATUS_LED_CYW43 1} means the LED is the CYW43439 GPIO
 * ({@code LED_BUILTIN}, Pico 2 W) and {@code STATUS_LED_GPIO} is −1.
 * Otherwise {@code STATUS_LED_GPIO} is the MCU pin, or −1 if there is none.
 * Pico 2 / Kiwi use GPIO25. Pico 2 W must not drive GPIO25: it is the radio
 * chip select (WL_CS, F-4).
 */
#if defined(BOARD_PICO2W)
#define STATUS_LED_CYW43 1
#define STATUS_LED_GPIO  (-1)
#else
#ifndef STATUS_LED_CYW43
#define STATUS_LED_CYW43 0
#endif
#ifndef STATUS_LED_GPIO
#if defined(BOARD_PICO2) || defined(BOARD_KIWI)
#define STATUS_LED_GPIO 25
#else
#define STATUS_LED_GPIO (-1)
#endif
#endif
#endif

/**
 * PCM1808 MCLK math assumes PLL_SYS / clk_sys at 150 MHz with GPOUT
 * div 12+0x3500 (Pico 2 / Kiwi board default).
 */
#ifndef PCM1808_EXPECT_SYS_HZ
#define PCM1808_EXPECT_SYS_HZ 150000000u
#endif
#ifndef PCM1808_EXPECT_USB_HZ
#define PCM1808_EXPECT_USB_HZ 48000000u
#endif

/** Optional UART debug pins (Serial1); only used when {@code DIAG_UART=1}. */
#ifndef DIAG_UART_TX_GPIO
#define DIAG_UART_TX_GPIO 0
#endif
#ifndef DIAG_UART_RX_GPIO
#define DIAG_UART_RX_GPIO 1
#endif

/**
 * TPS22918 BOOST_EN (active HIGH). Pin unassigned until Kiwi Rev B spare
 * GPIO is validated. Default OFF ({@code BOOST_SWITCH_PRESENT=0}).
 */
#ifndef BOOST_SWITCH_PRESENT
#define BOOST_SWITCH_PRESENT 0
#endif
#if BOOST_SWITCH_PRESENT
#ifndef BOOST_EN_GPIO
#error "BOOST_EN_GPIO must be assigned when BOOST_SWITCH_PRESENT=1"
#endif
#endif

/**
 * PCM1808 I²S + MCLK defaults (Kiwi / legacy bench wiring).
 * Pico 2 PCB overrides via {@code platformio.ini} ({@code pico2_pcm1808}):
 * MCLK GP21, LRCK GP20, DOUT GP19, BCK GP18.
 */
#ifndef PCM1808_MCLK_GPIO
#define PCM1808_MCLK_GPIO 21
#endif
#ifndef PCM1808_BCK_GPIO
#define PCM1808_BCK_GPIO 2
#endif
#ifndef PCM1808_LRCK_GPIO
#define PCM1808_LRCK_GPIO 3
#endif
#ifndef PCM1808_DOUT_GPIO
#define PCM1808_DOUT_GPIO 4
#endif

/*
 * SH1107 128x128 I2C OLED, driven from core 1 (display_ui.cpp).
 * DISPLAY_SDA_GPIO / DISPLAY_SCL_GPIO must be an SDA / SCL pair of the same
 * I2C block (GPIO n: block (n / 2) % 2, SDA when n is even), e.g. GP4/GP5
 * (I2C0) or GP6/GP7 (I2C1). DISPLAY_ROTATION (0..3, quarter turns; default 3 =
 * 90 deg CCW, as installed in the case) orients the panel. DISPLAY_I2C_ADDR is
 * tried first, then its 0x3C/0x3D sibling. DISPLAY_CONTRAST is the streaming
 * brightness (CLI: display contrast <n>).
 */
#ifndef DISPLAY_SH1107
#define DISPLAY_SH1107 0
#endif
#if DISPLAY_SH1107
#if !defined(DISPLAY_SDA_GPIO) || !defined(DISPLAY_SCL_GPIO)
#error "DISPLAY_SH1107=1 needs DISPLAY_SDA_GPIO and DISPLAY_SCL_GPIO"
#endif
#ifndef DISPLAY_I2C_ADDR
#define DISPLAY_I2C_ADDR 0x3D /* this board's module; 0x3C is also tried */
#endif
#ifndef DISPLAY_ROTATION
#define DISPLAY_ROTATION 3
#endif
#ifndef DISPLAY_CONTRAST
#define DISPLAY_CONTRAST 0x7F /* while streaming, 1..255; idle uses 0x04 */
#endif
#endif
