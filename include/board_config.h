/**
 * @file board_config.h
 * @brief Board identity, pin map, and PCM1808 clock expectations.
 *
 * RP2350/RP2354 only (Pico 2 / Kiwi). RP2040 PlatformIO envs removed.
 * Pin selection is driven by {@code platformio.ini} build flags:
 * | Env family | MCU | Default ADC |
 * |------------|-----|-------------|
 * | pico2*     | RP2350A | GPIO26 / ch0 |
 * | kiwi*      | RP2350B | GPIO40 / ch0 |
 *
 * Override {@code ADC_GPIO}, {@code ADC_CHANNEL}, and {@code PCM1808_*} pins
 * via {@code -D} as needed. {@code BOARD_NAME} / {@code AUDIO_SOURCE_NAME}
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

#if defined(BOARD_PICO2)
#define BOARD_NAME "Pico2"
#elif defined(BOARD_KIWI)
#define BOARD_NAME "KiwiBoard"
#else
#define BOARD_NAME "RP2350"
#endif

#if AUDIO_SOURCE == SYNTHETIC
#define AUDIO_SOURCE_NAME "SYNTHETIC"
#elif AUDIO_SOURCE == PCM1808
#define AUDIO_SOURCE_NAME "PCM1808"
#else
#define AUDIO_SOURCE_NAME "ADC"
#endif

/**
 * Onboard status LED GPIO, or −1 if none.
 * Pico 2 / Kiwi use GPIO25.
 */
#ifndef STATUS_LED_GPIO
#if defined(BOARD_PICO2) || defined(BOARD_KIWI)
#define STATUS_LED_GPIO 25
#else
#define STATUS_LED_GPIO (-1)
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

/** PCM1808 I²S + MCLK defaults (P3.0); overridable via {@code -D}. */
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
