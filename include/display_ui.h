/**
 * @file display_ui.h
 * @brief SH1107 128×128 I²C OLED readout, run entirely on core 1.
 *
 * Shows the rolling mean (avg_s, default 10 s) daily rate (s/d), amplitude (°) and beat error (ms),
 * the nominal beat rate (e.g. 28,800 bph), the lift angle, a GPS badge when
 * the timebase is PPS-disciplined, and a 2-minute rate trend. Values come from {@link ui_snapshot_read};
 * core 1 never touches the DSP graph, USB or Serial, so a slow I²C transfer
 * (~55 ms per 2 KB frame at 400 kHz) cannot delay the audio stream on core 0.
 *
 * Enabled with {@code DISPLAY_SH1107=1}; pins {@code DISPLAY_SDA_GPIO} /
 * {@code DISPLAY_SCL_GPIO} (an I²C0 or I²C1 pin pair), address
 * {@code DISPLAY_I2C_ADDR} (default 0x3D; 0x3C is tried too), orientation
 * {@code DISPLAY_ROTATION} (0..3). The display may be connected or removed at
 * any time; it is probed every few seconds.
 *
 * @see ui_snapshot.h, docs/DISPLAY.md
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** What core 1 has seen on the display bus (for the CLI {@code display}). */
typedef struct {
  bool enabled;         /**< Built with DISPLAY_SH1107=1 and core 1 started. */
  uint8_t sda_gpio;
  uint8_t scl_gpio;
  uint8_t i2c_port;     /**< 0 = I2C0 (Wire), 1 = I2C1 (Wire1). */
  uint8_t addr_config;  /**< DISPLAY_I2C_ADDR. */
  uint8_t addr_active;  /**< Address the display answered at, 0 = not found. */
  bool ready;           /**< Initialised and being refreshed. */
  uint32_t probes;      /**< Detection attempts (every 2 s while not found). */
  uint32_t inits;       /**< Successful initialisations. */
  uint32_t init_failures; /**< Answered, but the init sequence failed. */
  uint32_t lost;        /**< Times the display stopped answering. */
  uint8_t found[8];     /**< Addresses that ACKed in the last bus scan. */
  uint8_t found_n;
  uint32_t last_scan_ms; /**< millis() of that scan, 0 = never. */
  uint32_t frames_sent;    /**< Frames written over I²C. */
  uint32_t frames_skipped; /**< Frames identical to the last one (not sent). */
} display_ui_status_t;

/** Consistent copy of the core 1 display status; safe from core 0. */
void display_ui_get_status(display_ui_status_t *out);

/**
 * Active (streaming) contrast 1..255; takes effect on the next frame. Not
 * persisted: the boot value is {@code DISPLAY_CONTRAST}. Safe from core 0.
 */
void display_ui_set_contrast(uint8_t level);

/** Frame period in ms (clamped 100..5000); default 500 (2 Hz). Safe from core 0. */
void display_ui_set_period_ms(uint32_t ms);

/** Current contrast / period settings (either pointer may be NULL). */
void display_ui_get_settings(uint8_t *contrast, uint32_t *period_ms);

/** Core 1 setup1(): configures the I²C bus. */
void display_ui_setup(void);

/** Core 1 loop1(): probe / render / sleep until the next frame. */
void display_ui_loop(void);

#ifdef __cplusplus
}
#endif
