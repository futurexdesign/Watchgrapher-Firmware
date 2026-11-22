/**
 * @file flash_store.h
 * @brief Single owner of the EEPROM-emulation flash sector.
 *
 * Arduino-Pico emulates EEPROM with one 4 KiB flash sector: the whole logical
 * image is held in RAM, edited there, and {@code commit()} erases the sector
 * and reprograms it. This module is the only code that calls
 * {@code EEPROM.begin} / {@code EEPROM.commit}; every persisted feature owns a
 * fixed region of the image and goes through the functions below, so two
 * features can never commit over each other with stale data.
 *
 * Layout (1024 bytes):
 * <pre>
 *   [   0, 256)  TIMEBASE  clock calibration record (timebase.cpp)
 *   [ 256, 512)  DEVICE    device settings record  (device_mode.cpp)
 *   [ 512,1024)  TBLOG     timebase calibration log
 * </pre>
 * The image size grew from 256 to 1024 bytes. {@code EEPROM.begin(n)} copies
 * the first {@code n} bytes of the sector into RAM, so bytes 0..255 (the old
 * image) read back unchanged and the rest read as erased flash (0xFF), which
 * every record's magic check treats as "nothing stored".
 *
 * Writes only stage bytes in RAM. {@link flash_store_commit_paused} programs
 * the sector. A flash erase stalls execute-in-place for tens of milliseconds
 * with interrupts off (and Arduino-Pico's {@code EEPROM.commit} idles core 1
 * via {@code rp2040.idleOtherCore}, so the display core is safe), which would
 * wreck a live USB isochronous stream or a PIO/DMA capture. The commit
 * therefore refuses while USB audio streams and pauses capture otherwise.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Total logical size of the flash image (bytes). */
#define FLASH_STORE_SIZE            1024u
/** Timebase calibration record region. */
#define FLASH_STORE_TIMEBASE_OFFSET 0u
#define FLASH_STORE_TIMEBASE_SIZE   256u
/** Device settings region. */
#define FLASH_STORE_DEVICE_OFFSET   256u
#define FLASH_STORE_DEVICE_SIZE     256u
/** Timebase calibration log region. */
#define FLASH_STORE_TBLOG_OFFSET    512u
#define FLASH_STORE_TBLOG_SIZE      512u

/** Outcome of {@link flash_store_commit_paused}. */
typedef enum {
  /** Committed and verified (or there was nothing to write). */
  FLASH_STORE_COMMIT_OK = 0,
  /** USB audio is streaming; nothing was written. Retry after the host
   * closes the stream. */
  FLASH_STORE_COMMIT_BUSY,
  /** The commit or the read-back verification failed. */
  FLASH_STORE_COMMIT_FAILED
} flash_store_commit_result_t;

/**
 * Loads the flash image into RAM ({@code EEPROM.begin(FLASH_STORE_SIZE)}).
 * Call once from {@code setup()} before any other function here and before
 * {@code timebase_init} / {@code device_mode_init}.
 */
void flash_store_init(void);

/**
 * Copies bytes out of the RAM image.
 *
 * @param offset Absolute offset in the image (use the FLASH_STORE_*_OFFSET
 *        constants plus a field offset).
 * @param out    Destination buffer.
 * @param len    Number of bytes.
 * @return {@code false} (and {@code out} untouched) if the range leaves the
 *         image or the store is not initialized.
 */
bool flash_store_read(uint32_t offset, void *out, size_t len);

/**
 * Stages bytes in the RAM image. Nothing reaches flash until
 * {@link flash_store_commit_paused}.
 *
 * @param offset Absolute offset in the image.
 * @param data   Source bytes.
 * @param len    Number of bytes.
 * @return {@code false} if the range leaves the image or the store is not
 *         initialized (nothing is written).
 */
bool flash_store_write(uint32_t offset, const void *data, size_t len);

/**
 * Writes the staged image to flash.
 *
 * Refuses with {@link FLASH_STORE_COMMIT_BUSY} while
 * {@code usb_audio_streaming()}. Otherwise, if capture is running it is
 * stopped first, the sector is erased and reprogrammed, the flash is read
 * back and compared with the RAM image, and capture is restarted (a fresh
 * stream: DSP and quartz state reset) if it had been running. Does nothing
 * (and does not interrupt capture) when no staged byte differs from flash.
 *
 * Main-loop (core 0) context only.
 */
flash_store_commit_result_t flash_store_commit_paused(void);

/**
 * CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF) used by every
 * record in the image.
 *
 * @param data Bytes to checksum.
 * @param len  Number of bytes.
 */
uint32_t flash_store_crc32(const void *data, size_t len);

#ifdef __cplusplus
}
#endif
