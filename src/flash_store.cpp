/**
 * @file flash_store.cpp
 * @brief EEPROM-emulation flash image: region access and paused commit.
 *
 * Findings about Arduino-Pico's EEPROM (framework-arduinopico, EEPROM.cpp):
 * - {@code begin(size)} rounds {@code size} up to 256, allocates a RAM copy
 *   and {@code memcpy}s that many bytes from the sector, so a larger size
 *   keeps the existing bytes and exposes the rest as erased (0xFF).
 * - {@code commit()} is a no-op when no byte changed; otherwise it does
 *   {@code noInterrupts(); rp2040.idleOtherCore();} erase the 4 KiB sector,
 *   program {@code size} bytes, {@code resumeOtherCore(); interrupts();}. So
 *   core 1 (the OLED loop) is parked for the duration and the write is
 *   multicore safe; only core 0's own work (USB, PIO/DMA capture) has to be
 *   quiesced by us. A 256-byte-page-aligned program of the whole image also
 *   restores the regions we did not touch, because the RAM copy holds them.
 *
 * @see flash_store.h
 */

#include "flash_store.h"

#include "audio_source.h"
#include "usb_audio.h"

#include <Arduino.h>
#include <EEPROM.h>

#include <cstring>

/** Start of the EEPROM-emulation sector in the XIP window (linker symbol). */
extern "C" uint8_t _EEPROM_start;

namespace {

bool g_ready = false;
/** A staged byte differs from the flash content as of the last commit. */
bool g_dirty = false;

/** @return {@code true} if [offset, offset+len) lies inside the image. */
bool in_range(uint32_t offset, size_t len) {
  return g_ready && offset <= FLASH_STORE_SIZE &&
         len <= FLASH_STORE_SIZE - offset;
}

/** Compares the RAM image with the flash sector (XIP read). */
bool flash_matches_image() {
  /* _EEPROM_start is a linker symbol marking the start of the flash sector,
   * not a single byte, so indexing past it is intended. */
  const uint8_t *flash = &_EEPROM_start;
  for (uint32_t i = 0; i < FLASH_STORE_SIZE; i++) {
    // cppcheck-suppress objectIndex
    if (flash[i] != EEPROM.read(static_cast<int>(i))) {
      return false;
    }
  }
  return true;
}

} // namespace

void flash_store_init(void) {
  EEPROM.begin(FLASH_STORE_SIZE);
  g_ready = true;
  g_dirty = false;
}

bool flash_store_read(uint32_t offset, void *out, size_t len) {
  if (!out || !in_range(offset, len)) {
    return false;
  }
  auto *p = static_cast<uint8_t *>(out);
  for (size_t i = 0; i < len; i++) {
    p[i] = EEPROM.read(static_cast<int>(offset + i));
  }
  return true;
}

bool flash_store_write(uint32_t offset, const void *data, size_t len) {
  if (!data || !in_range(offset, len)) {
    return false;
  }
  const auto *p = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < len; i++) {
    const auto addr = static_cast<int>(offset + i);
    if (EEPROM.read(addr) != p[i]) {
      EEPROM.write(addr, p[i]);
      g_dirty = true;
    }
  }
  return true;
}

flash_store_commit_result_t flash_store_commit_paused(void) {
  if (!g_ready) {
    return FLASH_STORE_COMMIT_FAILED;
  }
  /* The erase stalls XIP and USB IRQs for tens of ms: never mid-stream. */
  if (usb_audio_streaming()) {
    return FLASH_STORE_COMMIT_BUSY;
  }
  if (!g_dirty) {
    return FLASH_STORE_COMMIT_OK;
  }
  const bool was_running = audio_source_capture_pause();
  bool ok = EEPROM.commit() && flash_matches_image();
  if (was_running) {
    audio_source_capture_resume();
  }
  if (ok) {
    g_dirty = false;
  }
  return ok ? FLASH_STORE_COMMIT_OK : FLASH_STORE_COMMIT_FAILED;
}

uint32_t flash_store_crc32(const void *data, size_t len) {
  const auto *p = static_cast<const uint8_t *>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}
