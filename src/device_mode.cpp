/**
 * @file device_mode.cpp
 * @brief Persisted device mode, timebase-log threshold, and run control.
 *
 * The settings record sits in the DEVICE region of the flash image. It is
 * versioned and CRC-checked, so an erased or foreign region reads as
 * "defaults" (USB_MIC, 0.2 ppm, not saved).
 *
 * @see device_mode.h, flash_store.h
 */

#include "device_mode.h"

#include "flash_store.h"
#include "usb_audio.h"

#include <cstddef>
#include <cstring>
#include <strings.h>

namespace {

constexpr uint32_t kMagic = 0x31564544u; /* "DEV1" little-endian */
constexpr uint16_t kVersion = 1u;

/** On-flash settings record (little-endian, no padding). */
struct Record {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t mode;
  float log_delta_ppm;
  uint32_t crc;
};
static_assert(sizeof(Record) <= FLASH_STORE_DEVICE_SIZE,
              "record exceeds DEVICE region");

/** In-RAM settings. */
struct Settings {
  device_mode_t mode;
  float log_delta_ppm;
};

constexpr Settings kDefaults = {DEVICE_MODE_USB_MIC,
                                DEVICE_LOG_DELTA_PPM_DEFAULT};

Settings g_settings = kDefaults;
/** Flash holds a valid record. */
bool g_have_saved = false;
/** The mode stored in flash (valid when {@code g_have_saved}). */
device_mode_t g_saved_mode = DEVICE_MODE_USB_MIC;

uint32_t record_crc(const Record &r) {
  return flash_store_crc32(&r, offsetof(Record, crc));
}

/** Reads and validates the record. */
bool load_record(Settings *out) {
  Record r{};
  if (!flash_store_read(FLASH_STORE_DEVICE_OFFSET, &r, sizeof(r))) {
    return false;
  }
  if (r.magic != kMagic || r.version != kVersion || r.size != sizeof(Record) ||
      r.crc != record_crc(r)) {
    return false;
  }
  if (r.mode > static_cast<uint32_t>(DEVICE_MODE_STANDALONE)) {
    return false;
  }
  /* An out-of-range delta (e.g. from a firmware with other limits) falls back
   * to the default on its own, so it never discards the stored mode. */
  const float delta = timebase_log_delta_valid(r.log_delta_ppm)
                          ? r.log_delta_ppm
                          : TIMEBASE_LOG_DELTA_PPM_DEFAULT;
  *out = {static_cast<device_mode_t>(r.mode), delta};
  return true;
}

/** Stages {@code s} in the flash image (no commit). */
bool stage_record(const Settings &s) {
  Record r{};
  r.magic = kMagic;
  r.version = kVersion;
  r.size = sizeof(Record);
  r.mode = static_cast<uint32_t>(s.mode);
  r.log_delta_ppm = s.log_delta_ppm;
  r.crc = record_crc(r);
  return flash_store_write(FLASH_STORE_DEVICE_OFFSET, &r, sizeof(r));
}

/**
 * Makes {@code next} the live settings and persists it. On failure the
 * previous settings are restored in RAM and re-staged, so a later commit by
 * another module cannot write the rejected value.
 */
device_mode_set_result_t persist(const Settings &next) {
  if (usb_audio_streaming()) {
    return DEVICE_SET_BUSY;
  }
  const Settings prev = g_settings;
  g_settings = next;
  device_mode_set_result_t res = DEVICE_SET_FAILED;
  if (stage_record(next)) {
    switch (flash_store_commit_paused()) {
    case FLASH_STORE_COMMIT_OK:
      res = DEVICE_SET_OK;
      break;
    case FLASH_STORE_COMMIT_BUSY:
      res = DEVICE_SET_BUSY;
      break;
    default:
      break;
    }
  }
  if (res != DEVICE_SET_OK) {
    g_settings = prev;
    (void)stage_record(prev);
    return res;
  }
  g_have_saved = true;
  g_saved_mode = next.mode;
  return DEVICE_SET_OK;
}

} // namespace

void device_mode_init(void) {
  Settings s{};
  if (load_record(&s)) {
    g_settings = s;
    g_have_saved = true;
    g_saved_mode = s.mode;
  } else {
    g_settings = kDefaults;
    g_have_saved = false;
  }
}

void device_boot_run(void) {
  if (g_settings.mode == DEVICE_MODE_STANDALONE) {
    (void)device_run_start();
  }
}

device_mode_t device_mode_get(void) { return g_settings.mode; }

device_mode_set_result_t device_mode_set(device_mode_t mode) {
  if (mode != DEVICE_MODE_USB_MIC && mode != DEVICE_MODE_STANDALONE) {
    return DEVICE_SET_BAD_PARAM;
  }
  if (mode == g_settings.mode) {
    return DEVICE_SET_OK;
  }
  Settings next = g_settings;
  next.mode = mode;
  /* persist() stops capture around the write and restarts it, now under the
   * new mode (the ring feed follows device_mode_get()). */
  const device_mode_set_result_t res = persist(next);
  if (res != DEVICE_SET_OK) {
    return res;
  }
  if (mode == DEVICE_MODE_STANDALONE) {
    (void)device_run_start();
  } else {
    audio_source_set_run_requested(false);
  }
  return DEVICE_SET_OK;
}

device_run_result_t device_run_start(void) {
  if (!audio_source_backend_ok()) {
    return DEVICE_RUN_BACKEND_ERROR;
  }
  audio_source_set_run_requested(true);
  return DEVICE_RUN_OK;
}

void device_run_stop(void) { audio_source_set_run_requested(false); }

void device_status_get(device_status_t *out) {
  if (!out) {
    return;
  }
  out->mode = g_settings.mode;
  out->running = audio_source_capture_running();
  out->run_requested = audio_source_run_requested();
  out->usb_streaming = usb_audio_streaming();
  out->stream_state = audio_source_state();
  out->saved = g_have_saved && g_saved_mode == g_settings.mode;
}

const char *device_mode_name(device_mode_t mode) {
  return mode == DEVICE_MODE_STANDALONE ? "standalone" : "usbMic";
}

bool device_mode_from_name(const char *name, device_mode_t *out) {
  if (!name || !out) {
    return false;
  }
  if (strcasecmp(name, "usbmic") == 0 || strcasecmp(name, "usb_mic") == 0 ||
      strcasecmp(name, "usb") == 0) {
    *out = DEVICE_MODE_USB_MIC;
    return true;
  }
  if (strcasecmp(name, "standalone") == 0 || strcasecmp(name, "sa") == 0) {
    *out = DEVICE_MODE_STANDALONE;
    return true;
  }
  return false;
}

float device_log_delta_ppm_get(void) { return g_settings.log_delta_ppm; }

device_mode_set_result_t device_log_delta_ppm_set(float ppm) {
  if (!timebase_log_delta_valid(ppm)) {
    return DEVICE_SET_BAD_PARAM;
  }
  Settings next = g_settings;
  next.log_delta_ppm = ppm;
  return persist(next);
}
