/**
 * @file usb_audio.cpp
 * @brief UAC2 mic data path and TinyUSB audio control callbacks.
 *
 * Pushes one software 1 ms frame (48 mono samples) per millisecond into
 * TinyUSB; EP IN flow control selects the on-wire 47/48/49 size. Mute is
 * honored; volume is tracked for compliance only (not applied to PCM).
 *
 * @see usb_audio.h, usb_descriptors.cpp
 */

#include "usb_audio.h"

#include "app_config.h"
#include "audio_ring.h"
#include "audio_source.h"
#include "diagnostics.h"

#include <Adafruit_TinyUSB.h>
#include <string.h>

/** Registers the Adafruit_USBD_Interface mic (implemented in usb_descriptors.cpp). */
bool usb_audio_register_interface(void);

static bool g_streaming;
static uint8_t g_mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];
static int16_t g_volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];
static uint32_t g_samp_freq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
static uint8_t g_clk_valid = 1;

static const char *kUsbManufacturer = "TimegrapherStand";
static const char *kUsbProduct = "Watch Timing Microphone (Prototype)";

static audio_control_range_4_n_t(1) g_sample_freq_rng;
static int16_t g_pkt_buf[APP_PKT_SAMPLES_NOM];
static int16_t g_last_sample;
static uint32_t g_feed_ms;

bool usb_audio_streaming(void) { return g_streaming; }

/**
 * Pops up to 48 samples from the ring, holds the last sample on short
 * underrun (avoids zipper noise from zeros), applies mute, and
 * {@code tud_audio_write}s one frame.
 *
 * During {@code STREAM_PREFILL} the ring is not drained — silence is sent
 * until ~10–20 ms of PCM accumulates. Feeding immediately on alt-set open
 * kept occupancy near 0 and produced ~5 ms held-sample plateaus.
 *
 * Must not burst multiple frames from a single call site without pacing —
 * flooding the TinyUSB FIFO forced every ISO packet to wMaxPacketSize and
 * caused pitch/warp distortion on the host.
 */
static void usb_audio_feed_frame(void) {
  if (!g_streaming || !tud_audio_mounted()) {
    return;
  }

  stream_state_t st = audio_source_state();
  if (st != STREAM_STATE_STREAM_ACTIVE) {
    /* Host expects ISO once alt≠0; silence until ring is prefilled. */
    memset(g_pkt_buf, 0, sizeof(g_pkt_buf));
  } else {
    /*
     * Drain whenever ACTIVE. Do not hold at a high watermark — with 5 ms DMA
     * blocks that caused ~50% duty frozen plateaus (241-sample holds) while
     * CDC still reported und=0 / healthy ring occupancy.
     */
    uint32_t got = audio_ring_pop(g_pkt_buf, APP_PKT_SAMPLES_NOM, false);
    for (uint32_t i = got; i < APP_PKT_SAMPLES_NOM; i++) {
      g_pkt_buf[i] = g_last_sample;
    }
    if (got > 0) {
      g_last_sample = g_pkt_buf[got - 1];
    }
  }

  if (g_mute[0] || g_mute[1]) {
    memset(g_pkt_buf, 0, sizeof(g_pkt_buf));
  }

  uint16_t bytes = (uint16_t)(APP_PKT_SAMPLES_NOM * APP_BYTES_PER_SAMPLE);
  uint16_t written = tud_audio_write(g_pkt_buf, bytes);
  diag_counters_t *d = diagnostics_get();
  d->usb_in_packet_count++;
  d->usb_in_sample_count += APP_PKT_SAMPLES_NOM;
  if (written < bytes) {
    d->usb_short_write_count++;
  }
}

bool usb_audio_init(void) {
  g_streaming = false;
  memset(g_mute, 0, sizeof(g_mute));
  for (uint32_t i = 0; i < sizeof(g_volume) / sizeof(g_volume[0]); i++) {
    g_volume[i] = 0; /* 0 dB */
  }

  g_sample_freq_rng.wNumSubRanges = 1;
  g_sample_freq_rng.subrange[0].bMin = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
  g_sample_freq_rng.subrange[0].bMax = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
  g_sample_freq_rng.subrange[0].bRes = 0;

  /*
   * Adafruit TinyUSB always enumerates a CDC Serial interface (required for
   * the Arduino port to link). We add UAC2 mic alongside it — documented
   * Arduino-path divergence from the audio-only spec.
   */
  TinyUSBDevice.setID(0xCafe, 0x4003);
  TinyUSBDevice.setManufacturerDescriptor(kUsbManufacturer);
  TinyUSBDevice.setProductDescriptor(kUsbProduct);

  if (!usb_audio_register_interface()) {
    return false;
  }

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  return true;
}

void usb_audio_task(void) {
  if (!g_streaming) {
    return;
  }
  uint32_t now = millis();
  if (now == g_feed_ms) {
    return;
  }
  /* Catch up at most a few frames if the loop stalled briefly. */
  uint32_t lag = now - g_feed_ms;
  if (lag > 5u) {
    lag = 5u;
  }
  if (g_feed_ms == 0) {
    lag = 1u;
  }
  g_feed_ms = now;
  for (uint32_t i = 0; i < lag; i++) {
    usb_audio_feed_frame();
  }
}

//--------------------------------------------------------------------+
// TinyUSB device callbacks
//--------------------------------------------------------------------+

extern "C" {

void tud_mount_cb(void) {
  diagnostics_get()->usb_enumeration_count++;
  audio_source_set_configured(true);
}

void tud_umount_cb(void) {
  g_streaming = false;
  audio_source_set_configured(false);
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  diagnostics_get()->usb_suspend_count++;
  audio_source_on_suspend();
}

void tud_resume_cb(void) { audio_source_on_resume(); }

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *req) {
  (void)rhport;
  /* SET_INTERFACE: wValue low byte = alternate setting (not high byte). */
  uint8_t const alt = tu_u16_low(req->wValue);
  if (alt != 0) {
    g_streaming = true;
    g_feed_ms = 0;
    g_last_sample = 0;
    diagnostics_get()->usb_audio_stream_open_count++;
    audio_source_on_stream_open();
  }
  return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport,
                                   tusb_control_request_t const *req) {
  (void)rhport;
  (void)req;
  g_streaming = false;
  g_feed_ms = 0;
  diagnostics_get()->usb_audio_stream_close_count++;
  audio_source_on_stream_close();
  return true;
}

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *req,
                             uint8_t *buf) {
  (void)rhport;
  (void)req;
  (void)buf;
  return false;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *req,
                              uint8_t *buf) {
  (void)rhport;
  (void)req;
  (void)buf;
  return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *req,
                                 uint8_t *buf) {
  (void)rhport;
  uint8_t channelNum = TU_U16_LOW(req->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(req->wValue);
  uint8_t entityID = TU_U16_HIGH(req->wIndex);

  TU_VERIFY(req->bRequest == AUDIO_CS_REQ_CUR);

  if (entityID == 2) {
    TU_VERIFY(channelNum <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX);
    switch (ctrlSel) {
    case AUDIO_FU_CTRL_MUTE:
      TU_VERIFY(req->wLength == sizeof(audio_control_cur_1_t));
      g_mute[channelNum] = ((audio_control_cur_1_t *)buf)->bCur;
      return true;
    case AUDIO_FU_CTRL_VOLUME:
      TU_VERIFY(req->wLength == sizeof(audio_control_cur_2_t));
      g_volume[channelNum] = ((audio_control_cur_2_t *)buf)->bCur;
      /* Volume is tracked for host compliance only — not applied to PCM. */
      return true;
    default:
      return false;
    }
  }
  return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *req) {
  (void)rhport;
  (void)req;
  return false;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport,
                              tusb_control_request_t const *req) {
  (void)rhport;
  (void)req;
  return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *req) {
  (void)rhport;
  uint8_t channelNum = TU_U16_LOW(req->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(req->wValue);
  uint8_t entityID = TU_U16_HIGH(req->wIndex);

  if (entityID == 1) {
    if (ctrlSel == AUDIO_TE_CTRL_CONNECTOR) {
      audio_desc_channel_cluster_t ret;
      ret.bNrChannels = 1;
      ret.bmChannelConfig = (audio_channel_config_t)0;
      ret.iChannelNames = 0;
      return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &ret,
                                                       sizeof(ret));
    }
    return false;
  }

  if (entityID == 2) {
    TU_VERIFY(channelNum <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX);
    switch (ctrlSel) {
    case AUDIO_FU_CTRL_MUTE:
      return tud_audio_buffer_and_schedule_control_xfer(
          rhport, req, &g_mute[channelNum], 1);
    case AUDIO_FU_CTRL_VOLUME:
      if (req->bRequest == AUDIO_CS_REQ_CUR) {
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, req, &g_volume[channelNum], sizeof(g_volume[channelNum]));
      }
      if (req->bRequest == AUDIO_CS_REQ_RANGE) {
        audio_control_range_2_n_t(1) range;
        range.wNumSubRanges = 1;
        range.subrange[0].bMin = 0; /* Unity only — no software gain */
        range.subrange[0].bMax = 0;
        range.subrange[0].bRes = 0;
        return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &range,
                                                          sizeof(range));
      }
      return false;
    default:
      return false;
    }
  }

  if (entityID == 4) {
    switch (ctrlSel) {
    case AUDIO_CS_CTRL_SAM_FREQ:
      if (req->bRequest == AUDIO_CS_REQ_CUR) {
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, req, &g_samp_freq, sizeof(g_samp_freq));
      }
      if (req->bRequest == AUDIO_CS_REQ_RANGE) {
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, req, &g_sample_freq_rng, sizeof(g_sample_freq_rng));
      }
      return false;
    case AUDIO_CS_CTRL_CLK_VALID:
      return tud_audio_buffer_and_schedule_control_xfer(
          rhport, req, &g_clk_valid, sizeof(g_clk_valid));
    default:
      return false;
    }
  }

  return false;
}

} /* extern "C" */
