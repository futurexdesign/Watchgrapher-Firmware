/**
 * @file display_ui.cpp
 * @brief SH1107 128×128 OLED readout on core 1.
 *
 * Layout (pixels; fonts 6×8 / 12×16 / 18×24 at text size 1 / 2 / 3):
 *
 *     y   0-15   28,800 bph                       (GPS)   header (or status)
 *     y  17      ─────────────────────────────────────
 *     y  21      RATE                           10s avg   (avg_s)
 *     y  31-54          +12.3 s/d                         size 3, centred
 *     y  59      ─────────────────────────────────────
 *     y  62      AMP LA 52°        BEAT ERR               LA = lift angle
 *     y  72-87   285°              0.4ms                  size 2
 *     y  91      ─────────────────────────────────────
 *     y  94      TREND 2 min                   ±5 s/d
 *     y 104-127  rate trend, 1 px per second, zero line dotted
 *
 * The GPS badge (top right, only in PPS builds) is solid while the timebase is
 * disciplined by GPS 1 PPS and an outline while it is not.
 *
 * Rate, amplitude and beat error are the rolling means over the configured
 * integration time (beat param avg_s, default 10 s). They show as
 * placeholders until timing is locked on a verified timebase; the trend has a
 * gap wherever there was no lock.
 *
 * @see display_ui.h
 */

#include "display_ui.h"

#include "board_config.h"

#if DISPLAY_SH1107

#include "ui_format.h"
#include "ui_snapshot.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <Wire.h>
#include <hardware/sync.h>

#include <cmath>
#include <cstring>

/* RP2350 I²C pin map: GPIO n is I2C((n / 2) % 2), SDA if n is even. */
static_assert((DISPLAY_SDA_GPIO % 4 == 0 && DISPLAY_SCL_GPIO % 4 == 1) ||
                  (DISPLAY_SDA_GPIO % 4 == 2 && DISPLAY_SCL_GPIO % 4 == 3),
              "DISPLAY_SDA_GPIO / DISPLAY_SCL_GPIO must be an SDA/SCL pair of one I2C block");
static_assert(DISPLAY_ROTATION >= 0 && DISPLAY_ROTATION <= 3, "DISPLAY_ROTATION is 0..3");
static_assert(DISPLAY_CONTRAST >= 1 && DISPLAY_CONTRAST <= 255, "DISPLAY_CONTRAST is 1..255");

namespace {

constexpr int16_t kSize = 128;
constexpr uint32_t kI2cHz = 400000u;  /* SH1107 fast-mode limit; 2 KB ≈ 55 ms */
constexpr uint32_t kFrameMs = 500u;   /* 2 Hz: the rolling means move slowly */
constexpr uint32_t kTrendMs = 1000u;  /* one trend column per second */
constexpr uint32_t kProbeMs = 2000u;  /* look for a (re)connected display */
constexpr uint32_t kRecheckMs = 5000u;
constexpr uint8_t kContrastIdle = 0x04;
constexpr uint8_t kDegree = 248;      /* '°' (CP437) */
constexpr uint8_t kPlusMinus = 241;   /* '±' */

/* Trend graph geometry. */
constexpr int16_t kTrendTop = 104;
constexpr int16_t kTrendBottom = 127;
constexpr int16_t kTrendMid = (kTrendTop + kTrendBottom) / 2;
constexpr int16_t kTrendHalf = (kTrendBottom - kTrendTop) / 2;
constexpr uint32_t kTrendLen = 120; /* 2 minutes */
constexpr int16_t kTrendX0 = (kSize - static_cast<int16_t>(kTrendLen)) / 2;

TwoWire &bus() {
  return ((DISPLAY_SDA_GPIO / 2) % 2 == 0) ? Wire : Wire1;
}

/*
 * Created per successful init and deleted on loss: the library allocates its
 * I2C device object and frame buffer in begin() and only frees them in the
 * destructor, so re-running begin() on the same object after a hot-plug
 * would leak.
 */
Adafruit_SH1107 *g_oled = nullptr;
bool g_ready = false;
uint8_t g_addr = 0; /* address the display answered at, 0 = none */

/* Bus diagnostics for display_ui_get_status (written by core 1 only). */
constexpr uint8_t kMaxFound = 8;
volatile uint32_t g_status_seq = 0;
display_ui_status_t g_status = {};
uint32_t g_last_probe_ms = 0;
uint32_t g_last_check_ms = 0;
uint32_t g_last_trend_ms = 0;

/** Rate history, oldest first after {@code g_trend_head}; NaN = gap. */
float g_trend[kTrendLen];
uint32_t g_trend_head = 0;

/*
 * Live settings from core 0 (CLI {@code display contrast|period}); core 1
 * applies them on its next frame. Single 32-bit words, so no lock is needed.
 */
volatile uint32_t g_contrast_req = DISPLAY_CONTRAST;
volatile uint32_t g_period_req = kFrameMs;
uint8_t g_contrast_applied = 0; /* 0 = not sent since init */

/** Frame-buffer hash of the last frame sent; the I²C write is skipped if equal. */
uint32_t g_last_hash = 0;
bool g_have_hash = false;

/**
 * GPS badge: "GPS" in a 23 × 11 pill, inverse (solid) when {@code locked},
 * an outline otherwise.
 */
void draw_gps(int16_t x0, int16_t y0, bool locked) {
  constexpr int16_t kW = 23, kH = 11, kR = 5;
  if (locked) {
    g_oled->fillRoundRect(x0, y0, kW, kH, kR, SH110X_WHITE);
    g_oled->setTextColor(SH110X_BLACK);
  } else {
    g_oled->drawRoundRect(x0, y0, kW, kH, kR, SH110X_WHITE);
  }
  g_oled->setTextSize(1);
  g_oled->setCursor(x0 + 3, y0 + 2);
  g_oled->print("GPS");
  g_oled->setTextColor(SH110X_WHITE);
}

uint32_t frame_hash() {
  const uint8_t *p = g_oled->getBuffer();
  uint32_t h = 2166136261u; /* FNV-1a */
  for (uint32_t i = 0; i < kSize * kSize / 8u; i++) {
    h = (h ^ p[i]) * 16777619u;
  }
  return h;
}

bool probe_addr(uint8_t addr) {
  bus().beginTransmission(addr);
  return bus().endTransmission() == 0;
}

/** Publishes {@code g_status} edits made inside {@code fn} (seqlock writer). */
template <typename F> void update_status(F fn) {
  g_status_seq = g_status_seq + 1u;
  __dmb();
  fn(g_status);
  __dmb();
  g_status_seq = g_status_seq + 1u;
}

/** Records every 7-bit address that ACKs (for the CLI {@code display} report). */
void scan_bus(uint32_t now) {
  uint8_t found[kMaxFound];
  uint8_t n = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (probe_addr(a) && n < kMaxFound) {
      found[n++] = a;
    }
  }
  update_status([&](display_ui_status_t &st) {
    memcpy(st.found, found, n);
    st.found_n = n;
    st.last_scan_ms = now;
  });
}

/**
 * Finds the display: the configured address first, then its SH1107 / SSD1306
 * sibling (0x3C <-> 0x3D), since modules ship strapped either way.
 *
 * @return The address that answered, or 0.
 */
uint8_t find_display() {
  const uint8_t candidates[] = {static_cast<uint8_t>(DISPLAY_I2C_ADDR),
                                static_cast<uint8_t>(DISPLAY_I2C_ADDR ^ 0x01u)};
  for (uint8_t a : candidates) {
    if (probe_addr(a)) {
      return a;
    }
  }
  return 0;
}

/** Creates and initialises the driver at {@code addr}. */
bool init_display(uint8_t addr) {
  delete g_oled;
  g_oled = new Adafruit_SH1107(kSize, kSize, &bus(), -1, kI2cHz, kI2cHz);
  if (!g_oled || !g_oled->begin(addr, false)) {
    delete g_oled;
    g_oled = nullptr;
    return false;
  }
  g_oled->setRotation(DISPLAY_ROTATION);
  g_oled->cp437(true);
  g_contrast_applied = 0;
  g_have_hash = false;
  return true;
}

bool is_locked(const ui_snapshot_t &s, bool have) {
  return have && s.streaming && s.timebase_ok && s.locked;
}

/** Header text while there is no lock. */
const char *status_text(const ui_snapshot_t &s, bool have) {
  if (!have) {
    return "starting...";
  }
  if (!s.streaming) {
    return "waiting for host";
  }
  if (!s.timebase_ok) {
    return "checking clock...";
  }
  if (s.events_seen == 0u) {
    return "listening...";
  }
  return "locking...";
}

void trend_push(float v) {
  g_trend[g_trend_head] = v;
  g_trend_head = (g_trend_head + 1u) % kTrendLen;
}

/** Text width in pixels for the classic font at {@code size}. */
int16_t text_w(const char *s, uint8_t size) {
  return static_cast<int16_t>(strlen(s) * 6u * size);
}

void draw_header(const ui_snapshot_t &s, bool have, bool locked) {
  g_oled->setTextSize(1);
  g_oled->setCursor(0, 4);
  if (locked) {
    char vph[12];
    ui_format_thousands(s.vph, vph, sizeof(vph));
    g_oled->print(vph);
    g_oled->print(" bph");
  } else {
    g_oled->print(status_text(s, have));
  }
  if (have && s.pps_enabled) {
    draw_gps(kSize - 23, 2, s.pps_locked);
  }
  g_oled->drawFastHLine(0, 17, kSize, SH110X_WHITE);
}

void draw_rate(const ui_snapshot_t &s, bool locked) {
  char rate[8];
  ui_format_rate(locked && s.avg.rate_valid, s.avg.rate_s_day, rate, sizeof(rate));
  g_oled->setTextSize(1);
  g_oled->setCursor(0, 21);
  g_oled->print("RATE");
  char cap[12];
  if (s.avg_s > 0.f) {
    snprintf(cap, sizeof(cap), "%gs avg", static_cast<double>(s.avg_s));
  } else {
    snprintf(cap, sizeof(cap), "avg");
  }
  g_oled->setCursor(kSize - text_w(cap, 1), 21);
  g_oled->print(cap);

  /* Centre value + units as one group. */
  const int16_t w = text_w(rate, 3) + 3 + text_w("s/d", 1);
  const int16_t x = (kSize - w) / 2;
  g_oled->setTextSize(3);
  g_oled->setCursor(x, 31);
  g_oled->print(rate);
  g_oled->setTextSize(1);
  g_oled->setCursor(x + text_w(rate, 3) + 3, 45);
  g_oled->print("s/d");
  g_oled->drawFastHLine(0, 59, kSize, SH110X_WHITE);
}

void draw_amp_be(const ui_snapshot_t &s, bool locked) {
  char amp[8], be[8];
  ui_format_amp(locked && s.avg.amplitude_valid, s.avg.amplitude_deg, amp, sizeof(amp));
  ui_format_be(locked && s.avg.beat_error_valid, s.avg.beat_error_ms, be, sizeof(be));
  constexpr int16_t kCol2 = 68;

  g_oled->setTextSize(1);
  char lift[8];
  ui_format_lift(s.lift_angle_deg, lift, sizeof(lift));
  g_oled->setCursor(0, 62);
  g_oled->print("AMP LA ");
  g_oled->print(lift);
  g_oled->write(kDegree);
  g_oled->setCursor(kCol2, 62);
  g_oled->print("BEAT ERR");

  g_oled->setTextSize(2);
  g_oled->setCursor(0, 72);
  g_oled->print(amp);
  /* The CP437 '°' is a heavy blob at size 2; a small ring reads better. */
  g_oled->drawCircle(text_w(amp, 2) + 3, 75, 2, SH110X_WHITE);
  g_oled->setCursor(kCol2, 72);
  g_oled->print(be);
  g_oled->setTextSize(1);
  g_oled->setCursor(kCol2 + text_w(be, 2) + 2, 80);
  g_oled->print("ms");
  g_oled->drawFastHLine(0, 91, kSize, SH110X_WHITE);
}

void draw_trend() {
  /* Unroll the ring oldest → newest. */
  float v[kTrendLen];
  for (uint32_t i = 0; i < kTrendLen; i++) {
    v[i] = g_trend[(g_trend_head + i) % kTrendLen];
  }
  const float scale = ui_trend_scale(v, kTrendLen);

  char cap[16];
  snprintf(cap, sizeof(cap), "%c%.0f s/d", static_cast<char>(kPlusMinus),
           static_cast<double>(scale));
  g_oled->setTextSize(1);
  g_oled->setCursor(0, 94);
  g_oled->print("TREND 2 min");
  g_oled->setCursor(kSize - text_w(cap, 1), 94);
  g_oled->print(cap);

  /* Dotted zero line. */
  for (int16_t x = kTrendX0; x < kTrendX0 + static_cast<int16_t>(kTrendLen); x += 3) {
    g_oled->drawPixel(x, kTrendMid, SH110X_WHITE);
  }

  int16_t prev_y = -1;
  for (uint32_t i = 0; i < kTrendLen; i++) {
    if (!std::isfinite(v[i])) {
      prev_y = -1; /* gap */
      continue;
    }
    /* Positive (fast) plots upwards. */
    float yf = static_cast<float>(kTrendMid) - v[i] / scale * static_cast<float>(kTrendHalf);
    yf = fminf(static_cast<float>(kTrendBottom), fmaxf(static_cast<float>(kTrendTop), yf));
    const auto y = static_cast<int16_t>(lroundf(yf));
    const auto x = static_cast<int16_t>(kTrendX0 + static_cast<int16_t>(i));
    if (prev_y >= 0) {
      g_oled->drawLine(x - 1, prev_y, x, y, SH110X_WHITE);
    } else {
      g_oled->drawPixel(x, y, SH110X_WHITE);
    }
    prev_y = y;
  }
}

void render(const ui_snapshot_t &s, bool have) {
  const bool locked = is_locked(s, have);

  /* Dim while idle to slow OLED wear; the command is sent only on change. */
  const bool idle = !(have && s.streaming);
  const uint32_t req = g_contrast_req;
  const uint8_t contrast =
      idle ? kContrastIdle : static_cast<uint8_t>(req == 0u ? 1u : (req > 255u ? 255u : req));
  if (contrast != g_contrast_applied) {
    g_oled->setContrast(contrast);
    g_contrast_applied = contrast;
  }

  g_oled->clearDisplay();
  g_oled->setTextColor(SH110X_WHITE);
  draw_header(s, have, locked);
  draw_rate(s, locked);
  draw_amp_be(s, locked);
  draw_trend();

  /* Most frames are identical (the rolling means and trend move at ~1 Hz):
   * only send a frame that changed, so the panel is written as little as
   * possible. */
  const uint32_t h = frame_hash();
  if (g_have_hash && h == g_last_hash) {
    update_status([](display_ui_status_t &st) { st.frames_skipped++; });
    return;
  }
  g_oled->display();
  g_last_hash = h;
  g_have_hash = true;
  update_status([](display_ui_status_t &st) { st.frames_sent++; });
}

} // namespace

void display_ui_get_status(display_ui_status_t *out) {
  if (!out) {
    return;
  }
  for (int attempt = 0; attempt < 64; attempt++) {
    const uint32_t before = g_status_seq;
    if (before & 1u) {
      continue;
    }
    __dmb();
    memcpy(out, &g_status, sizeof(*out));
    __dmb();
    if (g_status_seq == before) {
      return;
    }
  }
}

void display_ui_set_contrast(uint8_t level) {
  g_contrast_req = level == 0u ? 1u : level;
}

void display_ui_set_period_ms(uint32_t ms) {
  g_period_req = ms < 100u ? 100u : (ms > 5000u ? 5000u : ms);
}

void display_ui_get_settings(uint8_t *contrast, uint32_t *period_ms) {
  if (contrast) {
    *contrast = static_cast<uint8_t>(g_contrast_req);
  }
  if (period_ms) {
    *period_ms = g_period_req;
  }
}

void display_ui_setup(void) {
  update_status([](display_ui_status_t &st) {
    st.enabled = true;
    st.sda_gpio = DISPLAY_SDA_GPIO;
    st.scl_gpio = DISPLAY_SCL_GPIO;
    st.i2c_port = (DISPLAY_SDA_GPIO / 2) % 2;
    st.addr_config = DISPLAY_I2C_ADDR;
  });
  for (float &v : g_trend) {
    v = NAN;
  }
  bus().setSDA(DISPLAY_SDA_GPIO);
  bus().setSCL(DISPLAY_SCL_GPIO);
  bus().begin();
  bus().setClock(kI2cHz);
  g_last_probe_ms = millis() - kProbeMs; /* probe on the first loop */
}

void display_ui_loop(void) {
  const uint32_t now = millis();

  if (!g_ready) {
    if (now - g_last_probe_ms >= kProbeMs) {
      g_last_probe_ms = now;
      const uint8_t addr = find_display();
      bool ok = false;
      if (addr != 0u) {
        ok = init_display(addr);
      } else {
        scan_bus(now); /* show what *is* on the bus */
      }
      update_status([&](display_ui_status_t &st) {
        st.probes++;
        st.addr_active = ok ? addr : 0u;
        if (addr != 0u) {
          ok ? st.inits++ : st.init_failures++;
        }
        st.ready = ok;
      });
      if (ok) {
        g_addr = addr;
        g_ready = true;
        g_last_check_ms = now;
        scan_bus(now);
      }
    }
    if (!g_ready) {
      delay(100);
      return;
    }
  }

  /* Detect removal so a re-inserted module gets re-initialised. */
  if (now - g_last_check_ms >= kRecheckMs) {
    g_last_check_ms = now;
    if (!probe_addr(g_addr)) {
      g_ready = false;
      update_status([](display_ui_status_t &st) {
        st.ready = false;
        st.addr_active = 0u;
        st.lost++;
      });
      return;
    }
  }

  ui_snapshot_t s{};
  const bool have = ui_snapshot_read(&s);

  /* The trend samples at 1 Hz whether or not the frame rate slips. */
  if (now - g_last_trend_ms >= kTrendMs) {
    g_last_trend_ms = now;
    const bool ok = is_locked(s, have) && s.avg.rate_valid;
    trend_push(ok ? s.avg.rate_s_day : NAN);
  }

  render(s, have);

  const uint32_t period = g_period_req;
  const uint32_t spent = millis() - now;
  delay(spent < period ? period - spent : 1u);
}

#else /* !DISPLAY_SH1107 */

void display_ui_get_status(display_ui_status_t *out) {
  if (out) {
    *out = {};
  }
}
void display_ui_set_contrast(uint8_t level) {
  g_contrast_req = level == 0u ? 1u : level;
}

void display_ui_set_period_ms(uint32_t ms) {
  g_period_req = ms < 100u ? 100u : (ms > 5000u ? 5000u : ms);
}

void display_ui_get_settings(uint8_t *contrast, uint32_t *period_ms) {
  if (contrast) {
    *contrast = static_cast<uint8_t>(g_contrast_req);
  }
  if (period_ms) {
    *period_ms = g_period_req;
  }
}

void display_ui_setup(void) {}
void display_ui_loop(void) {}
void display_ui_set_contrast(uint8_t) {}
void display_ui_set_period_ms(uint32_t) {}
void display_ui_get_settings(uint8_t *contrast, uint32_t *period_ms) {
  if (contrast) {
    *contrast = 0;
  }
  if (period_ms) {
    *period_ms = 0;
  }
}

#endif
