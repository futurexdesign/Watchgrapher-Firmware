/**
 * @file render_main.cpp
 * @brief Host renderer for the OLED readout: runs the real
 *        {@code src/display_ui.cpp} and Adafruit GFX against a frame-buffer
 *        stand-in and writes each screen state as a PBM image.
 *
 * Usage: {@code oled_render --list} prints one {@code name<TAB>caption} line
 * per scenario; {@code oled_render <name> <out.pbm>} renders one scenario.
 * One process per scenario, because display_ui.cpp keeps its trend history
 * and frame state in file-local globals. Driven by render.sh.
 */

#include "display_ui.h"
#include "ui_snapshot.h"

#include <Adafruit_SH110X.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string_view>
#include <vector>

namespace {

/** Simulated clock; starts late enough that "now - 0" passes every timer. */
std::uint32_t g_now_ms = 10000u;

/** Snapshot the display reads; scenarios edit it between frames. */
ui_snapshot_t g_snap{};
/** False models "core 0 has not published yet". */
bool g_have = true;

/** One screen state to render. */
struct Scenario {
  /** File stem. */
  const char *name;
  /** One-line description for the contact sheet. */
  const char *caption;
  /** Fills the snapshot (called once, before the first frame). */
  std::function<void(ui_snapshot_t &)> setup;
  /**
   * Optional rate history: called once per simulated second with the time
   * since start; returning NaN leaves a gap (no lock for that second).
   * Without it the scenario renders a few frames only.
   */
  std::function<float(int)> trend = nullptr;
};

/**
 * Pico 2 PCM1808 build with the GPS PPS input, USB mic mode, idle. A scenario
 * that shows a readout sets {@code standalone}; USB mode shows the "USB AUDIO"
 * section instead.
 */
void base(ui_snapshot_t &s) {
  s = ui_snapshot_t{};
  s.lift_angle_deg = 52.0f;
  s.avg_s = 10.0f;
  s.pps_enabled = true;
  s.quartz.mode = DSP_QUARTZ_MODE_MECHANICAL;
  s.quartz.gate_s = 60.0f;
}

/** Running with a verified timebase. */
void running(ui_snapshot_t &s, bool standalone = false) {
  s.streaming = true;
  s.standalone = standalone;
  s.timebase_ok = true;
}

/** Locked at {@code vph} with typical averages. */
void locked(ui_snapshot_t &s, std::uint32_t vph, float rate, float amp,
            float be) {
  running(s, s.standalone);
  s.locked = true;
  s.vph = vph;
  s.events_seen = 1200u;
  s.avg.rate_s_day = rate;
  s.avg.amplitude_deg = amp;
  s.avg.beat_error_ms = be;
  s.avg.samples = 80u;
  s.avg.rate_valid = s.avg.amplitude_valid = s.avg.beat_error_valid = true;
}

/** A rate that settles from +40 to about +12 s/d after a regulator nudge. */
float settling(int t) {
  return 12.0f + 28.0f * std::exp(-static_cast<float>(t) / 25.0f) +
         0.8f * std::sin(static_cast<float>(t) / 4.0f);
}

/** Pico 2 W build: BLE advertising, no central. */
void ble_adv(ui_snapshot_t &s) {
  s.ble_built = true;
  s.ble_enabled = true;
}

/** Pico 2 W build: a central is connected. */
void ble_conn(ui_snapshot_t &s) {
  ble_adv(s);
  s.ble_connected = true;
}

/** Standalone motor mode, measuring, calibrated, with typical details. */
void motor(ui_snapshot_t &s, dsp_quartz_inhibition_t inhib) {
  running(s, true);
  s.events_seen = 30u;
  ui_quartz_t &q = s.quartz;
  q.mode = DSP_QUARTZ_MODE_MOTOR;
  q.state = DSP_QUARTZ_STATE_MEASURING;
  q.inhibition = inhib;
  q.calibrated = true;
  q.raw_rate_s_day = 3.123f;
  q.raw_rate_valid = true;
  q.rate_s_day = inhib == DSP_QUARTZ_INHIB_CORRECTED ? 0.273f : 3.123f;
  q.rate_valid = true;
  q.elapsed_s = 23.0f;
  q.gate_s = 60.0f;
  if (inhib == DSP_QUARTZ_INHIB_CORRECTED ||
      inhib == DSP_QUARTZ_INHIB_CORRECTION_PENDING) {
    q.inhibition_period_s = 60.0f;
    q.inhibition_period_valid = true;
  }
  if (inhib != DSP_QUARTZ_INHIB_NONE &&
      inhib != DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE &&
      inhib != DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION) {
    q.jump_us = 244.1f;
    q.jump_n = 2u;
  }
}

/** Standalone LCD (f0 found) or fork mode, measuring, calibrated. */
void tone(ui_snapshot_t &s, dsp_quartz_mode_t mode) {
  running(s, true);
  ui_quartz_t &q = s.quartz;
  q.mode = mode;
  q.state = DSP_QUARTZ_STATE_MEASURING;
  q.calibrated = true;
  q.rate_s_day = mode == DSP_QUARTZ_MODE_LCD ? 0.273f : -1.042f;
  q.rate_valid = true;
  q.ref_hz = mode == DSP_QUARTZ_MODE_LCD ? 64.0f : 360.0f;
  q.blocks = mode == DSP_QUARTZ_MODE_LCD ? 123u : 480u;
  q.blocks_valid = true;
  q.elapsed_s = mode == DSP_QUARTZ_MODE_LCD ? 61.0f : 120.0f;
}

const std::vector<Scenario> &scenarios() {
  static const std::vector<Scenario> list = {
      // --- Mechanical: header states -------------------------------------
      {"mech_01_starting", "Boot: core 0 has not published yet",
       [](ui_snapshot_t &s) {
         base(s);
         g_have = false;
       }},
      {"mech_02_no_stream", "USB mic mode, no host stream (dimmed)",
       [](ui_snapshot_t &s) { base(s); }},
      {"mech_03_stopped", "Standalone, run stopped (dimmed)",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
       }},
      {"mech_04_clock_check",
       "USB mic mode, capture started, rate not verified",
       [](ui_snapshot_t &s) {
         base(s);
         s.streaming = true;
       }},
      {"mech_05_listening", "USB mic mode, running, no ticks yet",
       [](ui_snapshot_t &s) {
         base(s);
         running(s);
       }},
      {"mech_06_locking", "Standalone, ticks seen, beat rate not locked",
       [](ui_snapshot_t &s) {
         base(s);
         running(s, true);
         s.events_seen = 14u;
       }},
      // --- Mechanical: locked ----------------------------------------------
      {"mech_07_locked_sa", "Standalone, locked 28,800 bph, GPS not locked",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         locked(s, 28800u, 12.3f, 285.0f, 0.4f);
       },
       settling},
      {"mech_08_locked_sa_gps", "Standalone, timebase disciplined by GPS",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         s.pps_locked = true;
         locked(s, 21600u, -4.7f, 262.0f, 1.2f);
       },
       [](int t) {
         return -4.7f + 1.5f * std::sin(static_cast<float>(t) / 9.0f);
       }},
      {"mech_09_trend_gaps",
       "Standalone, 18,000 bph, lock lost twice (trend gaps)",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         s.pps_enabled = false;
         locked(s, 18000u, 3.1f, 231.0f, 2.6f);
       },
       [](int t) {
         if ((t > 30 && t < 42) || (t > 80 && t < 86)) {
           return NAN;
         }
         return 3.0f + 6.0f * std::sin(static_cast<float>(t) / 14.0f);
       }},
      {"mech_10_fast_scale", "Standalone, 36,000 bph fast: trend rescales",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         s.pps_enabled = false;
         locked(s, 36000u, 63.0f, 301.0f, 0.1f);
       },
       [](int t) { return 20.0f + 0.4f * static_cast<float>(t); }},
      // --- Bluetooth badge (Pico 2 W) -------------------------------------
      {"ble_01_advertising", "Pico 2 W, SA: BLE advertising, GPS not locked",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         ble_adv(s);
         locked(s, 28800u, 12.3f, 285.0f, 0.4f);
       },
       settling},
      {"ble_02_connected_gps",
       "Pico 2 W, SA: BLE central connected, GPS locked",
       [](ui_snapshot_t &s) {
         base(s);
         ble_conn(s);
         s.standalone = true;
         s.pps_locked = true;
         locked(s, 28800u, 12.3f, 285.0f, 0.4f);
       },
       settling},
      {"ble_03_no_pps", "Pico 2 W, SA, no PPS: badge takes the corner",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         s.pps_enabled = false;
         ble_conn(s);
         locked(s, 28800u, 12.3f, 285.0f, 0.4f);
       },
       settling},
      {"ble_04_status_text", "Pico 2 W, USB: longest status beside both badges",
       [](ui_snapshot_t &s) {
         base(s);
         ble_adv(s);
         s.pps_locked = true;
         running(s);
         s.events_seen = 3u;
         g_have = true;
       }},
      // --- Quartz: motor --------------------------------------------------
      {"motor_01_no_stream", "Motor: USB mode, no stream",
       [](ui_snapshot_t &s) {
         base(s);
         s.quartz.mode = DSP_QUARTZ_MODE_MOTOR;
       }},
      {"motor_02_clock_check", "Motor: capture rate not verified",
       [](ui_snapshot_t &s) {
         base(s);
         s.standalone = true;
         s.streaming = true;
         s.quartz.mode = DSP_QUARTZ_MODE_MOTOR;
         s.quartz.state = DSP_QUARTZ_STATE_WAIT_TIMEBASE;
       }},
      {"motor_03_unsupported", "Motor: verified rate is not 48 kHz",
       [](ui_snapshot_t &s) {
         base(s);
         running(s, true);
         s.quartz.mode = DSP_QUARTZ_MODE_MOTOR;
         s.quartz.state = DSP_QUARTZ_STATE_UNSUPPORTED_RATE;
       }},
      {"motor_04_acquiring", "Motor: fewer than three pulses, uncalibrated",
       [](ui_snapshot_t &s) {
         base(s);
         running(s, true);
         s.quartz.mode = DSP_QUARTZ_MODE_MOTOR;
         s.quartz.state = DSP_QUARTZ_STATE_ACQUIRING;
         s.quartz.elapsed_s = 2.0f;
       }},
      {"motor_05_may_incl", "Motor: short gate, no jump yet",
       [](ui_snapshot_t &s) {
         base(s);
         motor(s, DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION);
       }},
      {"motor_06_need_one_more", "Motor: one inhibition jump seen",
       [](ui_snapshot_t &s) {
         base(s);
         motor(s, DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE);
       }},
      {"motor_07_correcting", "Motor: two jumps, waiting for next pulse",
       [](ui_snapshot_t &s) {
         base(s);
         motor(s, DSP_QUARTZ_INHIB_CORRECTION_PENDING);
       }},
      {"motor_08_corrected", "Motor: corrected rate, GPS locked",
       [](ui_snapshot_t &s) {
         base(s);
         s.pps_locked = true;
         motor(s, DSP_QUARTZ_INHIB_CORRECTED);
       }},
      {"motor_09_no_inhibition", "Motor: long gate done, no inhibition",
       [](ui_snapshot_t &s) {
         base(s);
         motor(s, DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE);
         s.quartz.state = DSP_QUARTZ_STATE_DONE;
         s.quartz.elapsed_s = 120.0f;
         s.quartz.gate_s = 120.0f;
       }},
      {"motor_10_hints", "Motor: CLIP and WEAK hints, uncalibrated",
       [](ui_snapshot_t &s) {
         base(s);
         motor(s, DSP_QUARTZ_INHIB_CORRECTED);
         s.quartz.calibrated = false;
         s.quartz.hint_clipped = true;
         s.quartz.hint_weak_signal = true;
       }},
      {"motor_11_long_jump", "Motor: 1 ms jump drops the @8192 suffix",
       [](ui_snapshot_t &s) {
         base(s);
         motor(s, DSP_QUARTZ_INHIB_CORRECTED);
         s.quartz.jump_us = 1220.7f;
         s.quartz.jump_n = 10u;
       }},
      {"motor_12_ble_unc", "Pico 2 W, USB tag + BT + GPS: UNCAL reads UNC",
       [](ui_snapshot_t &s) {
         base(s);
         ble_adv(s);
         motor(s, DSP_QUARTZ_INHIB_CORRECTED);
         s.standalone = false;
         s.quartz.calibrated = false;
       }},
      {"motor_13_ble_sa_uncal", "Pico 2 W, SA tag + BT + GPS: full UNCAL fits",
       [](ui_snapshot_t &s) {
         base(s);
         ble_conn(s);
         motor(s, DSP_QUARTZ_INHIB_CORRECTED);
         s.quartz.calibrated = false;
       }},
      // --- USB microphone mode: header as usual, "USB AUDIO" below ----------
      {"usb_01_locked", "USB mode, 28,800 bph detected, BT + GPS badges",
       [](ui_snapshot_t &s) {
         base(s);
         ble_conn(s);
         s.pps_locked = true;
         locked(s, 28800u, 12.3f, 285.0f, 0.4f);
       }},
      {"usb_02_listening", "USB mode, listening: status stays in the header",
       [](ui_snapshot_t &s) {
         base(s);
         running(s);
       }},
      {"usb_03_quartz_motor", "USB mode, motor mode: quartz header, USB Audio",
       [](ui_snapshot_t &s) {
         base(s);
         s.quartz.mode = DSP_QUARTZ_MODE_MOTOR;
         s.quartz.calibrated = true;
         running(s);
       }},
      // --- Quartz: LCD and fork -------------------------------------------
      {"lcd_01_searching", "LCD: pattern search, f0 not found yet",
       [](ui_snapshot_t &s) {
         base(s);
         running(s, true);
         s.quartz.mode = DSP_QUARTZ_MODE_LCD;
         s.quartz.state = DSP_QUARTZ_STATE_ACQUIRING;
         s.quartz.calibrated = true;
         s.quartz.elapsed_s = 1.0f;
       }},
      {"lcd_02_measuring", "LCD: measuring, oscillator-rate note",
       [](ui_snapshot_t &s) {
         base(s);
         tone(s, DSP_QUARTZ_MODE_LCD);
       }},
      {"lcd_03_clip", "LCD: RIGHT pickup clipped, uncalibrated",
       [](ui_snapshot_t &s) {
         base(s);
         tone(s, DSP_QUARTZ_MODE_LCD);
         s.quartz.calibrated = false;
         s.quartz.hint_clipped = true;
       }},
      {"lcd_04_ble_uncal", "Pico 2 W, LCD + BT + GPS: full UNCAL fits",
       [](ui_snapshot_t &s) {
         base(s);
         ble_conn(s);
         tone(s, DSP_QUARTZ_MODE_LCD);
         s.quartz.calibrated = false;
       }},
      {"fork_01_measuring", "Fork 360 Hz: measuring, GPS locked",
       [](ui_snapshot_t &s) {
         base(s);
         s.pps_locked = true;
         tone(s, DSP_QUARTZ_MODE_FORK);
       }},
      {"fork_02_clip", "Fork: LEFT pickup clipped",
       [](ui_snapshot_t &s) {
         base(s);
         tone(s, DSP_QUARTZ_MODE_FORK);
         s.quartz.hint_clipped = true;
       }},
  };
  return list;
}

/** Writes the panel as a plain PBM, with the contrast in a comment. */
bool write_pbm(const char *path) {
  const Adafruit_SH1107 *p = Adafruit_SH1107::last();
  if (p == nullptr) {
    std::fprintf(stderr, "no panel was created\n");
    return false;
  }
  std::FILE *f = std::fopen(path, "w");
  if (f == nullptr) {
    std::perror(path);
    return false;
  }
  std::fprintf(f, "P1\n# contrast %u\n128 128\n", p->contrast());
  for (int y = 0; y < 128; y++) {
    for (int x = 0; x < 128; x++) {
      std::fputc(p->lit(x, y) ? '1' : '0', f);
    }
    std::fputc('\n', f);
  }
  return std::fclose(f) == 0;
}

} // namespace

std::uint32_t millis() { return g_now_ms; }
void delay(std::uint32_t ms) { g_now_ms += ms; }

bool ui_snapshot_read(ui_snapshot_t *out) {
  if (!g_have) {
    return false;
  }
  *out = g_snap;
  return true;
}

int main(int argc, char **argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--list") {
    for (const Scenario &sc : scenarios()) {
      std::printf("%s\t%s\n", sc.name, sc.caption);
    }
    return 0;
  }
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s --list | <scenario> <out.pbm>\n", argv[0]);
    return 2;
  }
  const Scenario *sc = nullptr;
  for (const Scenario &c : scenarios()) {
    if (std::string_view(argv[1]) == c.name) {
      sc = &c;
    }
  }
  if (sc == nullptr) {
    std::fprintf(stderr, "unknown scenario %s\n", argv[1]);
    return 2;
  }
  sc->setup(g_snap);
  display_ui_setup();
  if (sc->trend) {
    /* 2 Hz frames, one trend column per second: fill all 120 columns. */
    const float final_rate = g_snap.avg.rate_s_day;
    for (int frame = 0; frame < 2 * 125; frame++) {
      const float r = sc->trend(frame / 2);
      g_snap.locked = !std::isnan(r);
      g_snap.avg.rate_s_day = std::isnan(r) ? 0.0f : r;
      display_ui_loop();
    }
    /* Last frame shows the scenario's headline values. */
    g_snap.locked = true;
    g_snap.avg.rate_s_day = final_rate;
    display_ui_loop();
  } else {
    for (int frame = 0; frame < 4; frame++) {
      display_ui_loop();
    }
  }
  return write_pbm(argv[2]) ? 0 : 1;
}
