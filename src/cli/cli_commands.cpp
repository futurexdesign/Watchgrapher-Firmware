/**
 * @file cli_commands.cpp
 * @brief Static command table: help, version, status, dsp → control_api.
 */

#include "cli/cli_commands.h"

#include "audio_source.h"
#include "board_config.h"
#include "cli/cli_core.h"
#include "control_api.h"
#include "diagnostics.h"
#include "usb_audio.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <cstdlib>

namespace Cli {
namespace {

#ifndef FW_VERSION
#define FW_VERSION "0.1.0-dev"
#endif

const __FlashStringHelper *streamStateName(stream_state_t s) {
  switch (s) {
  case STREAM_STATE_USB_UNCONFIGURED:
    return F("unconfigured");
  case STREAM_STATE_USB_CONFIGURED_IDLE:
    return F("idle");
  case STREAM_STATE_STREAM_PREFILL:
    return F("prefill");
  case STREAM_STATE_STREAM_ACTIVE:
    return F("active");
  case STREAM_STATE_USB_SUSPENDED:
    return F("suspended");
  case STREAM_STATE_STREAM_ERROR:
    return F("error");
  }
  return F("unknown");
}

/** Extended help for {@code help} / {@code ?}. */
void helpHelp(void) {
  Serial.println(F("  List all commands, or show details for one."));
  Serial.println(F("  Examples:"));
  Serial.println(F("    help"));
  Serial.println(F("    help dsp"));
  Serial.println(F("    ? status"));
}

/** Extended help for {@code version}. */
void helpVersion(void) {
  Serial.println(F("  Prints firmware version, board name, audio source, and"));
  Serial.println(F("  compile date/time. Alias: ver"));
}

/** Extended help for {@code status}. */
void helpStatus(void) {
  Serial.println(F("  USB mount, streaming, stream FSM state, audio source,"));
  Serial.println(F("  DSP master bypass flag, and uptime in milliseconds."));
}

/** Extended help for {@code diag}. */
void helpDiag(void) {
  Serial.println(F("  Enable or disable the ~2 s diagnostic status dump on CDC"));
  Serial.println(F("  (and UART when DIAG_UART=1). Counters always accumulate."));
  Serial.println(F("  Examples:"));
  Serial.println(F("    diag          # show on|off"));
  Serial.println(F("    diag on"));
  Serial.println(F("    diag off"));
}

void cmdDiag(int argc, char **argv) {
  if (argc >= 2 &&
      (equalsIgnoreCase(argv[1], "help") || equalsIgnoreCase(argv[1], "?"))) {
    printHelp("diag");
    return;
  }
  if (argc >= 2) {
    if (equalsIgnoreCase(argv[1], "on") || equalsIgnoreCase(argv[1], "1") ||
        equalsIgnoreCase(argv[1], "enable")) {
      diagnostics_set_print_enabled(true);
      Serial.println(F("CLI: diag prints=on"));
      return;
    }
    if (equalsIgnoreCase(argv[1], "off") || equalsIgnoreCase(argv[1], "0") ||
        equalsIgnoreCase(argv[1], "disable")) {
      diagnostics_set_print_enabled(false);
      Serial.println(F("CLI: diag prints=off"));
      return;
    }
    Serial.println(F("CLI: usage: diag [on|off]"));
    return;
  }
  Serial.print(F("CLI: diag prints="));
  Serial.println(diagnostics_print_enabled() ? F("on") : F("off"));
}

/**
 * Prints the full DSP subcommand reference (also used by {@code dsp help}).
 */
void helpDsp(void) {
  Serial.println(F("  DSP control plane (transport-agnostic; same API later via HID)."));
  Serial.println(F("  Analysis never changes PCM. Process mutates only when enabled"));
  Serial.println(F("  and master bypass is off. Order: notch iir fir gate gain agc lim."));
  Serial.println(F("  Subcommands:"));
  Serial.println(F("    list                 Blocks + enabled flags"));
  Serial.println(F("    status | bypass [on|off]  Master process bypass (bare=on)"));
  Serial.println(F("    on|off|enable|disable <block>"));
  Serial.println(F("    get|set <block> <param> [value]"));
  Serial.println(F("    events | beat|rate | metrics | timing|cpu | match | spectrum"));
  Serial.println(F("    errors [clear] | help [sub|block]"));
  Serial.println(F("  Shortcuts:"));
  Serial.println(F("    dsp set gain 12 | dsp on fir | dsp set fir preset 1"));
  Serial.println(F("  Try: dsp help fir_bank | dsp help matched_filter | dsp help spectrum"));
}

/**
 * Per-block description + params for {@code dsp help <block>}.
 * @return true if {@code name} matched a known block (or alias).
 */
bool helpDspBlock(const char *name) {
  auto id = control_api_block_from_name(name);
  if (id >= DSP_BLOCK_COUNT) {
    return false;
  }
  Serial.print(F("CLI: block "));
  Serial.println(control_api_block_name(id));
  switch (id) {
  case DSP_BLOCK_BYPASS:
    Serial.println(F("  Master gate for the process path (not a real DSP stage)."));
    Serial.println(F("  When on, PCM is passed through unchanged; analysis still runs."));
    Serial.println(F("  Usage: dsp bypass [on|off]   (bare bypass = on)"));
    break;
  case DSP_BLOCK_LEVEL_METER:
    Serial.println(F("  Analysis: tracks pre/post peak and RMS (see dsp metrics)."));
    Serial.println(F("  Always on at init; does not change PCM."));
    break;
  case DSP_BLOCK_GAIN:
    Serial.println(F("  Process: fixed make-up gain in dB applied to every sample."));
    Serial.println(F("  Params: db"));
    Serial.println(F("  Usage: dsp set gain 12   |  dsp set gain db 12  |  dsp get gain db"));
    break;
  case DSP_BLOCK_LIMITER:
    Serial.println(F("  Process: hard clamp at +/-1.0 FS before PCM24 quantize."));
    Serial.println(F("  Safety net when gain/AGC/filter boost ticks. No params."));
    Serial.println(F("  Usage: dsp on limiter  |  dsp off limiter"));
    break;
  case DSP_BLOCK_FILTER_BANK:
    Serial.println(F("  Process: cascaded one-pole HPF+LPF to isolate tick energy."));
    Serial.println(F("  Params: preset  0=off 1=wide(~160-16k) 2=med(~400-12k) 3=narrow(~800-8k)"));
    Serial.println(F("  Alias: filter. Usage: dsp set filter preset 2  |  dsp on filter"));
    break;
  case DSP_BLOCK_EVENT_DETECT:
    Serial.println(F("  Analysis: opens a window on impulsive peaks above noise*threshold."));
    Serial.println(F("  Feeds event_log and beat_rate. Does not change PCM."));
    Serial.println(F("  Params: threshold, preroll_ms, tail_ms"));
    Serial.println(F("  Alias: event. Usage: dsp set event threshold 6"));
    break;
  case DSP_BLOCK_BEAT_RATE:
    Serial.println(F("  Analysis: unlock intervals -> vph, rate s/d, beat error ms, amplitude."));
    Serial.println(F("  Common: 14400 16200 18000 19800 21600 25200 28800 36000 43200 57600"));
    Serial.println(F("  Params: liftAngle (10..90, default 51), vph (0=auto)"));
    Serial.println(F("  Alias: beat, rate, vph. Usage: dsp beat  |  dsp set beat liftAngle 52"));
    break;
  case DSP_BLOCK_NOISE_FLOOR:
    Serial.println(F("  Analysis: slow EMA of quiet-block RMS used by event_detect/gate."));
    Serial.println(F("  Alias: noise. No params. Visible in dsp metrics noise=..."));
    break;
  case DSP_BLOCK_EVENT_LOG:
    Serial.println(F("  Analysis: ring of recent event windows (dsp events)."));
    Serial.println(F("  Enable flag is informational; detector writes when event_detect on."));
    break;
  case DSP_BLOCK_SOFT_GATE:
    Serial.println(F("  Process: attenuates between ticks; opens fast on impulses."));
    Serial.println(F("  Preserves transients while lowering hiss between beats."));
    Serial.println(F("  Params: floor_db, attack_ms, release_ms"));
    Serial.println(F("  Alias: gate. Usage: dsp on gate  |  dsp set gate floor_db -12"));
    break;
  case DSP_BLOCK_NOTCH:
    Serial.println(F("  Process: narrow RBJ notch for mains hum (50 or 60 Hz)."));
    Serial.println(F("  Params: hz|freq (50 or 60 only)"));
    Serial.println(F("  Usage: dsp on notch  |  dsp set notch 60  |  dsp set notch hz 50"));
    break;
  case DSP_BLOCK_AGC:
    Serial.println(F("  Process: slow peak follower toward a target FS level."));
    Serial.println(F("  Adapts on energetic (tick) blocks only; clamped +/-12 dB."));
    Serial.println(F("  Params: target (0..1 FS), tau_s, gain_db (read-only)"));
    Serial.println(F("  Usage: dsp on agc  |  dsp set agc target 0.35  |  dsp set agc tau_s 1.5"));
    break;
  case DSP_BLOCK_MATCHED_FILTER:
    Serial.println(F("  Analysis: learns a 64-sample tick template; correlates each block."));
    Serial.println(F("  When learned, gates event opens on match score. Does not change PCM."));
    Serial.println(F("  Params: threshold (default 0.35), learn (0|1), score (read-only)"));
    Serial.println(F("  Alias: match. Usage: dsp on match  |  dsp match  |  dsp set match threshold 0.4"));
    break;
  case DSP_BLOCK_FIR_BANK:
    Serial.println(F("  Process: 64-tap Hamming FIR bandpass (linear phase)."));
    Serial.println(F("  Params: preset  0=off 1=wide 2=med 3=narrow (same bands as filter_bank)"));
    Serial.println(F("  Alias: fir. Usage: dsp set fir preset 1  |  dsp on fir  |  dsp bypass off"));
    break;
  case DSP_BLOCK_SPECTRUM:
    Serial.println(F("  Analysis: 512-point real FFT; reports top tonal peaks."));
    Serial.println(F("  Params: stride (1..16 FFT fills between updates). Read via dsp spectrum."));
    Serial.println(F("  Alias: fft, spec. Usage: dsp on spectrum  |  dsp spectrum"));
    break;
  default:
    break;
  }
  Serial.print(F("  enabled="));
  Serial.println(control_api_block_enabled(id) ? F("1") : F("0"));
  return true;
}

/**
 * Subcommand-specific help for {@code dsp help <sub>}.
 *
 * @param sub Subcommand name (may be nullptr for full help).
 */
void helpDspSub(const char *sub) {
  if (sub == nullptr || sub[0] == '\0' || equalsIgnoreCase(sub, "help")) {
    helpDsp();
    return;
  }
  if (helpDspBlock(sub)) {
    return;
  }
  if (equalsIgnoreCase(sub, "list")) {
    Serial.println(F("CLI: dsp list — print each block id and enabled=0|1"));
    Serial.println(F("  Tip: dsp help <block> describes what that block does."));
    return;
  }
  if (equalsIgnoreCase(sub, "status") || equalsIgnoreCase(sub, "bypass")) {
    Serial.println(F("CLI: dsp status              — show master bypass"));
    Serial.println(F("CLI: dsp bypass              — turn master bypass ON"));
    Serial.println(F("CLI: dsp bypass on|off       — set master bypass explicitly"));
    Serial.println(F("  When bypass is on, process path is identity; analysis may still run."));
    Serial.println(F("  Settings stay stored and apply again after 'dsp bypass off'."));
    return;
  }
  if (equalsIgnoreCase(sub, "enable") || equalsIgnoreCase(sub, "disable") ||
      equalsIgnoreCase(sub, "on") || equalsIgnoreCase(sub, "off")) {
    Serial.println(F("CLI: dsp on|enable <block>     — enable (process clears bypass)"));
    Serial.println(F("CLI: dsp off|disable <block>   — disable"));
    Serial.println(F("  Process: notch filter fir soft_gate gain agc limiter"));
    Serial.println(F("  Analysis: level_meter event_detect beat_rate noise match spectrum"));
    return;
  }
  if (equalsIgnoreCase(sub, "get") || equalsIgnoreCase(sub, "set")) {
    Serial.println(F("CLI: dsp get <block> <param>"));
    Serial.println(F("CLI: dsp set <block> <param> <value>"));
    Serial.println(F("  Shortcuts: dsp set gain 12  |  dsp set notch 60  |  dsp set fir preset 1"));
    Serial.println(F("  gain: db"));
    Serial.println(F("  filter_bank / fir_bank: preset (0..3)"));
    Serial.println(F("  event_detect: threshold, preroll_ms, tail_ms"));
    Serial.println(F("  notch: hz|freq (50|60)"));
    Serial.println(F("  agc: target, tau_s  (gain_db read-only)"));
    Serial.println(F("  soft_gate: floor_db, attack_ms, release_ms"));
    Serial.println(F("  matched_filter: threshold, learn"));
    Serial.println(F("  spectrum: stride"));
    Serial.println(F("  Per-block detail: dsp help <block>"));
    return;
  }
  if (equalsIgnoreCase(sub, "events")) {
    Serial.println(F("CLI: dsp events — recent analysis windows (pk/snr/energy/rise)"));
    return;
  }
  if (equalsIgnoreCase(sub, "beat") || equalsIgnoreCase(sub, "rate")) {
    Serial.println(F("CLI: dsp beat|rate — vph / rate s/d / beat error / amplitude"));
    Serial.println(F("  Common: 14400 16200 18000 19800 21600 25200 28800 36000 43200 57600"));
    return;
  }
  if (equalsIgnoreCase(sub, "metrics")) {
    Serial.println(F("CLI: dsp metrics — pre/post peak & RMS, noise, agc_db, match, clip, lim"));
    return;
  }
  if (equalsIgnoreCase(sub, "timing") || equalsIgnoreCase(sub, "cpu")) {
    Serial.println(F("CLI: dsp timing|cpu        — DSP block + main-loop CPU headroom"));
    Serial.println(F("CLI: dsp timing reset      — clear maxima / EMAs / busy window"));
    Serial.println(F("  block_us last/mean/max vs budget_us (n/source_hz); loop busy%."));
    Serial.println(F("  Also snapshots underrun/short-write/ring. PCM1808 path feeds DSP."));
    return;
  }
  if (equalsIgnoreCase(sub, "spectrum") || equalsIgnoreCase(sub, "fft") ||
      equalsIgnoreCase(sub, "match")) {
    Serial.println(F("CLI: dsp spectrum — top FFT peaks (enable spectrum first)"));
    Serial.println(F("CLI: dsp match    — matched-filter score / learned flag"));
    return;
  }
  if (equalsIgnoreCase(sub, "errors")) {
    Serial.println(F("CLI: dsp errors       — show auto-eject / fault log"));
    Serial.println(F("CLI: dsp errors clear — clear the log"));
    return;
  }
  Serial.print(F("CLI: no dsp help for '"));
  Serial.print(sub);
  Serial.println(F("' (try: dsp help, or dsp help gain)"));
}

void cmdHelp(int argc, char **argv) { printHelp(argc > 1 ? argv[1] : nullptr); }

void cmdVersion(int argc, char **argv) {
  if (argc >= 2 &&
      (equalsIgnoreCase(argv[1], "help") || equalsIgnoreCase(argv[1], "?"))) {
    printHelp("version");
    return;
  }
  Serial.print(F("CLI: timegrapher-stand fw "));
  Serial.print(F(FW_VERSION));
  Serial.print(F(" board="));
  Serial.print(F(BOARD_NAME));
  Serial.print(F(" source="));
  Serial.print(F(AUDIO_SOURCE_NAME));
  Serial.print(F(" built "));
  Serial.print(F(__DATE__));
  Serial.print(' ');
  Serial.println(F(__TIME__));
}

void cmdStatus(int argc, char **argv) {
  if (argc >= 2 &&
      (equalsIgnoreCase(argv[1], "help") || equalsIgnoreCase(argv[1], "?"))) {
    printHelp("status");
    return;
  }
  Serial.print(F("CLI: mounted="));
  Serial.print(TinyUSBDevice.mounted() ? F("true") : F("false"));
  Serial.print(F(" streaming="));
  Serial.print(usb_audio_streaming() ? F("true") : F("false"));
  Serial.print(F(" state="));
  Serial.print(streamStateName(audio_source_state()));
  Serial.print(F(" source="));
  Serial.print(F(AUDIO_SOURCE_NAME));
  Serial.print(F(" dsp_bypass="));
  Serial.print(control_api_get_bypass() ? F("true") : F("false"));
  Serial.print(F(" uptime_ms="));
  Serial.println(millis());
}

void cmdDsp(int argc, char **argv) {
  if (Serial.availableForWrite() < 128) {
    return;
  }

  /* dsp / dsp help / dsp help <sub> */
  if (argc < 2 || equalsIgnoreCase(argv[1], "help") ||
      equalsIgnoreCase(argv[1], "?")) {
    if (argc >= 3) {
      helpDspSub(argv[2]);
    } else if (argc < 2) {
      Serial.println(F("CLI: usage: dsp <subcommand>  (try: dsp help)"));
      helpDsp();
    } else {
      Serial.println(F("CLI: help dsp"));
      helpDsp();
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "list")) {
    Serial.println(F("CLI: blocks:"));
    for (int i = 0; i < static_cast<int>(DSP_BLOCK_COUNT); i++) {
      auto id = static_cast<dsp_block_id_t>(i);
      Serial.print(F("  "));
      Serial.print(control_api_block_name(id));
      Serial.print(F(" enabled="));
      Serial.println(control_api_block_enabled(id) ? F("1") : F("0"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "status") || equalsIgnoreCase(argv[1], "bypass")) {
    if (equalsIgnoreCase(argv[1], "bypass")) {
      /*
       * Bare "dsp bypass" means engage bypass (skip process path). Optional
       * on|off still supported; settings like gain stay stored for bypass off.
       */
      if (argc < 3) {
        control_api_set_bypass(true);
      } else {
        bool on = equalsIgnoreCase(argv[2], "on") || equalsIgnoreCase(argv[2], "1");
        bool off = equalsIgnoreCase(argv[2], "off") || equalsIgnoreCase(argv[2], "0");
        if (!on && !off) {
          helpDspSub("bypass");
          return;
        }
        control_api_set_bypass(on);
      }
    }
    Serial.print(F("CLI: dsp bypass="));
    Serial.println(control_api_get_bypass() ? F("on") : F("off"));
    return;
  }

  if (equalsIgnoreCase(argv[1], "enable") || equalsIgnoreCase(argv[1], "disable") ||
      equalsIgnoreCase(argv[1], "on") || equalsIgnoreCase(argv[1], "off")) {
    if (argc < 3) {
      helpDspSub("enable");
      return;
    }
    auto id = control_api_block_from_name(argv[2]);
    if (id >= DSP_BLOCK_COUNT) {
      Serial.println(F("CLI: unknown block (try: dsp list / dsp help <block>)"));
      return;
    }
    const bool turn_on = equalsIgnoreCase(argv[1], "enable") ||
                         equalsIgnoreCase(argv[1], "on");
    const bool ok = control_api_set_block_enabled(id, turn_on);
    Serial.println(ok ? F("CLI: ok") : F("CLI: failed (block not enableable)"));
    return;
  }

  if (equalsIgnoreCase(argv[1], "get")) {
    if (argc < 4) {
      helpDspSub("get");
      return;
    }
    auto id = control_api_block_from_name(argv[2]);
    float v = 0.f;
    if (!control_api_get_param(id, argv[3], &v)) {
      Serial.println(F("CLI: get failed (try: dsp help <block>)"));
      return;
    }
    Serial.print(F("CLI: "));
    Serial.print(argv[3]);
    Serial.print('=');
    Serial.println(v, 4);
    return;
  }

  if (equalsIgnoreCase(argv[1], "set")) {
    /*
     * Accepted forms:
     *   dsp set <block> <param> <value>   e.g. dsp set gain db 12
     *   dsp set gain <db>                 e.g. dsp set gain 12
     *   dsp set gain <db> db              e.g. dsp set gain 12 db
     *   dsp set notch <50|60>             e.g. dsp set notch 60
     */
    dsp_block_id_t id = DSP_BLOCK_COUNT;
    const char *param = nullptr;
    float v = 0.f;
    if (argc >= 5) {
      id = control_api_block_from_name(argv[2]);
      if (id == DSP_BLOCK_GAIN && equalsIgnoreCase(argv[4], "db") &&
          !equalsIgnoreCase(argv[3], "db")) {
        param = "db";
        v = strtof(argv[3], nullptr);
      } else {
        param = argv[3];
        v = strtof(argv[4], nullptr);
      }
    } else if (argc == 4 && equalsIgnoreCase(argv[2], "gain")) {
      id = DSP_BLOCK_GAIN;
      param = "db";
      v = strtof(argv[3], nullptr);
    } else if (argc == 4 &&
               (equalsIgnoreCase(argv[2], "notch") ||
                control_api_block_from_name(argv[2]) == DSP_BLOCK_NOTCH)) {
      id = DSP_BLOCK_NOTCH;
      param = "hz";
      v = strtof(argv[3], nullptr);
    } else if (argc == 5 &&
               (equalsIgnoreCase(argv[2], "fir") ||
                control_api_block_from_name(argv[2]) == DSP_BLOCK_FIR_BANK) &&
               equalsIgnoreCase(argv[3], "preset")) {
      id = DSP_BLOCK_FIR_BANK;
      param = "preset";
      v = strtof(argv[4], nullptr);
    } else {
      helpDspSub("set");
      return;
    }
    if (!control_api_set_param(id, param, v)) {
      Serial.println(F("CLI: set failed (try: dsp help <block>)"));
      return;
    }
    Serial.println(F("CLI: ok"));
    return;
  }

  if (equalsIgnoreCase(argv[1], "events")) {
    dsp_event_t ev[8];
    uint32_t n = control_api_copy_events(ev, 8);
    Serial.print(F("CLI: events n="));
    Serial.println(n);
    for (uint32_t i = 0; i < n; i++) {
      Serial.print(F("  start="));
      Serial.print(static_cast<uint32_t>(ev[i].start_sample));
      Serial.print(F(" peak="));
      Serial.print(static_cast<uint32_t>(ev[i].peak_sample));
      Serial.print(F(" unlock="));
      Serial.print(static_cast<uint32_t>(ev[i].unlock_sample));
      Serial.print(F(" drop="));
      Serial.print(static_cast<uint32_t>(ev[i].drop_sample));
      Serial.print(F(" lift="));
      Serial.print(ev[i].lift_samples);
      Serial.print(F(" dur="));
      Serial.print(ev[i].duration_samples);
      Serial.print(F(" rise="));
      Serial.print(ev[i].rise_samples);
      Serial.print(F(" pk="));
      Serial.print(ev[i].peak_abs, 4);
      Serial.print(F(" snr_db="));
      Serial.print(ev[i].snr_db, 1);
      Serial.print(F(" E="));
      Serial.print(ev[i].energy, 4);
      Serial.print(F(" conf="));
      Serial.println(ev[i].confidence, 3);
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "beat") || equalsIgnoreCase(argv[1], "rate")) {
    dsp_beat_rate_t b{};
    control_api_get_beat_rate(&b);
    Serial.print(F("CLI: beat vph_raw="));
    Serial.print(b.vph_raw, 1);
    Serial.print(F(" nearest="));
    Serial.print(b.vph_nearest);
    Serial.print(F(" locked="));
    Serial.print(b.locked ? F("1") : F("0"));
    Serial.print(F(" conf="));
    Serial.print(b.confidence, 3);
    Serial.print(F(" interval_samp="));
    Serial.print(b.interval_samples);
    Serial.print(F(" events="));
    Serial.print(b.events_seen);
    Serial.print(F(" intervals="));
    Serial.print(b.intervals_held);
    Serial.print(F(" src_hz="));
    Serial.println(b.source_hz);
    auto print_window = [](const __FlashStringHelper *label,
                           const dsp_beat_timing_window_t &w) {
      Serial.print(label);
      Serial.print(F(" rate="));
      if (w.rate_valid) {
        Serial.print(w.rate_s_day, 2);
      } else {
        Serial.print(F("n/a"));
      }
      Serial.print(F(" be_ms="));
      if (w.beat_error_valid) {
        Serial.print(w.beat_error_ms, 3);
      } else {
        Serial.print(F("n/a"));
      }
      Serial.print(F(" amp="));
      if (w.amplitude_valid) {
        Serial.print(w.amplitude_deg, 1);
      } else {
        Serial.print(F("n/a"));
      }
      Serial.print(F(" n="));
      Serial.println(w.samples);
    };
    print_window(F("CLI: beat instant"), b.instant);
    print_window(F("CLI: beat avg1s"), b.avg_1s);
    print_window(F("CLI: beat avg10s"), b.avg_10s);
    Serial.print(F("CLI: beat lift_angle="));
    Serial.print(b.lift_angle_deg, 1);
    Serial.print(F(" lift_ms="));
    Serial.println(b.lift_time_ms, 3);
    if (b.events_seen == 0) {
      Serial.println(
          F("CLI: tip: no events yet - check dsp metrics noise_floor / threshold"));
    } else if (b.intervals_held < 4) {
      Serial.println(F("CLI: tip: need >=4 accepted tick intervals (keep watch running)"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "metrics")) {
    dsp_metrics_t m{};
    control_api_get_metrics(&m);
    Serial.print(F("CLI: metrics pre_pk="));
    Serial.print(m.pre_peak, 4);
    Serial.print(F(" post_pk="));
    Serial.print(m.post_peak, 4);
    Serial.print(F(" pre_rms="));
    Serial.print(m.pre_rms, 4);
    Serial.print(F(" post_rms="));
    Serial.print(m.post_rms, 4);
    Serial.print(F(" noise="));
    Serial.print(m.noise_floor, 6);
    Serial.print(F(" agc_db="));
    Serial.print(m.agc_gain_db, 2);
    Serial.print(F(" match="));
    Serial.print(m.match_score, 3);
    Serial.print(F(" clip="));
    Serial.print(m.clip_count);
    Serial.print(F(" lim="));
    Serial.println(m.limiter_count);
    return;
  }

  if (equalsIgnoreCase(argv[1], "timing") || equalsIgnoreCase(argv[1], "cpu")) {
    if (argc > 2 && equalsIgnoreCase(argv[2], "reset")) {
      control_api_reset_timing();
      Serial.println(F("CLI: timing reset"));
      return;
    }
    dsp_timing_t t{};
    control_api_get_timing(&t);
    uint32_t headroom = 0;
    if (t.dsp_block_budget_us > 0) {
      if (t.dsp_block_us_max < t.dsp_block_budget_us) {
        headroom = static_cast<uint32_t>(
            (static_cast<uint64_t>(t.dsp_block_budget_us - t.dsp_block_us_max) *
             100ull) /
            static_cast<uint64_t>(t.dsp_block_budget_us));
      }
    }
    Serial.print(F("CLI: timing block_us last="));
    Serial.print(t.dsp_block_us_last);
    Serial.print(F(" mean="));
    Serial.print(t.dsp_block_us_mean);
    Serial.print(F(" max="));
    Serial.print(t.dsp_block_us_max);
    Serial.print(F(" budget="));
    Serial.print(t.dsp_block_budget_us);
    Serial.print(F(" (headroom_max≈"));
    Serial.print(headroom);
    Serial.print(F("%) n="));
    Serial.print(t.dsp_block_samples);
    Serial.print(F(" blocks="));
    Serial.println(t.dsp_blocks);
    Serial.print(F("         loop busy="));
    Serial.print(t.loop_busy_pct);
    Serial.print(F("% loop_us last="));
    Serial.print(t.loop_us_last);
    Serial.print(F(" mean="));
    Serial.print(t.loop_us_mean);
    Serial.print(F(" max="));
    Serial.print(t.loop_us_max);
    Serial.print(F(" idle_us/s="));
    Serial.println(t.idle_us_per_s);
    Serial.print(F("         und="));
    Serial.print(t.underruns);
    Serial.print(F(" short="));
    Serial.print(t.short_writes);
    Serial.print(F(" ring["));
    Serial.print(t.ring_min);
    Serial.print(',');
    Serial.print(t.ring_max);
    Serial.println(']');
    if (t.dsp_blocks == 0) {
      Serial.println(
          F("CLI: tip: dsp_graph_process not called yet (PCM1808 path feeds it)"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "spectrum") || equalsIgnoreCase(argv[1], "fft") ||
      equalsIgnoreCase(argv[1], "spec")) {
    dsp_spectrum_t s{};
    control_api_get_spectrum(&s);
    Serial.print(F("CLI: spectrum valid="));
    Serial.print(s.valid ? F("1") : F("0"));
    Serial.print(F(" fft_n="));
    Serial.print(s.fft_n);
    Serial.print(F(" src_hz="));
    Serial.print(s.source_hz);
    Serial.print(F(" n_peaks="));
    Serial.println(s.n_peaks);
    for (uint32_t i = 0; i < s.n_peaks; i++) {
      Serial.print(F("  "));
      Serial.print(s.peak_hz[i], 1);
      Serial.print(F(" Hz  mag="));
      Serial.println(s.peak_mag[i], 4);
    }
    if (!control_api_block_enabled(DSP_BLOCK_SPECTRUM)) {
      Serial.println(F("CLI: tip: dsp on spectrum"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "match") || equalsIgnoreCase(argv[1], "matched")) {
    dsp_matched_t m{};
    control_api_get_matched(&m);
    Serial.print(F("CLI: match score="));
    Serial.print(m.score, 3);
    Serial.print(F(" learned="));
    Serial.print(m.learned ? F("1") : F("0"));
    Serial.print(F(" tmpl_len="));
    Serial.print(m.template_len);
    Serial.print(F(" tmpl_peak="));
    Serial.println(m.template_peak, 4);
    if (!control_api_block_enabled(DSP_BLOCK_MATCHED_FILTER)) {
      Serial.println(F("CLI: tip: dsp on match"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "errors")) {
    if (argc > 2 && equalsIgnoreCase(argv[2], "clear")) {
      control_api_clear_errors();
      Serial.println(F("CLI: errors cleared"));
      return;
    }
    dsp_error_t err[8];
    uint32_t n = control_api_copy_errors(err, 8);
    Serial.print(F("CLI: errors n="));
    Serial.println(n);
    for (uint32_t i = 0; i < n; i++) {
      Serial.print(F("  block="));
      Serial.print(control_api_block_name(err[i].block));
      Serial.print(F(" reason="));
      Serial.print(static_cast<int>(err[i].reason));
      Serial.print(F(" t_ms="));
      Serial.println(err[i].uptime_ms);
    }
    return;
  }

  Serial.print(F("CLI: dsp: unknown subcommand '"));
  Serial.print(argv[1]);
  Serial.println(F("' (try: dsp help)"));
}

static const CliCommand kCommands[] = {
    {"help", "?", cmdHelp, F("Show command list or help <cmd>"), F("help [command]"),
     helpHelp},
    {"version", "ver", cmdVersion, F("Firmware / board / source"), F("version"),
     helpVersion},
    {"status", nullptr, cmdStatus, F("USB / stream / DSP summary"), F("status"),
     helpStatus},
    {"diag", nullptr, cmdDiag, F("Periodic diagnostic dump on|off"),
     F("diag [on|off]"), helpDiag},
    {"dsp", nullptr, cmdDsp, F("DSP control plane"),
     F("dsp list|bypass|on|off|get|set|events|beat|metrics|timing|match|spectrum|errors|help"),
     helpDsp},
};

} // namespace

void registerAllCommands() {
  constexpr size_t kCount = sizeof(kCommands) / sizeof(kCommands[0]);
  registerCommands(kCommands, kCount);
}

} // namespace Cli
