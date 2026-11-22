/**
 * @file cli_commands.cpp
 * @brief Static command table: help, version, status, device, run, dsp,
 *        timebase, quartz → control_api.
 */

#include "cli/cli_commands.h"

#include "audio_source.h"
#include "board_config.h"
#include "cli/cli_core.h"
#include "cli/device_format.h"
#include "control_api.h"
#include "diagnostics.h"
#include "display_ui.h"
#include "fw_identity.h"
#include "pcm1808_rate.h"
#include "timebase.h"
#include "usb_audio.h"

#include "cli/quartz_format.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <hardware/timer.h>

namespace Cli {
namespace {

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
  Serial.println(
      F("  Then the device mode / run state (details: device, run status)."));
  Serial.println(F("  Last line: quartz headline (details: quartz status)."));
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
  Serial.println(F("  DSP control plane (transport-agnostic; same API over WebHID)."));
  Serial.println(F("  Analysis never changes PCM. Process mutates only when enabled"));
  Serial.println(F("  and master bypass is off. Order: notch iir fir gate gain agc lim."));
  Serial.println(F("  Subcommands:"));
  Serial.println(F("    list                 Blocks + enabled flags"));
  Serial.println(F("    status | bypass [on|off]  Master process bypass (bare=on)"));
  Serial.println(F("    on|off|enable|disable <block>"));
  Serial.println(F("    get|set <block> <param> [value]"));
  Serial.println(F("    events | beat|rate | metrics | timing|cpu | match | spectrum"));
  Serial.println(
      F("    beats [n]            Per-beat records (n 1..16, default 8)"));
  Serial.println(F("    graph                Pipeline: process chain, analysis "
                   "taps, quartz"));
  Serial.println(F("    errors [clear] | help [sub|block]"));
  Serial.println(F("  Shortcuts:"));
  Serial.println(F("    dsp set gain 12 | dsp on fir | dsp set fir preset 1"));
  Serial.println(F("  Try: dsp help fir_bank | dsp help matched_filter | dsp help spectrum"));
  Serial.println(F("  Try: dsp help beats | dsp help graph"));
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
    Serial.println(F("  Params: db (-40..40)"));
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
    Serial.println(F("  Params: threshold (1..100, x noise), preroll_ms (0..100), tail_ms (1..200)"));
    Serial.println(F("  Alias: event. Usage: dsp set event threshold 6"));
    break;
  case DSP_BLOCK_BEAT_RATE:
    Serial.println(F("  Analysis: unlock intervals -> vph lock; least-squares fit of"));
    Serial.println(F("  sub-sample unlock times -> rate s/d, beat error ms; lift -> amplitude."));
    Serial.println(F("  Common: 14400 16200 18000 19800 21600 25200 28800 36000 43200 57600"));
    Serial.println(F("  Params: lift_angle (10..90, default 51), vph (0=auto),"));
    Serial.println(F("    window_s (2..30 fit window, default 6),"));
    Serial.println(F("    avg_s (1..60 reported average / display, default 10),"));
    Serial.println(F("    cal_ppm (-1000..1000 clock error; same as 'timebase set')"));
    Serial.println(F("  Alias: beat, rate, vph. Usage: dsp beat  |  dsp set beat lift_angle 52"));
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
    Serial.println(F("  Params: floor_db (-60..0), attack_ms (0.1..50), release_ms (1..500)"));
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
  if (equalsIgnoreCase(sub, "beats")) {
    Serial.println(
        F("CLI: dsp beats [n] — the last n per-beat records, newest first"));
    Serial.println(
        F("  n = 1..16 (default 8) of the 64 kept. One record per tick the"));
    Serial.println(
        F("  beat fit saw while locked (the raw data of a tick-tock chart)."));
    Serial.println(
        F("  seq     per-stream counter (restarts at 0 on a stream reset)"));
    Serial.println(F(
        "  grid    bumps whenever the fit restarts its grid (status started)"));
    Serial.println(
        F("  k       grid index in half-periods since the grid origin"));
    Serial.println(
        F("  p       parity k&1: 0 = tick, 1 = tock (stable within a grid)"));
    Serial.println(
        F("  u_us    arrival time minus the nominal grid (us); the even and"));
    Serial.println(F("          odd k clouds are 2x the beat error apart"));
    Serial.println(F(
        "  amp     amplitude (deg) of this beat, n/a if the lift time is bad"));
    Serial.println(F("  lift    lift time (ms)"));
    Serial.println(F(
        "  status  accepted | started | rejected (off-grid, not in the fit)"));
    return;
  }
  if (equalsIgnoreCase(sub, "graph")) {
    Serial.println(
        F("CLI: dsp graph — the DSP pipeline as text (same graph as the"));
    Serial.println(F("  WebHID hello caps.pipeline)"));
    Serial.println(F(
        "  chain      input > process blocks in order > pcm24 > usb. Process"));
    Serial.println(
        F("             blocks run only when enabled and bypass is off."));
    Serial.println(F(
        "  analysis   read-only taps of the raw input, run before the chain;"));
    Serial.println(F("             listed in execution order with '<-' naming "
                     "their inputs"));
    Serial.println(
        F("             (eventDetect feeds eventLog and beatRate)."));
    Serial.println(F("  quartz     the quartz engine reads the same capture "
                     "block (PCM1808"));
    Serial.println(F("             builds only; see 'help quartz')."));
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
  Serial.print(F(TG_BOARD_NAME));
  Serial.print(F(" source="));
  Serial.print(F(AUDIO_SOURCE_NAME));
  Serial.print(F(" built "));
  Serial.print(F(__DATE__));
  Serial.print(' ');
  Serial.println(F(__TIME__));
}

/* Defined with the quartz command below; status prints its headline. */
void printQuartzSummary(const dsp_quartz_t &q);

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
#if AUDIO_SOURCE == PCM1808
  const auto *d = diagnostics_get();
  Serial.print(F("CLI: pcm1808 rate="));
  Serial.print(pcm1808_rate_status_name(
      static_cast<pcm1808_rate_status_t>(d->pcm1808_rate_status)));
  Serial.print(F(" measured_hz="));
  Serial.print(static_cast<double>(d->pcm1808_frame_rate_mhz) / 1000.0, 3);
  Serial.print(F(" mclk_ratio="));
  Serial.print(d->pcm1808_mclk_ratio);
  Serial.print(F(" upsample="));
  Serial.print(d->pcm1808_upsample);
  dsp_beat_rate_t b{};
  control_api_get_beat_rate(&b);
  Serial.print(F(" dsp_hz="));
  Serial.print(b.source_hz);
  Serial.print(F(" timebase="));
  Serial.println(b.timebase_ok ? F("ok") : F("unverified"));
#endif
  device_status_t dev{};
  control_api_get_device(&dev);
  char devLine[160];
  Serial.print(F("CLI: "));
  Serial.println(formatDeviceStatus(dev, control_api_device_mode_name(dev.mode),
                                    devLine, sizeof(devLine)));
  dsp_quartz_t q{};
  control_api_get_quartz(&q);
  printQuartzSummary(q);
}

/** Scratch for {@code dsp beats} (file scope: core 0 has a 4 KB stack). */
dsp_beat_t g_cliBeats[kBeatsCliMax];

/**
 * {@code dsp beats [n]}: prints the newest {@code n} per-beat records,
 * newest first, as a table.
 *
 * @param argc Argument count after {@code dsp} ({@code argv[0]} is "beats").
 * @param argv Arguments.
 */
void printBeats(int argc, char **argv) {
  uint32_t n = kBeatsDefault;
  if (argc >= 2 && !parseBeatsCount(argv[1], n)) {
    Serial.println(F("CLI: ERR usage: dsp beats [n 1..16]"));
    return;
  }
  const uint32_t got = control_api_copy_beats(g_cliBeats, n);
  Serial.print(F("CLI: beats n="));
  Serial.print(got);
  Serial.print(F(" seq_next="));
  Serial.print(control_api_beat_seq());
  Serial.println(F(" (newest first)"));
  if (got == 0) {
    Serial.println(
        F("CLI: tip: no beats yet - needs a locked beat rate (dsp beat)"));
    return;
  }
  Serial.print(F("  "));
  Serial.println(kBeatsHeader);
  char row[96];
  for (uint32_t i = 0; i < got; i++) {
    Serial.print(F("  "));
    Serial.println(formatBeatRow(g_cliBeats[i], row, sizeof(row)));
  }
}

/**
 * {@code dsp graph}: prints the pipeline as text: the process chain, the
 * analysis taps in execution order, and the quartz engine on builds that
 * have it.
 */
void printGraph() {
  uint32_t count = 0;
  const auto *nodes = control_api_pipeline(&count);
  Serial.println(F("CLI: dsp graph (<- names a node's inputs)"));
  Serial.print(F("  chain: "));
  bool first = true;
  for (uint32_t i = 0; i < count; i++) {
    const auto kind = nodes[i].kind;
    if (kind != DSP_NODE_SOURCE && kind != DSP_NODE_PROCESS &&
        kind != DSP_NODE_SINK) {
      continue;
    }
    if (!first) {
      Serial.print(F(" > "));
    }
    Serial.print(nodes[i].key);
    first = false;
  }
  Serial.println();
  Serial.println(
      F("  analysis (read-only taps of input, run first, in order):"));
  char line[112];
  for (uint32_t i = 0; i < count; i++) {
    if (nodes[i].kind == DSP_NODE_ANALYSIS) {
      Serial.print(F("    "));
      Serial.println(formatPipelineNode(nodes[i], line, sizeof(line)));
    }
  }
  for (uint32_t i = 0; i < count; i++) {
    if (nodes[i].kind != DSP_NODE_QUARTZ) {
      continue;
    }
    if (control_api_quartz_supported()) {
      Serial.print(F("  quartz (engine, not in the audio path): "));
      Serial.println(formatPipelineNode(nodes[i], line, sizeof(line)));
    } else {
      Serial.println(F("  quartz: not built (needs the PCM1808 source)"));
    }
  }
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
    const char *value_text = nullptr;
    if (argc >= 5) {
      id = control_api_block_from_name(argv[2]);
      if (id == DSP_BLOCK_GAIN && equalsIgnoreCase(argv[4], "db") &&
          !equalsIgnoreCase(argv[3], "db")) {
        param = "db";
        value_text = argv[3];
      } else {
        param = argv[3];
        value_text = argv[4];
      }
    } else if (argc == 4 && control_api_block_from_name(argv[2]) == DSP_BLOCK_GAIN) {
      id = DSP_BLOCK_GAIN;
      param = "db";
      value_text = argv[3];
    } else if (argc == 4 && control_api_block_from_name(argv[2]) == DSP_BLOCK_NOTCH) {
      id = DSP_BLOCK_NOTCH;
      param = "hz";
      value_text = argv[3];
    } else {
      helpDspSub("set");
      return;
    }
    float v = 0.f;
    if (!parseFloat(value_text, v)) {
      Serial.print(F("CLI: not a number: "));
      Serial.println(value_text);
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
    Serial.print(b.source_hz);
    if (b.source_hz_measured > 0.f) {
      Serial.print(F(" measured_hz="));
      Serial.print(b.source_hz_measured, 3);
    }
    Serial.print(F(" timebase="));
    Serial.println(b.timebase_ok ? F("ok") : F("unverified"));
    Serial.print(F("CLI: beat fit_points="));
    Serial.print(b.rate_points);
    Serial.print(F(" jitter_us="));
    Serial.print(b.jitter_us, 1);
    Serial.print(F(" cal_ppm="));
    Serial.print(b.cal_ppm, 3);
    Serial.print(F(" cal_source="));
    Serial.println(b.cal_source ? b.cal_source : "none");
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
    {
      char label[32];
      snprintf(label, sizeof(label), "CLI: beat avg%gs", static_cast<double>(b.avg_s));
      Serial.print(label);
      print_window(F(""), b.avg_long);
    }
    Serial.print(F("CLI: beat lift_angle="));
    Serial.print(b.lift_angle_deg, 1);
    Serial.print(F(" lift_ms="));
    Serial.println(b.lift_time_ms, 3);
    if (!b.timebase_ok) {
      Serial.println(F("CLI: tip: capture rate not verified yet - no lock "
                       "(see status)"));
    } else if (b.events_seen == 0) {
      Serial.println(
          F("CLI: tip: no events yet - check dsp metrics noise_floor / threshold"));
    } else if (b.intervals_held < 4) {
      Serial.println(F("CLI: tip: need >=4 accepted tick intervals (keep watch running)"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "beats")) {
    printBeats(argc - 1, argv + 1);
    return;
  }

  if (equalsIgnoreCase(argv[1], "graph")) {
    printGraph();
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
    if (t.quartz_blocks > 0) {
      /* Shares the block budget above with the DSP graph. */
      Serial.print(F("         quartz_us last="));
      Serial.print(t.quartz_block_us_last);
      Serial.print(F(" mean="));
      Serial.print(t.quartz_block_us_mean);
      Serial.print(F(" max="));
      Serial.print(t.quartz_block_us_max);
      Serial.print(F(" blocks="));
      Serial.println(t.quartz_blocks);
    }
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

/** Extended help for {@code time}. */
void helpTime(void) {
  Serial.println(F("  Prints the device microsecond timer (time_us_64) at once."));
  Serial.println(F("  Used by scripts/calibrate_timebase.py to measure clock error"));
  Serial.println(F("  against the host's NTP-disciplined clock."));
}

/**
 * Prints {@code time_us_64()} as the first output of the handler so the
 * host's round-trip midpoint is as close as possible to the sample.
 */
void cmdTime(int, char **) {
  const uint64_t t = time_us_64();
  char buf[40];
  snprintf(buf, sizeof(buf), "CLI: time t_us=%llu",
           static_cast<unsigned long long>(t));
  Serial.println(buf);
}

/** Extended help for {@code timebase}. */
void helpTimebase(void) {
  Serial.println(F("  Sample-clock calibration (ppm, + = board clock fast). One value"));
  Serial.println(F("  corrects every capture path: all clocks share the 12 MHz crystal."));
  Serial.println(F("  Measure it with scripts/calibrate_timebase.py (NTP), or a GPS"));
  Serial.println(F("  PPS input on builds with -DPPS_GPIO=N (overrides while locked)."));
  Serial.println(F("  With -DGPS_UART_RX=N the NMEA fix gates PPS and dates calibrations."));
  Serial.println(F("  Examples:"));
  Serial.println(F("    timebase                         # show"));
  Serial.println(F("    timebase set -23.4 [manual|ntp] [unix_date] [duration_s]"));
  Serial.println(F("    timebase save                    # to flash (not while streaming)"));
  Serial.println(F("    timebase clear                   # 0 ppm in RAM (save to persist)"));
  Serial.println(F("    timebase gps                     # fix, UTC, DOP, "
                   "position, satellites"));
  Serial.println(F("    timebase log                     # calibration history "
                   "(24 entries)"));
  Serial.println(F("    timebase log clear               # erase the history "
                   "(writes flash)"));
  Serial.println(F("    timebase logdelta [ppm]          # show / set the PPS "
                   "log threshold"));
  Serial.println(F("  If an exact reference reads R s/d: ppm = -R / 0.0864."));
  Serial.println(F(
      "  Try: timebase help gps | timebase help log | timebase help logdelta"));
}

/** Help for {@code timebase gps}. */
void helpTimebaseGps(void) {
  Serial.println(
      F("CLI: timebase gps - GPS detail (builds with -DGPS_UART_RX=N)"));
  Serial.println(
      F("  Fix, UTC, DOPs (n/a = unknown) and position, then a satellite"));
  Serial.println(F(
      "  table: sys (GP GL GA BD QZ or ?), prn, el / az (deg, - = unknown),"));
  Serial.println(
      F("  snr (dB-Hz, - = not tracked) and used (in the position fix)."));
  Serial.println(
      F("  Satellites not heard for 5 s are dropped. Max 32 are listed."));
}

/** Help for {@code timebase log}. */
void helpTimebaseLog(void) {
  Serial.println(F("CLI: timebase log [clear] - calibration history"));
  Serial.println(
      F("  The last 24 calibrations, oldest first: date (UTC), ppm, source,"));
  Serial.println(
      F("  duration and why it was logged (save | delta | monthly)."));
  Serial.println(
      F("  'timebase log clear' erases it (flash write: refused while a USB"));
  Serial.println(F("  audio stream is open)."));
  Serial.println(
      F("  Policy: every successful 'timebase save' is logged ('save')."));
  Serial.println(
      F("  A GPS PPS measurement (locked >= 10 min, full 64-pulse window,"));
  Serial.println(
      F("  dated by GPS UTC) is logged when it differs from the previous"));
  Serial.println(
      F("  entry by >= logdelta ppm and at least a day later ('delta'), or"));
  Serial.println(
      F("  30 days after it ('monthly'); the first PPS entry is 'delta'."));
  Serial.println(
      F("  Only 24 are kept, oldest dropped. Entries are written to flash"));
  Serial.println(
      F("  only when no USB audio stream is open and no quartz measurement"));
  Serial.println(F("  is running; until then 'pending=yes'."));
}

/** Help for {@code timebase logdelta}. */
void helpTimebaseLogDelta(void) {
  Serial.println(F("CLI: timebase logdelta [ppm] - PPS log threshold"));
  Serial.println(
      F("  Show or set (0.05..10 ppm, default 0.2, saved to flash) the"));
  Serial.println(F("  change versus the previous log entry that makes a PPS"));
  Serial.println(
      F("  measurement worth logging as 'delta' (at least a day after the"));
  Serial.println(
      F("  previous entry; a 'monthly' entry is logged regardless). Setting"));
  Serial.println(
      F("  it writes flash: refused while a USB audio stream is open."));
}

/**
 * {@code timebase help [<sub>]}.
 *
 * @param sub {@code gps}, {@code log}, {@code logdelta}, or null for the
 *        overview.
 */
void helpTimebaseSub(const char *sub) {
  if (sub == nullptr || equalsIgnoreCase(sub, "help")) {
    printHelp("timebase");
  } else if (equalsIgnoreCase(sub, "gps")) {
    helpTimebaseGps();
  } else if (equalsIgnoreCase(sub, "log")) {
    helpTimebaseLog();
  } else if (equalsIgnoreCase(sub, "logdelta")) {
    helpTimebaseLogDelta();
  } else {
    Serial.print(F("CLI: no timebase help for '"));
    Serial.print(sub);
    Serial.println(F("' (try: timebase help)"));
  }
}

void printTimebase(void) {
  timebase_info_t tb{};
  timebase_get(&tb);
  char buf[200];
  snprintf(buf, sizeof(buf),
           "CLI: timebase ppm=%.3f source=%s cal_ppm=%.3f cal_source=%s "
           "date_unix=%lu duration_s=%lu saved=%s",
           static_cast<double>(tb.ppm), timebase_source_name(tb.source),
           static_cast<double>(tb.cal_ppm), timebase_source_name(tb.cal_source),
           static_cast<unsigned long>(tb.cal_date_unix),
           static_cast<unsigned long>(tb.cal_duration_s),
           tb.cal_saved ? "yes" : "no");
  Serial.println(buf);
  if (tb.pps_enabled) {
    snprintf(buf, sizeof(buf),
             "CLI: timebase pps locked=%s ppm=%.4f window=%lu good=%lu rejects=%lu",
             tb.pps_locked ? "yes" : "no", static_cast<double>(tb.pps_ppm),
             static_cast<unsigned long>(tb.pps_pulses),
             static_cast<unsigned long>(tb.pps_good),
             static_cast<unsigned long>(tb.pps_rejects));
    Serial.println(buf);
    if (tb.gps_enabled) {
      snprintf(buf, sizeof(buf), "CLI: timebase pps ignored_no_fix=%lu",
               static_cast<unsigned long>(tb.pps_no_fix));
      Serial.println(buf);
    }
  }
  if (tb.gps_enabled) {
    char age[16];
    if (tb.gps_age_ms == 0xFFFFFFFFu) {
      snprintf(age, sizeof(age), "never");
    } else {
      snprintf(age, sizeof(age), "%lums", static_cast<unsigned long>(tb.gps_age_ms));
    }
    snprintf(buf, sizeof(buf),
             "CLI: timebase gps fix=%s quality=%u sats=%u hdop=%.2f utc=%lu "
             "last_rmc=%s sentences=%lu errors=%lu overruns=%lu",
             tb.gps_fix ? "yes" : "no", tb.gps_quality, tb.gps_sats,
             static_cast<double>(tb.gps_hdop), static_cast<unsigned long>(tb.gps_utc_unix),
             age, static_cast<unsigned long>(tb.gps_sentences),
             static_cast<unsigned long>(tb.gps_errors),
             static_cast<unsigned long>(tb.gps_overruns));
    Serial.println(buf);
  }
  snprintf(buf, sizeof(buf),
           "CLI: timebase log logdelta_ppm=%.3f pending=%s (see: timebase log)",
           static_cast<double>(tb.log_delta_ppm),
           tb.log_pending ? "yes" : "no");
  Serial.println(buf);
}

/** Parses a non-negative decimal integer into {@code out}. */
bool parseU32(const char *s, uint32_t &out) {
  if (!s || !*s) {
    return false;
  }
  char *end = nullptr;
  const unsigned long v = strtoul(s, &end, 10);
  if (*end != '\0' || s[0] == '-') {
    return false;
  }
  out = static_cast<uint32_t>(v);
  return true;
}

/** Scratch for {@code timebase gps} / {@code log} (core 0 has a 4 KB stack). */
timebase_gps_t g_cliGps;
timebase_log_t g_cliLog;

/** {@code timebase gps}: fix, UTC, DOPs, position and the satellite table. */
void printTimebaseGps() {
  if (!control_api_gps_built()) {
    Serial.println(
        F("CLI: ERR GPS not built in (build with -DGPS_UART_RX=<gpio>)"));
    return;
  }
  control_api_get_gps(&g_cliGps);
  char line[128];
  Serial.print(F("CLI: timebase "));
  Serial.println(formatGpsFix(g_cliGps, line, sizeof(line)));
  Serial.print(F("CLI: timebase "));
  Serial.println(formatGpsDop(g_cliGps, line, sizeof(line)));
  Serial.print(F("CLI: timebase "));
  Serial.println(formatGpsPos(g_cliGps, line, sizeof(line)));
  Serial.print(F("CLI: timebase gps sats n="));
  Serial.println(g_cliGps.n_sats);
  if (g_cliGps.n_sats == 0) {
    return;
  }
  Serial.println(kSatHeader);
  for (uint8_t i = 0; i < g_cliGps.n_sats && i < TIMEBASE_GPS_MAX_SATS; i++) {
    Serial.println(formatSatRow(g_cliGps.sats[i], line, sizeof(line)));
  }
}

/** {@code timebase log}: the calibration history, oldest first. */
void printTimebaseLog() {
  control_api_timebase_log(&g_cliLog);
  char line[112];
  char utc[24];
  snprintf(
      line, sizeof(line),
      "CLI: timebase log entries=%lu/%u pending=%s logdelta_ppm=%.3f "
      "last_write=%s",
      static_cast<unsigned long>(g_cliLog.count), TIMEBASE_LOG_CAP,
      control_api_timebase_log_pending() ? "yes" : "no",
      static_cast<double>(control_api_timebase_log_delta_ppm()),
      formatUtc(control_api_timebase_log_last_write_unix(), utc, sizeof(utc)));
  Serial.println(line);
  if (g_cliLog.count == 0) {
    return;
  }
  Serial.println(kLogHeader);
  for (uint32_t i = 0; i < g_cliLog.count; i++) {
    const auto *e = timebase_log_get(&g_cliLog, i);
    if (e != nullptr) {
      Serial.println(formatLogRow(i, *e, line, sizeof(line)));
    }
  }
}

/** {@code timebase log clear}. */
void clearTimebaseLog() {
  switch (control_api_timebase_log_clear()) {
  case TIMEBASE_SAVE_OK:
    Serial.println(F("CLI: timebase log cleared"));
    break;
  case TIMEBASE_SAVE_BUSY:
    Serial.println(F("CLI: ERR timebase log clear refused while streaming "
                     "(close the audio stream first)"));
    break;
  case TIMEBASE_SAVE_FAILED:
    Serial.println(F("CLI: ERR timebase flash write failed"));
    break;
  }
}

/**
 * {@code timebase logdelta [ppm]}: shows or sets the PPS log threshold.
 *
 * @param argc Argument count after {@code timebase} ({@code argv[0]} is
 *        "logdelta").
 * @param argv Arguments.
 */
void cmdTimebaseLogDelta(int argc, char **argv) {
  if (argc < 2) {
    char line[80];
    snprintf(line, sizeof(line),
             "CLI: timebase logdelta_ppm=%.3f (range 0.05..10, default 0.2)",
             static_cast<double>(control_api_timebase_log_delta_ppm()));
    Serial.println(line);
    return;
  }
  float ppm = 0.f;
  if (!parseLogDelta(argv[1], ppm)) {
    Serial.println(F("CLI: ERR usage: timebase logdelta [ppm 0.05..10]"));
    return;
  }
  switch (control_api_set_timebase_log_delta_ppm(ppm)) {
  case TIMEBASE_LOGDELTA_OK:
    break;
  case TIMEBASE_LOGDELTA_BAD_PARAM:
    Serial.println(F("CLI: ERR usage: timebase logdelta [ppm 0.05..10]"));
    return;
  case TIMEBASE_LOGDELTA_BUSY:
    Serial.println(F("CLI: ERR timebase logdelta refused while streaming "
                     "(close the audio stream first)"));
    return;
  case TIMEBASE_LOGDELTA_FAILED:
    Serial.println(F("CLI: ERR timebase flash write failed"));
    return;
  }
  char line[64];
  snprintf(line, sizeof(line), "CLI: timebase logdelta_ppm=%.3f saved",
           static_cast<double>(control_api_timebase_log_delta_ppm()));
  Serial.println(line);
}

void cmdTimebase(int argc, char **argv) {
  if (argc < 2) {
    printTimebase();
    return;
  }
  if (equalsIgnoreCase(argv[1], "help") || equalsIgnoreCase(argv[1], "?")) {
    helpTimebaseSub(argc >= 3 ? argv[2] : nullptr);
    return;
  }
  if (equalsIgnoreCase(argv[1], "gps") || equalsIgnoreCase(argv[1], "log") ||
      equalsIgnoreCase(argv[1], "logdelta")) {
    /* "timebase <sub> help" prints that subcommand's help. */
    const bool gps = equalsIgnoreCase(argv[1], "gps");
    const bool log = equalsIgnoreCase(argv[1], "log");
    if (argc >= 3 &&
        (equalsIgnoreCase(argv[2], "help") || equalsIgnoreCase(argv[2], "?"))) {
      helpTimebaseSub(argv[1]);
    } else if (gps) {
      if (Serial.availableForWrite() >= 128) {
        printTimebaseGps();
      }
    } else if (log && argc >= 3) {
      if (equalsIgnoreCase(argv[2], "clear")) {
        clearTimebaseLog();
      } else {
        Serial.println(F("CLI: ERR usage: timebase log [clear]"));
      }
    } else if (log) {
      if (Serial.availableForWrite() >= 128) {
        printTimebaseLog();
      }
    } else {
      cmdTimebaseLogDelta(argc - 1, argv + 1);
    }
    return;
  }
  if (equalsIgnoreCase(argv[1], "set")) {
    float ppm = 0.f;
    timebase_source_t src = TIMEBASE_SRC_MANUAL;
    uint32_t date = 0, dur = 0;
    if (argc < 3 || !parseFloat(argv[2], ppm) ||
        (argc > 3 && !timebase_source_from_name(argv[3], &src)) ||
        (argc > 4 && !parseU32(argv[4], date)) ||
        (argc > 5 && !parseU32(argv[5], dur)) ||
        !timebase_set(ppm, src, date, dur)) {
      Serial.println(F("CLI: ERR usage: timebase set <ppm -1000..1000> "
                       "[manual|ntp] [unix_date] [duration_s]"));
      return;
    }
    printTimebase();
    return;
  }
  if (equalsIgnoreCase(argv[1], "save")) {
    switch (timebase_save()) {
    case TIMEBASE_SAVE_OK:
      Serial.println(F("CLI: timebase saved"));
      break;
    case TIMEBASE_SAVE_BUSY:
      Serial.println(F("CLI: ERR timebase save refused while streaming "
                       "(close the audio stream first)"));
      break;
    case TIMEBASE_SAVE_FAILED:
      Serial.println(F("CLI: ERR timebase flash write failed"));
      break;
    }
    return;
  }
  if (equalsIgnoreCase(argv[1], "clear")) {
    timebase_clear();
    printTimebase();
    return;
  }
  Serial.println(
      F("CLI: ERR usage: timebase [set|save|clear|gps|log|logdelta|help]"));
}

/** Extended help for {@code display}. */
void helpDisplay(void) {
  Serial.println(F("  OLED status as seen by core 1: pins, configured / answering"));
  Serial.println(F("  I2C address, init counts, and the addresses that ACKed in the"));
  Serial.println(F("  last bus scan. The display is probed every 2 s until found."));
  Serial.println(F("  display contrast <1-255>  brightness while streaming (not saved)"));
  Serial.println(F("  display period <ms>       frame period 100..5000 (default 500)"));
  Serial.println(F("  Unchanged frames are not re-sent (sent/skipped counts)."));
}

void cmdDisplay(int argc, char **argv) {
  if (argc >= 2 && (equalsIgnoreCase(argv[1], "help") || equalsIgnoreCase(argv[1], "?"))) {
    printHelp("display");
    return;
  }
  if (argc >= 3 && (equalsIgnoreCase(argv[1], "contrast") || equalsIgnoreCase(argv[1], "period"))) {
    char *end = nullptr;
    const long v = strtol(argv[2], &end, 0);
    const bool contrast = equalsIgnoreCase(argv[1], "contrast");
    if (end == argv[2] || *end != '\0' || (contrast && (v < 1 || v > 255)) ||
        (!contrast && (v < 100 || v > 5000))) {
      Serial.println(contrast ? F("CLI: error contrast must be 1..255")
                              : F("CLI: error period must be 100..5000 ms"));
      return;
    }
    if (contrast) {
      display_ui_set_contrast(static_cast<uint8_t>(v));
    } else {
      display_ui_set_period_ms(static_cast<uint32_t>(v));
    }
  }
  display_ui_status_t st{};
  display_ui_get_status(&st);
  if (!st.enabled) {
    Serial.println(F("CLI: display not running (built with DISPLAY_SH1107=0, or "
                     "core 1 has not started)"));
    return;
  }
  char addr[8];
  snprintf(addr, sizeof(addr), st.addr_active ? "0x%02x" : "none", st.addr_active);
  char buf[160];
  snprintf(buf, sizeof(buf),
           "CLI: display sh1107 sda=GP%u scl=GP%u i2c%u addr_cfg=0x%02x addr=%s "
           "ready=%s probes=%lu inits=%lu init_fail=%lu lost=%lu",
           st.sda_gpio, st.scl_gpio, st.i2c_port, st.addr_config, addr,
           st.ready ? "yes" : "no", static_cast<unsigned long>(st.probes),
           static_cast<unsigned long>(st.inits),
           static_cast<unsigned long>(st.init_failures), static_cast<unsigned long>(st.lost));
  Serial.println(buf);
  uint8_t contrast = 0;
  uint32_t period = 0;
  display_ui_get_settings(&contrast, &period);
  snprintf(buf, sizeof(buf), "CLI: display contrast=%u period_ms=%lu sent=%lu skipped=%lu",
           contrast, static_cast<unsigned long>(period),
           static_cast<unsigned long>(st.frames_sent),
           static_cast<unsigned long>(st.frames_skipped));
  Serial.println(buf);
  if (st.last_scan_ms == 0u) {
    Serial.println(F("CLI: display scan: not run yet"));
    return;
  }
  Serial.print(F("CLI: display scan: "));
  if (st.found_n == 0u) {
    Serial.println(F("no devices ACK (check SDA/SCL wiring, power, pull-ups)"));
    return;
  }
  for (uint8_t i = 0; i < st.found_n; i++) {
    snprintf(buf, sizeof(buf), "%s0x%02x", i ? " " : "", st.found[i]);
    Serial.print(buf);
  }
  Serial.println(st.addr_active ? "" : "  (no 0x3c/0x3d: wrong address or not an SH1107?)");
}

/* -------------------------------------------------------------------------
 * quartz / qz: quartz-watch timing (stepper motor, LCD, tuning fork).
 *
 * Formatting only: every value comes from control_api_get_quartz() and the
 * quartz parameter getters; ranges and allowed values come from the engine
 * constants through cli/quartz_format.h. On builds without the quartz
 * engine (ADC, SYNTHETIC) control_api reports mechanical and refuses every
 * setter; the commands then say so instead of printing zeros.
 * ---------------------------------------------------------------------- */

/** The quartz engine runs only on the PCM1808 capture path. */
constexpr bool kQuartzBuild = (AUDIO_SOURCE == PCM1808);

/**
 * Formats through a stack buffer and prints. The variadic wrappers below
 * are used instead of {@code Serial.printf}, whose declaration has no
 * format attribute, so the compiler checks every format string here.
 *
 * @param newline End the output with a line break.
 * @param fmt printf format.
 * @param ap Arguments.
 */
void quartzVPrint(bool newline, const char *fmt, va_list ap) {
  /* Fits the longest status line (mode=... reason="<45 chars>"). */
  char buf[224];
  vsnprintf(buf, sizeof(buf), fmt, ap);
  if (newline) {
    Serial.println(buf);
  } else {
    Serial.print(buf);
  }
}

/**
 * Prints one formatted line (see {@link quartzVPrint}).
 *
 * @param fmt printf format.
 */
[[gnu::format(printf, 1, 2)]] void quartzPrintLine(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  quartzVPrint(true, fmt, ap);
  va_end(ap);
}

/**
 * Prints formatted text without a line end (see {@link quartzVPrint}).
 *
 * @param fmt printf format.
 */
[[gnu::format(printf, 1, 2)]] void quartzPrint(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  quartzVPrint(false, fmt, ap);
  va_end(ap);
}

/**
 * @param s Token (may be null).
 * @return {@code true} for {@code help} or {@code ?}.
 */
[[nodiscard]] bool isHelpWord(const char *s) {
  return equalsIgnoreCase(s, "help") || equalsIgnoreCase(s, "?");
}

/** Says that this build has no quartz engine (setters and getters fail). */
void printQuartzUnavailable() {
  Serial.println(F("CLI: ERR quartz timing needs the PCM1808 source (this "
                   "build: " AUDIO_SOURCE_NAME ")"));
}

/** Prints the mode names as {@code mechanical|motor|lcd|fork}. */
void printQuartzModeNames() {
  for (int i = 0; i < static_cast<int>(DSP_QUARTZ_MODE_COUNT); i++) {
    if (i > 0) {
      Serial.print('|');
    }
    Serial.print(
        control_api_quartz_mode_name(static_cast<dsp_quartz_mode_t>(i)));
  }
}

/**
 * @param q Snapshot.
 * @return {@code true} while the engine is gated or off (mechanical,
 *         unverified or unsupported capture rate): the per-mode detail is
 *         all zero then and is not worth printing.
 */
[[nodiscard]] bool quartzGated(const dsp_quartz_t &q) {
  return q.mode == DSP_QUARTZ_MODE_MECHANICAL ||
         q.state == DSP_QUARTZ_STATE_IDLE ||
         q.state == DSP_QUARTZ_STATE_WAIT_TIMEBASE ||
         q.state == DSP_QUARTZ_STATE_UNSUPPORTED_RATE;
}

/**
 * Prints the one-line headline: mode, state, headline rate (tagged UNCAL
 * when no calibration exists), elapsed time against the gate, and what the
 * user may conclude (the §5 message for motor). Shared by
 * {@code quartz status} and the top-level {@code status}.
 *
 * Example:
 * {@code CLI: quartz motor measuring rate=-0.412 s/d t=73.0s gate=auto -
 * corrected for inhibition}
 *
 * @param q Snapshot from {@link control_api_get_quartz}.
 * @see cmdStatus, quartzStatus
 */
void printQuartzSummary(const dsp_quartz_t &q) {
  const char *mode = control_api_quartz_mode_name(q.mode);
  const char *state = control_api_quartz_state_name(q.state);
  if (quartzGated(q)) {
    /* The engine's mechanical reason ("mechanical mode") only repeats the
     * mode; say how to start instead. Other builds keep their reason. */
    const char *why = (kQuartzBuild && q.mode == DSP_QUARTZ_MODE_MECHANICAL)
                          ? "quartz timing off (quartz mode motor|lcd|fork)"
                          : q.reason;
    quartzPrintLine("CLI: quartz %s %s - %s", mode, state, why);
    return;
  }
  char rate[16];
  char gate[12];
  char tail[48] = "";
  quartzFormatRate(q.rate_valid, q.rate_s_day, rate, sizeof(rate));
  if (q.mode == DSP_QUARTZ_MODE_MOTOR) {
    snprintf(tail, sizeof(tail), " gate=%s",
             quartzFormatGate(q.gate_s, gate, sizeof(gate)));
  }
  /* What the number means: the §5 status for motor, else the reason the
   * state gives (or the nominal the rate refers to). */
  const char *msg = q.reason;
  char what[32];
  if (q.mode == DSP_QUARTZ_MODE_MOTOR && q.motor.valid) {
    msg = quartzInhibitionMessage(q.motor.inhibition);
  } else if (q.mode == DSP_QUARTZ_MODE_LCD && q.lcd.valid) {
    snprintf(what, sizeof(what), "oscillator rate, f0=%luHz",
             static_cast<unsigned long>(q.lcd.f0_hz));
    msg = what;
  } else if (q.mode == DSP_QUARTZ_MODE_FORK && q.fork.valid) {
    char hz[12];
    snprintf(what, sizeof(what), "f_nom=%sHz",
             quartzFormatHz(q.fork.f_nom_hz, hz, sizeof(hz)));
    msg = what;
  }
  const bool done = q.state == DSP_QUARTZ_STATE_DONE;
  quartzPrintLine("CLI: quartz %s %s rate=%s s/d%s t=%.1fs%s%s%s%s%s", mode,
                  state, rate, (q.rate_valid && !q.calibrated) ? " UNCAL" : "",
                  static_cast<double>(q.elapsed_s), tail,
                  msg[0] != '\0' ? " - " : "", msg, done ? "; " : "",
                  done ? q.reason : "");
}

/**
 * Motor detail of {@code quartz status}: pulses and step, both rates with
 * their uncalibrated values, the §5 status, inhibition period and jump
 * (with n at 8192 Hz), fit quality, detector levels, and the fault / clip /
 * weak-signal hints with the handoff §8 advice.
 *
 * @param m Motor sub-struct of the snapshot.
 */
void printQuartzMotor(const dsp_quartz_motor_t &m) {
  char a[16], b[16], c[16], d[16];
  quartzPrintLine("CLI: quartz motor pulses=%lu fit=%lu step_s=%g span_s=%.1f",
                  static_cast<unsigned long>(m.pulses),
                  static_cast<unsigned long>(m.fit_pulses),
                  static_cast<double>(m.step_s), static_cast<double>(m.span_s));
  quartzPrintLine(
      "CLI: quartz motor rate_with=%s (uncal=%s) rate_without=%s (uncal=%s) "
      "s/d",
      quartzFormatRate(m.valid, m.rate_with_s_day, a, sizeof(a)),
      quartzFormatRate(m.valid, m.rate_with_uncal_s_day, b, sizeof(b)),
      quartzFormatRate(m.valid, m.rate_without_s_day, c, sizeof(c)),
      quartzFormatRate(m.valid, m.rate_without_uncal_s_day, d, sizeof(d)));
  quartzPrintLine("CLI: quartz motor inhibition=%s \"%s\"",
                  control_api_quartz_inhibition_name(m.inhibition),
                  quartzInhibitionMessage(m.inhibition));
  if (m.inhibition_period_valid) {
    snprintf(a, sizeof(a), "%.1f", static_cast<double>(m.inhibition_period_s));
  } else {
    snprintf(a, sizeof(a), "n/a");
  }
  quartzPrintLine("CLI: quartz motor events=%lu cycles=%lu period_s=%s",
                  static_cast<unsigned long>(m.events),
                  static_cast<unsigned long>(m.cycles_used), a);
  if (m.events > 0u) {
    /* n = jump / 122.07 us: the pulses deleted at the 8192 Hz divider
     * (handoff §5: watchmakers use it). */
    quartzPrintLine("CLI: quartz motor jump_us=%.1f n=%lu jump_mean_us=%.1f "
                    "n_mean=%lu",
                    static_cast<double>(m.jump_last_us),
                    static_cast<unsigned long>(m.jump_last_n),
                    static_cast<double>(m.jump_mean_us),
                    static_cast<unsigned long>(m.jump_mean_n));
  }
  quartzPrintLine("CLI: quartz motor residual_rms_us=%.2f "
                  "polarity_offset_us=%.2f",
                  static_cast<double>(m.residual_rms_us),
                  static_cast<double>(m.polarity_offset_us));
  quartzPrintLine("CLI: quartz motor noise_sigma=%.0f threshold=%.0f "
                  "peak_snr=%.1f",
                  static_cast<double>(m.noise_sigma),
                  static_cast<double>(m.threshold),
                  static_cast<double>(m.peak_snr));
  quartzPrintLine("CLI: quartz motor extras=%lu clipped=%s clipped_pulses=%lu",
                  static_cast<unsigned long>(m.extra_pulses),
                  m.clipped ? "yes" : "no",
                  static_cast<unsigned long>(m.clipped_pulses));
  if (m.extra_pulses > 0u) {
    quartzPrintLine("CLI: quartz fault: %lu detection/correction pulse(s) "
                    "(log as fault indication)",
                    static_cast<unsigned long>(m.extra_pulses));
  }
  if (m.hint_clipped) {
    Serial.println(F("CLI: quartz hint: signal clipping: open JP1"));
  }
  if (m.hint_weak_signal) {
    Serial.println(F("CLI: quartz hint: weak signal: reposition the watch"));
  }
}

/**
 * LCD detail of {@code quartz status}: progress, pattern rate, harmonics,
 * blocks, rate and retries, plus the §8 reminder that the LCD cannot see
 * inhibition.
 *
 * @param l LCD sub-struct of the snapshot.
 */
void printQuartzLcd(const dsp_quartz_lcd_t &l) {
  char harm[32] = "-";
  size_t used = 0;
  for (uint8_t i = 0; i < l.harmonic_count && i < DSP_QUARTZ_MAX_HARMONICS &&
                      used < sizeof(harm);
       i++) {
    const int w = snprintf(harm + used, sizeof(harm) - used, "%s%u",
                           i ? "," : "", static_cast<unsigned>(l.harmonics[i]));
    if (w < 0) {
      break;
    }
    used += static_cast<size_t>(w);
  }
  char a[16], b[16];
  quartzPrintLine("CLI: quartz lcd status=%s f0_hz=%lu harmonics=%s blocks=%lu "
                  "retries=%lu",
                  control_api_quartz_lcd_status_name(l.status),
                  static_cast<unsigned long>(l.f0_hz), harm,
                  static_cast<unsigned long>(l.blocks),
                  static_cast<unsigned long>(l.retries));
  quartzPrintLine("CLI: quartz lcd rate=%s (uncal=%s) s/d clipped=%s",
                  quartzFormatRate(l.valid, l.rate_s_day, a, sizeof(a)),
                  quartzFormatRate(l.valid, l.rate_uncal_s_day, b, sizeof(b)),
                  l.clipped ? "yes" : "no");
  Serial.println(F("CLI: quartz note: the LCD cannot see inhibition: this is "
                   "the oscillator rate and may differ from motor mode"));
  if (l.clipped) {
    Serial.println(F("CLI: quartz hint: signal clipping: open JP2"));
  }
}

/**
 * Fork detail of {@code quartz status}: nominal frequency, blocks, rate.
 *
 * @param f Fork sub-struct of the snapshot.
 */
void printQuartzFork(const dsp_quartz_fork_t &f) {
  char hz[12], a[16], b[16];
  quartzPrintLine("CLI: quartz fork f_nom_hz=%s blocks=%lu rate=%s (uncal=%s) "
                  "s/d clipped=%s",
                  quartzFormatHz(f.f_nom_hz, hz, sizeof(hz)),
                  static_cast<unsigned long>(f.blocks),
                  quartzFormatRate(f.valid, f.rate_s_day, a, sizeof(a)),
                  quartzFormatRate(f.valid, f.rate_uncal_s_day, b, sizeof(b)),
                  f.clipped ? "yes" : "no");
  if (f.clipped) {
    Serial.println(F("CLI: quartz hint: signal clipping: open JP1"));
  }
}

/**
 * {@code quartz} / {@code quartz status}: headline first, then gating,
 * parameters, calibration and the active mode's detail.
 *
 * @param argc Unused.
 * @param argv Unused.
 */
void quartzStatus(int, char **) {
  dsp_quartz_t q{};
  control_api_get_quartz(&q);
  printQuartzSummary(q);
  char gate[12];
  quartzPrintLine("CLI: quartz mode=%s state=%s src_hz=%lu timebase=%s "
                  "elapsed_s=%.1f gate=%s loss_restarts=%lu reason=\"%s\"",
                  control_api_quartz_mode_name(q.mode),
                  control_api_quartz_state_name(q.state),
                  static_cast<unsigned long>(q.source_hz),
                  q.timebase_ok ? "ok" : "unverified",
                  static_cast<double>(q.elapsed_s),
                  quartzFormatGate(q.gate_s, gate, sizeof(gate)),
                  static_cast<unsigned long>(q.sample_loss_restarts), q.reason);
  if (!kQuartzBuild) {
    return; /* The headline already says why (no quartz engine). */
  }
  char hz[12];
  quartzPrintLine("CLI: quartz params ts=%s ksigma=%.1f fork_hz=%s",
                  control_api_quartz_ts_method_name(q.ts_method),
                  static_cast<double>(q.k_sigma),
                  quartzFormatHz(q.fork_hz, hz, sizeof(hz)));
  quartzPrintLine("CLI: quartz cal_ppm=%.3f cal_source=%s calibrated=%s",
                  static_cast<double>(q.cal_ppm), q.cal_source,
                  q.calibrated ? "yes" : "no");
  if (!q.calibrated && q.mode != DSP_QUARTZ_MODE_MECHANICAL) {
    Serial.println(F("CLI: quartz UNCALIBRATED: rates use the raw board clock "
                     "(see 'help timebase')"));
  }
  switch (q.state) {
  case DSP_QUARTZ_STATE_IDLE:
    Serial.println(F("CLI: tip: quartz mode motor|lcd|fork starts a "
                     "measurement"));
    return;
  case DSP_QUARTZ_STATE_WAIT_TIMEBASE:
    Serial.println(F("CLI: tip: waiting for the capture rate to be verified "
                     "(see status)"));
    return;
  case DSP_QUARTZ_STATE_UNSUPPORTED_RATE:
    quartzPrintLine("CLI: tip: quartz timing needs a verified %lu Hz capture",
                    static_cast<unsigned long>(quartz::engine::kRateHz));
    return;
  default:
    break;
  }
  switch (q.mode) {
  case DSP_QUARTZ_MODE_MOTOR:
    printQuartzMotor(q.motor);
    break;
  case DSP_QUARTZ_MODE_LCD:
    printQuartzLcd(q.lcd);
    break;
  case DSP_QUARTZ_MODE_FORK:
    printQuartzFork(q.fork);
    break;
  default:
    break;
  }
}

/**
 * Prints where the new mode listens and which jumper matters, right after a
 * mode change, so a terminal user places the watch correctly.
 *
 * @param mode The mode just selected.
 */
void printQuartzModeTip(dsp_quartz_mode_t mode) {
  switch (mode) {
  case DSP_QUARTZ_MODE_MOTOR:
    Serial.println(F("CLI: tip: motor reads LEFT (inductive coil). JP1 bridged "
                     "(x11) by default; open it on 'signal clipping'"));
    break;
  case DSP_QUARTZ_MODE_LCD:
    Serial.println(F("CLI: tip: lcd reads RIGHT (capacitive electrode), JP2 "
                     "open. Inhibition is invisible on the LCD"));
    break;
  case DSP_QUARTZ_MODE_FORK: {
    float hz = 0.f;
    char text[12] = "?";
    if (control_api_get_quartz_param("forkHz", &hz)) {
      quartzFormatHz(hz, text, sizeof(text));
    }
    quartzPrintLine("CLI: tip: fork reads LEFT at f_nom=%s Hz (quartz fork "
                    "<Hz>); allow at least 8 s",
                    text);
    break;
  }
  default:
    break;
  }
}

/**
 * {@code quartz mode [mechanical|motor|lcd|fork]}: shows or selects the
 * mode. Selecting the active mode again changes nothing (the engine treats
 * it as a no-op), which the reply says, pointing at {@code restart}.
 *
 * @param argc Argument count, {@code argv[0]} = "mode".
 * @param argv Arguments.
 */
void quartzMode(int argc, char **argv) {
  if (argc < 2) {
    Serial.print(F("CLI: quartz mode="));
    Serial.println(control_api_quartz_mode_name(control_api_get_quartz_mode()));
    return;
  }
  const dsp_quartz_mode_t want = control_api_quartz_mode_from_name(argv[1]);
  if (want >= DSP_QUARTZ_MODE_COUNT) {
    Serial.print(F("CLI: ERR unknown mode '"));
    Serial.print(argv[1]);
    Serial.print(F("' ("));
    printQuartzModeNames();
    Serial.println(')');
    return;
  }
  const dsp_quartz_mode_t before = control_api_get_quartz_mode();
  if (!control_api_set_quartz_mode(want)) {
    printQuartzUnavailable();
    return;
  }
  Serial.print(F("CLI: quartz mode="));
  Serial.print(control_api_quartz_mode_name(want));
  if (before == want) {
    Serial.println(F(" (unchanged; 'quartz restart' starts over)"));
    return;
  }
  Serial.println(want == DSP_QUARTZ_MODE_MECHANICAL ? F(" (quartz timing off)")
                                                    : F(" (new measurement)"));
  printQuartzModeTip(want);
}

/**
 * Reads a quartz parameter, reporting a build without the engine.
 *
 * @param name Engine parameter name.
 * @param out Receives the value.
 * @return {@code false} (after printing why) if it cannot be read.
 */
[[nodiscard]] bool quartzGetParam(const char *name, float &out) {
  if (!control_api_get_quartz_param(name, &out)) {
    printQuartzUnavailable();
    return false;
  }
  return true;
}

/**
 * Ends a get/set reply line: notes that a set restarted the measurement,
 * which the engine does only when the value changed and the active mode
 * uses the parameter.
 *
 * @param changed The set changed the stored value.
 * @param uses The mode that uses the parameter.
 */
void printQuartzSetNote(bool changed, dsp_quartz_mode_t uses) {
  if (changed && control_api_get_quartz_mode() == uses) {
    Serial.println(F(" (measurement restarted)"));
  } else {
    Serial.println();
  }
}

/** Prints the gate usage tail (range from the engine constants). */
void printQuartzGateUsage() {
  quartzPrint("quartz gate [0|auto|<%g..%g s>]",
              static_cast<double>(quartz::engine::kGateMinS),
              static_cast<double>(quartz::engine::kGateMaxS));
}

/** Prints the fork usage tail (values from {@link quartz::kForkFrequencies}).
 */
void printQuartzForkUsage() {
  char list[48];
  quartzPrint("quartz fork <%s> (Hz)",
              quartzFormatForkList(list, sizeof(list)));
}

/** Prints the timestamp-method usage tail (names from control_api). */
void printQuartzTsUsage() {
  Serial.print(F("quartz ts ["));
  for (int i = 0; i < static_cast<int>(DSP_QUARTZ_TS_COUNT); i++) {
    if (i > 0) {
      Serial.print('|');
    }
    Serial.print(control_api_quartz_ts_method_name(
        static_cast<dsp_quartz_ts_method_t>(i)));
  }
  Serial.print(']');
}

/** Prints the kSigma usage tail (range from the engine constants). */
void printQuartzKSigmaUsage() {
  quartzPrint("quartz ksigma <%g..%g>",
              static_cast<double>(quartz::engine::kKSigmaMin),
              static_cast<double>(quartz::engine::kKSigmaMax));
}

/**
 * Prints {@code CLI: ERR <what> - usage: <usage>}.
 *
 * @param what Why the command failed.
 * @param usage Prints the accepted form; may be null.
 */
void printQuartzError(const char *what, void (*usage)()) {
  Serial.print(F("CLI: ERR "));
  Serial.print(what);
  if (usage != nullptr) {
    Serial.print(F(" - usage: "));
    usage();
  }
  Serial.println();
}

/**
 * Writes one quartz parameter and, on refusal, prints the engine's reason
 * ({@link control_api_quartz_param_result_name}) with the accepted form.
 * The engine owns every range check; the CLI only explains its answer.
 *
 * @param name Engine parameter name.
 * @param value New value.
 * @param usage Prints the accepted form after the reason.
 * @return {@code true} if the engine accepted the value.
 */
[[nodiscard]] bool quartzSetParam(const char *name, float value,
                                  void (*usage)()) {
  const dsp_quartz_param_result_t r =
      control_api_set_quartz_param_ex(name, value);
  if (r == DSP_QUARTZ_PARAM_OK) {
    return true;
  }
  if (r == DSP_QUARTZ_PARAM_NO_ENGINE) {
    printQuartzUnavailable();
    return false;
  }
  char what[48];
  snprintf(what, sizeof(what), "%s %s", name,
           control_api_quartz_param_result_name(r));
  printQuartzError(what, usage);
  return false;
}

/**
 * The common get / set flow of the parameter subcommands: read the current
 * value; with an argument, parse it, let the engine accept or refuse it,
 * and read back what the engine stored (it snaps the fork to the exact list
 * value, e.g. 1024/3 for 341.33).
 *
 * @param name Engine parameter name.
 * @param argc Argument count, {@code argv[0]} = the subcommand.
 * @param argv Arguments.
 * @param parse Turns {@code argv[1]} into a value; false = not parseable.
 * @param usage Prints the accepted form.
 * @param before Receives the value before the call.
 * @param after Receives the value after the call.
 * @return {@code false} (after printing why) on any failure.
 */
[[nodiscard]] bool quartzGetSet(const char *name, int argc, char **argv,
                                bool (*parse)(const char *, float &),
                                void (*usage)(), float &before, float &after) {
  if (!quartzGetParam(name, before)) {
    return false;
  }
  after = before;
  if (argc < 2) {
    return true;
  }
  float v = 0.f;
  if (!parse(argv[1], v)) {
    char what[48];
    snprintf(what, sizeof(what), "bad %s value '%s'", name, argv[1]);
    printQuartzError(what, usage);
    return false;
  }
  return quartzSetParam(name, v, usage) && quartzGetParam(name, after);
}

/**
 * {@code quartz gate [0|auto|<seconds>]}: shows or sets the motor gate.
 *
 * @param argc Argument count, {@code argv[0]} = "gate".
 * @param argv Arguments.
 */
void quartzGate(int argc, char **argv) {
  float before = 0.f;
  float gate = 0.f;
  if (!quartzGetSet("gate", argc, argv, quartzParseGate, printQuartzGateUsage,
                    before, gate)) {
    return;
  }
  char text[12];
  Serial.print(F("CLI: quartz gate="));
  Serial.print(quartzFormatGate(gate, text, sizeof(text)));
  if (gate == quartz::engine::kGateUntilEvents) {
    Serial.print(F(" (until 2 inhibition events)"));
  }
  printQuartzSetNote(gate != before, DSP_QUARTZ_MODE_MOTOR);
}

/**
 * {@code quartz fork [<Hz>]}: shows or sets the tuning-fork nominal
 * frequency. The engine does the matching (within 0.01 Hz); a refusal
 * lists {@link quartz::kForkFrequencies}.
 *
 * @param argc Argument count, {@code argv[0]} = "fork".
 * @param argv Arguments.
 */
void quartzFork(int argc, char **argv) {
  float before = 0.f;
  float hz = 0.f;
  if (!quartzGetSet("forkHz", argc, argv, quartzParseNumber,
                    printQuartzForkUsage, before, hz)) {
    return;
  }
  char text[12];
  Serial.print(F("CLI: quartz fork_hz="));
  Serial.print(quartzFormatHz(hz, text, sizeof(text)));
  printQuartzSetNote(hz != before, DSP_QUARTZ_MODE_FORK);
}

/**
 * Parses a timestamp method: a name known to control_api, or its numeric
 * {@code tsMethod} value (the engine range-checks the number).
 *
 * @param s Token.
 * @param out Receives the parameter value.
 * @return {@code false} if {@code s} is neither.
 */
[[nodiscard]] bool parseQuartzTs(const char *s, float &out) {
  const dsp_quartz_ts_method_t m = control_api_quartz_ts_method_from_name(s);
  if (m < DSP_QUARTZ_TS_COUNT) {
    out = static_cast<float>(m);
    return true;
  }
  long v = 0;
  if (parseLong(s, v)) {
    out = static_cast<float>(v);
    return true;
  }
  return false;
}

/**
 * {@code quartz ts [threshold|xcorr]}: shows or sets the motor timestamp
 * method.
 *
 * @param argc Argument count, {@code argv[0]} = "ts".
 * @param argv Arguments.
 */
void quartzTs(int argc, char **argv) {
  float before = 0.f;
  float after = 0.f;
  if (!quartzGetSet("tsMethod", argc, argv, parseQuartzTs, printQuartzTsUsage,
                    before, after)) {
    return;
  }
  Serial.print(F("CLI: quartz ts="));
  Serial.print(control_api_quartz_ts_method_name(
      static_cast<dsp_quartz_ts_method_t>(static_cast<int>(after))));
  printQuartzSetNote(after != before, DSP_QUARTZ_MODE_MOTOR);
}

/**
 * {@code quartz ksigma [<n>]}: shows or sets the motor detection threshold
 * in noise sigmas.
 *
 * @param argc Argument count, {@code argv[0]} = "ksigma".
 * @param argv Arguments.
 */
void quartzKSigma(int argc, char **argv) {
  float before = 0.f;
  float k = 0.f;
  if (!quartzGetSet("kSigma", argc, argv, quartzParseNumber,
                    printQuartzKSigmaUsage, before, k)) {
    return;
  }
  Serial.print(F("CLI: quartz ksigma="));
  Serial.print(k, 1);
  printQuartzSetNote(k != before, DSP_QUARTZ_MODE_MOTOR);
}

/**
 * {@code quartz restart}: starts a new measurement in the active mode
 * (also after a motor gate froze the result).
 *
 * @param argc Unused.
 * @param argv Unused.
 */
void quartzRestart(int, char **) {
  if (!kQuartzBuild) {
    printQuartzUnavailable();
    return;
  }
  if (!control_api_quartz_restart()) {
    Serial.println(F("CLI: ERR nothing to restart in mechanical mode (quartz "
                     "mode motor|lcd|fork)"));
    return;
  }
  Serial.print(F("CLI: quartz restarted mode="));
  Serial.println(control_api_quartz_mode_name(control_api_get_quartz_mode()));
}

/**
 * {@code quartz pulses [n]}: the most recent motor pulses, oldest first so
 * the newest ends up next to the prompt.
 *
 * @param argc Argument count, {@code argv[0]} = "pulses".
 * @param argv Arguments.
 */
void quartzPulses(int argc, char **argv) {
  if (!kQuartzBuild) {
    printQuartzUnavailable();
    return;
  }
  uint32_t want = kQuartzPulsesDefault;
  if (argc >= 2 && (!parseU32(argv[1], want) || want == 0u ||
                    want > DSP_QUARTZ_PULSE_LOG_CAP)) {
    quartzPrintLine("CLI: ERR usage: quartz pulses [1..%u]",
                    static_cast<unsigned>(DSP_QUARTZ_PULSE_LOG_CAP));
    return;
  }
  const dsp_quartz_mode_t mode = control_api_get_quartz_mode();
  if (mode != DSP_QUARTZ_MODE_MOTOR) {
    Serial.print(F("CLI: ERR pulses need motor mode (mode="));
    Serial.print(control_api_quartz_mode_name(mode));
    Serial.println(')');
    return;
  }
  std::array<dsp_quartz_pulse_t, DSP_QUARTZ_PULSE_LOG_CAP> p{};
  const uint32_t n = control_api_copy_quartz_pulses(p.data(), want);
  quartzPrintLine("CLI: quartz pulses n=%lu (oldest first)",
                  static_cast<unsigned long>(n));
  for (uint32_t i = n; i-- > 0u;) {
    const dsp_quartz_pulse_t &e = p[i];
    quartzPrintLine("  seq=%lu t_s=%.6f pol=%c peak=%.0f snr=%.1f extras=%u "
                    "clipped=%u closed=%u",
                    static_cast<unsigned long>(e.sequence), e.t_s,
                    e.polarity < 0 ? '-' : '+', static_cast<double>(e.peak),
                    static_cast<double>(e.snr), static_cast<unsigned>(e.extras),
                    e.clipped ? 1u : 0u, e.closed ? 1u : 0u);
  }
}

/* Help text keeps one source line per printed line, like the existing
 * help functions above, so the terminal layout is visible in the source. */
// clang-format off
/** Overview for {@code help quartz} / {@code quartz help}. */
void helpQuartz(void) {
  Serial.println(F("  Quartz-watch timing on the PCM1808 build. Needs a verified"));
  Serial.println(F("  48 kHz capture. Rates in s/d, + = watch gains, corrected by"));
  Serial.println(F("  the timebase ppm (UNCAL until one is set: see 'help timebase')."));
  Serial.println(F("  Modes (one at a time; the mechanical beat path always runs):"));
  Serial.println(F("    mechanical  quartz timing off (default at boot)"));
  Serial.println(F("    motor       stepper-motor pulses, LEFT (inductive coil, JP1)"));
  Serial.println(F("    lcd         LCD drive, RIGHT (capacitive electrode, JP2 open)"));
  Serial.println(F("    fork        tuning-fork tone, LEFT (Accutron and similar)"));
  Serial.println(F("  Subcommands:"));
  Serial.println(F("    status                 Full readout, headline first (bare 'quartz')"));
  Serial.println(F("    mode [<mode>]          Show / select the mode"));
  Serial.println(F("    gate [0|auto|<s>]      Motor gate: auto = until 2 inhibition events"));
  Serial.println(F("    fork [<Hz>]            Tuning-fork nominal frequency"));
  Serial.println(F("    ts [threshold|xcorr]   Motor timestamp method"));
  Serial.println(F("    ksigma [<n>]           Motor detection threshold (noise sigmas)"));
  Serial.println(F("    restart                New measurement (alias: reset)"));
  Serial.println(F("    pulses [n]             Recent motor pulses"));
  Serial.println(F("    help [<sub>|inhibition|jumpers]"));
  Serial.println(F("  Motor: provisional rate after 3 pulses, refined with each pulse."));
  Serial.println(F("  Most quartz watches are trimmed by inhibition: every P s (10 or"));
  Serial.println(F("  60 s typical, up to 960 s) the IC deletes pulses. The headline is"));
  Serial.println(F("  the rate WITH inhibition, known only after 2 inhibition events;"));
  Serial.println(F("  before that, and with a short gate, the RAW rate is shown and the"));
  Serial.println(F("  status says so. The LCD cannot see inhibition, so LCD and motor"));
  Serial.println(F("  rates on the same watch can legitimately differ."));
  Serial.println(F("  Parameters and mode are not saved: the board boots in mechanical."));
  Serial.println(F("  Try: quartz help gate | quartz help inhibition | quartz help jumpers"));
}

/** Help for {@code quartz status}. */
void helpQuartzStatus(void) {
  Serial.println(F("CLI: quartz [status] - full quartz readout (also: qz)"));
  Serial.println(F("  Line 1 is the headline: mode, state, rate (UNCAL if no"));
  Serial.println(F("  calibration), signal time t, motor gate, and what the rate means."));
  Serial.println(F("  States: idle (mechanical), waitTimebase, unsupportedRate,"));
  Serial.println(F("  acquiring (no rate yet), measuring, done (motor gate reached)."));
  Serial.println(F("  Then: gating and reason, parameters, calibration, and the mode's"));
  Serial.println(F("  detail. Rates are calibrated; uncal= is against the board clock."));
  Serial.println(F("  motor: rate_with (headline, inhibition included), rate_without (raw"));
  Serial.println(F("    oscillator), inhibition status, events, period_s, jump_us with n"));
  Serial.println(F("    at 8192 Hz, residual_rms_us (jitter), polarity_offset_us,"));
  Serial.println(F("    noise_sigma / threshold (codes), peak_snr, extras, clipped."));
  Serial.println(F("    'fault:' extras = detection/correction pulses: a fault indication."));
  Serial.println(F("    'hint:' signal clipping: open JP1 | weak signal: reposition the watch"));
  Serial.println(F("  lcd: status, f0_hz (16/32/64/128), harmonics, blocks (0.5 s),"));
  Serial.println(F("    rate, retries (pattern searches restarted)."));
  Serial.println(F("  fork: f_nom_hz, blocks (0.25 s), rate."));
  Serial.println(F("  Example: qz"));
}

/** Help for {@code quartz mode}. */
void helpQuartzMode(void) {
  Serial.println(F("CLI: quartz mode [mechanical|motor|lcd|fork]"));
  Serial.println(F("  Show or select what is measured. A different mode starts a new"));
  Serial.println(F("  measurement; the same mode again changes nothing (use restart)."));
  Serial.println(F("    mechanical  quartz off (default); beat_rate runs in every mode"));
  Serial.println(F("    motor       analog quartz: stepper pulses on LEFT (inductive"));
  Serial.println(F("                coil, JP1); rate with and without inhibition"));
  Serial.println(F("    lcd         LCD digital: drive on RIGHT (capacitive, JP2 open);"));
  Serial.println(F("                oscillator rate, cannot see inhibition"));
  Serial.println(F("    fork        tuning fork on LEFT at 'quartz fork' Hz"));
  Serial.println(F("  The state stays waitTimebase / unsupportedRate until a 48 kHz"));
  Serial.println(F("  capture is verified. Builds without PCM1808 accept only mechanical."));
  Serial.println(F("  Examples:"));
  Serial.println(F("    quartz mode           # show"));
  Serial.println(F("    quartz mode motor"));
  Serial.println(F("    qz mode mechanical    # back to mechanical watches"));
}

/** Help for {@code quartz gate}; the numbers come from the engine. */
void helpQuartzGate(void) {
  Serial.println(F("CLI: quartz gate [0|auto|<seconds>]"));
  Serial.println(F("  Motor measurement length (signal time). Reaching it freezes the"));
  Serial.println(F("  result (state=done) until 'quartz restart'. Default: auto."));
  Serial.println(F("  Changing it in motor mode restarts the measurement."));
  Serial.println(F("    auto | 0  until 2 inhibition events: freezes as soon as the"));
  Serial.println(F("              rate with inhibition exists. A watch without"));
  Serial.println(F("              inhibition never freezes; it keeps refining."));
  quartzPrintLine("    %g..%-5g  seconds of signal (10 and 60 are usual); fewer",
                  static_cast<double>(quartz::engine::kGateMinS),
                  static_cast<double>(quartz::engine::kGateMaxS));
  Serial.println(F("              than 3 pulses in the gate give no rate."));
  Serial.println(F("  A gate shorter than 2 inhibition cycles can only report the RAW"));
  Serial.println(F("  rate (handoff 5): e.g. 10 s on a 60 s-inhibition watch reads"));
  Serial.println(F("  'may include inhibition'. Use auto, or more than 2 periods."));
  Serial.println(F("  'no inhibition detected' needs a gate of at least 2x the longest"));
  quartzPrintLine("  period (%g s = 2 x %g s) with no jump > %g us.",
                  quartz::engine::kNoJumpSpanS,
                  quartz::engine::kLongestInhibitionPeriodS,
                  quartz::engine::kJumpThresholdUs);
  Serial.println(F("  Examples:"));
  Serial.println(F("    quartz gate           # show"));
  Serial.println(F("    quartz gate auto"));
  Serial.println(F("    quartz gate 60"));
}

/** Help for {@code quartz fork}; the values come from the library. */
void helpQuartzFork(void) {
  char list[48];
  char def[12];
  Serial.println(F("CLI: quartz fork [<Hz>]"));
  Serial.println(F("  Nominal tuning-fork frequency for fork mode (the rate is the"));
  Serial.println(F("  error against it). Changing it in fork mode restarts."));
  quartzPrintLine("  Allowed: %s Hz (341.33 = exactly 1024/3). Default %s.",
                  quartzFormatForkList(list, sizeof(list)),
                  quartzFormatHz(quartz::engine::kDefaultForkHz, def,
                                 sizeof(def)));
  Serial.println(F("  360 Hz is the Accutron 214. Read the rate after at least 8 s."));
  Serial.println(F("  Examples:"));
  Serial.println(F("    quartz fork           # show"));
  Serial.println(F("    quartz fork 341.33"));
}

/** Help for {@code quartz ts}. */
void helpQuartzTs(void) {
  Serial.println(F("CLI: quartz ts [threshold|xcorr]   (alias: tsmethod; 0|1)"));
  Serial.println(F("  How each motor pulse's leading edge is timed (handoff 4)."));
  Serial.println(F("    threshold  50% crossing of the leading spike, interpolated"));
  Serial.println(F("               between samples (default; cheap)"));
  Serial.println(F("    xcorr      cross-correlation with a template fixed after the"));
  Serial.println(F("               first 4 pulses; lower error at high SNR and with"));
  Serial.println(F("               shape changes. t_s carries a constant offset that"));
  Serial.println(F("               cancels in rates."));
  Serial.println(F("  Changing it in motor mode restarts the measurement."));
  Serial.println(F("  Examples: quartz ts | quartz ts xcorr"));
}

/** Help for {@code quartz ksigma}; the range comes from the engine. */
void helpQuartzKSigma(void) {
  Serial.println(F("CLI: quartz ksigma [<n>]   (alias: k_sigma)"));
  Serial.println(F("  Motor pulse detection threshold, in noise sigmas after the"));
  quartzPrintLine("  400 Hz high-pass (threshold = n x noise_sigma). Range %g..%g, "
                  "default 8.",
                  static_cast<double>(quartz::engine::kKSigmaMin),
                  static_cast<double>(quartz::engine::kKSigmaMax));
  Serial.println(F("  Lower it for weak pulses (low peak_snr, missed pulses); raise it"));
  Serial.println(F("  if noise makes false pulses. Near 4, noise alone crosses it."));
  Serial.println(F("  Changing it in motor mode restarts the measurement."));
  Serial.println(F("  Examples: quartz ksigma | quartz ksigma 6"));
}

/** Help for {@code quartz restart}. */
void helpQuartzRestart(void) {
  Serial.println(F("CLI: quartz restart   (alias: reset)"));
  Serial.println(F("  Discards the current measurement and starts a new one in the"));
  Serial.println(F("  active mode, also after a motor gate froze the result (done)."));
  Serial.println(F("  Use it after moving the watch. Fails in mechanical mode."));
}

/** Help for {@code quartz pulses}. */
void helpQuartzPulses(void) {
  Serial.println(F("CLI: quartz pulses [n]"));
  quartzPrintLine("  The last n motor pulses (1..%u, default %lu), oldest first. Motor "
                  "mode only.",
                  static_cast<unsigned>(DSP_QUARTZ_PULSE_LOG_CAP),
                  static_cast<unsigned long>(kQuartzPulsesDefault));
  Serial.println(F("    seq      pulse number since the measurement start"));
  Serial.println(F("    t_s      leading-edge time since the start (s, sub-sample)"));
  Serial.println(F("    pol      polarity of the leading spike (alternates)"));
  Serial.println(F("    peak     leading-spike peak after the high-pass (codes)"));
  Serial.println(F("    snr      peak / noise sigma"));
  Serial.println(F("    extras   detection/correction bursts in its group (a fault)"));
  Serial.println(F("    clipped  a raw code hit the rail (open JP1)"));
  Serial.println(F("    closed   its 50 ms quiet gap has passed (extras final)"));
  Serial.println(F("  Example: quartz pulses 4"));
}

/** Help topic {@code quartz help inhibition}: the handoff §5 table. */
void helpQuartzInhibition(void) {
  Serial.println(F("CLI: quartz inhibition status (motor; handoff 5)"));
  Serial.println(F("  rate_without = raw oscillator (slope within a segment)."));
  Serial.println(F("  rate_with    = mean over whole inhibition cycles: what the wearer"));
  Serial.println(F("                 sees, and the headline once it exists."));
  Serial.println(F("    none                    fewer than 3 pulses: no rate"));
  Serial.println(F("    may_include_inhibition  no jump, short gate: RAW rate; use a"));
  Serial.println(F("                            longer gate"));
  Serial.println(F("    need_one_more_cycle     one jump: RAW rate; one more cycle"));
  Serial.println(F("    correction_pending      two jumps: corrected at the next pulse"));
  Serial.println(F("                            of the right polarity"));
  Serial.println(F("    corrected               rate with inhibition; period and jump"));
  Serial.println(F("                            are details"));
  quartzPrintLine("    no_jump_long_gate       no jump in a gate of %g s or more:",
                  quartz::engine::kNoJumpSpanS);
  Serial.println(F("                            no inhibition detected"));
  quartzPrintLine("  A jump is a step interval more than %g us too long. n = jump / "
                  "122.07 us",
                  quartz::engine::kJumpThresholdUs);
  Serial.println(F("  is the number of pulses deleted at 8192 Hz (watchmakers use it)."));
}

/** Help topic {@code quartz help jumpers}: JP1 / JP2 guidance (§2, §8). */
void helpQuartzJumpers(void) {
  Serial.println(F("CLI: quartz jumpers"));
  Serial.println(F("  JP1  LEFT (inductive coil) gain, used by motor and fork."));
  Serial.println(F("       Bridged (default) = x11 for 20-1000 uV pulses; clips near"));
  Serial.println(F("       1.4 mV EMF. Open it (x1) on 'signal clipping: open JP1'."));
  Serial.println(F("  JP2  RIGHT (capacitive electrode) gain, used by lcd. Keep it OPEN"));
  Serial.println(F("       (x1): bridged clips above ~0.33 pF LCD coupling."));
  Serial.println(F("  'weak signal: reposition the watch': median peak_snr below"));
  quartzPrintLine("       %g (about 30 uV EMF). The rate still converges, but each "
                  "pulse",
                  static_cast<double>(quartz::engine::kWeakSnr));
  Serial.println(F("       jitters by more than 2 us; move the movement over the coil."));
}

// clang-format on

/** One {@code quartz} subcommand or help topic. */
struct QuartzSubcommand {
  const char *name;                   ///< Subcommand word
  const char *alias;                  ///< Alternative word, or nullptr
  void (*run)(int argc, char **argv); ///< Handler; nullptr = help topic only
  void (*help)(void);                 ///< {@code quartz help <name>}
};

/** {@code quartz} subcommands and help topics, in help order. */
constexpr std::array<QuartzSubcommand, 10> kQuartzSubcommands{{
    {"status", nullptr, quartzStatus, helpQuartzStatus},
    {"mode", nullptr, quartzMode, helpQuartzMode},
    {"gate", nullptr, quartzGate, helpQuartzGate},
    {"fork", nullptr, quartzFork, helpQuartzFork},
    {"ts", "tsmethod", quartzTs, helpQuartzTs},
    {"ksigma", "k_sigma", quartzKSigma, helpQuartzKSigma},
    {"restart", "reset", quartzRestart, helpQuartzRestart},
    {"pulses", nullptr, quartzPulses, helpQuartzPulses},
    {"inhibition", nullptr, nullptr, helpQuartzInhibition},
    {"jumpers", "jp", nullptr, helpQuartzJumpers},
}};

/**
 * @param word Subcommand or alias (any case).
 * @return The entry, or {@code nullptr}.
 */
[[nodiscard]] const QuartzSubcommand *findQuartzSubcommand(const char *word) {
  for (const auto &s : kQuartzSubcommands) {
    if (equalsIgnoreCase(word, s.name) ||
        (s.alias != nullptr && equalsIgnoreCase(word, s.alias))) {
      return &s;
    }
  }
  return nullptr;
}

/**
 * {@code quartz help [<sub>]}: the overview, or one subcommand's help.
 *
 * @param sub Subcommand / topic, or nullptr for the overview.
 */
void helpQuartzSub(const char *sub) {
  if (sub == nullptr || isHelpWord(sub)) {
    printHelp("quartz");
    return;
  }
  const auto *s = findQuartzSubcommand(sub);
  if (s == nullptr) {
    Serial.print(F("CLI: no quartz help for '"));
    Serial.print(sub);
    Serial.println(F("' (try: quartz help)"));
    return;
  }
  s->help();
}

/**
 * {@code quartz} / {@code qz}: dispatches to {@link kQuartzSubcommands}.
 * Bare {@code quartz} is {@code quartz status}; {@code quartz help [sub]}
 * and {@code quartz <sub> help} print help.
 *
 * @param argc Argument count, {@code argv[0]} = "quartz" or "qz".
 * @param argv Arguments.
 */
void cmdQuartz(int argc, char **argv) {
  if (Serial.availableForWrite() < 128) {
    return;
  }
  if (argc < 2) {
    quartzStatus(argc, argv);
    return;
  }
  if (isHelpWord(argv[1])) {
    helpQuartzSub(argc >= 3 ? argv[2] : nullptr);
    return;
  }
  const auto *s = findQuartzSubcommand(argv[1]);
  if (s == nullptr) {
    Serial.print(F("CLI: quartz: unknown subcommand '"));
    Serial.print(argv[1]);
    Serial.println(F("' (try: quartz help)"));
    return;
  }
  if (s->run == nullptr || (argc >= 3 && isHelpWord(argv[2]))) {
    s->help();
    return;
  }
  s->run(argc - 1, argv + 1);
}

/* -------------------------------------------------------------------------
 * device / run: operating mode and capture run control.
 *
 * Formatting only: state comes from control_api_get_device() and the
 * setters from control_api_*; the line layout is in cli/device_format.h.
 * ---------------------------------------------------------------------- */

// clang-format off
/** Extended help for {@code device}. */
void helpDevice(void) {
  Serial.println(F("  Operating mode of the device. Two modalities:"));
  Serial.println(F("    usbmic      USB microphone: a PC timegrapher program records the"));
  Serial.println(F("                audio, and capture + analysis run while it streams."));
  Serial.println(F("                The default."));
  Serial.println(F("    standalone  the device is the timegrapher: capture and analysis"));
  Serial.println(F("                run on their own with no host stream, started at boot"));
  Serial.println(F("                and when this mode is selected (results: display, CLI,"));
  Serial.println(F("                WebHID). The USB audio feed is silent if a host opens"));
  Serial.println(F("                the stream anyway."));
  Serial.println(F("  The mode is saved to flash and kept across reboots."));
  Serial.println(F("  Changing it writes flash, so it is refused while a USB audio"));
  Serial.println(F("  stream is open (close the host's recording first)."));
  Serial.println(F("  Examples:"));
  Serial.println(F("    device                  # status (also: device status)"));
  Serial.println(F("    device mode             # show the mode"));
  Serial.println(F("    device mode usbmic"));
  Serial.println(F("    device mode standalone"));
  Serial.println(F("  See also: help run"));
}

/** Prints the {@code device} status line. */
void printDeviceStatus() {
  device_status_t st{};
  control_api_get_device(&st);
  char line[160];
  Serial.print(F("CLI: "));
  Serial.println(formatDeviceStatus(st, control_api_device_mode_name(st.mode),
                                    line, sizeof(line)));
}

/**
 * {@code device} / {@code device status} / {@code device mode [<mode>]}.
 *
 * @param argc Argument count, {@code argv[0]} = "device".
 * @param argv Arguments.
 */
void cmdDevice(int argc, char **argv) {
  if (argc < 2 || equalsIgnoreCase(argv[1], "status")) {
    printDeviceStatus();
    return;
  }
  if (isHelpWord(argv[1])) {
    printHelp("device");
    return;
  }
  if (!equalsIgnoreCase(argv[1], "mode")) {
    Serial.println(F("CLI: ERR usage: device [status|mode [usbmic|standalone]]"));
    return;
  }
  if (argc < 3) {
    device_status_t st{};
    control_api_get_device(&st);
    Serial.print(F("CLI: device mode="));
    Serial.println(control_api_device_mode_name(st.mode));
    return;
  }
  device_mode_t mode;
  if (!control_api_device_mode_from_name(argv[2], &mode)) {
    Serial.println(F("CLI: ERR usage: device mode usbmic|standalone"));
    return;
  }
  switch (control_api_set_device_mode(mode)) {
  case DEVICE_SET_OK:
    break;
  case DEVICE_SET_BUSY:
    Serial.println(F("CLI: ERR device mode change refused while streaming "
                     "(close the audio stream first)"));
    return;
  case DEVICE_SET_BAD_PARAM:
    Serial.println(F("CLI: ERR usage: device mode usbmic|standalone"));
    return;
  case DEVICE_SET_FAILED:
    Serial.println(F("CLI: ERR device flash write failed"));
    return;
  }
  printDeviceStatus();
}

/** Extended help for {@code run}. */
void helpRun(void) {
  Serial.println(F("  Starts or stops capture + timing analysis WITHOUT the USB audio"));
  Serial.println(F("  stream, so the device can measure on its own (results: display,"));
  Serial.println(F("  'dsp beat', 'quartz', WebHID). Capture runs while the USB stream"));
  Serial.println(F("  is open or the run is requested."));
  Serial.println(F("    run start    request the run (a no-op if already requested)"));
  Serial.println(F("    run stop     clear the request; capture stops unless a USB audio"));
  Serial.println(F("                 stream is open (then 'running' stays yes)"));
  Serial.println(F("    run status   same line as 'device'"));
  Serial.println(F("  In standalone mode the run is requested at boot and when that mode"));
  Serial.println(F("  is selected; after 'run stop' it stays stopped until 'run start'"));
  Serial.println(F("  or the next boot. See also: help device"));
}

/**
 * {@code run start|stop|status}.
 *
 * @param argc Argument count, {@code argv[0]} = "run".
 * @param argv Arguments.
 */
void cmdRun(int argc, char **argv) {
  if (argc < 2 || equalsIgnoreCase(argv[1], "status")) {
    printDeviceStatus();
    return;
  }
  if (isHelpWord(argv[1])) {
    printHelp("run");
    return;
  }
  if (equalsIgnoreCase(argv[1], "start")) {
    if (control_api_run_start() != DEVICE_RUN_OK) {
      Serial.println(F("CLI: ERR capture backend error (see: status)"));
      return;
    }
    printDeviceStatus();
    return;
  }
  if (equalsIgnoreCase(argv[1], "stop")) {
    control_api_run_stop();
    printDeviceStatus();
    device_status_t st{};
    control_api_get_device(&st);
    if (st.running) {
      Serial.println(F("CLI: tip: capture continues while the USB audio stream is open"));
    }
    return;
  }
  Serial.println(F("CLI: ERR usage: run [start|stop|status]"));
}
// clang-format on

static const CliCommand kCommands[] = {
    {"help", "?", cmdHelp, F("Show command list or help <cmd>"),
     F("help [command]"), helpHelp},
    {"version", "ver", cmdVersion, F("Firmware / board / source"), F("version"),
     helpVersion},
    {"status", nullptr, cmdStatus, F("USB / stream / DSP summary"), F("status"),
     helpStatus},
    {"time", nullptr, cmdTime,
     F("Device microsecond timer (calibration probe)"), F("time"), helpTime},
    {"timebase", "tb", cmdTimebase,
     F("Sample-clock calibration (ppm), GPS, history"),
     F("timebase [set <ppm> [src] [date] [dur]|save|clear|gps|log "
       "[clear]|logdelta [ppm]]"),
     helpTimebase},
    {"device", "dev", cmdDevice,
     F("Mode: USB microphone or standalone timegrapher"),
     F("device [status|mode [usbmic|standalone]]"), helpDevice},
    {"run", nullptr, cmdRun,
     F("Start / stop capture + timing without a USB stream"),
     F("run [start|stop|status]"), helpRun},
    {"display", "oled", cmdDisplay,
     F("OLED status, bus scan, contrast / period"),
     F("display [contrast <1-255>|period <ms>]"), helpDisplay},
    {"diag", nullptr, cmdDiag, F("Periodic diagnostic dump on|off"),
     F("diag [on|off]"), helpDiag},
    {"dsp", nullptr, cmdDsp, F("DSP control plane"),
     F("dsp "
       "list|bypass|on|off|get|set|events|beat|beats|graph|metrics|timing|"
       "match|spectrum|errors|help"),
     helpDsp},
    {"quartz", "qz", cmdQuartz, F("Quartz watch timing: motor / LCD / fork"),
     F("quartz [status|mode|gate|fork|ts|ksigma|restart|pulses|help] [arg]"),
     helpQuartz},
};

} // namespace

void registerAllCommands() {
  constexpr size_t kCount = sizeof(kCommands) / sizeof(kCommands[0]);
  registerCommands(kCommands, kCount);
}

} // namespace Cli
