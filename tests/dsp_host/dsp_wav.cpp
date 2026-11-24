/**
 * @file dsp_wav.cpp
 * @brief Host driver: runs the firmware mechanical DSP chain over a WAV file.
 *
 * Links the unmodified src/dsp_graph.cpp, dsp_blocks.cpp, the three
 * mechanical sources and perf_timing.cpp against the shims in
 * tests/dsp_host/shim/. The signal is fed the way the synthetic source feeds it
 * (audio_source.cpp synthetic_reset / synthetic_next): S16 sample s becomes
 * PCM24 {@code s << 8}, then
 * {@code dsp_sample_from_pcm1808_left24}; 240-sample blocks; source index
 * advances by 240 per block; source rate 48 kHz, timebase verified.
 *
 * Usage: dsp_wav file.wav [--loops N] [--lift DEG] [--avg S] [--every S]
 *                         [--timing unlock|drop] [--settle S]
 *                         [--ppm-at S,PPM] [--mode usb|standalone]
 *                         [--mode-at S,usb|standalone] [--beats] [--metrics]
 * Input: mono 16-bit PCM or 32-bit float WAV at 48 kHz. Output: one JSON line
 * per --every seconds of signal, then one summary line.
 *
 * {@code --beats} also prints one {@code {"beat":{...}}} line per recorded
 * beat. {@code --metrics} adds a final Result line (method "firmware") in the
 * shared format of docs/plans/TIMING_ENGINE_PLAN.md section 2 (T0b), read by
 * scripts/synth_clips/compare.py. {@code --timing drop} sets the
 * {@code timingLandmark} parameter (stage B, tick_shape.h); with
 * {@code --metrics} a {@code {"tick_shape":{...}}} line with the stage-B
 * counters precedes the Result line. {@code --settle} sets the {@code settleS}
 * parameter; {@code --ppm-at S,PPM} sets the clock correction to PPM once the
 * signal reaches S seconds (a cut of reason "clock"). Each progress line and
 * the summary carry the analysis state, cut count and last cut reason; the
 * summary adds the first time and the number of blocks in state noWatch.
 * A host build made with {@code build.sh OUT_DIR --profile} also emits a
 * {@code tick_shape_profile} line with {@code --metrics}. Stage timings are
 * host wall-clock nanoseconds and each peak identifies its block by the
 * first source sample in the looped recording.
 *
 * {@code --mode usb} runs the graph as in USB microphone mode
 * ({@code dsp_graph_set_analysis(false)}: tick detection and the A/h lock
 * only); the default {@code standalone} runs the full engine.
 * {@code --mode-at S,MODE} switches to MODE once the signal reaches S seconds,
 * as {@code device_mode_set} does on the board. The progress and summary lines
 * carry {@code analysis} ("on" / "off") and {@code dropTimedBeats}.
 */

#include "dsp_blocks.h"
#include "dsp_graph.h"
#include "host_clock.h"
#include "mechanical/tick_shape.h"

#ifdef TG_HOST_PROFILE_TICK_SHAPE
#include "tick_shape_profile.h"
#include <cinttypes>
#endif

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kRateHz = 48000u;
/** Block size of the synthetic source (APP_ADC_DMA_BUF_SAMPLES). */
constexpr uint32_t kBlock = 240u;

/** @return Little-endian u16/u32 from {@code p}. */
uint32_t le(const uint8_t *p, int bytes) {
  uint32_t v = 0;
  for (int i = bytes - 1; i >= 0; i--) {
    v = (v << 8) | p[i];
  }
  return v;
}

/**
 * Loads a mono 48 kHz WAV (PCM16 or float32) as float full-scale samples.
 * S16 goes through the firmware path {@code (s << 8)} -> left24 -> float.
 * @return false and fills {@code err} on any format mismatch.
 */
bool load_wav(const char *path, std::vector<float> *out, std::string *err) {
  FILE *f = std::fopen(path, "rb");
  if (f == nullptr) {
    *err = "cannot open file";
    return false;
  }
  std::vector<uint8_t> d;
  uint8_t buf[65536];
  size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
    d.insert(d.end(), buf, buf + n);
  }
  std::fclose(f);
  if (d.size() < 12 || std::memcmp(d.data(), "RIFF", 4) != 0 ||
      std::memcmp(d.data() + 8, "WAVE", 4) != 0) {
    *err = "not a RIFF/WAVE file";
    return false;
  }
  uint32_t fmt_tag = 0, ch = 0, rate = 0, bits = 0;
  bool have_fmt = false;
  size_t pos = 12;
  while (pos + 8 <= d.size()) {
    const uint32_t sz = le(&d[pos + 4], 4);
    const uint8_t *body = &d[pos + 8];
    const size_t avail = d.size() - (pos + 8);
    const size_t len = sz < avail ? sz : avail;
    if (std::memcmp(&d[pos], "fmt ", 4) == 0 && len >= 16) {
      fmt_tag = le(body, 2);
      ch = le(body + 2, 2);
      rate = le(body + 4, 4);
      bits = le(body + 14, 2);
      have_fmt = true;
    } else if (std::memcmp(&d[pos], "data", 4) == 0) {
      if (!have_fmt) {
        *err = "data chunk before fmt chunk";
        return false;
      }
      if (ch != 1u || rate != kRateHz) {
        *err = "need mono 48 kHz (got " + std::to_string(ch) + " ch, " +
               std::to_string(rate) + " Hz)";
        return false;
      }
      if (fmt_tag == 1u && bits == 16u) {
        for (size_t i = 0; i + 2 <= len; i += 2) {
          const auto s = static_cast<int16_t>(le(body + i, 2));
          const int32_t pcm24 = static_cast<int32_t>(s) * 256;
          out->push_back(dsp_sample_from_pcm1808_left24(pcm24));
        }
        return true;
      }
      if (fmt_tag == 3u && bits == 32u) {
        for (size_t i = 0; i + 4 <= len; i += 4) {
          const uint32_t u = le(body + i, 4);
          float x = 0.f;
          std::memcpy(&x, &u, sizeof(x));
          out->push_back(x);
        }
        return true;
      }
      *err = "need 16-bit PCM or 32-bit float";
      return false;
    }
    pos += 8 + sz + (sz & 1u);
  }
  *err = "no data chunk";
  return false;
}

/** Prints one {@code getBeat}-style JSON line for the current state. */
void print_beat(double t_s) {
  dsp_beat_rate_t b;
  dsp_graph_get_beat(&b);
  const auto &a = b.avg_long;
  std::printf(
      "{\"t\":%.2f,\"locked\":%s,\"state\":\"%s\",\"analysis\":\"%s\","
      "\"cuts\":%u,"
      "\"lastCut\":\"%s\",\"stretchS\":%.2f,\"rateValid\":%s,"
      "\"timingSource\":\"%s\",\"handovers\":%u,\"settled\":%s,"
      "\"vphNearest\":%u,\"rateSDay\":%.3f,"
      "\"avgLong\":{\"rateSDay\":%.3f,\"rateValid\":%s,\"beatErrorMs\":%.3f,"
      "\"beatErrorValid\":%s,\"amplitudeDeg\":%.1f,\"amplitudeValid\":%s,"
      "\"samples\":%u},\"liftTimeMs\":%.3f,\"eventsSeen\":%u}\n",
      t_s, b.locked ? "true" : "false", dsp_beat_state_name(b.state),
      b.analysis ? "on" : "off", b.cuts, dsp_cut_reason_name(b.last_cut),
      static_cast<double>(b.stretch_s), b.rate_valid ? "true" : "false",
      b.timing_drop ? "drop" : "unlock", b.handovers,
      b.settled ? "true" : "false", b.vph_nearest, b.rate_s_day, a.rate_s_day,
      a.rate_valid ? "true" : "false", a.beat_error_ms,
      a.beat_error_valid ? "true" : "false", a.amplitude_deg,
      a.amplitude_valid ? "true" : "false", a.samples, b.lift_time_ms,
      b.events_seen);
}

/** Prints a float as JSON, or {@code null} when it is not finite. */
void print_num(const char *key, double v, const char *fmt) {
  if (std::isfinite(v)) {
    std::printf("\"%s\":", key);
    std::printf(fmt, v);
  } else {
    std::printf("\"%s\":null", key);
  }
}

/** Prints one {@code {"beat":{...}}} line for a recorded beat. */
void print_beat_record(const dsp_beat_t &r) {
  static const char *const kStatus[] = {"accepted", "started", "rejected"};
  const auto st = static_cast<unsigned>(r.status);
  std::printf(
      "{\"beat\":{\"seq\":%u,\"grid\":%u,\"k\":%d,\"parity\":%u,"
      "\"side\":\"%s\",\"t_s\":%.6f,\"u_us\":%.2f,\"status\":\"%s\","
      "\"amplitude_deg\":%.1f,\"amplitude_valid\":%s,\"lift_ms\":%.3f}}\n",
      r.seq, r.grid_id, r.k, static_cast<unsigned>(r.parity),
      (r.parity & 1u) != 0u ? "tock" : "tick", r.t_s,
      static_cast<double>(r.u_us), st < 3u ? kStatus[st] : "unknown",
      static_cast<double>(r.amplitude_deg),
      r.amplitude_valid ? "true" : "false", static_cast<double>(r.lift_ms));
}

/**
 * Parses a device mode name.
 *
 * @param name "usb" or "standalone".
 * @param analysis Receives whether the timing engine runs in that mode.
 * @return {@code true} if the name is known.
 */
bool parse_mode(const char *name, bool *analysis) {
  if (std::strcmp(name, "usb") == 0) {
    *analysis = false;
    return true;
  }
  if (std::strcmp(name, "standalone") == 0) {
    *analysis = true;
    return true;
  }
  return false;
}

/** Aggregates for the {@code --metrics} Result line. */
struct Metrics {
  uint32_t blocks = 0;
  uint32_t locked_blocks = 0;
  uint32_t beats = 0;
  uint32_t beats_amp = 0;
  double first_valid_s =
      -1.0;                /**< Signal time of the first valid avgLong rate. */
  double process_ns = 0.0; /**< Summed wall-clock time in dsp_graph_process. */
  uint32_t no_watch_blocks = 0;   /**< Blocks that ended in state noWatch. */
  double first_no_watch_s = -1.0; /**< Signal time of the first such block. */
};

#ifdef TG_HOST_PROFILE_TICK_SHAPE
/** Prints count, total time and per-call and per-block peaks of each stage. */
void print_tick_shape_profile() {
  using namespace tick_shape_profile;
  const Stage *stages[] = {&push, &match, &fold, &window_add, &window_step};
  std::printf("{\"tick_shape_profile\":{");
  for (const Stage *stage : stages) {
    std::printf("%s\"%s\":{\"calls\":%" PRIu64 ",\"total_ns\":%" PRIu64
                ",\"max_call_ns\":%" PRIu64 ",\"max_call_sample\":%" PRIu64
                ",\"max_block_ns\":%" PRIu64 ",\"max_block_sample\":%" PRIu64
                "}",
                stage == stages[0] ? "" : ",", stage->name, stage->calls,
                stage->total_ns, stage->max_call_ns, stage->max_call_sample,
                stage->max_block_ns, stage->max_block_sample);
  }
  std::printf("}}\n");
}
#endif

/** Prints the Result line (shared format, method "firmware") for {@code file}.
 */
void print_result(const char *file, const Metrics &m) {
  dsp_beat_rate_t b;
  dsp_graph_get_beat(&b);
  const auto &a = b.avg_long;
  const char *base = std::strrchr(file, '/');
  base = base != nullptr ? base + 1 : file;
  std::printf("{\"file\":\"%s\",\"method\":\"firmware\",\"locked\":%s,", base,
              b.locked ? "true" : "false");
  if (b.locked && b.vph_nearest != 0u) {
    std::printf("\"bph\":%u,", b.vph_nearest);
  } else {
    std::printf("\"bph\":null,");
  }
  if (a.rate_valid) {
    print_num("rate_s_day", static_cast<double>(a.rate_s_day), "%.3f");
  } else {
    std::printf("\"rate_s_day\":null");
  }
  std::printf(",");
  if (a.beat_error_valid) {
    print_num("be_ms", static_cast<double>(a.beat_error_ms), "%.4f");
  } else {
    std::printf("\"be_ms\":null");
  }
  if (a.beat_error_valid) {
    std::printf(",");
    print_num("be_signed_ms", static_cast<double>(b.beat_error_signed_ms),
              "%.4f");
    std::printf(",\"be_source\":\"%s\",",
                dsp_be_source_name(b.beat_error_source));
  } else {
    std::printf(",\"be_signed_ms\":null,\"be_source\":null,");
  }
  if (a.amplitude_valid) {
    print_num("amp_deg", static_cast<double>(a.amplitude_deg), "%.2f");
  } else {
    std::printf("\"amp_deg\":null");
  }
  std::printf(",");
  if (b.amplitude_tick_valid) {
    print_num("amp_tick_deg", static_cast<double>(b.amplitude_tick_deg),
              "%.2f");
  } else {
    std::printf("\"amp_tick_deg\":null");
  }
  std::printf(",");
  if (b.amplitude_tock_valid) {
    print_num("amp_tock_deg", static_cast<double>(b.amplitude_tock_deg),
              "%.2f");
  } else {
    std::printf("\"amp_tock_deg\":null");
  }
  std::printf(",\"windows_used\":%u,\"windows_total\":%u,",
              b.amplitude_windows_used, b.amplitude_windows_total);
  print_num("jitter_us", static_cast<double>(b.jitter_us), "%.2f");
  std::printf(",\"lock_fraction\":%.4f,\"amp_valid_fraction\":%.4f,",
              m.blocks != 0u ? static_cast<double>(m.locked_blocks) / m.blocks
                             : 0.0,
              m.beats != 0u ? static_cast<double>(m.beats_amp) / m.beats : 0.0);
  if (m.first_valid_s >= 0.0) {
    print_num("first_valid_s", m.first_valid_s, "%.3f");
  } else {
    std::printf("\"first_valid_s\":null");
  }
  std::printf(",");
  print_num("host_ns_per_block", m.blocks != 0u ? m.process_ns / m.blocks : 0.0,
            "%.0f");
  std::printf("}\n");
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: %s file.wav [--loops N] [--lift DEG] [--avg S] "
                 "[--every S] [--timing unlock|drop] [--settle S] "
                 "[--ppm-at S,PPM] [--beats] [--metrics]\n",
                 argv[0]);
    return 2;
  }
  uint32_t loops = 10;
  float lift = 0.f;
  float avg = 0.f;
  double every = 2.0;
  float timing = 0.f; /* timingLandmark: 0 unlock, 1 drop */
  float settle = 0.f;
  double ppm_at_s = -1.0; /* --ppm-at: signal time of the clock step */
  float ppm_step = 0.f;
  bool analysis = true;    /* --mode: false = USB microphone mode */
  double mode_at_s = -1.0; /* --mode-at: signal time of the mode switch */
  bool mode_at_analysis = true;
  bool want_beats = false;
  bool want_metrics = false;
  for (int i = 2; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--beats") {
      want_beats = true;
      continue;
    }
    if (a == "--metrics") {
      want_metrics = true;
      continue;
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "missing value for %s\n", a.c_str());
      return 2;
    }
    if (a == "--timing") {
      const std::string mode = argv[++i];
      if (mode != "unlock" && mode != "drop") {
        std::fprintf(stderr, "error: --timing needs unlock or drop\n");
        return 2;
      }
      timing = mode == "drop" ? 1.f : 0.f;
      continue;
    }
    if (a == "--mode") {
      if (!parse_mode(argv[++i], &analysis)) {
        std::fprintf(stderr, "error: --mode needs usb or standalone\n");
        return 2;
      }
      continue;
    }
    if (a == "--mode-at") {
      char name[16];
      if (std::sscanf(argv[++i], "%lf,%15s", &mode_at_s, name) != 2 ||
          !parse_mode(name, &mode_at_analysis)) {
        std::fprintf(stderr, "error: --mode-at needs S,usb|standalone\n");
        return 2;
      }
      continue;
    }
    if (a == "--ppm-at") {
      if (std::sscanf(argv[++i], "%lf,%f", &ppm_at_s, &ppm_step) != 2) {
        std::fprintf(stderr, "error: --ppm-at needs S,PPM\n");
        return 2;
      }
      continue;
    }
    const double v = std::atof(argv[++i]);
    if (a == "--settle") {
      settle = static_cast<float>(v);
    } else if (a == "--loops") {
      loops = static_cast<uint32_t>(v);
    } else if (a == "--lift") {
      lift = static_cast<float>(v);
    } else if (a == "--avg") {
      avg = static_cast<float>(v);
    } else if (a == "--every") {
      every = v;
    } else {
      std::fprintf(stderr, "unknown option %s\n", a.c_str());
      return 2;
    }
  }

  std::vector<float> wav;
  std::string err;
  if (!load_wav(argv[1], &wav, &err)) {
    std::fprintf(stderr, "error: %s: %s\n", argv[1], err.c_str());
    return 1;
  }
  if (wav.empty() || every <= 0.0) {
    std::fprintf(stderr, "error: empty input or bad --every\n");
    return 1;
  }

  /* Same bring-up as control_api init + audio_source synthetic_reset. */
  dsp_graph_init();
  dsp_graph_set_source_rate(kRateHz);
  dsp_graph_reset_stream();
  dsp_graph_set_timebase(true, static_cast<float>(kRateHz));
  if (lift > 0.f &&
      !dsp_graph_set_param(DSP_BLOCK_BEAT_RATE, "liftAngle", lift)) {
    std::fprintf(stderr, "error: liftAngle %.1f rejected\n",
                 static_cast<double>(lift));
    return 1;
  }
  if (avg > 0.f && !dsp_graph_set_param(DSP_BLOCK_BEAT_RATE, "avgS", avg)) {
    std::fprintf(stderr, "error: avgS %.1f rejected\n",
                 static_cast<double>(avg));
    return 1;
  }

  if (!dsp_graph_set_param(DSP_BLOCK_BEAT_RATE, "settleS", settle)) {
    std::fprintf(stderr, "error: settleS %.1f rejected\n",
                 static_cast<double>(settle));
    return 1;
  }
  if (!dsp_graph_set_param(DSP_BLOCK_BEAT_RATE, "timingLandmark", timing)) {
    std::fprintf(stderr, "error: timingLandmark rejected\n");
    return 1;
  }
  dsp_graph_set_analysis(analysis);

  const uint64_t total = static_cast<uint64_t>(wav.size()) * loops;
  const auto every_samples = static_cast<uint64_t>(every * kRateHz);
  uint64_t next_print = every_samples;
  uint64_t fed = 0;
  uint64_t pos = 0; /* index into the looped clip */
  uint32_t beat_seq = 0;
  Metrics m;
  float in[kBlock];
  int32_t out[kBlock];
  dsp_beat_t recs[64];

  while (fed + kBlock <= total) {
    for (uint32_t i = 0; i < kBlock; i++) {
      in[i] = wav[pos];
      if (++pos == wav.size()) {
        pos = 0;
      }
    }
    if (ppm_at_s >= 0.0 && static_cast<double>(fed) >= ppm_at_s * kRateHz) {
      ppm_at_s = -1.0;
      if (!dsp_graph_set_param(DSP_BLOCK_BEAT_RATE, "calPpm", ppm_step)) {
        std::fprintf(stderr, "error: calPpm rejected\n");
        return 1;
      }
    }
    if (mode_at_s >= 0.0 && static_cast<double>(fed) >= mode_at_s * kRateHz) {
      mode_at_s = -1.0;
      dsp_graph_set_analysis(mode_at_analysis);
    }
    host_clock::set_samples(fed + kBlock, kRateHz);
#ifdef TG_HOST_PROFILE_TICK_SHAPE
    tick_shape_profile::block_sample = fed;
#endif
    const auto t0 = std::chrono::steady_clock::now();
    (void)dsp_graph_process(in, kBlock, fed, out);
    const auto t1 = std::chrono::steady_clock::now();
    m.process_ns += static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    fed += kBlock;
    m.blocks++;

    const uint32_t got = dsp_graph_copy_beats_since(beat_seq, recs, 64u);
    for (uint32_t i = 0; i < got; i++) {
      m.beats++;
      if (recs[i].amplitude_valid) {
        m.beats_amp++;
      }
      if (want_beats) {
        print_beat_record(recs[i]);
      }
      beat_seq = recs[i].seq + 1u;
    }

    dsp_beat_rate_t b;
    dsp_graph_get_beat(&b);
    if (b.locked) {
      m.locked_blocks++;
    }
    if (b.state == DSP_BEAT_STATE_NO_WATCH) {
      m.no_watch_blocks++;
      if (m.first_no_watch_s < 0.0) {
        m.first_no_watch_s = static_cast<double>(fed) / kRateHz;
      }
    }
    if (m.first_valid_s < 0.0 && b.avg_long.rate_valid) {
      m.first_valid_s = static_cast<double>(fed) / kRateHz;
    }
    if (fed >= next_print) {
      print_beat(static_cast<double>(fed) / kRateHz);
      next_print += every_samples;
    }
  }

  dsp_beat_rate_t b;
  dsp_graph_get_beat(&b);
  const auto &a = b.avg_long;
  std::printf(
      "{\"summary\":true,\"seconds\":%.2f,\"locked\":%s,\"state\":\"%s\","
      "\"analysis\":\"%s\",\"dropTimedBeats\":%u,"
      "\"cuts\":%u,\"lastCut\":\"%s\",\"handovers\":%u,\"noWatchBlocks\":%u,"
      "\"firstNoWatchS\":%.2f,\"vphNearest\":%u,"
      "\"lockedFraction\":%.3f,\"avgLong\":{\"rateSDay\":%.3f,\"rateValid\":%s,"
      "\"beatErrorMs\":%.3f,\"beatErrorValid\":%s,\"amplitudeDeg\":%.1f,"
      "\"amplitudeValid\":%s,\"samples\":%u},\"jitterUs\":%.1f,"
      "\"beatsRecorded\":%u,\"beatsAmplitudeValid\":%u,"
      "\"amplitudeTickDeg\":%.1f,\"amplitudeTockDeg\":%.1f,"
      "\"amplitudeTickValid\":%s,\"amplitudeTockValid\":%s,"
      "\"windowsUsed\":%u,\"windowsTotal\":%u,"
      "\"beatErrorSignedMs\":%.3f,\"beatErrorSource\":\"%s\"}\n",
      static_cast<double>(fed) / kRateHz, b.locked ? "true" : "false",
      dsp_beat_state_name(b.state), b.analysis ? "on" : "off",
      b.drop_timed_beats, b.cuts, dsp_cut_reason_name(b.last_cut), b.handovers,
      m.no_watch_blocks, m.first_no_watch_s, b.vph_nearest,
      m.blocks != 0u ? static_cast<double>(m.locked_blocks) / m.blocks : 0.0,
      a.rate_s_day, a.rate_valid ? "true" : "false", a.beat_error_ms,
      a.beat_error_valid ? "true" : "false", a.amplitude_deg,
      a.amplitude_valid ? "true" : "false", a.samples, b.jitter_us, m.beats,
      m.beats_amp, static_cast<double>(b.amplitude_tick_deg),
      static_cast<double>(b.amplitude_tock_deg),
      b.amplitude_tick_valid ? "true" : "false",
      b.amplitude_tock_valid ? "true" : "false", b.amplitude_windows_used,
      b.amplitude_windows_total, static_cast<double>(b.beat_error_signed_ms),
      dsp_be_source_name(b.beat_error_source));
  if (want_metrics) {
    const tick_shape_stats_t st = tick_shape_stats();
    std::printf("{\"tick_shape\":{\"events\":%u,\"timed\":%u,\"learning\":%u,"
                "\"low_quality\":%u,\"out_of_ring\":%u,\"restarts\":%u,"
                "\"drop_timed_beats\":%u,\"drop_fallbacks\":%u}}\n",
                st.events, st.timed, st.learning, st.low_quality,
                st.out_of_ring, st.restarts, b.drop_timed_beats,
                b.drop_fallbacks);
#ifdef TG_HOST_PROFILE_TICK_SHAPE
    print_tick_shape_profile();
#endif
    print_result(argv[1], m);
  }
  return 0;
}
