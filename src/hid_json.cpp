/**
 * @file hid_json.cpp
 * @brief Host-testable JSON encoders of the HID control plane (bounded
 *        append, sanitising, quartz timing payloads).
 *
 * @see hid_json.h, hid_control.cpp, docs/HID_CONTROL_PLANE.md
 */

#include "hid_json.h"

#include "control_api.h"

#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace hid_json {

namespace {

/** @return {@code "true"} / {@code "false"} for a JSON boolean. */
[[nodiscard]] constexpr const char *jbool(bool b) noexcept {
  return b ? "true" : "false";
}

/**
 * @param v Value.
 * @return {@code v} widened for a {@code printf} {@code %g}, or 0 when it is
 *         not finite (NaN and infinities are not JSON numbers).
 */
[[nodiscard]] double num(float v) noexcept {
  return std::isfinite(v) ? static_cast<double>(v) : 0.0;
}

/** @copydoc num(float) */
[[nodiscard]] double num(double v) noexcept {
  return std::isfinite(v) ? v : 0.0;
}

/**
 * Formats a uint64 as decimal without relying on {@code %llu} support.
 *
 * @param v Value.
 * @param out Destination, at least 21 bytes.
 */
void u64_dec(uint64_t v, char (&out)[24]) noexcept {
  char tmp[24];
  std::size_t n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + static_cast<int>(v % 10u));
    v /= 10u;
  } while (v != 0u && n < sizeof(tmp));
  for (std::size_t i = 0; i < n; i++) {
    out[i] = tmp[n - 1u - i];
  }
  out[n] = '\0';
}

/**
 * Appends the motor detail object.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @param m Motor detail.
 * @return New length, or -1.
 */
[[nodiscard]] int encode_motor(char *dst, std::size_t dstlen, int off,
                               const dsp_quartz_motor_t &m) {
  return appendf(
      dst, dstlen, off,
      "{\"valid\":%s,\"pulses\":%lu,\"fitPulses\":%lu,\"stepS\":%.6g,"
      "\"spanS\":%.7g,\"rateWithSDay\":%.6g,\"rateWithUncalSDay\":%.6g,"
      "\"rateWithoutSDay\":%.6g,\"rateWithoutUncalSDay\":%.6g,"
      "\"inhibition\":\"%s\",\"inhibitionPeriodValid\":%s,"
      "\"inhibitionPeriodS\":%.6g,\"events\":%lu,\"cyclesUsed\":%lu,"
      "\"jumpLastUs\":%.6g,\"jumpLastN\":%lu,\"jumpMeanUs\":%.6g,"
      "\"jumpMeanN\":%lu,\"residualRmsUs\":%.6g,\"polarityOffsetUs\":%.6g,"
      "\"clipped\":%s,\"clippedPulses\":%lu,\"extraPulses\":%lu,"
      "\"noiseSigma\":%.6g,\"threshold\":%.6g,\"peakSnr\":%.6g,"
      "\"hintWeakSignal\":%s,\"hintClipped\":%s,\"noJumpSpanS\":%.6g,"
      "\"jumpThresholdUs\":%.6g,\"rebuilding\":%s}",
      jbool(m.valid), static_cast<unsigned long>(m.pulses),
      static_cast<unsigned long>(m.fit_pulses), num(m.step_s), num(m.span_s),
      num(m.rate_with_s_day), num(m.rate_with_uncal_s_day),
      num(m.rate_without_s_day), num(m.rate_without_uncal_s_day),
      control_api_quartz_inhibition_name(m.inhibition),
      jbool(m.inhibition_period_valid), num(m.inhibition_period_s),
      static_cast<unsigned long>(m.events),
      static_cast<unsigned long>(m.cycles_used), num(m.jump_last_us),
      static_cast<unsigned long>(m.jump_last_n), num(m.jump_mean_us),
      static_cast<unsigned long>(m.jump_mean_n), num(m.residual_rms_us),
      num(m.polarity_offset_us), jbool(m.clipped),
      static_cast<unsigned long>(m.clipped_pulses),
      static_cast<unsigned long>(m.extra_pulses), num(m.noise_sigma),
      num(m.threshold), num(m.peak_snr), jbool(m.hint_weak_signal),
      jbool(m.hint_clipped), num(m.no_jump_span_s), num(m.jump_threshold_us),
      jbool(m.rebuilding));
}

/**
 * Appends the LCD detail object.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @param l LCD detail.
 * @return New length, or -1.
 */
[[nodiscard]] int encode_lcd(char *dst, std::size_t dstlen, int off,
                             const dsp_quartz_lcd_t &l) {
  off = appendf(dst, dstlen, off,
                "{\"valid\":%s,\"status\":\"%s\",\"f0Hz\":%lu,\"harmonics\":[",
                jbool(l.valid), control_api_quartz_lcd_status_name(l.status),
                static_cast<unsigned long>(l.f0_hz));
  const auto count = l.harmonic_count < DSP_QUARTZ_MAX_HARMONICS
                         ? static_cast<uint32_t>(l.harmonic_count)
                         : DSP_QUARTZ_MAX_HARMONICS;
  for (uint32_t i = 0; i < count; i++) {
    off = appendf(dst, dstlen, off, "%s%u", i != 0u ? "," : "",
                  static_cast<unsigned>(l.harmonics[i]));
  }
  return appendf(dst, dstlen, off,
                 "],\"blocks\":%lu,\"rateSDay\":%.6g,\"rateUncalSDay\":%.6g,"
                 "\"retries\":%lu,\"clipped\":%s}",
                 static_cast<unsigned long>(l.blocks), num(l.rate_s_day),
                 num(l.rate_uncal_s_day), static_cast<unsigned long>(l.retries),
                 jbool(l.clipped));
}

/**
 * Appends the tuning-fork detail object.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity.
 * @param off Current length.
 * @param f Fork detail.
 * @return New length, or -1.
 */
[[nodiscard]] int encode_fork(char *dst, std::size_t dstlen, int off,
                              const dsp_quartz_fork_t &f) {
  return appendf(dst, dstlen, off,
                 "{\"valid\":%s,\"fNomHz\":%.6g,\"blocks\":%lu,"
                 "\"rateSDay\":%.6g,\"rateUncalSDay\":%.6g,\"clipped\":%s}",
                 jbool(f.valid), num(f.f_nom_hz),
                 static_cast<unsigned long>(f.blocks), num(f.rate_s_day),
                 num(f.rate_uncal_s_day), jbool(f.clipped));
}

} // namespace

int appendf(char *dst, std::size_t dstlen, int off, const char *fmt, ...) {
  if (off < 0 || static_cast<std::size_t>(off) >= dstlen) {
    return -1;
  }
  va_list ap;
  va_start(ap, fmt);
  const auto n = std::vsnprintf(
      dst + off, dstlen - static_cast<std::size_t>(off), fmt, ap);
  va_end(ap);
  if (n < 0 ||
      static_cast<std::size_t>(off) + static_cast<std::size_t>(n) >= dstlen) {
    return -1;
  }
  return off + n;
}

void json_sanitize(const char *in, char *out, std::size_t outlen) {
  std::size_t i = 0;
  for (; in != nullptr && *in != '\0' && i + 1u < outlen; in++) {
    const auto c = static_cast<unsigned char>(*in);
    if (c >= 0x20 && c != '"' && c != '\\') {
      out[i++] = static_cast<char>(c);
    }
  }
  out[i] = '\0';
}

int encode_quartz_settings(char *dst, std::size_t dstlen, int off,
                           const dsp_quartz_t &q) {
  return appendf(dst, dstlen, off,
                 "{\"mode\":\"%s\",\"gateS\":%.6g,\"forkHz\":%.6g,"
                 "\"tsMethod\":\"%s\",\"kSigma\":%.6g}",
                 control_api_quartz_mode_name(q.mode), num(q.gate_s),
                 num(q.fork_hz), control_api_quartz_ts_method_name(q.ts_method),
                 num(q.k_sigma));
}

int encode_quartz(char *dst, std::size_t dstlen, int off,
                  const dsp_quartz_t &q) {
  char reason[kQuartzTextMax + 1u];
  char cal_source[kQuartzTextMax + 1u];
  json_sanitize(q.reason, reason, sizeof(reason));
  json_sanitize(q.cal_source != nullptr ? q.cal_source : "none", cal_source,
                sizeof(cal_source));
  off = appendf(
      dst, dstlen, off,
      "{\"mode\":\"%s\",\"state\":\"%s\",\"measurementId\":%lu,\"reason\":\"%"
      "s\",\"sourceHz\":%lu,"
      "\"timebaseOk\":%s,\"elapsedS\":%.7g,\"gateS\":%.6g,\"forkHz\":%.6g,"
      "\"tsMethod\":\"%s\",\"kSigma\":%.6g,\"sampleLossRestarts\":%lu,"
      "\"rateSDay\":%.6g,\"rateValid\":%s,\"calPpm\":%.6g,"
      "\"calSource\":\"%s\",\"calibrated\":%s,\"motor\":",
      control_api_quartz_mode_name(q.mode),
      control_api_quartz_state_name(q.state),
      static_cast<unsigned long>(q.measurement_id), reason,
      static_cast<unsigned long>(q.source_hz), jbool(q.timebase_ok),
      num(q.elapsed_s), num(q.gate_s), num(q.fork_hz),
      control_api_quartz_ts_method_name(q.ts_method), num(q.k_sigma),
      static_cast<unsigned long>(q.sample_loss_restarts), num(q.rate_s_day),
      jbool(q.rate_valid), num(q.cal_ppm), cal_source, jbool(q.calibrated));
  off = q.mode == DSP_QUARTZ_MODE_MOTOR
            ? encode_motor(dst, dstlen, off, q.motor)
            : appendf(dst, dstlen, off, "null");
  off = appendf(dst, dstlen, off, ",\"lcd\":");
  off = q.mode == DSP_QUARTZ_MODE_LCD ? encode_lcd(dst, dstlen, off, q.lcd)
                                      : appendf(dst, dstlen, off, "null");
  off = appendf(dst, dstlen, off, ",\"fork\":");
  off = q.mode == DSP_QUARTZ_MODE_FORK ? encode_fork(dst, dstlen, off, q.fork)
                                       : appendf(dst, dstlen, off, "null");
  return appendf(dst, dstlen, off, "}");
}

int encode_quartz_pulse(char *dst, std::size_t dstlen, int off,
                        const dsp_quartz_pulse_t &p) {
  char index[24];
  u64_dec(p.sample_index, index);
  return appendf(dst, dstlen, off,
                 "{\"measurementId\":%lu,\"sequence\":%lu,\"sampleIndex\":\"%"
                 "s\",\"tS\":%.7f,"
                 "\"peak\":%.6g,\"snr\":%.6g,\"polarity\":%d,\"extras\":%u,"
                 "\"clipped\":%s,\"closed\":%s}",
                 static_cast<unsigned long>(p.measurement_id),
                 static_cast<unsigned long>(p.sequence), index, num(p.t_s),
                 num(p.peak), num(p.snr), static_cast<int>(p.polarity),
                 static_cast<unsigned>(p.extras), jbool(p.clipped),
                 jbool(p.closed));
}

int encode_quartz_pulses(char *dst, std::size_t dstlen, int off,
                         const dsp_quartz_pulse_t *pulses, uint32_t n) {
  off = appendf(dst, dstlen, off, "{\"pulses\":[");
  for (uint32_t i = 0; pulses != nullptr && i < n; i++) {
    if (i != 0u) {
      off = appendf(dst, dstlen, off, ",");
    }
    off = encode_quartz_pulse(dst, dstlen, off, pulses[i]);
  }
  return appendf(dst, dstlen, off, "]}");
}

namespace {

/**
 * @param v Value.
 * @param lim Limit.
 * @return {@code v} clamped to +-{@code lim}, 0 when not finite.
 */
[[nodiscard]] double clamp_num(double v, double lim) noexcept {
  if (!std::isfinite(v)) {
    return 0.0;
  }
  return v > lim ? lim : (v < -lim ? -lim : v);
}

/** @return The {@code streamState} name of a stream state. */
[[nodiscard]] const char *stream_state_text(stream_state_t s) noexcept {
  switch (s) {
  case STREAM_STATE_USB_UNCONFIGURED:
    return "unconfigured";
  case STREAM_STATE_USB_CONFIGURED_IDLE:
    return "idle";
  case STREAM_STATE_STREAM_PREFILL:
    return "prefill";
  case STREAM_STATE_STREAM_ACTIVE:
    return "active";
  case STREAM_STATE_USB_SUSPENDED:
    return "suspended";
  case STREAM_STATE_STREAM_ERROR:
    return "error";
  }
  return "error";
}

/** @return {@code "usbMic"} / {@code "standalone"} (see device_mode_name). */
[[nodiscard]] const char *device_mode_text(device_mode_t m) noexcept {
  return m == DEVICE_MODE_STANDALONE ? "standalone" : "usbMic";
}

/** @return The {@code status} name of a beat outcome. */
[[nodiscard]] const char *beat_status_text(dsp_beat_status_t s) noexcept {
  switch (s) {
  case DSP_BEAT_ACCEPTED:
    return "accepted";
  case DSP_BEAT_STARTED:
    return "started";
  case DSP_BEAT_REJECTED:
    return "rejected";
  }
  return "rejected";
}

/** @return The {@code group} name of a pipeline node kind. */
[[nodiscard]] const char *node_group_text(uint8_t kind) noexcept {
  switch (kind) {
  case DSP_NODE_SOURCE:
    return "source";
  case DSP_NODE_ANALYSIS:
    return "analysis";
  case DSP_NODE_PROCESS:
    return "process";
  case DSP_NODE_SINK:
    return "sink";
  case DSP_NODE_QUARTZ:
    return "quartz";
  }
  return "process";
}

/** @return The GNSS code of an {@code nmea_sys_t} value. */
[[nodiscard]] const char *gps_sys_text(uint8_t sys) noexcept {
  switch (sys) {
  case 1:
    return "GP";
  case 2:
    return "GL";
  case 3:
    return "GA";
  case 4:
    return "BD";
  case 5:
    return "QZ";
  default:
    return "??";
  }
}

/** @return The {@code source} name of a log entry ({@code timebase_source_t}
 * value; manual for anything that is not ntp / pps). */
[[nodiscard]] const char *log_source_text(uint8_t src) noexcept {
  switch (src) {
  case TIMEBASE_SRC_NTP:
    return "ntp";
  case TIMEBASE_SRC_PPS:
    return "pps";
  default:
    return "manual";
  }
}

/**
 * Formats {@code snprintf}-style into {@code buf} and returns the length,
 * or 0 if it did not fit (never negative).
 */
[[gnu::format(printf, 3, 4)]] std::size_t fmt_into(char *buf, std::size_t n,
                                                   const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int w = std::vsnprintf(buf, n, fmt, ap);
  va_end(ap);
  return (w < 0 || static_cast<std::size_t>(w) >= n)
             ? 0u
             : static_cast<std::size_t>(w);
}

/** Longest encoded log entry (bytes). */
constexpr std::size_t kLogEntryJsonMax = 128u;

} // namespace

int encode_pipeline(char *dst, std::size_t dstlen, int off,
                    const dsp_pipeline_node_t *nodes, uint32_t count,
                    bool include_quartz) {
  off = appendf(dst, dstlen, off, "[");
  bool first = true;
  for (uint32_t i = 0; nodes != nullptr && i < count; i++) {
    const auto &n = nodes[i];
    if (!include_quartz && n.kind == DSP_NODE_QUARTZ) {
      continue;
    }
    off = appendf(dst, dstlen, off,
                  "%s{\"key\":\"%s\",\"group\":\"%s\",\"order\":%u,"
                  "\"inputs\":[",
                  first ? "" : ",", n.key, node_group_text(n.kind),
                  static_cast<unsigned>(n.order));
    first = false;
    bool first_in = true;
    for (uint8_t k = 0; k < n.n_inputs; k++) {
      /* An input that names an omitted node is dropped with it. */
      if (!include_quartz && std::strcmp(n.inputs[k], "quartz") == 0) {
        continue;
      }
      off = appendf(dst, dstlen, off, "%s\"%s\"", first_in ? "" : ",",
                    n.inputs[k]);
      first_in = false;
    }
    off = appendf(dst, dstlen, off, "]}");
  }
  return appendf(dst, dstlen, off, "]");
}

int encode_hello(char *dst, std::size_t dstlen, int off, const HelloParams &p) {
  off = appendf(
      dst, dstlen, off,
      "{\"protocolVersion\":1,\"fwVersion\":\"%s\",\"board\":\"%s\","
      "\"source\":\"%s\",\"vid\":%u,\"pid\":%u,\"caps\":{"
      "\"maxJsonBytes\":%u,\"metricsHzMax\":%u,\"beatHzMax\":%u,"
      "\"timingHzMax\":%u,\"timing\":true,\"quartz\":%s,"
      "\"quartzHzMax\":%u,\"quartzPulsesMax\":%u,"
      "\"deviceMode\":true,\"beats\":true,\"beatsMax\":%u,\"gps\":%s,"
      "\"pps\":%s,\"timebaseLog\":true,\"timebaseLogMax\":%u,\"blocks\":[",
      p.fw_version, p.board, p.source, p.vid, p.pid, p.max_json,
      p.metrics_hz_max, p.beat_hz_max, p.timing_hz_max, jbool(p.quartz),
      p.quartz_hz_max, p.quartz_pulses_max, static_cast<unsigned>(kBeatsMax),
      jbool(p.gps), jbool(p.pps), static_cast<unsigned>(kTimebaseLogMax));
  for (int i = 1; i < static_cast<int>(DSP_BLOCK_COUNT) && off >= 0; i++) {
    off = appendf(dst, dstlen, off, "%s\"%s\"", i > 1 ? "," : "",
                  dsp_block_json_name(static_cast<dsp_block_id_t>(i)));
  }
  off = appendf(dst, dstlen, off, "],\"pipeline\":");
  uint32_t count = 0;
  const auto *nodes = dsp_pipeline_nodes(&count);
  off = encode_pipeline(dst, dstlen, off, nodes, count, p.quartz);
  return appendf(dst, dstlen, off, "}}");
}

int encode_device_status(char *dst, std::size_t dstlen, int off,
                         const device_status_t &s) {
  return appendf(dst, dstlen, off,
                 "{\"mode\":\"%s\",\"running\":%s,\"runRequested\":%s,"
                 "\"usbStreaming\":%s,\"streamState\":\"%s\",\"saved\":%s}",
                 device_mode_text(s.mode), jbool(s.running),
                 jbool(s.run_requested), jbool(s.usb_streaming),
                 stream_state_text(s.stream_state), jbool(s.saved));
}

int encode_beat(char *dst, std::size_t dstlen, int off, const dsp_beat_t &b) {
  char index[24];
  u64_dec(b.sample_index, index);
  return appendf(
      dst, dstlen, off,
      "{\"seq\":%lu,\"gridId\":%lu,\"sampleIndex\":\"%s\",\"tS\":%.7f,"
      "\"k\":%ld,\"parity\":%u,\"uUs\":%.6g,\"nominalUs\":%.6g,"
      "\"amplitudeDeg\":%.6g,\"amplitudeValid\":%s,\"liftMs\":%.6g,"
      "\"status\":\"%s\"}",
      static_cast<unsigned long>(b.seq), static_cast<unsigned long>(b.grid_id),
      index, clamp_num(b.t_s, 1e12), static_cast<long>(b.k),
      static_cast<unsigned>(b.parity & 1u), num(b.u_us), num(b.nominal_us),
      num(b.amplitude_deg), jbool(b.amplitude_valid), num(b.lift_ms),
      beat_status_text(b.status));
}

int encode_beats(char *dst, std::size_t dstlen, int off,
                 const dsp_beat_t *beats, uint32_t n, uint32_t *emitted) {
  uint32_t done = 0;
  off = appendf(dst, dstlen, off, "{\"beats\":[");
  /* "]}" closes the object; the envelope's "}" needs kEnvelopeTail more. */
  constexpr std::size_t kClose = 2u + kEnvelopeTail;
  for (uint32_t i = 0; beats != nullptr && i < n && off >= 0; i++) {
    char tmp[kBeatJsonMax + 8u];
    const int w = encode_beat(tmp, sizeof(tmp), 0, beats[i]);
    if (w < 0 || static_cast<std::size_t>(off) + (i != 0u ? 1u : 0u) +
                         static_cast<std::size_t>(w) + kClose >=
                     dstlen) {
      break;
    }
    off = appendf(dst, dstlen, off, "%s%s", i != 0u ? "," : "", tmp);
    done++;
  }
  if (emitted != nullptr) {
    *emitted = done;
  }
  return appendf(dst, dstlen, off, "]}");
}

int encode_event(char *dst, std::size_t dstlen, int off,
                 const dsp_event_t &ev) {
  char s0[24], s1[24], s2[24], s3[24];
  u64_dec(ev.start_sample, s0);
  u64_dec(ev.peak_sample, s1);
  u64_dec(ev.unlock_sample, s2);
  u64_dec(ev.drop_sample, s3);
  return appendf(
      dst, dstlen, off,
      "{\"startSample\":\"%s\",\"peakSample\":\"%s\","
      "\"unlockSample\":\"%s\",\"dropSample\":\"%s\","
      "\"durationSamples\":%lu,\"riseSamples\":%lu,"
      "\"liftSamples\":%lu,"
      "\"peakAbs\":%.6g,\"noiseFloor\":%.6g,\"snrDb\":%.6g,"
      "\"energy\":%.6g,\"confidence\":%.6g}",
      s0, s1, s2, s3, static_cast<unsigned long>(ev.duration_samples),
      static_cast<unsigned long>(ev.rise_samples),
      static_cast<unsigned long>(ev.lift_samples), num(ev.peak_abs),
      num(ev.noise_floor), num(ev.snr_db), num(ev.energy), num(ev.confidence));
}

int encode_events(char *dst, std::size_t dstlen, int off,
                  const dsp_event_t *events, uint32_t n, uint32_t *emitted) {
  uint32_t done = 0;
  off = appendf(dst, dstlen, off, "{\"events\":[");
  /* "]}" closes the object; the envelope's "}" needs kEnvelopeTail more. */
  constexpr std::size_t kClose = 2u + kEnvelopeTail;
  for (uint32_t i = 0; events != nullptr && i < n && off >= 0; i++) {
    char tmp[kEventJsonMax + 8u];
    const int w = encode_event(tmp, sizeof(tmp), 0, events[i]);
    if (w < 0 || static_cast<std::size_t>(off) + (i != 0u ? 1u : 0u) +
                         static_cast<std::size_t>(w) + kClose >=
                     dstlen) {
      break;
    }
    off = appendf(dst, dstlen, off, "%s%s", i != 0u ? "," : "", tmp);
    done++;
  }
  if (emitted != nullptr) {
    *emitted = done;
  }
  return appendf(dst, dstlen, off, "]}");
}

bool parse_i64(const char *s, int64_t *out) {
  if (s == nullptr || out == nullptr) {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const long long n = std::strtoll(s, &end, 10);
  if (end == s || errno == ERANGE) {
    return false;
  }
  *out = static_cast<int64_t>(n);
  return true;
}

void i64_dec(int64_t v, char (&out)[24]) noexcept {
  /* Magnitude as unsigned so INT64_MIN does not overflow. */
  const uint64_t mag =
      v < 0 ? 0u - static_cast<uint64_t>(v) : static_cast<uint64_t>(v);
  char digits[24];
  u64_dec(mag, digits);
  if (v < 0) {
    out[0] = '-';
    std::memcpy(out + 1, digits, std::strlen(digits) + 1u);
  } else {
    std::memcpy(out, digits, std::strlen(digits) + 1u);
  }
}

int encode_ping(char *dst, std::size_t dstlen, int off, int64_t t,
                uint32_t uptime_ms) {
  char ts[24];
  i64_dec(t, ts);
  return appendf(dst, dstlen, off, "{\"t\":%s,\"uptimeMs\":%lu}", ts,
                 static_cast<unsigned long>(uptime_ms));
}

int encode_gps(char *dst, std::size_t dstlen, int off,
               const timebase_gps_t &g) {
  off = appendf(
      dst, dstlen, off,
      "{\"fix\":%s,\"quality\":%u,\"utcUnix\":%lu,\"ageMs\":%ld,"
      "\"satsUsed\":%u,\"hdop\":%.3g,\"pdop\":%.3g,\"vdop\":%.3g,"
      "\"lat\":%.7f,\"lon\":%.7f,\"altM\":%.1f,\"posValid\":%s,"
      "\"satCols\":[\"sys\",\"prn\",\"el\",\"az\",\"snr\",\"used\"],"
      "\"sats\":[",
      jbool(g.fix), static_cast<unsigned>(g.quality),
      static_cast<unsigned long>(g.utc_unix), static_cast<long>(g.age_ms),
      static_cast<unsigned>(g.sats_used), num(g.hdop), num(g.pdop), num(g.vdop),
      clamp_num(g.lat, 90.0), clamp_num(g.lon, 180.0),
      clamp_num(static_cast<double>(g.alt_m), 1e6), jbool(g.pos_valid));
  const uint32_t n = g.n_sats < kGpsSatsMax ? g.n_sats : kGpsSatsMax;
  for (uint32_t i = 0; i < n; i++) {
    const auto &s = g.sats[i];
    off = appendf(dst, dstlen, off, "%s[\"%s\",%u,%d,%d,%d,%d]",
                  i != 0u ? "," : "", gps_sys_text(s.sys),
                  static_cast<unsigned>(s.prn), static_cast<int>(s.el),
                  static_cast<int>(s.az), static_cast<int>(s.snr),
                  s.used ? 1 : 0);
  }
  return appendf(dst, dstlen, off, "]}");
}

int encode_timebase_log(char *dst, std::size_t dstlen, int off,
                        const timebase_log_t &log, uint32_t offset,
                        bool pending, float delta_ppm, uint32_t last_write_unix,
                        uint32_t *emitted) {
  const uint32_t total =
      log.count < kTimebaseLogMax ? log.count : kTimebaseLogMax;
  char trailer[128];
  const std::size_t tlen = fmt_into(
      trailer, sizeof(trailer),
      "],\"total\":%lu,\"offset\":%lu,\"pending\":%s,\"logDeltaPpm\":%.6g,"
      "\"lastWriteUnix\":%lu}",
      static_cast<unsigned long>(total),
      static_cast<unsigned long>(offset < total ? offset : total),
      jbool(pending), num(delta_ppm),
      static_cast<unsigned long>(last_write_unix));
  off = appendf(dst, dstlen, off, "{\"entries\":[");
  if (off < 0 || tlen == 0u) {
    return -1;
  }
  /* Room for entries: what is left minus the trailer, the envelope's tail
   * and the NUL. */
  const std::size_t reserve = tlen + kEnvelopeTail + 1u;
  const std::size_t room =
      static_cast<std::size_t>(off) + reserve < dstlen
          ? dstlen - static_cast<std::size_t>(off) - reserve
          : 0u;
  std::size_t used = 0;
  uint32_t done = 0;
  char text[kLogEntryJsonMax];
  for (uint32_t i = offset; i < total; i++) {
    const auto *e = timebase_log_get(&log, i);
    const std::size_t w =
        e == nullptr
            ? 0u
            : fmt_into(text, sizeof(text),
                       "{\"dateUnix\":%lu,\"ppm\":%.5g,\"source\":\"%s\","
                       "\"durationS\":%lu,\"reason\":\"%s\"}",
                       static_cast<unsigned long>(e->date_unix), num(e->ppm),
                       log_source_text(e->source),
                       static_cast<unsigned long>(e->duration_s),
                       timebase_log_reason_name(e->reason));
    const std::size_t need = w + (done != 0u ? 1u : 0u);
    if (w == 0u || used + need > room) {
      break;
    }
    off = appendf(dst, dstlen, off, "%s%s", done != 0u ? "," : "", text);
    used += need;
    done++;
  }
  if (emitted != nullptr) {
    *emitted = done;
  }
  return appendf(dst, dstlen, off, "%s", trailer);
}

} // namespace hid_json
