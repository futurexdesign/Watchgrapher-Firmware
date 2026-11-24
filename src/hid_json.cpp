/**
 * @file hid_json.cpp
 * @brief Host-testable JSON encoders of the HID control plane (bounded
 *        append, sanitising, quartz timing payloads).
 *
 * @see hid_json.h, hid_control.cpp, docs/HID_CONTROL_PLANE.md
 */

#include "hid_json.h"

#include "control/json_scan.h"
#include "control_api.h"

#include <array>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
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
      "{\"mode\":\"%s\",\"state\":\"%s\",\"analysis\":\"%s\","
      "\"measurementId\":%lu,\"reason\":\"%s\",\"sourceHz\":%lu,"
      "\"timebaseOk\":%s,\"elapsedS\":%.7g,\"gateS\":%.6g,\"forkHz\":%.6g,"
      "\"tsMethod\":\"%s\",\"kSigma\":%.6g,\"sampleLossRestarts\":%lu,"
      "\"rateSDay\":%.6g,\"rateValid\":%s,\"calPpm\":%.6g,"
      "\"calSource\":\"%s\",\"calibrated\":%s,\"motor\":",
      control_api_quartz_mode_name(q.mode),
      control_api_quartz_state_name(q.state), q.analysis ? "on" : "off",
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
      "\"deviceMode\":true,\"beats\":true,\"beatsMax\":%u,"
      "\"beatsPacked\":%u,\"gps\":%s,"
      "\"pps\":%s,\"timebaseLog\":true,\"timebaseLogMax\":%u,%s%s\"blocks\":[",
      p.fw_version, p.board, p.source, p.vid, p.pid, p.max_json,
      p.metrics_hz_max, p.beat_hz_max, p.timing_hz_max, jbool(p.quartz),
      p.quartz_hz_max, p.quartz_pulses_max, static_cast<unsigned>(kBeatsMax),
      static_cast<unsigned>(kBeatsPackedFormat), jbool(p.gps), jbool(p.pps),
      static_cast<unsigned>(kTimebaseLogMax), p.ble ? "\"ble\":true," : "",
      p.cli ? "\"cli\":true," : "");
  for (int i = 1; i < static_cast<int>(DSP_BLOCK_COUNT) && off >= 0; i++) {
    off = appendf(dst, dstlen, off, "%s\"%s\"", i > 1 ? "," : "",
                  dsp_block_json_name(static_cast<dsp_block_id_t>(i)));
  }
  off = appendf(dst, dstlen, off, "],\"pipeline\":");
  uint32_t count = 0;
  const auto *nodes = dsp_pipeline_nodes(&count);
  off = encode_pipeline(dst, dstlen, off, nodes, count, p.quartz);
  off = appendf(dst, dstlen, off, "}");
  if (p.link_kind != nullptr) {
    off = appendf(dst, dstlen, off, ",\"link\":{\"kind\":\"%s\"", p.link_kind);
    if (p.link_mtu != 0u) {
      off = appendf(dst, dstlen, off, ",\"mtu\":%u", p.link_mtu);
    }
    if (p.link_frame_max != 0u) {
      off = appendf(dst, dstlen, off, ",\"frameMax\":%u", p.link_frame_max);
    }
    off = appendf(dst, dstlen, off, "},\"control\":");
    off = encode_control(
        dst, dstlen, off,
        ControlParams{p.control_owner, p.control_self, p.lease_ms});
  }
  return appendf(dst, dstlen, off, "}");
}

int encode_control(char *dst, std::size_t dstlen, int off,
                   const ControlParams &c) {
  if (c.owner != nullptr) {
    return appendf(dst, dstlen, off,
                   "{\"owner\":\"%s\",\"self\":%s,\"leaseMs\":%u}", c.owner,
                   jbool(c.self), c.lease_ms);
  }
  return appendf(dst, dstlen, off,
                 "{\"owner\":null,\"self\":%s,\"leaseMs\":%u}", jbool(c.self),
                 c.lease_ms);
}

int encode_link_status(char *dst, std::size_t dstlen, int off,
                       const LinkStatusParams &s) {
  off = appendf(dst, dstlen, off, "{\"usbHid\":{\"mounted\":%s}",
                jbool(s.usb_mounted));
  if (s.ble != nullptr) {
    const BleLinkParams &b = *s.ble;
    char name[16];
    char address[24];
    json_sanitize(b.name, name, sizeof(name));
    json_sanitize(b.address, address, sizeof(address));
    off = appendf(dst, dstlen, off,
                  ",\"ble\":{\"enabled\":%s,\"name\":\"%s\",\"address\":\"%s\","
                  "\"advertising\":%s,\"connected\":%s,\"mtu\":%u,"
                  "\"connIntervalUs\":%u,\"connParamStep\":%u,"
                  "\"connParamRejects\":%u,\"rxFrames\":%u,\"txFrames\":%u,"
                  "\"rxDropped\":%u,\"rxInvalid\":%u,\"txRetries\":%u,"
                  "\"disconnects\":%u,\"lastDisconnectReason\":%u,\"core\":%u",
                  jbool(b.enabled), name, address, jbool(b.advertising),
                  jbool(b.connected), b.mtu, b.conn_interval_us,
                  b.conn_param_step, b.conn_param_rejects, b.rx_frames,
                  b.tx_frames, b.rx_dropped, b.rx_invalid, b.tx_retries,
                  b.disconnects, b.last_disconnect_reason, b.core);
    if (b.core == 1u && b.core1_stack_free >= 0) {
      off = appendf(dst, dstlen, off, ",\"core1StackFree\":%lu",
                    static_cast<unsigned long>(b.core1_stack_free));
    }
    off = appendf(dst, dstlen, off, "}");
  }
  off = appendf(dst, dstlen, off, ",\"control\":");
  off = encode_control(dst, dstlen, off, s.control);
  return appendf(dst, dstlen, off, "}");
}

int encode_diag(char *dst, std::size_t dstlen, int off,
                const diag_counters_t &d) {
  off = appendf(dst, dstlen, off, "{");
  bool first = true;
  const auto put = [&](const char *key, uint32_t v) {
    off = appendf(dst, dstlen, off, "%s\"%s\":%lu", first ? "" : ",", key,
                  static_cast<unsigned long>(v));
    first = false;
  };
  put("usbEnumerationCount", d.usb_enumeration_count);
  put("usbAudioStreamOpenCount", d.usb_audio_stream_open_count);
  put("usbAudioStreamCloseCount", d.usb_audio_stream_close_count);
  put("usbSuspendCount", d.usb_suspend_count);
  put("usbResumeCount", d.usb_resume_count);
  put("boostOnCount", d.boost_on_count);
  put("boostOffCount", d.boost_off_count);
  put("preampSettleStartCount", d.preamp_settle_start_count);
  put("preampSettleDoneCount", d.preamp_settle_done_count);
  put("preampStartAbortedCount", d.preamp_start_aborted_count);
  put("usbSuspendDurationMs", d.usb_suspend_duration_ms);
  put("adcTotalSampleCount", d.adc_total_sample_count);
  put("adcConversionErrorCount", d.adc_conversion_error_count);
  put("dmaCompleteCount", d.dma_complete_count);
  put("dmaOverrunCount", d.dma_overrun_count);
  put("pcmRingUnderrunCount", d.pcm_ring_underrun_count);
  put("pcmRingOverrunCount", d.pcm_ring_overrun_count);
  put("usbInPacketCount", d.usb_in_packet_count);
  put("usbInSampleCount", d.usb_in_sample_count);
  put("usbShortWriteCount", d.usb_short_write_count);
  put("ringMinOccupancy", d.ring_min_occupancy);
  put("ringMaxOccupancy", d.ring_max_occupancy);
  put("adcMinCode", d.adc_min_code);
  put("adcMaxCode", d.adc_max_code);
  put("adcMeanCodeAccum", d.adc_mean_code_accum);
  put("adcMeanCodeCount", d.adc_mean_code_count);
  put("adcDcCounts", d.adc_dc_counts);
  put("adcAcPeakCounts", d.adc_ac_peak_counts);
  put("adcPcmPeakAbs", d.adc_pcm_peak_abs);
  put("pcm1808ClockConfigOk", d.pcm1808_clock_config_ok);
  put("pcm1808ClkSysHz", d.pcm1808_clk_sys_hz);
  put("pcm1808ClkUsbHz", d.pcm1808_clk_usb_hz);
  put("pcm1808MclkNominalHz", d.pcm1808_mclk_nominal_hz);
  put("pcm1808StartupWaitCount", d.pcm1808_startup_wait_count);
  put("pcm1808DmaBlocks", d.pcm1808_dma_blocks);
  put("pcm1808StereoFramesReceived", d.pcm1808_stereo_frames_received);
  put("pcm1808MonoSamplesEmitted", d.pcm1808_mono_samples_emitted);
  put("pcm1808DmaBlockOverruns", d.pcm1808_dma_block_overruns);
  put("pcm1808PioFifoStallOrOverflow", d.pcm1808_pio_fifo_stall_or_overflow);
  put("pcm1808SamplePeakAbs", d.pcm1808_sample_peak_abs);
  put("pcm1808SaturatedCount", d.pcm1808_saturated_count);
  put("pcm1808RawLeftPeakAbs", d.pcm1808_raw_left_peak_abs);
  put("pcm1808RawRightPeakAbs", d.pcm1808_raw_right_peak_abs);
  put("pcm1808FrameRateMhz", d.pcm1808_frame_rate_mhz);
  put("pcm1808RateStatus", d.pcm1808_rate_status);
  put("pcm1808MclkRatio", d.pcm1808_mclk_ratio);
  put("pcm1808Upsample", d.pcm1808_upsample);
  put("pcm1808BlocksLost", d.pcm1808_blocks_lost);
  return appendf(dst, dstlen, off, "}");
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

namespace {

/** Bytes of the packed header (docs/plans/BLE_TRANSPORT_PLAN.md 14.2). */
constexpr std::size_t kPackedHeaderBytes = 26u;
/** Bytes of one packed beat. */
constexpr std::size_t kPackedBeatBytes = 17u;
/** Characters of the {@code {"p":"} prefix. */
constexpr std::size_t kPackedOpen = 6u;
/** Characters of the {@code "}} suffix. */
constexpr std::size_t kPackedClose = 2u;

/**
 * Streams little-endian bytes into padded base64 without a staging buffer.
 * The caller checks the total size before writing, so {@code put} never
 * runs past the destination.
 */
class Base64Writer {
public:
  /** @param dst Where the next character goes. */
  explicit Base64Writer(char *dst) noexcept : dst_(dst) {}

  /** @param v Next byte of the binary block. */
  void put(uint8_t v) noexcept {
    carry_[filled_++] = v;
    if (filled_ == 3u) {
      emit(carry_[0], carry_[1], carry_[2], 3u);
      filled_ = 0;
    }
  }

  /** @param v Unsigned value, written little-endian in {@code bytes}. */
  void put_le(uint64_t v, unsigned bytes) noexcept {
    for (unsigned i = 0; i < bytes; i++) {
      put(static_cast<uint8_t>(v >> (8u * i)));
    }
  }

  /**
   * Flushes the last one or two bytes with {@code =} padding.
   *
   * @return Where the next character goes (one past the last written).
   */
  char *finish() noexcept {
    if (filled_ != 0u) {
      emit(carry_[0], filled_ > 1u ? carry_[1] : uint8_t{0}, uint8_t{0},
           filled_);
      filled_ = 0;
    }
    return dst_;
  }

private:
  /**
   * Writes four characters for up to three bytes.
   *
   * @param a First byte.
   * @param b Second byte (ignored when {@code count < 2}).
   * @param c Third byte (ignored when {@code count < 3}).
   * @param count Valid bytes, 1..3.
   */
  void emit(uint8_t a, uint8_t b, uint8_t c, unsigned count) noexcept {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const uint32_t w =
        (static_cast<uint32_t>(a) << 16) | (static_cast<uint32_t>(b) << 8) | c;
    dst_[0] = kAlphabet[(w >> 18) & 0x3Fu];
    dst_[1] = kAlphabet[(w >> 12) & 0x3Fu];
    dst_[2] = count > 1u ? kAlphabet[(w >> 6) & 0x3Fu] : '=';
    dst_[3] = count > 2u ? kAlphabet[w & 0x3Fu] : '=';
    dst_ += 4;
  }

  char *dst_;
  std::array<uint8_t, 3> carry_{};
  unsigned filled_ = 0;
};

/** @return Bit pattern of {@code v}, 0 when it is not finite. */
[[nodiscard]] uint32_t f32_bits(float v) noexcept {
  uint32_t bits = 0;
  const float safe = std::isfinite(v) ? v : 0.0f;
  std::memcpy(&bits, &safe, sizeof(bits));
  return bits;
}

/**
 * @param v Value.
 * @param lo Lowest result.
 * @param hi Highest result.
 * @return {@code v} rounded to the nearest integer and clamped to
 *         [lo, hi]; 0 (clamped) for NaN.
 */
[[nodiscard]] int64_t round_clamp(double v, int64_t lo, int64_t hi) noexcept {
  if (!std::isfinite(v)) {
    v = 0.0;
  }
  const double r = std::round(v);
  if (r <= static_cast<double>(lo)) {
    return lo;
  }
  if (r >= static_cast<double>(hi)) {
    return hi;
  }
  return static_cast<int64_t>(r);
}

/**
 * @param prev Previous beat of the batch.
 * @param b Candidate beat.
 * @return true when {@code b} can follow {@code prev} in the same batch.
 */
[[nodiscard]] bool packed_continues(const dsp_beat_t &prev,
                                    const dsp_beat_t &b) noexcept {
  if (b.grid_id != prev.grid_id ||
      f32_bits(b.nominal_us) != f32_bits(prev.nominal_us)) {
    return false;
  }
  const uint32_t d_seq = b.seq - prev.seq;
  const bool seq_ok = b.seq > prev.seq && d_seq <= 255u;
  const bool sample_ok = b.sample_index >= prev.sample_index &&
                         b.sample_index - prev.sample_index <= UINT32_MAX;
  const int64_t d_k = static_cast<int64_t>(b.k) - static_cast<int64_t>(prev.k);
  return seq_ok && sample_ok && d_k >= INT16_MIN && d_k <= INT16_MAX;
}

/**
 * Writes one packed beat (17 bytes).
 *
 * @param w Sink.
 * @param prev Previous beat of the batch, or {@code b} itself for the first.
 * @param b Beat.
 * @param source_hz Sample rate for the fractional sample.
 */
void put_packed_beat(Base64Writer &w, const dsp_beat_t &prev,
                     const dsp_beat_t &b, uint32_t source_hz) noexcept {
  uint8_t status = 0;
  if (b.status == DSP_BEAT_STARTED) {
    status = 1u;
  } else if (b.status == DSP_BEAT_REJECTED) {
    status = 2u;
  }
  const auto flags = static_cast<uint8_t>(
      (b.parity & 1u) | (b.amplitude_valid ? 2u : 0u) | (status << 2u));
  const double frac_samples =
      source_hz != 0u ? num(b.t_s) * static_cast<double>(source_hz) -
                            static_cast<double>(b.sample_index)
                      : 0.0;
  const int64_t d_k = static_cast<int64_t>(b.k) - static_cast<int64_t>(prev.k);
  const double amp = b.amplitude_valid ? num(b.amplitude_deg) : 0.0;
  w.put(static_cast<uint8_t>(b.seq - prev.seq));
  w.put(flags);
  w.put_le(b.sample_index - prev.sample_index, 4);
  w.put(static_cast<uint8_t>(round_clamp(frac_samples * 256.0, 0, 255)));
  w.put_le(static_cast<uint64_t>(static_cast<uint16_t>(d_k)), 2);
  w.put_le(static_cast<uint32_t>(static_cast<int32_t>(
               round_clamp(num(b.u_us) * 10.0, INT32_MIN, INT32_MAX))),
           4);
  w.put_le(static_cast<uint64_t>(round_clamp(amp * 100.0, 0, UINT16_MAX)), 2);
  w.put_le(static_cast<uint64_t>(
               round_clamp(num(b.lift_ms) * 1000.0, 0, UINT16_MAX)),
           2);
}

} // namespace

int encode_beats_packed(char *dst, std::size_t dstlen, int off,
                        const dsp_beat_t *beats, uint32_t n, uint32_t source_hz,
                        uint32_t *emitted) {
  /* Decide the batch length first so the writer never needs a bound. */
  uint32_t count = 0;
  const std::size_t base = off >= 0 ? static_cast<std::size_t>(off) : 0u;
  while (beats != nullptr && count < n && count < kBeatsPackedMax &&
         (count == 0u || packed_continues(beats[count - 1u], beats[count]))) {
    const std::size_t bytes =
        kPackedHeaderBytes + (count + 1u) * kPackedBeatBytes;
    const std::size_t chars = 4u * ((bytes + 2u) / 3u);
    if (base + kPackedOpen + chars + kPackedClose + kEnvelopeTail >= dstlen) {
      break;
    }
    count++;
  }
  if (emitted != nullptr) {
    *emitted = count;
  }
  off = appendf(dst, dstlen, off, "{\"p\":\"");
  if (count != 0u && off >= 0) {
    const dsp_beat_t &first = beats[0];
    Base64Writer w(dst + off);
    w.put(kBeatsPackedFormat);
    w.put(static_cast<uint8_t>(count));
    w.put_le(first.seq, 4);
    w.put_le(first.grid_id, 4);
    w.put_le(f32_bits(first.nominal_us), 4);
    w.put_le(first.sample_index, 8);
    w.put_le(static_cast<uint32_t>(first.k), 4);
    for (uint32_t i = 0; i < count; i++) {
      put_packed_beat(w, i != 0u ? beats[i - 1u] : first, beats[i], source_hz);
    }
    off = static_cast<int>(w.finish() - dst);
  }
  return appendf(dst, dstlen, off, "\"}");
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
  return control::json::parse_i64(s, out);
}

void i64_dec(int64_t v, char (&out)[24]) noexcept {
  control::json::i64_dec(v, out);
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
