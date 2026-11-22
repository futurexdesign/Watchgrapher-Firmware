/**
 * @file hid_json.h
 * @brief Host-testable JSON encoders of the HID control plane: the
 *        bounded {@code appendf} writer, string sanitising, the quartz
 *        timing payloads ({@code getQuartz}, {@code getQuartzPulses}, the
 *        {@code quartz} object of {@code getState}, and the matching
 *        notifies), and the device / beats / pipeline / GPS / timebase-log
 *        payloads plus the {@code hello} result.
 *
 * No Arduino or TinyUSB dependency: the only external symbols are the
 * quartz name helpers of {@link control_api.h} (used by the quartz
 * encoders) and {@code dsp_block_json_name} / {@code dsp_block_info} of
 * dsp_blocks.cpp (used by {@link encode_hello}), so a host test can link
 * this file with small stand-ins for them. The framer, the request
 * decoder and the op table stay in {@code hid_control.cpp}.
 *
 * Size policy: a message is at most 2048 bytes including its envelope
 * ({@code {"v":1,"id":N,"ok":true,"result":...}}). Encoders of unbounded
 * lists never overflow: {@link encode_beats} and
 * {@link encode_timebase_log} emit a prefix that fits (the callers carry the
 * rest in the next notify / page) and keep one byte free for the envelope's
 * closing brace; {@link encode_gps} always fits.
 *
 * Every encoder appends at {@code off} into {@code dst} (capacity
 * {@code dstlen}, NUL-terminated) and returns the new offset, or -1 once
 * the output does not fit; a negative {@code off} is passed through, so
 * calls can be chained and checked once.
 *
 * @see hid_control.cpp, docs/HID_CONTROL_PLANE.md §5.3 / §8.5,
 *      tests/test_hid_json.cpp
 */

#pragma once

#include "device_mode.h"
#include "dsp_blocks.h"
#include "dsp_quartz_types.h"
#include "dsp_types.h"
#include "timebase.h"
#include "timebase_log.h"

#include <cstddef>
#include <cstdint>

namespace hid_json {

/**
 * Most pulses one {@code getQuartzPulses} reply carries. A pulse object is
 * at most {@link kQuartzPulseJsonMax} bytes, so 8 keep the reply within the
 * 2048-byte message limit with margin; the full log
 * ({@link DSP_QUARTZ_PULSE_LOG_CAP}, 16) would not fit.
 */
inline constexpr uint32_t kQuartzPulsesMax = 8u;

/** Upper bound of one encoded {@link dsp_quartz_pulse_t} object (bytes),
 * checked by the host test. */
inline constexpr std::size_t kQuartzPulseJsonMax = 224u;

/** Longest {@code reason} / {@code calSource} text copied into a payload
 * (bytes, without the NUL); longer engine strings are truncated. */
inline constexpr std::size_t kQuartzTextMax = 95u;

/**
 * Bounded {@code snprintf} append.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst} including the NUL.
 * @param off Current length; a negative value is passed through.
 * @param fmt {@code printf} format.
 * @return {@code off} plus the characters written, or -1 if {@code off} is
 *         negative or the result (with its NUL) does not fit.
 */
[[nodiscard]] int appendf(char *dst, std::size_t dstlen, int off,
                          const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

/**
 * Copies {@code in} to {@code out}, dropping control characters, {@code "}
 * and {@code \}, so the result can sit inside a JSON string unescaped.
 *
 * @param in Source; {@code nullptr} gives "".
 * @param out Destination.
 * @param outlen Capacity of {@code out} including the NUL; longer input is
 *        truncated. Must be at least 1.
 */
void json_sanitize(const char *in, char *out, std::size_t outlen);

/**
 * Appends the {@code quartz} object of {@code getState}:
 * {@code {"mode","gateS","forkHz","tsMethod","kSigma"}}.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param q Snapshot from {@link control_api_get_quartz}.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_quartz_settings(char *dst, std::size_t dstlen, int off,
                                         const dsp_quartz_t &q);

/**
 * Appends the {@code getQuartz} result / {@code quartz} notify data: every
 * field of {@link dsp_quartz_t} in camelCase, enums as their stable names,
 * and the {@code motor} / {@code lcd} / {@code fork} detail objects, of
 * which only the active mode's is an object and the others are
 * {@code null} (all three in mechanical mode). Non-finite floats are
 * written as 0; validity is carried by the {@code *Valid} flags.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param q Snapshot from {@link control_api_get_quartz}.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_quartz(char *dst, std::size_t dstlen, int off,
                                const dsp_quartz_t &q);

/**
 * Appends one pulse object ({@code getQuartzPulses} element and the
 * {@code quartzPulse} notify data). {@code sampleIndex} is a decimal
 * string (uint64).
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param p Pulse from {@link control_api_copy_quartz_pulses}.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_quartz_pulse(char *dst, std::size_t dstlen, int off,
                                      const dsp_quartz_pulse_t &p);

/**
 * Appends the {@code getQuartzPulses} result {@code {"pulses":[...]}},
 * newest first as given.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param pulses Pulses, newest first.
 * @param n Number of entries in {@code pulses}.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_quartz_pulses(char *dst, std::size_t dstlen, int off,
                                       const dsp_quartz_pulse_t *pulses,
                                       uint32_t n);

/** Most beats one {@code beats} notify carries. */
inline constexpr uint32_t kBeatsPerNotifyMax = 8u;

/** {@code caps.beatsMax}: largest {@code getBeats.max} (the newest beats). */
inline constexpr uint32_t kBeatsMax = 8u;

/** Upper bound of one encoded {@link dsp_beat_t} object (bytes), checked
 * by the host test with extreme values. */
inline constexpr std::size_t kBeatJsonMax = 288u;

/** Upper bound of one encoded {@link dsp_event_t} object (bytes), checked
 * by the host test with extreme values (four 20-digit sample strings and
 * ten numbers). */
inline constexpr std::size_t kEventJsonMax = 384u;

/** {@code caps.timebaseLogMax}. */
inline constexpr uint32_t kTimebaseLogMax = TIMEBASE_LOG_CAP;

/** Most satellites a {@code getGps} reply lists. */
inline constexpr uint32_t kGpsSatsMax = TIMEBASE_GPS_MAX_SATS;

/** Bytes an encoder of a list leaves free after its own output, for the
 * envelope's closing {@code "}"}. */
inline constexpr std::size_t kEnvelopeTail = 1u;

/** Fixed inputs of the {@code hello} result (everything the firmware, not
 * dsp_blocks, decides). */
struct HelloParams {
  const char *fw_version; /**< {@code fwVersion}. */
  const char *board;      /**< {@code board}. */
  const char *source;     /**< {@code source} (audio backend name). */
  unsigned vid;           /**< USB vendor id. */
  unsigned pid;           /**< USB product id. */
  unsigned max_json;      /**< {@code caps.maxJsonBytes}. */
  unsigned metrics_hz_max;
  unsigned beat_hz_max;
  unsigned timing_hz_max;
  unsigned quartz_hz_max;
  unsigned quartz_pulses_max;
  bool quartz; /**< Build has the quartz engine (hides the quartz node). */
  bool gps;    /**< Build has GPS_UART_RX ({@code caps.gps}). */
  bool pps;    /**< Build has PPS_GPIO ({@code caps.pps}). */
};

/**
 * Appends the {@code hello} result (protocol version, identity and
 * {@code caps} including the block list, the new feature caps and the
 * {@code pipeline}). The pipeline comes from {@link dsp_pipeline_nodes}.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param p Parameters.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_hello(char *dst, std::size_t dstlen, int off,
                               const HelloParams &p);

/**
 * Appends the {@code PipelineNode} array
 * {@code [{"key","group","order","inputs":[..]},...]}. When
 * {@code include_quartz} is false the {@code quartz} node is omitted and
 * so is every input that references it.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param nodes Node table ({@link dsp_pipeline_nodes}).
 * @param count Number of nodes.
 * @param include_quartz Keep the quartz node.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_pipeline(char *dst, std::size_t dstlen, int off,
                                  const dsp_pipeline_node_t *nodes,
                                  uint32_t count, bool include_quartz);

/**
 * Appends a {@code DeviceStatus} object
 * {@code {"mode","running","runRequested","usbStreaming","streamState",
 * "saved"}}.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param s Status from {@link control_api_get_device}.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_device_status(char *dst, std::size_t dstlen, int off,
                                       const device_status_t &s);

/**
 * Appends one {@code DspBeat} object {@code {"seq","gridId","sampleIndex"
 * (decimal string),"tS","k","parity","uUs","nominalUs","amplitudeDeg",
 * "amplitudeValid","liftMs","status"}}. {@code tS} is clamped to
 * +-1e12 s; non-finite floats are 0.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param b Beat.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_beat(char *dst, std::size_t dstlen, int off,
                              const dsp_beat_t &b);

/**
 * Appends {@code {"beats":[...]}} in the given order (newest first for
 * {@code getBeats}, oldest first for the notify). Stops before the first
 * beat that would not fit (keeping {@link kEnvelopeTail} free), so the
 * reply holds a prefix of {@code beats}: for {@code getBeats} the newest
 * ones, for the notify the oldest ones, the rest follow in the next notify.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param beats Beats.
 * @param n Number of entries in {@code beats}.
 * @param emitted Receives how many leading beats were written (may be null).
 * @return New length, or -1 if not even the empty list fits.
 */
[[nodiscard]] int encode_beats(char *dst, std::size_t dstlen, int off,
                               const dsp_beat_t *beats, uint32_t n,
                               uint32_t *emitted);

/**
 * Appends one {@code DspEvent} object {@code {"startSample","peakSample",
 * "unlockSample","dropSample" (decimal strings),"durationSamples",
 * "riseSamples","liftSamples","peakAbs","noiseFloor","snrDb","energy",
 * "confidence"}}. Non-finite floats are 0.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param ev Event.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_event(char *dst, std::size_t dstlen, int off,
                               const dsp_event_t &ev);

/**
 * Appends {@code {"events":[...]}} in the given order (newest first for
 * {@code getEvents}, oldest first for the {@code event} notify). Stops before
 * the first event that would not fit (keeping {@link kEnvelopeTail} free), so
 * the reply holds a prefix of {@code events}: for {@code getEvents} the
 * newest ones, for the notify the oldest unsent ones (the caller advances
 * its cursor by {@code emitted} only).
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param events Events.
 * @param n Number of entries in {@code events}.
 * @param emitted Receives how many leading events were written (may be null).
 * @return New length, or -1 if not even the empty list fits.
 */
[[nodiscard]] int encode_events(char *dst, std::size_t dstlen, int off,
                                const dsp_event_t *events, uint32_t n,
                                uint32_t *emitted);

/**
 * Parses a signed decimal integer at the start of {@code s} (optional
 * leading white space and sign) as a full 64-bit value, independent of the
 * width of {@code long} (32-bit on the RP2350). Trailing characters are
 * ignored (the value sits inside a JSON object). A value outside the int64
 * range is rejected.
 *
 * @param s Text starting at the number.
 * @param out Receives the value on success.
 * @return true if a number was parsed.
 */
[[nodiscard]] bool parse_i64(const char *s, int64_t *out);

/**
 * Formats an int64 as decimal without relying on {@code %lld} (newlib-nano
 * printf has no long long support).
 *
 * @param v Value.
 * @param out Destination (at most 21 bytes are used).
 */
void i64_dec(int64_t v, char (&out)[24]) noexcept;

/**
 * Appends the {@code ping} result {@code {"t":<int64>,"uptimeMs":<u32>}};
 * {@code t} is echoed in full 64-bit width so a JavaScript
 * {@code Date.now()} round-trips.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param t Echoed client timestamp.
 * @param uptime_ms Device uptime.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_ping(char *dst, std::size_t dstlen, int off, int64_t t,
                              uint32_t uptime_ms);

/**
 * Appends a {@code GpsInfo} object. Satellites are compact tuples so that
 * all {@link kGpsSatsMax} (32) always fit one message:
 * {@code "satCols":["sys","prn","el","az","snr","used"],"sats":
 * [["GP",12,45,123,38,1],...]} ({@code used} is 0 / 1). Positions are
 * clamped to their physical range and non-finite floats are 0.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param g Snapshot from {@link control_api_get_gps}.
 * @return New length, or -1 if it does not fit.
 */
[[nodiscard]] int encode_gps(char *dst, std::size_t dstlen, int off,
                             const timebase_gps_t &g);

/**
 * Appends a {@code TimebaseLog} page {@code {"entries":[...],"total",
 * "offset","pending","logDeltaPpm","lastWriteUnix"}}. {@code entries} are
 * the log entries {@code [offset, offset+n)}, oldest first, with as many as
 * fit one message; the client repeats with {@code offset += n} until
 * {@code offset + n == total}. A full ring of 24 needs at most 2 pages.
 * An {@code offset} at or past {@code total} gives an empty page.
 *
 * @param dst Output buffer.
 * @param dstlen Capacity of {@code dst}.
 * @param off Current length (negative passes through).
 * @param log Log snapshot.
 * @param offset First entry of the page (0 = oldest).
 * @param pending A flash write is deferred.
 * @param delta_ppm Append threshold.
 * @param last_write_unix Last flash write (UTC), 0 = none.
 * @param emitted Receives n, the entries in this page (may be null).
 * @return New length, or -1 if not even the empty page fits.
 */
[[nodiscard]] int encode_timebase_log(char *dst, std::size_t dstlen, int off,
                                      const timebase_log_t &log,
                                      uint32_t offset, bool pending,
                                      float delta_ppm, uint32_t last_write_unix,
                                      uint32_t *emitted);

} // namespace hid_json
