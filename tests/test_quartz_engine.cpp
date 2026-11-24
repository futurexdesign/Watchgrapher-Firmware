/**
 * @file test_quartz_engine.cpp
 * @brief Host test for the quartz engine: drives {@link quartz::engine}
 *        with the handoff vectors in 240-frame stereo blocks through the
 *        same {@code process()} entry point the capture loop calls.
 *
 * Covers: mode switching and channel routing (motor / fork LEFT, LCD
 * RIGHT), timebase and source-rate gating, every reset trigger (stream,
 * rate, timebase, mode, parameter, restart, sample loss, index gap), the
 * motor gate (seconds and "until two events") freezing the result, the
 * calibrated vs uncalibrated rates with an injected ppm (and the freeze of
 * that ppm), parameter validation and names, the weak-signal / clip hints,
 * the pulse log, the timing hook, and that the motor / LCD / fork results
 * match the expected values of the handoff §9 table.
 *
 * Build and run (from the repository root):
 *   tests/build_quartz_tests.sh [BUILD_DIR]
 * which builds the CMSIS-DSP kernels for the host and runs this test after
 * the library tests, as
 *   test_quartz_engine docs/handoff/vectors
 */

#include "quartz/quartz_engine.h"
#include "quartz/quartz_types.h"
#include "wav24.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

namespace qe = quartz::engine;

/** Frames per block, as the PCM1808 DMA delivers them. */
constexpr std::size_t kBlock = 240u;

int g_failures = 0; /**< Failed checks. */
int g_checks = 0;   /**< Checks run. */

/** Records one check and prints it when it fails. */
void check(bool ok, const char *what) {
  g_checks++;
  if (!ok) {
    g_failures++;
    std::printf("FAIL: %s\n", what);
  }
}

/** Records a numeric check against a tolerance, printing both values. */
void check_near(double got, double want, double tol, const char *what) {
  const bool ok = std::fabs(got - want) <= tol;
  g_checks++;
  if (!ok) {
    g_failures++;
  }
  std::printf("%s %-44s got %+.5f want %+.5f (tol %.4g)\n",
              ok ? "ok  " : "FAIL", what, got, want, tol);
}

// ------------------------------------------------------------- fake hooks

/** Calibration the fake source reports. */
qe::Calibration g_cal{};
/** Fake µs clock: advances 7 µs per read so every block "takes" 7 µs. */
uint32_t g_clock_us = 0u;
/** Timing sink calls and their last value. */
uint32_t g_timed_calls = 0u;
uint32_t g_timed_last = 0u;

qe::Calibration fake_calibration() noexcept { return g_cal; }
uint32_t fake_now_us() noexcept { return g_clock_us += 7u; }
void fake_block_timed(uint32_t us) noexcept {
  g_timed_calls++;
  g_timed_last = us;
}

/** Fresh engine with the fakes, a verified 48 kHz timebase, no calibration. */
void fresh_engine() {
  g_cal = qe::Calibration{};
  qe::Hooks h{};
  h.calibration = fake_calibration;
  h.now_us = fake_now_us;
  h.block_timed = fake_block_timed;
  qe::init(h);
  qe::set_source_rate(qe::kRateHz);
  qe::set_timebase(true);
}

/** @return A snapshot. */
dsp_quartz_t snap() {
  dsp_quartz_t q{};
  qe::get(&q);
  return q;
}

// ---------------------------------------------------------- vector feed

std::string g_dir; /**< Vector directory. */

/** Absolute source index of the next block, as the capture counts it. */
uint64_t g_index = 0u;

/** What {@link feed} does with the channels. */
enum class Route : uint8_t {
  kStereo,  /**< L and R as recorded. */
  kSwapped, /**< L ↔ R (routing check). */
};

/**
 * Streams a vector through {@link quartz::engine::process} in 240-frame
 * blocks, continuing the absolute index.
 *
 * @param name File name in the vector directory.
 * @param route Channel routing.
 * @param max_s Stop after this many seconds (0 = whole file).
 * @return {@code false} if the file could not be opened.
 */
bool feed(const char *name, Route route = Route::kStereo, double max_s = 0.0) {
  wav24::Reader r;
  std::string err;
  if (!r.open(g_dir + "/" + name, err)) {
    std::printf("FAIL: %s: %s\n", name, err.c_str());
    g_failures++;
    return false;
  }
  const auto limit = max_s > 0.0 ? static_cast<uint64_t>(max_s * r.rate())
                                 : static_cast<uint64_t>(r.frames());
  std::array<int32_t, kBlock> l{};
  std::array<int32_t, kBlock> rr{};
  uint64_t done = 0u;
  while (done < limit) {
    const std::size_t n = r.read(l.data(), rr.data(), kBlock);
    if (n == 0u) {
      break;
    }
    if (route == Route::kSwapped) {
      qe::process(rr.data(), l.data(), n, g_index);
    } else {
      qe::process(l.data(), rr.data(), n, g_index);
    }
    g_index += n;
    done += n;
  }
  return true;
}

/** Feeds {@code blocks} blocks of silence (continuing the index). */
void feed_silence(std::size_t blocks) {
  const std::array<int32_t, kBlock> z{};
  for (std::size_t b = 0; b < blocks; b++) {
    qe::process(z.data(), z.data(), kBlock, g_index);
    g_index += kBlock;
  }
}

// ----------------------------------------------------------------- tests

/** Names, parameter ranges and aliases. */
void test_names_and_params() {
  std::printf("-- names and parameters\n");
  fresh_engine();
  check(qe::mode_from_name("motor") == DSP_QUARTZ_MODE_MOTOR, "mode motor");
  check(qe::mode_from_name("LCD") == DSP_QUARTZ_MODE_LCD, "mode LCD (case)");
  check(qe::mode_from_name("fork") == DSP_QUARTZ_MODE_FORK, "mode fork");
  check(qe::mode_from_name("mechanical") == DSP_QUARTZ_MODE_MECHANICAL,
        "mode mechanical");
  check(qe::mode_from_name("bogus") == DSP_QUARTZ_MODE_COUNT, "mode bogus");
  check(qe::mode_from_name(nullptr) == DSP_QUARTZ_MODE_COUNT, "mode null");
  for (int m = 0; m < DSP_QUARTZ_MODE_COUNT; m++) {
    const auto mode = static_cast<dsp_quartz_mode_t>(m);
    check(qe::mode_from_name(qe::mode_name(mode)) == mode, "mode round trip");
  }
  check(std::strcmp(qe::state_name(DSP_QUARTZ_STATE_WAIT_TIMEBASE),
                    "waitTimebase") == 0,
        "state name");
  check(std::strcmp(qe::inhibition_name(DSP_QUARTZ_INHIB_CORRECTED),
                    "corrected") == 0,
        "inhibition name");
  check(std::strcmp(qe::ts_method_name(DSP_QUARTZ_TS_XCORR), "xcorr") == 0,
        "ts name");
  check(std::strcmp(qe::lcd_status_name(DSP_QUARTZ_LCD_TRACKING), "tracking") ==
            0,
        "lcd status name");
  check(!qe::set_mode(DSP_QUARTZ_MODE_COUNT), "mode out of range refused");

  float v = -1.f;
  check(qe::get_param("gate", &v) && v == 0.f, "default gate 0");
  check(qe::get_param("forkHz", &v) && v == 360.f, "default fork 360");
  check(qe::get_param("ts_method", &v) && v == 0.f, "default threshold");
  check(qe::get_param("kSigma", &v) && v == 8.f, "default kSigma 8");

  check(!qe::set_param("gate", 2.f), "gate 2 refused");
  check(qe::set_param("gate", 3.f), "gate 3 ok");
  check(qe::set_param("gate", 10.f), "gate 10 ok");
  check(qe::set_param("gate", 960.f), "gate 960 ok");
  check(!qe::set_param("gate", 961.f), "gate 961 refused");
  check(!qe::set_param("gate", -1.f), "gate -1 refused");
  check(!qe::set_param("gate", NAN), "gate NaN refused");
  check(qe::set_param("gate", 0.f), "gate 0 ok");
  check(qe::set_param("forkHz", 341.33f) && qe::get_param("fork_hz", &v) &&
            std::fabs(v - 1024.f / 3.f) < 1e-3f,
        "fork 341.33 -> 1024/3");
  check(!qe::set_param("fork_hz", 350.f), "fork 350 refused");
  check(qe::set_param("fork_hz", 360.f), "fork alias");
  check(qe::set_param("tsMethod", 1.f), "ts xcorr");
  check(!qe::set_param("tsMethod", 2.f), "ts 2 refused");
  check(!qe::set_param("tsMethod", 0.5f), "ts 0.5 refused");
  check(qe::set_param("ts_method", 0.f), "ts alias");
  check(!qe::set_param("kSigma", 3.9f), "kSigma 3.9 refused");
  check(!qe::set_param("k_sigma", 20.1f), "kSigma 20.1 refused");
  check(qe::set_param("k_sigma", 8.f), "kSigma alias");
  check(!qe::set_param("nope", 1.f), "unknown param");
  check(!qe::set_param(nullptr, 1.f), "null param");
  check(!qe::get_param("gate", nullptr), "null out");

  // Refusal reasons.
  check(qe::set_param_ex("gate", 10.f) == DSP_QUARTZ_PARAM_OK, "ex ok");
  check(qe::set_param_ex("gate", 2.f) == DSP_QUARTZ_PARAM_OUT_OF_RANGE,
        "ex out of range");
  check(qe::set_param_ex("kSigma", NAN) == DSP_QUARTZ_PARAM_OUT_OF_RANGE,
        "ex NaN of a known name is out of range");
  check(qe::set_param_ex("tsMethod", INFINITY) == DSP_QUARTZ_PARAM_OUT_OF_RANGE,
        "ex inf tsMethod out of range");
  check(qe::set_param_ex("bogus", 1.f) == DSP_QUARTZ_PARAM_UNKNOWN_NAME,
        "ex unknown name");
  check(qe::set_param_ex("bogus", NAN) == DSP_QUARTZ_PARAM_UNKNOWN_NAME,
        "ex unknown name wins over NaN");
  check(qe::set_param_ex(nullptr, 1.f) == DSP_QUARTZ_PARAM_UNKNOWN_NAME,
        "ex null name");
  check(std::strcmp(qe::param_result_name(DSP_QUARTZ_PARAM_OUT_OF_RANGE),
                    "outOfRange") == 0,
        "param result name");
  check(qe::set_param("gate", 0.f), "gate back to 0");

  // Timestamp-method names.
  check(qe::ts_method_from_name("XCORR") == DSP_QUARTZ_TS_XCORR,
        "ts from name (case)");
  check(qe::ts_method_from_name("threshold") == DSP_QUARTZ_TS_THRESHOLD,
        "ts threshold from name");
  check(qe::ts_method_from_name("fft") == DSP_QUARTZ_TS_COUNT, "ts bogus name");
  check(qe::ts_method_from_name(nullptr) == DSP_QUARTZ_TS_COUNT,
        "ts null name");

  // Published fit constants: only in motor mode, equal to the engine's.
  auto q = snap();
  check(q.motor.no_jump_span_s == 0.f && q.motor.jump_threshold_us == 0.f,
        "fit constants zero outside motor");
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  qe::set_timebase(false);
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_WAIT_TIMEBASE &&
            q.motor.no_jump_span_s ==
                static_cast<float>(2.0 * qe::kLongestInhibitionPeriodS) &&
            q.motor.jump_threshold_us ==
                static_cast<float>(qe::kJumpThresholdUs),
        "fit constants published in every motor state");
}

/** Mechanical idle, timebase / rate gating, timing hook. */
void test_gating() {
  std::printf("-- gating\n");
  fresh_engine();
  qe::set_timebase(false);
  auto q = snap();
  check(q.state == DSP_QUARTZ_STATE_IDLE, "mechanical is idle");
  const uint32_t calls = g_timed_calls;
  feed_silence(2);
  check(g_timed_calls == calls + 2u && g_timed_last == 7u,
        "timing hook once per block");

  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  check(qe::mode() == DSP_QUARTZ_MODE_MOTOR, "mode motor");
  feed("ind_pulses_1mV.wav");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_WAIT_TIMEBASE, "waitTimebase");
  check(q.motor.pulses == 0u && !q.rate_valid, "nothing measured unverified");
  check(std::strlen(q.reason) > 0u, "waitTimebase has a reason");

  qe::set_timebase(true);
  qe::set_source_rate(24000u);
  feed("ind_pulses_1mV.wav");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_UNSUPPORTED_RATE, "unsupportedRate");
  check(q.motor.pulses == 0u, "nothing measured at 24 kHz");

  qe::set_source_rate(qe::kRateHz);
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_ACQUIRING, "acquiring once ungated");
  check(std::strcmp(q.reason, "waiting for audio stream") == 0,
        "acquiring reason before samples");
  feed("ind_pulses_1mV.wav");
  q = snap();
  check(q.motor.pulses == 3u, "1 mV: 3 pulses once ungated");
  check(q.state == DSP_QUARTZ_STATE_MEASURING, "measuring with 3 pulses");

  // Timebase change in either direction restarts.
  qe::set_timebase(false);
  qe::set_timebase(true);
  q = snap();
  check(q.motor.pulses == 0u && q.state == DSP_QUARTZ_STATE_ACQUIRING,
        "timebase change restarts");
  feed("ind_pulses_1mV.wav");
  qe::set_timebase(true); // same value: no-op
  check(snap().motor.pulses == 3u, "same timebase is a no-op");
  qe::set_source_rate(qe::kRateHz); // same value: no-op
  check(snap().motor.pulses == 3u, "same rate is a no-op");
  qe::reset_stream();
  check(snap().motor.pulses == 0u, "stream reset restarts");
}

/**
 * Analysis off (USB microphone mode, TIMING_ENGINE_PLAN.md section 8): nothing
 * is measured or timed, the snapshot says so, the mode setting is kept, and
 * switching back starts a new measurement.
 */
void test_analysis_off() {
  std::printf("-- analysis off\n");
  fresh_engine();
  check(qe::analysis() && snap().analysis, "analysis defaults to on");
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  feed("ind_pulses_1mV.wav");
  check(snap().motor.pulses == 3u && snap().analysis, "measuring while on");

  const uint32_t id = snap().measurement_id;
  qe::set_analysis(false);
  auto q = snap();
  check(!qe::analysis() && !q.analysis, "analysis off");
  check(q.state == DSP_QUARTZ_STATE_IDLE && !q.rate_valid &&
            q.motor.pulses == 0u && std::strlen(q.reason) > 0u,
        "off: idle, nothing reported, with a reason");
  check(q.mode == DSP_QUARTZ_MODE_MOTOR && qe::mode() == DSP_QUARTZ_MODE_MOTOR,
        "off: the mode setting is kept");
  check(q.measurement_id != id, "off: new measurement id");

  const uint32_t calls = g_timed_calls;
  feed("ind_pulses_1mV.wav");
  q = snap();
  check(g_timed_calls == calls, "off: no block is processed or timed");
  check(q.state == DSP_QUARTZ_STATE_IDLE && q.motor.pulses == 0u,
        "off: fed blocks measure nothing");
  qe::set_analysis(false); // same value: no-op
  check(snap().measurement_id == q.measurement_id, "off again is a no-op");

  const uint32_t id_off = snap().measurement_id;
  qe::set_analysis(true);
  q = snap();
  check(q.analysis && q.measurement_id != id_off, "on: new measurement id");
  check(q.state == DSP_QUARTZ_STATE_ACQUIRING && q.motor.pulses == 0u,
        "on: acquiring from scratch");
  feed("ind_pulses_1mV.wav");
  check(snap().motor.pulses == 3u && g_timed_calls > calls,
        "on again: measures");
}

/** LEFT / RIGHT routing and index-gap / sample-loss restarts. */
void test_routing_and_restarts() {
  std::printf("-- routing and restarts\n");
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  feed("ind_pulses_1mV.wav", Route::kSwapped);
  check(snap().motor.pulses == 0u, "motor ignores RIGHT");
  qe::reset_stream();
  feed("ind_pulses_1mV.wav");
  check(snap().motor.pulses == 3u, "motor reads LEFT");

  // A gap in the source index restarts the measurement.
  g_index += kBlock;
  feed_silence(1);
  auto q = snap();
  check(q.motor.pulses == 0u && q.elapsed_s < 0.01f, "index gap restarts");

  feed("ind_pulses_1mV.wav");
  qe::notify_sample_loss();
  q = snap();
  check(q.motor.pulses == 0u && q.sample_loss_restarts == 1u,
        "sample loss restarts and is counted");

  check(qe::set_mode(DSP_QUARTZ_MODE_LCD), "set lcd");
  feed("lcd_static_64Hz.wav", Route::kSwapped, 6.0);
  q = snap();
  check(q.lcd.f0_hz != 64u, "lcd ignores LEFT");
  check(q.lcd.retries > 0u, "lcd retries after no pattern");
  check(q.sample_loss_restarts == 0u, "mode change clears loss count");
  check(qe::restart(), "restart lcd");
  feed("lcd_static_64Hz.wav", Route::kStereo, 6.0);
  check(snap().lcd.f0_hz == 64u, "lcd reads RIGHT");

  check(qe::set_mode(DSP_QUARTZ_MODE_MECHANICAL), "back to mechanical");
  check(!qe::restart(), "restart refused in mechanical");
  dsp_quartz_pulse_t p[4]{};
  check(qe::copy_pulses(p, 4u) == 0u, "no pulses outside motor");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_IDLE && !q.rate_valid,
        "mechanical idle again");
}

/** Weak / clip hints and the pulse log on the three-pulse vectors. */
void test_hints_and_pulses() {
  std::printf("-- hints and pulse log\n");
  struct Case {
    const char *file;
    bool weak;
    bool clip;
  };
  const std::array<Case, 4> cases = {
      {{"ind_pulses_20uV.wav", true, false},
       {"ind_pulses_100uV.wav", false, false},
       {"ind_pulses_1mV.wav", false, false},
       {"ind_pulses_2p5mV_clipping.wav", false, true}}};
  for (const auto &c : cases) {
    fresh_engine();
    check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
    feed(c.file);
    const auto q = snap();
    std::printf("     %-32s pulses %u sigma %.0f snr %.1f weak %d clip %d\n",
                c.file, q.motor.pulses,
                static_cast<double>(q.motor.noise_sigma),
                static_cast<double>(q.motor.peak_snr), q.motor.hint_weak_signal,
                q.motor.hint_clipped);
    check(q.motor.pulses == 3u, "3 pulses");
    check(q.motor.hint_weak_signal == c.weak, "weak-signal hint");
    check(q.motor.hint_clipped == c.clip, "clip hint");
  }

  // Pulse log on the 12-pulse duty sweep: newest first, capped by max_n.
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  feed("ind_duty_sweep.wav");
  std::array<dsp_quartz_pulse_t, DSP_QUARTZ_PULSE_LOG_CAP + 4u> p{};
  const uint32_t n = qe::copy_pulses(p.data(), static_cast<uint32_t>(p.size()));
  check(n == 12u, "12 pulses logged");
  bool order = true;
  bool snr = true;
  bool alt = true;
  for (uint32_t i = 0; i < n; i++) {
    order = order && p[i].sequence == n - 1u - i;
    order = order && (i == 0u || p[i].t_s < p[i - 1u].t_s);
    snr = snr && p[i].snr > qe::kWeakSnr && p[i].peak > 0.f;
    alt = alt && (i == 0u || p[i].polarity == -p[i - 1u].polarity);
  }
  check(order, "newest first, timestamps decreasing");
  check(snr, "per-pulse SNR recorded");
  check(alt, "alternating polarity");
  check(p[n - 1u].closed && p[n - 1u].t_s > 0.0, "oldest closed, t_s > 0");
  check(qe::copy_pulses(p.data(), 5u) == 5u, "max_n honoured");
}

/** Motor results, gate freezing and calibration. */
void test_motor() {
  std::printf("-- motor\n");
  // "Until two events" on the 10 s inhibition vector: the gate freezes at
  // the first same-polarity pulse after the second event.
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  feed("ind_inhibition_10s.wav");
  auto q = snap();
  check(q.state == DSP_QUARTZ_STATE_DONE, "gate 0: done after 2 events");
  check(q.motor.inhibition == DSP_QUARTZ_INHIB_CORRECTED, "corrected");
  check(q.motor.events == 2u, "two events");
  check_near(q.motor.rate_without_uncal_s_day, 1.2096, 0.01,
             "10s raw (without inhibition)");
  check_near(q.motor.rate_with_uncal_s_day, 0.154896, 0.02,
             "10s corrected (with inhibition)");
  check(q.motor.inhibition_period_valid && q.motor.inhibition_period_s == 10.f,
        "10s period exact");
  check(q.motor.jump_last_n == 1u && q.motor.jump_mean_n == 1u,
        "10s n = 1 at 8192 Hz");
  check_near(q.motor.jump_last_us, quartz::kInhibitionQuantumUs, 3.0,
             "10s last jump (us)");
  check(q.rate_valid && q.rate_s_day == q.motor.rate_with_s_day,
        "headline = rate with inhibition");
  check(!q.calibrated && std::strcmp(q.cal_source, "none") == 0 &&
            q.rate_s_day == q.motor.rate_with_uncal_s_day,
        "no calibration: calibrated == uncal, flagged");
  const float frozen_elapsed = q.elapsed_s;
  const uint32_t frozen_pulses = q.motor.pulses;
  std::printf("     froze at %.3f s with %u pulses\n",
              static_cast<double>(frozen_elapsed), frozen_pulses);
  feed("ind_inhibition_10s.wav");
  q = snap();
  check(q.elapsed_s == frozen_elapsed && q.motor.pulses == frozen_pulses,
        "done result frozen");
  // A ppm change after the freeze does not move the frozen result.
  g_cal = {5.f, "manual", true};
  q = snap();
  check(!q.calibrated && q.cal_ppm == 0.f, "frozen calibration kept");
  qe::notify_sample_loss();
  check(snap().state == DSP_QUARTZ_STATE_DONE, "sample loss keeps done");

  // Restart with a calibration: calibrated = apply_timebase_ppm(uncal).
  check(qe::restart(), "restart");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_ACQUIRING && q.motor.pulses == 0u &&
            q.elapsed_s == 0.f,
        "restart clears done");
  feed("ind_inhibition_10s.wav");
  q = snap();
  check(q.calibrated && q.cal_ppm == 5.f &&
            std::strcmp(q.cal_source, "manual") == 0,
        "calibration reported");
  const double want_with = quartz::apply_timebase_ppm(
      static_cast<double>(q.motor.rate_with_uncal_s_day), 5.0);
  const double want_without = quartz::apply_timebase_ppm(
      static_cast<double>(q.motor.rate_without_uncal_s_day), 5.0);
  check_near(q.motor.rate_with_s_day, want_with, 1e-4,
             "calibrated with = apply_timebase_ppm");
  check_near(q.motor.rate_without_s_day, want_without, 1e-4,
             "calibrated without = apply_timebase_ppm");
  check_near(q.motor.rate_with_s_day - q.motor.rate_with_uncal_s_day,
             86400.0 * 5e-6, 1e-3, "+5 ppm adds +0.432 s/d");
  g_cal = qe::Calibration{};

  // Timed gate: 10 s on the 31 s no-inhibition vector.
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  check(qe::set_param("gate", 10.f), "gate 10");
  feed("ind_rate_no_inhibition.wav");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_DONE, "gate 10: done");
  check(q.elapsed_s == 10.f, "gate 10: elapsed exactly 10 s");
  check(q.motor.inhibition == DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION,
        "short gate: may include inhibition");
  std::printf("     gate 10 s: %u pulses\n", q.motor.pulses);
  check_near(q.rate_s_day, -0.432, 0.05, "10 s gate rate");
  // A gate change restarts: the whole record now gives the §9 value.
  check(qe::set_param("gate", 60.f), "gate 60");
  check(snap().state == DSP_QUARTZ_STATE_ACQUIRING, "gate change restarts");
  feed("ind_rate_no_inhibition.wav");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_MEASURING, "31 s < 60 s gate: measuring");
  check(q.motor.pulses == 31u, "31 pulses");
  check(!q.motor.hint_weak_signal, "50 uV is not weak");
  check_near(q.rate_s_day, -0.432, 0.01, "no-inhibition rate");

  // 60 s inhibition, "until two events".
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  feed("ind_inhibition_60s.wav");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_DONE, "60s: done after 2 events");
  check_near(q.motor.rate_without_s_day, 0.903139, 0.01, "60s raw");
  check_near(q.motor.rate_with_s_day, 0.200005, 0.02, "60s corrected");
  check(q.motor.inhibition_period_s == 60.f, "60s period exact");
  check(q.motor.jump_last_n == 4u, "60s n = 4 at 8192 Hz");

  // 20 s step, xcorr timestamps.
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  check(qe::set_param("tsMethod", 1.f), "xcorr");
  feed("ind_step_20s.wav");
  q = snap();
  check(q.ts_method == DSP_QUARTZ_TS_XCORR, "xcorr reported");
  check(q.motor.step_s == 20.f, "step 20 s exact");
  check_near(q.rate_s_day, 0.5, 0.01, "20 s step rate (xcorr)");
}

/** LCD and fork results. */
void test_lcd_and_fork() {
  std::printf("-- lcd and fork\n");
  struct Lcd {
    const char *file;
    uint32_t f0;
    double rate;
  };
  const std::array<Lcd, 3> lcds = {{{"lcd_static_64Hz.wav", 64u, 0.2592},
                                    {"lcd_multiplex_32Hz.wav", 32u, -0.6912},
                                    {"lcd_weak_static_64Hz.wav", 64u, 0.0864}}};
  for (const auto &c : lcds) {
    fresh_engine();
    check(qe::set_mode(DSP_QUARTZ_MODE_LCD), "set lcd");
    feed(c.file);
    const auto q = snap();
    check(q.state == DSP_QUARTZ_STATE_MEASURING, "lcd measuring");
    check(q.lcd.f0_hz == c.f0, "lcd f0 exact");
    check(q.lcd.harmonic_count > 0u && q.lcd.blocks > 2u, "lcd harmonics");
    check(q.rate_valid && q.rate_s_day == q.lcd.rate_s_day, "lcd headline");
    check_near(q.rate_s_day, c.rate, 0.005, c.file);
  }

  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_FORK), "set fork");
  feed("ind_tuning_fork_360Hz.wav");
  auto q = snap();
  check(q.state == DSP_QUARTZ_STATE_MEASURING && q.fork.f_nom_hz == 360.f,
        "fork measuring at 360 Hz");
  check_near(q.rate_s_day, 0.5184, 0.005, "fork 360 Hz rate");
  // Selecting another fork restarts: the 360 Hz record no longer fits.
  check(qe::set_param("forkHz", 480.f), "fork 480");
  q = snap();
  check(q.state == DSP_QUARTZ_STATE_ACQUIRING && q.fork.blocks == 0u,
        "fork change restarts");
}

/** Every reset trigger bumps the measurement id once; reads never do. */
void test_measurement_id() {
  std::printf("-- measurement id\n");
  fresh_engine();
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");
  uint32_t id = snap().measurement_id;
  auto bumped = [&id](uint32_t by, const char *what) {
    const uint32_t now = snap().measurement_id;
    check(now == id + by, what);
    id = now;
  };
  feed("ind_pulses_1mV.wav");
  bumped(0u, "first arm after a restart keeps the id");
  bumped(0u, "reading does not change the id");
  dsp_quartz_pulse_t p[3]{};
  check(qe::copy_pulses(p, 3u) == 3u && p[0].measurement_id == id &&
            p[2].measurement_id == id,
        "pulses carry the measurement id");
  check(qe::restart(), "restart");
  bumped(1u, "restart bumps");
  check(qe::set_mode(DSP_QUARTZ_MODE_LCD), "mode lcd");
  bumped(1u, "mode change bumps");
  check(qe::set_mode(DSP_QUARTZ_MODE_LCD), "same mode");
  bumped(0u, "same mode does not bump");
  check(qe::set_param("gate", 10.f), "gate of another mode");
  bumped(0u, "inactive-mode param does not bump");
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "mode motor");
  bumped(1u, "mode change bumps");
  check(qe::set_param("gate", 60.f), "gate change");
  bumped(1u, "active-mode param change bumps");
  check(qe::set_param("gate", 60.f), "same gate");
  bumped(0u, "unchanged param does not bump");
  qe::set_timebase(true);
  bumped(0u, "same timebase does not bump");
  qe::set_timebase(false);
  bumped(1u, "timebase change bumps");
  qe::set_timebase(true);
  bumped(1u, "timebase change bumps again");
  qe::set_source_rate(24000u);
  bumped(1u, "rate change bumps");
  qe::set_source_rate(qe::kRateHz);
  bumped(1u, "rate change bumps again");
  qe::reset_stream();
  bumped(1u, "stream reset bumps");
  qe::notify_sample_loss();
  bumped(0u, "sample loss before arming does not bump");
  feed_silence(1);
  qe::notify_sample_loss();
  bumped(1u, "sample loss while running bumps");
  feed_silence(1);
  bumped(0u, "arming after it does not bump");
  g_index += kBlock;
  feed_silence(1);
  bumped(1u, "index gap bumps");
}

/** Real µs clock for the host timing measurement. */
uint32_t real_now_us() noexcept {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
uint32_t g_real_max_us = 0u; /**< Worst block (µs). */
uint64_t g_real_sum_us = 0u; /**< Sum over blocks (µs). */
uint64_t g_real_blocks = 0u; /**< Blocks timed. */
void real_block_timed(uint32_t us) noexcept {
  g_real_max_us = std::max(g_real_max_us, us);
  g_real_sum_us += us;
  g_real_blocks++;
}

/**
 * Motor fit past its store: a synthetic 1 Hz train of 2100 pulses (35 min),
 * no inhibition, watch +2 s/d, fed through process() exactly like the
 * vectors. Each pulse is a 5 ms rectangle of alternating polarity whose
 * leading edge is a one-sample linear ramp at the true fractional onset, on
 * top of approximately Gaussian noise (SNR about 200). The onset fraction
 * advances by 0.89 sample per step, so any edge-phase bias of the threshold
 * timestamp averages out of the rate.
 *
 * Also times every process() call with a real clock (informational: host
 * CPU, not the M33).
 */
void test_long_train() {
  std::printf("-- long synthetic train (incremental fit past 2048)\n");
  fresh_engine();
  qe::Hooks h{};
  h.calibration = fake_calibration;
  h.now_us = real_now_us;
  h.block_timed = real_block_timed;
  qe::init(h);
  qe::set_source_rate(qe::kRateHz);
  qe::set_timebase(true);
  check(qe::set_mode(DSP_QUARTZ_MODE_MOTOR), "set motor");

  constexpr double kFs = 48000.0;
  constexpr double kRate = 2.0; // s/d, watch gains
  constexpr std::size_t kPulses = 2100u;
  constexpr double kAmp = 200000.0;
  constexpr double kWidthS = 0.005;
  const double period = kFs / (1.0 + kRate / quartz::kSecondsPerDay);
  const double t0 = 0.5 * kFs; // first onset (samples)
  const auto total =
      static_cast<uint64_t>(t0 + period * static_cast<double>(kPulses));
  const auto width = static_cast<uint64_t>(kWidthS * kFs);

  uint32_t rng = 12345u;
  auto noise = [&rng]() {
    // Sum of four uniforms: near-Gaussian, sigma ~1000 codes.
    double s = 0.0;
    for (int k = 0; k < 4; k++) {
      rng = rng * 1664525u + 1013904223u;
      s += static_cast<double>(rng >> 8) / 16777216.0 - 0.5;
    }
    return s * 1732.0;
  };

  g_real_max_us = 0u;
  g_real_sum_us = 0u;
  g_real_blocks = 0u;
  std::array<int32_t, kBlock> l{};
  std::array<int32_t, kBlock> r{};
  const uint64_t base = g_index;
  for (uint64_t i0 = 0; i0 < total; i0 += kBlock) {
    for (std::size_t j = 0; j < kBlock; j++) {
      const auto i = static_cast<double>(i0 + j);
      // Pulse whose window may contain sample i.
      const double k = std::floor((i - t0) / period);
      double x = noise();
      if (k >= 0.0 && k < static_cast<double>(kPulses)) {
        const double onset = t0 + k * period;
        const double d = i - onset; // > 0 inside the pulse
        if (d > 0.0 && d < static_cast<double>(width)) {
          const double pol = (static_cast<uint64_t>(k) % 2u) == 0u ? 1.0 : -1.0;
          x += pol * kAmp * std::min(d, 1.0);
        }
      }
      l[j] = static_cast<int32_t>(std::lround(x));
    }
    qe::process(l.data(), r.data(), kBlock, base + i0);
  }
  g_index = base + ((total + kBlock - 1u) / kBlock) * kBlock;

  const auto q = snap();
  std::printf("     pulses %u fit %u step %.0f rate %+.5f status %s "
              "rebuilding %d\n",
              q.motor.pulses, q.motor.fit_pulses,
              static_cast<double>(q.motor.step_s),
              static_cast<double>(q.rate_s_day),
              qe::inhibition_name(q.motor.inhibition), q.motor.rebuilding);
  check(q.motor.pulses == kPulses, "all 2100 pulses detected");
  check(q.state == DSP_QUARTZ_STATE_MEASURING, "gate 0 without events runs");
  check(q.motor.step_s == 1.f, "step 1 s");
  check(q.motor.events == 0u, "no false inhibition events");
  check(q.motor.inhibition == DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE,
        "no inhibition detected after a long gate");
  check(!q.motor.rebuilding, "fit not left rebuilding");
  check_near(q.rate_s_day, kRate, 0.01, "long-train rate past 2048 pulses");
  std::printf("     host per-block quartz time: worst %u us, mean %.2f us "
              "over %llu blocks (budget 5000 us)\n",
              g_real_max_us,
              static_cast<double>(g_real_sum_us) /
                  static_cast<double>(g_real_blocks),
              static_cast<unsigned long long>(g_real_blocks));
}

} // namespace

int main(int argc, char **argv) {
  g_dir = argc > 1 ? argv[1] : "docs/handoff/vectors";
  test_names_and_params();
  test_gating();
  test_analysis_off();
  test_routing_and_restarts();
  test_hints_and_pulses();
  test_motor();
  test_lcd_and_fork();
  test_measurement_id();
  test_long_train();
  std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
