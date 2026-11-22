/**
 * @file quartz_engine.cpp
 * @brief Quartz-watch timing owner: mode, gating, parameters, the variant
 *        pipeline, motor gate / freeze, calibration and the control-plane
 *        snapshot.
 *
 * State is file-scope (like mechanical/beat_analysis.cpp): one pipeline
 * variant, the parameters, the gating inputs and the per-measurement
 * bookkeeping. A "measurement" starts when the pipeline is configured and
 * reset at the absolute index of the first block it sees ("armed"), and
 * ends at the next reset trigger or, in motor mode, when the gate freezes
 * it. Every reset trigger only clears the bookkeeping and disarms; the
 * pipeline itself is re-configured lazily at the next eligible block, so a
 * trigger costs nothing on the control path and the new measurement's
 * origin is always the true index of its first sample.
 *
 * @see quartz/quartz_engine.h
 */

#include "quartz/quartz_engine.h"

#include "quartz/lcd_timing.h"
#include "quartz/motor_timing.h"
#include "quartz/quartz_types.h"
#include "quartz/tone_timing.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <optional>
#include <type_traits>
#include <variant>

namespace quartz::engine {
namespace {

/** The one pipeline store. Index 0 (monostate) = mechanical mode. */
using Pipeline =
    std::variant<std::monostate, MotorTiming, LcdTiming, ToneTiming>;

// A trivially destructible variant with a constexpr default constructor is
// constant-initialised into .bss with no static constructor or atexit
// destructor, so a build that never references the engine (ADC, synthetic)
// drops its ~41 KB with --gc-sections.
static_assert(std::is_trivially_destructible_v<Pipeline>,
              "pipeline must not need a static destructor");
static_assert(DSP_QUARTZ_PULSE_LOG_CAP == MotorTiming::kRecentPulses,
              "pulse log must mirror the library history");
static_assert(DSP_QUARTZ_MAX_HARMONICS == HarmonicPhaseTracker::kMaxHarmonics,
              "harmonic list must mirror the tracker");

/** Library detector defaults (kSigma 8, 50 ms grouping). */
constexpr PulseDetector::Config kDefaultDetector{};

/** Library fit defaults, the reference's values. */
constexpr MotorRateFit::Config kDefaultFit{};
// The engine's published constants are what it configures; these asserts
// make a change of the library (reference) defaults a review point instead
// of a silent divergence.
static_assert(kLongestInhibitionPeriodS ==
                  kDefaultFit.longest_inhibition_period_s,
              "engine and library disagree on the longest period");
static_assert(kJumpThresholdUs == kDefaultFit.jump_threshold_us,
              "engine and library disagree on the jump threshold");

/** Tolerance for matching a requested fork frequency (Hz): accepts the
 * displayed 341.33 for the exact 1024/3 Hz, while the specified forks are
 * at least 18 Hz apart. */
constexpr double kForkMatchHz = 0.01;

/** Signal time between motor re-solves when no new pulse arrived (s): picks
 * up the extras and clip flags a group close adds 50 ms after its pulse,
 * and the §5 status change once the span reaches twice the longest
 * expected inhibition period. */
constexpr double kResolvePeriodS = 1.0;

/**
 * @param hz Requested frequency (Hz).
 * @return Index into {@link kForkFrequencies}, or nullopt if none matches.
 */
constexpr std::optional<std::size_t> fork_index(double hz) noexcept {
  for (std::size_t i = 0; i < kForkFrequencies.size(); i++) {
    const double d = kForkFrequencies[i].hz() - hz;
    if (d < kForkMatchHz && d > -kForkMatchHz) {
      return i;
    }
  }
  return std::nullopt;
}

static_assert(fork_index(kDefaultForkHz).has_value(),
              "default fork must be a specified frequency");

/** User parameters; they survive every reset. */
struct Params {
  float gate_s = kGateUntilEvents;                  /**< Motor gate. */
  std::size_t fork = *fork_index(kDefaultForkHz);   /**< Fork index. */
  TimestampMethod ts = TimestampMethod::kThreshold; /**< Edge method. */
  float k_sigma = kDefaultDetector.k_sigma;         /**< Threshold. */
};

/** Bookkeeping of one measurement; cleared by every reset trigger. */
struct Measurement {
  bool armed = false;           /**< Pipeline configured and reset. */
  bool config_failed = false;   /**< The pipeline refused the rate. */
  uint64_t origin = 0;          /**< Absolute index of the first sample. */
  uint64_t next_index = 0;      /**< Index the next block must start at. */
  uint64_t samples = 0;         /**< Samples fed (signal time × fs). */
  bool done = false;            /**< Motor gate reached; result frozen. */
  const char *done_reason = ""; /**< Why it froze. */
  Calibration frozen{};         /**< Correction captured at the freeze. */
  std::optional<MotorRate> motor_rate; /**< Last motor solve. */
  uint32_t solved_pulses = 0;          /**< Pulse count at the last solve. */
  uint64_t solved_at = 0; /**< {@link samples} at the last solve. */
  /** Peak SNR per pulse, slot = sequence % cap. */
  std::array<float, DSP_QUARTZ_PULSE_LOG_CAP> snr{};
  uint32_t snr_pulses = 0;  /**< Pulses whose SNR was recorded. */
  uint32_t lcd_retries = 0; /**< LCD pattern searches restarted. */
};

Pipeline g_pipeline;
Hooks g_hooks{};
Params g_params{};
Measurement g_meas{};
dsp_quartz_mode_t g_mode = DSP_QUARTZ_MODE_MECHANICAL;
uint32_t g_source_hz = 0u;
bool g_timebase_ok = false;
uint32_t g_sample_loss_restarts = 0u;
/** Incremented on every (re)start of a measurement; never reset, so a
 * client never sees an old value reused (see dsp_quartz_t). */
uint32_t g_measurement_id = 0u;

/** Disarms and clears the bookkeeping, and starts a new measurement id; the
 * next block arms the pipeline afresh. Every reset trigger comes here. */
void restart_measurement() noexcept {
  g_measurement_id++;
  g_meas = Measurement{};
}

/** @return The correction in effect now (none if no source is injected). */
Calibration current_calibration() noexcept {
  if (g_hooks.calibration == nullptr) {
    return Calibration{};
  }
  Calibration c = g_hooks.calibration();
  if (c.source == nullptr) {
    c.source = "none";
  }
  return c;
}

/**
 * @param uncal Rate against the board clock (s/d).
 * @param cal Correction.
 * @return The rate against true time (s/d).
 */
float calibrate(double uncal, const Calibration &cal) noexcept {
  return static_cast<float>(
      apply_timebase_ppm(uncal, static_cast<double>(cal.ppm)));
}

/**
 * The gating state, which wins over any measurement state.
 *
 * @param reason Receives why, when gated.
 * @return The gated state, or nullopt when the engine may measure.
 */
std::optional<dsp_quartz_state_t> gated(const char **reason) noexcept {
  if (g_mode == DSP_QUARTZ_MODE_MECHANICAL) {
    *reason = "mechanical mode";
    return DSP_QUARTZ_STATE_IDLE;
  }
  if (!g_timebase_ok) {
    *reason = "capture rate not verified";
    return DSP_QUARTZ_STATE_WAIT_TIMEBASE;
  }
  if (g_source_hz != kRateHz) {
    *reason = "quartz timing needs a 48 kHz capture rate";
    return DSP_QUARTZ_STATE_UNSUPPORTED_RATE;
  }
  return std::nullopt;
}

/** @return Motor pipeline parameters from the user parameters. */
MotorTiming::Config motor_config() noexcept {
  MotorTiming::Config cfg{};
  cfg.method = g_params.ts;
  cfg.detector.k_sigma = g_params.k_sigma;
  // Set explicitly (not left to the library default) so the values the
  // control plane reports are, by construction, the ones in use.
  cfg.fit.longest_inhibition_period_s = kLongestInhibitionPeriodS;
  cfg.fit.jump_threshold_us = kJumpThresholdUs;
  return cfg;
}

/**
 * Configures and resets the active pipeline so that its sample 0 is
 * {@code index}. Configure always precedes reset because configure resets
 * to index 0 and the phase methods reference absolute indices.
 *
 * @param index Absolute index of the first sample of the measurement.
 */
void arm(uint64_t index) noexcept {
  // The first arm after a restart belongs to that restart's id; a re-arm of
  // a running measurement (a gap in the source index) is a new measurement.
  if (g_meas.armed) {
    g_measurement_id++;
  }
  g_meas = Measurement{};
  bool ok = false;
  if (auto *m = std::get_if<MotorTiming>(&g_pipeline)) {
    ok = m->configure(g_source_hz, motor_config());
    if (ok) {
      m->reset(index);
    }
  } else if (auto *l = std::get_if<LcdTiming>(&g_pipeline)) {
    ok = l->configure(g_source_hz);
    if (ok) {
      l->reset(index);
    }
  } else if (auto *t = std::get_if<ToneTiming>(&g_pipeline)) {
    const ToneTiming::Config cfg{kForkFrequencies[g_params.fork],
                                 ToneTiming::kForkBlockSeconds};
    ok = t->configure(g_source_hz, cfg);
    if (ok) {
      t->reset(index);
    }
  }
  g_meas.armed = ok;
  g_meas.config_failed = !ok;
  g_meas.origin = index;
  g_meas.next_index = index;
}

/** Freezes the motor result (state DONE). */
void freeze(const char *why) noexcept {
  g_meas.done = true;
  g_meas.done_reason = why;
  g_meas.frozen = current_calibration();
}

/** Records the peak SNR of pulses detected since the last call. */
void record_snr(const MotorTiming &m) noexcept {
  const uint32_t total = m.pulse_count();
  const float sigma = m.detector().noise_sigma();
  for (; g_meas.snr_pulses < total; g_meas.snr_pulses++) {
    const auto info = m.recent(total - 1u - g_meas.snr_pulses);
    const float snr =
        (info.has_value() && sigma > 0.f) ? info->peak / sigma : 0.f;
    g_meas.snr[g_meas.snr_pulses % DSP_QUARTZ_PULSE_LOG_CAP] = snr;
  }
}

/**
 * Motor block: feed up to the gate, re-solve when something changed, and
 * freeze at the gate.
 */
void feed_motor(MotorTiming &m, const int32_t *left, std::size_t n) noexcept {
  const auto fs = static_cast<double>(g_source_hz);
  const bool timed_gate = g_params.gate_s > kGateUntilEvents;
  const auto gate_samples =
      timed_gate ? static_cast<uint64_t>(
                       std::llround(static_cast<double>(g_params.gate_s) * fs))
                 : 0u;
  // Split the block at the gate so the frozen record is exactly gate_s long;
  // once the gate is reached no more samples are fed (see below).
  std::size_t use = n;
  if (timed_gate) {
    use = g_meas.samples >= gate_samples
              ? 0u
              : static_cast<std::size_t>(
                    std::min<uint64_t>(n, gate_samples - g_meas.samples));
  }
  if (use > 0u) {
    m.process(left, use);
    g_meas.samples += use;
    record_snr(m);
  }

  // Solve schedule: on every new pulse, once per second of signal (group
  // closes add extras and clip flags; the status changes with the span), at
  // the gate, and on EVERY block while the fit is rebuilding. A rebuild
  // replays at most MotorRateFit::kReplayPulsesPerCall pulses per call, so
  // a once-per-second schedule alone would take minutes to catch up after a
  // step change on a full store; per block it takes well under a second.
  // solve_incremental() has a bounded cost per call, which is what makes
  // calling it inside the capture path safe.
  const bool gate_hit = timed_gate && g_meas.samples >= gate_samples;
  const auto since_solve =
      static_cast<double>(g_meas.samples - g_meas.solved_at);
  if (gate_hit || m.fit().rebuilding() ||
      m.pulse_count() != g_meas.solved_pulses ||
      since_solve >= kResolvePeriodS * fs) {
    g_meas.motor_rate = m.solve_incremental();
    g_meas.solved_pulses = m.pulse_count();
    g_meas.solved_at = g_meas.samples;
  }

  // While rebuilding, solve_incremental() returns the last good result, not
  // one that covers the whole record: never freeze that. With a timed gate
  // the record is already complete (nothing more is fed), so the next blocks
  // only finish the replay and the freeze follows.
  if (m.fit().rebuilding()) {
    return;
  }
  if (gate_hit) {
    freeze(g_meas.motor_rate ? "gate reached" : "gate reached before 3 pulses");
  } else if (!timed_gate && g_meas.motor_rate &&
             g_meas.motor_rate->status == InhibitionStatus::kCorrected) {
    freeze("two inhibition events seen");
  }
}

/** LCD block; a failed pattern search starts a new one at the next block
 * (the watch is often placed after the mode is selected). */
void feed_lcd(LcdTiming &l, const int32_t *right, std::size_t n) noexcept {
  l.process(right, n);
  g_meas.samples += n;
  if (l.result().status == LcdStatus::kNoPattern) {
    l.reset(g_meas.next_index + n);
    g_meas.lcd_retries++;
  }
}

/** Body of {@link process} without the timing. */
void run_block(const int32_t *left, const int32_t *right, std::size_t n,
               uint64_t index) noexcept {
  const char *why = nullptr;
  if (n == 0u || gated(&why) || g_meas.done || g_meas.config_failed) {
    return;
  }
  if (!g_meas.armed || index != g_meas.next_index) {
    arm(index);
    if (!g_meas.armed) {
      return;
    }
  }
  if (auto *m = std::get_if<MotorTiming>(&g_pipeline)) {
    if (left != nullptr) {
      feed_motor(*m, left, n);
    }
  } else if (auto *l = std::get_if<LcdTiming>(&g_pipeline)) {
    if (right != nullptr) {
      feed_lcd(*l, right, n);
    }
  } else if (auto *t = std::get_if<ToneTiming>(&g_pipeline)) {
    if (left != nullptr) {
      t->process(left, n);
      g_meas.samples += n;
    }
  }
  g_meas.next_index = index + n;
}

/** @return {@code s} as the C enum ({@code NONE} is never produced). */
dsp_quartz_inhibition_t to_c(InhibitionStatus s) noexcept {
  switch (s) {
  case InhibitionStatus::kNoJumpLongGate:
    return DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE;
  case InhibitionStatus::kMayIncludeInhibition:
    return DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION;
  case InhibitionStatus::kNeedOneMoreCycle:
    return DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE;
  case InhibitionStatus::kCorrectionPending:
    return DSP_QUARTZ_INHIB_CORRECTION_PENDING;
  case InhibitionStatus::kCorrected:
    return DSP_QUARTZ_INHIB_CORRECTED;
  }
  return DSP_QUARTZ_INHIB_NONE;
}

/** @return {@code s} as the C enum. */
dsp_quartz_lcd_status_t to_c(LcdStatus s) noexcept {
  switch (s) {
  case LcdStatus::kDetectingPeriod:
    return DSP_QUARTZ_LCD_DETECTING_PERIOD;
  case LcdStatus::kNoPattern:
    return DSP_QUARTZ_LCD_NO_PATTERN;
  case LcdStatus::kUnsupportedRate:
    return DSP_QUARTZ_LCD_UNSUPPORTED_RATE;
  case LcdStatus::kTracking:
    return DSP_QUARTZ_LCD_TRACKING;
  }
  return DSP_QUARTZ_LCD_DETECTING_PERIOD;
}

/**
 * Median of the recorded pulse SNRs (at most the last
 * {@code DSP_QUARTZ_PULSE_LOG_CAP}). The median keeps one odd pulse (a
 * correction burst, a knock) from toggling the weak-signal hint.
 *
 * @return The median, 0 with no pulse.
 */
float median_snr() noexcept {
  const auto n = static_cast<std::size_t>(
      std::min<uint32_t>(g_meas.snr_pulses, DSP_QUARTZ_PULSE_LOG_CAP));
  if (n == 0u) {
    return 0.f;
  }
  std::array<float, DSP_QUARTZ_PULSE_LOG_CAP> v = g_meas.snr;
  std::sort(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n));
  return (n % 2u) != 0u ? v[n / 2u] : 0.5f * (v[n / 2u - 1u] + v[n / 2u]);
}

/** Fills the motor detail. @return {@code true} if a rate exists. */
bool fill_motor(const MotorTiming &m, const Calibration &cal,
                dsp_quartz_motor_t &o) noexcept {
  o.pulses = m.pulse_count();
  o.noise_sigma = m.detector().noise_sigma();
  o.threshold = m.detector().threshold();
  o.clipped = m.clipped();
  o.peak_snr = median_snr();
  o.hint_weak_signal = g_meas.snr_pulses > 0u && o.peak_snr < kWeakSnr;
  o.hint_clipped = o.clipped;
  o.rebuilding = m.fit().rebuilding();
  const auto &r = g_meas.motor_rate;
  if (!r) {
    return false;
  }
  o.valid = true;
  o.fit_pulses = r->pulses;
  o.step_s = static_cast<float>(r->step_s);
  o.span_s = static_cast<float>(r->span_s);
  o.rate_with_uncal_s_day =
      static_cast<float>(r->rate_with_inhibition_s_per_day);
  o.rate_with_s_day = calibrate(r->rate_with_inhibition_s_per_day, cal);
  o.rate_without_uncal_s_day =
      static_cast<float>(r->rate_without_inhibition_s_per_day);
  o.rate_without_s_day = calibrate(r->rate_without_inhibition_s_per_day, cal);
  o.inhibition = to_c(r->status);
  o.inhibition_period_valid = r->inhibition_period_s.has_value();
  o.inhibition_period_s =
      static_cast<float>(r->inhibition_period_s.value_or(0.0));
  o.events = r->inhibition_events;
  o.cycles_used = r->inhibition_cycles_used;
  // The stream jump list belongs to the incremental path (the batch jump
  // list is only filled by solve(), which the engine never calls).
  const std::size_t jumps = m.fit().stream_jump_count();
  if (jumps > 0u) {
    const float last = m.fit().stream_jump_us(jumps - 1u);
    o.jump_last_us = last;
    o.jump_last_n = static_cast<uint32_t>(
        std::lround(static_cast<double>(last) / kInhibitionQuantumUs));
  }
  o.jump_mean_us = static_cast<float>(r->jump_mean_us);
  o.jump_mean_n = r->n_at_8192;
  o.residual_rms_us = static_cast<float>(r->residual_rms_us);
  o.polarity_offset_us = static_cast<float>(r->polarity_offset_us);
  o.clipped_pulses = r->clipped_pulses;
  o.extra_pulses = r->extra_pulses;
  return true;
}

/** Fills the LCD detail. @return {@code true} if a rate exists. */
bool fill_lcd(const LcdTiming &l, const Calibration &cal,
              dsp_quartz_lcd_t &o) noexcept {
  const LcdResult r = l.result();
  o.status = to_c(r.status);
  o.f0_hz = r.pattern_hz;
  o.retries = g_meas.lcd_retries;
  o.clipped = l.clipped();
  if (!r.rate) {
    return false;
  }
  o.valid = true;
  o.harmonic_count = r.rate->harmonic_count;
  for (std::size_t i = 0; i < r.rate->harmonic_count; i++) {
    o.harmonics[i] = r.rate->harmonics[i];
  }
  o.blocks = r.rate->blocks;
  o.rate_uncal_s_day = static_cast<float>(r.rate->rate_s_per_day);
  o.rate_s_day = calibrate(r.rate->rate_s_per_day, cal);
  return true;
}

/** Fills the fork detail. @return {@code true} if a rate exists. */
bool fill_fork(const ToneTiming &t, const Calibration &cal,
               dsp_quartz_fork_t &o) noexcept {
  o.f_nom_hz = static_cast<float>(kForkFrequencies[g_params.fork].hz());
  o.clipped = t.clipped();
  const auto r = t.result();
  if (!r) {
    return false;
  }
  o.valid = true;
  o.blocks = r->blocks;
  o.rate_uncal_s_day = static_cast<float>(r->rate_s_per_day);
  o.rate_s_day = calibrate(r->rate_s_per_day, cal);
  return true;
}

/** @return Why a running measurement has no rate yet. */
const char *acquiring_reason(const dsp_quartz_t &q) noexcept {
  if (g_meas.samples == 0u) {
    return "waiting for audio stream";
  }
  switch (q.mode) {
  case DSP_QUARTZ_MODE_MOTOR:
    return "waiting for 3 pulses";
  case DSP_QUARTZ_MODE_LCD:
    if (q.lcd.status == DSP_QUARTZ_LCD_UNSUPPORTED_RATE) {
      return "LCD pattern period does not fit the buffer";
    }
    if (q.lcd.status == DSP_QUARTZ_LCD_TRACKING) {
      return "collecting phase blocks";
    }
    return g_meas.lcd_retries > 0u ? "no LCD pattern yet; retrying"
                                   : "detecting LCD pattern rate";
  case DSP_QUARTZ_MODE_FORK:
    return "collecting phase blocks";
  default:
    return "";
  }
}

/**
 * @param param Requested name.
 * @param camel camelCase name.
 * @param snake snake_case alias (may equal camel).
 * @return {@code true} if {@code param} is either.
 */
bool name_is(const char *param, const char *camel, const char *snake) noexcept {
  return std::strcmp(param, camel) == 0 || std::strcmp(param, snake) == 0;
}

/** Case-insensitive ASCII equality. */
bool equals_ignore_case(const char *a, const char *b) noexcept {
  while (*a != '\0' && *b != '\0') {
    if (std::tolower(static_cast<unsigned char>(*a)) !=
        std::tolower(static_cast<unsigned char>(*b))) {
      return false;
    }
    a++;
    b++;
  }
  return *a == *b;
}

/** Stable mode names, indexed by {@code dsp_quartz_mode_t}. */
constexpr std::array<const char *, DSP_QUARTZ_MODE_COUNT> kModeNames = {
    "mechanical", "motor", "lcd", "fork"};

/** Stable state names, indexed by {@code dsp_quartz_state_t}. */
constexpr std::array<const char *, DSP_QUARTZ_STATE_COUNT> kStateNames = {
    "idle",      "waitTimebase", "unsupportedRate",
    "acquiring", "measuring",    "done"};

/** Stable timestamp-method names, indexed by {@code dsp_quartz_ts_method_t}.
 */
constexpr std::array<const char *, DSP_QUARTZ_TS_COUNT> kTsNames = {"threshold",
                                                                    "xcorr"};

/** Stable parameter-result names, indexed by
 * {@code dsp_quartz_param_result_t}. */
constexpr std::array<const char *, DSP_QUARTZ_PARAM_COUNT> kParamResultNames = {
    "ok", "unknownName", "outOfRange", "noEngine"};

} // namespace

void init(const Hooks &hooks) noexcept {
  g_hooks = hooks;
  g_params = Params{};
  g_pipeline.emplace<std::monostate>();
  g_mode = DSP_QUARTZ_MODE_MECHANICAL;
  g_source_hz = 0u;
  g_timebase_ok = false;
  g_sample_loss_restarts = 0u;
  restart_measurement();
}

void reset_stream() noexcept { restart_measurement(); }

void set_source_rate(uint32_t hz) noexcept {
  if (hz == g_source_hz) {
    return;
  }
  g_source_hz = hz;
  restart_measurement();
}

void set_timebase(bool ok) noexcept {
  if (ok == g_timebase_ok) {
    return;
  }
  g_timebase_ok = ok;
  restart_measurement();
}

void notify_sample_loss() noexcept {
  // A frozen result was complete before the loss; keep it.
  if (!g_meas.armed || g_meas.done) {
    return;
  }
  g_sample_loss_restarts++;
  restart_measurement();
}

void process(const int32_t *left, const int32_t *right, std::size_t n,
             uint64_t source_index) noexcept {
  const bool timed = g_hooks.now_us != nullptr && g_hooks.block_timed;
  const uint32_t t0 = timed ? g_hooks.now_us() : 0u;
  run_block(left, right, n, source_index);
  if (timed) {
    g_hooks.block_timed(g_hooks.now_us() - t0);
  }
}

bool set_mode(dsp_quartz_mode_t mode) noexcept {
  if (static_cast<unsigned>(mode) >= DSP_QUARTZ_MODE_COUNT) {
    return false;
  }
  if (mode == g_mode) {
    return true;
  }
  switch (mode) {
  case DSP_QUARTZ_MODE_MOTOR:
    g_pipeline.emplace<MotorTiming>();
    break;
  case DSP_QUARTZ_MODE_LCD:
    g_pipeline.emplace<LcdTiming>();
    break;
  case DSP_QUARTZ_MODE_FORK:
    g_pipeline.emplace<ToneTiming>();
    break;
  default:
    g_pipeline.emplace<std::monostate>();
    break;
  }
  g_mode = mode;
  g_sample_loss_restarts = 0u;
  restart_measurement();
  return true;
}

dsp_quartz_mode_t mode() noexcept { return g_mode; }

bool restart() noexcept {
  if (g_mode == DSP_QUARTZ_MODE_MECHANICAL) {
    return false;
  }
  restart_measurement();
  return true;
}

bool set_param(const char *param, float value) noexcept {
  return set_param_ex(param, value) == DSP_QUARTZ_PARAM_OK;
}

dsp_quartz_param_result_t set_param_ex(const char *param,
                                       float value) noexcept {
  // Name first, so a bad value of a known name reads OUT_OF_RANGE.
  if (param == nullptr) {
    return DSP_QUARTZ_PARAM_UNKNOWN_NAME;
  }
  const bool finite = std::isfinite(value);
  if (std::strcmp(param, "gate") == 0) {
    if (!finite || (value != kGateUntilEvents &&
                    (value < kGateMinS || value > kGateMaxS))) {
      return DSP_QUARTZ_PARAM_OUT_OF_RANGE;
    }
    if (value != g_params.gate_s) {
      g_params.gate_s = value;
      if (g_mode == DSP_QUARTZ_MODE_MOTOR) {
        restart_measurement();
      }
    }
    return DSP_QUARTZ_PARAM_OK;
  }
  if (name_is(param, "forkHz", "fork_hz")) {
    const auto idx =
        finite ? fork_index(static_cast<double>(value)) : std::nullopt;
    if (!idx) {
      return DSP_QUARTZ_PARAM_OUT_OF_RANGE;
    }
    if (*idx != g_params.fork) {
      g_params.fork = *idx;
      if (g_mode == DSP_QUARTZ_MODE_FORK) {
        restart_measurement();
      }
    }
    return DSP_QUARTZ_PARAM_OK;
  }
  if (name_is(param, "tsMethod", "ts_method")) {
    TimestampMethod ts{};
    if (value == static_cast<float>(DSP_QUARTZ_TS_THRESHOLD)) {
      ts = TimestampMethod::kThreshold;
    } else if (value == static_cast<float>(DSP_QUARTZ_TS_XCORR)) {
      ts = TimestampMethod::kXcorr;
    } else {
      return DSP_QUARTZ_PARAM_OUT_OF_RANGE; // also NaN / inf
    }
    if (ts != g_params.ts) {
      g_params.ts = ts;
      if (g_mode == DSP_QUARTZ_MODE_MOTOR) {
        restart_measurement();
      }
    }
    return DSP_QUARTZ_PARAM_OK;
  }
  if (name_is(param, "kSigma", "k_sigma")) {
    if (!finite || value < kKSigmaMin || value > kKSigmaMax) {
      return DSP_QUARTZ_PARAM_OUT_OF_RANGE;
    }
    if (value != g_params.k_sigma) {
      g_params.k_sigma = value;
      if (g_mode == DSP_QUARTZ_MODE_MOTOR) {
        restart_measurement();
      }
    }
    return DSP_QUARTZ_PARAM_OK;
  }
  return DSP_QUARTZ_PARAM_UNKNOWN_NAME;
}

bool get_param(const char *param, float *out) noexcept {
  if (param == nullptr || out == nullptr) {
    return false;
  }
  if (std::strcmp(param, "gate") == 0) {
    *out = g_params.gate_s;
    return true;
  }
  if (name_is(param, "forkHz", "fork_hz")) {
    *out = static_cast<float>(kForkFrequencies[g_params.fork].hz());
    return true;
  }
  if (name_is(param, "tsMethod", "ts_method")) {
    *out = g_params.ts == TimestampMethod::kXcorr
               ? static_cast<float>(DSP_QUARTZ_TS_XCORR)
               : static_cast<float>(DSP_QUARTZ_TS_THRESHOLD);
    return true;
  }
  if (name_is(param, "kSigma", "k_sigma")) {
    *out = g_params.k_sigma;
    return true;
  }
  return false;
}

void get(dsp_quartz_t *out) noexcept {
  if (out == nullptr) {
    return;
  }
  dsp_quartz_t q{};
  q.mode = g_mode;
  q.source_hz = g_source_hz;
  q.timebase_ok = g_timebase_ok;
  q.gate_s = g_params.gate_s;
  q.fork_hz = static_cast<float>(kForkFrequencies[g_params.fork].hz());
  q.ts_method = g_params.ts == TimestampMethod::kXcorr
                    ? DSP_QUARTZ_TS_XCORR
                    : DSP_QUARTZ_TS_THRESHOLD;
  q.k_sigma = g_params.k_sigma;
  q.sample_loss_restarts = g_sample_loss_restarts;
  q.measurement_id = g_measurement_id;
  if (g_mode == DSP_QUARTZ_MODE_MOTOR) {
    q.motor.no_jump_span_s = static_cast<float>(kNoJumpSpanS);
    q.motor.jump_threshold_us = static_cast<float>(kJumpThresholdUs);
  }
  // A frozen result keeps the correction it was frozen with, so a PPS
  // update cannot change a finished measurement.
  const Calibration cal = g_meas.done ? g_meas.frozen : current_calibration();
  q.cal_ppm = cal.ppm;
  q.cal_source = cal.source;
  q.calibrated = cal.calibrated;
  q.reason = "";

  if (const auto gs = gated(&q.reason)) {
    q.state = *gs;
    *out = q;
    return;
  }
  if (g_meas.config_failed) {
    q.state = DSP_QUARTZ_STATE_UNSUPPORTED_RATE;
    q.reason = "pipeline rejected the source rate";
    *out = q;
    return;
  }
  // Without an arm the pipeline still holds the previous measurement:
  // report nothing from it.
  bool have_rate = false;
  if (g_meas.armed) {
    q.elapsed_s = static_cast<float>(static_cast<double>(g_meas.samples) /
                                     static_cast<double>(g_source_hz));
    if (const auto *m = std::get_if<MotorTiming>(&g_pipeline)) {
      have_rate = fill_motor(*m, cal, q.motor);
      q.rate_s_day = q.motor.rate_with_s_day;
    } else if (const auto *l = std::get_if<LcdTiming>(&g_pipeline)) {
      have_rate = fill_lcd(*l, cal, q.lcd);
      q.rate_s_day = q.lcd.rate_s_day;
    } else if (const auto *t = std::get_if<ToneTiming>(&g_pipeline)) {
      have_rate = fill_fork(*t, cal, q.fork);
      q.rate_s_day = q.fork.rate_s_day;
    }
  }
  q.rate_valid = have_rate;
  if (g_meas.done) {
    q.state = DSP_QUARTZ_STATE_DONE;
    q.reason = g_meas.done_reason;
  } else if (have_rate) {
    q.state = DSP_QUARTZ_STATE_MEASURING;
  } else {
    q.state = DSP_QUARTZ_STATE_ACQUIRING;
    q.reason = acquiring_reason(q);
  }
  *out = q;
}

uint32_t copy_pulses(dsp_quartz_pulse_t *out, uint32_t max_n) noexcept {
  const auto *m = std::get_if<MotorTiming>(&g_pipeline);
  if (out == nullptr || m == nullptr || !g_meas.armed) {
    return 0u;
  }
  const auto fs = static_cast<double>(g_source_hz);
  uint32_t n = 0u;
  for (; n < max_n && n < DSP_QUARTZ_PULSE_LOG_CAP; n++) {
    const auto info = m->recent(n);
    if (!info) {
      break;
    }
    dsp_quartz_pulse_t p{};
    p.sequence = info->sequence;
    p.sample_index = info->i_first;
    p.t_s = (info->t_samples - static_cast<double>(g_meas.origin)) / fs;
    p.peak = info->peak;
    p.snr = info->sequence < g_meas.snr_pulses
                ? g_meas.snr[info->sequence % DSP_QUARTZ_PULSE_LOG_CAP]
                : 0.f;
    p.polarity = info->polarity;
    p.extras = info->extras;
    p.clipped = info->clipped;
    p.closed = info->closed;
    p.measurement_id = g_measurement_id;
    out[n] = p;
  }
  return n;
}

const char *mode_name(dsp_quartz_mode_t mode) noexcept {
  const auto i = static_cast<std::size_t>(mode);
  return i < kModeNames.size() ? kModeNames[i] : "unknown";
}

dsp_quartz_mode_t mode_from_name(const char *name) noexcept {
  if (name == nullptr) {
    return DSP_QUARTZ_MODE_COUNT;
  }
  for (std::size_t i = 0; i < kModeNames.size(); i++) {
    if (equals_ignore_case(name, kModeNames[i])) {
      return static_cast<dsp_quartz_mode_t>(i);
    }
  }
  return DSP_QUARTZ_MODE_COUNT;
}

const char *state_name(dsp_quartz_state_t s) noexcept {
  const auto i = static_cast<std::size_t>(s);
  return i < kStateNames.size() ? kStateNames[i] : "unknown";
}

const char *inhibition_name(dsp_quartz_inhibition_t s) noexcept {
  switch (s) {
  case DSP_QUARTZ_INHIB_NONE:
    return "none";
  case DSP_QUARTZ_INHIB_NO_JUMP_LONG_GATE:
    return to_string(InhibitionStatus::kNoJumpLongGate);
  case DSP_QUARTZ_INHIB_MAY_INCLUDE_INHIBITION:
    return to_string(InhibitionStatus::kMayIncludeInhibition);
  case DSP_QUARTZ_INHIB_NEED_ONE_MORE_CYCLE:
    return to_string(InhibitionStatus::kNeedOneMoreCycle);
  case DSP_QUARTZ_INHIB_CORRECTION_PENDING:
    return to_string(InhibitionStatus::kCorrectionPending);
  case DSP_QUARTZ_INHIB_CORRECTED:
    return to_string(InhibitionStatus::kCorrected);
  default:
    return "unknown";
  }
}

const char *ts_method_name(dsp_quartz_ts_method_t m) noexcept {
  const auto i = static_cast<std::size_t>(m);
  return i < kTsNames.size() ? kTsNames[i] : "unknown";
}

dsp_quartz_ts_method_t ts_method_from_name(const char *name) noexcept {
  if (name == nullptr) {
    return DSP_QUARTZ_TS_COUNT;
  }
  for (std::size_t i = 0; i < kTsNames.size(); i++) {
    if (equals_ignore_case(name, kTsNames[i])) {
      return static_cast<dsp_quartz_ts_method_t>(i);
    }
  }
  return DSP_QUARTZ_TS_COUNT;
}

const char *param_result_name(dsp_quartz_param_result_t r) noexcept {
  const auto i = static_cast<std::size_t>(r);
  return i < kParamResultNames.size() ? kParamResultNames[i] : "unknown";
}

const char *lcd_status_name(dsp_quartz_lcd_status_t s) noexcept {
  switch (s) {
  case DSP_QUARTZ_LCD_DETECTING_PERIOD:
    return to_string(LcdStatus::kDetectingPeriod);
  case DSP_QUARTZ_LCD_NO_PATTERN:
    return to_string(LcdStatus::kNoPattern);
  case DSP_QUARTZ_LCD_UNSUPPORTED_RATE:
    return to_string(LcdStatus::kUnsupportedRate);
  case DSP_QUARTZ_LCD_TRACKING:
    return to_string(LcdStatus::kTracking);
  default:
    return "unknown";
  }
}

} // namespace quartz::engine
