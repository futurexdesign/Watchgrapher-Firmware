/**
 * @file motor_timing.h
 * @brief Stepper-motor pulse timing for analog quartz watches (LEFT,
 *        inductive pickup): streaming detection, leading-edge timestamps and
 *        the segmented least-squares rate with inhibition handling.
 *
 * Streaming port of {@code detect_motor_pulses()}, {@code timestamp_pulses()}
 * and {@code motor_rate()} in {@code docs/handoff/ref/timing_ref.py}
 * (handoff §4 and §5). The reference works on a whole record; here samples
 * arrive in blocks (240 frames from the DMA) and nothing is buffered beyond a
 * 128-sample history ring and the per-pulse (timestamp, polarity) store.
 *
 * Pipeline, per sample:
 *   raw code -> float -> 400 Hz 4th-order Butterworth high-pass ->
 *   {@link quartz::PulseDetector} (noise-relative threshold, 50 ms grouping)
 *   -> leading-edge window -> {@link quartz::threshold_timestamp} or
 *   {@link quartz::XcorrTimestamper} -> {@link quartz::MotorRateFit}.
 *
 * Everything is fixed-capacity and allocation-free; the classes hold CMSIS
 * instances with pointers into themselves and are therefore not copyable.
 *
 * Pure C++17 plus CMSIS-DSP; no Arduino / Pico SDK.
 *
 * @see motor_timing.cpp, quartz_types.h, tests/test_quartz_units.cpp,
 *      tests/test_quartz_vectors.cpp
 */

#pragma once

#include "quartz/quartz_dsp.h"
#include "quartz/quartz_types.h"

#include "arm_math.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace quartz {

/** Motor step intervals seen on watches (s), {@code ALLOWED_STEP_S} in the
 * reference and {@code step_interval_s_allowed} in signal_spec.yaml. */
inline constexpr std::array<double, 8> kAllowedStepS = {1.0,  2.0,  2.5,  5.0,
                                                        10.0, 20.0, 30.0, 60.0};

/** High-pass corner for motor pulses (Hz): removes DC and 60/180 Hz hum
 * before thresholding (handoff §4, §8 "Hum"). */
inline constexpr double kMotorHighpassHz = 400.0;

/** Samples of history kept before the first over-threshold sample. Covers
 * the 0.2 ms xcorr pre-window at 96 kHz (19 samples) plus room for the
 * threshold method to walk back down the leading edge. */
inline constexpr uint32_t kPulsePreHistory = 64u;

/** Capacity of a captured leading-edge window (samples); a power of two so
 * the detector's history ring can be indexed with a mask. */
inline constexpr std::size_t kPulseWindowCap = 128u;

/**
 * Sample counts derived from the sample rate, each matching the reference's
 * own integer conversion so window positions line up sample for sample.
 */
struct PulseGeometry {
  uint32_t fs_hz = 0;       /**< Sample rate (Hz). */
  double gap_samples = 0.0; /**< {@code gap_s·fs}: a new group starts when
                                 over-threshold samples are further apart. */
  uint32_t peak_len = 0;    /**< {@code int(0.3e-3·fs) + 2}: samples from
                                 {@code i_first} searched for the peak. */
  uint32_t pre = 0;         /**< {@code round(0.2e-3·fs)}: xcorr window
                                 start before the peak. */
  uint32_t post = 0;        /**< {@code round(0.3e-3·fs)}: xcorr window
                                 length after the peak. */
  uint32_t after = 0;       /**< Samples after {@code i_first} a window must
                                 reach: {@code peak_len - 1 + post}. */
  uint32_t window_len = 0;  /**< {@code kPulsePreHistory + after + 1}. */
  uint32_t settle = 0;      /**< High-pass settling skipped at start. */
  uint32_t noise_block = 0; /**< Samples per noise sub-block (10 ms). */
  uint32_t burst_gap = 0;   /**< Quiet run that separates bursts (2 ms). */
  uint32_t max_drive = 0;   /**< Longest drive pulse (20 ms). */

  /**
   * @param fs_hz Sample rate (Hz), at most {@link kMaxSampleRateHz}.
   * @param gap_s Group gap (s), 0.05 in the reference.
   * @return The geometry, or {@code std::nullopt} if the window would not
   *         fit {@link kPulseWindowCap}.
   */
  [[nodiscard]] static std::optional<PulseGeometry>
  for_rate(uint32_t fs_hz, double gap_s) noexcept;
};

/**
 * High-passed samples around one pulse's leading edge, captured when the
 * last sample the timestamp methods need has arrived (about 0.6 ms after
 * {@code i_first}), long before the 50 ms group gap closes the pulse.
 */
struct PulseWindow {
  std::array<float, kPulseWindowCap> y{}; /**< High-passed samples. */
  uint32_t len = 0;                       /**< Valid samples in {@link y}. */
  uint64_t start = 0;   /**< Absolute index of {@code y[0]}. */
  uint64_t i_first = 0; /**< First sample over threshold (absolute). */
  uint64_t i_peak = 0;  /**< argmax |y| over the first {@code peak_len}
                             samples from {@code i_first} (absolute). */
  int8_t polarity = 0;  /**< sign(y[i_peak]). */
  float peak = 0.f;     /**< |y[i_peak]| (codes). */
};

/**
 * Leading-edge time by the 50 % crossing, the port of
 * {@code timestamp_pulses(method="threshold")}: walk back from the peak
 * while the polarity-corrected sample stays above half the peak, then
 * interpolate linearly between the two samples that straddle the half level.
 *
 * At 48 kHz the edge rises in under one sample, so this is biased by the
 * sub-sample phase of the edge (handoff §3.1); it is still within the
 * acceptance limits and costs a handful of operations.
 *
 * @param w Captured window.
 * @return Absolute sample position of the 50 % crossing (sub-sample).
 */
[[nodiscard]] double threshold_timestamp(const PulseWindow &w) noexcept;

/**
 * Streaming port of {@code detect_motor_pulses()}.
 *
 * The reference thresholds the whole record at {@code k_sigma} times a
 * robust noise sigma ({@code 1.4826·MAD}) and groups over-threshold samples
 * whose spacing exceeds 50 ms. A median is not available sample by sample,
 * so sigma here is {@code sqrt(π/2)·mean|y|} (exact for Gaussian noise; the
 * handoff rounds the factor to 1.25), averaged over 10 ms sub-blocks of
 * quiet samples only: samples over threshold and samples inside an open
 * group never feed it, so pulses do not inflate their own threshold.
 *
 * Start-up: the first 20 ms are skipped while the high-pass settles, then
 * the median of eight 10 ms sub-block means seeds sigma (the median keeps a
 * pulse in that window from biasing it). Detection is armed after these
 * 100 ms; a pulse inside the first 120 ms of a stream is not detected.
 *
 * Grouping is the reference's: a group opens on the first sample with
 * {@code |y| > thr} and absorbs every later over-threshold sample less than
 * or equal to {@code gap_s·fs} samples after the previous one. Each group is
 * one pulse; over-threshold bursts that start more than 20 ms (the longest
 * drive pulse) after the onset and after at least 2 ms of quiet are counted
 * as detection/correction "extras" (handoff §3.1), which the reference does
 * not report.
 */
class PulseDetector {
public:
  /** Detector parameters (reference defaults). */
  struct Config {
    float k_sigma = 8.0f; /**< Threshold in noise sigmas. */
    double gap_s = 0.05;  /**< Group gap (s). */
  };

  /** What {@link process} stopped for. */
  enum class EventKind : uint8_t {
    kNone = 0,    /**< Block consumed, nothing happened. */
    kWindowReady, /**< A pulse's leading-edge window is complete. */
    kGroupClosed, /**< The 50 ms quiet gap after a pulse has passed. */
  };

  /** One detector event. */
  struct Event {
    EventKind kind = EventKind::kNone; /**< Event type. */
    /** For {@code kWindowReady}: the captured window, valid until the next
     * call to {@link process}. */
    const PulseWindow *window = nullptr;
    uint64_t i_last = 0;  /**< For {@code kGroupClosed}: last sample over
                               threshold. */
    uint8_t extras = 0;   /**< For {@code kGroupClosed}: extra bursts. */
    bool clipped = false; /**< For {@code kGroupClosed}: a raw code in
                               [i_first - pre, i_last] hit the rail. */
  };

  /**
   * Sets the rate and parameters and resets.
   *
   * @param fs_hz Sample rate (Hz).
   * @param cfg Parameters.
   * @return {@code false} if the rate is unsupported (state unchanged).
   */
  [[nodiscard]] bool configure(uint32_t fs_hz, const Config &cfg) noexcept;

  /**
   * Restarts detection (settling and noise calibration included).
   *
   * @param first_index Absolute index of the next sample.
   */
  void reset(uint64_t first_index) noexcept;

  /**
   * Consumes samples until the first event or the end of the block.
   *
   * @param y High-passed samples.
   * @param raw The matching raw codes (for the clip flag).
   * @param n Number of samples.
   * @param ev Receives the event; {@code kNone} if the block was consumed.
   * @return Samples consumed; call again with the remainder after an event.
   */
  [[nodiscard]] std::size_t process(const float *y, const int32_t *raw,
                                    std::size_t n, Event &ev) noexcept;

  /** @return Current noise sigma estimate (codes), 0 before calibration. */
  [[nodiscard]] float noise_sigma() const noexcept;

  /** @return Current threshold (codes), 0 before calibration. */
  [[nodiscard]] float threshold() const noexcept { return thr_; }

  /** @return {@code true} once the noise estimate is seeded. */
  [[nodiscard]] bool armed() const noexcept { return phase_ == Phase::kArmed; }

  /** @return The geometry in use. */
  [[nodiscard]] const PulseGeometry &geometry() const noexcept { return geo_; }

private:
  /** Detector start-up phase. */
  enum class Phase : uint8_t { kSettling, kCalibrating, kArmed };

  /** Sub-blocks whose median seeds sigma. */
  static constexpr std::size_t kCalibBlocks = 8u;
  /** Sub-blocks in the exponential average of mean|y| (time constant
   * 0.32 s at 10 ms sub-blocks). */
  static constexpr float kNoiseAvgBlocks = 32.f;
  /** History ring length; a power of two covering one window. */
  static constexpr std::size_t kRing = kPulseWindowCap;

  /**
   * Armed-phase work for one sample.
   *
   * @param idx Absolute index.
   * @param a |y| of the high-passed sample.
   * @param ev Receives an event.
   * @return {@code true} if an event was produced.
   */
  bool step_armed(uint64_t idx, float a, Event &ev) noexcept;

  /** Copies the window around the open group's onset into {@link window_}.
   */
  void capture_window() noexcept;

  /** Recomputes {@link thr_} from {@link mean_abs_}. */
  void update_threshold() noexcept;

  PulseGeometry geo_{};            /**< Rate-derived sample counts. */
  Config cfg_{};                   /**< Parameters. */
  Phase phase_ = Phase::kSettling; /**< Start-up phase. */
  uint64_t next_index_ = 0;        /**< Absolute index of the next sample. */
  uint32_t settle_left_ = 0;       /**< Settling samples still to skip. */

  std::array<float, kRing> ring_{}; /**< Last {@link kRing} y samples. */
  bool have_clip_ = false;          /**< {@link last_clip_} is valid. */
  uint64_t last_clip_ = 0;          /**< Index of the latest railed raw code. */

  std::array<float, kCalibBlocks> calib_{}; /**< Seed sub-block means. */
  std::size_t calib_count_ = 0;             /**< Seed sub-blocks collected. */
  float blk_sum_ = 0.f;    /**< Σ|y| of quiet samples, sub-block. */
  uint32_t blk_quiet_ = 0; /**< Quiet samples in the sub-block. */
  uint32_t blk_n_ = 0;     /**< Samples in the sub-block. */
  float mean_abs_ = 0.f;   /**< Averaged mean|y| of quiet samples. */
  float thr_ = 0.f;        /**< Detection threshold (codes). */

  bool group_open_ = false;    /**< A pulse group is open. */
  bool captured_ = false;      /**< Its window has been captured. */
  bool group_clipped_ = false; /**< A raw code railed in the group. */
  uint8_t extras_ = 0;         /**< Extra bursts in the group. */
  uint64_t i_first_ = 0;       /**< Group onset (absolute). */
  uint64_t last_over_ = 0;     /**< Latest over-threshold sample. */
  PulseWindow window_{};       /**< Last captured window. */
};

/**
 * Cross-correlation timestamps, the port of
 * {@code timestamp_pulses(method="xcorr")} and {@code _frac_xcorr()}.
 *
 * Each pulse contributes a segment {@code pol·y[i_peak-pre, i_peak+post)}
 * (24 samples at 48 kHz, inside the first chopping on-time even at 37.5 %
 * duty). The reference averages ALL pulses into the template (two re-align
 * passes); a stream cannot, so the template is built the same way from the
 * first {@link kTemplatePulses} pulses and then fixed (handoff §4 firmware
 * notes). Until then every new pulse rebuilds the template and re-times the
 * pending pulses, so a record with fewer than four pulses is timed exactly as
 * the reference times it.
 *
 * Fractional lag, as {@code _frac_xcorr(up=64)}: the cross-spectrum
 * {@code R = S·conj(T)} of both segments zero-padded to
 * {@code m = 2^ceil(log2(2n))} points is inverse-transformed at 64x
 * oversampling (band-limited interpolation of the correlation), the maximum
 * is located and refined with a parabola. Implementation: the forward
 * transforms are CMSIS {@code arm_rfft_fast_f32} at 512 points (every
 * {@code 512/m}-th bin of the zero-padded 512-point transform is the
 * {@code m}-point bin); a CMSIS inverse 512-point transform of R gives the
 * correlation on a 1/8-sample grid, whose maximum is then refined on the
 * exact 1/64-sample grid by evaluating the same trigonometric polynomial
 * directly at the few points around it. The result equals the reference's
 * 4096-point inverse transform at those points, without a 4096-point table.
 */
class XcorrTimestamper {
public:
  /** Pulses averaged before the template is fixed. */
  static constexpr std::size_t kTemplatePulses = 4u;
  /** Longest segment: 0.2 + 0.3 ms at 96 kHz is 48 samples. */
  static constexpr std::size_t kSegCap = 64u;
  /** CMSIS real FFT length (the vendored tables are for 512). */
  static constexpr std::size_t kFftLen = 512u;
  /** Oversampling of the correlation, {@code up} in the reference. */
  static constexpr uint32_t kUpsample = 64u;
  /** Most cross-spectrum bins kept: {@code m/2 + 1} for m = 128. */
  static constexpr std::size_t kMaxBins = 65u;

  /** Initialises the CMSIS FFT instance. */
  XcorrTimestamper() noexcept;

  /**
   * Sets the segment length and resets.
   *
   * @param seg_len {@code pre + post} samples.
   * @return {@code false} if the padded length would exceed 128 or the
   *         segment is shorter than 4 samples.
   */
  [[nodiscard]] bool configure(std::size_t seg_len) noexcept;

  /** Drops the template and any pending pulses. */
  void reset() noexcept;

  /** @return {@code true} once the template is fixed. */
  [[nodiscard]] bool fixed() const noexcept { return fixed_; }

  /**
   * Adds one of the first pulses: rebuilds the template from all pending
   * segments and re-times them. Fixes the template at
   * {@link kTemplatePulses}. Must not be called once fixed.
   *
   * @param seg {@code pol·y} segment of {@code seg_len} samples.
   * @param seg_start Absolute index of {@code seg[0]}.
   * @return Pending pulses (1..kTemplatePulses); their timestamps are
   *         {@link pending_timestamp}.
   */
  std::size_t add_template_pulse(const float *seg, double seg_start) noexcept;

  /**
   * @param i Pending pulse, {@code < kTemplatePulses}.
   * @return Its timestamp against the current template (absolute samples).
   */
  [[nodiscard]] double pending_timestamp(std::size_t i) const noexcept {
    return pending_start_[i] + pending_lag_[i];
  }

  /**
   * Times a pulse against the current template.
   *
   * @param seg {@code pol·y} segment.
   * @param seg_start Absolute index of {@code seg[0]}.
   * @return {@code seg_start + lag}, absolute samples. Like the reference,
   *         this is the segment start plus the lag, not the edge itself: the
   *         template's own edge position is a constant that cancels in rates.
   */
  [[nodiscard]] double timestamp(const float *seg, double seg_start) noexcept {
    return seg_start + lag(seg);
  }

  /**
   * Fractional delay of a segment relative to the template,
   * {@code _frac_xcorr(seg, tmpl)}.
   *
   * @param seg Segment of {@code seg_len} samples.
   * @return Lag in samples, in {@code (-m/2, m/2]}.
   */
  [[nodiscard]] double lag(const float *seg) noexcept;

  /**
   * Shifts a segment earlier by {@code lag} samples with an n-point
   * band-limited (DFT phase-ramp) shift, as the reference's re-alignment:
   * {@code irfft(rfft(s)·exp(2πi·f·lag), n)}.
   *
   * @param seg Input segment.
   * @param lag Shift (samples).
   * @param out Output segment (must not alias {@code seg}).
   */
  void fractional_shift(const float *seg, double lag,
                        float *out) const noexcept;

private:
  /**
   * A complex value as two floats. Plain structs with hand-written products
   * keep std::complex, and with it libgcc's __mulsc3/__muldc3/__divdc3
   * helpers (~3.8 KB of RAM-resident code on the target), out of the
   * library.
   */
  struct BinF {
    float re; /**< Real part. */
    float im; /**< Imaginary part. */
  };
  /** A complex value as two doubles (cross-spectrum, twiddles). */
  struct BinD {
    double re; /**< Real part. */
    double im; /**< Imaginary part. */
  };

  /** Rebuilds the template from the pending segments (mean, then two
   * re-align passes) and re-times them. */
  void rebuild() noexcept;

  /** Installs a template and caches its spectrum. */
  void set_template(const float *tmpl) noexcept;

  /**
   * m-point spectrum (bins 0..m/2) of a zero-padded segment via the
   * 512-point CMSIS real FFT.
   *
   * @param seg Segment.
   * @param out Receives {@code m/2 + 1} bins.
   */
  void spectrum(const float *seg, std::array<BinF, kMaxBins> &out) noexcept;

  /**
   * Correlation at fine-grid point {@code q} (of {@code m·kUpsample}),
   * up to the constant 1/N: {@code Re R0 + 2 Σ Re(R_j e^{2πi j q / N})}.
   *
   * @param r Cross-spectrum bins.
   * @param q Fine-grid index (taken modulo N).
   * @return Unscaled correlation value.
   */
  [[nodiscard]] double fine_correlation(const std::array<BinD, kMaxBins> &r,
                                        int64_t q) const noexcept;

  arm_rfft_fast_instance_f32 rfft_{};    /**< CMSIS 512-point real FFT. */
  std::array<float, kFftLen> fft_in_{};  /**< FFT input (scratch). */
  std::array<float, kFftLen> fft_out_{}; /**< FFT output (scratch). */
  /** cos/sin(2π·t/n), t < n, for the n-point shift DFT. */
  std::array<BinD, kSegCap> twiddle_{};
  std::size_t n_ = 0;      /**< Segment length. */
  std::size_t m_ = 0;      /**< Zero-padded correlation length. */
  std::size_t stride_ = 0; /**< 512 / m. */

  /** Segments of the pulses averaged into the template so far. */
  std::array<std::array<float, kSegCap>, kTemplatePulses> pending_{};
  std::array<double, kTemplatePulses> pending_start_{}; /**< Seg starts. */
  std::array<double, kTemplatePulses> pending_lag_{};   /**< Their lags. */
  std::size_t pending_count_ = 0;                       /**< Pending pulses. */
  bool fixed_ = false;                                  /**< Template fixed. */

  std::array<float, kSegCap> tmpl_{}; /**< Current template. */
  /** Template spectrum, bins 0..m/2. */
  std::array<BinF, kMaxBins> tmpl_spec_{};
};

/**
 * Pulse store and segmented least-squares rate, the port of
 * {@code motor_rate()} (handoff §4, §5).
 *
 * Steps, each as the reference does them:
 *  1. step = the allowed interval nearest {@code median(diff(t))};
 *  2. step numbers {@code n_k = round((t_k - t_0)/step)}, so a missed pulse
 *     leaves a gap instead of doubling an interval;
 *  3. {@code dev_k = diff(t)_k - diff(n)_k·median(diff(t)/max(diff(n),1))};
 *     an inhibition event sits between k and k+1 when {@code dev_k} exceeds
 *     20 µs (the smallest real jump is 61 µs; timing noise is ~1 µs);
 *  4. least squares {@code t_k = a[s_k] + b·n_k + c·pol_k} with one intercept
 *     per segment between events, a common period {@code b} and a polarity
 *     offset {@code c}. Intercepts are eliminated by centring t, n and pol
 *     inside each segment; {@code b} and {@code c} then follow from a 2x2
 *     normal-equation solve. O(N), no matrix library;
 *  5. with two or more events, the rate with inhibition is the mean period
 *     from the first pulse after the first event to the first pulse of the
 *     same polarity after the last event, and the inhibition period is the
 *     median event spacing.
 *
 * Timestamps are held as absolute sample positions in double and rebased on
 * the oldest pulse (in seconds) for the solve, so a run of hours keeps
 * sub-nanosecond resolution. Intervals feeding the medians are stored as
 * float32 offsets from the step, which keeps them exact to ~1e-12 s.
 *
 * Memory: {@link kCapacity} pulses (34 min at one step per second; 960 s,
 * the longest inhibition period, plus margin). When full, the oldest pulse
 * is dropped, so a long run becomes a sliding window.
 *
 * <b>Two solvers.</b> {@link solve} is the batch reference above. It is
 * O(N) in soft-float double plus two nth_element medians over up to 2047
 * intervals, which takes milliseconds on the M33 at N = 2048. It stays as
 * the reference and for host analysis. {@link solve_incremental} gives the
 * same answer for a cost that does not grow with N, for use inside the
 * capture path:
 *  - <b>step</b>: snapped exactly as the batch does it while the store
 *    holds at most {@link kEventWindow}+1 pulses. After that, the snap of
 *    the median of the last {@link kEventWindow} raw intervals is checked
 *    on every pulse; a disagreement schedules one rebuild with the new
 *    step;
 *  - <b>events</b>: interval k is tested against the median per-step
 *    interval of the first {@link kEventWindow} intervals (k below that),
 *    or of the last {@link kEventWindow} intervals including k. The batch
 *    instead uses one median over the whole record. This is a documented
 *    deviation. Event decisions are identical unless a jump lies within
 *    the difference of the two medians (well under 1 µs) of the 20 µs
 *    threshold. Jump sizes differ by that much too;
 *  - <b>fit</b>: per-segment raw moments in a segment-local frame:
 *    n - n0, and u = t - t0 - P·(n - n0), where P is the median per-step
 *    interval of the first window. P is within ~1 µs of the true period, so
 *    u stays at ms level over any run, even 60 s steps at ±100 ppm. When a
 *    segment closes at an event, its centred moments are folded into a
 *    global sum, so any number of segments costs O(1) memory, and the solve
 *    is the same intercept-eliminated 2x2 system. The residual is
 *    maintained algebraically: Σr² = S_uu - b'·S_nu - c·S_pu;
 *  - <b>rate with inhibition, period, status</b>: the first pulse after
 *    the first event and, in the open segment, the first pulse of each
 *    polarity are kept, which gives j in O(1). Event start positions and
 *    jump sizes go into a ring of {@link kMaxEvents}; the period median is
 *    recomputed only when an event arrives.
 * The incremental state covers every pulse ingested since the last
 * (re)build, so after more than {@link kCapacity} pulses it is cumulative,
 * while the batch solve slides. It never reads an evicted slot. The anchors
 * it needs later are held as scalars: the time origin, the first pulse after
 * the first event, the open segment's first pulse of each polarity, and the
 * event step numbers. The replay cursor also moves down when a slot is
 * evicted.
 *
 * Overflow policy: once an ingested pulse has been evicted, the state can no
 * longer be rebuilt from the store, because a replay would silently shrink
 * the coverage to the last 2048 pulses. Rebuilds are therefore suppressed
 * from then on. The only rebuild triggers are a step re-snap and a re-timed
 * pulse:
 *  - after 2048 consistent pulses the step cannot legitimately change, so a
 *    later window that snaps differently (e.g. a run of missed pulses) is
 *    ignored;
 *  - set_time() only re-times the first xcorr pulses, which are long gone.
 * A real change of watch or setup needs a reset (a new gate).
 *
 * Window size {@link kEventWindow} = 31:
 *  - the median survives 15 outliers, while events come at most once per
 *    10 steps (≤ 4 in a window);
 *  - its noise, ≈1.25·√2·σ/√31 ≈ 0.8 µs at the weakest pulses
 *    (σ = 2.6 µs), is small against the 20 µs threshold;
 *  - 31 floats sort in a few hundred operations;
 *  - an odd count gives a unique median.
 */
class MotorRateFit {
public:
  /** Pulses held. */
  static constexpr std::size_t kCapacity = 2048u;
  /** Inhibition events whose jump size is kept for reporting. */
  static constexpr std::size_t kMaxEvents = 256u;
  /** Recent per-step intervals in the incremental event median (odd). */
  static constexpr std::size_t kEventWindow = 31u;
  /**
   * Pulses the incremental path ingests per {@link solve_incremental} call
   * while it catches up after a rebuild. Bounds the call to about 16 x 1.5 k
   * soft-double operations (~2 ms on the M33 in the worst case, typically
   * well under 1 ms).
   */
  static constexpr std::size_t kReplayPulsesPerCall = 16u;

  /** Fit parameters (reference defaults). */
  struct Config {
    /** Force the step interval (s) instead of snapping the median. */
    std::optional<double> step_s;
    /** Interval excess (µs) that marks an inhibition event. */
    double jump_threshold_us = 20.0;
    /** Longest inhibition period the user expects (s). With no jump,
     * {@code kNoJumpLongGate} needs the pulses to cover at least twice this:
     * (n_last - n_first + 1)·step >= 2·P_max (see MotorRateFit::solve). Typical
     * watches use 10 or 60 s; the specification maximum is 960 s. */
    double longest_inhibition_period_s = 60.0;
  };

  /**
   * Sets the rate and parameters and clears the store.
   *
   * @param fs_hz Sample rate (Hz), converts sample positions to seconds.
   * @param cfg Parameters.
   */
  void configure(uint32_t fs_hz, const Config &cfg) noexcept;

  /** Clears the store. */
  void reset() noexcept;

  /**
   * Appends a pulse; drops the oldest when full.
   *
   * @param t_samples Leading-edge time (absolute samples, sub-sample).
   * @param polarity +1 / -1.
   */
  void push(double t_samples, int8_t polarity) noexcept;

  /**
   * Replaces the timestamp of a stored pulse (used while the xcorr template
   * is still being built).
   *
   * @param i Pulse index, 0 = oldest held.
   * @param t_samples New timestamp (absolute samples).
   */
  void set_time(std::size_t i, double t_samples) noexcept;

  /** @return Pulses held. */
  [[nodiscard]] std::size_t size() const noexcept { return count_; }

  /** @param i Pulse index, 0 = oldest. @return Its timestamp (samples). */
  [[nodiscard]] double time_samples(std::size_t i) const noexcept {
    return t_[slot(i)];
  }

  /** @param i Pulse index, 0 = oldest. @return Its polarity. */
  [[nodiscard]] int8_t polarity(std::size_t i) const noexcept {
    return pol_[slot(i)];
  }

  /**
   * Solves the fit over the held pulses. Not {@code const}: it uses the
   * member scratch buffer and refreshes the jump list.
   *
   * @return The rate, or {@code std::nullopt} with fewer than three pulses
   *         or a degenerate geometry (every segment a single pulse).
   */
  [[nodiscard]] std::optional<MotorRate> solve() noexcept;

  /**
   * Incremental solve: ingests any pulses pushed since the last call (at
   * most {@link kReplayPulsesPerCall}) and returns the current rate.
   *
   * Cost per call in steady state (one new pulse, measured and estimated):
   *  - ingest: two nth_element calls on 31 floats (event median and step
   *    check, ~2 x 150 compare/swaps on the FPU), about 45 double
   *    operations and one llround;
   *  - solve: about 40 double operations, including 3 divisions and one
   *    sqrt, independent of the pulse count;
   *  - when an event arrives: one more nth_element over at most
   *    {@link kMaxEvents} event gaps (the period median).
   * In soft-float that is about 100 double operations, roughly 10 k cycles
   * (~70 µs at 150 MHz), per pulse. While the store holds at most
   * {@link kEventWindow}+1 pulses, every new pulse re-runs the (≤32-pulse)
   * warm-up synchronously, because the batch rule's median changes with
   * each interval. That costs at most 32 ingests without the sliding
   * median. A rebuild (step change, or {@link set_time} on an
   * already-ingested pulse) replays at most {@link kReplayPulsesPerCall}
   * pulses per call; meanwhile the last result is returned unchanged and
   * {@link rebuilding} is true.
   *
   * Equal to {@link solve} (rates within 1e-6 s/d, identical events,
   * period, step and status) whenever the windowed and global medians
   * agree on every event decision, and the store has never overflowed.
   *
   * @return The rate, or {@code std::nullopt} with fewer than three pulses
   *         or a degenerate geometry.
   */
  [[nodiscard]] std::optional<MotorRate> solve_incremental() noexcept;

  /** @return {@code true} while a rebuild is still replaying the store
   *          ({@link solve_incremental} returns the last good result). */
  [[nodiscard]] bool rebuilding() const noexcept {
    return count_ >= 3u && (inc_.dirty || inc_.next < count_);
  }

  /** @return Jumps kept by the incremental path (the last
   *          {@link kMaxEvents}). */
  [[nodiscard]] std::size_t stream_jump_count() const noexcept {
    return inc_.ev_stored;
  }

  /**
   * @param i Jump index, 0 = oldest kept, {@code < stream_jump_count()}.
   * @return Its size (µs) as measured when it was detected.
   */
  [[nodiscard]] float stream_jump_us(std::size_t i) const noexcept {
    return inc_
        .ev_jump[(inc_.ev_head + kMaxEvents - inc_.ev_stored + i) % kMaxEvents];
  }

  /** @return Jumps recorded by the last {@link solve} (at most
   *          {@link kMaxEvents}). */
  [[nodiscard]] std::size_t jump_count() const noexcept { return jump_count_; }

  /** @param i Jump index. @return Its size (µs) from the last solve. */
  [[nodiscard]] float jump_us(std::size_t i) const noexcept {
    return jump_us_[i];
  }

private:
  /** Raw moments of one segment in its local frame (dn, pol, u). */
  struct Moments {
    double m = 0.0;   /**< Pulses. */
    double sn = 0.0;  /**< Σ dn. */
    double sp = 0.0;  /**< Σ pol. */
    double su = 0.0;  /**< Σ u (s). */
    double snn = 0.0; /**< Σ dn². */
    double snp = 0.0; /**< Σ dn·pol. */
    double spp = 0.0; /**< Σ pol². */
    double snu = 0.0; /**< Σ dn·u. */
    double spu = 0.0; /**< Σ pol·u. */
    double suu = 0.0; /**< Σ u². */
  };

  /** Within-segment centred sums (intercepts eliminated), additive across
   * segments. */
  struct Centred {
    double nn = 0.0; /**< Σ n'n'. */
    double np = 0.0; /**< Σ n'p'. */
    double pp = 0.0; /**< Σ p'p'. */
    double nu = 0.0; /**< Σ n'u'. */
    double pu = 0.0; /**< Σ p'u'. */
    double uu = 0.0; /**< Σ u'u'. */
  };

  /** A pulse remembered for the rate with inhibition. */
  struct Mark {
    bool valid = false; /**< Set. */
    double t = 0.0;     /**< Time (s since the incremental origin). */
    double n = 0.0;     /**< Step number. */
    int8_t pol = 0;     /**< Polarity. */
  };

  /** State of the incremental path. */
  struct Incremental {
    bool dirty = true; /**< Restart before the next ingest. */
    /** The state includes pulses already evicted from the store, so a
     * replay could not reproduce it; rebuilds are suppressed from then on. */
    bool overflowed = false;
    double step = 0.0; /**< Step in use (s); 0 = not yet known. */
    /** Frame period (s): u = t - t0 - frame·dn. The frozen median per-step
     * interval, within ~1 µs of the true period, so u stays at the ms level
     * and Σu² cannot swamp Σr². */
    double frame = 0.0;
    double new_step = 0.0;   /**< Step for a pending rebuild; 0 = keep. */
    std::size_t next = 0;    /**< Store index of the next pulse to ingest. */
    double t_ref = 0.0;      /**< Absolute samples of time zero. */
    bool have_prev = false;  /**< A previous pulse exists. */
    double prev_t = 0.0;     /**< Previous pulse time (s). */
    double prev_n = 0.0;     /**< Previous pulse step number. */
    uint32_t intervals = 0;  /**< Intervals ingested since restart. */
    double frozen_med = 0.0; /**< Median per-step interval of the first
                                  kEventWindow intervals (s). */
    std::size_t frozen_count = 0; /**< Intervals behind frozen_med. */
    std::array<float, kEventWindow> win_ratio{}; /**< Per-step interval -
                                                      step (s). */
    std::array<float, kEventWindow> win_iv{};    /**< Raw intervals (s). */
    std::size_t win_head = 0;                    /**< Next ring slot. */
    std::size_t win_n = 0;                       /**< Valid ring entries. */
    Moments open{};                   /**< Open segment's raw moments. */
    double open_t0 = 0.0;             /**< Open segment's first time (s). */
    double open_n0 = 0.0;             /**< Open segment's first step number. */
    std::array<Mark, 2> open_first{}; /**< First -1 / +1 pulse in it. */
    Centred closed{};                 /**< Folded sums of closed segments. */
    uint32_t pulses = 0;              /**< Pulses ingested. */
    double t_last = 0.0;              /**< Last pulse time (s). */
    uint32_t events = 0;              /**< Events detected. */
    double jump_sum = 0.0;            /**< Σ jump (µs). */
    double jump_min = 0.0;            /**< Smallest jump (µs). */
    double jump_max = 0.0;            /**< Largest jump (µs). */
    std::array<int32_t, kMaxEvents> ev_n{};  /**< Step number after each
                                                  kept event. */
    std::array<float, kMaxEvents> ev_jump{}; /**< Kept jump sizes (µs). */
    std::size_t ev_head = 0;                 /**< Next event slot. */
    std::size_t ev_stored = 0;               /**< Kept events. */
    Mark first_after{};              /**< First pulse after the first event. */
    std::optional<double> period_s;  /**< Cached inhibition period. */
    std::optional<MotorRate> cached; /**< Last complete result. */
  };

  /** @return Within-segment centred sums from raw moments. */
  [[nodiscard]] static Centred centre(const Moments &mo) noexcept;

  /** Closes the open segment at an event and opens one at this pulse. */
  void inc_event(double t, double n, int8_t pol, double jump_s) noexcept;

  /** Restarts the incremental state on the store with a step (s). */
  void inc_restart(double step) noexcept;

  /** Ingests store pulse {@code k} into the incremental state. */
  void inc_ingest(std::size_t k) noexcept;

  /** @return The incremental result from the current sums. */
  [[nodiscard]] std::optional<MotorRate> inc_result() const noexcept;

  /** @return The batch step rule over the first {@code count} intervals of
   *          the store (or the configured step). */
  [[nodiscard]] double snap_store_step(std::size_t count) noexcept;

  /** @param i Logical index. @return Ring slot. */
  [[nodiscard]] std::size_t slot(std::size_t i) const noexcept {
    return (head_ + i) & (kCapacity - 1u);
  }

  /** @param k Pulse. @return Seconds since the oldest held pulse. */
  [[nodiscard]] double rel_s(std::size_t k) const noexcept;

  /**
   * Median of {@code scratch_[0..n)} with numpy semantics (mean of the two
   * middle values for even n). Reorders the scratch buffer.
   *
   * @param n Count, > 0.
   * @return The median.
   */
  [[nodiscard]] double scratch_median(std::size_t n) noexcept;

  uint32_t fs_hz_ = 48000u;             /**< Sample rate (Hz). */
  Config cfg_{};                        /**< Parameters. */
  std::array<double, kCapacity> t_{};   /**< Timestamps (absolute samples). */
  std::array<int8_t, kCapacity> pol_{}; /**< Polarities. */
  std::size_t head_ = 0;                /**< Slot of the oldest pulse. */
  std::size_t count_ = 0;               /**< Pulses held. */
  std::array<float, kCapacity> scratch_{};     /**< Median work space. */
  std::array<uint16_t, kMaxEvents> event_k_{}; /**< Event positions. */
  std::array<float, kMaxEvents> jump_us_{};    /**< Jump sizes (µs). */
  std::size_t jump_count_ = 0;                 /**< Valid jumps. */
  Incremental inc_{};                          /**< Incremental solver state. */
};

/**
 * The motor-timing pipeline: raw LEFT codes in, rate out.
 *
 * Owns the high-pass, the detector, the xcorr timestamper and the fit, plus
 * a 16-entry history of recent pulses for the CLI. Feed it blocks with
 * {@link process}; read the rate with {@link solve} whenever needed (a
 * provisional rate exists from the third pulse). Calibration is not applied:
 * pass the result through {@link apply_timebase_ppm}.
 */
class MotorTiming {
public:
  /** Pipeline parameters. */
  struct Config {
    TimestampMethod method = TimestampMethod::kThreshold; /**< Edge method. */
    PulseDetector::Config detector{}; /**< Detection parameters. */
    MotorRateFit::Config fit{};       /**< Fit parameters. */
  };

  /** Recent pulses kept for display. */
  static constexpr std::size_t kRecentPulses = 16u;

  MotorTiming() noexcept = default;
  MotorTiming(const MotorTiming &) = delete;
  MotorTiming &operator=(const MotorTiming &) = delete;
  MotorTiming(MotorTiming &&) = delete;
  MotorTiming &operator=(MotorTiming &&) = delete;
  ~MotorTiming() = default;

  /**
   * Configures for a sample rate and resets.
   *
   * @param fs_hz Sample rate (Hz), at most {@link kMaxSampleRateHz}.
   * @param cfg Parameters.
   * @return {@code false} if the rate is unsupported.
   */
  [[nodiscard]] bool configure(uint32_t fs_hz, const Config &cfg) noexcept;

  /**
   * Starts a new measurement.
   *
   * @param first_index Absolute index of the next sample fed.
   */
  void reset(uint64_t first_index = 0) noexcept;

  /**
   * Feeds raw LEFT codes.
   *
   * @param codes Sign-extended 24-bit codes.
   * @param n Count (any length).
   */
  void process(const int32_t *codes, std::size_t n) noexcept;

  /** @return The current rate (see {@link MotorRateFit::solve}) with the
   *          detector's extras and clip counts filled in. */
  [[nodiscard]] std::optional<MotorRate> solve() noexcept;

  /**
   * The capture-path solve: {@link MotorRateFit::solve_incremental} with
   * the detector's extras and clip counts filled in. Bounded cost per call
   * (see there); call it after every new pulse and periodically.
   *
   * @return The current rate.
   */
  [[nodiscard]] std::optional<MotorRate> solve_incremental() noexcept;

  /** @return Pulses detected since reset. */
  [[nodiscard]] uint32_t pulse_count() const noexcept { return pulses_; }

  /**
   * @param i 0 = most recent.
   * @return A recent pulse, or {@code std::nullopt} past the history.
   */
  [[nodiscard]] std::optional<PulseInfo> recent(std::size_t i) const noexcept;

  /** @return The fit, for per-pulse timestamps and jump sizes. */
  [[nodiscard]] const MotorRateFit &fit() const noexcept { return fit_; }

  /** @return The detector (noise sigma, threshold, geometry). */
  [[nodiscard]] const PulseDetector &detector() const noexcept { return det_; }

  /** @return {@code true} if any raw code has hit the rail since reset. */
  [[nodiscard]] bool clipped() const noexcept { return clipped_; }

private:
  /** Handles a captured leading-edge window. */
  void on_window(const PulseWindow &w) noexcept;

  /** Handles a group close. */
  void on_group_closed(const PulseDetector::Event &ev) noexcept;

  /** @param seq Pulse sequence number. @return Its history entry or null. */
  PulseInfo *find_recent(uint32_t seq) noexcept;

  Config cfg_{};                    /**< Parameters. */
  uint32_t fs_hz_ = 0;              /**< Sample rate; 0 = unconfigured. */
  Highpass4 hpf_;                   /**< 400 Hz high-pass. */
  PulseDetector det_;               /**< Streaming detector. */
  XcorrTimestamper xcorr_;          /**< Template correlator. */
  MotorRateFit fit_;                /**< Pulse store and solver. */
  std::array<float, kChunk> buf_{}; /**< Filtered chunk. */
  std::array<PulseInfo, kRecentPulses> recent_{}; /**< History ring. */
  uint32_t pulses_ = 0;                           /**< Pulses since reset. */
  uint32_t extras_ = 0;         /**< Extra bursts since reset. */
  uint32_t clipped_pulses_ = 0; /**< Clipped pulses since reset. */
  bool clipped_ = false;        /**< Any railed code since reset. */
};

} // namespace quartz
