/**
 * @file lcd_timing.h
 * @brief LCD-drive timing (RIGHT, capacitive pickup): pattern-rate detection
 *        and the harmonic phase-slope rate, both streaming.
 *
 * Streaming port of {@code lcd_rate()}, {@code lcd_period()} and
 * {@code harmonic_phase_rate()} in {@code docs/handoff/ref/timing_ref.py}
 * (handoff §6). The shared {@link quartz::HarmonicPhaseTracker} is also the
 * engine of {@code tone_rate()} (see tone_timing.h).
 *
 * Key idea, the "fold": every harmonic {@code k·f0} has the carrier
 * {@code exp(-2πi·k·a·n/b)} with {@code f0/fs = a/b} in lowest terms, which
 * repeats every {@code b} samples. A block's DFT at any of these harmonics
 * therefore equals the DFT of the block folded modulo {@code b}:
 *
 *   Σ_n u[n]·e^{-2πi·k·a·n/b} = Σ_{r<b} F[r]·e^{-2πi·k·a·r/b},
 *   F[r] = Σ_{n ≡ r (mod b)} u[n],
 *
 * where {@code u} is the Hann-windowed block and {@code n} the ABSOLUTE
 * sample index. Per sample the tracker does one window multiply and one add;
 * at the end of a block it takes a few b-point DFTs. Folding on the absolute
 * index is also what references every block's phase to absolute sample time
 * exactly, with integer arithmetic only.
 *
 * Pure C++17 plus CMSIS-DSP; no Arduino / Pico SDK.
 *
 * @see lcd_timing.cpp, tone_timing.h, quartz_dsp.h
 */

#pragma once

#include "quartz/quartz_dsp.h"
#include "quartz/quartz_types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace quartz {

/** High-pass corner for the LCD channel (Hz), {@code lcd_rate()}: removes
 * hum and DC; the spikes' energy is at kHz harmonics. */
inline constexpr double kLcdHighpassHz = 400.0;

/**
 * Block-wise harmonic phase tracking and phase-slope rate, the port of
 * {@code harmonic_phase_rate()}.
 *
 * For each block of {@code block_len} samples (block {@code b} spans
 * absolute indices {@code origin + b·block_len ...}):
 *  1. the block is Hann-windowed ({@code numpy.hanning}, symmetric) and
 *     folded modulo the carrier period (see the file comment);
 *  2. on the first block only, if no harmonics were given, every
 *     {@code k·f0 <= f_max} is ranked by Goertzel magnitude and the four
 *     strongest kept (the reference ranks FFT bins of its first block);
 *  3. for each kept {@code k}, {@code Z = single-bin DFT} of the fold, its
 *     angle unwrapped block to block ({@link PhaseUnwrapper}), divided by k
 *     and fitted against the block-centre time ({@link LineFit});
 *  4. {@code eps = Σ k·slope_k / Σ k / (2π·f0)}, rate {@code = 86400·eps}.
 *
 * Harmonic selection is spread out. Ranking takes {@code floor(f_max/f0)}
 * Goertzel passes over the {@code b}-sample fold: 39 x 750 at f0 = 64 Hz,
 * but 156 x 3000 (~470 k multiply-adds) at f0 = 16 Hz, too much for one
 * call. So when the first block ends, its fold is parked in the second
 * half of the buffer, folding continues into the first half, and each
 * {@link process} call ranks whole candidates until it has spent
 * {@link kRankStepsPerCall} Goertzel sample-steps. 0.5 s blocks of 240-sample
 * calls give about 100 calls, which finish the ranking within the next block
 * at every candidate rate (16 Hz needs ~4.7 k steps per call). The parked
 * block's phases are pushed as soon as ranking completes. If a block ends
 * with ranking still pending (only possible with very large calls), the
 * remainder runs at that point, so the fit always sees the blocks in order
 * and the numbers are identical to ranking everything at once.
 *
 * Worst-case work in one 240-sample {@link process} call:
 *  - 240 x (window rotation + multiply + fold add);
 *  - ranking: at most {@code max(kRankStepsPerCall, b)} Goertzel steps
 *    (8192 for every b <= 6000; the loop stops before a candidate would
 *    exceed the budget, unless it would otherwise make no progress);
 *  - at most one block end: four {@code b}-point single-bin DFTs, plus four
 *    more for the parked block on the call that completes ranking
 *    (8 x 3000 at f0 = 16 Hz, 48 kHz).
 *
 * Cost per sample: one window value (float rotation), one multiply
 * ({@code arm_mult_f32}) and one add into the fold. Per block: four
 * {@code b}-point single-bin DFTs ({@code arm_dot_prod_f32}).
 *
 * The fold buffer is borrowed, not owned: the owner passes a buffer that
 * outlives this tracker (LcdTiming lends the delay line it no longer needs
 * once the pattern rate is known). {@code f_max} keeps {@code k·f0} low
 * enough that ±100 ppm moves the phase by much less than half a cycle per
 * block, so the unwrap is unambiguous.
 */
class HarmonicPhaseTracker {
public:
  /** Harmonics tracked at most (the reference keeps the top four). */
  static constexpr std::size_t kMaxHarmonics = 4u;
  /** Goertzel sample-steps of harmonic ranking per {@link process} call:
   * twice the 16 Hz need (~4.7 k), and at most ~25 k float operations,
   * under 0.2 ms on the M33. */
  static constexpr uint32_t kRankStepsPerCall = 8192u;

  /** Tracker parameters. */
  struct Config {
    Rational f0{};          /**< Fundamental (exact, Hz). */
    uint32_t fs_hz = 0;     /**< Sample rate (Hz). */
    uint32_t block_len = 0; /**< Samples per block, {@code int(block_s·fs)}. */
    double f_max_hz = 2500.0; /**< Highest harmonic frequency considered. */
    /** Fixed harmonic numbers; used when {@link harmonic_count} > 0,
     * otherwise the top four are selected on the first block. */
    std::array<uint16_t, kMaxHarmonics> harmonics{};
    uint8_t harmonic_count = 0; /**< Valid entries in {@link harmonics}. */
    uint64_t origin = 0;        /**< Absolute index of block 0's start. */
    uint32_t first_block = 0;   /**< First block index to use. */
  };

  /**
   * Configures and resets.
   *
   * @param fold Fold buffer of at least {@code b} floats ({@code 2b} when
   *        harmonics are auto-selected: the second half parks the first
   *        block while it is ranked), where {@code f0/fs = a/b}. It must
   *        outlive the tracker's use.
   * @param fold_cap Its capacity (floats).
   * @param cfg Parameters.
   * @return {@code false} if the carrier period (twice it when
   *         auto-selecting) does not fit the buffer,
   *         no harmonic lies at or below {@code f_max}, a harmonic would be
   *         at or above fs/2, or the block is shorter than two samples.
   */
  [[nodiscard]] bool configure(float *fold, std::size_t fold_cap,
                               const Config &cfg) noexcept;

  /** Clears the fold, harmonic selection, unwrap state and fits. */
  void reset() noexcept;

  /**
   * Feeds high-passed samples with consecutive absolute indices. Samples
   * before the first tracked block are ignored.
   *
   * @param y Samples.
   * @param n Count.
   * @param first_index Absolute index of {@code y[0]}.
   */
  void process(const float *y, std::size_t n, uint64_t first_index) noexcept;

  /** @return {@code true} once configured. */
  [[nodiscard]] bool configured() const noexcept { return base_ != nullptr; }

  /** @return Completed blocks in the fit. */
  [[nodiscard]] uint32_t blocks() const noexcept { return blocks_; }

  /** @return The rate once at least two blocks are in, else nullopt. */
  [[nodiscard]] std::optional<PhaseRate> result() const noexcept;

  /** @return The carrier period b (samples) of the fold. */
  [[nodiscard]] uint32_t fold_period() const noexcept { return ratio_.b; }

private:
  /** Ends the current block: parks it for ranking (first block, auto
   * selection), or pushes its phases; then clears the active fold. */
  void finish_block() noexcept;

  /**
   * Ranks pending candidates on the parked fold.
   *
   * @param budget Goertzel sample-steps allowed; {@code UINT32_MAX} runs
   *        the ranking to completion.
   */
  void rank_step(uint32_t budget) noexcept;

  /**
   * Adds one block's phases to the unwrappers and fits.
   *
   * @param fold The block's fold ({@code b} floats).
   * @param block_index The block's index (for its centre time).
   */
  void push_block(const float *fold, uint32_t block_index) noexcept;

  /** @param k Harmonic. @return Its carrier ratio {@code (k·a mod b)/b}. */
  [[nodiscard]] FrequencyRatio harmonic_ratio(uint32_t k) const noexcept;

  float *base_ = nullptr;     /**< Borrowed buffer (owner's storage). */
  float *fold_ = nullptr;     /**< Active fold: {@code base_} or
                                   {@code base_ + b}. */
  float *parked_ = nullptr;   /**< The other half: holds the first block
                                   while it is ranked. */
  Config cfg_{};              /**< Parameters. */
  FrequencyRatio ratio_{};    /**< f0 / fs = a / b. */
  double hann_step_ = 0.0;    /**< 2π / (block_len - 1). */
  uint64_t start_ = 0;        /**< Absolute index of the first tracked
                                   block. */
  uint64_t next_index_ = 0;   /**< Expected next absolute index. */
  uint32_t pos_ = 0;          /**< Position inside the current block. */
  uint32_t block_index_ = 0;  /**< Index of the current block. */
  uint32_t fold_pos_ = 0;     /**< (absolute index) mod b. */
  bool selected_ = false;     /**< Harmonics chosen. */
  bool ranking_ = false;      /**< The parked block is being ranked. */
  uint32_t rank_k_ = 0;       /**< Next candidate harmonic to rank. */
  uint32_t rank_kmax_ = 0;    /**< Last candidate, floor(f_max / f0). */
  uint32_t parked_block_ = 0; /**< Index of the parked block. */
  std::array<double, kMaxHarmonics> best_mag_{}; /**< Top-four |Z|. */
  std::array<uint16_t, kMaxHarmonics> ks_{};     /**< Harmonics in use. */
  uint8_t k_count_ = 0; /**< Valid entries in {@link ks_}. */
  std::array<PhaseUnwrapper, kMaxHarmonics> unwrap_{}; /**< Per k. */
  std::array<LineFit, kMaxHarmonics> fits_{};          /**< Per k. */
  uint32_t blocks_ = 0;                                /**< Completed blocks. */
  double tc0_ = 0.0;              /**< Centre time of the first block (s). */
  std::array<float, kChunk> w_{}; /**< Window chunk (scratch). */
  std::array<float, kChunk> s_{}; /**< Sine scratch for the window. */
  std::array<float, kChunk> u_{}; /**< Windowed chunk (scratch). */
};

/**
 * The LCD pipeline: raw RIGHT codes in, pattern rate and phase-slope rate
 * out.
 *
 * Stage 1 ({@code lcd_period()}): over the first 2 s, the high-passed signal
 * goes into a delay line and four lag products are accumulated, one per
 * candidate {@code f in {128, 64, 32, 16} Hz} at lag {@code round(fs/f)}.
 * The reference subtracts the 2 s mean before correlating; with the sums
 * {@code S = Σy}, {@code Q = Σy²}, {@code P_L = Σ y[n]·y[n-L]} and the head
 * and tail sums of the first and last L samples, the mean-removed
 * correlation follows exactly without a second pass:
 *
 *   r0   = Q - N·m²
 *   r(L) = P_L - m·(2S - head_L - tail_L) + (N - L)·m²,   m = S/N.
 *
 * f0 is the highest candidate with {@code r(L)/r0 > 0.8}.
 *
 * Stage 2 ({@code harmonic_phase_rate(y, fs, f0)}): the delay line is
 * re-used as the tracker's fold buffer, 0.5 s blocks, {@code f_max} 2.5 kHz,
 * top four harmonics chosen on the first tracked block. The tracker needs
 * two folds of {@code b = fs/f0} for its spread-out harmonic ranking. That
 * fits every pattern rate at 48 kHz (16 Hz: 2 x 3000 <= 6256). At 96 kHz,
 * which is only the crystal capture mode, a 16 Hz pattern would need
 * 2 x 6000 and is reported as {@code kUnsupportedRate}; 32 Hz and above
 * fit.
 *
 * Streaming difference from the reference (reported per handoff rule 1):
 * the reference selects harmonics on block 0 and fits blocks 0..end, which
 * needs the first 2 s again after f0 is known. Keeping 2 s of samples is
 * not affordable, so tracking starts at the first block boundary after the
 * 2 s period decision (block 4 at 0.5 s blocks). The fit loses those four
 * blocks; on the 20 s vectors the rate moves by under 0.001 s/d.
 */
class LcdTiming {
public:
  /** Candidate pattern rates, highest first (Hz). */
  static constexpr std::array<uint32_t, 4> kCandidatesHz = {128u, 64u, 32u,
                                                            16u};
  /** Autocorrelation record (s). */
  static constexpr double kPeriodSeconds = 2.0;
  /** Normalised autocorrelation a candidate must exceed. */
  static constexpr double kPeriodThreshold = 0.8;
  /** Phase block (s). */
  static constexpr double kBlockSeconds = 0.5;
  /** Highest harmonic frequency (Hz). */
  static constexpr double kFMaxHz = 2500.0;
  /** Longest lag: fs/16 at the highest rate. */
  static constexpr std::size_t kMaxLag = kMaxSampleRateHz / 16u;
  /** Delay line / fold buffer (floats): the longest lag plus one chunk, so
   * a chunk never overwrites the partners of its own samples. */
  static constexpr std::size_t kBufferCap = kMaxLag + kChunk;

  LcdTiming() noexcept = default;
  LcdTiming(const LcdTiming &) = delete;
  LcdTiming &operator=(const LcdTiming &) = delete;
  LcdTiming(LcdTiming &&) = delete;
  LcdTiming &operator=(LcdTiming &&) = delete;
  ~LcdTiming() = default;

  /**
   * @param fs_hz Sample rate (Hz), at most {@link kMaxSampleRateHz}.
   * @return {@code false} if unsupported.
   */
  [[nodiscard]] bool configure(uint32_t fs_hz) noexcept;

  /**
   * Starts a new measurement (period detection first).
   *
   * @param first_index Absolute index of the next sample fed; block 0 and
   *        the phase origin are referenced to absolute indices.
   */
  void reset(uint64_t first_index = 0) noexcept;

  /**
   * Feeds raw RIGHT codes.
   *
   * @param codes Sign-extended 24-bit codes.
   * @param n Count (any length).
   */
  void process(const int32_t *codes, std::size_t n) noexcept;

  /** @return Status, pattern rate, autocorrelations and the rate. */
  [[nodiscard]] LcdResult result() const noexcept;

  /** @return {@code true} if any raw code has hit the rail since reset. */
  [[nodiscard]] bool clipped() const noexcept { return clipped_; }

private:
  /**
   * Adds samples to the period-detection sums (only those inside the 2 s
   * record).
   *
   * @param y High-passed samples.
   * @param n Count.
   * @return Samples consumed (less than n once the record is complete).
   */
  std::size_t accumulate_period(const float *y, std::size_t n) noexcept;

  /** Evaluates the four candidates and starts the tracker. */
  void decide_period() noexcept;

  /**
   * Σ buf[pa+i]·buf[pb+i], i < len, positions modulo the ring, via
   * {@code arm_dot_prod_f32} on contiguous pieces.
   */
  [[nodiscard]] double ring_dot(std::size_t pa, std::size_t pb,
                                std::size_t len) const noexcept;

  uint32_t fs_hz_ = 0;           /**< Sample rate; 0 = unconfigured. */
  Highpass4 hpf_;                /**< 400 Hz high-pass. */
  HarmonicPhaseTracker tracker_; /**< Stage 2. */
  /** Delay line in stage 1, fold buffer in stage 2. */
  std::array<float, kBufferCap> buf_{};
  std::array<float, kChunk> chunk_{}; /**< Filtered chunk. */
  uint64_t origin_ = 0;               /**< Absolute index of sample 0. */
  uint64_t seen_ = 0;                 /**< Samples since reset. */
  uint32_t record_len_ = 0;           /**< int(2·fs). */
  std::array<uint32_t, 4> lag_{};     /**< round(fs / f) per candidate. */
  std::array<double, 4> cross_{};     /**< P_L per candidate. */
  std::array<double, 4> head_{};      /**< Σ of the first L samples. */
  double sum_ = 0.0;                  /**< Σ y over the record. */
  double sumsq_ = 0.0;                /**< Σ y² over the record. */
  LcdStatus status_ = LcdStatus::kDetectingPeriod; /**< Progress. */
  uint32_t pattern_hz_ = 0;                        /**< Chosen f0 (Hz). */
  std::array<float, 4> autocorr_{}; /**< r(L)/r0 per candidate. */
  bool clipped_ = false;            /**< Any railed code since reset. */
};

} // namespace quartz
