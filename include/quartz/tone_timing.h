/**
 * @file tone_timing.h
 * @brief Tone timing for tuning-fork watches (LEFT, 300–720 Hz) and the
 *        32 768 Hz crystal (RIGHT, fs = 96 kHz): complex demodulation at the
 *        nominal frequency and a phase-slope rate.
 *
 * Streaming port of {@code tone_rate()} in
 * {@code docs/handoff/ref/timing_ref.py} (handoff §7): the same maths as the
 * LCD harmonic phase method with the single harmonic k = 1, so it runs on
 * {@link quartz::HarmonicPhaseTracker}. Unlike the LCD path nothing has to
 * be learned first, so tracking starts at block 0 and the blocks are exactly
 * the reference's.
 *
 * The nominal frequency is an exact fraction ({@link quartz::Rational}):
 * the 341.33 Hz fork is 1024/3 Hz, and its carrier period at 48 kHz is the
 * exact 1125 samples that contain 8 cycles.
 *
 * Pure C++17 plus CMSIS-DSP; no Arduino / Pico SDK.
 *
 * @see tone_timing.cpp, lcd_timing.h
 */

#pragma once

#include "quartz/lcd_timing.h"
#include "quartz/quartz_dsp.h"
#include "quartz/quartz_types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace quartz {

/**
 * The tone pipeline: raw codes in, phase-slope rate out.
 *
 * High-pass at {@code min(200 Hz, f_nom/2)} (as {@code tone_rate(hp=True)}:
 * removes hum below a fork tone and DC below the crystal), Hann-windowed
 * blocks folded modulo the carrier period, one exact single-bin DFT per
 * block, phase unwrap and an online straight-line fit.
 *
 * Block length trades noise against unwrap safety: at {@code block_s} the
 * phase advances {@code 2π·f·ε·block_s} per block, which must stay well
 * under π for any plausible error ε. 0.25 s at 720 Hz allows ±2800 ppm;
 * 0.05 s at 32 768 Hz allows ±300 ppm.
 */
class ToneTiming {
public:
  /** Block length for tuning forks (s), handoff §7. */
  static constexpr double kForkBlockSeconds = 0.25;
  /** Block length for the 32 768 Hz crystal (s), handoff §7. */
  static constexpr double kCrystalBlockSeconds = 0.05;
  /** Highest high-pass corner (Hz). */
  static constexpr double kMaxHighpassHz = 200.0;
  /**
   * Fold buffer (floats): the carrier period {@code b} of {@code f/fs} in
   * lowest terms must fit. The largest among the specified tones is
   * b = 1125 (1024/3 Hz at 48 or 96 kHz: 8 or 4 cycles per 1125 samples);
   * the crystal at 96 kHz is 128/375 and 300/360/480/720 Hz need at most
   * 800. An arbitrary integer frequency may need up to fs samples and is
   * refused by {@link configure} rather than silently mis-measured.
   */
  static constexpr std::size_t kFoldCap = 2048u;

  /** Tone parameters. */
  struct Config {
    Rational f_nominal{};               /**< Nominal tone (exact, Hz). */
    double block_s = kForkBlockSeconds; /**< Phase block (s). */
  };

  ToneTiming() noexcept = default;
  ToneTiming(const ToneTiming &) = delete;
  ToneTiming &operator=(const ToneTiming &) = delete;
  ToneTiming(ToneTiming &&) = delete;
  ToneTiming &operator=(ToneTiming &&) = delete;
  ~ToneTiming() = default;

  /**
   * @param fs_hz Sample rate (Hz), at most {@link kMaxSampleRateHz}.
   * @param cfg Parameters.
   * @return {@code false} if the tone is above Nyquist, its carrier period
   *         does not fit {@link kFoldCap}, or the block is too short.
   */
  [[nodiscard]] bool configure(uint32_t fs_hz, const Config &cfg) noexcept;

  /**
   * Starts a new measurement.
   *
   * @param first_index Absolute index of the next sample fed (block 0 and
   *        the phase origin).
   */
  void reset(uint64_t first_index = 0) noexcept;

  /**
   * Feeds raw codes.
   *
   * @param codes Sign-extended 24-bit codes.
   * @param n Count (any length).
   */
  void process(const int32_t *codes, std::size_t n) noexcept;

  /** @return The rate once two blocks are complete, else nullopt. */
  [[nodiscard]] std::optional<PhaseRate> result() const noexcept {
    return tracker_.result();
  }

  /** @return {@code true} if any raw code has hit the rail since reset. */
  [[nodiscard]] bool clipped() const noexcept { return clipped_; }

private:
  /**
   * Tracker parameters for a tone: harmonic 1 only, blocks from 0.
   *
   * @param fs_hz Sample rate (Hz).
   * @param cfg Tone parameters.
   * @param origin Absolute index of block 0.
   * @return The tracker configuration.
   */
  [[nodiscard]] static HarmonicPhaseTracker::Config
  tracker_config(uint32_t fs_hz, const Config &cfg, uint64_t origin) noexcept;

  uint32_t fs_hz_ = 0;                 /**< Sample rate; 0 = unconfigured. */
  Config cfg_{};                       /**< Parameters. */
  Highpass4 hpf_;                      /**< min(200, f/2) Hz high-pass. */
  HarmonicPhaseTracker tracker_;       /**< k = 1 phase tracker. */
  std::array<float, kFoldCap> fold_{}; /**< Tracker fold buffer. */
  std::array<float, kChunk> chunk_{};  /**< Filtered chunk. */
  uint64_t next_index_ = 0; /**< Absolute index of the next sample. */
  bool clipped_ = false;    /**< Any railed code since reset. */
};

} // namespace quartz
