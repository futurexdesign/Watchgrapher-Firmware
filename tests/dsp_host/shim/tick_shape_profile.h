/**
 * @file tick_shape_profile.h
 * @brief Host wall-clock measurements for individual stage-B operations.
 */

#pragma once

#include <chrono>
#include <cstdint>

namespace tick_shape_profile {

/** Duration and location of calls to one stage over the looped WAV. */
struct Stage {
  const char *name;               /**< Stage name in the metrics output. */
  uint64_t calls = 0;             /**< Completed calls. */
  uint64_t total_ns = 0;          /**< Sum of all call durations. */
  uint64_t max_call_ns = 0;       /**< Largest individual call duration. */
  uint64_t max_call_sample = 0;   /**< Start of the block with that call. */
  uint64_t max_block_ns = 0;      /**< Largest sum of calls in one block. */
  uint64_t max_block_sample = 0;  /**< Start of the block with that sum. */
  uint64_t last_block_sample = 0; /**< Start of the most recent block. */
  uint64_t last_block_ns = 0;     /**< Elapsed stage time in that block. */
};

/** First source sample of the block currently running in the harness. */
inline uint64_t block_sample = 0;
/** Envelope filtering and decimation for one block. */
inline Stage push{"envelope_push"};
/** Coarse and fine correlation against one side's template. */
inline Stage match{"template_match"};
/** Template update, including the scan for its drop landmark. */
inline Stage fold{"template_fold"};
/** Addition of one aligned beat to its shape window. */
inline Stage window_add{"window_add"};
/** Landmark extraction for one side of a sealed shape window. */
inline Stage window_step{"window_step"};

/** Times each instrumented call, including paths which return early. */
class Timer {
  using Clock = std::chrono::steady_clock;
  Stage &stage_;
  Clock::time_point start_;

public:
  /** Starts measuring wall-clock time for {@code stage}. */
  explicit Timer(Stage &stage) : stage_(stage), start_(Clock::now()) {}

  /** Records elapsed time and peaks for individual calls and whole blocks. */
  ~Timer() {
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now() - start_);
    const auto ns = static_cast<uint64_t>(elapsed.count());
    stage_.calls++;
    stage_.total_ns += ns;
    if (ns > stage_.max_call_ns) {
      stage_.max_call_ns = ns;
      stage_.max_call_sample = block_sample;
    }
    if (block_sample != stage_.last_block_sample) {
      stage_.last_block_sample = block_sample;
      stage_.last_block_ns = 0;
    }
    stage_.last_block_ns += ns;
    if (stage_.last_block_ns > stage_.max_block_ns) {
      stage_.max_block_ns = stage_.last_block_ns;
      stage_.max_block_sample = block_sample;
    }
  }
};

} // namespace tick_shape_profile
