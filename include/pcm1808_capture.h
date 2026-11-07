/**
 * @file pcm1808_capture.h
 * @brief External PCM1808 I²S capture (PIO RX + DMA → mono PCM16 ring).
 *
 * Active when {@code AUDIO_SOURCE == PCM1808}. Expects board-generated MCLK
 * ({@link pcm1808_clock_start}) and codec-driven BCK/LRCK into the PIO program.
 * Left channel only is converted via {@link pcm1808_left_slot_to_pcm16}.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Claims a PIO state machine, loads the I²S RX program, and configures
 * ping-pong DMA into stereo slot buffers. Does not enable capture until
 * {@link pcm1808_capture_start}.
 *
 * @return {@code false} if no free PIO SM / program space.
 */
bool pcm1808_capture_init(void);

/**
 * Requests hardware start (or completes a deferred start once MCLK warmup
 * and {@link pcm1808_capture_is_ready} allow). Safe if already running.
 */
void pcm1808_capture_start(void);

/**
 * Stops PIO + DMA and clears ready flags. Safe if already stopped.
 */
void pcm1808_capture_stop(void);

/**
 * @return {@code true} while PIO/DMA are actively filling buffers.
 */
bool pcm1808_capture_is_running(void);

/**
 * @return {@code true} when MCLK warmup deadline has passed and clocks look OK
 *         — capture may proceed.
 */
bool pcm1808_capture_is_ready(void);

/**
 * Converts completed DMA stereo blocks to mono PCM16 and pushes the ring.
 * Call from the main loop while streaming.
 */
void pcm1808_capture_process(void);

#ifdef __cplusplus
}
#endif
