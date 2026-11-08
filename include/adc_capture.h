/**
 * @file adc_capture.h
 * @brief Onboard SAR ADC capture path (DMA + FIFO → PCM24).
 *
 * Active when {@code AUDIO_SOURCE == ADC}. Samples {@code ADC_GPIO} at
 * {@code APP_ADC_RAW_RATE_HZ}, boxcar-averages {@code ADC_OVERSAMPLE} raw
 * conversions into each 48 kHz PCM sample, optionally removes a learned
 * midrail DC, scales by {@code ADC_PCM_SCALE} into a 24-bit container, and
 * pushes into the audio ring.
 *
 * Start/stop must run from the main loop (not TinyUSB control callbacks) —
 * aborting DMA from a USB ISR context can hang the stack on RP2040/RP2350.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes the ADC peripheral, GPIO analog mode, sample clock divider,
 * FIFO, and a single DMA channel with IRQ0 completion handler.
 *
 * Does not start conversions; call {@link adc_capture_start} when the host
 * opens the UAC streaming alternate setting.
 *
 * @return {@code true} on success (always, unless channel claim fails hard).
 */
bool adc_capture_init(void);

/**
 * Starts free-running ADC → DMA ping-pong into the internal raw buffers.
 *
 * Resets DC-learning state, drains the FIFO, and arms the first DMA transfer.
 * Safe to call if already running (no-op).
 */
void adc_capture_start(void);

/**
 * Stops ADC conversions and aborts DMA. Drains the FIFO and clears ready flags.
 * Safe to call if already stopped.
 */
void adc_capture_stop(void);

/**
 * @return {@code true} if ADC+DMA capture is currently armed and running.
 */
bool adc_capture_is_running(void);

/**
 * Drains any DMA buffers marked ready: oversamples/averages, converts to
 * PCM24, and pushes into the audio ring. Must be called frequently from the
 * main loop while streaming so ping-pong buffers do not overrun.
 */
void adc_capture_process(void);

/**
 * Converts one ADC FIFO word to signed PCM24 (int32 container).
 *
 * Masks the 12-bit result, applies DC removal / scale / optional LPF per
 * {@code app_config.h}, expands into the 24-bit range, and updates
 * diagnostics. Bit 15 set indicates a conversion error; the previous valid
 * code is reused and an error counter is incremented.
 *
 * Also used by host-side unit tests (mirror the scaling contract there).
 *
 * @param fifo_word Raw 16-bit FIFO entry (bits 11:0 = code, bit 15 = ERR).
 * @return Clamped signed 24-bit PCM sample in an int32.
 */
int32_t adc_convert_sample(uint16_t fifo_word);

#ifdef __cplusplus
}
#endif
