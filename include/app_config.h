/**
 * @file app_config.h
 * @brief Application-wide audio rates, buffer sizes, and ADC tuning knobs.
 *
 * {@code AUDIO_SOURCE} is normally set by {@code platformio.ini}
 * ({@code SYNTHETIC} / {@code ADC} / {@code PCM1808}). Most numeric defaults
 * can be overridden with {@code -DNAME=value} at build time.
 */

#pragma once

#include <stdint.h>

/** Compile-time backend selectors (match {@code -DAUDIO_SOURCE=...}). */
#define SYNTHETIC 1
#define ADC       2
#define PCM1808   3

#ifndef AUDIO_SOURCE
#define AUDIO_SOURCE ADC
#endif

#define APP_SAMPLE_RATE_HZ          48000u
#define APP_CHANNELS                1u
/** USB / ring sample width: signed 24-bit PCM in a 32-bit container (USB packs 3 LE bytes). */
#define APP_BYTES_PER_SAMPLE        3u
#define APP_SAMPLES_PER_MS          (APP_SAMPLE_RATE_HZ / 1000u) /* 48 */
/** Full-scale positive peak for signed 24-bit PCM. */
#define APP_PCM24_FULL_SCALE        8388607
#define APP_PCM24_MIN               (-8388608)

#define APP_RING_CAPACITY           8192u
#define APP_RING_PREFILL_SAMPLES    960u  /* ~20 ms cushion before USB drain */

/** PCM samples produced per DMA block after oversample decimation (~5 ms). */
#define APP_ADC_DMA_BUF_SAMPLES     240u
#define APP_ADC_DMA_BUF_COUNT       2u

/**
 * SAR ADC oversampling factor: capture at 48 kHz × N, boxcar-average N raw
 * codes into each PCM sample. Reduces uncorrelated converter noise ~√N.
 */
#ifndef ADC_OVERSAMPLE
#define ADC_OVERSAMPLE              8u
#endif
#if ADC_OVERSAMPLE < 1u
#error "ADC_OVERSAMPLE must be >= 1"
#endif
#define APP_ADC_RAW_RATE_HZ \
  (APP_SAMPLE_RATE_HZ * static_cast<uint32_t>(ADC_OVERSAMPLE))
#define APP_ADC_DMA_BUF_RAW_SAMPLES \
  (APP_ADC_DMA_BUF_SAMPLES * static_cast<uint32_t>(ADC_OVERSAMPLE))

/** PCM1808 DMA: 240 stereo frames × 2 slots = 480 uint32 words / 5 ms block. */
#define APP_PCM1808_DMA_FRAMES      240u
#define APP_PCM1808_DMA_BUF_COUNT   2u

/**
 * Extra digital gain on the PCM1808 path when DSP bypass is enabled (float
 * full-scale × gain before PCM24 quantize). 1 = unity. Does not affect the
 * bit-exact format helper used by host unit tests.
 * Override: {@code -DPCM1808_PCM_GAIN=N}.
 */
#ifndef PCM1808_PCM_GAIN
#define PCM1808_PCM_GAIN            1
#endif
#if PCM1808_PCM_GAIN < 1
#error "PCM1808_PCM_GAIN must be >= 1"
#endif

/*
 * Repeat each mono sample this many times into the 48 kHz USB ring.
 * Empirically the PCM1808 path was producing ~24 k frames/s (DMA ~100
 * blocks/s of 240 frames) — typically LRCK at 24 kHz from 512×fs straps
 * or half-rate MCLK. 2× upsample restores pitch and stops underrun holds.
 * Set to 1 if scope shows LRCK ≈ 48 kHz.
 */
#ifndef PCM1808_OUTPUT_UPSAMPLE
#define PCM1808_OUTPUT_UPSAMPLE     2
#endif
#if PCM1808_OUTPUT_UPSAMPLE < 1
#error "PCM1808_OUTPUT_UPSAMPLE must be >= 1"
#endif

/**
 * Fallback midrail in ADC counts when DC learning is disabled or not ready.
 * Ideal 1.65 V bias on a 3.3 V reference ≈ 2048.
 */
#ifndef ADC_DC_OFFSET_COUNTS
#define ADC_DC_OFFSET_COUNTS        2048
#endif

/**
 * When 1, learn midrail from the first DMA block and subtract it before
 * scaling. When 0, treat code 0 as “zero” (raw); a hardware midrail bias then
 * rails USB high — only for special bench tests.
 */
#ifndef ADC_DC_REMOVE
#define ADC_DC_REMOVE               1
#endif

/**
 * Multiplier from centered ADC counts in the historical 16-bit domain (no AGC).
 * Applied then left-shifted 8 bits into PCM24. 16 maps ±2048 onto ±32768
 * before the shift (unity into 24-bit). Values &gt; 16 boost quiet ticks but
 * clip earlier. Override: {@code -DADC_PCM_SCALE=N}.
 */
#ifndef ADC_PCM_SCALE
#define ADC_PCM_SCALE               48
#endif

/**
 * Optional one-pole PCM low-pass after convert: {@code y += (x - y) >> SHIFT}.
 * 0 = off. Non-zero softens SAR hash but also attenuates short watch-tick
 * impulses — keep 0 unless deliberately filtering.
 */
#ifndef ADC_LPF_SHIFT
#define ADC_LPF_SHIFT               0
#endif

/** Occupancy targets for optional 47/48/49 samples-per-USB-frame steering. */
#define APP_PKT_SAMPLES_MIN         47u
#define APP_PKT_SAMPLES_NOM         48u
#define APP_PKT_SAMPLES_MAX         49u

/**
 * When 1, {@link audio_ring_packet_samples} always returns 48 (hosts that
 * dislike variable ISO sizes).
 */
#ifndef APP_FIXED_PACKET_48
#define APP_FIXED_PACKET_48         0
#endif

#ifndef DIAG_UART
#define DIAG_UART                   0
#endif

#ifndef DIAG_UART_BAUD
#define DIAG_UART_BAUD              115200
#endif

/**
 * Compile-time: allow CDC ({@code Serial}) diagnostic dump lines when the
 * runtime print flag is on ({@code diag on} in the CLI). Prints start
 * disabled; counters still accumulate either way.
 */
#ifndef DIAG_CDC
#define DIAG_CDC                    1
#endif

/**
 * Nonblocking TL071 / 8 V rail warm-up after TPS22918 enable (milliseconds).
 * Provisional — measure rails before treating as final.
 */
#ifndef PREAMP_SETTLE_MS
#define PREAMP_SETTLE_MS            50u
#endif
