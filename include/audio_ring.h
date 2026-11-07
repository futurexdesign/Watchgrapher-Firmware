/**
 * @file audio_ring.h
 * @brief Lock-free-ish mono PCM16 ring between capture and USB IN feeder.
 *
 * Single producer (capture / synth) and single consumer (USB audio task).
 * Indices are free-running; capacity must be a power of two. One slot is left
 * empty so full vs empty can be distinguished without an extra flag.
 *
 * Occupancy is sampled for diagnostics and (optionally) for 47/48/49 packet
 * size steering via {@link audio_ring_packet_samples}.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Clears the ring and resets diagnostic min/max occupancy trackers.
 * Equivalent to {@link audio_ring_reset} at boot.
 */
void audio_ring_init(void);

/**
 * Discards all buffered samples and zeroes storage. Call on stream open/close
 * and when stopping capture so stale PCM is never sent to the host.
 */
void audio_ring_reset(void);

/**
 * @return Number of PCM16 samples currently buffered (0 .. capacity−1).
 */
uint32_t audio_ring_occupancy(void);

/**
 * @return Free slots available for {@link audio_ring_push} (leaves one empty).
 */
uint32_t audio_ring_free(void);

/**
 * @return {@code true} when occupancy ≥ {@code APP_RING_PREFILL_SAMPLES}
 *         (~10 ms), enough to begin ISO feeding without immediate underrun.
 */
bool audio_ring_is_prefilled(void);

/**
 * Producer: append PCM16 samples.
 *
 * Stops early and increments {@code pcm_ring_overrun_count} if the ring fills.
 *
 * @param samples Pointer to {@code count} signed 16-bit mono samples.
 * @param count   Number of samples to attempt.
 * @return Number of samples actually written (≤ {@code count}).
 */
uint32_t audio_ring_push(const int16_t *samples, uint32_t count);

/**
 * Consumer: remove up to {@code count} samples into {@code out}.
 *
 * On underrun, increments {@code pcm_ring_underrun_count}. If
 * {@code pad_silence} is true, remaining slots are zero-filled and counted as
 * read; if false, the call returns early with a short count (USB path prefers
 * holding the last sample instead — see {@code usb_audio.cpp}).
 *
 * @param out          Destination buffer, length ≥ {@code count}.
 * @param count        Maximum samples to pop.
 * @param pad_silence  Zero-fill remainder on underrun when true.
 * @return Samples written to {@code out}.
 */
uint32_t audio_ring_pop(int16_t *out, uint32_t count, bool pad_silence);

/**
 * Suggests how many samples to put in the next USB 1 ms ISO frame.
 *
 * When {@code APP_FIXED_PACKET_48} is 0, returns 47 / 48 / 49 based on ring
 * occupancy to absorb crystal vs SOF drift. When fixed mode is on, always 48.
 *
 * @note The current USB feeder pushes a nominal 48-sample software frame and
 *       relies on TinyUSB EP IN flow control for on-wire size; this helper
 *       remains available for alternate schedulers / tests.
 *
 * @return {@code APP_PKT_SAMPLES_MIN}, {@code _NOM}, or {@code _MAX}.
 */
uint32_t audio_ring_packet_samples(void);

#ifdef __cplusplus
}
#endif
