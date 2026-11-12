/**
 * @file pcm1808_i2s.pio.h
 * @brief Board-selecting wrapper for PCM1808 I²S PIO (autogen companions).
 *
 * wait gpio immediates are baked per board — do not share one program blob.
 * Regenerate with: {@code python3 scripts/gen_pcm1808_pio.py}
 */
#pragma once

#if defined(BOARD_PICO2)
#include "pcm1808_i2s_pico2.pio.h"
#else
#include "pcm1808_i2s_legacy.pio.h"
#endif
