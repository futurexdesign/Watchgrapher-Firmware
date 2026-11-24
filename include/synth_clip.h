/**
 * @file synth_clip.h
 * @brief The looped table of the SYNTHETIC audio source.
 *
 * By default the build loops the generated tick-tock recording
 * (include/tick_tock_pcm.h). A build with {@code -DTG_SYNTH_CLIP=<id>} loops
 * a trimmed watch recording from assets/synth_clips/ instead: the pre-build
 * script scripts/synth_clip_header.py generates {@code synth_clip_pcm.h} into
 * the build directory and defines {@code TG_SYNTH_CLIP_NAME}. Both tables
 * expose the same names: {@code kTickTockPcm} and {@code TICK_TOCK_PCM_LEN}.
 *
 * Include only from the synthetic source (audio_source.cpp): the table is
 * defined here, not declared.
 */

#pragma once

#if defined(TG_SYNTH_CLIP)
#if !defined(TG_SYNTH_CLIP_NAME)
#error                                                                         \
    "TG_SYNTH_CLIP is set but scripts/synth_clip_header.py did not run (extra_scripts in platformio.ini)"
#endif
#include "synth_clip_pcm.h"
#else
#include "tick_tock_pcm.h"
#endif
