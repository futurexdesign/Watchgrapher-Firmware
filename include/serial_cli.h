/**
 * @file serial_cli.h
 * @brief USB CDC serial command-line interface entry points.
 *
 * Runs cooperatively from {@code loop()}; never blocks. Shares CDC with an
 * optional periodic {@link diagnostics_poll} dump — off until {@code diag on}.
 *
 * The CLI is built only when {@code TG_SERIAL_CLI=1} (the {@code *_cli} envs,
 * plan §5.7). Otherwise {@code src/serial_cli.cpp} and {@code src/cli/} are
 * not compiled and the entry points below are inline no-ops, so callers need
 * no {@code #if}. The CDC interface itself stays in every build.
 */

#pragma once

#include "app_config.h"

#if TG_SERIAL_CLI

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Registers commands and prints the banner and prompt. Call once from
 * {@code setup()} after {@link diagnostics_init} (which opens {@code Serial}).
 */
void serial_cli_init(void);

/**
 * Services pending CDC input. Call every {@code loop()} iteration after
 * {@link usb_audio_task}.
 */
void serial_cli_tick(void);

#ifdef __cplusplus
}
#endif

#else /* !TG_SERIAL_CLI */

/** No-op: the serial CLI is not built. Writes nothing to {@code Serial}. */
static inline void serial_cli_init(void) {}

/** No-op: the serial CLI is not built. */
static inline void serial_cli_tick(void) {}

#endif /* TG_SERIAL_CLI */
