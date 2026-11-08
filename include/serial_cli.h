/**
 * @file serial_cli.h
 * @brief USB CDC serial command-line interface entry points.
 *
 * Runs cooperatively from {@code loop()}; never blocks. Shares CDC with an
 * optional periodic {@link diagnostics_poll} dump — off until {@code diag on}.
 */

#pragma once

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
