/**
 * @file cli_commands.h
 * @brief Built-in command table registration for the USB serial CLI.
 */

#pragma once

namespace Cli {

/**
 * Registers all built-in commands with {@code cli_core}.
 * Call once from {@link serial_cli_init}.
 */
void registerAllCommands();

} // namespace Cli
