/**
 * @file cli_core.h
 * @brief Minimal USB CDC line editor, tokenizer, and command dispatcher.
 *
 * UberKiwi-style: {@code > } prompt, {@code CLI:} reply prefix, case-insensitive
 * command/alias matching, {@code help} / {@code help <cmd>}. No command history;
 * ANSI escape sequences (arrows, Home/End, ...) are swallowed, not interpreted.
 */

#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace Cli {

/** Maximum characters in the input line (including NUL). */
static constexpr uint8_t kLineCapacity = 96;
/** Maximum argv slots after tokenization (command + args). */
static constexpr uint8_t kMaxArgs = 12;

/** Command handler: argc/argv with argv[0] = command name as typed. */
using CliHandler = void (*)(int argc, char **argv);

/**
 * Optional extended help printer for {@code help <cmd>} (multi-line details).
 * May be {@code nullptr} when summary/usage alone are enough.
 */
using CliHelpFn = void (*)(void);

/**
 * One registered CLI command (flash-resident help strings).
 */
struct CliCommand {
  const char *name;                   ///< Primary verb
  const char *aliases;                ///< Space-separated aliases, or nullptr
  CliHandler handler;                 ///< Dispatch target
  const __FlashStringHelper *summary; ///< One-line help
  const __FlashStringHelper *usage;   ///< Usage string for {@code help <cmd>}
  CliHelpFn details;                  ///< Extended help, or nullptr
};

/**
 * Resets the line editor and prints the ready banner plus prompt.
 */
void coreInit();

/**
 * Drains CDC {@code Serial} into the line editor. Non-blocking: returns when
 * no bytes are pending.
 */
void coreTick();

/**
 * Installs the command table used by {@link #dispatchLine}.
 *
 * @param cmds Static array (not copied).
 * @param count Number of entries.
 */
void registerCommands(const CliCommand *cmds, size_t count);

/**
 * Trims, tokenizes (in place), looks up the command, and invokes its handler.
 *
 * @param line Editable NUL-terminated command line (mutated by tokenizer).
 */
void dispatchLine(char *line);

/**
 * Prints the command list or details for one command.
 *
 * @param topic Command name/alias, or {@code nullptr} for the summary list.
 */
void printHelp(const char *topic);

/**
 * Case-insensitive C-string equality.
 *
 * @return {@code true} if both null or contents match ignoring case.
 */
bool equalsIgnoreCase(const char *a, const char *b);

/**
 * Parses a full-string decimal integer (no trailing junk).
 *
 * @param s Input token.
 * @param out Parsed value on success.
 * @return {@code false} if {@code s} is null/empty/non-integer.
 */
bool parseLong(const char *s, long &out);

} // namespace Cli
