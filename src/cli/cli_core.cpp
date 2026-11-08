/**
 * @file cli_core.cpp
 * @brief Line editor (insert / backspace / enter), tokenizer, and dispatcher.
 *
 * @see cli_core.h
 */

#include "cli/cli_core.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace Cli {
namespace {

char line_buffer[kLineCapacity];
uint8_t line_len = 0;

/** Escape-sequence swallow state (arrows etc. are ignored, never echoed). */
enum class EscState : uint8_t { None, Esc, Csi, Ss3 };
EscState esc_state = EscState::None;
/** True right after CR so the LF of a CRLF pair is dropped. */
bool saw_cr = false;

const CliCommand *commands = nullptr;
size_t command_count = 0;

/**
 * Submits the current line: echoes newline, dispatches, resets, reprints prompt.
 */
void submitLine() {
  Serial.println();
  line_buffer[line_len] = '\0';
  dispatchLine(line_buffer);
  line_len = 0;
  line_buffer[0] = '\0';
  Serial.print(F("> "));
}

/**
 * Feeds one received byte to the editor.
 *
 * @param c Byte from the host terminal.
 */
void handleByte(char c) {
  if (esc_state == EscState::None && c == '\n' && saw_cr) {
    saw_cr = false;
    return;
  }

  if (esc_state == EscState::Esc) {
    if (c == '[') {
      esc_state = EscState::Csi;
      return;
    }
    if (c == 'O') {
      esc_state = EscState::Ss3;
      return;
    }
    esc_state = EscState::None;
    return;
  }
  if (esc_state == EscState::Csi) {
    /* Parameter/intermediate bytes 0x20-0x3F; final byte 0x40-0x7E ends it. */
    if (c >= 0x40 && c <= 0x7E) {
      esc_state = EscState::None;
    }
    return;
  }
  if (esc_state == EscState::Ss3) {
    esc_state = EscState::None;
    return;
  }

  if (c == '\x1b') {
    esc_state = EscState::Esc;
    return;
  }
  if (c == '\r') {
    saw_cr = true;
    submitLine();
    return;
  }
  if (c == '\n') {
    submitLine();
    return;
  }
  saw_cr = false;

  if (c == '\b' || c == 0x7f) {
    if (line_len > 0) {
      line_len--;
      line_buffer[line_len] = '\0';
      Serial.print(F("\b \b"));
    }
    return;
  }
  if (c == 0x03) { /* Ctrl-C: abandon line */
    line_len = 0;
    line_buffer[0] = '\0';
    Serial.println(F("^C"));
    Serial.print(F("> "));
    return;
  }
  if (c >= 32 && c < 127 && line_len < kLineCapacity - 1) {
    line_buffer[line_len++] = c;
    line_buffer[line_len] = '\0';
    Serial.write(static_cast<uint8_t>(c));
  }
}

/**
 * Trims leading/trailing spaces and tabs in place.
 */
void trimInPlace(char *text) {
  char *start = text;
  while (*start == ' ' || *start == '\t') {
    start++;
  }
  if (start != text) {
    memmove(text, start, strlen(start) + 1);
  }
  char *end = text + strlen(text);
  while (end > text && (end[-1] == ' ' || end[-1] == '\t')) {
    end--;
  }
  *end = '\0';
}

/**
 * @return {@code true} if {@code token} matches one of the space-separated
 *         {@code aliases} (case-insensitive).
 */
bool matchAlias(const char *token, const char *aliases) {
  if (aliases == nullptr || token == nullptr) {
    return false;
  }
  const char *p = aliases;
  while (*p != '\0') {
    while (*p == ' ') {
      p++;
    }
    const char *start = p;
    while (*p != '\0' && *p != ' ') {
      p++;
    }
    const size_t len = static_cast<size_t>(p - start);
    if (len > 0 && len < 24) {
      char word[24];
      memcpy(word, start, len);
      word[len] = '\0';
      if (equalsIgnoreCase(token, word)) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @return Matching command by name or alias, or {@code nullptr}.
 */
const CliCommand *findCommand(const char *name) {
  for (size_t i = 0; i < command_count; i++) {
    if (equalsIgnoreCase(name, commands[i].name) ||
        matchAlias(name, commands[i].aliases)) {
      return &commands[i];
    }
  }
  return nullptr;
}

} // namespace

void coreInit() {
  line_len = 0;
  line_buffer[0] = '\0';
  esc_state = EscState::None;
  saw_cr = false;
  Serial.println();
  Serial.println(F("Serial CLI ready. Type 'help' for commands."));
  Serial.print(F("> "));
}

void coreTick() {
  while (Serial.available() > 0) {
    handleByte(static_cast<char>(Serial.read()));
  }
}

bool equalsIgnoreCase(const char *a, const char *b) {
  if (a == nullptr || b == nullptr) {
    return a == b;
  }
  while (*a && *b) {
    if (tolower(static_cast<unsigned char>(*a)) !=
        tolower(static_cast<unsigned char>(*b))) {
      return false;
    }
    a++;
    b++;
  }
  return *a == *b;
}

bool parseLong(const char *s, long &out) {
  if (s == nullptr || *s == '\0') {
    return false;
  }
  char *end = nullptr;
  const long v = strtol(s, &end, 10);
  if (end == s || end == nullptr || *end != '\0') {
    return false;
  }
  out = v;
  return true;
}

void registerCommands(const CliCommand *cmds, size_t count) {
  commands = cmds;
  command_count = (cmds != nullptr) ? count : 0;
}

void dispatchLine(char *line) {
  if (line == nullptr) {
    return;
  }
  trimInPlace(line);
  if (line[0] == '\0') {
    return;
  }

  char *argv[kMaxArgs] = {};
  int argc = 0;
  char *save = nullptr;
  for (char *tok = strtok_r(line, " \t", &save);
       tok != nullptr && argc < kMaxArgs;
       tok = strtok_r(nullptr, " \t", &save)) {
    argv[argc++] = tok;
  }
  if (argc == kMaxArgs && strtok_r(nullptr, " \t", &save) != nullptr) {
    Serial.println(F("CLI: too many arguments (truncated)"));
  }
  if (argc == 0) {
    return;
  }

  const CliCommand *cmd = findCommand(argv[0]);
  if (cmd == nullptr || cmd->handler == nullptr) {
    Serial.println(F("CLI: unknown command (try 'help')"));
    return;
  }
  cmd->handler(argc, argv);
}

void printHelp(const char *topic) {
  if (Serial.availableForWrite() < 64) {
    return;
  }

  if (topic != nullptr && topic[0] != '\0') {
    const CliCommand *cmd = findCommand(topic);
    if (cmd == nullptr) {
      Serial.print(F("CLI: no help for '"));
      Serial.print(topic);
      Serial.println(F("'"));
      return;
    }
    Serial.print(F("CLI: help "));
    Serial.print(cmd->name);
    if (cmd->aliases != nullptr && cmd->aliases[0] != '\0') {
      Serial.print(F(" ("));
      Serial.print(cmd->aliases);
      Serial.print(F(")"));
    }
    Serial.println();
    if (cmd->summary != nullptr) {
      Serial.print(F("  "));
      Serial.println(cmd->summary);
    }
    if (cmd->usage != nullptr) {
      Serial.print(F("  Usage: "));
      Serial.println(cmd->usage);
    }
    if (cmd->details != nullptr) {
      cmd->details();
    }
    return;
  }

  Serial.println(F("CLI: commands:"));
  for (size_t i = 0; i < command_count; i++) {
    Serial.print(F("  "));
    Serial.print(commands[i].name);
    if (commands[i].aliases != nullptr && commands[i].aliases[0] != '\0') {
      Serial.print(F(" ("));
      Serial.print(commands[i].aliases);
      Serial.print(F(")"));
    }
    if (commands[i].summary != nullptr) {
      Serial.print(F(" - "));
      Serial.print(commands[i].summary);
    }
    Serial.println();
  }
  Serial.println(F("  Use 'help <command>' or '<command> help' for details."));
}

} // namespace Cli
