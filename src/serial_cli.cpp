/**
 * @file serial_cli.cpp
 * @brief Glue between the main loop and the Cli namespace / control_api.
 */

#include "serial_cli.h"

#include "cli/cli_commands.h"
#include "cli/cli_core.h"
#include "control_api.h"

#include <Arduino.h>
#include <cstring>

void serial_cli_init(void) {
  control_api_init();
  Cli::registerAllCommands();
  Cli::coreInit();
}

void serial_cli_tick(void) {
  control_api_watchdog_poll();

  char msg[96];
  if (control_api_pop_async_message(msg, sizeof(msg))) {
    if (Serial.availableForWrite() >= static_cast<int>(strlen(msg)) + 2) {
      Serial.println(msg);
    }
  }

  Cli::coreTick();
}
