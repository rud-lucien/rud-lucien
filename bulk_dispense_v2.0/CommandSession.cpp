#include "CommandSession.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Utils.h"
#include "Commands.h"
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include <Wire.h>

/************************************************************
 * CommandSession.cpp
 *
 * This file implements the functions for managing an 
 * asynchronous command session in the Bulk Dispense system.
 *
 * A command session marks the period during which a command (or
 * a batch of commands) is processed. It provides tags for action
 * start and end (including duration), and tracks asynchronous commands
 * so that the session only ends when all async operations have completed.
 *
 * Global Variables:
 *   - commandSessionActive: true if a session is active.
 *   - commandSessionStartTime: the millis() timestamp when the session started.
 *   - pendingAsyncCommands: number of asynchronous commands still pending.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Global Session State Variables
// ============================================================
bool commandSessionActive = false;
unsigned long commandSessionStartTime = 0;
int pendingAsyncCommands = 0;

// ============================================================
// Session Management Functions
// ============================================================

void startCommandSession(Stream* stream) {
  if (!commandSessionActive) {
    commandSessionActive = true;
    commandSessionStartTime = millis();
    stream->println(F("[ACTION START]"));
  }
}

void endCommandSession(Stream* stream) {
  if (commandSessionActive) {
    unsigned long duration = millis() - commandSessionStartTime;
    stream->print(F("[ACTION END] Duration: "));
    stream->print(duration);
    stream->println(F(" ms"));
    // Reset session state.
    commandSessionActive = false;
    pendingAsyncCommands = 0;
  }
}

void registerAsyncCommand() {
  pendingAsyncCommands++;
}

void asyncCommandCompleted(Stream* stream) {
  pendingAsyncCommands--;
  // End the session when no async commands are pending.
  if (pendingAsyncCommands <= 0) {
    endCommandSession(stream);
  }
}

// ============================================================
// Helper Function for Determining Asynchronous Commands
// ============================================================
bool isAsyncCommand(const char* command) {
  // Treat the following commands as asynchronous:
  // Drain Trough (DT), Prime (P), Dispense (D), and Stop Drain (SDT).
  // (Fill (F) is intentionally excluded because its action ends immediately.)
  return (strncmp(command, "DT", 2) == 0 ||
          strncmp(command, "P", 1) == 0 ||
          strncmp(command, "D", 1) == 0 ||
          strncmp(command, "SDT", 3) == 0);
}

