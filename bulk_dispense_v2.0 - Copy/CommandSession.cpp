#include "CommandSession.h"

bool commandSessionActive = false;
unsigned long commandSessionStartTime = 0;
int pendingAsyncCommands = 0;

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
    // Reset session state
    commandSessionActive = false;
    pendingAsyncCommands = 0;
  }
}

void registerAsyncCommand() {
  pendingAsyncCommands++;
}

void asyncCommandCompleted(Stream* stream) {
  pendingAsyncCommands--;
  // If no more asynchronous commands are pending, end the session.
  if (pendingAsyncCommands <= 0) {
    endCommandSession(stream);
  }
}

// Example implementation: Mark commands starting with "F", "DT", or "P" as asynchronous.
bool isAsyncCommand(const char* command) {
  // Treat Drain Trough (DT), Prime (P), Dispense (D), and Stop Drain (SDT) as asynchronous.
  // Do not treat Fill (F) as asynchronous because we want to end the action immediately.
  return (strncmp(command, "DT", 2) == 0 ||
          strncmp(command, "P", 1) == 0 ||
          strncmp(command, "D", 1) == 0 ||
          strncmp(command, "SDT", 3) == 0);
}


