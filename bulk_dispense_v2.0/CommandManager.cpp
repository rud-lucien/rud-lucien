#include "CommandManager.h"

// Module-level state variables.
static bool sessionActive = false;
static unsigned long sessionStartTime = 0;
static int pendingCommands = 0;

volatile bool commandLineBeingProcessed = false;

void cm_startSession(Stream* stream) {
  if (!sessionActive) {
    sessionActive = true;
    sessionStartTime = millis();
    stream->println(F("[ACTION START]"));
  }
}

void cm_endSession(Stream* stream) {
  if (sessionActive) {
    unsigned long duration = millis() - sessionStartTime;
    stream->print(F("[ACTION END] Duration: "));
    stream->print(duration);
    stream->println(F(" ms"));
    sessionActive = false;
    pendingCommands = 0;
  }
}

void cm_registerCommand(void) {
  pendingCommands++;
}


void cm_commandCompleted(Stream* stream) {
    if (!sessionActive) {  // If the session was aborted, ignore.
      return;
    }
    if (pendingCommands > 0) {
      pendingCommands--;
      stream->print(F("[DEBUG] Command completed. Pending: "));
      stream->println(pendingCommands);
    }
    if (pendingCommands <= 0 && !commandLineBeingProcessed && sessionActive) {
      cm_endSession(stream);
    }
  }
  
  

bool cm_isSessionActive(void) {
  return sessionActive;
}

int cm_getPendingCommands(void) {
    return pendingCommands;
  }


void cm_abortSession(Stream* stream) {
    // End the session (this will print [ACTION END] and reset timers)
    cm_endSession(stream);
    
    // Then set sessionActive to false and clear pending commands
    sessionActive = false;
    pendingCommands = 0;
  }
  
  
  
  