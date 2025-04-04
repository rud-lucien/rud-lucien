#include "CommandManager.h"

// // Module-level state variables.
// static bool sessionActive = false;
// static unsigned long sessionStartTime = 0;
// static int pendingCommands = 0;

// volatile bool commandLineBeingProcessed = false;

// void cm_startSession(Stream* stream) {
//   if (!sessionActive) {
//     sessionActive = true;
//     sessionStartTime = millis();
//     stream->println(F("[ACTION START]"));
//   }
// }

// void cm_endSession(Stream* stream) {
//   if (sessionActive) {
//     unsigned long duration = millis() - sessionStartTime;
//     stream->print(F("[ACTION END] Duration: "));
//     stream->print(duration);
//     stream->println(F(" ms"));
//     sessionActive = false;
//     pendingCommands = 0;
//   }
// }

// void cm_registerCommand(void) {
//   pendingCommands++;
// }


// void cm_commandCompleted(Stream* stream) {
//     if (!sessionActive) {  // If the session was aborted, ignore.
//       return;
//     }
//     if (pendingCommands > 0) {
//       pendingCommands--;
//       stream->print(F("[DEBUG] Command completed. Pending: "));
//       stream->println(pendingCommands);
//     }
//     if (pendingCommands <= 0 && !commandLineBeingProcessed && sessionActive) {
//       cm_endSession(stream);
//     }
//   }
  
  

// bool cm_isSessionActive(void) {
//   return sessionActive;
// }

// int cm_getPendingCommands(void) {
//     return pendingCommands;
//   }


// void cm_abortSession(Stream* stream) {
//     // End the session (this will print [ACTION END] and reset timers)
//     cm_endSession(stream);
    
//     // Then set sessionActive to false and clear pending commands
//     sessionActive = false;
//     pendingCommands = 0;
//   }

#include "CommandManager.h"

// Module-level state variables
static StreamSession serialSession = {false, 0, &Serial};
static StreamSession networkSession = {false, 0, nullptr};
static int pendingCommands = 0;

volatile bool commandLineBeingProcessed = false;

// Helper to get session for a stream
static StreamSession* getSessionForStream(Stream* stream) {
    if (stream == &Serial) return &serialSession;
    if (stream == &currentClient) {
        networkSession.stream = &currentClient;  // Update network stream pointer
        return &networkSession;
    }
    return nullptr;
}

void cm_startSession(Stream* stream) {
    StreamSession* session = getSessionForStream(stream);
    if (!session) return;
    
    if (!session->active) {
        session->active = true;
        session->startTime = millis();
        stream->println(F("[ACTION START]"));
    }
}

void cm_endSession(Stream* stream) {
    StreamSession* session = getSessionForStream(stream);
    if (!session || !session->active) return;
    
    unsigned long duration = millis() - session->startTime;
    stream->print(F("[ACTION END] Duration: "));
    stream->print(duration);
    stream->println(F(" ms"));
    session->active = false;
}


void cm_registerCommand(void) {
    pendingCommands++;
}

// void cm_commandCompleted(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     if (!session || !session->active) return;
    
//     if (pendingCommands > 0) {
//         pendingCommands--;
//         stream->print(F("[DEBUG] Command completed. Pending: "));
//         stream->println(pendingCommands);
//     }
    
//     if (pendingCommands <= 0 && !commandLineBeingProcessed) {
//         cm_endSession(stream);
//     }
// }

// void cm_commandCompleted(Stream* stream) {
//   StreamSession* session = getSessionForStream(stream);
//   if (!session || !session->active) return;  // If session doesn't exist or was aborted, ignore
  
//   if (pendingCommands > 0) {
//       pendingCommands--;
//       sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
//       sendMessage(String(pendingCommands).c_str(), stream, currentClient);
//   }
  
//   // Only end session if ALL conditions are met (like in original version)
//   if (pendingCommands <= 0 && !commandLineBeingProcessed && session->active) {
//       cm_endSession(stream);
//   }
// }

void cm_commandCompleted(Stream* stream) {
  StreamSession* session = getSessionForStream(stream);
  if (!session || !session->active) {  // If session doesn't exist or was aborted, ignore
      return;
  }

  if (pendingCommands > 0) {
      pendingCommands--;
      // Use the original message format but with sendMessage for network support
      sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
      sendMessage(String(pendingCommands).c_str(), stream, currentClient);
  }

  // Use the same conditions as the original version
  if (pendingCommands <= 0 && !commandLineBeingProcessed && session->active) {
      // Only end the session on the stream that started it
      if ((stream == &Serial && serialSession.active) || 
          (stream == &currentClient && networkSession.active)) {
          cm_endSession(stream);
      }
  }
}


bool cm_isSessionActive(void) {
    return serialSession.active || networkSession.active;
}

int cm_getPendingCommands(void) {
    return pendingCommands;
}

void cm_abortSession(Stream* stream) {
    StreamSession* session = getSessionForStream(stream);
    if (!session) return;
    
    cm_endSession(stream);
    session->active = false;
    pendingCommands = 0;
} 
  
  