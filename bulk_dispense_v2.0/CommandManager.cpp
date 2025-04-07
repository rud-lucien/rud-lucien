#include "CommandManager.h"

// Make sure this extern declaration is at the top of the file
extern unsigned long networkCommandStartTime;
// unsigned long serialCommandStartTime = 0;
extern unsigned long serialCommandStartTime;

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

// void cm_startSession(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     if (!session) return;
    
//     if (!session->active) {
//         session->active = true;
//         session->startTime = millis();
//         stream->println(F("[ACTION START]"));
//     }
// }

void cm_startSession(Stream* stream) {
    StreamSession* session = getSessionForStream(stream);
    if (!session) return;
    
    if (!session->active) {
        session->active = true;
        session->startTime = millis();
        
        // Set the appropriate command start time based on stream type
        if (stream == &Serial) {
            serialCommandStartTime = millis();
        }
        else if (stream == &currentClient) {
            networkCommandStartTime = millis();
        }
        
        stream->println(F("[ACTION START]"));
    }
}

// void cm_endSession(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     if (!session || !session->active) return;
    
//     unsigned long duration = millis() - session->startTime;
//     stream->print(F("[ACTION END] Duration: "));
//     stream->print(duration);
//     stream->println(F(" ms"));
//     session->active = false;
// }

// void cm_endSession(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     if (!session || !session->active) return;
    
//     // For network commands, print ACTION END
//     if (stream == &currentClient && networkCommandStartTime > 0) {
//         Serial.print(F("[ACTION END] Duration: "));
//         Serial.print(millis() - networkCommandStartTime);
//         Serial.println(F(" ms"));
//         networkCommandStartTime = 0;
//     }
//     // For serial commands, print ACTION END
//     else if (stream == &Serial && serialCommandStartTime > 0) {
//         unsigned long duration = millis() - serialCommandStartTime;
//         stream->print(F("[ACTION END] Duration: "));
//         stream->print(duration);
//         stream->println(F(" ms"));
//         serialCommandStartTime = 0;
//     }
    
//     // Normal session ending logic
//     session->active = false;
    
//     sendMessage(F("[SESSION ENDED]"), stream, currentClient);
// }

void cm_endSession(Stream* stream) {
    StreamSession* session = getSessionForStream(stream);
    if (!session || !session->active) return;
    
    // For network commands, print ACTION END
    if (stream == &currentClient && networkCommandStartTime > 0) {
        // Print to Serial monitor
        Serial.print(F("[ACTION END] Duration: "));
        Serial.print(millis() - networkCommandStartTime);
        Serial.println(F(" ms"));
        
        // Also send to network client
        if (hasActiveClient && currentClient.connected()) {
            currentClient.print(F("[ACTION END] Duration: "));
            currentClient.print(millis() - networkCommandStartTime);
            currentClient.println(F(" ms"));
        }
        
        networkCommandStartTime = 0;
    }
    // For serial commands, print ACTION END
    else if (stream == &Serial && serialCommandStartTime > 0) {
        unsigned long duration = millis() - serialCommandStartTime;
        stream->print(F("[ACTION END] Duration: "));
        stream->print(duration);
        stream->println(F(" ms"));
        serialCommandStartTime = 0;
    }
    
    // Normal session ending logic
    session->active = false;
    
    sendMessage(F("[SESSION ENDED]"), stream, currentClient);
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

// void cm_commandCompleted(Stream* stream) {
//   StreamSession* session = getSessionForStream(stream);
//   if (!session || !session->active) {  // If session doesn't exist or was aborted, ignore
//       return;
//   }

//   if (pendingCommands > 0) {
//       pendingCommands--;
//       // Use the original message format but with sendMessage for network support
//       sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
//       sendMessage(String(pendingCommands).c_str(), stream, currentClient);
//   }

  
  
//   // Use the same conditions as the original version
//   if (pendingCommands <= 0 && !commandLineBeingProcessed && session->active) {
//       // Only end the session on the stream that started it
//       if ((stream == &Serial && serialSession.active) || 
//           (stream == &currentClient && networkSession.active)) {
//           cm_endSession(stream);
//       }
//   }
// }

// void cm_commandCompleted(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     if (!session || !session->active) {
//         Serial.println(F("[DEBUG-COMPLETE] Command completed called but session inactive"));
//         return;
//     }
    
//     // Log which stream is completing the command
//     Serial.print(F("[DEBUG-COMPLETE] Command completed on "));
//     Serial.println((stream == &Serial) ? F("SERIAL") : F("NETWORK"));
    
//     // Only decrement pending commands ONCE per command 
//     // (from the stream that initiated the session)
//     if ((stream == &Serial && serialSession.active) ||
//         (stream == &currentClient && networkSession.active)) {
//         if (pendingCommands > 0) {
//             pendingCommands--;
//             Serial.print(F("[DEBUG-COMPLETE] Decremented pending commands to: "));
//             Serial.println(pendingCommands);
            
//             // Use the original message format but with sendMessage for network support
//             sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
//             sendMessage(String(pendingCommands).c_str(), stream, currentClient);
//         } else {
//             Serial.println(F("[DEBUG-COMPLETE] Command completed called but pendingCommands already 0"));
//         }
        
//         // Use the same conditions as the original version
//         if (pendingCommands <= 0 && !commandLineBeingProcessed && session->active) {
//             Serial.println(F("[DEBUG-COMPLETE] Conditions met to end session"));
//             cm_endSession(stream);
//         }
//     } else {
//         Serial.println(F("[DEBUG-COMPLETE] Skipped counter decrement - not session stream"));
//     }
//   }



// void cm_commandCompleted(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     bool wasActive = session ? session->active : false;
    
//     if (pendingCommands > 0) {
//       pendingCommands--;
      
//       // Keep this user-facing message
//       sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
//       sendMessage(String(pendingCommands).c_str(), stream, currentClient);
//     }
    
//     // End session if needed
//     if (pendingCommands <= 0 && !commandLineBeingProcessed && wasActive) {
//       // For network commands, print ACTION END when the last command completes
//       if (stream == &currentClient && networkCommandStartTime > 0) {
//         // Print ACTION END to Serial (so it shows in the serial monitor)
//         Serial.print(F("[ACTION END] Duration: "));
//         Serial.print(millis() - networkCommandStartTime);
//         Serial.println(F(" ms"));
        
//         // Also send to network client
//         if (hasActiveClient && currentClient.connected()) {
//           currentClient.print(F("[ACTION END] Duration: "));
//           currentClient.print(millis() - networkCommandStartTime);
//           currentClient.println(F(" ms"));
//         }
        
//         // Reset the start time
//         networkCommandStartTime = 0;
//       }
      
//       cm_endSession(stream);
//     }
//   }

// // Modify cm_commandCompleted to handle both streams
// void cm_commandCompleted(Stream* stream) {
//     StreamSession* session = getSessionForStream(stream);
//     bool wasActive = session ? session->active : false;
    
//     if (pendingCommands > 0) {
//         pendingCommands--;
        
//         // Keep this user-facing message
//         sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
//         sendMessage(String(pendingCommands).c_str(), stream, currentClient);
//     }
    
//     // End session if needed
//     if (pendingCommands <= 0 && !commandLineBeingProcessed && wasActive) {
//         // For network commands, handle ACTION END directly in cm_endSession
//         if (stream == &currentClient) {
//             // No need for additional code here, handled in cm_endSession
//         }
//         // For serial commands, also handled in cm_endSession now
//         else if (stream == &Serial) {
//             // No need for additional code here, handled in cm_endSession
//         }
        
//         cm_endSession(stream);
//     }
// }

void cm_commandCompleted(Stream* stream) {
    StreamSession* session = getSessionForStream(stream);
    bool wasActive = session ? session->active : false;
    
    if (pendingCommands > 0) {
        pendingCommands--;
        
        // Keep this user-facing message
        sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
        sendMessage(String(pendingCommands).c_str(), stream, currentClient);
    }
    
    // End session if needed
    if (pendingCommands <= 0 && !commandLineBeingProcessed && wasActive) {
        cm_endSession(stream);
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
  
  