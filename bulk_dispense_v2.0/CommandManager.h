#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Controllino.h>
#include <Stream.h>
#include "NetworkConfig.h"
#include "Utils.h"


// // Start a new command session.
// void cm_startSession(Stream* stream);

// // End the current command session.
// void cm_endSession(Stream* stream);

// // Register a new asynchronous (or long‐running) command.
// void cm_registerCommand(void);

// // Indicate that an async command has completed.
// void cm_commandCompleted(Stream* stream);

// // Check if a command session is active.
// bool cm_isSessionActive(void);

// int cm_getPendingCommands(void);

// extern volatile bool commandLineBeingProcessed;

// void cm_abortSession(Stream* stream);

// Structure to track session state per stream
struct StreamSession {
    bool active;
    unsigned long startTime;
    Stream* stream;
    int pendingCommands;
};

// Function declarations
void cm_startSession(Stream* stream);
void cm_endSession(Stream* stream);
void cm_registerCommand(void);
void cm_commandCompleted(Stream* stream);
bool cm_isSessionActive(void);
int cm_getPendingCommands(void);
void cm_abortSession(Stream* stream);

extern volatile bool commandLineBeingProcessed;
void resetCommandTimers();

#endif // COMMAND_MANAGER_H
