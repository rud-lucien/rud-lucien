#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Controllino.h>
#include <Stream.h>
#include "NetworkConfig.h"
#include "Utils.h"

// Structure to track session state per stream
struct StreamSession
{
    bool active;
    unsigned long startTime;
    Stream *stream;
    int pendingCommands;
};

// Function declarations
void cm_startSession(Stream *stream);
void cm_endSession(Stream *stream);
void cm_registerCommand(void);
void cm_commandCompleted(Stream *stream);
bool cm_isSessionActive(void);
int cm_getPendingCommands(void);
void cm_abortSession(Stream *stream);

extern volatile bool commandLineBeingProcessed;
void resetCommandTimers();

#endif // COMMAND_MANAGER_H
