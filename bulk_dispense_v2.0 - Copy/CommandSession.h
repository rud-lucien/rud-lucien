#ifndef COMMANDSESSION_H
#define COMMANDSESSION_H

#include <Controllino.h>
// #include <Stream.h>

// Global session state variables
extern bool commandSessionActive;
extern unsigned long commandSessionStartTime;
extern int pendingAsyncCommands;

// Call this at the beginning of a batch of commands.
void startCommandSession(Stream* stream);

// Call this to finish the session (if no async commands remain).
void endCommandSession(Stream* stream);

// Call this when starting an asynchronous command.
void registerAsyncCommand();

// Call this when an asynchronous command finishes.
void asyncCommandCompleted(Stream* stream);

// Optionally, helper to decide if a command is asynchronous.
bool isAsyncCommand(const char* command);

#endif // COMMANDSESSION_H
