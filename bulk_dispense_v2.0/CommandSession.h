#ifndef COMMANDSESSION_H
#define COMMANDSESSION_H

#include <Controllino.h>

/************************************************************
 * CommandSession.h
 *
 * This header declares the functions and global variables
 * used for managing an asynchronous command session in the 
 * Bulk Dispense system.
 *
 * The session management provides:
 *   - Start and end of a command session (marked with action tags).
 *   - Registration and completion tracking for asynchronous commands.
 *   - A helper function to determine if a command should be 
 *     treated as asynchronous.
 *
 * Global Variables:
 *   - commandSessionActive: Indicates whether a session is active.
 *   - commandSessionStartTime: Timestamp for when the session began.
 *   - pendingAsyncCommands: Count of async commands still pending.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ==================== Global Session State ====================
extern bool commandSessionActive;
extern unsigned long commandSessionStartTime;
extern int pendingAsyncCommands;

// ==================== Session Management Function Prototypes ====================
/**
 * startCommandSession()
 * -----------------------
 * Begins a new command session, marking the start with an action tag.
 *
 * @param stream The output stream for logging.
 */
void startCommandSession(Stream* stream);

/**
 * endCommandSession()
 * ---------------------
 * Ends the command session by printing an action end tag with duration,
 * and resets session state.
 *
 * @param stream The output stream for logging.
 */
void endCommandSession(Stream* stream);

/**
 * registerAsyncCommand()
 * ------------------------
 * Registers the start of an asynchronous command.
 */
void registerAsyncCommand();

/**
 * asyncCommandCompleted()
 * -------------------------
 * Signals the completion of an asynchronous command.
 * When no async commands remain, the session is ended.
 *
 * @param stream The output stream for logging.
 */
void asyncCommandCompleted(Stream* stream);

/**
 * isAsyncCommand()
 * ------------------
 * Determines whether a given command string should be treated as
 * asynchronous.
 *
 * @param command The command string.
 * @return true if the command is asynchronous; false otherwise.
 */
bool isAsyncCommand(const char* command);

#endif // COMMANDSESSION_H

