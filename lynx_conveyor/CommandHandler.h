#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "Commands.h" // For Commander class
#include "OutputManager.h"
#include "Utils.h"
#include "MotorController.h"

// Command types for operation-in-progress filtering
enum CommandType
{
    CMD_EMERGENCY, // Always allowed (stop, abort, estop)
    CMD_READ_ONLY, // Always allowed (status, get position, etc.)
    CMD_MODIFYING, // Rejected during operations (move, config, etc.)
    CMD_TEST       // Special handling for test commands
};

// External declarations
extern Commander commander;
extern Commander::systemCommand_t API_tree[];
extern bool operationInProgress;

// Global flag to indicate a test is running
extern bool testInProgress;
extern bool testAbortRequested;

// Function declarations
void initCommandHandler();
void handleSerialCommands();
CommandType getCommandType(const char *command);
bool canExecuteCommand(const char *command);
const char *getOperationTypeName(int type);
char *trimLeadingSpaces(char *str);
void sendCommandRejection(const char *command, const char *reason);

#endif // COMMAND_HANDLER_H