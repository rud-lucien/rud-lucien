#ifndef COMMAND_CONTROLLER_H
#define COMMAND_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <Arduino.h>
#include "EthernetController.h"
#include "OutputManager.h"
#include "Utils.h"
#include "MotorController.h"

//=============================================================================
// TYPES AND CONSTANTS
//=============================================================================
// Command types for operation-in-progress filtering
enum CommandType
{
    CMD_EMERGENCY, // Always allowed (stop, abort, estop)
    CMD_READ_ONLY, // Always allowed (status, get position, etc.)
    CMD_MODIFYING  // Rejected during operations (move, config, etc.)
};

// Command lookup table structure
struct CommandInfo
{
    const char *name;
    CommandType type;
    uint8_t flags; // Bit flags for command properties
};

// Command flags
#define CMD_FLAG_ASYNC 0x01      // Command is asynchronous
#define CMD_FLAG_NO_HISTORY 0x02 // Command should not be logged to history

// Maximum command buffer size
#define MAX_COMMAND_LENGTH 64

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Operation state tracking
extern bool operationInProgress;

// Command tracking for system state reporting
extern char lastExecutedCommand[MAX_COMMAND_LENGTH];
extern unsigned long lastCommandTime;
extern bool lastCommandSuccess;
extern CommandType lastCommandType;
extern char lastCommandSource[16]; // "SERIAL" or "NETWORK"
extern unsigned long systemStartTime;

// Client tracking for async operations
extern Stream *persistentClient;

// Command processing buffers
extern char serialCommandBuffer[MAX_COMMAND_LENGTH];
extern char ethernetCommandBuffer[MAX_COMMAND_LENGTH];

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// Command Processing
bool processCommand(const char *rawCommand, Stream *output, const char *sourceTag = nullptr);
void handleSerialCommands();
void handleEthernetCommands();

// Command Validation
CommandType getCommandType(const char *command);
bool canExecuteCommand(const char *command);

// Client Management
Stream *getPersistentClient();
void clearPersistentClient();

// Utility Functions
const char *getOperationTypeName(int type);
char *trimLeadingSpaces(char *str);
void sendCommandRejection(const char *command, const char *reason);
const CommandInfo *findCommand(const char *cmdName);
bool isCommandExcludedFromHistory(const char *command);

// Command tracking functions
const char *getLastCommandStatus();
void initializeSystemStartTime();

// Command execution functions (to be implemented when Commands.h is ready)
bool executeCommand(const char *command, Stream *output);

#endif // COMMAND_CONTROLLER_H
