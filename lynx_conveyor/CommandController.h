#ifndef COMMAND_CONTROLLER_H
#define COMMAND_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include "Commands.h"
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
    CMD_MODIFYING, // Rejected during operations (move, config, etc.)
    CMD_TEST       // Special handling for test commands
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

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Commander API
extern Commander commander;
extern Commander::systemCommand_t API_tree[];
extern const size_t API_tree_size;

// Operation state tracking
extern bool operationInProgress;
extern bool testInProgress;
extern volatile bool testAbortRequested;

// Client tracking for async operations
extern Stream *persistentClient;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================
// Initialization
void initTestFlags();

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
void requestTestAbort(const char *source);
const CommandInfo *findCommand(const char *cmdName);
bool isCommandExcludedFromHistory(const char *command);

#endif // COMMAND_CONTROLLER_H