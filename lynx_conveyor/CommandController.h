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
extern Stream* persistentClient;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================
// Initialization
void initTestFlags();

// Command Processing
bool processCommand(const char* rawCommand, Stream* output);
void handleSerialCommands();
void handleEthernetCommands();

// Command Validation
CommandType getCommandType(const char *command);
bool canExecuteCommand(const char *command);

// Client Management
Stream* getPersistentClient();
void clearPersistentClient();

// Utility Functions
const char *getOperationTypeName(int type);
char *trimLeadingSpaces(char *str);
void sendCommandRejection(const char *command, const char *reason);
void requestTestAbort(const char *source);

#endif // COMMAND_CONTROLLER_H