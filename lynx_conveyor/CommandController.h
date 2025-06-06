#ifndef COMMAND_CONTROLLER_H
#define COMMAND_CONTROLLER_H

//-------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------
#include "Commands.h" 
#include "EthernetController.h"
#include "OutputManager.h"
#include "Utils.h"
#include "MotorController.h"

//-------------------------------------------------------------------------
// Types and Enums
//-------------------------------------------------------------------------
// Command types for operation-in-progress filtering
enum CommandType
{
    CMD_EMERGENCY, // Always allowed (stop, abort, estop)
    CMD_READ_ONLY, // Always allowed (status, get position, etc.)
    CMD_MODIFYING, // Rejected during operations (move, config, etc.)
    CMD_TEST       // Special handling for test commands
};

//-------------------------------------------------------------------------
// External Declarations
//-------------------------------------------------------------------------
extern Commander commander;
extern Commander::systemCommand_t API_tree[];
extern const size_t API_tree_size;
extern bool operationInProgress;

// Global flag to indicate a test is running
extern bool testInProgress;
extern volatile bool testAbortRequested;
extern Stream* persistentClient;
//-------------------------------------------------------------------------
// Core Command Processing Functions
//-------------------------------------------------------------------------
// Initialization
void initTestFlags();
bool processCommand(const char* rawCommand, Stream* output);

// Command Handling Functions
void handleSerialCommands();
void handleEthernetCommands();


// Command Validation Functions
CommandType getCommandType(const char *command);
bool canExecuteCommand(const char *command);

//-------------------------------------------------------------------------
// Utility Functions
//-------------------------------------------------------------------------
const char *getOperationTypeName(int type);
char *trimLeadingSpaces(char *str);
void sendCommandRejection(const char *command, const char *reason);

#endif // COMMAND_CONTROLLER_H