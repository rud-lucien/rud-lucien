#ifndef COMMANDS_H
#define COMMANDS_H

#include "ClearCore.h"
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include "MotorController.h"
#include "ValveController.h"
#include "Sensors.h"
#include "LabwareAutomation.h"
#include "Logging.h"
#include "CommandController.h"
#include "Utils.h"
#include "EncoderController.h"
#include "EthernetController.h"
#include "LogHistory.h"
#include "PositionConfig.h" // Include PositionConfig for position management
#include "RailAutomation.h" // Include RailAutomation for automated rail operations
#include "SystemState.h"    // Include SystemState for system status reporting

//=============================================================================
// COMMAND CONSTANTS
//=============================================================================

// Command tree size
#define COMMAND_SIZE 15

// Structure for subcommand lookup
struct SubcommandInfo
{
    const char *name; // Subcommand name
    int code;         // Command code for switch statement
};

int findSubcommandCode(const char *subcommand, const SubcommandInfo *commandTable, size_t tableSize);

//=============================================================================
// EXTERNAL REFERENCES
//=============================================================================

// Reference to the variable from lynx_conveyor.ino
extern uint8_t ccioBoardCount; // Number of CCIO-8 boards detected

// Global Command Tree and Commander Object
extern Commander::systemCommand_t API_tree[COMMAND_SIZE];
extern const size_t API_tree_size;
extern Commander commander;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// Core Command Functions
// Top-level command handlers
//-----------------------------------------------------------------------------
bool cmd_print_help(char *args, CommandCaller *caller);


//-----------------------------------------------------------------------------
// Status and Logging Commands
// Functions for reporting system state
//-----------------------------------------------------------------------------
bool cmd_log(char *args, CommandCaller *caller);
bool cmd_system(char *args, CommandCaller *caller);
bool cmd_labware(char *args, CommandCaller *caller);
bool cmd_goto(char *args, CommandCaller *caller);
bool cmd_network(char *args, CommandCaller *caller);
bool cmd_encoder(char *args, CommandCaller *caller);
bool cmd_jog(char *args, CommandCaller *caller);
bool cmd_teach(char *args, CommandCaller *caller);

//-----------------------------------------------------------------------------
// Rail Control Commands
// Functions for controlling individual rail systems
//-----------------------------------------------------------------------------
bool cmd_rail1(char *args, CommandCaller *caller);
bool cmd_rail2(char *args, CommandCaller *caller);

#endif // COMMANDS_H