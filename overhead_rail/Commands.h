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
#include "Utils.h"
#include "EncoderController.h"
#include "EthernetController.h"
#include "LogHistory.h"
#include "PositionConfig.h" 
#include "RailAutomation.h" 
#include "SystemState.h"    

//=============================================================================
// COMMAND CONSTANTS
//=============================================================================

// Buffer size for command argument processing
#define COMMAND_BUFFER_SIZE 256

// Maximum number of commands (set generously to avoid manual updates)
#define MAX_COMMANDS 20

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

// Global Command Tree and Commander Object
extern Commander::systemCommand_t API_tree[MAX_COMMANDS];
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
// System-Level Commands
// System state, monitoring, and network management
//-----------------------------------------------------------------------------
bool cmd_system(char *args, CommandCaller *caller);
bool cmd_log(char *args, CommandCaller *caller);
bool cmd_network(char *args, CommandCaller *caller);

//-----------------------------------------------------------------------------
// Hardware Control Commands
// Direct control of individual rail systems and manual interfaces
//-----------------------------------------------------------------------------
bool cmd_rail1(char *args, CommandCaller *caller);
bool cmd_rail2(char *args, CommandCaller *caller);
bool cmd_encoder(char *args, CommandCaller *caller);
bool cmd_jog(char *args, CommandCaller *caller);

//-----------------------------------------------------------------------------
// Automation Commands
// High-level coordinated operations and position management
//-----------------------------------------------------------------------------
bool cmd_labware(char *args, CommandCaller *caller);
bool cmd_goto(char *args, CommandCaller *caller);
bool cmd_teach(char *args, CommandCaller *caller);

#endif // COMMANDS_H