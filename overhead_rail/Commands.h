#ifndef COMMANDS_H
#define COMMANDS_H

#include "ClearCore.h"
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include "MotorController.h"
#include "ValveController.h"
#include "Sensors.h"
#include "LabwareAutomation.h"
// #include "Logging.h" // TODO: need to figure out what I want to log
#include "CommandController.h"
#include "Utils.h"
#include "EncoderController.h"
#include "EthernetController.h"
#include "LogHistory.h"
#include "PositionConfig.h" // Include PositionConfig for position management
#include "RailAutomation.h" // Include RailAutomation for automated rail operations

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
// Motor Control Commands
// Functions for motor movement and configuration
//-----------------------------------------------------------------------------
// bool cmd_motor(char *args, CommandCaller *caller);
// bool cmd_move(char *args, CommandCaller *caller);
// bool cmd_jog(char *args, CommandCaller *caller);

//-----------------------------------------------------------------------------
// Valve Control Commands
// Functions for operating pneumatic valves
//-----------------------------------------------------------------------------
// bool cmd_lock(char *args, CommandCaller *caller);
// bool cmd_unlock(char *args, CommandCaller *caller);

//-----------------------------------------------------------------------------
// Status and Logging Commands
// Functions for reporting system state
//-----------------------------------------------------------------------------
bool cmd_log(char *args, CommandCaller *caller);
bool cmd_labware(char *args, CommandCaller *caller);
bool cmd_goto(char *args, CommandCaller *caller);

//-----------------------------------------------------------------------------
// Rail Control Commands
// Functions for controlling individual rail systems
//-----------------------------------------------------------------------------
bool cmd_rail1(char *args, CommandCaller *caller);
bool cmd_rail2(char *args, CommandCaller *caller);

// Goto command helper functions
bool performGotoPreflightChecks(Location targetLocation, bool hasLabware);

// WC1 location implementations
bool executeWC1WithLabware();
bool executeWC1NoLabware();

// WC2 location implementations
bool executeWC2WithLabware();
bool executeWC2NoLabware();

// WC3 location implementations
bool executeWC3WithLabware();
bool executeWC3NoLabware();

#endif // COMMANDS_H