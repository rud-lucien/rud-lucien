#ifndef COMMANDS_H
#define COMMANDS_H

#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include "MotorController.h"
#include "ValveController.h"
#include "Logging.h"
#include "Tests.h"

// Add this line to reference the variable from lynx_conveyor.ino
extern uint8_t ccioBoardCount; // Number of CCIO-8 boards detected

#define COMMAND_SIZE 9 // Increased from 10

// Command Function Prototypes
bool cmd_print_help(char *args, CommandCaller *caller);

// Motor commands
bool cmd_motor(char *args, CommandCaller *caller);
bool cmd_move(char *args, CommandCaller *caller);

// Valve control commands
bool cmd_lock(char *args, CommandCaller *caller);
bool cmd_unlock(char *args, CommandCaller *caller);

// Status and logging commands
bool cmd_log(char *args, CommandCaller *caller);

// Jog commands
bool cmd_jog(char *args, CommandCaller *caller);

// Helper functions
char *trimLeadingSpaces(char *str);
void handleSerialCommands();

// Global Command Tree and Commander Object
extern Commander::systemCommand_t API_tree[9];
extern const int API_tree_size;
extern Commander commander;

#endif // COMMANDS_H