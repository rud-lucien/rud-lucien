#ifndef COMMANDS_H
#define COMMANDS_H

#include "Commander-API.hpp"  // For CommandCaller and systemCommand_t
#include "Commander-IO.hpp"

// Command function prototypes:
void cmd_set_log_frequency(char* args, CommandCaller* caller);
void cmd_fan(char* args, CommandCaller* caller);
void cmd_fan_auto(char* args, CommandCaller* caller);
void cmd_set_reagent_valve(char* args, CommandCaller* caller);
void cmd_set_media_valve(char* args, CommandCaller* caller);
void cmd_set_waste_valve(char* args, CommandCaller* caller);
void cmd_set_pressure_valve(char* args, CommandCaller* caller);
void cmd_calibrate_pressure_valve(char* args, CommandCaller* caller);
void cmd_start_flow_sensor_manually(char* args, CommandCaller* caller);
void cmd_stop_flow_sensor_manually(char* args, CommandCaller* caller);
void cmd_reset_flow_dispense(char* args, CommandCaller* caller);
void cmd_reset_flow_total(char* args, CommandCaller* caller);
void cmd_reset_i2c(char* args, CommandCaller* caller);
void cmd_dispense_reagent(char* args, CommandCaller* caller);
void cmd_stop_dispense(char* args, CommandCaller* caller);

// Global command tree and Commander object declarations:
extern Commander::systemCommand_t API_tree[15];
extern Commander commander;

#endif // COMMANDS_H
