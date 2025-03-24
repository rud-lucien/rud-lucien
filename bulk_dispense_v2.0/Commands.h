#ifndef COMMANDS_H
#define COMMANDS_H

#include "Commander-API.hpp"   // Provides CommandCaller and systemCommand_t
#include "Commander-IO.hpp"    // Provides I/O abstractions for the Commander

/************************************************************
 * Commands.h
 * 
 * This header declares the command functions for the Bulk 
 * Dispense system. It includes prototypes for all commands 
 * (e.g., for setting valves, dispensing, priming, filling, 
 * draining, etc.) and declares the global command tree and 
 * the Commander object.
 *
 * Command List:
 *   LF    - Set log frequency
 *   FN    - Manual fan control (on/off)
 *   FNAUTO- Enable fan auto control
 *   R     - Set reagent valve state (1-4, 0/1)
 *   M     - Set media valve state (1-4, 0/1)
 *   W     - Set waste valve state (1-4, 0/1)
 *   PV    - Set pressure valve (percentage)
 *   CALPV - Calibrate pressure valve
 *   STARTFSM - Start flow sensor measurement (manual)
 *   STOPFSM  - Stop flow sensor measurement (manual)
 *   RFS   - Reset flow sensor dispense volume
 *   RTF   - Reset total volume for a flow sensor
 *   RESETI2C - Reset the I2C bus
 *   D     - Dispense reagent (1-4, optional volume)
 *   STOPD - Stop dispensing (trough or all)
 *   P     - Prime valves (1-4)
 *   F     - Fill reagent (1-4)
 *   DT    - Drain trough (1-4)
 *   SDT   - Stop draining trough (1-4 or all)
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Command Function Prototypes
// ============================================================
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
void cmd_prime_valves(char* args, CommandCaller* caller);
void cmd_fill_reagent(char* args, CommandCaller* caller);
void cmd_drain_trough(char* args, CommandCaller* caller);
void cmd_stop_drain_trough(char* args, CommandCaller* caller);

// ============================================================
// Global Command Tree and Commander Object
// ============================================================
extern Commander::systemCommand_t API_tree[19];
extern Commander commander;

#endif // COMMANDS_H
