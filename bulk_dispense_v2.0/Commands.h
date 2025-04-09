#ifndef COMMANDS_H
#define COMMANDS_H

#include "Commander-API.hpp" // Provides CommandCaller and systemCommand_t
#include "Commander-IO.hpp"  // Provides I/O abstractions for the Commander

/************************************************************
 * Commands.cpp
 *
 * This file implements all command functions for the Bulk
 * Dispense system. Each command function processes its
 * arguments, interacts with hardware and sensors, and logs
 * messages accordingly. The global command tree is also defined
 * here.
 *
 * Commands include:
 *   LF      - Set log frequency: LF <ms>
 *   FN      - Fan manual control: FN <0/1> (0 = off, 1 = on)
 *   FNAUTO  - Enable fan auto control
 *   R       - Reagent valve control: R <1-4> <0/1>
 *   M       - Media valve control: M <1-4> <0/1>
 *   W       - Waste valve control: W <1-4> <0/1>
 *   PV      - Set pressure valve: PV <percentage>
 *   CALPV   - Calibrate pressure valve
 *   STARTFSM- Start flow sensor measurement: STARTFSM <1-4>
 *   STOPFSM - Stop flow sensor measurement: STOPFSM <1-4>
 *   RF      - Reset flow sensor dispense volume: RFS <1-4>
 *   RTF     - Reset total volume for flow sensor: RTF <1-4>
 *   RESETI2C- Reset the I2C bus
 *   D       - Dispense reagent: D <1-4> [volume] (continuous if omitted)
 *   STOPD   - Stop dispensing: STOPD <1-4> or STOPD all
 *   P       - Prime valves: P <1-4>
 *   F       - Fill reagent: F <1-4>
 *   DT      - Drain trough: DT <1-4>
 *   SDT     - Stop draining trough: SDT <1-4> or SDT all
 *   SETFS   - Set flow sensor fluid type: SETFS <1-4> <W/I>
 *   SETFSCOR- Set flow sensor correction: SETFSCOR <1-4> <slope> <offset>
 *   ENFSCOR - Enable/disable flow correction: ENFSCOR <1-4> <0/1>
 *   SHOWFSCOR- Show flow correction settings: SHOWFSCOR <1-4>
 *   LOGHELP - Display detailed log field definitions and diagnostic info
 *   STANDBY - Abort all automated operations and set the system to a
 *             safe, idle state (standby mode)
 *   SS      - Display overall system state summary
 *   DI      - Display device network information (Serial only)
 *   help/h/H- Display general command help and usage information
 *
 * Author: Rud Lucien
 * Date: 2025-04-08
 * Version: 2.0
 ************************************************************/

// ============================================================
// Command Function Prototypes
// ============================================================
void cmd_set_log_frequency(char *args, CommandCaller *caller);
void cmd_fan(char *args, CommandCaller *caller);
void cmd_fan_auto(char *args, CommandCaller *caller);
void cmd_set_reagent_valve(char *args, CommandCaller *caller);
void cmd_set_media_valve(char *args, CommandCaller *caller);
void cmd_set_waste_valve(char *args, CommandCaller *caller);
void cmd_set_pressure_valve(char *args, CommandCaller *caller);
void cmd_calibrate_pressure_valve(char *args, CommandCaller *caller);
void cmd_start_flow_sensor_manually(char *args, CommandCaller *caller);
void cmd_stop_flow_sensor_manually(char *args, CommandCaller *caller);
void cmd_reset_flow_dispense(char *args, CommandCaller *caller);
void cmd_reset_flow_total(char *args, CommandCaller *caller);
void cmd_reset_i2c(char *args, CommandCaller *caller);
void cmd_dispense_reagent(char *args, CommandCaller *caller);
void cmd_stop_dispense(char *args, CommandCaller *caller);
void cmd_prime_valves(char *args, CommandCaller *caller);
void cmd_fill_reagent(char *args, CommandCaller *caller);
void cmd_drain_trough(char *args, CommandCaller *caller);
void cmd_stop_drain_trough(char *args, CommandCaller *caller);
void cmd_log_help(char *args, CommandCaller *caller);
void cmd_standby(char *args, CommandCaller *caller);
void cmd_print_help(char *args, CommandCaller *caller);
void cmd_device_info(char *args, CommandCaller *caller);
void cmd_set_flow_sensor_fluid(char *args, CommandCaller *caller);
void cmd_set_flow_sensor_correction(char *args, CommandCaller *caller);
void cmd_enable_flow_sensor_correction(char *args, CommandCaller *caller);
void cmd_show_flow_sensor_correction(char *args, CommandCaller *caller);


// ============================================================
// Global Command Tree and Commander Object
// ============================================================
extern Commander::systemCommand_t API_tree[30];
extern Commander commander;

#endif // COMMANDS_H
