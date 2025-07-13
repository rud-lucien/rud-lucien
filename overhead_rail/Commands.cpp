#include "Commands.h"
#include "Logging.h"
#include "RailAutomation.h"
#include "HandoffController.h"
#include "LabwareAutomation.h"

// ============================================================
// Binary search function for subcommand lookup
// ============================================================
int findSubcommandCode(const char *subcommand, const SubcommandInfo *commandTable, size_t tableSize)
{
    int left = 0;
    int right = tableSize - 1;

    while (left <= right)
    {
        int mid = left + (right - left) / 2;
        int cmp = strcmp(subcommand, commandTable[mid].name);

        if (cmp == 0)
            return commandTable[mid].code; // Found - return the code

        if (cmp < 0)
            right = mid - 1;
        else
            left = mid + 1;
    }

    return 0; // Not found - return 0 (default case)
}

// ============================================================
// Global Command Tree and Commander Object
// ============================================================

bool cmd_print_help(char *args, CommandCaller *caller)
{
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    char *trimmed = trimLeadingSpaces(localArgs);

    // If the user requested help for a specific command,
    // we currently do not have detailed help implemented.
    if (strlen(trimmed) > 0)
    {
        Console.error(F("Detailed help for specific commands is not implemented."));
        return false;
    }
    else
    {
        // No specific command requested; print general help.
        // Pass a pointer to the Serial stream.

        // Print general help information.
        Console.println(F("--------------------------------------------------"));
        Console.println(F("Overhead Rail System Command Help:"));
        Console.println(F("--------------------------------------------------"));

        commander.printHelp(caller, true, true);

        Console.println(F("--------------------------------------------------"));
        return true;
    }
}

Commander commander;

Commander::systemCommand_t API_tree[] = {
    systemCommand("help", "Display help information for all commands", cmd_print_help),
    systemCommand("h", "Display help information for all commands", cmd_print_help),
    systemCommand("H", "Display help information for all commands", cmd_print_help),

    // Logging command
    systemCommand("log", "Logging controls and history:\r\n"
                         "  log on [interval] - Enable periodic logging (interval in ms, default 250)\r\n"
                         "  log off           - Disable periodic logging\r\n"
                         "  log now           - Log system state immediately\r\n"
                         "  log history       - Show complete operation log history\r\n"
                         "  log errors        - Show only errors and warnings for quick debugging\r\n"
                         "  log last [count]  - Show last N log entries (default: 10)\r\n"
                         "  log stats         - Show log buffer statistics and overflow info\r\n"
                         "  log help          - Display detailed logging information",
                  cmd_log),

    // Labware automation command
    systemCommand("labware", "Labware automation and state management:\r\n"
                            "  labware status      - Display current labware tracking state\r\n"
                            "  labware audit       - Automatically validate and fix labware state\r\n"
                            "  labware reset       - Clear all labware tracking (nuclear option)\r\n"
                            "  labware help        - Display detailed labware automation instructions",
                  cmd_labware),

    // Automated labware movement command
    systemCommand("goto", "Automated work cell movement with labware tracking:\r\n"
                         "  goto <location> <status>  - Move to work cell with labware status\r\n"
                         "  Locations: wc1, wc2, wc3\r\n"
                         "  Status: with-labware, no-labware\r\n"
                         "  Examples:\r\n"
                         "    goto wc1 with-labware   - Move to WC1 with labware\r\n"
                         "    goto wc2 no-labware     - Move to WC2 without labware\r\n"
                         "    goto wc3 with-labware   - Move to WC3 with labware",
                  cmd_goto),

    // State command to display system state
    // systemCommand("system", "System commands:\r\n"
    //                         "  system,state    - Display current system state (sensors, actuators, positions)\r\n"
    //                         "  system,safety   - Display comprehensive safety validation status\r\n"
    //                         "  system,trays    - Display tray tracking and statistics\r\n"
    //                         "  system,reset    - Reset system state after failure to retry operation\r\n"
    //                         "  system,help     - Display detailed instructions for system commands\r\n"
    //                         "                    (Use 'log,history' or 'log,errors' for operation troubleshooting)",
    //               cmd_system_state),


    // Jog command
    // systemCommand("jog", "Jog motor:\r\n"
    //                      "  jog,+         - Jog forward by current increment\r\n"
    //                      "  jog,-         - Jog backward by current increment\r\n"
    //                      "  jog,inc,X     - Get or set jog increment (X in mm or 'default')\r\n"
    //                      "  jog,speed,X   - Get or set jog speed (X in RPM or 'default')\r\n"
    //                      "  jog,status    - Display current jog settings\r\n"
    //                      "  jog,help      - Display usage instructions and comparison with handwheel",
    //               cmd_jog),


    // Encoder control commands
    // systemCommand("encoder", "Encoder handwheel control:\r\n"
    //                          "  encoder,enable  - Enable encoder control\r\n"
    //                          "  encoder,disable - Disable encoder control\r\n"
    //                          "  encoder,multiplier,X - Set encoder multiplier (X = 1, 10, or 100)\r\n"
    //                          "  encoder,help    - Display setup instructions and usage tips",
    //               cmd_encoder),

    // Abort command
    // systemCommand("abort", "Abort any running test", cmd_abort),

    // Network management command
    // systemCommand("network", "Network management:\r\n"
    //                          "  network,status   - Display current network status and connected clients\r\n"
    //                          "  network,close,X  - Close a specific client connection (X = client number)\r\n"
    //                          "  network,closeall - Close all client connections\r\n"
    //                          "  network,help     - Display detailed network management instructions",
    //               cmd_network),

    // Teach position command
    // systemCommand("teach", "Teach position commands:\r\n"
    //                        "  teach,1   - Teach position 1 (loading position)\r\n"
    //                        "  teach,2   - Teach position 2 (middle position)\r\n"
    //                        "  teach,3   - Teach position 3 (unloading position)\r\n"
    //                        "  teach,reset - Reset all positions to factory defaults\r\n"
    //                        "  teach,status - Display current taught positions and SD card status\r\n"
    //                        "  teach,help   - Display detailed usage instructions",
    //               cmd_teach),

    // Rail 1 control command
    systemCommand("rail1", "Rail 1 Control Commands:\r\n"
                          "  rail1 init          - Initialize Rail 1 motor system\r\n"
                          "  rail1 clear-fault   - Clear motor fault condition\r\n"
                          "  rail1 abort         - Abort current operation gracefully\r\n"
                          "  rail1 stop          - Emergency stop motor movement\r\n"
                          "  rail1 home          - Home carriage to reference position\r\n"
                          "  rail1 move-wc1 no-labware     - Move empty carriage to WC1\r\n"
                          "  rail1 move-wc1 with-labware   - Move carriage with labware to WC1\r\n"
                          "  rail1 move-wc2 no-labware     - Move empty carriage to WC2\r\n"
                          "  rail1 move-wc2 with-labware   - Move carriage with labware to WC2\r\n"
                          "  rail1 move-staging no-labware     - Move empty carriage to staging position\r\n"
                          "  rail1 move-staging with-labware   - Move carriage with labware to staging position\r\n"
                          "  rail1 move-handoff no-labware - Move empty carriage to handoff\r\n"
                          "  rail1 move-handoff with-labware - Move carriage with labware to handoff\r\n"
                          "  rail1 move-mm-to X no-labware - Move empty carriage to absolute position X mm\r\n"
                          "  rail1 move-mm-to X with-labware - Move carriage with labware to absolute position X mm\r\n"
                          "  rail1 move-rel X no-labware   - Move empty carriage X mm relative to current position\r\n"
                          "  rail1 move-rel X with-labware - Move carriage with labware X mm relative to current position\r\n"
                          "  rail1 status        - Show comprehensive system status and diagnostics\r\n"
                          "  rail1 help          - Display detailed usage instructions",
                  cmd_rail1),

    // Rail 2 control command
    systemCommand("rail2", "Rail 2 Control Commands:\r\n"
                          "  rail2 init          - Initialize Rail 2 motor system\r\n"
                          "  rail2 clear-fault   - Clear motor fault condition\r\n"
                          "  rail2 abort         - Abort current operation gracefully\r\n"
                          "  rail2 stop          - Emergency stop motor movement\r\n"
                          "  rail2 extend        - Extend pneumatic drive\r\n"
                          "  rail2 retract       - Retract pneumatic drive\r\n"
                          "  rail2 home          - Home carriage to reference position\r\n"
                          "  rail2 move-wc3 no-labware     - Move empty carriage to WC3\r\n"
                          "  rail2 move-wc3 with-labware   - Move carriage with labware to WC3\r\n"
                          "  rail2 move-handoff no-labware - Move empty carriage to handoff\r\n"
                          "  rail2 move-handoff with-labware - Move carriage with labware to handoff\r\n"
                          "  rail2 move-mm-to X no-labware - Move empty carriage to absolute position X mm\r\n"
                          "  rail2 move-mm-to X with-labware - Move carriage with labware to absolute position X mm\r\n"
                          "  rail2 move-rel X no-labware   - Move empty carriage X mm relative to current position\r\n"
                          "  rail2 move-rel X with-labware - Move carriage with labware X mm relative to current position\r\n"
                          "  rail2 status        - Show comprehensive system status and diagnostics\r\n"
                          "  rail2 help          - Display detailed usage instructions\r\n"
                          "  SAFETY: Cylinder auto-retracts for ANY movement involving collision zone (500-700mm)",
                  cmd_rail2),
};

const size_t API_tree_size = sizeof(API_tree) / sizeof(Commander::systemCommand_t);

// ============================================================
// Log Command Implementation
// ============================================================

bool cmd_log(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';
    
    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Check for empty argument
    if (strlen(trimmed) == 0) {
        Console.error(F("Missing parameter. Usage: log <action>"));
        return false;
    }
    
    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");
    
    if (action == NULL) {
        Console.error(F("Invalid format. Usage: log <action>"));
        return false;
    }
    
    // Trim leading spaces from action
    action = trimLeadingSpaces(action);
    
    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++) {
        action[i] = tolower(action[i]);
    }
    
    // Handle different log commands
    if (strcmp(action, "on") == 0) {
        // Enable periodic logging
        unsigned long interval = DEFAULT_LOG_INTERVAL;
        
        if (param1 != NULL) {
            interval = atol(param1);
            if (interval < 100) {
                Console.error(F("LOG_INTERVAL_TOO_SMALL: Minimum logging interval is 100ms"));
                return false;
            }
            if (interval > 60000) {
                Console.error(F("LOG_INTERVAL_TOO_LARGE: Maximum logging interval is 60000ms (1 minute)"));
                return false;
            }
        }
        
        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset timer
        
        Console.acknowledge((String(F("PERIODIC_LOGGING_ENABLED: Interval set to ")) + String(interval) + F("ms")).c_str());
        return true;
    }
    else if (strcmp(action, "off") == 0) {
        // Disable periodic logging
        logging.logInterval = 0;
        
        Console.acknowledge(F("PERIODIC_LOGGING_DISABLED: No automatic logging"));
        return true;
    }
    else if (strcmp(action, "now") == 0) {
        // Log system state immediately
        Console.acknowledge(F("SYSTEM_STATE_LOGGED: Current system state captured"));
        logSystemState();
        return true;
    }
    else if (strcmp(action, "history") == 0) {
        // Show complete operation log history
        Console.acknowledge(F("DISPLAYING_LOG_HISTORY: Complete operation log follows:"));
        opLogHistory.printHistory();
        return true;
    }
    else if (strcmp(action, "errors") == 0) {
        // Show only errors and warnings
        Console.acknowledge(F("DISPLAYING_ERROR_LOG: Error and warning entries follow:"));
        opLogHistory.printErrors();
        return true;
    }
    else if (strcmp(action, "last") == 0) {
        // Show last N log entries (default: 10)
        uint8_t count = 10;
        
        if (param1 != NULL) {
            count = atoi(param1);
            if (count > 50) {
                count = 50; // Limit to history size
            }
            if (count < 1) {
                count = 1;
            }
        }
        
        Console.acknowledge((String(F("DISPLAYING_LOG_LAST: Last ")) + String(count) + F(" log entries follow:")).c_str());
        opLogHistory.printLastN(count);
        return true;
    }
    else if (strcmp(action, "stats") == 0) {
        // Show log buffer statistics
        Console.acknowledge(F("DISPLAYING_LOG_STATS: Buffer statistics and status follow:"));
        opLogHistory.printStats();
        
        // Also show current logging status
        Console.serialInfo(F("Current logging status:"));
        if (logging.logInterval > 0) {
            Console.serialInfo((String(F("  Periodic logging: ENABLED (")) + String(logging.logInterval) + F("ms interval)")).c_str());
        } else {
            Console.serialInfo(F("  Periodic logging: DISABLED"));
        }
        return true;
    }
    else if (strcmp(action, "help") == 0) {
        // Display detailed help information
        Console.acknowledge(F("DISPLAYING_LOG_HELP: Logging system guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("Logging System Commands"));
        Console.println(F("============================================"));
        Console.println(F("PERIODIC LOGGING:"));
        Console.println(F("  log on [interval]   - Enable periodic system state logging"));
        Console.println(F("                        interval: 100-60000ms (default: 250ms)"));
        Console.println(F("  log off             - Disable periodic logging"));
        Console.println(F(""));
        Console.println(F("IMMEDIATE LOGGING:"));
        Console.println(F("  log now             - Log current system state immediately"));
        Console.println(F(""));
        Console.println(F("LOG HISTORY REVIEW:"));
        Console.println(F("  log history         - Show complete operation log history"));
        Console.println(F("  log errors          - Show only errors and warnings"));
        Console.println(F("  log last [count]    - Show last N entries (default: 10, max: 50)"));
        Console.println(F(""));
        Console.println(F("DIAGNOSTICS:"));
        Console.println(F("  log stats           - Show log buffer statistics and current status"));
        Console.println(F(""));
        Console.println(F("LOGGED INFORMATION:"));
        Console.println(F("- Valve states and sensor feedback"));
        Console.println(F("- All carriage and labware sensors"));
        Console.println(F("- Motor states, homing status, HLFB for both rails"));
        Console.println(F("- Position, target, velocity for both rails"));
        Console.println(F("- E-stop status and air pressure"));
        Console.println(F("- Network client connections"));
        Console.println(F("- MPG (encoder) status and active rail"));
        Console.println(F(""));
        Console.println(F("COLOR CODING:"));
        Console.println(F("- Green: Normal/good states (homed, sensors active, etc.)"));
        Console.println(F("- Yellow: Active states (moving, extended, not homed)"));
        Console.println(F("- Red: Problem states (faults, low pressure, E-stop)"));
        Console.println(F("- Cyan: Section headers for easy scanning"));
        Console.println(F("============================================"));
        return true;
    }
    else {
        Console.error(F("Unknown log command. Available: on, off, now, history, errors, last, stats, help"));
        return false;
    }
}

// ============================================================
// Rail 2 Control Commands
// ============================================================

// Define the rail2 subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo RAIL2_COMMANDS[] = {
    {"abort", 0},
    {"clear-fault", 1},
    {"extend", 2},
    {"help", 3},
    {"home", 4},
    {"init", 5},
    {"move-handoff", 6},
    {"move-mm-to", 7},
    {"move-rel", 8},
    {"move-wc3", 9},
    {"retract", 10},
    {"status", 11},
    {"stop", 12}
};

static const size_t RAIL2_COMMAND_COUNT = sizeof(RAIL2_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_rail2(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';
    
    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Check for empty argument
    if (strlen(trimmed) == 0) {
        Console.error(F("Missing parameter. Usage: rail2 <action>"));
        return false;
    }
    
    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");
    char *param2 = strtok(nullptr, " ");
    
    if (action == NULL) {
        Console.error(F("Invalid format. Usage: rail2 <action>"));
        return false;
    }
    
    // Trim leading spaces from action
    action = trimLeadingSpaces(action);
    
    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++) {
        action[i] = tolower(action[i]);
    }
    
    // Declare all variables at the beginning before switch
    ValveOperationResult result;
    double currentPos = 0.0;
    double targetPosition = 0.0;
    double calculatedTargetPos = 0.0;
    PositionTarget targetPos;
    bool carriageLoaded = false;
    bool movementInCollisionZone = false;
    ValvePosition valveState;
    bool sensorRetracted = false;
    bool sensorExtended = false;
    bool validationResult = false;
    bool actuallyRetracted = false;
    bool actuallyExtended = false;
    String motorStatus; // For status consolidation
    
    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, RAIL2_COMMANDS, RAIL2_COMMAND_COUNT);
    
    // Use switch-case for cleaner flow control
    switch (cmdCode) {
    
    case 0: // "abort" - Abort current operation gracefully
        return executeRailAbort(2);
        
    case 1: // "clear-fault" - Clear motor fault condition
        return executeRailClearFault(2);
        
    case 2: // "extend" - Extend pneumatic drive
        if (!isPressureSufficient()) {
            Console.error(F("INSUFFICIENT_PRESSURE: Air pressure too low for valve operation"));
            return false;
        }
        
        Console.serialInfo(F("Extending pneumatic drive..."));
        result = extendCylinder();
        
        if (result == VALVE_OP_SUCCESS) {
            Console.acknowledge(F("CYLINDER_EXTENDED: Pneumatic drive is now extended"));
            return true;
        } else {
            Console.error((String(F("EXTEND_FAILED: ")) + getValveOperationResultName(result)).c_str());
            return false;
        }
        
    case 3: // "help" - Display help information
        Console.acknowledge(F("DISPLAYING_RAIL2_HELP: Command reference follows:"));
        Console.println(F("============================================"));
        Console.println(F("Rail 2 Control Commands"));
        Console.println(F("============================================"));
        Console.println(F("MOTOR CONTROL:"));
        Console.println(F("  rail2 init          - Initialize Rail 2 motor system"));
        Console.println(F("  rail2 clear-fault   - Clear motor fault condition"));
        Console.println(F("  rail2 abort         - Abort current operation gracefully"));
        Console.println(F("  rail2 stop          - Emergency stop motor movement"));
        Console.println(F(""));
        Console.println(F("PNEUMATIC DRIVE CONTROL:"));
        Console.println(F("  rail2 extend        - Extend pneumatic drive"));
        Console.println(F("  rail2 retract       - Retract pneumatic drive"));
        Console.println(F(""));
        Console.println(F("HOMING OPERATION:"));
        Console.println(F("  rail2 home          - Home carriage (find WC3 position)"));
        Console.println(F(""));
        Console.println(F("CARRIAGE MOVEMENT:"));
        Console.println(F("  rail2 move-wc3 no-labware     - Move empty carriage to WC3"));
        Console.println(F("  rail2 move-handoff no-labware - Move empty carriage to handoff"));
        Console.println(F("  rail2 move-wc3 with-labware   - Move carriage with labware to WC3"));
        Console.println(F("  rail2 move-handoff with-labware - Move carriage with labware to handoff"));
        Console.println(F(""));
        Console.println(F("MANUAL POSITIONING:"));
        Console.print(F("  rail2 move-mm-to X no-labware   - Move empty carriage to absolute position X mm (0-"));
        Console.print(RAIL2_MAX_TRAVEL_MM);
        Console.println(F(")"));
        Console.println(F("  rail2 move-mm-to X with-labware - Move carriage with labware to absolute position X mm"));
        Console.println(F("  rail2 move-rel X no-labware     - Move empty carriage X mm relative (+ forward, - backward)"));
        Console.println(F("  rail2 move-rel X with-labware   - Move carriage with labware X mm relative"));
        Console.println(F(""));
        Console.println(F("STATUS AND DIAGNOSTICS:"));
        Console.println(F("  rail2 status        - Show comprehensive system status"));
        Console.println(F(""));
        Console.println(F("SAFETY NOTES:"));
        Console.println(F("- Always specify labware status for movement commands"));
        Console.println(F("- Home carriage before first use"));
        Console.println(F("- Check sensors before movement operations"));
        Console.println(F("- Ensure sufficient air pressure for pneumatic operations"));
        Console.println(F("- CRITICAL: Cylinder automatically retracts during ANY movement that"));
        Console.println(F("  involves collision zone (500-700mm), including"));
        Console.println(F("  crossing, entering, exiting, or moving within the zone to prevent Rail 1 collision"));
        Console.println(F("- Manual extension required after reaching safe positions"));
        Console.println(F("- Safe zones: 0-499mm and 701-1000mm (cylinder can remain extended)"));
        Console.println(F("============================================"));
        return true;
        
    case 5: // "init" - Initialize Rail 2 motor system
        return executeRailInit(2);
        
    case 10: // "retract" - Retract pneumatic drive
        if (!isPressureSufficient()) {
            Console.error(F("INSUFFICIENT_PRESSURE: Air pressure too low for valve operation"));
            return false;
        }
        
        Console.serialInfo(F("Retracting pneumatic drive..."));
        result = retractCylinder();
        
        if (result == VALVE_OP_SUCCESS) {
            Console.acknowledge(F("CYLINDER_RETRACTED: Pneumatic drive is now retracted"));
            return true;
        } else {
            Console.error((String(F("RETRACT_FAILED: ")) + getValveOperationResultName(result)).c_str());
            return false;
        }
        
    case 4: // "home" - Home carriage
        return executeRailHome(2);
        
    case 7: // "move-mm-to" - Move to absolute millimeter position
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail2 move-mm-to <position_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the position value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        
        // Execute the movement using helper function
        return executeRailMoveToPosition(2, targetPosition, carriageLoaded);
        
    case 8: // "move-rel" - Move relative distance
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail2 move-rel <distance_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the relative distance value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        
        // Execute the movement using helper function
        return executeRailMoveRelative(2, targetPosition, carriageLoaded);
        
    case 9: // "move-wc3" - Move carriage to WC3
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded)) return false;
        
        // Use Rail 2-specific helper function
        return moveRail2CarriageToWC3(carriageLoaded);
        
    case 6: // "move-handoff" - Move carriage to handoff
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded)) return false;
        
        // Use Rail 2-specific helper function
        return moveRail2CarriageToHandoff(carriageLoaded);
        
    case 11: // "status" - Show system status
        Console.acknowledge(F("DISPLAYING_RAIL2_STATUS: Comprehensive system diagnostics follow:"));
        Console.serialInfo(F("============================================"));
        Console.serialInfo(F("Rail 2 System Status"));
        Console.serialInfo(F("============================================"));
        
        // Safety status
        Console.serialInfo(F("SAFETY STATUS:"));
        if (isEStopActive()) {
            Console.serialInfo(F("  E-Stop Status: ACTIVE (UNSAFE)"));
        } else {
            Console.serialInfo(F("  E-Stop Status: INACTIVE (Safe)"));
        }
        
        // Motor status
        Console.serialInfo(F("MOTOR STATUS:"));
        Console.serialInfo(isMotorReady(2) ? F("  Motor Ready: YES") : F("  Motor Ready: NO"));
        Console.serialInfo(isHomingComplete(2) ? F("  Motor Homed: YES") : F("  Motor Homed: NO"));
        Console.serialInfo(isMotorMoving(2) ? F("  Motor Moving: YES") : F("  Motor Moving: NO"));
        Console.serialInfo(isHomingInProgress(2) ? F("  Motor Homing: YES") : F("  Motor Homing: NO"));
        
        // Current position
        Console.serialInfo(F("CURRENT POSITION:"));
        if (!isHomingComplete(2)) {
            Console.serialInfo(F("  Position: UNKNOWN (not homed) - Use 'rail2 home' first"));
        } else {
            currentPos = getMotorPositionMm(2);
            Console.serialInfo((String(F("  Position: ")) + String(currentPos, 2) + F("mm")).c_str());
            
            if (isCarriageAtWC3()) {
                Console.serialInfo(F("  Location: AT WC3"));
            } else if (isCarriageAtRail2Handoff()) {
                Console.serialInfo(F("  Location: AT HANDOFF"));
            } else if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) {
                Console.serialInfo(F("  Location: IN COLLISION ZONE"));
            } else {
                Console.serialInfo(F("  Location: BETWEEN POSITIONS"));
            }
        }
        
        // Pneumatic system
        Console.serialInfo(F("PNEUMATIC SYSTEM:"));
        Console.serialInfo((String(F("  Air Pressure: ")) + String(getPressurePsi(), 1) + F(" PSI")).c_str());
        Console.serialInfo(isPressureSufficient() ? F("  Pressure Status: OK") : F("  Pressure Status: LOW"));
        valveState = getValvePosition();
        Console.serialInfo((String(F("  Valve Position: ")) + getValvePositionName(valveState)).c_str());
        
        // Cylinder sensors
        Console.serialInfo(F("CYLINDER SENSORS:"));
        sensorRetracted = isCylinderRetracted();
        sensorExtended = isCylinderExtended();
        Console.serialInfo(sensorRetracted ? F("  Retracted Sensor: ACTIVE") : F("  Retracted Sensor: INACTIVE"));
        Console.serialInfo(sensorExtended ? F("  Extended Sensor: ACTIVE") : F("  Extended Sensor: INACTIVE"));
        validationResult = validateValvePosition();
        Console.serialInfo(validationResult ? F("  Sensor Validation: PASS") : F("  Sensor Validation: FAIL"));
        
        // Position detection
        Console.serialInfo(F("POSITION DETECTION:"));
        Console.serialInfo(isCarriageAtWC3() ? F("  WC3 Detection: YES") : F("  WC3 Detection: NO"));
        Console.serialInfo(isCarriageAtRail2Handoff() ? F("  Handoff Detection: YES") : F("  Handoff Detection: NO"));
        
        // Labware detection
        Console.serialInfo(F("LABWARE DETECTION:"));
        Console.serialInfo(isLabwarePresentAtWC3() ? F("  WC3 Labware Present: YES") : F("  WC3 Labware Present: NO"));
        Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff Labware Present: YES") : F("  Handoff Labware Present: NO"));
        
        // Collision zone analysis
        Console.serialInfo(F("COLLISION ZONE ANALYSIS:"));
        if (isHomingComplete(2)) {
            bool inCollisionZone = (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END);
            actuallyRetracted = isCylinderActuallyRetracted();
            actuallyExtended = isCylinderActuallyExtended();
            
            if (inCollisionZone) {
                Console.serialInfo(F("  Current Zone: COLLISION"));
                if (actuallyRetracted) {
                    Console.serialInfo(F("  Collision Status: SAFE (cylinder retracted)"));
                } else {
                    Console.serialInfo(F("  Collision Status: UNSAFE (cylinder extended in collision zone)"));
                }
            } else {
                Console.serialInfo(F("  Current Zone: SAFE"));
                Console.serialInfo(F("  Collision Status: SAFE"));
            }
        } else {
            Console.serialInfo(F("  Current Zone: UNKNOWN (motor not homed)"));
            Console.serialInfo(F("  Collision Status: UNKNOWN"));
        }
        
        Console.serialInfo(F("============================================"));
        return true;
        
    case 12: // "stop" - Emergency stop motor movement
        return executeRailStop(2);
        
    default: // Unknown command
        Console.error(F("Unknown action. Available: init, clear-fault, abort, stop, extend, retract, home, move-wc3, move-handoff, move-mm-to, move-rel, status, and help"));
        return false;
    }
    
    return false; // Should never reach here
}

// ============================================================
// Rail 1 Control Commands
// ============================================================

// Define the rail1 subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo RAIL1_COMMANDS[] = {
    {"abort", 0},
    {"clear-fault", 1},
    {"help", 2},
    {"home", 3},
    {"init", 4},
    {"move-handoff", 5},
    {"move-mm-to", 6},
    {"move-rel", 7},
    {"move-staging", 8},
    {"move-wc1", 9},
    {"move-wc2", 10},
    {"status", 11},
    {"stop", 12}
};

static const size_t RAIL1_COMMAND_COUNT = sizeof(RAIL1_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_rail1(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';
    
    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Check for empty argument
    if (strlen(trimmed) == 0) {
        Console.error(F("Missing parameter. Usage: rail1 <action>"));
        return false;
    }
    
    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");
    char *param2 = strtok(nullptr, " ");
    
    if (action == NULL) {
        Console.error(F("Invalid format. Usage: rail1 <action>"));
        return false;
    }
    
    // Trim leading spaces from action
    action = trimLeadingSpaces(action);
    
    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++) {
        action[i] = tolower(action[i]);
    }
    
    // Declare all variables at the beginning before switch
    double currentPos = 0.0;
    double targetPosition = 0.0;
    double calculatedTargetPos = 0.0;
    bool carriageLoaded = false;
    String motorStatus; // For status consolidation
    String location; // For location description
    
    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, RAIL1_COMMANDS, RAIL1_COMMAND_COUNT);
    
    // Use switch-case for cleaner flow control
    switch (cmdCode) {
    
    case 0: // "abort" - Abort current operation gracefully
        return executeRailAbort(1);
        
    case 1: // "clear-fault" - Clear motor fault condition
        return executeRailClearFault(1);
        
    case 2: // "help" - Display help information
        Console.acknowledge(F("DISPLAYING_RAIL1_HELP: Command reference follows:"));
        Console.println(F("============================================"));
        Console.println(F("Rail 1 Control Commands"));
        Console.println(F("============================================"));
        Console.println(F("MOTOR CONTROL:"));
        Console.println(F("  rail1 init          - Initialize Rail 1 motor system"));
        Console.println(F("  rail1 clear-fault   - Clear motor fault condition"));
        Console.println(F("  rail1 abort         - Abort current operation gracefully"));
        Console.println(F("  rail1 stop          - Emergency stop motor movement"));
        Console.println(F(""));
        Console.println(F("HOMING OPERATION:"));
        Console.println(F("  rail1 home          - Home carriage (find home position)"));
        Console.println(F(""));
        Console.println(F("CARRIAGE MOVEMENT:"));
        Console.println(F("  rail1 move-wc1 no-labware     - Move empty carriage to WC1"));
        Console.println(F("  rail1 move-wc2 no-labware     - Move empty carriage to WC2"));
        Console.println(F("  rail1 move-staging no-labware - Move empty carriage to staging"));
        Console.println(F("  rail1 move-handoff no-labware - Move empty carriage to handoff"));
        Console.println(F("  rail1 move-wc1 with-labware   - Move carriage with labware to WC1"));
        Console.println(F("  rail1 move-wc2 with-labware   - Move carriage with labware to WC2"));
        Console.println(F("  rail1 move-staging with-labware - Move carriage with labware to staging"));
        Console.println(F("  rail1 move-handoff with-labware - Move carriage with labware to handoff"));
        Console.println(F(""));
        Console.println(F("MANUAL POSITIONING:"));
        Console.print(F("  rail1 move-mm-to X no-labware   - Move empty carriage to absolute position X mm (0-"));
        Console.print(RAIL1_MAX_TRAVEL_MM);
        Console.println(F(")"));
        Console.println(F("  rail1 move-mm-to X with-labware - Move carriage with labware to absolute position X mm"));
        Console.println(F("  rail1 move-rel X no-labware     - Move empty carriage X mm relative (+ forward, - backward)"));
        Console.println(F("  rail1 move-rel X with-labware   - Move carriage with labware X mm relative"));
        Console.println(F(""));
        Console.println(F("STATUS AND DIAGNOSTICS:"));
        Console.println(F("  rail1 status        - Show comprehensive system status"));
        Console.println(F(""));
        Console.println(F("POSITION REFERENCE:"));
        Console.print(F("- Home: "));
        Console.print(RAIL1_HOME_POSITION);
        Console.println(F("mm (reference position)"));
        Console.print(F("- WC2: "));
        Console.print(RAIL1_WC2_PICKUP_DROPOFF);
        Console.println(F("mm (Workcell 2 pickup/dropoff)"));
        Console.print(F("- WC1: "));
        Console.print(RAIL1_WC1_PICKUP_DROPOFF);
        Console.println(F("mm (Workcell 1 pickup/dropoff)"));
        Console.print(F("- Staging: "));
        Console.print(RAIL1_STAGING_POSITION);
        Console.println(F("mm (coordination position for Rail 1-2 transfers)"));
        Console.print(F("- Handoff: "));
        Console.print(RAIL1_HANDOFF);
        Console.println(F("mm (transfer to Rail 2, same as home)"));
        Console.println(F(""));
        Console.println(F("SAFETY NOTES:"));
        Console.println(F("- Always specify labware status for movement commands"));
        Console.println(F("- Home carriage before first use"));
        Console.println(F("- Check sensors before movement operations"));
        Console.println(F("- Staging position is critical for coordinated Rail 1-2 operations"));
        Console.println(F("============================================"));
        return true;
        
    case 4: // "init" - Initialize Rail 1 motor system
        return executeRailInit(1);
        
    case 3: // "home" - Home carriage
        return executeRailHome(1);
        
    case 6: // "move-mm-to" - Move to absolute millimeter position
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail1 move-mm-to <position_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the position value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        
        // Execute the movement using helper function
        return executeRailMoveToPosition(1, targetPosition, carriageLoaded);
        
    case 7: // "move-rel" - Move relative distance
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail1 move-rel <distance_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the relative distance value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        
        // Execute the movement using helper function
        return executeRailMoveRelative(1, targetPosition, carriageLoaded);
        
    case 9: // "move-wc1" - Move carriage to WC1
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded)) return false;
        
        // Use Rail 1-specific helper function
        return moveRail1CarriageToWC1(carriageLoaded);
        
    case 10: // "move-wc2" - Move carriage to WC2
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded)) return false;
        
        // Use Rail 1-specific helper function
        return moveRail1CarriageToWC2(carriageLoaded);
        
    case 8: // "move-staging" - Move carriage to staging
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded)) return false;
        
        // Use Rail 1-specific helper function
        return moveRail1CarriageToStaging(carriageLoaded);
        
    case 5: // "move-handoff" - Move carriage to handoff
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded)) return false;
        
        // Use Rail 1-specific helper function
        return moveRail1CarriageToHandoff(carriageLoaded);
        
    case 11: // "status" - Show system status
        Console.acknowledge(F("DISPLAYING_RAIL1_STATUS: System diagnostics follow:"));
        Console.serialInfo(F("============================================"));
        Console.serialInfo(F("Rail 1 System Status"));
        Console.serialInfo(F("============================================"));
        
        // Safety status
        Console.serialInfo(F("SAFETY STATUS:"));
        if (isEStopActive()) {
            Console.serialInfo(F("  E-Stop Status: ACTIVE (UNSAFE)"));
        } else {
            Console.serialInfo(F("  E-Stop Status: INACTIVE (Safe)"));
        }
        
        // Motor status
        Console.serialInfo(F("MOTOR STATUS:"));
        Console.serialInfo(isMotorReady(1) ? F("  Motor Ready: YES") : F("  Motor Ready: NO"));
        Console.serialInfo(isHomingComplete(1) ? F("  Motor Homed: YES") : F("  Motor Homed: NO"));
        Console.serialInfo(isMotorMoving(1) ? F("  Motor Moving: YES") : F("  Motor Moving: NO"));
        Console.serialInfo(isHomingInProgress(1) ? F("  Motor Homing: YES") : F("  Motor Homing: NO"));
        
        // Current position
        Console.serialInfo(F("CURRENT POSITION:"));
        if (!isHomingComplete(1)) {
            Console.serialInfo(F("  Position: UNKNOWN (not homed) - Use 'rail1 home' first"));
        } else {
            currentPos = getMotorPositionMm(1);
            Console.serialInfo((String(F("  Position: ")) + String(currentPos, 2) + F("mm")).c_str());
            
            if (abs(currentPos - RAIL1_HOME_POSITION) < 50.0) {
                Console.serialInfo(F("  Location: AT HOME/HANDOFF"));
            } else if (abs(currentPos - RAIL1_WC2_PICKUP_DROPOFF) < 50.0) {
                Console.serialInfo(F("  Location: AT WC2"));
            } else if (abs(currentPos - RAIL1_WC1_PICKUP_DROPOFF) < 50.0) {
                Console.serialInfo(F("  Location: AT WC1"));
            } else if (abs(currentPos - RAIL1_STAGING_POSITION) < 50.0) {
                Console.serialInfo(F("  Location: AT STAGING"));
            } else {
                Console.serialInfo(F("  Location: BETWEEN POSITIONS"));
            }
        }
        
        // Position detection
        Console.serialInfo(F("POSITION DETECTION:"));
        Console.serialInfo(isCarriageAtWC1() ? F("  WC1 Detection: YES") : F("  WC1 Detection: NO"));
        Console.serialInfo(isCarriageAtWC2() ? F("  WC2 Detection: YES") : F("  WC2 Detection: NO"));
        Console.serialInfo(isCarriageAtRail1Handoff() ? F("  Handoff Detection: YES") : F("  Handoff Detection: NO"));
        
        // Labware detection
        Console.serialInfo(F("LABWARE DETECTION:"));
        Console.serialInfo(isLabwarePresentAtWC1() ? F("  WC1 Labware Present: YES") : F("  WC1 Labware Present: NO"));
        Console.serialInfo(isLabwarePresentAtWC2() ? F("  WC2 Labware Present: YES") : F("  WC2 Labware Present: NO"));
        Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff Labware Present: YES") : F("  Handoff Labware Present: NO"));
        
        Console.serialInfo(F("============================================"));
        return true;
        
    case 12: // "stop" - Emergency stop motor movement
        return executeRailStop(1);
        
    default: // Unknown command
        Console.error(F("Unknown action. Available: init, clear-fault, abort, stop, home, move-wc1, move-wc2, move-staging, move-handoff, move-mm-to, move-rel, status, and help"));
        return false;
    }
    
    return false; // Should never reach here
}

// ============================================================
// Labware Automation Command Implementation
// ============================================================

// Define the labware subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo LABWARE_COMMANDS[] = {
    {"audit", 0},
    {"help", 1},
    {"reset", 2},
    {"status", 3}
};

static const size_t LABWARE_COMMAND_COUNT = sizeof(LABWARE_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_labware(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';
    
    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Check for empty argument
    if (strlen(trimmed) == 0) {
        Console.error(F("Missing parameter. Usage: labware <action>"));
        return false;
    }
    
    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    
    if (action == NULL) {
        Console.error(F("Invalid format. Usage: labware <action>"));
        return false;
    }
    
    // Trim leading spaces from action
    action = trimLeadingSpaces(action);
    
    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++) {
        action[i] = tolower(action[i]);
    }
    
    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, LABWARE_COMMANDS, LABWARE_COMMAND_COUNT);
    
    // Use switch-case for cleaner flow control
    switch (cmdCode) {
    
    case 0: // "audit" - Automatically validate and fix labware state
        Console.acknowledge(F("LABWARE_AUDIT_INITIATED: Analyzing system state and validating labware positions"));
        
        if (performLabwareAudit()) {
            Console.acknowledge(F("AUDIT_COMPLETE: System ready for automation commands"));
        } else {
            Console.error(F("AUDIT_FAILED: Unable to validate labware state"));
        }
        return true;
        
    case 1: // "help" - Display help information
        Console.acknowledge(F("DISPLAYING_LABWARE_HELP: Automation system guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("Labware Automation Commands"));
        Console.println(F("============================================"));
        Console.println(F("STATE MANAGEMENT:"));
        Console.println(F("  labware status      - Display current labware tracking state"));
        Console.println(F("                        Shows rail states, sensor readings, confidence levels"));
        Console.println(F(""));
        Console.println(F("RECOVERY OPERATIONS:"));
        Console.println(F("  labware audit       - Automatically validate and fix labware state"));
        Console.println(F("                        Moves to nearest sensor, reads actual state"));
        Console.println(F("                        Updates tracking based on ground truth"));
        Console.println(F("  labware reset       - Clear all labware tracking (nuclear option)"));
        Console.println(F("                        Wipes all state, requires manual re-establishment"));
        Console.println(F(""));
        Console.println(F("SYSTEM ARCHITECTURE:"));
        Console.println(F("- Rail 1: Checkpoint-based tracking (sensors at WC1, WC2, handoff)"));
        Console.println(F("- Rail 2: Continuous tracking (carriage-mounted sensor)"));
        Console.println(F("- Confidence levels: HIGH (real-time), MEDIUM (recent sensor), LOW (inferred)"));
        Console.println(F(""));
        Console.println(F("USAGE SCENARIOS:"));
        Console.println(F("- After motor faults: Use 'labware audit' to validate state"));
        Console.println(F("- System confusion: Use 'labware reset' to start fresh"));
        Console.println(F("- Regular monitoring: Use 'labware status' to check state"));
        Console.println(F(""));
        Console.println(F("SAFETY FEATURES:"));
        Console.println(F("- Conservative movement (with-labware speeds during audit)"));
        Console.println(F("- Sensor validation (ground truth confirmation)"));
        Console.println(F("- Collision avoidance (audit only moves to WC1/WC2)"));
        Console.println(F("============================================"));
        return true;
        
    case 2: // "reset" - Clear all labware tracking (nuclear option)
        Console.acknowledge(F("NUCLEAR_RESET_INITIATED: Clearing all labware tracking state"));
        clearLabwareState();
        Console.acknowledge(F("RESET_COMPLETE: Use 'labware audit' to establish current state"));
        return true;
        
    case 3: // "status" - Display current labware tracking state
        Console.acknowledge(F("DISPLAYING_LABWARE_STATUS: Current tracking state follows:"));
        printLabwareSystemStatus();
        return true;
        
    default: // Unknown command
        Console.error(F("Unknown labware command. Available: status, audit, reset, help"));
        return false;
    }
    
    return false; // Should never reach here
}

//=============================================================================
// AUTOMATED LABWARE MOVEMENT COMMAND IMPLEMENTATION
//=============================================================================

// Goto command lookup table for binary search
const SubcommandInfo GOTO_ACTIONS[] = {
    {"no-labware", 0},     // Move to location without labware
    {"with-labware", 1}    // Move to location with labware
};

const size_t GOTO_ACTION_COUNT = sizeof(GOTO_ACTIONS) / sizeof(GOTO_ACTIONS[0]);

// Location lookup table for goto command
const SubcommandInfo GOTO_LOCATIONS[] = {
    {"wc1", 0},        // Work Cell 1
    {"wc2", 1},        // Work Cell 2
    {"wc3", 2}         // Work Cell 3
};

const size_t GOTO_LOCATION_COUNT = sizeof(GOTO_LOCATIONS) / sizeof(GOTO_LOCATIONS[0]);

bool cmd_goto(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';
    
    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Check for empty argument
    if (strlen(trimmed) == 0) {
        Console.error(F("Missing parameters. Usage: goto <location> <status>"));
        Console.error(F("Example: goto wc1 pickup"));
        return false;
    }
    
    // Parse location and action arguments
    char *location = strtok(trimmed, " ");
    char *action = strtok(NULL, " ");
    
    if (location == NULL || action == NULL) {
        Console.error(F("Invalid format. Usage: goto <location> <status>"));
        Console.error(F("Locations: wc1, wc2, wc3"));
        Console.error(F("Status: with-labware, no-labware"));
        return false;
    }
    
    // Trim and convert to lowercase
    location = trimLeadingSpaces(location);
    action = trimLeadingSpaces(action);
    
    for (int i = 0; location[i]; i++) {
        location[i] = tolower(location[i]);
    }
    for (int i = 0; action[i]; i++) {
        action[i] = tolower(action[i]);
    }
    
    // Use binary search to find location and action codes
    int locationCode = findSubcommandCode(location, GOTO_LOCATIONS, GOTO_LOCATION_COUNT);
    int actionCode = findSubcommandCode(action, GOTO_ACTIONS, GOTO_ACTION_COUNT);
    
    if (locationCode == -1) {
        Console.error((String(F("Unknown location: ")) + String(location)).c_str());
        Console.error(F("Available locations: wc1, wc2, wc3"));
        return false;
    }
    
    if (actionCode == -1) {
        Console.error((String(F("Unknown action: ")) + String(action)).c_str());
        Console.error(F("Available actions: with-labware, no-labware"));
        return false;
    }
    
    // Convert codes to enums for processing
    Location targetLocation;
    bool hasLabware;
    
    switch (locationCode) {
        case 0: targetLocation = LOCATION_WC1; break;
        case 1: targetLocation = LOCATION_WC2; break;
        case 2: targetLocation = LOCATION_WC3; break;
        default: targetLocation = LOCATION_UNKNOWN; break;
    }
    
    switch (actionCode) {
        case 0: hasLabware = false; break;  // no-labware
        case 1: hasLabware = true; break;   // with-labware
        default: hasLabware = false; break;
    }
    
    // Pre-flight checks before attempting automated movement
    if (!performGotoPreflightChecks(targetLocation, hasLabware)) {
        return false;
    }
    
    Console.acknowledge((String(F("GOTO_INITIATED: Moving to ")) + getLocationName(targetLocation) + 
                        String(F(" ")) + (hasLabware ? F("with-labware") : F("no-labware"))).c_str());
    
    // Execute the automated movement based on location and labware status
    switch (locationCode) {
    
    case 0: // "wc1" - Work Cell 1
        if (hasLabware) {
            Console.serialInfo(F("WC1_WITH_LABWARE: Moving to WC1 with labware"));
            return executeWC1WithLabware();
        } else {
            Console.serialInfo(F("WC1_NO_LABWARE: Moving to WC1 without labware"));
            return executeWC1NoLabware();
        }
        break;
        
    case 1: // "wc2" - Work Cell 2
        if (hasLabware) {
            Console.serialInfo(F("WC2_WITH_LABWARE: Moving to WC2 with labware"));
            return executeWC2WithLabware();
        } else {
            Console.serialInfo(F("WC2_NO_LABWARE: Moving to WC2 without labware"));
            return executeWC2NoLabware();
        }
        break;
        
    case 2: // "wc3" - Work Cell 3
        if (hasLabware) {
            Console.serialInfo(F("WC3_WITH_LABWARE: Moving to WC3 with labware"));
            return executeWC3WithLabware();
        } else {
            Console.serialInfo(F("WC3_NO_LABWARE: Moving to WC3 without labware"));
            return executeWC3NoLabware();
        }
        break;
        
    default: // Unknown location (should not reach here due to earlier validation)
        Console.error(F("Internal error: Invalid location code"));
        return false;
    }
    
    return false; // Should never reach here
}

//=============================================================================
// GOTO PREFLIGHT CHECKS
//=============================================================================

bool performGotoPreflightChecks(Location targetLocation, bool hasLabware) {
    // Perform comprehensive pre-flight validation before automated movement
    Console.serialInfo(F("PREFLIGHT_CHECKS: Validating system state for automated movement"));
    
    // TODO: Implement comprehensive preflight checks
    // 1. Verify labware system is ready for automation
    // 2. Check rail homing status
    // 3. Validate current labware state matches intended action
    // 4. Check for conflicts (dual labware, collision zones)
    // 5. Verify target location is reachable and safe
    // 6. Check pneumatic pressure for labware operations
    // 7. Validate motion limits and safety zones
    
    Console.serialInfo(F("PREFLIGHT_PLACEHOLDER: Comprehensive validation not yet implemented"));
    Console.serialInfo(F("- System ready check"));
    Console.serialInfo(F("- Rail homing validation"));
    Console.serialInfo(F("- Labware state consistency"));
    Console.serialInfo(F("- Collision avoidance"));
    Console.serialInfo(F("- Safety zone validation"));
    
    return true; // Placeholder - always pass for now
}

//=============================================================================
// WC1 LOCATION IMPLEMENTATIONS
//=============================================================================

bool executeWC1WithLabware() {
    // Move to WC1 with labware
    Console.serialInfo(F("WC1_WITH_LABWARE_PLACEHOLDER: Implementation pending"));
    
    // TODO: Implement WC1 with-labware movement
    // 1. Validate Rail 1 has labware
    // 2. Move Rail 1 to WC1 position at with-labware speeds
    // 3. Update labware tracking
    
    return true;
}

bool executeWC1NoLabware() {
    // Move to WC1 without labware
    Console.serialInfo(F("WC1_NO_LABWARE_PLACEHOLDER: Implementation pending"));
    
    // TODO: Implement WC1 no-labware movement
    // 1. Validate Rail 1 is empty
    // 2. Move Rail 1 to WC1 position at empty speeds
    // 3. Update position tracking
    
    return true;
}

//=============================================================================
// WC2 LOCATION IMPLEMENTATIONS
//=============================================================================

bool executeWC2WithLabware() {
    // Move to WC2 with labware
    Console.serialInfo(F("WC2_WITH_LABWARE_PLACEHOLDER: Implementation pending"));
    
    // TODO: Implement WC2 with-labware movement
    // 1. Validate Rail 1 has labware
    // 2. Move Rail 1 to WC2 position at with-labware speeds
    // 3. Update labware tracking
    
    return true;
}

bool executeWC2NoLabware() {
    // Move to WC2 without labware
    Console.serialInfo(F("WC2_NO_LABWARE_PLACEHOLDER: Implementation pending"));
    
    // TODO: Implement WC2 no-labware movement
    // 1. Validate Rail 1 is empty
    // 2. Move Rail 1 to WC2 position at empty speeds
    // 3. Update position tracking
    
    return true;
}

//=============================================================================
// WC3 LOCATION IMPLEMENTATIONS
//=============================================================================

bool executeWC3WithLabware() {
    // Move to WC3 with labware
    Console.serialInfo(F("WC3_WITH_LABWARE_PLACEHOLDER: Implementation pending"));
    
    // TODO: Implement WC3 with-labware movement
    // 1. Validate Rail 2 has labware
    // 2. Move Rail 2 to WC3 position at with-labware speeds
    // 3. Update labware tracking
    
    return true;
}

bool executeWC3NoLabware() {
    // Move to WC3 without labware
    Console.serialInfo(F("WC3_NO_LABWARE_PLACEHOLDER: Implementation pending"));
    
    // TODO: Implement WC3 no-labware movement
    // 1. Validate Rail 2 is empty
    // 2. Move Rail 2 to WC3 position at empty speeds
    // 3. Update position tracking
    
    return true;
}