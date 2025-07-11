#include "Commands.h"
#include "Logging.h"
#include "RailAutomation.h"
#include "HandoffController.h"

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

    // Unified lock/unlock commands
    // systemCommand("lock", "Lock a tray or shuttle:\r\n"
    //                       "  lock,1..3    - Lock specific tray position\r\n"
    //                       "  lock,shuttle - Lock the shuttle\r\n"
    //                       "  lock,help    - Display detailed lock instructions",
    //               cmd_lock),

    // systemCommand("unlock", "Unlock a tray, shuttle, or all valves:\r\n"
    //                         "  unlock,1..3    - Unlock specific tray position\r\n"
    //                         "  unlock,shuttle - Unlock the shuttle\r\n"
    //                         "  unlock,all     - Unlock all valves\r\n"
    //                         "  unlock,help    - Display detailed unlock instructions",
    //               cmd_unlock),

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

    // State command to display system state
    // systemCommand("system", "System commands:\r\n"
    //                         "  system,state    - Display current system state (sensors, actuators, positions)\r\n"
    //                         "  system,safety   - Display comprehensive safety validation status\r\n"
    //                         "  system,trays    - Display tray tracking and statistics\r\n"
    //                         "  system,reset    - Reset system state after failure to retry operation\r\n"
    //                         "  system,help     - Display detailed instructions for system commands\r\n"
    //                         "                    (Use 'log,history' or 'log,errors' for operation troubleshooting)",
    //               cmd_system_state),

    // Motor control commands
    // systemCommand("motor", "Motor control:\r\n"
    //                        "  motor,init   - Initialize motor system and prepare for operation\r\n"
    //                        "  motor,status - Display detailed motor status and configuration\r\n"
    //                        "  motor,clear  - Clear motor fault condition to restore operation\r\n"
    //                        "  motor,home   - Home the motor (find zero position)\r\n"
    //                        "  motor,abort  - Abort current operation gracefully\r\n"
    //                        "  motor,stop   - Emergency stop motor movement immediately\r\n"
    //                        "  motor,help   - Display comprehensive motor control instructions",
    //               cmd_motor),

    // Move command
    // systemCommand("move", "Move motor to position:\r\n"
    //                       "  move,home      - Move to home (zero) position\r\n"
    //                       "  move,1..4      - Move to predefined positions 1 through 4\r\n"
    //                       "  move,counts,X  - Move to absolute position X in encoder counts (0-64333)\r\n"
    //                       "  move,mm,X      - Move to absolute position X in millimeters (0-1050.0)\r\n"
    //                       "  move,rel,X     - Move X millimeters relative to current position (+ forward, - backward)\r\n"
    //                       "  move,help      - Display detailed command usage and troubleshooting",
    //               cmd_move),

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
                Console.error(F("LOG_INTERVAL_TOO_SMALL"));
                Console.serialInfo(F("Minimum logging interval is 100ms"));
                return false;
            }
            if (interval > 60000) {
                Console.error(F("LOG_INTERVAL_TOO_LARGE"));
                Console.serialInfo(F("Maximum logging interval is 60000ms (1 minute)"));
                return false;
            }
        }
        
        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset timer
        
        Console.acknowledge(F("OK_LOG_ON"));
        Console.serialInfo(F("Periodic logging enabled with interval: "));
        Console.serialInfo(String(interval).c_str());
        Console.serialInfo(F("ms"));
        return true;
    }
    else if (strcmp(action, "off") == 0) {
        // Disable periodic logging
        logging.logInterval = 0;
        
        Console.acknowledge(F("OK_LOG_OFF"));
        Console.serialInfo(F("Periodic logging disabled"));
        return true;
    }
    else if (strcmp(action, "now") == 0) {
        // Log system state immediately
        Console.acknowledge(F("OK_LOG_NOW"));
        logSystemState();
        return true;
    }
    else if (strcmp(action, "history") == 0) {
        // Show complete operation log history
        Console.acknowledge(F("OK_LOG_HISTORY"));
        Console.serialInfo(F("Complete operation log history:"));
        opLogHistory.printHistory();
        return true;
    }
    else if (strcmp(action, "errors") == 0) {
        // Show only errors and warnings
        Console.acknowledge(F("OK_LOG_ERRORS"));
        Console.serialInfo(F("Error and warning log entries:"));
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
        
        Console.acknowledge(F("OK_LOG_LAST"));
        Console.serialInfo(F("Last "));
        Console.serialInfo(String(count).c_str());
        Console.serialInfo(F(" log entries:"));
        opLogHistory.printLastN(count);
        return true;
    }
    else if (strcmp(action, "stats") == 0) {
        // Show log buffer statistics
        Console.acknowledge(F("OK_LOG_STATS"));
        Console.serialInfo(F("Log buffer statistics:"));
        opLogHistory.printStats();
        
        // Also show current logging status
        Console.serialInfo(F("Current logging status:"));
        if (logging.logInterval > 0) {
            Console.serialInfo(F("  Periodic logging: ENABLED ("));
            Console.serialInfo(String(logging.logInterval).c_str());
            Console.serialInfo(F("ms interval)"));
        } else {
            Console.serialInfo(F("  Periodic logging: DISABLED"));
        }
        return true;
    }
    else if (strcmp(action, "help") == 0) {
        // Display detailed help information
        Console.acknowledge(F("LOG_HELP"));
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
    
    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, RAIL2_COMMANDS, RAIL2_COMMAND_COUNT);
    
    // Use switch-case for cleaner flow control
    switch (cmdCode) {
    
    case 0: // "abort" - Abort current operation gracefully
        // Check if motor is initialized before attempting to abort
        if (!isMotorReady(2)) {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(F("Rail 2 motor is not initialized. Nothing to abort."));
            return false;
        }

        Console.serialInfo(F("Aborting current Rail 2 operation..."));

        // Only meaningful to abort if we're moving or homing
        if (isMotorMoving(2) || isHomingInProgress(2)) {
            if (isHomingInProgress(2)) {
                abortHoming(2);
            } else {
                stopMotion(2);
            }

            Console.acknowledge(F("OK_ABORT"));
            Console.serialInfo(F("Rail 2 operation aborted successfully"));
            return true;
        } else {
            Console.error(F("NO_ACTIVE_OPERATION"));
            Console.serialInfo(F("No active operation to abort"));
            return false;
        }
        
    case 1: // "clear-fault" - Clear motor fault condition
        Console.serialInfo(F("Attempting to clear Rail 2 motor fault..."));

        if (clearMotorFaultWithStatus(2)) {
            Console.acknowledge(F("OK_CLEAR_FAULT"));
            Console.serialInfo(F("Rail 2 motor fault cleared successfully"));
            return true;
        } else {
            Console.error(F("CLEAR_FAULT_FAILED"));
            Console.serialInfo(F("Failed to clear Rail 2 motor fault"));
            Console.serialInfo(F("Motor may still be in fault state."));
            Console.serialInfo(F("Try power cycling the system if fault persists."));
            return false;
        }
        
    case 2: // "extend" - Extend pneumatic drive
        if (!isPressureSufficient()) {
            Console.error(F("INSUFFICIENT_PRESSURE"));
            Console.serialInfo(F("Air pressure too low for valve operation"));
            return false;
        }
        
        Console.serialInfo(F("Extending pneumatic drive..."));
        result = extendCylinder();
        
        if (result == VALVE_OP_SUCCESS) {
            Console.acknowledge(F("OK_EXTEND"));
            Console.serialInfo(F("Pneumatic drive extended successfully"));
            return true;
        } else {
            Console.error(F("EXTEND_FAILED"));
            Console.serialInfo(F("Failed to extend pneumatic drive:"));
            Console.serialInfo(getValveOperationResultName(result));
            return false;
        }
        
    case 3: // "help" - Display help information
        Console.acknowledge(F("RAIL2_HELP"));
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
        Console.serialInfo(F("Initializing Rail 2 motor..."));

        // Diagnostic: Print state before initialization
        Console.serialInfo(F("[DIAGNOSTIC] Checking Rail 2 motor state before initialization"));

        initMotorSystem();

        if (isMotorReady(2)) {
            Console.acknowledge(F("OK_INIT"));
            Console.serialInfo(F("Rail 2 motor initialized successfully"));
            return true;
        } else {
            Console.error(F("INIT_FAILED"));
            Console.serialInfo(F("Rail 2 motor initialization failed."));
            Console.serialInfo(F("Check connections and power."));
            return false;
        }
        
    case 10: // "retract" - Retract pneumatic drive
        if (!isPressureSufficient()) {
            Console.error(F("INSUFFICIENT_PRESSURE"));
            Console.serialInfo(F("Air pressure too low for valve operation"));
            return false;
        }
        
        Console.serialInfo(F("Retracting pneumatic drive..."));
        result = retractCylinder();
        
        if (result == VALVE_OP_SUCCESS) {
            Console.acknowledge(F("OK_RETRACT"));
            Console.serialInfo(F("Pneumatic drive retracted successfully"));
            return true;
        } else {
            Console.error(F("RETRACT_FAILED"));
            Console.serialInfo(F("Failed to retract pneumatic drive:"));
            Console.serialInfo(getValveOperationResultName(result));
            return false;
        }
        
    case 4: // "home" - Home carriage
        if (!checkRailMovementReadiness(2)) return false;
        
        // Use helper function for cylinder safety (homing always involves collision zone)
        if (!ensureCylinderRetractedForSafeMovement(true)) return false;
        
        Console.serialInfo(F("Initiating Rail 2 homing sequence..."));
        if (initiateHomingSequence(2)) {
            Console.acknowledge(F("OK_HOME"));
            Console.serialInfo(F("Homing sequence initiated successfully"));
            return true;
        } else {
            Console.error(F("HOMING_START_FAILED"));
            Console.serialInfo(F("Failed to start homing sequence - check motor status"));
            return false;
        }
        
    case 7: // "move-mm-to" - Move to absolute millimeter position
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail2 move-mm-to <position_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the position value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        if (!checkRailMovementReadiness(2)) return false;
        
        // CRITICAL SAFETY: Check if any part of the movement path requires cylinder retraction to prevent Rail 1 collision
        // Collision zone is between RAIL2_COLLISION_ZONE_START and RAIL2_COLLISION_ZONE_END
        // We must check current position, target position, and the entire movement path
        currentPos = getMotorPositionMm(2);
        movementInCollisionZone = false;
        
        // Check if current position is in collision zone
        if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true;
        }
        
        // Check if target position is in collision zone
        if (targetPosition >= RAIL2_COLLISION_ZONE_START && targetPosition <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true;
        }
        
        // Check if movement path crosses collision zone (even if start/end are outside zone)
        if (currentPos < RAIL2_COLLISION_ZONE_START && targetPosition >= RAIL2_COLLISION_ZONE_START) {
            movementInCollisionZone = true; // Entering collision zone
        }
        if (currentPos > RAIL2_COLLISION_ZONE_END && targetPosition <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true; // Entering collision zone from far side
        }
        if ((currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) && 
            (targetPosition < RAIL2_COLLISION_ZONE_START || targetPosition > RAIL2_COLLISION_ZONE_END)) {
            movementInCollisionZone = true; // Exiting collision zone
        }
        
        // Use helper function for cylinder safety
        if (!ensureCylinderRetractedForSafeMovement(movementInCollisionZone)) return false;
        
        Console.serialInfo(carriageLoaded ? 
            F("Moving carriage with labware to absolute position...") :
            F("Moving empty carriage to absolute position..."));
        
        // Execute the movement
        if (moveToPositionMm(2, targetPosition, carriageLoaded)) {
            Console.acknowledge(F("OK_MOVE_MM_TO"));
            Console.serialInfo(F("Carriage successfully moved to target position"));
            return true;
        } else {
            Console.error(F("MOVEMENT_FAILED"));
            Console.serialInfo(F("Failed to move carriage to target position - check motor status"));
            return false;
        }
        
    case 8: // "move-rel" - Move relative distance
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail2 move-rel <distance_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the relative distance value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        if (!checkRailMovementReadiness(2)) return false;
        
        // CRITICAL SAFETY: Check if any part of the movement path requires cylinder retraction to prevent Rail 1 collision
        // Calculate target position and check if movement involves collision zone (500-700mm)
        currentPos = getMotorPositionMm(2);
        calculatedTargetPos = currentPos + targetPosition;
        movementInCollisionZone = false;
        
        // Check if current position is in collision zone
        if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true;
        }
        
        // Check if target position is in collision zone
        if (calculatedTargetPos >= RAIL2_COLLISION_ZONE_START && calculatedTargetPos <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true;
        }
        
        // Check if movement path crosses collision zone boundary
        if (currentPos < RAIL2_COLLISION_ZONE_START && calculatedTargetPos >= RAIL2_COLLISION_ZONE_START) {
            movementInCollisionZone = true; // Entering collision zone
        }
        if (currentPos > RAIL2_COLLISION_ZONE_END && calculatedTargetPos <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true; // Entering collision zone from far side
        }
        if ((currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) && 
            (calculatedTargetPos < RAIL2_COLLISION_ZONE_START || calculatedTargetPos > RAIL2_COLLISION_ZONE_END)) {
            movementInCollisionZone = true; // Exiting collision zone
        }
        
        // Use helper function for cylinder safety
        if (!ensureCylinderRetractedForSafeMovement(movementInCollisionZone)) return false;
        
        Console.serialInfo(carriageLoaded ? 
            F("Moving carriage with labware relative distance...") :
            F("Moving empty carriage relative distance..."));
        
        // Execute the movement
        if (moveRelativeManual(2, targetPosition, carriageLoaded)) {
            Console.acknowledge(F("OK_MOVE_REL"));
            Console.serialInfo(F("Carriage successfully moved relative distance"));
            return true;
        } else {
            Console.error(F("MOVEMENT_FAILED"));
            Console.serialInfo(F("Failed to move carriage relative distance - check motor status"));
            return false;
        }
        
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
        Console.acknowledge(F("RAIL2_STATUS"));
        Console.serialInfo(F("============================================"));
        Console.serialInfo(F("Rail 2 System Status"));
        Console.serialInfo(F("============================================"));
        
        // E-Stop status (critical safety information first)
        Console.serialInfo(F("SAFETY STATUS:"));
        Console.serialInfo(isEStopActive() ? F("  E-Stop: ACTIVE (UNSAFE - Cannot operate)") : F("  E-Stop: INACTIVE (Safe to operate)"));
        
        // Motor status
        Console.serialInfo(F("MOTOR STATUS:"));
        Console.serialInfo(isMotorReady(2) ? F("  Motor Ready: YES") : F("  Motor Ready: NO"));
        Console.serialInfo(isHomingComplete(2) ? F("  Homing Status: COMPLETE") : F("  Homing Status: NOT HOMED"));
        Console.serialInfo(isMotorMoving(2) ? F("  Motor Moving: YES") : F("  Motor Moving: NO"));
        Console.serialInfo(isHomingInProgress(2) ? F("  Homing in Progress: YES") : F("  Homing in Progress: NO"));
        
        // Current position and location analysis
        Console.serialInfo(F("CURRENT LOCATION:"));
        if (!isHomingComplete(2)) {
            Console.serialInfo(F("  Raw Position: UNKNOWN (Motor not homed)"));
            Console.serialInfo(F("  Location: UNKNOWN - Motor must be homed first"));
            Console.serialInfo(F("  Use 'rail2 home' to establish position reference"));
        } else {
            currentPos = getMotorPositionMm(2);
            Console.serialInfo(F("  Raw Position: "));
            Console.serialInfo(String(currentPos, 2).c_str());
            Console.serialInfo(F(" mm from home"));
            
            // Determine and display meaningful location
            if (isCarriageAtWC3()) {
                Console.serialInfo(F("  Location: AT WORKCELL 3 (Home position)"));
            } else if (isCarriageAtRail2Handoff()) {
                Console.serialInfo(F("  Location: AT HANDOFF POSITION (Rail 1-2 transfer)"));
            } else {
                // Determine location based on position ranges using defined constants
                double homeToCollisionMidpoint = (RAIL2_HOME_POSITION + RAIL2_COLLISION_ZONE_START) / 2.0;
                double handoffNearRange = 50.0; // ±50mm around handoff position
                
                if (currentPos < homeToCollisionMidpoint) {
                    Console.serialInfo(F("  Location: NEAR WORKCELL 3 (close to home)"));
                } else if (abs(currentPos - RAIL2_HANDOFF) <= handoffNearRange) {
                    Console.serialInfo(F("  Location: NEAR HANDOFF POSITION (Rail 1-2 transfer area)"));
                } else if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) {
                    Console.serialInfo(F("  Location: IN COLLISION ZONE (between WC3 and handoff)"));
                } else if (currentPos > RAIL2_HANDOFF) {
                    Console.serialInfo(F("  Location: BEYOND HANDOFF (far end of rail)"));
                } else {
                    Console.serialInfo(F("  Location: BETWEEN WC3 AND HANDOFF"));
                }
            }
        }
        
        // Pneumatic system status
        Console.serialInfo(F("PNEUMATIC SYSTEM:"));
        Console.serialInfo(F("  Air Pressure: "));
        Console.serialInfo(String(getPressurePsi(), 1).c_str());
        Console.serialInfo(F(" PSI"));
        Console.serialInfo(isPressureSufficient() ? F("  Pressure Status: SUFFICIENT") : F("  Pressure Status: INSUFFICIENT"));
        
        // Valve position and validation
        valveState = getValvePosition();
        Console.serialInfo(F("  Valve Controller State: "));
        Console.serialInfo(getValvePositionName(valveState));
        
        // Raw sensor readings
        sensorRetracted = isCylinderRetracted();
        sensorExtended = isCylinderExtended();
        Console.serialInfo(sensorRetracted ? F("  Retracted Sensor: ACTIVE") : F("  Retracted Sensor: INACTIVE"));
        Console.serialInfo(sensorExtended ? F("  Extended Sensor: ACTIVE") : F("  Extended Sensor: INACTIVE"));
        
        // Validated position states
        actuallyRetracted = isCylinderActuallyRetracted();
        actuallyExtended = isCylinderActuallyExtended();
        Console.serialInfo(actuallyRetracted ? F("  Actually Retracted: YES") : F("  Actually Retracted: NO"));
        Console.serialInfo(actuallyExtended ? F("  Actually Extended: YES") : F("  Actually Extended: NO"));
        
        // Valve validation result
        validationResult = validateValvePosition();
        if (validationResult) {
            Console.serialInfo(F("  Valve Validation: PASS"));
        } else {
            Console.serialError(F("  Valve Validation: FAIL - SENSOR/VALVE MISMATCH"));
            Console.serialInfo(F("    Check sensor wiring and valve operation"));
        }
        
        // Position sensors confirmation
        Console.serialInfo(F("POSITION SENSORS:"));
        Console.serialInfo(isCarriageAtWC3() ? F("  WC3 Sensor: ACTIVE (carriage detected)") : F("  WC3 Sensor: INACTIVE"));
        Console.serialInfo(isCarriageAtRail2Handoff() ? F("  Handoff Sensor: ACTIVE (carriage detected)") : F("  Handoff Sensor: INACTIVE"));
        
        // Labware detection
        Console.serialInfo(F("LABWARE DETECTION:"));
        Console.serialInfo(isLabwarePresentAtWC3() ? F("  WC3: PRESENT") : F("  WC3: NOT PRESENT"));
        Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff: PRESENT") : F("  Handoff: NOT PRESENT"));
        
        // Safety zone status
        Console.serialInfo(F("COLLISION SAFETY:"));
        if (!isHomingComplete(2)) {
            Console.serialInfo(F("  Current Zone: UNKNOWN (Motor not homed)"));
            Console.serialInfo(F("  Safety Status: UNKNOWN - Home motor first to determine position"));
        } else {
            if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) {
                Console.serialInfo(F("  Current Zone: COLLISION ZONE (500-700mm)"));
                Console.serialInfo(actuallyRetracted ? F("  Safety Status: SAFE (Cylinder retracted)") : F("  Safety Status: UNSAFE (Cylinder not retracted in collision zone)"));
            } else {
                Console.serialInfo(F("  Current Zone: SAFE ZONE"));
                Console.serialInfo(F("  Safety Status: SAFE (Outside collision zone)"));
            }
        }
        
        Console.serialInfo(F("============================================"));
        return true;
        
    case 12: // "stop" - Emergency stop motor movement
        // Check if motor is initialized
        if (!isMotorReady(2)) {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(F("Rail 2 motor is not initialized. Cannot perform stop."));
            return false;
        }

        Console.serialInfo(F("EMERGENCY STOP initiated for Rail 2!"));

        // Execute emergency stop
        stopMotion(2);

        Console.acknowledge(F("OK_EMERGENCY_STOP"));
        Console.serialInfo(F("Rail 2 motor movement halted. Position may no longer be accurate."));
        Console.serialInfo(F("Re-homing recommended after emergency stop."));

        return true;
        
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
    
    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, RAIL1_COMMANDS, RAIL1_COMMAND_COUNT);
    
    // Use switch-case for cleaner flow control
    switch (cmdCode) {
    
    case 0: // "abort" - Abort current operation gracefully
        // Check if motor is initialized before attempting to abort
        if (!isMotorReady(1)) {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(F("Rail 1 motor is not initialized. Nothing to abort."));
            return false;
        }

        Console.serialInfo(F("Aborting current Rail 1 operation..."));

        // Only meaningful to abort if we're moving or homing
        if (isMotorMoving(1) || isHomingInProgress(1)) {
            if (isHomingInProgress(1)) {
                abortHoming(1);
            } else {
                stopMotion(1);
            }

            Console.acknowledge(F("OK_ABORT"));
            Console.serialInfo(F("Rail 1 operation aborted successfully"));
            return true;
        } else {
            Console.error(F("NO_ACTIVE_OPERATION"));
            Console.serialInfo(F("No active operation to abort"));
            return false;
        }
        
    case 1: // "clear-fault" - Clear motor fault condition
        Console.serialInfo(F("Attempting to clear Rail 1 motor fault..."));

        if (clearMotorFaultWithStatus(1)) {
            Console.acknowledge(F("OK_CLEAR_FAULT"));
            Console.serialInfo(F("Rail 1 motor fault cleared successfully"));
            return true;
        } else {
            Console.error(F("CLEAR_FAULT_FAILED"));
            Console.serialInfo(F("Failed to clear Rail 1 motor fault"));
            Console.serialInfo(F("Motor may still be in fault state."));
            Console.serialInfo(F("Try power cycling the system if fault persists."));
            return false;
        }
        
    case 2: // "help" - Display help information
        Console.acknowledge(F("RAIL1_HELP"));
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
        Console.serialInfo(F("Initializing Rail 1 motor..."));

        initMotorSystem();

        if (isMotorReady(1)) {
            Console.acknowledge(F("OK_INIT"));
            Console.serialInfo(F("Rail 1 motor initialized successfully"));
            return true;
        } else {
            Console.error(F("INIT_FAILED"));
            Console.serialInfo(F("Rail 1 motor initialization failed."));
            Console.serialInfo(F("Check connections and power."));
            return false;
        }
        
    case 3: // "home" - Home carriage
        if (!checkRailMovementReadiness(1)) return false;
        
        Console.serialInfo(F("Initiating Rail 1 homing sequence..."));
        if (initiateHomingSequence(1)) {
            Console.acknowledge(F("OK_HOME"));
            Console.serialInfo(F("Homing sequence initiated successfully"));
            return true;
        } else {
            Console.error(F("HOMING_START_FAILED"));
            Console.serialInfo(F("Failed to start homing sequence - check motor status"));
            return false;
        }
        
    case 6: // "move-mm-to" - Move to absolute millimeter position
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail1 move-mm-to <position_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the position value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        if (!checkRailMovementReadiness(1)) return false;
        
        Console.serialInfo(carriageLoaded ? 
            F("Moving carriage with labware to absolute position...") :
            F("Moving empty carriage to absolute position..."));
        
        // Execute the movement
        if (moveToPositionMm(1, targetPosition, carriageLoaded)) {
            Console.acknowledge(F("OK_MOVE_MM_TO"));
            Console.serialInfo(F("Carriage successfully moved to target position"));
            return true;
        } else {
            Console.error(F("MOVEMENT_FAILED"));
            Console.serialInfo(F("Failed to move carriage to target position - check motor status"));
            return false;
        }
        
    case 7: // "move-rel" - Move relative distance
        if (param1 == NULL || param2 == NULL) {
            Console.error(F("Missing parameters. Usage: rail1 move-rel <distance_mm> <with-labware|no-labware>"));
            return false;
        }
        
        // Parse the relative distance value
        targetPosition = atof(param1);
        
        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded)) return false;
        if (!checkRailMovementReadiness(1)) return false;
        
        Console.serialInfo(carriageLoaded ? 
            F("Moving carriage with labware relative distance...") :
            F("Moving empty carriage relative distance..."));
        
        // Execute the movement
        if (moveRelativeManual(1, targetPosition, carriageLoaded)) {
            Console.acknowledge(F("OK_MOVE_REL"));
            Console.serialInfo(F("Carriage successfully moved relative distance"));
            return true;
        } else {
            Console.error(F("MOVEMENT_FAILED"));
            Console.serialInfo(F("Failed to move carriage relative distance - check motor status"));
            return false;
        }
        
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
        Console.acknowledge(F("RAIL1_STATUS"));
        Console.serialInfo(F("============================================"));
        Console.serialInfo(F("Rail 1 System Status"));
        Console.serialInfo(F("============================================"));
        
        // E-Stop status (critical safety information first)
        Console.serialInfo(F("SAFETY STATUS:"));
        Console.serialInfo(isEStopActive() ? F("  E-Stop: ACTIVE (UNSAFE - Cannot operate)") : F("  E-Stop: INACTIVE (Safe to operate)"));
        
        // Motor status
        Console.serialInfo(F("MOTOR STATUS:"));
        Console.serialInfo(isMotorReady(1) ? F("  Motor Ready: YES") : F("  Motor Ready: NO"));
        Console.serialInfo(isHomingComplete(1) ? F("  Homing Status: COMPLETE") : F("  Homing Status: NOT HOMED"));
        Console.serialInfo(isMotorMoving(1) ? F("  Motor Moving: YES") : F("  Motor Moving: NO"));
        Console.serialInfo(isHomingInProgress(1) ? F("  Homing in Progress: YES") : F("  Homing in Progress: NO"));
        
        // Current position and location analysis
        Console.serialInfo(F("CURRENT LOCATION:"));
        if (!isHomingComplete(1)) {
            Console.serialInfo(F("  Raw Position: UNKNOWN (Motor not homed)"));
            Console.serialInfo(F("  Location: UNKNOWN - Motor must be homed first"));
            Console.serialInfo(F("  Use 'rail1 home' to establish position reference"));
        } else {
            currentPos = getMotorPositionMm(1);
            Console.serialInfo(F("  Raw Position: "));
            Console.serialInfo(String(currentPos, 2).c_str());
            Console.serialInfo(F(" mm from home"));
            
            // Determine and display meaningful location based on Rail 1 positions
            if (abs(currentPos - RAIL1_HOME_POSITION) < 50.0) {
                Console.serialInfo(F("  Location: AT HOME/HANDOFF (Rail 1-2 transfer)"));
            } else if (abs(currentPos - RAIL1_WC2_PICKUP_DROPOFF) < 50.0) {
                Console.serialInfo(F("  Location: AT WORKCELL 2 (pickup/dropoff)"));
            } else if (abs(currentPos - RAIL1_WC1_PICKUP_DROPOFF) < 50.0) {
                Console.serialInfo(F("  Location: AT WORKCELL 1 (pickup/dropoff)"));
            } else if (abs(currentPos - RAIL1_STAGING_POSITION) < 50.0) {
                Console.serialInfo(F("  Location: AT STAGING POSITION (coordination point)"));
            } else {
                // Determine location based on position ranges using defined constants
                double homeToWc2Midpoint = (RAIL1_HOME_POSITION + RAIL1_WC2_PICKUP_DROPOFF) / 2.0;
                double wc2ToWc1Midpoint = (RAIL1_WC2_PICKUP_DROPOFF + RAIL1_WC1_PICKUP_DROPOFF) / 2.0;
                double wc1ToStagingMidpoint = (RAIL1_WC1_PICKUP_DROPOFF + RAIL1_STAGING_POSITION) / 2.0;
                
                if (currentPos < homeToWc2Midpoint) {
                    Console.serialInfo(F("  Location: BETWEEN HOME AND WC2"));
                } else if (currentPos < wc2ToWc1Midpoint) {
                    Console.serialInfo(F("  Location: BETWEEN WC2 AND WC1"));
                } else if (currentPos < wc1ToStagingMidpoint) {
                    Console.serialInfo(F("  Location: BETWEEN WC1 AND STAGING"));
                } else {
                    Console.serialInfo(F("  Location: BEYOND STAGING (far end of rail)"));
                }
            }
        }
        
        Console.serialInfo(F("============================================"));
        return true;
        
    case 12: // "stop" - Emergency stop motor movement
        // Check if motor is initialized
        if (!isMotorReady(1)) {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(F("Rail 1 motor is not initialized. Cannot perform stop."));
            return false;
        }

        Console.serialInfo(F("EMERGENCY STOP initiated for Rail 1!"));

        // Execute emergency stop
        stopMotion(1);

        Console.acknowledge(F("OK_EMERGENCY_STOP"));
        Console.serialInfo(F("Rail 1 motor movement halted. Position may no longer be accurate."));
        Console.serialInfo(F("Re-homing recommended after emergency stop."));

        return true;
        
    default: // Unknown command
        Console.error(F("Unknown action. Available: init, clear-fault, abort, stop, home, move-wc1, move-wc2, move-staging, move-handoff, move-mm-to, move-rel, status, and help"));
        return false;
    }
    
    return false; // Should never reach here
}