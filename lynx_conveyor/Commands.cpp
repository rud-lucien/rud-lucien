#include "Commands.h"

// External declaration for the logging structure
extern LoggingManagement logging;
extern const unsigned long DEFAULT_LOG_INTERVAL;

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
        Console.println(F("Lynx Conveyor System Command Help:"));
        Console.println(F("--------------------------------------------------"));

        commander.printHelp(caller, true, true);

        Console.println(F("--------------------------------------------------"));
        return true;
    }
}

// Lock a tray or shuttle
bool cmd_lock(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: lock,<1|2|3|shuttle|help>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, ",");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: lock,<1|2|3|shuttle|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Declare all variables at the beginning before switch
    DoubleSolenoidValve *valve = NULL;
    CylinderSensor *sensor = NULL;
    DoubleSolenoidValve *trayValve = NULL;
    CylinderSensor *traySensor = NULL;
    int trayNum = 0;

    // Helper function to map subcommand to an integer for switch
    int cmdCode = 0;
    if (strcmp(subcommand, "all") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "shuttle") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "1") == 0 ||
             strcmp(subcommand, "2") == 0 ||
             strcmp(subcommand, "3") == 0)
    {
        cmdCode = 4;
        trayNum = atoi(subcommand);
    }
    // If none of the above, leave cmdCode as 0 (unknown command)

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "all"
        // Removed "lock all" functionality as requested
        Console.error(F("'lock,all' is not supported for safety reasons. Engage trays individually."));
        return false;

    case 2: // "shuttle"
        if (ccioBoardCount <= 0)
        {
            Console.error(F("No CCIO-8 board detected. Shuttle valve not available."));
            return false;
        }

        Console.info(F("Engaging shuttle with sensor verification..."));
        valve = getShuttleValve();
        sensor = getShuttleSensor();

        if (!valve || !sensor)
        {
            Console.error(F("Failed to access shuttle valve or sensor. Possible causes:"));
            Console.println(F("  - CCIO board detected but shuttle valve not configured"));
            Console.println(F("  - System memory corruption"));
            Console.println(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (valve->position == VALVE_POSITION_LOCK)
        {
            Console.info(F("Shuttle already engaged"));

            // Verify actual position with sensor
            if (sensorRead(*sensor) == true)
            { // Sensor true = locked
                Console.println(F("[OK] Shuttle lock confirmed by sensor"));
            }
            else
            {
                Console.println(F("[WARNING] Shuttle should be locked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try locking with sensor feedback
        Console.info(F("Locking shuttle..."));
        if (safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
        {
            Console.info(F("Shuttle engaged and confirmed by sensor"));
            return true;
        }

        Console.error(F("Failed to engage shuttle - sensor did not confirm lock"));
        Console.println(F("[WARNING] Check air pressure and valve functionality"));
        return false;

    case 3: // "help"
        Console.println(F("\n===== LOCK COMMAND HELP ====="));

        Console.println(F("\nOVERVIEW:"));
        Console.println(F("  The lock command engages pneumatic locks on trays and the shuttle,"));
        Console.println(F("  securing them in position. All operations include sensor verification"));
        Console.println(F("  to confirm successful locking."));

        Console.println(F("\nCOMMAND REFERENCE:"));
        Console.println(F("  lock,1 - Engage lock on tray at position 1 (loading position)"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Will report success only when sensor confirms lock"));

        Console.println(F("  lock,2 - Engage lock on tray at position 2 (middle position)"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Will report success only when sensor confirms lock"));

        Console.println(F("  lock,3 - Engage lock on tray at position 3 (unloading position)"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Will report success only when sensor confirms lock"));

        Console.println(F("  lock,shuttle - Engage lock on the shuttle"));
        Console.println(F("    > Prevents shuttle from moving between positions"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Required before unlocking any trays for safety"));

        Console.println(F("\nSAFETY NOTES:"));
        Console.println(F("  • 'lock,all' is not supported for safety reasons"));
        Console.println(F("  • Always lock the shuttle before unlocking any trays"));
        Console.println(F("  • System uses sensor verification to confirm actual locking"));
        Console.println(F("  • Sufficient pneumatic pressure is required for all valve operations"));
        Console.println(F("  • Failed locking may indicate mechanical issues or low air pressure"));

        Console.println(F("\nSENSOR VERIFICATION:"));
        Console.println(F("  • Each lock has a corresponding sensor that confirms its position"));
        Console.println(F("  • Command waits up to 1 second for sensor to confirm lock"));
        Console.println(F("  • Returns success only when sensor confirms the lock operation"));
        Console.println(F("  • Sensor mismatches are shown in status logs with [!] indicator"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If lock fails, check air pressure"));
        Console.println(F("  • Verify sensor connections if lock command doesn't register"));
        Console.println(F("  • Use 'system,state' to see detailed valve and sensor status"));
        Console.println(F("  • For persistent issues, check valve functionality"));
        Console.println(F("-------------------------------------------"));
        return true;

    case 4: // Tray numbers (1, 2, or 3)
        // Try to parse as tray number
        trayNum = atoi(subcommand);

        Console.print(F("[INFO] Engaging tray "));
        Console.print(trayNum);
        Console.println(F(" with sensor verification..."));

        // Get the appropriate valve and sensor
        switch (trayNum)
        {
        case 1:
            trayValve = getTray1Valve();
            traySensor = getTray1Sensor();
            break;
        case 2:
            trayValve = getTray2Valve();
            traySensor = getTray2Sensor();
            break;
        case 3:
            trayValve = getTray3Valve();
            traySensor = getTray3Sensor();
            break;
        default:
            return false;
        }

        if (!trayValve || !traySensor)
        {
            Console.print(F("[ERROR] Failed to access tray "));
            Console.print(trayNum);
            Console.println(F(" valve or sensor. Possible causes:"));
            Console.println(F("  - Hardware initialization issue"));
            Console.println(F("  - Valve controller not properly initialized"));
            Console.println(F("  - System memory corruption"));
            Console.println(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (trayValve->position == VALVE_POSITION_LOCK)
        {
            Console.print(F("[INFO] Tray "));
            Console.print(trayNum);
            Console.println(F(" already engaged"));

            // Verify actual position with sensor
            if (sensorRead(*traySensor) == true)
            { // Sensor true = locked
                Console.println(F("[OK] Tray lock confirmed by sensor"));
            }
            else
            {
                Console.println(F("[WARNING] Tray should be locked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try locking with sensor feedback
        if (safeValveOperation(*trayValve, *traySensor, VALVE_POSITION_LOCK, 1000))
        {
            Console.print(F("[INFO] Tray "));
            Console.print(trayNum);
            Console.println(F(" engaged and confirmed by sensor"));
            return true;
        }

        Console.print(F("[ERROR] Failed to engage tray "));
        Console.println(trayNum);
        Console.println(F("[WARNING] Check air pressure and valve functionality"));
        return false;

    default: // Unknown command
        Console.print(F("[ERROR] Unknown lock subcommand: "));
        Console.println(subcommand);
        Console.println(F("Valid options are '1', '2', '3', 'shuttle', or 'help'"));
        return false;
    }

    return false; // Should never reach here
}

// Unlock a tray or shuttle
bool cmd_unlock(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: unlock,<1|2|3|shuttle|all|help>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, ",");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: unlock,<1|2|3|shuttle|all|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Declare all variables at the beginning before switch
    DoubleSolenoidValve *valve = NULL;
    CylinderSensor *sensor = NULL;
    DoubleSolenoidValve *trayValve = NULL;
    CylinderSensor *traySensor = NULL;
    int trayNum = 0;

    // Helper function to map subcommand to an integer for switch
    int cmdCode = 0;
    if (strcmp(subcommand, "all") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "shuttle") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "1") == 0 ||
             strcmp(subcommand, "2") == 0 ||
             strcmp(subcommand, "3") == 0)
    {
        cmdCode = 4;
        trayNum = atoi(subcommand);
    }
    // If none of the above, leave cmdCode as 0 (unknown command)

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "all"
        Console.info(F("Disengaging all valves with sensor verification..."));
        if (safeUnlockAllValves(1000))
        {
            Console.info(F("All valves successfully disengaged"));
            return true;
        }
        else
        {
            Console.println(F("[WARNING] Some valves could not be disengaged - check air pressure"));
            return false;
        }
        break;

    case 2: // "shuttle"
        if (ccioBoardCount <= 0)
        {
            Console.error(F("No CCIO-8 board detected. Shuttle valve not available."));
            return false;
        }

        Console.info(F("Disengaging shuttle with sensor verification..."));
        valve = getShuttleValve();
        sensor = getShuttleSensor();

        if (!valve || !sensor)
        {
            Console.error(F("Failed to access shuttle valve or sensor. Possible causes:"));
            Console.println(F("  - CCIO board detected but shuttle valve not configured"));
            Console.println(F("  - System memory corruption"));
            Console.println(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (valve->position == VALVE_POSITION_UNLOCK)
        {
            Console.info(F("Shuttle already disengaged"));

            // Verify actual position with sensor
            if (sensorRead(*sensor) == false)
            { // Sensor false = unlocked
                Console.println(F("[OK] Shuttle unlock confirmed by sensor"));
            }
            else
            {
                Console.println(F("[WARNING] Shuttle should be unlocked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try unlocking with sensor feedback
        Console.info(F("Unlocking shuttle..."));
        if (safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000))
        {
            Console.info(F("Shuttle disengaged and confirmed by sensor"));
            return true;
        }

        Console.error(F("Failed to disengage shuttle - sensor did not confirm unlock"));
        Console.println(F("[WARNING] Check air pressure and valve functionality"));
        return false;
        break;

    case 3: // "help"
        Console.println(F("\n===== UNLOCK COMMAND HELP ====="));

        Console.println(F("\nOVERVIEW:"));
        Console.println(F("  The unlock command disengages pneumatic locks on trays and the shuttle,"));
        Console.println(F("  allowing them to be removed or permitting shuttle movement. All operations"));
        Console.println(F("  include sensor verification to confirm successful unlocking."));

        Console.println(F("\nCOMMAND REFERENCE:"));
        Console.println(F("  unlock,1 - Disengage lock on tray at position 1 (loading position)"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Will report success only when sensor confirms unlock"));

        Console.println(F("  unlock,2 - Disengage lock on tray at position 2 (middle position)"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Will report success only when sensor confirms unlock"));

        Console.println(F("  unlock,3 - Disengage lock on tray at position 3 (unloading position)"));
        Console.println(F("    > Verified by cylinder position sensor"));
        Console.println(F("    > Will report success only when sensor confirms unlock"));

        Console.println(F("  unlock,shuttle - Disengage lock on the shuttle"));
        Console.println(F("    > Allows shuttle to move between positions"));
        Console.println(F("    > Verified by cylinder position sensor"));

        Console.println(F("  unlock,all - Disengage all locks in the system"));
        Console.println(F("    > Emergency recovery function"));
        Console.println(F("    > Uses sensor verification for all valves"));
        Console.println(F("    > Reports success only when all sensors confirm unlock"));

        Console.println(F("\nSAFETY NOTES:"));
        Console.println(F("  • Ensure trays are properly supported before unlocking"));
        Console.println(F("  • System uses sensor verification to confirm actual unlocking"));
        Console.println(F("  • Failed unlocking may indicate mechanical issues"));
        Console.println(F("  • Sufficient pneumatic pressure is required for all valve operations"));

        Console.println(F("\nSENSOR VERIFICATION:"));
        Console.println(F("  • Each lock has a corresponding sensor that confirms its position"));
        Console.println(F("  • Command waits up to 1 second for sensor to confirm unlock"));
        Console.println(F("  • Returns success only when sensor confirms the unlock operation"));
        Console.println(F("  • Sensor mismatches are shown in status logs with [!] indicator"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If unlock fails, check air pressure"));
        Console.println(F("  • Verify sensor connections if unlock command doesn't register"));
        Console.println(F("  • Use 'system,state' to see detailed valve and sensor status"));
        Console.println(F("  • For persistent issues, check valve functionality"));
        Console.println(F("-------------------------------------------"));

        return true;
        break;

    case 4: // Tray numbers (1, 2, or 3)
        // Try to parse as tray number
        trayNum = atoi(subcommand);

        Console.print(F("[INFO] Disengaging tray "));
        Console.print(trayNum);
        Console.println(F(" with sensor verification..."));

        // Get the appropriate valve and sensor
        switch (trayNum)
        {
        case 1:
            trayValve = getTray1Valve();
            traySensor = getTray1Sensor();
            break;
        case 2:
            trayValve = getTray2Valve();
            traySensor = getTray2Sensor();
            break;
        case 3:
            trayValve = getTray3Valve();
            traySensor = getTray3Sensor();
            break;
        default:
            return false;
        }

        if (!trayValve || !traySensor)
        {
            Console.print(F("[ERROR] Failed to access tray "));
            Console.print(trayNum);
            Console.println(F(" valve or sensor. Possible causes:"));
            Console.println(F("  - Hardware initialization issue"));
            Console.println(F("  - Valve controller not properly initialized"));
            Console.println(F("  - System memory corruption"));
            Console.println(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (trayValve->position == VALVE_POSITION_UNLOCK)
        {
            Console.print(F("[INFO] Tray "));
            Console.print(trayNum);
            Console.println(F(" already disengaged"));

            // Verify actual position with sensor
            if (sensorRead(*traySensor) == false)
            { // Sensor false = unlocked
                Console.println(F("[OK] Tray unlock confirmed by sensor"));
            }
            else
            {
                Console.println(F("[WARNING] Tray should be unlocked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try unlocking with sensor feedback
        if (safeValveOperation(*trayValve, *traySensor, VALVE_POSITION_UNLOCK, 1000))
        {
            Console.print(F("[INFO] Tray "));
            Console.print(trayNum);
            Console.println(F(" disengaged and confirmed by sensor"));
            return true;
        }

        Console.print(F("[ERROR] Failed to disengage tray "));
        Console.println(trayNum);
        Console.println(F("[WARNING] Check air pressure and valve functionality"));
        return false;
        break;

    default: // Unknown command
        Console.print(F("[ERROR] Unknown unlock subcommand: "));
        Console.println(subcommand);
        Console.println(F("Valid options are '1', '2', '3', 'shuttle', 'all', or 'help'"));
        return false;
        break;
    }

    return false; // Should never reach here
}

// Log command handler
bool cmd_log(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: log,<on[,interval]|off|now|help>"));
        return false;
    }

    // Parse the first argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: log,<on[,interval]|off|now|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Map subcommand to integer for switch statement
    int cmdCode = 0;
    if (strcmp(subcommand, "on") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "off") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "now") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 4;

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "on"
    {
        // Check if an interval was provided
        char *intervalStr = strtok(NULL, " ");
        unsigned long interval = DEFAULT_LOG_INTERVAL;

        if (intervalStr != NULL)
        {
            // Parse the interval value
            intervalStr = trimLeadingSpaces(intervalStr);
            unsigned long parsedInterval = strtoul(intervalStr, NULL, 10);

            if (parsedInterval > 0)
            {
                interval = parsedInterval;
                Console.print(F("[INFO] Logging enabled with interval of "));
                Console.print(interval);
                Console.println(F(" ms"));
            }
            else
            {
                Console.println(F("[WARNING] Invalid interval. Using default."));
                interval = DEFAULT_LOG_INTERVAL;
            }
        }
        else
        {
            // Use default interval
            Console.print(F("[INFO] Logging enabled with default interval of "));
            Console.print(DEFAULT_LOG_INTERVAL);
            Console.println(F(" ms"));
        }

        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset the timer
        return true;
    }

    case 2: // "off"
    {
        Console.info(F("Logging disabled"));
        logging.logInterval = 0; // Setting to 0 disables logging
        return true;
    }

    case 3: // "now"
    {
        Console.info(F("Logging system state now"));
        // Log immediately regardless of interval
        logSystemState();
        return true;
    }

    case 4: // "help"
    {
        Console.println(F("\n===== LOGGING SYSTEM HELP ====="));

        Console.println(F("\nOVERVIEW:"));
        Console.println(F("  The logging system captures complete system state at regular intervals"));
        Console.println(F("  or on demand, providing detailed information for debugging and monitoring."));

        Console.println(F("\nCOMMAND REFERENCE:"));
        Console.println(F("  log,on[,interval] - Enable periodic logging"));
        Console.println(F("    > interval = Optional logging frequency in milliseconds"));
        Console.println(F("    > Default interval: 250 ms (4 logs per second)"));
        Console.println(F("    > Example: log,on,1000 - Log every 1 second"));
        Console.println(F("    > Example: log,on - Log every 250ms (default)"));

        Console.println(F("  log,off - Disable periodic logging"));
        Console.println(F("    > Stops the automatic logging of system state"));
        Console.println(F("    > Does not affect manual logging with log,now"));

        Console.println(F("  log,now - Log system state immediately"));
        Console.println(F("    > Records a single log entry regardless of periodic settings"));
        Console.println(F("    > Useful for capturing state at specific moments"));

        Console.println(F("\nLOG CONTENT:"));
        Console.println(F("  • Valves - Lock status of all trays and shuttle with sensor verification"));
        Console.println(F("    > [!] indicator shows sensor/command mismatch"));
        Console.println(F("  • Pneumatics - Air pressure status (sufficient/insufficient)"));
        Console.println(F("    > Critical for valve actuation and safe operations"));
        Console.println(F("  • Sensors - Tray presence detection at each position"));
        Console.println(F("  • System - Motor state, homing status, E-Stop and HLFB status"));
        Console.println(F("  • Position - Current, target, and last positions (mm and counts)"));
        Console.println(F("  • Velocity - Current speed, percentage of max, and speed limits"));
        Console.println(F("  • Jog - Current jog increment and speed settings"));
        Console.println(F("  • MPG - Handwheel control status, multiplier, and mm/rotation"));

        Console.println(F("\nPERFORMANCE CONSIDERATIONS:"));
        Console.println(F("  • Default 250ms interval is optimal for most debugging"));
        Console.println(F("  • Very frequent logging (< 100ms) may impact system responsiveness"));
        Console.println(F("  • For long-term monitoring, consider 1000-5000ms intervals"));

        Console.println(F("\nREADING LOG OUTPUT:"));
        Console.println(F("  • Each section is separated by | characters for readability"));
        Console.println(F("  • Position values shown in both mm and encoder counts"));
        Console.println(F("  • Lock status shows ? if sensor doesn't match expected state"));
        Console.println(F("  • Velocity shown with percentage of maximum when moving"));

        Console.println(F("\nTROUBLESHOOTING TIPS:"));
        Console.println(F("  • Use log,now before and after commands to track state changes"));
        Console.println(F("  • Watch for sensor/valve mismatches [!] indicating hardware issues"));
        Console.println(F("  • Compare HLFB status with motor state to identify drive problems"));
        Console.println(F("  • Verify position values match expected targets during movements"));
        Console.println(F("-------------------------------------------"));

        return true;
    }

    default: // Unknown command
    {
        Console.error(F("Invalid log subcommand. Use 'on', 'off', 'now', or 'help'."));
        return false;
    }
    }

    return false; // Should never reach here
}

// Motor command handler for consolidated motor operations
bool cmd_motor(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: motor,<init|status|clear|home|abort|stop|help>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, ",");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: motor,<init|status|clear|home|abort|stop|help>"));
        return false;
    }

    // Trim leading spaces from subcommandn
    subcommand = trimLeadingSpaces(subcommand);

    // Handle motor subcommands using switch for better readability
    int subcommandCode = 0;
    if (strcmp(subcommand, "init") == 0)
        subcommandCode = 1;
    else if (strcmp(subcommand, "status") == 0)
        subcommandCode = 2;
    else if (strcmp(subcommand, "clear") == 0)
        subcommandCode = 3;
    else if (strcmp(subcommand, "home") == 0)
        subcommandCode = 4;
    else if (strcmp(subcommand, "abort") == 0)
        subcommandCode = 5;
    else if (strcmp(subcommand, "stop") == 0)
        subcommandCode = 6;
    else if (strcmp(subcommand, "help") == 0)
        subcommandCode = 7;

    switch (subcommandCode)
    {
    case 1:
    { // init
        Console.info(F("Initializing motor..."));

        // Diagnostic: Print state before initialization
        Console.print(F("[DIAGNOSTIC] Motor state before init: "));
        switch (motorState)
        {
        case MOTOR_STATE_IDLE:
            Console.println(F("IDLE"));
            break;
        case MOTOR_STATE_MOVING:
            Console.println(F("MOVING"));
            break;
        case MOTOR_STATE_HOMING:
            Console.println(F("HOMING"));
            break;
        case MOTOR_STATE_FAULTED:
            Console.println(F("FAULTED"));
            break;
        case MOTOR_STATE_NOT_READY:
            Console.println(F("NOT READY"));
            break;
        default:
            Console.println(F("UNKNOWN"));
            break;
        }

        initMotorSystem();

        // Diagnostic: Print state after initialization
        Console.print(F("[DIAGNOSTIC] Motor state after init: "));
        switch (motorState)
        {
        case MOTOR_STATE_IDLE:
            Console.println(F("IDLE"));
            break;
        case MOTOR_STATE_MOVING:
            Console.println(F("MOVING"));
            break;
        case MOTOR_STATE_HOMING:
            Console.println(F("HOMING"));
            break;
        case MOTOR_STATE_FAULTED:
            Console.println(F("FAULTED"));
            break;
        case MOTOR_STATE_NOT_READY:
            Console.println(F("NOT READY"));
            break;
        default:
            Console.println(F("UNKNOWN"));
            break;
        }

        if (motorState == MOTOR_STATE_NOT_READY || motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor initialization failed. Check connections and power."));
            return false;
        }
        else
        {
            Console.info(F("Motor initialization successful"));
            return true;
        }
        break;
    }

    case 2:
    { // status
        Console.info(F("Motor Status:"));

        // Display motor state
        Console.print(F("  State: "));
        switch (motorState)
        {
        case MOTOR_STATE_IDLE:
            Console.println(F("IDLE"));
            break;
        case MOTOR_STATE_MOVING:
            Console.println(F("MOVING"));
            break;
        case MOTOR_STATE_HOMING:
            Console.println(F("HOMING"));
            break;
        case MOTOR_STATE_FAULTED:
            Console.println(F("FAULTED"));
            break;
        case MOTOR_STATE_NOT_READY:
            Console.println(F("NOT READY"));
            break;
        default:
            Console.println(F("UNKNOWN"));
            break;
        }

        // Display homing status
        Console.print(F("  Homed: "));
        Console.println(isHomed ? F("YES") : F("NO"));

        // Display position information based on homing status
        if (isHomed)
        {
            double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
            int32_t rawPosition = MOTOR_CONNECTOR.PositionRefCommanded();
            int32_t normalizedPosition = normalizeEncoderValue(rawPosition);

            Console.print(F("  Current Position: "));
            Console.print(calculatedPositionMm, 2);
            Console.print(F(" mm ("));
            Console.print(normalizedPosition);
            Console.println(F(" counts)"));

            // Add last completed position display using existing variables
            Console.print(F("  Last Completed Position: "));
            if (hasLastTarget)
            {
                Console.print(lastTargetPositionMm, 2);
                Console.print(F(" mm ("));
                Console.print(normalizeEncoderValue(lastTargetPulses));
                Console.println(F(" counts)"));
            }
            else
            {
                Console.println(F("None - No movements completed yet"));
            }
        }
        else
        {
            Console.println(F("  Current Position: UNKNOWN - Motor not homed"));
            Console.println(F("  Last Completed Position: UNKNOWN - Motor not homed"));

            // Only show raw encoder count if motor is initialized
            if (motorState != MOTOR_STATE_NOT_READY)
            {
                int32_t rawPosition = MOTOR_CONNECTOR.PositionRefCommanded();
                Console.print(F("  Encoder Reading: "));
                // Also normalize here
                Console.print(normalizeEncoderValue(rawPosition));
                Console.println(F(" counts (reference point not established)"));
            }
            else
            {
                Console.println(F("  Encoder Reading: Not available - Motor not initialized"));
            }
        }

        // Show velocity configuration instead of current velocity
        Console.println(F("  Velocity Settings:"));

        // Regular movement velocity
        Console.print(F("    Move Operations: "));
        Console.print(ppsToRpm(currentVelMax), 1);
        Console.println(F(" RPM"));

        // Homing velocity - updated to show only the approach velocity we actually use
        Console.print(F("    Homing: "));
        Console.print(HOME_APPROACH_VELOCITY_RPM);
        Console.println(F(" RPM"));

        // Jog velocity and increment
        Console.print(F("    Jogging: "));
        Console.print(currentJogSpeedRpm);
        Console.print(F(" RPM, "));
        Console.print(currentJogIncrementMm, 2);
        Console.println(F(" mm/jog"));

        // Only show current velocity if motor is moving
        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            int32_t velocity = MOTOR_CONNECTOR.VelocityRefCommanded();
            double velocityRpm = (double)velocity * 60.0 / PULSES_PER_REV;
            Console.print(F("    Current: "));
            Console.print(velocityRpm, 1);
            Console.print(F(" RPM ("));
            Console.print(velocity);
            Console.println(F(" pulses/sec)"));
        }

        // Display acceleration limit
        Console.print(F("  Acceleration: "));
        Console.print((double)currentAccelMax * 60.0 / PULSES_PER_REV, 1);
        Console.println(F(" RPM/sec"));

        // Display travel limits based on homing status
        Console.println(F("  Travel Limits:"));
        if (isHomed)
        {
            // Display both mm and counts when homed
            Console.print(F("    Range: 0.00 to "));
            Console.print(MAX_TRAVEL_MM, 2);
            Console.println(F(" mm"));
            Console.print(F("            0 to "));
            // MAX_TRAVEL_PULSES is already defined with the correct sign
            Console.print(MAX_TRAVEL_PULSES);
            Console.println(F(" counts"));
        }
        else
        {
            Console.println(F("    UNKNOWN - Motor not homed"));
        }

        // Display fault status
        Console.print(F("  Fault Status: "));
        if (MOTOR_CONNECTOR.HlfbState() == ClearCore::MotorDriver::HLFB_ASSERTED)
        {
            Console.println(F("NO FAULT"));
        }
        else
        {
            Console.println(F("FAULT DETECTED"));
        }

        // Display E-Stop status
        Console.print(F("  E-Stop: "));
        Console.println(isEStopActive() ? F("TRIGGERED (EMERGENCY STOP)") : F("RELEASED (READY)"));

        // If motor is not ready, provide additional information for troubleshooting
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            Console.println(F("\n  [NOTE] Motor must be initialized with 'motor,init' command"));
            Console.println(F("         before position control is available."));
        }

        return true;
        break;
    }

    case 3:
    { // clear
        Console.info(F("Attempting to clear motor fault..."));

        if (clearMotorFaultWithStatus())
        {
            Console.info(F("Motor fault cleared successfully"));
            return true;
        }
        else
        {
            Console.error(F("Failed to clear motor fault. Motor may still be in fault state."));
            Console.println(F("  Try power cycling the system if fault persists."));
            return false;
        }
        break;
    }

    case 4:
    { // home
        // Check necessary preconditions for homing
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            Console.error(F("Motor is not initialized. Use 'motor,init' first."));
            return false;
        }

        if (motorState == MOTOR_STATE_HOMING)
        {
            Console.println(F("[WARNING] Homing sequence is already in progress."));
            return false;
        }

        if (isEStopActive())
        {
            Console.error(F("Cannot home while E-Stop is active. Release E-Stop and try again."));
            return false;
        }

        if (motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor is in fault state. Use 'motor,clear' to clear fault before homing."));
            return false;
        }

        Console.info(F("Starting homing sequence..."));

        // Begin homing
        initiateHomingSequence();

        // Check if homing was initiated by examining the motor state
        if (motorState == MOTOR_STATE_HOMING)
        {
            Console.info(F("Homing sequence initiated. Motor will move to find home position."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start homing sequence. Check motor status."));
            return false;
        }
        break;
    }

    case 5:
    { // abort
        // Check if motor is initialized before attempting to abort
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            Console.error(F("Motor is not initialized. Nothing to abort."));
            return false;
        }

        Console.info(F("Aborting current operation..."));

        // Only meaningful to abort if we're moving or homing
        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            if (motorState == MOTOR_STATE_HOMING)
            {
                abortHoming();
            }
            else
            {
                MOTOR_CONNECTOR.MoveStopAbrupt();
            }

            // Update motor state
            motorState = MOTOR_STATE_IDLE;

            Console.info(F("Operation aborted successfully."));
            return true;
        }
        else
        {
            Console.println(F("[WARNING] No active operation to abort."));
            return false;
        }
        break;
    }

    case 6:
    { // stop
        // Check if motor is initialized
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            Console.error(F("Motor is not initialized. Cannot perform stop."));
            return false;
        }

        Console.info(F("EMERGENCY STOP initiated!"));

        // Execute emergency stop
        MOTOR_CONNECTOR.MoveStopAbrupt();
        motorState = MOTOR_STATE_IDLE;

        Console.info(F("Motor movement halted. Position may no longer be accurate."));
        Console.println(F("[WARNING] Re-homing recommended after emergency stop."));

        return true;
        break;
    }

    case 7: // help
    {
        Console.println(F("\n===== MOTOR CONTROL SYSTEM HELP ====="));

        Console.println(F("\nCOMMAND REFERENCE:"));
        Console.println(F("  motor,init - Initialize motor system and prepare for operation"));
        Console.println(F("    > Must be run after power-up before any other motor commands"));
        Console.println(F("    > Configures motor parameters and communication"));
        Console.println(F("    > Does not move the motor or establish position reference"));

        Console.println(F("  motor,home - Find home position and establish reference point"));
        Console.println(F("    > Required before absolute positioning commands can be used"));
        Console.println(F("    > Motor will move slowly until it contacts the home limit switch"));
        Console.println(F("    > After contact, motor backs off to establish precise zero position"));
        Console.println(F("    > Home position is offset 5mm from physical limit for safety"));

        Console.println(F("  motor,status - Display detailed motor status and configuration"));
        Console.println(F("    > Shows current state, position, velocity settings, and limits"));
        Console.println(F("    > Use to verify proper operation or troubleshoot issues"));

        Console.println(F("  motor,clear - Clear motor fault condition"));
        Console.println(F("    > Use after resolving the condition that caused the fault"));
        Console.println(F("    > Common faults: excessive load, hitting physical limit, E-Stop"));

        Console.println(F("  motor,abort - Gracefully stop current movement"));
        Console.println(F("    > Controlled deceleration to stop the motor"));
        Console.println(F("    > Position information is maintained"));
        Console.println(F("    > Use to cancel a movement without generating a fault"));

        Console.println(F("  motor,stop - Emergency stop motor movement immediately"));
        Console.println(F("    > Immediate halt of motor operation"));
        Console.println(F("    > May cause position inaccuracy"));
        Console.println(F("    > Use only when necessary to prevent damage or injury"));

        Console.println(F("\nTYPICAL SEQUENCE:"));
        Console.println(F("  1. motor,init   - Initialize the motor system"));
        Console.println(F("  2. motor,home   - Establish reference position"));
        Console.println(F("  3. move,X       - Move to desired positions"));
        Console.println(F("  4. jog commands - Make fine adjustments"));
        Console.println(F("  5. encoder      - Use handwheel for manual control"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If motor won't move: Check E-Stop, then run motor,status"));
        Console.println(F("  • After fault: Use motor,clear to reset fault condition"));
        Console.println(F("  • If position seems incorrect: Re-home the system"));
        Console.println(F("  • Unexpected behavior: Check that motor is initialized"));
        Console.println(F("  • Jerky movement: Try using slower speed or smaller increments"));

        Console.println(F("\nSAFETY NOTES:"));
        Console.println(F("  • Always ensure proper clearance before moving the shuttle"));
        Console.println(F("  • Use E-Stop if unexpected movement occurs"));
        Console.println(F("  • After E-Stop, clear faults before resuming operation"));
        Console.println(F("  • Motor movements will halt automatically at travel limits"));
        Console.println(F("-------------------------------------------"));

        return true;
        break;
    }

    default:
    {
        Console.print(F("[ERROR] Unknown motor command: "));
        Console.println(subcommand);
        Console.println(F("Valid options are 'init', 'status', 'clear', 'home', 'abort', 'stop', or 'help'"));
        return false;
        break;
    }
    }

    return false; // Should never reach here, but included for completeness
}

// Move command handler with support for absolute encoder counts
bool cmd_move(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: move,<home|1|2|3|4|counts,X|mm,X|rel,X|help>"));
        return false;
    }

    // Get the first parameter (subcommand position)
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: move,<home|1|2|3|4|counts,X|mm,X|rel,X|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // State checks (in order of importance) for all movement commands
    // 1. Check if motor is initialized
    if (motorState == MOTOR_STATE_NOT_READY)
    {
        Console.error(F("Motor is not initialized. Use 'motor,init' first."));
        return false;
    }

    // 2. Check if E-Stop is active - most critical safety check
    if (isEStopActive())
    {
        Console.error(F("Cannot move while E-Stop is active. Release E-Stop and try again."));
        return false;
    }

    // 3. Check for fault condition
    if (motorState == MOTOR_STATE_FAULTED)
    {
        Console.error(F("Motor is in fault state. Use 'motor,clear' to clear fault before moving."));
        return false;
    }

    // 4. Check if motor is already moving
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
    {
        Console.error(F("Motor is already moving. Use 'motor,abort' to stop current movement first."));
        return false;
    }

    // Map subcommand to integer for switch statement
    int cmdCode = 0;
    if (strcmp(subcommand, "home") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "1") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "2") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "3") == 0)
        cmdCode = 4;
    else if (strcmp(subcommand, "4") == 0)
        cmdCode = 5;
    else if (strcmp(subcommand, "mm") == 0)
        cmdCode = 6;
    else if (strcmp(subcommand, "counts") == 0)
        cmdCode = 7;
    else if (strcmp(subcommand, "rel") == 0)
        cmdCode = 8;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 9;

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "home"
    {
        // Check if motor is already homed
        if (isHomed)
        {
            Console.info(F("Moving to home position..."));
            if (moveToPositionMm(0.0))
            {
                Console.info(F("Move to home initiated."));
                return true;
            }
            else
            {
                Console.error(F("Failed to start movement to home position."));
                return false;
            }
        }
        else
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first to establish home position."));
            return false;
        }
        break;
    }

    case 2: // "1" (Position 1)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        Console.info(F("Moving to position 1..."));
        if (moveToPosition(POSITION_1))
        {
            Console.info(F("Move to position 1 initiated."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 1."));
            return false;
        }
        break;
    }

    case 3: // "2" (Position 2)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        Console.info(F("Moving to position 2..."));
        if (moveToPosition(POSITION_2))
        {
            Console.info(F("Move to position 2 initiated."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 2."));
            return false;
        }
        break;
    }

    case 4: // "3" (Position 3)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        Console.info(F("Moving to position 3..."));
        if (moveToPosition(POSITION_3))
        {
            Console.info(F("Move to position 3 initiated."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 3."));
            return false;
        }
        break;
    }

    case 5: // "4" (Position 4)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        Console.info(F("Moving to position 4..."));
        if (moveToPosition(POSITION_4))
        {
            Console.info(F("Move to position 4 initiated."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 4."));
            return false;
        }
        break;
    }

    case 6: // "mm" (absolute position in mm)
    {
        // Get the mm value from the next token
        char *mmStr = strtok(NULL, " ");
        if (mmStr == NULL)
        {
            Console.error(F("Missing mm value. Usage: move,mm,X"));
            return false;
        }

        // Parse the mm value
        mmStr = trimLeadingSpaces(mmStr);
        double targetMm = atof(mmStr);

        // Check if motor is homed for precise positioning
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            Console.println(F("[WARNING] Moving to absolute positions without homing is unsafe."));
            return false;
        }

        // Check if position is within bounds
        if (targetMm < 0.0 || targetMm > MAX_TRAVEL_MM)
        {
            Console.print(F("[ERROR] Position out of range. Valid range: 0 to "));
            Console.print(MAX_TRAVEL_MM, 1);
            Console.println(F(" mm"));
            return false;
        }

        // Position is within bounds, proceed with movement
        Console.print(F("[INFO] Moving to absolute position: "));
        Console.print(targetMm, 2);
        Console.println(F(" mm"));

        if (moveToPositionMm(targetMm))
        {
            Console.info(F("Movement initiated successfully."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to requested position."));
            return false;
        }
        break;
    }

    case 7: // "counts" (absolute position in encoder counts)
    {
        // Get the counts value from the next token
        char *countsStr = strtok(NULL, " ");
        if (countsStr == NULL)
        {
            Console.error(F("Missing counts value. Usage: move,counts,X"));
            return false;
        }

        // Parse the counts value
        countsStr = trimLeadingSpaces(countsStr);
        int32_t targetCounts = atol(countsStr);

        // Check if motor is homed for precise positioning
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            Console.println(F("[WARNING] Moving to absolute positions without homing is unsafe."));
            return false;
        }

        // Check if position is within bounds
        if (targetCounts < 0 || targetCounts > MAX_TRAVEL_PULSES)
        {
            Console.print(F("[ERROR] Position out of range. Valid range: 0 to "));
            Console.print(MAX_TRAVEL_PULSES);
            Console.println(F(" counts"));
            return false;
        }

        // Position is within bounds, proceed with movement
        Console.print(F("[INFO] Moving to absolute position: "));
        Console.print(targetCounts);
        Console.println(F(" counts"));

        if (moveToAbsolutePosition(targetCounts))
        {
            Console.info(F("Movement initiated successfully."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to requested position."));
            return false;
        }
        break;
    }

    case 8: // "rel" (relative position in mm)
    {
        // Get the mm value from the next token
        char *relStr = strtok(NULL, " ");
        if (relStr == NULL)
        {
            Console.error(F("Missing relative distance value. Usage: move,rel,X"));
            return false;
        }

        // Parse the relative distance value
        relStr = trimLeadingSpaces(relStr);
        double relDistanceMm = atof(relStr);

        // Check if motor is homed for precise positioning
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            Console.println(F("[WARNING] Moving without homing is unsafe."));
            return false;
        }

        // Get current position
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm + relDistanceMm;

        // Check if target position is within bounds
        if (targetPositionMm < 0.0 || targetPositionMm > MAX_TRAVEL_MM)
        {
            Console.print(F("[ERROR] Target position out of range. Valid range: 0 to "));
            Console.print(MAX_TRAVEL_MM, 1);
            Console.println(F(" mm"));
            Console.print(F("[INFO] Current position: "));
            Console.print(currentPositionMm, 2);
            Console.print(F(" mm, Requested move: "));
            Console.print(relDistanceMm, 2);
            Console.println(F(" mm"));
            return false;
        }

        // Display move information
        Console.print(F("[INFO] Moving "));
        Console.print(relDistanceMm, 2);
        Console.print(F(" mm from current position ("));
        Console.print(currentPositionMm, 2);
        Console.print(F(" mm) to "));
        Console.print(targetPositionMm, 2);
        Console.println(F(" mm"));

        if (moveToPositionMm(targetPositionMm))
        {
            Console.info(F("Relative movement initiated successfully."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start relative movement."));
            return false;
        }
        break;
    }

    case 9: // "help"
    {
        Console.println(F("\n===== MOVE COMMAND HELP ====="));

        Console.println(F("\nPREREQUISITES:"));
        Console.println(F("  • Motor must be initialized (motor,init)"));
        Console.println(F("  • Motor must be homed for accurate positioning (motor,home)"));
        Console.println(F("  • E-Stop must be inactive"));
        Console.println(F("  • Motor must not be in fault state"));
        Console.println(F("  • No other movement can be in progress"));

        Console.println(F("\nCOMMAND TYPES:"));
        Console.println(F("  move,home - Move to home (zero) position"));
        Console.println(F("    > Reference position offset 5mm from hardstop"));
        Console.println(F("    > Always available after homing"));

        Console.println(F("  move,1 through move,4 - Move to predefined positions"));
        Console.println(F("    > Position 1: Loading position (28.7mm)"));
        Console.println(F("    > Position 2: Middle position (456.0mm)"));
        Console.println(F("    > Position 3: Unloading position (883.58mm)"));
        Console.println(F("    > Position 4: Max travel (1050.0mm)"));

        Console.println(F("  move,mm,X - Move to absolute position X in millimeters"));
        Console.println(F("    > Valid range: 0 to 1050.0 mm"));
        Console.println(F("    > Most intuitive way to specify exact positions"));
        Console.println(F("    > Example: move,mm,500.5 - moves to 500.5mm"));

        Console.println(F("  move,counts,X - Move to absolute position X in encoder counts"));
        Console.println(F("    > Valid range: 0 to 64,333 counts"));
        Console.println(F("    > Used for precise control or debugging"));
        Console.println(F("    > 1mm ≈ 61.27 counts (3200 pulses/rev ÷ 52.23mm/rev)"));

        Console.println(F("  move,rel,X - Move X millimeters relative to current position"));
        Console.println(F("    > Use positive values to move forward"));
        Console.println(F("    > Use negative values to move backward"));
        Console.println(F("    > Example: move,rel,-10 - moves 10mm backward"));
        Console.println(F("    > Movement is constrained to valid range (0-1050.0mm)"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If movement fails, check motor status with 'motor,status'"));
        Console.println(F("  • If at travel limits, you can only move within the allowed range"));
        Console.println(F("  • After E-Stop, clear faults with 'motor,clear' before moving"));
        Console.println(F("  • For short, precise movements, consider using 'jog' commands"));
        Console.println(F("  • For interactive positioning, use 'encoder' handwheel control"));
        Console.println(F("-------------------------------------------"));

        return true;
        break;
    }

    default: // Unknown command
    {
        Console.print(F("[ERROR] Invalid position: "));
        Console.println(subcommand);
        Console.println(F("Valid options: home, 1, 2, 3, 4, counts, mm, rel, help"));
        return false;
        break;
    }
    }

    return false; // Should never reach here, but included for completeness
}

// Jog command handler
bool cmd_jog(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: jog,<+|-|inc|speed|status|help>"));
        return false;
    }

    // Parse the argument - we'll use spaces as separators (as they've been converted from commas)
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: jog,<+|-|inc|speed|status|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // State checks (in order of importance) for movement commands
    if (strcmp(subcommand, "+") == 0 || strcmp(subcommand, "-") == 0)
    {
        // 1. Check if motor is initialized
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            Console.error(F("Motor is not initialized. Use 'motor,init' first."));
            return false;
        }

        // 2. Check if E-Stop is active - most critical safety check
        if (isEStopActive())
        {
            Console.error(F("Cannot jog while E-Stop is active. Release E-Stop and try again."));
            return false;
        }

        // 3. Check for fault condition
        if (motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor is in fault state. Use 'motor,clear' to clear fault before jogging."));
            return false;
        }

        // 4. Check if motor is homing
        if (motorState == MOTOR_STATE_HOMING)
        {
            Console.error(F("Cannot jog while homing is in progress."));
            return false;
        }

        // 5. Check if motor is already moving
        if (motorState == MOTOR_STATE_MOVING)
        {
            Console.error(F("Motor is already moving. Use 'motor,abort' to stop current movement first."));
            return false;
        }

        // 6. Check if motor is homed
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first."));
            return false;
        }
    }

    // Handle jog subcommands using switch for better readability
    int subcommandCode = 0;
    if (strcmp(subcommand, "+") == 0)
        subcommandCode = 1;
    else if (strcmp(subcommand, "-") == 0)
        subcommandCode = 2;
    else if (strcmp(subcommand, "inc") == 0)
        subcommandCode = 3;
    else if (strcmp(subcommand, "speed") == 0)
        subcommandCode = 4;
    else if (strcmp(subcommand, "status") == 0)
        subcommandCode = 5;
    else if (strcmp(subcommand, "help") == 0)
        subcommandCode = 6;

    switch (subcommandCode)
    {
    case 1:
    { // jog forward (+)
        // Calculate the target position by adding the jog increment to the current position
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm + currentJogIncrementMm;

        // Check if target position is within bounds
        if (targetPositionMm > MAX_TRAVEL_MM)
        {
            Console.print(F("[ERROR] Cannot jog beyond maximum position limit of "));
            Console.print(MAX_TRAVEL_MM, 1);
            Console.println(F(" mm"));
            Console.print(F("  Current position: "));
            Console.print(currentPositionMm, 2);
            Console.println(F(" mm"));
            return false;
        }

        // Display jog information
        Console.print(F("[INFO] Jogging forward "));
        Console.print(currentJogIncrementMm, 2);
        Console.print(F(" mm from position "));
        Console.print(currentPositionMm, 2);
        Console.print(F(" mm to "));
        Console.print(targetPositionMm, 2);
        Console.println(F(" mm"));

        // Perform the jog movement using the jogMotor function
        if (jogMotor(true))
        { // true = forward direction
            Console.info(F("Jog movement initiated"));
            return true;
        }
        else
        {
            Console.error(F("Failed to initiate jog movement"));
            return false;
        }
        break;
    }

    case 2:
    { // jog backward (-)
        // Calculate the target position by subtracting the jog increment from the current position
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm - currentJogIncrementMm;

        // Check if target position is within bounds
        if (targetPositionMm < 0.0)
        {
            Console.error(F("Cannot jog beyond minimum position limit of 0 mm"));
            Console.print(F("  Current position: "));
            Console.print(currentPositionMm, 2);
            Console.println(F(" mm"));
            return false;
        }

        // Display jog information
        Console.print(F("[INFO] Jogging backward "));
        Console.print(currentJogIncrementMm, 2);
        Console.print(F(" mm from position "));
        Console.print(currentPositionMm, 2);
        Console.print(F(" mm to "));
        Console.print(targetPositionMm, 2);
        Console.println(F(" mm"));

        // Perform the jog movement using the jogMotor function
        if (jogMotor(false))
        { // false = backward direction
            Console.info(F("Jog movement initiated"));
            return true;
        }
        else
        {
            Console.error(F("Failed to initiate jog movement"));
            return false;
        }
        break;
    }

    case 3:
    { // set/get increment
        // Check if a value was provided
        char *incStr = strtok(NULL, " ");
        if (incStr == NULL)
        {
            // Just display the current increment
            Console.print(F("[INFO] Current jog increment: "));
            Console.print(currentJogIncrementMm, 2);
            Console.println(F(" mm"));
            return true;
        }
        else
        {
            // Trim leading spaces from increment value
            incStr = trimLeadingSpaces(incStr);

            // Handle "default" keyword
            if (strcmp(incStr, "default") == 0)
            {
                if (setJogIncrement(DEFAULT_JOG_INCREMENT))
                {
                    Console.print(F("[INFO] Jog increment set to default ("));
                    Console.print(currentJogIncrementMm, 2);
                    Console.println(F(" mm)"));
                    return true;
                }
                else
                {
                    Console.error(F("Failed to set default jog increment"));
                    return false;
                }
            }

            // Parse value as double
            double newIncrement = atof(incStr);

            // Set new jog increment
            if (setJogIncrement(newIncrement))
            {
                Console.print(F("[INFO] Jog increment set to "));
                Console.print(currentJogIncrementMm, 2);
                Console.println(F(" mm"));
                return true;
            }
            else
            {
                Console.error(F("Invalid jog increment value"));
                return false;
            }
        }
        break;
    }

    case 4:
    { // set/get speed
        // Check if a value was provided
        char *speedStr = strtok(NULL, " ");
        if (speedStr == NULL)
        {
            // Just display the current speed
            Console.print(F("[INFO] Current jog speed: "));
            Console.print(currentJogSpeedRpm);
            Console.println(F(" RPM"));
            return true;
        }
        else
        {
            // Trim leading spaces from speed value
            speedStr = trimLeadingSpaces(speedStr);

            // Handle "default" keyword
            if (strcmp(speedStr, "default") == 0)
            {
                if (setJogSpeed(DEFAULT_JOG_SPEED, currentJogIncrementMm))
                {
                    Console.print(F("[INFO] Jog speed set to default ("));
                    Console.print(currentJogSpeedRpm);
                    Console.println(F(" RPM)"));
                    return true;
                }
                else
                {
                    Console.error(F("Failed to set default jog speed"));
                    return false;
                }
            }

            // Parse value as int
            int newSpeed = atoi(speedStr);

            // Set new jog speed
            if (setJogSpeed(newSpeed, currentJogIncrementMm))
            {
                Console.print(F("[INFO] Jog speed set to "));
                Console.print(currentJogSpeedRpm);
                Console.println(F(" RPM"));
                return true;
            }
            else
            {
                Console.error(F("Invalid jog speed value"));
                return false;
            }
        }
        break;
    }

    case 5:
    { // status
        // Display current jog settings
        Console.info(F("Current jog settings:"));

        // Jog increment
        Console.print(F("  Increment: "));
        Console.print(currentJogIncrementMm, 2);
        Console.println(F(" mm"));

        // Jog speed
        Console.print(F("  Speed: "));
        Console.print(currentJogSpeedRpm);
        Console.println(F(" RPM"));

        // Current position
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        Console.print(F("  Current position: "));
        Console.print(currentPositionMm, 2);
        Console.println(F(" mm"));

        // Position limits for jogging from current position
        double maxForwardJog = MAX_TRAVEL_MM - currentPositionMm;
        double maxBackwardJog = currentPositionMm;

        Console.print(F("  Max forward jog: "));
        Console.print(maxForwardJog, 2);
        Console.println(F(" mm"));

        Console.print(F("  Max backward jog: "));
        Console.print(maxBackwardJog, 2);
        Console.println(F(" mm"));

        return true;
        break;
    }

    case 6: // Add the new help case here
    {       // help
        Console.println(F("\n===== JOG MOVEMENT SYSTEM HELP ====="));

        Console.println(F("\nOVERVIEW:"));
        Console.println(F("  The jog system provides precise, incremental movements in either direction"));
        Console.println(F("  for accurate positioning and testing. Each jog moves the motor by a fixed"));
        Console.println(F("  distance that you can configure."));

        Console.println(F("\nCOMMAND REFERENCE:"));
        Console.println(F("  jog,+ - Move forward by one increment"));
        Console.println(F("    > Each press moves exactly one increment in the forward direction"));
        Console.println(F("    > Movement stops automatically after the increment is completed"));
        Console.println(F("  jog,- - Move backward by one increment"));
        Console.println(F("    > Each press moves exactly one increment in the backward direction"));
        Console.println(F("    > Movement stops automatically after the increment is completed"));
        Console.println(F("  jog,inc,X - Set movement increment size"));
        Console.println(F("    > X = distance in millimeters (example: jog,inc,5.0)"));
        Console.println(F("    > Using jog,inc without a value displays the current setting"));
        Console.println(F("    > Using jog,inc,default resets to standard increment"));
        Console.println(F("  jog,speed,X - Set movement speed"));
        Console.println(F("    > X = speed in RPM (example: jog,speed,300)"));
        Console.println(F("    > Using jog,speed without a value displays the current setting"));
        Console.println(F("    > Using jog,speed,default resets to standard speed"));
        Console.println(F("  jog,status - Display current jog settings and position information"));

        Console.println(F("\nJOG VS. HANDWHEEL COMPARISON:"));
        Console.println(F("  Jog System (jog command):"));
        Console.println(F("    • Fixed, precise movements with each command"));
        Console.println(F("    • Better for repeatable, exact positioning"));
        Console.println(F("    • Simple to use via command line"));
        Console.println(F("    • Good for testing and calibration"));
        Console.println(F("    • Can be used in scripts and automated sequences"));

        Console.println(F("  Handwheel System (encoder command):"));
        Console.println(F("    • Continuous, manual control with physical handwheel"));
        Console.println(F("    • Better for interactive positioning and fine adjustments"));
        Console.println(F("    • More intuitive for operators doing manual work"));
        Console.println(F("    • Allows variable speed based on rotation speed"));
        Console.println(F("    • Provides tactile feedback during positioning"));

        Console.println(F("\nWHEN TO USE JOG:"));
        Console.println(F("  • For test sequences that need repeatable movements"));
        Console.println(F("  • When working remotely via serial connection"));
        Console.println(F("  • When you need precisely measured movements"));
        Console.println(F("  • For calibration procedures"));
        Console.println(F("  • When you don't have access to the physical handwheel"));

        Console.println(F("\nUSAGE TIPS:"));
        Console.println(F("  • Set a smaller increment (1-5mm) for precise positioning"));
        Console.println(F("  • Set a larger increment (10-50mm) for faster travel"));
        Console.println(F("  • Use jog,status to see your current position and limits"));
        Console.println(F("  • The motor must be homed before jogging can be used"));
        Console.println(F("  • Jogging is automatically limited to prevent over-travel"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If jog commands fail, check if motor is initialized and homed"));
        Console.println(F("  • If at travel limit, you can only jog in the opposite direction"));
        Console.println(F("  • After E-Stop, clear any faults before attempting to jog"));
        Console.println(F("  • If motor is already moving, wait for it to complete or use motor,abort"));
        Console.println(F("-------------------------------------------"));

        return true;
        break;
    }

    default:
    {
        Console.print(F("[ERROR] Unknown jog command: "));
        Console.println(subcommand);
        Console.println(F("Valid options are '+', '-', 'inc', 'speed', 'status', or 'help'")); // Update this line too
        return false;
        break;
    }
    }

    return false; // Should never reach here, but included for completeness
}

bool cmd_system_state(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Determine the subcommand
    char *subcommand = strtok(trimmed, " ");
    // If no subcommand provided, display usage
    if (subcommand == NULL || strlen(subcommand) == 0)
    {
        Console.info(F("Usage: system,state - Display current system state"));
        Console.println(F("                 system,safety - Display safety validation status"));
        Console.println(F("                 system,trays - Display tray system status"));
        Console.println(F("                 system,network - Display Ethernet interface status"));
        Console.println(F("                 system,reset - Reset system state after failure"));
        return false;
    }

    // Map subcommand to integer for switch statement
    int cmdCode = 0;
    if (strcmp(subcommand, "state") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "safety") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "trays") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "network") == 0)
        cmdCode = 4;
    else if (strcmp(subcommand, "reset") == 0)
        cmdCode = 5;

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "state"
    {
        // Capture and print system state
        SystemState currentState = captureSystemState();
        printSystemState(currentState);
        return true;
    }

    case 2: // "safety"
    {
        // Capture system state, validate safety, and print results
        SystemState currentState = captureSystemState();
        SafetyValidationResult safety = validateSafety(currentState);

        Console.println(F("\n===== SAFETY VALIDATION STATUS ====="));
        printSafetyStatus(safety);

        return true;
    }

    case 3: // "trays"
    {
        // Display tray system status
        SystemState currentState = captureSystemState();
        updateTrayTrackingFromSensors(currentState);

        Console.println(F("\n===== TRAY SYSTEM STATUS ====="));
        Console.print(F("Total trays in system: "));
        Console.println(trayTracking.totalTraysInSystem);

        Console.println(F("\nPosition occupancy:"));
        Console.print(F("  Position 1 (Loading): "));
        Console.println(trayTracking.position1Occupied ? F("OCCUPIED") : F("EMPTY"));
        Console.print(F("  Position 2 (Middle): "));
        Console.println(trayTracking.position2Occupied ? F("OCCUPIED") : F("EMPTY"));
        Console.print(F("  Position 3 (Unloading): "));
        Console.println(trayTracking.position3Occupied ? F("OCCUPIED") : F("EMPTY"));

        Console.println(F("\nOperation statistics:"));
        Console.print(F("  Total loads completed: "));
        Console.println(trayTracking.totalLoadsCompleted);
        Console.print(F("  Total unloads completed: "));
        Console.println(trayTracking.totalUnloadsCompleted);

        if (trayTracking.lastLoadTime > 0)
        {
            Console.print(F("  Last load: "));
            Console.print((millis() - trayTracking.lastLoadTime) / 1000);
            Console.println(F(" seconds ago"));
        }

        if (trayTracking.lastUnloadTime > 0)
        {
            Console.print(F("  Last unload: "));
            Console.print((millis() - trayTracking.lastUnloadTime) / 1000);
            Console.println(F(" seconds ago"));
        }
        return true;
    }

    case 4: // "network"
    {
        Console.println(F("\n===== ETHERNET INTERFACE STATUS ====="));

        // Show initialization status
        Console.print(F("Ethernet Status: "));
        Console.println(ethernetInitialized ? F("INITIALIZED") : F("NOT INITIALIZED"));

        if (ethernetInitialized)
        {
            // Display IP address
            IPAddress ip = Ethernet.localIP();
            Console.print(F("IP Address: "));
            Console.print(ip[0]);
            Console.print(F("."));
            Console.print(ip[1]);
            Console.print(F("."));
            Console.print(ip[2]);
            Console.print(F("."));
            Console.println(ip[3]);

            // Display MAC address
            byte mac[6];
            Ethernet.MACAddress(mac);
            Console.print(F("MAC Address: "));
            for (int i = 0; i < 6; i++)
            {
                if (mac[i] < 16)
                    Console.print(F("0"));
                Console.print(mac[i], HEX);
                if (i < 5)
                    Console.print(F(":"));
            }
            Console.println();

            // Display port
            Console.print(F("Server Port: "));
            Console.println(ETHERNET_PORT);

            // Get client count using the shared function
            int connectedCount = getConnectedClientCount();

            // Display connected clients
            Console.println(F("\nConnected Clients:"));

            if (connectedCount == 0)
            {
                Console.println(F("  No clients connected"));
            }
            else
            {
                // Only iterate through to display client details
                for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
                {
                    if (clients[i] && clients[i].connected())
                    {
                        Console.print(F("  Client "));
                        Console.print(i + 1);
                        Console.print(F(": "));
                        Console.print(clients[i].remoteIP());
                        Console.print(F(":"));
                        Console.println(clients[i].remotePort());
                    }
                }
            }

            Console.print(F("Total Connections: "));
            Console.print(connectedCount);
            Console.print(F(" of "));
            Console.println(MAX_ETHERNET_CLIENTS);
        }

        return true;
    }

    case 5: // "reset"
    {
        // Reset the system state after a failure
        Console.println(F("\n===== RESETTING SYSTEM STATE ====="));

        // Capture the current state before resetting
        SystemState preResetState = captureSystemState();
        bool wasFaulted = (preResetState.motorState == MOTOR_STATE_FAULTED);
        bool wasOperationInProgress = operationInProgress;

        // Perform the reset operation
        resetSystemState();

        // Capture state after reset for comparison
        SystemState postResetState = captureSystemState();

        // Provide feedback on what was reset
        if (wasFaulted)
        {
            Console.info(F("Motor fault condition cleared"));
        }

        if (wasOperationInProgress)
        {
            Console.info(F("Operation state cleared"));
        }

        Console.info(F("System state has been reset and is ready for new commands"));

        return true;
    }

    default: // Unknown subcommand
    {
        // Unknown subcommand
        Console.print(F("[ERROR] Unknown system command: "));
        Console.println(subcommand);
        Console.println(F("Valid options are 'system,state', 'system,safety', 'system,trays', 'system,network', or 'system,reset'"));
        return false;
    }
    }

    return false; // Should never reach here
}

// Tray command handler
bool cmd_tray(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: tray,<load|unload|placed|released|status|help>"));
        return false;
    }

    // Parse the subcommand
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: tray,<load|unload|placed|released|status|help>"));
        return false;
    }

    // Skip leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle the different subcommands
    if (strcmp(subcommand, "tray") == 0)
    {
        // If the first token is "tray", get the next token which is the actual subcommand
        subcommand = strtok(NULL, " ");
        if (subcommand == NULL)
        {
            Console.error(F("Missing subcommand. Usage: tray,<load|unload|placed|released|status|help>"));
            return false;
        }
        subcommand = trimLeadingSpaces(subcommand);
    }

    // Map subcommand to integer for switch statement
    int cmdCode = 0;
    if (strcmp(subcommand, "load") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "placed") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "released") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "unload") == 0)
        cmdCode = 4;
    else if (strcmp(subcommand, "removed") == 0)
        cmdCode = 5;
    else if (strcmp(subcommand, "status") == 0)
        cmdCode = 6;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 7;

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "load"
    {
        // Check for second parameter "request"
        char *action = strtok(NULL, " ");
        if (action == NULL || strcmp(action, "request") != 0)
        {
            Console.error(F("Invalid format. Usage: tray,load,request"));
            return false;
        }
        // Mitsubishi robot is requesting to load a tray

        // 1. Check if the system can accept a tray
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);

        // Check for full system BEFORE checking position 1
        // Check if all positions are occupied (system is full)
        if (trayTracking.position1Occupied && trayTracking.position2Occupied && trayTracking.position3Occupied)
        {
            Console.info(F("System is full - all positions occupied"));
            Console.println(F("SYSTEM_FULL"));
            return false;
        }

        // 2. Verify position 1 is free and no operations are in progress
        if (trayTracking.position1Occupied)
        {
            Console.println(F("POSITION_OCCUPIED"));
            return false;
        }

        if (operationInProgress)
        {
            Console.println(F("SYSTEM_BUSY"));
            return false;
        }

        // 3. Validate safety constraints
        SafetyValidationResult safety = validateSafety(state);
        if (!safety.safeToLoadTrayToPos1)
        {
            Console.print(F("UNSAFE: "));
            Console.println(safety.loadTrayPos1UnsafeReason);
            return false;
        }

        // 4. Set the target position for position 1
        if (!moveToPositionMm(POSITION_1_MM))
        {
            Console.println(F("ERROR_MOVE_FAILURE"));
            return false;
        }

        // 5. System is ready to receive tray
        Console.println(F("READY_TO_RECEIVE"));

        // Add helpful message about the overall loading process
        if (trayTracking.totalTraysInSystem == 0)
        {
            Console.println(F("[NOTE] First tray will be moved to position 3 after placement"));
        }
        else if (trayTracking.totalTraysInSystem == 1)
        {
            Console.println(F("[NOTE] Second tray will be moved to position 2 after placement"));
        }
        else
        {
            Console.println(F("[NOTE] Third tray will remain at position 1 after placement"));
        }

        return true;
    }

    case 2: // "placed"
    {
        // Mitsubishi robot has placed the tray
        // Just mark position 1 as occupied without incrementing total
        trayTracking.position1Occupied = true;
        trayTracking.lastLoadTime = millis();

        // 1. Verify tray sensor shows tray is present
        SystemState state = captureSystemState();
        if (!state.tray1Present)
        {
            Console.println(F("ERROR_NO_TRAY_DETECTED"));
            Console.println(F("[NOTE] If you encounter issues, use 'system,reset' to reset the system state"));
            return false;
        }

        // 2. Lock the tray in position
        DoubleSolenoidValve *valve = getTray1Valve(); // Tray 1 valve
        CylinderSensor *sensor = getTray1Sensor();    // Add this line to get the sensor

        if (valve && sensor)
        {
            // Check current state first
            if (valve->position != VALVE_POSITION_LOCK)
            {
                if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
                {
                    Console.error(F("Failed to lock tray - sensor didn't confirm"));
                    Console.println(F("[WARNING] Check air pressure and valve functionality"));
                    return false;
                }
                Console.println(F("TRAY_SECURED"));

                // Only update the timestamp, not the count (workflow functions will handle counting)
                trayTracking.lastLoadTime = millis();
                return true;
            }
            else
            {
                // Valve already in lock position
                Console.println(F("TRAY_ALREADY_SECURED"));
                // Only update the timestamp, not the count (workflow functions will handle counting)
                trayTracking.lastLoadTime = millis();
                return true;
            }
        }
        else
        {
            Console.println(F("ERROR_LOCK_FAILURE"));
            return false;
        }
    }

    case 3: // "released"
    {
        // Start the automated operation
        beginOperation();

        // Set the operation details
        currentOperation.inProgress = true;
        currentOperation.type = OPERATION_LOADING;
        currentOperation.startTime = millis();

        // No need to manually set target - it will be set in processTrayLoading()

        Console.println(F("STARTING_PROCESSING"));
        return true;
    }

    case 4: // "unload"
    {
        // Use direct string search approach instead of strtok()
        char *originalArgs = args; // Original command string

        // Look for "unload" then "request" in sequence
        char *unloadPos = strstr(originalArgs, "unload");
        if (unloadPos && strstr(unloadPos + 6, "request"))
        {
            // Mitsubishi robot is requesting to unload a tray

            // 1. Check if there are any trays in the system to unload
            SystemState state = captureSystemState();
            updateTrayTrackingFromSensors(state);

            if (trayTracking.totalTraysInSystem == 0)
            {
                Console.info(F("No trays available to unload"));
                Console.println(F("NO_TRAYS"));
                return false;
            }

            // 2. Check if an operation is already in progress
            if (operationInProgress)
            {
                Console.println(F("SYSTEM_BUSY"));
                return false;
            }

            // 3. Check if there's a tray at position 1
            if (state.tray1Present)
            {
                // Tray already at position 1, just need to unlock it
                DoubleSolenoidValve *valve = getTray1Valve();
                CylinderSensor *sensor = getTray1Sensor();

                if (valve && sensor)
                {
                    // Unlock the tray
                    if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000))
                    {
                        Console.error(F("Failed to unlock tray - sensor didn't confirm"));
                        Console.println(F("[WARNING] Check air pressure and valve functionality"));
                        return false;
                    }

                    Console.println(F("[NOTE] Unloading tray from position 1 (loading position)"));
                    Console.info(F("Tray at position 1 unlocked and ready for removal"));
                    Console.println(F("TRAY_READY"));
                    return true;
                }
                else
                {
                    Console.error(F("Failed to access tray 1 valve or sensor"));
                    Console.println(F("VALVE_ACCESS_ERROR"));
                    return false;
                }
            }
            else
            {
                // Need to start unloading operation to move a tray to position 1
                if (state.tray2Present)
                {
                    Console.println(F("[NOTE] Moving tray from position 2 to position 1 for unloading"));
                }
                else if (state.tray3Present)
                {
                    Console.println(F("[NOTE] Moving tray from position 3 to position 1 for unloading"));
                }

                beginOperation();

                // Set the operation details
                currentOperation.inProgress = true;
                currentOperation.type = OPERATION_UNLOADING;
                currentOperation.startTime = millis();

                Console.println(F("PREPARING_TRAY"));
                return true;
            }
        }
        else
        {
            Console.error(F("Invalid format. Usage: tray,unload,request"));
            return false;
        }
    }

    case 5: // "removed"
    {
        // Mitsubishi robot has removed the tray from position 1

        // 1. Verify tray sensor shows tray is no longer present
        SystemState state = captureSystemState();
        if (state.tray1Present)
        {
            Console.println(F("ERROR_TRAY_STILL_PRESENT"));
            Console.println(F("[NOTE] Sensor still detects a tray at position 1"));
            return false;
        }

        // 2. Update tracking information
        unloadFirstTray();

        // 3. Manually increment the unload counter
        // This ensures it's incremented even if unloadFirstTray() didn't do it
        // (which happens when a tray was moved from position 3 to position 1)
        trayTracking.totalUnloadsCompleted++;

        Console.println(F("TRAY_REMOVAL_CONFIRMED"));
        Console.print(F("[INFO] Total unloads completed: "));
        Console.println(trayTracking.totalUnloadsCompleted);

        return true;
    }

    case 6: // "status"
    {
        // Return machine-readable status of tray system
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);

        // Output total trays in system
        Console.print(F("TRAYS_TOTAL:"));
        Console.println(trayTracking.totalTraysInSystem);

        // Output position occupancy (1=occupied, 0=empty)
        Console.print(F("POS1:"));
        Console.println(trayTracking.position1Occupied ? 1 : 0);
        Console.print(F("POS2:"));
        Console.println(trayTracking.position2Occupied ? 1 : 0);
        Console.print(F("POS3:"));
        Console.println(trayTracking.position3Occupied ? 1 : 0);

        // Output lock status (1=locked, 0=unlocked)
        Console.print(F("LOCK1:"));
        Console.println(state.tray1Locked ? 1 : 0);
        Console.print(F("LOCK2:"));
        Console.println(state.tray2Locked ? 1 : 0);
        Console.print(F("LOCK3:"));
        Console.println(state.tray3Locked ? 1 : 0);

        // Add pneumatic pressure status (1=sufficient, 0=insufficient)
        Console.print(F("PRESSURE:"));
        Console.println(isPressureSufficient() ? 1 : 0);

        // Output operation statistics
        Console.print(F("LOADS:"));
        Console.println(trayTracking.totalLoadsCompleted);
        Console.print(F("UNLOADS:"));
        Console.println(trayTracking.totalUnloadsCompleted);

        return true;
    }

    case 7: // "help"
    {
        Console.println(F("\n===== TRAY SYSTEM HELP ====="));
        Console.println(F("\nTRAY LOADING SEQUENCE:"));
        Console.println(F("  1. tray,load,request - Request permission to load a tray"));
        Console.println(F("     > System will validate position 1 is empty and move shuttle there"));
        Console.println(F("     > System responds with 'READY_TO_RECEIVE' when ready"));
        Console.println(F("  2. tray,placed - Notify system that tray has been physically placed"));
        Console.println(F("     > System will lock the tray at position 1"));
        Console.println(F("     > System responds with 'TRAY_SECURED' when complete"));
        Console.println(F("  3. tray,released - Notify system to start processing the tray"));
        Console.println(F("     > System will move tray to appropriate position based on system state"));
        Console.println(F("     > First tray goes to position 3, second to position 2, third stays at position 1"));

        Console.println(F("\nTRAY UNLOADING SEQUENCE:"));
        Console.println(F("  1. tray,unload,request - Request permission to unload a tray"));
        Console.println(F("     > If tray at position 1, system unlocks it and responds 'TRAY_READY'"));
        Console.println(F("     > If tray at positions 2 or 3, system moves it to position 1 first"));
        Console.println(F("     > System responds with 'PREPARING_TRAY' during movement"));
        Console.println(F("  2. tray,removed - Notify system that tray has been physically removed"));
        Console.println(F("     > System updates internal tracking"));
        Console.println(F("     > System responds with 'TRAY_REMOVAL_CONFIRMED'"));

        Console.println(F("\nTRAY STATUS COMMAND:"));
        Console.println(F("  tray,status - Returns machine-readable status information"));
        Console.println(F("  Returned values:"));
        Console.println(F("    TRAYS_TOTAL:[0-3] - Total number of trays in system"));
        Console.println(F("    POS1:[0|1] - Position 1 occupancy (0=empty, 1=occupied)"));
        Console.println(F("    POS2:[0|1] - Position 2 occupancy (0=empty, 1=occupied)"));
        Console.println(F("    POS3:[0|1] - Position 3 occupancy (0=empty, 1=occupied)"));
        Console.println(F("    LOCK1:[0|1] - Position 1 lock status (0=unlocked, 1=locked)"));
        Console.println(F("    LOCK2:[0|1] - Position 2 lock status (0=unlocked, 1=locked)"));
        Console.println(F("    LOCK3:[0|1] - Position 3 lock status (0=unlocked, 1=locked)"));
        Console.println(F("    PRESSURE:[0|1] - Pneumatic pressure status (0=insufficient, 1=sufficient)"));
        Console.println(F("    LOADS:[number] - Total number of loads completed"));
        Console.println(F("    UNLOADS:[number] - Total number of unloads completed"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If an operation fails, use 'system,reset' to reset the system state"));
        Console.println(F("  • Use 'system,trays' for human-readable tray system status"));
        Console.println(F("  • Use 'system,safety' to diagnose safety constraint issues"));
        Console.println(F("-------------------------------------------"));

        return true;
    }

    default: // Unknown command
    {
        Console.print(F("[ERROR] Unknown tray command: "));
        Console.println(subcommand);
        Console.println(F("Valid options are 'load,request', 'unload,request', 'placed', 'removed', 'released', 'status', or 'help'"));
        return false;
    }
    }

    return false; // Should never reach here
}

bool cmd_test(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.println(F("Available tests:"));
        Console.println(F("  home     - Test homing repeatability"));
        Console.println(F("  position - Test position cycling (for tray loading)"));
        Console.println(F("  tray     - Test complete tray handling operations"));
        Console.println(F("  help     - Display detailed test information"));
        Console.println(F("Usage: test,<test_name>"));
        return true;
    }

    // Parse the first argument - we'll use spaces as separators (commas converted to spaces)
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: test,<home|position|tray|help>"));
        return false;
    }

    // Trim leading spaces from test type
    subcommand = trimLeadingSpaces(subcommand);

    // Check motor initialization first (except for help command)
    if (strcmp(subcommand, "help") != 0 && motorState == MOTOR_STATE_NOT_READY)
    {
        Console.error(F("Motor not initialized. Run 'motor,init' first."));
        return false;
    }

    // Check E-Stop condition (except for help command)
    if (strcmp(subcommand, "help") != 0 && isEStopActive())
    {
        Console.error(F("Cannot run tests while E-Stop is active."));
        return false;
    }

    // Map subcommand to integer for switch statement
    int cmdCode = 0;
    if (strcmp(subcommand, "home") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "position") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "tray") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 4;

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "home"
    {
        Console.info(F("Starting homing repeatability test..."));
        if (testHomingRepeatability())
        {
            Console.info(F("Homing repeatability test completed successfully."));
            return true;
        }
        else
        {
            Console.error(F("Homing repeatability test failed or was aborted."));
            return false;
        }
        break;
    }

    case 2: // "position"
    {
        Console.info(F("Starting position cycling test..."));

        if (testPositionCycling())
        {
            Console.info(F("Position cycling test completed successfully."));
            return true;
        }
        else
        {
            Console.info(F("Position cycling test failed or was aborted."));
            return false;
        }
        break;
    }

    case 3: // "tray"
    {
        Console.info(F("Starting tray handling test..."));

        // Add pneumatic pressure validation before starting the test
        if (!isPressureSufficient())
        {
            Console.error(F("Cannot run tray test - pneumatic pressure insufficient"));
            Console.println(F("[WARNING] Ensure air supply is connected and pressure is adequate"));
            Console.println(F("[INFO] Use 'tray,status' to check PRESSURE status"));
            return false;
        }

        if (testTrayHandling())
        {
            Console.info(F("Tray handling test completed successfully."));
            return true;
        }
        else
        {
            Console.error(F("Tray handling test failed or was aborted."));
            return false;
        }
        break;
    }

    case 4: // "help"
    {
        Console.println(F("\n===== TEST SYSTEM HELP ====="));

        Console.println(F("\nOVERVIEW:"));
        Console.println(F("  The test system provides automated sequences for validating"));
        Console.println(F("  system functionality and repeatability. Tests are designed"));
        Console.println(F("  to verify proper operation of critical system components."));

        Console.println(F("\nAVAILABLE TESTS:"));
        Console.println(F("  test,home - Homing repeatability test"));
        Console.println(F("    > Performs multiple homing operations to test precision"));
        Console.println(F("    > Moves between home position and test position"));
        Console.println(F("    > Useful for verifying encoder and limit switch reliability"));
        Console.println(F("    > Test runs for approximately 20 cycles"));

        Console.println(F("  test,position - Position cycling test"));
        Console.println(F("    > Cycles through positions used in tray loading"));
        Console.println(F("    > Tests movements between positions 1, 2, and 3"));
        Console.println(F("    > Verifies motor accuracy and repeatability"));
        Console.println(F("    > Test runs for approximately 10 cycles"));

        Console.println(F("  test,tray - Comprehensive tray handling test"));
        Console.println(F("    > Tests complete tray movement operations"));
        Console.println(F("    > Includes valve operations for locking/unlocking"));
        Console.println(F("    > Verifies sensors, positioning, and control sequences"));
        Console.println(F("    > Most thorough test of the entire system"));

        Console.println(F("\nRUNNING TESTS:"));
        Console.println(F("  • Motor must be initialized (motor,init) before testing"));
        Console.println(F("  • Home position must be established for position tests"));
        Console.println(F("  • E-Stop must be inactive"));
        Console.println(F("  • Tests can be aborted by typing 'abort' during execution"));
        Console.println(F("  • Status messages display progress throughout the test"));

        Console.println(F("\nTRAY TEST REQUIREMENTS:"));
        Console.println(F("  • A tray must be present at position 1 to start"));
        Console.println(F("  • Positions 2 and 3 must be clear initially"));
        Console.println(F("  • Air system must be functioning properly"));
        Console.println(F("  • All valves and sensors must be operational"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If a test fails, check the specific error message"));
        Console.println(F("  • For position errors: verify motor operation with 'move' commands"));
        Console.println(F("  • For valve errors: check air pressure and connections"));
        Console.println(F("  • For sensor errors: verify sensor readings with 'system,state'"));
        Console.println(F("-------------------------------------------"));

        return true;
        break;
    }

    default: // Unknown command
    {
        Console.print(F("[ERROR] Unknown test type: "));
        Console.println(subcommand);
        Console.println(F("Available tests: 'home', 'position', 'tray', or 'help'"));
        return false;
        break;
    }
    }

    return false; // Should never reach here
}

bool cmd_encoder(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        // Display current encoder status
        Console.info(F("MPG Handwheel Controls:"));
        Console.println(F("  encoder,enable          - Enable MPG handwheel control"));
        Console.println(F("  encoder,disable         - Disable MPG handwheel control"));
        Console.println(F("  encoder,multiplier,[1|10|100] - Set movement multiplier"));
        Console.println(F("  encoder,help            - Display detailed usage instructions"));

        // Show current status
        if (encoderControlActive)
        {
            Console.println(F("\n[STATUS] MPG control is currently ENABLED"));
            Console.print(F("[STATUS] Current multiplier: x"));
            Console.println(currentMultiplier);

            // Show position information if motor is homed
            if (isHomed)
            {
                double positionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
                Console.print(F("[STATUS] Current position: "));
                Console.print(positionMm, 2);
                Console.println(F(" mm"));
            }
        }
        else
        {
            Console.println(F("\n[STATUS] MPG control is currently DISABLED"));

            // Show reasons why encoder control might not be available
            if (!motorInitialized)
            {
                Console.println(F("[NOTE] Motor needs to be initialized first (motor,init)"));
            }
            else if (!isHomed)
            {
                Console.println(F("[NOTE] Motor needs to be homed first (motor,home)"));
            }
        }

        Console.println(F("\n[MULTIPLIERS] Effect of one full handwheel rotation (100 pulses):"));
        Console.print(F("  x1: ~"));
        Console.print(100 * MULTIPLIER_X1 / PULSES_PER_MM, 2);
        Console.println(F(" mm (fine adjustment)"));
        Console.print(F("  x10: ~"));
        Console.print(100 * MULTIPLIER_X10 / PULSES_PER_MM, 2);
        Console.println(F(" mm (medium adjustment)"));
        Console.print(F("  x100: ~"));
        Console.print(100 * MULTIPLIER_X100 / PULSES_PER_MM, 2);
        Console.println(F(" mm (coarse adjustment)"));

        return true;
    }

    // Parse the argument - make sure we're handling comma separators correctly too
    // Replace commas with spaces first (if you're using spaces as delimiters)
    for (int i = 0; trimmed[i] != '\0'; i++)
    {
        if (trimmed[i] == ',')
        {
            trimmed[i] = ' ';
        }
    }

    // Get the subcommand
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: encoder,<enable|disable|multiplier>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Map subcommand to integer for switch statement
    int cmdCode = 0;
    if (strcmp(subcommand, "enable") == 0)
        cmdCode = 1;
    else if (strcmp(subcommand, "disable") == 0)
        cmdCode = 2;
    else if (strcmp(subcommand, "multiplier") == 0)
        cmdCode = 3;
    else if (strcmp(subcommand, "help") == 0)
        cmdCode = 4;

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "enable"
    {
        // Check preconditions
        if (!motorInitialized)
        {
            Console.error(F("Motor must be initialized before enabling MPG control"));
            Console.info(F("Use 'motor,init' first"));
            return false;
        }

        if (!isHomed)
        {
            Console.error(F("Motor must be homed before enabling MPG control"));
            Console.info(F("Use 'motor,home' to establish a reference position"));
            return false;
        }

        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            Console.error(F("Cannot enable MPG control while motor is moving"));
            Console.info(F("Wait for current movement to complete or use 'motor,abort'"));
            return false;
        }

        if (motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Cannot enable MPG control while motor is in fault state"));
            Console.info(F("Use 'motor,clear' to clear fault first"));
            return false;
        }

        if (isEStopActive())
        {
            Console.error(F("Cannot enable MPG control while E-Stop is active"));
            return false;
        }

        // Enable encoder control
        encoderControlActive = true;

        // Reset encoder position
        EncoderIn.Position(0);
        lastEncoderPosition = 0;
        lastEncoderUpdateTime = millis();

        Console.print(F("[INFO] MPG handwheel control enabled - current position: "));
        Console.print(pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()), 2);
        Console.println(F(" mm"));
        Console.print(F("[INFO] Using multiplier x"));
        Console.print(getMultiplierName(currentMultiplier));
        Console.print(F(" ("));
        Console.print(currentMultiplier);
        Console.println(F(")"));
        Console.info(F("Issue 'encoder,disable' when finished with manual control"));

        return true;
        break;
    }

    case 2: // "disable"
    {
        // Disable encoder control
        encoderControlActive = false;
        Console.info(F("MPG handwheel control disabled"));
        return true;
        break;
    }

    case 3: // "multiplier"
    {
        // Look for the NEXT argument - this is the same approach used in move,mm,X
        char *originalArgs = args; // Save the original args string

        // Find the "multiplier" substring within args
        char *multiplierPos = strstr(originalArgs, "multiplier");
        if (multiplierPos != NULL)
        {
            // Move past "multiplier"
            multiplierPos += strlen("multiplier");

            // Skip any spaces or commas
            while (*multiplierPos && (*multiplierPos == ' ' || *multiplierPos == ','))
            {
                multiplierPos++;
            }

            // Now multiplierPos should point to the actual value
            if (*multiplierPos)
            {
                // Parse the actual multiplier value
                int multiplier = atoi(multiplierPos);

                // Set the multiplier based on the input value
                switch (multiplier)
                {
                case 1:
                    setEncoderMultiplier(1);
                    Console.info(F("Multiplier set to x1 (fine adjustment)"));
                    break;
                case 10:
                    setEncoderMultiplier(10);
                    Console.info(F("Multiplier set to x10 (medium adjustment)"));
                    break;
                case 100:
                    setEncoderMultiplier(100);
                    Console.info(F("Multiplier set to x100 (coarse adjustment)"));
                    break;
                default:
                    Console.error(F("Invalid multiplier. Use 1, 10, or 100."));
                    return false;
                }

                // Show current multiplier and effect
                Console.print(F("[INFO] Current multiplier value: "));
                Console.println(currentMultiplier);
                double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
                Console.print(F("[INFO] One full rotation moves ~"));
                Console.print(mmPerRotation, 2);
                Console.println(F(" mm"));
                return true;
            }
        }

        // If we get here, display the current multiplier
        Console.print(F("[INFO] Current multiplier: x"));
        Console.print(getMultiplierName(currentMultiplier));
        Console.print(F(" ("));
        Console.print(currentMultiplier);
        Console.println(F(")"));

        // Show what one full rotation will move
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        Console.print(F("[INFO] One full rotation moves ~"));
        Console.print(mmPerRotation, 2);
        Console.println(F(" mm"));

        return true;
        break;
    }

    case 4: // "help"
    {
        Console.println(F("\n===== MPG HANDWHEEL SYSTEM HELP ====="));

        Console.println(F("\nSETUP SEQUENCE:"));
        Console.println(F("  1. 'motor,init' - Initialize the motor system"));
        Console.println(F("  2. 'motor,home' - Home the motor to establish reference position"));
        Console.println(F("  3. 'encoder,enable' - Activate MPG handwheel control"));
        Console.println(F("  4. 'encoder,multiplier,X' - Set desired precision (X = 1, 10, or 100)"));

        Console.println(F("\nCOMMAND REFERENCE:"));
        Console.println(F("  encoder,enable - Activate handwheel control mode"));
        Console.println(F("    > Motor position will respond directly to handwheel rotation"));
        Console.println(F("    > One full rotation (100 pulses) moves distance based on multiplier"));
        Console.println(F("  encoder,disable - Deactivate handwheel control mode"));
        Console.println(F("    > Returns system to command-based position control"));
        Console.println(F("  encoder,multiplier,X - Set movement precision"));
        Console.println(F("    > X=1: Fine adjustment (~1.63mm per rotation)"));
        Console.println(F("    > X=10: Medium adjustment (~16.3mm per rotation)"));
        Console.println(F("    > X=100: Coarse adjustment (~163mm per rotation)"));

        Console.println(F("\nAUTOMATIC DISABLING CONDITIONS:"));
        Console.println(F("  • E-Stop activation - Safety override disables all motor control"));
        Console.println(F("  • Motor fault condition - Requires 'motor,clear' to reset"));
        Console.println(F("  • Power cycle or system reset"));
        Console.println(F("  • When 'move' or 'jog' commands are issued"));

        Console.println(F("\nMOVEMENT CONSTRAINTS:"));
        Console.println(F("  • Hard limit at 0mm (home position)"));
        Console.println(F("  • Hard limit at maximum travel position (~1050mm)"));
        Console.println(F("  • Movement stops automatically at travel limits"));
        Console.println(F("  • No movement allowed if motor is in fault state"));

        Console.println(F("\nUSAGE TIPS:"));
        Console.println(F("  • Start with x1 multiplier for precise positioning"));
        Console.println(F("  • Use x10 or x100 for longer movements"));
        Console.println(F("  • Monitor current position using 'motor,status' command"));
        Console.println(F("  • Use 'encoder,disable' when finished with manual control"));
        Console.println(F("  • Slow, steady handwheel rotation produces smoother movement"));

        Console.println(F("\nTROUBLESHOOTING:"));
        Console.println(F("  • If encoder doesn't respond: Check if motor is initialized and homed"));
        Console.println(F("  • Erratic movement: Try lower multiplier setting"));
        Console.println(F("  • No movement at limits: System is preventing over-travel"));
        Console.println(F("  • After E-Stop: Must re-enable encoder control manually"));
        Console.println(F("-------------------------------------------"));

        return true;
        break;
    }

    default: // Unknown command
    {
        Console.print(F("[ERROR] Unknown encoder command: "));
        Console.println(subcommand);
        Console.println(F("Valid options are 'enable', 'disable', 'multiplier', or 'help'"));
        return false;
        break;
    }
    }

    return false; // Should never reach here
}

bool cmd_abort(char *args, CommandCaller *caller)
{
    requestTestAbort("abort command");
    return true;
}

Commander commander;

Commander::systemCommand_t API_tree[] = {
    systemCommand("help", "Display help information for all commands", cmd_print_help),
    systemCommand("h", "Display help information for all commands", cmd_print_help),
    systemCommand("H", "Display help information for all commands", cmd_print_help),

    // Unified lock/unlock commands
    systemCommand("lock", "Lock a tray or shuttle:\r\n"
                          "  lock,1..3    - Lock specific tray position\r\n"
                          "  lock,shuttle - Lock the shuttle\r\n"
                          "  lock,help    - Display detailed lock instructions",
                  cmd_lock),

    systemCommand("unlock", "Unlock a tray, shuttle, or all valves:\r\n"
                            "  unlock,1..3    - Unlock specific tray position\r\n"
                            "  unlock,shuttle - Unlock the shuttle\r\n"
                            "  unlock,all     - Unlock all valves\r\n"
                            "  unlock,help    - Display detailed unlock instructions",
                  cmd_unlock),

    // Logging command
    systemCommand("log", "Logging controls:\r\n"
                         "  log,on[,interval] - Enable periodic logging (interval in ms)\r\n"
                         "  log,off           - Disable periodic logging\r\n"
                         "  log,now           - Log system state immediately\r\n"
                         "  log,help          - Display detailed logging information",
                  cmd_log),

    // State command to display system state
    systemCommand("system", "System commands:\r\n"
                            "  system,state    - Display current system state (sensors, actuators, positions)\r\n"
                            "  system,safety   - Display comprehensive safety validation status\r\n"
                            "  system,trays    - Display tray tracking and statistics\r\n"
                            "  system,network  - Display Ethernet connection status and IP address\r\n"
                            "  system,reset    - Reset system state after failure to retry operation",
                  cmd_system_state),

    // Motor control commands
    systemCommand("motor", "Motor control:\r\n"
                           "  motor,init   - Initialize motor system and prepare for operation\r\n"
                           "  motor,status - Display detailed motor status and configuration\r\n"
                           "  motor,clear  - Clear motor fault condition to restore operation\r\n"
                           "  motor,home   - Home the motor (find zero position)\r\n"
                           "  motor,abort  - Abort current operation gracefully\r\n"
                           "  motor,stop   - Emergency stop motor movement immediately\r\n"
                           "  motor,help   - Display comprehensive motor control instructions",
                  cmd_motor),

    // Move command
    systemCommand("move", "Move motor to position:\r\n"
                          "  move,home      - Move to home (zero) position\r\n"
                          "  move,1..4      - Move to predefined positions 1 through 4\r\n"
                          "  move,counts,X  - Move to absolute position X in encoder counts (0-64333)\r\n"
                          "  move,mm,X      - Move to absolute position X in millimeters (0-1050.0)\r\n"
                          "  move,rel,X     - Move X millimeters relative to current position (+ forward, - backward)\r\n"
                          "  move,help      - Display detailed command usage and troubleshooting",
                  cmd_move),

    // Jog command
    systemCommand("jog", "Jog motor:\r\n"
                         "  jog,+         - Jog forward by current increment\r\n"
                         "  jog,-         - Jog backward by current increment\r\n"
                         "  jog,inc,X     - Get or set jog increment (X in mm or 'default')\r\n"
                         "  jog,speed,X   - Get or set jog speed (X in RPM or 'default')\r\n"
                         "  jog,status    - Display current jog settings\r\n"
                         "  jog,help      - Display usage instructions and comparison with handwheel",
                  cmd_jog),

    // Tray command
    systemCommand("tray", "Tray operations:\r\n"
                          "  tray,load,request   - Request to load a tray (Mitsubishi)\r\n"
                          "  tray,unload,request - Request to unload a tray (Mitsubishi)\r\n"
                          "  tray,placed    - Notify tray has been placed (Mitsubishi)\r\n"
                          "  tray,removed   - Notify tray has been removed (Mitsubishi)\r\n"
                          "  tray,released  - Notify tray has been released (Mitsubishi)\r\n"
                          "  tray,status    - Get tray system status (machine-readable)\r\n"
                          "  tray,help      - Display detailed usage instructions",
                  cmd_tray),

    // Test command
    systemCommand("test", "Run tests on the system:\r\n"
                          "  test,home     - Run homing repeatability test\r\n"
                          "  test,position - Run position cycling test for tray loading\r\n"
                          "  test,tray     - Run tray handling test (request, place, release)\r\n"
                          "  test,help     - Display detailed test information and requirements",
                  cmd_test),

    // Encoder control commands
    systemCommand("encoder", "Encoder handwheel control:\r\n"
                             "  encoder,enable  - Enable encoder control\r\n"
                             "  encoder,disable - Disable encoder control\r\n"
                             "  encoder,multiplier,X - Set encoder multiplier (X = 1, 10, or 100)\r\n"
                             "  encoder,help    - Display setup instructions and usage tips",
                  cmd_encoder),

    // Abort command
    systemCommand("abort", "Abort any running test", cmd_abort),
};

const size_t API_tree_size = sizeof(API_tree) / sizeof(Commander::systemCommand_t);