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
        caller->println(F("[ERROR] Detailed help for specific commands is not implemented."));
        return false;
    }
    else
    {
        // No specific command requested; print general help.
        // Pass a pointer to the Serial stream.

        // Print general help information.
        caller->println(F("--------------------------------------------------"));
        caller->println(F("Lynx Conveyor System Command Help:"));
        caller->println(F("--------------------------------------------------"));

        commander.printHelp(caller, true, false);

        caller->println(F("--------------------------------------------------"));
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
        caller->println(F("[ERROR] Missing parameter. Usage: lock,<1|2|3|shuttle|help>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, ",");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: lock,<1|2|3|shuttle|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle locking based on subcommand
    if (strcmp(subcommand, "all") == 0)
    {
        // Removed "lock all" functionality as requested
        caller->println(F("[ERROR] 'lock,all' is not supported for safety reasons. Engage trays individually."));
        return false;
    }
    else if (strcmp(subcommand, "shuttle") == 0)
    {
        if (ccioBoardCount > 0)
        {
            caller->println(F("[INFO] Engaging shuttle with sensor verification..."));
            DoubleSolenoidValve *valve = getShuttleValve();
            CylinderSensor *sensor = getShuttleSensor();

            if (valve && sensor)
            {
                // Check current state first
                if (valve->position == VALVE_POSITION_LOCK)
                {
                    caller->println(F("[INFO] Shuttle already engaged"));

                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == true)
                    { // Sensor true = locked
                        caller->println(F("[OK] Shuttle lock confirmed by sensor"));
                    }
                    else
                    {
                        caller->println(F("[WARNING] Shuttle should be locked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try locking with sensor feedback
                    caller->println(F("[INFO] Locking shuttle..."));
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
                    {
                        caller->println(F("[INFO] Shuttle engaged and confirmed by sensor"));
                        return true;
                    }
                    else
                    {
                        caller->println(F("[ERROR] Failed to engage shuttle - sensor did not confirm lock"));
                        caller->println(F("[WARNING] Check air pressure and valve functionality"));
                        return false;
                    }
                }
            }
            else
            {
                caller->println(F("[ERROR] Failed to access shuttle valve or sensor. Possible causes:"));
                caller->println(F("  - CCIO board detected but shuttle valve not configured"));
                caller->println(F("  - System memory corruption"));
                caller->println(F("Try restarting the system or run 'status' to check valve configuration"));
                return false;
            }
        }
        else
        {
            caller->println(F("[ERROR] No CCIO-8 board detected. Shuttle valve not available."));
            return false;
        }
    }
    else
    {
        // Try to parse as tray number
        int trayNum = atoi(subcommand);
        if (trayNum >= 1 && trayNum <= 3)
        {
            caller->print(F("[INFO] Engaging tray "));
            caller->print(trayNum);
            caller->println(F(" with sensor verification..."));

            DoubleSolenoidValve *valve = NULL;
            CylinderSensor *sensor = NULL;

            // Get the appropriate valve and sensor
            switch (trayNum)
            {
            case 1:
                valve = getTray1Valve();
                sensor = getTray1Sensor();
                break;
            case 2:
                valve = getTray2Valve();
                sensor = getTray2Sensor();
                break;
            case 3:
                valve = getTray3Valve();
                sensor = getTray3Sensor();
                break;
            default:
                return false;
            }

            if (valve && sensor)
            {
                // Check current state first
                if (valve->position == VALVE_POSITION_LOCK)
                {
                    caller->print(F("[INFO] Tray "));
                    caller->print(trayNum);
                    caller->println(F(" already engaged"));

                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == true)
                    { // Sensor true = locked
                        caller->println(F("[OK] Tray lock confirmed by sensor"));
                    }
                    else
                    {
                        caller->println(F("[WARNING] Tray should be locked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try locking with sensor feedback
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
                    {
                        caller->print(F("[INFO] Tray "));
                        caller->print(trayNum);
                        caller->println(F(" engaged and confirmed by sensor"));
                        return true;
                    }
                    else
                    {
                        caller->print(F("[ERROR] Failed to engage tray "));
                        caller->println(trayNum);
                        caller->println(F("[WARNING] Check air pressure and valve functionality"));
                        return false;
                    }
                }
            }
            else
            {
                caller->print(F("[ERROR] Failed to access tray "));
                caller->print(trayNum);
                caller->println(F(" valve or sensor. Possible causes:"));
                caller->println(F("  - Hardware initialization issue"));
                caller->println(F("  - Valve controller not properly initialized"));
                caller->println(F("  - System memory corruption"));
                caller->println(F("Try restarting the system or run 'status' to check valve configuration"));
                return false;
            }
        }
        else if (strcmp(subcommand, "help") == 0)
        {
            caller->println(F("\n===== LOCK COMMAND HELP ====="));

            caller->println(F("\nOVERVIEW:"));
            caller->println(F("  The lock command engages pneumatic locks on trays and the shuttle,"));
            caller->println(F("  securing them in position. All operations include sensor verification"));
            caller->println(F("  to confirm successful locking."));

            caller->println(F("\nCOMMAND REFERENCE:"));
            caller->println(F("  lock,1 - Engage lock on tray at position 1 (loading position)"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Will report success only when sensor confirms lock"));

            caller->println(F("  lock,2 - Engage lock on tray at position 2 (middle position)"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Will report success only when sensor confirms lock"));

            caller->println(F("  lock,3 - Engage lock on tray at position 3 (unloading position)"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Will report success only when sensor confirms lock"));

            caller->println(F("  lock,shuttle - Engage lock on the shuttle"));
            caller->println(F("    > Prevents shuttle from moving between positions"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Required before unlocking any trays for safety"));

            caller->println(F("\nSAFETY NOTES:"));
            caller->println(F("  • 'lock,all' is not supported for safety reasons"));
            caller->println(F("  • Always lock the shuttle before unlocking any trays"));
            caller->println(F("  • System uses sensor verification to confirm actual locking"));
            caller->println(F("  • Failed locking may indicate mechanical issues or low air pressure"));

            caller->println(F("\nSENSOR VERIFICATION:"));
            caller->println(F("  • Each lock has a corresponding sensor that confirms its position"));
            caller->println(F("  • Command waits up to 1 second for sensor to confirm lock"));
            caller->println(F("  • Returns success only when sensor confirms the lock operation"));
            caller->println(F("  • Sensor mismatches are shown in status logs with [!] indicator"));

            caller->println(F("\nTROUBLESHOOTING:"));
            caller->println(F("  • If lock fails, check air pressure"));
            caller->println(F("  • Verify sensor connections if lock command doesn't register"));
            caller->println(F("  • Use 'system,state' to see detailed valve and sensor status"));
            caller->println(F("  • For persistent issues, check valve functionality"));

            return true;
        }
        else
        {
            caller->print(F("[ERROR] Unknown lock subcommand: "));
            caller->println(subcommand);
            caller->println(F("Valid options are '1', '2', '3', 'shuttle', or 'help'"));
            return false;
        }
    }

    return false;
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
        caller->println(F("[ERROR] Missing parameter. Usage: unlock,<1|2|3|shuttle|all|help>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, ",");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: unlock,<1|2|3|shuttle|all|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle unlocking based on subcommand
    if (strcmp(subcommand, "all") == 0)
    {
        caller->println(F("[INFO] Disengaging all valves with sensor verification..."));
        if (safeUnlockAllValves(1000))
        {
            caller->println(F("[INFO] All valves successfully disengaged"));
            return true;
        }
        else
        {
            caller->println(F("[WARNING] Some valves could not be disengaged - check air pressure"));
            return false;
        }
    }
    else if (strcmp(subcommand, "shuttle") == 0)
    {
        if (ccioBoardCount > 0)
        {
            caller->println(F("[INFO] Disengaging shuttle with sensor verification..."));
            DoubleSolenoidValve *valve = getShuttleValve();
            CylinderSensor *sensor = getShuttleSensor();

            if (valve && sensor)
            {
                // Check current state first
                if (valve->position == VALVE_POSITION_UNLOCK)
                {
                    caller->println(F("[INFO] Shuttle already disengaged"));

                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == false)
                    { // Sensor false = unlocked
                        caller->println(F("[OK] Shuttle unlock confirmed by sensor"));
                    }
                    else
                    {
                        caller->println(F("[WARNING] Shuttle should be unlocked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try unlocking with sensor feedback
                    caller->println(F("[INFO] Unlocking shuttle..."));
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000))
                    {
                        caller->println(F("[INFO] Shuttle disengaged and confirmed by sensor"));
                        return true;
                    }
                    else
                    {
                        caller->println(F("[ERROR] Failed to disengage shuttle - sensor did not confirm unlock"));
                        caller->println(F("[WARNING] Check air pressure and valve functionality"));
                        return false;
                    }
                }
            }
            else
            {
                caller->println(F("[ERROR] Failed to access shuttle valve or sensor. Possible causes:"));
                caller->println(F("  - CCIO board detected but shuttle valve not configured"));
                caller->println(F("  - System memory corruption"));
                caller->println(F("Try restarting the system or run 'status' to check valve configuration"));
                return false;
            }
        }
        else
        {
            caller->println(F("[ERROR] No CCIO-8 board detected. Shuttle valve not available."));
            return false;
        }
    }
    else
    {
        // Try to parse as tray number
        int trayNum = atoi(subcommand);
        if (trayNum >= 1 && trayNum <= 3)
        {
            caller->print(F("[INFO] Disengaging tray "));
            caller->print(trayNum);
            caller->println(F(" with sensor verification..."));

            DoubleSolenoidValve *valve = NULL;
            CylinderSensor *sensor = NULL;

            // Get the appropriate valve and sensor
            switch (trayNum)
            {
            case 1:
                valve = getTray1Valve();
                sensor = getTray1Sensor();
                break;
            case 2:
                valve = getTray2Valve();
                sensor = getTray2Sensor();
                break;
            case 3:
                valve = getTray3Valve();
                sensor = getTray3Sensor();
                break;
            default:
                return false;
            }

            if (valve && sensor)
            {
                // Check current state first
                if (valve->position == VALVE_POSITION_UNLOCK)
                {
                    caller->print(F("[INFO] Tray "));
                    caller->print(trayNum);
                    caller->println(F(" already disengaged"));

                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == false)
                    { // Sensor false = unlocked
                        caller->println(F("[OK] Tray unlock confirmed by sensor"));
                    }
                    else
                    {
                        caller->println(F("[WARNING] Tray should be unlocked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try unlocking with sensor feedback
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000))
                    {
                        caller->print(F("[INFO] Tray "));
                        caller->print(trayNum);
                        caller->println(F(" disengaged and confirmed by sensor"));
                        return true;
                    }
                    else
                    {
                        caller->print(F("[ERROR] Failed to disengage tray "));
                        caller->println(trayNum);
                        caller->println(F("[WARNING] Check air pressure and valve functionality"));
                        return false;
                    }
                }
            }
            else
            {
                caller->print(F("[ERROR] Failed to access tray "));
                caller->print(trayNum);
                caller->println(F(" valve or sensor. Possible causes:"));
                caller->println(F("  - Hardware initialization issue"));
                caller->println(F("  - Valve controller not properly initialized"));
                caller->println(F("  - System memory corruption"));
                caller->println(F("Try restarting the system or run 'status' to check valve configuration"));
                return false;
            }
        }
        else if (strcmp(subcommand, "help") == 0)
        {
            caller->println(F("\n===== UNLOCK COMMAND HELP ====="));

            caller->println(F("\nOVERVIEW:"));
            caller->println(F("  The unlock command disengages pneumatic locks on trays and the shuttle,"));
            caller->println(F("  allowing them to be removed or permitting shuttle movement. All operations"));
            caller->println(F("  include sensor verification to confirm successful unlocking."));

            caller->println(F("\nCOMMAND REFERENCE:"));
            caller->println(F("  unlock,1 - Disengage lock on tray at position 1 (loading position)"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Will report success only when sensor confirms unlock"));

            caller->println(F("  unlock,2 - Disengage lock on tray at position 2 (middle position)"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Will report success only when sensor confirms unlock"));

            caller->println(F("  unlock,3 - Disengage lock on tray at position 3 (unloading position)"));
            caller->println(F("    > Verified by cylinder position sensor"));
            caller->println(F("    > Will report success only when sensor confirms unlock"));

            caller->println(F("  unlock,shuttle - Disengage lock on the shuttle"));
            caller->println(F("    > Allows shuttle to move between positions"));
            caller->println(F("    > Verified by cylinder position sensor"));

            caller->println(F("  unlock,all - Disengage all locks in the system"));
            caller->println(F("    > Emergency recovery function"));
            caller->println(F("    > Uses sensor verification for all valves"));
            caller->println(F("    > Reports success only when all sensors confirm unlock"));

            caller->println(F("\nSAFETY NOTES:"));
            caller->println(F("  • Ensure trays are properly supported before unlocking"));
            caller->println(F("  • System uses sensor verification to confirm actual unlocking"));
            caller->println(F("  • Failed unlocking may indicate mechanical issues"));

            caller->println(F("\nSENSOR VERIFICATION:"));
            caller->println(F("  • Each lock has a corresponding sensor that confirms its position"));
            caller->println(F("  • Command waits up to 1 second for sensor to confirm unlock"));
            caller->println(F("  • Returns success only when sensor confirms the unlock operation"));
            caller->println(F("  • Sensor mismatches are shown in status logs with [!] indicator"));

            caller->println(F("\nTROUBLESHOOTING:"));
            caller->println(F("  • If unlock fails, check air pressure"));
            caller->println(F("  • Verify sensor connections if unlock command doesn't register"));
            caller->println(F("  • Use 'system,state' to see detailed valve and sensor status"));
            caller->println(F("  • For persistent issues, check valve functionality"));

            return true;
        }
        else
        {
            caller->print(F("[ERROR] Unknown unlock subcommand: "));
            caller->println(subcommand);
            caller->println(F("Valid options are '1', '2', '3', 'shuttle', 'all', or 'help'"));
            return false;
        }
    }

    return false;
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
        caller->println(F("[ERROR] Missing parameter. Usage: log,<on[,interval]|off|now|help>"));
        return false;
    }

    // Parse the first argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: log,<on[,interval]|off|now|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle based on subcommand
    if (strcmp(subcommand, "on") == 0)
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
                caller->print(F("[INFO] Logging enabled with interval of "));
                caller->print(interval);
                caller->println(F(" ms"));
            }
            else
            {
                caller->println(F("[WARNING] Invalid interval. Using default."));
                interval = DEFAULT_LOG_INTERVAL;
            }
        }
        else
        {
            // Use default interval
            caller->print(F("[INFO] Logging enabled with default interval of "));
            caller->print(DEFAULT_LOG_INTERVAL);
            caller->println(F(" ms"));
        }

        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset the timer
        return true;
    }
    else if (strcmp(subcommand, "off") == 0)
    {
        caller->println(F("[INFO] Logging disabled"));
        logging.logInterval = 0; // Setting to 0 disables logging
        return true;
    }
    else if (strcmp(subcommand, "now") == 0)
    {
        caller->println(F("[INFO] Logging system state now"));
        // Log immediately regardless of interval
        logSystemState(); // Changed: removed the parameter
        return true;
    }
    else if (strcmp(subcommand, "help") == 0)
    {
        caller->println(F("\n===== LOGGING SYSTEM HELP ====="));

        caller->println(F("\nOVERVIEW:"));
        caller->println(F("  The logging system captures complete system state at regular intervals"));
        caller->println(F("  or on demand, providing detailed information for debugging and monitoring."));

        caller->println(F("\nCOMMAND REFERENCE:"));
        caller->println(F("  log,on[,interval] - Enable periodic logging"));
        caller->println(F("    > interval = Optional logging frequency in milliseconds"));
        caller->println(F("    > Default interval: 250 ms (4 logs per second)"));
        caller->println(F("    > Example: log,on,1000 - Log every 1 second"));
        caller->println(F("    > Example: log,on - Log every 250ms (default)"));

        caller->println(F("  log,off - Disable periodic logging"));
        caller->println(F("    > Stops the automatic logging of system state"));
        caller->println(F("    > Does not affect manual logging with log,now"));

        caller->println(F("  log,now - Log system state immediately"));
        caller->println(F("    > Records a single log entry regardless of periodic settings"));
        caller->println(F("    > Useful for capturing state at specific moments"));

        caller->println(F("\nLOG CONTENT:"));
        caller->println(F("  • Valves - Lock status of all trays and shuttle with sensor verification"));
        caller->println(F("    > [!] indicator shows sensor/command mismatch"));
        caller->println(F("  • Sensors - Tray presence detection at each position"));
        caller->println(F("  • System - Motor state, homing status, E-Stop and HLFB status"));
        caller->println(F("  • Position - Current, target, and last positions (mm and counts)"));
        caller->println(F("  • Velocity - Current speed, percentage of max, and speed limits"));
        caller->println(F("  • Jog - Current jog increment and speed settings"));
        caller->println(F("  • MPG - Handwheel control status, multiplier, and mm/rotation"));

        caller->println(F("\nPERFORMANCE CONSIDERATIONS:"));
        caller->println(F("  • Default 250ms interval is optimal for most debugging"));
        caller->println(F("  • Very frequent logging (< 100ms) may impact system responsiveness"));
        caller->println(F("  • For long-term monitoring, consider 1000-5000ms intervals"));

        caller->println(F("\nREADING LOG OUTPUT:"));
        caller->println(F("  • Each section is separated by | characters for readability"));
        caller->println(F("  • Position values shown in both mm and encoder counts"));
        caller->println(F("  • Lock status shows ? if sensor doesn't match expected state"));
        caller->println(F("  • Velocity shown with percentage of maximum when moving"));

        caller->println(F("\nTROUBLESHOOTING TIPS:"));
        caller->println(F("  • Use log,now before and after commands to track state changes"));
        caller->println(F("  • Watch for sensor/valve mismatches [!] indicating hardware issues"));
        caller->println(F("  • Compare HLFB status with motor state to identify drive problems"));
        caller->println(F("  • Verify position values match expected targets during movements"));

        return true;
    }
    else
    {
        caller->println(F("[ERROR] Invalid log subcommand. Use 'on', 'off', 'now', or 'help'."));
        return false;
    }

    return false;
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
        caller->println(F("[ERROR] Missing parameter. Usage: motor,<init|status|clear|home|abort|stop|help>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *subcommand = strtok(trimmed, ",");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: motor,<init|status|clear|home|abort|stop|help>"));
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
        caller->println(F("[INFO] Initializing motor..."));

        // Diagnostic: Print state before initialization
        caller->print(F("[DIAGNOSTIC] Motor state before init: "));
        switch (motorState)
        {
        case MOTOR_STATE_IDLE:
            caller->println(F("IDLE"));
            break;
        case MOTOR_STATE_MOVING:
            caller->println(F("MOVING"));
            break;
        case MOTOR_STATE_HOMING:
            caller->println(F("HOMING"));
            break;
        case MOTOR_STATE_FAULTED:
            caller->println(F("FAULTED"));
            break;
        case MOTOR_STATE_NOT_READY:
            caller->println(F("NOT READY"));
            break;
        default:
            caller->println(F("UNKNOWN"));
            break;
        }

        initMotorSystem();

        // Diagnostic: Print state after initialization
        caller->print(F("[DIAGNOSTIC] Motor state after init: "));
        switch (motorState)
        {
        case MOTOR_STATE_IDLE:
            caller->println(F("IDLE"));
            break;
        case MOTOR_STATE_MOVING:
            caller->println(F("MOVING"));
            break;
        case MOTOR_STATE_HOMING:
            caller->println(F("HOMING"));
            break;
        case MOTOR_STATE_FAULTED:
            caller->println(F("FAULTED"));
            break;
        case MOTOR_STATE_NOT_READY:
            caller->println(F("NOT READY"));
            break;
        default:
            caller->println(F("UNKNOWN"));
            break;
        }

        if (motorState == MOTOR_STATE_NOT_READY || motorState == MOTOR_STATE_FAULTED)
        {
            caller->println(F("[ERROR] Motor initialization failed. Check connections and power."));
            return false;
        }
        else
        {
            caller->println(F("[INFO] Motor initialization successful"));
            return true;
        }
        break;
    }

    case 2:
    { // status
        caller->println(F("[INFO] Motor Status:"));

        // Display motor state
        caller->print(F("  State: "));
        switch (motorState)
        {
        case MOTOR_STATE_IDLE:
            caller->println(F("IDLE"));
            break;
        case MOTOR_STATE_MOVING:
            caller->println(F("MOVING"));
            break;
        case MOTOR_STATE_HOMING:
            caller->println(F("HOMING"));
            break;
        case MOTOR_STATE_FAULTED:
            caller->println(F("FAULTED"));
            break;
        case MOTOR_STATE_NOT_READY:
            caller->println(F("NOT READY"));
            break;
        default:
            caller->println(F("UNKNOWN"));
            break;
        }

        // Display homing status
        caller->print(F("  Homed: "));
        caller->println(isHomed ? F("YES") : F("NO"));

        // Display position information based on homing status
        if (isHomed)
        {
            double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
            int32_t rawPosition = MOTOR_CONNECTOR.PositionRefCommanded();
            int32_t normalizedPosition = normalizeEncoderValue(rawPosition);

            caller->print(F("  Current Position: "));
            caller->print(calculatedPositionMm, 2);
            caller->print(F(" mm ("));
            caller->print(normalizedPosition);
            caller->println(F(" counts)"));

            // Add last completed position display using existing variables
            caller->print(F("  Last Completed Position: "));
            if (hasLastTarget)
            {
                caller->print(lastTargetPositionMm, 2);
                caller->print(F(" mm ("));
                caller->print(normalizeEncoderValue(lastTargetPulses));
                caller->println(F(" counts)"));
            }
            else
            {
                caller->println(F("None - No movements completed yet"));
            }
        }
        else
        {
            caller->println(F("  Current Position: UNKNOWN - Motor not homed"));
            caller->println(F("  Last Completed Position: UNKNOWN - Motor not homed"));

            // Only show raw encoder count if motor is initialized
            if (motorState != MOTOR_STATE_NOT_READY)
            {
                int32_t rawPosition = MOTOR_CONNECTOR.PositionRefCommanded();
                caller->print(F("  Encoder Reading: "));
                // Also normalize here
                caller->print(normalizeEncoderValue(rawPosition));
                caller->println(F(" counts (reference point not established)"));
            }
            else
            {
                caller->println(F("  Encoder Reading: Not available - Motor not initialized"));
            }
        }

        // Show velocity configuration instead of current velocity
        caller->println(F("  Velocity Settings:"));

        // Regular movement velocity
        caller->print(F("    Move Operations: "));
        caller->print(ppsToRpm(currentVelMax), 1);
        caller->println(F(" RPM"));

        // Homing velocity - updated to show only the approach velocity we actually use
        caller->print(F("    Homing: "));
        caller->print(HOME_APPROACH_VELOCITY_RPM);
        caller->println(F(" RPM"));

        // Jog velocity and increment
        caller->print(F("    Jogging: "));
        caller->print(currentJogSpeedRpm);
        caller->print(F(" RPM, "));
        caller->print(currentJogIncrementMm, 2);
        caller->println(F(" mm/jog"));

        // Only show current velocity if motor is moving
        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            int32_t velocity = MOTOR_CONNECTOR.VelocityRefCommanded();
            double velocityRpm = (double)velocity * 60.0 / PULSES_PER_REV;
            caller->print(F("    Current: "));
            caller->print(velocityRpm, 1);
            caller->print(F(" RPM ("));
            caller->print(velocity);
            caller->println(F(" pulses/sec)"));
        }

        // Display acceleration limit
        caller->print(F("  Acceleration: "));
        caller->print((double)currentAccelMax * 60.0 / PULSES_PER_REV, 1);
        caller->println(F(" RPM/sec"));

        // Display travel limits based on homing status
        caller->println(F("  Travel Limits:"));
        if (isHomed)
        {
            // Display both mm and counts when homed
            caller->print(F("    Range: 0.00 to "));
            caller->print(MAX_TRAVEL_MM, 2);
            caller->println(F(" mm"));
            caller->print(F("            0 to "));
            // MAX_TRAVEL_PULSES is already defined with the correct sign
            caller->print(MAX_TRAVEL_PULSES);
            caller->println(F(" counts"));
        }
        else
        {
            caller->println(F("    UNKNOWN - Motor not homed"));
        }

        // Display fault status
        caller->print(F("  Fault Status: "));
        if (MOTOR_CONNECTOR.HlfbState() == ClearCore::MotorDriver::HLFB_ASSERTED)
        {
            caller->println(F("NO FAULT"));
        }
        else
        {
            caller->println(F("FAULT DETECTED"));
        }

        // Display E-Stop status
        caller->print(F("  E-Stop: "));
        caller->println(isEStopActive() ? F("TRIGGERED (EMERGENCY STOP)") : F("RELEASED (READY)"));

        // If motor is not ready, provide additional information for troubleshooting
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            caller->println(F("\n  [NOTE] Motor must be initialized with 'motor,init' command"));
            caller->println(F("         before position control is available."));
        }

        return true;
        break;
    }

    case 3:
    { // clear
        caller->println(F("[INFO] Attempting to clear motor fault..."));

        if (clearMotorFaultWithStatus())
        {
            caller->println(F("[INFO] Motor fault cleared successfully"));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to clear motor fault. Motor may still be in fault state."));
            caller->println(F("  Try power cycling the system if fault persists."));
            return false;
        }
        break;
    }

    case 4:
    { // home
        // Check necessary preconditions for homing
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            caller->println(F("[ERROR] Motor is not initialized. Use 'motor,init' first."));
            return false;
        }

        if (motorState == MOTOR_STATE_HOMING)
        {
            caller->println(F("[WARNING] Homing sequence is already in progress."));
            return false;
        }

        if (isEStopActive())
        {
            caller->println(F("[ERROR] Cannot home while E-Stop is active. Release E-Stop and try again."));
            return false;
        }

        if (motorState == MOTOR_STATE_FAULTED)
        {
            caller->println(F("[ERROR] Motor is in fault state. Use 'motor,clear' to clear fault before homing."));
            return false;
        }

        caller->println(F("[INFO] Starting homing sequence..."));

        // Begin homing
        initiateHomingSequence();

        // Check if homing was initiated by examining the motor state
        if (motorState == MOTOR_STATE_HOMING)
        {
            caller->println(F("[INFO] Homing sequence initiated. Motor will move to find home position."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start homing sequence. Check motor status."));
            return false;
        }
        break;
    }

    case 5:
    { // abort
        // Check if motor is initialized before attempting to abort
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            caller->println(F("[ERROR] Motor is not initialized. Nothing to abort."));
            return false;
        }

        caller->println(F("[INFO] Aborting current operation..."));

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

            caller->println(F("[INFO] Operation aborted successfully."));
            return true;
        }
        else
        {
            caller->println(F("[WARNING] No active operation to abort."));
            return false;
        }
        break;
    }

    case 6:
    { // stop
        // Check if motor is initialized
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            caller->println(F("[ERROR] Motor is not initialized. Cannot perform stop."));
            return false;
        }

        caller->println(F("[INFO] EMERGENCY STOP initiated!"));

        // Execute emergency stop
        MOTOR_CONNECTOR.MoveStopAbrupt();
        motorState = MOTOR_STATE_IDLE;

        caller->println(F("[INFO] Motor movement halted. Position may no longer be accurate."));
        caller->println(F("[WARNING] Re-homing recommended after emergency stop."));

        return true;
        break;
    }

    case 7: // help
    {
        caller->println(F("\n===== MOTOR CONTROL SYSTEM HELP ====="));

        caller->println(F("\nCOMMAND REFERENCE:"));
        caller->println(F("  motor,init - Initialize motor system and prepare for operation"));
        caller->println(F("    > Must be run after power-up before any other motor commands"));
        caller->println(F("    > Configures motor parameters and communication"));
        caller->println(F("    > Does not move the motor or establish position reference"));

        caller->println(F("  motor,home - Find home position and establish reference point"));
        caller->println(F("    > Required before absolute positioning commands can be used"));
        caller->println(F("    > Motor will move slowly until it contacts the home limit switch"));
        caller->println(F("    > After contact, motor backs off to establish precise zero position"));
        caller->println(F("    > Home position is offset 5mm from physical limit for safety"));

        caller->println(F("  motor,status - Display detailed motor status and configuration"));
        caller->println(F("    > Shows current state, position, velocity settings, and limits"));
        caller->println(F("    > Use to verify proper operation or troubleshoot issues"));

        caller->println(F("  motor,clear - Clear motor fault condition"));
        caller->println(F("    > Use after resolving the condition that caused the fault"));
        caller->println(F("    > Common faults: excessive load, hitting physical limit, E-Stop"));

        caller->println(F("  motor,abort - Gracefully stop current movement"));
        caller->println(F("    > Controlled deceleration to stop the motor"));
        caller->println(F("    > Position information is maintained"));
        caller->println(F("    > Use to cancel a movement without generating a fault"));

        caller->println(F("  motor,stop - Emergency stop motor movement immediately"));
        caller->println(F("    > Immediate halt of motor operation"));
        caller->println(F("    > May cause position inaccuracy"));
        caller->println(F("    > Use only when necessary to prevent damage or injury"));

        caller->println(F("\nTYPICAL SEQUENCE:"));
        caller->println(F("  1. motor,init   - Initialize the motor system"));
        caller->println(F("  2. motor,home   - Establish reference position"));
        caller->println(F("  3. move,X       - Move to desired positions"));
        caller->println(F("  4. jog commands - Make fine adjustments"));
        caller->println(F("  5. encoder      - Use handwheel for manual control"));

        caller->println(F("\nTROUBLESHOOTING:"));
        caller->println(F("  • If motor won't move: Check E-Stop, then run motor,status"));
        caller->println(F("  • After fault: Use motor,clear to reset fault condition"));
        caller->println(F("  • If position seems incorrect: Re-home the system"));
        caller->println(F("  • Unexpected behavior: Check that motor is initialized"));
        caller->println(F("  • Jerky movement: Try using slower speed or smaller increments"));

        caller->println(F("\nSAFETY NOTES:"));
        caller->println(F("  • Always ensure proper clearance before moving the shuttle"));
        caller->println(F("  • Use E-Stop if unexpected movement occurs"));
        caller->println(F("  • After E-Stop, clear faults before resuming operation"));
        caller->println(F("  • Motor movements will halt automatically at travel limits"));

        return true;
        break;
    }

    default:
    {
        caller->print(F("[ERROR] Unknown motor command: "));
        caller->println(subcommand);
        caller->println(F("Valid options are 'init', 'status', 'clear', 'home', 'abort', 'stop', or 'help'"));
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
        caller->println(F("[ERROR] Missing parameter. Usage: move,<home|1|2|3|4|counts,X|mm,X|rel,X|help>"));
        return false;
    }

    // Get the first parameter (subcommand position)
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: move,<home|1|2|3|4|counts,X|mm,X|rel,X|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // State checks (in order of importance) for all movement commands
    // 1. Check if motor is initialized
    if (motorState == MOTOR_STATE_NOT_READY)
    {
        caller->println(F("[ERROR] Motor is not initialized. Use 'motor,init' first."));
        return false;
    }

    // 2. Check if E-Stop is active - most critical safety check
    if (isEStopActive())
    {
        caller->println(F("[ERROR] Cannot move while E-Stop is active. Release E-Stop and try again."));
        return false;
    }

    // 3. Check for fault condition
    if (motorState == MOTOR_STATE_FAULTED)
    {
        caller->println(F("[ERROR] Motor is in fault state. Use 'motor,clear' to clear fault before moving."));
        return false;
    }

    // 4. Check if motor is already moving
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
    {
        caller->println(F("[ERROR] Motor is already moving. Use 'motor,abort' to stop current movement first."));
        return false;
    }

    // Handle predefined positions
    if (strcmp(subcommand, "home") == 0)
    {
        // Check if motor is already homed
        if (isHomed)
        {
            caller->println(F("[INFO] Moving to home position..."));
            if (moveToPositionMm(0.0))
            {
                caller->println(F("[INFO] Move to home initiated."));
                return true;
            }
            else
            {
                caller->println(F("[ERROR] Failed to start movement to home position."));
                return false;
            }
        }
        else
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first to establish home position."));
            return false;
        }
    }
    else if (strcmp(subcommand, "1") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[INFO] Moving to position 1..."));
        if (moveToPosition(POSITION_1))
        {
            caller->println(F("[INFO] Move to position 1 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 1."));
            return false;
        }
    }
    else if (strcmp(subcommand, "2") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[INFO] Moving to position 2..."));
        if (moveToPosition(POSITION_2))
        {
            caller->println(F("[INFO] Move to position 2 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 2."));
            return false;
        }
    }
    else if (strcmp(subcommand, "3") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[INFO] Moving to position 3..."));
        if (moveToPosition(POSITION_3))
        {
            caller->println(F("[INFO] Move to position 3 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 3."));
            return false;
        }
    }
    else if (strcmp(subcommand, "4") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[INFO] Moving to position 4..."));
        if (moveToPosition(POSITION_4))
        {
            caller->println(F("[INFO] Move to position 4 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 4."));
            return false;
        }
    }

    // Special handling for "mm" command for absolute positioning in millimeters
    else if (strcmp(subcommand, "mm") == 0)
    {
        // Get the mm value from the next token
        char *mmStr = strtok(NULL, " ");
        if (mmStr == NULL)
        {
            caller->println(F("[ERROR] Missing mm value. Usage: move,mm,X"));
            return false;
        }

        // Parse the mm value
        mmStr = trimLeadingSpaces(mmStr);
        double targetMm = atof(mmStr);

        // Check if motor is homed for precise positioning
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' first."));
            caller->println(F("[WARNING] Moving to absolute positions without homing is unsafe."));
            return false;
        }

        // Check if position is within bounds
        if (targetMm < 0.0 || targetMm > MAX_TRAVEL_MM)
        {
            caller->print(F("[ERROR] Position out of range. Valid range: 0 to "));
            caller->print(MAX_TRAVEL_MM, 1);
            caller->println(F(" mm"));
            return false;
        }

        // Position is within bounds, proceed with movement
        caller->print(F("[INFO] Moving to absolute position: "));
        caller->print(targetMm, 2);
        caller->println(F(" mm"));

        if (moveToPositionMm(targetMm))
        {
            caller->println(F("[INFO] Movement initiated successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to requested position."));
            return false;
        }
    }

    // Special handling for "counts" command for absolute positioning in encoder counts
    else if (strcmp(subcommand, "counts") == 0)
    {
        // Get the counts value from the next token
        char *countsStr = strtok(NULL, " ");
        if (countsStr == NULL)
        {
            caller->println(F("[ERROR] Missing counts value. Usage: move,counts,X"));
            return false;
        }

        // Parse the counts value
        countsStr = trimLeadingSpaces(countsStr);
        int32_t targetCounts = atol(countsStr);

        // Check if motor is homed for precise positioning
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' first."));
            caller->println(F("[WARNING] Moving to absolute positions without homing is unsafe."));
            return false;
        }

        // Check if position is within bounds
        if (targetCounts < 0 || targetCounts > MAX_TRAVEL_PULSES)
        {
            caller->print(F("[ERROR] Position out of range. Valid range: 0 to "));
            caller->print(MAX_TRAVEL_PULSES);
            caller->println(F(" counts"));
            return false;
        }

        // Position is within bounds, proceed with movement
        caller->print(F("[INFO] Moving to absolute position: "));
        caller->print(targetCounts);
        caller->println(F(" counts"));

        if (moveToAbsolutePosition(targetCounts))
        {
            caller->println(F("[INFO] Movement initiated successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to requested position."));
            return false;
        }
    }

    // Special handling for "rel" command for relative positioning
    else if (strcmp(subcommand, "rel") == 0)
    {
        // Get the mm value from the next token
        char *relStr = strtok(NULL, " ");
        if (relStr == NULL)
        {
            caller->println(F("[ERROR] Missing relative distance value. Usage: move,rel,X"));
            return false;
        }

        // Parse the relative distance value
        relStr = trimLeadingSpaces(relStr);
        double relDistanceMm = atof(relStr);

        // Check if motor is homed for precise positioning
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' first."));
            caller->println(F("[WARNING] Moving without homing is unsafe."));
            return false;
        }

        // Get current position
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm + relDistanceMm;

        // Check if target position is within bounds
        if (targetPositionMm < 0.0 || targetPositionMm > MAX_TRAVEL_MM)
        {
            caller->print(F("[ERROR] Target position out of range. Valid range: 0 to "));
            caller->print(MAX_TRAVEL_MM, 1);
            caller->println(F(" mm"));
            caller->print(F("[INFO] Current position: "));
            caller->print(currentPositionMm, 2);
            caller->print(F(" mm, Requested move: "));
            caller->print(relDistanceMm, 2);
            caller->println(F(" mm"));
            return false;
        }

        // Display move information
        caller->print(F("[INFO] Moving "));
        caller->print(relDistanceMm, 2);
        caller->print(F(" mm from current position ("));
        caller->print(currentPositionMm, 2);
        caller->print(F(" mm) to "));
        caller->print(targetPositionMm, 2);
        caller->println(F(" mm"));

        if (moveToPositionMm(targetPositionMm))
        {
            caller->println(F("[INFO] Relative movement initiated successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start relative movement."));
            return false;
        }
    }
    else if (strcmp(subcommand, "help") == 0)
    {
        caller->println(F("\n===== MOVE COMMAND HELP ====="));

        caller->println(F("\nPREREQUISITES:"));
        caller->println(F("  • Motor must be initialized (motor,init)"));
        caller->println(F("  • Motor must be homed for accurate positioning (motor,home)"));
        caller->println(F("  • E-Stop must be inactive"));
        caller->println(F("  • Motor must not be in fault state"));
        caller->println(F("  • No other movement can be in progress"));

        caller->println(F("\nCOMMAND TYPES:"));
        caller->println(F("  move,home - Move to home (zero) position"));
        caller->println(F("    > Reference position offset 5mm from hardstop"));
        caller->println(F("    > Always available after homing"));

        caller->println(F("  move,1 through move,4 - Move to predefined positions"));
        caller->println(F("    > Position 1: Loading position (28.7mm)"));
        caller->println(F("    > Position 2: Middle position (456.0mm)"));
        caller->println(F("    > Position 3: Unloading position (883.58mm)"));
        caller->println(F("    > Position 4: Max travel (1050.0mm)"));

        caller->println(F("  move,mm,X - Move to absolute position X in millimeters"));
        caller->println(F("    > Valid range: 0 to 1050.0 mm"));
        caller->println(F("    > Most intuitive way to specify exact positions"));
        caller->println(F("    > Example: move,mm,500.5 - moves to 500.5mm"));

        caller->println(F("  move,counts,X - Move to absolute position X in encoder counts"));
        caller->println(F("    > Valid range: 0 to 64,333 counts"));
        caller->println(F("    > Used for precise control or debugging"));
        caller->println(F("    > 1mm ≈ 61.27 counts (3200 pulses/rev ÷ 52.23mm/rev)"));

        caller->println(F("  move,rel,X - Move X millimeters relative to current position"));
        caller->println(F("    > Use positive values to move forward"));
        caller->println(F("    > Use negative values to move backward"));
        caller->println(F("    > Example: move,rel,-10 - moves 10mm backward"));
        caller->println(F("    > Movement is constrained to valid range (0-1050.0mm)"));

        caller->println(F("\nTROUBLESHOOTING:"));
        caller->println(F("  • If movement fails, check motor status with 'motor,status'"));
        caller->println(F("  • If at travel limits, you can only move within the allowed range"));
        caller->println(F("  • After E-Stop, clear faults with 'motor,clear' before moving"));
        caller->println(F("  • For short, precise movements, consider using 'jog' commands"));
        caller->println(F("  • For interactive positioning, use 'encoder' handwheel control"));

        return true;
    }
    else
    {
        caller->print(F("[ERROR] Invalid position: "));
        caller->println(subcommand);
        caller->println(F("Valid options: home, 1, 2, 3, 4, counts, mm, rel, help"));
        return false;
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
        caller->println(F("[ERROR] Missing parameter. Usage: jog,<+|-|inc|speed|status|help>"));
        return false;
    }

    // Parse the argument - we'll use spaces as separators (as they've been converted from commas)
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: jog,<+|-|inc|speed|status|help>"));
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
            caller->println(F("[ERROR] Motor is not initialized. Use 'motor,init' first."));
            return false;
        }

        // 2. Check if E-Stop is active - most critical safety check
        if (isEStopActive())
        {
            caller->println(F("[ERROR] Cannot jog while E-Stop is active. Release E-Stop and try again."));
            return false;
        }

        // 3. Check for fault condition
        if (motorState == MOTOR_STATE_FAULTED)
        {
            caller->println(F("[ERROR] Motor is in fault state. Use 'motor,clear' to clear fault before jogging."));
            return false;
        }

        // 4. Check if motor is homing
        if (motorState == MOTOR_STATE_HOMING)
        {
            caller->println(F("[ERROR] Cannot jog while homing is in progress."));
            return false;
        }

        // 5. Check if motor is already moving
        if (motorState == MOTOR_STATE_MOVING)
        {
            caller->println(F("[ERROR] Motor is already moving. Use 'motor,abort' to stop current movement first."));
            return false;
        }

        // 6. Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
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
            caller->print(F("[ERROR] Cannot jog beyond maximum position limit of "));
            caller->print(MAX_TRAVEL_MM, 1);
            caller->println(F(" mm"));
            caller->print(F("  Current position: "));
            caller->print(currentPositionMm, 2);
            caller->println(F(" mm"));
            return false;
        }

        // Display jog information
        caller->print(F("[INFO] Jogging forward "));
        caller->print(currentJogIncrementMm, 2);
        caller->print(F(" mm from position "));
        caller->print(currentPositionMm, 2);
        caller->print(F(" mm to "));
        caller->print(targetPositionMm, 2);
        caller->println(F(" mm"));

        // Perform the jog movement using the jogMotor function
        if (jogMotor(true))
        { // true = forward direction
            caller->println(F("[INFO] Jog movement initiated"));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to initiate jog movement"));
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
            caller->println(F("[ERROR] Cannot jog beyond minimum position limit of 0 mm"));
            caller->print(F("  Current position: "));
            caller->print(currentPositionMm, 2);
            caller->println(F(" mm"));
            return false;
        }

        // Display jog information
        caller->print(F("[INFO] Jogging backward "));
        caller->print(currentJogIncrementMm, 2);
        caller->print(F(" mm from position "));
        caller->print(currentPositionMm, 2);
        caller->print(F(" mm to "));
        caller->print(targetPositionMm, 2);
        caller->println(F(" mm"));

        // Perform the jog movement using the jogMotor function
        if (jogMotor(false))
        { // false = backward direction
            caller->println(F("[INFO] Jog movement initiated"));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to initiate jog movement"));
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
            caller->print(F("[INFO] Current jog increment: "));
            caller->print(currentJogIncrementMm, 2);
            caller->println(F(" mm"));
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
                    caller->print(F("[INFO] Jog increment set to default ("));
                    caller->print(currentJogIncrementMm, 2);
                    caller->println(F(" mm)"));
                    return true;
                }
                else
                {
                    caller->println(F("[ERROR] Failed to set default jog increment"));
                    return false;
                }
            }

            // Parse value as double
            double newIncrement = atof(incStr);

            // Set new jog increment
            if (setJogIncrement(newIncrement))
            {
                caller->print(F("[INFO] Jog increment set to "));
                caller->print(currentJogIncrementMm, 2);
                caller->println(F(" mm"));
                return true;
            }
            else
            {
                caller->println(F("[ERROR] Invalid jog increment value"));
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
            caller->print(F("[INFO] Current jog speed: "));
            caller->print(currentJogSpeedRpm);
            caller->println(F(" RPM"));
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
                    caller->print(F("[INFO] Jog speed set to default ("));
                    caller->print(currentJogSpeedRpm);
                    caller->println(F(" RPM)"));
                    return true;
                }
                else
                {
                    caller->println(F("[ERROR] Failed to set default jog speed"));
                    return false;
                }
            }

            // Parse value as int
            int newSpeed = atoi(speedStr);

            // Set new jog speed
            if (setJogSpeed(newSpeed, currentJogIncrementMm))
            {
                caller->print(F("[INFO] Jog speed set to "));
                caller->print(currentJogSpeedRpm);
                caller->println(F(" RPM"));
                return true;
            }
            else
            {
                caller->println(F("[ERROR] Invalid jog speed value"));
                return false;
            }
        }
        break;
    }

    case 5:
    { // status
        // Display current jog settings
        caller->println(F("[INFO] Current jog settings:"));

        // Jog increment
        caller->print(F("  Increment: "));
        caller->print(currentJogIncrementMm, 2);
        caller->println(F(" mm"));

        // Jog speed
        caller->print(F("  Speed: "));
        caller->print(currentJogSpeedRpm);
        caller->println(F(" RPM"));

        // Current position
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        caller->print(F("  Current position: "));
        caller->print(currentPositionMm, 2);
        caller->println(F(" mm"));

        // Position limits for jogging from current position
        double maxForwardJog = MAX_TRAVEL_MM - currentPositionMm;
        double maxBackwardJog = currentPositionMm;

        caller->print(F("  Max forward jog: "));
        caller->print(maxForwardJog, 2);
        caller->println(F(" mm"));

        caller->print(F("  Max backward jog: "));
        caller->print(maxBackwardJog, 2);
        caller->println(F(" mm"));

        return true;
        break;
    }

    case 6: // Add the new help case here
    {       // help
        caller->println(F("\n===== JOG MOVEMENT SYSTEM HELP ====="));

        caller->println(F("\nOVERVIEW:"));
        caller->println(F("  The jog system provides precise, incremental movements in either direction"));
        caller->println(F("  for accurate positioning and testing. Each jog moves the motor by a fixed"));
        caller->println(F("  distance that you can configure."));

        caller->println(F("\nCOMMAND REFERENCE:"));
        caller->println(F("  jog,+ - Move forward by one increment"));
        caller->println(F("    > Each press moves exactly one increment in the forward direction"));
        caller->println(F("    > Movement stops automatically after the increment is completed"));
        caller->println(F("  jog,- - Move backward by one increment"));
        caller->println(F("    > Each press moves exactly one increment in the backward direction"));
        caller->println(F("    > Movement stops automatically after the increment is completed"));
        caller->println(F("  jog,inc,X - Set movement increment size"));
        caller->println(F("    > X = distance in millimeters (example: jog,inc,5.0)"));
        caller->println(F("    > Using jog,inc without a value displays the current setting"));
        caller->println(F("    > Using jog,inc,default resets to standard increment"));
        caller->println(F("  jog,speed,X - Set movement speed"));
        caller->println(F("    > X = speed in RPM (example: jog,speed,300)"));
        caller->println(F("    > Using jog,speed without a value displays the current setting"));
        caller->println(F("    > Using jog,speed,default resets to standard speed"));
        caller->println(F("  jog,status - Display current jog settings and position information"));

        caller->println(F("\nJOG VS. HANDWHEEL COMPARISON:"));
        caller->println(F("  Jog System (jog command):"));
        caller->println(F("    • Fixed, precise movements with each command"));
        caller->println(F("    • Better for repeatable, exact positioning"));
        caller->println(F("    • Simple to use via command line"));
        caller->println(F("    • Good for testing and calibration"));
        caller->println(F("    • Can be used in scripts and automated sequences"));

        caller->println(F("  Handwheel System (encoder command):"));
        caller->println(F("    • Continuous, manual control with physical handwheel"));
        caller->println(F("    • Better for interactive positioning and fine adjustments"));
        caller->println(F("    • More intuitive for operators doing manual work"));
        caller->println(F("    • Allows variable speed based on rotation speed"));
        caller->println(F("    • Provides tactile feedback during positioning"));

        caller->println(F("\nWHEN TO USE JOG:"));
        caller->println(F("  • For test sequences that need repeatable movements"));
        caller->println(F("  • When working remotely via serial connection"));
        caller->println(F("  • When you need precisely measured movements"));
        caller->println(F("  • For calibration procedures"));
        caller->println(F("  • When you don't have access to the physical handwheel"));

        caller->println(F("\nUSAGE TIPS:"));
        caller->println(F("  • Set a smaller increment (1-5mm) for precise positioning"));
        caller->println(F("  • Set a larger increment (10-50mm) for faster travel"));
        caller->println(F("  • Use jog,status to see your current position and limits"));
        caller->println(F("  • The motor must be homed before jogging can be used"));
        caller->println(F("  • Jogging is automatically limited to prevent over-travel"));

        caller->println(F("\nTROUBLESHOOTING:"));
        caller->println(F("  • If jog commands fail, check if motor is initialized and homed"));
        caller->println(F("  • If at travel limit, you can only jog in the opposite direction"));
        caller->println(F("  • After E-Stop, clear any faults before attempting to jog"));
        caller->println(F("  • If motor is already moving, wait for it to complete or use motor,abort"));

        return true;
        break;
    }

    default:
    {
        caller->print(F("[ERROR] Unknown jog command: "));
        caller->println(subcommand);
        caller->println(F("Valid options are '+', '-', 'inc', 'speed', 'status', or 'help'")); // Update this line too
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
        caller->println(F("[INFO] Usage: system,state - Display current system state"));
        caller->println(F("                 system,safety - Display safety validation status"));
        caller->println(F("                 system,trays - Display tray system status"));
        caller->println(F("                 system,reset - Reset system state after failure"));
        return false;
    }

    // Handle subcommands
    if (strcmp(subcommand, "state") == 0)
    {
        // Capture and print system state
        SystemState currentState = captureSystemState();
        printSystemState(currentState);
        return true;
    }
    // Add this new section to display safety validation
    else if (strcmp(subcommand, "safety") == 0)
    {
        // Capture system state, validate safety, and print results
        SystemState currentState = captureSystemState();
        SafetyValidationResult safety = validateSafety(currentState);

        caller->println(F("\n===== SAFETY VALIDATION STATUS ====="));
        printSafetyStatus(safety);

        return true;
    }
    else if (strcmp(subcommand, "trays") == 0)
    {
        // Display tray system status
        SystemState currentState = captureSystemState();
        updateTrayTrackingFromSensors(currentState);

        caller->println(F("\n===== TRAY SYSTEM STATUS ====="));
        caller->print(F("Total trays in system: "));
        caller->println(trayTracking.totalTraysInSystem);

        caller->println(F("\nPosition occupancy:"));
        caller->print(F("  Position 1 (Loading): "));
        caller->println(trayTracking.position1Occupied ? F("OCCUPIED") : F("EMPTY"));
        caller->print(F("  Position 2 (Middle): "));
        caller->println(trayTracking.position2Occupied ? F("OCCUPIED") : F("EMPTY"));
        caller->print(F("  Position 3 (Unloading): "));
        caller->println(trayTracking.position3Occupied ? F("OCCUPIED") : F("EMPTY"));

        caller->println(F("\nOperation statistics:"));
        caller->print(F("  Total loads completed: "));
        caller->println(trayTracking.totalLoadsCompleted);
        caller->print(F("  Total unloads completed: "));
        caller->println(trayTracking.totalUnloadsCompleted);

        if (trayTracking.lastLoadTime > 0)
        {
            caller->print(F("  Last load: "));
            caller->print((millis() - trayTracking.lastLoadTime) / 1000);
            caller->println(F(" seconds ago"));
        }

        if (trayTracking.lastUnloadTime > 0)
        {
            caller->print(F("  Last unload: "));
            caller->print((millis() - trayTracking.lastUnloadTime) / 1000);
            caller->println(F(" seconds ago"));
        }
        return true;
    }
    else if (strcmp(subcommand, "reset") == 0)
    {
        // Reset the system state after a failure
        caller->println(F("\n===== RESETTING SYSTEM STATE ====="));

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
            caller->println(F("[INFO] Motor fault condition cleared"));
        }

        if (wasOperationInProgress)
        {
            caller->println(F("[INFO] Operation state cleared"));
        }

        caller->println(F("[INFO] System state has been reset and is ready for new commands"));

        return true;
    }
    else
    {
        // Unknown subcommand
        caller->print(F("[ERROR] Unknown system command: "));
        caller->println(subcommand);
        caller->println(F("Valid options are 'system,state', 'system,safety', 'system,trays', or 'system,reset'"));
        return false;
    }
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
        caller->println(F("[ERROR] Missing parameter. Usage: tray,<load|unload|placed|released|status|help>"));
        return false;
    }

    // Parse the subcommand
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: tray,<load|unload|placed|released|status|help>"));
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
            caller->println(F("[ERROR] Missing subcommand. Usage: tray,<load|unload|placed|released|status|help>"));
            return false;
        }
        subcommand = trimLeadingSpaces(subcommand);
    }

    // Now process the actual command
    if (strcmp(subcommand, "load") == 0)
    {
        // Check for second parameter "request"
        char *action = strtok(NULL, " ");
        if (action == NULL || strcmp(action, "request") != 0)
        {
            caller->println(F("[ERROR] Invalid format. Usage: tray,load,request"));
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
            caller->println(F("[INFO] System is full - all positions occupied"));
            caller->println(F("SYSTEM_FULL"));
            return false;
        }

        // 2. Verify position 1 is free and no operations are in progress
        if (trayTracking.position1Occupied)
        {
            caller->println(F("POSITION_OCCUPIED"));
            return false;
        }

        if (operationInProgress)
        {
            caller->println(F("SYSTEM_BUSY"));
            return false;
        }

        // 3. Validate safety constraints
        SafetyValidationResult safety = validateSafety(state);
        if (!safety.safeToLoadTrayToPos1)
        {
            caller->print(F("UNSAFE: "));
            caller->println(safety.loadTrayPos1UnsafeReason);
            return false;
        }

        // 4. Set the target position for position 1
        if (!moveToPositionMm(POSITION_1_MM))
        {
            caller->println(F("ERROR_MOVE_FAILURE"));
            return false;
        }

        // 5. System is ready to receive tray                caller->println(F("READY_TO_RECEIVE"));

        // Add helpful message about the overall loading process
        if (trayTracking.totalTraysInSystem == 0)
        {
            caller->println(F("[NOTE] First tray will be moved to position 3 after placement"));
        }
        else if (trayTracking.totalTraysInSystem == 1)
        {
            caller->println(F("[NOTE] Second tray will be moved to position 2 after placement"));
        }
        else
        {
            caller->println(F("[NOTE] Third tray will remain at position 1 after placement"));
        }

        return true;
    }
    else if (strcmp(subcommand, "placed") == 0)
    {
        // Mitsubishi robot has placed the tray
        // Just mark position 1 as occupied without incrementing total
        trayTracking.position1Occupied = true;
        trayTracking.lastLoadTime = millis();

        // 1. Verify tray sensor shows tray is present
        SystemState state = captureSystemState();
        if (!state.tray1Present)
        {
            caller->println(F("ERROR_NO_TRAY_DETECTED"));
            caller->println(F("[NOTE] If you encounter issues, use 'system,reset' to reset the system state"));
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
                    caller->println(F("[ERROR] Failed to lock tray - sensor didn't confirm"));
                    caller->println(F("[WARNING] Check air pressure and valve functionality"));
                    return false;
                }
                caller->println(F("TRAY_SECURED"));

                // Only update the timestamp, not the count (workflow functions will handle counting)
                trayTracking.lastLoadTime = millis();
                return true;
            }
            else
            {
                // Valve already in lock position
                caller->println(F("TRAY_ALREADY_SECURED"));
                // Only update the timestamp, not the count (workflow functions will handle counting)
                trayTracking.lastLoadTime = millis();
                return true;
            }
        }
        else
        {
            caller->println(F("ERROR_LOCK_FAILURE"));
            return false;
        }
    }
    else if (strcmp(subcommand, "released") == 0)
    {
        // Start the automated operation
        beginOperation();

        // Set the operation details
        currentOperation.inProgress = true;
        currentOperation.type = OPERATION_LOADING;
        currentOperation.startTime = millis();

        // No need to manually set target - it will be set in processTrayLoading()

        caller->println(F("STARTING_PROCESSING"));
        return true;
    }
    else if (strcmp(subcommand, "unload") == 0)
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
                caller->println(F("[INFO] No trays available to unload"));
                caller->println(F("NO_TRAYS"));
                return false;
            }

            // 2. Check if an operation is already in progress
            if (operationInProgress)
            {
                caller->println(F("SYSTEM_BUSY"));
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
                        caller->println(F("[ERROR] Failed to unlock tray - sensor didn't confirm"));
                        caller->println(F("[WARNING] Check air pressure and valve functionality"));
                        return false;
                    }

                    caller->println(F("[NOTE] Unloading tray from position 1 (loading position)"));
                    caller->println(F("[INFO] Tray at position 1 unlocked and ready for removal"));
                    caller->println(F("TRAY_READY"));
                    return true;
                }
                else
                {
                    caller->println(F("[ERROR] Failed to access tray 1 valve or sensor"));
                    caller->println(F("VALVE_ACCESS_ERROR"));
                    return false;
                }
            }
            else
            {
                // Need to start unloading operation to move a tray to position 1
                if (state.tray2Present)
                {
                    caller->println(F("[NOTE] Moving tray from position 2 to position 1 for unloading"));
                }
                else if (state.tray3Present)
                {
                    caller->println(F("[NOTE] Moving tray from position 3 to position 1 for unloading"));
                }

                beginOperation();

                // Set the operation details
                currentOperation.inProgress = true;
                currentOperation.type = OPERATION_UNLOADING;
                currentOperation.startTime = millis();

                caller->println(F("PREPARING_TRAY"));
                return true;
            }
        }
        else
        {
            caller->println(F("[ERROR] Invalid format. Usage: tray,unload,request"));
            return false;
        }
    }
    else if (strcmp(subcommand, "removed") == 0)
    {
        // Mitsubishi robot has removed the tray from position 1

        // 1. Verify tray sensor shows tray is no longer present
        SystemState state = captureSystemState();
        if (state.tray1Present)
        {
            caller->println(F("ERROR_TRAY_STILL_PRESENT"));
            caller->println(F("[NOTE] Sensor still detects a tray at position 1"));
            return false;
        }

        // 2. Update tracking information
        unloadFirstTray();

        // 3. Manually increment the unload counter
        // This ensures it's incremented even if unloadFirstTray() didn't do it
        // (which happens when a tray was moved from position 3 to position 1)
        trayTracking.totalUnloadsCompleted++;

        caller->println(F("TRAY_REMOVAL_CONFIRMED"));
        caller->print(F("[INFO] Total unloads completed: "));
        caller->println(trayTracking.totalUnloadsCompleted);

        return true;
    }
    else if (strcmp(subcommand, "status") == 0)
    {
        // Return machine-readable status of tray system
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);

        // Output total trays in system
        caller->print(F("TRAYS_TOTAL:"));
        caller->println(trayTracking.totalTraysInSystem);

        // Output position occupancy (1=occupied, 0=empty)
        caller->print(F("POS1:"));
        caller->println(trayTracking.position1Occupied ? 1 : 0);
        caller->print(F("POS2:"));
        caller->println(trayTracking.position2Occupied ? 1 : 0);
        caller->print(F("POS3:"));
        caller->println(trayTracking.position3Occupied ? 1 : 0);

        // Output lock status (1=locked, 0=unlocked)
        caller->print(F("LOCK1:"));
        caller->println(state.tray1Locked ? 1 : 0);
        caller->print(F("LOCK2:"));
        caller->println(state.tray2Locked ? 1 : 0);
        caller->print(F("LOCK3:"));
        caller->println(state.tray3Locked ? 1 : 0);

        // Output operation statistics
        caller->print(F("LOADS:"));
        caller->println(trayTracking.totalLoadsCompleted);
        caller->print(F("UNLOADS:"));
        caller->println(trayTracking.totalUnloadsCompleted);

        return true;
    }
    else if (strcmp(subcommand, "help") == 0)
    {
        caller->println(F("\n===== TRAY SYSTEM HELP ====="));
        caller->println(F("\nTRAY LOADING SEQUENCE:"));
        caller->println(F("  1. tray,load,request - Request permission to load a tray"));
        caller->println(F("     > System will validate position 1 is empty and move shuttle there"));
        caller->println(F("     > System responds with 'READY_TO_RECEIVE' when ready"));
        caller->println(F("  2. tray,placed - Notify system that tray has been physically placed"));
        caller->println(F("     > System will lock the tray at position 1"));
        caller->println(F("     > System responds with 'TRAY_SECURED' when complete"));
        caller->println(F("  3. tray,released - Notify system to start processing the tray"));
        caller->println(F("     > System will move tray to appropriate position based on system state"));
        caller->println(F("     > First tray goes to position 3, second to position 2, third stays at position 1"));

        caller->println(F("\nTRAY UNLOADING SEQUENCE:"));
        caller->println(F("  1. tray,unload,request - Request permission to unload a tray"));
        caller->println(F("     > If tray at position 1, system unlocks it and responds 'TRAY_READY'"));
        caller->println(F("     > If tray at positions 2 or 3, system moves it to position 1 first"));
        caller->println(F("     > System responds with 'PREPARING_TRAY' during movement"));
        caller->println(F("  2. tray,removed - Notify system that tray has been physically removed"));
        caller->println(F("     > System updates internal tracking"));
        caller->println(F("     > System responds with 'TRAY_REMOVAL_CONFIRMED'"));

        caller->println(F("\nTRAY STATUS COMMAND:"));
        caller->println(F("  tray,status - Returns machine-readable status information"));
        caller->println(F("  Returned values:"));
        caller->println(F("    TRAYS_TOTAL:[0-3] - Total number of trays in system"));
        caller->println(F("    POS1:[0|1] - Position 1 occupancy (0=empty, 1=occupied)"));
        caller->println(F("    POS2:[0|1] - Position 2 occupancy (0=empty, 1=occupied)"));
        caller->println(F("    POS3:[0|1] - Position 3 occupancy (0=empty, 1=occupied)"));
        caller->println(F("    LOCK1:[0|1] - Position 1 lock status (0=unlocked, 1=locked)"));
        caller->println(F("    LOCK2:[0|1] - Position 2 lock status (0=unlocked, 1=locked)"));
        caller->println(F("    LOCK3:[0|1] - Position 3 lock status (0=unlocked, 1=locked)"));
        caller->println(F("    LOADS:[number] - Total number of loads completed"));
        caller->println(F("    UNLOADS:[number] - Total number of unloads completed"));

        caller->println(F("\nTROUBLESHOOTING:"));
        caller->println(F("  • If an operation fails, use 'system,reset' to reset the system state"));
        caller->println(F("  • Use 'system,trays' for human-readable tray system status"));
        caller->println(F("  • Use 'system,safety' to diagnose safety constraint issues"));

        return true;
    }
    else
    {
        caller->print(F("[ERROR] Unknown tray command: "));
        caller->println(subcommand);
        caller->println(F("Valid options are 'load,request', 'unload,request', 'placed', 'removed', 'released', 'status', or 'help'"));
        return false;
    }
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
        caller->println(F("Available tests:"));
        caller->println(F("  home     - Test homing repeatability"));
        caller->println(F("  position - Test position cycling (for tray loading)"));
        caller->println(F("  tray     - Test complete tray handling operations"));
        caller->println(F("  help     - Display detailed test information"));
        caller->println(F("Usage: test,<test_name>"));
        return true;
    }

    // Parse the first argument - we'll use spaces as separators (commas converted to spaces)
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: test,<home|position|tray|help>"));
        return false;
    }

    // Trim leading spaces from test type
    subcommand = trimLeadingSpaces(subcommand);

    // Check motor initialization first
    if (motorState == MOTOR_STATE_NOT_READY)
    {
        caller->println(F("[ERROR] Motor not initialized. Run 'motor,init' first."));
        return false;
    }

    // Check E-Stop condition
    if (isEStopActive())
    {
        caller->println(F("[ERROR] Cannot run tests while E-Stop is active."));
        return false;
    }

    // Run the appropriate test
    if (strcmp(subcommand, "home") == 0)
    {
        caller->println(F("[INFO] Starting homing repeatability test..."));
        if (testHomingRepeatability())
        {
            caller->println(F("[INFO] Homing repeatability test completed successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Homing repeatability test failed or was aborted."));
            return false;
        }
    }
    else if (strcmp(subcommand, "position") == 0)
    {
        caller->println(F("[INFO] Starting position cycling test..."));

        if (testPositionCycling())
        {
            caller->println(F("[INFO] Position cycling test completed successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Position cycling test failed or was aborted."));
            return false;
        }
    }
    else if (strcmp(subcommand, "tray") == 0)
    {
        caller->println(F("[INFO] Starting tray handling test..."));

        if (testTrayHandling())
        {
            caller->println(F("[INFO] Tray handling test completed successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Tray handling test failed or was aborted."));
            return false;
        }
    }
    else if (strcmp(subcommand, "help") == 0)
    {
        caller->println(F("\n===== TEST SYSTEM HELP ====="));

        caller->println(F("\nOVERVIEW:"));
        caller->println(F("  The test system provides automated sequences for validating"));
        caller->println(F("  system functionality and repeatability. Tests are designed"));
        caller->println(F("  to verify proper operation of critical system components."));

        caller->println(F("\nAVAILABLE TESTS:"));
        caller->println(F("  test,home - Homing repeatability test"));
        caller->println(F("    > Performs multiple homing operations to test precision"));
        caller->println(F("    > Moves between home position and test position"));
        caller->println(F("    > Useful for verifying encoder and limit switch reliability"));
        caller->println(F("    > Test runs for approximately 20 cycles"));

        caller->println(F("  test,position - Position cycling test"));
        caller->println(F("    > Cycles through positions used in tray loading"));
        caller->println(F("    > Tests movements between positions 1, 2, and 3"));
        caller->println(F("    > Verifies motor accuracy and repeatability"));
        caller->println(F("    > Test runs for approximately 10 cycles"));

        caller->println(F("  test,tray - Comprehensive tray handling test"));
        caller->println(F("    > Tests complete tray movement operations"));
        caller->println(F("    > Includes valve operations for locking/unlocking"));
        caller->println(F("    > Verifies sensors, positioning, and control sequences"));
        caller->println(F("    > Most thorough test of the entire system"));

        caller->println(F("\nRUNNING TESTS:"));
        caller->println(F("  • Motor must be initialized (motor,init) before testing"));
        caller->println(F("  • Home position must be established for position tests"));
        caller->println(F("  • E-Stop must be inactive"));
        caller->println(F("  • Tests can be aborted by typing any character"));
        caller->println(F("  • Status messages display progress throughout the test"));

        caller->println(F("\nTRAY TEST REQUIREMENTS:"));
        caller->println(F("  • A tray must be present at position 1 to start"));
        caller->println(F("  • Positions 2 and 3 must be clear initially"));
        caller->println(F("  • Air system must be functioning properly"));
        caller->println(F("  • All valves and sensors must be operational"));

        caller->println(F("\nTROUBLESHOOTING:"));
        caller->println(F("  • If a test fails, check the specific error message"));
        caller->println(F("  • For position errors: verify motor operation with 'move' commands"));
        caller->println(F("  • For valve errors: check air pressure and connections"));
        caller->println(F("  • For sensor errors: verify sensor readings with 'system,state'"));

        return true;
    }
    else
    {
        caller->print(F("[ERROR] Unknown test type: "));
        caller->println(subcommand);
        caller->println(F("Available tests: 'home', 'position', 'tray', or 'help'"));
        return false;
    }
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
        caller->println(F("[INFO] MPG Handwheel Controls:"));
        caller->println(F("  encoder,enable          - Enable MPG handwheel control"));
        caller->println(F("  encoder,disable         - Disable MPG handwheel control"));
        caller->println(F("  encoder,multiplier,[1|10|100] - Set movement multiplier"));
        caller->println(F("  encoder,help            - Display detailed usage instructions"));

        // Show current status
        if (encoderControlActive)
        {
            caller->println(F("\n[STATUS] MPG control is currently ENABLED"));
            caller->print(F("[STATUS] Current multiplier: x"));
            caller->println(currentMultiplier);

            // Show position information if motor is homed
            if (isHomed)
            {
                double positionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
                caller->print(F("[STATUS] Current position: "));
                caller->print(positionMm, 2);
                caller->println(F(" mm"));
            }
        }
        else
        {
            caller->println(F("\n[STATUS] MPG control is currently DISABLED"));

            // Show reasons why encoder control might not be available
            if (!motorInitialized)
            {
                caller->println(F("[NOTE] Motor needs to be initialized first (motor,init)"));
            }
            else if (!isHomed)
            {
                caller->println(F("[NOTE] Motor needs to be homed first (motor,home)"));
            }
        }

        caller->println(F("\n[MULTIPLIERS] Effect of one full handwheel rotation (100 pulses):"));
        caller->print(F("  x1: ~"));
        caller->print(100 * MULTIPLIER_X1 / PULSES_PER_MM, 2);
        caller->println(F(" mm (fine adjustment)"));
        caller->print(F("  x10: ~"));
        caller->print(100 * MULTIPLIER_X10 / PULSES_PER_MM, 2);
        caller->println(F(" mm (medium adjustment)"));
        caller->print(F("  x100: ~"));
        caller->print(100 * MULTIPLIER_X100 / PULSES_PER_MM, 2);
        caller->println(F(" mm (coarse adjustment)"));

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
        caller->println(F("[ERROR] Invalid format. Usage: encoder,<enable|disable|multiplier>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle commands
    if (strcmp(subcommand, "enable") == 0)
    {
        // Check preconditions
        if (!motorInitialized)
        {
            caller->println(F("[ERROR] Motor must be initialized before enabling MPG control"));
            caller->println(F("[INFO] Use 'motor,init' first"));
            return false;
        }

        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor must be homed before enabling MPG control"));
            caller->println(F("[INFO] Use 'motor,home' to establish a reference position"));
            return false;
        }

        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            caller->println(F("[ERROR] Cannot enable MPG control while motor is moving"));
            caller->println(F("[INFO] Wait for current movement to complete or use 'motor,abort'"));
            return false;
        }

        if (motorState == MOTOR_STATE_FAULTED)
        {
            caller->println(F("[ERROR] Cannot enable MPG control while motor is in fault state"));
            caller->println(F("[INFO] Use 'motor,clear' to clear fault first"));
            return false;
        }

        if (isEStopActive())
        {
            caller->println(F("[ERROR] Cannot enable MPG control while E-Stop is active"));
            return false;
        }

        // Enable encoder control
        encoderControlActive = true;

        // Reset encoder position
        EncoderIn.Position(0);
        lastEncoderPosition = 0;
        lastEncoderUpdateTime = millis();

        caller->print(F("[INFO] MPG handwheel control enabled - current position: "));
        caller->print(pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()), 2);
        caller->println(F(" mm"));
        caller->print(F("[INFO] Using multiplier x"));
        caller->print(getMultiplierName(currentMultiplier));
        caller->print(F(" ("));
        caller->print(currentMultiplier);
        caller->println(F(")"));
        caller->println(F("[INFO] Issue 'encoder,disable' when finished with manual control"));

        return true;
    }
    else if (strcmp(subcommand, "disable") == 0)
    {
        // Disable encoder control
        encoderControlActive = false;
        caller->println(F("[INFO] MPG handwheel control disabled"));
        return true;
    }
    else if (strcmp(subcommand, "multiplier") == 0)
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
                    caller->println(F("[INFO] Multiplier set to x1 (fine adjustment)"));
                    break;
                case 10:
                    setEncoderMultiplier(10);
                    caller->println(F("[INFO] Multiplier set to x10 (medium adjustment)"));
                    break;
                case 100:
                    setEncoderMultiplier(100);
                    caller->println(F("[INFO] Multiplier set to x100 (coarse adjustment)"));
                    break;
                default:
                    caller->println(F("[ERROR] Invalid multiplier. Use 1, 10, or 100."));
                    return false;
                }

                // Show current multiplier and effect
                caller->print(F("[INFO] Current multiplier value: "));
                caller->println(currentMultiplier);
                double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
                caller->print(F("[INFO] One full rotation moves ~"));
                caller->print(mmPerRotation, 2);
                caller->println(F(" mm"));
                return true;
            }
        }

        // If we get here, display the current multiplier
        caller->print(F("[INFO] Current multiplier: x"));
        caller->print(getMultiplierName(currentMultiplier));
        caller->print(F(" ("));
        caller->print(currentMultiplier);
        caller->println(F(")"));

        // Show what one full rotation will move
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        caller->print(F("[INFO] One full rotation moves ~"));
        caller->print(mmPerRotation, 2);
        caller->println(F(" mm"));

        return true;
    }
    else if (strcmp(subcommand, "help") == 0)
    {
        caller->println(F("\n===== MPG HANDWHEEL SYSTEM HELP ====="));

        caller->println(F("\nSETUP SEQUENCE:"));
        caller->println(F("  1. 'motor,init' - Initialize the motor system"));
        caller->println(F("  2. 'motor,home' - Home the motor to establish reference position"));
        caller->println(F("  3. 'encoder,enable' - Activate MPG handwheel control"));
        caller->println(F("  4. 'encoder,multiplier,X' - Set desired precision (X = 1, 10, or 100)"));

        caller->println(F("\nCOMMAND REFERENCE:"));
        caller->println(F("  encoder,enable - Activate handwheel control mode"));
        caller->println(F("    > Motor position will respond directly to handwheel rotation"));
        caller->println(F("    > One full rotation (100 pulses) moves distance based on multiplier"));
        caller->println(F("  encoder,disable - Deactivate handwheel control mode"));
        caller->println(F("    > Returns system to command-based position control"));
        caller->println(F("  encoder,multiplier,X - Set movement precision"));
        caller->println(F("    > X=1: Fine adjustment (~1.63mm per rotation)"));
        caller->println(F("    > X=10: Medium adjustment (~16.3mm per rotation)"));
        caller->println(F("    > X=100: Coarse adjustment (~163mm per rotation)"));

        caller->println(F("\nAUTOMATIC DISABLING CONDITIONS:"));
        caller->println(F("  • E-Stop activation - Safety override disables all motor control"));
        caller->println(F("  • Motor fault condition - Requires 'motor,clear' to reset"));
        caller->println(F("  • Power cycle or system reset"));
        caller->println(F("  • When 'move' or 'jog' commands are issued"));

        caller->println(F("\nMOVEMENT CONSTRAINTS:"));
        caller->println(F("  • Hard limit at 0mm (home position)"));
        caller->println(F("  • Hard limit at maximum travel position (~1050mm)"));
        caller->println(F("  • Movement stops automatically at travel limits"));
        caller->println(F("  • No movement allowed if motor is in fault state"));

        caller->println(F("\nUSAGE TIPS:"));
        caller->println(F("  • Start with x1 multiplier for precise positioning"));
        caller->println(F("  • Use x10 or x100 for longer movements"));
        caller->println(F("  • Monitor current position using 'motor,status' command"));
        caller->println(F("  • Use 'encoder,disable' when finished with manual control"));
        caller->println(F("  • Slow, steady handwheel rotation produces smoother movement"));

        caller->println(F("\nTROUBLESHOOTING:"));
        caller->println(F("  • If encoder doesn't respond: Check if motor is initialized and homed"));
        caller->println(F("  • Erratic movement: Try lower multiplier setting"));
        caller->println(F("  • No movement at limits: System is preventing over-travel"));
        caller->println(F("  • After E-Stop: Must re-enable encoder control manually"));

        return true;
    }

    return false;
}

Commander commander;

Commander::systemCommand_t API_tree[] = {
    systemCommand("help", "Display help information for all commands", cmd_print_help),
    systemCommand("h", "Display help information for all commands", cmd_print_help),
    systemCommand("H", "Display help information for all commands", cmd_print_help),

    // Unified lock/unlock commands
    systemCommand("lock", "Lock a tray or shuttle:\n"
                          "  lock,1..3    - Lock specific tray position\n"
                          "  lock,shuttle - Lock the shuttle\n"
                          "  lock,help    - Display detailed lock instructions",
                  cmd_lock),

    systemCommand("unlock", "Unlock a tray, shuttle, or all valves:\n"
                            "  unlock,1..3    - Unlock specific tray position\n"
                            "  unlock,shuttle - Unlock the shuttle\n"
                            "  unlock,all     - Unlock all valves\n"
                            "  unlock,help    - Display detailed unlock instructions",
                  cmd_unlock),

    // Logging command
    systemCommand("log", "Logging controls:\n"
                         "  log,on[,interval] - Enable periodic logging (interval in ms)\n"
                         "  log,off           - Disable periodic logging\n"
                         "  log,now           - Log system state immediately\n"
                         "  log,help          - Display detailed logging information",
                  cmd_log),

    // State command to display system state
    systemCommand("system", "System commands:\n"
                            "  system,state  - Display current system state (sensors, actuators, positions)\n"
                            "  system,safety - Display comprehensive safety validation status\n"
                            "  system,trays  - Display tray tracking and statistics\n"
                            "  system,reset  - Reset system state after failure to retry operation",
                  cmd_system_state),

    // Motor control commands
    systemCommand("motor", "Motor control:\n"
                           "  motor,init   - Initialize motor system and prepare for operation\n"
                           "  motor,status - Display detailed motor status and configuration\n"
                           "  motor,clear  - Clear motor fault condition to restore operation\n"
                           "  motor,home   - Home the motor (find zero position)\n"
                           "  motor,abort  - Abort current operation gracefully\n"
                           "  motor,stop   - Emergency stop motor movement immediately\n"
                           "  motor,help   - Display comprehensive motor control instructions",
                  cmd_motor),

    // Move command
    systemCommand("move", "Move motor to position:\n"
                          "  move,home      - Move to home (zero) position\n"
                          "  move,1..4      - Move to predefined positions 1 through 4\n"
                          "  move,counts,X  - Move to absolute position X in encoder counts (0-64333)\n"
                          "  move,mm,X      - Move to absolute position X in millimeters (0-1050.0)\n"
                          "  move,rel,X     - Move X millimeters relative to current position (+ forward, - backward)\n"
                          "  move,help      - Display detailed command usage and troubleshooting",
                  cmd_move),

    // Jog command
    systemCommand("jog", "Jog motor:\n"
                         "  jog,+         - Jog forward by current increment\n"
                         "  jog,-         - Jog backward by current increment\n"
                         "  jog,inc,X     - Get or set jog increment (X in mm or 'default')\n"
                         "  jog,speed,X   - Get or set jog speed (X in RPM or 'default')\n"
                         "  jog,status    - Display current jog settings\n"
                         "  jog,help      - Display usage instructions and comparison with handwheel",
                  cmd_jog),

    // Tray command
    systemCommand("tray", "Tray operations:\n"
                          "  tray,load,request   - Request to load a tray (Mitsubishi)\n"
                          "  tray,unload,request - Request to unload a tray (Mitsubishi)\n"
                          "  tray,placed    - Notify tray has been placed (Mitsubishi)\n"
                          "  tray,removed   - Notify tray has been removed (Mitsubishi)\n"
                          "  tray,released  - Notify tray has been released (Mitsubishi)\n"
                          "  tray,status    - Get tray system status (machine-readable)\n"
                          "  tray,help      - Display detailed usage instructions",
                  cmd_tray),

    // Test command
    systemCommand("test", "Run tests on the system:\n"
                          "  test,home     - Run homing repeatability test\n"
                          "  test,position - Run position cycling test for tray loading\n"
                          "  test,tray     - Run tray handling test (request, place, release)\n"
                          "  test,help     - Display detailed test information and requirements",
                  cmd_test),

    // Encoder control commands
    systemCommand("encoder", "Encoder handwheel control:\n"
                             "  encoder,enable  - Enable encoder control\n"
                             "  encoder,disable - Disable encoder control\n"
                             "  encoder,multiplier,X - Set encoder multiplier (X = 1, 10, or 100)\n"
                             "  encoder,help    - Display setup instructions and usage tips",
                  cmd_encoder),
};

const size_t API_tree_size = sizeof(API_tree) / sizeof(Commander::systemCommand_t);