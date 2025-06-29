#include "Commands.h"

// External declaration for the logging structure
extern LoggingManagement logging;
extern const unsigned long DEFAULT_LOG_INTERVAL;

// Binary search function for subcommand lookup
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
        Console.println(F("Lynx Conveyor System Command Help:"));
        Console.println(F("--------------------------------------------------"));

        commander.printHelp(caller, true, true);

        Console.println(F("--------------------------------------------------"));
        return true;
    }
}

// Define the lock subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo LOCK_COMMANDS[] = {
    {"1", 4},
    {"2", 4},
    {"3", 4},
    {"all", 1},
    {"help", 3},
    {"shuttle", 2}};

static const size_t LOCK_COMMAND_COUNT = sizeof(LOCK_COMMANDS) / sizeof(SubcommandInfo);

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

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, LOCK_COMMANDS, LOCK_COMMAND_COUNT);

    // Handle tray numbers special case
    if (cmdCode == 4)
    {
        trayNum = atoi(subcommand);
    }

    char msg[100];

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "all"
        Console.error(F("'lock,all' is not supported for safety reasons. Engage trays individually."));
        return false;

    case 2: // "shuttle"
        if (ccioBoardCount <= 0)
        {
            Console.error(F("No CCIO-8 board detected. Shuttle valve not available."));
            return false;
        }

        Console.serialInfo(F("Engaging shuttle with sensor verification..."));
        valve = getShuttleValve();
        sensor = getShuttleSensor();

        if (!valve || !sensor)
        {
            Console.error(F("Failed to access shuttle valve or sensor. Possible causes:"));
            Console.serialInfo(F("  - CCIO board detected but shuttle valve not configured"));
            Console.serialInfo(F("  - System memory corruption"));
            Console.serialInfo(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (valve->position == VALVE_POSITION_LOCK)
        {
            Console.serialInfo(F("Shuttle already engaged"));

            // Verify actual position with sensor
            if (sensorRead(*sensor) == true)
            { // Sensor true = locked
                Console.acknowledge(F("SHUTTLE_ALREADY_LOCKED"));
            }
            else
            {
                Console.error(F("Shuttle should be locked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try locking with sensor feedback
        Console.serialInfo(F("Locking shuttle..."));
        if (safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
        {
            Console.acknowledge(F("SHUTTLE_LOCKED"));
            return true;
        }

        Console.error(F("Failed to engage shuttle - sensor did not confirm lock"));
        Console.serialInfo(F("[WARNING] Check air pressure and valve functionality"));
        return false;

    case 3: // "help"
        Console.acknowledge(F("LOCK_HELP"));
        Console.println(F(
            "\n===== LOCK COMMAND HELP =====\n"
            "\nOVERVIEW:\n"
            "  The lock command engages pneumatic locks on trays and the shuttle,\n"
            "  securing them in position. All operations include sensor verification\n"
            "  to confirm successful locking.\n"
            "\nCOMMAND REFERENCE:\n"
            "  lock,1 - Engage lock on tray at position 1 (loading position)\n"
            "    > Verified by cylinder position sensor\n"
            "    > Will report success only when sensor confirms lock\n"
            "\n"
            "  lock,2 - Engage lock on tray at position 2 (middle position)\n"
            "    > Verified by cylinder position sensor\n"
            "    > Will report success only when sensor confirms lock\n"
            "\n"
            "  lock,3 - Engage lock on tray at position 3 (unloading position)\n"
            "    > Verified by cylinder position sensor\n"
            "    > Will report success only when sensor confirms lock\n"
            "\n"
            "  lock,shuttle - Engage lock on the shuttle\n"
            "    > Prevents shuttle from moving between positions\n"
            "    > Verified by cylinder position sensor\n"
            "    > Required before unlocking any trays for safety\n"
            "\nSAFETY NOTES:\n"
            "  • 'lock,all' is not supported for safety reasons\n"
            "  • Always lock the shuttle before unlocking any trays\n"
            "  • System uses sensor verification to confirm actual locking\n"
            "  • Sufficient pneumatic pressure is required for all valve operations\n"
            "  • Failed locking may indicate mechanical issues or low air pressure\n"
            "\nSENSOR VERIFICATION:\n"
            "  • Each lock has a corresponding sensor that confirms its position\n"
            "  • Command waits up to 1 second for sensor to confirm lock\n"
            "  • Returns success only when sensor confirms the lock operation\n"
            "  • Sensor mismatches are shown in status logs with [!] indicator\n"
            "\nTROUBLESHOOTING:\n"
            "  • If lock fails, check air pressure\n"
            "  • Verify sensor connections if lock command doesn't register\n"
            "  • Use 'system,state' to see detailed valve and sensor status\n"
            "  • For persistent issues, check valve functionality\n"
            "-------------------------------------------"
        ));
        return true;

    case 4: // Tray numbers (1, 2, or 3)
        trayNum = atoi(subcommand);

        sprintf(msg, "Engaging tray %d with sensor verification...", trayNum);
        Console.serialInfo(msg);

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
            sprintf(msg, "Failed to access tray %d valve or sensor", trayNum);
            Console.error(msg);
            Console.serialInfo(F("Possible causes:"));
            Console.serialInfo(F("  - Hardware initialization issue"));
            Console.serialInfo(F("  - Valve controller not properly initialized"));
            Console.serialInfo(F("  - System memory corruption"));
            Console.serialInfo(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (trayValve->position == VALVE_POSITION_LOCK)
        {
            sprintf(msg, "Tray %d already engaged", trayNum);
            Console.serialInfo(msg);

            // Verify actual position with sensor
            if (sensorRead(*traySensor) == true)
            { // Sensor true = locked
                sprintf(msg, "TRAY_%d_ALREADY_LOCKED", trayNum);
                Console.acknowledge(msg);
            }
            else
            {
                Console.error(F("Tray should be locked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try locking with sensor feedback
        if (safeValveOperation(*trayValve, *traySensor, VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
        {
            sprintf(msg, "TRAY_%d_LOCKED", trayNum);
            Console.acknowledge(msg);
            return true;
        }

        sprintf(msg, "Failed to engage tray %d", trayNum);
        Console.error(msg);
        Console.serialInfo(F("Check air pressure and valve functionality"));
        return false;

    default: // Unknown command
        sprintf(msg, "Unknown lock subcommand: %s", subcommand);
        Console.error(msg);
        Console.serialInfo(F("Valid options are '1', '2', '3', 'shuttle', or 'help'"));
        return false;
    }

    return false; // Should never reach here
}

// Define the unlock subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo UNLOCK_COMMANDS[] = {
    {"1", 4},
    {"2", 4},
    {"3", 4},
    {"all", 1},
    {"help", 3},
    {"shuttle", 2}};

static const size_t UNLOCK_COMMAND_COUNT = sizeof(UNLOCK_COMMANDS) / sizeof(SubcommandInfo);

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

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, UNLOCK_COMMANDS, UNLOCK_COMMAND_COUNT);

    // Handle tray numbers special case
    if (cmdCode == 4)
    {
        trayNum = atoi(subcommand);
    }

    char msg[100];

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "all"
        Console.serialInfo(F("Disengaging all valves with sensor verification..."));
        if (safeUnlockAllValves(1000))
        {
            Console.acknowledge(F("ALL_VALVES_UNLOCKED"));
            return true;
        }
        else
        {
            Console.error(F("Some valves could not be disengaged - check air pressure"));
            return false;
        }

    case 2: // "shuttle"
        if (ccioBoardCount <= 0)
        {
            Console.error(F("No CCIO-8 board detected. Shuttle valve not available."));
            return false;
        }

        Console.serialInfo(F("Disengaging shuttle with sensor verification..."));
        valve = getShuttleValve();
        sensor = getShuttleSensor();

        if (!valve || !sensor)
        {
            Console.error(F("Failed to access shuttle valve or sensor. Possible causes:"));
            Console.serialInfo(F("  - CCIO board detected but shuttle valve not configured"));
            Console.serialInfo(F("  - System memory corruption"));
            Console.serialInfo(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (valve->position == VALVE_POSITION_UNLOCK)
        {
            Console.serialInfo(F("Shuttle already disengaged"));

            // Verify actual position with sensor
            if (sensorRead(*sensor) == false)
            { // Sensor false = unlocked
                Console.acknowledge(F("SHUTTLE_ALREADY_UNLOCKED"));
            }
            else
            {
                Console.error(F("Shuttle should be unlocked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try unlocking with sensor feedback
        Console.serialInfo(F("Unlocking shuttle..."));
        if (safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
        {
            Console.acknowledge(F("SHUTTLE_UNLOCKED"));
            return true;
        }

        Console.error(F("Failed to disengage shuttle - sensor did not confirm unlock"));
        Console.serialInfo(F("Check air pressure and valve functionality"));
        return false;

    case 3: // "help"
        Console.acknowledge(F("UNLOCK_HELP"));
        Console.println(F(
            "\n===== UNLOCK COMMAND HELP =====\n"
            "\nOVERVIEW:\n"
            "  The unlock command disengages pneumatic locks on trays and the shuttle,\n"
            "  allowing them to be removed or permitting shuttle movement. All operations\n"
            "  include sensor verification to confirm successful unlocking.\n"
            "\nCOMMAND REFERENCE:\n"
            "  unlock,1 - Disengage lock on tray at position 1 (loading position)\n"
            "    > Verified by cylinder position sensor\n"
            "    > Will report success only when sensor confirms unlock\n"
            "\n"
            "  unlock,2 - Disengage lock on tray at position 2 (middle position)\n"
            "    > Verified by cylinder position sensor\n"
            "    > Will report success only when sensor confirms unlock\n"
            "\n"
            "  unlock,3 - Disengage lock on tray at position 3 (unloading position)\n"
            "    > Verified by cylinder position sensor\n"
            "    > Will report success only when sensor confirms unlock\n"
            "\n"
            "  unlock,shuttle - Disengage lock on the shuttle\n"
            "    > Allows shuttle to move between positions\n"
            "    > Verified by cylinder position sensor\n"
            "\n"
            "  unlock,all - Disengage all locks in the system\n"
            "    > Emergency recovery function\n"
            "    > Uses sensor verification for all valves\n"
            "    > Reports success only when all sensors confirm unlock\n"
            "\nSAFETY NOTES:\n"
            "  • Ensure trays are properly supported before unlocking\n"
            "  • System uses sensor verification to confirm actual unlocking\n"
            "  • Failed unlocking may indicate mechanical issues\n"
            "  • Sufficient pneumatic pressure is required for all valve operations\n"
            "\nSENSOR VERIFICATION:\n"
            "  • Each lock has a corresponding sensor that confirms its position\n"
            "  • Command waits up to 1 second for sensor to confirm unlock\n"
            "  • Returns success only when sensor confirms the unlock operation\n"
            "  • Sensor mismatches are shown in status logs with [!] indicator\n"
            "\nTROUBLESHOOTING:\n"
            "  • If unlock fails, check air pressure\n"
            "  • Verify sensor connections if unlock command doesn't register\n"
            "  • Use 'system,state' to see detailed valve and sensor status\n"
            "  • For persistent issues, check valve functionality\n"
            "-------------------------------------------"
        ));

        return true;

    case 4: // Tray numbers (1, 2, or 3)
        trayNum = atoi(subcommand);

        sprintf(msg, "Disengaging tray %d with sensor verification...", trayNum);
        Console.serialInfo(msg);

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
            sprintf(msg, "Failed to access tray %d valve or sensor.", trayNum);
            Console.error(msg);
            Console.serialInfo(F("Possible causes:"));
            Console.serialInfo(F("  - Hardware initialization issue"));
            Console.serialInfo(F("  - Valve controller not properly initialized"));
            Console.serialInfo(F("  - System memory corruption"));
            Console.serialInfo(F("Try restarting the system or run 'status' to check valve configuration"));
            return false;
        }

        // Check current state first
        if (trayValve->position == VALVE_POSITION_UNLOCK)
        {
            sprintf(msg, "Tray %d already disengaged", trayNum);
            Console.serialInfo(msg);

            // Verify actual position with sensor
            if (sensorRead(*traySensor) == false)
            { // Sensor false = unlocked
                sprintf(msg, "TRAY_%d_ALREADY_UNLOCKED", trayNum);
                Console.acknowledge(msg);
            }
            else
            {
                Console.error(F("Tray should be unlocked but sensor doesn't confirm - check air pressure"));
            }
            return true;
        }

        // Try unlocking with sensor feedback
        if (safeValveOperation(*trayValve, *traySensor, VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
        {
            sprintf(msg, "TRAY_%d_UNLOCKED", trayNum);
            Console.acknowledge(msg);
            return true;
        }

        sprintf(msg, "Failed to disengage tray %d", trayNum);
        Console.error(msg);
        Console.serialInfo(F("Check air pressure and valve functionality"));
        return false;

    default: // Unknown command
        sprintf(msg, "Unknown unlock subcommand: %s", subcommand);
        Console.error(msg);
        Console.serialInfo(F("Valid options are '1', '2', '3', 'shuttle', 'all', or 'help'"));
        return false;
    }

    return false; // Should never reach here
}

// Define the log subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo LOG_COMMANDS[] = {
    {"help", 4},
    {"now", 3},
    {"off", 2},
    {"on", 1}};

static const size_t LOG_COMMAND_COUNT = sizeof(LOG_COMMANDS) / sizeof(SubcommandInfo);

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

    // Parse the first argument - we'll use spaces as separators
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Invalid format. Usage: log,<on[,interval]|off|now|help>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, LOG_COMMANDS, LOG_COMMAND_COUNT);

    char msg[100];

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
                sprintf(msg, "Logging enabled with interval of %lu ms", interval);
                Console.serialInfo(msg);

                // Acknowledgment to both outputs
                sprintf(msg, "LOG_ENABLED_%lu", interval);
                Console.acknowledge(msg);
            }
            else
            {
                Console.serialInfo(F("Invalid interval. Using default."));
                interval = DEFAULT_LOG_INTERVAL;

                // Acknowledgment to both outputs
                sprintf(msg, "LOG_ENABLED_DEFAULT_%lu", DEFAULT_LOG_INTERVAL);
                Console.acknowledge(msg);
            }
        }
        else
        {
            // Use default interval
            sprintf(msg, "Logging enabled with default interval of %lu ms", DEFAULT_LOG_INTERVAL);
            Console.serialInfo(msg);

            // Acknowledgment to both outputs
            sprintf(msg, "LOG_ENABLED_DEFAULT_%lu", DEFAULT_LOG_INTERVAL);
            Console.acknowledge(msg);
        }

        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset the timer
        return true;
    }

    case 2: // "off"
    {
        // Acknowledgment to both outputs
        Console.acknowledge(F("LOG_DISABLED"));

        // Additional info to Serial only
        Console.serialInfo(F("Logging disabled"));

        logging.logInterval = 0; // Setting to 0 disables logging
        return true;
    }

    case 3: // "now"
    {
        // Acknowledgment to both outputs
        Console.acknowledge(F("LOG_NOW"));

        Console.serialInfo(F("Logging system state now"));
        // Log immediately regardless of interval
        logSystemState();
        return true;
    }

    case 4: // "help"
    {
        Console.acknowledge(F("LOG_HELP"));
        Console.println(F(
            "\n===== LOGGING SYSTEM HELP =====\n"
            "\nOVERVIEW:\n"
            "  The logging system captures complete system state at regular intervals\n"
            "  or on demand, providing detailed information for debugging and monitoring.\n"
            "\nCOMMAND REFERENCE:\n"
            "  log,on[,interval] - Enable periodic logging\n"
            "    > interval = Optional logging frequency in milliseconds\n"
            "    > Default interval: 250 ms (4 logs per second)\n"
            "    > Example: log,on,1000 - Log every 1 second\n"
            "    > Example: log,on - Log every 250ms (default)\n"
            "\n"
            "  log,off - Disable periodic logging\n"
            "    > Stops the automatic logging of system state\n"
            "    > Does not affect manual logging with log,now\n"
            "\n"
            "  log,now - Log system state immediately\n"
            "    > Records a single log entry regardless of periodic settings\n"
            "    > Useful for capturing state at specific moments\n"
            "\nLOG CONTENT:\n"
            "  • Valves - Lock status of all trays and shuttle with sensor verification\n"
            "    > [!] indicator shows sensor/command mismatch\n"
            "  • Pneumatics - Air pressure status (sufficient/insufficient)\n"
            "    > Critical for valve actuation and safe operations\n"
            "  • Sensors - Tray presence detection at each position\n"
            "  • System - Motor state, homing status, E-Stop and HLFB status\n"
            "  • Position - Current, target, and last positions (mm and counts)\n"
            "  • Velocity - Current speed, percentage of max, and speed limits\n"
            "  • Jog - Current jog increment and speed settings\n"
            "  • MPG - Handwheel control status, multiplier, and mm/rotation\n"
            "\nOUTPUT DESTINATION:\n"
            "  • Logs are only sent to the serial port\n"
            "  • Ethernet clients do not receive log output\n"
            "  • Connect via USB serial to view logs\n"
            "\nPERFORMANCE CONSIDERATIONS:\n"
            "  • Default 250ms interval is optimal for most debugging\n"
            "  • Very frequent logging (< 100ms) may impact system responsiveness\n"
            "  • For long-term monitoring, consider 1000-5000ms intervals\n"
            "\nREADING LOG OUTPUT:\n"
            "  • Each section is separated by | characters for readability\n"
            "  • Position values shown in both mm and encoder counts\n"
            "  • Lock status shows ? if sensor doesn't match expected state\n"
            "  • Velocity shown with percentage of maximum when moving\n"
            "\nTROUBLESHOOTING TIPS:\n"
            "  • Use log,now before and after commands to track state changes\n"
            "  • Watch for sensor/valve mismatches [!] indicating hardware issues\n"
            "  • Compare HLFB status with motor state to identify drive problems\n"
            "  • Verify position values match expected targets during movements\n"
            "-------------------------------------------"
        ));

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

// Define the motor subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo MOTOR_COMMANDS[] = {
    {"abort", 5},
    {"clear", 3},
    {"help", 7},
    {"home", 4},
    {"init", 1},
    {"status", 2},
    {"stop", 6}};

static const size_t MOTOR_COMMAND_COUNT = sizeof(MOTOR_COMMANDS) / sizeof(SubcommandInfo);

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

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle motor subcommands using switch for better readability
    int subcommandCode = findSubcommandCode(subcommand, MOTOR_COMMANDS, MOTOR_COMMAND_COUNT);

    char msg[100]; // Shared buffer for all formatted output

    switch (subcommandCode)
    {
    case 1: // init
    {
        Console.serialInfo(F("Initializing motor..."));

        // Diagnostic: Print state before initialization
        sprintf(msg, "[DIAGNOSTIC] Motor state before init: %s", 
            motorState == MOTOR_STATE_IDLE ? "IDLE" :
            motorState == MOTOR_STATE_MOVING ? "MOVING" :
            motorState == MOTOR_STATE_HOMING ? "HOMING" :
            motorState == MOTOR_STATE_FAULTED ? "FAULTED" :
            motorState == MOTOR_STATE_NOT_READY ? "NOT READY" : "UNKNOWN");
        Console.serialInfo(msg);

        initMotorSystem();

        // Diagnostic: Print state after initialization
        sprintf(msg, "[DIAGNOSTIC] Motor state after init: %s", 
            motorState == MOTOR_STATE_IDLE ? "IDLE" :
            motorState == MOTOR_STATE_MOVING ? "MOVING" :
            motorState == MOTOR_STATE_HOMING ? "HOMING" :
            motorState == MOTOR_STATE_FAULTED ? "FAULTED" :
            motorState == MOTOR_STATE_NOT_READY ? "NOT READY" : "UNKNOWN");
        Console.serialInfo(msg);

        if (motorState == MOTOR_STATE_NOT_READY || motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor initialization failed."));
            Console.serialInfo(F("Check connections and power."));
            return false;
        }
        else
        {
            Console.acknowledge(F("MOTOR_INITIALIZED"));
            return true;
        }
    }

    case 2: // status
    {
        Console.acknowledge(F("MOTOR_STATUS"));
        Console.println(F("Motor Status:"));

        // Display motor state
        sprintf(msg, "  State: %s",
            motorState == MOTOR_STATE_IDLE ? "IDLE" :
            motorState == MOTOR_STATE_MOVING ? "MOVING" :
            motorState == MOTOR_STATE_HOMING ? "HOMING" :
            motorState == MOTOR_STATE_FAULTED ? "FAULTED" :
            motorState == MOTOR_STATE_NOT_READY ? "NOT READY" : "UNKNOWN");
        Console.println(msg);

        // Display homing status
        Console.print(F("  Homed: "));
        Console.println(isHomed ? F("YES") : F("NO"));

        // Display position information based on homing status
        if (isHomed)
        {
            double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
            int32_t rawPosition = MOTOR_CONNECTOR.PositionRefCommanded();
            int32_t normalizedPosition = normalizeEncoderValue(rawPosition);

            sprintf(msg, "  Current Position: %.2f mm (%ld counts)", calculatedPositionMm, (long)normalizedPosition);
            Console.println(msg);

            Console.print(F("  Last Completed Position: "));
            if (hasLastTarget)
            {
                sprintf(msg, "%.2f mm (%ld counts)", lastTargetPositionMm, (long)normalizeEncoderValue(lastTargetPulses));
                Console.println(msg);
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
                sprintf(msg, "  Encoder Reading: %ld counts (reference point not established)", (long)normalizeEncoderValue(rawPosition));
                Console.println(msg);
            }
            else
            {
                Console.println(F("  Encoder Reading: Not available - Motor not initialized"));
            }
        }

        // Show velocity configuration instead of current velocity
        Console.println(F("  Velocity Settings:"));

        // Regular movement velocity
        sprintf(msg, "    Move Operations: %.1f RPM", ppsToRpm(currentVelMax));
        Console.println(msg);

        // Homing velocity
        sprintf(msg, "    Homing: %d RPM", HOME_APPROACH_VELOCITY_RPM);
        Console.println(msg);

        // Jog velocity and increment
        sprintf(msg, "    Jogging: %d RPM, %.2f mm/jog", currentJogSpeedRpm, currentJogIncrementMm);
        Console.println(msg);

        // Only show current velocity if motor is moving
        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            int32_t velocity = MOTOR_CONNECTOR.VelocityRefCommanded();
            double velocityRpm = (double)velocity * 60.0 / PULSES_PER_REV;
            sprintf(msg, "    Current: %.1f RPM (%ld pulses/sec)", velocityRpm, (long)velocity);
            Console.println(msg);
        }

        // Display acceleration limit
        sprintf(msg, "  Acceleration: %.1f RPM/sec", (double)currentAccelMax * 60.0 / PULSES_PER_REV);
        Console.println(msg);

        // Display travel limits based on homing status
        Console.println(F("  Travel Limits:"));
        if (isHomed)
        {
            sprintf(msg, "    Range: 0.00 to %.2f mm", MAX_TRAVEL_MM);
            Console.println(msg);
            sprintf(msg, "            0 to %ld counts", (long)MAX_TRAVEL_PULSES);
            Console.println(msg);
        }
        else
        {
            Console.println(F("    UNKNOWN - Motor not homed"));
        }

        // Display fault status
        Console.print(F("  Fault Status: "));
        Console.println(MOTOR_CONNECTOR.HlfbState() == ClearCore::MotorDriver::HLFB_ASSERTED ? F("NO FAULT") : F("FAULT DETECTED"));

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
    }

    case 3:
    { // clear
        Console.serialInfo(F("Attempting to clear motor fault..."));

        if (clearMotorFaultWithStatus())
        {
            Console.acknowledge(F("MOTOR_FAULT_CLEARED"));
            return true;
        }
        else
        {
            Console.error(F("Failed to clear motor fault"));
            Console.serialInfo(F("Motor may still be in fault state."));
            Console.serialInfo(F("Try power cycling the system if fault persists."));
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
            Console.error(F("Homing sequence is already in progress."));
            return false;
        }

        if (isEStopActive())
        {
            Console.error(F("Cannot home while E-Stop is active"));
            Console.serialInfo(F("Release E-Stop and try again."));
            return false;
        }

        if (motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor is in fault state. Use 'motor,clear' first."));
            return false;
        }

        Console.serialInfo(F("Starting homing sequence..."));

        // Begin homing
        initiateHomingSequence();

        // Check if homing was initiated by examining the motor state
        if (motorState == MOTOR_STATE_HOMING)
        {
            Console.acknowledge(F("HOMING_STARTED"));
            Console.serialInfo(F("Homing sequence initiated. Motor will move to find home position."));
            return true;
        }
        else
        {
            Console.error(F("Failed to start homing sequence"));
            Console.serialInfo(F("Check motor status."));
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

        Console.serialInfo(F("Aborting current operation..."));

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

            Console.acknowledge(F("OPERATION_ABORTED"));
            return true;
        }
        else
        {
            Console.error(F("No active operation to abort"));
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

        Console.serialInfo(F("EMERGENCY STOP initiated!"));

        // Execute emergency stop
        MOTOR_CONNECTOR.MoveStopAbrupt();
        motorState = MOTOR_STATE_IDLE;

        Console.acknowledge(F("EMERGENCY_STOP_EXECUTED"));
        Console.serialInfo(F("Motor movement halted. Position may no longer be accurate."));
        Console.serialInfo(F("Re-homing recommended after emergency stop."));

        return true;
        break;
    }

    case 7: // help
    {
        Console.acknowledge(F("MOTOR_HELP"));
        Console.println(F(
            "\n===== MOTOR CONTROL SYSTEM HELP =====\n"
            "\nCOMMAND REFERENCE:\n"
            "  motor,init - Initialize motor system and prepare for operation\n"
            "    > Must be run after power-up before any other motor commands\n"
            "    > Configures motor parameters and communication\n"
            "    > Does not move the motor or establish position reference\n"
            "\n"
            "  motor,home - Find home position and establish reference point\n"
            "    > Required before absolute positioning commands can be used\n"
            "    > Motor will move slowly until it contacts the home limit switch\n"
            "    > After contact, motor backs off to establish precise zero position\n"
            "    > Home position is offset 5mm from physical limit for safety\n"
            "\n"
            "  motor,status - Display detailed motor status and configuration\n"
            "    > Shows current state, position, velocity settings, and limits\n"
            "    > Use to verify proper operation or troubleshoot issues\n"
            "\n"
            "  motor,clear - Clear motor fault condition\n"
            "    > Use after resolving the condition that caused the fault\n"
            "    > Common faults: excessive load, hitting physical limit, E-Stop\n"
            "\n"
            "  motor,abort - Gracefully stop current movement\n"
            "    > Controlled deceleration to stop the motor\n"
            "    > Position information is maintained\n"
            "    > Use to cancel a movement without generating a fault\n"
            "\n"
            "  motor,stop - Emergency stop motor movement immediately\n"
            "    > Immediate halt of motor operation\n"
            "    > May cause position inaccuracy\n"
            "    > Use only when necessary to prevent damage or injury\n"
            "\n"
            "TYPICAL SEQUENCE:\n"
            "  1. motor,init   - Initialize the motor system\n"
            "  2. motor,home   - Establish reference position\n"
            "  3. move,X       - Move to desired positions\n"
            "  4. jog commands - Make fine adjustments\n"
            "  5. encoder      - Use handwheel for manual control\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If motor won't move: Check E-Stop, then run motor,status\n"
            "  • After fault: Use motor,clear to reset fault condition\n"
            "  • If position seems incorrect: Re-home the system\n"
            "  • Unexpected behavior: Check that motor is initialized\n"
            "  • Jerky movement: Try using slower speed or smaller increments\n"
            "\n"
            "SAFETY NOTES:\n"
            "  • Always ensure proper clearance before moving the shuttle\n"
            "  • Use E-Stop if unexpected movement occurs\n"
            "  • After E-Stop, clear faults before resuming operation\n"
            "  • Motor movements will halt automatically at travel limits\n"
            "-------------------------------------------"
        ));

        return true;
        break;
    }

    default:
    {
        sprintf(msg, "[ERROR], Unknown motor command: %s", subcommand);
        Console.error(msg);
        Console.serialInfo(F("Valid options are 'init', 'status', 'clear', 'home', 'abort', 'stop', or 'help'"));
        return false;
    }
    }

    return false; // Should never reach here, but included for completeness
}

// Define the move subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo MOVE_COMMANDS[] = {
    {"1", 2},
    {"2", 3},
    {"3", 4},
    {"4", 5},
    {"counts", 7},
    {"help", 9},
    {"home", 1},
    {"mm", 6},
    {"rel", 8}};

static const size_t MOVE_COMMAND_COUNT = sizeof(MOVE_COMMANDS) / sizeof(SubcommandInfo);

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
    if (motorState == MOTOR_STATE_NOT_READY)
    {
        Console.error(F("Motor is not initialized. Use 'motor,init' first."));
        return false;
    }
    if (isEStopActive())
    {
        Console.error(F("Cannot move while E-Stop is active"));
        Console.serialInfo(F("Release E-Stop and try again."));
        return false;
    }
    if (motorState == MOTOR_STATE_FAULTED)
    {
        Console.error(F("Motor is in fault state. Use 'motor,clear' first."));
        return false;
    }
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
    {
        Console.error(F("Motor is already moving. Use 'motor,abort' first."));
        return false;
    }

    int cmdCode = findSubcommandCode(subcommand, MOVE_COMMANDS, MOVE_COMMAND_COUNT);
    char msg[100];

    switch (cmdCode)
    {
    case 1: // "home"
        if (isHomed)
        {
            Console.serialInfo(F("Moving to home position..."));
            if (moveToPositionMm(0.0))
            {
                Console.acknowledge(F("MOVE_TO_HOME_STARTED"));
                return true;
            }
            else
            {
                Console.error(F("Failed to start movement to home position"));
                return false;
            }
        }
        else
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            return false;
        }

    case 2: // "1"
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            return false;
        }
        Console.serialInfo(F("Moving to position 1..."));
        if (moveToPosition(POSITION_1))
        {
            Console.acknowledge(F("MOVE_TO_POS1_STARTED"));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 1"));
            return false;
        }

    case 3: // "2"
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            return false;
        }
        Console.serialInfo(F("Moving to position 2..."));
        if (moveToPosition(POSITION_2))
        {
            Console.acknowledge(F("MOVE_TO_POS2_STARTED"));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 2"));
            return false;
        }

    case 4: // "3"
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            return false;
        }
        Console.serialInfo(F("Moving to position 3..."));
        if (moveToPosition(POSITION_3))
        {
            Console.acknowledge(F("MOVE_TO_POS3_STARTED"));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 3"));
            return false;
        }

    case 5: // "4"
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            return false;
        }
        Console.serialInfo(F("Moving to position 4..."));
        if (moveToPosition(POSITION_4))
        {
            Console.acknowledge(F("MOVE_TO_POS4_STARTED"));
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to position 4"));
            return false;
        }

    case 6: // "mm"
    {
        char *mmStr = strtok(NULL, " ");
        if (mmStr == NULL)
        {
            Console.error(F("Missing mm value. Usage: move,mm,X"));
            return false;
        }
        mmStr = trimLeadingSpaces(mmStr);
        double targetMm = atof(mmStr);

        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            Console.serialWarning(F("Moving to absolute positions without homing is unsafe."));
            return false;
        }
        if (targetMm < 0.0 || targetMm > MAX_TRAVEL_MM)
        {
            sprintf(msg, "[ERROR] Position out of range. Valid range: 0 to %.1f mm", MAX_TRAVEL_MM);
            Console.error(msg);
            return false;
        }

        sprintf(msg, "Moving to absolute position: %.2f mm", targetMm);
        Console.serialInfo(msg);

        if (moveToPositionMm(targetMm))
        {
            sprintf(msg, "[ACK], MOVE_TO_MM_%.2f", targetMm);
            Console.acknowledge(msg);
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to requested position."));
            return false;
        }
    }

    case 7: // "counts"
    {
        char *countsStr = strtok(NULL, " ");
        if (countsStr == NULL)
        {
            Console.error(F("Missing counts value. Usage: move,counts,X"));
            return false;
        }
        countsStr = trimLeadingSpaces(countsStr);
        int32_t targetCounts = atol(countsStr);

        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            Console.serialWarning(F("Moving to absolute positions without homing is unsafe."));
            return false;
        }
        if (targetCounts < 0 || targetCounts > MAX_TRAVEL_PULSES)
        {
            sprintf(msg, "[ERROR] Position out of range. Valid range: 0 to %ld counts", (long)MAX_TRAVEL_PULSES);
            Console.error(msg);
            return false;
        }

        sprintf(msg, "Moving to absolute position: %ld counts", (long)targetCounts);
        Console.serialInfo(msg);

        if (moveToAbsolutePosition(targetCounts))
        {
            sprintf(msg, "[ACK], MOVE_TO_COUNTS_%ld", (long)targetCounts);
            Console.acknowledge(msg);
            return true;
        }
        else
        {
            Console.error(F("Failed to start movement to requested position."));
            return false;
        }
    }

    case 8: // "rel"
    {
        char *relStr = strtok(NULL, " ");
        if (relStr == NULL)
        {
            Console.error(F("Missing relative distance value. Usage: move,rel,X"));
            return false;
        }
        relStr = trimLeadingSpaces(relStr);
        double relDistanceMm = atof(relStr);

        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' first."));
            Console.serialWarning(F("Moving without homing is unsafe."));
            return false;
        }

        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm + relDistanceMm;

        if (targetPositionMm < 0.0 || targetPositionMm > MAX_TRAVEL_MM)
        {
            sprintf(msg, "Target position out of range. Valid range: 0 to %.1f mm", MAX_TRAVEL_MM);
            Console.error(msg);
            sprintf(msg, "Current position: %.2f mm, Requested move: %.2f mm", currentPositionMm, relDistanceMm);
            Console.serialInfo(msg);
            return false;
        }

        sprintf(msg, "Moving %.2f mm from current position (%.2f mm) to %.2f mm",
                relDistanceMm, currentPositionMm, targetPositionMm);
        Console.serialInfo(msg);

        if (moveToPositionMm(targetPositionMm))
        {
            sprintf(msg, "[ACK], MOVE_REL_%.2f_TO_%.2f", relDistanceMm, targetPositionMm);
            Console.acknowledge(msg);
            return true;
        }
        else
        {
            Console.error(F("Failed to start relative movement."));
            return false;
        }
    }

    case 9: // "help"
        Console.acknowledge(F("MOVE_HELP"));
        Console.println(F(
            "\n===== MOVE COMMAND HELP =====\n"
            "\nPREREQUISITES:\n"
            "  • Motor must be initialized (motor,init)\n"
            "  • Motor must be homed for accurate positioning (motor,home)\n"
            "  • E-Stop must be inactive\n"
            "  • Motor must not be in fault state\n"
            "  • No other movement can be in progress\n"
            "\n"
            "COMMAND TYPES:\n"
            "  move,home - Move to home (zero) position\n"
            "    > Reference position offset 5mm from hardstop\n"
            "    > Always available after homing\n"
            "  move,1 through move,4 - Move to predefined positions\n"
            "    > Position 1: Loading position (28.7mm)\n"
            "    > Position 2: Middle position (456.0mm)\n"
            "    > Position 3: Unloading position (883.58mm)\n"
            "    > Position 4: Max travel (1050.0mm)\n"
            "  move,mm,X - Move to absolute position X in millimeters\n"
            "    > Valid range: 0 to 1050.0 mm\n"
            "    > Most intuitive way to specify exact positions\n"
            "    > Example: move,mm,500.5 - moves to 500.5mm\n"
            "  move,counts,X - Move to absolute position X in encoder counts\n"
            "    > Valid range: 0 to 64,333 counts\n"
            "    > Used for precise control or debugging\n"
            "    > 1mm ≈ 61.27 counts (3200 pulses/rev ÷ 52.23mm/rev)\n"
            "  move,rel,X - Move X millimeters relative to current position\n"
            "    > Use positive values to move forward\n"
            "    > Use negative values to move backward\n"
            "    > Example: move,rel,-10 - moves 10mm backward\n"
            "    > Movement is constrained to valid range (0-1050.0mm)\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If movement fails, check motor status with 'motor,status'\n"
            "  • If at travel limits, you can only move within the allowed range\n"
            "  • After E-Stop, clear faults with 'motor,clear' before moving\n"
            "  • For short, precise movements, consider using 'jog' commands\n"
            "  • For interactive positioning, use 'encoder' handwheel control\n"
            "-------------------------------------------"
        ));
        return true;

    default:
        sprintf(msg, "[ERROR], Invalid position: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options: home, 1, 2, 3, 4, counts, mm, rel, help"));
        return false;
    }

    return false; // Should never reach here, but included for completeness
}

// Define the jog subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo JOG_COMMANDS[] = {
    {"+", 1},
    {"-", 2},
    {"help", 6},
    {"inc", 3},
    {"speed", 4},
    {"status", 5}};

static const size_t JOG_COMMAND_COUNT = sizeof(JOG_COMMANDS) / sizeof(SubcommandInfo);

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
        if (motorState == MOTOR_STATE_NOT_READY)
        {
            Console.error(F("Motor is not initialized. Use 'motor,init' first."));
            return false;
        }
        if (isEStopActive())
        {
            Console.error(F("Cannot jog while E-Stop is active"));
            Console.serialInfo(F("Release E-Stop and try again."));
            return false;
        }
        if (motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor is in fault state. Use 'motor,clear' first."));
            return false;
        }
        if (motorState == MOTOR_STATE_HOMING)
        {
            Console.error(F("Cannot jog while homing is in progress."));
            return false;
        }
        if (motorState == MOTOR_STATE_MOVING)
        {
            Console.error(F("Motor is already moving. Use 'motor,abort' first."));
            return false;
        }
        if (!isHomed)
        {
            Console.error(F("Motor is not homed. Use 'motor,home' command first."));
            return false;
        }
    }

    int subcommandCode = findSubcommandCode(subcommand, JOG_COMMANDS, JOG_COMMAND_COUNT);
    char msg[100];

    switch (subcommandCode)
    {
    case 1: // jog forward (+)
    {
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm + currentJogIncrementMm;

        if (targetPositionMm > MAX_TRAVEL_MM)
        {
            Console.error(F("Cannot jog beyond maximum position limit"));
            sprintf(msg, "Maximum position: %.1f mm | Current position: %.2f mm", MAX_TRAVEL_MM, currentPositionMm);
            Console.serialInfo(msg);
            return false;
        }

        sprintf(msg, "Jogging forward %.2f mm from position %.2f mm to %.2f mm",
                currentJogIncrementMm, currentPositionMm, targetPositionMm);
        Console.serialInfo(msg);

        if (jogMotor(true))
        {
            sprintf(msg, "[ACK], JOG_FORWARD_%.2f mm", currentJogIncrementMm);
            Console.acknowledge(msg);
            return true;
        }
        else
        {
            Console.error(F("Failed to initiate jog movement"));
            return false;
        }
    }

    case 2: // jog backward (-)
    {
        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        double targetPositionMm = currentPositionMm - currentJogIncrementMm;

        if (targetPositionMm < 0.0)
        {
            Console.error(F("Cannot jog beyond minimum position limit"));
            sprintf(msg, "Current position: %.2f mm", currentPositionMm);
            Console.serialInfo(msg);
            return false;
        }

        sprintf(msg, "Jogging backward %.2f mm from position %.2f mm to %.2f mm",
                currentJogIncrementMm, currentPositionMm, targetPositionMm);
        Console.serialInfo(msg);

        if (jogMotor(false))
        {
            sprintf(msg, "[ACK], JOG_BACKWARD_%.2f mm", currentJogIncrementMm);
            Console.acknowledge(msg);
            return true;
        }
        else
        {
            Console.error(F("Failed to initiate jog movement"));
            return false;
        }
    }

    case 3: // set/get increment
    {
        char *incStr = strtok(NULL, " ");
        if (incStr == NULL)
        {
            sprintf(msg, "[ACK], JOG_INC_%.2f mm", currentJogIncrementMm);
            Console.acknowledge(msg);
            return true;
        }
        else
        {
            incStr = trimLeadingSpaces(incStr);

            if (strcmp(incStr, "default") == 0)
            {
                if (setJogIncrement(DEFAULT_JOG_INCREMENT))
                {
                    sprintf(msg, "[INFO] Jog increment set to default (%.2f mm)", currentJogIncrementMm);
                    Console.serialInfo(msg);
                    return true;
                }
                else
                {
                    Console.error(F("Failed to set default jog increment"));
                    return false;
                }
            }

            double newIncrement = atof(incStr);

            if (setJogIncrement(newIncrement))
            {
                sprintf(msg, "[ACK], JOG_INC_SET_%.2f mm", currentJogIncrementMm);
                Console.acknowledge(msg);
                return true;
            }
            else
            {
                Console.error(F("Invalid jog increment value"));
                return false;
            }
        }
    }

    case 4: // set/get speed
    {
        char *speedStr = strtok(NULL, " ");
        if (speedStr == NULL)
        {
            sprintf(msg, "[INFO] Current jog speed: %d RPM", currentJogSpeedRpm);
            Console.serialInfo(msg);
            return true;
        }
        else
        {
            speedStr = trimLeadingSpaces(speedStr);

            if (strcmp(speedStr, "default") == 0)
            {
                if (setJogSpeed(DEFAULT_JOG_SPEED, currentJogIncrementMm))
                {
                    sprintf(msg, "[ACK], JOG_SPEED_DEFAULT_%d RPM", currentJogSpeedRpm);
                    Console.acknowledge(msg);
                    return true;
                }
                else
                {
                    Console.error(F("Failed to set default jog speed"));
                    return false;
                }
            }

            int newSpeed = atoi(speedStr);

            if (setJogSpeed(newSpeed, currentJogIncrementMm))
            {
                sprintf(msg, "[ACK], JOG_SPEED_SET_%d RPM", currentJogSpeedRpm);
                Console.acknowledge(msg);
                return true;
            }
            else
            {
                Console.error(F("Invalid jog speed value"));
                return false;
            }
        }
    }

    case 5: // status
    {
        Console.acknowledge(F("JOG_STATUS"));
        Console.println(F("[INFO] Current jog settings:"));

        sprintf(msg, "  Increment: %.2f mm", currentJogIncrementMm);
        Console.println(msg);

        sprintf(msg, "  Speed: %d RPM", currentJogSpeedRpm);
        Console.println(msg);

        double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        sprintf(msg, "  Current position: %.2f mm", currentPositionMm);
        Console.println(msg);

        double maxForwardJog = MAX_TRAVEL_MM - currentPositionMm;
        double maxBackwardJog = currentPositionMm;

        sprintf(msg, "  Max forward jog: %.2f mm", maxForwardJog);
        Console.println(msg);

        sprintf(msg, "  Max backward jog: %.2f mm", maxBackwardJog);
        Console.println(msg);

        return true;
    }

    case 6: // help
    {
        Console.acknowledge(F("JOG_HELP"));
        Console.println(F(
            "\n===== JOG MOVEMENT SYSTEM HELP =====\n"
            "\nOVERVIEW:\n"
            "  The jog system provides precise, incremental movements in either direction\n"
            "  for accurate positioning and testing. Each jog moves the motor by a fixed\n"
            "  distance that you can configure.\n"
            "\n"
            "COMMAND REFERENCE:\n"
            "  jog,+ - Move forward by one increment\n"
            "    > Each press moves exactly one increment in the forward direction\n"
            "    > Movement stops automatically after the increment is completed\n"
            "  jog,- - Move backward by one increment\n"
            "    > Each press moves exactly one increment in the backward direction\n"
            "    > Movement stops automatically after the increment is completed\n"
            "  jog,inc,X - Set movement increment size\n"
            "    > X = distance in millimeters (example: jog,inc,5.0)\n"
            "    > Using jog,inc without a value displays the current setting\n"
            "    > Using jog,inc,default resets to standard increment\n"
            "  jog,speed,X - Set movement speed\n"
            "    > X = speed in RPM (example: jog,speed,300)\n"
            "    > Using jog,speed without a value displays the current setting\n"
            "    > Using jog,speed,default resets to standard speed\n"
            "  jog,status - Display current jog settings and position information\n"
            "\n"
            "JOG VS. HANDWHEEL COMPARISON:\n"
            "  Jog System (jog command):\n"
            "    • Fixed, precise movements with each command\n"
            "    • Better for repeatable, exact positioning\n"
            "    • Simple to use via command line\n"
            "    • Good for testing and calibration\n"
            "    • Can be used in scripts and automated sequences\n"
            "\n"
            "  Handwheel System (encoder command):\n"
            "    • Continuous, manual control with physical handwheel\n"
            "    • Better for interactive positioning and fine adjustments\n"
            "    • More intuitive for operators doing manual work\n"
            "    • Allows variable speed based on rotation speed\n"
            "    • Provides tactile feedback during positioning\n"
            "\n"
            "WHEN TO USE JOG:\n"
            "  • For test sequences that need repeatable movements\n"
            "  • When working remotely via serial connection\n"
            "  • When you need precisely measured movements\n"
            "  • For calibration procedures\n"
            "  • When you don't have access to the physical handwheel\n"
            "\n"
            "USAGE TIPS:\n"
            "  • Set a smaller increment (1-5mm) for precise positioning\n"
            "  • Set a larger increment (10-50mm) for faster travel\n"
            "  • Use jog,status to see your current position and limits\n"
            "  • The motor must be homed before jogging can be used\n"
            "  • Jogging is automatically limited to prevent over-travel\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If jog commands fail, check if motor is initialized and homed\n"
            "  • If at travel limit, you can only jog in the opposite direction\n"
            "  • After E-Stop, clear any faults before attempting to jog\n"
            "  • If motor is already moving, wait for it to complete or use motor,abort\n"
            "-------------------------------------------"
        ));

        return true;
    }

    default:
    {
        sprintf(msg, "[ERROR], Unknown jog command: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options are '+', '-', 'inc', 'speed', 'status', or 'help'"));
        return false;
    }
    }

    return false; // Should never reach here, but included for completeness
}

// Define the system subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo SYSTEM_COMMANDS[] = {
    {"help", 6},
    {"history", 5},
    {"reset", 4},
    {"safety", 2},
    {"state", 1},
    {"trays", 3}};

static const size_t SYSTEM_COMMAND_COUNT = sizeof(SYSTEM_COMMANDS) / sizeof(SubcommandInfo);

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
        Console.error(F("Missing subcommand. Valid options: state, safety, trays, history, reset"));
        return false;
    }

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, SYSTEM_COMMANDS, SYSTEM_COMMAND_COUNT);

    char msg[100];

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "state"
    {
        Console.acknowledge(F("SYSTEM_STATE"));
        // Capture and print system state
        SystemState currentState = captureSystemState();
        printSystemState(currentState);
        return true;
    }

    case 2: // "safety"
    {
        Console.acknowledge(F("SAFETY_STATE"));

        // Capture system state, validate safety, and print results
        SystemState currentState = captureSystemState();
        SafetyValidationResult safety = validateSafety(currentState);

        printSafetyStatus(safety);

        return true;
    }

    case 3: // "trays"
    {
        // Display tray system status
        SystemState currentState = captureSystemState();
        updateTrayTrackingFromSensors(currentState);
        Console.acknowledge(F("TRAY_STATUS"));

        sprintf(msg, "Total trays in system: %d", trayTracking.totalTraysInSystem);
        Console.println(msg);

        Console.println(F("\nPosition occupancy:"));
        sprintf(msg, "  Position 1 (Loading): %s", trayTracking.position1Occupied ? "OCCUPIED" : "EMPTY");
        Console.println(msg);
        sprintf(msg, "  Position 2 (Middle): %s", trayTracking.position2Occupied ? "OCCUPIED" : "EMPTY");
        Console.println(msg);
        sprintf(msg, "  Position 3 (Unloading): %s", trayTracking.position3Occupied ? "OCCUPIED" : "EMPTY");
        Console.println(msg);

        Console.println(F("\nOperation statistics:"));
        sprintf(msg, "  Total loads completed: %d", trayTracking.totalLoadsCompleted);
        Console.println(msg);
        sprintf(msg, "  Total unloads completed: %d", trayTracking.totalUnloadsCompleted);
        Console.println(msg);

        if (trayTracking.lastLoadTime > 0)
        {
            unsigned long secondsAgo = timeDiff(millis(), trayTracking.lastLoadTime) / 1000;
            Console.print(F("  Last load: "));
            printHumanReadableTime(secondsAgo);
            Console.println(F(" ago"));
        }

        if (trayTracking.lastUnloadTime > 0)
        {
            unsigned long secondsAgo = timeDiff(millis(), trayTracking.lastUnloadTime) / 1000;
            Console.print(F("  Last unload: "));
            printHumanReadableTime(secondsAgo);
            Console.println(F(" ago"));
        }
        return true;
    }

    case 4: // "reset"
    {
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
            Console.serialInfo(F("FAULT_CLEARED"));
        }

        if (wasOperationInProgress)
        {
            Console.serialInfo(F("OPERATION_CLEARED"));
        }

        return true;
    }

    case 5: // "history"
    {
        Console.acknowledge(F("SYSTEM_HISTORY"));
        opLogHistory.printHistory();
        return true;
    }

    case 6: // "help"
    {
        Console.acknowledge(F("SYSTEM_HELP"));
        Console.println(F(
            "\n===== SYSTEM COMMAND HELP =====\n"
            "\nOVERVIEW:\n"
            "  The system command provides access to core system functions,\n"
            "  status information, and diagnostic capabilities. Use these\n"
            "  commands to monitor and manage the overall system state.\n"
            "\n"
            "COMMAND REFERENCE:\n"
            "  system,state - Display comprehensive system state\n"
            "    > Shows all sensor readings, valve positions, and motor status\n"
            "    > Provides complete snapshot of current hardware state\n"
            "    > Use for diagnostics and troubleshooting\n"
            "\n"
            "  system,safety - Display safety validation status\n"
            "    > Shows detailed safety checks and their current status\n"
            "    > Reports any safety constraints preventing operations\n"
            "    > Provides reasons why operations might be blocked\n"
            "\n"
            "  system,trays - Display tray tracking information\n"
            "    > Shows which positions are occupied\n"
            "    > Reports total number of trays in the system\n"
            "    > Provides statistics on load/unload operations\n"
            "    > Shows timestamps of recent operations\n"
            "\n"
            "  system,reset - Reset system state after failure\n"
            "    > Clears fault conditions\n"
            "    > Aborts any in-progress operations\n"
            "    > Use when system is in an inconsistent state\n"
            "    > Emergency recovery function for error conditions\n"
            "\n"
            "  system,history - Display operation log history\n"
            "    > Shows recent operational messages in chronological order\n"
            "    > Useful for troubleshooting failures\n"
            "    > Captures the last 20 operational events\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • For hardware issues: Check 'system,state' for sensor/valve status\n"
            "  • For operation failures: Check 'system,safety' for constraints\n"
            "  • For tray inconsistencies: Use 'system,trays' to verify positions\n"
            "  • When stuck in error state: Try 'system,reset' to recover\n"
            "  • After E-Stop activation: Use 'motor,clear' followed by 'system,reset'\n"
            "  • For debugging overnight failures: Use 'system,history' to see operational logs\n"
            "  • Review history to understand sequence of events leading to failures\n"
            "  • History logs are preserved until power cycle or buffer fills (20 entries)\n"
            "  • Log critical commands/operations before leaving system unattended\n"
            "-------------------------------------------"
        ));

        return true;
    }

    default: // Unknown subcommand
    {
        sprintf(msg, "Unknown system command: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options are 'system,state', 'system,safety', 'system,trays', 'system,reset', 'system,history', or 'system,help'"));
        return false;
    }
    }

    return false; // Should never reach here
}

// Define the tray subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo TRAY_COMMANDS[] = {
    {"gripped", 5},
    {"help", 8},
    {"load", 1}, // Base command, will check for "ready" or "request" separately
    {"placed", 2},
    {"released", 3},
    {"removed", 6},
    {"status", 7},
    {"unload", 4} // Base command, will check for "ready" or "request" separately
};

static const size_t TRAY_COMMAND_COUNT = sizeof(TRAY_COMMANDS) / sizeof(SubcommandInfo);

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
        Console.error(F("Missing parameter. Usage: tray,<load|unload|load,ready|unload,ready|placed|gripped|released|status|help>"));
        return false;
    }

    // Parse the subcommand - SPLIT BY BOTH SPACES AND COMMAS
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL)
    {
        Console.error(F("Missing parameter. Usage: tray,<load|unload|load,ready|unload,ready|placed|gripped|released|status|help>"));
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
            Console.error(F("Missing parameter. Usage: tray,<load|unload|load,ready|unload,ready|placed|gripped|released|status|help>"));
            return false;
        }
        subcommand = trimLeadingSpaces(subcommand);
    }

    // First use binary search to find the base command
    int cmdCode = findSubcommandCode(subcommand, TRAY_COMMANDS, TRAY_COMMAND_COUNT);

    // Shared buffer for all formatted output in this function
    char msg[100];

    // Handle the compound commands for load and unload with the second parameter
    if (cmdCode == 1 || cmdCode == 4)
    { // "load" or "unload"
        char *action = strtok(NULL, " ");
        if (action != NULL && strcmp(action, "ready") == 0)
        {
            // Convert to the corresponding "ready" command code
            cmdCode = (cmdCode == 1) ? 9 : 10; // 9 for load,ready, 10 for unload,ready
        }
        // For all other cases, keep the original code (1 for load, 4 for unload)
    }
    else if (cmdCode == 0)
    { // Command not found in binary search
        sprintf(msg, "[ERROR] Unknown tray command: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options: load,request | unload,request | load,ready | unload,ready | placed | gripped | removed | released | status | help"));
        return false;
    }

    // Check system safety state before processing any tray command
    // (except help and status which should always work)
    if (cmdCode != 7 && cmdCode != 8)
    { // Skip check for status and help commands
        // Capture current system state and validate safety
        SystemState state = captureSystemState();
        SafetyValidationResult safety = validateSafety(state);

        // Comprehensive safety validation with prioritized messages
        bool safeToExecute = true;
        String errorReason = "";

        // Check emergency conditions first (highest priority)
        if (state.eStopActive)
        {
            safeToExecute = false;
            errorReason = F("E-STOP_ACTIVE");
        }
        // Check motor fault conditions
        else if (state.motorState == MOTOR_STATE_FAULTED &&
                 (cmdCode == 1 || cmdCode == 4 || cmdCode == 9 || cmdCode == 10))
        {
            safeToExecute = false;
            errorReason = F("MOTOR_FAULTED");
        }
        // Check pneumatic pressure for valve operations
        else if (!safety.pneumaticPressureSufficient &&
                 (cmdCode == 2 || cmdCode == 3 || cmdCode == 5 || cmdCode == 6))
        {
            safeToExecute = false;
            errorReason = F("INSUFFICIENT_PRESSURE");
        }
        // Check for lock/unlock operation failures
        else if ((!safety.lockOperationSuccessful || !safety.unlockOperationSuccessful) &&
                 (cmdCode == 1 || cmdCode == 2 || cmdCode == 3 || cmdCode == 4 ||
                  cmdCode == 5 || cmdCode == 6 || cmdCode == 9 || cmdCode == 10))
        {
            safeToExecute = false;
            errorReason = F("VALVE_OPERATION_FAILURE");

            // Provide specific details about which operation failed
            if (!safety.lockOperationSuccessful)
            {
                Console.serialInfo(safety.lockFailureDetails.c_str());
            }
            if (!safety.unlockOperationSuccessful)
            {
                Console.serialInfo(safety.unlockFailureDetails.c_str());
            }
        }
        // Check operation sequence validity
        else if (!safety.operationSequenceValid)
        {
            safeToExecute = false;
            errorReason = F("SEQUENCE_ERROR");
        }
        // Check position tracking (your original checks)
        else if (!safety.targetPositionValid || !safety.trayPositionValid)
        {
            safeToExecute = false;
            errorReason = F("POSITION_TRACKING_ERROR");
        }

        if (!safeToExecute)
        {
            sprintf(msg, "[ERROR], %s", errorReason.c_str());
            Console.error(msg);
            Console.serialInfo(F("Cannot execute tray commands while system is in an unsafe state"));
            Console.serialInfo(F("Use 'system,reset' to clear the alert and try again"));
            Console.serialInfo(F("For diagnosis, use 'system,safety' to see detailed system status"));
            return false;
        }
    }

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "load,request"
    {
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);
        SafetyValidationResult safety = validateSafety(state);

        if (!safety.safeToMove)
        {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(safety.moveUnsafeReason.c_str());
            Console.serialInfo(F("Motor must be initialized and homed before loading/unloading operations"));
            return false;
        }

        if (trayTracking.position1Occupied && trayTracking.position2Occupied && trayTracking.position3Occupied)
        {
            Console.error(F("SYSTEM_FULL"));
            Console.serialInfo(F("System is full - all positions occupied"));
            return false;
        }

        if (trayTracking.position1Occupied)
        {
            Console.error(F("POSITION_OCCUPIED"));
            return false;
        }

        if (operationInProgress)
        {
            Console.error(F("SYSTEM_BUSY"));
            return false;
        }

        if (!safety.safeToLoadTrayToPos1)
        {
            Console.error(F("UNSAFE_TO_LOAD"));
            Console.serialInfo(safety.loadTrayPos1UnsafeReason.c_str());
            return false;
        }

        if (!moveToPositionMm(POSITION_1_MM))
        {
            Console.error(F("MOVE_FAILURE"));
            return false;
        }

        Console.acknowledge(F("READY_TO_RECEIVE"));

        // Add helpful message about the overall loading process
        sprintf(msg, "%s tray will be moved to position %s after placement",
            trayTracking.totalTraysInSystem == 0 ? "First" :
            trayTracking.totalTraysInSystem == 1 ? "Second" : "Third",
            trayTracking.totalTraysInSystem == 0 ? "3" :
            trayTracking.totalTraysInSystem == 1 ? "2" : "1");
        Console.serialInfo(msg);

        return true;
    }

    case 2: // "placed"
    {
        trayTracking.position1Occupied = true;
        trayTracking.lastLoadTime = millis();

        SystemState state = captureSystemState();
        if (!state.tray1Present)
        {
            Console.error(F("NO_TRAY_DETECTED"));
            Console.serialInfo(F("If you encounter issues, use 'system,reset' to reset the system state"));
            return false;
        }

        DoubleSolenoidValve *valve = getTray1Valve();
        CylinderSensor *sensor = getTray1Sensor();

        if (valve && sensor)
        {
            if (valve->position != VALVE_POSITION_LOCK)
            {
                if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
                {
                    Console.error(F("LOCK_FAILURE"));
                    Console.serialInfo(F("Failed to lock tray - sensor didn't confirm"));
                    Console.serialInfo(F("Check air pressure and valve functionality"));
                    return false;
                }
                Console.acknowledge(F("TRAY_SECURED"));
                trayTracking.lastLoadTime = millis();
                return true;
            }
            else
            {
                Console.acknowledge(F("TRAY_ALREADY_SECURED"));
                trayTracking.lastLoadTime = millis();
                return true;
            }
        }
        else
        {
            Console.error(F("LOCK_FAILURE"));
            return false;
        }
    }

    case 3: // "released"
    {
        beginOperation();
        currentOperation.inProgress = true;
        currentOperation.type = OPERATION_LOADING;
        currentOperation.startTime = millis();
        Console.acknowledge(F("STARTING_PROCESSING"));
        return true;
    }

    case 4: // "unload,request"
    {
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);

        if (!state.isHomed || state.motorState == MOTOR_STATE_FAULTED ||
            state.motorState == MOTOR_STATE_NOT_READY)
        {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(F("Motor not initialized or homed"));
            Console.serialInfo(F("Use 'motor,init' and 'motor,home' commands"));
            return false;
        }

        if (trayTracking.totalTraysInSystem == 0)
        {
            Console.error(F("NO_TRAYS_TO_UNLOAD"));
            return false;
        }

        if (operationInProgress)
        {
            Console.error(F("SYSTEM_BUSY"));
            return false;
        }

        if (state.tray1Present)
        {
            Console.serialInfo(F("Tray at position 1 (loading position) ready to be gripped"));
            Console.serialInfo(F("Tray at position 1 is locked and ready for gripping"));
            Console.acknowledge(F("TRAY_READY_FOR_GRIP"));

            DoubleSolenoidValve *valve = getTray1Valve();
            CylinderSensor *sensor = getTray1Sensor();

            if (valve && sensor && valve->position != VALVE_POSITION_LOCK)
            {
                safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS);
            }

            return true;
        }
        else
        {
            if (state.tray2Present)
            {
                Console.serialInfo(F("Moving tray from position 2 to position 1 for unloading"));
            }
            else if (state.tray3Present)
            {
                Console.serialInfo(F("Moving tray from position 3 to position 1 for unloading"));
            }

            beginOperation();
            currentOperation.inProgress = true;
            currentOperation.type = OPERATION_UNLOADING;
            currentOperation.startTime = millis();

            Console.acknowledge(F("PREPARING_TRAY"));
            return true;
        }
    }

    case 5: // "gripped"
    {
        SystemState state = captureSystemState();
        SafetyValidationResult safety = validateSafety(state);

        if (!safety.safeToUnlockGrippedTray)
        {
            Console.error(F("UNSAFE_TO_UNLOCK"));
            Console.serialInfo(safety.grippedTrayUnlockUnsafeReason.c_str());
            return false;
        }

        DoubleSolenoidValve *valve = getTray1Valve();
        CylinderSensor *sensor = getTray1Sensor();

        if (!valve || !sensor)
        {
            Console.error(F("VALVE_ACCESS_ERROR"));
            Console.serialInfo(F("Failed to access tray 1 valve or sensor"));
            return false;
        }

        if (valve->position == VALVE_POSITION_UNLOCK)
        {
            Console.acknowledge(F("TRAY_ALREADY_UNLOCKED"));
            return true;
        }

        Console.serialInfo(F("Mitsubishi has gripped the tray. Unlocking tray at position 1..."));
        if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
        {
            Console.error(F("UNLOCK_FAILURE"));
            Console.serialInfo(F("Failed to unlock tray - sensor didn't confirm"));
            Console.serialInfo(F("Check air pressure and valve functionality"));
            return false;
        }

        Console.acknowledge(F("TRAY_UNLOCKED"));
        Console.serialInfo(F("Mitsubishi can now remove the tray"));
        return true;
    }

    case 6: // "removed"
    {
        SystemState state = captureSystemState();
        if (state.tray1Present)
        {
            Console.error(F("TRAY_STILL_PRESENT"));
            Console.serialInfo(F("Sensor still detects a tray at position 1"));
            return false;
        }

        unloadFirstTray();
        trayTracking.totalUnloadsCompleted++;

        Console.acknowledge(F("TRAY_REMOVAL_CONFIRMED"));
        sprintf(msg, "[INFO] Total unloads completed: %d", trayTracking.totalUnloadsCompleted);
        Console.serialInfo(msg);
        return true;
    }

    case 7: // "status"
    {
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);

        sprintf(msg, "TRAYS_TOTAL:%d", trayTracking.totalTraysInSystem);
        Console.println(msg);

        sprintf(msg, "POS1:%d", trayTracking.position1Occupied ? 1 : 0);
        Console.println(msg);
        sprintf(msg, "POS2:%d", trayTracking.position2Occupied ? 1 : 0);
        Console.println(msg);
        sprintf(msg, "POS3:%d", trayTracking.position3Occupied ? 1 : 0);
        Console.println(msg);

        sprintf(msg, "LOCK1:%d", state.tray1Locked ? 1 : 0);
        Console.println(msg);
        sprintf(msg, "LOCK2:%d", state.tray2Locked ? 1 : 0);
        Console.println(msg);
        sprintf(msg, "LOCK3:%d", state.tray3Locked ? 1 : 0);
        Console.println(msg);

        sprintf(msg, "PRESSURE:%d", isPressureSufficient() ? 1 : 0);
        Console.println(msg);

        sprintf(msg, "LOADS:%d", trayTracking.totalLoadsCompleted);
        Console.println(msg);
        sprintf(msg, "UNLOADS:%d", trayTracking.totalUnloadsCompleted);
        Console.println(msg);

        return true;
    }

    case 8: // "help"
    {
        Console.acknowledge(F("TRAY_HELP"));
        Console.println(F(
            "\n===== TRAY SYSTEM HELP =====\n"
            "\nTRAY LOADING SEQUENCE:\n"
            "  1. tray,load,request - Request permission to load a tray\n"
            "     > System will validate position 1 is empty and move shuttle there\n"
            "     > System responds with 'READY_TO_RECEIVE' when ready\n"
            "  2. tray,load,ready - Check if system is ready to receive a tray\n"
            "     > Returns [ACK] READY_TO_LOAD if system can accept a tray\n"
            "     > Returns [BUSY] if system is processing another tray\n"
            "     > Returns [ERROR] with reason if system cannot accept a tray\n"
            "  3. tray,placed - Notify system that tray has been physically placed\n"
            "     > System will lock the tray at position 1\n"
            "     > System responds with 'TRAY_SECURED' when complete\n"
            "  4. tray,released - Notify system to start processing the tray\n"
            "     > System will move tray to appropriate position based on system state\n"
            "     > First tray goes to position 3, second to position 2, third stays at position 1\n"
            "\n"
            "TRAY UNLOADING SEQUENCE:\n"
            "  1. tray,unload,request - Request permission to unload a tray\n"
            "     > If tray at position 1, system prepares it (shuttle retracted, tray locked)\n"
            "     > If tray at positions 2 or 3, system moves it to position 1 first\n"
            "     > System responds with 'TRAY_READY_FOR_GRIP' when at position 1\n"
            "  2. tray,unload,ready - Check if a tray is ready for pickup\n"
            "     > Returns [ACK] TRAY_READY_FOR_PICKUP if a tray is available\n"
            "     > Returns [BUSY] if system is preparing a tray for unloading\n"
            "     > Returns [ERROR] with reason if no tray is available\n"
            "     > This command can be used during operations\n"
            "  3. tray,gripped - Notify system that robot has gripped the tray\n"
            "     > Robot must have secure grip on tray before sending this command\n"
            "     > System unlocks the tray and responds with 'TRAY_UNLOCKED'\n"
            "  4. tray,removed - Notify system that tray has been physically removed\n"
            "     > System updates internal tracking\n"
            "     > System responds with 'TRAY_REMOVAL_CONFIRMED'\n"
            "\n"
            "TRAY STATUS COMMAND:\n"
            "  tray,status - Returns machine-readable status information\n"
            "  Returned values:\n"
            "    TRAYS_TOTAL:[0-3] - Total number of trays in system\n"
            "    POS1:[0|1] - Position 1 occupancy (0=empty, 1=occupied)\n"
            "    POS2:[0|1] - Position 2 occupancy (0=empty, 1=occupied)\n"
            "    POS3:[0|1] - Position 3 occupancy (0=empty, 1=occupied)\n"
            "    LOCK1:[0|1] - Position 1 lock status (0=unlocked, 1=locked)\n"
            "    LOCK2:[0|1] - Position 2 lock status (0=unlocked, 1=locked)\n"
            "    LOCK3:[0|1] - Position 3 lock status (0=unlocked, 1=locked)\n"
            "    PRESSURE:[0|1] - Pneumatic pressure status (0=insufficient, 1=sufficient)\n"
            "    LOADS:[number] - Total number of loads completed\n"
            "    UNLOADS:[number] - Total number of unloads completed\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If an operation fails, use 'system,reset' to reset the system state\n"
            "  • Use 'system,trays' for human-readable tray system status\n"
            "  • Use 'system,safety' to diagnose safety constraint issues\n"
            "-------------------------------------------"
        ));

        return true;
    }

    case 9: // "load,ready"
    {
        SystemState state = captureSystemState();

        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            Console.println(F("[BUSY], LOADING_OPERATION_IN_PROGRESS"));
            Console.serialInfo(F("System is currently processing a tray, please wait..."));
            return true;
        }

        if (motorState == MOTOR_STATE_NOT_READY || motorState == MOTOR_STATE_FAULTED || !isHomed)
        {
            Console.error(F("MOTOR_NOT_READY"));
            Console.serialInfo(F("Motor must be initialized and homed"));
            return false;
        }

        if (trayTracking.position1Occupied && trayTracking.position2Occupied && trayTracking.position3Occupied)
        {
            Console.error(F("SYSTEM_FULL"));
            Console.serialInfo(F("All positions are occupied"));
            return false;
        }

        if (trayTracking.position1Occupied)
        {
            Console.error(F("POSITION1_OCCUPIED"));
            Console.serialInfo(F("Position 1 already has a tray"));
            return false;
        }

        Console.acknowledge(F("READY_TO_LOAD"));
        Console.serialInfo(F("System is ready to receive a tray at position 1"));
        return true;
    }

    case 10: // "unload,ready"
    {
        SystemState state = captureSystemState();

        if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            Console.println(F("[BUSY], UNLOADING_OPERATION_IN_PROGRESS"));
            Console.serialInfo(F("Tray is being prepared at position 1, please wait..."));
            return true;
        }

        if (trayTracking.totalTraysInSystem == 0)
        {
            Console.error(F("NO_TRAYS_TO_UNLOAD"));
            Console.serialInfo(F("System has no trays to unload"));
            return false;
        }

        if (state.tray1Present && state.tray1Locked)
        {
            Console.acknowledge(F("TRAY_READY_FOR_PICKUP"));
            Console.serialInfo(F("Tray at position 1 is locked and ready to be gripped"));
            return true;
        }
        else if (state.tray1Present && !state.tray1Locked)
        {
            Console.error(F("TRAY_NOT_LOCKED"));
            Console.serialInfo(F("Tray at position 1 is not locked"));
            return false;
        }
        else if (!state.tray1Present && (state.tray2Present || state.tray3Present))
        {
            Console.error(F("NO_TRAY_AT_POSITION1"));
            Console.serialInfo(F("No tray at position 1, but trays exist at other positions"));
            Console.serialInfo(F("Use tray,unload,request to move a tray to position 1"));
            return true;
        }
        else
        {
            Console.error(F("SYSTEM_STATE_INCONSISTENT"));
            Console.serialInfo(F("Tray tracking inconsistency detected"));
            return false;
        }
    }

    default:
    {
        sprintf(msg, "[ERROR] Unknown tray command: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options: load,request | unload,request | load,ready | unload,ready | placed | gripped | removed | released | status | help"));
        return false;
    }
    }

    return false; // Should never reach here
}

// Define the test subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo TEST_COMMANDS[] = {
    {"help", 4},
    {"home", 1},
    {"position", 2},
    {"tray", 3}};

static const size_t TEST_COMMAND_COUNT = sizeof(TEST_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_test(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    char msg[100];

    // Check for empty argument - show brief help to both outputs
    if (strlen(trimmed) == 0)
    {
        Console.println(F("[ACK], TEST_COMMANDS"));
        Console.serialInfo(F("Available: home | position | tray | help"));

        // More detailed options to Serial only
        Console.serialInfo(F("  home     - Test homing repeatability"));
        Console.serialInfo(F("  position - Test position cycling (for tray loading)"));
        Console.serialInfo(F("  tray     - Test complete tray handling operations"));
        Console.serialInfo(F("  help     - Display detailed test information"));
        Console.serialInfo(F("Usage: test,<test_name>"));
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

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, TEST_COMMANDS, TEST_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "home"
    {
        Console.acknowledge(F("TEST_HOME_STARTED"));
        Console.serialInfo(F("Starting homing repeatability test..."));
        if (testHomingRepeatability())
        {
            Console.acknowledge(F("TEST_HOME_COMPLETED"));
            return true;
        }
        else
        {
            Console.error(F("Homing test failed or was aborted"));
            return false;
        }
    }

    case 2: // "position"
    {
        Console.acknowledge(F("TEST_POSITION_STARTED"));
        Console.serialInfo(F("Starting position cycling test..."));

        if (testPositionCycling())
        {
            Console.acknowledge(F("TEST_POSITION_COMPLETED"));
            return true;
        }
        else
        {
            Console.error(F("Position test failed or was aborted"));
            return false;
        }
    }

    case 3: // "tray"
    {
        Console.acknowledge(F("TEST_TRAY_STARTED"));
        Console.serialInfo(F("Starting tray handling test..."));

        // Add pneumatic pressure validation before starting the test
        if (!isPressureSufficient())
        {
            Console.error(F("Pneumatic pressure insufficient"));
            Console.serialInfo(F("Cannot run tray test - ensure air supply is connected"));
            Console.serialInfo(F("Use 'tray,status' to check PRESSURE status"));
            return false;
        }

        if (testTrayHandling())
        {
            Console.acknowledge(F("TEST_TRAY_COMPLETED"));
            return true;
        }
        else
        {
            Console.error(F("Tray test failed or was aborted"));
            return false;
        }
    }

    case 4: // "help"
    {
        Console.acknowledge(F("TEST_HELP"));
        Console.println(F(
            "\n===== TEST SYSTEM HELP =====\n"
            "\nOVERVIEW:\n"
            "  The test system provides automated sequences for validating\n"
            "  system functionality and repeatability. Tests are designed\n"
            "  to verify proper operation of critical system components.\n"
            "\n"
            "AVAILABLE TESTS:\n"
            "  test,home - Homing repeatability test\n"
            "    > Performs multiple homing operations to test precision\n"
            "    > Moves between home position and test position\n"
            "    > Useful for verifying encoder and limit switch reliability\n"
            "    > Test runs for approximately 20 cycles\n"
            "\n"
            "  test,position - Position cycling test\n"
            "    > Cycles through positions used in tray loading\n"
            "    > Tests movements between positions 1, 2, and 3\n"
            "    > Verifies motor accuracy and repeatability\n"
            "    > Test runs for approximately 10 cycles\n"
            "\n"
            "  test,tray - Comprehensive tray handling test\n"
            "    > Tests complete tray movement operations\n"
            "    > Includes valve operations for locking/unlocking\n"
            "    > Verifies sensors, positioning, and control sequences\n"
            "    > Most thorough test of the entire system\n"
            "\n"
            "RUNNING TESTS:\n"
            "  • Motor must be initialized (motor,init) before testing\n"
            "  • Home position must be established for position tests\n"
            "  • E-Stop must be inactive\n"
            "  • Tests can be aborted by typing 'abort' during execution\n"
            "  • Status messages display progress throughout the test\n"
            "\n"
            "TRAY TEST REQUIREMENTS:\n"
            "  • A tray must be present at position 1 to start\n"
            "  • Positions 2 and 3 must be clear initially\n"
            "  • Air system must be functioning properly\n"
            "  • All valves and sensors must be operational\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If a test fails, check the specific error message\n"
            "  • For position errors: verify motor operation with 'move' commands\n"
            "  • For valve errors: check air pressure and connections\n"
            "  • For sensor errors: verify sensor readings with 'system,state'\n"
            "-------------------------------------------"
        ));

        return true;
    }

    default: // Unknown command
    {
        sprintf(msg, "[ERROR], Unknown test type: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options: home, position, tray, help"));
        return false;
    }
    }

    return false; // Should never reach here
}

// Define the encoder subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo ENCODER_COMMANDS[] = {
    {"disable", 2},
    {"enable", 1},
    {"help", 4},
    {"multiplier", 3}};

static const size_t ENCODER_COMMAND_COUNT = sizeof(ENCODER_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_encoder(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    char msg[100];

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        // Brief status to both outputs
        Console.acknowledge(F("ENCODER_STATUS"));

        // Show simple status to both outputs
        if (encoderControlActive)
        {
            sprintf(msg, "ENABLED:%d", currentMultiplier);
            Console.println(msg);
        }
        else
        {
            Console.println(F("DISABLED"));
        }

        // Detailed info to Serial only
        Console.serialInfo(F("MPG Handwheel Controls:"));
        Console.serialInfo(F("  encoder,enable          - Enable MPG handwheel control"));
        Console.serialInfo(F("  encoder,disable         - Disable MPG handwheel control"));
        Console.serialInfo(F("  encoder,multiplier,[1|10|100] - Set movement multiplier"));
        Console.serialInfo(F("  encoder,help            - Display detailed usage instructions"));

        // Show current status
        if (encoderControlActive)
        {
            Console.serialInfo(F("\nMPG control is currently ENABLED"));
            sprintf(msg, "[STATUS] Current multiplier: x%d", currentMultiplier);
            Console.serialInfo(msg);

            // Show position information if motor is homed
            if (isHomed)
            {
                double positionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
                sprintf(msg, "[INFO] Current position: %.2f mm", positionMm);
                Console.serialInfo(msg);
            }
        }
        else
        {
            Console.serialInfo(F("\nMPG control is currently DISABLED"));

            // Show reasons why encoder control might not be available
            if (!motorInitialized)
            {
                Console.serialInfo(F("Motor needs to be initialized first (motor,init)"));
            }
            else if (!isHomed)
            {
                Console.serialInfo(F("Motor needs to be homed first (motor,home)"));
            }
        }

        Console.serialInfo(F("\nMULTIPLIERS - Effect of one full handwheel rotation (100 pulses):"));
        sprintf(msg, "  x1: ~%.2f mm (fine adjustment)", 100 * MULTIPLIER_X1 / PULSES_PER_MM);
        Console.serialInfo(msg);
        sprintf(msg, "  x10: ~%.2f mm (medium adjustment)", 100 * MULTIPLIER_X10 / PULSES_PER_MM);
        Console.serialInfo(msg);
        sprintf(msg, "  x100: ~%.2f mm (coarse adjustment)", 100 * MULTIPLIER_X100 / PULSES_PER_MM);
        Console.serialInfo(msg);

        return true;
    }

    // Parse the argument - replace commas with spaces
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

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, ENCODER_COMMANDS, ENCODER_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {
    case 1: // "enable"
    {
        // Check preconditions
        if (!motorInitialized)
        {
            Console.error(F("Motor not initialized"));
            Console.serialInfo(F("Use 'motor,init' first"));
            return false;
        }

        if (!isHomed)
        {
            Console.error(F("Motor not homed"));
            Console.serialInfo(F("Use 'motor,home' to establish a reference position"));
            return false;
        }

        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
        {
            Console.error(F("Motor is moving"));
            Console.serialInfo(F("Wait for current movement to complete or use 'motor,abort'"));
            return false;
        }

        if (motorState == MOTOR_STATE_FAULTED)
        {
            Console.error(F("Motor in fault state"));
            Console.serialInfo(F("Use 'motor,clear' to clear fault first"));
            return false;
        }

        if (isEStopActive())
        {
            Console.error(F("E-Stop is active"));
            return false;
        }

        // Enable encoder control
        encoderControlActive = true;

        // Reset encoder position
        EncoderIn.Position(0);
        lastEncoderPosition = 0;
        lastEncoderUpdateTime = millis();

        Console.acknowledge(F("ENCODER_ENABLED"));

        double posMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
        sprintf(msg, "MPG handwheel control enabled - current position: %.2f mm", posMm);
        Console.serialInfo(msg);

        sprintf(msg, "[INFO] Using multiplier x%s (%d)", getMultiplierName(currentMultiplier), currentMultiplier);
        Console.serialInfo(msg);

        Console.serialInfo(F("Issue 'encoder,disable' when finished with manual control"));

        return true;
    }

    case 2: // "disable"
    {
        // Disable encoder control
        encoderControlActive = false;
        Console.acknowledge(F("ENCODER_DISABLED"));
        Console.serialInfo(F("MPG handwheel control disabled"));
        return true;
    }

    case 3: // "multiplier"
    {
        // Get the next token (the multiplier value)
        char *multStr = strtok(NULL, " ");

        if (multStr != NULL)
        {
            // Parse the multiplier value
            int multiplier = atoi(multStr);

            // Set the multiplier based on the input value
            switch (multiplier)
            {
            case 1:
                setEncoderMultiplier(1);
                Console.acknowledge(F("ENCODER_MULT_1"));
                Console.serialInfo(F("Multiplier set to x1 (fine adjustment)"));
                break;
            case 10:
                setEncoderMultiplier(10);
                Console.acknowledge(F("ENCODER_MULT_10"));
                Console.serialInfo(F("Multiplier set to x10 (medium adjustment)"));
                break;
            case 100:
                setEncoderMultiplier(100);
                Console.acknowledge(F("ENCODER_MULT_100"));
                Console.serialInfo(F("Multiplier set to x100 (coarse adjustment)"));
                break;
            default:
                Console.error(F("Invalid multiplier. Use 1, 10, or 100."));
                return false;
            }

            sprintf(msg, "[INFO] Current multiplier value: %d", currentMultiplier);
            Console.serialInfo(msg);
            double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
            sprintf(msg, "[INFO] One full rotation moves ~%.2f mm", mmPerRotation);
            Console.serialInfo(msg);
            return true;
        }
        else
        {
            // No value provided, just display current multiplier
            sprintf(msg, "[ACK], ENCODER_MULT_%d", currentMultiplier);
            Console.println(msg);

            sprintf(msg, "[INFO] Current multiplier: x%s (%d)", getMultiplierName(currentMultiplier), currentMultiplier);
            Console.serialInfo(msg);

            double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
            sprintf(msg, "[INFO] One full rotation moves ~%.2f mm", mmPerRotation);
            Console.serialInfo(msg);

            return true;
        }
    }

    case 4: // "help"
    {
        Console.acknowledge(F("ENCODER_HELP"));
        Console.println(F(
            "\n===== MPG HANDWHEEL SYSTEM HELP =====\n"
            "\nSETUP SEQUENCE:\n"
            "  1. 'motor,init' - Initialize the motor system\n"
            "  2. 'motor,home' - Home the motor to establish reference position\n"
            "  3. 'encoder,enable' - Activate MPG handwheel control\n"
            "  4. 'encoder,multiplier,X' - Set desired precision (X = 1, 10, or 100)\n"
            "\n"
            "COMMAND REFERENCE:\n"
            "  encoder,enable - Activate handwheel control mode\n"
            "    > Motor position will respond directly to handwheel rotation\n"
            "    > One full rotation (100 pulses) moves distance based on multiplier\n"
            "  encoder,disable - Deactivate handwheel control mode\n"
            "    > Returns system to command-based position control\n"
            "  encoder,multiplier,X - Set movement precision\n"
            "    > X=1: Fine adjustment (~1.63mm per rotation)\n"
            "    > X=10: Medium adjustment (~16.3mm per rotation)\n"
            "    > X=100: Coarse adjustment (~163mm per rotation)\n"
            "\n"
            "AUTOMATIC DISABLING CONDITIONS:\n"
            "  • E-Stop activation - Safety override disables all motor control\n"
            "  • Motor fault condition - Requires 'motor,clear' to reset\n"
            "  • Power cycle or system reset\n"
            "  • When 'move' or 'jog' commands are issued\n"
            "\n"
            "MOVEMENT CONSTRAINTS:\n"
            "  • Hard limit at 0mm (home position)\n"
            "  • Hard limit at maximum travel position (~1050mm)\n"
            "  • Movement stops automatically at travel limits\n"
            "  • No movement allowed if motor is in fault state\n"
            "\n"
            "USAGE TIPS:\n"
            "  • Start with x1 multiplier for precise positioning\n"
            "  • Use x10 or x100 for longer movements\n"
            "  • Monitor current position using 'motor,status' command\n"
            "  • Use 'encoder,disable' when finished with manual control\n"
            "  • Slow, steady handwheel rotation produces smoother movement\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If encoder doesn't respond: Check if motor is initialized and homed\n"
            "  • Erratic movement: Try lower multiplier setting\n"
            "  • No movement at limits: System is preventing over-travel\n"
            "  • After E-Stop: Must re-enable encoder control manually\n"
            "-------------------------------------------"
        ));

        return true;
    }

    default: // Unknown command
    {
        sprintf(msg, "[ERROR], Unknown encoder command: %s", subcommand);
        Console.error(msg);
        Console.error(F("Valid options are 'enable', 'disable', 'multiplier', or 'help'"));
        return false;
    }
    }

    return false; // Should never reach here
}

bool cmd_abort(char *args, CommandCaller *caller)
{
    requestTestAbort("abort command");
    return true;
}

// Define the network subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo NETWORK_COMMANDS[] = {
    {"close", 2},
    {"closeall", 3},
    {"help", 4},
    {"status", 1}};

static const size_t NETWORK_COMMAND_COUNT = sizeof(NETWORK_COMMANDS) / sizeof(SubcommandInfo);

// Network management command
bool cmd_network(char *args, CommandCaller *caller)
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
        Console.error(F("Missing subcommand. Usage: network,<status|close|closeall|help>"));
        return false;
    }

    char msg[100];
    char *subcommand = strtok(trimmed, " ");

    if (subcommand == nullptr)
    {
        // No subcommand provided, show usage
        Console.error(F("Missing subcommand. Usage: network,<status|close|closeall|help>"));
        return false;
    }

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(subcommand, NETWORK_COMMANDS, NETWORK_COMMAND_COUNT);

    switch (cmdCode)
    {
    case 1: // "status"
    {
        Console.acknowledge(F("NETWORK_STATUS"));

        // Show initialization status
        Console.print(F("Ethernet Status: "));
        Console.println(ethernetInitialized ? F("INITIALIZED") : F("NOT INITIALIZED"));

        if (ethernetInitialized)
        {
            // Display IP address
            IPAddress ip = Ethernet.localIP();
            sprintf(msg, "IP Address: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            Console.serialInfo(msg);

            // Display MAC address
            byte mac[6];
            Ethernet.MACAddress(mac);
            sprintf(msg, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            Console.serialInfo(msg);

            // Display port
            sprintf(msg, "Server Port: %d", ETHERNET_PORT);
            Console.serialInfo(msg);

            // Get client count using the shared function
            int connectedCount = getConnectedClientCount();

            // Display connected clients
            Console.serialInfo(F("\nConnected Clients:"));

            if (connectedCount == 0)
            {
                Console.serialInfo(F("  No clients connected"));
            }
            else
            {
                // Only iterate through to display client details
                for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
                {
                    if (clients[i] && clients[i].connected())
                    {
                        IPAddress cip = clients[i].remoteIP();
                        int cport = clients[i].remotePort();
                        sprintf(msg, "  Client %d: %d.%d.%d.%d:%d", i + 1, cip[0], cip[1], cip[2], cip[3], cport);
                        Console.serialInfo(msg);
                    }
                }
            }

            sprintf(msg, "Total Connections: %d of %d", connectedCount, MAX_ETHERNET_CLIENTS);
            Console.serialInfo(msg);
        }

        return true;
    }

    case 2: // "close"
    {
        // Parse the client index
        char *indexStr = strtok(NULL, " ");
        if (indexStr == nullptr)
        {
            Console.error(F("Missing client index"));
            Console.serialInfo(F("Usage: network,close,<client_number>"));
            Console.serialInfo(F("Use network,status to see client numbers"));
            return false;
        }

        int index = atoi(indexStr) - 1; // Convert to 0-based index

        if (index >= 0 && index < MAX_ETHERNET_CLIENTS && clients[index] && clients[index].connected())
        {
            IPAddress ip = clients[index].remoteIP();
            int port = clients[index].remotePort();
            clients[index].stop();

            Console.acknowledge(F("CLIENT_DISCONNECTED"));
            sprintf(msg, "Closed connection from %d.%d.%d.%d:%d", ip[0], ip[1], ip[2], ip[3], port);
            Console.serialInfo(msg);
            return true;
        }
        else
        {
            Console.error(F("INVALID_CLIENT_INDEX"));
            sprintf(msg, "Client index must be between 1 and %d and the client must be connected", MAX_ETHERNET_CLIENTS);
            Console.serialInfo(msg);
            return false;
        }
    }

    case 3: // "closeall"
    {
        int count = 0;
        for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
        {
            if (clients[i] && clients[i].connected())
            {
                clients[i].stop();
                count++;
            }
        }

        Console.acknowledge(F("ALL_CLIENTS_DISCONNECTED"));
        sprintf(msg, "Closed %d connections", count);
        Console.serialInfo(msg);
        return true;
    }

    case 4: // "help"
    {
        Console.acknowledge(F("NETWORK_HELP"));
        Console.println(F(
            "\n===== NETWORK MANAGEMENT HELP =====\n"
            "\nOVERVIEW:\n"
            "  The network system provides commands to monitor and manage\n"
            "  Ethernet connections to the conveyor controller.\n"
            "\n"
            "COMMAND REFERENCE:\n"
            "  network,status - Display network status and connections\n"
            "    > Shows IP address, MAC address, and port\n"
            "    > Lists all currently connected clients\n"
            "    > Displays connection count and maximum capacity\n"
            "\n"
            "  network,close,X - Close a specific client connection\n"
            "    > X = Client number from the status display (1-based)\n"
            "    > Example: network,close,1 - closes the first client\n"
            "    > Useful for disconnecting individual stale connections\n"
            "\n"
            "  network,closeall - Close all client connections\n"
            "    > Forcibly disconnects all connected clients\n"
            "    > Use to recover from connection management issues\n"
            "    > Clients can reconnect after being closed\n"
            "\n"
            "CONNECTION MANAGEMENT:\n"
        ));
        sprintf(msg, "  • System supports up to %d simultaneous client connections", MAX_ETHERNET_CLIENTS);
        Console.println(msg);
        Console.println(F(
            "  • Inactive connections time out after 2 minutes\n"
            "  • System automatically checks for stale connections every 30s\n"
            "  • Commands can be issued through any connected client\n"
            "\n"
            "TROUBLESHOOTING:\n"
            "  • If unable to connect: Check IP address and network settings\n"
            "  • If 'no free slots' error: Use network,status to check connections\n"
            "  • For stale connections: Use network,close or network,closeall\n"
            "  • When cable is disconnected, all clients will be closed automatically\n"
            "-------------------------------------------"
        ));

        return true;
    }

    default: // Unknown command
    {
        sprintf(msg, "Unknown network subcommand: %s", subcommand);
        Console.error(msg);
        Console.serialInfo(F("Valid options are 'status', 'close', 'closeall', or 'help'"));
        return false;
    }
    }

    return false; // Should never reach here
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
                            "  system,reset    - Reset system state after failure to retry operation\r\n"
                            "  system,history  - Display operation log history for troubleshooting\r\n"
                            "  system,help     - Display detailed instructions for system commands",
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
                          "  tray,gripped   - Notify tray has been gripped (Mitsubishi)\r\n"
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

    // Network management command
    systemCommand("network", "Network management:\r\n"
                             "  network,status   - Display current network status and connected clients\r\n"
                             "  network,close,X  - Close a specific client connection (X = client number)\r\n"
                             "  network,closeall - Close all client connections\r\n"
                             "  network,help     - Display detailed network management instructions",
                  cmd_network),
};

const size_t API_tree_size = sizeof(API_tree) / sizeof(Commander::systemCommand_t);