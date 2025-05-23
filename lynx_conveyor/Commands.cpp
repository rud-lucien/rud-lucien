#include "Commands.h"
#include "Arduino.h"
#include "ClearCore.h"
#include "ValveController.h"
#include "MotorController.h"
#include "Tests.h"
#include "Utils.h"
#include "EncoderController.h"

// External declaration for the logging structure
extern LoggingManagement logging;
extern const unsigned long DEFAULT_LOG_INTERVAL;

// ============================================================
// Global Command Tree and Commander Object
// ============================================================

// Function to handle serial input for commands

char *trimLeadingSpaces(char *str)
{
    while (*str && isspace(*str))
    {
        str++;
    }
    return str;
}

void handleSerialCommands()
{
    static char commandBuffer[64]; // Increase from 32 to 64 bytes
    static uint8_t commandIndex = 0;

    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n')
        {
            commandBuffer[commandIndex] = '\0'; // Null-terminate the command

            // Print the received command for debugging
            Serial.print(F("[SERIAL COMMAND] Received: "));
            Serial.println(commandBuffer);

            // Pre-process: Replace all commas with spaces
            for (uint8_t i = 0; i < commandIndex; i++)
            {
                if (commandBuffer[i] == ',')
                {
                    commandBuffer[i] = ' ';
                }
            }

            // Execute the command with commas converted to spaces
            bool success = commander.execute(commandBuffer, &Serial);

            // Print error ONLY when commander couldn't find the command
            // Most functions that return false already print their own error messages
            if (!success) {
                // Check if this looks like a valid main command by checking against the command tree
                bool isKnownCommand = false;
                for (size_t i = 0; i < sizeof(API_tree) / sizeof(Commander::systemCommand_t); i++) {
                    const char* cmdName = API_tree[i].name;
                    // See if the command matches the first part of the input
                    if (strncmp(commandBuffer, cmdName, strlen(cmdName)) == 0) {
                        isKnownCommand = true;
                        break;
                    }
                }
                
                // Only print generic error if command wasn't found in the command tree
                if (!isKnownCommand) {
                    Serial.println(F("[ERROR] Command not found"));
                }
            }

            commandIndex = 0; // Reset buffer index
        }
        else if (c != '\r') // Ignore carriage returns
        {
            if (commandIndex < (sizeof(commandBuffer) - 1))
            {
                commandBuffer[commandIndex++] = c;
            }
            else
            {
                // Command too long - prevent buffer overflow
                commandIndex = sizeof(commandBuffer) - 1;
                // Add notification of truncation
                Serial.println(F("[WARNING] Command truncated - exceeded maximum length"));
            }
        }
    }
}

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
        caller->println(F("[ERROR] Missing parameter. Usage: lock,<1|2|3|shuttle>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *target = strtok(trimmed, ",");
    if (target == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: lock,<1|2|3|shuttle>"));
        return false;
    }

    // Trim leading spaces from target
    target = trimLeadingSpaces(target);

    // Handle locking based on target
    if (strcmp(target, "all") == 0)
    {
        // Removed "lock all" functionality as requested
        caller->println(F("[ERROR] 'lock,all' is not supported for safety reasons. Engage trays individually."));
        return false;
    }
    else if (strcmp(target, "shuttle") == 0)
    {
        if (ccioBoardCount > 0)
        {
            caller->println(F("[MESSAGE] Engaging shuttle with sensor verification..."));
            DoubleSolenoidValve *valve = getShuttleValve();
            CylinderSensor *sensor = getShuttleSensor();
            
            if (valve && sensor)
            {
                // Check current state first
                if (valve->position == VALVE_POSITION_LOCK)
                {
                    caller->println(F("[MESSAGE] Shuttle already engaged"));
                    
                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == true) { // Sensor true = locked
                        caller->println(F("[OK] Shuttle lock confirmed by sensor"));
                    } else {
                        caller->println(F("[WARNING] Shuttle should be locked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try locking with sensor feedback
                    caller->println(F("[MESSAGE] Locking shuttle..."));
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000)) {
                        caller->println(F("[MESSAGE] Shuttle engaged and confirmed by sensor"));
                        return true;
                    } else {
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
        int trayNum = atoi(target);
        if (trayNum >= 1 && trayNum <= 3)
        {
            caller->print(F("[MESSAGE] Engaging tray "));
            caller->print(trayNum);
            caller->println(F(" with sensor verification..."));
            
            DoubleSolenoidValve *valve = NULL;
            CylinderSensor *sensor = NULL;
            
            // Get the appropriate valve and sensor
            switch (trayNum) {
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
                    caller->print(F("[MESSAGE] Tray "));
                    caller->print(trayNum);
                    caller->println(F(" already engaged"));
                    
                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == true) { // Sensor true = locked
                        caller->println(F("[OK] Tray lock confirmed by sensor"));
                    } else {
                        caller->println(F("[WARNING] Tray should be locked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try locking with sensor feedback
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000)) {
                        caller->print(F("[MESSAGE] Tray "));
                        caller->print(trayNum);
                        caller->println(F(" engaged and confirmed by sensor"));
                        return true;
                    } else {
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
        else
        {
            caller->println(F("[ERROR] Invalid tray number. Must be 1, 2, or 3."));
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
        caller->println(F("[ERROR] Missing parameter. Usage: unlock,<1|2|3|shuttle|all>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *target = strtok(trimmed, ",");
    if (target == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: unlock,<1|2|3|shuttle|all>"));
        return false;
    }

    // Trim leading spaces from target
    target = trimLeadingSpaces(target);

    // Handle unlocking based on target
    if (strcmp(target, "all") == 0)
    {
        caller->println(F("[MESSAGE] Disengaging all valves with sensor verification..."));
        if (safeUnlockAllValves(1000)) {
            caller->println(F("[MESSAGE] All valves successfully disengaged"));
            return true;
        } else {
            caller->println(F("[WARNING] Some valves could not be disengaged - check air pressure"));
            return false;
        }
    }
    else if (strcmp(target, "shuttle") == 0)
    {
        if (ccioBoardCount > 0)
        {
            caller->println(F("[MESSAGE] Disengaging shuttle with sensor verification..."));
            DoubleSolenoidValve *valve = getShuttleValve();
            CylinderSensor *sensor = getShuttleSensor();
            
            if (valve && sensor)
            {
                // Check current state first
                if (valve->position == VALVE_POSITION_UNLOCK)
                {
                    caller->println(F("[MESSAGE] Shuttle already disengaged"));
                    
                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == false) { // Sensor false = unlocked
                        caller->println(F("[OK] Shuttle unlock confirmed by sensor"));
                    } else {
                        caller->println(F("[WARNING] Shuttle should be unlocked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try unlocking with sensor feedback
                    caller->println(F("[MESSAGE] Unlocking shuttle..."));
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000)) {
                        caller->println(F("[MESSAGE] Shuttle disengaged and confirmed by sensor"));
                        return true;
                    } else {
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
        int trayNum = atoi(target);
        if (trayNum >= 1 && trayNum <= 3)
        {
            caller->print(F("[MESSAGE] Disengaging tray "));
            caller->print(trayNum);
            caller->println(F(" with sensor verification..."));
            
            DoubleSolenoidValve *valve = NULL;
            CylinderSensor *sensor = NULL;
            
            // Get the appropriate valve and sensor
            switch (trayNum) {
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
                    caller->print(F("[MESSAGE] Tray "));
                    caller->print(trayNum);
                    caller->println(F(" already disengaged"));
                    
                    // Verify actual position with sensor
                    if (sensorRead(*sensor) == false) { // Sensor false = unlocked
                        caller->println(F("[OK] Tray unlock confirmed by sensor"));
                    } else {
                        caller->println(F("[WARNING] Tray should be unlocked but sensor doesn't confirm - check air pressure"));
                    }
                    return true;
                }
                else
                {
                    // Try unlocking with sensor feedback
                    if (safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000)) {
                        caller->print(F("[MESSAGE] Tray "));
                        caller->print(trayNum);
                        caller->println(F(" disengaged and confirmed by sensor"));
                        return true;
                    } else {
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
        else
        {
            caller->println(F("[ERROR] Invalid tray number. Must be 1, 2, or 3."));
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
        caller->println(F("[ERROR] Missing parameter. Usage: log,<on[,interval]|off|now>"));
        return false;
    }

    // Parse the first argument - we'll use commas as separators
    char *action = strtok(trimmed, ",");
    if (action == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: log,<on[,interval]|off|now>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Handle based on action
    if (strcmp(action, "on") == 0)
    {
        // Check if an interval was provided
        char *intervalStr = strtok(NULL, ",");
        unsigned long interval = DEFAULT_LOG_INTERVAL;

        if (intervalStr != NULL)
        {
            // Parse the interval value
            intervalStr = trimLeadingSpaces(intervalStr);
            unsigned long parsedInterval = strtoul(intervalStr, NULL, 10);

            if (parsedInterval > 0)
            {
                interval = parsedInterval;
                caller->print(F("[MESSAGE] Logging enabled with interval of "));
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
            caller->print(F("[MESSAGE] Logging enabled with default interval of "));
            caller->print(DEFAULT_LOG_INTERVAL);
            caller->println(F(" ms"));
        }

        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset the timer
        return true;
    }
    else if (strcmp(action, "off") == 0)
    {
        caller->println(F("[MESSAGE] Logging disabled"));
        logging.logInterval = 0; // Setting to 0 disables logging
        return true;
    }
    else if (strcmp(action, "now") == 0)
    {
        caller->println(F("[MESSAGE] Logging system state now"));
        // Log immediately regardless of interval
        logSystemState(); // Changed: removed the parameter
        return true;
    }
    else
    {
        caller->println(F("[ERROR] Invalid log action. Use 'on', 'off', or 'now'."));
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
        caller->println(F("[ERROR] Missing parameter. Usage: motor,<init|status|clear|home|abort|stop>"));
        return false;
    }

    // Parse the argument - we'll use commas as separators
    char *action = strtok(trimmed, ",");
    if (action == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: motor,<init|status|clear|home|abort|stop>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Handle motor subcommands using switch for better readability
    int actionCode = 0;
    if (strcmp(action, "init") == 0)
        actionCode = 1;
    else if (strcmp(action, "status") == 0)
        actionCode = 2;
    else if (strcmp(action, "clear") == 0)
        actionCode = 3;
    else if (strcmp(action, "home") == 0)
        actionCode = 4;
    else if (strcmp(action, "abort") == 0)
        actionCode = 5;
    else if (strcmp(action, "stop") == 0)
        actionCode = 6;

    switch (actionCode)
    {
    case 1:
    { // init
        caller->println(F("[MESSAGE] Initializing motor..."));

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
            caller->println(F("[MESSAGE] Motor initialization successful"));
            return true;
        }
        break;
    }

    case 2:
    { // status
        caller->println(F("[MESSAGE] Motor Status:"));

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

        // Homing velocities - corrected to use the actual parameters
        caller->print(F("    Homing: "));
        caller->print(HOME_APPROACH_VELOCITY_RPM);
        caller->print(F(" RPM initial, "));
        caller->print(HOME_FINAL_VELOCITY_RPM);
        caller->println(F(" RPM final"));

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
        caller->println(F("[MESSAGE] Attempting to clear motor fault..."));

        if (clearMotorFaultWithStatus())
        {
            caller->println(F("[MESSAGE] Motor fault cleared successfully"));
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

        caller->println(F("[MESSAGE] Starting homing sequence..."));

        // Begin homing
        initiateHomingSequence();

        // Check if homing was initiated by examining the motor state
        if (motorState == MOTOR_STATE_HOMING)
        {
            caller->println(F("[MESSAGE] Homing sequence initiated. Motor will move to find home position."));
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

        caller->println(F("[MESSAGE] Aborting current operation..."));

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

            caller->println(F("[MESSAGE] Operation aborted successfully."));
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

        caller->println(F("[MESSAGE] EMERGENCY STOP initiated!"));

        // Execute emergency stop
        MOTOR_CONNECTOR.MoveStopAbrupt();
        motorState = MOTOR_STATE_IDLE;

        caller->println(F("[MESSAGE] Motor movement halted. Position may no longer be accurate."));
        caller->println(F("[WARNING] Re-homing recommended after emergency stop."));

        return true;
        break;
    }

    default:
    {
        caller->print(F("[ERROR] Unknown motor command: "));
        caller->println(action);
        caller->println(F("Valid options are 'init', 'status', 'clear', 'home', 'abort', or 'stop'"));
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
        caller->println(F("[ERROR] Missing parameter. Usage: move,<home|1|2|3|4|counts,X|mm,X|rel,X>"));
        return false;
    }

    // Get the first parameter (target position)
    char *target = strtok(trimmed, " ");
    if (target == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: move,<home|1|2|3|4|counts,X|mm,X|rel,X>"));
        return false;
    }

    // Trim leading spaces from target
    target = trimLeadingSpaces(target);

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
    if (strcmp(target, "home") == 0)
    {
        // Check if motor is already homed
        if (isHomed)
        {
            caller->println(F("[MESSAGE] Moving to home position..."));
            if (moveToPositionMm(0.0))
            {
                caller->println(F("[MESSAGE] Move to home initiated."));
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
    else if (strcmp(target, "1") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[MESSAGE] Moving to position 1..."));
        if (moveToPosition(POSITION_1))
        {
            caller->println(F("[MESSAGE] Move to position 1 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 1."));
            return false;
        }
    }
    else if (strcmp(target, "2") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[MESSAGE] Moving to position 2..."));
        if (moveToPosition(POSITION_2))
        {
            caller->println(F("[MESSAGE] Move to position 2 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 2."));
            return false;
        }
    }
    else if (strcmp(target, "3") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[MESSAGE] Moving to position 3..."));
        if (moveToPosition(POSITION_3))
        {
            caller->println(F("[MESSAGE] Move to position 3 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 3."));
            return false;
        }
    }
    else if (strcmp(target, "4") == 0)
    {
        // Check if motor is homed
        if (!isHomed)
        {
            caller->println(F("[ERROR] Motor is not homed. Use 'motor,home' command first."));
            return false;
        }

        caller->println(F("[MESSAGE] Moving to position 4..."));
        if (moveToPosition(POSITION_4))
        {
            caller->println(F("[MESSAGE] Move to position 4 initiated."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to position 4."));
            return false;
        }
    }

    // Special handling for "mm" command for absolute positioning in millimeters
    else if (strcmp(target, "mm") == 0)
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
        caller->print(F("[MESSAGE] Moving to absolute position: "));
        caller->print(targetMm, 2);
        caller->println(F(" mm"));

        if (moveToPositionMm(targetMm))
        {
            caller->println(F("[MESSAGE] Movement initiated successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to requested position."));
            return false;
        }
    }

    // Special handling for "counts" command for absolute positioning in encoder counts
    else if (strcmp(target, "counts") == 0)
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
        caller->print(F("[MESSAGE] Moving to absolute position: "));
        caller->print(targetCounts);
        caller->println(F(" counts"));

        if (moveToAbsolutePosition(targetCounts))
        {
            caller->println(F("[MESSAGE] Movement initiated successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start movement to requested position."));
            return false;
        }
    }

    // Special handling for "rel" command for relative positioning
    else if (strcmp(target, "rel") == 0)
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
        caller->print(F("[MESSAGE] Moving "));
        caller->print(relDistanceMm, 2);
        caller->print(F(" mm from current position ("));
        caller->print(currentPositionMm, 2);
        caller->print(F(" mm) to "));
        caller->print(targetPositionMm, 2);
        caller->println(F(" mm"));

        if (moveToPositionMm(targetPositionMm))
        {
            caller->println(F("[MESSAGE] Relative movement initiated successfully."));
            return true;
        }
        else
        {
            caller->println(F("[ERROR] Failed to start relative movement."));
            return false;
        }
    }
    else
    {
        caller->print(F("[ERROR] Invalid position: "));
        caller->println(target);
        caller->println(F("Valid options: home, 1, 2, 3, 4, counts, mm, rel"));
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
        caller->println(F("[ERROR] Missing parameter. Usage: jog,<+|-|inc|speed|status>"));
        return false;
    }

    // Parse the argument - we'll use spaces as separators (as they've been converted from commas)
    char *action = strtok(trimmed, " ");
    if (action == NULL)
    {
        caller->println(F("[ERROR] Invalid format. Usage: jog,<+|-|inc|speed|status>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // State checks (in order of importance) for movement commands
    if (strcmp(action, "+") == 0 || strcmp(action, "-") == 0)
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
    int actionCode = 0;
    if (strcmp(action, "+") == 0)
        actionCode = 1;
    else if (strcmp(action, "-") == 0)
        actionCode = 2;
    else if (strcmp(action, "inc") == 0)
        actionCode = 3;
    else if (strcmp(action, "speed") == 0)
        actionCode = 4;
    else if (strcmp(action, "status") == 0)
        actionCode = 5;

    switch (actionCode)
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
        caller->print(F("[MESSAGE] Jogging forward "));
        caller->print(currentJogIncrementMm, 2);
        caller->print(F(" mm from position "));
        caller->print(currentPositionMm, 2);
        caller->print(F(" mm to "));
        caller->print(targetPositionMm, 2);
        caller->println(F(" mm"));

        // Perform the jog movement using the jogMotor function
        if (jogMotor(true))
        { // true = forward direction
            caller->println(F("[MESSAGE] Jog movement initiated"));
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
        caller->print(F("[MESSAGE] Jogging backward "));
        caller->print(currentJogIncrementMm, 2);
        caller->print(F(" mm from position "));
        caller->print(currentPositionMm, 2);
        caller->print(F(" mm to "));
        caller->print(targetPositionMm, 2);
        caller->println(F(" mm"));

        // Perform the jog movement using the jogMotor function
        if (jogMotor(false))
        { // false = backward direction
            caller->println(F("[MESSAGE] Jog movement initiated"));
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
            caller->print(F("[MESSAGE] Current jog increment: "));
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
                    caller->print(F("[MESSAGE] Jog increment set to default ("));
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
                caller->print(F("[MESSAGE] Jog increment set to "));
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
            caller->print(F("[MESSAGE] Current jog speed: "));
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
                if (setJogSpeed(DEFAULT_JOG_SPEED))
                {
                    caller->print(F("[MESSAGE] Jog speed set to default ("));
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
            if (setJogSpeed(newSpeed))
            {
                caller->print(F("[MESSAGE] Jog speed set to "));
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
        caller->println(F("[MESSAGE] Current jog settings:"));

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

    default:
    {
        caller->print(F("[ERROR] Unknown jog command: "));
        caller->println(action);
        caller->println(F("Valid options are '+', '-', 'inc', 'speed', or 'status'"));
        return false;
        break;
    }
    }

    return false; // Should never reach here, but included for completeness
}

bool cmd_system_state(char *args, CommandCaller *caller) {
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Determine the subcommand
    char *subcommand = strtok(trimmed, " ");
    
    // If no subcommand provided, display usage
    if (subcommand == NULL || strlen(subcommand) == 0) {
        caller->println(F("[MESSAGE] Usage: system,state - Display current system state"));
        caller->println(F("                 system,safety - Display safety validation status"));
        caller->println(F("                 system,trays - Display tray system status"));
        return false;
    }
    
    // Handle subcommands
    if (strcmp(subcommand, "state") == 0) {
        // Capture and print system state
        SystemState currentState = captureSystemState();
        printSystemState(currentState, caller);
        return true;
    }
    // Add this new section to display safety validation
    else if (strcmp(subcommand, "safety") == 0) {
        // Capture system state, validate safety, and print results
        SystemState currentState = captureSystemState();
        SafetyValidationResult safety = validateSafety(currentState);
        
        caller->println(F("\n===== SAFETY VALIDATION STATUS ====="));
        printSafetyStatus(safety, caller);
        
        return true;
    }
    else if (strcmp(subcommand, "trays") == 0) {
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
        
        if (trayTracking.lastLoadTime > 0) {
            caller->print(F("  Last load: "));
            caller->print((millis() - trayTracking.lastLoadTime) / 1000);
            caller->println(F(" seconds ago"));
        }
        
        if (trayTracking.lastUnloadTime > 0) {
            caller->print(F("  Last unload: "));
            caller->print((millis() - trayTracking.lastUnloadTime) / 1000);
            caller->println(F(" seconds ago"));
        }
        
        return true;
    }
    else {
        // Unknown subcommand
        caller->print(F("[ERROR] Unknown system command: "));
        caller->println(subcommand);
        caller->println(F("Valid options are 'system,state', 'system,safety', or 'system,trays'"));
        return false;
    }
}

// Tray command handler
bool cmd_tray(char *args, CommandCaller *caller) {
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);
    
    // Check for empty argument
    if (strlen(trimmed) == 0) {
        caller->println(F("[ERROR] Missing parameter. Usage: tray,<request|placed|released|status>"));
        return false;
    }

    // Parse the subcommand
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL) {
        caller->println(F("[ERROR] Invalid format. Usage: tray,<request|placed|released|status>"));
        return false;
    }
    
    // Skip leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);
    
    // Handle the different subcommands
    if (strcmp(subcommand, "request") == 0) {
        // Mitsubishi robot is requesting to load a tray
        
        // 1. Check if the system can accept a tray
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);
        
        // 2. Verify position 1 is free and no operations are in progress
        if (trayTracking.position1Occupied) {
            caller->println(F("POSITION_OCCUPIED"));
            return false;
        }
        
        if (operationInProgress) {
            caller->println(F("SYSTEM_BUSY"));
            return false;
        }
        
        // 3. Validate safety constraints
        SafetyValidationResult safety = validateSafety(state);
        if (!safety.safeToLoadTrayToPos1) {
            caller->print(F("UNSAFE: "));
            caller->println(safety.loadTrayPos1UnsafeReason);
            return false;
        }
        
        // 4. Set the target position for position 1
        if (!moveToPositionMm(POSITION_1_MM)) {
            caller->println(F("ERROR_MOVE_FAILURE"));
            return false;
        }
        
        // 5. System is ready to receive tray
        caller->println(F("READY_TO_RECEIVE"));
        return true;
    }
    else if (strcmp(subcommand, "placed") == 0) {
        // Mitsubishi robot has placed the tray
        
        // 1. Verify tray sensor shows tray is present
        SystemState state = captureSystemState();
        if (!state.tray1Present) {
            caller->println(F("ERROR_NO_TRAY_DETECTED"));
            return false;
        }
        
        // 2. Lock the tray in position
        DoubleSolenoidValve *valve = getTray1Valve(); // Tray 1 valve
        CylinderSensor *sensor = getTray1Sensor();    // Add this line to get the sensor

        if (valve && sensor) {
            // Check current state first
            if (valve->position != VALVE_POSITION_LOCK) {
                if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000)) {
                    caller->println(F("[ERROR] Failed to lock tray - sensor didn't confirm"));
                    caller->println(F("[WARNING] Check air pressure and valve functionality"));
                    return false;
                }
                caller->println(F("TRAY_SECURED"));
                
                // Update tray tracking
                addTrayAtPosition1();
                return true;
            }
            else {
                // Valve already in lock position
                caller->println(F("TRAY_ALREADY_SECURED"));
                addTrayAtPosition1();
                return true;
            }
        }
        else {
            caller->println(F("ERROR_LOCK_FAILURE"));
            return false;
        }
    }
    else if (strcmp(subcommand, "released") == 0) {
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
    else if (strcmp(subcommand, "status") == 0) {
        // Return system status in machine-readable format
        SystemState state = captureSystemState();
        updateTrayTrackingFromSensors(state);
        
        // Format: STATUS:P1:[0/1],P2:[0/1],P3:[0/1],OP:[0/1]
        // Where P1/P2/P3 are positions, OP is operation in progress
        caller->print(F("STATUS:P1:"));
        caller->print(trayTracking.position1Occupied ? "1" : "0");
        caller->print(F(",P2:"));
        caller->print(trayTracking.position2Occupied ? "1" : "0");
        caller->print(F(",P3:"));
        caller->print(trayTracking.position3Occupied ? "1" : "0");
        caller->print(F(",OP:"));
        caller->println(operationInProgress ? "1" : "0");
        
        return true;
    }
    else {
        caller->print(F("[ERROR] Unknown tray command: "));
        caller->println(subcommand);
        caller->println(F("Valid options are 'request', 'placed', 'released', or 'status'"));
        return false;
    }
}

bool cmd_test(char *args, CommandCaller *caller) {
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0) {
        caller->println(F("Available tests:"));
        caller->println(F("  home     - Test homing repeatability"));
        caller->println(F("  position - Test position cycling (for tray loading)"));
        caller->println(F("  tray     - Test complete tray handling operations"));
        caller->println(F("Usage: test,<test_name>"));
        return true;
    }

    // Parse the first argument - we'll use spaces as separators (commas converted to spaces)
    char *testType = strtok(trimmed, " ");
    if (testType == NULL) {
        caller->println(F("[ERROR] Invalid format. Usage: test,<home|position|tray>"));
        return false;
    }

    // Trim leading spaces from test type
    testType = trimLeadingSpaces(testType);
    
    // Check motor initialization first
    if (motorState == MOTOR_STATE_NOT_READY) {
        caller->println(F("[ERROR] Motor not initialized. Run 'motor,init' first."));
        return false;
    }

    // Check E-Stop condition
    if (isEStopActive()) {
        caller->println(F("[ERROR] Cannot run tests while E-Stop is active."));
        return false;
    }
    
    // Run the appropriate test
    if (strcmp(testType, "home") == 0) {
        caller->println(F("[MESSAGE] Starting homing repeatability test..."));
        if (testHomingRepeatability()) {
            caller->println(F("[MESSAGE] Homing repeatability test completed successfully."));
            return true;
        } else {
            caller->println(F("[ERROR] Homing repeatability test failed or was aborted."));
            return false;
        }
    } 
    else if (strcmp(testType, "position") == 0) {
        caller->println(F("[MESSAGE] Starting position cycling test..."));
        
        if (testPositionCycling()) {
            caller->println(F("[MESSAGE] Position cycling test completed successfully."));
            return true;
        } else {
            caller->println(F("[ERROR] Position cycling test failed or was aborted."));
            return false;
        }
    }
    else if (strcmp(testType, "tray") == 0) {
        caller->println(F("[MESSAGE] Starting tray handling test..."));
        
        if (testTrayHandling()) {
            caller->println(F("[MESSAGE] Tray handling test completed successfully."));
            return true;
        } else {
            caller->println(F("[ERROR] Tray handling test failed or was aborted."));
            return false;
        }
    }
    else {
        caller->print(F("[ERROR] Unknown test type: "));
        caller->println(testType);
        caller->println(F("Available tests: 'home', 'position', 'tray'"));
        return false;
    }
}

bool cmd_encoder(char *args, CommandCaller *caller) {
    // Create a local copy of arguments
    char localArgs[COMMAND_SIZE];
    strncpy(localArgs, args, COMMAND_SIZE);
    localArgs[COMMAND_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0) {
        // Display current encoder status
        caller->println(F("[MESSAGE] MPG Handwheel Controls:"));
        caller->println(F("  encoder,enable          - Enable MPG handwheel control"));
        caller->println(F("  encoder,disable         - Disable MPG handwheel control"));
        caller->println(F("  encoder,multiplier,[1|10|100] - Set movement multiplier"));

        // Add the setup sequence here where users will actually see it
        caller->println(F("\n[SETUP SEQUENCE]"));
        caller->println(F("  1. 'motor,init' - Initialize the motor"));
        caller->println(F("  2. 'motor,home' - Establish a reference position"));
        caller->println(F("  3. 'encoder,enable' - Activate handwheel control"));
        caller->println(F("  4. 'encoder,multiplier,[1|10|100]' - Set step multiplier"));
        
        // Show current status
        if (encoderControlActive) {
            caller->println(F("\n[STATUS] MPG control is currently ENABLED"));
            caller->print(F("[STATUS] Current multiplier: x"));
            caller->println(currentMultiplier);
            
            // Show position information if motor is homed
            if (isHomed) {
                double positionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
                caller->print(F("[STATUS] Current position: "));
                caller->print(positionMm, 2);
                caller->println(F(" mm"));
            }
        } else {
            caller->println(F("\n[STATUS] MPG control is currently DISABLED"));
            
            // Show reasons why encoder control might not be available
            if (!motorInitialized) {
                caller->println(F("[NOTE] Motor needs to be initialized first (motor,init)"));
            } else if (!isHomed) {
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
    for (int i = 0; trimmed[i] != '\0'; i++) {
        if (trimmed[i] == ',') {
            trimmed[i] = ' ';
        }
    }

    // Get the subcommand
    char *subcommand = strtok(trimmed, " ");
    if (subcommand == NULL) {
        caller->println(F("[ERROR] Invalid format. Usage: encoder,<enable|disable|multiplier>"));
        return false;
    }

    // Trim leading spaces from subcommand
    subcommand = trimLeadingSpaces(subcommand);

    // Handle commands
    if (strcmp(subcommand, "enable") == 0) {
        // Check preconditions
        if (!motorInitialized) {
            caller->println(F("[ERROR] Motor must be initialized before enabling MPG control"));
            caller->println(F("[MESSAGE] Use 'motor,init' first"));
            return false;
        }
        
        if (!isHomed) {
            caller->println(F("[ERROR] Motor must be homed before enabling MPG control"));
            caller->println(F("[MESSAGE] Use 'motor,home' to establish a reference position"));
            return false;
        }
        
        if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING) {
            caller->println(F("[ERROR] Cannot enable MPG control while motor is moving"));
            caller->println(F("[MESSAGE] Wait for current movement to complete or use 'motor,abort'"));
            return false;
        }
        
        if (motorState == MOTOR_STATE_FAULTED) {
            caller->println(F("[ERROR] Cannot enable MPG control while motor is in fault state"));
            caller->println(F("[MESSAGE] Use 'motor,clear' to clear fault first"));
            return false;
        }
        
        if (isEStopActive()) {
            caller->println(F("[ERROR] Cannot enable MPG control while E-Stop is active"));
            return false;
        }
        
        // Enable encoder control
        encoderControlActive = true;
        
        // Reset encoder position
        EncoderIn.Position(0);
        lastEncoderPosition = 0;
        lastEncoderUpdateTime = millis();
        
        caller->print(F("[MESSAGE] MPG handwheel control enabled - current position: "));
        caller->print(pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()), 2);
        caller->println(F(" mm"));
        caller->print(F("[MESSAGE] Using multiplier x"));
        caller->print(getMultiplierName(currentMultiplier));
        caller->print(F(" ("));
        caller->print(currentMultiplier);
        caller->println(F(")"));
        caller->println(F("[MESSAGE] Issue 'encoder,disable' when finished with manual control"));
        
        return true;
    }
    else if (strcmp(subcommand, "disable") == 0) {
        // Disable encoder control
        encoderControlActive = false;
        caller->println(F("[MESSAGE] MPG handwheel control disabled"));
        return true;
    }
    else if (strcmp(subcommand, "multiplier") == 0) {
        // Look for the NEXT argument - this is the same approach used in move,mm,X
        char* originalArgs = args; // Save the original args string
        
        // Find the "multiplier" substring within args
        char* multiplierPos = strstr(originalArgs, "multiplier");
        if (multiplierPos != NULL) {
            // Move past "multiplier"
            multiplierPos += strlen("multiplier");
            
            // Skip any spaces or commas
            while (*multiplierPos && (*multiplierPos == ' ' || *multiplierPos == ',')) {
                multiplierPos++;
            }
            
            // Now multiplierPos should point to the actual value
            if (*multiplierPos) {
                // Parse the actual multiplier value
                int multiplier = atoi(multiplierPos);
                
                // Set the multiplier based on the input value
                switch (multiplier) {
                    case 1:
                        setEncoderMultiplier(1);
                        caller->println(F("[MESSAGE] Multiplier set to x1 (fine adjustment)"));
                        break;
                    case 10:
                        setEncoderMultiplier(10);
                        caller->println(F("[MESSAGE] Multiplier set to x10 (medium adjustment)"));
                        break;
                    case 100:
                        setEncoderMultiplier(100);
                        caller->println(F("[MESSAGE] Multiplier set to x100 (coarse adjustment)"));
                        break;
                    default:
                        caller->println(F("[ERROR] Invalid multiplier. Use 1, 10, or 100."));
                        return false;
                }
                
                // Show current multiplier and effect
                caller->print(F("[MESSAGE] Current multiplier value: "));
                caller->println(currentMultiplier);
                double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
                caller->print(F("[MESSAGE] One full rotation moves ~"));
                caller->print(mmPerRotation, 2);
                caller->println(F(" mm"));
                return true;
            }
        }
        
        // If we get here, display the current multiplier
        caller->print(F("[MESSAGE] Current multiplier: x"));
        caller->print(getMultiplierName(currentMultiplier));
        caller->print(F(" ("));
        caller->print(currentMultiplier);
        caller->println(F(")"));
        
        // Show what one full rotation will move
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        caller->print(F("[MESSAGE] One full rotation moves ~"));
        caller->print(mmPerRotation, 2);
        caller->println(F(" mm"));
        
        return true;
    }
    else if (strcmp(subcommand, "help") == 0) {
        caller->println(F("[MESSAGE] MPG Handwheel Setup & Usage:"));
        caller->println(F("\n[SETUP SEQUENCE]"));
        caller->println(F("  1. 'motor,init' - Initialize the motor"));
        caller->println(F("  2. 'motor,home' - Establish a reference position"));
        caller->println(F("  3. 'encoder,enable' - Activate handwheel control"));
    }
    
    return false;
}

Commander commander;

Commander::systemCommand_t API_tree[] = {
    systemCommand("help", "Display help information for all commands", cmd_print_help),
    systemCommand("h", "Display help information for all commands", cmd_print_help),
    systemCommand("H", "Display help information for all commands", cmd_print_help),

    // Unified lock/unlock commands
    systemCommand("lock", "Lock a tray or shuttle (usage: lock,1 or lock,2 or lock,3 or lock,shuttle)", cmd_lock),
    systemCommand("unlock", "Unlock a tray, shuttle, or all valves (usage: unlock,1 or unlock,shuttle or unlock,all)", cmd_unlock),

    // Logging command
    systemCommand("log", "Logging controls (usage: log,on[,interval] or log,off or log,now)", cmd_log),

    // NEW: State command to display system state
    systemCommand("system", "System commands:\n"
                            "  system,state  - Display current system state (sensors, actuators, positions)\n"
                            "  system,safety - Display comprehensive safety validation status\n"
                            "  system,trays  - Display tray tracking and statistics",
                 cmd_system_state),

    // Motor control commands
    systemCommand("motor", "Motor control:\n"
                           "  motor,init   - Initialize motor system and prepare for operation\n"
                           "  motor,status - Display detailed motor status and configuration\n"
                           "  motor,clear  - Clear motor fault condition to restore operation\n"
                           "  motor,home   - Home the motor (find zero position)\n"
                           "  motor,abort  - Abort current operation gracefully\n"
                           "  motor,stop   - Emergency stop motor movement immediately",
                  cmd_motor),

    // Move command
    systemCommand("move", "Move motor to position:\n"
                          "  move,home      - Move to home (zero) position\n"
                          "  move,1..4      - Move to predefined positions 1 through 4\n"
                          "  move,counts,X  - Move to absolute position X in encoder counts (0-66917)\n"
                          "  move,mm,X      - Move to absolute position X in millimeters (0-1092.2)\n"
                          "  move,rel,X     - Move X millimeters relative to current position (+ forward, - backward)",
                  cmd_move),

    // Jog command
    systemCommand("jog", "Jog motor:\n"
                         "  jog,+         - Jog forward by current increment\n"
                         "  jog,-         - Jog backward by current increment\n"
                         "  jog,inc,X     - Get or set jog increment (X in mm or 'default')\n"
                         "  jog,speed,X   - Get or set jog speed (X in RPM or 'default')\n"
                         "  jog,status    - Display current jog settings",
                  cmd_jog),

    // Tray command
    systemCommand("tray", "Tray operations:\n"
                          "  tray,request   - Request to load a tray (Mitsubishi)\n"
                          "  tray,placed    - Notify tray has been placed (Mitsubishi)\n"
                          "  tray,released  - Notify tray has been released (Mitsubishi)\n"
                          "  tray,status    - Get tray system status (machine-readable)",
                  cmd_tray),

    // Test command
    systemCommand("test", "Run tests on the system:\n"
                     "  test,home     - Run homing repeatability test\n"
                     "  test,position - Run position cycling test for tray loading\n"
                     "  test,tray     - Run tray handling test (request, place, release)",
             cmd_test),
            
    // Encoder control commands
    systemCommand("encoder", "Encoder handwheel control:\n"
                         "  encoder,enable  - Enable encoder control\n"
                         "  encoder,disable - Disable encoder control\n"
                         "  encoder,multiplier,X - Set encoder multiplier (X = 1, 10, or 100)\n"
                         "  encoder,help    - Display setup instructions and usage tips",
              cmd_encoder),
            };
