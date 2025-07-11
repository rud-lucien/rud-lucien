#include "Utils.h"

// Define variables that were declared in Utils.h
// Position tracking
double commandedPositionMm = -1; // Initialize to -1

// Tray tracking structure
TrayTracking trayTracking = {
    .totalTraysInSystem = 0,
    .position1Occupied = false,
    .position2Occupied = false,
    .position3Occupied = false,
    .lastLoadTime = 0,
    .lastUnloadTime = 0,
    .totalLoadsCompleted = 0,
    .totalUnloadsCompleted = 0};

// Operation state tracking
bool operationInProgress = false;
bool newCommandReceived = false;
unsigned long operationStartTime = 0;
unsigned long operationTimeoutMs = 60000; // 60 seconds default
int currentOperationStep = 0;
int expectedOperationStep = 0;
bool operationEncoderState = false;

// State machine timing variables and substep tracking
unsigned long valveActuationStartTime = 0;
unsigned long safetyDelayStartTime = 0;
unsigned long sensorVerificationStartTime = 0;

// Valve actuation and safety delay constants
const unsigned long VALVE_ACTUATION_TIME_MS = 500;         // Increased from 500ms for more reliable actuation (previosly 750ms)
const unsigned long SAFETY_DELAY_AFTER_UNLOCK_MS = 500;    // Safety delay after unlocking a tray (previously 1000ms)
const unsigned long SAFETY_DELAY_BEFORE_MOVEMENT_MS = 500; // Safety delay before starting motor movement (previously 1000ms)
const unsigned long SAFETY_DELAY_AFTER_MOVEMENT_MS = 500;  // Safety delay after motor has completed movement (previously 1000ms)
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 200;    // Delay for stable sensor readings

// Tray status structure
TrayStatus trayStatus = {false, false, false, 0, OPERATION_NONE};

// Current operation structure
OperationStatus currentOperation = {false, OPERATION_NONE, 0, 0, false, ""};

// Add this variable definition near your other global variables:
SystemState previousState;

// Add these near your other global variables
bool lastLockOperationFailed = false;
bool lastUnlockOperationFailed = false;
char lastLockFailureDetails[128] = "";
char lastUnlockFailureDetails[128] = "";
unsigned long lockFailureTimestamp = 0;
unsigned long unlockFailureTimestamp = 0;

// Safe time difference calculation that handles rollover
unsigned long timeDiff(unsigned long current, unsigned long previous)
{
    return current - previous; // Correctly handles rollover with unsigned arithmetic
}

// Safe timeout check that handles rollover
bool timeoutElapsed(unsigned long current, unsigned long previous, unsigned long timeout)
{
    return timeDiff(current, previous) >= timeout;
}

// Safe waiting check
bool waitTimeReached(unsigned long current, unsigned long previous, unsigned long waitTime)
{
    return timeDiff(current, previous) >= waitTime;
}

// Helper function to print time in a human-readable format
void printHumanReadableTime(unsigned long secondsAgo)
{
    char timeMsg[80];

    if (secondsAgo < 60)
    {
        // Less than a minute
        sprintf(timeMsg, "%lu second%s", secondsAgo, (secondsAgo != 1) ? "s" : "");
    }
    else if (secondsAgo < 3600)
    {
        // Less than an hour
        unsigned long minutes = secondsAgo / 60;
        unsigned long seconds = secondsAgo % 60;
        if (seconds > 0)
        {
            sprintf(timeMsg, "%lu minute%s %lu second%s",
                    minutes, (minutes != 1) ? "s" : "",
                    seconds, (seconds != 1) ? "s" : "");
        }
        else
        {
            sprintf(timeMsg, "%lu minute%s", minutes, (minutes != 1) ? "s" : "");
        }
    }
    else if (secondsAgo < 86400)
    {
        // Less than a day
        unsigned long hours = secondsAgo / 3600;
        unsigned long minutes = (secondsAgo % 3600) / 60;
        if (minutes > 0)
        {
            sprintf(timeMsg, "%lu hour%s %lu minute%s",
                    hours, (hours != 1) ? "s" : "",
                    minutes, (minutes != 1) ? "s" : "");
        }
        else
        {
            sprintf(timeMsg, "%lu hour%s", hours, (hours != 1) ? "s" : "");
        }
    }
    else
    {
        // More than a day
        unsigned long days = secondsAgo / 86400;
        unsigned long hours = (secondsAgo % 86400) / 3600;
        if (hours > 0)
        {
            sprintf(timeMsg, "%lu day%s %lu hour%s",
                    days, (days != 1) ? "s" : "",
                    hours, (hours != 1) ? "s" : "");
        }
        else
        {
            sprintf(timeMsg, "%lu day%s", days, (days != 1) ? "s" : "");
        }
    }

    Console.print(timeMsg);
}

// Formats time as hours:minutes:seconds since system startup
void formatAbsoluteTime(unsigned long timeMs, char *buffer)
{
    // Calculate hours, minutes, seconds from milliseconds
    unsigned long totalSeconds = timeMs / 1000;
    unsigned long hours = totalSeconds / 3600;
    unsigned long minutes = (totalSeconds % 3600) / 60;
    unsigned long seconds = totalSeconds % 60;

    // Format as HH:MM:SS
    snprintf(buffer, 12, "%02lu:%02lu:%02lu", hours, minutes, seconds);
}

// Generic tray movement function - core implementation
bool moveTray(int fromPosition, int toPosition)
{
    bool *sourceOccupied = nullptr;
    bool *targetOccupied = nullptr;

    // Map position numbers to tracking variables
    switch (fromPosition)
    {
    case 1:
        sourceOccupied = &trayTracking.position1Occupied;
        break;
    case 2:
        sourceOccupied = &trayTracking.position2Occupied;
        break;
    case 3:
        sourceOccupied = &trayTracking.position3Occupied;
        break;
    default:
        return false;
    }

    switch (toPosition)
    {
    case 1:
        targetOccupied = &trayTracking.position1Occupied;
        break;
    case 2:
        targetOccupied = &trayTracking.position2Occupied;
        break;
    case 3:
        targetOccupied = &trayTracking.position3Occupied;
        break;
    default:
        return false;
    }

    // Perform the move if conditions are met
    if (*sourceOccupied && !*targetOccupied)
    {
        *sourceOccupied = false;
        *targetOccupied = true;
        return true;
    }

    return false; // Cannot move tray
}

// Load first tray (moves from position 1 to position 3)
bool loadFirstTray()
{
    Console.serialInfo(F("Moving first tray from position 1 to position 3"));

    // Increment counter regardless - this is a successful loading operation
    trayTracking.totalLoadsCompleted++;

    // The move itself may have already happened physically
    bool moveResult = moveTray(1, 3);

    // If the move wasn't recorded, make sure the tray is tracked
    if (!moveResult)
    {
        // Only update system count if it was actually a new tray
        SystemState state = captureSystemState();
        if (state.tray3Present && !trayTracking.position3Occupied)
        {
            trayTracking.position1Occupied = false;
            trayTracking.position3Occupied = true;
            trayTracking.totalTraysInSystem++;
        }
    }
    else
    {
        trayTracking.totalTraysInSystem++;
    }

    return true; // Operation was successful even if tracking update wasn't needed
}

// Load second tray (moves from position 1 to position 2)
bool loadSecondTray()
{
    Console.serialInfo(F("Moving second tray from position 1 to position 2"));

    // Increment counter regardless - this is a successful loading operation
    trayTracking.totalLoadsCompleted++;

    // The move itself may have already happened physically
    bool moveResult = moveTray(1, 2);

    // If the move wasn't recorded, make sure the tray is tracked
    if (!moveResult)
    {
        // Only update system count if it was actually a new tray
        SystemState state = captureSystemState();
        if (state.tray2Present && !trayTracking.position2Occupied)
        {
            trayTracking.position1Occupied = false;
            trayTracking.position2Occupied = true;
            trayTracking.totalTraysInSystem++;
        }
    }
    else
    {
        trayTracking.totalTraysInSystem++;
    }

    return true; // Operation was successful even if tracking update wasn't needed
}

// Load third tray (stays at position 1)
bool loadThirdTray()
{
    Console.serialInfo(F("Third tray remains at position 1"));

    // Always increment counters - this is a successful loading operation
    trayTracking.totalLoadsCompleted++;

    // Only increment tray count if position 1 wasn't already occupied
    SystemState state = captureSystemState();
    if (state.tray1Present && !trayTracking.position1Occupied)
    {
        trayTracking.position1Occupied = true;
        trayTracking.totalTraysInSystem++;
    }

    return true; // Operation was successful
}

// Unload first tray (directly from position 1)
bool unloadFirstTray()
{
    Console.serialInfo(F("Unloading tray from position 1"));
    if (trayTracking.position1Occupied)
    {
        trayTracking.position1Occupied = false;
        trayTracking.totalTraysInSystem--;
        trayTracking.lastUnloadTime = millis();
        return true;
    }
    return false;
}

// Unload second tray (moves from position 2 to position 1)
bool unloadSecondTray()
{
    Console.serialInfo(F("Moving tray from position 2 to position 1 for unloading"));
    return moveTray(2, 1);
}

// Unload third tray (moves from position 3 to position 1)
bool unloadThirdTray()
{
    Console.serialInfo(F("Moving tray from position 3 to position 1 for unloading"));
    return moveTray(3, 1);
}

// Determine which loading workflow to use based on system state
int determineLoadingWorkflow()
{
    if (!trayTracking.position3Occupied)
        return 1; // First tray - goes to position 3
    else if (!trayTracking.position2Occupied)
        return 2; // Second tray - goes to position 2
    else
        return 3; // Third tray - stays at position 1
}

// Determine which unloading workflow to use based on system state
int determineUnloadingWorkflow()
{
    if (trayTracking.position1Occupied)
        return 1; // Unload from position 1
    else if (trayTracking.position2Occupied)
        return 2; // Move from position 2 to 1, then unload
    else if (trayTracking.position3Occupied)
        return 3; // Move from position 3 to 1, then unload
    else
        return 0; // No trays to unload
}

// Update tray tracking based on sensor readings
void updateTrayTrackingFromSensors(const SystemState &state)
{
    // This should sync our tracking with physical sensor readings
    // Could be called periodically to correct any tracking errors

    // Check for inconsistency between tracking and sensors
    if (trayTracking.position1Occupied != state.tray1Present ||
        trayTracking.position2Occupied != state.tray2Present ||
        trayTracking.position3Occupied != state.tray3Present)
    {

        // Update tracking to match sensors
        trayTracking.position1Occupied = state.tray1Present;
        trayTracking.position2Occupied = state.tray2Present;
        trayTracking.position3Occupied = state.tray3Present;

        // Recalculate total trays
        trayTracking.totalTraysInSystem =
            (state.tray1Present ? 1 : 0) +
            (state.tray2Present ? 1 : 0) +
            (state.tray3Present ? 1 : 0);
    }
}

// Capture the current system state
SystemState captureSystemState()
{
    SystemState state;

    // Capture motor state
    state.motorState = motorState;
    state.isHomed = isHomed;
    state.currentPositionMm = currentPositionMm;
    state.hlfbStatus = MOTOR_CONNECTOR.HlfbState();

    // Capture cylinder sensor states
    state.tray1CylinderActivated = sensorRead(*getTray1Sensor());
    state.tray2CylinderActivated = sensorRead(*getTray2Sensor());
    state.tray3CylinderActivated = sensorRead(*getTray3Sensor());
    state.shuttleCylinderActivated = sensorRead(*getShuttleSensor());

    // Determine lock states from sensors
    // INVERTED LOGIC: Sensors are ACTIVATED when UNLOCKED
    state.tray1Locked = !state.tray1CylinderActivated;
    state.tray2Locked = !state.tray2CylinderActivated;
    state.tray3Locked = !state.tray3CylinderActivated;
    state.shuttleLocked = !state.shuttleCylinderActivated;

    // Capture tray presence
    state.tray1Present = sensorRead(*getTray1DetectionSensor());
    state.tray2Present = sensorRead(*getTray2DetectionSensor());
    state.tray3Present = sensorRead(*getTray3DetectionSensor());

    // Capture E-stop status
    state.eStopActive = isEStopActive();

    // Capture hardware status
    state.ccioBoardPresent = hasCCIO;

    // Capture tray tracking state
    state.totalTraysInSystem = trayTracking.totalTraysInSystem;
    state.position1Occupied = trayTracking.position1Occupied;
    state.position2Occupied = trayTracking.position2Occupied;
    state.position3Occupied = trayTracking.position3Occupied;

    return state;
}

// Print the current system state for diagnostics
void printSystemState(const SystemState &state)
{
    char msg[200];

    Console.println(F("[DIAGNOSTIC] System State:"));

    // Motor
    const char *motorStateStr;
    switch (state.motorState)
    {
    case MOTOR_STATE_IDLE:
        motorStateStr = "IDLE";
        break;
    case MOTOR_STATE_MOVING:
        motorStateStr = "MOVING";
        break;
    case MOTOR_STATE_HOMING:
        motorStateStr = "HOMING";
        break;
    case MOTOR_STATE_FAULTED:
        motorStateStr = "FAULTED";
        break;
    case MOTOR_STATE_NOT_READY:
        motorStateStr = "NOT_READY";
        break;
    default:
        motorStateStr = "UNKNOWN";
        break;
    }
    sprintf(msg, "  Motor: %s", motorStateStr);
    Console.println(msg);

    sprintf(msg, "  Homed: %s", state.isHomed ? "YES" : "NO");
    Console.println(msg);

    if (state.isHomed)
    {
        sprintf(msg, "  Position: %.2f mm", state.currentPositionMm);
        Console.println(msg);
    }
    else
    {
        Console.println(F("  Position: UNKNOWN"));
    }

    const char *hlfbStatusStr;
    switch (state.hlfbStatus)
    {
    case MotorDriver::HLFB_ASSERTED:
        hlfbStatusStr = "ASSERTED (In Position/Ready)";
        break;
    case MotorDriver::HLFB_DEASSERTED:
        hlfbStatusStr = "DEASSERTED (Moving/Fault)";
        break;
    case MotorDriver::HLFB_UNKNOWN:
    default:
        hlfbStatusStr = "UNKNOWN";
        break;
    }
    sprintf(msg, "  HLFB Status: %s", hlfbStatusStr);
    Console.println(msg);

    // Enhanced Valve States with Position Verification
    Console.println(F("\n  Valve States (with position verification):"));
    
    // Array of valve/sensor pairs for easy iteration
    const char *valveNames[4] = {"Tray 1", "Tray 2", "Tray 3", "Shuttle"};
    DoubleSolenoidValve *valves[4] = {
        getTray1Valve(), getTray2Valve(), getTray3Valve(), getShuttleValve()
    };
    CylinderSensor *sensors[4] = {
        getTray1Sensor(), getTray2Sensor(), getTray3Sensor(), getShuttleSensor()
    };
    bool sensorStates[4] = {
        state.tray1CylinderActivated, state.tray2CylinderActivated, 
        state.tray3CylinderActivated, state.shuttleCylinderActivated
    };
    
    for (int i = 0; i < 4; i++)
    {
        if (valves[i] && sensors[i])
        {
            bool isLocked = (valves[i]->position == VALVE_POSITION_LOCK);
            bool sensorState = sensorStates[i];
            bool positionVerified = (sensorState == !isLocked); // Sensor active = unlocked
            
            const char *valveStatus = isLocked ? "LOCKED" : "UNLOCKED";
            const char *verificationStatus = positionVerified ? "" : " [MISMATCH!]";
            
            sprintf(msg, "    %s: %s%s", valveNames[i], valveStatus, verificationStatus);
            Console.println(msg);
            
            // If there's a mismatch, provide additional detail
            if (!positionVerified)
            {
                sprintf(msg, "      WARNING: Valve set to %s but sensor reads %s", 
                        valveStatus, 
                        sensorState ? "ACTIVATED (unlocked)" : "NOT ACTIVATED (locked)");
                Console.println(msg);
            }
        }
        else
        {
            sprintf(msg, "    %s: VALVE/SENSOR ACCESS ERROR", valveNames[i]);
            Console.println(msg);
        }
    }

    // Cylinder sensors (raw readings) - Keep this for reference
    Console.println(F("\n  Cylinder Sensors (raw readings):"));
    sprintf(msg, "    Tray 1: %s", state.tray1CylinderActivated ? "ACTIVATED (UNLOCKED)" : "NOT ACTIVATED (LOCKED)");
    Console.println(msg);

    sprintf(msg, "    Tray 2: %s", state.tray2CylinderActivated ? "ACTIVATED (UNLOCKED)" : "NOT ACTIVATED (LOCKED)");
    Console.println(msg);

    sprintf(msg, "    Tray 3: %s", state.tray3CylinderActivated ? "ACTIVATED (UNLOCKED)" : "NOT ACTIVATED (LOCKED)");
    Console.println(msg);

    sprintf(msg, "    Shuttle: %s", state.shuttleCylinderActivated ? "ACTIVATED (UNLOCKED)" : "NOT ACTIVATED (LOCKED)");
    Console.println(msg);

    // Lock states (derived from sensor readings) - Keep this but note it's sensor-based
    Console.println(F("\n  Lock States (sensor-derived):"));
    sprintf(msg, "    Tray 1: %s", state.tray1Locked ? "LOCKED" : "UNLOCKED");
    Console.println(msg);

    sprintf(msg, "    Tray 2: %s", state.tray2Locked ? "LOCKED" : "UNLOCKED");
    Console.println(msg);

    sprintf(msg, "    Tray 3: %s", state.tray3Locked ? "LOCKED" : "UNLOCKED");
    Console.println(msg);

    sprintf(msg, "    Shuttle: %s", state.shuttleLocked ? "LOCKED" : "UNLOCKED");
    Console.println(msg);

    // Tray presence detection
    Console.println(F("\n  Tray Detection:"));
    sprintf(msg, "    Position 1: %s", state.tray1Present ? "TRAY PRESENT" : "NO TRAY");
    Console.println(msg);

    sprintf(msg, "    Position 2: %s", state.tray2Present ? "TRAY PRESENT" : "NO TRAY");
    Console.println(msg);

    sprintf(msg, "    Position 3: %s", state.tray3Present ? "TRAY PRESENT" : "NO TRAY");
    Console.println(msg);

    // Safety systems
    Console.println(F("\n  Safety Systems:"));
    sprintf(msg, "    E-Stop: %s", state.eStopActive ? "ACTIVE (Emergency Stop)" : "INACTIVE (Normal Operation)");
    Console.println(msg);

    // Hardware status
    Console.println(F("\n  Hardware Status:"));
    sprintf(msg, "    CCIO Board: %s", state.ccioBoardPresent ? "PRESENT" : "NOT DETECTED");
    Console.println(msg);

    sprintf(msg, "    Network Clients: %d connected", getConnectedClientCount());
    Console.println(msg);

    // Pneumatic system status
    float pressure = getPressurePsi();
    sprintf(msg, "    Pneumatic System: %.1f PSI %s", pressure,
            (pressure < MIN_SAFE_PRESSURE) ? "(INSUFFICIENT)" : "(OK)");
    Console.println(msg);

    // Enhanced Safety Summary - now includes valve/sensor mismatches
    Console.println(F("\n  Safety Summary:"));

    // Check if any tray is locked while motor is moving
    bool unsafeMotion = state.motorState == MOTOR_STATE_MOVING &&
                        (state.tray1Locked || state.tray2Locked || state.tray3Locked);
    sprintf(msg, "    Safe Motion: %s", unsafeMotion ? "NO - TRAYS LOCKED DURING MOTION" : "YES");
    Console.println(msg);

    // Check for missing trays that are locked
    bool missingTraysLocked = (state.tray1Locked && !state.tray1Present) ||
                              (state.tray2Locked && !state.tray2Present) ||
                              (state.tray3Locked && !state.tray3Present);
    sprintf(msg, "    Tray/Lock Mismatch: %s", missingTraysLocked ? "YES - LOCK WITHOUT TRAY" : "NO");
    Console.println(msg);

    // NEW: Check for valve/sensor mismatches
    bool valveSensorMismatch = false;
    for (int i = 0; i < 4; i++)
    {
        if (valves[i] && sensors[i])
        {
            bool isLocked = (valves[i]->position == VALVE_POSITION_LOCK);
            bool sensorState = sensorStates[i];
            bool positionVerified = (sensorState == !isLocked);
            
            if (!positionVerified)
            {
                valveSensorMismatch = true;
                break;
            }
        }
    }
    sprintf(msg, "    Valve/Sensor Alignment: %s", valveSensorMismatch ? "MISMATCH DETECTED [!]" : "VERIFIED");
    Console.println(msg);

    // Encoder status
    Console.println(F("\n  MPG Handwheel:"));
    sprintf(msg, "    Status: %s", encoderControlActive ? "ENABLED" : "DISABLED");
    Console.println(msg);

    if (encoderControlActive)
    {
        // Calculate how much one full rotation moves (100 pulses typical for MPG handwheels)
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        sprintf(msg, "    Multiplier: x%s (%.2f mm/rotation)", getMultiplierName(currentMultiplier), mmPerRotation);
        Console.println(msg);
    }

    Console.println(F("-------------------------------------------"));
}

// Validate safety conditions based on the current system state
// Returns a SafetyValidationResult with all validation checks and their results
SafetyValidationResult validateSafety(const SystemState &state)
{
    SafetyValidationResult result;

    // Initialize all safety flags to safe by default
    // Each will be set to false if a safety condition is not met
    result.safeToMove = true;
    result.pneumaticPressureSufficient = true;
    result.safeToLockTray1 = true;
    result.safeToLockTray2 = true;
    result.safeToLockTray3 = true;
    result.safeToLockShuttle = true;
    result.safeToUnlockShuttle = true;
    result.safeToLoadTrayToPos1 = true;
    result.safeToLoadTrayToPos2 = true;
    result.safeToLoadTrayToPos3 = true;
    result.safeToUnloadTrayFromPos1 = true;
    result.safeToUnloadTrayFromPos2 = true;
    result.safeToUnloadTrayFromPos3 = true;
    result.safeToUnlockGrippedTray = true;
    result.lockOperationSuccessful = true;
    result.unlockOperationSuccessful = true;
    result.commandStateValid = true;
    result.trayPositionValid = true;
    result.targetPositionValid = true;
    result.safeToAcceptNewCommand = true;
    result.operationWithinTimeout = true;
    result.operationSequenceValid = true;
    result.failureReason = ABORT_REASON_UNKNOWN;

    //=============================================================================
    // PNEUMATIC SYSTEM VALIDATION
    //=============================================================================
    // Validates that pneumatic pressure is sufficient for safe valve operations
    // This is critical for all pneumatic actions including locking/unlocking trays

    // Check if pressure is sufficient for valve actuation
    if (!isPressureSufficient())
    {
        result.pneumaticPressureSufficient = false;
        strcpy_P(result.pressureUnsafeReason, PSTR("Pneumatic pressure below minimum threshold"));

        // Set abort reason during operations requiring pneumatics
        // This will cause active operations to abort if pressure is lost
        if (operationInProgress &&
            (currentOperation.type == OPERATION_LOADING || currentOperation.type == OPERATION_UNLOADING))
        {
            result.failureReason = ABORT_REASON_PNEUMATIC_FAILURE;
        }
    }

    //=============================================================================
    // MOTOR MOVEMENT SAFETY
    //=============================================================================
    // Validates conditions that must be met before motor movement is allowed
    // Movement is blocked if any of these conditions are not satisfied

    // Mechanical Safety Constraints
    // Check which specific position has a locked tray that's blocking movement
    if (isAtPosition(state.currentPositionMm, POSITION_1_MM) && state.tray1Locked)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("Cannot move - Tray at position 1 is locked"));
    }
    else if (isAtPosition(state.currentPositionMm, POSITION_2_MM) && state.tray2Locked)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("Cannot move - Tray at position 2 is locked"));
    }
    else if (isAtPosition(state.currentPositionMm, POSITION_3_MM) && state.tray3Locked)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("Cannot move - Tray at position 3 is locked"));
    }

    // System State Requirements
    // Motor must be homed before movement to ensure position accuracy
    if (!state.isHomed)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("Motor not homed"));
        // This is a prerequisite safety check, not an abort condition
    }

    // Emergency Conditions
    // E-stop immediately prevents all movement and triggers abort
    if (state.eStopActive)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("E-stop active"));
        result.failureReason = ABORT_REASON_ESTOP; // E-stop is an immediate abort condition
    }

    // Hardware Presence Verification
    // CCIO board must be present for safe I/O operations
    if (!state.ccioBoardPresent)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("CCIO board not detected"));
        // No abort reason - this is a hardware presence check
    }

    // Motor Fault Detection
    // Prevents movement when motor is in fault state
    if (state.motorState == MOTOR_STATE_FAULTED)
    {
        result.safeToMove = false;
        strcpy_P(result.moveUnsafeReason, PSTR("Motor in fault state"));
        // No abort reason - motor already in fault state
    }

    //=============================================================================
    // CYLINDER LOCK/UNLOCK SAFETY
    //=============================================================================
    // Validates conditions for safely locking and unlocking tray cylinders
    // Prevents valve actuation in unsafe conditions

    // Tray Presence Verification
    // Cannot lock cylinders if no tray is present at position
    if (!state.tray1Present)
    {
        result.safeToLockTray1 = false;
        strcpy_P(result.tray1LockUnsafeReason, PSTR("No tray detected"));
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray2Present)
    {
        result.safeToLockTray2 = false;
        strcpy_P(result.tray2LockUnsafeReason, PSTR("No tray detected"));
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray3Present)
    {
        result.safeToLockTray3 = false;
        strcpy_P(result.tray3LockUnsafeReason, PSTR("No tray detected"));
        // No abort reason - this is a prerequisite check
    }

    // Movement Status Safety
    // Cannot lock trays while motor is moving
    if (state.motorState == MOTOR_STATE_MOVING)
    {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        strcpy_P(result.tray1LockUnsafeReason, PSTR("Motor is moving"));
        strcpy_P(result.tray2LockUnsafeReason, PSTR("Motor is moving"));
        strcpy_P(result.tray3LockUnsafeReason, PSTR("Motor is moving"));

        // Sequence validation: Detect unexpected movement during lock/unlock steps
        // For tray loading, steps 0-7 involve lock/unlock ops, while 8+ are for movement
        if (operationInProgress &&
            (previousState.motorState != MOTOR_STATE_MOVING) &&
            ((currentOperation.type == OPERATION_LOADING && currentOperationStep < 8) ||
             (currentOperation.type == OPERATION_UNLOADING && currentOperationStep < 3)))
        {
            // This means movement started during a lock/unlock operation - sequence violation
            result.operationSequenceValid = false;

            // Build detailed error message with operation-specific context
            const char* stepDescription = "";
            
            // Add specific step information for clearer diagnostics
            if (currentOperation.type == OPERATION_LOADING)
            {
                switch (currentOperationStep)
                {
                case 0:
                    stepDescription = "initial tray load preparation";
                    break;
                case 1:
                    stepDescription = "tray detection verification";
                    break;
                case 2:
                    stepDescription = "shuttle locking";
                    break;
                case 3:
                    stepDescription = "shuttle lock verification";
                    break;
                case 4:
                    stepDescription = "shuttle unlocking";
                    break;
                case 5:
                    stepDescription = "tray locking";
                    break;
                case 6:
                    stepDescription = "tray lock verification";
                    break;
                case 7:
                    stepDescription = "pre-movement preparation";
                    break;
                default:
                    stepDescription = "lock operation";
                    break;
                }
            }
            else
            {
                switch (currentOperationStep)
                {
                case 0:
                    stepDescription = "tray unload preparation";
                    break;
                case 1:
                    stepDescription = "sensor verification";
                    break;
                case 2:
                    stepDescription = "movement to source position";
                    break;
                case 3:
                    stepDescription = "movement monitoring";
                    break;
                case 4:
                    stepDescription = "shuttle locking";
                    break;
                default:
                    stepDescription = "lock operation";
                    break;
                }
            }

            snprintf(result.operationSequenceMessage, sizeof(result.operationSequenceMessage),
                     "Motor unexpectedly started moving during %s (step %d)", 
                     stepDescription, currentOperationStep);

            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    }

    // Shuttle/Tray Lock Exclusivity
    // Cannot lock trays when shuttle is locked (mechanical interference)
    if (state.shuttleLocked)
    {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        strcpy_P(result.tray1LockUnsafeReason, PSTR("Shuttle is locked"));
        strcpy_P(result.tray2LockUnsafeReason, PSTR("Shuttle is locked"));
        strcpy_P(result.tray3LockUnsafeReason, PSTR("Shuttle is locked"));

        // Sequence validation: Detect unexpected shuttle locking
        // We expect the shuttle to lock during specific operation steps only
        if (operationInProgress &&
            !previousState.shuttleLocked && state.shuttleLocked &&
            (currentOperation.type != OPERATION_LOADING ||
             (currentOperationStep != 2 && currentOperationStep != 3)))
        {
            // This means shuttle was locked unexpectedly outside the expected step
            result.operationSequenceValid = false;

            // Build detailed error message with operation context
            if (currentOperation.type == OPERATION_LOADING)
            {
                snprintf(result.operationSequenceMessage, sizeof(result.operationSequenceMessage),
                         "Shuttle unexpectedly locked during tray loading operation at step %d (expected at steps 2-3)",
                         currentOperationStep);
            }
            else if (currentOperation.type == OPERATION_UNLOADING)
            {
                // Provide specific guidance about when shuttle should be locked during unloading
                if (currentOperationStep < 3)
                {
                    snprintf(result.operationSequenceMessage, sizeof(result.operationSequenceMessage),
                             "Shuttle unexpectedly locked during tray unloading operation at step %d (expected at steps 4-5)",
                             currentOperationStep);
                }
                else if (currentOperationStep == 4 || currentOperationStep == 5)
                {
                    // This is a valid step for shuttle locking during unloading
                    result.operationSequenceValid = true; // Override the error
                    return result;                        // Skip setting failure reason
                }
                else
                {
                    snprintf(result.operationSequenceMessage, sizeof(result.operationSequenceMessage),
                             "Shuttle unexpectedly locked during tray unloading operation at step %d (unexpected at this step)",
                             currentOperationStep);
                }
            }
            else
            {
                strcpy(result.operationSequenceMessage, "Shuttle unexpectedly locked during operation at unexpected step");
            }

            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    }

    //=============================================================================
    // TRAY LOADING VALIDATION
    //=============================================================================
    // Validates conditions required for safely loading new trays
    // Prevents loading trays into occupied positions or exceeding capacity

    // Position Occupancy Checks
    // Cannot load trays into already occupied positions
    if (state.tray1Present)
    {
        result.safeToLoadTrayToPos1 = false;
        strcpy_P(result.loadTrayPos1UnsafeReason, PSTR("Position already occupied"));
        // No abort reason - this is a prerequisite check
    }

    if (state.tray2Present)
    {
        result.safeToLoadTrayToPos2 = false;
        strcpy_P(result.loadTrayPos2UnsafeReason, PSTR("Position already occupied"));
        // No abort reason - this is a prerequisite check
    }

    if (state.tray3Present)
    {
        result.safeToLoadTrayToPos3 = false;
        strcpy_P(result.loadTrayPos3UnsafeReason, PSTR("Position already occupied"));
        // No abort reason - this is a prerequisite check
    }

    // System Capacity Constraint
    // Cannot load more than 3 trays into the system
    if (state.tray1Present && state.tray2Present && state.tray3Present)
    {
        result.safeToLoadTrayToPos1 = false;
        result.safeToLoadTrayToPos2 = false;
        result.safeToLoadTrayToPos3 = false;
        strcpy_P(result.loadTrayPos1UnsafeReason, PSTR("All positions occupied"));
        strcpy_P(result.loadTrayPos2UnsafeReason, PSTR("All positions occupied"));
        strcpy_P(result.loadTrayPos3UnsafeReason, PSTR("All positions occupied"));
        // No abort reason - this is a prerequisite check
    }

    //=============================================================================
    // TRAY UNLOADING VALIDATION
    //=============================================================================
    // Validates conditions required for safely unloading trays
    // Enforces FILO (First-In-Last-Out) sequence for tray removal

    // Tray Presence Requirement
    // Cannot unload from empty positions
    if (!state.tray1Present)
    {
        result.safeToUnloadTrayFromPos1 = false;
        strcpy_P(result.unloadTrayPos1UnsafeReason, PSTR("No tray detected"));
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray2Present)
    {
        result.safeToUnloadTrayFromPos2 = false;
        strcpy_P(result.unloadTrayPos2UnsafeReason, PSTR("No tray detected"));
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray3Present)
    {
        result.safeToUnloadTrayFromPos3 = false;
        strcpy_P(result.unloadTrayPos3UnsafeReason, PSTR("No tray detected"));
        // No abort reason - this is a prerequisite check
    }

    // FILO (First-In-Last-Out) Sequence Enforcement
    // Trays must be unloaded in reverse order of loading

    // Position 1 must be unloaded first (most recently loaded)
    if (state.tray1Present)
    {
        result.safeToUnloadTrayFromPos2 = false;
        result.safeToUnloadTrayFromPos3 = false;
        strcpy_P(result.unloadTrayPos2UnsafeReason, PSTR("Tray 1 must be unloaded first"));
        strcpy_P(result.unloadTrayPos3UnsafeReason, PSTR("Tray 1 must be unloaded first"));
        // No abort reason - this is a prerequisite check
    }

    // Position 2 must be unloaded before position 3
    if (state.tray2Present && !state.tray1Present)
    {
        result.safeToUnloadTrayFromPos3 = false;
        strcpy_P(result.unloadTrayPos3UnsafeReason, PSTR("Tray 2 must be unloaded first"));
        // No abort reason - this is a prerequisite check
    }

    //=============================================================================
    // GRIPPED TRAY UNLOCK VALIDATION
    //=============================================================================
    // Validates conditions required for safely unlocking a tray that is being gripped
    // by the Mitsubishi robot during the unloading process

    // Tray presence check - can only unlock at position 1
    if (!state.tray1Present)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("No tray at position 1"));
        // No abort reason - this is a prerequisite check
    }

    // Tray lock check - must be locked before unlocking
    if (!state.tray1Locked)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("Tray not locked"));
        // No abort reason - this is a prerequisite check
    }

    // Shuttle retraction check - shuttle must be retracted for robot access
    if (state.shuttleLocked)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("Shuttle must be retracted"));
        // No abort reason - this is a prerequisite check
    }

    // Operation state check - can't unlock during active operations
    if (operationInProgress)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("Operation in progress"));
        // No abort reason - this is a prerequisite check
    }

    // Motor movement check - can't unlock while motor is moving
    if (state.motorState == MOTOR_STATE_MOVING)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("Motor is moving"));
        // No abort reason - this is a prerequisite check
    }

    // E-Stop check - can't unlock when E-Stop is active
    if (state.eStopActive)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("E-stop active"));
        // This is consistent with other E-stop handling
    }

    // Pneumatic pressure check - can't unlock without sufficient pressure
    if (!result.pneumaticPressureSufficient)
    {
        result.safeToUnlockGrippedTray = false;
        strcpy_P(result.grippedTrayUnlockUnsafeReason, PSTR("Insufficient pneumatic pressure"));
        // This uses the existing pneumatic pressure validation
    }

    //=============================================================================
    // LOCK/UNLOCK OPERATION VALIDATION
    //=============================================================================
    // Validates that recent lock/unlock operations succeeded
    // Triggers safety violations when lock/unlock operations fail during automated operations

    // Check for recent lock operation failures
    if (lastLockOperationFailed)
    {
        result.lockOperationSuccessful = false;
        strcpy(result.lockFailureDetails, lastLockFailureDetails);

        // If we're in an operation, mark the sequence as invalid
        if (operationInProgress)
        {
            result.operationSequenceValid = false;
            strcpy_P(result.operationSequenceMessage, PSTR("Lock operation failed: "));
            strcat(result.operationSequenceMessage, lastLockFailureDetails);
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;

            // Add operation context for better diagnostics
            if (currentOperation.type == OPERATION_LOADING)
            {
                strcat_P(result.operationSequenceMessage, PSTR(" during loading operation step "));
                char stepStr[10];
                itoa(currentOperationStep, stepStr, 10);
                strcat(result.operationSequenceMessage, stepStr);
            }
            else if (currentOperation.type == OPERATION_UNLOADING)
            {
                strcat_P(result.operationSequenceMessage, PSTR(" during unloading operation step "));
                char stepStr[10];
                itoa(currentOperationStep, stepStr, 10);
                strcat(result.operationSequenceMessage, stepStr);
            }
        }
    }

    // Check for recent unlock operation failures
    if (lastUnlockOperationFailed)
    {
        result.unlockOperationSuccessful = false;
        strcpy(result.unlockFailureDetails, lastUnlockFailureDetails);

        // If we're in an operation, mark the sequence as invalid
        if (operationInProgress)
        {
            result.operationSequenceValid = false;
            strcpy_P(result.operationSequenceMessage, PSTR("Unlock operation failed: "));
            strcat(result.operationSequenceMessage, lastUnlockFailureDetails);
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;

            // Add operation context for better diagnostics
            if (currentOperation.type == OPERATION_LOADING)
            {
                strcat_P(result.operationSequenceMessage, PSTR(" during loading operation step "));
                char stepStr[10];
                itoa(currentOperationStep, stepStr, 10);
                strcat(result.operationSequenceMessage, stepStr);
            }
            else if (currentOperation.type == OPERATION_UNLOADING)
            {
                strcat_P(result.operationSequenceMessage, PSTR(" during unloading operation step "));
                char stepStr[10];
                itoa(currentOperationStep, stepStr, 10);
                strcat(result.operationSequenceMessage, stepStr);
            }
        }
    }

    //=============================================================================
    // POSITION AND STATE VALIDATION
    //=============================================================================
    // Validates that motor position and system state match expectations
    // Detects position errors and unexpected conditions

    // Motor Position Accuracy Validation
    // Verifies that motor has reached commanded position within tolerance
    if (commandedPositionMm >= 0)
    {
        if (abs(state.currentPositionMm - commandedPositionMm) > POSITION_TOLERANCE_MM)
        {
            result.commandStateValid = false;

            // Create detailed error message with position values
            snprintf_P(result.stateValidationMessage, sizeof(result.stateValidationMessage),
                      PSTR("Position mismatch: current position %d mm vs. commanded %d mm (diff: %d mm)"),
                      state.currentPositionMm, commandedPositionMm, 
                      abs(state.currentPositionMm - commandedPositionMm));

            // Add motor state context for better diagnostics
            if (state.motorState == MOTOR_STATE_MOVING)
            {
                strcat_P(result.stateValidationMessage, PSTR(" - Motor still moving"));
            }
            else if (state.motorState == MOTOR_STATE_FAULTED)
            {
                strcat_P(result.stateValidationMessage, PSTR(" - Motor in FAULT state"));
            }
            else
            {
                strcat_P(result.stateValidationMessage, PSTR(" - Motor stopped before reaching target"));
            }

            // Set abort reason during operations - motor timeout or blockage
            if (operationInProgress)
            {
                result.failureReason = ABORT_REASON_MOTOR_TIMEOUT;
                result.operationSequenceValid = false;

                // Add operation context for more detailed error reporting
                strcat_P(result.stateValidationMessage, PSTR(" during "));
                if (currentOperation.type == OPERATION_LOADING)
                {
                    strcat_P(result.stateValidationMessage, PSTR("loading operation"));
                }
                else if (currentOperation.type == OPERATION_UNLOADING)
                {
                    strcat_P(result.stateValidationMessage, PSTR("unloading operation"));
                }
                else
                {
                    strcat_P(result.stateValidationMessage, PSTR("operation"));
                }
                strcat_P(result.stateValidationMessage, PSTR(" step "));
                char stepStr[10];
                itoa(currentOperationStep, stepStr, 10);
                strcat(result.stateValidationMessage, stepStr);

                strcpy(result.operationSequenceMessage, result.stateValidationMessage);
            }
        }
    }

    // Tray Position Validation
    // Verifies that trays are present at expected positions based on motor position
    bool tray1ExpectedPresent = isAtPosition(state.currentPositionMm, POSITION_1_MM);
    bool tray2ExpectedPresent = isAtPosition(state.currentPositionMm, POSITION_2_MM);
    bool tray3ExpectedPresent = isAtPosition(state.currentPositionMm, POSITION_3_MM);

    // Skip validation during active tray movement operations
    bool inTrayMovementOperation = (operationInProgress &&
                                    ((currentOperation.type == OPERATION_LOADING &&
                                      (currentOperationStep == 8 || currentOperationStep == 14)) ||
                                     (currentOperation.type == OPERATION_UNLOADING &&
                                      currentOperationStep == 9)) &&
                                    state.motorState == MOTOR_STATE_MOVING);

    // Skip validation during unloading preparation steps
    bool startingUnloadOperation = (operationInProgress &&
                                    currentOperation.type == OPERATION_UNLOADING &&
                                    currentOperationStep <= 3);

    // Only validate when not in movement or special operations
    if (!inTrayMovementOperation && !startingUnloadOperation)
    {
        // Safety critical: Verify expected trays are present
        if (tray1ExpectedPresent && !state.tray1Present)
        {
            result.trayPositionValid = false;

            // Detailed error message with position information
            snprintf_P(result.stateValidationMessage, sizeof(result.stateValidationMessage),
                      PSTR("ERROR: Expected tray at position 1 is missing (Motor at %d mm)"),
                      state.currentPositionMm);

            // Add operation context and set failure reason
            if (operationInProgress)
            {
                // Copy message to operation sequence for consistency
                strcpy(result.operationSequenceMessage, result.stateValidationMessage);
                result.operationSequenceValid = false;
                result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
            }
        }
    }

    // If not in an operation, tray position validation should pass
    // This ensures the system doesn't block commands when idle
    if (!operationInProgress)
    {
        result.trayPositionValid = true;
        strcpy_P(result.stateValidationMessage, PSTR("System idle - position validation not required"));
    }

    // Target Position Validation
    // Verifies that commanded target position is within allowed range
    bool skipTargetValidation = (operationInProgress &&
                                 currentOperation.type == OPERATION_UNLOADING &&
                                 (currentOperationStep >= 4 && currentOperationStep <= 10));

    if (hasCurrentTarget)
    {
        if (currentTargetPositionMm < 0 || currentTargetPositionMm > MAX_TRAVEL_MM)
        {
            result.targetPositionValid = false;
            strcpy_P(result.stateValidationMessage, PSTR("Target position out of range"));
            strcpy(result.operationSequenceMessage, result.stateValidationMessage);
        }
    }
    else if (operationInProgress && !skipTargetValidation)
    {
        // Missing target is only a problem during active operations
        result.targetPositionValid = false;
        strcpy_P(result.stateValidationMessage, PSTR("No target position set during active operation"));
        strcpy(result.operationSequenceMessage, result.stateValidationMessage);
    }
    else if (!hasCurrentTarget)
    {
        // No target position when system is idle is perfectly fine
        result.targetPositionValid = true;
        strcpy_P(result.stateValidationMessage, PSTR("System idle - no target needed"));
    }

    //=============================================================================
    // OPERATION SEQUENCE VALIDATION
    //=============================================================================
    // Validates that operations follow correct sequence and timing
    // Prevents command conflicts and detects sequence errors

    // Command Exclusivity Check
    // No new commands allowed during active operations
    if (operationInProgress)
    {
        if (newCommandReceived)
        {
            result.safeToAcceptNewCommand = false;
            strcpy_P(result.operationSequenceMessage, PSTR("Operation in progress, cannot accept new command"));
            strcat_P(result.operationSequenceMessage, PSTR(" (use 'system,reset' after failure)"));
        }
    }

    // Operation Timeout Detection
    // Detects and reports operations that exceed their timeout
    if (operationInProgress && timeoutElapsed(millis(), operationStartTime, operationTimeoutMs))
    {
        result.operationWithinTimeout = false;
        strcpy_P(result.operationSequenceMessage, PSTR("Operation exceeded timeout"));
        result.failureReason = ABORT_REASON_OPERATION_TIMEOUT;
    }

    // Operation Step Sequence Validation
    // Verifies that operation steps are executed in correct sequence
    if (operationInProgress && currentOperationStep != expectedOperationStep)
    {
        result.operationSequenceValid = false;

        // Create detailed error message with operation context
        strcpy_P(result.operationSequenceMessage, PSTR("Operation sequence mismatch: "));

        // Add operation-specific type information
        switch (currentOperation.type)
        {
        case OPERATION_LOADING:
            strcat_P(result.operationSequenceMessage, PSTR("Tray loading operation"));
            break;
        case OPERATION_UNLOADING:
            strcat_P(result.operationSequenceMessage, PSTR("Tray unloading operation"));
            break;
        default:
            strcat_P(result.operationSequenceMessage, PSTR("Current operation"));
            break;
        }

        // Add step information for debugging
        strcat_P(result.operationSequenceMessage, PSTR(" at step "));
        char stepStr[10];
        itoa(currentOperationStep, stepStr, 10);
        strcat(result.operationSequenceMessage, stepStr);
        strcat_P(result.operationSequenceMessage, PSTR(" (expected: "));
        char expectedStr[10];
        itoa(expectedOperationStep, expectedStr, 10);
        strcat(result.operationSequenceMessage, expectedStr);
        strcat_P(result.operationSequenceMessage, PSTR(")"));

        result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
    }

    return result;
}

// Function to print out safety validation results
void printSafetyStatus(const SafetyValidationResult &result)
{
    char msg[300];

    Console.println(F("[SAFETY] Validation Results:"));

    // Movement safety
    if (result.safeToMove)
    {
        Console.println(F("  Motor Movement: SAFE - System ready for movement"));
    }
    else
    {
        sprintf(msg, "  Motor Movement: UNSAFE - %s", result.moveUnsafeReason);
        Console.println(msg);
    }

    // Pneumatic pressure safety status
    if (result.pneumaticPressureSufficient)
    {
        sprintf(msg, "  Pneumatic System: SAFE - %.1f PSI (sufficient pressure for valve operations)", getPressurePsi());
        Console.println(msg);
    }
    else
    {
        sprintf(msg, "  Pneumatic System: UNSAFE - %s", result.pressureUnsafeReason);
        Console.println(msg);
        sprintf(msg, "    Current pressure: %.1f PSI, Minimum required: %.1f", getPressurePsi(), MIN_SAFE_PRESSURE);
        Console.println(msg);
    }

    // Tray locking safety with enhanced safety messages
    Console.println(F("  Tray Locking:"));

    // Helper function to get tray lock status message
    auto getTrayLockMessage = [](bool safe, bool operationInProgress, int operationType) -> const char *
    {
        if (!safe)
            return nullptr; // Will be handled separately
        if (operationInProgress && operationType == OPERATION_LOADING)
            return "SAFE TO LOCK - Part of loading sequence";
        else if (operationInProgress && operationType == OPERATION_UNLOADING)
            return "SAFE TO LOCK - Part of unloading sequence";
        else
            return "SAFE TO LOCK - Tray present and system ready";
    };

    if (result.safeToLockTray1)
    {
        sprintf(msg, "    Tray 1: %s", getTrayLockMessage(true, operationInProgress, currentOperation.type));
        Console.println(msg);
    }
    else
    {
        sprintf(msg, "    Tray 1: UNSAFE TO LOCK - %s", result.tray1LockUnsafeReason);
        Console.println(msg);
    }

    if (result.safeToLockTray2)
    {
        sprintf(msg, "    Tray 2: %s", getTrayLockMessage(true, operationInProgress, currentOperation.type));
        Console.println(msg);
    }
    else
    {
        sprintf(msg, "    Tray 2: UNSAFE TO LOCK - %s", result.tray2LockUnsafeReason);
        Console.println(msg);
    }

    if (result.safeToLockTray3)
    {
        sprintf(msg, "    Tray 3: %s", getTrayLockMessage(true, operationInProgress, currentOperation.type));
        Console.println(msg);
    }
    else
    {
        sprintf(msg, "    Tray 3: UNSAFE TO LOCK - %s", result.tray3LockUnsafeReason);
        Console.println(msg);
    }

    // Shuttle actuation safety
    Console.println(F("  Shuttle Control:"));

    if (result.safeToLockShuttle)
    {
        if (operationInProgress && currentOperation.type == OPERATION_LOADING &&
            (currentOperationStep == 2 || currentOperationStep == 3))
        {
            Console.println(F("    Lock: SAFE - Part of tray loading sequence"));
        }
        else
        {
            Console.println(F("    Lock: SAFE - System ready for shuttle locking"));
        }
    }
    else
    {
        sprintf(msg, "    Lock: UNSAFE - %s", result.shuttleLockUnsafeReason);
        Console.println(msg);
    }

    if (result.safeToUnlockShuttle)
    {
        if (operationInProgress && currentOperation.type == OPERATION_LOADING &&
            (currentOperationStep == 4 || currentOperationStep == 5))
        {
            Console.println(F("    Unlock: SAFE - Part of tray loading sequence"));
        }
        else
        {
            Console.println(F("    Unlock: SAFE - System ready for shuttle unlocking"));
        }
    }
    else
    {
        sprintf(msg, "    Unlock: UNSAFE - %s", result.shuttleUnlockUnsafeReason);
        Console.println(msg);
    }

    // Lock/unlock operation status
    Console.println(F("\n  Lock/Unlock Operations:"));

    if (result.lockOperationSuccessful)
    {
        Console.println(F("    Lock Operations: SUCCESSFUL - No recent lock failures"));
    }
    else
    {
        sprintf(msg, "    Lock Operations: FAILED - %s", result.lockFailureDetails);
        Console.println(msg);

        if (lockFailureTimestamp > 0)
        {
            unsigned long elapsedTime = timeDiff(millis(), lockFailureTimestamp) / 1000;
            sprintf(msg, "              Failure occurred %lu seconds ago", elapsedTime);
            Console.println(msg);
        }
    }

    if (result.unlockOperationSuccessful)
    {
        Console.println(F("    Unlock Operations: SUCCESSFUL - No recent unlock failures"));
    }
    else
    {
        sprintf(msg, "    Unlock Operations: FAILED - %s", result.unlockFailureDetails);
        Console.println(msg);

        if (unlockFailureTimestamp > 0)
        {
            unsigned long elapsedTime = timeDiff(millis(), unlockFailureTimestamp) / 1000;
            sprintf(msg, "              Failure occurred %lu seconds ago", elapsedTime);
            Console.println(msg);
        }
    }

    if (!result.lockOperationSuccessful || !result.unlockOperationSuccessful)
    {
        Console.println(F("    Recovery: Use 'tray,released' or 'tray,gripped' to retry the operation, or 'system,reset' to clear the alert"));
    }

    // System State Validation
    Console.println(F("\n  System State Validation:"));

    // Command/Actual State
    if (result.commandStateValid)
    {
        Console.println(F("    Command/Actual State: VALID - System position matches commanded position"));
    }
    else
    {
        sprintf(msg, "    Command/Actual State: INVALID - %s", result.stateValidationMessage);
        Console.println(msg);
    }

    // Tray Positions
    if (result.trayPositionValid)
    {
        Console.println(F("    Tray Positions: VALID - Tray presence matches expected positions"));
    }
    else
    {
        sprintf(msg, "    Tray Positions: INVALID - %s", result.stateValidationMessage);
        Console.println(msg);
    }

    // Target Position
    if (result.targetPositionValid)
    {
        if (hasCurrentTarget)
        {
            sprintf(msg, "    Target Position: VALID - Target position %.2f mm is within safe range", currentTargetPositionMm);
            Console.println(msg);
        }
        else
        {
            Console.println(F("    Target Position: VALID - No target position currently set"));
        }
    }
    else
    {
        sprintf(msg, "    Target Position: INVALID - %s", result.stateValidationMessage);
        Console.println(msg);
    }

    // Operational sequence validation
    Console.println(F("\n  Operational Sequence:"));

    if (result.safeToAcceptNewCommand)
    {
        if (operationInProgress)
        {
            const char *opTypeStr;
            switch (currentOperation.type)
            {
            case OPERATION_LOADING:
                opTypeStr = "LOADING";
                break;
            case OPERATION_UNLOADING:
                opTypeStr = "UNLOADING";
                break;
            default:
                opTypeStr = "OTHER";
                break;
            }
            sprintf(msg, "    Accept New Commands: SAFE - Current operation: %s (Step %d)", opTypeStr, currentOperationStep);
            Console.println(msg);
        }
        else
        {
            Console.println(F("    Accept New Commands: SAFE - No operation in progress"));
        }
    }
    else
    {
        sprintf(msg, "    Accept New Commands: UNSAFE - %s", result.operationSequenceMessage);
        Console.println(msg);
    }

    if (result.operationWithinTimeout)
    {
        if (operationInProgress)
        {
            unsigned long elapsedTime = timeDiff(millis(), operationStartTime);
            sprintf(msg, "    Operation Timing: WITHIN TIMEOUT - Elapsed: %lu.%lu s / %lu s",
                    elapsedTime / 1000, (elapsedTime % 1000) / 100, operationTimeoutMs / 1000);
            Console.println(msg);
        }
        else
        {
            Console.println(F("    Operation Timing: WITHIN TIMEOUT - No operation active"));
        }
    }
    else
    {
        sprintf(msg, "    Operation Timing: TIMEOUT - %s", result.operationSequenceMessage);
        Console.println(msg);
    }

    if (result.operationSequenceValid)
    {
        if (operationInProgress)
        {
            if (currentOperationStep == expectedOperationStep)
            {
                sprintf(msg, "    Operation Sequence: VALID - Current step: %d (as expected)", currentOperationStep);
                Console.println(msg);
            }
            else
            {
                sprintf(msg, "    Operation Sequence: VALID - Current step: %d (expected: %d)",
                        currentOperationStep, expectedOperationStep);
                Console.println(msg);
            }
        }
        else
        {
            Console.println(F("    Operation Sequence: VALID - No operation in progress"));
        }
    }
    else
    {
        sprintf(msg, "    Operation Sequence: INVALID - %s", result.operationSequenceMessage);
        Console.println(msg);

        if (strstr_P(result.operationSequenceMessage, PSTR("sequence mismatch")) != NULL)
        {
            Console.println(F("            Use 'system,reset' to reset the system"));
        }
    }

    // Tray loading operations
    Console.println(F("\n  Tray Loading Operations:"));

    if (result.safeToLoadTrayToPos1)
    {
        const char *loadContext;
        if (trayTracking.totalTraysInSystem == 0)
            loadContext = " - Ready for first tray";
        else if (trayTracking.totalTraysInSystem == 1)
            loadContext = " - Ready for second tray";
        else if (trayTracking.totalTraysInSystem == 2)
            loadContext = " - Ready for third tray";
        else
            loadContext = " - Ready for tray";

        sprintf(msg, "    Position 1: SAFE TO LOAD%s", loadContext);
        Console.println(msg);
    }
    else
    {
        sprintf(msg, "    Position 1: UNSAFE TO LOAD - %s", result.loadTrayPos1UnsafeReason);
        Console.println(msg);
    }

    if (result.safeToLoadTrayToPos2)
    {
        Console.println(F("    Position 2: SAFE TO LOAD - Direct loading possible"));
    }
    else
    {
        sprintf(msg, "    Position 2: UNSAFE TO LOAD - %s", result.loadTrayPos2UnsafeReason);
        Console.println(msg);
    }

    if (result.safeToLoadTrayToPos3)
    {
        Console.println(F("    Position 3: SAFE TO LOAD - Direct loading possible"));
    }
    else
    {
        sprintf(msg, "    Position 3: UNSAFE TO LOAD - %s", result.loadTrayPos3UnsafeReason);
        Console.println(msg);
    }

    // Tray unloading operations
    Console.println(F("\n  Tray Unloading Operations:"));

    if (result.safeToUnloadTrayFromPos1)
    {
        Console.println(F("    Position 1: SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        sprintf(msg, "    Position 1: UNSAFE TO UNLOAD - %s", result.unloadTrayPos1UnsafeReason);
        Console.println(msg);
    }

    if (result.safeToUnloadTrayFromPos2)
    {
        Console.println(F("    Position 2: SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        sprintf(msg, "    Position 2: UNSAFE TO UNLOAD - %s", result.unloadTrayPos2UnsafeReason);
        Console.println(msg);
    }

    if (result.safeToUnloadTrayFromPos3)
    {
        Console.println(F("    Position 3: SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        sprintf(msg, "    Position 3: UNSAFE TO UNLOAD - %s", result.unloadTrayPos3UnsafeReason);
        Console.println(msg);
    }

    // System Summary
    Console.println(F("\n  System Summary:"));

    if (result.operationSequenceValid && result.trayPositionValid && result.commandStateValid &&
        (result.targetPositionValid || !operationInProgress))
    {
        Console.println(F("    Status: NORMAL - System operating correctly"));

        if (operationInProgress)
        {
            const char *opTypeStr;
            switch (currentOperation.type)
            {
            case OPERATION_LOADING:
                opTypeStr = "LOADING";
                break;
            case OPERATION_UNLOADING:
                opTypeStr = "UNLOADING";
                break;
            default:
                opTypeStr = "OTHER";
                break;
            }
            sprintf(msg, "    Current Operation: %s (Step %d)", opTypeStr, currentOperationStep);
            Console.println(msg);
        }
    }
    else
    {
        Console.println(F("    Status: ALERT - System requires attention"));

        if (!result.operationSequenceValid)
        {
            Console.println(F("    Reason: Operation sequence error detected"));
            sprintf(msg, "            %s", result.operationSequenceMessage);
            Console.println(msg);
        }
        else if (!result.trayPositionValid)
        {
            Console.println(F("    Reason: Tray position error detected"));
            sprintf(msg, "            %s", result.stateValidationMessage);
            Console.println(msg);
        }
        else if (!result.targetPositionValid && operationInProgress)
        {
            Console.println(F("    Reason: Target position error detected"));
            sprintf(msg, "            %s", result.stateValidationMessage);
            Console.println(msg);
        }
        else if (!result.commandStateValid)
        {
            Console.println(F("    Reason: Motor position error detected"));
            sprintf(msg, "            %s", result.stateValidationMessage);
            Console.println(msg);
        }

        Console.println(F("    Recovery: Use 'system,reset' to reset system state and try again"));
    }
}

// Motor position helper functions
bool isAtPosition(double currentPosition, double targetPosition)
{
    return abs(currentPosition - targetPosition) <= POSITION_TOLERANCE_MM;
}

// Add this function implementation near your other movement helper functions:
bool isMovingToPosition(double targetPosition, double currentTargetPositionMm)
{
    return abs(currentTargetPositionMm - targetPosition) <= POSITION_TOLERANCE_MM;
}

// Check if the path is clear for loading movements between two positions
bool isPathClearForLoading(double startPosition, double targetPosition, const SystemState &state)
{
    // If moving from position 1 to position 3, check that position 2 is clear
    if (isAtPosition(startPosition, POSITION_1_MM) && isMovingToPosition(POSITION_3_MM, targetPosition))
    {
        if (state.tray2Present || state.tray3Present)
        {
            return false; // Path not clear
        }
        return true;
    }

    // If moving from position 1 to position 2, make sure position 2 is clear
    if (isAtPosition(startPosition, POSITION_1_MM) && isMovingToPosition(POSITION_2_MM, targetPosition))
    {
        if (state.tray2Present)
        {
            return false; // Path not clear
        }
        return true;
    }

    // When returning to position 1 (empty shuttle), no path checking needed
    if ((isAtPosition(startPosition, POSITION_2_MM) || isAtPosition(startPosition, POSITION_3_MM)) &&
        isMovingToPosition(POSITION_1_MM, targetPosition))
    {
        return true; // Always safe for empty shuttle to return
    }

    // Default - assume the path is clear for any other case
    return true;
}

// Check if the path is clear for unloading movements between two positions
bool isPathClearForUnloading(double startPosition, double targetPosition, const SystemState &state)
{
    // SCENARIO 1: Moving from loading position (1) to pick up a tray (2 or 3)
    // When going to pick up a tray, no path checking needed as shuttle is empty
    if (isAtPosition(startPosition, POSITION_1_MM) &&
        (isMovingToPosition(POSITION_2_MM, targetPosition) ||
         isMovingToPosition(POSITION_3_MM, targetPosition)))
    {
        // Always safe to move an empty shuttle from position 1 to pick up a tray
        return true;
    }

    // SCENARIO 2: Moving FROM position 3 TO position 1 WITH a tray
    // Must check that position 2 is clear to avoid collision
    // AND must check that position 1 is clear (destination)
    if (isAtPosition(startPosition, POSITION_3_MM) && isMovingToPosition(POSITION_1_MM, targetPosition))
    {
        if (state.tray2Present)
        {
            return false; // Path blocked at position 2
        }

        if (state.tray1Present)
        {
            return false; // Destination already occupied
        }
        return true;
    }

    // SCENARIO 3: Moving FROM position 2 TO position 1 WITH a tray
    // Must check that position 1 is clear (destination)
    if (isAtPosition(startPosition, POSITION_2_MM) && isMovingToPosition(POSITION_1_MM, targetPosition))
    {
        if (state.tray1Present)
        {
            return false; // Destination already occupied
        }
        return true;
    }

    // Default - assume the path is clear for any other case
    return true;
}

// Process tray operations based on the current operation state
void processTrayOperations()
{
    // Only do something if operation is in progress
    if (!currentOperation.inProgress)
    {
        return;
    }

    unsigned long currentTime = millis();

    // Check for timeout
    if (timeoutElapsed(currentTime, currentOperation.startTime, operationTimeoutMs))
    {
        // Handle timeout
        Console.serialError(F("Tray operation timeout"));
        currentOperation.inProgress = false;
        currentOperation.success = false;
        strncpy(currentOperation.message, "TIMEOUT", sizeof(currentOperation.message));
        return;
    }

    // Process based on operation type
    switch (currentOperation.type)
    {
    case OPERATION_LOADING:
        processTrayLoading();
        break;

    case OPERATION_UNLOADING:
        processTrayUnloading();
        break;

    case OPERATION_TRAY_ADVANCE:
        processTrayAdvance();
        break;

    default:
        // Unknown operation, cancel it
        currentOperation.inProgress = false;
        break;
    }
}

// Process the tray loading operation state machine
void processTrayLoading()
{
    // Current time for non-blocking timing checks
    unsigned long currentMillis = millis();

    // Target position based on tray tracking
    static double targetPosition = 0;
    static bool isShuttleNeeded = false;

    // Single message buffer for all sprintf operations in this function
    char messageBuffer[150];

    switch (currentOperationStep)
    {
    case 0: // Initial checks and determine destination
    {
        // Capture system state to get latest sensor readings
        SystemState state = captureSystemState();

        Console.serialInfo(F("Starting tray loading process - initial checks"));

        // Verify tray is at position 1 and locked
        if (!state.tray1Present)
        {
            Console.serialError(F("No tray detected at position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "NO_TRAY", sizeof(currentOperation.message));
            return;
        }

        if (!state.tray1Locked)
        {
            Console.serialError(F("Tray at position 1 not locked"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY_NOT_LOCKED", sizeof(currentOperation.message));
            return;
        }

        // Determine target position based on occupied positions
        int workflow = determineLoadingWorkflow();
        if (workflow == 1)
        {
            // First tray - goes to position 3
            targetPosition = POSITION_3_MM;
            isShuttleNeeded = true;
            Console.serialInfo(F("First tray - target is position 3"));
        }
        else if (workflow == 2)
        {
            // Second tray - goes to position 2
            targetPosition = POSITION_2_MM;
            isShuttleNeeded = true;
            Console.serialInfo(F("Second tray - target is position 2"));
        }
        else
        {
            // Third tray - stays at position 1
            isShuttleNeeded = false;
            Console.serialInfo(F("Third tray - keeping at position 1"));
            // Skip to the final step for position 1
            updateOperationStep(12); // Special case - skip to completion
            return;
        }
        // Check if target position is occupied
        if ((targetPosition == POSITION_2_MM && state.tray2Present) ||
            (targetPosition == POSITION_3_MM && state.tray3Present))
        {
            Console.serialError(F("Target position already occupied"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TARGET_POSITION_OCCUPIED", sizeof(currentOperation.message));
            return;
        }

        // Check if path is clear to target position
        if (!isPathClearForLoading(state.currentPositionMm, targetPosition, state))
        {
            Console.serialError(F("Path to target position is blocked"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "PATH_BLOCKED", sizeof(currentOperation.message));
            return;
        }
        // All checks pass, advance to next step
        updateOperationStep(1);
        Console.serialInfo(F("Initial checks passed, starting tray advancement sequence"));

        // Start verification delay to ensure stable sensor readings
        sensorVerificationStartTime = currentMillis;
    }
    break;

    case 1: // Verify sensor readings after brief delay to ensure stability
    {
        // Wait for sensor verification delay
        if (!timeoutElapsed(currentMillis, sensorVerificationStartTime, SENSOR_VERIFICATION_DELAY_MS))
        {
            return;
        }

        // Double-check sensor readings after stabilization delay
        SystemState state = captureSystemState();

        // Re-verify tray presence and lock state at position 1
        if (!state.tray1Present)
        {
            Console.serialError(F("Tray at position 1 disappeared during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY1_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }

        if (!state.tray1Locked)
        {
            Console.serialError(F("Tray 1 lock status changed during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY1_LOCK_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step - shuttle locking if needed
        updateOperationStep(2);
        Console.serialInfo(F("Sensor verification complete"));
    }
    break;

    case 2: // Lock shuttle to grip tray (if needed)
    {
        if (!isShuttleNeeded)
        {
            // Skip shuttle locking if not moving the tray
            updateOperationStep(4);
            Console.serialInfo(F("Skipping shuttle operation (not needed for this move)"));
            return;
        }

        // Start locking shuttle
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to lock shuttle to grip tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to lock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Console.serialInfo(F("Initiated shuttle lock valve actuation"));
        }
        else
        {
            Console.serialError(F("Failed to access shuttle valve"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step
        updateOperationStep(3);
    }
    break;

    case 3: // Wait for shuttle lock valve actuation
    {
        // Wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Valve actuation time has elapsed, verify shuttle is locked
        SystemState state = captureSystemState();
        if (!state.shuttleLocked)
        {
            Console.serialError(F("Failed to lock shuttle - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }
        Console.serialInfo(F("Shuttle lock confirmed successful"));

        // Shuttle is locked, proceed to unlock tray at position 1
        updateOperationStep(4);
    }
    break;

    case 4: // Unlock tray at position 1
    {
        // Start unlocking tray 1
        DoubleSolenoidValve *valve = getTray1Valve();
        if (valve)
        {
            Console.serialInfo(F("Attempting to unlock tray at position 1"));

            if (!safeValveOperation(*valve, *getTray1Sensor(), VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to unlock tray 1 - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Console.serialInfo(F("Initiated tray 1 unlock valve actuation"));
        }
        else
        {
            Console.serialError(F("Failed to access tray 1 valve"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step
        updateOperationStep(5);
    }
    break;

    case 5: // Wait for tray unlock valve actuation
    {
        // Wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Valve actuation time has elapsed, verify unlock
        SystemState state = captureSystemState();
        if (state.tray1Locked)
        {
            Console.serialError(F("Failed to unlock tray at position 1 - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Tray 1 unlock confirmed successful"));
        // Add safety delay after unlocking tray before movement
        safetyDelayStartTime = currentMillis;
        updateOperationStep(6);
    }
    break;

    case 6: // Safety delay after unlocking tray before movement
    {
        // Wait for safety delay to elapse
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_AFTER_UNLOCK_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay after unlock completed"));
        // If we're not moving the tray, skip to completion
        if (!isShuttleNeeded)
        {
            // No movement needed, skip to tray tracking update
            updateOperationStep(12);
            return;
        }

        // Add additional safety delay before movement
        safetyDelayStartTime = currentMillis;
        updateOperationStep(7);
    }
    break;

    case 7: // Safety delay before starting motor movement
    {
        // Wait for safety delay before movement
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_BEFORE_MOVEMENT_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay before movement completed"));

        // Start movement to target position
        if (!moveToPositionMm(targetPosition))
        {
            Console.serialError(F("Failed to start movement to target position"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "MOVE_FAILURE", sizeof(currentOperation.message));
            return;
        }

        sprintf(messageBuffer, "Moving tray to position %d", targetPosition);
        Console.serialInfo(messageBuffer);
        // Advance to movement monitoring step
        updateOperationStep(8);
    }
    break;

    case 8: // Monitor motor movement and wait for completion
    {
        // Check if motor is still moving
        if (motorState == MOTOR_STATE_MOVING)
        {
            // Motor still moving, wait
            return;
        }

        Console.serialInfo(F("Motor movement completed"));

        // Motor has stopped, verify position with tolerance
        SystemState state = captureSystemState();
        const float POSITION_WARNING_TOLERANCE = 2.0; // 2mm tolerance for warnings vs errors
        bool reachedTarget = false;
        float targetPos = 0.0;

        if (targetPosition == POSITION_2_MM)
        {
            reachedTarget = isAtPosition(state.currentPositionMm, POSITION_2_MM);
            targetPos = POSITION_2_MM;
        }
        else if (targetPosition == POSITION_3_MM)
        {
            reachedTarget = isAtPosition(state.currentPositionMm, POSITION_3_MM);
            targetPos = POSITION_3_MM;
        }

        if (!reachedTarget)
        {
            // Report the error
            Console.serialError(F("Motor did not reach target position"));

            // Calculate how far off we are from the target
            float positionError = abs(state.currentPositionMm - targetPos);

            if (positionError <= POSITION_WARNING_TOLERANCE)
            {
                // Close enough to continue - log warning but continue operation
                sprintf(messageBuffer, "Position error of %.1fmm is within tolerance - continuing", positionError);
                Console.serialWarning(messageBuffer);

                // Continue with the loading sequence despite small position error
                safetyDelayStartTime = currentMillis;
                updateOperationStep(9);
            }
            else
            {
                // Major position error - mark as failure but still end the operation
                Console.error(F("TARGET_POSITION_ERROR"));
                sprintf(messageBuffer, "Position error of %.1fmm exceeds tolerance", positionError);
                Console.serialError(messageBuffer);

                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "TARGET_POSITION_FAILURE", sizeof(currentOperation.message));

                // CRITICAL: Always call endOperation() to clean up state
                // This ensures the system doesn't remain in "busy" state
                endOperation();
            }
            return;
        }

        // Add safety delay after movement completion
        safetyDelayStartTime = currentMillis;
        updateOperationStep(9);
    }
    break;

    case 9: // Safety delay after motor movement
    {
        // Wait for safety delay after movement
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_AFTER_MOVEMENT_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay after movement completed"));

        // Unlock shuttle now that we've reached the destination
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to unlock shuttle to release tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to unlock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Console.serialInfo(F("Initiated shuttle unlock valve actuation"));
        }
        else
        {
            Console.serialError(F("Failed to access shuttle valve"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step
        updateOperationStep(10);
    }
    break;

    case 10: // Wait for shuttle unlock valve actuation
    {
        // Wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Valve actuation time has elapsed, verify shuttle is unlocked
        SystemState state = captureSystemState();
        if (state.shuttleLocked)
        {
            Console.serialError(F("Failed to unlock shuttle - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Shuttle unlock confirmed successful"));
        // Add safety delay after shuttle unlock
        safetyDelayStartTime = currentMillis;
        updateOperationStep(11);
    }
    break;

    case 11: // Safety delay after shuttle unlock, then lock tray at target
    {
        // Wait for safety delay to elapse
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_AFTER_UNLOCK_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay after shuttle unlock completed"));

        // Lock tray at target position
        DoubleSolenoidValve *valve = NULL;
        CylinderSensor *sensor = NULL;

        if (targetPosition == POSITION_2_MM)
        {
            valve = getTray2Valve();
            sensor = getTray2Sensor();
            Console.serialInfo(F("Attempting to lock tray at position 2"));
        }
        else if (targetPosition == POSITION_3_MM)
        {
            valve = getTray3Valve();
            sensor = getTray3Sensor();
            Console.serialInfo(F("Attempting to lock tray at position 3"));
        }

        if (valve && sensor)
        {
            if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to lock tray at target position - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            sprintf(messageBuffer, "Initiated tray lock valve actuation at position %d", targetPosition);
            Console.serialInfo(messageBuffer);
        }
        else
        {
            Console.serialError(F("Failed to access target position valve or sensor"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step
        updateOperationStep(12);
    }
    break;

    case 12: // Wait for tray lock valve actuation at target and update tracking
    {
        // If we didn't move the tray (3rd tray case), skip lock verification
        if (!isShuttleNeeded)
        {
            // Just update tray tracking for position 1
            Console.serialInfo(F("Updating tray tracking for position 1"));
            loadThirdTray();

            // Operation complete
            Console.serialInfo(F("Tray loading at position 1 completed successfully"));
            currentOperation.inProgress = false;
            currentOperation.success = true;
            strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));

            // End operation and reset target tracking
            endOperation();
            return;
        }

        // For moved trays, wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Valve actuation time has elapsed, verify lock
        SystemState state = captureSystemState();
        bool lockSuccessful = false;

        if (targetPosition == POSITION_2_MM)
        {
            lockSuccessful = state.tray2Locked;
        }
        else if (targetPosition == POSITION_3_MM)
        {
            lockSuccessful = state.tray3Locked;
        }

        if (!lockSuccessful)
        {
            Console.serialError(F("Failed to lock tray at target position - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Tray lock at target position confirmed successful"));

        // Update tray tracking based on target position
        if (targetPosition == POSITION_2_MM)
        {
            // Move from position 1 to 2
            Console.serialInfo(F("Updating tray tracking: position 1 -> position 2"));
            loadSecondTray();
        }
        else if (targetPosition == POSITION_3_MM)
        {
            // Move from position 1 to 3
            Console.serialInfo(F("Updating tray tracking: position 1 -> position 3"));
            loadFirstTray();
        }
        // Add safety delay before returning to position 1
        safetyDelayStartTime = currentMillis;
        updateOperationStep(13);
    }
    break;

    case 13: // Safety delay before returning to position 1
    {
        // Wait for safety delay to elapse
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_BEFORE_MOVEMENT_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay before return movement completed"));

        // Now return the conveyor to position 1
        if (!moveToPositionMm(POSITION_1_MM))
        {
            Console.serialError(F("Failed to start movement to loading position"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "RETURN_MOVE_FAILURE", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step (watching for conveyor return)
        updateOperationStep(14);
        Console.serialInfo(F("Returning conveyor to loading position"));
    }
    break;

    case 14: // Monitor return movement to position 1
    {
        // Wait for motor return to position 1
        if (motorState == MOTOR_STATE_MOVING)
        {
            // Motor still moving, wait
            return;
        }

        Console.serialInfo(F("Return movement completed"));

        // Motor has stopped, verify position with tolerance
        SystemState state = captureSystemState();
        const float POSITION_WARNING_TOLERANCE = 2.0; // 2mm tolerance for warnings vs errors

        if (!isAtPosition(state.currentPositionMm, POSITION_1_MM))
        {
            // Report the error
            Console.serialError(F("Motor did not return to position 1"));

            // Calculate how far off we are from the target
            float positionError = abs(state.currentPositionMm - POSITION_1_MM);

            if (positionError <= POSITION_WARNING_TOLERANCE)
            {
                // Close enough to continue - log warning but mark operation as successful
                sprintf(messageBuffer, "Position error of %.1fmm is within tolerance - continuing", positionError);
                Console.serialWarning(messageBuffer);

                // Operation complete with warning
                Console.acknowledge(F("TRAY LOADING COMPLETE"));
                currentOperation.inProgress = false;
                currentOperation.success = true;
                strncpy(currentOperation.message, "[INFO] SUCCESS_WITH_POSITION_WARNING", sizeof(currentOperation.message));
            }
            else
            {
                // Major position error - mark as failure but still end the operation
                Console.error(F("POSITION_ERROR"));
                sprintf(messageBuffer, "Position error of %.1fmm exceeds tolerance", positionError);
                Console.serialError(messageBuffer);

                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "RETURN_FAILURE", sizeof(currentOperation.message));
            }

            // Always output load completed counter regardless of success/failure
            sprintf(messageBuffer, "Total loads completed: %d", trayTracking.totalLoadsCompleted);
            Console.serialInfo(messageBuffer);

            // CRITICAL: Always call endOperation() to clean up state
            // This ensures the system doesn't remain in "busy" state
            endOperation();

            return;
        }

        // Normal successful completion - no position error
        Console.acknowledge(F("TRAY LOADING COMPLETE"));
        currentOperation.inProgress = false;
        currentOperation.success = true;
        strncpy(currentOperation.message, "[INFO] SUCCESS", sizeof(currentOperation.message));

        sprintf(messageBuffer, "Total loads completed: %d", trayTracking.totalLoadsCompleted);
        Console.serialInfo(messageBuffer);

        // Ensure tray tracking matches sensor readings
        state = captureSystemState();
        updateTrayTrackingFromSensors(state);

        // End operation and reset target tracking
        endOperation();
    }
    break;
    }
}

// Process the tray unloading operation state machine
void processTrayUnloading()
{
    // Current time for non-blocking timing checks
    unsigned long currentMillis = millis();

    // Variables to track which tray is being unloaded
    static double sourcePosition = 0;
    static bool needsMovementToPos1 = false;

    // Single message buffer for all sprintf operations in this function
    char messageBuffer[150];

    switch (currentOperationStep)
    {
    case 0: // Initial checks and determine which tray to unload
    {
        // Capture system state to get latest sensor readings
        SystemState state = captureSystemState();

        Console.serialInfo(F("Starting tray unloading process - initial checks"));

        // Check if there are any trays in the system
        if (!state.tray1Present && !state.tray2Present && !state.tray3Present)
        {
            Console.serialError(F("No trays in system to unload"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "NO_TRAYS", sizeof(currentOperation.message));
            return;
        }

        // Determine which workflow to use for unloading
        int workflow = determineUnloadingWorkflow();
        if (workflow == 0)
        {
            // No trays to unload
            Console.serialError(F("No trays in system to unload"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "NO_TRAYS", sizeof(currentOperation.message));
            return;
        }
        else if (workflow == 1)
        {
            // Tray already at position 1, just need to unlock it
            needsMovementToPos1 = false;
            Console.serialInfo(F("Tray at position 1 ready for unloading"));
            // Skip to tray unlocking step
            updateOperationStep(11);
            return;
        }
        else if (workflow == 2)
        {
            // Need to move tray from position 2 to position 1
            sourcePosition = POSITION_2_MM;
            needsMovementToPos1 = true;
            Console.serialInfo(F("Will move tray from position 2 to position 1 for unloading"));
        }
        else if (workflow == 3)
        {
            // Need to move tray from position 3 to position 1
            sourcePosition = POSITION_3_MM;
            needsMovementToPos1 = true;
            Console.serialInfo(F("Will move tray from position 3 to position 1 for unloading"));
        }

        // If we need to move a tray, check path clearance
        if (needsMovementToPos1)
        {
            if (!isPathClearForUnloading(state.currentPositionMm, sourcePosition, state) ||
                !isPathClearForUnloading(sourcePosition, POSITION_1_MM, state))
            {
                Console.serialError(F("Path to source or target position is blocked"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "PATH_BLOCKED", sizeof(currentOperation.message));
                return;
            }

            // Start with sensor verification
            updateOperationStep(1);
            sensorVerificationStartTime = currentMillis;
        }
        else
        {
            // No movement needed, verify tray is locked at position 1
            if (!state.tray1Locked)
            {
                Console.serialError(F("Tray at position 1 not locked"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "TRAY_NOT_LOCKED", sizeof(currentOperation.message));
                return;
            }

            // Skip to tray unlocking (step 11)
            updateOperationStep(11);
        }
    }
    break;

    case 1: // Verify sensor readings for stability
    {
        // Wait for sensor verification delay
        if (!timeoutElapsed(currentMillis, sensorVerificationStartTime, SENSOR_VERIFICATION_DELAY_MS))
        {
            return;
        }

        // Double-check sensor readings after stabilization delay
        SystemState state = captureSystemState();

        // Verify that source position still has a tray
        if (sourcePosition == POSITION_2_MM && !state.tray2Present)
        {
            Console.serialError(F("Tray at position 2 disappeared during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY2_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }
        else if (sourcePosition == POSITION_3_MM && !state.tray3Present)
        {
            Console.serialError(F("Tray at position 3 disappeared during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY3_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }

        // Verify position 1 is still clear
        if (state.tray1Present)
        {
            Console.serialError(F("Position 1 unexpectedly occupied during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "POSITION1_OCCUPIED", sizeof(currentOperation.message));
            return;
        }

        // All verified, move to next step
        updateOperationStep(2);
        Console.serialInfo(F("Sensor verification complete"));
    }
    break;

    case 2: // Move to source position (2 or 3)
    {
        // Start movement to source position
        if (!moveToPositionMm(sourcePosition))
        {
            Console.serialError(F("Failed to start movement to source position"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "MOVE_FAILURE", sizeof(currentOperation.message));
            return;
        }

        sprintf(messageBuffer, "Moving to position %d", sourcePosition);
        Console.serialInfo(messageBuffer);

        // Advance to movement monitoring step
        updateOperationStep(3);
    }
    break;

    case 3: // Monitor motor movement to source position
    {
        // Check if motor is still moving
        if (motorState == MOTOR_STATE_MOVING)
        {
            // Motor still moving, wait
            return;
        }

        Console.serialInfo(F("Reached source position"));

        // Motor has stopped, verify position with tolerance
        SystemState state = captureSystemState();
        const float POSITION_WARNING_TOLERANCE = 2.0; // 2mm tolerance for warnings vs errors
        bool reachedSource = false;
        float targetPosition = 0.0;

        if (sourcePosition == POSITION_2_MM)
        {
            reachedSource = isAtPosition(state.currentPositionMm, POSITION_2_MM);
            targetPosition = POSITION_2_MM;
        }
        else if (sourcePosition == POSITION_3_MM)
        {
            reachedSource = isAtPosition(state.currentPositionMm, POSITION_3_MM);
            targetPosition = POSITION_3_MM;
        }

        if (!reachedSource)
        {
            // Report the error
            Console.serialError(F("Motor did not reach source position"));

            // Calculate how far off we are from the target
            float positionError = abs(state.currentPositionMm - targetPosition);

            if (positionError <= POSITION_WARNING_TOLERANCE)
            {
                // Close enough to continue - log warning but continue operation
                sprintf(messageBuffer, "Position error of %.2fmm is within tolerance - continuing", positionError);
                Console.serialWarning(messageBuffer);

                // Continue with the unloading sequence despite small position error
                safetyDelayStartTime = currentMillis;
                updateOperationStep(4);
            }
            else
            {
                // Major position error - mark as failure but still end the operation
                Console.error(F("SOURCE_POSITION_ERROR"));
                sprintf(messageBuffer, "Position error of %.2fmm exceeds tolerance", positionError);
                Console.serialError(messageBuffer);

                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SOURCE_POSITION_FAILURE", sizeof(currentOperation.message));

                // CRITICAL: Always call endOperation() to clean up state
                endOperation();
            }
            return;
        }

        // Add safety delay after movement
        safetyDelayStartTime = currentMillis;
        updateOperationStep(4);
    }
    break;

    case 4: // Safety delay after reaching source position
    {
        // Wait for safety delay to elapse
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_AFTER_MOVEMENT_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay after movement completed"));

        // Start locking shuttle to grip tray
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to lock shuttle to grip tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to lock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Console.serialInfo(F("Initiated shuttle lock valve actuation"));
        }
        else
        {
            Console.serialError(F("Failed to access shuttle valve"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }

        // Advance to next step
        updateOperationStep(5);
    }
    break;

    case 5: // Wait for shuttle lock valve actuation
    {
        // Wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Valve actuation time has elapsed, verify shuttle is locked
        SystemState state = captureSystemState();
        if (!state.shuttleLocked)
        {
            Console.serialError(F("Failed to lock shuttle - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Shuttle lock confirmed successful"));

        // Determine which valve to unlock based on source position
        DoubleSolenoidValve *valve = NULL;
        CylinderSensor *sensor = NULL;

        if (sourcePosition == POSITION_2_MM)
        {
            valve = getTray2Valve();
            sensor = getTray2Sensor();
            Console.serialInfo(F("Attempting to unlock tray at position 2"));
        }
        else
        {
            valve = getTray3Valve();
            sensor = getTray3Sensor();
            Console.serialInfo(F("Attempting to unlock tray at position 3"));
        }

        if (valve && sensor)
        {
            if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to unlock tray - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Console.serialInfo(F("Initiated tray unlock valve actuation"));
        }
        else
        {
            Console.serialError(F("Failed to access valve or sensor"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }

        // Advance to next step
        updateOperationStep(6);
    }
    break;

    case 6: // Wait for tray unlock valve actuation
    {
        // Wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Valve actuation time has elapsed, verify unlock
        SystemState state = captureSystemState();
        bool unlockSuccessful = false;

        if (sourcePosition == POSITION_2_MM)
        {
            unlockSuccessful = !state.tray2Locked;
        }
        else if (sourcePosition == POSITION_3_MM)
        {
            unlockSuccessful = !state.tray3Locked;
        }

        if (!unlockSuccessful)
        {
            Console.serialError(F("Failed to unlock tray - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Tray unlock confirmed successful"));

        // Add safety delay after unlocking
        safetyDelayStartTime = currentMillis;
        updateOperationStep(7);
    }
    break;

    case 7: // Safety delay after tray unlock
    {
        // Wait for safety delay to elapse
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_AFTER_UNLOCK_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay after unlock completed"));

        // Add additional safety delay before movement
        safetyDelayStartTime = currentMillis;
        updateOperationStep(8);
    }
    break;

    case 8: // Move to position 1 with the tray
    {
        // Wait for safety delay before movement
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_BEFORE_MOVEMENT_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay before movement completed"));

        // Start movement to position 1
        if (!moveToPositionMm(POSITION_1_MM))
        {
            Console.serialError(F("Failed to start movement to position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "MOVE_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Moving tray to position 1 for unloading"));

        // Advance to movement monitoring step
        updateOperationStep(9);
    }
    break;

    case 9: // Monitor motor movement to position 1
    {
        // Check if motor is still moving
        if (motorState == MOTOR_STATE_MOVING)
        {
            // Motor still moving, wait
            return;
        }

        Console.serialInfo(F("Reached position 1"));

        // Motor has stopped, verify position with tolerance
        SystemState state = captureSystemState();
        const float POSITION_WARNING_TOLERANCE = 2.0; // 2mm tolerance for warnings vs errors

        if (!isAtPosition(state.currentPositionMm, POSITION_1_MM))
        {
            // Report the error
            Console.serialError(F("Motor did not reach position 1"));

            // Calculate how far off we are from the target
            float positionError = abs(state.currentPositionMm - POSITION_1_MM);

            if (positionError <= POSITION_WARNING_TOLERANCE)
            {
                // Close enough to continue - log warning but mark operation as successful
                sprintf(messageBuffer, "Position error of %.2fmm is within tolerance - continuing", positionError);
                Console.serialWarning(messageBuffer);

                // Continue with the unloading sequence despite small position error
                safetyDelayStartTime = currentMillis;
                updateOperationStep(10);
            }
            else
            {
                // Major position error - mark as failure but still end the operation
                Console.error(F("POSITION_ERROR"));
                sprintf(messageBuffer, "Position error of %.2fmm exceeds tolerance", positionError);
                Console.serialError(messageBuffer);

                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "POSITION_FAILURE", sizeof(currentOperation.message));

                // CRITICAL: Always call endOperation() to clean up state
                // This ensures the system doesn't remain in "busy" state
                endOperation();
            }
            return;
        }

        // Add safety delay after movement
        safetyDelayStartTime = currentMillis;
        updateOperationStep(10);
    }
    break;

    case 10: // Safety delay after movement to position 1, unlock shuttle, lock tray
    {
        // Wait for safety delay after movement
        if (!timeoutElapsed(currentMillis, safetyDelayStartTime, SAFETY_DELAY_AFTER_MOVEMENT_MS))
        {
            return;
        }

        Console.serialInfo(F("Safety delay after movement completed"));

        // Unlock shuttle to release tray
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to unlock shuttle to release tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_UNLOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
            {
                Console.serialError(F("Failed to unlock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Console.serialInfo(F("Initiated shuttle unlock valve actuation"));
        }
        else
        {
            Console.serialError(F("Failed to access shuttle valve"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }

        // Wait for shuttle unlock and then proceed to next step
        updateOperationStep(11);
    }
    break;

    case 11: // Wait for shuttle unlock, then lock tray at position 1
    {
        if (needsMovementToPos1)
        {
            // For trays that were moved, wait for shuttle unlock to complete
            if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
            {
                // Not enough time has elapsed, check again next cycle
                return;
            }

            // Verify shuttle is unlocked
            SystemState state = captureSystemState();
            if (state.shuttleLocked)
            {
                Console.serialError(F("Failed to unlock shuttle - verification failed"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }

            Console.serialInfo(F("Shuttle unlock confirmed successful"));

            // Lock tray at position 1
            DoubleSolenoidValve *valve = getTray1Valve();
            CylinderSensor *sensor = getTray1Sensor();

            if (valve && sensor)
            {
                Console.serialInfo(F("Attempting to lock tray at position 1"));

                if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, VALVE_SENSOR_CONFIRMATION_TIMEOUT_MS))
                {
                    Console.serialError(F("Failed to lock tray at position 1 - sensor didn't confirm"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                valveActuationStartTime = currentMillis;
                Console.serialInfo(F("Initiated tray lock valve actuation at position 1"));
            }
            else
            {
                Console.serialError(F("Failed to access tray 1 valve or sensor"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
                return;
            }

            // Advance to next step to wait for lock confirmation
            updateOperationStep(12);
        }
        else
        {
            // For trays already at position 1, just verify it's locked
            SystemState state = captureSystemState();
            if (!state.tray1Locked)
            {
                Console.serialError(F("Tray at position 1 not locked"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "TRAY_NOT_LOCKED", sizeof(currentOperation.message));
                return;
            }

            // Skip ahead to unlock tray for removal
            updateOperationStep(13);
        }
    }
    break;

    case 12: // Wait for tray lock at position 1, update tracking
    {
        // Wait for valve actuation time to elapse
        if (!timeoutElapsed(currentMillis, valveActuationStartTime, VALVE_ACTUATION_TIME_MS))
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Verify lock
        SystemState state = captureSystemState();
        if (!state.tray1Locked)
        {
            Console.serialError(F("Failed to lock tray at position 1 - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Console.serialInfo(F("Tray lock at position 1 confirmed successful"));

        // Update tray tracking based on source position
        if (sourcePosition == POSITION_2_MM)
        {
            unloadSecondTray();
        }
        else if (sourcePosition == POSITION_3_MM)
        {
            unloadThirdTray();
        }

        // Now proceed to final unlock for removal
        updateOperationStep(13);
    }
    break;

    case 13: // NEW STEP: Notify that tray is ready for gripping (but keep it locked)
    {
        // Verify again that the tray is locked
        SystemState state = captureSystemState();
        if (!state.tray1Locked)
        {
            Console.serialError(F("Tray at position 1 not properly locked"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY_NOT_LOCKED", sizeof(currentOperation.message));
            return;
        }

        // Verify shuttle retraction
        if (state.shuttleLocked)
        {
            Console.serialError(F("Shuttle unexpectedly locked - must be retracted for robot access"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_NOT_RETRACTED", sizeof(currentOperation.message));
            return;
        }

        // All good - tray is at position 1, locked, and shuttle is retracted
        Console.acknowledge(F("TRAY_READY_FOR_GRIP"));
        Console.serialInfo(F("Tray locked at position 1 and ready for robot to grip"));

        // Operation complete - Mitsubishi will need to send tray,gripped command before removing
        currentOperation.inProgress = false;
        currentOperation.success = true;
        strncpy(currentOperation.message, "TRAY_READY_FOR_GRIP", sizeof(currentOperation.message));

        // Ensure tray tracking matches sensor readings
        updateTrayTrackingFromSensors(state);

        // End operation
        endOperation();
    }
    break;
    }
}

// Process the tray advancement operation state machine
void processTrayAdvance()
{
    // Similar implementation to processTrayLoading
    // Will be filled in later

    // For now, just finish immediately
    currentOperation.inProgress = false;
    currentOperation.success = true;
    strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));
}

// Begin an operation and track it
void beginOperation()
{
    // Store previous encoder state if we need to restore it later
    bool wasEncoderActive = encoderControlActive;

    // Disable encoder control if it's active
    if (encoderControlActive)
    {
        encoderControlActive = false;
        Console.serialInfo(F("MPG handwheel control temporarily disabled during automated operation"));
    }

    // Store the encoder state so we can restore it if needed
    operationEncoderState = wasEncoderActive;

    operationInProgress = true;
    operationStartTime = millis();
    // Use the updateOperationStep function to keep steps in sync
    updateOperationStep(0);
}

// End an operation and update target tracking
void endOperation()
{
    operationInProgress = false;

    // Update target tracking in MotorController
    lastTargetPositionMm = currentTargetPositionMm;
    lastTargetPulses = currentTargetPulses;
    hasLastTarget = true;
    hasCurrentTarget = false; // Clear current target

    // Restore encoder control if it was active before
    if (operationEncoderState)
    {
        encoderControlActive = true;
        Console.serialInfo(F("MPG handwheel control re-enabled after automated operation"));
    }

    // Clear any pending new command flag
    newCommandReceived = false;

    // Synchronize tray tracking with physical sensors
    SystemState state = captureSystemState();
    updateTrayTrackingFromSensors(state);
}

// Add near your other operation management functions (like endOperation)
const char *getAbortReasonString(AbortReason reason)
{
    switch (reason)
    {
    case ABORT_REASON_ESTOP:
        return "Emergency Stop";
    case ABORT_REASON_MOTOR_TIMEOUT:
        return "Motor Movement Timeout";
    case ABORT_REASON_OPERATION_TIMEOUT:
        return "Operation Sequence Timeout";
    case ABORT_REASON_SENSOR_MISMATCH:
        return "Unexpected Sensor Reading";
    case ABORT_REASON_COMMUNICATION_LOSS:
        return "Robot Communication Loss";
    case ABORT_REASON_PNEUMATIC_FAILURE:
        return "Insufficient Pneumatic Pressure";
    default:
        return "Unknown Reason";
    }
}

void abortOperation(AbortReason reason)
{
    // Stop any motion immediately
    MOTOR_CONNECTOR.MoveStopAbrupt();

    // Disable the motor
    MOTOR_CONNECTOR.EnableRequest(false);

    // Update motor state - use FAULTED since we're aborting
    motorState = MOTOR_STATE_FAULTED;

    // Log the abort with clear reason
    char abortMessage[128];
    sprintf(abortMessage, "[ABORT] Operation aborted: %s", getAbortReasonString(reason));
    Console.serialError(abortMessage);

    // Update operation status
    currentOperation.inProgress = false;
    currentOperation.success = false;
    strncpy(currentOperation.message, getAbortReasonString(reason), sizeof(currentOperation.message));

    // Reset operation step counter using our helper function
    updateOperationStep(0);

    // End operation and update target tracking
    endOperation();

    // Provide information about recovery
    Console.serialInfo(F("[RECOVERY] To reset system and try again, use the 'system,reset' command"));
}

// Helper function to update operation steps
void updateOperationStep(int newStep)
{
    // Update the current step
    currentOperationStep = newStep;
    // Synchronize the expected step with the current step
    expectedOperationStep = newStep;

    // For debugging
    char diagnosticMessage[64];
    sprintf(diagnosticMessage, "Operation step updated: %d", newStep);
    Console.serialDiagnostic(diagnosticMessage);
}

void resetTrayTracking()
{
    trayTracking.totalTraysInSystem = 0;
    trayTracking.position1Occupied = false;
    trayTracking.position2Occupied = false;
    trayTracking.position3Occupied = false;
    trayTracking.lastLoadTime = 0;
    trayTracking.lastUnloadTime = 0;
    trayTracking.totalLoadsCompleted = 0;
    trayTracking.totalUnloadsCompleted = 0;
}

void initSystemStateVariables()
{
    // Initialize operation state variables
    operationInProgress = false;
    newCommandReceived = false;
    currentOperationStep = 0;
    operationStartTime = 0;
    operationTimeoutMs = 60000; // 60 seconds default

    // Initialize target position tracking variables
    hasCurrentTarget = true; // This is key - it prevents the initial target position error
    hasLastTarget = false;
    currentTargetType = POSITION_1; // Set default position
    lastTargetType = POSITION_UNDEFINED;
    currentTargetPositionMm = 0.0; // Will be updated after homing
    lastTargetPositionMm = 0.0;
    currentTargetPulses = 0; // Will be updated after homing
    lastTargetPulses = 0;

    // Initialize lock/unlock failure tracking
    lastLockOperationFailed = false;
    lastUnlockOperationFailed = false;
    lastLockFailureDetails[0] = '\0';
    lastUnlockFailureDetails[0] = '\0';
    lockFailureTimestamp = 0;
    unlockFailureTimestamp = 0;

    // Initialize tray tracking state variables
    // (assuming you have a function that does this already)
    resetTrayTracking();

    Console.serialInfo(F("System state variables initialized"));
}

// Function to reset the system state after a failure
void resetSystemState()
{
    // Track overall reset success
    bool resetSuccessful = true;

    // Single message buffer for all sprintf operations in this function
    char messageBuffer[150];

    // Call the initialization function to set all variables to default values
    initSystemStateVariables();

    // End operation to update target tracking (if needed beyond what init does)
    endOperation();

    // Reset operation counters
    trayTracking.totalLoadsCompleted = 0;
    trayTracking.totalUnloadsCompleted = 0;
    trayTracking.lastLoadTime = 0;
    trayTracking.lastUnloadTime = 0;
    Console.serialInfo(F("Tray tracking state reset"));

    // Enhanced fault clearing with retry logic
    if (motorState == MOTOR_STATE_FAULTED)
    {
        // Track whether fault clearing was successful
        bool faultCleared = false;
        int clearAttempts = 0;
        const int maxClearAttempts = 3;

        // Print initial message about fault clearing attempts
        sprintf(messageBuffer, "Motor faults detected - will attempt clearing up to %d times", maxClearAttempts);
        Console.serialInfo(messageBuffer);

        while (!faultCleared && clearAttempts < maxClearAttempts)
        {
            clearAttempts++;

            // Initiate fault clearing process
            clearMotorFaults();
            sprintf(messageBuffer, "Attempt %d/%d", clearAttempts, maxClearAttempts);
            Console.serialInfo(messageBuffer);

            // Wait for fault clearing to complete
            unsigned long startTime = millis();
            unsigned long timeoutMs = 2000; // 2 second timeout

            while (isFaultClearingInProgress() && !timeoutElapsed(millis(), startTime, timeoutMs))
            {
                // Call processFaultClearing repeatedly to advance the state machine
                processFaultClearing();
                delay(10); // Small delay to avoid blocking
            }

            // Check if fault clearing timed out
            if (isFaultClearingInProgress())
            {
                Console.serialWarning(F("Fault clearing timed out"));
                // Force fault clearing process to end
                faultClearState = FAULT_CLEAR_FINISHED;
                processFaultClearing(); // Process one more time to finish
            }

            // Check if alerts were successfully cleared
            if (!MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
            {
                faultCleared = true;
                Console.serialInfo(F("[SUCCESS] Motor faults cleared successfully"));
                break; // Exit the retry loop
            }
            else
            {
                Console.serialWarning(F("Faults still present after clearing attempt"));
                printMotorAlerts();
                // Add a brief delay between attempts
                delay(500);
            }
        }

        if (!faultCleared)
        {
            Console.serialError(F("Failed to clear motor faults - Try 'motor,clear' command or cycle system power"));
            resetSuccessful = false; // Mark reset as unsuccessful
        }
    }

    // Only try to re-enable motor if fault clearing was successful or no faults existed
    if (resetSuccessful && !MOTOR_CONNECTOR.EnableRequest() && !isEStopActive())
    {
        MOTOR_CONNECTOR.EnableRequest(true);
        Console.serialInfo(F("Re-enabling motor"));
    }

    // Update motor state if not currently faulted (alerts check is already in the condition)
    if (resetSuccessful && motorState == MOTOR_STATE_FAULTED && !MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        motorState = MOTOR_STATE_IDLE;
        Console.serialInfo(F("Motor state reset to IDLE"));
    }

    // Always update tray tracking regardless of motor state
    SystemState state = captureSystemState();
    updateTrayTrackingFromSensors(state);
    Console.serialInfo(F("Tray tracking synchronized with sensors"));

    // Log the reset action with appropriate status
    if (resetSuccessful)
    {
        Console.serialInfo(F("[SUCCESS] System state has been fully reset"));
        Console.acknowledge(F("SYSTEM_RESET_COMPLETE"));
    }
    else
    {
        Console.serialWarning(F("System reset partially completed - motor faults persist"));
        Console.error(F("MOTOR_FAULT_PERSIST"));
    }
}

void resetLockUnlockFailures()
{
    lastLockOperationFailed = false;
    lastUnlockOperationFailed = false;
    lastLockFailureDetails[0] = '\0';
    lastUnlockFailureDetails[0] = '\0';
}