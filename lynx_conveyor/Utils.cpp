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
const unsigned long VALVE_ACTUATION_TIME_MS = 500;          // Increased from 500ms for more reliable actuation (previosly 750ms)
const unsigned long SAFETY_DELAY_AFTER_UNLOCK_MS = 500;    // Safety delay after unlocking a tray (previously 1000ms)
const unsigned long SAFETY_DELAY_BEFORE_MOVEMENT_MS = 500; // Safety delay before starting motor movement (previously 1000ms)
const unsigned long SAFETY_DELAY_AFTER_MOVEMENT_MS = 500;  // Safety delay after motor has completed movement (previously 1000ms)
const unsigned long SENSOR_VERIFICATION_DELAY_MS = 200;     // Delay for stable sensor readings

// Tray status structure
TrayStatus trayStatus = {false, false, false, 0, OPERATION_NONE};

// Current operation structure
OperationStatus currentOperation = {false, OPERATION_NONE, 0, 0, false, ""};

// Add this variable definition near your other global variables:
SystemState previousState;

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
    Console.println(F("[DIAGNOSTIC] System State:"));

    // Motor
    Console.print(F("  Motor: "));
    switch (state.motorState)
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
        Console.println(F("NOT_READY"));
        break;
    }

    Console.print(F("  Homed: "));
    Console.println(state.isHomed ? F("YES") : F("NO"));

    Console.print(F("  Position: "));
    if (state.isHomed)
    {
        Console.print(state.currentPositionMm);
        Console.println(F(" mm"));
    }
    else
    {
        Console.println(F("UNKNOWN"));
    }

    Console.print(F("  HLFB Status: "));
    switch (state.hlfbStatus)
    {
    case MotorDriver::HLFB_ASSERTED:
        Console.println(F("ASSERTED (In Position/Ready)"));
        break;
    case MotorDriver::HLFB_DEASSERTED:
        Console.println(F("DEASSERTED (Moving/Fault)"));
        break;
    case MotorDriver::HLFB_UNKNOWN:
    default:
        Console.println(F("UNKNOWN"));
        break;
    }

    // Cylinder sensors (raw readings)
    Console.println(F("\n  Cylinder Sensors:"));
    Console.print(F("    Tray 1: "));
    Console.println(state.tray1CylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    Console.print(F("    Tray 2: "));
    Console.println(state.tray2CylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    Console.print(F("    Tray 3: "));
    Console.println(state.tray3CylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    Console.print(F("    Shuttle: "));
    Console.println(state.shuttleCylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    // Lock states (derived from sensor readings)
    Console.println(F("\n  Lock States:"));
    Console.print(F("    Tray 1: "));
    Console.println(state.tray1Locked ? F("LOCKED") : F("UNLOCKED"));

    Console.print(F("    Tray 2: "));
    Console.println(state.tray2Locked ? F("LOCKED") : F("UNLOCKED"));

    Console.print(F("    Tray 3: "));
    Console.println(state.tray3Locked ? F("LOCKED") : F("UNLOCKED"));

    Console.print(F("    Shuttle: "));
    Console.println(state.shuttleLocked ? F("LOCKED") : F("UNLOCKED"));

    // Tray presence detection
    Console.println(F("\n  Tray Detection:"));
    Console.print(F("    Position 1: "));
    Console.println(state.tray1Present ? F("TRAY PRESENT") : F("NO TRAY"));

    Console.print(F("    Position 2: "));
    Console.println(state.tray2Present ? F("TRAY PRESENT") : F("NO TRAY"));

    Console.print(F("    Position 3: "));
    Console.println(state.tray3Present ? F("TRAY PRESENT") : F("NO TRAY"));

    // Safety systems
    Console.println(F("\n  Safety Systems:"));
    Console.print(F("    E-Stop: "));
    Console.println(state.eStopActive ? F("ACTIVE (Emergency Stop)") : F("INACTIVE (Normal Operation)"));

    // Hardware status
    Console.println(F("\n  Hardware Status:"));
    Console.print(F("    CCIO Board: "));
    Console.println(state.ccioBoardPresent ? F("PRESENT") : F("NOT DETECTED"));

    // network status
    Console.print(F("    Network Clients: "));
    Console.print(getConnectedClientCount());
    Console.println(F(" connected"));

    // Pneumatic system status
    Console.print(F("    Pneumatic System: "));
    float pressure = getPressurePsi();
    Console.print(pressure);
    Console.print(F(" PSI "));
    if (pressure < MIN_SAFE_PRESSURE)
    {
        Console.println(F("(INSUFFICIENT)"));
    }
    else
    {
        Console.println(F("(OK)"));
    }

    // summary of critical safety conditions
    Console.println(F("\n  Safety Summary:"));

    // Check if any tray is locked while motor is moving
    bool unsafeMotion = state.motorState == MOTOR_STATE_MOVING &&
                        (state.tray1Locked || state.tray2Locked || state.tray3Locked);
    Console.print(F("    Safe Motion: "));
    Console.println(unsafeMotion ? F("NO - TRAYS LOCKED DURING MOTION") : F("YES"));

    // Check for missing trays that are locked
    bool missingTraysLocked = (state.tray1Locked && !state.tray1Present) ||
                              (state.tray2Locked && !state.tray2Present) ||
                              (state.tray3Locked && !state.tray3Present);
    Console.print(F("    Tray/Lock Mismatch: "));
    Console.println(missingTraysLocked ? F("YES - LOCK WITHOUT TRAY") : F("NO"));

    // Encoder status
    Console.println(F("\n  MPG Handwheel:"));
    Console.print(F("    Status: "));
    Console.println(encoderControlActive ? F("ENABLED") : F("DISABLED"));

    if (encoderControlActive)
    {
        Console.print(F("    Multiplier: x"));
        Console.print(getMultiplierName(currentMultiplier));
        Console.print(F(" ("));

        // Calculate how much one full rotation moves (100 pulses typical for MPG handwheels)
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        Console.print(mmPerRotation, 2);
        Console.println(F(" mm/rotation)"));
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
        result.pressureUnsafeReason = F("Pneumatic pressure below minimum threshold");

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
    // Prevents movement if any tray is locked, which could damage hardware
    // if (state.tray1Locked || state.tray2Locked || state.tray3Locked)
    // {
    //     result.safeToMove = false;
    //     result.moveUnsafeReason = F("Tray locks engaged");
    //     // This is a prerequisite safety check, not an abort condition
    // }

    // Check which specific position has a locked tray that's blocking movement
    if (isAtPosition(state.currentPositionMm, POSITION_1_MM) && state.tray1Locked)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Cannot move - Tray at position 1 is locked");
    }
    else if (isAtPosition(state.currentPositionMm, POSITION_2_MM) && state.tray2Locked)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Cannot move - Tray at position 2 is locked");
    }
    else if (isAtPosition(state.currentPositionMm, POSITION_3_MM) && state.tray3Locked)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Cannot move - Tray at position 3 is locked");
    }

    // System State Requirements
    // Motor must be homed before movement to ensure position accuracy
    if (!state.isHomed)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Motor not homed");
        // This is a prerequisite safety check, not an abort condition
    }

    // Emergency Conditions
    // E-stop immediately prevents all movement and triggers abort
    if (state.eStopActive)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("E-stop active");
        result.failureReason = ABORT_REASON_ESTOP; // E-stop is an immediate abort condition
    }

    // Hardware Presence Verification
    // CCIO board must be present for safe I/O operations
    if (!state.ccioBoardPresent)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("CCIO board not detected");
        // No abort reason - this is a hardware presence check
    }

    // Motor Fault Detection
    // Prevents movement when motor is in fault state
    if (state.motorState == MOTOR_STATE_FAULTED)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Motor in fault state");
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
        result.tray1LockUnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray2Present)
    {
        result.safeToLockTray2 = false;
        result.tray2LockUnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray3Present)
    {
        result.safeToLockTray3 = false;
        result.tray3LockUnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    // Movement Status Safety
    // Cannot lock trays while motor is moving
    if (state.motorState == MOTOR_STATE_MOVING)
    {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        result.tray1LockUnsafeReason = F("Motor is moving");
        result.tray2LockUnsafeReason = F("Motor is moving");
        result.tray3LockUnsafeReason = F("Motor is moving");

        // Sequence validation: Detect unexpected movement during lock/unlock steps
        // For tray loading, steps 0-7 involve lock/unlock ops, while 8+ are for movement
        if (operationInProgress &&
            (previousState.motorState != MOTOR_STATE_MOVING) &&
            ((currentOperation.type == OPERATION_LOADING && currentOperationStep < 8) ||
             (currentOperation.type == OPERATION_UNLOADING && currentOperationStep < 3)))
        {
            // This means movement started during a lock/unlock operation - sequence violation
            result.operationSequenceValid = false;

            // Provide detailed error message with operation-specific context
            result.operationSequenceMessage = F("Motor unexpectedly started moving during ");

            // Add specific step information for clearer diagnostics
            if (currentOperation.type == OPERATION_LOADING)
            {
                switch (currentOperationStep)
                {
                case 0:
                    result.operationSequenceMessage += F("initial tray load preparation");
                    break;
                case 1:
                    result.operationSequenceMessage += F("tray detection verification");
                    break;
                case 2:
                    result.operationSequenceMessage += F("shuttle locking");
                    break;
                case 3:
                    result.operationSequenceMessage += F("shuttle lock verification");
                    break;
                case 4:
                    result.operationSequenceMessage += F("shuttle unlocking");
                    break;
                case 5:
                    result.operationSequenceMessage += F("tray locking");
                    break;
                case 6:
                    result.operationSequenceMessage += F("tray lock verification");
                    break;
                case 7:
                    result.operationSequenceMessage += F("pre-movement preparation");
                    break;
                default:
                    result.operationSequenceMessage += F("lock operation");
                    break;
                }
            }
            else
            {
                switch (currentOperationStep)
                {
                case 0:
                    result.operationSequenceMessage += F("tray unload preparation");
                    break;
                case 1:
                    result.operationSequenceMessage += F("sensor verification");
                    break;
                case 2:
                    result.operationSequenceMessage += F("movement to source position");
                    break;
                case 3:
                    result.operationSequenceMessage += F("movement monitoring");
                    break;
                case 4:
                    result.operationSequenceMessage += F("shuttle locking");
                    break;
                }
            }

            result.operationSequenceMessage += F(" (step ");
            result.operationSequenceMessage += String(currentOperationStep);
            result.operationSequenceMessage += F(")");

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
        result.tray1LockUnsafeReason = F("Shuttle is locked");
        result.tray2LockUnsafeReason = F("Shuttle is locked");
        result.tray3LockUnsafeReason = F("Shuttle is locked");

        // Sequence validation: Detect unexpected shuttle locking
        // We expect the shuttle to lock during specific operation steps only
        if (operationInProgress &&
            !previousState.shuttleLocked && state.shuttleLocked &&
            (currentOperation.type != OPERATION_LOADING ||
             (currentOperationStep != 2 && currentOperationStep != 3)))
        {
            // This means shuttle was locked unexpectedly outside the expected step
            result.operationSequenceValid = false;

            // Provide detailed error message with operation context
            result.operationSequenceMessage = F("Shuttle unexpectedly locked during ");

            // Add operation-specific context for clearer diagnostics
            if (currentOperation.type == OPERATION_LOADING)
            {
                result.operationSequenceMessage += F("tray loading operation at step ");
                result.operationSequenceMessage += String(currentOperationStep);
                result.operationSequenceMessage += F(" (expected at steps 2-3)");
            }
            else if (currentOperation.type == OPERATION_UNLOADING)
            {
                result.operationSequenceMessage += F("tray unloading operation at step ");
                result.operationSequenceMessage += String(currentOperationStep);

                // Provide specific guidance about when shuttle should be locked during unloading
                if (currentOperationStep < 3)
                {
                    result.operationSequenceMessage += F(" (expected at steps 4-5)");
                }
                else if (currentOperationStep == 4 || currentOperationStep == 5)
                {
                    // This is a valid step for shuttle locking during unloading
                    result.operationSequenceValid = true; // Override the error
                    return result;                        // Skip setting failure reason
                }
                else
                {
                    result.operationSequenceMessage += F(" (unexpected at this step)");
                }
            }
            else
            {
                result.operationSequenceMessage += F("operation at unexpected step");
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
        result.loadTrayPos1UnsafeReason = F("Position already occupied");
        // No abort reason - this is a prerequisite check
    }

    if (state.tray2Present)
    {
        result.safeToLoadTrayToPos2 = false;
        result.loadTrayPos2UnsafeReason = F("Position already occupied");
        // No abort reason - this is a prerequisite check
    }

    if (state.tray3Present)
    {
        result.safeToLoadTrayToPos3 = false;
        result.loadTrayPos3UnsafeReason = F("Position already occupied");
        // No abort reason - this is a prerequisite check
    }

    // System Capacity Constraint
    // Cannot load more than 3 trays into the system
    if (state.tray1Present && state.tray2Present && state.tray3Present)
    {
        result.safeToLoadTrayToPos1 = false;
        result.safeToLoadTrayToPos2 = false;
        result.safeToLoadTrayToPos3 = false;
        result.loadTrayPos1UnsafeReason = F("All positions occupied");
        result.loadTrayPos2UnsafeReason = F("All positions occupied");
        result.loadTrayPos3UnsafeReason = F("All positions occupied");
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
        result.unloadTrayPos1UnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray2Present)
    {
        result.safeToUnloadTrayFromPos2 = false;
        result.unloadTrayPos2UnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray3Present)
    {
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos3UnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    // FILO (First-In-Last-Out) Sequence Enforcement
    // Trays must be unloaded in reverse order of loading

    // Position 1 must be unloaded first (most recently loaded)
    if (state.tray1Present)
    {
        result.safeToUnloadTrayFromPos2 = false;
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos2UnsafeReason = F("Tray 1 must be unloaded first");
        result.unloadTrayPos3UnsafeReason = F("Tray 1 must be unloaded first");
        // No abort reason - this is a prerequisite check
    }

    // Position 2 must be unloaded before position 3
    if (state.tray2Present && !state.tray1Present)
    {
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos3UnsafeReason = F("Tray 2 must be unloaded first");
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
        result.grippedTrayUnlockUnsafeReason = F("No tray at position 1");
        // No abort reason - this is a prerequisite check
    }

    // Tray lock check - must be locked before unlocking
    if (!state.tray1Locked)
    {
        result.safeToUnlockGrippedTray = false;
        result.grippedTrayUnlockUnsafeReason = F("Tray not locked");
        // No abort reason - this is a prerequisite check
    }

    // Shuttle retraction check - shuttle must be retracted for robot access
    if (state.shuttleLocked)
    {
        result.safeToUnlockGrippedTray = false;
        result.grippedTrayUnlockUnsafeReason = F("Shuttle must be retracted");
        // No abort reason - this is a prerequisite check
    }

    // Operation state check - can't unlock during active operations
    if (operationInProgress)
    {
        result.safeToUnlockGrippedTray = false;
        result.grippedTrayUnlockUnsafeReason = F("Operation in progress");
        // No abort reason - this is a prerequisite check
    }

    // Motor movement check - can't unlock while motor is moving
    if (state.motorState == MOTOR_STATE_MOVING)
    {
        result.safeToUnlockGrippedTray = false;
        result.grippedTrayUnlockUnsafeReason = F("Motor is moving");
        // No abort reason - this is a prerequisite check
    }

    // E-Stop check - can't unlock when E-Stop is active
    if (state.eStopActive)
    {
        result.safeToUnlockGrippedTray = false;
        result.grippedTrayUnlockUnsafeReason = F("E-stop active");
        // This is consistent with other E-stop handling
    }

    // Pneumatic pressure check - can't unlock without sufficient pressure
    if (!result.pneumaticPressureSufficient)
    {
        result.safeToUnlockGrippedTray = false;
        result.grippedTrayUnlockUnsafeReason = F("Insufficient pneumatic pressure");
        // This uses the existing pneumatic pressure validation
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
            result.stateValidationMessage = F("Position mismatch: current position ");
            result.stateValidationMessage += String(state.currentPositionMm);
            result.stateValidationMessage += F(" mm vs. commanded ");
            result.stateValidationMessage += String(commandedPositionMm);
            result.stateValidationMessage += F(" mm (diff: ");
            result.stateValidationMessage += String(abs(state.currentPositionMm - commandedPositionMm));
            result.stateValidationMessage += F(" mm)");

            // Add motor state context for better diagnostics
            if (state.motorState == MOTOR_STATE_MOVING)
            {
                result.stateValidationMessage += F(" - Motor still moving");
            }
            else if (state.motorState == MOTOR_STATE_FAULTED)
            {
                result.stateValidationMessage += F(" - Motor in FAULT state");
            }
            else
            {
                result.stateValidationMessage += F(" - Motor stopped before reaching target");
            }

            // Set abort reason during operations - motor timeout or blockage
            if (operationInProgress)
            {
                result.failureReason = ABORT_REASON_MOTOR_TIMEOUT;
                result.operationSequenceValid = false;

                // Add operation context for more detailed error reporting
                result.stateValidationMessage += F(" during ");
                if (currentOperation.type == OPERATION_LOADING)
                {
                    result.stateValidationMessage += F("loading operation");
                }
                else if (currentOperation.type == OPERATION_UNLOADING)
                {
                    result.stateValidationMessage += F("unloading operation");
                }
                else
                {
                    result.stateValidationMessage += F("operation");
                }
                result.stateValidationMessage += F(" step ");
                result.stateValidationMessage += String(currentOperationStep);

                result.operationSequenceMessage = result.stateValidationMessage;
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
            result.stateValidationMessage = F("ERROR: Expected tray at position 1 is missing (Motor at ");
            result.stateValidationMessage += String(state.currentPositionMm);
            result.stateValidationMessage += F(" mm)");

            // Add operation context and set failure reason
            if (operationInProgress)
            {
                // Copy message to operation sequence for consistency
                result.operationSequenceMessage = result.stateValidationMessage;
                result.operationSequenceValid = false;
                result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
            }
        }
    }

    // Target Position Validation
    // Verifies that commanded target position is within allowed range
    bool skipTargetValidation = (operationInProgress &&
                                 currentOperation.type == OPERATION_UNLOADING &&
                                 (currentOperationStep >= 4 && currentOperationStep <= 7));

    if (hasCurrentTarget)
    {
        if (currentTargetPositionMm < 0 || currentTargetPositionMm > MAX_TRAVEL_MM)
        {
            result.targetPositionValid = false;
            result.stateValidationMessage = F("Target position out of range");
            result.operationSequenceMessage = result.stateValidationMessage;
        }
    }
    else if (!skipTargetValidation) // Only fail if validation isn't temporarily disabled
    {
        // Only a problem if we expect a target position
        result.targetPositionValid = false;
        result.stateValidationMessage = F("No target position set");
        result.operationSequenceMessage = result.stateValidationMessage;
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
            result.operationSequenceMessage = F("Operation in progress, cannot accept new command");
            result.operationSequenceMessage += F(" (use 'system,reset' after failure)");
        }
    }

    // Operation Timeout Detection
    // Detects and reports operations that exceed their timeout
    if (operationInProgress && millis() - operationStartTime > operationTimeoutMs)
    {
        result.operationWithinTimeout = false;
        result.operationSequenceMessage = F("Operation exceeded timeout");
        result.failureReason = ABORT_REASON_OPERATION_TIMEOUT;
    }

    // Operation Step Sequence Validation
    // Verifies that operation steps are executed in correct sequence
    if (operationInProgress && currentOperationStep != expectedOperationStep)
    {
        result.operationSequenceValid = false;

        // Create detailed error message with operation context
        result.operationSequenceMessage = F("Operation sequence mismatch: ");

        // Add operation-specific type information
        switch (currentOperation.type)
        {
        case OPERATION_LOADING:
            result.operationSequenceMessage += F("Tray loading operation");
            break;
        case OPERATION_UNLOADING:
            result.operationSequenceMessage += F("Tray unloading operation");
            break;
        default:
            result.operationSequenceMessage += F("Current operation");
            break;
        }

        // Add step information for debugging
        result.operationSequenceMessage += F(" at step ");
        result.operationSequenceMessage += String(currentOperationStep);
        result.operationSequenceMessage += F(" (expected: ");
        result.operationSequenceMessage += String(expectedOperationStep);
        result.operationSequenceMessage += F(")");

        result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
    }

    return result;
}

// Function to print out safety validation results
void printSafetyStatus(const SafetyValidationResult &result)
{
    Console.println(F("[SAFETY] Validation Results:"));

    // Movement safety
    Console.print(F("  Motor Movement: "));
    if (result.safeToMove)
    {
        Console.println(F("SAFE - System ready for movement"));
    }
    else
    {
        Console.print(F("UNSAFE - "));
        Console.println(result.moveUnsafeReason);
    }

    // Pneumatic pressure safety status
    Console.print(F("  Pneumatic System: "));
    if (result.pneumaticPressureSufficient)
    {
        Console.print(F("SAFE - "));
        Console.print(getPressurePsi());
        Console.println(F(" PSI (sufficient pressure for valve operations)"));
    }
    else
    {
        Console.print(F("UNSAFE - "));
        Console.println(result.pressureUnsafeReason);
        Console.print(F("    Current pressure: "));
        Console.print(getPressurePsi());
        Console.print(F(" PSI, Minimum required: "));
        Console.println(MIN_SAFE_PRESSURE);
    }

    // Tray locking safety with enhanced safety messages
    Console.println(F("  Tray Locking:"));
    Console.print(F("    Tray 1: "));
    if (result.safeToLockTray1)
    {
        // Enhanced message with contextual information
        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            Console.println(F("SAFE TO LOCK - Part of loading sequence"));
        }
        else if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            Console.println(F("SAFE TO LOCK - Part of unloading sequence"));
        }
        else
        {
            Console.println(F("SAFE TO LOCK - Tray present and system ready"));
        }
    }
    else
    {
        Console.print(F("UNSAFE TO LOCK - "));
        Console.println(result.tray1LockUnsafeReason);
    }

    Console.print(F("    Tray 2: "));
    if (result.safeToLockTray2)
    {
        // Enhanced message with contextual information
        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            Console.println(F("SAFE TO LOCK - Part of loading sequence"));
        }
        else if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            Console.println(F("SAFE TO LOCK - Part of unloading sequence"));
        }
        else
        {
            Console.println(F("SAFE TO LOCK - Tray present and system ready"));
        }
    }
    else
    {
        Console.print(F("UNSAFE TO LOCK - "));
        Console.println(result.tray2LockUnsafeReason);
    }

    Console.print(F("    Tray 3: "));
    if (result.safeToLockTray3)
    {
        // Enhanced message with contextual information
        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            Console.println(F("SAFE TO LOCK - Part of loading sequence"));
        }
        else if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            Console.println(F("SAFE TO LOCK - Part of unloading sequence"));
        }
        else
        {
            Console.println(F("SAFE TO LOCK - Tray present and system ready"));
        }
    }
    else
    {
        Console.print(F("UNSAFE TO LOCK - "));
        Console.println(result.tray3LockUnsafeReason);
    }

    // Shuttle actuation safety
    Console.println(F("  Shuttle Control:"));
    Console.print(F("    Lock: "));
    if (result.safeToLockShuttle)
    {
        // Enhanced shuttle lock message
        if (operationInProgress && currentOperation.type == OPERATION_LOADING &&
            (currentOperationStep == 2 || currentOperationStep == 3))
        {
            Console.println(F("SAFE - Part of tray loading sequence"));
        }
        else
        {
            Console.println(F("SAFE - System ready for shuttle locking"));
        }
    }
    else
    {
        Console.print(F("UNSAFE - "));
        Console.println(result.shuttleLockUnsafeReason);
    }
    Console.print(F("    Unlock: "));
    if (result.safeToUnlockShuttle)
    {
        // Enhanced shuttle unlock message
        if (operationInProgress && currentOperation.type == OPERATION_LOADING &&
            (currentOperationStep == 4 || currentOperationStep == 5))
        {
            Console.println(F("SAFE - Part of tray loading sequence"));
        }
        else
        {
            Console.println(F("SAFE - System ready for shuttle unlocking"));
        }
    }
    else
    {
        Console.print(F("UNSAFE - "));
        Console.println(result.shuttleUnlockUnsafeReason);
    }

    // System state validation status with enhanced messages
    Console.println(F("\n  System State Validation:"));
    Console.print(F("    Command/Actual State: "));
    if (result.commandStateValid)
    {
        Console.println(F("VALID - System position matches commanded position"));
    }
    else
    {
        Console.print(F("INVALID - "));
        Console.println(result.stateValidationMessage);
    }

    Console.print(F("    Tray Positions: "));
    if (result.trayPositionValid)
    {
        Console.println(F("VALID - Tray presence matches expected positions"));
    }
    else
    {
        Console.print(F("INVALID - "));
        Console.println(result.stateValidationMessage);
    }

    Console.print(F("    Target Position: "));
    if (result.targetPositionValid)
    {
        if (hasCurrentTarget)
        {
            Console.print(F("VALID - Target position "));
            Console.print(currentTargetPositionMm);
            Console.println(F(" mm is within safe range"));
        }
        else
        {
            Console.println(F("VALID - No target position currently set"));
        }
    }
    else
    {
        Console.print(F("INVALID - "));
        Console.println(result.stateValidationMessage);
    }

    // Operational sequence validation with enhanced messages
    Console.println(F("\n  Operational Sequence:"));
    Console.print(F("    Accept New Commands: "));
    if (result.safeToAcceptNewCommand)
    {
        if (operationInProgress)
        {
            Console.print(F("SAFE - Current operation: "));

            // Show current operation type and step for context
            switch (currentOperation.type)
            {
            case OPERATION_LOADING:
                Console.print(F("LOADING"));
                break;
            case OPERATION_UNLOADING:
                Console.print(F("UNLOADING"));
                break;
            default:
                Console.print(F("OTHER"));
                break;
            }
            Console.print(F(" (Step "));
            Console.print(currentOperationStep);
            Console.println(F(")"));
        }
        else
        {
            Console.println(F("SAFE - No operation in progress"));
        }
    }
    else
    {
        Console.print(F("UNSAFE - "));
        Console.println(result.operationSequenceMessage);
    }

    Console.print(F("    Operation Timing: "));
    if (result.operationWithinTimeout)
    {
        if (operationInProgress)
        {
            // Show elapsed time information for active operations
            unsigned long elapsedTime = millis() - operationStartTime;
            Console.print(F("WITHIN TIMEOUT - Elapsed: "));
            Console.print(elapsedTime / 1000);
            Console.print(F("."));
            Console.print((elapsedTime % 1000) / 100);
            Console.print(F("s / "));
            Console.print(operationTimeoutMs / 1000);
            Console.println(F("s"));
        }
        else
        {
            Console.println(F("WITHIN TIMEOUT - No operation active"));
        }
    }
    else
    {
        Console.print(F("TIMEOUT - "));
        Console.println(result.operationSequenceMessage);
    }
    Console.print(F("    Operation Sequence: "));
    if (result.operationSequenceValid)
    {
        if (operationInProgress)
        {
            Console.print(F("VALID - Current step: "));
            Console.print(currentOperationStep);
            if (currentOperationStep == expectedOperationStep)
            {
                Console.println(F(" (as expected)"));
            }
            else
            {
                Console.print(F(" (expected: "));
                Console.print(expectedOperationStep);
                Console.println(F(")"));
            }
        }
        else
        {
            Console.println(F("VALID - No operation in progress"));
        }
    }
    else
    {
        Console.print(F("INVALID - "));
        Console.println(result.operationSequenceMessage);

        // Add recovery guidance when appropriate
        if (result.operationSequenceMessage.indexOf(F("sequence mismatch")) >= 0)
        {
            Console.println(F("            Use 'system,reset' to reset the system"));
        }
    }

    // Add detailed tray loading safety information
    Console.println(F("\n  Tray Loading Operations:"));
    Console.print(F("    Position 1: "));
    if (result.safeToLoadTrayToPos1)
    {
        Console.print(F("SAFE TO LOAD"));
        // Add context about tray loading sequence
        if (trayTracking.totalTraysInSystem == 0)
        {
            Console.println(F(" - Ready for first tray"));
        }
        else if (trayTracking.totalTraysInSystem == 1)
        {
            Console.println(F(" - Ready for second tray"));
        }
        else if (trayTracking.totalTraysInSystem == 2)
        {
            Console.println(F(" - Ready for third tray"));
        }
        else
        {
            Console.println(F(" - Ready for tray"));
        }
    }
    else
    {
        Console.print(F("UNSAFE TO LOAD - "));
        Console.println(result.loadTrayPos1UnsafeReason);
    }

    Console.print(F("    Position 2: "));
    if (result.safeToLoadTrayToPos2)
    {
        Console.println(F("SAFE TO LOAD - Direct loading possible"));
    }
    else
    {
        Console.print(F("UNSAFE TO LOAD - "));
        Console.println(result.loadTrayPos2UnsafeReason);
    }

    Console.print(F("    Position 3: "));
    if (result.safeToLoadTrayToPos3)
    {
        Console.println(F("SAFE TO LOAD - Direct loading possible"));
    }
    else
    {
        Console.print(F("UNSAFE TO LOAD - "));
        Console.println(result.loadTrayPos3UnsafeReason);
    }

    // Add detailed tray unloading safety information
    Console.println(F("\n  Tray Unloading Operations:"));
    Console.print(F("    Position 1: "));
    if (result.safeToUnloadTrayFromPos1)
    {
        Console.println(F("SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        Console.print(F("UNSAFE TO UNLOAD - "));
        Console.println(result.unloadTrayPos1UnsafeReason);
    }

    Console.print(F("    Position 2: "));
    if (result.safeToUnloadTrayFromPos2)
    {
        Console.println(F("SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        Console.print(F("UNSAFE TO UNLOAD - "));
        Console.println(result.unloadTrayPos2UnsafeReason);
    }

    Console.print(F("    Position 3: "));
    if (result.safeToUnloadTrayFromPos3)
    {
        Console.println(F("SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        Console.print(F("UNSAFE TO UNLOAD - "));
        Console.println(result.unloadTrayPos3UnsafeReason);
    }

    // Add overall system status at the end
    Console.println(F("\n  System Summary:"));
    if (result.operationSequenceValid && result.trayPositionValid && result.commandStateValid)
    {
        Console.println(F("    Status: NORMAL - System operating correctly"));

        if (operationInProgress)
        {
            Console.print(F("    Current Operation: "));
            // Show current operation type for context
            switch (currentOperation.type)
            {
            case OPERATION_LOADING:
                Console.print(F("LOADING"));
                break;
            case OPERATION_UNLOADING:
                Console.print(F("UNLOADING"));
                break;
            default:
                Console.print(F("OTHER"));
                break;
            }
            Console.print(F(" (Step "));
            Console.print(currentOperationStep);
            Console.println(F(")"));
        }
    }
    else
    {
        Console.println(F("    Status: ALERT - System requires attention"));

        if (!result.operationSequenceValid)
        {
            Console.println(F("    Reason: Operation sequence error detected"));
            Console.print(F("            "));
            Console.println(result.operationSequenceMessage);
        }
        else if (!result.trayPositionValid)
        {
            Console.println(F("    Reason: Tray position error detected"));
            Console.print(F("            "));
            Console.println(result.stateValidationMessage);
        }
        else if (!result.commandStateValid)
        {
            Console.println(F("    Reason: Motor position error detected"));
            Console.print(F("            "));
            Console.println(result.stateValidationMessage);
        }

        // Add specific recovery guidance based on detected issues
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
    if (currentTime - currentOperation.startTime > operationTimeoutMs)
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
        if (currentMillis - sensorVerificationStartTime < SENSOR_VERIFICATION_DELAY_MS)
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

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_LOCK, 1000))
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
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
        {
            // Not enough time has elapsed, return and check again next cycle
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

            if (!safeValveOperation(*valve, *getTray1Sensor(), VALVE_POSITION_UNLOCK, 1000))
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
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
        {
            // Not enough time has elapsed, return and check again next cycle
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_AFTER_UNLOCK_MS)
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_BEFORE_MOVEMENT_MS)
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

        Serial.print(F("[INFO] Moving tray to position "));
        Serial.println(targetPosition);
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

        // Motor has stopped, verify position
        SystemState state = captureSystemState();
        bool reachedTarget = false;

        if (targetPosition == POSITION_2_MM)
        {
            reachedTarget = isAtPosition(state.currentPositionMm, POSITION_2_MM);
        }
        else if (targetPosition == POSITION_3_MM)
        {
            reachedTarget = isAtPosition(state.currentPositionMm, POSITION_3_MM);
        }

        if (!reachedTarget)
        {
            Console.serialError(F("Motor did not reach target position"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "POSITION_FAILURE", sizeof(currentOperation.message));
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_AFTER_MOVEMENT_MS)
        {
            return;
        }

        Console.serialInfo(F("Safety delay after movement completed"));

        // Unlock shuttle now that we've reached the destination
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to unlock shuttle to release tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_UNLOCK, 1000))
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
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
        {
            // Not enough time has elapsed, return and check again next cycle
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_AFTER_UNLOCK_MS)
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
            if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
            {
                Console.serialError(F("Failed to lock tray at target position - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.print(F("[INFO] Initiated tray lock valve actuation at position "));
            Serial.println(targetPosition);
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
            Serial.println(F("[INFO] Tray loading at position 1 completed successfully"));
            currentOperation.inProgress = false;
            currentOperation.success = true;
            strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));

            // End operation and reset target tracking
            endOperation();
            return;
        }

        // For moved trays, wait for valve actuation time to elapse
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
        {
            // Not enough time has elapsed, return and check again next cycle
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_BEFORE_MOVEMENT_MS)
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
            strncpy(currentOperation.message, "[ERROR] RETURN_MOVE_FAILURE", sizeof(currentOperation.message));
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

        // Motor has stopped, verify position
        SystemState state = captureSystemState();
        if (!isAtPosition(state.currentPositionMm, POSITION_1_MM))
        {
            Console.serialError(F("Motor did not return to position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "[ERROR] RETURN_FAILURE", sizeof(currentOperation.message));
            return;
        }

        // Operation complete
        Console.acknowledge(F("TRAY LOADING COMPLETE"));
        currentOperation.inProgress = false;
        currentOperation.success = true;
        strncpy(currentOperation.message, "[INFO] SUCCESS", sizeof(currentOperation.message));

        // KEEP THE DEBUG OUTPUT BUT GET THE VALUE FROM THE TRACKING STRUCT:
        Serial.print(F("[INFO] Total loads completed: "));
        Serial.println(trayTracking.totalLoadsCompleted);

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
        if (currentMillis - sensorVerificationStartTime < SENSOR_VERIFICATION_DELAY_MS)
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

        Serial.print(F("[INFO] Moving to position "));
        Serial.println(sourcePosition);

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

        // Motor has stopped, verify position
        SystemState state = captureSystemState();
        bool reachedSource = false;

        if (sourcePosition == POSITION_2_MM)
        {
            reachedSource = isAtPosition(state.currentPositionMm, POSITION_2_MM);
        }
        else if (sourcePosition == POSITION_3_MM)
        {
            reachedSource = isAtPosition(state.currentPositionMm, POSITION_3_MM);
        }

        if (!reachedSource)
        {
            Console.serialError(F("Motor did not reach source position"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "POSITION_FAILURE", sizeof(currentOperation.message));
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_AFTER_MOVEMENT_MS)
        {
            return;
        }

        Console.serialInfo(F("Safety delay after movement completed"));

        // Start locking shuttle to grip tray
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to lock shuttle to grip tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_LOCK, 1000))
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
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
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
            if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000))
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
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_AFTER_UNLOCK_MS)
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_BEFORE_MOVEMENT_MS)
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

        // Motor has stopped, verify position
        SystemState state = captureSystemState();
        if (!isAtPosition(state.currentPositionMm, POSITION_1_MM))
        {
            Console.serialError(F("Motor did not reach position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "POSITION_FAILURE", sizeof(currentOperation.message));
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
        if (currentMillis - safetyDelayStartTime < SAFETY_DELAY_AFTER_MOVEMENT_MS)
        {
            return;
        }

        Console.serialInfo(F("Safety delay after movement completed"));

        // Unlock shuttle to release tray
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Console.serialInfo(F("Attempting to unlock shuttle to release tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_UNLOCK, 1000))
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
            if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
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

                if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
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
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
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

        // Verify shuttle is retracted
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
    hasLastTarget = hasCurrentTarget;
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
    Serial.print(F("[ABORT] Operation aborted: "));
    Serial.println(getAbortReasonString(reason));

    // Update operation status
    currentOperation.inProgress = false;
    currentOperation.success = false;
    strncpy(currentOperation.message, getAbortReasonString(reason), sizeof(currentOperation.message));

    // Reset operation step counter using our helper function
    updateOperationStep(0);

    // End operation and update target tracking
    endOperation();

    // Provide information about recovery
    Serial.println(F("[RECOVERY] To reset system and try again, use the 'system,reset' command"));
}

// Helper function to update operation steps
void updateOperationStep(int newStep)
{
    // Update the current step
    currentOperationStep = newStep;
    // Synchronize the expected step with the current step
    expectedOperationStep = newStep;

    // For debugging
    Serial.print(F("[DIAGNOSTIC] Operation step updated: "));
    Serial.println(newStep);
}

// Function to reset the system state after a failure
void resetSystemState()
{
    // Reset operation state variables
    operationInProgress = false;
    newCommandReceived = false;

    // Reset step tracking
    updateOperationStep(0);

    // Clear current operation status
    currentOperation.inProgress = false;
    currentOperation.success = false;
    strncpy(currentOperation.message, "RESET", sizeof(currentOperation.message));
    // End operation to update target tracking
    endOperation();

    // Reset ALL target position tracking variables
    hasCurrentTarget = false;
    hasLastTarget = false;
    currentTargetType = POSITION_UNDEFINED;
    lastTargetType = POSITION_UNDEFINED;
    currentTargetPositionMm = 0.0;
    lastTargetPositionMm = 0.0;
    currentTargetPulses = 0;
    lastTargetPulses = 0;

    // Reset operation counters
    trayTracking.totalLoadsCompleted = 0;
    trayTracking.totalUnloadsCompleted = 0;
    trayTracking.lastLoadTime = 0;
    trayTracking.lastUnloadTime = 0;
    Serial.println(F("[RESET] Tray tracking state reset"));

    // Clear any fault conditions in the motor
    if (motorState == MOTOR_STATE_FAULTED)
    {
        // Initiate fault clearing process
        clearMotorFaults();
        Serial.println(F("[RESET] Clearing motor faults"));
    }

    // Re-enable the motor if it was disabled but not due to E-stop
    if (!MOTOR_CONNECTOR.EnableRequest() && !isEStopActive())
    {
        MOTOR_CONNECTOR.EnableRequest(true);
        Serial.println(F("[RESET] Re-enabling motor"));
    }

    // Update motor state if not currently faulted or in fault clearing process
    if (motorState == MOTOR_STATE_FAULTED && !isFaultClearingInProgress())
    {
        motorState = MOTOR_STATE_IDLE;
        Serial.println(F("[RESET] Motor state reset to IDLE"));
    }

    // Update tray tracking from physical sensors
    SystemState state = captureSystemState();
    updateTrayTrackingFromSensors(state);
    Serial.println(F("[RESET] Tray tracking synchronized with sensors"));

    // Log the reset action
    Serial.println(F("[SUCCESS] System state has been reset"));
}
