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
const unsigned long VALVE_ACTUATION_TIME_MS = 750;          // Increased from 500ms for more reliable actuation
const unsigned long SAFETY_DELAY_AFTER_UNLOCK_MS = 1000;    // Safety delay after unlocking a tray
const unsigned long SAFETY_DELAY_BEFORE_MOVEMENT_MS = 1000; // Safety delay before starting motor movement
const unsigned long SAFETY_DELAY_AFTER_MOVEMENT_MS = 1000;  // Safety delay after motor has completed movement
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
    Serial.println(F("[INFO] Moving first tray from position 1 to position 3"));

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
    Serial.println(F("[INFO] Moving second tray from position 1 to position 2"));

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
    Serial.println(F("[INFO] Third tray remains at position 1"));

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
    Serial.println(F("[INFO] Unloading tray from position 1"));
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
    Serial.println(F("[INFO] Moving tray from position 2 to position 1 for unloading"));
    return moveTray(2, 1);
}

// Unload third tray (moves from position 3 to position 1)
bool unloadThirdTray()
{
    Serial.println(F("[INFO] Moving tray from position 3 to position 1 for unloading"));
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
void printSystemState(const SystemState &state, Print *output)
{
    output->println(F("[DIAGNOSTIC] System State:"));

    // Motor
    output->print(F("  Motor: "));
    switch (state.motorState)
    {
    case MOTOR_STATE_IDLE:
        output->println(F("IDLE"));
        break;
    case MOTOR_STATE_MOVING:
        output->println(F("MOVING"));
        break;
    case MOTOR_STATE_HOMING:
        output->println(F("HOMING"));
        break;
    case MOTOR_STATE_FAULTED:
        output->println(F("FAULTED"));
        break;
    case MOTOR_STATE_NOT_READY:
        output->println(F("NOT_READY"));
        break;
    }

    output->print(F("  Homed: "));
    output->println(state.isHomed ? F("YES") : F("NO"));

    output->print(F("  Position: "));
    if (state.isHomed)
    {
        output->print(state.currentPositionMm);
        output->println(F(" mm"));
    }
    else
    {
        output->println(F("UNKNOWN"));
    }

    output->print(F("  HLFB Status: "));
    switch (state.hlfbStatus)
    {
    case MotorDriver::HLFB_ASSERTED:
        output->println(F("ASSERTED (In Position/Ready)"));
        break;
    case MotorDriver::HLFB_DEASSERTED:
        output->println(F("DEASSERTED (Moving/Fault)"));
        break;
    case MotorDriver::HLFB_UNKNOWN:
    default:
        output->println(F("UNKNOWN"));
        break;
    }

    // Cylinder sensors (raw readings)
    output->println(F("\n  Cylinder Sensors:"));
    output->print(F("    Tray 1: "));
    output->println(state.tray1CylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    output->print(F("    Tray 2: "));
    output->println(state.tray2CylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    output->print(F("    Tray 3: "));
    output->println(state.tray3CylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    output->print(F("    Shuttle: "));
    output->println(state.shuttleCylinderActivated ? F("ACTIVATED (UNLOCKED)") : F("NOT ACTIVATED (LOCKED)"));

    // Lock states (derived from sensor readings)
    output->println(F("\n  Lock States:"));
    output->print(F("    Tray 1: "));
    output->println(state.tray1Locked ? F("LOCKED") : F("UNLOCKED"));

    output->print(F("    Tray 2: "));
    output->println(state.tray2Locked ? F("LOCKED") : F("UNLOCKED"));

    output->print(F("    Tray 3: "));
    output->println(state.tray3Locked ? F("LOCKED") : F("UNLOCKED"));

    output->print(F("    Shuttle: "));
    output->println(state.shuttleLocked ? F("LOCKED") : F("UNLOCKED"));

    // Tray presence detection
    output->println(F("\n  Tray Detection:"));
    output->print(F("    Position 1: "));
    output->println(state.tray1Present ? F("TRAY PRESENT") : F("NO TRAY"));

    output->print(F("    Position 2: "));
    output->println(state.tray2Present ? F("TRAY PRESENT") : F("NO TRAY"));

    output->print(F("    Position 3: "));
    output->println(state.tray3Present ? F("TRAY PRESENT") : F("NO TRAY"));

    // Safety systems
    output->println(F("\n  Safety Systems:"));
    output->print(F("    E-Stop: "));
    output->println(state.eStopActive ? F("ACTIVE (Emergency Stop)") : F("INACTIVE (Normal Operation)"));

    // Hardware status
    output->println(F("\n  Hardware Status:"));
    output->print(F("    CCIO Board: "));
    output->println(state.ccioBoardPresent ? F("PRESENT") : F("NOT DETECTED"));

    // Add summary of critical safety conditions
    output->println(F("\n  Safety Summary:"));

    // Check if any tray is locked while motor is moving
    bool unsafeMotion = state.motorState == MOTOR_STATE_MOVING &&
                        (state.tray1Locked || state.tray2Locked || state.tray3Locked);
    output->print(F("    Safe Motion: "));
    output->println(unsafeMotion ? F("NO - TRAYS LOCKED DURING MOTION") : F("YES"));

    // Check for missing trays that are locked
    bool missingTraysLocked = (state.tray1Locked && !state.tray1Present) ||
                              (state.tray2Locked && !state.tray2Present) ||
                              (state.tray3Locked && !state.tray3Present);
    output->print(F("    Tray/Lock Mismatch: "));
    output->println(missingTraysLocked ? F("YES - LOCK WITHOUT TRAY") : F("NO"));

    // Encoder status
    output->println(F("\n  MPG Handwheel:"));
    output->print(F("    Status: "));
    output->println(encoderControlActive ? F("ENABLED") : F("DISABLED"));

    if (encoderControlActive)
    {
        output->print(F("    Multiplier: x"));
        output->print(getMultiplierName(currentMultiplier));
        output->print(F(" ("));

        // Calculate how much one full rotation moves (100 pulses typical for MPG handwheels)
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        output->print(mmPerRotation, 2);
        output->println(F(" mm/rotation)"));
    }

    output->println(F("-------------------------------------------"));
}

// Validate safety conditions based on the current system state
SafetyValidationResult validateSafety(const SystemState &state)
{
    SafetyValidationResult result;

    // Initialize all flags to safe by default
    result.safeToMove = true;
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
    result.commandStateValid = true;
    result.trayPositionValid = true;
    result.targetPositionValid = true;
    result.safeToAcceptNewCommand = true;
    result.operationWithinTimeout = true;
    result.operationSequenceValid = true;
    result.failureReason = ABORT_REASON_UNKNOWN; // Add this line

    //=============================================================================
    // Table 1: Basic Movement Safety Rules
    //=============================================================================

    // No movement with locked trays
    if (state.tray1Locked || state.tray2Locked || state.tray3Locked)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Tray locks engaged");
        // No abort reason - this is a prerequisite safety check
    }

    // No movement without homing
    if (!state.isHomed)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Motor not homed");
        // No abort reason - this is a prerequisite safety check
    }

    // No movement during E-stop
    if (state.eStopActive)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("E-stop active");
        result.failureReason = ABORT_REASON_ESTOP; // E-stop is an immediate abort condition
    }

    // No movement if CCIO board is not present
    if (!state.ccioBoardPresent)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("CCIO board not detected");
        // No abort reason - this is a hardware presence check
    }

    // No movement if motor is faulted
    if (state.motorState == MOTOR_STATE_FAULTED)
    {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Motor in fault state");
        // No abort reason - motor already in fault state
    }

    //=============================================================================
    // Table 2: Lock/Unlock Safety Rules
    //=============================================================================

    // No locking without tray present
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

    // No locking during movement
    if (state.motorState == MOTOR_STATE_MOVING)
    {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        result.tray1LockUnsafeReason = F("Motor is moving");
        result.tray2LockUnsafeReason = F("Motor is moving");
        result.tray3LockUnsafeReason = F("Motor is moving"); // Only flag unexpected movement if we're in a lock/unlock operation step
        // For tray loading, steps 0-7 involve lock/unlock ops, while 8+ are for movement
        if (operationInProgress &&
            (previousState.motorState != MOTOR_STATE_MOVING) &&
            ((currentOperation.type == OPERATION_LOADING && currentOperationStep < 8) ||
             (currentOperation.type == OPERATION_UNLOADING && currentOperationStep < 3)))
        {
            // This means movement started during a lock/unlock operation
            result.operationSequenceValid = false;

            // Enhance the error message with operation-specific context
            result.operationSequenceMessage = F("Motor unexpectedly started moving during ");

            // Add specific step information
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

    // No tray locking when shuttle is locked
    if (state.shuttleLocked)
    {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        result.tray1LockUnsafeReason = F("Shuttle is locked");
        result.tray2LockUnsafeReason = F("Shuttle is locked");
        result.tray3LockUnsafeReason = F("Shuttle is locked"); // Only flag unexpected shuttle locking when not in the shuttle locking step
        // We expect the shuttle to lock during the tray loading process step 2->3
        if (operationInProgress &&
            !previousState.shuttleLocked && state.shuttleLocked &&
            (currentOperation.type != OPERATION_LOADING ||
             (currentOperationStep != 2 && currentOperationStep != 3)))
        {
            // This means shuttle was locked unexpectedly outside the expected step
            result.operationSequenceValid = false;

            // Provide more detailed information about the unexpected shuttle locking
            result.operationSequenceMessage = F("Shuttle unexpectedly locked during ");

            // Add operation-specific context
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
    // Table 3: Tray Loading Sequence Rules
    //=============================================================================

    // No loading to occupied positions
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
    // Loading sequence validation
    // All trays are initially placed at position 1, then moved automatically
    // No special validation needed when no trays are present - first tray is loaded normally at position 1
    // and then moved to position 3 during the automated sequence
    // When position 3 occupied, second tray can be loaded at position 1
    // No special validation needed - second tray is loaded normally at position 1
    // and then moved to position 2 during the automated sequence

    // When positions 2 and 3 occupied, third tray stays at loading position (pos 1)
    // (This is handled by default since we don't need special validation)

    // Cannot load more than 3 trays
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
    // Table 4: Tray Unloading Sequence Rules
    //=============================================================================

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

    // First-in-last-out sequence validation
    // Tray 1 must be unloaded first
    if (state.tray1Present)
    {
        result.safeToUnloadTrayFromPos2 = false;
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos2UnsafeReason = F("Tray 1 must be unloaded first");
        result.unloadTrayPos3UnsafeReason = F("Tray 1 must be unloaded first");
        // No abort reason - this is a prerequisite check
    }

    // Tray 2 must be unloaded second
    if (state.tray2Present && !state.tray1Present)
    {
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos3UnsafeReason = F("Tray 2 must be unloaded first");
        // No abort reason - this is a prerequisite check
    }

    //=============================================================================
    // Table 5: System State Validation
    //=============================================================================
    // 1. Command vs. actual state mismatch - enhanced with detailed position information
    if (commandedPositionMm >= 0)
    {
        if (abs(state.currentPositionMm - commandedPositionMm) > POSITION_TOLERANCE_MM)
        {
            result.commandStateValid = false;

            // Enhanced error message with specific position values
            result.stateValidationMessage = F("Position mismatch: current position ");
            result.stateValidationMessage += String(state.currentPositionMm);
            result.stateValidationMessage += F(" mm vs. commanded ");
            result.stateValidationMessage += String(commandedPositionMm);
            result.stateValidationMessage += F(" mm (diff: ");
            result.stateValidationMessage += String(abs(state.currentPositionMm - commandedPositionMm));
            result.stateValidationMessage += F(" mm)");

            // Add more context about the potential issue
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

            // Add abort reason - this could indicate motor failure or blockage
            if (operationInProgress)
            {
                result.failureReason = ABORT_REASON_MOTOR_TIMEOUT;
                result.operationSequenceValid = false;

                // Add operation context
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

    // 2. Tray position validation
    bool tray1ExpectedPresent = isAtPosition(state.currentPositionMm, POSITION_1_MM);
    bool tray2ExpectedPresent = isAtPosition(state.currentPositionMm, POSITION_2_MM);
    bool tray3ExpectedPresent = isAtPosition(state.currentPositionMm, POSITION_3_MM);

    // Skip tray position validation during active tray movement operations
    bool inTrayMovementOperation = (operationInProgress &&
                                    ((currentOperation.type == OPERATION_LOADING &&
                                      (currentOperationStep == 8 || currentOperationStep == 14)) ||
                                     (currentOperation.type == OPERATION_UNLOADING &&
                                      currentOperationStep == 9)) &&
                                    state.motorState == MOTOR_STATE_MOVING);

    // Skip validation at start of unloading operations
    bool startingUnloadOperation = (operationInProgress &&
                                    currentOperation.type == OPERATION_UNLOADING &&
                                    currentOperationStep <= 3);

    if (!inTrayMovementOperation && !startingUnloadOperation)
    {
        // Only validate that expected trays ARE present (safety critical)
        if (tray1ExpectedPresent && !state.tray1Present)
        {
            result.trayPositionValid = false;

            // Detailed error message with position information
            result.stateValidationMessage = F("ERROR: Expected tray at position 1 is missing (Motor at ");
            result.stateValidationMessage += String(state.currentPositionMm);
            result.stateValidationMessage += F(" mm)");

            // Append operation context if in progress
            if (operationInProgress)
            {
                // IMPORTANT: Add this line to copy the detailed message
                result.operationSequenceMessage = result.stateValidationMessage;

                // Existing operation context code...
                result.operationSequenceValid = false;
                result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
            }
        }
    }

    // 4. Position target validation
    // Check if the target position is within allowed range
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
    else if (!skipTargetValidation) // Only fail if we shouldn't skip validation
    {
        // No target set yet
        result.targetPositionValid = false;
        result.stateValidationMessage = F("No target position set");
        result.operationSequenceMessage = result.stateValidationMessage;
    }

    //=============================================================================
    // Table 6: Operational Sequence Validation
    //=============================================================================
    // 1. No new commands during operations
    // Check if a new command was received while an operation is in progress
    if (operationInProgress)
    {
        if (newCommandReceived)
        {
            result.safeToAcceptNewCommand = false;
            result.operationSequenceMessage = F("Operation in progress, cannot accept new command");
            result.operationSequenceMessage += F(" (use 'system,reset' after failure)");
        }
    }

    // 2. Operation timeout
    // Check if the current operation has exceeded its timeout
    if (operationInProgress && millis() - operationStartTime > operationTimeoutMs)
    {
        result.operationWithinTimeout = false;
        result.operationSequenceMessage = F("Operation exceeded timeout");
        result.failureReason = ABORT_REASON_OPERATION_TIMEOUT; // Add this line
    }

    // 3. Operation state mismatch
    // Check if the current operation is in the expected step of its sequence with more detailed error message
    if (operationInProgress && currentOperationStep != expectedOperationStep)
    {
        result.operationSequenceValid = false;

        // Provide more detailed information about the sequence mismatch
        result.operationSequenceMessage = F("Operation sequence mismatch: ");

        // Add operation specific context to the message
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

        // Add step information
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
// Function to print out safety validation results
void printSafetyStatus(const SafetyValidationResult &result, Print *output)
{
    output->println(F("[SAFETY] Validation Results:"));

    // Movement safety
    output->print(F("  Motor Movement: "));
    if (result.safeToMove)
    {
        output->println(F("SAFE - System ready for movement"));
    }
    else
    {
        output->print(F("UNSAFE - "));
        output->println(result.moveUnsafeReason);
    }
    // Tray locking safety with enhanced safety messages
    output->println(F("  Tray Locking:"));
    output->print(F("    Tray 1: "));
    if (result.safeToLockTray1)
    {
        // Enhanced message with contextual information
        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            output->println(F("SAFE TO LOCK - Part of loading sequence"));
        }
        else if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            output->println(F("SAFE TO LOCK - Part of unloading sequence"));
        }
        else
        {
            output->println(F("SAFE TO LOCK - Tray present and system ready"));
        }
    }
    else
    {
        output->print(F("UNSAFE TO LOCK - "));
        output->println(result.tray1LockUnsafeReason);
    }

    output->print(F("    Tray 2: "));
    if (result.safeToLockTray2)
    {
        // Enhanced message with contextual information
        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            output->println(F("SAFE TO LOCK - Part of loading sequence"));
        }
        else if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            output->println(F("SAFE TO LOCK - Part of unloading sequence"));
        }
        else
        {
            output->println(F("SAFE TO LOCK - Tray present and system ready"));
        }
    }
    else
    {
        output->print(F("UNSAFE TO LOCK - "));
        output->println(result.tray2LockUnsafeReason);
    }

    output->print(F("    Tray 3: "));
    if (result.safeToLockTray3)
    {
        // Enhanced message with contextual information
        if (operationInProgress && currentOperation.type == OPERATION_LOADING)
        {
            output->println(F("SAFE TO LOCK - Part of loading sequence"));
        }
        else if (operationInProgress && currentOperation.type == OPERATION_UNLOADING)
        {
            output->println(F("SAFE TO LOCK - Part of unloading sequence"));
        }
        else
        {
            output->println(F("SAFE TO LOCK - Tray present and system ready"));
        }
    }
    else
    {
        output->print(F("UNSAFE TO LOCK - "));
        output->println(result.tray3LockUnsafeReason);
    }

    // Shuttle actuation safety
    output->println(F("  Shuttle Control:"));
    output->print(F("    Lock: "));
    if (result.safeToLockShuttle)
    {
        // Enhanced shuttle lock message
        if (operationInProgress && currentOperation.type == OPERATION_LOADING &&
            (currentOperationStep == 2 || currentOperationStep == 3))
        {
            output->println(F("SAFE - Part of tray loading sequence"));
        }
        else
        {
            output->println(F("SAFE - System ready for shuttle locking"));
        }
    }
    else
    {
        output->print(F("UNSAFE - "));
        output->println(result.shuttleLockUnsafeReason);
    }
    output->print(F("    Unlock: "));
    if (result.safeToUnlockShuttle)
    {
        // Enhanced shuttle unlock message
        if (operationInProgress && currentOperation.type == OPERATION_LOADING &&
            (currentOperationStep == 4 || currentOperationStep == 5))
        {
            output->println(F("SAFE - Part of tray loading sequence"));
        }
        else
        {
            output->println(F("SAFE - System ready for shuttle unlocking"));
        }
    }
    else
    {
        output->print(F("UNSAFE - "));
        output->println(result.shuttleUnlockUnsafeReason);
    }

    // System state validation status with enhanced messages
    output->println(F("\n  System State Validation:"));
    output->print(F("    Command/Actual State: "));
    if (result.commandStateValid)
    {
        output->println(F("VALID - System position matches commanded position"));
    }
    else
    {
        output->print(F("INVALID - "));
        output->println(result.stateValidationMessage);
    }

    output->print(F("    Tray Positions: "));
    if (result.trayPositionValid)
    {
        output->println(F("VALID - Tray presence matches expected positions"));
    }
    else
    {
        output->print(F("INVALID - "));
        output->println(result.stateValidationMessage);
    }

    output->print(F("    Target Position: "));
    if (result.targetPositionValid)
    {
        if (hasCurrentTarget)
        {
            output->print(F("VALID - Target position "));
            output->print(currentTargetPositionMm);
            output->println(F(" mm is within safe range"));
        }
        else
        {
            output->println(F("VALID - No target position currently set"));
        }
    }
    else
    {
        output->print(F("INVALID - "));
        output->println(result.stateValidationMessage);
    }

    // Operational sequence validation with enhanced messages
    output->println(F("\n  Operational Sequence:"));
    output->print(F("    Accept New Commands: "));
    if (result.safeToAcceptNewCommand)
    {
        if (operationInProgress)
        {
            output->print(F("SAFE - Current operation: "));

            // Show current operation type and step for context
            switch (currentOperation.type)
            {
            case OPERATION_LOADING:
                output->print(F("LOADING"));
                break;
            case OPERATION_UNLOADING:
                output->print(F("UNLOADING"));
                break;
            default:
                output->print(F("OTHER"));
                break;
            }
            output->print(F(" (Step "));
            output->print(currentOperationStep);
            output->println(F(")"));
        }
        else
        {
            output->println(F("SAFE - No operation in progress"));
        }
    }
    else
    {
        output->print(F("UNSAFE - "));
        output->println(result.operationSequenceMessage);
    }

    output->print(F("    Operation Timing: "));
    if (result.operationWithinTimeout)
    {
        if (operationInProgress)
        {
            // Show elapsed time information for active operations
            unsigned long elapsedTime = millis() - operationStartTime;
            output->print(F("WITHIN TIMEOUT - Elapsed: "));
            output->print(elapsedTime / 1000);
            output->print(F("."));
            output->print((elapsedTime % 1000) / 100);
            output->print(F("s / "));
            output->print(operationTimeoutMs / 1000);
            output->println(F("s"));
        }
        else
        {
            output->println(F("WITHIN TIMEOUT - No operation active"));
        }
    }
    else
    {
        output->print(F("TIMEOUT - "));
        output->println(result.operationSequenceMessage);
    }
    output->print(F("    Operation Sequence: "));
    if (result.operationSequenceValid)
    {
        if (operationInProgress)
        {
            output->print(F("VALID - Current step: "));
            output->print(currentOperationStep);
            if (currentOperationStep == expectedOperationStep)
            {
                output->println(F(" (as expected)"));
            }
            else
            {
                output->print(F(" (expected: "));
                output->print(expectedOperationStep);
                output->println(F(")"));
            }
        }
        else
        {
            output->println(F("VALID - No operation in progress"));
        }
    }
    else
    {
        output->print(F("INVALID - "));
        output->println(result.operationSequenceMessage);

        // Add recovery guidance when appropriate
        if (result.operationSequenceMessage.indexOf(F("sequence mismatch")) >= 0)
        {
            output->println(F("            Use 'system,reset' to reset the system"));
        }
    }

    // Add detailed tray loading safety information
    output->println(F("\n  Tray Loading Operations:"));
    output->print(F("    Position 1: "));
    if (result.safeToLoadTrayToPos1)
    {
        output->print(F("SAFE TO LOAD"));
        // Add context about tray loading sequence
        if (trayTracking.totalTraysInSystem == 0)
        {
            output->println(F(" - Ready for first tray"));
        }
        else if (trayTracking.totalTraysInSystem == 1)
        {
            output->println(F(" - Ready for second tray"));
        }
        else if (trayTracking.totalTraysInSystem == 2)
        {
            output->println(F(" - Ready for third tray"));
        }
        else
        {
            output->println(F(" - Ready for tray"));
        }
    }
    else
    {
        output->print(F("UNSAFE TO LOAD - "));
        output->println(result.loadTrayPos1UnsafeReason);
    }

    output->print(F("    Position 2: "));
    if (result.safeToLoadTrayToPos2)
    {
        output->println(F("SAFE TO LOAD - Direct loading possible"));
    }
    else
    {
        output->print(F("UNSAFE TO LOAD - "));
        output->println(result.loadTrayPos2UnsafeReason);
    }

    output->print(F("    Position 3: "));
    if (result.safeToLoadTrayToPos3)
    {
        output->println(F("SAFE TO LOAD - Direct loading possible"));
    }
    else
    {
        output->print(F("UNSAFE TO LOAD - "));
        output->println(result.loadTrayPos3UnsafeReason);
    }

    // Add detailed tray unloading safety information
    output->println(F("\n  Tray Unloading Operations:"));
    output->print(F("    Position 1: "));
    if (result.safeToUnloadTrayFromPos1)
    {
        output->println(F("SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        output->print(F("UNSAFE TO UNLOAD - "));
        output->println(result.unloadTrayPos1UnsafeReason);
    }

    output->print(F("    Position 2: "));
    if (result.safeToUnloadTrayFromPos2)
    {
        output->println(F("SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        output->print(F("UNSAFE TO UNLOAD - "));
        output->println(result.unloadTrayPos2UnsafeReason);
    }

    output->print(F("    Position 3: "));
    if (result.safeToUnloadTrayFromPos3)
    {
        output->println(F("SAFE TO UNLOAD - Tray ready for removal"));
    }
    else
    {
        output->print(F("UNSAFE TO UNLOAD - "));
        output->println(result.unloadTrayPos3UnsafeReason);
    }

    // Add overall system status at the end
    output->println(F("\n  System Summary:"));
    if (result.operationSequenceValid && result.trayPositionValid && result.commandStateValid)
    {
        output->println(F("    Status: NORMAL - System operating correctly"));

        if (operationInProgress)
        {
            output->print(F("    Current Operation: "));
            // Show current operation type for context
            switch (currentOperation.type)
            {
            case OPERATION_LOADING:
                output->print(F("LOADING"));
                break;
            case OPERATION_UNLOADING:
                output->print(F("UNLOADING"));
                break;
            default:
                output->print(F("OTHER"));
                break;
            }
            output->print(F(" (Step "));
            output->print(currentOperationStep);
            output->println(F(")"));
        }
    }
    else
    {
        output->println(F("    Status: ALERT - System requires attention"));

        if (!result.operationSequenceValid)
        {
            output->println(F("    Reason: Operation sequence error detected"));
            output->print(F("            "));
            output->println(result.operationSequenceMessage);
        }
        else if (!result.trayPositionValid)
        {
            output->println(F("    Reason: Tray position error detected"));
            output->print(F("            "));
            output->println(result.stateValidationMessage);
        }
        else if (!result.commandStateValid)
        {
            output->println(F("    Reason: Command/state mismatch detected"));
            output->print(F("            "));
            output->println(result.stateValidationMessage);
        }

        // Add specific recovery guidance based on detected issues
        output->println(F("    Recovery: Use 'system,reset' to reset system state and try again"));
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
        Serial.println(F("[ERROR] Tray operation timeout"));
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

        Serial.println(F("[INFO] Starting tray loading process - initial checks"));

        // Verify tray is at position 1 and locked
        if (!state.tray1Present)
        {
            Serial.println(F("[ERROR] No tray detected at position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "NO_TRAY", sizeof(currentOperation.message));
            return;
        }

        if (!state.tray1Locked)
        {
            Serial.println(F("[ERROR] Tray at position 1 not locked"));
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
            Serial.println(F("[INFO] First tray - target is position 3"));
        }
        else if (workflow == 2)
        {
            // Second tray - goes to position 2
            targetPosition = POSITION_2_MM;
            isShuttleNeeded = true;
            Serial.println(F("[INFO] Second tray - target is position 2"));
        }
        else
        {
            // Third tray - stays at position 1
            isShuttleNeeded = false;
            Serial.println(F("[INFO] Third tray - keeping at position 1"));
            // Skip to the final step for position 1
            updateOperationStep(12); // Special case - skip to completion
            return;
        }
        // Check if target position is occupied
        if ((targetPosition == POSITION_2_MM && state.tray2Present) ||
            (targetPosition == POSITION_3_MM && state.tray3Present))
        {
            Serial.println(F("[ERROR] Target position already occupied"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TARGET_POSITION_OCCUPIED", sizeof(currentOperation.message));
            return;
        }

        // Check if path is clear to target position
        if (!isPathClearForLoading(state.currentPositionMm, targetPosition, state))
        {
            Serial.println(F("[ERROR] Path to target position is blocked"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "PATH_BLOCKED", sizeof(currentOperation.message));
            return;
        }
        // All checks pass, advance to next step
        updateOperationStep(1);
        Serial.println(F("[INFO] Initial checks passed, starting tray advancement sequence"));

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
            Serial.println(F("[ERROR] Tray at position 1 disappeared during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY1_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }

        if (!state.tray1Locked)
        {
            Serial.println(F("[ERROR] Tray 1 lock status changed during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY1_LOCK_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step - shuttle locking if needed
        updateOperationStep(2);
        Serial.println(F("[INFO] Sensor verification complete"));
    }
    break;

    case 2: // Lock shuttle to grip tray (if needed)
    {
        if (!isShuttleNeeded)
        {
            // Skip shuttle locking if not moving the tray
            updateOperationStep(4);
            Serial.println(F("[INFO] Skipping shuttle operation (not needed for this move)"));
            return;
        }

        // Start locking shuttle
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Serial.println(F("[INFO] Attempting to lock shuttle to grip tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_LOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to lock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated shuttle lock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access shuttle valve"));
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
            Serial.println(F("[ERROR] Failed to lock shuttle - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }
        Serial.println(F("[INFO] Shuttle lock confirmed successful"));

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
            Serial.println(F("[INFO] Attempting to unlock tray at position 1"));

            if (!safeValveOperation(*valve, *getTray1Sensor(), VALVE_POSITION_UNLOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to unlock tray 1 - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated tray 1 unlock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access tray 1 valve"));
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
            Serial.println(F("[ERROR] Failed to unlock tray at position 1 - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Tray 1 unlock confirmed successful"));
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

        Serial.println(F("[INFO] Safety delay after unlock completed"));
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

        Serial.println(F("[INFO] Safety delay before movement completed"));

        // Start movement to target position
        if (!moveToPositionMm(targetPosition))
        {
            Serial.println(F("[ERROR] Failed to start movement to target position"));
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

        Serial.println(F("[INFO] Motor movement completed"));

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
            Serial.println(F("[ERROR] Motor did not reach target position"));
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

        Serial.println(F("[INFO] Safety delay after movement completed"));

        // Unlock shuttle now that we've reached the destination
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Serial.println(F("[INFO] Attempting to unlock shuttle to release tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_UNLOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to unlock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated shuttle unlock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access shuttle valve"));
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
            Serial.println(F("[ERROR] Failed to unlock shuttle - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Shuttle unlock confirmed successful"));
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

        Serial.println(F("[INFO] Safety delay after shuttle unlock completed"));

        // Lock tray at target position
        DoubleSolenoidValve *valve = NULL;
        CylinderSensor *sensor = NULL;

        if (targetPosition == POSITION_2_MM)
        {
            valve = getTray2Valve();
            sensor = getTray2Sensor();
            Serial.println(F("[INFO] Attempting to lock tray at position 2"));
        }
        else if (targetPosition == POSITION_3_MM)
        {
            valve = getTray3Valve();
            sensor = getTray3Sensor();
            Serial.println(F("[INFO] Attempting to lock tray at position 3"));
        }

        if (valve && sensor)
        {
            if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to lock tray at target position - sensor didn't confirm"));
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
            Serial.println(F("[ERROR] Failed to access target position valve or sensor"));
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
            Serial.println(F("[INFO] Updating tray tracking for position 1"));
            loadThirdTray();

            // Operation complete
            Serial.println(F("[SUCCESS] Tray loading at position 1 completed successfully"));
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
            Serial.println(F("[ERROR] Failed to lock tray at target position - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Tray lock at target position confirmed successful"));

        // Update tray tracking based on target position
        if (targetPosition == POSITION_2_MM)
        {
            // Move from position 1 to 2
            Serial.println(F("[INFO] Updating tray tracking: position 1 -> position 2"));
            loadSecondTray();
        }
        else if (targetPosition == POSITION_3_MM)
        {
            // Move from position 1 to 3
            Serial.println(F("[INFO] Updating tray tracking: position 1 -> position 3"));
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

        Serial.println(F("[INFO] Safety delay before return movement completed"));

        // Now return the conveyor to position 1
        if (!moveToPositionMm(POSITION_1_MM))
        {
            Serial.println(F("[ERROR] Failed to start movement to loading position"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "RETURN_MOVE_FAILURE", sizeof(currentOperation.message));
            return;
        }
        // Advance to next step (watching for conveyor return)
        updateOperationStep(14);
        Serial.println(F("[INFO] Returning conveyor to loading position"));
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

        Serial.println(F("[INFO] Return movement completed"));

        // Motor has stopped, verify position
        SystemState state = captureSystemState();
        if (!isAtPosition(state.currentPositionMm, POSITION_1_MM))
        {
            Serial.println(F("[ERROR] Motor did not return to position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "RETURN_FAILURE", sizeof(currentOperation.message));
            return;
        }

        // Operation complete
        Serial.println(F("[SUCCESS] Tray loading completed successfully"));
        currentOperation.inProgress = false;
        currentOperation.success = true;
        strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));

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

        Serial.println(F("[INFO] Starting tray unloading process - initial checks"));

        // Check if there are any trays in the system
        if (!state.tray1Present && !state.tray2Present && !state.tray3Present)
        {
            Serial.println(F("[ERROR] No trays in system to unload"));
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
            Serial.println(F("[ERROR] No trays in system to unload"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "NO_TRAYS", sizeof(currentOperation.message));
            return;
        }
        else if (workflow == 1)
        {
            // Tray already at position 1, just need to unlock it
            needsMovementToPos1 = false;
            Serial.println(F("[INFO] Tray at position 1 ready for unloading"));
            // Skip to tray unlocking step
            updateOperationStep(11);
            return;
        }
        else if (workflow == 2)
        {
            // Need to move tray from position 2 to position 1
            sourcePosition = POSITION_2_MM;
            needsMovementToPos1 = true;
            Serial.println(F("[INFO] Will move tray from position 2 to position 1 for unloading"));
        }
        else if (workflow == 3)
        {
            // Need to move tray from position 3 to position 1
            sourcePosition = POSITION_3_MM;
            needsMovementToPos1 = true;
            Serial.println(F("[INFO] Will move tray from position 3 to position 1 for unloading"));
        }

        // If we need to move a tray, check path clearance
        if (needsMovementToPos1)
        {
            if (!isPathClearForUnloading(state.currentPositionMm, sourcePosition, state) ||
                !isPathClearForUnloading(sourcePosition, POSITION_1_MM, state))
            {
                Serial.println(F("[ERROR] Path to source or target position is blocked"));
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
                Serial.println(F("[ERROR] Tray at position 1 not locked"));
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
            Serial.println(F("[ERROR] Tray at position 2 disappeared during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY2_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }
        else if (sourcePosition == POSITION_3_MM && !state.tray3Present)
        {
            Serial.println(F("[ERROR] Tray at position 3 disappeared during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "TRAY3_VERIFICATION_FAILED", sizeof(currentOperation.message));
            return;
        }

        // Verify position 1 is still clear
        if (state.tray1Present)
        {
            Serial.println(F("[ERROR] Position 1 unexpectedly occupied during verification"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "POSITION1_OCCUPIED", sizeof(currentOperation.message));
            return;
        }

        // All verified, move to next step
        updateOperationStep(2);
        Serial.println(F("[INFO] Sensor verification complete"));
    }
    break;

    case 2: // Move to source position (2 or 3)
    {
        // Start movement to source position
        if (!moveToPositionMm(sourcePosition))
        {
            Serial.println(F("[ERROR] Failed to start movement to source position"));
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

        Serial.println(F("[INFO] Reached source position"));

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
            Serial.println(F("[ERROR] Motor did not reach source position"));
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

        Serial.println(F("[INFO] Safety delay after movement completed"));

        // Start locking shuttle to grip tray
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Serial.println(F("[INFO] Attempting to lock shuttle to grip tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_LOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to lock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated shuttle lock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access shuttle valve"));
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
            Serial.println(F("[ERROR] Failed to lock shuttle - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Shuttle lock confirmed successful"));

        // Determine which valve to unlock based on source position
        DoubleSolenoidValve *valve = NULL;
        CylinderSensor *sensor = NULL;

        if (sourcePosition == POSITION_2_MM)
        {
            valve = getTray2Valve();
            sensor = getTray2Sensor();
            Serial.println(F("[INFO] Attempting to unlock tray at position 2"));
        }
        else
        {
            valve = getTray3Valve();
            sensor = getTray3Sensor();
            Serial.println(F("[INFO] Attempting to unlock tray at position 3"));
        }

        if (valve && sensor)
        {
            if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_UNLOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to unlock tray - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated tray unlock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access valve or sensor"));
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
            Serial.println(F("[ERROR] Failed to unlock tray - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Tray unlock confirmed successful"));

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

        Serial.println(F("[INFO] Safety delay after unlock completed"));

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

        Serial.println(F("[INFO] Safety delay before movement completed"));

        // Start movement to position 1
        if (!moveToPositionMm(POSITION_1_MM))
        {
            Serial.println(F("[ERROR] Failed to start movement to position 1"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "MOVE_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Moving tray to position 1 for unloading"));

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

        Serial.println(F("[INFO] Reached position 1"));

        // Motor has stopped, verify position
        SystemState state = captureSystemState();
        if (!isAtPosition(state.currentPositionMm, POSITION_1_MM))
        {
            Serial.println(F("[ERROR] Motor did not reach position 1"));
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

        Serial.println(F("[INFO] Safety delay after movement completed"));

        // Unlock shuttle to release tray
        DoubleSolenoidValve *shuttleValve = getShuttleValve();
        if (shuttleValve)
        {
            Serial.println(F("[INFO] Attempting to unlock shuttle to release tray"));

            if (!safeValveOperation(*shuttleValve, *getShuttleSensor(), VALVE_POSITION_UNLOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to unlock shuttle - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated shuttle unlock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access shuttle valve"));
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
                Serial.println(F("[ERROR] Failed to unlock shuttle - verification failed"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }

            Serial.println(F("[INFO] Shuttle unlock confirmed successful"));

            // Lock tray at position 1
            DoubleSolenoidValve *valve = getTray1Valve();
            CylinderSensor *sensor = getTray1Sensor();

            if (valve && sensor)
            {
                Serial.println(F("[INFO] Attempting to lock tray at position 1"));

                if (!safeValveOperation(*valve, *sensor, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[ERROR] Failed to lock tray at position 1 - sensor didn't confirm"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                valveActuationStartTime = currentMillis;
                Serial.println(F("[INFO] Initiated tray lock valve actuation at position 1"));
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access tray 1 valve or sensor"));
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
                Serial.println(F("[ERROR] Tray at position 1 not locked"));
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
            Serial.println(F("[ERROR] Failed to lock tray at position 1 - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Tray lock at position 1 confirmed successful"));

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

    case 13: // Final unlock of tray at position 1 for removal
    {
        // Unlock tray at position 1 for removal
        DoubleSolenoidValve *valve = getTray1Valve();
        if (valve)
        {
            Serial.println(F("[INFO] Unlocking tray at position 1 for removal"));

            if (!safeValveOperation(*valve, *getTray1Sensor(), VALVE_POSITION_UNLOCK, 1000))
            {
                Serial.println(F("[ERROR] Failed to unlock tray 1 - sensor didn't confirm"));
                currentOperation.inProgress = false;
                currentOperation.success = false;
                strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
                return;
            }
            valveActuationStartTime = currentMillis;
            Serial.println(F("[INFO] Initiated tray 1 unlock valve actuation"));
        }
        else
        {
            Serial.println(F("[ERROR] Failed to access tray 1 valve"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
            return;
        }

        // Advance to next step
        updateOperationStep(14);
    }
    break;

    case 14: // Wait for tray unlock and notify ready for removal
    {
        // Wait for valve actuation time to elapse
        if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS)
        {
            // Not enough time has elapsed, check again next cycle
            return;
        }

        // Verify tray is unlocked
        SystemState state = captureSystemState();
        if (state.tray1Locked)
        {
            Serial.println(F("[ERROR] Failed to unlock tray at position 1 - verification failed"));
            currentOperation.inProgress = false;
            currentOperation.success = false;
            strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
            return;
        }

        Serial.println(F("[INFO] Tray unlocked and ready for removal"));
        Serial.println(F("TRAY_READY"));

        // Operation complete - mitsubishi will need to send tray,removed command
        currentOperation.inProgress = false;
        currentOperation.success = true;
        strncpy(currentOperation.message, "TRAY_READY", sizeof(currentOperation.message));

        // Ensure tray tracking matches sensor readings
        state = captureSystemState();
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
    if (encoderControlActive) {
        encoderControlActive = false;
        Serial.println(F("[INFO] MPG handwheel control temporarily disabled during automated operation"));
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
    if (operationEncoderState) {
        encoderControlActive = true;
        Serial.println(F("[INFO] MPG handwheel control re-enabled after automated operation"));
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
    Serial.print(F("[DEBUG] Operation step updated: "));
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
    Serial.println(F("[RESET] System state has been reset"));
}
