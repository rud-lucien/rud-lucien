#include "Utils.h"

// Define variables that were declared in Utils.h
// Position tracking
double commandedPositionMm = -1;  // Initialize to -1

// Tray tracking structure
TrayTracking trayTracking = {
    .totalTraysInSystem = 0,
    .position1Occupied = false,
    .position2Occupied = false,
    .position3Occupied = false,
    .lastLoadTime = 0,
    .lastUnloadTime = 0,
    .totalLoadsCompleted = 0,
    .totalUnloadsCompleted = 0
};

// Operation state tracking
bool operationInProgress = false;
bool newCommandReceived = false;
unsigned long operationStartTime = 0;
unsigned long operationTimeoutMs = 60000;  // 60 seconds default
int currentOperationStep = 0;
int expectedOperationStep = 0;

// State machine substep tracking
unsigned long valveActuationStartTime = 0;
const unsigned long VALVE_ACTUATION_TIME_MS = 500;

// Tray status structure
TrayStatus trayStatus = {false, false, false, 0, OPERATION_NONE};

// Current operation structure
OperationStatus currentOperation = {false, OPERATION_NONE, 0, 0, false, ""};

// Add this variable definition near your other global variables:
SystemState previousState;

// Add a tray to position 1 (loading position)
bool addTrayAtPosition1() {
    if (!trayTracking.position1Occupied) {
        trayTracking.position1Occupied = true;
        trayTracking.totalTraysInSystem++;
        trayTracking.lastLoadTime = millis();
        trayTracking.totalLoadsCompleted++;
        return true;
    }
    return false; // Position already occupied
}

// Remove a tray from position 3 (unloading position)
bool removeTrayAtPosition3() {
    if (trayTracking.position3Occupied) {
        trayTracking.position3Occupied = false;
        trayTracking.totalTraysInSystem--;
        trayTracking.lastUnloadTime = millis();
        trayTracking.totalUnloadsCompleted++;
        return true;
    }
    return false; // No tray at position 3
}

// Move trays forward in the system (after loading or unloading)
bool advanceTrays() {
    // Check if we can advance (position 2 must be free to move from 1→2)
    if (trayTracking.position1Occupied && !trayTracking.position2Occupied) {
        // Move tray from position 1 to 2
        trayTracking.position1Occupied = false;
        trayTracking.position2Occupied = true;
        return true;
    }
    
    // Check if we can advance (position 3 must be free to move from 2→3)
    if (trayTracking.position2Occupied && !trayTracking.position3Occupied) {
        // Move tray from position 2 to 3
        trayTracking.position2Occupied = false;
        trayTracking.position3Occupied = true;
        return true;
    }
    
    return false; // Could not advance trays
}

// Move tray directly from position 1 to position 3
bool moveTraysFromPos1ToPos3() {
    if (trayTracking.position1Occupied && !trayTracking.position3Occupied) {
        trayTracking.position1Occupied = false;
        trayTracking.position3Occupied = true;
        return true;
    }
    return false; // Cannot move trays
}

// Update tray tracking based on sensor readings
void updateTrayTrackingFromSensors(const SystemState &state) {
    // This should sync our tracking with physical sensor readings
    // Could be called periodically to correct any tracking errors
    
    // Check for inconsistency between tracking and sensors
    if (trayTracking.position1Occupied != state.tray1Present ||
        trayTracking.position2Occupied != state.tray2Present ||
        trayTracking.position3Occupied != state.tray3Present) {
        
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
SystemState captureSystemState() {
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
    // For example: LOCKED when sensor is ACTIVATED
    state.tray1Locked = state.tray1CylinderActivated;
    state.tray2Locked = state.tray2CylinderActivated;
    state.tray3Locked = state.tray3CylinderActivated;
    state.shuttleLocked = state.shuttleCylinderActivated;
    
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
void printSystemState(const SystemState &state, Print* output) {
    output->println(F("[DIAGNOSTIC] System State:"));
    
    // Motor
    output->print(F("  Motor: "));
    switch (state.motorState) {
        case MOTOR_STATE_IDLE: output->println(F("IDLE")); break;
        case MOTOR_STATE_MOVING: output->println(F("MOVING")); break;
        case MOTOR_STATE_HOMING: output->println(F("HOMING")); break;
        case MOTOR_STATE_FAULTED: output->println(F("FAULTED")); break;
        case MOTOR_STATE_NOT_READY: output->println(F("NOT_READY")); break;
    }
    
    output->print(F("  Homed: "));
    output->println(state.isHomed ? F("YES") : F("NO"));
    
    output->print(F("  Position: "));
    if (state.isHomed) {
        output->print(state.currentPositionMm);
        output->println(F(" mm"));
    } else {
        output->println(F("UNKNOWN"));
    }
    
    output->print(F("  HLFB Status: "));
    switch (state.hlfbStatus) {
        case MotorDriver::HLFB_ASSERTED: output->println(F("ASSERTED (In Position/Ready)")); break;
        case MotorDriver::HLFB_DEASSERTED: output->println(F("DEASSERTED (Moving/Fault)")); break;
        case MotorDriver::HLFB_UNKNOWN: 
        default: output->println(F("UNKNOWN")); break;
    }
    
    // Cylinder sensors (raw readings)
    output->println(F("\n  Cylinder Sensors:"));
    output->print(F("    Tray 1: ")); 
    output->println(state.tray1CylinderActivated ? F("ACTIVATED") : F("NOT ACTIVATED"));
    
    output->print(F("    Tray 2: ")); 
    output->println(state.tray2CylinderActivated ? F("ACTIVATED") : F("NOT ACTIVATED"));
    
    output->print(F("    Tray 3: ")); 
    output->println(state.tray3CylinderActivated ? F("ACTIVATED") : F("NOT ACTIVATED"));
    
    output->print(F("    Shuttle: ")); 
    output->println(state.shuttleCylinderActivated ? F("ACTIVATED") : F("NOT ACTIVATED"));
    
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
    
    output->println(F("-------------------------------------------"));
}

// Validate safety conditions based on the current system state
SafetyValidationResult validateSafety(const SystemState &state) {
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
    if (state.tray1Locked || state.tray2Locked || state.tray3Locked) {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Tray locks engaged");
        // No abort reason - this is a prerequisite safety check
    }

    // No movement without homing
    if (!state.isHomed) {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Motor not homed");
        // No abort reason - this is a prerequisite safety check
    }

    // No movement during E-stop
    if (state.eStopActive) {
        result.safeToMove = false;
        result.moveUnsafeReason = F("E-stop active");
        result.failureReason = ABORT_REASON_ESTOP;  // E-stop is an immediate abort condition
    }

    // No movement if CCIO board is not present
    if (!state.ccioBoardPresent) {
        result.safeToMove = false;
        result.moveUnsafeReason = F("CCIO board not detected");
        // No abort reason - this is a hardware presence check
    }

    // No movement if motor is faulted
    if (state.motorState == MOTOR_STATE_FAULTED) {
        result.safeToMove = false;
        result.moveUnsafeReason = F("Motor in fault state");
        // No abort reason - motor already in fault state
    }
    
    //=============================================================================
    // Table 2: Lock/Unlock Safety Rules
    //=============================================================================

    // No locking without tray present
    if (!state.tray1Present) {
        result.safeToLockTray1 = false;
        result.tray1LockUnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray2Present) {
        result.safeToLockTray2 = false;
        result.tray2LockUnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    if (!state.tray3Present) {
        result.safeToLockTray3 = false;
        result.tray3LockUnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }

    // No locking during movement
    if (state.motorState == MOTOR_STATE_MOVING) {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        result.tray1LockUnsafeReason = F("Motor is moving");
        result.tray2LockUnsafeReason = F("Motor is moving");
        result.tray3LockUnsafeReason = F("Motor is moving");
        
        // If an attempt to lock during movement is detected during an operation:
        if (operationInProgress && 
            (previousState.motorState != MOTOR_STATE_MOVING)) {
            // This means movement started during a lock operation
            result.operationSequenceValid = false; 
            result.operationSequenceMessage = F("Motor unexpectedly started moving during lock operation"); 
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH; 
        }
    }

    // No tray locking when shuttle is locked
    if (state.shuttleLocked) {
        result.safeToLockTray1 = false;
        result.safeToLockTray2 = false;
        result.safeToLockTray3 = false;
        result.tray1LockUnsafeReason = F("Shuttle is locked");
        result.tray2LockUnsafeReason = F("Shuttle is locked");
        result.tray3LockUnsafeReason = F("Shuttle is locked");
        
        // If an attempt to lock with shuttle locked is detected during an operation:
        if (operationInProgress && 
            !previousState.shuttleLocked && state.shuttleLocked) {
            // This means shuttle was locked unexpectedly during operation
            result.operationSequenceValid = false; 
            result.operationSequenceMessage = F("Shuttle unexpectedly locked during operation"); // Add this line
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    }
    
    //=============================================================================
    // Table 3: Tray Loading Sequence Rules
    //=============================================================================
    
    // No loading to occupied positions
    if (state.tray1Present) {
        result.safeToLoadTrayToPos1 = false;
        result.loadTrayPos1UnsafeReason = F("Position already occupied");
        // No abort reason - this is a prerequisite check
    }
    
    if (state.tray2Present) {
        result.safeToLoadTrayToPos2 = false;
        result.loadTrayPos2UnsafeReason = F("Position already occupied");
        // No abort reason - this is a prerequisite check
    }
    
    if (state.tray3Present) {
        result.safeToLoadTrayToPos3 = false;
        result.loadTrayPos3UnsafeReason = F("Position already occupied");
        // No abort reason - this is a prerequisite check
    }
    
    // Loading sequence validation
    // When no trays present, first tray goes to position 3
    if (!state.tray1Present && !state.tray2Present && !state.tray3Present) {
        result.safeToLoadTrayToPos1 = false;
        result.safeToLoadTrayToPos2 = false;
        result.loadTrayPos1UnsafeReason = F("First tray must go to position 3");
        result.loadTrayPos2UnsafeReason = F("First tray must go to position 3");
        // No abort reason - this is a prerequisite check
    }
    
    // When position 3 occupied, second tray goes to position 2
    if (state.tray3Present && !state.tray2Present && !state.tray1Present) {
        result.safeToLoadTrayToPos1 = false;
        result.loadTrayPos1UnsafeReason = F("Second tray must go to position 2");
        // No abort reason - this is a prerequisite check
    }
    
    // When positions 2 and 3 occupied, third tray stays at loading position (pos 1)
    // (This is handled by default since we don't need special validation)
    
    // Cannot load more than 3 trays
    if (state.tray1Present && state.tray2Present && state.tray3Present) {
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
    if (!state.tray1Present) {
        result.safeToUnloadTrayFromPos1 = false;
        result.unloadTrayPos1UnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }
    
    if (!state.tray2Present) {
        result.safeToUnloadTrayFromPos2 = false;
        result.unloadTrayPos2UnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }
    
    if (!state.tray3Present) {
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos3UnsafeReason = F("No tray detected");
        // No abort reason - this is a prerequisite check
    }
    
    // First-in-last-out sequence validation
    // Tray 1 must be unloaded first
    if (state.tray1Present) {
        result.safeToUnloadTrayFromPos2 = false;
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos2UnsafeReason = F("Tray 1 must be unloaded first");
        result.unloadTrayPos3UnsafeReason = F("Tray 1 must be unloaded first");
        // No abort reason - this is a prerequisite check
    }
    
    // Tray 2 must be unloaded second
    if (state.tray2Present && !state.tray1Present) {
        result.safeToUnloadTrayFromPos3 = false;
        result.unloadTrayPos3UnsafeReason = F("Tray 2 must be unloaded first");
        // No abort reason - this is a prerequisite check
    }
    
    //=============================================================================
    // Table 5: System State Validation
    //=============================================================================

    // 1. Command vs. actual state mismatch
    if (commandedPositionMm >= 0) {
        if (abs(state.currentPositionMm - commandedPositionMm) > POSITION_TOLERANCE_MM) {
            result.commandStateValid = false;
            result.stateValidationMessage = F("Position mismatch: commanded vs actual");
            
            // Add abort reason - this could indicate motor failure or blockage
            if (operationInProgress) {
                result.failureReason = ABORT_REASON_MOTOR_TIMEOUT;
                result.operationSequenceValid = false;
            }
        }
    }

    // 2. Tray position validation
    bool tray1ExpectedPresent = isMotorAtPosition1(state.currentPositionMm);
    bool tray2ExpectedPresent = isMotorAtPosition2(state.currentPositionMm);
    bool tray3ExpectedPresent = isMotorAtPosition3(state.currentPositionMm);

    // For tray 1
    if (tray1ExpectedPresent && !state.tray1Present) {
        result.trayPositionValid = false;
        result.stateValidationMessage = F("ERROR: Expected tray at position 1 is missing");
        
        if (operationInProgress) {
            result.operationSequenceValid = false;
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    } 
    else if (!tray1ExpectedPresent && state.tray1Present && state.currentPositionMm > POSITION_TOLERANCE_MM) {
        result.trayPositionValid = false;
        result.stateValidationMessage = F("ERROR: Unexpected object detected at position 1");
        
        if (operationInProgress) {
            result.operationSequenceValid = false;
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    }

    // For tray 2
    if (tray2ExpectedPresent && !state.tray2Present) {
        result.trayPositionValid = false;
        result.stateValidationMessage = F("ERROR: Expected tray at position 2 is missing");
        
        if (operationInProgress) {
            result.operationSequenceValid = false;
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    } 
    else if (!tray2ExpectedPresent && state.tray2Present) {
        result.trayPositionValid = false;
        result.stateValidationMessage = F("ERROR: Unexpected object detected at position 2");
        
        if (operationInProgress) {
            result.operationSequenceValid = false;
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    }

    // For tray 3
    if (tray3ExpectedPresent && !state.tray3Present) {
        result.trayPositionValid = false;
        result.stateValidationMessage = F("ERROR: Expected tray at position 3 is missing");
        
        if (operationInProgress) {
            result.operationSequenceValid = false;
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    } 
    else if (!tray3ExpectedPresent && state.tray3Present) {
        result.trayPositionValid = false;
        result.stateValidationMessage = F("ERROR: Unexpected object detected at position 3");
        
        if (operationInProgress) {
            result.operationSequenceValid = false;
            result.failureReason = ABORT_REASON_SENSOR_MISMATCH;
        }
    }

    // 4. Position target validation
    // Check if the target position is within allowed range
    if (hasCurrentTarget) {
        if (currentTargetPositionMm < 0 || currentTargetPositionMm > MAX_TRAVEL_MM) {
            result.targetPositionValid = false;
            result.stateValidationMessage = F("Target position out of range");
        }
    } else {
        // No target set yet
        result.targetPositionValid = false;
        result.stateValidationMessage = F("No target position set");
    }

    //=============================================================================
    // Table 6: Operational Sequence Validation
    //=============================================================================

    // 1. No new commands during operations
    // Check if a new command was received while an operation is in progress
    if (operationInProgress) {
        if (newCommandReceived) {
            result.safeToAcceptNewCommand = false;
            result.operationSequenceMessage = F("Operation in progress, cannot accept new command");
        }
    }

    // 2. Operation timeout
    // Check if the current operation has exceeded its timeout
    if (operationInProgress && millis() - operationStartTime > operationTimeoutMs) {
        result.operationWithinTimeout = false;
        result.operationSequenceMessage = F("Operation exceeded timeout");
        result.failureReason = ABORT_REASON_OPERATION_TIMEOUT;  // Add this line
    }

    // 3. Operation state mismatch
    // Check if the current operation is in the expected step of its sequence
    if (operationInProgress && currentOperationStep != expectedOperationStep) {
        result.operationSequenceValid = false;
        result.operationSequenceMessage = F("Operation sequence mismatch");
        result.failureReason = ABORT_REASON_SENSOR_MISMATCH;  // Add this line - using SENSOR_MISMATCH as the closest match
    }

    return result;
}

// Function to print out safety validation results
void printSafetyStatus(const SafetyValidationResult &result, Print* output) {
    output->println(F("[SAFETY] Validation Results:"));
    
    // Movement safety
    output->print(F("  Motor Movement: "));
    if (result.safeToMove) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.moveUnsafeReason);
    }
    
    // Tray locking safety
    output->println(F("  Tray Locking:"));
    output->print(F("    Tray 1: "));
    if (result.safeToLockTray1) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.tray1LockUnsafeReason);
    }
    
    output->print(F("    Tray 2: "));
    if (result.safeToLockTray2) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.tray2LockUnsafeReason);
    }
    
    output->print(F("    Tray 3: "));
    if (result.safeToLockTray3) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.tray3LockUnsafeReason);
    }
    
    // Shuttle actuation safety
    output->println(F("  Shuttle Control:"));
    output->print(F("    Lock: "));
    if (result.safeToLockShuttle) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.shuttleLockUnsafeReason);
    }
    
    output->print(F("    Unlock: "));
    if (result.safeToUnlockShuttle) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.shuttleUnlockUnsafeReason);
    }
    
    // System state validation status
    output->println(F("\n  System State Validation:"));
    output->print(F("    Command/Actual State: "));
    if (result.commandStateValid) {
        output->println(F("VALID"));
    } else {
        output->print(F("INVALID - "));
        output->println(result.stateValidationMessage);
    }

    output->print(F("    Tray Positions: "));
    if (result.trayPositionValid) {
        output->println(F("VALID"));
    } else {
        output->print(F("INVALID - "));
        output->println(result.stateValidationMessage);
    }

    output->print(F("    Target Position: "));
    if (result.targetPositionValid) {
        output->println(F("VALID"));
    } else {
        output->print(F("INVALID - "));
        output->println(result.stateValidationMessage);
    }

    // Operational sequence validation
    output->println(F("\n  Operational Sequence:"));
    output->print(F("    Accept New Commands: "));
    if (result.safeToAcceptNewCommand) {
        output->println(F("SAFE"));
    } else {
        output->print(F("UNSAFE - "));
        output->println(result.operationSequenceMessage);
    }

    output->print(F("    Operation Timing: "));
    if (result.operationWithinTimeout) {
        output->println(F("WITHIN TIMEOUT"));
    } else {
        output->print(F("TIMEOUT - "));
        output->println(result.operationSequenceMessage);
    }

    output->print(F("    Operation Sequence: "));
    if (result.operationSequenceValid) {
        output->println(F("VALID"));
    } else {
        output->print(F("INVALID - "));
        output->println(result.operationSequenceMessage);
    }
}

// Motor position helper functions
bool isMotorAtPosition1(double currentPosition) {
    // Use position definition from MotorController.h
    return abs(currentPosition - POSITION_1_MM) <= POSITION_TOLERANCE_MM;
}

bool isMotorAtPosition2(double currentPosition) {
    // Use position definition from MotorController.h
    return abs(currentPosition - POSITION_2_MM) <= POSITION_TOLERANCE_MM;
}

bool isMotorAtPosition3(double currentPosition) {
    // Use position definition from MotorController.h
    return abs(currentPosition - POSITION_3_MM) <= POSITION_TOLERANCE_MM;
}

// Add this function implementation near your other movement helper functions:
bool isMovingToPosition(double targetPosition, double currentTargetPositionMm) {
    return abs(currentTargetPositionMm - targetPosition) <= POSITION_TOLERANCE_MM;
}

// Check if the path is clear for movement between two positions
bool isPathClear(double startPosition, double targetPosition, const SystemState &state) {
    // If moving from position 1 to position 3, check that position 2 is clear
    if (isMotorAtPosition1(startPosition) && isMovingToPosition(POSITION_3_MM, targetPosition)) {
        if (state.tray2Present || state.tray3Present) {
            return false; // Path not clear
        }
        return true;
    }
    
    // If moving from position 3 to position 1, check that position 2 is clear
    if (isMotorAtPosition3(startPosition) && isMovingToPosition(POSITION_1_MM, targetPosition)) {
        if (state.tray2Present || state.tray1Present) {
            return false; // Path not clear
        }
        return true;
    }
    
    // If moving from position 1 to position 2, make sure position 2 is clear
    if (isMotorAtPosition1(startPosition) && isMovingToPosition(POSITION_2_MM, targetPosition)) {
        if (state.tray2Present) {
            return false; // Path not clear
        }
        return true;
    }
    
    // If moving from position 2 to position 1, make sure position 1 is clear
    if (isMotorAtPosition2(startPosition) && isMovingToPosition(POSITION_1_MM, targetPosition)) {
        if (state.tray1Present) {
            return false; // Path not clear
        }
        return true;
    }
    
    // If moving from position 2 to position 3, make sure position 3 is clear
    if (isMotorAtPosition2(startPosition) && isMovingToPosition(POSITION_3_MM, targetPosition)) {
        if (state.tray3Present) {
            return false; // Path not clear
        }
        return true;
    }
    
    // If moving from position 3 to position 2, make sure position 2 is clear
    if (isMotorAtPosition3(startPosition) && isMovingToPosition(POSITION_2_MM, targetPosition)) {
        if (state.tray2Present) {
            return false; // Path not clear
        }
        return true;
    }
    
    // Default - assume the path is clear if we don't have a specific case
    return true;
}

// Process tray operations based on the current operation state
void processTrayOperations() {
    // Only do something if operation is in progress
    if (!currentOperation.inProgress) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Check for timeout
    if (currentTime - currentOperation.startTime > operationTimeoutMs) {
        // Handle timeout
        Serial.println(F("[ERROR] Tray operation timeout"));
        currentOperation.inProgress = false;
        currentOperation.success = false;
        strncpy(currentOperation.message, "TIMEOUT", sizeof(currentOperation.message));
        return;
    }
    
    // Process based on operation type
    switch (currentOperation.type) {
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
void processTrayLoading() {
    // Current time for non-blocking timing checks
    unsigned long currentMillis = millis();
    
    // Target position based on tray tracking
    static double targetPosition = 0;
    static bool isShuttleNeeded = false;
    
    switch (currentOperationStep) {
        case 0: // Initial checks and determine destination
            {
                // Capture system state to get latest sensor readings
                SystemState state = captureSystemState();
                
                // Verify tray is at position 1 and locked
                if (!state.tray1Present) {
                    Serial.println(F("[ERROR] No tray detected at position 1"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "NO_TRAY", sizeof(currentOperation.message));
                    return;
                }
                
                if (!state.tray1Locked) {
                    Serial.println(F("[ERROR] Tray at position 1 not locked"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "TRAY_NOT_LOCKED", sizeof(currentOperation.message));
                    return;
                }
                
                // Determine target position based on tray tracking
                if (trayTracking.totalTraysInSystem == 0) {
                    // First tray - moves to position 3
                    targetPosition = POSITION_3_MM;
                    isShuttleNeeded = true;
                    Serial.println(F("[INFO] First tray - target is position 3"));
                } 
                else if (trayTracking.totalTraysInSystem == 1) {
                    // Second tray - moves to position 2
                    targetPosition = POSITION_2_MM;
                    isShuttleNeeded = true;
                    Serial.println(F("[INFO] Second tray - target is position 2"));
                } 
                else if (trayTracking.totalTraysInSystem == 2) {
                    // Third tray - stays at position 1
                    isShuttleNeeded = false;
                    Serial.println(F("[INFO] Third tray - keeping at position 1"));
                    // Skip to the final step for position 1
                    currentOperationStep = 8; // Special case - skip to completion
                    return;
                } 
                else {
                    // Error: too many trays
                    Serial.println(F("[ERROR] Cannot load more than 3 trays"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "MAX_TRAYS_REACHED", sizeof(currentOperation.message));
                    return;
                }
                
                // Check if target position is occupied
                if ((targetPosition == POSITION_2_MM && state.tray2Present) ||
                    (targetPosition == POSITION_3_MM && state.tray3Present)) {
                    Serial.println(F("[ERROR] Target position already occupied"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "TARGET_POSITION_OCCUPIED", sizeof(currentOperation.message));
                    return;
                }
                
                // Check if path is clear to target position
                if (!isPathClear(state.currentPositionMm, targetPosition, state)) {
                    Serial.println(F("[ERROR] Path to target position is blocked"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "PATH_BLOCKED", sizeof(currentOperation.message));
                    return;
                }
                
                // All checks pass, advance to next step
                currentOperationStep = 1;
                Serial.println(F("[MESSAGE] Starting tray advancement sequence"));
            }
            break;
            
        case 1: // Lock shuttle to grip tray (if needed)
            {
                if (!isShuttleNeeded) {
                    // Skip shuttle locking if not moving the tray
                    currentOperationStep = 3;
                    return;
                }
                
                // Start locking shuttle
                DoubleSolenoidValve *shuttleValve = getShuttleValve();
                if (shuttleValve) {
                    lockValve(*shuttleValve);
                    valveActuationStartTime = currentMillis;
                    Serial.println(F("[MESSAGE] Locking shuttle to grip tray"));
                } else {
                    Serial.println(F("[ERROR] Failed to access shuttle valve"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
                }
                
                // Advance to next step
                currentOperationStep = 2;
            }
            break;
            
        case 2: // Wait for shuttle lock valve actuation
            {
                // Wait for valve actuation time to elapse
                if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS) {
                    // Not enough time has elapsed, return and check again next cycle
                    return;
                }
                
                // Valve actuation time has elapsed, verify shuttle is locked
                SystemState state = captureSystemState();
                if (!state.shuttleLocked) {
                    Serial.println(F("[ERROR] Failed to lock shuttle"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "SHUTTLE_LOCK_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                
                // Shuttle is locked, proceed to unlock tray at position 1
                currentOperationStep = 3;
            }
            break;
            
        case 3: // Unlock tray at position 1
            {
                // Start unlocking tray 1
                DoubleSolenoidValve *valve = getTray1Valve();
                if (valve) {
                    unlockValve(*valve);
                    valveActuationStartTime = currentMillis;
                    Serial.println(F("[MESSAGE] Unlocking tray at position 1"));
                } else {
                    Serial.println(F("[ERROR] Failed to access tray 1 valve"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
                }
                
                // Advance to next step
                currentOperationStep = 4;
            }
            break;
            
        case 4: // Wait for tray unlock valve actuation
            {
                // Wait for valve actuation time to elapse
                if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS) {
                    // Not enough time has elapsed, return and check again next cycle
                    return;
                }
                
                // Valve actuation time has elapsed, verify unlock
                SystemState state = captureSystemState();
                if (state.tray1Locked) {
                    Serial.println(F("[ERROR] Failed to unlock tray at position 1"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "UNLOCK_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                
                // Set target position (only if we're moving the tray)
                if (isShuttleNeeded) {
                    // Move to the target position we determined in step 0
                    if (!moveToPositionMm(targetPosition)) {
                        Serial.println(F("[ERROR] Failed to start movement to target position"));
                        currentOperation.inProgress = false;
                        currentOperation.success = false;
                        strncpy(currentOperation.message, "MOVE_FAILURE", sizeof(currentOperation.message));
                        return;
                    }
                    
                    // Advance to next step
                    currentOperationStep = 5;
                    Serial.print(F("[MESSAGE] Moving tray to position "));
                    Serial.println(targetPosition);
                } else {
                    // No movement needed, skip to tray tracking update
                    currentOperationStep = 8;
                }
            }
            break;
            
        case 5: // Monitor motor movement and wait for completion
            {
                // Check if motor is still moving
                if (motorState == MOTOR_STATE_MOVING) {
                    // Motor still moving, wait
                    return;
                }
                
                // Motor has stopped, verify position
                SystemState state = captureSystemState();
                bool reachedTarget = false;
                
                if (targetPosition == POSITION_2_MM) {
                    reachedTarget = isMotorAtPosition2(state.currentPositionMm);
                } else if (targetPosition == POSITION_3_MM) {
                    reachedTarget = isMotorAtPosition3(state.currentPositionMm);
                }
                
                if (!reachedTarget) {
                    Serial.println(F("[ERROR] Motor did not reach target position"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "POSITION_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                
                // Unlock shuttle now that we've reached the destination
                DoubleSolenoidValve *shuttleValve = getShuttleValve();
                if (shuttleValve) {
                    unlockValve(*shuttleValve);
                    valveActuationStartTime = currentMillis;
                    Serial.println(F("[MESSAGE] Unlocking shuttle to release tray"));
                } else {
                    Serial.println(F("[ERROR] Failed to access shuttle valve"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
                }
                
                // Advance to next step
                currentOperationStep = 6;
            }
            break;
            
        case 6: // Wait for shuttle unlock valve actuation
            {
                // Wait for valve actuation time to elapse
                if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS) {
                    // Not enough time has elapsed, return and check again next cycle
                    return;
                }
                
                // Valve actuation time has elapsed, verify shuttle is unlocked
                SystemState state = captureSystemState();
                if (state.shuttleLocked) {
                    Serial.println(F("[ERROR] Failed to unlock shuttle"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "SHUTTLE_UNLOCK_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                
                // Lock tray at target position
                DoubleSolenoidValve *valve = NULL;
                
                if (targetPosition == POSITION_2_MM) {
                    valve = getTray2Valve();
                } else if (targetPosition == POSITION_3_MM) {
                    valve = getTray3Valve();
                }
                
                if (valve) {
                    lockValve(*valve);
                    valveActuationStartTime = currentMillis;
                    Serial.print(F("[MESSAGE] Locking tray at position "));
                    Serial.println(targetPosition);
                } else {
                    Serial.println(F("[ERROR] Failed to access target position valve"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "VALVE_ACCESS_ERROR", sizeof(currentOperation.message));
                }
                
                // Advance to next step
                currentOperationStep = 7;
            }
            break;
            
        case 7: // Wait for tray lock valve actuation at target
            {
                // Wait for valve actuation time to elapse
                if (currentMillis - valveActuationStartTime < VALVE_ACTUATION_TIME_MS) {
                    // Not enough time has elapsed, return and check again next cycle
                    return;
                }
                
                // Valve actuation time has elapsed, verify lock
                SystemState state = captureSystemState();
                bool lockSuccessful = false;
                
                if (targetPosition == POSITION_2_MM) {
                    lockSuccessful = state.tray2Locked;
                } else if (targetPosition == POSITION_3_MM) {
                    lockSuccessful = state.tray3Locked;
                }
                
                if (!lockSuccessful) {
                    Serial.println(F("[ERROR] Failed to lock tray at target position"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "LOCK_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                
                // Update tray tracking based on target position
                if (targetPosition == POSITION_2_MM) {
                    // Move from position 1 to 2
                    advanceTrays();
                } else if (targetPosition == POSITION_3_MM) {
                    // Move from position 1 to 3
                    moveTraysFromPos1ToPos3();
                }
                
                // Now return the conveyor to position 1
                if (!moveToPositionMm(POSITION_1_MM)) {
                    Serial.println(F("[ERROR] Failed to start movement to loading position"));
                    currentOperation.inProgress = false;
                    currentOperation.success = false;
                    strncpy(currentOperation.message, "RETURN_MOVE_FAILURE", sizeof(currentOperation.message));
                    return;
                }
                
                // Advance to next step (watching for conveyor return)
                currentOperationStep = 8;
                Serial.println(F("[MESSAGE] Returning to loading position"));
            }
            break;
            
        case 8: // Final stage - either wait for motor return or just update tracking
            {
                // If we didn't move (3rd tray stays at position 1), skip waiting
                if (!isShuttleNeeded) {
                    // Just update tray tracking for position 1
                    addTrayAtPosition1();
                } else {
                    // Wait for motor return to position 1
                    if (motorState == MOTOR_STATE_MOVING) {
                        // Motor still moving, wait
                        return;
                    }
                    
                    // Motor has stopped, verify position
                    SystemState state = captureSystemState();
                    if (!isMotorAtPosition1(state.currentPositionMm)) {
                        Serial.println(F("[ERROR] Motor did not return to position 1"));
                        currentOperation.inProgress = false;
                        currentOperation.success = false;
                        strncpy(currentOperation.message, "RETURN_FAILURE", sizeof(currentOperation.message));
                        return;
                    }
                }
                
                // Operation complete
                Serial.println(F("[MESSAGE] Tray loading completed successfully"));
                currentOperation.inProgress = false;
                currentOperation.success = true;
                strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));
                
                // End operation and reset target tracking
                endOperation();
            }
            break;
    }
}

// Process the tray unloading operation state machine
void processTrayUnloading() {
    // Similar implementation to processTrayLoading
    // Will be filled in later
    
    // For now, just finish immediately
    currentOperation.inProgress = false;
    currentOperation.success = true;
    strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));
}

// Process the tray advancement operation state machine
void processTrayAdvance() {
    // Similar implementation to processTrayLoading
    // Will be filled in later
    
    // For now, just finish immediately
    currentOperation.inProgress = false;
    currentOperation.success = true;
    strncpy(currentOperation.message, "SUCCESS", sizeof(currentOperation.message));
}

// Begin an operation and track it
void beginOperation() {
    operationInProgress = true;
    operationStartTime = millis();
    currentOperationStep = 0;
    expectedOperationStep = 0;
}

// End an operation and update target tracking
void endOperation() {
    operationInProgress = false;
    
    // Update target tracking in MotorController
    lastTargetPositionMm = currentTargetPositionMm;
    lastTargetPulses = currentTargetPulses;
    hasLastTarget = hasCurrentTarget;
    hasCurrentTarget = false;  // Clear current target
}

// Add near your other operation management functions (like endOperation)
const char* getAbortReasonString(AbortReason reason) {
    switch (reason) {
        case ABORT_REASON_ESTOP: return "Emergency Stop";
        case ABORT_REASON_MOTOR_TIMEOUT: return "Motor Movement Timeout";
        case ABORT_REASON_OPERATION_TIMEOUT: return "Operation Sequence Timeout";
        case ABORT_REASON_SENSOR_MISMATCH: return "Unexpected Sensor Reading";
        case ABORT_REASON_COMMUNICATION_LOSS: return "Robot Communication Loss";
        default: return "Unknown Reason";
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

    // Reset operation step counter
    currentOperationStep = 0;

    // End operation and update target tracking
    endOperation();
}
