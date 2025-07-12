#include "RailAutomation.h"
#include "Logging.h"

//=============================================================================
// REUSABLE RAIL AUTOMATION HELPER FUNCTIONS
//=============================================================================
// These functions are designed to be rail-agnostic and reusable across:
// - Rail 1 and Rail 2 manual commands
// - Future automated movement functions
// - Handoff and transfer operations

bool checkRailMovementReadiness(int railNumber) {
    if (isEStopActive()) {
        Console.error(F("ESTOP_ACTIVE"));
        Console.serialInfo(F("Cannot move while emergency stop is active"));
        return false;
    }
    
    if (!isMotorReady(railNumber)) {
        Console.error(F("MOTOR_NOT_READY"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 motor not ready - check initialization") : F("Rail 2 motor not ready - check initialization"));
        return false;
    }
    
    if (!isHomingComplete(railNumber)) {
        Console.error(F("NOT_HOMED"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 must be homed first") : F("Rail 2 must be homed first"));
        return false;
    }
    
    return true;
}

bool parseAndValidateLabwareParameter(char* param, bool& carriageLoaded) {
    if (param == NULL) {
        Console.error(F("Missing labware parameter. Use: with-labware or no-labware"));
        return false;
    }
    
    // Convert parameter to lowercase for case-insensitive comparison
    for (int i = 0; param[i]; i++) {
        param[i] = tolower(param[i]);
    }
    
    if (strcmp(param, "with-labware") == 0) {
        carriageLoaded = true;
        
        // Verify labware is actually present when specified
        if (!isLabwarePresentAtHandoff() && !isLabwarePresentAtWC3()) {
            Console.error(F("LABWARE_NOT_DETECTED"));
            Console.serialInfo(F("No labware detected at current position. Check labware sensors or use 'no-labware'"));
            return false;
        }
    } else if (strcmp(param, "no-labware") == 0) {
        carriageLoaded = false;
    } else {
        Console.error(F("Invalid labware parameter. Use: with-labware or no-labware"));
        return false;
    }
    
    return true;
}

bool ensureCylinderRetractedForSafeMovement(bool movementInCollisionZone) {
    // Only perform cylinder safety checks if movement involves collision zone
    if (!movementInCollisionZone) {
        return true; // No collision zone involvement, cylinder state doesn't matter
    }
    
    // Check if cylinder is already retracted
    if (isCylinderActuallyRetracted()) {
        return true; // Already safe
    }
    
    Console.serialInfo(F("SAFETY: Movement involves collision zone - retracting cylinder..."));
    
    // Check air pressure before attempting retraction
    if (!isPressureSufficient()) {
        Console.error(F("INSUFFICIENT_PRESSURE"));
        Console.serialInfo(F("Cannot retract cylinder - insufficient air pressure for safe movement"));
        return false;
    }
    
    // Attempt to retract cylinder
    ValveOperationResult result = retractCylinder();
    if (result != VALVE_OP_SUCCESS) {
        Console.error(F("CYLINDER_RETRACT_FAILED"));
        Console.serialInfo(F("Failed to retract cylinder before movement - cannot proceed safely:"));
        Console.serialInfo(getValveOperationResultName(result));
        return false;
    }
    
    // Verify the cylinder is actually retracted after operation
    if (!isCylinderActuallyRetracted()) {
        Console.error(F("CYLINDER_VERIFICATION_FAILED"));
        Console.serialInfo(F("Cylinder operation completed but sensors indicate cylinder is not retracted"));
        Console.serialInfo(F("Check valve operation and sensor readings before proceeding"));
        return false;
    }
    
    Console.serialInfo(F("Cylinder successfully retracted - safe to proceed with movement"));
    return true;
}

//=============================================================================
// RAIL 1 SPECIFIC AUTOMATED MOVEMENT FUNCTIONS
//=============================================================================
// These functions encapsulate the complete movement logic for Rail 1 predefined positions
// Including all safety checks and movement execution

bool moveRail1CarriageToWC1(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_WC1_PICKUP_DROPOFF_POS;
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to WC1 position...") :
        F("Moving empty carriage to WC1 position..."));
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC1_REACHED_WITH_LABWARE") : F("WC1_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to WC1 position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to WC1 - check motor status"));
        return false;
    }
}

bool moveRail1CarriageToWC2(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_WC2_PICKUP_DROPOFF_POS;
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to WC2 position...") :
        F("Moving empty carriage to WC2 position..."));
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC2_REACHED_WITH_LABWARE") : F("WC2_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to WC2 position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to WC2 - check motor status"));
        return false;
    }
}

bool moveRail1CarriageToStaging(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_STAGING_POS;
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to staging position...") :
        F("Moving empty carriage to staging position..."));
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("STAGING_REACHED_WITH_LABWARE") : F("STAGING_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to staging position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to staging - check motor status"));
        return false;
    }
}

bool moveRail1CarriageToHandoff(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_HANDOFF_POS;
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to handoff position...") :
        F("Moving empty carriage to handoff position..."));
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("HANDOFF_REACHED_WITH_LABWARE") : F("HANDOFF_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to handoff position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to handoff - check motor status"));
        return false;
    }
}

//=============================================================================
// RAIL 2 SPECIFIC AUTOMATED MOVEMENT FUNCTIONS
//=============================================================================
// These functions encapsulate the complete movement logic for Rail 2 predefined positions
// Including all safety checks, cylinder retraction, and movement execution

bool moveRail2CarriageToWC3(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(2)) return false;
    
    // Use helper function for cylinder safety (predefined moves always involve collision zone)
    if (!ensureCylinderRetractedForSafeMovement(true)) return false;
    
    PositionTarget targetPos = RAIL2_WC3_PICKUP_DROPOFF_POS;
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to WC3 position...") :
        F("Moving empty carriage to WC3 position..."));
    
    // Execute the movement
    if (moveToPositionFromCurrent(2, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC3_REACHED_WITH_LABWARE") : F("WC3_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to WC3 position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to WC3 - check motor status"));
        return false;
    }
}

bool moveRail2CarriageToHandoff(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(2)) return false;
    
    // Use helper function for cylinder safety (predefined moves always involve collision zone)
    if (!ensureCylinderRetractedForSafeMovement(true)) return false;
    
    PositionTarget targetPos = RAIL2_HANDOFF_POS;
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to handoff position...") :
        F("Moving empty carriage to handoff position..."));
    
    // Execute the movement
    if (moveToPositionFromCurrent(2, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("HANDOFF_REACHED_WITH_LABWARE") : F("HANDOFF_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to handoff position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to handoff - check motor status"));
        return false;
    }
}

//=============================================================================
// COMMON RAIL COMMAND HELPER FUNCTIONS
//=============================================================================
// These functions encapsulate the common commands shared between rail1 and rail2
// to eliminate code duplication while preserving rail-specific safety logic

bool executeRailInit(int railNumber) {
    Console.serialInfo(railNumber == 1 ? F("Initializing Rail 1 motor...") : F("Initializing Rail 2 motor..."));

    // Rail 2 specific diagnostic
    if (railNumber == 2) {
        Console.serialInfo(F("[DIAGNOSTIC] Checking Rail 2 motor state before initialization"));
    }

    // Initialize only the specific rail motor
    bool railReady = initRailMotor(railNumber);

    if (railReady) {
        Console.acknowledge(F("MOTOR_INITIALIZED"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 motor initialized successfully") : F("Rail 2 motor initialized successfully"));
        return true;
    } else {
        Console.error(F("INIT_FAILED"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 motor initialization failed.") : F("Rail 2 motor initialization failed."));
        Console.serialInfo(F("Check connections and power."));
        return false;
    }
}

bool executeRailClearFault(int railNumber) {
    Console.serialInfo(railNumber == 1 ? F("Attempting to clear Rail 1 motor fault...") : F("Attempting to clear Rail 2 motor fault..."));

    if (clearMotorFaultWithStatus(railNumber)) {
        Console.acknowledge(F("FAULT_CLEARED"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 motor fault cleared successfully") : F("Rail 2 motor fault cleared successfully"));
        return true;
    } else {
        Console.error(F("CLEAR_FAULT_FAILED"));
        Console.serialInfo(railNumber == 1 ? F("Failed to clear Rail 1 motor fault") : F("Failed to clear Rail 2 motor fault"));
        Console.serialInfo(F("Motor may still be in fault state."));
        Console.serialInfo(F("Try power cycling the system if fault persists."));
        return false;
    }
}

bool executeRailAbort(int railNumber) {
    // Check if motor is initialized before attempting to abort
    if (!isMotorReady(railNumber)) {
        Console.error(F("MOTOR_NOT_READY"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 motor is not initialized. Nothing to abort.") : F("Rail 2 motor is not initialized. Nothing to abort."));
        return false;
    }

    Console.serialInfo(railNumber == 1 ? F("Aborting current Rail 1 operation...") : F("Aborting current Rail 2 operation..."));

    // Only meaningful to abort if we're moving or homing
    if (isMotorMoving(railNumber) || isHomingInProgress(railNumber)) {
        if (isHomingInProgress(railNumber)) {
            abortHoming(railNumber);
        } else {
            stopMotion(railNumber);
        }

        Console.acknowledge(F("OPERATION_ABORTED"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 operation aborted successfully") : F("Rail 2 operation aborted successfully"));
        return true;
    } else {
        Console.error(F("NO_ACTIVE_OPERATION"));
        Console.serialInfo(F("No active operation to abort"));
        return false;
    }
}

bool executeRailStop(int railNumber) {
    // Check if motor is initialized
    if (!isMotorReady(railNumber)) {
        Console.error(F("MOTOR_NOT_READY"));
        Console.serialInfo(railNumber == 1 ? F("Rail 1 motor is not initialized. Cannot perform stop.") : F("Rail 2 motor is not initialized. Cannot perform stop."));
        return false;
    }

    Console.serialInfo(railNumber == 1 ? F("EMERGENCY STOP initiated for Rail 1!") : F("EMERGENCY STOP initiated for Rail 2!"));

    // Execute emergency stop
    stopMotion(railNumber);

    Console.acknowledge(F("EMERGENCY_STOP_EXECUTED"));
    Console.serialInfo(railNumber == 1 ? F("Rail 1 motor movement halted. Position may no longer be accurate.") : F("Rail 2 motor movement halted. Position may no longer be accurate."));
    Console.serialInfo(F("Re-homing recommended after emergency stop."));

    return true;
}

bool executeRailHome(int railNumber) {
    if (!checkRailMovementReadiness(railNumber)) return false;
    
    // Rail 2 specific safety: Use helper function for cylinder safety (homing always involves collision zone)
    if (railNumber == 2) {
        if (!ensureCylinderRetractedForSafeMovement(true)) return false;
    }
    
    Console.serialInfo(railNumber == 1 ? F("Initiating Rail 1 homing sequence...") : F("Initiating Rail 2 homing sequence..."));
    if (initiateHomingSequence(railNumber)) {
        Console.acknowledge(F("HOMING_STARTED"));
        Console.serialInfo(F("Homing sequence initiated successfully"));
        return true;
    } else {
        Console.error(F("HOMING_START_FAILED"));
        Console.serialInfo(F("Failed to start homing sequence - check motor status"));
        return false;
    }
}

bool executeRailMoveToPosition(int railNumber, double positionMm, bool carriageLoaded) {
    // Use helper functions for common validation
    if (!checkRailMovementReadiness(railNumber)) return false;
    
    // Rail 2 specific collision zone safety logic
    if (railNumber == 2) {
        // CRITICAL SAFETY: Check if any part of the movement path requires cylinder retraction to prevent Rail 1 collision
        // Collision zone is between RAIL2_COLLISION_ZONE_START and RAIL2_COLLISION_ZONE_END
        // We must check current position, target position, and the entire movement path
        double currentPos = getMotorPositionMm(2);
        bool movementInCollisionZone = false;
        
        // Check if current position is in collision zone
        if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true;
        }
        
        // Check if target position is in collision zone
        if (positionMm >= RAIL2_COLLISION_ZONE_START && positionMm <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true;
        }
        
        // Check if movement path crosses collision zone (even if start/end are outside zone)
        if (currentPos < RAIL2_COLLISION_ZONE_START && positionMm >= RAIL2_COLLISION_ZONE_START) {
            movementInCollisionZone = true; // Entering collision zone
        }
        if (currentPos > RAIL2_COLLISION_ZONE_END && positionMm <= RAIL2_COLLISION_ZONE_END) {
            movementInCollisionZone = true; // Entering collision zone from far side
        }
        if ((currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END) && 
            (positionMm < RAIL2_COLLISION_ZONE_START || positionMm > RAIL2_COLLISION_ZONE_END)) {
            movementInCollisionZone = true; // Exiting collision zone
        }
        
        // Use helper function for cylinder safety
        if (!ensureCylinderRetractedForSafeMovement(movementInCollisionZone)) return false;
    }
    
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware to absolute position...") :
        F("Moving empty carriage to absolute position..."));
    
    // Execute the movement
    if (moveToPositionMm(railNumber, positionMm, carriageLoaded)) {
        Console.acknowledge(F("POSITION_REACHED"));
        Console.serialInfo(F("Carriage successfully moved to target position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to target position - check motor status"));
        return false;
    }
}

bool executeRailMoveRelative(int railNumber, double distanceMm, bool carriageLoaded) {
    // Use helper functions for common validation
    if (!checkRailMovementReadiness(railNumber)) return false;
    
    // Rail 2 specific collision zone safety logic
    if (railNumber == 2) {
        // CRITICAL SAFETY: Check if any part of the movement path requires cylinder retraction to prevent Rail 1 collision
        // Calculate target position and check if movement involves collision zone (500-700mm)
        double currentPos = getMotorPositionMm(2);
        double calculatedTargetPos = currentPos + distanceMm;
        bool movementInCollisionZone = false;
        
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
    }
    
    Console.serialInfo(carriageLoaded ? 
        F("Moving carriage with labware relative distance...") :
        F("Moving empty carriage relative distance..."));
    
    // Execute the movement
    if (moveRelativeManual(railNumber, distanceMm, carriageLoaded)) {
        Console.acknowledge(F("MOVE_COMPLETED"));
        Console.serialInfo(F("Carriage successfully moved relative distance"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage relative distance - check motor status"));
        return false;
    }
}
