#include "RailAutomation.h"
#include "Logging.h"

//=============================================================================
// CONSOLE OUTPUT FORMAT STRINGS
//=============================================================================
// Consolidated format strings for cleaner, more concise rail automation messaging

// Movement operation format strings
const char FMT_RAIL_MOVEMENT[] PROGMEM = "Rail %d → %s %s";
const char FMT_RAIL_POSITION_MOVEMENT[] PROGMEM = "Rail %d → %.1fmm %s";
const char FMT_RAIL_RELATIVE_MOVEMENT[] PROGMEM = "Rail %d → %+.1fmm %s";

// Initialization and control format strings  
const char FMT_RAIL_OPERATION[] PROGMEM = "Rail %d: %s";
const char FMT_SMART_HOMING[] PROGMEM = "Rail %d: Smart re-homing (saves %.1fs)";

// Safety and error format strings
const char FMT_COLLISION_ERROR[] PROGMEM = "Rail 1 blocked: Rail 2 %s with extended cylinder";
const char FMT_CYLINDER_SAFETY[] PROGMEM = "Collision zone movement - cylinder %s";

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
        return false;
    }
    
    if (!isMotorReady(railNumber)) {
        Console.error(F("MOTOR_NOT_READY"));
        return false;
    }
    
    if (!isHomingComplete(railNumber)) {
        Console.error(F("NOT_HOMED"));
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
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_CYLINDER_SAFETY, "retracting for safety");
    Console.serialInfo(msg);
    
    // Check air pressure before attempting retraction
    if (!isPressureSufficient()) {
        Console.error(F("INSUFFICIENT_PRESSURE"));
        return false;
    }
    
    // Attempt to retract cylinder
    ValveOperationResult result = retractCylinder();
    if (result != VALVE_OP_SUCCESS) {
        Console.error(F("CYLINDER_RETRACT_FAILED"));
        Console.serialInfo(getValveOperationResultName(result));
        return false;
    }
    
    // Verify the cylinder is actually retracted after operation
    if (!isCylinderActuallyRetracted()) {
        Console.error(F("CYLINDER_VERIFICATION_FAILED"));
        Console.serialInfo(F("Sensor indicates cylinder not retracted - check valve operation"));
        return false;
    }
    
    sprintf_P(msg, FMT_CYLINDER_SAFETY, "retracted - safe to proceed");
    Console.serialInfo(msg);
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
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 1, "WC1", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC1_REACHED_WITH_LABWARE") : F("WC1_REACHED"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}

bool moveRail1CarriageToWC2(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_WC2_PICKUP_DROPOFF_POS;
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 1, "WC2", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC2_REACHED_WITH_LABWARE") : F("WC2_REACHED"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}

bool moveRail1CarriageToStaging(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_STAGING_POS;
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 1, "Staging", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("STAGING_REACHED_WITH_LABWARE") : F("STAGING_REACHED"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}

bool moveRail1CarriageToHandoff(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_HANDOFF_POS;
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 1, "Handoff", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("HANDOFF_REACHED_WITH_LABWARE") : F("HANDOFF_REACHED"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
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
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 2, "WC3", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(2, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC3_REACHED_WITH_LABWARE") : F("WC3_REACHED"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}

bool moveRail2CarriageToHandoff(bool carriageLoaded) {
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(2)) return false;
    
    // Use helper function for cylinder safety (predefined moves always involve collision zone)
    if (!ensureCylinderRetractedForSafeMovement(true)) return false;
    
    PositionTarget targetPos = RAIL2_HANDOFF_POS;
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 2, "Handoff", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(2, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("HANDOFF_REACHED_WITH_LABWARE") : F("HANDOFF_REACHED"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}

//=============================================================================
// COMMON RAIL COMMAND HELPER FUNCTIONS
//=============================================================================
// These functions encapsulate the common commands shared between rail1 and rail2
// to eliminate code duplication while preserving rail-specific safety logic

bool executeRailInit(int railNumber) {
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Initializing motor...");
    Console.serialInfo(msg);

    // Initialize only the specific rail motor
    bool railReady = initRailMotor(railNumber);

    if (railReady) {
        Console.acknowledge(F("MOTOR_INITIALIZED"));
        return true;
    } else {
        Console.error(F("INIT_FAILED"));
        sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Motor initialization failed");
        Console.serialInfo(msg);
        return false;
    }
}

bool executeRailClearFault(int railNumber) {
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Clearing motor fault...");
    Console.serialInfo(msg);

    if (clearMotorFaultWithStatus(railNumber)) {
        Console.acknowledge(F("FAULT_CLEARED"));
        return true;
    } else {
        Console.error(F("CLEAR_FAULT_FAILED"));
        sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Fault clear failed - try power cycle");
        Console.serialInfo(msg);
        return false;
    }
}

bool executeRailAbort(int railNumber) {
    // Check if motor is initialized before attempting to abort
    if (!isMotorReady(railNumber)) {
        Console.error(F("MOTOR_NOT_READY"));
        return false;
    }

    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Aborting operation...");
    Console.serialInfo(msg);

    // Only meaningful to abort if we're moving or homing
    if (isMotorMoving(railNumber) || isHomingInProgress(railNumber)) {
        if (isHomingInProgress(railNumber)) {
            abortHoming(railNumber);
        } else {
            stopMotion(railNumber);
        }

        Console.acknowledge(F("OPERATION_ABORTED"));
        return true;
    } else {
        Console.error(F("NO_ACTIVE_OPERATION"));
        return false;
    }
}

bool executeRailStop(int railNumber) {
    // Check if motor is initialized
    if (!isMotorReady(railNumber)) {
        Console.error(F("MOTOR_NOT_READY"));
        return false;
    }

    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "EMERGENCY STOP!");
    Console.serialInfo(msg);

    // Execute emergency stop
    stopMotion(railNumber);

    Console.acknowledge(F("EMERGENCY_STOP_EXECUTED"));
    Console.serialInfo(F("Position accuracy lost - re-homing recommended"));

    return true;
}

bool executeRailHome(int railNumber) {
    // For the first-time homing or when smart homing isn't beneficial, use standard approach
    int32_t estimatedTimeSavingsMs;
    if (!isSmartHomingBeneficial(railNumber, &estimatedTimeSavingsMs)) {
        // Standard homing approach - perform all safety checks
        if (!checkRailMovementReadiness(railNumber)) return false;
        
        // Rail 1 specific safety: Check for Rail 2 collision risks at handoff intersection
        if (railNumber == 1) {
            // CRITICAL SAFETY: Rail 1 homes to position 0mm (handoff intersection)
            // Check if Rail 2 carriage is at handoff with extended cylinder (direct collision)
            if (isCarriageAtRail2Handoff() && isCylinderActuallyExtended()) {
                Console.error(F("COLLISION_RISK_RAIL2_HANDOFF"));
                char msg[LARGE_MSG_SIZE];
                sprintf_P(msg, FMT_COLLISION_ERROR, "at handoff");
                Console.serialInfo(msg);
                return false;
            }
            
            // Check if Rail 2 is anywhere in collision zone with extended cylinder
            double rail2Position = getMotorPositionMm(2);
            if (rail2Position >= RAIL2_COLLISION_ZONE_START && rail2Position <= RAIL2_COLLISION_ZONE_END && 
                isCylinderActuallyExtended()) {
                Console.error(F("COLLISION_RISK_RAIL2_COLLISION_ZONE"));
                char msg[LARGE_MSG_SIZE];
                sprintf_P(msg, FMT_COLLISION_ERROR, "in collision zone");
                Console.serialInfo(msg);
                return false;
            }
        }
        
        // Rail 2 specific safety: Use helper function for cylinder safety (homing always involves collision zone)
        if (railNumber == 2) {
            if (!ensureCylinderRetractedForSafeMovement(true)) return false;
        }
        
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Homing...");
        Console.serialInfo(msg);
        
        if (initiateHomingSequence(railNumber)) {
            Console.acknowledge(F("HOMING_STARTED"));
            return true;
        } else {
            Console.error(F("HOMING_START_FAILED"));
            return false;
        }
    } else {
        // Smart homing approach for re-homing (skip movement readiness since rail is already homed)
        
        // Still perform critical safety checks for collisions
        if (railNumber == 1) {
            // Same collision safety checks as standard homing
            if (isCarriageAtRail2Handoff() && isCylinderActuallyExtended()) {
                Console.error(F("COLLISION_RISK_RAIL2_HANDOFF"));
                char msg[LARGE_MSG_SIZE];
                sprintf_P(msg, FMT_COLLISION_ERROR, "at handoff");
                Console.serialInfo(msg);
                return false;
            }
            
            double rail2Position = getMotorPositionMm(2);
            if (rail2Position >= RAIL2_COLLISION_ZONE_START && rail2Position <= RAIL2_COLLISION_ZONE_END && 
                isCylinderActuallyExtended()) {
                Console.error(F("COLLISION_RISK_RAIL2_COLLISION_ZONE"));
                char msg[LARGE_MSG_SIZE];
                sprintf_P(msg, FMT_COLLISION_ERROR, "in collision zone");
                Console.serialInfo(msg);
                return false;
            }
        }
        
        if (railNumber == 2) {
            if (!ensureCylinderRetractedForSafeMovement(true)) return false;
        }
        
        char msg[MEDIUM_MSG_SIZE];
        double estimatedTimeSavingsSeconds = estimatedTimeSavingsMs / 1000.0;
        sprintf_P(msg, FMT_SMART_HOMING, railNumber, estimatedTimeSavingsSeconds);
        Console.serialInfo(msg);
        
        if (initiateSmartHomingSequence(railNumber)) {
            Console.acknowledge(F("SMART_HOMING_STARTED"));
            return true;
        } else {
            Console.error(F("SMART_HOMING_START_FAILED"));
            // Fall back to standard homing
            sprintf_P(msg, FMT_RAIL_OPERATION, railNumber, "Smart homing failed - using standard");
            Console.serialInfo(msg);
            
            if (initiateHomingSequence(railNumber)) {
                Console.acknowledge(F("HOMING_STARTED"));
                return true;
            } else {
                Console.error(F("HOMING_START_FAILED"));
                return false;
            }
        }
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
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
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
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}
