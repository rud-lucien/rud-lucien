#include "RailAutomation.h"
#include "HandoffController.h"
#include "LabwareAutomation.h"
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
// CROSS-RAIL LABWARE DETECTION HELPER FUNCTIONS
//=============================================================================
// These functions help determine where labware is currently located to enable
// intelligent cross-rail transfers when needed

bool isLabwareCurrentlyOnRail1() {
    // Check if labware is detected at any Rail 1 position
    return isLabwarePresentAtWC1() || isLabwarePresentAtWC2() || 
           (isCarriageAtRail1Handoff() && isLabwarePresentAtHandoff());
}

bool isLabwareCurrentlyOnRail2() {
    // Check if labware is detected at any Rail 2 position
    // Include both carriage-mounted sensor and handoff area when Rail 2 carriage is at handoff
    bool rail2CarriageLabware = isLabwarePresentOnRail2();
    bool rail2AtHandoffWithLabware = false;
    
    // Only check handoff if CCIO is available (handoff sensors require CCIO)
    if (isCarriageAtRail2Handoff() && isLabwarePresentAtHandoff()) {
        rail2AtHandoffWithLabware = true;
    }
    
    return rail2CarriageLabware || rail2AtHandoffWithLabware;
}

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
        
        // Verify labware is actually present somewhere in the system when specified
        // Use the LabwareAutomation tracking system instead of raw sensor readings
        // because Rail 1 can only detect labware when carriage is at sensor locations
        bool rail1HasLabware = labwareSystem.rail1.hasLabware;
        bool rail2HasLabware = labwareSystem.rail2.hasLabware;
        
        if (!rail1HasLabware && !rail2HasLabware) {
            Console.error(F("LABWARE_NOT_DETECTED"));
            Console.serialInfo(F("  No labware detected on either rail - use 'labware audit' to verify state"));
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
    
    // Check if CCIO is available - cylinder operations require CCIO
    if (!isPressureSufficient()) {
        Console.error(F("INSUFFICIENT_PRESSURE"));
        Console.serialInfo(F("  Cylinder safety operations require adequate air pressure"));
        return false;
    }
    
    // Check if cylinder is already retracted
    if (isCylinderActuallyRetracted()) {
        return true; // Already safe
    }
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_CYLINDER_SAFETY, "retracting for safety");
    Console.serialInfo(msg);
    
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
    // SMART CROSS-RAIL LOGIC: Check if labware is on Rail 2 and needs transfer
    if (carriageLoaded && isLabwareCurrentlyOnRail2()) {
        Console.serialInfo(F("CROSS_RAIL_TRANSFER_REQUIRED: Labware detected on Rail 2, initiating transfer to WC1"));
        
        // Initiate handoff from Rail 2 to Rail 1 with WC1 as final destination
        HandoffResult handoffResult = startHandoff(HANDOFF_RAIL2_TO_RAIL1, DEST_WC1);
        
        if (handoffResult == HANDOFF_SUCCESS) {
            // Handoff initiated successfully - it will handle the complete transfer
            Console.acknowledge(F("HANDOFF_INITIATED: Rail 2 → Rail 1 → WC1 transfer started"));
            return true;
        } else {
            // Handoff failed to start
            Console.error(F("HANDOFF_START_FAILED: Cannot initiate cross-rail transfer"));
            char errorMsg[MEDIUM_MSG_SIZE];
            sprintf_P(errorMsg, PSTR("Handoff error: %s"), getHandoffResultName(handoffResult));
            Console.serialInfo(errorMsg);
            return false;
        }
    }
    
    // STANDARD RAIL 1 LOGIC: Labware already on Rail 1 or empty movement
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_WC1_PICKUP_DROPOFF_POS;
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 1, "WC1", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC1_REACHED_WITH_LABWARE") : F("WC1_REACHED"));
        
        // Update operation counters for successful labware operations
        if (carriageLoaded) {
            incrementDeliveryCounter(); // Delivering labware to WC1
        } else {
            incrementPickupCounter();   // Picking up labware from WC1
        }
        
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        return false;
    }
}

bool moveRail1CarriageToWC2(bool carriageLoaded) {
    // SMART CROSS-RAIL LOGIC: Check if labware is on Rail 2 and needs transfer
    if (carriageLoaded && isLabwareCurrentlyOnRail2()) {
        Console.serialInfo(F("CROSS_RAIL_TRANSFER_REQUIRED: Labware detected on Rail 2, initiating transfer to WC2"));
        
        // Initiate handoff from Rail 2 to Rail 1 with WC2 as final destination
        HandoffResult handoffResult = startHandoff(HANDOFF_RAIL2_TO_RAIL1, DEST_WC2);
        
        if (handoffResult == HANDOFF_SUCCESS) {
            // Handoff initiated successfully - it will handle the complete transfer
            Console.acknowledge(F("HANDOFF_INITIATED: Rail 2 → Rail 1 → WC2 transfer started"));
            return true;
        } else {
            // Handoff failed to start
            Console.error(F("HANDOFF_START_FAILED: Cannot initiate cross-rail transfer"));
            char errorMsg[MEDIUM_MSG_SIZE];
            sprintf_P(errorMsg, PSTR("Handoff error: %s"), getHandoffResultName(handoffResult));
            Console.serialInfo(errorMsg);
            return false;
        }
    }
    
    // STANDARD RAIL 1 LOGIC: Labware already on Rail 1 or empty movement
    // Use existing helper functions for validation
    if (!checkRailMovementReadiness(1)) return false;
    
    PositionTarget targetPos = RAIL1_WC2_PICKUP_DROPOFF_POS;
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_MOVEMENT, 1, "WC2", carriageLoaded ? "with labware" : "empty");
    Console.serialInfo(msg);
    
    // Execute the movement
    if (moveToPositionFromCurrent(1, targetPos, carriageLoaded)) {
        Console.acknowledge(carriageLoaded ? F("WC2_REACHED_WITH_LABWARE") : F("WC2_REACHED"));
        
        // Update operation counters for successful labware operations
        if (carriageLoaded) {
            incrementDeliveryCounter(); // Delivering labware to WC2
        } else {
            incrementPickupCounter();   // Picking up labware from WC2
        }
        
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
    // SMART CROSS-RAIL LOGIC: Check if labware is on Rail 1 and needs transfer
    if (carriageLoaded && isLabwareCurrentlyOnRail1()) {
        Console.serialInfo(F("CROSS_RAIL_TRANSFER_REQUIRED: Labware detected on Rail 1, initiating transfer to WC3"));
        
        // Initiate handoff from Rail 1 to Rail 2 with WC3 as final destination
        HandoffResult handoffResult = startHandoff(HANDOFF_RAIL1_TO_RAIL2, DEST_WC3);
        
        if (handoffResult == HANDOFF_SUCCESS) {
            // Handoff initiated successfully - it will handle the complete transfer
            Console.acknowledge(F("HANDOFF_INITIATED: Rail 1 → Rail 2 → WC3 transfer started"));
            return true;
        } else {
            // Handoff failed to start
            Console.error(F("HANDOFF_START_FAILED: Cannot initiate cross-rail transfer"));
            char errorMsg[MEDIUM_MSG_SIZE];
            sprintf_P(errorMsg, PSTR("Handoff error: %s"), getHandoffResultName(handoffResult));
            Console.serialInfo(errorMsg);
            return false;
        }
    }
    
    // STANDARD RAIL 2 LOGIC: Labware already on Rail 2 or empty movement
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
        
        // Update operation counters for successful labware operations
        if (carriageLoaded) {
            incrementDeliveryCounter(); // Delivering labware to WC3
        } else {
            incrementPickupCounter();   // Picking up labware from WC3
        }
        
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

//=============================================================================
// GOTO PREFLIGHT VALIDATION SYSTEM
//=============================================================================
// Comprehensive safety and readiness validation before automated goto operations
// This prevents unsafe movements and provides clear feedback about system issues

bool performGotoPreflightChecks(Location targetLocation, bool hasLabware) {
    Console.serialInfo(F("PREFLIGHT_CHECKS: Validating system state for automated movement"));
    
    bool allChecksPass = true;
    
    //-------------------------------------------------------------------------
    // 1. LABWARE AUTOMATION VALIDATION
    //-------------------------------------------------------------------------
    if (!isLabwareSystemReady()) {
        Console.error(F("PREFLIGHT_FAIL: Labware automation system not ready"));
        if (hasLabwareConflict()) {
            Console.serialInfo(F("  Issue: Dual labware conflict detected (both rails have labware)"));
            Console.serialInfo(F("  Solution: Use manual rail commands to resolve, then 'labware audit'"));
        } else {
            Console.serialInfo(F("  Issue: Automation not enabled"));
            Console.serialInfo(F("  Solution: Use 'labware audit' to validate and enable automation"));
        }
        return false;
    }
    
    //-------------------------------------------------------------------------
    // 2. EMERGENCY STOP VALIDATION
    //-------------------------------------------------------------------------
    if (isEStopActive()) {
        Console.error(F("PREFLIGHT_FAIL: Emergency stop is active"));
        return false;
    }
    
    //-------------------------------------------------------------------------
    // 3. RAIL HOMING VALIDATION  
    //-------------------------------------------------------------------------
    // Both rails must be homed for safe automated movement
    if (!isHomingComplete(1)) {
        Console.error(F("PREFLIGHT_FAIL: Rail 1 not homed (use: rail1 home)"));
        allChecksPass = false;
    }
    
    if (!isHomingComplete(2)) {
        Console.error(F("PREFLIGHT_FAIL: Rail 2 not homed (use: rail2 home)"));
        allChecksPass = false;
    }
    
    //-------------------------------------------------------------------------
    // 4. RAIL SYSTEM READINESS
    //-------------------------------------------------------------------------
    // Check that both rail systems are ready for movement
    if (!checkRailMovementReadiness(1)) {
        Console.error(F("PREFLIGHT_FAIL: Rail 1 system not ready"));
        allChecksPass = false;
    }
    
    if (!checkRailMovementReadiness(2)) {
        Console.error(F("PREFLIGHT_FAIL: Rail 2 system not ready"));
        allChecksPass = false;
    }
    
    //-------------------------------------------------------------------------
    // 5. PNEUMATIC SYSTEM VALIDATION
    //-------------------------------------------------------------------------
    // Required for labware operations and collision avoidance
    if (!isPressureSufficient()) {
        Console.error(F("PREFLIGHT_FAIL: Insufficient air pressure"));
        allChecksPass = false;
    }
    
    //-------------------------------------------------------------------------
    // 6. LABWARE STATE CONSISTENCY VALIDATION
    //-------------------------------------------------------------------------
    // Verify that intended action matches actual labware state
    if (hasLabware) {
        // For with-labware operations, ensure labware exists somewhere in system
        bool rail1HasLabware = isLabwareCurrentlyOnRail1();
        bool rail2HasLabware = isLabwareCurrentlyOnRail2();
        
        if (!rail1HasLabware && !rail2HasLabware) {
            Console.error(F("PREFLIGHT_FAIL: No labware detected in system for 'with-labware' operation"));
            Console.serialInfo(F("  Solution: Use 'labware audit' to validate state or use 'no-labware' command"));
            allChecksPass = false;
        }
        
        // Check for dual labware conflict (both rails have labware)
        if (rail1HasLabware && rail2HasLabware) {
            Console.error(F("PREFLIGHT_FAIL: Dual labware detected - system cannot determine source"));
            Console.serialInfo(F("  Solution: Use manual rail commands to resolve, or 'labware reset' + audit"));
            allChecksPass = false;
        }
    }
    
    //-------------------------------------------------------------------------
    // 7. DESTINATION VALIDATION
    //-------------------------------------------------------------------------
    // Check if target location is reachable and safe
    switch (targetLocation) {
        case LOCATION_WC1:
            // WC1 is on Rail 1
            if (hasLabware && isLabwarePresentAtWC1()) {
                Console.error(F("PREFLIGHT_FAIL: WC1 already has labware (delivery blocked)"));
                Console.serialInfo(F("  Solution: Use 'goto wc1 no-labware' to pickup, or clear WC1 first"));
                allChecksPass = false;
            }
            if (!hasLabware && !isLabwarePresentAtWC1()) {
                Console.error(F("PREFLIGHT_FAIL: WC1 has no labware to pickup"));
                Console.serialInfo(F("  Solution: Use 'goto wc1 with-labware' to deliver, or verify WC1 has labware"));
                allChecksPass = false;
            }
            break;
            
        case LOCATION_WC2:
            // WC2 is on Rail 1
            if (hasLabware && isLabwarePresentAtWC2()) {
                Console.error(F("PREFLIGHT_FAIL: WC2 already has labware (delivery blocked)"));
                Console.serialInfo(F("  Solution: Use 'goto wc2 no-labware' to pickup, or clear WC2 first"));
                allChecksPass = false;
            }
            if (!hasLabware && !isLabwarePresentAtWC2()) {
                Console.error(F("PREFLIGHT_FAIL: WC2 has no labware to pickup"));
                Console.serialInfo(F("  Solution: Use 'goto wc2 with-labware' to deliver, or verify WC2 has labware"));
                allChecksPass = false;
            }
            break;
            
        case LOCATION_WC3:
            // WC3 is on Rail 2
            if (hasLabware && isLabwarePresentOnRail2()) {
                Console.error(F("PREFLIGHT_FAIL: WC3 already has labware (delivery blocked)"));
                Console.serialInfo(F("  Solution: Use 'goto wc3 no-labware' to pickup, or clear WC3 first"));
                allChecksPass = false;
            }
            if (!hasLabware && !isLabwarePresentOnRail2()) {
                Console.error(F("PREFLIGHT_FAIL: WC3 has no labware to pickup"));
                Console.serialInfo(F("  Solution: Use 'goto wc3 with-labware' to deliver, or verify WC3 has labware"));
                allChecksPass = false;
            }
            break;
            
        default:
            Console.error(F("PREFLIGHT_FAIL: Invalid destination location"));
            allChecksPass = false;
            break;
    }
    
    //-------------------------------------------------------------------------
    // 8. COLLISION ZONE SAFETY VALIDATION
    //-------------------------------------------------------------------------
    // Ensure Rail 2 cylinder state is safe for movements
    if (targetLocation == LOCATION_WC3) {
        // Moving to WC3 requires Rail 2 collision zone safety
        double rail2Position = getMotorPositionMm(2);
        bool inCollisionZone = (rail2Position >= RAIL2_COLLISION_ZONE_START && 
                               rail2Position <= RAIL2_COLLISION_ZONE_END);
        
        if (inCollisionZone && !isCylinderActuallyRetracted()) {
            Console.error(F("PREFLIGHT_FAIL: Rail 2 cylinder extended in collision zone"));
            Console.serialInfo(F("  Solution: Use 'rail2 retract' to ensure safe movement"));
            allChecksPass = false;
        }
    }
    
    //-------------------------------------------------------------------------
    // FINAL VALIDATION RESULT
    //-------------------------------------------------------------------------
    if (allChecksPass) {
        Console.acknowledge(F("PREFLIGHT_PASS: All systems ready for automated movement"));
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("Target: %s | Mode: %s"), 
                 getLocationName(targetLocation), 
                 hasLabware ? "with-labware" : "no-labware");
        Console.serialInfo(msg);
        return true;
    } else {
        Console.error(F("PREFLIGHT_FAIL: System not ready for automated movement"));
        Console.serialInfo(F("Address the issues above before using goto commands"));
        Console.serialInfo(F("Alternative: Use manual rail commands for direct control"));
        return false;
    }
}
