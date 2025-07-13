#include "HandoffController.h"
#include "LabwareAutomation.h"
#include "MotorController.h"  // Add for position validation functions
#include "PositionConfig.h"   // Add for position constants
#include "Logging.h"
#include "Sensors.h"
#include "ValveController.h"
#include "Utils.h"

//=============================================================================
// CONSOLE OUTPUT FORMAT STRINGS
//=============================================================================
// Consolidated format strings for cleaner, more concise handoff messaging

// Handoff operation format strings
const char FMT_HANDOFF_INIT[] PROGMEM = "Handoff: %s → %s";
const char FMT_HANDOFF_STATE[] PROGMEM = "Handoff: %s";
const char FMT_HANDOFF_ERROR[] PROGMEM = "Handoff error: %s";
const char FMT_HANDOFF_COLLISION[] PROGMEM = "Collision risk: %s has labware";

//=============================================================================
// GLOBAL HANDOFF STATE
//=============================================================================

HandoffState_t handoffState = {
    HANDOFF_IDLE,                    // currentState
    HANDOFF_SUCCESS,                 // currentResult
    HANDOFF_RAIL1_TO_RAIL2,         // direction
    DEST_WC3,                       // destination
    0,                              // operationStartTime
    HANDOFF_TIMEOUT_DEFAULT         // currentTimeout
};

//=============================================================================
// MAIN HANDOFF FUNCTIONS
//=============================================================================

HandoffResult startHandoff(HandoffDirection dir, HandoffDestination dest) {
    // Validate parameters
    if (!validateHandoffParameters(dir, dest)) {
        handoffState.currentResult = HANDOFF_ERROR_INVALID_PARAMS;
        Console.error(F("INVALID_HANDOFF_PARAMETERS"));
        return handoffState.currentResult;
    }
    
    // Check if handoff already in progress
    if (handoffState.currentState != HANDOFF_IDLE) {
        Console.error(F("HANDOFF_ALREADY_ACTIVE"));
        return HANDOFF_ERROR_SYSTEM_STATE;
    }
    
    // Check system readiness
    if (!checkHandoffSystemReadiness()) {
        handoffState.currentResult = HANDOFF_ERROR_SYSTEM_STATE;
        return handoffState.currentResult;
    }
    
    // CRITICAL: Check for collision safety - both carriages cannot have labware
    if (!checkHandoffCollisionSafety(dir, dest)) {
        handoffState.currentResult = HANDOFF_ERROR_COLLISION;
        return handoffState.currentResult;
    }
    
    // ENHANCED: Pre-flight position validation
    if (dir == HANDOFF_RAIL1_TO_RAIL2) {
        Console.serialInfo(F("HANDOFF_PREFLIGHT: Validating Rail 1 ready for handoff"));
        // Don't require exact handoff position yet, just validate rail is homed and operational
        if (!isHomingComplete(1)) {
            Console.error(F("HANDOFF_PREFLIGHT_FAILED: Rail 1 not homed"));
            handoffState.currentResult = HANDOFF_ERROR_POSITION;
            return handoffState.currentResult;
        }
    }
    else if (dir == HANDOFF_RAIL2_TO_RAIL1) {
        Console.serialInfo(F("HANDOFF_PREFLIGHT: Validating Rail 2 ready for handoff"));
        if (!isHomingComplete(2)) {
            Console.error(F("HANDOFF_PREFLIGHT_FAILED: Rail 2 not homed"));
            handoffState.currentResult = HANDOFF_ERROR_POSITION;
            return handoffState.currentResult;
        }
    }
    
    // Initialize handoff operation
    handoffState.direction = dir;
    handoffState.destination = dest;
    handoffState.currentState = HANDOFF_MOVING_SOURCE_TO_POS;
    handoffState.operationStartTime = millis();
    
    // Set timeout based on destination
    handoffState.currentTimeout = (dest == DEST_WC3) ? HANDOFF_TIMEOUT_WC3_EXT : HANDOFF_TIMEOUT_DEFAULT;
    
    char msg[MEDIUM_MSG_SIZE];
    const char* dirStr = (dir == HANDOFF_RAIL1_TO_RAIL2) ? "Rail1→Rail2" : "Rail2→Rail1";
    const char* destStr = (dest == DEST_WC1) ? "WC1" : 
                          (dest == DEST_WC2) ? "WC2" : 
                          (dest == DEST_STAGING) ? "Staging" : "WC3";
    sprintf_P(msg, FMT_HANDOFF_INIT, dirStr, destStr);
    Console.serialInfo(msg);
    
    Console.serialInfo(F("HANDOFF_INITIATED_WITH_VALIDATION: Position validation enabled"));
    
    return HANDOFF_SUCCESS;
}

HandoffResult updateHandoff() {
    // Check for timeout on any operation
    if (isHandoffOperationTimedOut()) {
        Console.error(F("HANDOFF_OPERATION_TIMEOUT"));
        
        char timeoutMsg[SMALL_MSG_SIZE];
        sprintf_P(timeoutMsg, PSTR("Handoff timed out in state: %s"), getHandoffStateName(handoffState.currentState));
        Console.serialInfo(timeoutMsg);
        
        handoffState.currentState = HANDOFF_ERROR;
        handoffState.currentResult = HANDOFF_ERROR_TIMEOUT;
        return handoffState.currentResult;
    }
    
    // Check for E-stop at any time
    if (isEStopActive()) {
        Console.error(F("EMERGENCY_STOP_ACTIVE"));
        Console.serialInfo(F("Handoff cancelled due to E-stop activation"));
        handoffState.currentState = HANDOFF_ERROR;
        handoffState.currentResult = HANDOFF_ERROR_ESTOP;
        return handoffState.currentResult;
    }
    
    // State machine
    switch (handoffState.currentState) {
        case HANDOFF_IDLE:
            // Nothing to do
            return HANDOFF_SUCCESS;
            
        case HANDOFF_MOVING_SOURCE_TO_POS:
            if (moveSourceRailToHandoffPosition()) {
                // ENHANCED: Validate source rail reached handoff position
                bool positionValid = false;
                
                if (handoffState.direction == HANDOFF_RAIL1_TO_RAIL2) {
                    if (validateRail1AtHandoffPosition()) {
                        Console.serialInfo(F("HANDOFF_RAIL1_POSITIONED: Rail 1 validated at handoff"));
                        positionValid = true;
                    } else {
                        Console.error(F("HANDOFF_POSITION_ERROR: Rail 1 failed position validation"));
                        handoffState.currentState = HANDOFF_ERROR;
                        handoffState.currentResult = HANDOFF_ERROR_POSITION;
                        return handoffState.currentResult;
                    }
                }
                else if (handoffState.direction == HANDOFF_RAIL2_TO_RAIL1) {
                    if (validateRail2AtHandoffPosition()) {
                        Console.serialInfo(F("HANDOFF_RAIL2_POSITIONED: Rail 2 validated at handoff"));
                        positionValid = true;
                    } else {
                        Console.error(F("HANDOFF_POSITION_ERROR: Rail 2 failed position validation"));
                        handoffState.currentState = HANDOFF_ERROR;
                        handoffState.currentResult = HANDOFF_ERROR_POSITION;
                        return handoffState.currentResult;
                    }
                }
                
                if (positionValid) {
                    char msg[MEDIUM_MSG_SIZE];
                    sprintf_P(msg, FMT_HANDOFF_STATE, "Extending cylinder");
                    Console.serialInfo(msg);
                    handoffState.currentState = HANDOFF_EXTENDING_CYLINDER;
                    handoffState.operationStartTime = millis(); // Reset timer for next phase
                }
            }
            break;
            
        case HANDOFF_EXTENDING_CYLINDER:
            {
                ValveOperationResult result = extendCylinder();
                if (result == VALVE_OP_SUCCESS && isCylinderActuallyExtended()) {
                    char msg[MEDIUM_MSG_SIZE];
                    sprintf_P(msg, FMT_HANDOFF_STATE, "Waiting for transfer");
                    Console.serialInfo(msg);
                    handoffState.currentState = HANDOFF_WAITING_TRANSFER;
                    handoffState.operationStartTime = millis(); // Reset timer for transfer phase
                } else if (result != VALVE_OP_SUCCESS) {
                    Console.error(F("CYLINDER_EXTENSION_FAILED"));
                    char errorMsg[MEDIUM_MSG_SIZE];
                    sprintf_P(errorMsg, FMT_HANDOFF_ERROR, getValveOperationResultName(result));
                    Console.serialInfo(errorMsg);
                    handoffState.currentState = HANDOFF_ERROR;
                    handoffState.currentResult = HANDOFF_ERROR_VALVE;
                }
            }
            break;
            
        case HANDOFF_WAITING_TRANSFER:
            if (verifyHandoffLabwareTransfer()) {
                char msg[MEDIUM_MSG_SIZE];
                sprintf_P(msg, FMT_HANDOFF_STATE, "Retracting cylinder");
                Console.serialInfo(msg);
                handoffState.currentState = HANDOFF_RETRACTING_CYLINDER;
                handoffState.operationStartTime = millis(); // Reset timer for retraction
            }
            break;
            
        case HANDOFF_RETRACTING_CYLINDER:
            {
                ValveOperationResult result = retractCylinder();
                if (result == VALVE_OP_SUCCESS && isCylinderActuallyRetracted()) {
                    char msg[MEDIUM_MSG_SIZE];
                    sprintf_P(msg, FMT_HANDOFF_STATE, "Moving to destination");
                    Console.serialInfo(msg);
                    handoffState.currentState = HANDOFF_MOVING_DEST_TO_TARGET;
                    handoffState.operationStartTime = millis(); // Reset timer for final movement
                } else if (result != VALVE_OP_SUCCESS) {
                    Console.error(F("CYLINDER_RETRACTION_FAILED"));
                    char errorMsg[MEDIUM_MSG_SIZE];
                    sprintf_P(errorMsg, FMT_HANDOFF_ERROR, getValveOperationResultName(result));
                    Console.serialInfo(errorMsg);
                    handoffState.currentState = HANDOFF_ERROR;
                    handoffState.currentResult = HANDOFF_ERROR_VALVE;
                }
            }
            break;
            
        case HANDOFF_MOVING_DEST_TO_TARGET:
            if (moveDestinationRailToTargetPosition()) {
                // ENHANCED: Validate destination positioning for both directions
                bool destinationValid = false;
                
                if (handoffState.direction == HANDOFF_RAIL1_TO_RAIL2) {
                    // Rail 2 moving to final destination (WC3)
                    if (validateRailReadyForHandoff(2, RAIL2_WC3_PICKUP_DROPOFF)) {
                        Console.serialInfo(F("HANDOFF_DEST_POSITIONED: Rail 2 validated at WC3"));
                        destinationValid = true;
                    } else {
                        Console.error(F("HANDOFF_DEST_ERROR: Rail 2 failed WC3 position validation"));
                        handoffState.currentState = HANDOFF_ERROR;
                        handoffState.currentResult = HANDOFF_ERROR_POSITION;
                        return handoffState.currentResult;
                    }
                }
                else if (handoffState.direction == HANDOFF_RAIL2_TO_RAIL1) {
                    // Rail 1 moving to final destination
                    if (handoffState.destination == DEST_WC1) {
                        if (validateRailReadyForHandoff(1, RAIL1_WC1_PICKUP_DROPOFF)) {
                            Console.serialInfo(F("HANDOFF_DEST_POSITIONED: Rail 1 validated at WC1"));
                            destinationValid = true;
                        } else {
                            Console.error(F("HANDOFF_DEST_ERROR: Rail 1 failed WC1 position validation"));
                            handoffState.currentState = HANDOFF_ERROR;
                            handoffState.currentResult = HANDOFF_ERROR_POSITION;
                            return handoffState.currentResult;
                        }
                    }
                    else if (handoffState.destination == DEST_WC2) {
                        if (validateRailReadyForHandoff(1, RAIL1_WC2_PICKUP_DROPOFF)) {
                            Console.serialInfo(F("HANDOFF_DEST_POSITIONED: Rail 1 validated at WC2"));
                            destinationValid = true;
                        } else {
                            Console.error(F("HANDOFF_DEST_ERROR: Rail 1 failed WC2 position validation"));
                            handoffState.currentState = HANDOFF_ERROR;
                            handoffState.currentResult = HANDOFF_ERROR_POSITION;
                            return handoffState.currentResult;
                        }
                    }
                    else if (handoffState.destination == DEST_STAGING) {
                        if (validateRailReadyForHandoff(1, RAIL1_STAGING_POSITION)) {
                            Console.serialInfo(F("HANDOFF_DEST_POSITIONED: Rail 1 validated at staging"));
                            destinationValid = true;
                        } else {
                            Console.error(F("HANDOFF_DEST_ERROR: Rail 1 failed staging position validation"));
                            handoffState.currentState = HANDOFF_ERROR;
                            handoffState.currentResult = HANDOFF_ERROR_POSITION;
                            return handoffState.currentResult;
                        }
                    }
                }
                
                if (destinationValid) {
                    // ENHANCED: Increment cross-rail counter with position validation
                    incrementCrossRailCounter();
                    
                    Console.acknowledge(F("HANDOFF_COMPLETED_WITH_VALIDATION: Cross-rail transfer successful"));
                    handoffState.currentState = HANDOFF_COMPLETED;
                    handoffState.currentResult = HANDOFF_SUCCESS;
                }
            }
            break;
            
        case HANDOFF_COMPLETED:
            // Operation complete - return to idle on next call
            handoffState.currentState = HANDOFF_IDLE;
            return HANDOFF_SUCCESS;
            
        case HANDOFF_ERROR:
            // Enhanced error reporting with proper string formatting
            {
                char errorMsg[100];
                sprintf_P(errorMsg, PSTR("HANDOFF_FAILED: %s"), getHandoffResultName(handoffState.currentResult));
                Console.error(errorMsg);
            }
            
            // Attempt safe recovery - retract cylinder if extended
            if (isCylinderActuallyExtended()) {
                Console.serialInfo(F("HANDOFF_RECOVERY: Retracting cylinder for safety"));
                retractCylinder();
            }
            
            return handoffState.currentResult;
    }
    
    return HANDOFF_SUCCESS; // Operation in progress
}

bool isHandoffInProgress() {
    return (handoffState.currentState != HANDOFF_IDLE && 
            handoffState.currentState != HANDOFF_COMPLETED && 
            handoffState.currentState != HANDOFF_ERROR);
}

HandoffState getCurrentHandoffState() {
    return handoffState.currentState;
}

HandoffResult getLastHandoffResult() {
    return handoffState.currentResult;
}

//=============================================================================
// ENHANCED POSITION VALIDATION FUNCTIONS
//=============================================================================
// Leverage existing MotorController position validation capabilities

bool validateRailReadyForHandoff(int railNumber, double expectedPosition) {
    // Leverage existing MotorController functions
    if (!isHomingComplete(railNumber)) {
        char errorMsg[80];
        sprintf_P(errorMsg, PSTR("POSITION_VALIDATION_FAILED: Rail %d not homed"), railNumber);
        Console.error(errorMsg);
        return false;
    }
    
    // Get current position and check tolerance manually
    double currentPos = getMotorPositionMm(railNumber);
    double tolerance = MOVEMENT_POSITION_TOLERANCE_MM;
    
    if (abs(currentPos - expectedPosition) > tolerance) {
        char errorMsg[120];
        sprintf_P(errorMsg, PSTR("POSITION_VALIDATION_FAILED: Rail %d at %.1fmm, expected %.1fmm"), 
                 railNumber, currentPos, expectedPosition);
        Console.error(errorMsg);
        return false;
    }
    
    char infoMsg[80];
    sprintf_P(infoMsg, PSTR("POSITION_VALIDATED: Rail %d ready at %.1fmm"), railNumber, expectedPosition);
    Console.serialInfo(infoMsg);
    return true;
}

bool validateRail1AtHandoffPosition() {
    // Rail 1 handoff position is at 0mm (home position)
    return validateRailReadyForHandoff(1, 0.0);
}

bool validateRail2AtHandoffPosition() {
    // Rail 2 handoff position - use the defined constant
    return validateRailReadyForHandoff(2, RAIL2_HANDOFF);
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

bool validateHandoffParameters(HandoffDirection dir, HandoffDestination dest) {
    // Validate direction-destination combinations
    if (dir == HANDOFF_RAIL1_TO_RAIL2 && dest != DEST_WC3) {
        Console.error(F("Invalid handoff: Rail1→Rail2 only supports WC3 destination"));
        return false;
    }
    
    if (dir == HANDOFF_RAIL2_TO_RAIL1 && dest == DEST_WC3) {
        Console.error(F("Invalid handoff: Rail2→Rail1 cannot target WC3"));
        return false;
    }
    
    return true;
}

bool checkHandoffSystemReadiness() {
    // Check E-stop
    if (isEStopActive()) {
        Console.error(F("EMERGENCY_STOP_ENGAGED"));
        return false;
    }
    
    // Check both rails ready
    if (!checkRailMovementReadiness(1)) {
        Console.error(F("RAIL1_SYSTEM_NOT_READY"));
        return false;
    }
    
    if (!checkRailMovementReadiness(2)) {
        Console.error(F("RAIL2_SYSTEM_NOT_READY"));
        return false;
    }
    
    // Check air pressure
    if (!isPressureSufficient()) {
        Console.error(F("AIR_PRESSURE_TOO_LOW"));
        return false;
    }
    
    return true;
}

bool checkHandoffCollisionSafety(HandoffDirection dir, HandoffDestination dest) {
    // Check for the dangerous scenario where both carriages have labware
    bool rail1HasLabware = isLabwarePresentAtWC1() || isLabwarePresentAtWC2();
    bool rail2HasLabware = isLabwarePresentAtWC3() || isLabwarePresentAtHandoff();
    
    if (rail1HasLabware && rail2HasLabware) {
        Console.error(F("CARRIAGE_COLLISION_RISK"));
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_HANDOFF_COLLISION, "both rails");
        Console.serialInfo(msg);
        return false;
    }
    
    // Verify source carriage actually has labware to transfer
    if (dir == HANDOFF_RAIL1_TO_RAIL2 && !rail1HasLabware) {
        Console.error(F("RAIL1_NO_LABWARE_TO_TRANSFER"));
        return false;
    }
    
    if (dir == HANDOFF_RAIL2_TO_RAIL1 && !rail2HasLabware) {
        Console.error(F("RAIL2_NO_LABWARE_TO_TRANSFER"));
        return false;
    }
    
    // Destination-specific collision checks
    if (dir == HANDOFF_RAIL2_TO_RAIL1) {
        bool destinationOccupied = false;
        
        switch (dest) {
            case DEST_WC1:
                destinationOccupied = isLabwarePresentAtWC1();
                break;
            case DEST_WC2:
                destinationOccupied = isLabwarePresentAtWC2();
                break;
            case DEST_STAGING:
                Console.error(F("Invalid destination: Staging is not a delivery target"));
                return false;
            default:
                Console.error(F("Invalid destination for Rail 2 → Rail 1 handoff"));
                return false;
        }
        
        if (destinationOccupied) {
            Console.error(F("TARGET_POSITION_OCCUPIED"));
            return false;
        }
    } else if (dir == HANDOFF_RAIL1_TO_RAIL2) {
        // Rail1 → Rail2 always goes to WC3
        if (isLabwarePresentAtWC3()) {
            Console.error(F("WC3_POSITION_OCCUPIED"));
            return false;
        }
    }
    
    return true;
}

bool moveSourceRailToHandoffPosition() {
    if (handoffState.direction == HANDOFF_RAIL1_TO_RAIL2) {
        // Moving Rail 1 to handoff - check if labware is present
        bool hasLabware = isLabwarePresentAtWC1() || isLabwarePresentAtWC2();
        return moveRail1CarriageToHandoff(hasLabware);
    } else {
        // Moving Rail 2 to handoff - check if labware is present  
        bool hasLabware = isLabwarePresentAtWC3() || isLabwarePresentAtHandoff();
        return moveRail2CarriageToHandoff(hasLabware);
    }
}

bool moveDestinationRailToTargetPosition() {
    if (handoffState.direction == HANDOFF_RAIL1_TO_RAIL2) {
        // Moving Rail 2 to WC3 (collision already checked in startHandoff)
        return moveRail2CarriageToWC3(true); // Labware should be present after transfer
    } else {
        // Moving Rail 1 to target destination (collision already checked in startHandoff)
        bool hasLabware = true; // Labware should be present after transfer
        
        switch (handoffState.destination) {
            case DEST_WC1:
                return moveRail1CarriageToWC1(hasLabware);
            case DEST_WC2:
                return moveRail1CarriageToWC2(hasLabware);
            case DEST_STAGING:
                Console.error(F("Invalid destination: Staging is not a delivery target"));
                return false;
            default:
                Console.error(F("Invalid destination for Rail 2 → Rail 1 handoff"));
                return false;
        }
    }
}

bool verifyHandoffLabwareTransfer() {
    // Use sensor timeout for this verification
    if (timeoutElapsed(millis(), handoffState.operationStartTime, HANDOFF_SENSOR_TIMEOUT)) {
        Console.error(F("LABWARE_SENSOR_TIMEOUT"));
        
        // SAFETY: Automatically retract cylinder on sensor timeout to prevent collision
        if (isCylinderActuallyExtended()) {
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, FMT_HANDOFF_ERROR, "Auto-retracting cylinder after timeout");
            Console.serialInfo(msg);
            ValveOperationResult retractResult = retractCylinder();
            if (retractResult != VALVE_OP_SUCCESS) {
                Console.error(F("CRITICAL_PNEUMATIC_FAILURE"));
            }
        }
        
        handoffState.currentState = HANDOFF_ERROR;
        handoffState.currentResult = HANDOFF_ERROR_SENSOR;
        return false;
    }
    
    // Enhanced verification: Check both source and destination labware states
    bool labwareAtHandoff = isLabwarePresentAtHandoff();
    bool sourceStillHasLabware = false;
    bool sourceExpectedEmpty = false;
    
    if (handoffState.direction == HANDOFF_RAIL1_TO_RAIL2) {
        // Rail 1 → Rail 2: Check if Rail 1 still has labware (should be gone after transfer)
        sourceStillHasLabware = isLabwarePresentAtWC1() || isLabwarePresentAtWC2();
        sourceExpectedEmpty = true;
        
        // Successful transfer: labware detected at handoff AND source is now empty
        if (labwareAtHandoff && !sourceStillHasLabware) {
            return true; // Transfer confirmed
        }
        
        // Failure scenarios with specific diagnostics
        if (!labwareAtHandoff && sourceStillHasLabware) {
            Console.error(F("LABWARE_TRANSFER_MECHANISM_FAILED"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
        
        if (!labwareAtHandoff && !sourceStillHasLabware) {
            Console.error(F("RAIL1_LABWARE_DISAPPEARED"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SOURCE_MISSING;
            return false;
        }
        
        if (labwareAtHandoff && sourceStillHasLabware) {
            Console.error(F("SENSOR_MALFUNCTION_DETECTED"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
        
    } else {
        // Rail 2 → Rail 1: Check if Rail 2 still has labware (should be gone after transfer)
        sourceStillHasLabware = isLabwarePresentAtWC3();
        sourceExpectedEmpty = true;
        
        // Successful transfer: labware detected at handoff AND source is now empty
        if (labwareAtHandoff && !sourceStillHasLabware) {
            return true; // Transfer confirmed
        }
        
        // Failure scenarios with specific diagnostics
        if (!labwareAtHandoff && sourceStillHasLabware) {
            Console.error(F("LABWARE_TRANSFER_MECHANISM_FAILED"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
        
        if (!labwareAtHandoff && !sourceStillHasLabware) {
            Console.error(F("RAIL2_LABWARE_DISAPPEARED"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SOURCE_MISSING;
            return false;
        }
        
        if (labwareAtHandoff && sourceStillHasLabware) {
            Console.error(F("SENSOR_MALFUNCTION_DETECTED"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
    }
    
    // Should never reach here, but safety fallback
    Console.error(F("HANDOFF_VERIFICATION_ERROR"));
    handoffState.currentState = HANDOFF_ERROR;
    handoffState.currentResult = HANDOFF_ERROR_SENSOR;
    return false;
}

bool isHandoffOperationTimedOut() {
    return timeoutElapsed(millis(), handoffState.operationStartTime, handoffState.currentTimeout);
}

const char* getHandoffResultName(HandoffResult result) {
    switch (result) {
        case HANDOFF_SUCCESS:              return "OPERATION_SUCCESSFUL";
        case HANDOFF_ERROR_ESTOP:          return "EMERGENCY_STOP_ACTIVATED";
        case HANDOFF_ERROR_TIMEOUT:        return "OPERATION_TIMEOUT";
        case HANDOFF_ERROR_SENSOR:         return "SENSOR_MALFUNCTION";
        case HANDOFF_ERROR_MOVEMENT:       return "RAIL_MOVEMENT_FAILED";
        case HANDOFF_ERROR_VALVE:          return "PNEUMATIC_SYSTEM_ERROR";
        case HANDOFF_ERROR_INVALID_PARAMS: return "INVALID_PARAMETERS";
        case HANDOFF_ERROR_SYSTEM_STATE:   return "SYSTEM_NOT_READY";
        case HANDOFF_ERROR_COLLISION:      return "COLLISION_RISK_DETECTED";
        case HANDOFF_ERROR_SOURCE_MISSING: return "SOURCE_LABWARE_MISSING";
        case HANDOFF_ERROR_POSITION:       return "POSITION_VALIDATION_FAILED";
        default:                           return "UNKNOWN_ERROR";
    }
}

const char* getHandoffStateName(HandoffState state) {
    switch (state) {
        case HANDOFF_IDLE:                    return "SYSTEM_IDLE";
        case HANDOFF_MOVING_SOURCE_TO_POS:    return "POSITIONING_SOURCE_RAIL";
        case HANDOFF_EXTENDING_CYLINDER:      return "EXTENDING_TRANSFER_CYLINDER";
        case HANDOFF_WAITING_TRANSFER:        return "WAITING_FOR_LABWARE_TRANSFER";
        case HANDOFF_RETRACTING_CYLINDER:     return "RETRACTING_TRANSFER_CYLINDER";
        case HANDOFF_MOVING_DEST_TO_TARGET:   return "MOVING_TO_DESTINATION";
        case HANDOFF_COMPLETED:               return "OPERATION_COMPLETED";
        case HANDOFF_ERROR:                   return "ERROR_STATE";
        default:                              return "UNKNOWN_STATE";
    }
}
