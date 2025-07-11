#include "HandoffController.h"
#include "Logging.h"
#include "Sensors.h"
#include "ValveController.h"

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
        Console.error(F("HANDOFF_INVALID_PARAMS"));
        return handoffState.currentResult;
    }
    
    // Check if handoff already in progress
    if (handoffState.currentState != HANDOFF_IDLE) {
        Console.error(F("HANDOFF_IN_PROGRESS"));
        Console.serialInfo(F("Handoff already in progress - use cancelHandoff() first"));
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
    
    // Initialize handoff operation
    handoffState.direction = dir;
    handoffState.destination = dest;
    handoffState.currentState = HANDOFF_MOVING_SOURCE_TO_POS;
    handoffState.operationStartTime = millis();
    
    // Set timeout based on destination
    handoffState.currentTimeout = (dest == DEST_WC3) ? HANDOFF_TIMEOUT_WC3_EXT : HANDOFF_TIMEOUT_DEFAULT;
    
    Console.serialInfo(F("Starting handoff operation..."));
    
    // Create direction message
    char dirMsg[50];
    sprintf_P(dirMsg, PSTR("Direction: %s"), 
        (dir == HANDOFF_RAIL1_TO_RAIL2) ? "Rail1→Rail2" : "Rail2→Rail1");
    Console.serialInfo(dirMsg);
    
    // Create destination message  
    char destMsg[50];
    sprintf_P(destMsg, PSTR("Destination: %s"), 
        (dest == DEST_WC1) ? "WC1" : 
        (dest == DEST_WC2) ? "WC2" : 
        (dest == DEST_STAGING) ? "Staging" : "WC3");
    Console.serialInfo(destMsg);
    
    return HANDOFF_SUCCESS;
}

HandoffResult updateHandoff() {
    // Check for timeout on any operation
    if (isHandoffOperationTimedOut()) {
        Console.error(F("HANDOFF_TIMEOUT"));
        
        char timeoutMsg[80];
        sprintf_P(timeoutMsg, PSTR("Handoff timed out in state: %s"), getHandoffStateName(handoffState.currentState));
        Console.serialInfo(timeoutMsg);
        
        handoffState.currentState = HANDOFF_ERROR;
        handoffState.currentResult = HANDOFF_ERROR_TIMEOUT;
        return handoffState.currentResult;
    }
    
    // Check for E-stop at any time
    if (isEStopActive()) {
        Console.error(F("HANDOFF_ESTOP"));
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
                Console.serialInfo(F("Source rail positioned - extending cylinder..."));
                handoffState.currentState = HANDOFF_EXTENDING_CYLINDER;
                handoffState.operationStartTime = millis(); // Reset timer for next phase
            }
            break;
            
        case HANDOFF_EXTENDING_CYLINDER:
            {
                ValveOperationResult result = extendCylinder();
                if (result == VALVE_OP_SUCCESS && isCylinderActuallyExtended()) {
                    Console.serialInfo(F("Cylinder extended - waiting for labware transfer..."));
                    handoffState.currentState = HANDOFF_WAITING_TRANSFER;
                    handoffState.operationStartTime = millis(); // Reset timer for transfer phase
                } else if (result != VALVE_OP_SUCCESS) {
                    Console.error(F("HANDOFF_VALVE_ERROR"));
                    
                    char errorMsg[80];
                    sprintf_P(errorMsg, PSTR("Failed to extend cylinder: %s"), 
                        getValveOperationResultName(result));
                    Console.serialInfo(errorMsg);
                    
                    handoffState.currentState = HANDOFF_ERROR;
                    handoffState.currentResult = HANDOFF_ERROR_VALVE;
                }
            }
            break;
            
        case HANDOFF_WAITING_TRANSFER:
            if (verifyHandoffLabwareTransfer()) {
                Console.serialInfo(F("Labware transfer confirmed - retracting cylinder..."));
                handoffState.currentState = HANDOFF_RETRACTING_CYLINDER;
                handoffState.operationStartTime = millis(); // Reset timer for retraction
            }
            break;
            
        case HANDOFF_RETRACTING_CYLINDER:
            {
                ValveOperationResult result = retractCylinder();
                if (result == VALVE_OP_SUCCESS && isCylinderActuallyRetracted()) {
                    Console.serialInfo(F("Cylinder retracted - moving destination rail..."));
                    handoffState.currentState = HANDOFF_MOVING_DEST_TO_TARGET;
                    handoffState.operationStartTime = millis(); // Reset timer for final movement
                } else if (result != VALVE_OP_SUCCESS) {
                    Console.error(F("HANDOFF_VALVE_ERROR"));
                    
                    char errorMsg[80];
                    sprintf_P(errorMsg, PSTR("Failed to retract cylinder: %s"), 
                        getValveOperationResultName(result));
                    Console.serialInfo(errorMsg);
                    
                    handoffState.currentState = HANDOFF_ERROR;
                    handoffState.currentResult = HANDOFF_ERROR_VALVE;
                }
            }
            break;
            
        case HANDOFF_MOVING_DEST_TO_TARGET:
            if (moveDestinationRailToTargetPosition()) {
                Console.acknowledge(F("HANDOFF_COMPLETE"));
                Console.serialInfo(F("Handoff operation completed successfully"));
                handoffState.currentState = HANDOFF_COMPLETED;
                handoffState.currentResult = HANDOFF_SUCCESS;
            }
            break;
            
        case HANDOFF_COMPLETED:
            // Operation complete - return to idle on next call
            handoffState.currentState = HANDOFF_IDLE;
            return HANDOFF_SUCCESS;
            
        case HANDOFF_ERROR:
            // Error state - manual intervention required
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

void cancelHandoff() {
    if (handoffState.currentState != HANDOFF_IDLE) {
        Console.serialInfo(F("Cancelling handoff operation..."));
        handoffState.currentState = HANDOFF_IDLE;
        handoffState.currentResult = HANDOFF_SUCCESS;
        
        // Attempt to safely retract cylinder if extended
        if (isCylinderActuallyExtended()) {
            Console.serialInfo(F("Retracting cylinder for safety..."));
            retractCylinder();
        }
    }
}

void resetHandoff() {
    handoffState.currentState = HANDOFF_IDLE;
    handoffState.currentResult = HANDOFF_SUCCESS;
    handoffState.operationStartTime = 0;
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
        Console.error(F("ESTOP_ACTIVE"));
        Console.serialInfo(F("Cannot start handoff while emergency stop is active"));
        return false;
    }
    
    // Check both rails ready
    if (!checkRailMovementReadiness(1)) {
        Console.error(F("RAIL1_NOT_READY"));
        return false;
    }
    
    if (!checkRailMovementReadiness(2)) {
        Console.error(F("RAIL2_NOT_READY"));
        return false;
    }
    
    // Check air pressure
    if (!isPressureSufficient()) {
        Console.error(F("INSUFFICIENT_PRESSURE"));
        Console.serialInfo(F("Insufficient air pressure for handoff operation"));
        return false;
    }
    
    return true;
}

bool checkHandoffCollisionSafety(HandoffDirection dir, HandoffDestination dest) {
    // Check for the dangerous scenario where both carriages have labware
    // This would result in a collision during handoff operation
    
    bool rail1HasLabware = isLabwarePresentAtWC1() || isLabwarePresentAtWC2();
    bool rail2HasLabware = isLabwarePresentAtWC3() || isLabwarePresentAtHandoff();
    
    if (rail1HasLabware && rail2HasLabware) {
        Console.error(F("COLLISION_RISK"));
        Console.serialInfo(F("CRITICAL: Both Rail 1 and Rail 2 have labware detected"));
        Console.serialInfo(F("Cannot perform handoff - collision risk between carriages"));
        
        // Provide specific guidance based on direction
        if (dir == HANDOFF_RAIL1_TO_RAIL2) {
            Console.serialInfo(F("To proceed: Remove labware from WC3/handoff position first"));
        } else {
            Console.serialInfo(F("To proceed: Remove labware from WC1/WC2 positions first"));
        }
        
        return false;
    }
    
    // Additional check: Verify source carriage actually has labware to transfer
    if (dir == HANDOFF_RAIL1_TO_RAIL2 && !rail1HasLabware) {
        Console.error(F("NO_SOURCE_LABWARE"));
        Console.serialInfo(F("Rail 1 has no labware to transfer to Rail 2"));
        return false;
    }
    
    if (dir == HANDOFF_RAIL2_TO_RAIL1 && !rail2HasLabware) {
        Console.error(F("NO_SOURCE_LABWARE"));
        Console.serialInfo(F("Rail 2 has no labware to transfer to Rail 1"));
        return false;
    }
    
    // Destination-specific collision checks
    if (dir == HANDOFF_RAIL2_TO_RAIL1) {
        bool destinationOccupied = false;
        const char* destinationName = "";
        
        switch (dest) {
            case DEST_WC1:
                destinationOccupied = isLabwarePresentAtWC1();
                destinationName = "WC1";
                break;
            case DEST_WC2:
                destinationOccupied = isLabwarePresentAtWC2();
                destinationName = "WC2";
                break;
            case DEST_STAGING:
                // Staging position doesn't have a dedicated sensor
                // Assume it's available for now
                destinationOccupied = false;
                destinationName = "Staging";
                break;
            default:
                Console.error(F("Invalid destination for Rail 2 → Rail 1 handoff"));
                return false;
        }
        
        if (destinationOccupied) {
            Console.error(F("DESTINATION_OCCUPIED"));
            char errorMsg[80];
            sprintf_P(errorMsg, PSTR("Cannot deliver to %s - position already occupied"), destinationName);
            Console.serialInfo(errorMsg);
            return false;
        }
    } else if (dir == HANDOFF_RAIL1_TO_RAIL2) {
        // Rail1 → Rail2 always goes to WC3
        if (isLabwarePresentAtWC3()) {
            Console.error(F("DESTINATION_OCCUPIED"));
            Console.serialInfo(F("Cannot deliver to WC3 - position already occupied"));
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
                return moveRail1CarriageToStaging(hasLabware);
            default:
                Console.error(F("Invalid destination for Rail 2 → Rail 1 handoff"));
                return false;
        }
    }
}

bool verifyHandoffLabwareTransfer() {
    // Use sensor timeout for this verification
    if (timeoutElapsed(millis(), handoffState.operationStartTime, HANDOFF_SENSOR_TIMEOUT)) {
        Console.error(F("SENSOR_TIMEOUT"));
        Console.serialInfo(F("Labware transfer verification timed out"));
        
        // SAFETY: Automatically retract cylinder on sensor timeout to prevent collision
        if (isCylinderActuallyExtended()) {
            Console.serialInfo(F("SAFETY: Retracting cylinder due to sensor timeout..."));
            ValveOperationResult retractResult = retractCylinder();
            if (retractResult != VALVE_OP_SUCCESS) {
                Console.error(F("CRITICAL_SAFETY_ERROR"));
                Console.serialInfo(F("Failed to automatically retract cylinder after timeout - manual intervention required"));
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
            Console.error(F("TRANSFER_FAILED"));
            Console.serialInfo(F("Labware still detected at WC1/WC2 but not at handoff position"));
            Console.serialInfo(F("Transfer mechanism may have failed - check cylinder operation"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
        
        if (!labwareAtHandoff && !sourceStillHasLabware) {
            Console.error(F("SOURCE_LABWARE_MISSING"));
            Console.serialInfo(F("CRITICAL: Labware disappeared from Rail 1 during handoff"));
            Console.serialInfo(F("Labware was expected at WC1/WC2 but is no longer detected"));
            Console.serialInfo(F("Check for mechanical issues or sensor malfunctions"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SOURCE_MISSING;
            return false;
        }
        
        if (labwareAtHandoff && sourceStillHasLabware) {
            Console.error(F("DUPLICATE_LABWARE"));
            Console.serialInfo(F("WARNING: Labware detected at both source (WC1/WC2) and handoff"));
            Console.serialInfo(F("This suggests sensor malfunction or unexpected labware duplication"));
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
            Console.error(F("TRANSFER_FAILED"));
            Console.serialInfo(F("Labware still detected at WC3 but not at handoff position"));
            Console.serialInfo(F("Transfer mechanism may have failed - check cylinder operation"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
        
        if (!labwareAtHandoff && !sourceStillHasLabware) {
            Console.error(F("SOURCE_LABWARE_MISSING"));
            Console.serialInfo(F("CRITICAL: Labware disappeared from Rail 2 during handoff"));
            Console.serialInfo(F("Labware was expected at WC3 but is no longer detected"));
            Console.serialInfo(F("Check for mechanical issues or sensor malfunctions"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SOURCE_MISSING;
            return false;
        }
        
        if (labwareAtHandoff && sourceStillHasLabware) {
            Console.error(F("DUPLICATE_LABWARE"));
            Console.serialInfo(F("WARNING: Labware detected at both source (WC3) and handoff"));
            Console.serialInfo(F("This suggests sensor malfunction or unexpected labware duplication"));
            handoffState.currentState = HANDOFF_ERROR;
            handoffState.currentResult = HANDOFF_ERROR_SENSOR;
            return false;
        }
    }
    
    // Should never reach here, but safety fallback
    Console.error(F("TRANSFER_VERIFICATION_UNKNOWN"));
    Console.serialInfo(F("Unexpected state during transfer verification"));
    handoffState.currentState = HANDOFF_ERROR;
    handoffState.currentResult = HANDOFF_ERROR_SENSOR;
    return false;
}

bool isHandoffOperationTimedOut() {
    return timeoutElapsed(millis(), handoffState.operationStartTime, handoffState.currentTimeout);
}

const char* getHandoffResultName(HandoffResult result) {
    switch (result) {
        case HANDOFF_SUCCESS:              return "SUCCESS";
        case HANDOFF_ERROR_ESTOP:          return "E_STOP";
        case HANDOFF_ERROR_TIMEOUT:        return "TIMEOUT";
        case HANDOFF_ERROR_SENSOR:         return "SENSOR_ERROR";
        case HANDOFF_ERROR_MOVEMENT:       return "MOVEMENT_ERROR";
        case HANDOFF_ERROR_VALVE:          return "VALVE_ERROR";
        case HANDOFF_ERROR_INVALID_PARAMS: return "INVALID_PARAMS";
        case HANDOFF_ERROR_SYSTEM_STATE:   return "SYSTEM_NOT_READY";
        case HANDOFF_ERROR_COLLISION:      return "COLLISION_RISK";
        case HANDOFF_ERROR_SOURCE_MISSING: return "SOURCE_MISSING";
        default:                           return "UNKNOWN";
    }
}

const char* getHandoffStateName(HandoffState state) {
    switch (state) {
        case HANDOFF_IDLE:                    return "IDLE";
        case HANDOFF_MOVING_SOURCE_TO_POS:    return "MOVING_SOURCE";
        case HANDOFF_EXTENDING_CYLINDER:      return "EXTENDING_CYLINDER";
        case HANDOFF_WAITING_TRANSFER:        return "WAITING_TRANSFER";
        case HANDOFF_RETRACTING_CYLINDER:     return "RETRACTING_CYLINDER";
        case HANDOFF_MOVING_DEST_TO_TARGET:   return "MOVING_DESTINATION";
        case HANDOFF_COMPLETED:               return "COMPLETED";
        case HANDOFF_ERROR:                   return "ERROR";
        default:                              return "UNKNOWN";
    }
}
