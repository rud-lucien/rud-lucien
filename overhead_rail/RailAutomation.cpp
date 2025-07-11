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
        Console.acknowledge(carriageLoaded ? F("OK_MOVE_WC1_WITH") : F("OK_MOVE_WC1_NO"));
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
        Console.acknowledge(carriageLoaded ? F("OK_MOVE_WC2_WITH") : F("OK_MOVE_WC2_NO"));
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
        Console.acknowledge(carriageLoaded ? F("OK_MOVE_STAGING_WITH") : F("OK_MOVE_STAGING_NO"));
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
        Console.acknowledge(carriageLoaded ? F("OK_MOVE_HANDOFF_WITH") : F("OK_MOVE_HANDOFF_NO"));
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
        Console.acknowledge(carriageLoaded ? F("OK_MOVE_WC3_WITH") : F("OK_MOVE_WC3_NO"));
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
        Console.acknowledge(carriageLoaded ? F("OK_MOVE_HANDOFF_WITH") : F("OK_MOVE_HANDOFF_NO"));
        Console.serialInfo(F("Carriage successfully moved to handoff position"));
        return true;
    } else {
        Console.error(F("MOVEMENT_FAILED"));
        Console.serialInfo(F("Failed to move carriage to handoff - check motor status"));
        return false;
    }
}
