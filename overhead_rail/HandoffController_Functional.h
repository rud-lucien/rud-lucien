#ifndef HANDOFF_CONTROLLER_H
#define HANDOFF_CONTROLLER_H

#include "ClearCore.h"
#include "RailAutomation.h"
#include "Utils.h"

//=============================================================================
// HANDOFF SYSTEM CONSTANTS
//=============================================================================

// Handoff timeout configurations (milliseconds)
#define HANDOFF_TIMEOUT_DEFAULT     30000    // 30 seconds default timeout
#define HANDOFF_TIMEOUT_WC3_EXT     45000    // 45 seconds for WC3 with extension
#define HANDOFF_SENSOR_TIMEOUT      10000    // 10 seconds for sensor confirmation
#define HANDOFF_MOVEMENT_TIMEOUT    20000    // 20 seconds for rail movements

//=============================================================================
// HANDOFF ENUMS
//=============================================================================

// Handoff direction
enum HandoffDirection {
    HANDOFF_RAIL1_TO_RAIL2,  // Rail 1 → Rail 2
    HANDOFF_RAIL2_TO_RAIL1   // Rail 2 → Rail 1
};

// Handoff destination (simplified)
enum HandoffDestination {
    DEST_WC1,        // Rail 1 WC1 position
    DEST_WC2,        // Rail 1 WC2 position  
    DEST_STAGING,    // Rail 1 staging position
    DEST_WC3         // Rail 2 WC3 position (includes automatic extension)
};

// Handoff operation result
enum HandoffResult {
    HANDOFF_SUCCESS,              // Operation completed successfully
    HANDOFF_ERROR_ESTOP,          // E-stop active during operation
    HANDOFF_ERROR_TIMEOUT,        // Operation timed out
    HANDOFF_ERROR_SENSOR,         // Sensor verification failed
    HANDOFF_ERROR_MOVEMENT,       // Rail movement failed
    HANDOFF_ERROR_VALVE,          // Pneumatic operation failed
    HANDOFF_ERROR_INVALID_PARAMS, // Invalid parameters provided
    HANDOFF_ERROR_SYSTEM_STATE    // System not ready for handoff
};

// Internal handoff state (for non-blocking operation)
enum HandoffState {
    HANDOFF_IDLE,                    // No handoff in progress
    HANDOFF_MOVING_SOURCE_TO_POS,    // Moving source rail to handoff position
    HANDOFF_EXTENDING_CYLINDER,      // Extending cylinder for transfer
    HANDOFF_WAITING_TRANSFER,        // Waiting for labware transfer confirmation
    HANDOFF_RETRACTING_CYLINDER,     // Retracting cylinder after transfer
    HANDOFF_MOVING_DEST_TO_TARGET,   // Moving destination rail to target position
    HANDOFF_COMPLETED,               // Handoff completed successfully
    HANDOFF_ERROR                    // Error state
};

//=============================================================================
// HANDOFF STATE STRUCTURE (for functional approach)
//=============================================================================

struct HandoffState_t {
    HandoffState currentState;
    HandoffResult currentResult;
    HandoffDirection direction;
    HandoffDestination destination;
    unsigned long operationStartTime;
    unsigned long currentTimeout;
};

//=============================================================================
// GLOBAL HANDOFF STATE
//=============================================================================

extern HandoffState_t handoffState;

//=============================================================================
// FUNCTIONAL HANDOFF API
//=============================================================================

// Main handoff operations
HandoffResult startHandoff(HandoffDirection direction, HandoffDestination destination);
HandoffResult updateHandoff(); // Call periodically to advance state machine

// Status query functions
bool isHandoffInProgress();
HandoffState getCurrentHandoffState();
HandoffResult getLastHandoffResult();

// Utility functions
void cancelHandoff(); // Emergency cancel
void resetHandoff();  // Reset to idle state

// Helper functions (could be static/private in .cpp)
bool validateHandoffParameters(HandoffDirection dir, HandoffDestination dest);
bool checkHandoffSystemReadiness();
bool moveSourceRailToHandoffPosition();
bool moveDestinationRailToTargetPosition();
bool verifyHandoffLabwareTransfer();
bool isHandoffOperationTimedOut();
const char* getHandoffResultName(HandoffResult result);
const char* getHandoffStateName(HandoffState state);

#endif // HANDOFF_CONTROLLER_H
