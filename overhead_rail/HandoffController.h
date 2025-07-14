#ifndef HANDOFF_CONTROLLER_H
#define HANDOFF_CONTROLLER_H

#include "ClearCore.h"
#include "RailAutomation.h"
#include "Utils.h"

//=============================================================================
// HANDOFF SYSTEM CONSTANTS
//=============================================================================

// Handoff timeout configurations (milliseconds) - operation-specific
#define HANDOFF_TIMEOUT_RAIL_MOVEMENT          5000     // 5 seconds for rail positioning (max 900mm at 250 RPM = ~3s)
#define HANDOFF_TIMEOUT_PNEUMATIC_EXTEND       5000     // 5 seconds for cylinder extension (valve default 2s + margin)
#define HANDOFF_TIMEOUT_PNEUMATIC_RETRACT      3000     // 3 seconds for cylinder retraction (faster than extension)
#define HANDOFF_TIMEOUT_SENSOR_VERIFY          3000     // 3 seconds for sensor confirmation (quick sensor response)
#define HANDOFF_TIMEOUT_COMPLETE_OPERATION     25000    // 25 seconds maximum for entire handoff sequence

// Handoff safety pause durations (milliseconds)
#define HANDOFF_PAUSE_AFTER_MOVE    1000     // 1 second pause after rail movement
#define HANDOFF_PAUSE_AFTER_EXTEND  500      // 0.5 second pause after cylinder extension
#define HANDOFF_PAUSE_AFTER_RETRACT 500      // 0.5 second pause after cylinder retraction

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
    HANDOFF_ERROR_SYSTEM_STATE,   // System not ready for handoff
    HANDOFF_ERROR_COLLISION,      // Both carriages have labware - collision risk
    HANDOFF_ERROR_SOURCE_MISSING, // Source labware disappeared during operation
    HANDOFF_ERROR_POSITION        // Position validation failure
};

// Internal handoff state machine phases (for non-blocking operation)
enum HandoffPhase {
    HANDOFF_IDLE,                    // No handoff in progress
    HANDOFF_MOVING_SOURCE_TO_POS,    // Moving source rail to handoff position
    HANDOFF_PAUSE_AFTER_MOVEMENT,    // Safety pause after rail movement
    HANDOFF_EXTENDING_CYLINDER,      // Extending cylinder for transfer
    HANDOFF_PAUSE_AFTER_EXTENSION,   // Safety pause after cylinder extension
    HANDOFF_WAITING_TRANSFER,        // Waiting for labware transfer confirmation
    HANDOFF_RETRACTING_CYLINDER,     // Retracting cylinder after transfer
    HANDOFF_PAUSE_AFTER_RETRACTION,  // Safety pause after cylinder retraction
    HANDOFF_MOVING_DEST_TO_TARGET,   // Moving destination rail to target position
    HANDOFF_COMPLETED,               // Handoff completed successfully
    HANDOFF_ERROR                    // Error state
};

//=============================================================================
// HANDOFF STATE STRUCTURE
//=============================================================================

struct HandoffState {
    HandoffPhase currentState;
    HandoffResult currentResult;
    HandoffDirection direction;
    HandoffDestination destination;
    unsigned long operationStartTime;
    unsigned long currentTimeout;
};

//=============================================================================
// GLOBAL HANDOFF STATE
//=============================================================================

extern HandoffState handoffState;

//=============================================================================
// FUNCTIONAL HANDOFF API
//=============================================================================

// Main handoff operations
HandoffResult startHandoff(HandoffDirection direction, HandoffDestination destination);
HandoffResult updateHandoff(); // Call periodically to advance state machine

// Status query functions
bool isHandoffInProgress();
HandoffPhase getCurrentHandoffState();
HandoffResult getLastHandoffResult();

// Helper functions (internal - defined in .cpp)
bool validateHandoffParameters(HandoffDirection dir, HandoffDestination dest);
bool checkHandoffSystemReadiness();
bool checkHandoffCollisionSafety(HandoffDirection dir, HandoffDestination dest);
bool moveSourceRailToHandoffPosition();
bool moveDestinationRailToTargetPosition();
bool verifyHandoffLabwareTransfer();
bool isHandoffOperationTimedOut();
bool isCurrentPhaseTimedOut();
unsigned long getCurrentPhaseTimeout(HandoffPhase phase, HandoffDestination dest);
const char* getHandoffResultName(HandoffResult result);
const char* getHandoffStateName(HandoffPhase state);

// Enhanced position validation using existing MotorController functions
bool validateRailReadyForHandoff(int railNumber, double expectedPosition);
bool validateRail1AtHandoffPosition();
bool validateRail2AtHandoffPosition();

#endif // HANDOFF_CONTROLLER_H
