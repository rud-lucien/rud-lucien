#ifndef UTILS_H
#define UTILS_H

//=============================================================================
// INCLUDES
//=============================================================================
#include "Arduino.h"
#include "ClearCore.h"
#include "ValveController.h"
#include "MotorController.h"
#include "Logging.h"
#include "Tests.h"
#include "Commands.h"
#include "EncoderController.h"
#include "OutputManager.h"
#include "EthernetController.h"

//=============================================================================
// FORWARD DECLARATIONS
//=============================================================================
struct LoggingManagement;
class CommandCaller;

//=============================================================================
// ENUMERATIONS
//=============================================================================
// Operation type enum
enum OperationType
{
    OPERATION_NONE = 0,
    OPERATION_LOADING,
    OPERATION_UNLOADING,
    OPERATION_MOVING,
    OPERATION_TRAY_ADVANCE
};

// Abort reason enum
enum AbortReason
{
    ABORT_REASON_ESTOP,
    ABORT_REASON_MOTOR_TIMEOUT,
    ABORT_REASON_OPERATION_TIMEOUT,
    ABORT_REASON_SENSOR_MISMATCH,
    ABORT_REASON_COMMUNICATION_LOSS,
    ABORT_REASON_PNEUMATIC_FAILURE,
    ABORT_REASON_UNKNOWN
};

//=============================================================================
// STRUCTURE DEFINITIONS
//=============================================================================
// System state structure - captures all sensor and actuator states
struct SystemState
{
    // Motor state
    MotorState motorState;
    bool isHomed;
    double currentPositionMm;
    MotorDriver::HlfbStates hlfbStatus;

    // Cylinder sensor states (raw readings)
    bool tray1CylinderActivated;
    bool tray2CylinderActivated;
    bool tray3CylinderActivated;
    bool shuttleCylinderActivated;

    // Derived lock states (based on cylinder sensors)
    bool tray1Locked;
    bool tray2Locked;
    bool tray3Locked;
    bool shuttleLocked;

    // Tray presence detection
    bool tray1Present;
    bool tray2Present;
    bool tray3Present;

    // Safety system
    bool eStopActive;

    // Hardware status
    bool ccioBoardPresent;

    // Tray tracking information
    uint8_t totalTraysInSystem;
    bool position1Occupied;
    bool position2Occupied;
    bool position3Occupied;
};

// Tray tracking structure
struct TrayTracking
{
    // Total count of trays in the system
    uint8_t totalTraysInSystem;

    // Position occupancy
    bool position1Occupied;
    bool position2Occupied;
    bool position3Occupied;

    // Last loading/unloading timestamps
    unsigned long lastLoadTime;
    unsigned long lastUnloadTime;

    // Count of load/unload operations since startup
    uint16_t totalLoadsCompleted;
    uint16_t totalUnloadsCompleted;
};

// Tray status structure
struct TrayStatus
{
    bool position1Occupied;
    bool position2Occupied;
    bool position3Occupied;
    unsigned long lastOperationTime;
    OperationType lastSuccessfulOperation;
};

// Operation status structure
struct OperationStatus
{
    bool inProgress;
    OperationType type;
    int trayNumber;
    unsigned long startTime;
    bool success;
    char message[32];
};

// Safety validation results structure
struct SafetyValidationResult
{
    // Motor movement safety flags
    bool safeToMove;
    String moveUnsafeReason;

    // Tray locking safety flags
    bool safeToLockTray1;
    bool safeToLockTray2;
    bool safeToLockTray3;
    String tray1LockUnsafeReason;
    String tray2LockUnsafeReason;
    String tray3LockUnsafeReason;

    // Shuttle actuation safety flags
    bool safeToLockShuttle;
    bool safeToUnlockShuttle;
    String shuttleLockUnsafeReason;
    String shuttleUnlockUnsafeReason;

    bool pneumaticPressureSufficient;
    String pressureUnsafeReason;

    // Tray loading safety flags
    bool safeToLoadTrayToPos1;
    bool safeToLoadTrayToPos2;
    bool safeToLoadTrayToPos3;
    String loadTrayPos1UnsafeReason;
    String loadTrayPos2UnsafeReason;
    String loadTrayPos3UnsafeReason;

    // Tray unloading safety flags
    bool safeToUnloadTrayFromPos1;
    bool safeToUnloadTrayFromPos2;
    bool safeToUnloadTrayFromPos3;
    String unloadTrayPos1UnsafeReason;
    String unloadTrayPos2UnsafeReason;
    String unloadTrayPos3UnsafeReason;
    bool safeToUnlockGrippedTray;
    String grippedTrayUnlockUnsafeReason;

    // System state validation
    bool commandStateValid;
    bool trayPositionValid;
    bool targetPositionValid;
    String stateValidationMessage;

    // Operational sequence validation
    bool safeToAcceptNewCommand;
    bool operationWithinTimeout;
    bool operationSequenceValid;
    String operationSequenceMessage;

    // Abort reason - directly indicates what type of abort should be triggered
    AbortReason failureReason;

    // Lock/unlock operation status
    bool lockOperationSuccessful;     // Set to false if a lock operation recently failed
    bool unlockOperationSuccessful;   // Set to false if an unlock operation recently failed
    String lockFailureDetails;        // Details about which lock operation failed
    String unlockFailureDetails;      // Details about which unlock operation failed
};

//=============================================================================
// CONSTANTS AND TIMING PARAMETERS
//=============================================================================
extern const unsigned long VALVE_ACTUATION_TIME_MS;
extern const unsigned long SAFETY_DELAY_AFTER_UNLOCK_MS;
extern const unsigned long SAFETY_DELAY_BEFORE_MOVEMENT_MS;
extern const unsigned long SAFETY_DELAY_AFTER_MOVEMENT_MS;
extern const unsigned long SENSOR_VERIFICATION_DELAY_MS;

//=============================================================================
// SYSTEM STATE TRACKING
//=============================================================================
// Position tracking
extern double commandedPositionMm; // Last commanded position, -1 means no command yet
extern SystemState previousState;  // NOW DEFINED AFTER SystemState

// Operation state tracking
extern bool operationInProgress;         // Flag indicating if an operation is running
extern bool newCommandReceived;          // Set when a new command comes in
extern unsigned long operationStartTime; // When the current operation started
extern unsigned long operationTimeoutMs; // Default timeout (10 seconds)
extern int currentOperationStep;         // Current step in operation sequence
extern int expectedOperationStep;        // Expected step at this point
extern bool operationEncoderState;       // True if encoder control is active during operation
extern bool homingEncoderState;          // Stores the encoder control state before homing begins

// Timing and state machine variables
extern unsigned long valveActuationStartTime;

// Lock/unlock operation status tracking
extern bool lastLockOperationFailed;
extern bool lastUnlockOperationFailed;
extern String lastLockFailureDetails;
extern String lastUnlockFailureDetails;
extern unsigned long lockFailureTimestamp;
extern unsigned long unlockFailureTimestamp;

// Global variable exports
extern TrayTracking trayTracking;
extern TrayStatus trayStatus;
extern OperationStatus currentOperation;

//=============================================================================
// TIME HANDLING FUNCTIONS
//=============================================================================
// Safe time difference calculation that handles rollover
unsigned long timeDiff(unsigned long current, unsigned long previous);

// Safe timeout check that handles rollover
bool timeoutElapsed(unsigned long current, unsigned long previous, unsigned long timeout);

// Safe waiting check
bool waitTimeReached(unsigned long current, unsigned long previous, unsigned long waitTime);

//=============================================================================
// SYSTEM STATE FUNCTIONS
//=============================================================================
// System state tracking functions
SystemState captureSystemState();
void printSystemState(const SystemState &state);
void initSystemStateVariables();
void resetSystemState(); // Function to reset the system state after a failure

//=============================================================================
// SAFETY FUNCTIONS
//=============================================================================
// Safety validation functions
SafetyValidationResult validateSafety(const SystemState &state);
void printSafetyStatus(const SafetyValidationResult &result);
void abortOperation(AbortReason reason);
const char *getAbortReasonString(AbortReason reason);

//=============================================================================
// MOTION AND POSITION FUNCTIONS
//=============================================================================
// Motor position helper functions
bool isAtPosition(double currentPosition, double targetPosition);
bool isMovingToPosition(double targetPosition, double currentTargetPositionMm);

//=============================================================================
// TRAY TRACKING AND OPERATION FUNCTIONS
//=============================================================================
void updateTrayTrackingFromSensors(const SystemState &state);
void resetTrayTracking();
void resetLockUnlockFailures();

// Process tray operations
void processTrayOperations();
void processTrayLoading();
void processTrayUnloading();
void processTrayAdvance();

// Operation management
void beginOperation();
void endOperation();
void updateOperationStep(int newStep);

// Tray tracking management functions
bool moveTray(int fromPosition, int toPosition);
bool loadFirstTray();
bool loadSecondTray();
bool loadThirdTray();
bool unloadFirstTray();
bool unloadSecondTray();
bool unloadThirdTray();
int determineLoadingWorkflow();
int determineUnloadingWorkflow();
bool isPathClearForLoading(double startPosition, double targetPosition, const SystemState &state);
bool isPathClearForUnloading(double startPosition, double targetPosition, const SystemState &state);

#endif // UTILS_H