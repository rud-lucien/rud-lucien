#ifndef UTILS_H
#define UTILS_H

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

// Forward declarations of types from other headers
// to avoid circular dependencies
struct LoggingManagement;
class CommandCaller;

// Position tracking
extern double commandedPositionMm; // Last commanded position, -1 means no command yet

// Operation state tracking
extern bool operationInProgress;         // Flag indicating if an operation is running
extern bool newCommandReceived;          // Set when a new command comes in
extern unsigned long operationStartTime; // When the current operation started
extern unsigned long operationTimeoutMs; // Default timeout (10 seconds)
extern int currentOperationStep;         // Current step in operation sequence
extern int expectedOperationStep;        // Expected step at this point
extern bool operationEncoderState;       // True if encoder control is active during operation
extern bool homingEncoderState;          // Stores the encoder control state before homing begins

// State machine timing variables
extern unsigned long valveActuationStartTime;
extern const unsigned long VALVE_ACTUATION_TIME_MS;
extern const unsigned long SAFETY_DELAY_AFTER_UNLOCK_MS;
extern const unsigned long SAFETY_DELAY_BEFORE_MOVEMENT_MS;
extern const unsigned long SAFETY_DELAY_AFTER_MOVEMENT_MS;
extern const unsigned long SENSOR_VERIFICATION_DELAY_MS;

// Operation type enum
enum OperationType
{
    OPERATION_NONE = 0,
    OPERATION_LOADING,
    OPERATION_UNLOADING,
    OPERATION_MOVING,
    OPERATION_TRAY_ADVANCE
};

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

// Global variable declaration
extern TrayTracking trayTracking;

// Tray status structure
struct TrayStatus
{
    bool position1Occupied;
    bool position2Occupied;
    bool position3Occupied;
    unsigned long lastOperationTime;
    OperationType lastSuccessfulOperation;
};

extern TrayStatus trayStatus;

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

extern OperationStatus currentOperation;

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

// Safety validation results - per operation type
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
};

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// System state tracking functions
SystemState captureSystemState();
void printSystemState(const SystemState &state);

// Safety validation functions
SafetyValidationResult validateSafety(const SystemState &state);
void printSafetyStatus(const SafetyValidationResult &result);

// Motor position helper functions
bool isAtPosition(double currentPosition, double targetPosition);

// Additional movement helper
bool isMovingToPosition(double targetPosition, double currentTargetPositionMm);

// Add to Utils.h in the FUNCTION DECLARATIONS section
void updateTrayTrackingFromSensors(const SystemState &state);

// Process tray operations
void processTrayOperations();

// Helper functions for specific operation types
void processTrayLoading();
void processTrayUnloading();
void processTrayAdvance();

// Operation management
void beginOperation();
void endOperation();
void updateOperationStep(int newStep); // New function for synchronized step tracking

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

void abortOperation(AbortReason reason);
const char *getAbortReasonString(AbortReason reason);
void resetSystemState(); // Function to reset the system state after a failure

// Global variable declaration
extern SystemState previousState;

#endif // UTILS_H