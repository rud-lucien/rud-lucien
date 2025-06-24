#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"
#include "ClearCore.h"
#include "OutputManager.h"

//=============================================================================
// PIN AND HARDWARE CONFIGURATION
//=============================================================================

// Motor connector on ClearCore
#define MOTOR_CONNECTOR ConnectorM0

// E-stop input
#define E_STOP_PIN 0                // E-stop connected to DI6 (Normally Closed)
#define E_STOP_CHECK_INTERVAL_MS 10 // Check E-stop every 10ms

//=============================================================================
// MOTION CONTROL CONSTANTS
//=============================================================================

// Motor & Motion parameters
#define PULSES_PER_REV 3200                         // Motor configured for 3200 pulses per revolution
#define MM_PER_REV 52.23                            // Calibrated: 52.23mm travel per revolution
#define PULSES_PER_MM (PULSES_PER_REV / MM_PER_REV) // ~61.27 pulses per mm

// Motion profile parameters (in RPM units)
#define MOTOR_VELOCITY_RPM 300         // Maximum velocity for motor operation (RPM)
#define MAX_ACCEL_RPM_PER_SEC 2500     // Maximum acceleration in RPM/s
#define EMPTY_SHUTTLE_VELOCITY_RPM 800 // Higher speed for empty shuttle movements

// Motion direction control (1 for normal direction, -1 for reversed)
#define MOTION_DIRECTION -1 // Set to -1 to reverse all motion

//=============================================================================
// HOMING PARAMETERS
//=============================================================================

#define HOME_TIMEOUT_MS 60000          // Timeout for homing operation (1 minute)
#define HOMING_DIRECTION 1             // Direction to move toward hard stop (1 for positive, -1 for negative)
#define HOME_APPROACH_VELOCITY_RPM 50  // Initial approach velocity in RPM (faster)
#define HOME_APPROACH_TIME_MS 2000     // Time to move at the initial approach velocity
#define HOME_OFFSET_DISTANCE_MM 5.0    // Distance to move away from hard stop in mm
#define HOME_HLFB_CHECK_INTERVAL_MS 20 // Time to wait between HLFB state checks

//=============================================================================
// TRAVEL AND POSITION DEFINITIONS
//=============================================================================

// System travel limits - based on physical measurement
#define MAX_TRAVEL_MM 1050.0                                       // Maximum travel in mm (measured)
#define MAX_TRAVEL_PULSES (int32_t)(MAX_TRAVEL_MM * PULSES_PER_MM) // ~65,142 counts

// Position definitions (in mm from home)
#define POSITION_HOME_MM 0.0        // Home position
#define POSITION_1_MM 28.78         // Position 1 (28.78 mm)
#define POSITION_2_MM 455.80        // Position 2 (455.80 mm)
#define POSITION_3_MM 884.20        // Position 3 (884.20 mm)
#define POSITION_4_MM MAX_TRAVEL_MM // Position 4 (max = 1050 mm)

// Position definitions in pulses (calculated from mm)
#define POSITION_HOME_PULSES (PULSES_PER_MM * POSITION_HOME_MM) // Home position in pulses
#define POSITION_1_PULSES (PULSES_PER_MM * POSITION_1_MM)       // Position 1 in pulses
#define POSITION_2_PULSES (PULSES_PER_MM * POSITION_2_MM)       // Position 2 in pulses
#define POSITION_3_PULSES (PULSES_PER_MM * POSITION_3_MM)       // Position 3 in pulses
#define POSITION_4_PULSES (PULSES_PER_MM * POSITION_4_MM)       // Position 4 in pulses

#define POSITION_TOLERANCE_MM 1.0 // Standard tolerance for position matching (mm)

//=============================================================================
// JOG PARAMETERS
//=============================================================================

#define DEFAULT_JOG_INCREMENT 1.0 // Default jog increment (1mm)
#define DEFAULT_JOG_SPEED 30      // 30 RPM (slow)

//=============================================================================
// TYPE DEFINITIONS
//=============================================================================

// Position targets
typedef enum
{
    POSITION_UNDEFINED = -1,
    POSITION_HOME = 0,
    POSITION_1 = 1,
    POSITION_2 = 2,
    POSITION_3 = 3,
    POSITION_4 = 4,
    POSITION_CUSTOM = 99
} PositionTarget;

// Motor operational states
enum MotorState
{
    MOTOR_STATE_NOT_READY,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_HOMING,
    MOTOR_STATE_FAULTED
};

#include "Utils.h" // Include Utils.h for utility functions and definitions here to prevent circular dependencies

// State machine for non-blocking fault clearing
enum FaultClearingState
{
    FAULT_CLEAR_IDLE,            // No fault clearing in progress
    FAULT_CLEAR_DISABLE,         // Starting disable phase
    FAULT_CLEAR_WAITING_DISABLE, // Waiting for disable to take effect
    FAULT_CLEAR_ENABLE,          // Starting enable phase
    FAULT_CLEAR_WAITING_ENABLE,  // Waiting for enable to take effect
    FAULT_CLEAR_ALERTS,          // Clearing alerts
    FAULT_CLEAR_FINISHED         // Process complete
};

// In MotorController.h
struct DecelerationConfig
{
    float decelerationDistanceMm;
    float minVelocityRPM;
    bool enableDeceleration;

    // Just declare the method
    int32_t getMinVelocityPPS() const;
};

#define VELOCITY_CHANGE_THRESHOLD 0.5 // 3 RPM minimum change to update velocity

// Distance thresholds for velocity scaling
#define VERY_SHORT_MOVE_THRESHOLD_MM 10.0 // Moves less than 10mm are very short
#define SHORT_MOVE_THRESHOLD_MM 30.0      // Moves less than 30mm are short
#define MEDIUM_MOVE_THRESHOLD_MM 100.0    // Moves less than 100mm are medium

// Velocity scaling for different move distances
#define VERY_SHORT_MOVE_VELOCITY_RPM 50 // CHANGED: 50 RPM for very short moves (was 25)
#define SHORT_MOVE_VELOCITY_RPM 175     // CHANGED: 75 RPM for short moves (was 50)
#define MEDIUM_MOVE_VELOCITY_RPM 125    // CHANGED: 125 RPM for medium moves (was 100)

// Deceleration configuration defaults
#define DEFAULT_DECELERATION_DISTANCE_MM 35.0 // Start decelerating when 35mm from target
#define DEFAULT_MIN_VELOCITY_RPM 50           // CHANGED: Minimum velocity of 50 RPM during deceleration (was 25.0)
#define DEFAULT_DECELERATION_ENABLED true     // Deceleration enabled by default

// Deceleration stages configuration - NO CHANGES NEEDED
#define DECEL_TRANSITION_POINT_RATIO 0.5f // First stage to second stage transition (50%)
#define DECEL_FIRST_STAGE_END_RATIO 0.4f  // First stage decelerates to 40% of range
#define DECEL_S_CURVE_MULTIPLIER 2.78f    // S-curve multiplier for first stage

// Very short move detection - NO CHANGE NEEDED
#define VERY_SHORT_MOVE_RATIO 1.1f // Moves < 110% of decel distance considered "very short"

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Motor state tracking
extern bool isHomed;                   // Flag indicating if the motor has been homed
extern bool motorInitialized;          // Flag indicating if the motor is initialized
extern double currentPositionMm;       // Current position in mm
extern MotorState motorState;          // Current motor state
extern PositionTarget currentPosition; // Current target position

// Motion parameters
extern int32_t currentVelMax;   // Current velocity maximum in pulse/sec
extern int32_t currentAccelMax; // Current acceleration maximum in pulse/sec²

// Homing state variables
extern bool homingInProgress;              // Flag indicating if homing is in progress
extern bool homingEncoderState;            // Stores the encoder control state before homing begins. Used to restore encoder control after homing completes
extern bool motorEnableCycleInProgress;    // Flag indicating if enable cycling is in progress
extern unsigned long enableCycleStartTime; // Timestamp for enable cycle timing
extern bool motorDisablePhaseComplete;     // Flag indicating disable phase completion

// Target tracking for logging
extern bool hasCurrentTarget;            // Flag indicating if we have an active target
extern bool hasLastTarget;               // Flag indicating if we have a previous target
extern PositionTarget currentTargetType; // Current target position type
extern PositionTarget lastTargetType;    // Last target position type
extern double currentTargetPositionMm;   // Current target position in mm
extern double lastTargetPositionMm;      // Last target position in mm
extern int32_t currentTargetPulses;      // Current target position in pulses
extern int32_t lastTargetPulses;         // Last target position in pulses

// Jog settings
extern double currentJogIncrementMm; // Current jog increment in mm
extern int currentJogSpeedRpm;       // Current jog speed in RPM

// Fault clearing state
extern FaultClearingState faultClearState; // Current fault clearing state
extern unsigned long faultClearTimer;      // Timestamp for fault clearing timing
extern bool faultClearInProgress;          // Flag indicating if fault clearing is in progress

// Deceleration configuration
extern DecelerationConfig motorDecelConfig; // Deceleration settings

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// System Initialization
// Setup the motor and control parameters
//-----------------------------------------------------------------------------
void initMotorSystem();

//-----------------------------------------------------------------------------
// Unit Conversion
// Convert between different measurement units
//-----------------------------------------------------------------------------
int32_t rpmToPps(double rpm);
double ppsToRpm(int32_t pps);
int32_t rpmPerSecToPpsPerSec(double rpmPerSec);
int32_t mmToPulses(double mm);
double pulsesToMm(int32_t pulses);
int32_t normalizeEncoderValue(int32_t rawValue);

//-----------------------------------------------------------------------------
// Homing Operations
// Functions for establishing a reference position
//-----------------------------------------------------------------------------
bool initiateHomingSequence();
void checkHomingProgress();
void completeHomingSequence();
bool isHomingComplete();
void resetHomingState();
void abortHoming();

//-----------------------------------------------------------------------------
// Motion Control
// Functions for moving the motor to various positions
//-----------------------------------------------------------------------------
bool moveToPosition(PositionTarget position);
bool moveToPositionMm(double positionMm);
bool moveRelative(double distanceMm);
bool moveToAbsolutePosition(int32_t position);
bool moveToPosition(int positionNumber);
void stopMotion();
void checkMoveProgress();

//-----------------------------------------------------------------------------
// Jog Control
// Incremental movement control for manual operation
//-----------------------------------------------------------------------------
bool jogMotor(bool direction, double customIncrement = -1.0);
bool setJogIncrement(double increment);
bool setJogSpeed(int speedRpm, double jogDistanceMm);

//-----------------------------------------------------------------------------
// Status and Feedback
// Functions for reporting motor state and diagnostics
//-----------------------------------------------------------------------------
bool isMotorReady();
bool isMotorMoving();
bool isMotorAtPosition();
bool isMotorInPosition();
bool hasMotorFault();
MotorState updateMotorState();
double getMotorPositionMm();
void printMotorStatus();
void printMotorAlerts();

//-----------------------------------------------------------------------------
// Fault Management
// Functions for detecting and clearing motor faults
//-----------------------------------------------------------------------------
bool clearMotorFaultWithStatus();
void clearMotorFaults();
void processFaultClearing();
bool isFaultClearingInProgress();

//-----------------------------------------------------------------------------
// Safety Functions
// E-stop and emergency handling
//-----------------------------------------------------------------------------
bool isEStopActive();
void handleEStop();
void updateMotorTarget(float targetPositionMm);

//-----------------------------------------------------------------------------
// Deceleration Configuration
// Functions for configuring deceleration settings
//-----------------------------------------------------------------------------

int32_t calculateDeceleratedVelocity(float distanceToTargetMm, int32_t maxVelocity);
#endif // MOTOR_CONTROLLER_H