#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

//=============================================================================
// DEPENDENCIES
//=============================================================================
#include <Arduino.h>
#include "ClearCore.h"
#include "OutputManager.h"
#include "Utils.h"

//=============================================================================
// HARDWARE CONFIGURATION
//=============================================================================
// Motor Assignments
#define RAIL1_MOTOR         ConnectorM0    // Rail 1 carriage motor
#define RAIL2_MOTOR         ConnectorM1    // Rail 2 carriage motor

// Emergency Stop Configuration
#define E_STOP_PIN A12                     // Emergency stop input pin (interrupt-capable)
#define E_STOP_CHECK_INTERVAL_MS 10        // E-stop monitoring frequency (for status polling)
#define E_STOP_INTERRUPT_DEBOUNCE_MS 50    // Debounce time for interrupt (prevent false triggers)

//=============================================================================
// MOTOR & MOTION PARAMETERS
//=============================================================================
// Common Motor Configuration
#define PULSES_PER_REV 800                 // Motor steps per revolution
#define MAX_ACCEL_RPM_PER_SEC 2500         // Maximum acceleration

// Rail-Specific Velocity Configuration
// Rail 1 (8.2m travel) - Longer distances, can handle higher speeds
#define RAIL1_LOADED_CARRIAGE_VELOCITY_RPM 325   // Velocity with labware
#define RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM 800    // Velocity without labware

// Rail 2 (1m travel) - Shorter distances, precision movements
#define RAIL2_LOADED_CARRIAGE_VELOCITY_RPM 250   // Slower for precision with labware
#define RAIL2_EMPTY_CARRIAGE_VELOCITY_RPM 600    // Moderate for short distance

// Rail 1 Parameters (8.2m travel)
#define RAIL1_MM_PER_REV 53.98             // Travel per revolution
// Integer math optimization: 800 pulses / 53.98mm = 14.823 pulses/mm
// Use scaled integer: 1482.3 * 100 = 148230 (pulses per 100mm)
#define RAIL1_PULSES_PER_MM_SCALED 148230  // Pulses per 100mm (for integer math)
#define RAIL1_PULSES_PER_MM (PULSES_PER_REV / RAIL1_MM_PER_REV)  // Keep for compatibility
#define RAIL1_LENGTH_MM 8200               // Total rail length
#define RAIL1_MAX_TRAVEL_MM 8000           // Usable travel distance

// Rail 2 Parameters (1m travel)
#define RAIL2_MM_PER_REV 53.98             // Travel per revolution  
// Integer math optimization: Same as Rail 1 (same mechanical setup)
#define RAIL2_PULSES_PER_MM_SCALED 148230  // Pulses per 100mm (for integer math)
#define RAIL2_PULSES_PER_MM (PULSES_PER_REV / RAIL2_MM_PER_REV)  // Keep for compatibility
#define RAIL2_LENGTH_MM 1000               // Total rail length
#define RAIL2_MAX_TRAVEL_MM 1000           // Usable travel distance

//=============================================================================
// POSITION DEFINITIONS
//=============================================================================
// Rail 1 Positions (distances from home in mm)
#define RAIL1_HOME_POSITION 0              // Home position
#define RAIL1_WC2_PICKUP_DROPOFF 3700      // Workcell 2 pickup/dropoff 
#define RAIL1_WC1_PICKUP_DROPOFF 5700      // Workcell 1 pickup/dropoff
#define RAIL1_STAGING_POSITION 150        // Staging position
#define RAIL1_HANDOFF 35                 // Handoff to Rail 2 this may or may not be the home position. need to figure this out

// Rail 2 Positions (distances from home in mm)
#define RAIL2_HOME_POSITION 0              // Home position  
#define RAIL2_HANDOFF 900                  // Handoff from Rail 1
#define RAIL2_WC3_PICKUP_DROPOFF 95       // Workcell 3 pickup/dropoff. this maybe the home position so for now I define them both

// Pre-calculated Position Values (pulses)
#define RAIL1_HOME_POSITION_PULSES (int32_t)(RAIL1_HOME_POSITION * RAIL1_PULSES_PER_MM)
#define RAIL1_WC2_PICKUP_DROPOFF_PULSES (int32_t)(RAIL1_WC2_PICKUP_DROPOFF * RAIL1_PULSES_PER_MM)
#define RAIL1_WC1_PICKUP_DROPOFF_PULSES (int32_t)(RAIL1_WC1_PICKUP_DROPOFF * RAIL1_PULSES_PER_MM)
#define RAIL1_STAGING_POSITION_PULSES (int32_t)(RAIL1_STAGING_POSITION * RAIL1_PULSES_PER_MM)
#define RAIL1_HANDOFF_PULSES (int32_t)(RAIL1_HANDOFF * RAIL1_PULSES_PER_MM)

#define RAIL2_HOME_POSITION_PULSES (int32_t)(RAIL2_HOME_POSITION * RAIL2_PULSES_PER_MM)
#define RAIL2_HANDOFF_PULSES (int32_t)(RAIL2_HANDOFF * RAIL2_PULSES_PER_MM)
#define RAIL2_WC3_PICKUP_DROPOFF_PULSES (int32_t)(RAIL2_WC3_PICKUP_DROPOFF * RAIL2_PULSES_PER_MM)

//=============================================================================
// MOTION DIRECTION DEFINITIONS
//=============================================================================
// Rail 1 Directions
#define RAIL1_DIRECTION_TOWARD_HANDOFF 1   // Positive: toward handoff
#define RAIL1_DIRECTION_TOWARD_HOME -1     // Negative: toward home

// Rail 2 Directions
#define RAIL2_DIRECTION_TOWARD_HANDOFF 1   // Positive: toward handoff/motor
#define RAIL2_DIRECTION_TOWARD_HOME -1     // Negative: toward home/WC3

// Generic Direction Constants
#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD -1

//=============================================================================
// HOMING CONFIGURATION
//=============================================================================
// Rail-Specific Homing Timeouts
#define RAIL1_HOME_TIMEOUT_MS 300000       // Rail 1 homing timeout (5 minutes for 8.2m rail at slow homing speed)
#define RAIL2_HOME_TIMEOUT_MS 60000        // Rail 2 homing timeout (1 minute for 1m rail - generous safety margin)

// Legacy timeout for backward compatibility
#define HOME_TIMEOUT_MS RAIL1_HOME_TIMEOUT_MS

// Homing Tolerances and Delays
#define HOME_VERIFICATION_TOLERANCE_PULSES 10
#define HOME_COMPLETION_DELAY_MS 1000

// Homing Motion Parameters
#define HOME_APPROACH_VELOCITY_RPM 40      // Homing approach speed
#define HOMING_MIN_MOVEMENT_PULSES 200     // Minimum travel before hardstop detection
#define HOMING_MIN_ADDITIONAL_PULSES 100   // Additional travel after minimum distance
#define HOMING_DEBOUNCE_TIME_MS 250        // HLFB state debounce time
#define HOMING_MIN_TIME_AFTER_DISTANCE_MS 500

// Rail-Specific Homing Parameters
#define RAIL1_HOMING_DIRECTION -1          // Rail 1 homes toward home position (negative direction)
#define RAIL1_HOME_OFFSET_DISTANCE_MM 5    // Offset from hardstop
#define RAIL2_HOMING_DIRECTION 1           // Rail 2 homes away from home position (positive direction)
#define RAIL2_HOME_OFFSET_DISTANCE_MM 5    // Offset from hardstop

//=============================================================================
// SMART HOMING CONSTANTS
//=============================================================================

// Smart homing speeds and distances
#define HOME_FAST_APPROACH_VELOCITY_RPM 200    // Fast approach speed for re-homing
#define HOME_PRECISION_DISTANCE_MM 50          // Distance to switch to precision homing
#define HOME_MIN_DISTANCE_FOR_SMART_MM 100     // Minimum distance to enable smart homing

// Pulse-based constants for integer math (calculated at compile time)
#define HOME_PRECISION_DISTANCE_PULSES_RAIL1 (HOME_PRECISION_DISTANCE_MM * RAIL1_PULSES_PER_MM)
#define HOME_PRECISION_DISTANCE_PULSES_RAIL2 (HOME_PRECISION_DISTANCE_MM * RAIL2_PULSES_PER_MM)
#define HOME_MIN_DISTANCE_PULSES_RAIL1 (HOME_MIN_DISTANCE_FOR_SMART_MM * RAIL1_PULSES_PER_MM)
#define HOME_MIN_DISTANCE_PULSES_RAIL2 (HOME_MIN_DISTANCE_FOR_SMART_MM * RAIL2_PULSES_PER_MM)

//=============================================================================
// MOTOR INITIALIZATION CONSTANTS
//=============================================================================
#define MOTOR_INIT_TIMEOUT_MS 2000         // Timeout for motor initialization (2 seconds)
#define MOTOR_INIT_POLL_DELAY_MS 10        // Polling delay during motor initialization

//=============================================================================
// JOGGING SPEED AND DISTANCE THRESHOLDS
//=============================================================================
// Distance thresholds for automatic jog speed capping (in mm)
#define JOG_VERY_SHORT_THRESHOLD_MM 1.0    // Very short jog distance threshold
#define JOG_SHORT_THRESHOLD_MM 5.0         // Short jog distance threshold  
#define JOG_MEDIUM_THRESHOLD_MM 20.0       // Medium jog distance threshold

// Maximum speeds for different jog distances (in RPM)
#define JOG_VERY_SHORT_MAX_SPEED_RPM 100   // Speed cap for very short jogs (≤1mm)
#define JOG_SHORT_MAX_SPEED_RPM 200        // Speed cap for short jogs (≤5mm)
#define JOG_MEDIUM_MAX_SPEED_RPM 400       // Speed cap for medium jogs (≤20mm)
#define JOG_LONG_MAX_SPEED_RPM 600         // Speed cap for long jogs (>20mm)

//=============================================================================
// MOVEMENT PROGRESS MONITORING CONSTANTS
//=============================================================================
#define MOVEMENT_TIMEOUT_MS 120000         // Maximum time allowed for any movement (2 minutes)
#define MOVEMENT_STALL_TIMEOUT_MS 5000     // Time without progress before declaring stall (5 seconds)
#define MOVEMENT_MIN_PROGRESS_PULSES 50    // Minimum pulse movement to avoid stall detection

//=============================================================================
// MOTOR HOMING CONSTANTS
//=============================================================================
// Rail 1 Homing Parameters
#define RAIL1_HOME_VELOCITY_RPM 100         // Homing speed for Rail 1
#define RAIL1_HOME_ACCEL_RPM_PER_SEC 500    // Homing acceleration for Rail 1

// Rail 2 Homing Parameters
#define RAIL2_HOME_VELOCITY_RPM 100         // Homing speed for Rail 2
#define RAIL2_HOME_ACCEL_RPM_PER_SEC 500    // Homing acceleration for Rail 2

//=============================================================================
// JOGGING CONFIGURATION
//=============================================================================
// Default Jog Speed Settings
#define RAIL1_DEFAULT_JOG_SPEED_RPM 200    // Default jog speed for Rail 1 (longer rail)
#define RAIL2_DEFAULT_JOG_SPEED_RPM 150    // Default jog speed for Rail 2 (precision rail)

// Default Jog Increment Settings  
#define RAIL1_DEFAULT_JOG_INCREMENT_MM 1.0   // Default jog increment for Rail 1 (1mm - fine positioning)
#define RAIL2_DEFAULT_JOG_INCREMENT_MM 0.5   // Default jog increment for Rail 2 (0.5mm - precision positioning)

// Maximum Jog Increment Limits
#define RAIL1_MAX_JOG_INCREMENT_MM 50.0      // Maximum jog increment for Rail 1 (50mm max)
#define RAIL2_MAX_JOG_INCREMENT_MM 25.0      // Maximum jog increment for Rail 2 (25mm max)

//=============================================================================
// MOVEMENT TRACKING CONFIGURATION
//=============================================================================
// Movement timeout and validation
#define MOVEMENT_TIMEOUT_MS 120000         // 2 minutes maximum movement time
#define MOVEMENT_POSITION_TOLERANCE_MM 2.0 // ±2mm position tolerance for target validation
#define MOVEMENT_STALL_CHECK_INTERVAL_MS 1000 // Check for stalled movement every 1 second
#define MOVEMENT_MIN_PROGRESS_MM 5.0       // Minimum movement progress per check interval
#define DECELERATION_UPDATE_INTERVAL_MS 50 // Update velocity every 50ms during deceleration

//=============================================================================
// DATA STRUCTURES
//=============================================================================
// Motor Operational States
enum MotorState {
    MOTOR_STATE_NOT_READY,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_HOMING,
    MOTOR_STATE_FAULTED
};

// Position Target Enumeration
typedef enum {
    POSITION_UNDEFINED = -1,
    
    // Rail 1 Positions
    RAIL1_HOME_POS = 0,
    RAIL1_WC2_PICKUP_DROPOFF_POS = 1,     
    RAIL1_WC1_PICKUP_DROPOFF_POS = 2,     
    RAIL1_STAGING_POS = 3,       
    RAIL1_HANDOFF_POS = 4,                

    // Rail 2 Positions
    RAIL2_HOME_POS = 5,                  
    RAIL2_HANDOFF_POS = 6,               
    RAIL2_WC3_PICKUP_DROPOFF_POS = 7,

    POSITION_CUSTOM = 99
} PositionTarget;

// Rail Deceleration Configuration
struct RailDecelerationConfig {
    int32_t longMoveDecelerationDistanceMm;
    int32_t mediumMoveDecelerationDistanceMm;  
    int32_t shortMoveDecelerationDistanceMm;
    int32_t longMoveThresholdMm;
    int32_t mediumMoveThresholdMm;
    int32_t minVelocityRPM;
    bool enableDeceleration;
};

// Motor Homing State
struct MotorHomingState {
    bool homingInProgress;
    bool isHomed;
    bool hlfbWentNonAsserted;
    bool minDistanceTraveled;
    unsigned long homingStartTime;
    unsigned long hlfbNonAssertedTime;
    unsigned long lastPositionCheckTime;
    unsigned long minTimeAfterDistanceReached;
    int32_t startPulses;
    int32_t lastCheckedPosition;
    int32_t positionAtMinDistance;
    int32_t pulsesTraveledAfterMinDistance;
    bool previousEncoderState;
};

// Motor Movement Target Tracking State
struct MotorTargetState {
    bool movementInProgress;
    PositionTarget targetPosition;          // Intended target position
    int32_t targetPositionPulses;          // Target position in pulses
    int32_t startPositionPulses;           // Starting position when movement began
    int32_t totalMoveDistancePulses;       // Total movement distance
    int32_t originalVelocityRpm;           // Original velocity before deceleration
    bool carriageLoaded;                   // Whether carriage has labware
    unsigned long movementStartTime;       // When movement started
    unsigned long lastProgressCheck;       // Last time we checked progress
    unsigned long lastDecelerationUpdate; // Last time velocity was updated during deceleration
    int32_t lastPositionCheck;             // Position at last progress check
    bool decelerationActive;               // Whether deceleration is currently active
    bool movementValidated;                // Whether final position was validated
    int movementTimeoutCount;              // Number of timeout warnings issued
};

// Fixed-point scaling for precise calculations


//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// System Initialization and Safety
bool isEStopActive();
void handleEStop();
void setupEStopInterrupt();               // Initialize interrupt-based E-stop monitoring
void eStopInterruptHandler();             // Interrupt service routine for E-stop
void printMotorAlerts(MotorDriver &motor, const char* motorName);
bool initSingleMotor(MotorDriver &motor, const char* motorName, int32_t velocityRpm, int32_t accelRpmPerSec);
bool initEStop();
bool initRailMotor(int railNumber);
bool initMotorManager();

// Unit Conversion Utilities
int32_t rpmToPps(double rpm);
int32_t rpmPerSecToPpsPerSec(double rpmPerSec);
int32_t rail1MmToPulses(double mm);
int32_t rail2MmToPulses(double mm);
double rail1PulsesToMm(int32_t pulses);
double rail2PulsesToMm(int32_t pulses);
int32_t mmToPulses(double mm, int rail);
double pulsesToMm(int32_t pulses, int rail);

// Optimized integer math conversion functions (for performance)
int32_t mmToPulsesScaled(int32_t mmScaled, int rail);     // Input: mm * 100, Output: pulses

// Position and Rail Utilities
int32_t getPositionPulses(PositionTarget target);
int getRailFromPosition(PositionTarget target);
bool isValidPositionForRail(PositionTarget target, int rail);
bool validateAllPredefinedPositions(); // Validate all positions against travel limits
const char* getPositionName(PositionTarget pos);

// Motor Control and Status
MotorDriver& getMotorByRail(int rail);
const char* getMotorName(int rail);
double getMotorPositionMm(int rail);
int32_t getCarriageVelocityRpm(int rail, bool carriageLoaded);  // Get rail-specific velocity

// Smart Homing Helper Functions
int32_t getHomePrecisionDistancePulses(int rail);
int32_t getHomeMinDistancePulses(int rail);

void stopMotion(int rail);
void stopAllMotion();
bool isMotorReady(int rail);
bool isMotorMoving(int rail);
bool isMotorInPosition(int rail);
bool hasMotorFault(int rail);
MotorState updateMotorState(int rail);
void printMotorStatus(int rail);
void printAllMotorStatus();

// Fault Management - E-stop Safe Interface
void clearMotorFaults(int rail);                                              // E-stop safe synchronous fault clearing
bool clearMotorFaultWithStatus(int rail);                                     // E-stop safe fault clearing with status
bool clearAlertsWithEStopMonitoring(MotorDriver &motor, const char* motorName); // Core E-stop safe clearing function

// Motion Deceleration
RailDecelerationConfig& getDecelerationConfig(int rail);
int32_t getDecelerationDistanceScaled(int rail, int32_t moveDistanceScaledMm);
int32_t calculateDeceleratedVelocity(int rail, int32_t distanceToTargetMm, int32_t totalMoveDistanceMm, int32_t maxVelocity);


// Positioning and Movement
bool moveToPositionFromCurrent(int rail, PositionTarget toPos, bool carriageLoaded);
bool moveToPositionMm(int rail, double positionMm, bool carriageLoaded = false);
bool moveRelativeMm(int rail, double relativeMm, bool carriageLoaded = false);

// Homing Operations
bool initiateHomingSequence(int rail);
void checkHomingProgress(int rail);
void completeHomingSequence(int rail);
void abortHoming(int rail);
void resetHomingState(int rail);
bool isHomingComplete(int rail);
bool isHomingInProgress(int rail);
int getHomingDirection(int rail);
double getHomeOffsetDistance(int rail);
unsigned long getHomingTimeout(int rail);

// Dual-Motor Homing Convenience Functions
bool initiateHomingSequenceAll();
void checkAllHomingProgress();
bool isAllHomingComplete();

// Smart Homing Functions
bool initiateSmartHomingSequence(int rail);
bool isSmartHomingBeneficial(int rail, int32_t* estimatedTimeSavingsMs);
void calculateSmartHomingPhases(int rail, int32_t currentPositionPulses, bool* useSmartHoming, 
                               int32_t* fastPhaseDistancePulses, int32_t* precisionPhaseDistancePulses);

// Movement Progress Monitoring
void checkMoveProgress();
void updateDecelerationVelocity(int rail);
bool checkMovementTimeout(int rail, unsigned long timeoutMs);  
bool checkMovementProgress(int rail);
MotorTargetState& getTargetState(int rail);

// Manual Jogging Operations
bool jogMotor(int rail, bool direction, double customIncrement = 0, bool carriageLoaded = false);
bool setJogIncrement(int rail, double increment);
bool setJogSpeed(int rail, int speedRpm, double jogDistanceMm = 0);
double getJogIncrement(int rail);
int getJogSpeed(int rail);

// Helper functions for per-motor state tracking
void setMotorVelocity(int rail, int32_t velocityPps);
int32_t getCurrentMotorVelocity(int rail);
void setMotorAcceleration(int rail, int32_t accelPpsPerSec);

//=============================================================================
// TIMEOUT RESET FUNCTIONS
//=============================================================================
void resetMotorTimeouts();                  // Reset all motor-related timeout tracking

#endif // MOTOR_CONTROLLER_H