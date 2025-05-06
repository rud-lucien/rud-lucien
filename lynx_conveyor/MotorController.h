#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"
#include "ClearCore.h"

// ----------------- Pin Definitions -----------------

// Motor connector on ClearCore
#define MOTOR_CONNECTOR ConnectorM0

// Home sensor input
#define HOME_SENSOR_PIN A10

// E-stop input
#define E_STOP_PIN DI6  // E-stop connected to DI6 (Normally Closed)
#define E_STOP_CHECK_INTERVAL_MS 10  // Check E-stop every 10ms

// ----------------- Constants -----------------

// Motor & Motion parameters
#define PULSES_PER_REV 3200              // Motor configured for 3200 pulses per revolution
#define MM_PER_REV 53.66                 // Calibrated: 53.66mm travel per revolution
#define PULSES_PER_MM (PULSES_PER_REV / MM_PER_REV)  // ~59.64 pulses per mm

// Motion profile parameters (in RPM units)
#define MOTOR_VELOCITY_RPM 1000           // Maximum velocity for motor operation (RPM)
#define MAX_ACCEL_RPM_PER_SEC 5000       // Maximum acceleration in RPM/s
#define ACCEL_LIMIT_RPM_PER_SEC 2500     // Acceleration limit for normal operation
#define HOME_VELOCITY_RPM 30             // Slower velocity used during homing

// Motion direction control (1 for normal direction, -1 for reversed)
#define MOTION_DIRECTION -1  // Set to -1 to reverse all motion

// Homing parameters
#define HOME_TIMEOUT_MS 60000            // Timeout for homing operation (1 minute)
#define SENSOR_DEBOUNCE_MS 50            // Debounce time for home sensor in milliseconds

// Homing direction: 1 for positive, -1 for negative
#define HOMING_DIRECTION 1  // Change to 1 for positive direction

// Move away sequence parameters
#define MOVE_AWAY_DISTANCE_MM 30.0        // Distance to move away from home sensor (in millimeters)
#define MOVE_AWAY_TIMEOUT_MS 3000         // Timeout for move away sequence (in milliseconds)

// System travel limits - based on physical measurement
#define MAX_TRAVEL_MM 1092.2            // Maximum travel in mm (measured)
#define MAX_TRAVEL_PULSES (int32_t)(MAX_TRAVEL_MM * PULSES_PER_MM)  // ~65,142 counts

// Position definitions (in mm from home)
#define POSITION_1_MM 0.0                // Position 1 (home position)
#define POSITION_2_MM (MAX_TRAVEL_MM * 0.25)  // Position 2 (25% = ~273 mm) 
#define POSITION_3_MM (MAX_TRAVEL_MM * 0.5)   // Position 3 (50% = ~546 mm)
#define POSITION_4_MM (MAX_TRAVEL_MM * 0.75)  // Position 4 (75% = ~819 mm)
#define POSITION_5_MM MAX_TRAVEL_MM          // Position 5 (max = 1092.2 mm)

// Position definitions in pulses (calculated from mm)
#define POSITION_1_PULSES (PULSES_PER_MM * POSITION_1_MM)       // Position 1 in pulses
#define POSITION_2_PULSES (PULSES_PER_MM * POSITION_2_MM)       // Position 2 in pulses  
#define POSITION_3_PULSES (PULSES_PER_MM * POSITION_3_MM)       // Position 3 in pulses
#define POSITION_4_PULSES (PULSES_PER_MM * POSITION_4_MM)       // Position 4 in pulses
#define POSITION_5_PULSES (PULSES_PER_MM * POSITION_5_MM)       // Position 5 in pulses

// ----------------- Type Definitions -----------------

enum MotorState {
    MOTOR_STATE_IDLE,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_HOMING,
    MOTOR_STATE_FAULTED,
    MOTOR_STATE_NOT_READY
};

enum PositionTarget {
    POSITION_1,
    POSITION_2,
    POSITION_3,
    POSITION_4,
    POSITION_5,
    POSITION_CUSTOM
};

// ----------------- Global Variables -----------------

extern bool isHomed;                     // Flag indicating if the motor has been homed
extern bool motorInitialized;            // Flag indicating if the motor is initialized
extern double currentPositionMm;         // Current position in mm
extern MotorState motorState;            // Current motor state
extern PositionTarget currentPosition;   // Current target position
extern int32_t currentVelMax;            // Current velocity maximum in pulse/sec
extern int32_t currentAccelMax;          // Current acceleration maximum in pulse/sec²
extern bool homingInProgress;            // Flag indicating if homing is in progress

// ----------------- Function Declarations -----------------

// Initialization functions
void initMotorSystem();

// RPM conversion utility functions
int32_t rpmToPps(double rpm);                // Convert RPM to Pulses Per Second
double ppsToRpm(int32_t pps);                // Convert Pulses Per Second to RPM
int32_t rpmPerSecToPpsPerSec(double rpmPerSec); // Convert RPM/s to Pulses/s^2

// Homing functions
bool homeSensorTriggered();
bool startHoming();
void checkHomingProgress();
bool isHomingComplete();
void checkHomeSensor();

// Motion control functions
bool moveToPosition(PositionTarget position);
/**
 * Move to a predefined position by position enum
 * What it does: Moves to one of the 5 preset positions using the PositionTarget enum
 * When to use: For internal code that uses the position enumerations
 * @param position Position target enum value
 * @return true if movement was successfully commanded, false if error
 */
bool moveToPosition(PositionTarget position);
/**
 * Move to an absolute position specified in millimeters
 * What it does: Moves the motor to a specific position measured in millimeters
 * When to use: For normal operation using real-world distance units
 * @param positionMm Target position in millimeters (0 = home position)
 * @return true if movement was successfully commanded, false if error
 */
bool moveToPositionMm(double positionMm);
/**
 * Move relative to the current position
 * What it does: Moves the motor a specific distance from its current position
 * When to use: For incremental movements without knowing/caring about absolute position
 * @param distanceMm Distance to move in mm (positive = forward, negative = backward)
 * @return true if movement was successfully commanded, false if error
 */
bool moveRelative(double distanceMm);
/**
 * Move to an absolute position specified in motor pulses
 * What it does: Moves the motor to a specific position measured in pulses
 * When to use: For precise low-level control or when interfacing with systems that use pulse units
 * @param position Position in pulses (0 = home position)
 * @return true if movement was successfully commanded, false if error
 */
bool moveToAbsolutePosition(int32_t position);
/**
 * Move to a predefined position by position number (1-5)
 * What it does: Moves to one of the 5 preset positions defined in MotorController.h
 * When to use: For moving to commonly used positions with a single command
 * @param positionNumber Position number (1-5) corresponding to defined positions
 * @return true if movement was successfully commanded, false if error
 */
bool moveToPosition(int positionNumber);
/**
 * Stop the motor immediately
 * What it does: Abruptly stops any current motion
 * When to use: For emergency stops or when motion needs to be canceled
 */
void stopMotion();

// E-stop and abort functions
/**
 * Check if E-stop is currently active
 * What it does: Reads the E-stop input pin and returns its state
 * @return true if E-stop is triggered, false if E-stop is inactive
 */
bool isEStopActive();

/**
 * Handle E-stop state changes
 * What it does: Monitors E-stop input and takes action when state changes
 */
void handleEStop();
void abortHoming();

// Status and feedback functions
bool isMotorReady();
bool isMotorMoving();
bool isMotorAtPosition();
bool isMotorInPosition();
bool hasMotorFault();
MotorState updateMotorState();
void clearMotorFault();
void clearMotorFaults();
/**
 * Get the current motor position in millimeters
 * What it does: Returns the current motor position converted to millimeters
 * When to use: For querying the current position in real-world units
 * @return Current position in millimeters
 */
double getMotorPositionMm();
void printMotorStatus();
void printMotorAlerts();
bool testMotorRange();

// Helper functions
/**
 * Convert millimeters to motor pulses
 * What it does: Converts a distance in millimeters to equivalent motor pulses
 * @param mm Distance in millimeters
 * @return Equivalent number of motor pulses
 */
int32_t mmToPulses(double mm);

/**
 * Convert motor pulses to millimeters
 * What it does: Converts motor pulses to equivalent distance in millimeters
 * @param pulses Number of motor pulses
 * @return Equivalent distance in millimeters
 */
double pulsesToMm(int32_t pulses);
void checkMoveProgress();

#endif // MOTOR_CONTROLLER_H