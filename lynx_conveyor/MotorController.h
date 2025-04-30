#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"
#include "ClearCore.h"

// ----------------- Pin Definitions -----------------

// Motor connector on ClearCore
#define MOTOR_CONNECTOR ConnectorM0

// Home sensor input
#define HOME_SENSOR_PIN A10

// ----------------- Constants -----------------

// Motor & Motion parameters
#define PULSES_PER_REV 3200              // Motor configured for 3200 pulses per revolution
#define MM_PER_REV 60.96                 // 60.96mm travel per revolution (30 teeth MXL pulley)
#define PULSES_PER_MM (PULSES_PER_REV / MM_PER_REV)  // ~52.53 pulses per mm

// Motion profile parameters (in RPM units)
#define MAX_VELOCITY_RPM 150             // Maximum velocity in RPM 
#define MAX_ACCEL_RPM_PER_SEC 5000       // Maximum acceleration in RPM/s
#define VELOCITY_LIMIT_RPM 75           // Velocity limit for normal operation
#define ACCEL_LIMIT_RPM_PER_SEC 2500     // Acceleration limit for normal operation
#define HOME_VELOCITY_RPM 50             // Slower velocity used during homing

// Homing parameters
#define HOME_TIMEOUT_MS 60000            // Timeout for homing operation (1 minute)
#define SENSOR_DEBOUNCE_MS 50            // Debounce time for home sensor in milliseconds

// Position definitions (in mm from home)
#define POSITION_1_MM 0.0                // Position 1 (home position)
#define POSITION_2_MM 250.0              // Position 2
#define POSITION_3_MM 500.0              // Position 3
#define POSITION_4_MM 750.0              // Position 4
#define POSITION_5_MM 1000.0             // Position 5 (max position)

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
bool moveToPositionMm(double positionMm);
bool moveRelative(double distanceMm);
bool moveToAbsolutePosition(int32_t position);
bool moveToPosition(int positionNumber);
void stopMotion();

// Status and feedback functions
bool isMotorReady();
bool isMotorMoving();
bool isMotorAtPosition();
bool isMotorInPosition();
bool hasMotorFault();
MotorState updateMotorState();
void clearMotorFault();
void clearMotorFaults();
double getMotorPositionMm();
void printMotorStatus();
void printMotorAlerts();
bool testMotorRange();

// Helper functions
int32_t mmToPulses(double mm);
double pulsesToMm(int32_t pulses);
void checkMoveProgress();

#endif // MOTOR_CONTROLLER_H