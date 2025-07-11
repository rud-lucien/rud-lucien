#ifndef VALVE_CONTROLLER_H
#define VALVE_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <Arduino.h>
#include "ClearCore.h"

//=============================================================================
// VALVE CONFIGURATION
//=============================================================================
// Hardware pin assignments (CCIO-IO4 for pneumatic cylinder valve)
#define PNEUMATIC_CYLINDER_VALVE_PIN CLEARCORE_PIN_CCIOA4

// Valve timing constants
#define VALVE_PULSE_DURATION_MS 100        // Duration to hold valve signal (ms)
#define VALVE_SENSOR_TIMEOUT_MS 2000       // Time to wait for sensor confirmation (ms)
#define VALVE_DEBOUNCE_TIME_MS 50          // Debounce time for sensor readings (ms)

// Pressure requirements for valve operation
#define MIN_VALVE_PRESSURE_SCALED 3000     // Minimum pressure for valve operation (30.00 PSI * 100)

//=============================================================================
// VALVE ENUMS AND STRUCTURES
//=============================================================================

// Valve position states for the pneumatic cylinder
enum ValvePosition
{
    VALVE_POSITION_RETRACTED,              // Valve de-energized, cylinder retracted (spring return)
    VALVE_POSITION_EXTENDED                // Valve energized, cylinder extended
};

// Valve operation results
enum ValveOperationResult
{
    VALVE_OP_SUCCESS,                      // Operation completed successfully
    VALVE_OP_TIMEOUT,                      // Sensor didn't confirm within timeout
    VALVE_OP_PRESSURE_LOW,                 // Insufficient air pressure
    VALVE_OP_NO_CCIO,                      // CCIO board not available
    VALVE_OP_ALREADY_AT_POSITION,          // Already at requested position
    VALVE_OP_SENSOR_ERROR                  // Sensor reading error
};

// Single solenoid valve structure (monostable 5/2-way)
struct PneumaticValve
{
    int controlPin;                        // Digital output pin to control valve
    ValvePosition currentPosition;         // Current valve position
    unsigned long lastOperationTime;      // Timestamp of last operation
    bool initialized;                      // Initialization status
};

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

extern PneumaticValve cylinderValve;      // Main pneumatic cylinder valve
extern unsigned long lastValveOperationTime; // Global timestamp for valve operations
extern bool lastValveOperationFailed;    // Status of last valve operation
extern char lastValveFailureDetails[100]; // Details of last failure

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// Initialization
void initValveSystem(bool hasCCIOBoard);

// Low-level valve control
void setValvePosition(ValvePosition position);
ValvePosition getValvePosition();
bool isValveAtPosition(ValvePosition position);

// Safe valve operations with sensor feedback
ValveOperationResult safeSetValvePosition(ValvePosition targetPosition, unsigned long timeoutMs = VALVE_SENSOR_TIMEOUT_MS);
ValveOperationResult extendCylinder(unsigned long timeoutMs = VALVE_SENSOR_TIMEOUT_MS);
ValveOperationResult retractCylinder(unsigned long timeoutMs = VALVE_SENSOR_TIMEOUT_MS);

// Status and diagnostics
void printValveStatus();
void printValveDetailedStatus();
const char* getValvePositionName(ValvePosition position);
const char* getValveOperationResultName(ValveOperationResult result);

// Safety and validation
bool isValveOperationSafe();
bool isPressureSufficientForValve();
bool isValveSystemReady();
bool validateValvePosition();  // Check if valve state matches sensor readings
bool isCylinderActuallyRetracted();  // Verify cylinder is retracted via sensors
bool isCylinderActuallyExtended();   // Verify cylinder is extended via sensors

// Utility functions
void resetValveErrorState();
unsigned long getTimeSinceLastValveOperation();

#endif // VALVE_CONTROLLER_H
