#ifndef VALVE_CONTROLLER_H
#define VALVE_CONTROLLER_H

#include "Arduino.h"
#include "ClearCore.h"

//=============================================================================
// TYPE DEFINITIONS
//=============================================================================

// Valve position states
enum ValvePosition
{
    VALVE_POSITION_UNLOCK,
    VALVE_POSITION_LOCK
};

// Double solenoid valve structure
struct DoubleSolenoidValve
{
    int unlockPin;
    int lockPin;
    ValvePosition position;
};

// Sensor structure
struct CylinderSensor
{
    int pin;
    bool lastState;
};

//=============================================================================
// PIN MAPPING CONSTANTS
//=============================================================================

// Valve solenoid pins - Use ClearCore CCIO board pins
#define TRAY_1_LOCK_PIN CLEARCORE_PIN_CCIOA0    // CCIO IO-0 connector
#define TRAY_1_UNLOCK_PIN CLEARCORE_PIN_CCIOA1  // CCIO IO-1 connector
#define TRAY_2_LOCK_PIN CLEARCORE_PIN_CCIOA2    // CCIO IO-2 connector
#define TRAY_2_UNLOCK_PIN CLEARCORE_PIN_CCIOA3  // CCIO IO-3 connector
#define TRAY_3_LOCK_PIN CLEARCORE_PIN_CCIOA4    // CCIO IO-4 connector
#define TRAY_3_UNLOCK_PIN CLEARCORE_PIN_CCIOA5  // CCIO IO-5 connector
#define SHUTTLE_LOCK_PIN CLEARCORE_PIN_CCIOA6   // CCIO IO-6 connector
#define SHUTTLE_UNLOCK_PIN CLEARCORE_PIN_CCIOA7 // CCIO IO-7 connector

// Cylinder position sensors (detect lock/unlock state)
#define TRAY_1_CYLINDER_SENSOR_PIN 4    // ClearCore IO-4 - Detects Tray 1 cylinder position
#define TRAY_2_CYLINDER_SENSOR_PIN 5    // ClearCore IO-5 - Detects Tray 2 cylinder position
#define TRAY_3_CYLINDER_SENSOR_PIN A9   // ClearCore A9 - Detects Tray 3 cylinder position
#define SHUTTLE_CYLINDER_SENSOR_PIN A10 // ClearCore A10 - Detects Shuttle cylinder position

// Tray detection sensors on IO pins of ClearCore main board
#define TRAY_1_DETECT_PIN 1 // ClearCore IO-1 - Detects tray at position 1
#define TRAY_2_DETECT_PIN 2 // ClearCore IO-2 - Detects tray at position 2
#define TRAY_3_DETECT_PIN 3 // ClearCore IO-3 - Detects tray at position 3

// Other constants
#define PULSE_DURATION 100 // Minimum recommended pulse duration in milliseconds

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Valve instances
extern DoubleSolenoidValve tray1Valve;
extern DoubleSolenoidValve tray2Valve;
extern DoubleSolenoidValve tray3Valve;
extern DoubleSolenoidValve shuttleValve;

// Cylinder position sensor instances
extern CylinderSensor tray1CylinderSensor;
extern CylinderSensor tray2CylinderSensor;
extern CylinderSensor tray3CylinderSensor;
extern CylinderSensor shuttleCylinderSensor;

// Tray detection sensor instances
extern CylinderSensor tray1DetectSensor;
extern CylinderSensor tray2DetectSensor;
extern CylinderSensor tray3DetectSensor;

// Arrays for batch operations
extern DoubleSolenoidValve *allValves[4];
extern const int valveCount;
extern const char *valveNames[4];

extern CylinderSensor *allCylinderSensors[4];
extern const int cylinderSensorCount;

extern CylinderSensor *allTrayDetectSensors[3];
extern const int trayDetectSensorCount;

// CCIO Board status
extern bool hasCCIO;

//=============================================================================
// FUNCTION DECLARATIONS
//-----------------------------------------------------------------------------
// System Initialization
// Setup valve and sensor systems
//-----------------------------------------------------------------------------
void initSensorSystem();
void initValveSystem(bool hasCCIOBoard);

//-----------------------------------------------------------------------------
// Basic Hardware Control
// Low-level control of hardware pins
//-----------------------------------------------------------------------------
void pulsePin(int pin, unsigned long duration);
int getActivationPin(const DoubleSolenoidValve &valve, ValvePosition target);

//-----------------------------------------------------------------------------
// Valve Core Operations
// Direct control of valves
//-----------------------------------------------------------------------------
void valveInit(DoubleSolenoidValve &valve);
void valveSetPosition(DoubleSolenoidValve &valve, ValvePosition target);
ValvePosition valveGetPosition(const DoubleSolenoidValve &valve);

//-----------------------------------------------------------------------------
// Valve Safety Operations
// Higher-level valve operations with appropriate safety considerations
//-----------------------------------------------------------------------------
void unsafeLockValve(DoubleSolenoidValve &valve);
void unsafeUnlockValve(DoubleSolenoidValve &valve);
bool safeUnlockAllValves(unsigned long timeoutMs = 1000);

//-----------------------------------------------------------------------------
// Sensor Operations
// Initialize and read sensors
//-----------------------------------------------------------------------------
void sensorInit(CylinderSensor &sensor, int pin);
bool sensorRead(CylinderSensor &sensor);

//-----------------------------------------------------------------------------
// Status Reporting
// Display valve and sensor states
//-----------------------------------------------------------------------------
void printValveStatus(const DoubleSolenoidValve &valve, const char *valveName);
void printSensorStatus(const CylinderSensor &sensor, const char *sensorName);
void printAllValveStatus();
void printAllSensorStatus();
void printTrayDetectionStatus();

//-----------------------------------------------------------------------------
// Batch Operations
// Perform operations on all valves
//-----------------------------------------------------------------------------
void withAllValves(void (*operation)(DoubleSolenoidValve &));

//-----------------------------------------------------------------------------
// Advanced Operations
// Operations with sensor feedback and safety features
//-----------------------------------------------------------------------------
bool waitForSensor(CylinderSensor &sensor, bool expectedState, unsigned long timeoutMs);
bool safeValveOperation(DoubleSolenoidValve &valve, CylinderSensor &sensor,
                        ValvePosition targetPosition, unsigned long timeoutMs);

//-----------------------------------------------------------------------------
// Accessor Functions
//-----------------------------------------------------------------------------

// Helper function to access shuttle valve specifically
DoubleSolenoidValve *getShuttleValve();

// Helper functions to access specific valves
DoubleSolenoidValve *getTray1Valve();
DoubleSolenoidValve *getTray2Valve();
DoubleSolenoidValve *getTray3Valve();

// Helper functions to access cylinder sensors
CylinderSensor *getTray1Sensor();
CylinderSensor *getTray2Sensor();
CylinderSensor *getTray3Sensor();
CylinderSensor *getShuttleSensor();

// Helper functions to access tray detection sensors
CylinderSensor *getTray1DetectionSensor();
CylinderSensor *getTray2DetectionSensor();
CylinderSensor *getTray3DetectionSensor();

#endif // VALVE_CONTROLLER_H