#ifndef VALVE_CONTROLLER_H
#define VALVE_CONTROLLER_H

#include "Arduino.h"
#include "ClearCore.h"

// ----------------- Type Definitions -----------------

enum ValvePosition {
    VALVE_POSITION_UNLOCK,
    VALVE_POSITION_LOCK
};

struct DoubleSolenoidValve {
    int unlockPin;
    int lockPin;
    ValvePosition position;
};

struct CylinderSensor {
    int pin;
    bool lastState;
};

// ----------------- Pin Mapping Constants -----------------

// Valve solenoid pins - Use ClearCore constants
#define TRAY_1_LOCK_PIN CLEARCORE_PIN_CCIOA0     // CCIO IO-0 connector  
#define TRAY_1_UNLOCK_PIN CLEARCORE_PIN_CCIOA1   // CCIO IO-1 connector
#define TRAY_2_LOCK_PIN CLEARCORE_PIN_CCIOA2     // CCIO IO-2 connector
#define TRAY_2_UNLOCK_PIN CLEARCORE_PIN_CCIOA3   // CCIO IO-3 connector
#define TRAY_3_LOCK_PIN CLEARCORE_PIN_CCIOA4     // CCIO IO-4 connector
#define TRAY_3_UNLOCK_PIN CLEARCORE_PIN_CCIOA5   // CCIO IO-5 connector
#define SHUTTLE_LOCK_PIN CLEARCORE_PIN_CCIOA6    // CCIO IO-6 connector
#define SHUTTLE_UNLOCK_PIN CLEARCORE_PIN_CCIOA7  // CCIO IO-7 connector

// Cylinder position sensors (detect lock/unlock state)
#define TRAY_1_CYLINDER_SENSOR_PIN 4    // ClearCore IO-4 - Detects Tray 1 cylinder position
#define TRAY_2_CYLINDER_SENSOR_PIN 5    // ClearCore IO-5 - Detects Tray 2 cylinder position
#define TRAY_3_CYLINDER_SENSOR_PIN A9   // ClearCore A9 - Detects Tray 3 cylinder position
#define SHUTTLE_CYLINDER_SENSOR_PIN A10 // ClearCore A10 - Detects Shuttle cylinder position

// Other constants
#define PULSE_DURATION 100  // Minimum recommended pulse duration in milliseconds


// Tray detection sensors on IO pins of ClearCore main board
#define TRAY_1_DETECT_PIN 1    // ClearCore IO-1 - Detects tray at position 1
#define TRAY_2_DETECT_PIN 2    // ClearCore IO-2 - Detects tray at position 2  
#define TRAY_3_DETECT_PIN 3    // ClearCore IO-3 - Detects tray at position 3

// ----------------- Global variables -----------------

// Valve instances
extern DoubleSolenoidValve tray1Valve;
extern DoubleSolenoidValve tray2Valve;
extern DoubleSolenoidValve tray3Valve;
extern DoubleSolenoidValve shuttleValve;

// Sensor instances
extern CylinderSensor tray1CylinderSensor;  
extern CylinderSensor tray2CylinderSensor;  
extern CylinderSensor tray3CylinderSensor;  
extern CylinderSensor shuttleCylinderSensor; 

// New tray detection sensor declarations
extern CylinderSensor tray1DetectSensor;
extern CylinderSensor tray2DetectSensor;
extern CylinderSensor tray3DetectSensor;

// Arrays for batch operations
extern DoubleSolenoidValve* allValves[4];
extern const int valveCount;
extern const char* valveNames[4];

extern CylinderSensor* allCylinderSensors[4];  // Changed from allSensors
extern const int cylinderSensorCount;  // Changed from sensorCount

// Array of tray detection sensor pointers for logging
extern CylinderSensor* allTrayDetectSensors[3];
extern const int trayDetectSensorCount;

// CCIO Board status
extern bool hasCCIO;

// ----------------- Function Declarations -----------------

// Initialization functions
void initSensorSystem();  // For all sensors on the main board
void initValveSystem(bool hasCCIOBoard);  // Single function for all valve initialization

// Low-level hardware functions
void pulsePin(int pin, unsigned long duration);
int getActivationPin(const DoubleSolenoidValve &valve, ValvePosition target);

// Valve operations
void valveInit(DoubleSolenoidValve &valve);
void valveSetPosition(DoubleSolenoidValve &valve, ValvePosition target);
void valveDeactivate(DoubleSolenoidValve &valve);
ValvePosition valveGetPosition(const DoubleSolenoidValve &valve);

// Higher-order valve operations
void withValve(DoubleSolenoidValve &valve, void (*operation)(DoubleSolenoidValve&));
void unlockValve(DoubleSolenoidValve &valve);
void lockValve(DoubleSolenoidValve &valve);
void deactivateValve(DoubleSolenoidValve &valve);

// Sensor operations
void sensorInit(CylinderSensor &sensor, int pin);
bool sensorRead(CylinderSensor &sensor);

// Status reporting
void printValveStatus(const DoubleSolenoidValve &valve, const char* valveName);
void printSensorStatus(const CylinderSensor &sensor, const char* sensorName);

// Batch operations
void withAllValves(void (*operation)(DoubleSolenoidValve&));
void printAllValveStatus();
void printAllSensorStatus();

// Advanced operations with sensor feedback
bool waitForSensor(CylinderSensor &sensor, bool expectedState, unsigned long timeoutMs);
bool safeValveOperation(DoubleSolenoidValve &valve, CylinderSensor &sensor,
                      ValvePosition targetPosition, unsigned long timeoutMs);

// Convenience functions
void lockAllValves();
void unlockAllValves();
DoubleSolenoidValve* getValveByIndex(int index);
CylinderSensor* getSensorByIndex(int index);
const char* getValveNameByIndex(int index);

void printTrayDetectionStatus();

#endif // VALVE_CONTROLLER_H