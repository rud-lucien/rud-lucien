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

// Valve solenoid pins
#define TRAY_1_LOCK_PIN 0     // IO-0 connector
#define TRAY_1_UNLOCK_PIN 1   // IO-1 connector
#define TRAY_2_LOCK_PIN 2     // IO-2 connector
#define TRAY_2_UNLOCK_PIN 3   // IO-3 connector
#define TRAY_3_LOCK_PIN 4     // IO-4 connector
#define TRAY_3_UNLOCK_PIN 5   // IO-5 connector
#define SHUTTLE_LOCK_PIN 64   // IO-6 connector (CCIO)
#define SHUTTLE_UNLOCK_PIN 65 // IO-7 connector (CCIO)

// Sensor pins
#define TRAY_1_SENSOR_PIN DI6    // Connect Tray 1 sensor to DI-6
#define TRAY_2_SENSOR_PIN DI7    // Connect Tray 2 sensor to DI-7
#define TRAY_3_SENSOR_PIN DI8    // Connect Tray 3 sensor to DI-8
#define SHUTTLE_SENSOR_PIN A9    // Connect Shuttle sensor to A9

// Other constants
#define PULSE_DURATION 100  // Minimum recommended pulse duration in milliseconds

// ----------------- Global variables -----------------

// Valve instances
extern DoubleSolenoidValve tray1Valve;
extern DoubleSolenoidValve tray2Valve;
extern DoubleSolenoidValve tray3Valve;
extern DoubleSolenoidValve shuttleValve;

// Sensor instances
extern CylinderSensor tray1Sensor;
extern CylinderSensor tray2Sensor;
extern CylinderSensor tray3Sensor;
extern CylinderSensor shuttleSensor;

// Arrays for batch operations
extern DoubleSolenoidValve* allValves[4];
extern const int valveCount;
extern const char* valveNames[4];

extern CylinderSensor* allSensors[4];
extern const int sensorCount;

// CCIO Board status
extern bool hasCCIO;

// ----------------- Function Declarations -----------------

// Initialization functions
void initValveSystem();
void initValveWithCCIO(bool hasCCIOBoard);

// Low-level hardware functions
void pulsePin(int pin, unsigned long duration);
void configurePinAsOutput(int pin);
void configureValvePins(const DoubleSolenoidValve &valve);
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

#endif // VALVE_CONTROLLER_H