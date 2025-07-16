#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "ClearCore.h"
#include "OutputManager.h"

//=============================================================================
// SENSOR PIN DEFINITIONS (Based on Pinout Diagrams)
//=============================================================================

// ClearCore Main Board Digital Inputs
#define CARRIAGE_SENSOR_WC1_PIN 3      // IO-3: Carriage at WC1 position
#define LABWARE_SENSOR_WC1_PIN 4       // IO-4: Labware present at WC1
#define CARRIAGE_SENSOR_WC2_PIN 1      // IO-1: Carriage at WC2 position  
#define LABWARE_SENSOR_WC2_PIN 2       // IO-2: Labware present at WC2
#define CARRIAGE_SENSOR_WC3_PIN 5      // IO-5: Carriage at WC3 position

// ClearCore Analog Inputs
#define PNEUMATICS_PRESSURE_SENSOR_PIN A10  // A10: Air pressure sensor

// CCIO-8 Board Digital Inputs (via CCIO) - Using proper ClearCore CCIO pin format
#define CARRIAGE_SENSOR_RAIL2_HANDOFF_PIN CLEARCORE_PIN_CCIOA0    // CCIO IO-0: Rail 2 handoff position
#define LABWARE_SENSOR_RAIL2_PIN CLEARCORE_PIN_CCIOA1             // CCIO IO-1: Rail 2 carriage-mounted labware sensor
#define CARRIAGE_SENSOR_RAIL1_HANDOFF_PIN CLEARCORE_PIN_CCIOA2    // CCIO IO-2: Rail 1 handoff position
#define LABWARE_SENSOR_RAIL1_HANDOFF_PIN CLEARCORE_PIN_CCIOA3     // CCIO IO-3: Rail 1 handoff labware sensor
#define PNEUMATICS_CYLINDER_VALVE_PIN CLEARCORE_PIN_CCIOA4        // CCIO IO-4: Valve write (pneumatics control)
#define CYLINDER_RETRACTED_SENSOR_PIN CLEARCORE_PIN_CCIOA5        // CCIO IO-5: Cylinder retracted sensor
#define CYLINDER_EXTENDED_SENSOR_PIN CLEARCORE_PIN_CCIOA6         // CCIO IO-6: Cylinder extended sensor

//=============================================================================
// SENSOR CONSTANTS
//=============================================================================

// CCIO Board status
extern bool hasCCIO;

// Pressure sensor constants (using integer math with 100x scaling for precision)
const uint16_t MIN_SAFE_PRESSURE_SCALED = 3000;    // 30.0 PSI * 100 (minimum for valve operation - matches valve controller)
const uint16_t MAX_PRESSURE_SCALED = 8700;         // 87.0 PSI * 100 (6 bar maximum range)
const uint16_t PRESSURE_WARNING_THRESHOLD_SCALED = 3000;  // 30.0 PSI * 100 (warning threshold - same as minimum)

// Pressure monitoring intervals (milliseconds)
const unsigned long PRESSURE_MONITORING_INTERVAL_MS = 10000;  // 10 seconds for periodic pressure checks
const unsigned long CYLINDER_WARNING_INTERVAL_MS = 10000;     // 10 seconds for cylinder position warnings

//=============================================================================
// SENSOR STRUCTURES
//=============================================================================

// Generic digital sensor structure
struct DigitalSensor {
    int pin;                    // Pin number (for ClearCore) or CCIO pin
    bool isCcioPin;            // True if this is a CCIO pin, false for ClearCore pin
    bool currentState;         // Current sensor state
    bool lastState;            // Previous sensor state  
    bool stateChanged;         // Flag indicating state change
    unsigned long lastChangeTime;  // When the state last changed
    const char* name;          // Sensor name for logging
};

// Pressure sensor structure
struct PressureSensor {
    uint8_t analogPin;         // Analog input pin
    uint16_t minPressure;      // Minimum pressure in scaled units (PSI * 100)
    uint16_t maxPressure;      // Maximum pressure in scaled units (PSI * 100)
};

// Cylinder position structure
struct CylinderPosition {
    bool retracted;            // True if cylinder is retracted
    bool extended;             // True if cylinder is extended
    bool positionKnown;        // True if position is definitively known
    unsigned long lastUpdateTime;  // When position was last updated
};

//=============================================================================
// SENSOR INSTANCES
//=============================================================================

// Carriage position sensors (detect rail at pickup/dropoff locations)
extern DigitalSensor carriageSensorWC1;
extern DigitalSensor carriageSensorWC2; 
extern DigitalSensor carriageSensorWC3;        // Rail 2 carriage at WC3 pickup/dropoff
extern DigitalSensor carriageSensorRail1Handoff;
extern DigitalSensor carriageSensorRail2Handoff;

// Labware presence sensors (detect if plates are present)
extern DigitalSensor labwareSensorWC1;         // Rail 1 labware at WC1
extern DigitalSensor labwareSensorWC2;         // Rail 1 labware at WC2
extern DigitalSensor labwareSensorRail2;       // Rail 2 carriage-mounted labware sensor (CCIO)
extern DigitalSensor labwareSensorHandoff;     // Handoff area labware sensor

// Cylinder position sensors
extern DigitalSensor cylinderRetractedSensor;
extern DigitalSensor cylinderExtendedSensor;

// Pressure sensor
extern PressureSensor airPressureSensor;

// Cylinder position state
extern CylinderPosition cylinderPosition;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// Initialization functions
void initSensorSystem(bool hasCCIOBoard);
void initDigitalSensor(DigitalSensor& sensor, int pin, bool isCcioPin, const char* name);
void initPressureSensor();

// Sensor reading functions
void updateAllSensors();
void updateDigitalSensor(DigitalSensor& sensor);
void updateCylinderPosition();

// Digital sensor helper functions
bool readDigitalSensor(const DigitalSensor& sensor);
bool sensorStateChanged(const DigitalSensor& sensor);
bool sensorActivated(const DigitalSensor& sensor);    // Just went from false to true
bool sensorDeactivated(const DigitalSensor& sensor);  // Just went from true to false

// Pressure sensor functions
uint16_t readPressureVoltageScaled(const PressureSensor& sensor);
uint16_t readPressureScaled(const PressureSensor& sensor);
float getPressurePsi();
bool isPressureSufficient();
bool isPressureWarningLevel();

// Position detection functions
bool isCarriageAtWC1();
bool isCarriageAtWC2();
bool isCarriageAtWC3();
bool isCarriageAtRail1Handoff();
bool isCarriageAtRail2Handoff();

// Labware detection functions
bool isLabwarePresentAtWC1();       // Rail 1 labware at WC1
bool isLabwarePresentAtWC2();       // Rail 1 labware at WC2
bool isLabwarePresentOnRail2();     // Rail 2 carriage-mounted labware sensor
bool isLabwarePresentAtRail1Handoff();   // Rail 1 handoff area labware

// Cylinder position functions
bool isCylinderRetracted();
bool isCylinderExtended();
bool isCylinderPositionKnown();

// Status and diagnostic functions
void printAllSensorStatus();
void printCarriagePositions();
void printLabwareStatus();
void printPressureStatus();
void printCylinderStatus();

// Sensor monitoring and alerts
void checkSensorAlerts();
void logSensorChanges();

// Timeout reset functions
void resetSensorTimeouts();

#endif // SENSORS_H
