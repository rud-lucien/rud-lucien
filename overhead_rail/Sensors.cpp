#include "Sensors.h"
#include "Utils.h"
#include "LogHistory.h"
#include "ClearCore.h"

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_SENSOR_INIT_CCIO[] PROGMEM = "Initialized sensor '%s' on CCIO pin (CLEARCORE_PIN_CCIOA#)";
const char FMT_SENSOR_INIT_CLEARCORE[] PROGMEM = "Initialized sensor '%s' on ClearCore pin %d";
const char FMT_INITIAL_PRESSURE[] PROGMEM = "Initial system pressure: %.2f PSI";
const char FMT_CARRIAGE_STATUS_FULL[] PROGMEM = "WC1: %s | WC2: %s | WC3: %s | R1_Handoff: %s | R2_Handoff: %s";
const char FMT_CARRIAGE_STATUS_LIMITED[] PROGMEM = "WC1: %s | WC2: %s | WC3: %s | R1_Handoff: N/A | R2_Handoff: N/A (No CCIO)";
const char FMT_LABWARE_STATUS_FULL[] PROGMEM = "WC1: %s | WC2: %s | WC3: %s | Handoff: %s";
const char FMT_LABWARE_STATUS_LIMITED[] PROGMEM = "WC1: %s | WC2: %s | WC3: %s | Handoff: N/A (No CCIO)";
const char FMT_AIR_PRESSURE[] PROGMEM = "Air Pressure: %.2f PSI";
const char FMT_CYLINDER_KNOWN[] PROGMEM = "Cylinder: %s";
const char FMT_CYLINDER_UNKNOWN[] PROGMEM = "Cylinder: UNKNOWN (Ret:%s Ext:%s)";

//=============================================================================
// SENSOR INSTANCES
//=============================================================================

// Carriage position sensors
DigitalSensor carriageSensorWC1;
DigitalSensor carriageSensorWC2;
DigitalSensor carriageSensorWC3;
DigitalSensor carriageSensorRail1Handoff;
DigitalSensor carriageSensorRail2Handoff;

// Labware presence sensors
DigitalSensor labwareSensorWC1;
DigitalSensor labwareSensorWC2;
DigitalSensor labwareSensorWC3;
DigitalSensor labwareSensorHandoff;

// Cylinder position sensors
DigitalSensor cylinderRetractedSensor;
DigitalSensor cylinderExtendedSensor;

// Pressure sensor
PressureSensor airPressureSensor;

// Cylinder position state
CylinderPosition cylinderPosition;

// CCIO Board status
bool hasCCIO = false;

// Timing variables for alerts only
static unsigned long lastCylinderWarning = 0;

//=============================================================================
// INITIALIZATION FUNCTIONS
//=============================================================================

void initSensorSystem(bool hasCCIOBoard)
{
    Console.serialInfo(F("Initializing overhead rail sensor system..."));

    // Store CCIO status
    hasCCIO = hasCCIOBoard;

    // Initialize carriage position sensors (ClearCore main board)
    initDigitalSensor(carriageSensorWC1, CARRIAGE_SENSOR_WC1_PIN, false, "Carriage_WC1");
    initDigitalSensor(carriageSensorWC2, CARRIAGE_SENSOR_WC2_PIN, false, "Carriage_WC2");
    initDigitalSensor(carriageSensorWC3, CARRIAGE_SENSOR_WC3_PIN, false, "Carriage_WC3");

    // Initialize labware presence sensors (ClearCore main board)
    initDigitalSensor(labwareSensorWC1, LABWARE_SENSOR_WC1_PIN, false, "Labware_WC1");
    initDigitalSensor(labwareSensorWC2, LABWARE_SENSOR_WC2_PIN, false, "Labware_WC2");
    initDigitalSensor(labwareSensorWC3, LABWARE_SENSOR_WC3_PIN, false, "Labware_WC3");

    // Initialize pressure sensor (always available)
    initPressureSensor();

    // Initialize CCIO-dependent sensors
    if (!hasCCIO) {
        Console.serialError(F("No CCIO board detected - rail handoff and cylinder sensors unavailable"));
        
        // Initialize cylinder position tracking with unknown state
        cylinderPosition.retracted = false;
        cylinderPosition.extended = false;
        cylinderPosition.positionKnown = false;
        cylinderPosition.lastUpdateTime = millis();
        
        Console.serialInfo(F("Sensor system initialization complete (limited functionality)"));
        return;
    }

    Console.serialInfo(F("Initializing CCIO sensors..."));

    // Initialize CCIO sensors
    initDigitalSensor(carriageSensorRail1Handoff, CARRIAGE_SENSOR_RAIL1_HANDOFF_PIN, true, "Carriage_R1_Handoff");
    initDigitalSensor(carriageSensorRail2Handoff, CARRIAGE_SENSOR_RAIL2_HANDOFF_PIN, true, "Carriage_R2_Handoff");
    initDigitalSensor(labwareSensorHandoff, LABWARE_SENSOR_HANDOFF_PIN, true, "Labware_Handoff");
    initDigitalSensor(cylinderRetractedSensor, CYLINDER_RETRACTED_SENSOR_PIN, true, "Cylinder_Retracted");
    initDigitalSensor(cylinderExtendedSensor, CYLINDER_EXTENDED_SENSOR_PIN, true, "Cylinder_Extended");

    // Initialize cylinder position tracking
    cylinderPosition.retracted = false;
    cylinderPosition.extended = false;
    cylinderPosition.positionKnown = false;
    cylinderPosition.lastUpdateTime = millis();

    Console.serialInfo(F("Sensor system initialization complete"));
}

void initDigitalSensor(DigitalSensor& sensor, int pin, bool isCcioPin, const char* name)
{
    sensor.pin = pin;
    sensor.isCcioPin = isCcioPin;
    sensor.currentState = false;
    sensor.lastState = false;
    sensor.stateChanged = false;
    sensor.lastChangeTime = millis();
    sensor.name = name;

    // Configure pin mode for all sensor pins
    if (isCcioPin) {
        // Configure CCIO pins as inputs using pinMode
        pinMode(pin, INPUT);
    } else {
        // Configure ClearCore pins as digital inputs
        // Note: A9 is used as a digital labware sensor, so it needs pinMode()
        // A10 is handled separately as an analog pressure sensor in initPressureSensor()
        if (pin == A9) {
            pinMode(pin, INPUT);  // A9 labware sensor needs digital input mode
        } else if (pin >= 0 && pin <= 5) {  // Valid ClearCore IO pins (0-5)
            pinMode(pin, INPUT);
        }
    }

    // Read initial state
    updateDigitalSensor(sensor);
    sensor.lastState = sensor.currentState;  // Prevent initial false state change

    char msg[100];
    if (isCcioPin) {
        sprintf_P(msg, FMT_SENSOR_INIT_CCIO, name);
    } else {
        sprintf_P(msg, FMT_SENSOR_INIT_CLEARCORE, name, pin);
    }
    Console.serialInfo(msg);
}

void initPressureSensor()
{
    // Initialize the pressure sensor
    airPressureSensor.analogPin = PNEUMATICS_PRESSURE_SENSOR_PIN;
    airPressureSensor.minPressure = 0;
    airPressureSensor.maxPressure = MAX_PRESSURE_SCALED;

    // Set the resolution of the ADC for better precision
    analogReadResolution(12); // 12-bit resolution for more precise readings

    Console.serialInfo(F("Pressure sensor initialized on pin A10"));

    // Read and report the initial pressure
    float initialPressure = getPressurePsi();
    char msg[200];
    sprintf_P(msg, FMT_INITIAL_PRESSURE, initialPressure);
    Console.serialInfo(msg);

    // Check if pressure is sufficient for valve operation
    if (!isPressureSufficient()) {
        Console.serialWarning(F("System pressure below minimum threshold (21.75 PSI) - Valve operations may be unreliable"));
    }
}

//=============================================================================
// SENSOR UPDATE FUNCTIONS
//=============================================================================

void updateAllSensors()
{
    // Update all digital sensors (ClearCore main board)
    updateDigitalSensor(carriageSensorWC1);
    updateDigitalSensor(carriageSensorWC2);
    updateDigitalSensor(carriageSensorWC3);
    
    updateDigitalSensor(labwareSensorWC1);
    updateDigitalSensor(labwareSensorWC2);
    updateDigitalSensor(labwareSensorWC3);

    // Update CCIO sensors only if CCIO board is present
    if (hasCCIO) {
        updateDigitalSensor(carriageSensorRail1Handoff);
        updateDigitalSensor(carriageSensorRail2Handoff);
        updateDigitalSensor(labwareSensorHandoff);
        updateDigitalSensor(cylinderRetractedSensor);
        updateDigitalSensor(cylinderExtendedSensor);

        // Update cylinder position logic
        updateCylinderPosition();
    }
    
    // Log any sensor changes
    logSensorChanges();
    
    // Check for sensor-related alerts
    checkSensorAlerts();
}

void updateDigitalSensor(DigitalSensor& sensor)
{
    sensor.lastState = sensor.currentState;
    sensor.currentState = readDigitalSensor(sensor);
    
    if (sensor.currentState != sensor.lastState) {
        sensor.stateChanged = true;
        sensor.lastChangeTime = millis();
    } else {
        sensor.stateChanged = false;
    }
}

void updateCylinderPosition()
{
    cylinderPosition.retracted = cylinderRetractedSensor.currentState;
    cylinderPosition.extended = cylinderExtendedSensor.currentState;
    cylinderPosition.lastUpdateTime = millis();
    
    // Determine if position is definitively known
    if (cylinderPosition.retracted && !cylinderPosition.extended) {
        cylinderPosition.positionKnown = true;  // Definitely retracted
    } else if (!cylinderPosition.retracted && cylinderPosition.extended) {
        cylinderPosition.positionKnown = true;  // Definitely extended
    } else {
        cylinderPosition.positionKnown = false; // Both sensors same state - ambiguous
    }
}

//=============================================================================
// DIGITAL SENSOR HELPER FUNCTIONS
//=============================================================================

bool readDigitalSensor(const DigitalSensor& sensor)
{
    if (sensor.isCcioPin) {
        // Read from CCIO board using ClearCore pin constant
        // The CLEARCORE_PIN_CCIOA# constants work directly with digitalRead
        return digitalRead(sensor.pin);
    } else {
        // Read from ClearCore pin using digitalRead for consistency
        return digitalRead(sensor.pin);
    }
}

bool sensorStateChanged(const DigitalSensor& sensor)
{
    return sensor.stateChanged;
}

bool sensorActivated(const DigitalSensor& sensor)
{
    return sensor.stateChanged && sensor.currentState && !sensor.lastState;
}

bool sensorDeactivated(const DigitalSensor& sensor)
{
    return sensor.stateChanged && !sensor.currentState && sensor.lastState;
}

//=============================================================================
// PRESSURE SENSOR FUNCTIONS
//=============================================================================

uint16_t readPressureVoltageScaled(const PressureSensor& sensor)
{
    int analogValue = analogRead(sensor.analogPin);
    // Convert to scaled voltage: (analogValue * 1000) / 4095 gives voltage * 100
    // For 0-10V range: (analogValue * 1000) / 4095
    return (analogValue * 1000UL) / 4095;
}

uint16_t readPressureScaled(const PressureSensor& sensor)
{
    uint16_t voltageScaled = readPressureVoltageScaled(sensor);
    // Convert voltage to pressure: (voltageScaled * maxPressureScaled) / 1000
    // Since voltageScaled is voltage * 100 and maxPressureScaled is PSI * 100
    // Result is PSI * 100
    return (voltageScaled * (uint32_t)sensor.maxPressure) / 1000UL;
}

float getPressurePsi()
{
    uint16_t pressureScaled = readPressureScaled(airPressureSensor);
    return pressureScaled / 100.0f;  // Convert back to float PSI for display
}

bool isPressureSufficient()
{
    uint16_t pressureScaled = readPressureScaled(airPressureSensor);
    return pressureScaled >= MIN_SAFE_PRESSURE_SCALED;
}

bool isPressureWarningLevel()
{
    uint16_t pressureScaled = readPressureScaled(airPressureSensor);
    return pressureScaled < PRESSURE_WARNING_THRESHOLD_SCALED;
}

//=============================================================================
// POSITION DETECTION FUNCTIONS
//=============================================================================

bool isCarriageAtWC1() { return carriageSensorWC1.currentState; }
bool isCarriageAtWC2() { return carriageSensorWC2.currentState; }
bool isCarriageAtWC3() { return carriageSensorWC3.currentState; }

bool isCarriageAtRail1Handoff() { 
    if (!hasCCIO) {
        Console.serialError(F("Cannot read Rail 1 handoff sensor: CCIO board not detected"));
        return false;
    }
    return carriageSensorRail1Handoff.currentState; 
}

bool isCarriageAtRail2Handoff() { 
    if (!hasCCIO) {
        Console.serialError(F("Cannot read Rail 2 handoff sensor: CCIO board not detected"));
        return false;
    }
    return carriageSensorRail2Handoff.currentState; 
}

//=============================================================================
// LABWARE DETECTION FUNCTIONS
//=============================================================================

bool isLabwarePresentAtWC1() { return labwareSensorWC1.currentState; }
bool isLabwarePresentAtWC2() { return labwareSensorWC2.currentState; }
bool isLabwarePresentAtWC3() { return labwareSensorWC3.currentState; }

bool isLabwarePresentAtHandoff() { 
    if (!hasCCIO) {
        Console.serialError(F("Cannot read handoff labware sensor: CCIO board not detected"));
        return false;
    }
    return labwareSensorHandoff.currentState; 
}

//=============================================================================
// CYLINDER POSITION FUNCTIONS
//=============================================================================

bool isCylinderRetracted() { 
    if (!hasCCIO) {
        Console.serialError(F("Cannot read cylinder sensors: CCIO board not detected"));
        return false;
    }
    return cylinderPosition.retracted && cylinderPosition.positionKnown; 
}

bool isCylinderExtended() { 
    if (!hasCCIO) {
        Console.serialError(F("Cannot read cylinder sensors: CCIO board not detected"));
        return false;
    }
    return cylinderPosition.extended && cylinderPosition.positionKnown; 
}

bool isCylinderPositionKnown() { 
    if (!hasCCIO) {
        return false;
    }
    return cylinderPosition.positionKnown; 
}

//=============================================================================
// STATUS AND DIAGNOSTIC FUNCTIONS
//=============================================================================

void printAllSensorStatus()
{
    Console.serialInfo(F("\n===== SENSOR STATUS REPORT ====="));
    printCarriagePositions();
    printLabwareStatus();
    printCylinderStatus();
    printPressureStatus();
    Console.serialInfo(F("================================\n"));
}

void printCarriagePositions()
{
    Console.serialInfo(F("--- Carriage Position Sensors ---"));
    char msg[250];
    
    if (hasCCIO) {
        sprintf_P(msg, FMT_CARRIAGE_STATUS_FULL,
                carriageSensorWC1.currentState ? "PRESENT" : "absent",
                carriageSensorWC2.currentState ? "PRESENT" : "absent", 
                carriageSensorWC3.currentState ? "PRESENT" : "absent",
                carriageSensorRail1Handoff.currentState ? "PRESENT" : "absent",
                carriageSensorRail2Handoff.currentState ? "PRESENT" : "absent");
    } else {
        sprintf_P(msg, FMT_CARRIAGE_STATUS_LIMITED,
                carriageSensorWC1.currentState ? "PRESENT" : "absent",
                carriageSensorWC2.currentState ? "PRESENT" : "absent", 
                carriageSensorWC3.currentState ? "PRESENT" : "absent");
    }
    Console.serialInfo(msg);
}

void printLabwareStatus()
{
    Console.serialInfo(F("--- Labware Presence Sensors ---"));
    char msg[200];
    
    if (hasCCIO) {
        sprintf_P(msg, FMT_LABWARE_STATUS_FULL,
                labwareSensorWC1.currentState ? "DETECTED" : "none",
                labwareSensorWC2.currentState ? "DETECTED" : "none",
                labwareSensorWC3.currentState ? "DETECTED" : "none", 
                labwareSensorHandoff.currentState ? "DETECTED" : "none");
    } else {
        sprintf_P(msg, FMT_LABWARE_STATUS_LIMITED,
                labwareSensorWC1.currentState ? "DETECTED" : "none",
                labwareSensorWC2.currentState ? "DETECTED" : "none",
                labwareSensorWC3.currentState ? "DETECTED" : "none");
    }
    Console.serialInfo(msg);
}

void printPressureStatus()
{
    char msg[200];
    float currentPressure = getPressurePsi();
    sprintf_P(msg, FMT_AIR_PRESSURE, currentPressure);
    Console.serialInfo(msg);

    if (!isPressureSufficient()) {
        Console.serialWarning(F("Pressure below minimum threshold for safe valve operation (21.75 PSI)"));
    }
}

void printCylinderStatus()
{
    Console.serialInfo(F("--- Cylinder Position ---"));
    char msg[150];
    
    if (!hasCCIO) {
        Console.serialInfo(F("Cylinder: N/A (No CCIO board detected)"));
        return;
    }
    
    if (cylinderPosition.positionKnown) {
        sprintf_P(msg, FMT_CYLINDER_KNOWN, 
                cylinderPosition.retracted ? "RETRACTED" : "EXTENDED");
        Console.serialInfo(msg);
    } else {
        sprintf_P(msg, FMT_CYLINDER_UNKNOWN, 
                cylinderPosition.retracted ? "Y" : "N",
                cylinderPosition.extended ? "Y" : "N");
        Console.serialWarning(msg);
    }
}

//=============================================================================
// MONITORING AND ALERT FUNCTIONS
//=============================================================================

void checkSensorAlerts()
{
    // Check for cylinder position issues (only if CCIO is available)
    if (hasCCIO && !cylinderPosition.positionKnown) {
        if (waitTimeReached(millis(), lastCylinderWarning, 10000)) {  // Warn every 10 seconds
            Console.serialWarning(F("Cylinder position ambiguous - check sensors"));
            lastCylinderWarning = millis();
        }
    }
}

void logSensorChanges()
{
    // Log carriage position changes (ClearCore sensors)
    if (sensorActivated(carriageSensorWC1)) {
        opLogHistory.addEntry("Carriage arrived at WC1", LogEntry::INFO);
    }
    if (sensorActivated(carriageSensorWC2)) {
        opLogHistory.addEntry("Carriage arrived at WC2", LogEntry::INFO);
    }
    if (sensorActivated(carriageSensorWC3)) {
        opLogHistory.addEntry("Carriage arrived at WC3", LogEntry::INFO);
    }
    
    // Log CCIO carriage position changes (only if CCIO is available)
    if (hasCCIO) {
        if (sensorActivated(carriageSensorRail1Handoff)) {
            opLogHistory.addEntry("Carriage arrived at Rail 1 handoff", LogEntry::INFO);
        }
        if (sensorActivated(carriageSensorRail2Handoff)) {
            opLogHistory.addEntry("Carriage arrived at Rail 2 handoff", LogEntry::INFO);
        }
    }
    
    // Log labware detection changes (ClearCore sensors)
    if (sensorActivated(labwareSensorWC1)) {
        opLogHistory.addEntry("Labware detected at WC1", LogEntry::INFO);
    }
    if (sensorDeactivated(labwareSensorWC1)) {
        opLogHistory.addEntry("Labware removed from WC1", LogEntry::INFO);
    }
    if (sensorActivated(labwareSensorWC2)) {
        opLogHistory.addEntry("Labware detected at WC2", LogEntry::INFO);
    }
    if (sensorDeactivated(labwareSensorWC2)) {
        opLogHistory.addEntry("Labware removed from WC2", LogEntry::INFO);
    }
    if (sensorActivated(labwareSensorWC3)) {
        opLogHistory.addEntry("Labware detected at WC3", LogEntry::INFO);
    }
    if (sensorDeactivated(labwareSensorWC3)) {
        opLogHistory.addEntry("Labware removed from WC3", LogEntry::INFO);
    }
    
    // Log CCIO labware and cylinder changes (only if CCIO is available)
    if (hasCCIO) {
        if (sensorActivated(labwareSensorHandoff)) {
            opLogHistory.addEntry("Labware detected at handoff", LogEntry::INFO);
        }
        if (sensorDeactivated(labwareSensorHandoff)) {
            opLogHistory.addEntry("Labware removed from handoff", LogEntry::INFO);
        }
        
        // Log cylinder position changes
        if (sensorActivated(cylinderRetractedSensor)) {
            opLogHistory.addEntry("Cylinder retracted", LogEntry::INFO);
        }
        if (sensorActivated(cylinderExtendedSensor)) {
            opLogHistory.addEntry("Cylinder extended", LogEntry::INFO);
        }
    }
}
