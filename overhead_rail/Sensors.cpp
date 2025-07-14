#include "Sensors.h"
#include "Utils.h"
#include "LogHistory.h"
#include "ClearCore.h"

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Streamlined format strings for operator clarity
const char FMT_SENSOR_INIT_SUMMARY[] PROGMEM = "Sensor system: %d sensors initialized (%s)";
const char FMT_SENSOR_STATUS_SUMMARY[] PROGMEM = "Sensors - Carriages: R1-Input=%s R1-Output=%s WC3=%s Handoff=%s | Labware: WC2-R1=%s Handoff-R1=%s Rail2=%s | Rail1-Home: Rail=%s Carriage=%s | Cylinder: %s | Pressure: %.1f PSI%s";
const char FMT_SENSOR_STATUS_LIMITED[] PROGMEM = "Sensors - Carriages: R1-Input=%s R1-Output=%s WC3=%s | Labware: WC2-R1=%s | Rail1-Home: Rail=%s Carriage=%s | Pressure: %.1f PSI%s [Limited: No CCIO]";

// Legacy format strings (preserved for detailed status functions)
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
DigitalSensor labwareSensorWC1;         // Rail 1 labware at WC1
DigitalSensor labwareSensorWC2;         // Rail 1 labware at WC2
DigitalSensor labwareSensorRail2;       // Rail 2 carriage-mounted labware sensor (CCIO)
DigitalSensor labwareSensorHandoff;     // Handoff area labware sensor

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
    Console.serialInfo(F("Initializing sensor system with pressure monitoring..."));

    // Store CCIO status
    hasCCIO = hasCCIOBoard;

    // Initialize carriage position sensors (ClearCore main board) - silently for reduced noise
    initDigitalSensor(carriageSensorWC1, CARRIAGE_SENSOR_WC1_PIN, false, "Carriage_WC1");
    initDigitalSensor(carriageSensorWC2, CARRIAGE_SENSOR_WC2_PIN, false, "Carriage_WC2");
    initDigitalSensor(carriageSensorWC3, CARRIAGE_SENSOR_WC3_PIN, false, "Carriage_WC3");

    // Initialize labware presence sensors (ClearCore main board) - silently
    initDigitalSensor(labwareSensorWC1, LABWARE_SENSOR_WC1_PIN, false, "Labware_WC1");
    initDigitalSensor(labwareSensorWC2, LABWARE_SENSOR_WC2_PIN, false, "Labware_WC2");

    // Initialize pressure sensor (always available) - silently
    initPressureSensor();

    // Initialize CCIO-dependent sensors
    if (!hasCCIO) {
        Console.serialError(F("No CCIO board detected - rail handoff and cylinder sensors unavailable"));
        
        // Initialize cylinder position tracking with unknown state
        cylinderPosition.retracted = false;
        cylinderPosition.extended = false;
        cylinderPosition.positionKnown = false;
        cylinderPosition.lastUpdateTime = millis();
        
        // Consolidated initialization summary
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_SENSOR_INIT_SUMMARY, 6, "Limited: 5 main sensors + pressure, no CCIO");
        Console.serialInfo(msg);
        return;
    }

    // Initialize CCIO sensors (silently)
    initDigitalSensor(carriageSensorRail1Handoff, CARRIAGE_SENSOR_RAIL1_HANDOFF_PIN, true, "Carriage_R1_Handoff");
    initDigitalSensor(carriageSensorRail2Handoff, CARRIAGE_SENSOR_RAIL2_HANDOFF_PIN, true, "Carriage_R2_Handoff");
    initDigitalSensor(labwareSensorRail2, LABWARE_SENSOR_RAIL2_PIN, true, "Labware_Rail2");
    initDigitalSensor(labwareSensorHandoff, LABWARE_SENSOR_HANDOFF_PIN, true, "Labware_Handoff");
    initDigitalSensor(cylinderRetractedSensor, CYLINDER_RETRACTED_SENSOR_PIN, true, "Cylinder_Retracted");
    initDigitalSensor(cylinderExtendedSensor, CYLINDER_EXTENDED_SENSOR_PIN, true, "Cylinder_Extended");

    // Initialize cylinder position tracking
    cylinderPosition.retracted = false;
    cylinderPosition.extended = false;
    cylinderPosition.positionKnown = false;
    cylinderPosition.lastUpdateTime = millis();

    // Consolidated initialization summary
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_SENSOR_INIT_SUMMARY, 12, "Full system: 11 sensors + pressure");
    Console.serialInfo(msg);
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

    // Individual sensor initialization messages removed for cleaner startup
    // Sensor details are tracked internally and available via status commands
}

void initPressureSensor()
{
    // Initialize the pressure sensor
    airPressureSensor.analogPin = PNEUMATICS_PRESSURE_SENSOR_PIN;
    airPressureSensor.minPressure = 0;
    airPressureSensor.maxPressure = MAX_PRESSURE_SCALED;

    // Set the resolution of the ADC for better precision
    analogReadResolution(12); // 12-bit resolution for more precise readings

    // Read initial pressure
    float initialPressure = getPressurePsi();
    
    // Check if pressure is sufficient for valve operation and warn if needed
    if (!isPressureSufficient()) {
        Console.serialWarning(F("System pressure below minimum threshold (21.75 PSI) - Valve operations may be unreliable"));
    }

    // Individual pressure sensor initialization messages removed for cleaner startup
    // Pressure readings are tracked and available via status commands
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

    // Update CCIO sensors only if CCIO board is present
    if (hasCCIO) {
        updateDigitalSensor(carriageSensorRail1Handoff);
        updateDigitalSensor(carriageSensorRail2Handoff);
        updateDigitalSensor(labwareSensorRail2);
        updateDigitalSensor(labwareSensorHandoff);
        updateDigitalSensor(cylinderRetractedSensor);
        updateDigitalSensor(cylinderExtendedSensor);

        // Update cylinder position logic
        updateCylinderPosition();
    }

    // Check for any sensor-related alerts
    logSensorChanges();
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
bool isLabwarePresentOnRail2() { 
    if (!hasCCIO) {
        Console.serialError(F("Cannot read Rail 2 labware sensor: CCIO board not detected"));
        return false;
    }
    return labwareSensorRail2.currentState; 
}

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
    char msg[LARGE_MSG_SIZE];
    
    if (hasCCIO) {
        sprintf_P(msg, FMT_SENSOR_STATUS_SUMMARY,
                carriageSensorWC1.currentState ? "PRESENT" : "absent",
                carriageSensorWC2.currentState ? "PRESENT" : "absent", 
                carriageSensorWC3.currentState ? "PRESENT" : "absent",
                carriageSensorRail2Handoff.currentState ? "PRESENT" : "absent",
                labwareSensorWC2.currentState ? "DETECTED" : "none",
                labwareSensorHandoff.currentState ? "DETECTED" : "none",
                labwareSensorRail2.currentState ? "DETECTED" : "none",
                carriageSensorRail1Handoff.currentState ? "PRESENT" : "absent",
                carriageSensorWC1.currentState ? "PRESENT" : "absent",
                cylinderPosition.positionKnown ? (cylinderPosition.retracted ? "RETRACTED" : "EXTENDED") : "UNKNOWN",
                getPressurePsi(),
                isPressureWarningLevel() ? " [LOW]" : "");
    } else {
        sprintf_P(msg, FMT_SENSOR_STATUS_LIMITED,
                carriageSensorWC1.currentState ? "PRESENT" : "absent",
                carriageSensorWC2.currentState ? "PRESENT" : "absent", 
                carriageSensorWC3.currentState ? "PRESENT" : "absent",
                labwareSensorWC2.currentState ? "DETECTED" : "none",
                carriageSensorRail1Handoff.currentState ? "PRESENT" : "absent",
                carriageSensorWC1.currentState ? "PRESENT" : "absent",
                getPressurePsi(),
                isPressureWarningLevel() ? " [LOW]" : "");
    }
    Console.serialInfo(msg);
}

void printCarriagePositions()
{
    // Simplified function - use printAllSensorStatus() for complete status
    printAllSensorStatus();
}

void printLabwareStatus()
{
    // Simplified function - use printAllSensorStatus() for complete status
    printAllSensorStatus();
}

void printPressureStatus()
{
    char msg[ALERT_MSG_SIZE];
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
    char msg[MEDIUM_MSG_SIZE];
    
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
        if (waitTimeReached(millis(), lastCylinderWarning, CYLINDER_WARNING_INTERVAL_MS)) {
            Console.serialWarning(F("Cylinder position ambiguous - check sensors"));
            lastCylinderWarning = millis();
        }
    }
}

void logSensorChanges()
{
    // Log Rail 1 carriage position changes
    if (sensorActivated(carriageSensorWC1)) {
        opLogHistory.addEntry("Carriage arrived at WC1", LogEntry::INFO);
    }
    if (sensorActivated(carriageSensorWC2)) {
        opLogHistory.addEntry("Carriage arrived at WC2", LogEntry::INFO);
    }
    if (sensorActivated(carriageSensorRail1Handoff)) {
        opLogHistory.addEntry("Carriage arrived at Rail 1 handoff", LogEntry::INFO);
    }
    
    // Log Rail 2 carriage position changes
    if (sensorActivated(carriageSensorWC3)) {
        opLogHistory.addEntry("Carriage arrived at WC3", LogEntry::INFO);
    }
    if (sensorActivated(carriageSensorRail2Handoff)) {
        opLogHistory.addEntry("Carriage arrived at Rail 2 handoff", LogEntry::INFO);
    }
    
    // Log Rail 1 labware detection changes
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
    if (sensorActivated(labwareSensorHandoff)) {
        opLogHistory.addEntry("Labware detected at handoff", LogEntry::INFO);
    }
    if (sensorDeactivated(labwareSensorHandoff)) {
        opLogHistory.addEntry("Labware removed from handoff", LogEntry::INFO);
    }
    
    // Log Rail 2 labware detection changes (CCIO sensor)
    if (hasCCIO) {
        if (sensorActivated(labwareSensorRail2)) {
            opLogHistory.addEntry("Labware detected on Rail 2 carriage", LogEntry::INFO);
        }
        if (sensorDeactivated(labwareSensorRail2)) {
            opLogHistory.addEntry("Labware removed from Rail 2 carriage", LogEntry::INFO);
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
