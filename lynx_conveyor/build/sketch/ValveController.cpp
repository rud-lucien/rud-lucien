#line 1 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/ValveController.cpp"
#include "ValveController.h"
#include "OutputManager.h"
#include "Utils.h"

// ----------------- Global variables -----------------

// Valve instances
DoubleSolenoidValve tray1Valve;
DoubleSolenoidValve tray2Valve;
DoubleSolenoidValve tray3Valve;
DoubleSolenoidValve shuttleValve;

// Sensor instances
CylinderSensor tray1CylinderSensor;
CylinderSensor tray2CylinderSensor;
CylinderSensor tray3CylinderSensor;
CylinderSensor shuttleCylinderSensor;

// Arrays for batch operations
DoubleSolenoidValve *allValves[4] = {&tray1Valve, &tray2Valve, &tray3Valve, &shuttleValve};
const int valveCount = 4;
const char *valveNames[4] = {"Tray 1", "Tray 2", "Tray 3", "Shuttle"};

// New tray detection sensors
CylinderSensor tray1DetectSensor;
CylinderSensor tray2DetectSensor;
CylinderSensor tray3DetectSensor;

// Update your allSensors array to include these new sensors if needed
CylinderSensor *allCylinderSensors[4] = {
    &tray1CylinderSensor, &tray2CylinderSensor,
    &tray3CylinderSensor, &shuttleCylinderSensor};
const int cylinderSensorCount = 4;

// CCIO Board status
bool hasCCIO = false;

PressureSensor airPressureSensor;
const float MIN_SAFE_PRESSURE = 21.75f; // Minimum pressure in PSI for valve operation (1.5 bar)
const float MAX_PRESSURE = 87.0f;       // Maximum pressure range (87 PSI)

// ----------------- Initialization functions -----------------

// Rename and refactor the initialization functions
void initSensorSystem()
{
    // Initialize all sensors on the main ClearCore board

    // 1. Cylinder position sensors
    sensorInit(tray1CylinderSensor, TRAY_1_CYLINDER_SENSOR_PIN);
    sensorInit(tray2CylinderSensor, TRAY_2_CYLINDER_SENSOR_PIN);
    sensorInit(tray3CylinderSensor, TRAY_3_CYLINDER_SENSOR_PIN);
    sensorInit(shuttleCylinderSensor, SHUTTLE_CYLINDER_SENSOR_PIN);

    // 2. Tray detection sensors
    sensorInit(tray1DetectSensor, TRAY_1_DETECT_PIN);
    sensorInit(tray2DetectSensor, TRAY_2_DETECT_PIN);
    sensorInit(tray3DetectSensor, TRAY_3_DETECT_PIN);

    // Initialize the pressure sensor
    airPressureSensor.analogPin = PRESSURE_SENSOR_PIN;
    airPressureSensor.minPressure = 0.0f;
    airPressureSensor.maxPressure = MAX_PRESSURE;

    Console.serialInfo(F("Sensor system initialized"));
}

void initPressureSensor() {
    // Initialize the pressure sensor
    airPressureSensor.analogPin = PRESSURE_SENSOR_PIN;
    airPressureSensor.minPressure = 0.0f;
    airPressureSensor.maxPressure = MAX_PRESSURE;
    
    // Set the resolution of the ADC for better precision
    analogReadResolution(12); // 12-bit resolution for more precise readings
    
    Console.serialInfo(F("Pressure sensor initialized on pin A11"));
    
    // Read and report the initial pressure
    float initialPressure = readPressure(airPressureSensor);
    char msg[200];
    sprintf(msg, "Initial system pressure: %.2f PSI", initialPressure);
    Console.serialInfo(msg);
    
    // Check if pressure is sufficient for valve operation
    if (!isPressureSufficient()) {
        Console.serialWarning(F("System pressure below minimum threshold (21.75 PSI) - Valve operations may be unreliable"));
    }
}

// Implement the pressure reading functions:
float readPressureVoltage(const PressureSensor &sensor) {
    int analogValue = analogRead(sensor.analogPin);
    return (analogValue / 4095.0) * 10.0;  // For 12-bit resolution (4095)
}

float readPressure(const PressureSensor &sensor) {
    float voltage = readPressureVoltage(sensor);
    return (voltage / 10.0) * sensor.maxPressure;
}

float getPressurePsi() {
    return readPressure(airPressureSensor);
}

bool isPressureSufficient() {
    float currentPressure = readPressure(airPressureSensor);
    return currentPressure >= MIN_SAFE_PRESSURE;
}

void printPressureStatus() {
    char msg[200];
    float currentPressure = readPressure(airPressureSensor);
    sprintf(msg, "Air Pressure: %.2f PSI", currentPressure);
    Console.serialInfo(msg);
    
    if (currentPressure < MIN_SAFE_PRESSURE) {
        Console.serialWarning(F("Pressure below minimum threshold for safe valve operation (21.75 PSI)"));
    }
}

void initValveSystem(bool hasCCIOBoard)
{
    // Store CCIO status
    hasCCIO = hasCCIOBoard;

    // Initialize the pressure sensor regardless of CCIO status
    initPressureSensor();


    if (!hasCCIO)
    {
        Console.serialError(F("No CCIO board detected - valve control unavailable"));
        return;
    }

    Console.serialInfo(F("Initializing valves with CCIO board..."));

    // Configure all valve pins on the CCIO board
    pinMode(TRAY_1_LOCK_PIN, OUTPUT);
    pinMode(TRAY_1_UNLOCK_PIN, OUTPUT);

    pinMode(TRAY_2_LOCK_PIN, OUTPUT);
    pinMode(TRAY_2_UNLOCK_PIN, OUTPUT);

    pinMode(TRAY_3_LOCK_PIN, OUTPUT);
    pinMode(TRAY_3_UNLOCK_PIN, OUTPUT);

    pinMode(SHUTTLE_LOCK_PIN, OUTPUT);
    pinMode(SHUTTLE_UNLOCK_PIN, OUTPUT);

    // Set up all valve pin configurations
    tray1Valve.unlockPin = TRAY_1_UNLOCK_PIN;
    tray1Valve.lockPin = TRAY_1_LOCK_PIN;

    tray2Valve.unlockPin = TRAY_2_UNLOCK_PIN;
    tray2Valve.lockPin = TRAY_2_LOCK_PIN;

    tray3Valve.unlockPin = TRAY_3_UNLOCK_PIN;
    tray3Valve.lockPin = TRAY_3_LOCK_PIN;

    shuttleValve.unlockPin = SHUTTLE_UNLOCK_PIN;
    shuttleValve.lockPin = SHUTTLE_LOCK_PIN;

    // Initialize all valves
    valveInit(tray1Valve);
    valveInit(tray2Valve);
    valveInit(tray3Valve);
    valveInit(shuttleValve);

    Console.serialInfo(F("Valve system initialized"));
}

// ----------------- Low-level hardware functions -----------------

void pulsePin(int pin, unsigned long duration)
{
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

int getActivationPin(const DoubleSolenoidValve &valve, ValvePosition target)
{
    return (target == VALVE_POSITION_UNLOCK) ? valve.unlockPin : valve.lockPin;
}

// ----------------- Valve operations -----------------

void valveInit(DoubleSolenoidValve &valve)
{

    // Force physical position to unlocked
    pulsePin(valve.unlockPin, PULSE_DURATION);

    // Set software state
    valve.position = VALVE_POSITION_UNLOCK;
}

void valveSetPosition(DoubleSolenoidValve &valve, ValvePosition target)
{
    // Don't do anything if already in the requested position
    if (valve.position == target)
    {
        return;
    }

    // Check if pressure is sufficient before actuating the valve
    if (!isPressureSufficient()) {
        char msg[200];
        sprintf(msg, "Cannot actuate valve - System pressure too low. Current: %.2f PSI, Minimum required: %.2f PSI", 
                readPressure(airPressureSensor), MIN_SAFE_PRESSURE);
        Console.serialError(msg);
        return;
    }

    // Get the appropriate pin and pulse it
    int pinToActivate = getActivationPin(valve, target);
    pulsePin(pinToActivate, PULSE_DURATION);

    // Update the position state
    valve.position = target;
}

ValvePosition valveGetPosition(const DoubleSolenoidValve &valve)
{
    return valve.position;
}

// ----------------- Higher-order valve operations -----------------

void withValve(DoubleSolenoidValve &valve, void (*operation)(DoubleSolenoidValve &))
{
    operation(valve);
}

// WARNING: These functions bypass sensor verification and safety checks
// Use only for testing, diagnostics, or emergency recovery

void unsafeUnlockValve(DoubleSolenoidValve &valve)
{
    Console.serialWarning(F("Using unsafe unlock - no sensor verification"));
    valveSetPosition(valve, VALVE_POSITION_UNLOCK);
}

void unsafeLockValve(DoubleSolenoidValve &valve)
{
    Console.serialWarning(F("Using unsafe lock - no sensor verification"));
    valveSetPosition(valve, VALVE_POSITION_LOCK);
}

// ----------------- Sensor operations -----------------

void sensorInit(CylinderSensor &sensor, int pin)
{
    sensor.pin = pin;
    pinMode(pin, INPUT);
    sensor.lastState = digitalRead(pin);
}

bool sensorRead(CylinderSensor &sensor)
{
    bool currentState = digitalRead(sensor.pin);
    sensor.lastState = currentState;
    return currentState;
}

// ----------------- Status reporting -----------------

void printValveStatus(const DoubleSolenoidValve &valve, const char *valveName)
{
    char msg[200];
    sprintf(msg, " %s: %s", valveName, 
            valve.position == VALVE_POSITION_UNLOCK ? "Unlocked" : "Locked");
    Console.serialDiagnostic(msg);
}

void printSensorStatus(const CylinderSensor &sensor, const char *sensorName)
{
    char msg[200];
    bool sensorState = sensorRead(const_cast<CylinderSensor &>(sensor));

    // IMPORTANT: TRUE means UNLOCKED, FALSE means LOCKED
    sprintf(msg, " %s Sensor: %s", sensorName, 
            sensorState ? "ACTIVATED (UNLOCKED)" : "NOT ACTIVATED (LOCKED)");
    Console.serialDiagnostic(msg);
}

// ----------------- Batch operations -----------------

void withAllValves(void (*operation)(DoubleSolenoidValve &))
{
    // Only perform operations if CCIO is present since all valves are on CCIO now
    if (!hasCCIO)
    {
        Console.serialError(F("Cannot operate valves: CCIO-8 board not initialized"));
        return;
    }

    for (int i = 0; i < valveCount; i++)
    {
        withValve(*allValves[i], operation);
    }
}

void printAllValveStatus()
{
    Console.serialDiagnostic(F(" Current valve positions:"));

    // First print pressure status
    char msg[200];
    float currentPressure = readPressure(airPressureSensor);
    sprintf(msg, " System Pressure: %.2f PSI %s", currentPressure,
            currentPressure < MIN_SAFE_PRESSURE ? "(INSUFFICIENT)" : "(OK)");
    Console.serialDiagnostic(msg);

    for (int i = 0; i < valveCount; i++)
    {
        // Skip shuttle valve if no CCIO board
        if (i == 3 && !hasCCIO)
            continue;
        printValveStatus(*allValves[i], valveNames[i]);
    }
}

void printAllSensorStatus()
{
    Console.serialDiagnostic(F(" Current sensor readings:"));
    for (int i = 0; i < cylinderSensorCount; i++)
    {
        printSensorStatus(*allCylinderSensors[i], valveNames[i]);
    }
}

// ----------------- Advanced operations with sensor feedback -----------------

bool waitForSensor(CylinderSensor &sensor, bool expectedState, unsigned long timeoutMs)
{
    unsigned long startTime = millis();
    while (sensorRead(sensor) != expectedState)
    {
        // Replace direct subtraction with timeoutElapsed helper
        if (timeoutElapsed(millis(), startTime, timeoutMs))
        {
            char msg[200];
            sprintf(msg, "Sensor timeout: waited %lu ms for expected state", timeoutMs);
            Console.serialError(msg);
            return false; // Timeout occurred
        }
        delay(10); // Short delay to prevent excessive CPU usage
    }
    return true; // Sensor reached expected state
}

bool safeValveOperation(DoubleSolenoidValve &valve, CylinderSensor &sensor,
                        ValvePosition targetPosition, unsigned long timeoutMs)
{
    // Determine expected sensor state based on valve position
    bool expectedSensorState = (targetPosition == VALVE_POSITION_UNLOCK);

    // Set the valve position
    valveSetPosition(valve, targetPosition);

    // Wait for sensor to confirm the operation
    bool success = waitForSensor(sensor, expectedSensorState, timeoutMs);

    // If operation failed, record it with valve type and position info
    if (!success) {
        // Determine valve type and position
        const char* valveType = "unknown";
        int valvePosition = 0;
        
        // Compare valve address to determine which valve it is
        if (&valve == getTray1Valve()) {
            valveType = "tray";
            valvePosition = 1;
        } 
        else if (&valve == getTray2Valve()) {
            valveType = "tray";
            valvePosition = 2;
        }
        else if (&valve == getTray3Valve()) {
            valveType = "tray";
            valvePosition = 3;
        }
        else if (&valve == getShuttleValve()) {
            valveType = "shuttle";
            valvePosition = 0;
        }
        
        // Record the failure based on operation type
        if (targetPosition == VALVE_POSITION_LOCK) {
            // Call the function to record a lock failure
            lastLockOperationFailed = true;
            lastLockFailureDetails = F("Failed to lock ");
            lastLockFailureDetails += valveType;
            lastLockFailureDetails += F(" at position ");
            lastLockFailureDetails += String(valvePosition);
            lastLockFailureDetails += F(" - sensor didn't confirm");
            lockFailureTimestamp = millis();  // Record the timestamp
        } else {
            // Call the function to record an unlock failure
            lastUnlockOperationFailed = true;
            lastUnlockFailureDetails = F("Failed to unlock ");
            lastUnlockFailureDetails += valveType;
            lastUnlockFailureDetails += F(" at position ");
            lastUnlockFailureDetails += String(valvePosition);
            lastUnlockFailureDetails += F(" - sensor didn't confirm");
            unlockFailureTimestamp = millis();  // Record the timestamp
        }
        
        char msg[200];
        sprintf(msg, "Valve operation failed: %s", 
                targetPosition == VALVE_POSITION_LOCK ? 
                lastLockFailureDetails.c_str() : lastUnlockFailureDetails.c_str());
        Console.serialError(msg);
    }
    
    return success;
}

// ----------------- Convenience functions -----------------

// 2. Add safe unlock all valves function
bool safeUnlockAllValves(unsigned long timeoutMs)
{
    Console.serialInfo(F("Safely unlocking all valves with sensor verification..."));
    bool allSucceeded = true;

    if (!safeValveOperation(*getTray1Valve(), *getTray1Sensor(), VALVE_POSITION_UNLOCK, timeoutMs))
    {
        Console.serialError(F("Failed to unlock Tray 1 valve - sensor did not confirm"));
        allSucceeded = false;
    }

    if (!safeValveOperation(*getTray2Valve(), *getTray2Sensor(), VALVE_POSITION_UNLOCK, timeoutMs))
    {
        Console.serialError(F("Failed to unlock Tray 2 valve - sensor did not confirm"));
        allSucceeded = false;
    }

    if (!safeValveOperation(*getTray3Valve(), *getTray3Sensor(), VALVE_POSITION_UNLOCK, timeoutMs))
    {
        Console.serialError(F("Failed to unlock Tray 3 valve - sensor did not confirm"));
        allSucceeded = false;
    }

    if (!safeValveOperation(*getShuttleValve(), *getShuttleSensor(), VALVE_POSITION_UNLOCK, timeoutMs))
    {
        Console.serialError(F("Failed to unlock Shuttle valve - sensor did not confirm"));
        allSucceeded = false;
    }

    return allSucceeded;
}



// Create an array for tray detection sensors for easier batch operations
CylinderSensor *allTrayDetectSensors[3] = {
    &tray1DetectSensor, &tray2DetectSensor, &tray3DetectSensor};
const int trayDetectSensorCount = 3;

void printTrayDetectionStatus()
{
    Console.serialDiagnostic(F(" Tray Detection Status:"));
    char msg[200];
    for (int i = 0; i < trayDetectSensorCount; i++)
    {
        bool trayDetected = sensorRead(*allTrayDetectSensors[i]);
        sprintf(msg, "  Tray %d: %s", i + 1, trayDetected ? "DETECTED" : "Not Present");
        Console.serialDiagnostic(msg);
    }
}

// Helper function to get a reference to the shuttle valve
DoubleSolenoidValve *getShuttleValve()
{
    if (!hasCCIO)
    {
        Console.serialError(F("Cannot access shuttle valve: CCIO board not detected"));
        return NULL;
    }
    // Shuttle valve is the fourth valve in the allValves array (index 3)
    return &shuttleValve;
}

// Helper function to get a reference to tray 1 valve
DoubleSolenoidValve *getTray1Valve()
{
    if (!hasCCIO)
    {
        Console.serialError(F("Cannot access tray 1 valve: CCIO board not detected"));
        return NULL;
    }
    return &tray1Valve;
}

// Helper function to get a reference to tray 2 valve
DoubleSolenoidValve *getTray2Valve()
{
    if (!hasCCIO)
    {
        Console.serialError(F("Cannot access tray 2 valve: CCIO board not detected"));
        return NULL;
    }
    return &tray2Valve;
}

// Helper function to get a reference to tray 3 valve
DoubleSolenoidValve *getTray3Valve()
{
    if (!hasCCIO)
    {
        Console.serialError(F("Cannot access tray 3 valve: CCIO board not detected"));
        return NULL;
    }
    return &tray3Valve;
}

// Helper function to get a reference to tray 1 cylinder sensor
CylinderSensor *getTray1Sensor()
{
    return &tray1CylinderSensor;
}

// Helper function to get a reference to tray 2 cylinder sensor
CylinderSensor *getTray2Sensor()
{
    return &tray2CylinderSensor;
}

// Helper function to get a reference to tray 3 cylinder sensor
CylinderSensor *getTray3Sensor()
{
    return &tray3CylinderSensor;
}

// Helper function to get a reference to shuttle cylinder sensor
CylinderSensor *getShuttleSensor()
{
    return &shuttleCylinderSensor;
}

// Helper function to get a reference to tray 1 detection sensor
CylinderSensor *getTray1DetectionSensor()
{
    return &tray1DetectSensor;
}

// Helper function to get a reference to tray 2 detection sensor
CylinderSensor *getTray2DetectionSensor()
{
    return &tray2DetectSensor;
}

// Helper function to get a reference to tray 3 detection sensor
CylinderSensor *getTray3DetectionSensor()
{
    return &tray3DetectSensor;
}