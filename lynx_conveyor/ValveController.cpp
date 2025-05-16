#include "ValveController.h"

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
DoubleSolenoidValve* allValves[4] = {&tray1Valve, &tray2Valve, &tray3Valve, &shuttleValve};
const int valveCount = 4;
const char* valveNames[4] = {"Tray 1", "Tray 2", "Tray 3", "Shuttle"};

// New tray detection sensors
CylinderSensor tray1DetectSensor;
CylinderSensor tray2DetectSensor;
CylinderSensor tray3DetectSensor;

// Update your allSensors array to include these new sensors if needed
CylinderSensor* allCylinderSensors[4] = {
    &tray1CylinderSensor, &tray2CylinderSensor, 
    &tray3CylinderSensor, &shuttleCylinderSensor
};
const int cylinderSensorCount = 4;

// CCIO Board status
bool hasCCIO = false;

// ----------------- Initialization functions -----------------

// Rename and refactor the initialization functions
void initSensorSystem() {
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
    
    Serial.println(F("[MESSAGE] Sensor system initialized"));
}

void initValveSystem(bool hasCCIOBoard) {
    // Store CCIO status
    hasCCIO = hasCCIOBoard;
    
    if (!hasCCIO) {
        Serial.println(F("[ERROR] No CCIO board detected - valve control unavailable"));
        return;
    }
    
    Serial.println(F("[MESSAGE] Initializing valves with CCIO board..."));
    
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
    
    Serial.println(F("[MESSAGE] Valve system initialized"));
}

// ----------------- Low-level hardware functions -----------------

void pulsePin(int pin, unsigned long duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

int getActivationPin(const DoubleSolenoidValve &valve, ValvePosition target) {
    return (target == VALVE_POSITION_UNLOCK) ? valve.unlockPin : valve.lockPin;
}

// ----------------- Valve operations -----------------

void valveInit(DoubleSolenoidValve &valve) {
    
    // Force physical position to unlocked
    pulsePin(valve.unlockPin, PULSE_DURATION);
    
    // Set software state
    valve.position = VALVE_POSITION_UNLOCK;
}

void valveSetPosition(DoubleSolenoidValve &valve, ValvePosition target) {
    // Don't do anything if already in the requested position
    if (valve.position == target) {
        return;
    }
    
    // Get the appropriate pin and pulse it
    int pinToActivate = getActivationPin(valve, target);
    pulsePin(pinToActivate, PULSE_DURATION);
    
    // Update the position state
    valve.position = target;
}

void valveDeactivate(DoubleSolenoidValve &valve) {
    digitalWrite(valve.unlockPin, LOW);
    digitalWrite(valve.lockPin, LOW);
    // Note: This doesn't change the physical valve position,
    // it just ensures no coil is energized
}

ValvePosition valveGetPosition(const DoubleSolenoidValve &valve) {
    return valve.position;
}

// ----------------- Higher-order valve operations -----------------

void withValve(DoubleSolenoidValve &valve, void (*operation)(DoubleSolenoidValve&)) {
    operation(valve);
}

void unlockValve(DoubleSolenoidValve &valve) {
    valveSetPosition(valve, VALVE_POSITION_UNLOCK);
}

void lockValve(DoubleSolenoidValve &valve) {
    valveSetPosition(valve, VALVE_POSITION_LOCK);
}

void deactivateValve(DoubleSolenoidValve &valve) {
    valveDeactivate(valve);
}

// ----------------- Sensor operations -----------------

void sensorInit(CylinderSensor &sensor, int pin) {
    sensor.pin = pin;
    pinMode(pin, INPUT);
    sensor.lastState = digitalRead(pin);
}

bool sensorRead(CylinderSensor &sensor) {
    bool currentState = digitalRead(sensor.pin);
    sensor.lastState = currentState;
    return currentState;
}

// ----------------- Status reporting -----------------

void printValveStatus(const DoubleSolenoidValve &valve, const char* valveName) {
    Serial.print(F("[DIAGNOSTIC] "));
    Serial.print(valveName);
    Serial.print(F(": "));
    Serial.println(valve.position == VALVE_POSITION_UNLOCK ? F("Unlocked") : F("Locked"));
}

void printSensorStatus(const CylinderSensor &sensor, const char* sensorName) {
    Serial.print(F("[DIAGNOSTIC] "));
    Serial.print(sensorName);
    Serial.print(F(" Sensor: "));
    Serial.println(sensorRead(const_cast<CylinderSensor&>(sensor)) ? F("Not activated") : F("Activated"));
}

// ----------------- Batch operations -----------------

void withAllValves(void (*operation)(DoubleSolenoidValve&)) {
    // Only perform operations if CCIO is present since all valves are on CCIO now
    if (!hasCCIO) {
        Serial.println(F("[ERROR] Cannot operate valves: CCIO-8 board not initialized"));
        return;
    }
    
    for (int i = 0; i < valveCount; i++) {
        withValve(*allValves[i], operation);
    }
}

void printAllValveStatus() {
    Serial.println(F("[DIAGNOSTIC] Current valve positions:"));
    for (int i = 0; i < valveCount; i++) {
        // Skip shuttle valve if no CCIO board
        if (i == 3 && !hasCCIO) continue;
        printValveStatus(*allValves[i], valveNames[i]);
    }
}

void printAllSensorStatus() {
    Serial.println(F("[DIAGNOSTIC] Current sensor readings:"));
    for (int i = 0; i < cylinderSensorCount; i++) {
        printSensorStatus(*allCylinderSensors[i], valveNames[i]);
    }
}

// ----------------- Advanced operations with sensor feedback -----------------

bool waitForSensor(CylinderSensor &sensor, bool expectedState, unsigned long timeoutMs) {
    unsigned long startTime = millis();
    while (sensorRead(sensor) != expectedState) {
        if (millis() - startTime > timeoutMs) {
            Serial.print(F("[ERROR] Sensor timeout: waited "));
            Serial.print(timeoutMs);
            Serial.println(F("ms for expected state"));
            return false; // Timeout occurred
        }
        delay(10); // Short delay to prevent excessive CPU usage
    }
    return true; // Sensor reached expected state
}

bool safeValveOperation(DoubleSolenoidValve &valve, CylinderSensor &sensor,
                      ValvePosition targetPosition, unsigned long timeoutMs) {
    // Determine expected sensor state based on valve position
    bool expectedSensorState = (targetPosition == VALVE_POSITION_LOCK);
    
    // Set the valve position
    valveSetPosition(valve, targetPosition);
    
    // Wait for sensor to confirm the operation
    return waitForSensor(sensor, expectedSensorState, timeoutMs);
}

// ----------------- Convenience functions -----------------

void lockAllValves() {
    withAllValves(lockValve);
}

void unlockAllValves() {
    withAllValves(unlockValve);
}


// Create an array for tray detection sensors for easier batch operations
CylinderSensor* allTrayDetectSensors[3] = {
    &tray1DetectSensor, &tray2DetectSensor, &tray3DetectSensor
};
const int trayDetectSensorCount = 3;

void printTrayDetectionStatus() {
    Serial.println(F("[DIAGNOSTIC] Tray Detection Status:"));
    for (int i = 0; i < trayDetectSensorCount; i++) {
        bool trayDetected = sensorRead(*allTrayDetectSensors[i]);
        Serial.print(F("  Tray "));
        Serial.print(i+1);
        Serial.print(F(": "));
        Serial.println(trayDetected ? F("DETECTED") : F("Not Present"));
    }
}

// Helper function to get a reference to the shuttle valve
DoubleSolenoidValve* getShuttleValve() {
    if (!hasCCIO) {
        Serial.println(F("[ERROR] Cannot access shuttle valve: CCIO board not detected"));
        return NULL;
    }
    // Shuttle valve is the fourth valve in the allValves array (index 3)
    return &shuttleValve;
}

// Helper function to get a reference to tray 1 valve
DoubleSolenoidValve* getTray1Valve() {
    if (!hasCCIO) {
        Serial.println(F("[ERROR] Cannot access tray 1 valve: CCIO board not detected"));
        return NULL;
    }
    return &tray1Valve;
}

// Helper function to get a reference to tray 2 valve
DoubleSolenoidValve* getTray2Valve() {
    if (!hasCCIO) {
        Serial.println(F("[ERROR] Cannot access tray 2 valve: CCIO board not detected"));
        return NULL;
    }
    return &tray2Valve;
}

// Helper function to get a reference to tray 3 valve
DoubleSolenoidValve* getTray3Valve() {
    if (!hasCCIO) {
        Serial.println(F("[ERROR] Cannot access tray 3 valve: CCIO board not detected"));
        return NULL;
    }
    return &tray3Valve;
}

// Helper function to get a reference to tray 1 cylinder sensor
CylinderSensor* getTray1Sensor() {
    return &tray1CylinderSensor;
}

// Helper function to get a reference to tray 2 cylinder sensor
CylinderSensor* getTray2Sensor() {
    return &tray2CylinderSensor;
}

// Helper function to get a reference to tray 3 cylinder sensor
CylinderSensor* getTray3Sensor() {
    return &tray3CylinderSensor;
}

// Helper function to get a reference to shuttle cylinder sensor
CylinderSensor* getShuttleSensor() {
    return &shuttleCylinderSensor;
}

// Helper function to get a reference to tray 1 detection sensor
CylinderSensor* getTray1DetectionSensor() {
    return &tray1DetectSensor;
}

// Helper function to get a reference to tray 2 detection sensor
CylinderSensor* getTray2DetectionSensor() {
    return &tray2DetectSensor;
}

// Helper function to get a reference to tray 3 detection sensor
CylinderSensor* getTray3DetectionSensor() {
    return &tray3DetectSensor;
}