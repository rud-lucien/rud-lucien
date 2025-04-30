#include "ValveController.h"

// ----------------- Global variables -----------------

// Valve instances
DoubleSolenoidValve tray1Valve;
DoubleSolenoidValve tray2Valve;
DoubleSolenoidValve tray3Valve;
DoubleSolenoidValve shuttleValve;

// Sensor instances
CylinderSensor tray1Sensor;
CylinderSensor tray2Sensor;
CylinderSensor tray3Sensor;
CylinderSensor shuttleSensor;

// Arrays for batch operations
DoubleSolenoidValve* allValves[4] = {&tray1Valve, &tray2Valve, &tray3Valve, &shuttleValve};
const int valveCount = 4;
const char* valveNames[4] = {"Tray 1", "Tray 2", "Tray 3", "Shuttle"};

CylinderSensor* allSensors[4] = {&tray1Sensor, &tray2Sensor, &tray3Sensor, &shuttleSensor};
const int sensorCount = 4;

// CCIO Board status
bool hasCCIO = false;

// ----------------- Initialization functions -----------------

void initValveSystem() {
    // Set up pins for tray valves
    tray1Valve.unlockPin = TRAY_1_UNLOCK_PIN;
    tray1Valve.lockPin = TRAY_1_LOCK_PIN;
    tray2Valve.unlockPin = TRAY_2_UNLOCK_PIN;
    tray2Valve.lockPin = TRAY_2_LOCK_PIN;
    tray3Valve.unlockPin = TRAY_3_UNLOCK_PIN;
    tray3Valve.lockPin = TRAY_3_LOCK_PIN;
    
    // Initialize tray valves
    valveInit(tray1Valve);
    valveInit(tray2Valve);
    valveInit(tray3Valve);
    
    // Set up pins for sensors
    tray1Sensor.pin = TRAY_1_SENSOR_PIN;
    tray2Sensor.pin = TRAY_2_SENSOR_PIN;
    tray3Sensor.pin = TRAY_3_SENSOR_PIN;
    shuttleSensor.pin = SHUTTLE_SENSOR_PIN;
    
    // Initialize sensors
    sensorInit(tray1Sensor, TRAY_1_SENSOR_PIN);
    sensorInit(tray2Sensor, TRAY_2_SENSOR_PIN);
    sensorInit(tray3Sensor, TRAY_3_SENSOR_PIN);
    
    // Configure A9 as digital input for shuttle sensor
    pinMode(A9, INPUT);
    sensorInit(shuttleSensor, SHUTTLE_SENSOR_PIN);
}

void initValveWithCCIO(bool hasCCIOBoard) {
    hasCCIO = hasCCIOBoard;
    
    if (hasCCIO) {
        // If CCIO is present, configure shuttle valve
        pinMode(SHUTTLE_LOCK_PIN, OUTPUT);
        pinMode(SHUTTLE_UNLOCK_PIN, OUTPUT);
        
        shuttleValve.unlockPin = SHUTTLE_UNLOCK_PIN;
        shuttleValve.lockPin = SHUTTLE_LOCK_PIN;
        valveInit(shuttleValve);
    }
}

// ----------------- Low-level hardware functions -----------------

void pulsePin(int pin, unsigned long duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

void configurePinAsOutput(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void configureValvePins(const DoubleSolenoidValve &valve) {
    configurePinAsOutput(valve.unlockPin);
    configurePinAsOutput(valve.lockPin);
}

int getActivationPin(const DoubleSolenoidValve &valve, ValvePosition target) {
    return (target == VALVE_POSITION_UNLOCK) ? valve.unlockPin : valve.lockPin;
}

// ----------------- Valve operations -----------------

void valveInit(DoubleSolenoidValve &valve) {
    // Configure pins
    configureValvePins(valve);
    
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
    Serial.print(valveName);
    Serial.print(": ");
    Serial.println(valve.position == VALVE_POSITION_UNLOCK ? "Unlocked" : "Locked");
}

void printSensorStatus(const CylinderSensor &sensor, const char* sensorName) {
    Serial.print(sensorName);
    Serial.print(" Sensor: ");
    Serial.println(sensorRead(const_cast<CylinderSensor&>(sensor)) ? "Not activated" : "Activated");
}

// ----------------- Batch operations -----------------

void withAllValves(void (*operation)(DoubleSolenoidValve&)) {
    for (int i = 0; i < valveCount; i++) {
        // Skip shuttle valve if no CCIO board
        if (i == 3 && !hasCCIO) continue;
        withValve(*allValves[i], operation);
    }
}

void printAllValveStatus() {
    Serial.println("Current valve positions:");
    for (int i = 0; i < valveCount; i++) {
        // Skip shuttle valve if no CCIO board
        if (i == 3 && !hasCCIO) continue;
        printValveStatus(*allValves[i], valveNames[i]);
    }
}

void printAllSensorStatus() {
    Serial.println("Current sensor readings:");
    for (int i = 0; i < sensorCount; i++) {
        printSensorStatus(*allSensors[i], valveNames[i]);
    }
}

// ----------------- Advanced operations with sensor feedback -----------------

bool waitForSensor(CylinderSensor &sensor, bool expectedState, unsigned long timeoutMs) {
    unsigned long startTime = millis();
    while (sensorRead(sensor) != expectedState) {
        if (millis() - startTime > timeoutMs) {
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

DoubleSolenoidValve* getValveByIndex(int index) {
    if (index >= 0 && index < valveCount) {
        // Skip shuttle valve if no CCIO
        if (index == 3 && !hasCCIO) return NULL;
        return allValves[index];
    }
    return NULL;
}

CylinderSensor* getSensorByIndex(int index) {
    if (index >= 0 && index < sensorCount) {
        return allSensors[index];
    }
    return NULL;
}

const char* getValveNameByIndex(int index) {
    if (index >= 0 && index < valveCount) {
        return valveNames[index];
    }
    return "Unknown";
}