/*
 Name:		lynx_linear_actuator.ino
 Created:	4/14/2025 12:00:17 PM
 Author:	rlucien
*/

#include "Arduino.h"
#include "ClearCore.h"

// Specify which ClearCore serial COM port is connected to the "COM IN" port
// of the CCIO-8 board. COM-1 may also be used.
#define CcioPort ConnectorCOM0

uint8_t ccioBoardCount; // Store the number of connected CCIO-8 boards here.
uint8_t ccioPinCount;   // Store the number of connected CCIO-8 pins here.

// These will be used to format the text that is printed to the serial port.
#define MAX_MSG_LEN 80
char msg[MAX_MSG_LEN + 1];

// Set this flag to true to use the CCIO-8 connectors as digital inputs.
// Set it to false to use the CCIO-8 connectors as digital outputs.
const bool inputMode = false;

// Definition for IO1 and IO2 pins (ClearCore connectors)
// These constants are needed to map ClearCore connectors to pins for Arduino API
#define TRAY_1_LOCK_PIN 0     // IO-0 connector
#define TRAY_1_UNLOCK_PIN 1   // IO-1 connector
#define TRAY_2_LOCK_PIN 2     // IO-2 connector
#define TRAY_2_UNLOCK_PIN 3   // IO-3 connector
#define TRAY_3_LOCK_PIN 4     // IO-4 connector
#define TRAY_3_UNLOCK_PIN 5   // IO-5 connector
#define SHUTTLE_LOCK_PIN 64   // IO-6 connector
#define SHUTTLE_UNLOCK_PIN 65 // IO-7 connector

// 1. Define constants and enums first
enum ValvePosition {
    VALVE_POSITION_UNLOCK,
    VALVE_POSITION_LOCK
};

// 2. Define struct types
struct DoubleSolenoidValve {
    int unlockPin;
    int lockPin;
    ValvePosition position;
};

// 3. Declare valve variables
DoubleSolenoidValve tray1Valve;
DoubleSolenoidValve tray2Valve;
DoubleSolenoidValve tray3Valve;
DoubleSolenoidValve shuttleValve;

// 4. Set up arrays that reference the valves
DoubleSolenoidValve* allValves[] = {&tray1Valve, &tray2Valve, &tray3Valve, &shuttleValve};
const int valveCount = 4;
const char* valveNames[] = {"Tray 1", "Tray 2", "Tray 3", "Shuttle"};

// Define sensor pins
#define TRAY_1_SENSOR_PIN DI6    // Connect Tray 1 sensor to DI-6
#define TRAY_2_SENSOR_PIN DI7    // Connect Tray 2 sensor to DI-7
#define TRAY_3_SENSOR_PIN DI8    // Connect Tray 3 sensor to DI-8
#define SHUTTLE_SENSOR_PIN A9   // Connect Shuttle sensor to DI-9

// Sensor state structure
struct CylinderSensor {
    int pin;
    bool lastState;
};

// Declare sensor variables
CylinderSensor tray1Sensor;
CylinderSensor tray2Sensor;
CylinderSensor tray3Sensor;
CylinderSensor shuttleSensor;

// Array of all sensors
CylinderSensor* allSensors[] = {&tray1Sensor, &tray2Sensor, &tray3Sensor, &shuttleSensor};
const int sensorCount = 4;

// Minimum recommended pulse duration in milliseconds
const unsigned long PULSE_DURATION = 100;

// Pure function for pulsing pins
void pulsePin(int pin, unsigned long duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
}

// Pin configuration function
void configurePinAsOutput(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

// Configure pins for a valve
void configureValvePins(const DoubleSolenoidValve &valve) {
    configurePinAsOutput(valve.unlockPin);
    configurePinAsOutput(valve.lockPin);
}

// Get the appropriate pin based on target position
int getActivationPin(const DoubleSolenoidValve &valve, ValvePosition target) {
    return (target == VALVE_POSITION_UNLOCK) ? valve.unlockPin : valve.lockPin;
}


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

// Higher-order function for valve operations
void withValve(DoubleSolenoidValve &valve, void (*operation)(DoubleSolenoidValve&)) {
    operation(valve);
}

// Operations that can be passed to withValve
void unlockValve(DoubleSolenoidValve &valve) {
    valveSetPosition(valve, VALVE_POSITION_UNLOCK);
}

void lockValve(DoubleSolenoidValve &valve) {
    valveSetPosition(valve, VALVE_POSITION_LOCK);
}

void deactivateValve(DoubleSolenoidValve &valve) {
    valveDeactivate(valve);
}

// Print valve status
void printValveStatus(const DoubleSolenoidValve &valve, const char* valveName) {
    Serial.print(valveName);
    Serial.print(": ");
    Serial.println(valve.position == VALVE_POSITION_UNLOCK ? "Unlocked" : "Locked");
}

// Function to apply an operation to multiple valves
void withAllValves(void (*operation)(DoubleSolenoidValve&)) {
    for (int i = 0; i < valveCount; i++) {
        // Skip shuttle valve if no CCIO board
        if (i == 3 && ccioBoardCount == 0) continue;
        withValve(*allValves[i], operation);
    }
}

// Function to print status of all valves
void printAllValveStatus() {
    Serial.println("Current valve positions:");
    for (int i = 0; i < valveCount; i++) {
        // Skip shuttle valve if no CCIO board
        if (i == 3 && ccioBoardCount == 0) continue;
        printValveStatus(*allValves[i], valveNames[i]);
    }
}

// Initialize sensor
void sensorInit(CylinderSensor &sensor, int pin) {
    sensor.pin = pin;
    pinMode(pin, INPUT);
    sensor.lastState = digitalRead(pin);
}

// Read sensor state
bool sensorRead(CylinderSensor &sensor) {
    bool currentState = digitalRead(sensor.pin);
    sensor.lastState = currentState;
    return currentState;
}

// Print sensor status
void printSensorStatus(const CylinderSensor &sensor, const char* sensorName) {
    Serial.print(sensorName);
    Serial.print(" Sensor: ");
    Serial.println(sensorRead(const_cast<CylinderSensor&>(sensor)) ? "Not activated" : "Activated");
}

// Function to print status of all sensors
void printAllSensorStatus() {
    Serial.println("Current sensor readings:");
    for (int i = 0; i < sensorCount; i++) {
        printSensorStatus(*allSensors[i], valveNames[i]);  // Reuse valve names for sensors
    }
}


// the setup function runs once when you press reset or power the board
void setup()
{
    Serial.begin(115200); // Use Arduino's Serial instead of ConnectorUsb

    // Set up the CCIO-8 COM port.
    CcioPort.Mode(Connector::CCIO);
    CcioPort.PortOpen();

    // Initialize the CCIO-8 board.
    ccioBoardCount = CcioMgr.CcioCount();
    ccioPinCount = ccioBoardCount * CCIO_PINS_PER_BOARD;

    // Wait for USB serial to connect (needed for native USB port only)
    while (!Serial)
    {
        ; // wait for serial port to connect
    }

    Serial.println("Initializing valves...");

    // Print the number of discovered CCIO-8 boards to the serial port.
    snprintf(msg, MAX_MSG_LEN, "Discovered %d CCIO-8 board", ccioBoardCount);
    Serial.print(msg);
    if (ccioBoardCount != 1)
    {
        Serial.print("s");
    }
    Serial.println("...");

    // Initialize main board valves
    tray1Valve.unlockPin = TRAY_1_UNLOCK_PIN;
    tray1Valve.lockPin = TRAY_1_LOCK_PIN;
    tray2Valve.unlockPin = TRAY_2_UNLOCK_PIN;
    tray2Valve.lockPin = TRAY_2_LOCK_PIN;
    tray3Valve.unlockPin = TRAY_3_UNLOCK_PIN;
    tray3Valve.lockPin = TRAY_3_LOCK_PIN;

    // Initialize the tray valves
    valveInit(tray1Valve);
    valveInit(tray2Valve);
    valveInit(tray3Valve);

    // Configure CCIO pins - This is the critical part!
    // Follow exactly like the example does
    if (ccioBoardCount > 0)
    {
        Serial.println("Configuring CCIO pins...");
        // Explicitly set each CCIO pin we need as an output, one by one
        pinMode(SHUTTLE_LOCK_PIN, OUTPUT);
        pinMode(SHUTTLE_UNLOCK_PIN, OUTPUT);

        // Initialize shuttle valve only after configuring its pins
        shuttleValve.unlockPin = SHUTTLE_UNLOCK_PIN;
        shuttleValve.lockPin = SHUTTLE_LOCK_PIN;
        valveInit(shuttleValve);
    }
    else
    {
        Serial.println("WARNING: No CCIO-8 boards detected! Shuttle valve will not function.");
    }

    // Initialize sensors
    Serial.println("Initializing cylinder position sensors...");

    // // Configure A9 as digital input
    // pinMode(A9, INPUT);
    
    tray1Sensor.pin = TRAY_1_SENSOR_PIN;
    tray2Sensor.pin = TRAY_2_SENSOR_PIN;
    tray3Sensor.pin = TRAY_3_SENSOR_PIN;
    shuttleSensor.pin = SHUTTLE_SENSOR_PIN;
    
    // Initialize all sensors
    sensorInit(tray1Sensor, TRAY_1_SENSOR_PIN);
    sensorInit(tray2Sensor, TRAY_2_SENSOR_PIN);
    sensorInit(tray3Sensor, TRAY_3_SENSOR_PIN);
    sensorInit(shuttleSensor, SHUTTLE_SENSOR_PIN);
    
    Serial.println("Cylinder sensors initialized");

    Serial.println("5/2-way valve controller initialized");
    Serial.println("Type 'help' for available commands");
}

// the loop function runs over and over again until power down or reset
void loop()
{
    // Check if there's data available to read from Serial
    if (Serial.available() > 0)
    {
        // Read the incoming command
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove any whitespace

        // Convert to lowercase for case-insensitive comparison
        command.toLowerCase();

        // Process the command
        if (command == "tray 1 l" || command == "t1l")
        {
            Serial.println("Command received: Locking tray 1");
            withValve(tray1Valve, lockValve);
        }
        else if (command == "tray 1 ul" || command == "t1ul")
        {
            Serial.println("Command received: Unlocking tray 1");
            withValve(tray1Valve, unlockValve);
        }
        else if (command == "tray 2 l" || command == "t2l")
        {
            Serial.println("Command received: Locking tray 2");
            withValve(tray2Valve, lockValve);
        }
        else if (command == "tray 2 ul" || command == "t2ul")
        {
            Serial.println("Command received: Unlocking tray 2");
            withValve(tray2Valve, unlockValve);
        }
        else if (command == "tray 3 l" || command == "t3l")
        {
            Serial.println("Command received: Locking tray 3");
            withValve(tray3Valve, lockValve);
        }
        else if (command == "tray 3 ul" || command == "t3ul")
        {
            Serial.println("Command received: Unlocking tray 3");
            withValve(tray3Valve, unlockValve);
        }
        else if (command == "shuttle l" || command == "sl")
        {
            Serial.println("Command received: Locking shuttle");
            withValve(shuttleValve, lockValve);
        }
        else if (command == "shuttle ul" || command == "sul")
        {
            Serial.println("Command received: Unlocking shuttle");
            withValve(shuttleValve, unlockValve);
        }
        else if (command == "status" || command == "s")
        {
            // Report the current status
            printAllValveStatus();
        }
        else if (command == "sensors" || command == "ss")
        {
            // Report the current sensor status
            printAllSensorStatus();
        }
        else if (command == "help" || command == "h")
        {
            // Display available commands
            Serial.println("Available commands:");
            Serial.println("Tray commands:");
            Serial.println("  tray 1 l or t1l - Lock tray 1");
            Serial.println("  tray 1 ul or t1ul - Unlock tray 1");
            Serial.println("  tray 2 l or t2l - Lock tray 2");
            Serial.println("  tray 2 ul or t2ul - Unlock tray 2");
            Serial.println("  tray 3 l or t3l - Lock tray 3");
            Serial.println("  tray 3 ul or t3ul - Unlock tray 3");
            Serial.println("Shuttle commands:");
            Serial.println("  shuttle l or sl - Lock shuttle");
            Serial.println("  shuttle ul or sul - Unlock shuttle");
            Serial.println("General commands:");
            Serial.println("  status or s - Display current valve positions");
            Serial.println("  help or h - Show this help message");
            Serial.println("Sensor commands:");
            Serial.println("  sensors or ss - Display all sensor readings");
        }
        else
        {
            Serial.println("Unknown command. Type 'help' for available commands.");
        }
    }
}

