#include "VacuumSensor.h"

// Constructor: Initializes the pin connected to the sensor
VacuumSensor::VacuumSensor(byte inputPin) {
    this->inputPin = inputPin;
}

// Setup function to configure the input pin
void VacuumSensor::setup() {
    pinMode(inputPin, INPUT);  // Configure the input pin as a digital input
}

// Function to check if vacuum pressure is detected
bool VacuumSensor::isVacuumDetected() {
    int sensorState = digitalRead(inputPin);  // Read the sensor state (HIGH or LOW)

    // In PNP mode, HIGH means vacuum detected, LOW means no vacuum detected
    return sensorState == HIGH;
}
