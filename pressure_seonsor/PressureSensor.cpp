#include "PressureSensor.h"

// Constructor
PressureSensor::PressureSensor(byte analogPin, float minPressure, float maxPressure) {
    this->analogPin = analogPin;
    this->minPressure = minPressure;
    this->maxPressure = maxPressure;
}

// Function to read the voltage from the analog input (0-10V)
float PressureSensor::readVoltage() {
    int analogValue = analogRead(analogPin);  // Read analog input (0-1023)
    return (analogValue / 1023.0) * 10.0;  // Convert to 0-10V range
}

// Function to read the pressure in psi
float PressureSensor::readPressure() {
    float voltage = readVoltage();
    return (voltage / 10.0) * maxPressure;  // Scale voltage (0-10V) to pressure (0-87 psi)
}
