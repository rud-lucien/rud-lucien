#include "PressureSensor.h"

// Constructor
PressureSensor::PressureSensor(byte analogPin, float minPressure, float maxPressure) {
    this->analogPin = analogPin;
    this->minPressure = minPressure;
    this->maxPressure = maxPressure;
}

// Setup function to configure the analog pin
void PressureSensor::setup() {
    pinMode(analogPin, INPUT);  // Configure the pin as input
}

// Function to read the current pressure in the correct units
float PressureSensor::readPressure() {
    float voltage = readVoltage();  // Read the voltage from AI12
    return voltageToPressure(voltage);  // Convert the voltage to pressure
}

// Utility function to read the voltage from the analog input pin (AI12)
float PressureSensor::readVoltage() {
    int analogValue = analogRead(analogPin);  // Read the analog input (0-1023)
    return map(analogValue, 0, 1023, 0, 10000) / 1000.0;  // Map to 0-10V range
}

// Convert voltage (0-10V) directly to pressure (0-50 psi)
float PressureSensor::voltageToPressure(float voltage) {
    // Map the 0-10V range to 0-50 psi
    return map(voltage * 1000, 0, 10000, minPressure * 1000, maxPressure * 1000) / 1000.0;
}
