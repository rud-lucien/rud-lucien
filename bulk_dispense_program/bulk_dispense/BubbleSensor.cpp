#include "BubbleSensor.h"

// Constructor: Initializes the pin connected to the sensor
BubbleSensor::BubbleSensor(byte inputPin) {
    this->inputPin = inputPin;
}

// Setup function to configure the input pin
void BubbleSensor::setup() {
    pinMode(inputPin, INPUT);  // Configure the input pin as a digital input
}

// Function to check if liquid is detected
bool BubbleSensor::isLiquidDetected() {
    int sensorState = digitalRead(inputPin);  // Read the sensor state (HIGH or LOW)

    // In PNP mode, HIGH means liquid detected, LOW means no liquid
    return sensorState == HIGH;
}
