#include "OverflowSensor.h"

// Constructor to initialize the sensor pin
OverflowSensor::OverflowSensor(byte pin) {
    sensorPin = pin;
    overflowDetected = false;  // Initially, no overflow detected
}

// Setup method to initialize the sensor pin
void OverflowSensor::setup() {
    pinMode(sensorPin, INPUT);  // Configure the sensor pin as input
}

// Method to check if the overflow sensor is triggered
bool OverflowSensor::isOverflowing() {
    int sensorState = digitalRead(sensorPin);  // Read the sensor pin
    overflowDetected = (sensorState == HIGH);  // Assuming HIGH means overflow detected
    return overflowDetected;
}

// Modified loop method: return 1 if overflow detected, 0 if not
int OverflowSensor::loop() {
    if (isOverflowing()) {
        return 1;  // Return 1 if overflow is detected
    } else {
        return 0;  // Return 0 if no overflow is detected
    }
}
