#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include <Arduino.h>

class PressureSensor {
private:
    byte analogPin;   // Pin connected to AI12
    float minPressure;  // Minimum pressure in psi
    float maxPressure;  // Maximum pressure in psi

public:
    // Constructor to initialize pin and pressure range
    PressureSensor(byte analogPin, float minPressure, float maxPressure);

    // Function to read pressure in psi
    float readPressure();

    // Read raw voltage from analog pin
    float readVoltage();
};

#endif
