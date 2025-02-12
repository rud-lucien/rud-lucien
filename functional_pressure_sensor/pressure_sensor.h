#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

// Struct to hold sensor configuration
struct PressureSensorConfig {
    byte analogPin;    // Pin connected to the sensor
    float minPressure; // Minimum pressure in psi
    float maxPressure; // Maximum pressure in psi
};

// Function prototypes (declarations)
float readVoltage(const PressureSensorConfig &config);
float readPressure(const PressureSensorConfig &config);

#endif
