#include "pressure_sensor.h"

// Pure function to read voltage from the sensor
float readVoltage(const PressureSensorConfig &config) {
    int analogValue = analogRead(config.analogPin);  // Read analog input (0-1023)
    return (analogValue / 1023.0) * 10.0;  // Convert to 0-10V range
}

// Pure function to read pressure in psi
float readPressure(const PressureSensorConfig &config) {
    float voltage = readVoltage(config);
    return (voltage / 10.0) * config.maxPressure;  // Scale voltage (0-10V) to pressure range
}
