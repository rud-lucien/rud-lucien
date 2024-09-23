#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include <Controllino.h>

class PressureSensor {
private:
    byte analogPin;  // Pin connected to AI12
    float minPressure;  // Minimum pressure in the sensor's range
    float maxPressure;  // Maximum pressure in the sensor's range

public:
    // Constructor to initialize the pin and pressure range
    PressureSensor(byte analogPin, float minPressure, float maxPressure);

    // Setup function to configure pins
    void setup();

    // Function to read the pressure in the correct units
    float readPressure();

    // Utility function to convert the analog input to voltage
    float readVoltage();

    // Convert voltage (0-10V) to pressure based on 4-20mA range
    float voltageToPressure(float voltage);
};

#endif
