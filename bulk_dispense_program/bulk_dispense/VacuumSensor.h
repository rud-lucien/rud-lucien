#ifndef VACUUMSENSOR_H
#define VACUUMSENSOR_H

#include <Controllino.h>

class VacuumSensor {
private:
    byte inputPin;  // The pin connected to the sensor output (AI0-AI3)

public:
    // Constructor to initialize the pin
    VacuumSensor(byte inputPin);

    // Setup function to configure the input pin
    void setup();

    // Function to check if vacuum is detected (returns true if vacuum is present)
    bool isVacuumDetected();
};

#endif