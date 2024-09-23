#ifndef OVERFLOWSENSOR_H
#define OVERFLOWSENSOR_H

#include <Controllino.h>

class OverflowSensor {
private:
    byte sensorPin;  // Pin connected to the overflow sensor
    bool overflowDetected; // Flag to track overflow state

public:
    // Constructor to initialize the sensor pin
    OverflowSensor(byte pin);

    // Setup method to initialize the sensor pin
    void setup();

    // Method to check if the overflow sensor is triggered
    bool isOverflowing();

    // Modified method: return 1 if overflow detected, 0 if not
    int loop();
};

#endif // OVERFLOWSENSOR_H
