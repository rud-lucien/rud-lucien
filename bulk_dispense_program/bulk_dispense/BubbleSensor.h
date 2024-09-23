#ifndef BUBBLESENSOR_H
#define BUBBLESENSOR_H

#include <Controllino.h>

class BubbleSensor {
private:
    byte inputPin;  // The pin connected to the sensor output (AI0-AI3)

public:
    // Constructor to initialize the pin
    BubbleSensor(byte inputPin);

    // Setup function to configure the input pin
    void setup();

    // Function to check if liquid is detected (returns true if liquid is present)
    bool isLiquidDetected();
};

#endif
