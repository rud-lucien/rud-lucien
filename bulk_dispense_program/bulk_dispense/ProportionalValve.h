#ifndef PROPORTIONALVALVE_H
#define PROPORTIONALVALVE_H

#include <Controllino.h>

class ProportionalValve {
private:
    byte controlPin;  // Pin connected to AO0 (analog output)
    byte feedbackPin; // Pin connected to AI13 (analog input)
    float controlVoltage; // Voltage sent to the valve

public:
    // Constructor to initialize the pins
    ProportionalValve(byte controlPin, byte feedbackPin);

    // Setup function to configure pins (if needed)
    void setup();

    // Set the valve position as a percentage (0-100%)
    void setPosition(float percentage);

    // Get the current position feedback from the valve (in volts)
    float getFeedback();

    // Utility function to convert percentage to control voltage (0-10V)
    float percentageToVoltage(float percentage);

    // Utility function to convert analogRead value to feedback voltage (0-10V)
    float readFeedbackVoltage();
};

#endif
