#include "ProportionalValve.h"

// Constructor
ProportionalValve::ProportionalValve(byte controlPin, byte feedbackPin) {
    this->controlPin = controlPin;
    this->feedbackPin = feedbackPin;
    this->controlVoltage = 0.0; // Default to 0V
}

// Setup function to configure pins
void ProportionalValve::setup() {
    // Set control pin (AO0) as an output
    pinMode(controlPin, OUTPUT);
    
    // Set feedback pin (AI13) as an input
    pinMode(feedbackPin, INPUT);
}

// Set the valve position as a percentage (0-100%)
void ProportionalValve::setPosition(float percentage) {
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;

    controlVoltage = percentageToVoltage(percentage);

    // Convert voltage (0-10V) to a PWM value (0-255) for analogWrite
    int pwmValue = map(controlVoltage, 0, 10, 0, 255);

    // Set the PWM output on the analog output pin
    analogWrite(controlPin, pwmValue);
}

// Get feedback in volts from the valve
float ProportionalValve::getFeedback() {
    return readFeedbackVoltage();
}

// Convert percentage (0-100%) to a control voltage (0-10V)
float ProportionalValve::percentageToVoltage(float percentage) {
    return (percentage / 100.0) * 10.0; // 0% maps to 0V, 100% maps to 10V
}

// Convert analogRead value (0-1023) from feedback pin to voltage (0-10V)
float ProportionalValve::readFeedbackVoltage() {
    int analogValue = analogRead(feedbackPin);
    // Map the ADC value (0-1023) to a voltage (0-10000 mV) and convert to volts
    return map(analogValue, 0, 1023, 0, 10000) / 1000.0;
}
