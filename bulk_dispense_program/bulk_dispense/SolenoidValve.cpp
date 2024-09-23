#include "SolenoidValve.h"

// Constructor: Initialize the valve with the control pin
SolenoidValve::SolenoidValve(byte pin) {
    this->controlPin = pin;
    this->valveState = false;  // Assume the valve starts closed
}

// Setup function to configure the pin
void SolenoidValve::setup() {
  pinMode(controlPin, OUTPUT);    // Set the control pin as an output
  digitalWrite(controlPin, LOW);  // Ensure the valve starts in the closed state
}

// Function to open the valve
void SolenoidValve::openValve() {
  digitalWrite(controlPin, HIGH);  // Apply 24V to open the valve
  valveState = true;               // Update the state to open
}

// Function to close the valve
void SolenoidValve::closeValve() {
  digitalWrite(controlPin, LOW);  // Remove 24V to close the valve
  valveState = false;             // Update the state to closed
}

// Function to check if the valve is open
bool SolenoidValve::isValveOpen() {
  return valveState;  // Return the current state of the valve
}
