#ifndef SOLENOIDVALVE_H
#define SOLENOIDVALVE_H

#include <Controllino.h>

class SolenoidValve {
private:
  int controlPin;   // The pin connected to the valve
  bool valveState;  // Track whether the valve is open (true) or closed (false)

public:
  // Constructor to initialize the valve with its control pin
  SolenoidValve(byte pin);

  // Setup function to initialize the pin
  void setup();

  // Function to open the valve (apply 24V)
  void openValve();

  // Function to close the valve (remove 24V)
  void closeValve();

  // Function to check if the valve is currently open
  bool isValveOpen();
};

#endif
