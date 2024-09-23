#ifndef FDXSENSOR_H
#define FDXSENSOR_H

#include "ModbusConnection.h"
#include <Controllino.h>

class FDXSensor {
private:
  ModbusConnection &modbus;  // Reference to the ModbusConnection object
  int resetChannel;          // DO pin to reset the sensor
  int registerAddress;       // Modbus register address for the sensor

  bool resetInProgress = false;       // Track if a reset is in progress
  unsigned long resetStartTime = 0;   // When the reset started
  unsigned long resetDuration = 100;  // Duration to apply 24V (in ms)

public:
  // Constructor
  FDXSensor(ModbusConnection &modbusConnection, int registerAddr, int resetPin);

  // Read the integrated flow value for this sensor (returns raw uint32_t)
  bool readIntegratedFlow(uint32_t &flowValue);

  // Get the scaled flow value as a float (converts to mL)
  float getScaledFlowValue();

  // Start the reset process (non-blocking)
  void startResetFlow();

  // Handle the reset process (to be called in loop)
  void handleReset();
};

#endif
