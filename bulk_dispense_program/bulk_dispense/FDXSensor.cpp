#include "FDXSensor.h"

FDXSensor::FDXSensor(ModbusConnection &modbusConnection, int registerAddr, int resetPin)
  : modbus(modbusConnection), registerAddress(registerAddr), resetChannel(resetPin) {
  pinMode(resetChannel, OUTPUT);  // Set up the reset pin as output
}

bool FDXSensor::readIntegratedFlow(uint32_t &flowValue) {
  int registerQuantity = 2;  // Two 16-bit registers (32 bits total)

  // Read the modbus registers and extract raw flow value
  if (modbus.readRegisters(registerAddress, registerQuantity, flowValue)) {
    return true;  // Successfully read the raw flow value
  } else {
    return false;  // Failed to read registers
  }
}

float FDXSensor::getScaledFlowValue() {
  uint32_t rawFlowValue;

  if (readIntegratedFlow(rawFlowValue)) {
    // Scale the raw flow value by dividing by 10 and return it as a float
    return rawFlowValue / 10.0;
  } else {
    // If reading fails, return -1 or any other sentinel value indicating failure
    return -1.0;
  }
}

void FDXSensor::startResetFlow() {
  // Start the reset process by applying 24V (setting the pin to HIGH)
  if (!resetInProgress) {
    Serial.println("Starting flow sensor reset...");
    digitalWrite(resetChannel, HIGH);
    resetStartTime = millis();  // Record the start time
    resetInProgress = true;     // Mark that reset is in progress
  }
}

void FDXSensor::handleReset() {
  if (resetInProgress && (millis() - resetStartTime >= resetDuration)) {
    // Enough time has passed, stop the reset by setting the pin to LOW
    digitalWrite(resetChannel, LOW);
    resetInProgress = false;  // Reset is complete
    Serial.println("Flow sensor reset completed.");
  }
}
