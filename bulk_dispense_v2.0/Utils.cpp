#include "Utils.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Commands.h"
#include "Commander-API.hpp"
#include <Wire.h>
#include <string.h>
#include <ctype.h>

// ------------------------------------------------------------------
// trimLeadingSpaces()
// ------------------------------------------------------------------
char* trimLeadingSpaces(char* str) {
  while (*str && isspace(*str)) {
    str++;
  }
  return str;
}

// ------------------------------------------------------------------
// processMultipleCommands()
// ------------------------------------------------------------------
void processMultipleCommands(char* commandLine, Stream* stream) {
  char* token = strtok(commandLine, ",");  // Split at commas
  while (token != NULL) {
    token = trimLeadingSpaces(token);
    if (strlen(token) > 0) {
      Serial.print(F("[COMMAND] Processing: "));
      Serial.println(token);
      // Execute command using the global commander (declared in Commands.cpp)
      commander.execute(token, stream);
    }
    token = strtok(NULL, ",");
  }
}

// ------------------------------------------------------------------
// handleSerialCommands()
// ------------------------------------------------------------------
void handleSerialCommands() {
  static char commandBuffer[COMMAND_SIZE];
  static uint8_t commandIndex = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      commandBuffer[commandIndex] = '\0';  // Null-terminate the string
      Serial.print(F("[COMMAND] Received: "));
      Serial.println(commandBuffer);
      processMultipleCommands(commandBuffer, &Serial);
      commandIndex = 0;      // Reset the buffer index for the next command
    } else if (c != '\r') {  // Ignore carriage return characters
      if (commandIndex < (COMMAND_SIZE - 1)) {
        commandBuffer[commandIndex++] = c;
      }
    }
  }
}

// ------------------------------------------------------------------
// checkAndSetPressure()
// ------------------------------------------------------------------
bool checkAndSetPressure(float thresholdPressure, float valvePosition, unsigned long timeout) {
  unsigned long startTime = millis();
  float currentPressure = readPressure(pressureSensor);
  float currentValvePos = proportionalValve.controlVoltage;  // current valve setting

  if (currentPressure >= thresholdPressure && currentValvePos == valvePosition) {
    Serial.println(F("[MESSAGE] System is already pressurized and valve is at the correct position."));
    return true;
  }

  // Set the valve to the desired position.
  proportionalValve = setValvePosition(proportionalValve, valvePosition);
  Serial.print(F("[MESSAGE] Pressure valve set to "));
  Serial.print(valvePosition);
  Serial.println(F("%. Waiting for pressure stabilization..."));

  // Wait until the pressure threshold is met or timeout occurs.
  while (millis() - startTime < timeout) {
    currentPressure = readPressure(pressureSensor);
    if (currentPressure >= thresholdPressure) {
      Serial.println(F("[MESSAGE] Pressure threshold reached."));
      return true;
    }
    delay(100);
  }

  Serial.print(F("[ERROR] Pressure threshold not reached. Current pressure: "));
  Serial.print(currentPressure);
  Serial.println(F(" psi. Operation aborted."));
  return false;
}

// ------------------------------------------------------------------
// resetI2CBus()
// ------------------------------------------------------------------
void resetI2CBus() {
  Serial.println(F("[MESSAGE] Resetting I2C bus..."));
  Wire.end();
  delay(100);
  Wire.begin();
}

// ------------------------------------------------------------------
// openDispenseValves()
// ------------------------------------------------------------------
void openDispenseValves(int troughNumber) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    Serial.println(F("[ERROR] Invalid trough number provided to openDispenseValves()"));
    return;
  }
  switch (troughNumber) {
    case 1:
      reagentValve1 = openValve(reagentValve1);
      mediaValve1 = openValve(mediaValve1);
      break;
    case 2:
      reagentValve2 = openValve(reagentValve2);
      mediaValve2 = openValve(mediaValve2);
      break;
    case 3:
      reagentValve3 = openValve(reagentValve3);
      mediaValve3 = openValve(mediaValve3);
      break;
    case 4:
      reagentValve4 = openValve(reagentValve4);
      mediaValve4 = openValve(mediaValve4);
      break;
  }
  Serial.print(F("[MESSAGE] Opened reagent and media valves for Trough "));
  Serial.println(troughNumber);
}

// ------------------------------------------------------------------
// closeDispenseValves()
// ------------------------------------------------------------------
void closeDispenseValves(int troughNumber) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    Serial.println(F("[ERROR] Invalid trough number provided to closeDispenseValves()"));
    return;
  }
  switch (troughNumber) {
    case 1:
      reagentValve1 = closeValve(reagentValve1);
      mediaValve1 = closeValve(mediaValve1);
      break;
    case 2:
      reagentValve2 = closeValve(reagentValve2);
      mediaValve2 = closeValve(mediaValve2);
      break;
    case 3:
      reagentValve3 = closeValve(reagentValve3);
      mediaValve3 = closeValve(mediaValve3);
      break;
    case 4:
      reagentValve4 = closeValve(reagentValve4);
      mediaValve4 = closeValve(mediaValve4);
      break;
  }
  Serial.print(F("[MESSAGE] Closed reagent and media valves for Trough "));
  Serial.println(troughNumber);
}

// ------------------------------------------------------------------
// stopDispenseOperation()
// ------------------------------------------------------------------
void stopDispenseOperation(int troughNumber, CommandCaller* caller) {
  // Close the dispense valves.
  closeDispenseValves(troughNumber);

  // Get the flow sensor associated with this trough.
  FlowSensor* sensor = flowSensors[troughNumber - 1];
  if (sensor) {
    caller->print(F("[MESSAGE] Trough "));
    caller->print(troughNumber);
    caller->print(F(" Dispense Stopped. Total Volume: "));
    caller->print(sensor->dispenseVolume, 1);
    caller->println(F(" mL."));

    // Stop measurement and reset the sensor's dispense volume.
    stopFlowSensorMeasurement(*sensor);
    sensor->dispenseVolume = 0.0;
  }

  // Mark the dispensing state as false.
  valveControls[troughNumber - 1].isDispensing = false;
}
