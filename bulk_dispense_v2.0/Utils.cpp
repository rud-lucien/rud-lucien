#include "Utils.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Commands.h"
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include "CommandSession.h"
#include <Wire.h>
#include <string.h>
#include <ctype.h>

/************************************************************
 * Utils.cpp
 *
 * This file implements the utility functions for the Bulk Dispense
 * system, including:
 *   - Command execution wrappers with asynchronous action tags.
 *   - String trimming and multi-command processing.
 *   - Serial command handling.
 *   - Pressure check and I2C bus reset routines.
 *   - Valve control and manual/fill mode helper functions.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Command Session Utilities
// ============================================================

void executeCommandWithActionTags(const char* command, Stream* stream) {
  unsigned long actionStartTime = millis();
  stream->println(F("[ACTION START]"));
  
  // Execute the command.
  commander.execute(command, stream);
  
  // Calculate and print elapsed time.
  unsigned long actionDuration = millis() - actionStartTime;
  stream->print(F("[ACTION END] Duration: "));
  stream->print(actionDuration);
  stream->println(F(" ms"));
}

// ============================================================
// String Utilities
// ============================================================

char* trimLeadingSpaces(char* str) {
  while (*str && isspace(*str)) {
    str++;
  }
  return str;
}

// ============================================================
// Command Processing
// ============================================================

bool isCommandPrefix(const char* token) {
  const int numCommands = sizeof(API_tree) / sizeof(API_tree[0]);
  for (int i = 0; i < numCommands; i++) {
    const char* cmdName = API_tree[i].name;
    size_t len = strlen(cmdName);
    if (strncmp(token, cmdName, len) == 0) {
      return true;
    }
  }
  return false;
}

void processMultipleCommands(char* commandLine, Stream* stream) {
  char* start = commandLine;
  char commandCopy[COMMAND_SIZE];

  // Start a new asynchronous command session.
  startCommandSession(stream);

  while (*start) {
    char* comma = strchr(start, ',');
    size_t len = (comma != NULL) ? (size_t)(comma - start) : strlen(start);
    if (len >= COMMAND_SIZE) {
      len = COMMAND_SIZE - 1;
    }
    strncpy(commandCopy, start, len);
    commandCopy[len] = '\0';  // Null-terminate the substring
    char* trimmed = trimLeadingSpaces(commandCopy);
    if (strlen(trimmed) > 0) {
      Serial.print(F("[DEBUG] Token extracted: '"));
      Serial.print(trimmed);
      Serial.println(F("'"));
      
      // If the command is asynchronous, register it.
      if (isAsyncCommand(trimmed)) {
        registerAsyncCommand();
      }
      
      // Execute the command.
      commander.execute(trimmed, stream);
    }
    
    if (comma == NULL) break;
    start = comma + 1;
  }

  // End session if no asynchronous commands are pending.
  if (pendingAsyncCommands == 0) {
    endCommandSession(stream);
  }
}

void handleSerialCommands() {
  static char commandBuffer[COMMAND_SIZE];
  static uint8_t commandIndex = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      commandBuffer[commandIndex] = '\0';  // Null-terminate the command
      Serial.print(F("[COMMAND] Received: "));
      Serial.println(commandBuffer);
      processMultipleCommands(commandBuffer, &Serial);
      commandIndex = 0;  // Reset buffer index
    } else if (c != '\r') {  // Ignore carriage returns
      if (commandIndex < (COMMAND_SIZE - 1)) {
        commandBuffer[commandIndex++] = c;
      }
    }
  }
}

// ============================================================
// Pressure & I2C Utilities
// ============================================================

bool isPressureOK(float thresholdPressure) {
  float currentPressure = readPressure(pressureSensor);
  return (currentPressure >= thresholdPressure);
}

void setPressureValve(int valvePosition) {
  proportionalValve = setValvePosition(proportionalValve, valvePosition);
  Serial.print(F("[MESSAGE] Pressure valve set to "));
  Serial.print(valvePosition);
  Serial.println(F("%. Waiting for pressure stabilization..."));
}

bool checkAndSetPressure(float thresholdPressure, int valvePosition, unsigned long timeout) {
  unsigned long startTime = millis();
  if (isPressureOK(thresholdPressure)) {
    Serial.println(F("[MESSAGE] System is already pressurized."));
    return true;
  }
  setPressureValve(valvePosition);
  while (millis() - startTime < timeout) {
    if (isPressureOK(thresholdPressure)) {
      Serial.println(F("[MESSAGE] Pressure threshold reached."));
      return true;
    }
    delay(100);
  }
  Serial.print(F("[ERROR] Pressure threshold not reached. Current pressure: "));
  Serial.print(readPressure(pressureSensor));
  Serial.println(F(" psi. Operation aborted."));
  return false;
}

void resetI2CBus() {
  Serial.println(F("[MESSAGE] Resetting I2C bus..."));
  Wire.end();
  delay(100);
  Wire.begin();
}

// ============================================================
// Valve Control Utilities
// ============================================================

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

void stopDispenseOperation(int troughNumber, Stream* stream) {
  if (valveControls[troughNumber - 1].isPriming) {
    stream->print(F("[MESSAGE] Priming stopped for Trough "));
    stream->println(troughNumber);
    closeDispenseValves(troughNumber);
    valveControls[troughNumber - 1].isPriming = false;
    valveControls[troughNumber - 1].manualControl = false;
  }
  closeDispenseValves(troughNumber);
  FlowSensor* sensor = flowSensors[troughNumber - 1];
  if (sensor) {
    stream->print(F("[MESSAGE] Trough "));
    stream->print(troughNumber);
    stream->print(F(" Dispense Stopped. Total Volume: "));
    stream->print(sensor->dispenseVolume, 1);
    stream->println(F(" mL."));
    stopFlowSensorMeasurement(*sensor);
    resetFlowSensorDispenseVolume(*sensor);
  }
  valveControls[troughNumber - 1].isDispensing = false;
}


bool areDispenseValvesOpen(int troughNumber) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) return false;
  switch (troughNumber) {
    case 1: return reagentValve1.isOpen && mediaValve1.isOpen;
    case 2: return reagentValve2.isOpen && mediaValve2.isOpen;
    case 3: return reagentValve3.isOpen && mediaValve3.isOpen;
    case 4: return reagentValve4.isOpen && mediaValve4.isOpen;
    default: return false;
  }
}

// ============================================================
// Manual & Fill Mode Control Utilities
// ============================================================

void enableManualControl(int index, Stream* stream) {
  valveControls[index].manualControl = true;
  stream->print(F("[MESSAGE] Manual control enabled for trough "));
  stream->println(index + 1);
}

void disableManualControl(int index, Stream* stream) {
  valveControls[index].manualControl = false;
  stream->print(F("[MESSAGE] Manual control disabled for trough "));
  stream->println(index + 1);
}

void enableFillMode(int troughNumber, Stream* stream) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) return;
  valveControls[troughNumber - 1].fillMode = true;
  stream->print(F("[MESSAGE] Fill mode enabled for trough "));
  stream->println(troughNumber);
}

void disableFillMode(int troughNumber, Stream* stream) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) return;
  if (valveControls[troughNumber - 1].fillMode) {
    valveControls[troughNumber - 1].fillMode = false;
    stream->print(F("[MESSAGE] Fill mode disabled for trough "));
    stream->println(troughNumber);
  }
}

void disableFillModeForAll(Stream* stream) {
  for (int i = 1; i <= NUM_OVERFLOW_SENSORS; i++) {
    disableFillMode(i, stream);
  }
}

bool isFillModeActive(int troughNumber) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) return false;
  return valveControls[troughNumber - 1].fillMode;
}

// ============================================================
// Helper Functions for Dispensing, Draining, and Priming
// ============================================================

void stopDispensingIfActive(int troughNumber, Stream* stream) {
  if (valveControls[troughNumber - 1].isDispensing) {
    stopDispenseOperation(troughNumber, stream);
    stream->print(F("[MESSAGE] Dispensing stopped for trough "));
    stream->println(troughNumber);
  }
}

bool isWasteBottleFullForTrough(int troughNumber, Stream* stream) {
  int bottleIndex = (troughNumber <= 2) ? 0 : 1;
  if (readBinarySensor(wasteBottleSensors[bottleIndex])) {
    stream->print(F("[ERROR] Waste bottle "));
    stream->print(bottleIndex + 1);
    stream->println(F(" is full. Cannot start drainage."));
    return true;
  }
  return false;
}

bool hasIncompatibleDrainage(int troughNumber, Stream* stream) {
  if ((troughNumber == 1 && valveControls[1].isDraining) ||
      (troughNumber == 2 && valveControls[0].isDraining)) {
    stream->println(F("[ERROR] Troughs 1 and 2 cannot be drained simultaneously."));
    return true;
  }
  if ((troughNumber == 3 && valveControls[3].isDraining) ||
      (troughNumber == 4 && valveControls[2].isDraining)) {
    stream->println(F("[ERROR] Troughs 3 and 4 cannot be drained simultaneously."));
    return true;
  }
  return false;
}

bool validateTroughNumber(int troughNumber, Stream* stream) {
  if (troughNumber < 1 || troughNumber > 4) {
    stream->println(F("[ERROR] Invalid trough number. Use 1-4."));
    return false;
  }
  return true;
}

void stopDispensingForFill(int troughNumber, Stream* stream) {
  if (valveControls[troughNumber - 1].isDispensing) {
    stopDispenseOperation(troughNumber, stream);
    stream->print(F("[MESSAGE] Dispense operation for trough "));
    stream->print(troughNumber);
    stream->println(F(" stopped prematurely due to fill command."));
  }
}

void stopPrimingForFill(int troughNumber, Stream* stream) {
  if (valveControls[troughNumber - 1].isPriming) {
    valveControls[troughNumber - 1].isPriming = false;
    closeDispenseValves(troughNumber);
    stream->print(F("[MESSAGE] Priming operation for trough "));
    stream->print(troughNumber);
    stream->println(F(" stopped prematurely due to fill command."));
  }
}

bool isValveAlreadyPrimed(int valveNumber, Stream* stream) {
  if (readBinarySensor(reagentBubbleSensors[valveNumber - 1])) {
    stream->print(F("[MESSAGE] Valve "));
    stream->print(valveNumber);
    stream->println(F(" already primed."));
    return true;
  }
  return false;
}

bool validateValveNumber(int valveNumber, Stream* stream) {
  if (valveNumber < 1 || valveNumber > 4) {
    stream->println(F("[ERROR] Invalid valve number. Use 1-4."));
    return false;
  }
  return true;
}

void setVacuumMonitoringAndCloseMainValve(int troughNumber, Stream* stream) {
  if (troughNumber <= 2) {
    globalVacuumMonitoring[0] = true;
    wasteValve1 = closeValve(wasteValve1);
  } else {
    globalVacuumMonitoring[1] = true;
    wasteValve2 = closeValve(wasteValve2);
  }
}

void abortAllAutomatedOperations(Stream* stream) {
  // Abort operations on each trough.
  for (int trough = 1; trough <= NUM_OVERFLOW_SENSORS; trough++) {
    if (valveControls[trough - 1].isDispensing) {
      stopDispenseOperation(trough, stream);
    }
    if (valveControls[trough - 1].isPriming) {
      stopPrimingForFill(trough, stream);
    }
    if (valveControls[trough - 1].fillMode) {
      disableFillMode(trough, stream);
    }
    if (valveControls[trough - 1].isDraining) {
      valveControls[trough - 1].isDraining = false;
      // Optionally, close any associated waste valves here.
    }
  }
  
  stream->println(F("[ERROR] Enclosure liquid detected. Automated operations halted. Resolve the leak before proceeding."));
  stream->println(F("[MESSAGE] All automated operations aborted due to enclosure leak."));
  
  if (commandSessionActive) {
    endCommandSession(stream);
  }
}




