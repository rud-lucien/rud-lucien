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
#include "CommandManager.h"
#include "SystemMonitor.h"

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
  
  // Set flag to indicate we are processing the command line.
  commandLineBeingProcessed = true;
  
  // Start the session using our command manager.
  cm_startSession(stream);  // This prints "[ACTION START]"
  
  while (*start) {
    char* comma = strchr(start, ',');
    size_t len = (comma != NULL) ? (size_t)(comma - start) : strlen(start);
    if (len >= COMMAND_SIZE) {
      len = COMMAND_SIZE - 1;
    }
    strncpy(commandCopy, start, len);
    commandCopy[len] = '\0';  // Null-terminate
    char* trimmed = trimLeadingSpaces(commandCopy);
    if (strlen(trimmed) > 0) {
      Serial.print(F("[DEBUG] Token extracted: '"));
      Serial.print(trimmed);
      Serial.println(F("'"));
      
      resetAsyncFlagsForCommand(trimmed);
      
      if (isAsyncCommand(trimmed)) {
        cm_registerCommand();
        // Dispatch asynchronous command.
        // Its callback must call cm_commandCompleted(stream) when done.
        commander.execute(trimmed, stream);
      } else {
        // For synchronous commands, register and execute.
        cm_registerCommand();
        commander.execute(trimmed, stream);
        // Synchronous commands finish immediately (even if error),
        // so complete them here.
        cm_commandCompleted(stream);
      }
    }
    if (comma == NULL)
      break;
    start = comma + 1;
  }
  
  // Finished processing the command line.
  commandLineBeingProcessed = false;
  
  // If there are no pending commands after processing, end the session.
  if (cm_getPendingCommands() <= 0 && cm_isSessionActive()) {
    cm_endSession(stream);
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
    asyncCommandCompleted(&Serial);
    return true;
  }
  if ((troughNumber == 3 && valveControls[3].isDraining) ||
      (troughNumber == 4 && valveControls[2].isDraining)) {
    stream->println(F("[ERROR] Troughs 3 and 4 cannot be drained simultaneously."));
    asyncCommandCompleted(&Serial);
    return true;
  }
  return false;
}

bool validateTroughNumber(int troughNumber, Stream* stream) {
  if (troughNumber < 1 || troughNumber > 4) {
    stream->println(F("[ERROR] Invalid trough number."));
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
    stream->println(F("[ERROR] Invalid valve number."));
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


// Helper function to force-stop a drain operation for a given trough.
// In the event of an enclosure leak, we want to immediately close
// both the primary and secondary drain valves.
void stopDrainOperation(int trough, Stream* stream) {
  stream->print(F("[MESSAGE] Stopping drain operation for trough "));
  stream->println(trough);

  // Force-close the primary drain valve(s) for this trough.
  // (Adjust these based on your hardware mapping.)
  switch(trough) {
    case 1:
      wasteValve1 = closeValve(wasteValve1);
      break;
    case 2:
      wasteValve2 = closeValve(wasteValve2);
      break;
    case 3:
      wasteValve3 = closeValve(wasteValve3);
      break;
    case 4:
      wasteValve4 = closeValve(wasteValve4);
      break;
    default:
      stream->println(F("[ERROR] Invalid trough number in stopDrainOperation."));
      return;
  }
  
  // Also, force-close any secondary drain valve(s) that might be used.
  // For example, if troughs 1 and 2 share a secondary valve, and 3 and 4 share another:
  if (trough == 1 || trough == 2) {
    // Force close the secondary valve for troughs 1 and 2.
    // (Even if under normal circumstances you might open it to equalize pressure.)
    wasteValve3 = closeValve(wasteValve3);
  }
  else if (trough == 3 || trough == 4) {
    wasteValve4 = closeValve(wasteValve4);
  }
  
  // Reset the drain timer.
  valveControls[trough - 1].drainStartTime = 0;
}


void abortAllAutomatedOperations(Stream* stream) {
  // Abort operations on each trough.
  for (int trough = 1; trough <= NUM_OVERFLOW_SENSORS; trough++) {
    int index = trough - 1;
    // If dispensing, stop it.
    if (valveControls[index].isDispensing) {
      stopDispenseOperation(trough, stream);
    }
    // If priming, stop it.
    if (valveControls[index].isPriming) {
      stopPrimingForFill(trough, stream);
    }
    // If in fill mode, disable it.
    if (valveControls[index].fillMode) {
      disableFillMode(trough, stream);
    }
    // If draining, stop the drain process.
    if (valveControls[index].isDraining) {
      stopDrainOperation(trough, stream);
    }
    // Stop asynchronous sensor operations.
    stopFlowSensorMeasurement(*flowSensors[index]);
    resetFlowSensorDispenseVolume(*flowSensors[index]);

    // Reset the trough control flags.
    valveControls[index].isDispensing = false;
    valveControls[index].isPriming = false;
    valveControls[index].fillMode = false;
    valveControls[index].isDraining = false;
    valveControls[index].manualControl = false;
    valveControls[index].targetVolume = -1;
    valveControls[index].lastFlowCheckTime = 0;
    valveControls[index].lastFlowChangeTime = 0;
    
    // Reset any asynchronous timers associated with this trough.
    valveControls[index].drainStartTime = 0;
  }

  // Also reset monitor-specific state (prime, fill, waste, enclosure leak) to get a clean slate.
  resetPrimeMonitorState();
  resetFillMonitorState();
  resetWasteMonitorState();
  resetEnclosureLeakMonitorState();
  
  stream->println(F("[ERROR] Enclosure liquid detected. Automated operations halted. Resolve the leak before proceeding."));
  stream->println(F("[MESSAGE] All automated operations aborted due to enclosure leak."));
  
  // Abort the command session so that an [ACTION END] message is printed.
  if (cm_isSessionActive()) {
    cm_abortSession(stream);
  }
}



String getOverallTroughState() {
  bool allIdle = true;
  String stateSummary = "";
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    String troughState = "";
    // If none of the operations are active, mark as "Idle".
    if (!valveControls[i].isDispensing && 
        !valveControls[i].isPriming && 
        !valveControls[i].fillMode && 
        !valveControls[i].isDraining) {
      troughState = "Idle";
    } else {
      allIdle = false;
      bool first = true;
      // If multiple operations are active, list them separated by commas.
      if (valveControls[i].isDispensing) {
        troughState += "Dispensing";
        first = false;
      }
      if (valveControls[i].isDraining) {
        if (!first) troughState += ", ";
        troughState += "Draining";
        first = false;
      }
      if (valveControls[i].fillMode) {
        if (!first) troughState += ", ";
        troughState += "Filling";
        first = false;
      }
      if (valveControls[i].isPriming) {
        if (!first) troughState += ", ";
        troughState += "Priming";
      }
    }
    stateSummary += "T" + String(i + 1) + ": " + troughState;
    if (i < NUM_OVERFLOW_SENSORS - 1) {
      stateSummary += " | ";
    }
  }
  if (allIdle) {
    return "Idle";
  } else {
    return "Active - " + stateSummary;
  }
}


String getOpenValvesString(bool v1, bool v2, bool v3, bool v4) {
  String openList = "";
  if (v1) { openList += "Valve 1"; }
  if (v2) { 
    if (openList.length() > 0) { openList += " & "; }
    openList += "Valve 2"; 
  }
  if (v3) { 
    if (openList.length() > 0) { openList += " & "; }
    openList += "Valve 3"; 
  }
  if (v4) { 
    if (openList.length() > 0) { openList += " & "; }
    openList += "Valve 4"; 
  }
  if (openList.length() == 0) {
    openList = "None open";
  }
  return openList;
}


// In your Utils.cpp (or the file where you defined resetAsyncFlagsForTrough)
void resetAsyncFlagsForTrough(int troughNumber) {
    if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS)
        return;
    
    // If the trough is busy with any asynchronous operation, do not reset its flags.
    // (Assuming valveControls is a global array and that these booleans indicate active operations.)
    if (valveControls[troughNumber - 1].isDispensing ||
        valveControls[troughNumber - 1].isPriming ||
        valveControls[troughNumber - 1].fillMode ||
        valveControls[troughNumber - 1].isDraining) {
        // Trough is busy – do not reset its async flags.
        return;
    }
    
    // Otherwise, reset only this trough’s async flags.
    dispenseAsyncCompleted[troughNumber - 1] = false;
    drainAsyncCompleted[troughNumber - 1] = false;
    primeAsyncCompleted[troughNumber - 1] = false;
}


// You can also add a helper that parses the trough number from a token.
// For example, if your token is "D 2 10", this will extract the 2.
void resetAsyncFlagsForCommand(const char* token) {
    int troughNumber = 0;
    // This assumes the command format starts with a letter followed by the trough number,
    // e.g. "D 1" or "P 2" etc.
    if (sscanf(token, "%*c %d", &troughNumber) == 1) {
        resetAsyncFlagsForTrough(troughNumber);
    }
}


// Example implementation: assumes OnOffValve has a bool member "isOpen"
// (if your valve type is defined differently, adjust accordingly)
bool isValveClosed(const OnOffValve &valve) {
  return !valve.isOpen;  
}

bool areAllValvesClosedForTrough(int troughNumber) {
  // Adjust the mapping if your trough-to-valve assignment is different.
  switch(troughNumber) {
    case 1: 
      return (isValveClosed(reagentValve1) &&
              isValveClosed(mediaValve1) &&
              isValveClosed(wasteValve1));
    case 2:
      return (isValveClosed(reagentValve2) &&
              isValveClosed(mediaValve2) &&
              isValveClosed(wasteValve2));
    case 3:
      return (isValveClosed(reagentValve3) &&
              isValveClosed(mediaValve3) &&
              isValveClosed(wasteValve3));
    case 4:
      return (isValveClosed(reagentValve4) &&
              isValveClosed(mediaValve4) &&
              isValveClosed(wasteValve4));
    default:
      return true;
  }
}

void set_valve_state(OnOffValve &valveVar, bool state, int valveNumber, ValveType type, CommandCaller *caller) {
  if (state) {
    valveVar = openValve(valveVar);  // openValve() returns an OnOffValve
    // When opening a valve manually, enable manual control for that trough.
    enableManualControl(valveNumber - 1, caller);  
  } else {
    valveVar = closeValve(valveVar); // closeValve() returns an OnOffValve
    // When closing, check if all valves in that trough are closed; if so, clear the manual flag.
    updateTroughManualControlFlag(type, valveNumber, caller);
  }
}

void updateTroughManualControlFlag(ValveType type, int valveNumber, CommandCaller *caller) {
  // Here we assume that the "valveNumber" maps directly to the trough number.
  int troughNumber = valveNumber;  // Adjust if your mapping is different.
  if (areAllValvesClosedForTrough(troughNumber)) {
    // All valves for this trough are closed—disable manual control.
    disableManualControl(troughNumber - 1, caller);
  }
}


