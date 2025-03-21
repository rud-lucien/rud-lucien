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
/**
 * Checks if the token starts with one of the valid command strings
 * defined in the Commander API tree.
 */
bool isCommandPrefix(const char* token) {
  const int numCommands = sizeof(API_tree) / sizeof(API_tree[0]);
  for (int i = 0; i < numCommands; i++) {
    const char* cmdName = API_tree[i].name; // use "name" as defined by the API macro
    size_t len = strlen(cmdName);
    if (strncmp(token, cmdName, len) == 0) {
      return true;
    }
  }
  return false;
}

void processMultipleCommands(char* commandLine, Stream* stream) {
  // Pointer to the current position in the command line.
  char* start = commandLine;
  // Temporary buffer for each command.
  char commandCopy[COMMAND_SIZE];

  while (*start) {
    // Find the next comma.
    char* comma = strchr(start, ',');
    
    // If a comma was found, compute the length of this command.
    size_t len;
    if (comma != NULL) {
      len = comma - start;
    } else {
      // No comma found; this is the last command.
      len = strlen(start);
    }

    // If the command length is too long, truncate it.
    if (len >= COMMAND_SIZE) {
      len = COMMAND_SIZE - 1;
    }

    // Copy the command substring into the buffer.
    strncpy(commandCopy, start, len);
    commandCopy[len] = '\0'; // Null-terminate

    // Trim any leading whitespace.
    char* trimmed = trimLeadingSpaces(commandCopy);

    // If there's anything in the token, execute it.
    if (strlen(trimmed) > 0) {
      Serial.print(F("[DEBUG] Token extracted: '"));
      Serial.print(trimmed);
      Serial.println(F("'"));
      commander.execute(trimmed, stream);
    }

    // If no comma was found, break out of the loop.
    if (comma == NULL) {
      break;
    }
    
    // Move start pointer to the character after the comma.
    start = comma + 1;
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
  
  // First, check if the pressure is already OK.
  if (isPressureOK(thresholdPressure)) {
    Serial.println(F("[MESSAGE] System is already pressurized."));
    return true;
  }
  
  // If not, set the valve.
  setPressureValve(valvePosition);
  
  // Now wait until the pressure reaches the threshold or timeout occurs.
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
  // If priming is in progress, stop it first.
  if (valveControls[troughNumber - 1].isPriming) {
    caller->print(F("[MESSAGE] Priming stopped for Trough "));
    caller->println(troughNumber);
    // Close the valves to stop priming.
    closeDispenseValves(troughNumber);
    // Reset priming flags.
    valveControls[troughNumber - 1].isPriming = false;
    valveControls[troughNumber - 1].manualControl = false;
  }
  
  // Now proceed with stopping dispensing.
  closeDispenseValves(troughNumber);
  
  FlowSensor* sensor = flowSensors[troughNumber - 1];
  if (sensor) {
    caller->print(F("[MESSAGE] Trough "));
    caller->print(troughNumber);
    caller->print(F(" Dispense Stopped. Total Volume: "));
    caller->print(sensor->dispenseVolume, 1);
    caller->println(F(" mL."));
    
    // Stop measurement and reset sensor's dispense volume.
    stopFlowSensorMeasurement(*sensor);
    resetFlowSensorDispenseVolume(*sensor);
  }
  
  valveControls[troughNumber - 1].isDispensing = false;
}

bool areDispenseValvesOpen(int troughNumber) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    return false;
  }
  switch (troughNumber) {
    case 1:
      return reagentValve1.isOpen && mediaValve1.isOpen;
    case 2:
      return reagentValve2.isOpen && mediaValve2.isOpen;
    case 3:
      return reagentValve3.isOpen && mediaValve3.isOpen;
    case 4:
      return reagentValve4.isOpen && mediaValve4.isOpen;
    default:
      return false;
  }
}

void enableManualControl(int index, CommandCaller* caller) {
  valveControls[index].manualControl = true;
  caller->print(F("[MESSAGE] Manual control enabled for trough "));
  caller->println(index + 1); // Trough numbers are 1-based.
}

void disableManualControl(int index, CommandCaller* caller) {
  valveControls[index].manualControl = false;
  caller->print(F("[MESSAGE] Manual control disabled for trough "));
  caller->println(index + 1);
}

// Enable Fill Mode
void enableFillMode(int troughNumber, CommandCaller* caller) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    return; // Prevent invalid access
  }

  valveControls[troughNumber - 1].fillMode = true;

  caller->print(F("[MESSAGE] Fill mode enabled for trough "));
  caller->println(troughNumber);
}

// Disable Fill Mode
void disableFillMode(int troughNumber, CommandCaller* caller) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    return; // Prevent invalid access
  }

  if (valveControls[troughNumber - 1].fillMode) {
    valveControls[troughNumber - 1].fillMode = false;
    caller->print(F("[MESSAGE] Fill mode disabled for trough "));
    caller->println(troughNumber);
  }
}

// Disable Fill Mode for All Troughs
void disableFillModeForAll(CommandCaller* caller) {
  for (int i = 1; i <= NUM_OVERFLOW_SENSORS; i++) {
    disableFillMode(i, caller);
  }
}

// Check if Fill Mode is Active
bool isFillModeActive(int troughNumber) {
  if (troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    return false;
  }
  return valveControls[troughNumber - 1].fillMode;
}

// Helper function to stop dispensing on a trough if it's active.
void stopDispensingIfActive(int troughNumber, CommandCaller* caller) {
  // Check if dispensing is in progress.
  if (valveControls[troughNumber - 1].isDispensing) {
    // Stop the dispense operation.
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispensing stopped for trough "));
    caller->println(troughNumber);
  }
}

// Helper function to check if the waste bottle for a given trough is full.
// Returns true if the bottle is full (and prints an error message); false otherwise.
bool isWasteBottleFullForTrough(int troughNumber, CommandCaller* caller) {
  // For troughs 1-2, use waste bottle sensor at index 0; for 3-4, use index 1.
  int bottleIndex = (troughNumber <= 2) ? 0 : 1;
  if (readBinarySensor(wasteBottleSensors[bottleIndex])) {
    caller->print(F("[ERROR] Waste bottle "));
    caller->print(bottleIndex + 1);  // Convert to 1-based numbering for display.
    caller->println(F(" is full. Cannot start drainage."));
    return true;
  }
  return false;
}

// Helper function to check for incompatible drainage conditions.
// Returns true if an incompatible drainage is detected (and prints an error message), false otherwise.
bool hasIncompatibleDrainage(int troughNumber, CommandCaller* caller) {
  // Check for incompatible drainage between troughs 1 and 2.
  if ((troughNumber == 1 && valveControls[1].isDraining) ||
      (troughNumber == 2 && valveControls[0].isDraining)) {
    caller->println(F("[ERROR] Troughs 1 and 2 cannot be drained simultaneously."));
    return true;
  }
  
  // Check for incompatible drainage between troughs 3 and 4.
  if ((troughNumber == 3 && valveControls[3].isDraining) ||
      (troughNumber == 4 && valveControls[2].isDraining)) {
    caller->println(F("[ERROR] Troughs 3 and 4 cannot be drained simultaneously."));
    return true;
  }
  
  return false;
}

// Helper function to validate the trough number.
// Returns true if valid; otherwise, prints an error and returns false.
bool validateTroughNumber(int troughNumber, CommandCaller* caller) {
  if (troughNumber < 1 || troughNumber > 4) {
    caller->println(F("[ERROR] Invalid trough number. Use 1-4."));
    return false;
  }
  return true;
}

// Helper function to stop dispensing on a trough when a fill command is issued.
void stopDispensingForFill(int troughNumber, CommandCaller* caller) {
  if (valveControls[troughNumber - 1].isDispensing) {
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispense operation for trough "));
    caller->print(troughNumber);
    caller->println(F(" stopped prematurely due to fill command."));
  }
}

// Helper function to stop priming on a trough when a fill command is issued.
void stopPrimingForFill(int troughNumber, CommandCaller* caller) {
  if (valveControls[troughNumber - 1].isPriming) {
    valveControls[troughNumber - 1].isPriming = false;
    closeDispenseValves(troughNumber);
    caller->print(F("[MESSAGE] Priming operation for trough "));
    caller->print(troughNumber);
    caller->println(F(" stopped prematurely due to fill command."));
  }
}

// Helper function to check if a valve is already primed.
// Returns true if already primed (and prints a message), false otherwise.
bool isValveAlreadyPrimed(int valveNumber, CommandCaller* caller) {
  // Note: valveNumber is assumed to be 1-based.
  if (readBinarySensor(reagentBubbleSensors[valveNumber - 1])) {
    caller->print(F("[MESSAGE] Valve "));
    caller->print(valveNumber);
    caller->println(F(" already primed."));
    return true;
  }
  return false;
}

// Helper function to validate a valve number (expected 1 to 4).
// Returns true if the valve number is valid; otherwise, prints an error message and returns false.
bool validateValveNumber(int valveNumber, CommandCaller* caller) {
  if (valveNumber < 1 || valveNumber > 4) {
    caller->println(F("[ERROR] Invalid valve number. Use 1-4."));
    return false;
  }
  return true;
}

// Helper function to set vacuum monitoring and close the main waste valve
// for a given trough.
void setVacuumMonitoringAndCloseMainValve(int troughNumber, CommandCaller* caller) {
  if (troughNumber <= 2) {
    globalVacuumMonitoring[0] = true; // Monitor waste bottle 1
    wasteValve1 = closeValve(wasteValve1);
  } else {
    globalVacuumMonitoring[1] = true; // Monitor waste bottle 2
    wasteValve2 = closeValve(wasteValve2);
  }
}











