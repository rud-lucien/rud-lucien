#include "Commands.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Logging.h"
#include "Utils.h"
#include <Controllino.h>
#include "CommandSession.h"

/************************************************************
 * Commands.cpp
 * 
 * This file implements all command functions for the Bulk 
 * Dispense system. Each command function processes its 
 * arguments, interacts with hardware and sensors, and logs 
 * messages accordingly. The global command tree is also defined
 * here.
 *
 * Commands include:
 *   LF: Set log frequency
 *   FN: Manual fan control (on/off)
 *   FNAUTO: Enable fan auto control
 *   R, M, W: Set valve states
 *   PV, CALPV: Pressure valve operations
 *   STARTFSM, STOPFSM, RFS, RTF, RESETI2C: Flow sensor & I2C operations
 *   D, STOPD: Dispense operations
 *   P: Prime valves
 *   F: Fill reagent
 *   DT: Drain trough
 *   SDT: Stop drain trough
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Command Function Definitions
// ============================================================

void cmd_set_log_frequency(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int newInterval = -1;
  if (sscanf(localArgs, "%d", &newInterval) == 1 && newInterval > 0) {
    logging.logInterval = newInterval;
    logging.previousLogTime = millis();  // <-- Reset timer to start from now
    caller->print(F("[MESSAGE] Log frequency set to "));
    caller->print(newInterval);
    caller->println(F(" ms"));
  } else {
    caller->println(F("[ERROR] Invalid log frequency. Use: LF <positive number>"));
  }
}


void cmd_fan(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int stateInt = -1;
  if (sscanf(localArgs, "%d", &stateInt) == 1 && (stateInt == 0 || stateInt == 1)) {
    bool state = (stateInt == 1);
    digitalWrite(fan.relayPin, state ? HIGH : LOW);
    Serial.print(F("[MESSAGE] Fan turned "));
    Serial.println(state ? F("ON") : F("OFF"));
    fanAutoMode = false;
    caller->println(F("[MESSAGE] Fan manual override active. Use FNAUTO to re-enable auto control."));
  } else {
    caller->println(F("[ERROR] Invalid fan command. Use: FN <0/1>"));
  }
}

void cmd_fan_auto(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Check if fan auto mode is already enabled.
  if (fanAutoMode) {
    caller->println(F("[MESSAGE] Fan auto control is already enabled."));
  } else {
    caller->println(F("[MESSAGE] Fan auto control re-enabled."));
    fanAutoMode = true;
  }

  // Immediately update the fan state based on the current temperature:
  TempHumidity th = readTempHumidity();
  if (th.valid) {
    if (th.temperature > ENCLOSURE_TEMP_SETPOINT) {
      setFanState(fan, true);  // Prints: "[MESSAGE] Fan state set to ON"
    } else {
      setFanState(fan, false); // Prints: "[MESSAGE] Fan state set to OFF"
    }
  } else {
    caller->println(F("[ERROR] Failed to read enclosure temperature for fan auto update."));
  }
}



void cmd_set_reagent_valve(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_REAGENT_VALVES &&
      (valveState == 0 || valveState == 1)) {
    disableFillMode(valveNumber, caller);
    bool state = (valveState == 1);
    if (state) {
      enableManualControl(valveNumber - 1, caller);
    }
    caller->print(F("[MESSAGE] Reagent valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");
    switch (valveNumber) {
      case 1: reagentValve1 = state ? openValve(reagentValve1) : closeValve(reagentValve1); break;
      case 2: reagentValve2 = state ? openValve(reagentValve2) : closeValve(reagentValve2); break;
      case 3: reagentValve3 = state ? openValve(reagentValve3) : closeValve(reagentValve3); break;
      case 4: reagentValve4 = state ? openValve(reagentValve4) : closeValve(reagentValve4); break;
    }
  } else {
    caller->println(F("[ERROR] Invalid reagent valve command. Use: R <1-4> <0/1>"));
  }
}

void cmd_set_media_valve(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_MEDIA_VALVES &&
      (valveState == 0 || valveState == 1)) {
    disableFillMode(valveNumber, caller);
    bool state = (valveState == 1);
    if (state) {
      enableManualControl(valveNumber - 1, caller);
    }
    caller->print(F("[MESSAGE] Media valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");
    switch (valveNumber) {
      case 1: mediaValve1 = state ? openValve(mediaValve1) : closeValve(mediaValve1); break;
      case 2: mediaValve2 = state ? openValve(mediaValve2) : closeValve(mediaValve2); break;
      case 3: mediaValve3 = state ? openValve(mediaValve3) : closeValve(mediaValve3); break;
      case 4: mediaValve4 = state ? openValve(mediaValve4) : closeValve(mediaValve4); break;
    }
  } else {
    caller->println(F("[ERROR] Invalid media valve command. Use: M <1-4> <0/1>"));
  }
}

void cmd_set_waste_valve(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_WASTE_VALVES &&
      (valveState == 0 || valveState == 1)) {
    disableFillMode(valveNumber, caller);
    bool state = (valveState == 1);
    if (state) {
      enableManualControl(valveNumber - 1, caller);
    }
    caller->print(F("[MESSAGE] Waste valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");
    switch (valveNumber) {
      case 1: wasteValve1 = state ? openValve(wasteValve1) : closeValve(wasteValve1); break;
      case 2: wasteValve2 = state ? openValve(wasteValve2) : closeValve(wasteValve2); break;
      case 3: wasteValve3 = state ? openValve(wasteValve3) : closeValve(wasteValve3); break;
      case 4: wasteValve4 = state ? openValve(wasteValve4) : closeValve(wasteValve4); break;
    }
  } else {
    caller->println(F("[ERROR] Invalid waste valve command. Use: W <1-4> <0/1>"));
  }
}

void cmd_set_pressure_valve(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int percentage = -1;
  if (sscanf(localArgs, "%d", &percentage) == 1 && percentage >= 0 && percentage <= 100) {
    proportionalValve = setValvePosition(proportionalValve, (float)percentage);
    caller->print(F("[MESSAGE] Pressure valve set to "));
    caller->print(percentage);
    caller->println(F("%."));
  } else {
    caller->println(F("[ERROR] Invalid value for pressure valve. Use a percentage between 0 and 100."));
  }
}

void cmd_calibrate_pressure_valve(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  caller->println(F("[MESSAGE] Calibrating pressure valve, please wait..."));
  calibrateProportionalValve();
  caller->println(F("[MESSAGE] Pressure valve calibration complete."));
}

void cmd_start_flow_sensor_manually(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= NUM_FLOW_SENSORS) {
    disableFillMode(sensorNumber, caller);
    enableManualControl(sensorNumber - 1, caller);
    
    FlowSensor* sensor = flowSensors[sensorNumber - 1];
    if (!sensor) {
      caller->print(F("[ERROR] Flow Sensor "));
      caller->print(sensorNumber);
      caller->println(F(" not found."));
      return;
    }
    if (startFlowSensorMeasurement(*sensor)) {
      caller->print(F("[MESSAGE] Manually started measurement for Flow Sensor "));
      caller->println(sensorNumber);
    } else {
      caller->print(F("[ERROR] Failed to start Flow Sensor "));
      caller->println(sensorNumber);
    }
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: STARTFSM <1-4>"));
  }
}

void cmd_stop_flow_sensor_manually(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= NUM_FLOW_SENSORS) {
    disableFillMode(sensorNumber - 1, caller);
    disableManualControl(sensorNumber - 1, caller);
    
    FlowSensor* sensor = flowSensors[sensorNumber - 1];
    if (!sensor) {
      caller->print(F("[ERROR] Flow Sensor "));
      caller->print(sensorNumber);
      caller->println(F(" not found."));
      return;
    }
    if (stopFlowSensorMeasurement(*sensor)) {
      caller->print(F("[MESSAGE] Manually stopped Flow Sensor "));
      caller->println(sensorNumber);
    } else {
      caller->print(F("[ERROR] Failed to stop Flow Sensor "));
      caller->println(sensorNumber);
    }
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: STOPFSM <1-4>"));
  }
}

void cmd_reset_flow_dispense(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS) {
    disableFillMode(sensorNumber, caller);
    enableManualControl(sensorNumber, caller);

    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorDispenseVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset dispense volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RFS <1-4>"));
  }
}

void cmd_reset_flow_total(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS) {
    disableFillMode(sensorNumber, caller);
    enableManualControl(sensorNumber, caller);

    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorTotalVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset total volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RTF <0-3>"));
  }
}

void cmd_reset_i2c(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    disableFillMode(i + 1, caller);
    enableManualControl(i, caller);
  }
  
  caller->println(F("[MESSAGE] Manual I2C bus reset initiated."));
  resetI2CBus();
  caller->println(F("[MESSAGE] I2C bus reset complete."));
}

void cmd_dispense_reagent(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  if (globalEnclosureLiquidError) {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int troughNumber = -1;
  float requestedVolume = -1.0;
  const float MIN_VOLUME = 1.0;
  const float MAX_VOLUME = 200.0;

  caller->print(F("[MESSAGE] Received command: D "));
  caller->println(localArgs);

  char* token = strtok(localArgs, " ");
  if (token != NULL) {
    troughNumber = atoi(token);
    token = strtok(NULL, " ");
    if (token != NULL) {
      requestedVolume = atof(token);
    } else {
      requestedVolume = -1.0;  // Continuous mode if no volume provided.
    }
  } else {
    caller->println(F("[ERROR] Invalid command format. Use: D <1-4> [volume]"));
    return;
  }

  caller->print(F("[MESSAGE] Dispense command received for Trough "));
  caller->print(troughNumber);
  caller->print(F(" with requested volume "));
  caller->println(requestedVolume);

  if (!validateTroughNumber(troughNumber, caller)) {
    return;
  }

  disableFillMode(troughNumber, caller);

  if (valveControls[troughNumber - 1].isDispensing) {
    caller->print(F("[WARNING] A dispense is already in progress for Trough "));
    caller->println(troughNumber);
    caller->println(F("Use STOPD <trough number> to stop it first."));
    return;
  }
  if (requestedVolume > 0) {
    if (requestedVolume < MIN_VOLUME) {
      caller->print(F("[ERROR] Requested volume too low. Minimum: "));
      caller->print(MIN_VOLUME);
      caller->println(F(" mL."));
      return;
    } else if (requestedVolume > MAX_VOLUME) {
      caller->print(F("[ERROR] Requested volume too high. Maximum: "));
      caller->print(MAX_VOLUME);
      caller->println(F(" mL."));
      return;
    }
  }

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Dispense aborted."));
    return;
  }

  if (readBinarySensor(overflowSensors[troughNumber - 1])) {
    caller->print(F("[ERROR] Cannot dispense: Overflow detected for Trough "));
    caller->println(troughNumber);
    return;
  }

  FlowSensor* sensor = flowSensors[troughNumber - 1];
  if (!sensor) {
    caller->print(F("[ERROR] No flow sensor found for Trough "));
    caller->println(troughNumber);
    return;
  }

  if (!startFlowSensorMeasurement(*sensor)) {
    caller->print(F("[ERROR] Failed to start flow sensor for Trough "));
    caller->println(troughNumber);
    return;
  }

  caller->print(F("[MESSAGE] Flow sensor measurement started for Trough "));
  caller->println(troughNumber);

  openDispenseValves(troughNumber);
  caller->print(F("[MESSAGE] Dispensing started for Trough "));
  caller->println(troughNumber);

  valveControls[troughNumber - 1].isDispensing = true;
  valveControls[troughNumber - 1].targetVolume = requestedVolume;

  // Mark this dispensing operation as asynchronous.
  dispenseAsyncCompleted[troughNumber - 1] = false;
}

void cmd_stop_dispense(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int troughNumber = -1;
  bool stopAll = false;

  if (strncmp(localArgs, "all", 3) == 0) {
    stopAll = true;
  } else if (sscanf(localArgs, "%d", &troughNumber) != 1 || troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    caller->println(F("[ERROR] Invalid trough number. Use STOPD <1-4> or STOPD all."));
    return;
  }

  if (stopAll) {
    caller->println(F("[MESSAGE] Stopping all dispensing operations..."));
    disableFillModeForAll(caller);
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      stopDispenseOperation(i + 1, caller);
    }
    caller->println(F("[MESSAGE] All dispensing operations stopped."));
  } else {
    disableFillMode(troughNumber, caller);
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispensing stopped for Trough "));
    caller->println(troughNumber);
  }
}

void cmd_prime_valves(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  if (globalEnclosureLiquidError) {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int localValveNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &localValveNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for prime command. Use: P <valve number>"));
    return;
  }
  
  if (!validateValveNumber(localValveNumber, caller)) {
    return;
  }

  disableFillMode(localValveNumber, caller);

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Prime aborted."));
    return;
  }

  if (isValveAlreadyPrimed(localValveNumber, caller)) {
    return;
  }

  openDispenseValves(localValveNumber);
  valveControls[localValveNumber - 1].isPriming = true;
  caller->print(F("[MESSAGE] Priming started for valve "));
  caller->println(localValveNumber);
}

void cmd_fill_reagent(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  if (globalEnclosureLiquidError) {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int troughNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for fill command. Use: F <valve number>"));
    return;
  }
  
  if (!validateTroughNumber(troughNumber, caller)) {
    return;
  }
  
  stopDispensingForFill(troughNumber, caller);
  stopPrimingForFill(troughNumber, caller);

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Fill aborted."));
    return;
  }
  
  resetFlowSensorDispenseVolume(*flowSensors[troughNumber - 1]);
  openDispenseValves(troughNumber);
  enableFillMode(troughNumber, caller);
  caller->print(F("[MESSAGE] Fill mode enabled for trough "));
  caller->println(troughNumber);

  // For fill mode, immediately complete the command.
  asyncCommandCompleted(&Serial);
}

void cmd_drain_trough(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  if (globalEnclosureLiquidError) {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int troughNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for drain command. Use: DT <trough number>"));
    return;
  }
  
  if (!validateTroughNumber(troughNumber, caller)) {
    return;
  }
  
  if (isWasteBottleFullForTrough(troughNumber, caller)) {
    return;
  }
    
  if (hasIncompatibleDrainage(troughNumber, caller)) {
    return;
  }
  
  stopDispensingIfActive(troughNumber, caller);
  disableFillMode(troughNumber, caller);
  valveControls[troughNumber - 1].isDraining = true;
  drainAsyncCompleted[troughNumber - 1] = false;
  
  switch (troughNumber) {
    case 1:
      wasteValve1 = openValve(wasteValve1);
      wasteValve3 = openValve(wasteValve3);
      caller->println(F("[MESSAGE] Draining trough 1... Waste valve 1 opened, waste valve 3 opened."));
      break;
    case 2:
      wasteValve1 = openValve(wasteValve1);
      wasteValve3 = closeValve(wasteValve3);
      caller->println(F("[MESSAGE] Draining trough 2... Waste valve 1 opened, waste valve 3 closed."));
      break;
    case 3:
      wasteValve2 = openValve(wasteValve2);
      wasteValve4 = openValve(wasteValve4);
      caller->println(F("[MESSAGE] Draining trough 3... Waste valve 2 opened, waste valve 4 opened."));
      break;
    case 4:
      wasteValve2 = openValve(wasteValve2);
      wasteValve4 = closeValve(wasteValve4);
      caller->println(F("[MESSAGE] Draining trough 4... Waste valve 2 opened, waste valve 4 closed."));
      break;
    default:
      caller->println(F("[ERROR] Invalid trough number. Use 1-4."));
      return;
  }
  // The asynchronous completion for drain will be signaled in monitorWasteSensors().
}

void cmd_stop_drain_trough(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  if (strncmp(localArgs, "all", 3) == 0) {
    for (int i = 0; i < 4; i++) {
      valveControls[i].isDraining = false;
    }
    globalVacuumMonitoring[0] = true;
    globalVacuumMonitoring[1] = true;
    wasteValve1 = closeValve(wasteValve1);
    wasteValve2 = closeValve(wasteValve2);
    caller->println(F("[MESSAGE] Draining stopped for all troughs. Waste valves closed."));
    return;
  }

  int troughNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1 || troughNumber < 1 || troughNumber > 4) {
    caller->println(F("[ERROR] Invalid arguments. Use: SDT <1-4> or SDT all."));
    return;
  }
  if (!validateTroughNumber(troughNumber, caller)) {
    return;
  }
  valveControls[troughNumber - 1].isDraining = false;
  setVacuumMonitoringAndCloseMainValve(troughNumber, caller);
  
  switch (troughNumber) {
    case 1:
      wasteValve3 = openValve(wasteValve3);
      caller->println(F("[MESSAGE] Draining stopped for trough 1."));
      break;
    case 2:
      wasteValve3 = closeValve(wasteValve3);
      caller->println(F("[MESSAGE] Draining stopped for trough 2."));
      break;
    case 3:
      wasteValve4 = openValve(wasteValve4);
      caller->println(F("[MESSAGE] Draining stopped for trough 3."));
      break;
    case 4:
      wasteValve4 = closeValve(wasteValve4);
      caller->println(F("[MESSAGE] Draining stopped for trough 4."));
      break;
    default:
      caller->println(F("[ERROR] Invalid trough number. Use 1-4 or 'all'."));
      return;
  }
  // Note: The completion of the stop-drain operation is determined by the vacuum release.
}

// ============================================================
// Global Command Tree and Commander Object
// ============================================================
Commander commander;

Commander::systemCommand_t API_tree[] = {
  systemCommand("LF", "Set log frequency: LF <ms>", cmd_set_log_frequency),
  systemCommand("FN", "Fan: FN <0/1> (0 = off, 1 = on)", cmd_fan),
  systemCommand("FNAUTO", "Enable fan auto control", cmd_fan_auto),
  systemCommand("R", "Reagent valve: R <1-4> <0/1>", cmd_set_reagent_valve),
  systemCommand("M", "Media valve: M <1-4> <0/1>", cmd_set_media_valve),
  systemCommand("W", "Waste valve: W <1-4> <0/1>", cmd_set_waste_valve),
  systemCommand("PV", "Pressure valve: PV <percentage>", cmd_set_pressure_valve),
  systemCommand("CALPV", "Calibrate pressure valve", cmd_calibrate_pressure_valve),
  systemCommand("STARTFSM", "Manually start flow sensor measurement: STARTFSM <1-4>", cmd_start_flow_sensor_manually),
  systemCommand("STOPFSM", "Manually stop flow sensor measurement: STOPFSM <1-4>", cmd_stop_flow_sensor_manually),
  systemCommand("RF", "Reset flow sensor dispense volume: RFS <0-3>", cmd_reset_flow_dispense),
  systemCommand("RTF", "Reset total volume for Flow Sensor: RTF <0-3>", cmd_reset_flow_total),
  systemCommand("RESETI2C", "Manually reset the I2C bus", cmd_reset_i2c),
  systemCommand("D", "Dispense reagent: D <1-4> [volume] (continuous if omitted)", cmd_dispense_reagent),
  systemCommand("STOPD", "Stop dispensing: STOPD <1-4> or STOPD all", cmd_stop_dispense),
  systemCommand("P", "Prime valves: P <1-4>", cmd_prime_valves),
  systemCommand("F", "Fill reagent: F <1-4>", cmd_fill_reagent),
  systemCommand("DT", "Drain trough: DT <1-4>", cmd_drain_trough),
  systemCommand("SDT", "Stop draining trough: SDT <1-4> or SDT all", cmd_stop_drain_trough)
};
