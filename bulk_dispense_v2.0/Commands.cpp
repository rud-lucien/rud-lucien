#include "Commands.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Logging.h"
#include "Utils.h"
#include <Controllino.h>



// ------------------------------------------------------------------
// Command Function Definitions
// ------------------------------------------------------------------
void cmd_set_log_frequency(char* args, CommandCaller* caller) {
  // Create a local copy of args
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int newInterval = -1;
  // Use localArgs for parsing rather than args.
  if (sscanf(localArgs, "%d", &newInterval) == 1 && newInterval > 0) {
    logging.logInterval = newInterval;
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

  fanAutoMode = true;
  caller->println(F("[MESSAGE] Fan auto control re-enabled."));
}

void cmd_set_reagent_valve(char* args, CommandCaller* caller) {
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_REAGENT_VALVES &&
      (valveState == 0 || valveState == 1)) {
    bool state = (valveState == 1);
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
    bool state = (valveState == 1);
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
    bool state = (valveState == 1);
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
  // Make a local copy of args.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS) {
    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorDispenseVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset dispense volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RFS <0-3>"));
  }
}


void cmd_reset_flow_total(char* args, CommandCaller* caller) {
  // Make a local copy of args.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS) {
    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorTotalVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset total volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RTF <0-3>"));
  }
}


void cmd_reset_i2c(char* args, CommandCaller* caller) {
  // Make a local copy of args (even if not used, for consistency).
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  Serial.println(F("[MESSAGE] Manual I2C bus reset initiated."));
  resetI2CBus();
  caller->println(F("[MESSAGE] I2C bus reset complete."));
}


void cmd_dispense_reagent(char* args, CommandCaller* caller) {
  // Create a local copy of the input arguments.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int troughNumber = -1;
  float requestedVolume = -1.0;
  const float MIN_VOLUME = 1.0;
  const float MAX_VOLUME = 200.0;

  caller->print(F("[MESSAGE] Received command: D "));
  caller->println(localArgs);

  // Use strtok to split the arguments in the local copy.
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

  caller->print(F("[MESSAGE] Parsed troughNumber: "));
  caller->print(troughNumber);
  caller->print(F(", requestedVolume: "));
  caller->println(requestedVolume);

  // Validate the trough number.
  if (troughNumber < 1 || troughNumber > 4) {
    caller->println(F("[ERROR] Invalid trough number. Use 1-4."));
    return;
  }
  // Check if a dispense is already in progress.
  if (valveControls[troughNumber - 1].isDispensing) {
    caller->print(F("[WARNING] A dispense is already in progress for Trough "));
    caller->println(troughNumber);
    caller->println(F("Use STOPD <trough number> to stop it first."));
    return;
  }
  // Validate requested volume if specified.
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

  // Check pressure.
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Dispense aborted."));
    return;
  }

  // Check for overflow.
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

  // Start flow sensor measurement.
  if (!startFlowSensorMeasurement(*sensor)) {
    caller->print(F("[ERROR] Failed to start flow sensor for Trough "));
    caller->println(troughNumber);
    return;
  }

  caller->print(F("[MESSAGE] Flow sensor measurement started for Trough "));
  caller->println(troughNumber);

  // Open valves and start dispensing.
  openDispenseValves(troughNumber);
  caller->print(F("[MESSAGE] Dispensing started for Trough "));
  caller->println(troughNumber);

  valveControls[troughNumber - 1].isDispensing = true;
  valveControls[troughNumber - 1].targetVolume = requestedVolume;
}


void cmd_stop_dispense(char* args, CommandCaller* caller) {
  // Create a local copy of the arguments.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int troughNumber = -1;
  bool stopAll = false;

  // Check if the local arguments specify "all" (case sensitive).
  if (strncmp(localArgs, "all", 3) == 0) {
    stopAll = true;
  } else if (sscanf(localArgs, "%d", &troughNumber) != 1 || troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    caller->println(F("[ERROR] Invalid trough number. Use STOPD <1-4> or STOPD all."));
    return;
  }

  if (stopAll) {
    caller->println(F("[MESSAGE] Stopping all dispensing operations..."));
    for (int i = 1; i <= NUM_OVERFLOW_SENSORS; i++) {
      stopDispenseOperation(i, caller);
    }
    caller->println(F("[MESSAGE] All dispensing operations stopped."));
  } else {
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispensing stopped for Trough "));
    caller->println(troughNumber);
  }
}


void cmd_prime_valves(char* args, CommandCaller* caller) {
  // Create a local copy of the args to work with.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int localValveNumber = -1;
  char extra;

  // Try to parse exactly one integer. If there's any extra character after it, that's an error.
  if (sscanf(localArgs, "%d %c", &localValveNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for prime command. Use: P <valve number>"));
    return;
  }
  
  // Check for valid valve number (1 to 4).
  if (localValveNumber < 1 || localValveNumber > 4) {
    caller->println(F("[ERROR] Invalid valve number. Use 1-4."));
    return;
  }

  // Pressure check (using same threshold as dispense)
  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Prime aborted."));
    return;
  }

  // Check if the reagent bubble sensor already detects liquid.
  if (readBinarySensor(reagentBubbleSensors[localValveNumber - 1])) {
    caller->print(F("[MESSAGE] Valve "));
    caller->print(localValveNumber);
    caller->println(F(" already primed."));
    return;
  }

  // If the valve is in fill mode, disable it.
  if (valveControls[localValveNumber - 1].fillMode) {
    valveControls[localValveNumber - 1].fillMode = false;
    caller->print(F("[MESSAGE] Fill mode disabled for trough "));
    caller->println(localValveNumber);
  }

  // Mark the valve as under manual control for priming.
  valveControls[localValveNumber - 1].manualControl = true;

  // Open the reagent and media valves for this trough.
  openDispenseValves(localValveNumber);

  // Mark that priming is in progress.
  valveControls[localValveNumber - 1].isPriming = true;

  caller->print(F("[MESSAGE] Priming started for valve "));
  caller->println(localValveNumber);
}



// ------------------------------------------------------------------
// Global Command Tree and Commander Object
// ------------------------------------------------------------------
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
  systemCommand("D", "Dispense reagent: D <1-4> [volume] (volume in mL, continuous if omitted)", cmd_dispense_reagent),
  systemCommand("STOPD", "Stop dispensing: STOPD <1-4> (stop specific trough) or STOPD ALL", cmd_stop_dispense),
  systemCommand("P", "Prime valves: P <1-4> (prime valves until liquid detected)", cmd_prime_valves)
};
