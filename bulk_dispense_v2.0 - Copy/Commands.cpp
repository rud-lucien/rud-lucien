#include "Commands.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Logging.h"
#include "Utils.h"
#include <Controllino.h>
#include "CommandSession.h"



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

    // Disable fill mode if active.
    disableFillMode(valveNumber, caller);

    bool state = (valveState == 1);
    
    // Enable manual control ONLY when opening the valve.
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

    // Disable fill mode if active.
    disableFillMode(valveNumber, caller);

    bool state = (valveState == 1);
    
    // Enable manual control ONLY when opening the valve.
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

    // Disable fill mode if active.
    disableFillMode(valveNumber, caller);

    bool state = (valveState == 1);
    
    // Enable manual control ONLY when opening the valve.
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
    // Disable fill mode if active.
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
    // Disable fill mode if active.
    disableFillMode(sensorNumber - 1, caller);
        
    // Set manual control to false.
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
  // Make a local copy of args.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS) {
    // Disable fill mode if active.
    disableFillMode(sensorNumber, caller);
       
    // Set manual control to true.
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
  // Make a local copy of args.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';
  
  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS) {
    // Disable fill mode if active.
    disableFillMode(sensorNumber, caller);
     
    // Set manual control to true.
    enableManualControl(sensorNumber, caller);

    // Create an array of pointers to the flow sensors.
    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorTotalVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset total volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RTF <0-3>"));
  }
}


void cmd_reset_i2c(char* args, CommandCaller* caller) {
  // Create a local copy of args (for consistency)
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // For every trough, disable fill mode using our helper function,
  // and enable manual control.
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    disableFillMode(i + 1, caller); // i+1 converts from 0-based to 1-based indexing.
    enableManualControl(i, caller); // This function takes 0-based index.
  }
  

  caller->println(F("[MESSAGE] Manual I2C bus reset initiated."));
  resetI2CBus();
  caller->println(F("[MESSAGE] I2C bus reset complete."));
}


void cmd_dispense_reagent(char* args, CommandCaller* caller) {
  // Create a local copy of the input arguments.
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

  caller->print(F("[MESSAGE] Dispense command received for Trough "));
  caller->print(troughNumber);
  caller->print(F(" with requested volume "));
  caller->println(requestedVolume);


  // Validate the trough number.
  if (!validateTroughNumber(troughNumber, caller)) {
  return;
  }


  // Disable fill mode if active
  disableFillMode(troughNumber, caller);

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

  // Register this dispensing operation as asynchronous.
  // Reset the async completion flag so that the monitor function will later call asyncCommandCompleted.
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
    // disable fill mode for all trough.
    disableFillModeForAll(caller);
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      // Stop the dispensing operation.
      stopDispenseOperation(i + 1, caller);
    }
    caller->println(F("[MESSAGE] All dispensing operations stopped."));
  } else {
    disableFillMode(troughNumber, caller);
    // Stop the dispensing operation.
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

  if (globalEnclosureLiquidError) {
  caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
  return;
  }


  int localValveNumber = -1;
  char extra;

  // Try to parse exactly one integer. If there's any extra character after it, that's an error.
  if (sscanf(localArgs, "%d %c", &localValveNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for prime command. Use: P <valve number>"));
    return;
  }
  
  // Check for valid valve number (1 to 4).
  if (!validateValveNumber(localValveNumber, caller)) {
  return;
}

  // Disable fill mode if active
  disableFillMode(localValveNumber, caller);

  // Pressure check (using same threshold as dispense)
  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Prime aborted."));
    return;
  }

  // Check if the reagent bubble sensor already detects liquid.
  if (isValveAlreadyPrimed(localValveNumber, caller)) {
  return;
}

  // Open the reagent and media valves for this trough.
  openDispenseValves(localValveNumber);

  // Mark that priming is in progress.
  valveControls[localValveNumber - 1].isPriming = true;
  caller->print(F("[MESSAGE] Priming started for valve "));
  caller->println(localValveNumber);
}


void cmd_fill_reagent(char* args, CommandCaller* caller) {
  // Create a local copy of the input arguments.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  if (globalEnclosureLiquidError) {
  caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
  return;
  }

  int troughNumber = -1;
  char extra; // to capture any extra token

  // Try to parse exactly one integer. If there's any extra token, that's an error.
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for fill command. Use: F <valve number>"));
    return;
  }
  
  // Validate trough number (must be between 1 and 4).
  if (!validateTroughNumber(troughNumber, caller)) {
  return;
}

  
  // If dispensing is in progress on this trough, stop it.
  stopDispensingForFill(troughNumber, caller);

  // If priming is in progress on this trough, stop it as well.
  stopPrimingForFill(troughNumber, caller);

  // Pressure check (using same threshold as dispense/prime).
  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Fill aborted."));
    return;
  }
  
  // Reset the flow sensor so we start fresh.
  resetFlowSensorDispenseVolume(*flowSensors[troughNumber - 1]);

  // Open the reagent and media valves.
  openDispenseValves(troughNumber);

  // Enable fill mode for this trough.
  enableFillMode(troughNumber, caller);

  // For fill mode (a long-running process), immediately mark the command as complete.
  // This causes the ACTION END tag to be printed now, even though fill mode continues.
  asyncCommandCompleted(&Serial);
}

void cmd_drain_trough(char* args, CommandCaller* caller) {
  // Create a local copy of the input arguments.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Check for enclosure liquid error first.
  if (globalEnclosureLiquidError) {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int troughNumber = -1;
  char extra; // to catch any extra tokens

  // Expect exactly one integer argument (the trough number)
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1) {
    caller->println(F("[ERROR] Invalid arguments for drain command. Use: DT <trough number>"));
    return;
  }
  
  // Validate the trough number.
  if (!validateTroughNumber(troughNumber, caller)) {
    return;
  }
  
  // Check if the corresponding waste bottle is full.
  // For troughs 1–2, use waste bottle sensor index 0; for 3–4, use index 1.
  if (isWasteBottleFullForTrough(troughNumber, caller)) {
    return;
  }
    
  // Check for incompatible drainage.
  if (hasIncompatibleDrainage(troughNumber, caller)) {
    return;
  }
  
  // Stop any dispensing or fill mode if active.
  stopDispensingIfActive(troughNumber, caller);
  disableFillMode(troughNumber, caller);
  
  // Set the draining flag.
  valveControls[troughNumber - 1].isDraining = true;
  
  // Reset the asynchronous completion flag for this trough.
  drainAsyncCompleted[troughNumber - 1] = false;
  
  // Execute the drain logic based on the trough number.
  // Use the hardware functions to update valve state.
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
  // Note: We do not call asyncCommandCompleted() here.
  // The asynchronous completion (ACTION END) will be signaled by monitorWasteSensors().
}



void cmd_stop_drain_trough(char* args, CommandCaller* caller) {
  // Create a local copy of the input arguments.
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Check if "all" was provided.
  if (strncmp(localArgs, "all", 3) == 0) {
    // Stop draining all troughs.
    for (int i = 0; i < 4; i++) {
      valveControls[i].isDraining = false;
    }
    // Set vacuum monitoring flags for both bottles.
    globalVacuumMonitoring[0] = true; // Monitor waste bottle 1
    globalVacuumMonitoring[1] = true; // Monitor waste bottle 2

    // Close vacuum valves using the hardware functions.
    wasteValve1 = closeValve(wasteValve1);
    wasteValve2 = closeValve(wasteValve2);

    caller->println(F("[MESSAGE] Draining stopped for all troughs. Waste valves closed."));
    return;
  }

  // Otherwise, expect a trough number (1-4).
  int troughNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1 || troughNumber < 1 || troughNumber > 4) {
    caller->println(F("[ERROR] Invalid arguments. Use: SDT <1-4> or SDT all."));
    return;
  }

  // Validate trough number.
  if (!validateTroughNumber(troughNumber, caller)) {
    return;
  }

  // Stop draining the specific trough.
  valveControls[troughNumber - 1].isDraining = false;

  // Determine which vacuum bottle to monitor and close its valve.
  setVacuumMonitoringAndCloseMainValve(troughNumber, caller);

  // Execute the stop drain logic based on the trough number.
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

  // Note: We do NOT call asyncCommandCompleted() here because the completion
  // of the stop-drain operation is determined by the vacuum release.
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
  systemCommand("P", "Prime valves: P <1-4> (prime valves until liquid detected)", cmd_prime_valves),
  systemCommand("F", "Fill reagent: F <1-4>", cmd_fill_reagent),
  systemCommand("DT", "Drain trough: DT <1-4> (drain the specified trough)", cmd_drain_trough),
  systemCommand("SDT", "Stop draining reagent trough: SDT <1-4> or SDT all", cmd_stop_drain_trough)

};
