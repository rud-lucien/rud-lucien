#include "Commands.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Logging.h"
#include "Utils.h"
#include <Controllino.h>
#include "CommandSession.h"
#include "CommandManager.h"
#include "SystemMonitor.h"
#include "NetworkConfig.h"

// ============================================================
// Command Function Definitions
// ============================================================

void cmd_set_log_frequency(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int newInterval = -1;
  if (sscanf(localArgs, "%d", &newInterval) == 1 && newInterval > 0)
  {
    logging.logInterval = newInterval;
    logging.previousLogTime = millis(); // <-- Reset timer to start from now
    caller->print(F("[MESSAGE] Log frequency set to "));
    caller->print(newInterval);
    caller->println(F(" ms"));
  }
  else
  {
    caller->println(F("[ERROR] Invalid log frequency. Use: LF <positive number>"));
  }
}

void cmd_fan(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int stateInt = -1;
  if (sscanf(localArgs, "%d", &stateInt) == 1 && (stateInt == 0 || stateInt == 1))
  {
    bool state = (stateInt == 1);
    digitalWrite(fan.relayPin, state ? HIGH : LOW);
    Serial.print(F("[MESSAGE] Fan turned "));
    Serial.println(state ? F("ON") : F("OFF"));
    fanAutoMode = false;
    caller->println(F("[MESSAGE] Fan manual override active. Use FNAUTO to re-enable auto control."));
  }
  else
  {
    caller->println(F("[ERROR] Invalid fan command. Use: FN <0/1>"));
  }
}

void cmd_fan_auto(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Check if fan auto mode is already enabled.
  if (fanAutoMode)
  {
    caller->println(F("[MESSAGE] Fan auto control is already enabled."));
  }
  else
  {
    caller->println(F("[MESSAGE] Fan auto control re-enabled."));
    fanAutoMode = true;
  }

  // Immediately update the fan state based on the current temperature:
  TempHumidity th = readTempHumidity();
  if (th.valid)
  {
    if (th.temperature > ENCLOSURE_TEMP_SETPOINT)
    {
      setFanState(fan, true); // Prints: "[MESSAGE] Fan state set to ON"
    }
    else
    {
      setFanState(fan, false); // Prints: "[MESSAGE] Fan state set to OFF"
    }
  }
  else
  {
    caller->println(F("[ERROR] Failed to read enclosure temperature for fan auto update."));
  }
}

void cmd_set_reagent_valve(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_REAGENT_VALVES &&
      (valveState == 0 || valveState == 1))
  {
    disableFillMode(valveNumber, caller);
    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Reagent valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");

    switch (valveNumber)
    {
    case 1:
      set_valve_state(reagentValve1, state, valveNumber, REAGENT, caller);
      break;
    case 2:
      set_valve_state(reagentValve2, state, valveNumber, REAGENT, caller);
      break;
    case 3:
      set_valve_state(reagentValve3, state, valveNumber, REAGENT, caller);
      break;
    case 4:
      set_valve_state(reagentValve4, state, valveNumber, REAGENT, caller);
      break;
    }
  }
  else
  {
    caller->println(F("[ERROR] Invalid reagent valve command. Use: R <1-4> <0/1>"));
  }
}

void cmd_set_media_valve(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_MEDIA_VALVES &&
      (valveState == 0 || valveState == 1))
  {
    disableFillMode(valveNumber, caller);
    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Media valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");

    switch (valveNumber)
    {
    case 1:
      set_valve_state(mediaValve1, state, valveNumber, MEDIA, caller);
      break;
    case 2:
      set_valve_state(mediaValve2, state, valveNumber, MEDIA, caller);
      break;
    case 3:
      set_valve_state(mediaValve3, state, valveNumber, MEDIA, caller);
      break;
    case 4:
      set_valve_state(mediaValve4, state, valveNumber, MEDIA, caller);
      break;
    }
  }
  else
  {
    caller->println(F("[ERROR] Invalid media valve command. Use: M <1-4> <0/1>"));
  }
}

void cmd_set_waste_valve(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int valveNumber = -1, valveState = -1;
  if (sscanf(localArgs, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_WASTE_VALVES &&
      (valveState == 0 || valveState == 1))
  {
    disableFillMode(valveNumber, caller);
    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Waste valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");

    switch (valveNumber)
    {
    case 1:
      set_valve_state(wasteValve1, state, valveNumber, WASTE, caller);
      break;
    case 2:
      set_valve_state(wasteValve2, state, valveNumber, WASTE, caller);
      break;
    case 3:
      set_valve_state(wasteValve3, state, valveNumber, WASTE, caller);
      break;
    case 4:
      set_valve_state(wasteValve4, state, valveNumber, WASTE, caller);
      break;
    }
  }
  else
  {
    caller->println(F("[ERROR] Invalid waste valve command. Use: W <1-4> <0/1>"));
  }
}

void cmd_set_pressure_valve(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  int percentage = -1;
  if (sscanf(localArgs, "%d", &percentage) == 1 && percentage >= 0 && percentage <= 100)
  {
    proportionalValve = setValvePosition(proportionalValve, (float)percentage);
    caller->print(F("[MESSAGE] Pressure valve set to "));
    caller->print(percentage);
    caller->println(F("%."));
  }
  else
  {
    caller->println(F("[ERROR] Invalid value for pressure valve. Use a percentage between 0 and 100."));
  }

  // Complete the command on both streams if needed
  cm_commandCompleted(serialStream);
  if (useNetworkStream)
  {
    cm_commandCompleted(networkStream);
  }
}

void cmd_calibrate_pressure_valve(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  caller->println(F("[MESSAGE] Calibrating pressure valve, please wait..."));
  calibrateProportionalValve();
  caller->println(F("[MESSAGE] Pressure valve calibration complete."));
}

void cmd_start_flow_sensor_manually(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= NUM_FLOW_SENSORS)
  {
    disableFillMode(sensorNumber, caller);
    enableManualControl(sensorNumber - 1, caller);

    FlowSensor *sensor = flowSensors[sensorNumber - 1];
    if (!sensor)
    {
      caller->print(F("[ERROR] Flow Sensor "));
      caller->print(sensorNumber);
      caller->println(F(" not found."));
      return;
    }
    if (startFlowSensorMeasurement(*sensor))
    {
      caller->print(F("[MESSAGE] Manually started measurement for Flow Sensor "));
      caller->println(sensorNumber);
    }
    else
    {
      caller->print(F("[ERROR] Failed to start Flow Sensor "));
      caller->println(sensorNumber);
    }
  }
  else
  {
    caller->println(F("[ERROR] Invalid sensor number. Use: STARTFSM <1-4>"));
  }
}

void cmd_stop_flow_sensor_manually(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= NUM_FLOW_SENSORS)
  {
    disableFillMode(sensorNumber - 1, caller);
    disableManualControl(sensorNumber - 1, caller);

    FlowSensor *sensor = flowSensors[sensorNumber - 1];
    if (!sensor)
    {
      caller->print(F("[ERROR] Flow Sensor "));
      caller->print(sensorNumber);
      caller->println(F(" not found."));
      return;
    }
    if (stopFlowSensorMeasurement(*sensor))
    {
      caller->print(F("[MESSAGE] Manually stopped Flow Sensor "));
      caller->println(sensorNumber);
    }
    else
    {
      caller->print(F("[ERROR] Failed to stop Flow Sensor "));
      caller->println(sensorNumber);
    }
  }
  else
  {
    caller->println(F("[ERROR] Invalid sensor number. Use: STOPFSM <1-4>"));
  }
}

void cmd_reset_flow_dispense(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS)
  {
    disableFillMode(sensorNumber, caller);
    enableManualControl(sensorNumber, caller);

    FlowSensor *sensors[] = {&flow1, &flow2, &flow3, &flow4};
    resetFlowSensorDispenseVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset dispense volume for Flow Sensor "));
    caller->println(sensorNumber);
  }
  else
  {
    caller->println(F("[ERROR] Invalid sensor number. Use: RFS <1-4>"));
  }
}

void cmd_reset_flow_total(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int sensorNumber = -1;
  if (sscanf(localArgs, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber < NUM_FLOW_SENSORS)
  {
    disableFillMode(sensorNumber, caller);
    enableManualControl(sensorNumber, caller);

    FlowSensor *sensors[] = {&flow1, &flow2, &flow3, &flow4};
    resetFlowSensorTotalVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset total volume for Flow Sensor "));
    caller->println(sensorNumber);
  }
  else
  {
    caller->println(F("[ERROR] Invalid sensor number. Use: RTF <0-3>"));
  }
}

void cmd_reset_i2c(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    disableFillMode(i + 1, caller);
    enableManualControl(i, caller);
  }

  caller->println(F("[MESSAGE] Manual I2C bus reset initiated."));
  resetI2CBus();
  caller->println(F("[MESSAGE] I2C bus reset complete."));
}

void cmd_dispense_reagent(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, we need to use the right stream
  // Check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  if (globalEnclosureLiquidError)
  {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  int troughNumber = -1;
  float requestedVolume = -1.0;
  const float MIN_VOLUME = 1.0;
  const float MAX_VOLUME = 200.0;

  caller->print(F("[MESSAGE] Received command: D "));
  caller->println(localArgs);

  // Parse arguments.
  char *token = strtok(localArgs, " ");
  if (token != NULL)
  {
    troughNumber = atoi(token);
    token = strtok(NULL, " ");
    if (token != NULL)
    {
      requestedVolume = atof(token);
    }
    else
    {
      requestedVolume = -1.0; // Continuous mode if no volume provided.
    }
  }
  else
  {
    // Invalid format.
    caller->println(F("[ERROR] Invalid command format. Use: D <1-4> [volume]"));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  caller->print(F("[MESSAGE] Dispense command received for Trough "));
  caller->print(troughNumber);
  caller->print(F(" with requested volume "));
  caller->println(requestedVolume);

  if (!validateTroughNumber(troughNumber, caller))
  {
    caller->println(F("[ERROR] Invalid command format. Use: D <1-4> [volume]"));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  disableFillMode(troughNumber, caller);

  if (valveControls[troughNumber - 1].isDispensing)
  {
    caller->print(F("[WARNING] A dispense is already in progress for Trough "));
    caller->println(troughNumber);
    caller->println(F("Use STOPD <trough number> to stop it first."));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  if (requestedVolume > 0)
  {
    if (requestedVolume < MIN_VOLUME)
    {
      caller->print(F("[ERROR] Requested volume too low. Minimum: "));
      caller->print(MIN_VOLUME);
      caller->println(F(" mL."));

      // Signal asynchronous command completion for both streams if needed
      cm_commandCompleted(serialStream);
      if (useNetworkStream)
      {
        cm_commandCompleted(networkStream);
      }

      return;
    }
    else if (requestedVolume > MAX_VOLUME)
    {
      caller->print(F("[ERROR] Requested volume too high. Maximum: "));
      caller->print(MAX_VOLUME);
      caller->println(F(" mL."));

      // Signal asynchronous command completion for both streams if needed
      cm_commandCompleted(serialStream);
      if (useNetworkStream)
      {
        cm_commandCompleted(networkStream);
      }

      return;
    }
  }

  // Pressure check
  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS))
  {
    caller->println(F("[ERROR] Pressure check failed. Dispense aborted."));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  // Check for overflow before starting dispense.
  if (readBinarySensor(overflowSensors[troughNumber - 1]))
  {
    caller->print(F("[ERROR] Cannot dispense: Overflow detected for Trough "));
    caller->println(troughNumber);

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  FlowSensor *sensor = flowSensors[troughNumber - 1];
  if (!sensor)
  {
    caller->print(F("[ERROR] No flow sensor found for Trough "));
    caller->println(troughNumber);

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  if (!startFlowSensorMeasurement(*sensor))
  {
    caller->print(F("[ERROR] Failed to start flow sensor for Trough "));
    caller->println(troughNumber);

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

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

  // *** IMPORTANT: Do NOT call cm_commandCompleted() here.
  // For an asynchronous command, the completion function should be called
  // later by the async callback when the dispense finishes (or times out).
}

void cmd_stop_dispense(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  int troughNumber = -1;
  bool stopAll = false;

  if (strncmp(localArgs, "all", 3) == 0)
  {
    stopAll = true;
  }
  else if (sscanf(localArgs, "%d", &troughNumber) != 1 || troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS)
  {
    caller->println(F("[ERROR] Invalid trough number. Use STOPD <1-4> or STOPD all."));
    return;
  }

  if (stopAll)
  {
    caller->println(F("[MESSAGE] Stopping all dispensing operations..."));
    disableFillModeForAll(caller);
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
    {
      stopDispenseOperation(i + 1, caller);
    }
    caller->println(F("[MESSAGE] All dispensing operations stopped."));
  }
  else
  {
    disableFillMode(troughNumber, caller);
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispensing stopped for Trough "));
    caller->println(troughNumber);
  }
}

void cmd_prime_valves(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, we need to use the right stream
  // Check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  if (globalEnclosureLiquidError)
  {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int localValveNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &localValveNumber, &extra) != 1)
  {
    // caller->println(F("[ERROR] Invalid arguments for prime command. Use: P <valve number>"));
    // asyncCommandCompleted(&Serial);
    return;
  }

  if (!validateValveNumber(localValveNumber, caller))
  {
    caller->println(F("[ERROR] Invalid arguments for prime command. Use: P <1-4>"));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  disableFillMode(localValveNumber, caller);

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS))
  {
    caller->println(F("[ERROR] Pressure check failed. Prime aborted."));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  if (isValveAlreadyPrimed(localValveNumber, caller))
  {
    caller->print(F("[WARNING] Valve "));
    caller->print(localValveNumber);
    caller->println(F(" is already primed."));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return;
  }

  openDispenseValves(localValveNumber);
  valveControls[localValveNumber - 1].isPriming = true;
  caller->print(F("[MESSAGE] Priming started for valve "));
  caller->println(localValveNumber);
}

void cmd_fill_reagent(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, we need to use the right stream
  // Check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  if (globalEnclosureLiquidError)
  {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int troughNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1)
  {
    // caller->println(F("[ERROR] Invalid arguments for fill command. Use: F <valve number>"));
    return;
  }

  if (!validateTroughNumber(troughNumber, caller))
  {
    caller->println(F("[ERROR] Invalid arguments for fill command. Use: F <1-4>"));
    return;
  }

  stopDispensingForFill(troughNumber, caller);
  stopPrimingForFill(troughNumber, caller);

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS))
  {
    caller->println(F("[ERROR] Pressure check failed. Dispense aborted."));

    // Signal asynchronous command completion for both streams if needed
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }

    return; // Abort further processing for this command.
  }

  resetFlowSensorDispenseVolume(*flowSensors[troughNumber - 1]);
  openDispenseValves(troughNumber);
  enableFillMode(troughNumber, caller);
}

void cmd_drain_trough(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, we need to use the right stream
  // Check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  // If enclosure liquid is detected, abort the drain command.
  if (globalEnclosureLiquidError)
  {
    caller->println(F("[ERROR] Enclosure liquid detected. Operation aborted. Resolve the leak before proceeding."));
    return;
  }

  int troughNumber = -1;
  char extra;
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1)
  {
    // caller->println(F("[ERROR] Invalid arguments for drain command. Use: DT <trough number>"));
    return;
  }

  if (!validateTroughNumber(troughNumber, caller))
  {
    caller->println(F("[ERROR] Invalid arguments for drain command. Use: DT <1-4>"));
    // Complete command on both streams if network client is active
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }
    return;
  }

  if (isWasteBottleFullForTrough(troughNumber, caller))
  {
    // Complete command on both streams if network client is active
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }
    return;
  }

  if (hasIncompatibleDrainage(troughNumber, caller))
  {
    // Complete command on both streams if network client is active
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }
    return;
  }

  stopDispensingIfActive(troughNumber, caller);
  disableFillMode(troughNumber, caller);

  int index = troughNumber - 1;

  // Prevent re-starting a drain if one is already in progress.
  if (valveControls[index].isDraining)
  {
    caller->println(F("[ERROR] Drain already in progress for this trough."));
    return;
  }

  // Start the draining process.
  valveControls[index].isDraining = true;
  drainAsyncCompleted[index] = false;

  switch (troughNumber)
  {
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
    caller->println(F("[ERROR] Drain command received invalid trough number. Use 1-4."));
    // Reset draining state in case of error.
    valveControls[index].isDraining = false;
    return;
  }
  // The asynchronous monitorWasteSensors() function will handle drain completion or timeout.
}

void cmd_stop_drain_trough(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, we need to use the right stream
  // Check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  // If the argument is "all", stop draining for every trough.
  if (strncmp(localArgs, "all", 3) == 0)
  {
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
    {
      if (valveControls[i].isDraining)
      { // only if a drain is active
        valveControls[i].isDraining = false;
        valveControls[i].drainStartTime = 0;
        // Mark the drain async task for this trough as complete if not already.
        if (!drainAsyncCompleted[i])
        {
          // Complete command on both streams if network client is active
          cm_commandCompleted(serialStream);
          if (useNetworkStream)
          {
            cm_commandCompleted(networkStream);
          }
          drainAsyncCompleted[i] = true;
        }
      }
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
  if (sscanf(localArgs, "%d %c", &troughNumber, &extra) != 1 || troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS)
  {
    caller->println(F("[ERROR] Invalid arguments. Use: SDT <trough number (1-4)> or SDT all."));
    return;
  }
  if (!validateTroughNumber(troughNumber, caller))
  {
    return;
  }
  int index = troughNumber - 1;
  // Stop the drain for this trough.
  valveControls[index].isDraining = false;
  valveControls[index].drainStartTime = 0; // clear recorded start time
  setVacuumMonitoringAndCloseMainValve(troughNumber, caller);

  // Change valve states as appropriate for the trough.
  switch (troughNumber)
  {
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
    caller->println(F("[ERROR] Invalid trough number. Use: SDT <1-4> or SDT all."));
    return;
  }
  // Mark this trough's drain async as complete if it wasn't already.
  if (!drainAsyncCompleted[index])
  {
    // Complete command on both streams if network client is active
    cm_commandCompleted(serialStream);
    if (useNetworkStream)
    {
      cm_commandCompleted(networkStream);
    }
    drainAsyncCompleted[index] = true;
  }
}

void cmd_log_help(char *args, CommandCaller *caller)
{
  // Create a local copy of args (even if not used, for consistency)
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  caller->println(F("Bulk Dispense System Log Field Definitions:"));
  caller->println(F("--------------------------------------------------"));

  // Hardware Status
  caller->println(F("[LOG]  : Log entry prefix indicating the start of a new status record."));
  caller->println(F("F      : Fan state (F1 = ON, F0 = OFF)."));
  caller->println(F("RVxxxx : Reagent Valve states. 4-digit binary (1 = OPEN, 0 = CLOSED)."));
  caller->println(F("         e.g., RV1000 means only valve 1 is open."));
  caller->println(F("MVxxxx : Media Valve states (same format as RV)."));
  caller->println(F("WVxxxx : Waste Valve states (same format as RV)."));

  // Proportional Valve
  caller->println(F("PV, V, PV%, P : Proportional Valve feedback."));
  caller->println(F("         V = measured voltage (e.g., 9.7)"));
  caller->println(F("         P = calculated percentage (e.g., 99.9%)."));

  // Sensor Readings
  caller->println(F("WSLxx  : Waste Line Sensor readings (binary; e.g., WSL00 means no detection)."));
  caller->println(F("WBLxx  : Waste Bottle Sensor readings (binary)."));
  caller->println(F("WVSxx  : Waste Vacuum Sensor readings (binary)."));
  caller->println(F("ELSx   : Enclosure Liquid Sensor (0 = no liquid, 1 = liquid detected)."));
  caller->println(F("BSxxxx : Bubble Sensor readings (4-digit binary)."));
  caller->println(F("OSxxxx : Overflow Sensor readings (4-digit binary)."));
  caller->println(F("PS, P  : Pressure sensor reading (in psi)."));
  caller->println(F("T, T   : Temperature (°C)."));
  caller->println(F("H, H   : Humidity (percentage)."));

  // Flow Sensor Data
  caller->println(F("FSn,...: Flow Sensor n data including:"));
  caller->println(F("         - Flow rate"));
  caller->println(F("         - Sensor temperature"));
  caller->println(F("         - Dispensed volume"));
  caller->println(F("         - Total volume"));
  caller->println(F("         - Sensor status flag (e.g., -1 for an invalid reading)"));

  // Operational States
  caller->println(F("DSxxxx : Dispensing state for each trough (4-digit binary; 1 = active, 0 = inactive)."));
  caller->println(F("TV, V1,V2,V3,V4 : Target Volume for each trough (in mL)."));
  caller->println(F("PRxxxx : Priming state for each trough (4-digit binary)."));
  caller->println(F("FMxxxx : Fill mode state for each trough (4-digit binary)."));
  caller->println(F("TDSxxxx: Trough Draining State for each trough (4-digit binary)."));

  // Diagnostic Summary
  caller->println(F("DIAG   : Diagnostic summary appended to each log entry, including:"));
  caller->println(F("         FAM   = Fan Auto Mode (ON/OFF)."));
  caller->println(F("         EERR  = Enclosure Liquid Error (TRUE/FALSE)."));
  caller->println(F("         GVM1  = Global Vacuum Monitoring for waste bottle 1 (TRUE/FALSE)."));
  caller->println(F("         GVM2  = Global Vacuum Monitoring for waste bottle 2 (TRUE/FALSE)."));
  caller->println(F("         MC1–MC4 = Manual Control flags for troughs (ON = manual override, OFF = automated)."));
  caller->println(F("         LF    = Logging frequency (in milliseconds)."));
  caller->println(F("         RC    = Registered commands (number of pending commands, >= 0)."));
  caller->println(F("         NET   = Network status (connected/disconnected)."));
  caller->println(F("         FSn   = Flow Sensor status for each trough (n = 1-4)"));
  caller->println(F("                 States: Not Dispensing, Valid, Invalid"));
  caller->println(F("--------------------------------------------------"));
}

void cmd_standby(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  // Copy and null-terminate the command arguments (if any)
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  caller->println(F("[MESSAGE] Executing STANDBY command. Shutting down system to safe state..."));

  // Abort all running operations for each trough.
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    // Stop asynchronous sensor operations.
    stopFlowSensorMeasurement(*flowSensors[i]);
    resetFlowSensorDispenseVolume(*flowSensors[i]);

    // Reset the trough control flags.
    valveControls[i].isDispensing = false;
    valveControls[i].isPriming = false;
    valveControls[i].fillMode = false;
    valveControls[i].isDraining = false;
    valveControls[i].manualControl = false;
    valveControls[i].targetVolume = -1;
    valveControls[i].lastFlowCheckTime = 0;
    valveControls[i].lastFlowChangeTime = 0;

    // Reset any asynchronous timers associated with this trough.
    valveControls[i].drainStartTime = 0;
    // (If you have other timers such as fillStartTime, reset them here as well.)
  }

  // Call the monitor reset helpers to clear any stale timers/flags
  resetPrimeMonitorState();
  resetFillMonitorState();
  resetWasteMonitorState();
  resetEnclosureLeakMonitorState();
  resetFillMonitorState();

  // Close all reagent and media valves.
  reagentValve1 = closeValve(reagentValve1);
  mediaValve1 = closeValve(mediaValve1);
  reagentValve2 = closeValve(reagentValve2);
  mediaValve2 = closeValve(mediaValve2);
  reagentValve3 = closeValve(reagentValve3);
  mediaValve3 = closeValve(mediaValve3);
  reagentValve4 = closeValve(reagentValve4);
  mediaValve4 = closeValve(mediaValve4);

  // Close all waste valves.
  wasteValve1 = closeValve(wasteValve1);
  wasteValve2 = closeValve(wasteValve2);
  wasteValve3 = closeValve(wasteValve3);
  wasteValve4 = closeValve(wasteValve4);

  // Close the pressure valve by setting its position to 0%.
  proportionalValve = setValvePosition(proportionalValve, 0.0);

  // Reset global vacuum monitoring flags.
  globalVacuumMonitoring[0] = false;
  globalVacuumMonitoring[1] = false;

  caller->println(F("[MESSAGE] All automated operations aborted. System is now in STANDBY mode."));

  // Abort both sessions if active
  if (cm_isSessionActive())
  {
    // First check if we have any active network clients
    if (hasActiveClient && currentClient.connected())
    {
      // For network commands, use the network stream
      cm_abortSession(&currentClient);
    }
    else
    {
      // Otherwise, use the Serial stream
      cm_abortSession(&Serial);
    }

    // These variables should be declared in CommandManager.h as extern
    // and we should include that header, but for now we'll just
    // call a function that will reset them
    resetCommandTimers();
  }
}

void cmd_get_system_state(char *args, CommandCaller *caller)
{
  // Make a local copy of the arguments (though not used here)
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Header
  caller->println(F("--------------------------------------------------"));
  caller->println(F("SYSTEM STATE SUMMARY"));
  caller->println(F("--------------------------------------------------"));

  // Overall system state: If any trough is active (dispensing, priming, filling, or draining)
  String overallState = "Overall System State: " + getOverallTroughState();
  caller->println(overallState);

  // Fan section
  caller->println(F("Fan:"));
  caller->print(F("  • Mode          : "));
  caller->println(fanAutoMode ? F("Auto") : F("Manual"));
  caller->print(F("  • Current State : "));
  bool fanState = (digitalRead(fan.relayPin) == HIGH);
  caller->println(fanState ? F("ON") : F("OFF"));
  caller->println();

  // Enclosure section
  caller->println(F("Enclosure:"));
  caller->print(F("  • Liquid Leak   : "));
  caller->println(globalEnclosureLiquidError ? F("Detected") : F("NONE"));
  caller->println();

  // Vacuum Monitoring section
  caller->println(F("Vacuum Monitoring:"));
  caller->print(F("  • Waste Bottle 1: "));
  caller->println(globalVacuumMonitoring[0] ? F("Active") : F("Inactive"));
  caller->print(F("  • Waste Bottle 2: "));
  caller->println(globalVacuumMonitoring[1] ? F("Active") : F("Inactive"));
  caller->println();

  // Manual Control per Trough
  caller->println(F("Manual Control (per Trough):"));
  for (int i = 0; i < 4; i++)
  {
    caller->print(F("  • Trough "));
    caller->print(i + 1);
    caller->print(F(": "));
    caller->println(valveControls[i].manualControl ? F("ON") : F("OFF"));
  }
  caller->println();

  // Flow Sensors section
  caller->println(F("Flow Sensors:"));
  for (int i = 0; i < 4; i++)
  {
    caller->print(F("  • FS"));
    caller->print(i + 1);
    caller->print(F(": Flow Rate: "));
    char buf[10];
    dtostrf(flowSensors[i]->flowRate, 4, 1, buf);
    caller->print(buf);
    caller->print(F(" mL/s, Status: "));
    // Three states: If not dispensing → IDLE; if dispensing and valid → VALID; otherwise → INVALID.
    if (!valveControls[i].isDispensing)
    {
      caller->print(F("IDLE/NOT MEASURING"));
    }
    else
    {
      caller->print(flowSensors[i]->isValidReading ? F("VALID") : F("INVALID"));
    }
    caller->print(F(", Current Dispense Volume: "));
    dtostrf(flowSensors[i]->dispenseVolume, 4, 1, buf);
    caller->print(buf);
    caller->print(F(" mL, Total Dispensed: "));
    dtostrf(flowSensors[i]->totalVolume, 4, 1, buf);
    caller->print(buf);
    caller->println(F(" mL"));
  }
  caller->println();

  // Priming, Dispensing, Filling, Draining Status
  caller->println(F("Priming Status:"));
  for (int i = 0; i < 4; i++)
  {
    caller->print(F("  • Trough "));
    caller->print(i + 1);
    caller->print(F(": "));
    caller->println(valveControls[i].isPriming ? F("PRIMING") : F("NOT PRIMING"));
  }
  caller->println();

  caller->println(F("Dispensing Status:"));
  for (int i = 0; i < 4; i++)
  {
    caller->print(F("  • Trough "));
    caller->print(i + 1);
    caller->print(F(": "));
    caller->println(valveControls[i].isDispensing ? F("DISPENSING") : F("NOT DISPENSING"));
  }
  caller->println();

  caller->println(F("Filling Status:"));
  for (int i = 0; i < 4; i++)
  {
    caller->print(F("  • Trough "));
    caller->print(i + 1);
    caller->print(F(": "));
    caller->println(valveControls[i].fillMode ? F("FILLING") : F("NOT FILLING"));
  }
  caller->println();

  caller->println(F("Draining Status:"));
  for (int i = 0; i < 4; i++)
  {
    caller->print(F("  • Trough "));
    caller->print(i + 1);
    caller->print(F(": "));
    caller->println(valveControls[i].isDraining ? F("DRAINING") : F("NOT DRAINING"));
  }
  caller->println();

  // Pressure Valve section
  caller->println(F("Pressure Valve:"));
  caller->print(F("  • Feedback Voltage : "));
  char voltageStr[10];
  dtostrf(getValveFeedback(proportionalValve), 4, 1, voltageStr);
  caller->print(voltageStr);
  caller->println(F(" V"));
  caller->print(F("  • Valve Position   : "));
  float feedback = getValveFeedback(proportionalValve);
  float valvePercent = (proportionalValveMaxFeedback > 0) ? (feedback / proportionalValveMaxFeedback) * 100.0 : 0.0;
  char percentStr[10];
  dtostrf(valvePercent, 4, 1, percentStr);
  caller->print(percentStr);
  caller->println(F("%"));
  caller->println();

  // Pressure Sensor section
  float currentPressure = readPressure(pressureSensor);
  caller->println(F("Pressure Sensor:"));
  caller->print(F("  • Reading          : "));
  char pressureStr[10];
  dtostrf(currentPressure, 4, 1, pressureStr);
  caller->print(pressureStr);
  caller->print(F(" psi - "));
  caller->println(currentPressure >= 15.0 ? F("(OK)") : F("(Insufficient)"));
  caller->println();

  // Environment section
  TempHumidity th = readTempHumidity();
  caller->println(F("Environment:"));
  if (th.valid)
  {
    caller->print(F("  • Temperature      : "));
    char tempStr[10];
    dtostrf(th.temperature, 4, 1, tempStr);
    caller->print(tempStr);
    caller->println(F(" °C"));
    caller->print(F("  • Humidity         : "));
    char humStr[10];
    dtostrf(th.humidity, 4, 1, humStr);
    caller->print(humStr);
    caller->println(F(" %"));
  }
  else
  {
    caller->println(F("  • Temperature      : Error reading sensor"));
    caller->println(F("  • Humidity         : Error reading sensor"));
  }
  caller->println();

  // Valve States section
  caller->println(F("Valve States:"));
  // Reagent Valves
  caller->print(F("  • Reagent Valves   : "));
  char reagentState[5];
  sprintf(reagentState, "%d%d%d%d",
          reagentValve1.isOpen ? 1 : 0,
          reagentValve2.isOpen ? 1 : 0,
          reagentValve3.isOpen ? 1 : 0,
          reagentValve4.isOpen ? 1 : 0);
  caller->print(reagentState);
  caller->print("  (");
  caller->print(getOpenValvesString(reagentValve1.isOpen, reagentValve2.isOpen, reagentValve3.isOpen, reagentValve4.isOpen));
  caller->println(")");
  // caller->println(F("  (e.g., Valve 1 & 3 Open)"));
  // Media Valves
  caller->print(F("  • Media Valves     : "));
  char mediaState[5];
  sprintf(mediaState, "%d%d%d%d",
          mediaValve1.isOpen ? 1 : 0,
          mediaValve2.isOpen ? 1 : 0,
          mediaValve3.isOpen ? 1 : 0,
          mediaValve4.isOpen ? 1 : 0);
  caller->print(mediaState);
  caller->print("  (");
  caller->print(getOpenValvesString(mediaValve1.isOpen, mediaValve2.isOpen, mediaValve3.isOpen, mediaValve4.isOpen));
  caller->println(")");
  // caller->println(F("  (e.g., Valve 1 & 4 Open)"));
  // Waste Valves
  caller->print(F("  • Waste Valves     : "));
  char wasteState[5];
  sprintf(wasteState, "%d%d%d%d",
          wasteValve1.isOpen ? 1 : 0,
          wasteValve2.isOpen ? 1 : 0,
          wasteValve3.isOpen ? 1 : 0,
          wasteValve4.isOpen ? 1 : 0);
  caller->print(wasteState);
  caller->print("  (");
  caller->print(getOpenValvesString(wasteValve1.isOpen, wasteValve2.isOpen, wasteValve3.isOpen, wasteValve4.isOpen));
  caller->println(")");
  // caller->println(F("  (e.g., Valve 4 Open)"));
  caller->println();

  // Command Session section
  caller->println(F("Command Session:"));
  caller->print(F("  • Status           : "));
  if (commandSessionActive)
  {
    caller->println(F("ACTIVE (asynchronous operations still in progress)"));
  }
  else
  {
    caller->println(F("INACTIVE (No asynchronous commands pending)"));
  }
  caller->println();

  // Logging section
  caller->println(F("Logging:"));
  caller->print(F("  • Frequency        : "));
  char logFreqStr[10];
  dtostrf(logging.logInterval, 4, 0, logFreqStr);
  caller->print(logFreqStr);
  caller->println(F(" ms"));
  caller->println();

  // Separator
  caller->println(F("--------------------------------------------------"));
  // Diagnostic Flags section
  caller->println(F("DIAGNOSTIC FLAGS:"));
  caller->print(F("  • Fan Auto Mode            : "));
  caller->println(fanAutoMode ? F("ON") : F("OFF"));
  caller->print(F("  • Enclosure Liquid Error   : "));
  caller->println(globalEnclosureLiquidError ? F("TRUE") : F("FALSE"));
  caller->print(F("  • Global Vacuum Monitoring : Bottle 1 = "));
  caller->print(globalVacuumMonitoring[0] ? F("TRUE") : F("FALSE"));
  caller->print(F(", Bottle 2 = "));
  caller->println(globalVacuumMonitoring[1] ? F("TRUE") : F("FALSE"));
  caller->print(F("  • Registered Commands      : "));
  caller->println(cm_getPendingCommands());
  caller->print(F("  • Network Connection     : "));
  caller->println(hasActiveClient ? F("CONNECTED") : F("DISCONNECTED"));
  caller->println(F("--------------------------------------------------"));
}

void cmd_print_help(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  char *trimmed = trimLeadingSpaces(localArgs);

  // If the user requested help for a specific command,
  // we currently do not have detailed help implemented.
  if (strlen(trimmed) > 0)
  {
    caller->println(F("[ERROR] Detailed help for specific commands is not implemented."));
  }
  else
  {
    // No specific command requested; print general help.
    // Pass a pointer to the Serial stream.

    // Print general help information.
    caller->println(F("--------------------------------------------------"));
    caller->println(F("Bulk Dispense System Command Help:"));
    caller->println(F("--------------------------------------------------"));

    commander.printHelp(caller, true, false);

    caller->println(F("--------------------------------------------------"));
  }
}

void cmd_device_info(char *args, CommandCaller *caller)
{
  char localArgs[COMMAND_SIZE];
  strncpy(localArgs, args, COMMAND_SIZE);
  localArgs[COMMAND_SIZE - 1] = '\0';

  // Get both streams that might be in use
  Stream *serialStream = &Serial;
  Stream *networkStream = &currentClient;

  // For completion notifications, check if the network client is active
  bool useNetworkStream = hasActiveClient && currentClient.connected();

  // Always show network info when command is received, but only to Serial
  // Print to serial directly regardless of the caller
  if (serialStream)
  {
    // Always print to Serial whether the command came from Serial or network
    serialStream->println(F("---- Device Information (Serial Only) ----"));

    IPAddress deviceIP = Ethernet.localIP();
    serialStream->print(F("Ethernet IP Address: "));
    serialStream->println(deviceIP);

    serialStream->print(F("TCP Server Listening on: "));
    serialStream->print(DEVICE_IP);
    serialStream->print(F(":"));
    serialStream->println(TCP_PORT);

    serialStream->print(F("MAC Address: "));
    for (int i = 0; i < 6; i++)
    {
      if (i > 0)
        serialStream->print(F(":"));
      if (MAC_ADDRESS[i] < 0x10)
        serialStream->print(F("0"));
      serialStream->print(MAC_ADDRESS[i], HEX);
    }
    serialStream->println();

    serialStream->print(F("Active TCP Connections: "));
    serialStream->println(hasActiveClient ? "YES" : "NO");

    serialStream->println(F("----------------------------------------"));
  }

  // If command came from network client, tell them it's only for serial
  if (useNetworkStream && caller != serialStream)
  {
    networkStream->println(F("[ERROR] Device information can only be accessed via Serial."));
  }

  // Complete the command on both streams if needed
  cm_commandCompleted(serialStream);
  if (useNetworkStream)
  {
    cm_commandCompleted(networkStream);
  }
}

// ============================================================
// Global Command Tree and Commander Object
// ============================================================
Commander commander;

Commander::systemCommand_t API_tree[] = {
    systemCommand("LF", "Set logging interval (ms). Usage: LF <ms>", cmd_set_log_frequency),
    systemCommand("FN", "Manually control fan state. Usage: FN <0/1> (0 = off, 1 = on)", cmd_fan),
    systemCommand("FNAUTO", "Re-enable automatic fan control", cmd_fan_auto),
    systemCommand("R", "Control reagent valve. Usage: R <trough 1-4> <0/1> (0 = close, 1 = open)", cmd_set_reagent_valve),
    systemCommand("M", "Control media valve. Usage: M <trough 1-4> <0/1> (0 = close, 1 = open)", cmd_set_media_valve),
    systemCommand("W", "Control waste valve. Usage: W <trough 1-4> <0/1> (0 = close, 1 = open)", cmd_set_waste_valve),
    systemCommand("PV", "Set pressure valve position as a percentage. Usage: PV <percentage> (0 = close, 100 = open)", cmd_set_pressure_valve),
    systemCommand("CALPV", "Calibrate pressure valve (auto-detect max feedback voltage)", cmd_calibrate_pressure_valve),
    systemCommand("STARTFSM", "Manually start flow sensor measurement. Usage: STARTFSM <sensor 1-4>", cmd_start_flow_sensor_manually),
    systemCommand("STOPFSM", "Manually stop flow sensor measurement. Usage: STOPFSM <sensor 1-4>", cmd_stop_flow_sensor_manually),
    systemCommand("RF", "Reset flow sensor dispense volume. Usage: RFS <sensor index 1-3>", cmd_reset_flow_dispense),
    systemCommand("RTF", "Reset total volume for a flow sensor. Usage: RTF <sensor index 1-4>", cmd_reset_flow_total),
    systemCommand("RESETI2C", "Reset the I2C bus (for communication issues)", cmd_reset_i2c),
    systemCommand("D", "Dispense reagent. Usage: D <trough 1-4> [volume in mL] (omitting volume enables continuous mode)", cmd_dispense_reagent),
    systemCommand("STOPD", "Stop dispensing. Usage: STOPD <trough 1-4> or STOPD all", cmd_stop_dispense),
    systemCommand("P", "Prime valves. Usage: P <trough 1-4> to prime specified trough(s)", cmd_prime_valves),
    systemCommand("F", "Fill reagent. Usage: F <trough 1-4> to fill the specified trough", cmd_fill_reagent),
    systemCommand("DT", "Drain trough. Usage: DT <trough 1-4> to initiate drainage", cmd_drain_trough),
    systemCommand("SDT", "Stop draining trough. Usage: SDT <trough 1-4> or SDT all", cmd_stop_drain_trough),
    systemCommand("LOGHELP", "Display detailed logging field definitions and diagnostic information", cmd_log_help),
    systemCommand("STANDBY", "Abort all automated operations and set the system to a safe idle (standby) state", cmd_standby),
    systemCommand("SS", "Display current system state summary", cmd_get_system_state),
    systemCommand("help", "Display help information for all commands", cmd_print_help),
    systemCommand("h", "Display help information for all commands", cmd_print_help),
    systemCommand("H", "Display help information for all commands", cmd_print_help),
    systemCommand("DI", "Display device network information (Serial only)", cmd_device_info)};
