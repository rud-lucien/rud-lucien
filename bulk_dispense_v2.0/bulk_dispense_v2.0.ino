#include <Wire.h>
#include <Controllino.h>
#include "Hardware.h"       // Hardware definitions and functions
#include "Sensors.h"        // Sensor functions
#include "Logging.h"        // Logging functions
#include "Commands.h"       // Commander API and command functions
#include "Utils.h"          // Utility functions
#include "SystemMonitor.h"  // System monitor functions
#include "CommandSession.h"

void setup() {
  Serial.begin(115200);
  Serial.println(F("[MESSAGE] System starting..."));

  // Explicitly reset key global flags
  globalVacuumMonitoring[0] = false;
  globalVacuumMonitoring[1] = false;
  globalEnclosureLiquidError = false;

  logging.previousLogTime = 0;
  fanAutoMode = true;
  proportionalValveMaxFeedback = 0.0;


  // Initialize valveControls for each trough (0 to NUM_OVERFLOW_SENSORS-1)
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    valveControls[i].isDispensing = false;
    valveControls[i].manualControl = false;
    valveControls[i].isPriming = false;
    valveControls[i].fillMode = false;
    valveControls[i].isDraining = false;
    valveControls[i].targetVolume = -1;  // -1 indicates continuous mode or unassigned
    valveControls[i].lastFlowValue = -1;
    valveControls[i].lastFlowCheckTime = 0;
    valveControls[i].lastFlowChangeTime = 0;
    valveControls[i].fillCheckTime = 0;
    valveControls[i].dispensingValveNumber = -1;
  }

  // Initialize hardware.
  fanSetup(fan);
  proportionalValveSetup(proportionalValve);
  calibrateProportionalValve();

  float systemPressure = readPressure(pressureSensor);
  if (systemPressure > 15.0) {
    Serial.print(F("[MESSAGE] System air pressure available: "));
    Serial.print(systemPressure);
    Serial.println(F(" psi."));
  } else {
    Serial.print(F("[WARNING] Low air pressure detected! Current pressure: "));
    Serial.print(systemPressure);
    Serial.println(F(" psi. Ensure air supply is available."));
  }
  proportionalValve = setValvePosition(proportionalValve, 0.0);

  // Initialize flow sensors.
  Serial.println(F("[MESSAGE] Initializing Flow Sensors..."));
  bool allFailed = true;
  bool anyStopped = false;
  String failedSensors = "";
  for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
    if (flowSensors[i]->sensorStopped) {
      anyStopped = true;
      continue;
    }
    if (initializeFlowSensor(*flowSensors[i])) {
      allFailed = false;
    } else {
      failedSensors += String(i) + " ";
    }
  }
  if (allFailed) {
    Serial.println(F("[WARNING] All flow sensors failed. Resetting I2C bus..."));
    resetI2CBus();
    delay(50);
    failedSensors = "";
    allFailed = true;
    for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
      if (!flowSensors[i]->sensorInitialized && !flowSensors[i]->sensorStopped) {
        if (initializeFlowSensor(*flowSensors[i])) {
          allFailed = false;
        } else {
          failedSensors += String(i) + " ";
        }
      }
    }
  }
  if (anyStopped) {
    Serial.println(F("[MESSAGE] Some flow sensors are intentionally stopped."));
  }
  if (allFailed) {
    Serial.println(F("[ERROR] All active flow sensors failed to initialize!"));
  } else if (failedSensors.length() > 0) {
    Serial.print(F("[ERROR] The following Flow Sensors failed to initialize: "));
    Serial.println(failedSensors);
  } else {
    Serial.println(F("[MESSAGE] All active flow sensors initialized successfully."));
  }
  delay(500);

  // Initialize temperature/humidity sensor.
  if (!tempHumSensorInit()) {
    Serial.println(F("[ERROR] Temp/Humidity sensor not detected!"));
  } else {
    Serial.println(F("[MESSAGE] Temp/Humidity sensor initialized successfully."));
  }

  // Set up Commander API.
  commander.attachTree(API_tree);
  commander.init();

  Serial.println(F("[MESSAGE] System ready."));
}

void loop() {
  unsigned long currentTime = millis();

  handleSerialCommands();
  monitorOverflowSensors(currentTime);
  monitorFlowSensors(currentTime);
  monitorPrimeSensors(currentTime);
  monitorFillSensors(currentTime);
  monitorWasteSensors(currentTime);
  monitorVacuumRelease(currentTime); 
  monitorEnclosureLiquidSensor(currentTime);
  monitorEnclosureTemp();
  monitorFlowSensorConnections();
  

  readFlowSensorData(flow1);
  readFlowSensorData(flow2);
  readFlowSensorData(flow3);
  readFlowSensorData(flow4);

  if (currentTime - logging.previousLogTime >= logging.logInterval) {
    logging.previousLogTime = currentTime;
    logSystemState();
  }
}
