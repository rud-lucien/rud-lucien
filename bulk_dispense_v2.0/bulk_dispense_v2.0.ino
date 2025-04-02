/************************************************************
 * Bulk Dispense v2.0 Main Sketch
 *
 * This sketch is the main entry point for the Bulk Dispense
 * system running on a Controllino Maxi Automation PLC.
 *
 * It performs the following functions:
 *  - Initializes global state and hardware (valves, sensors, fan, etc.)
 *  - Sets up the Commander API for command processing.
 *  - Enters the main loop to monitor system state (flow, temperature,
 *    enclosure, etc.) and process incoming commands.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

#include <Wire.h>
#include <Controllino.h>
#include "Hardware.h"      // Hardware definitions and functions
#include "Sensors.h"       // Sensor functions
#include "Logging.h"       // Logging functions
#include "Commands.h"      // Commander API and command functions
#include "Utils.h"         // Utility functions
#include "SystemMonitor.h" // System monitor functions

//=================================================================
// Setup Function: System Initialization
//=================================================================
void setup()
{
  // --- Serial and Global Flags Initialization ---
  Serial.begin(115200);
  Serial.println(F("[MESSAGE] System starting..."));

  // Reset key global flags
  globalVacuumMonitoring[0] = false;
  globalVacuumMonitoring[1] = false;
  globalEnclosureLiquidError = false;
  logging.previousLogTime = 0;
  fanAutoMode = true;
  proportionalValveMaxFeedback = 0.0;

  // --- Initialize Valve Controls for Each Trough ---
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    valveControls[i].isDispensing = false;
    valveControls[i].manualControl = false;
    valveControls[i].isPriming = false;
    valveControls[i].fillMode = false;
    valveControls[i].isDraining = false;
    valveControls[i].targetVolume = -1; // -1 indicates continuous mode or unassigned
    valveControls[i].lastFlowValue = -1;
    valveControls[i].lastFlowCheckTime = 0;
    valveControls[i].lastFlowChangeTime = 0;
    valveControls[i].fillCheckTime = 0;
    valveControls[i].dispensingValveNumber = -1;
  }

  // --- Initialize Hardware ---
  fanSetup(fan);
  proportionalValveSetup(proportionalValve);
  calibrateProportionalValve();

  float systemPressure = readPressure(pressureSensor);
  if (systemPressure > 15.0)
  {
    Serial.print(F("[MESSAGE] System air pressure available: "));
    Serial.print(systemPressure);
    Serial.println(F(" psi."));
  }
  else
  {
    Serial.print(F("[WARNING] Low air pressure detected! Current pressure: "));
    Serial.print(systemPressure);
    Serial.println(F(" psi. Ensure air supply is available."));
  }
  proportionalValve = setValvePosition(proportionalValve, 0.0);

  // --- Initialize Flow Sensors ---
  Serial.println(F("[MESSAGE] Setting up Flow Sensors..."));
  for (int i = 0; i < NUM_FLOW_SENSORS; i++)
  {
    // Just set initial states without trying to communicate
    flowSensors[i]->sensorInitialized = false;
    flowSensors[i]->sensorStopped = false;
    flowSensors[i]->sensorConnected = 0;
    flowSensors[i]->dispenseVolume = 0.0;
    flowSensors[i]->totalVolume = 0.0;
    flowSensors[i]->lastUpdateTime = 0;
    flowSensors[i]->isValidReading = false;
  }
  Serial.println(F("[MESSAGE] Flow Sensors ready for initialization on demand."));

  // --- Initialize Temperature/Humidity Sensor ---
  if (!tempHumSensorInit())
  {
    Serial.println(F("[ERROR] Temp/Humidity sensor not detected!"));
  }
  else
  {
    Serial.println(F("[MESSAGE] Temp/Humidity sensor initialized successfully."));
  }

  // --- Setup Commander API ---
  commander.attachTree(API_tree);
  commander.init();

  Serial.println(F("[MESSAGE] System ready."));
}

//=================================================================
// Loop Function: Main Program Execution
//=================================================================
void loop()
{
  unsigned long currentTime = millis();

  // Process incoming serial commands.
  handleSerialCommands();

  // Monitor various system parameters.
  monitorOverflowSensors(currentTime);
  monitorFlowSensors(currentTime);
  monitorPrimeSensors(currentTime);
  monitorFillSensors(currentTime);
  monitorWasteSensors(currentTime);
  monitorVacuumRelease(currentTime);
  monitorEnclosureLiquidSensor(currentTime);
  monitorEnclosureTemp(currentTime);
  monitorFlowSensorConnections();

  // Read flow sensor data.
  readFlowSensorData(flow1);
  readFlowSensorData(flow2);
  readFlowSensorData(flow3);
  readFlowSensorData(flow4);

  // Log system state periodically.
  if (currentTime - logging.previousLogTime >= logging.logInterval)
  {
    logging.previousLogTime = currentTime;
    logSystemState();
  }
}
