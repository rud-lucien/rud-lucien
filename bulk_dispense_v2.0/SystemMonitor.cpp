#include "SystemMonitor.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Utils.h"
#include <Wire.h>
#include "CommandSession.h"
#include "CommandManager.h"

/************************************************************
 * SystemMonitor.cpp
 * Implements monitoring functions for the Bulk Dispense system:
 *
 * 1. Prime System: Reagent priming monitoring
 * 2. Fill System: Trough filling monitoring
 * 3. Waste System: Drainage and vacuum monitoring
 * 4. Safety Systems: Enclosure and temperature monitoring
 ************************************************************/

// ============================================================
// Static Variables and Constants - File Scope
// ============================================================

// Prime monitoring variables
static unsigned long primeModeStartTime[NUM_OVERFLOW_SENSORS] = {0};
static unsigned long primeStableDetectTime[NUM_OVERFLOW_SENSORS] = {0};
static unsigned long primeAdditionalTime[NUM_OVERFLOW_SENSORS] = {0};
static unsigned long primeLowFlowTime[NUM_OVERFLOW_SENSORS] = {0};
static bool primeModeFailed[NUM_OVERFLOW_SENSORS] = {false};
static bool primeModeSuccess[NUM_OVERFLOW_SENSORS] = {false};

// Fill monitoring variables
static unsigned long fillModeStartTime[NUM_OVERFLOW_SENSORS] = {0};
static unsigned long fillModeLowFlowTime[NUM_OVERFLOW_SENSORS] = {0};
static float fillModeInitialVolume[NUM_OVERFLOW_SENSORS] = {0};
static unsigned long fillModeLastCheck[NUM_OVERFLOW_SENSORS] = {0};

// Waste monitoring variables
static unsigned long wasteDrainCompleteTime[2] = {0};
static bool wasteLiquidDetected[2] = {false};
static bool wasteVacuumReleased[2] = {false};

// Enclosure monitoring variables
static bool enclosureLeakAbortCalled = false;
static unsigned long enclosureLeakCheckTime = 0;
static unsigned long enclosureLeakErrorTime = 0;

// Constants
const unsigned long PRIME_TIMEOUT_MS = 6000;
const unsigned long STABLE_DETECTION_PERIOD_MS = 500;
const unsigned long ADDITIONAL_PRIME_TIME_MS = 2000;
const unsigned long PRIMING_FLOW_TIMEOUT_MS = 5000;
const float MIN_FLOW_RATE_PRIME = 5.0;

// ============================================================
// Dispensing System Implementation
// ============================================================

void handleOverflowCondition(int triggeredTrough)
{
  if (!valveControls[triggeredTrough - 1].isDispensing)
  {
    return;
  }

  // Send overflow warning
  sendMessage(F("[WARNING] Overflow detected for Trough "), &Serial, currentClient, false);
  sendMessage(String(triggeredTrough).c_str(), &Serial, currentClient);

  closeDispenseValves(triggeredTrough);

  // Report valve closure
  sendMessage(F("[MESSAGE] Closed reagent and media valves for Trough "), &Serial, currentClient, false);
  sendMessage(String(triggeredTrough).c_str(), &Serial, currentClient);

  FlowSensor *sensor = flowSensors[triggeredTrough - 1];
  if (sensor)
  {
    // Report final volume
    sendMessage(F("[MESSAGE] Dispensed volume before overflow: "), &Serial, currentClient, false);
    sendMessage(String(sensor->dispenseVolume, 1).c_str(), &Serial, currentClient, false);
    sendMessage(F(" mL."), &Serial, currentClient);

    stopFlowSensorMeasurement(*sensor);
    resetFlowSensorDispenseVolume(*sensor);
  }

  // Reset valve control states
  valveControls[triggeredTrough - 1].isDispensing = false;
  valveControls[triggeredTrough - 1].lastFlowCheckTime = 0;
  valveControls[triggeredTrough - 1].lastFlowChangeTime = 0;
  valveControls[triggeredTrough - 1].dispensingValveNumber = -1;
}

void handleTimeoutCondition(int troughNumber)
{
  closeDispenseValves(troughNumber);

  sendMessage(F("[ERROR] Timeout: No or insufficient flow detected for Trough "), &Serial, currentClient, false);
  sendMessage(String(troughNumber).c_str(), &Serial, currentClient, false);
  sendMessage(F(". Stopping dispense."), &Serial, currentClient);

  FlowSensor *sensor = flowSensors[troughNumber - 1];
  if (sensor)
  {
    sendMessage(F("[MESSAGE] Dispensed volume before timeout: "), &Serial, currentClient, false);
    sendMessage(String(sensor->dispenseVolume, 1).c_str(), &Serial, currentClient, false);
    sendMessage(F(" mL."), &Serial, currentClient);
    stopFlowSensorMeasurement(*sensor);
    resetFlowSensorDispenseVolume(*sensor);
  }

  valveControls[troughNumber - 1].isDispensing = false;
  valveControls[troughNumber - 1].lastFlowCheckTime = 0;
  valveControls[troughNumber - 1].lastFlowChangeTime = 0;
  valveControls[troughNumber - 1].dispensingValveNumber = -1;
  valveControls[troughNumber - 1].targetVolume = -1;
}

void monitorOverflowSensors(unsigned long currentTime)
{
  static unsigned long previousOverflowCheckTime = 0;

  // Check every 25ms (non-blocking)
  if (currentTime - previousOverflowCheckTime >= 25)
  {
    previousOverflowCheckTime = currentTime;
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
    {
      if (readBinarySensor(overflowSensors[i]))
      {
        handleOverflowCondition(i + 1);
      }
    }
  }
}

void monitorFlowSensors(unsigned long currentTime)
{
  static unsigned long previousCheckTime = 0;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_THRESHOLD = 1.0;
  const float MAX_TROUGH_VOLUME = 205.0;

  if (currentTime - previousCheckTime < 25)
  {
    return; // Only check every 25ms
  }
  previousCheckTime = currentTime;

  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    if (!valveControls[i].isDispensing)
      continue;

    FlowSensor *sensor = flowSensors[i];
    if (!sensor)
      continue;

    // 1. Check for overflow (highest priority)
    if (readBinarySensor(overflowSensors[i]))
    {
      flow_handleDispenseOverflow(i, sensor);
      if (!dispenseAsyncCompleted[i])
      {
        if (hasActiveClient)
        {
          cm_commandCompleted(&currentClient);
        }
        else
        {
          cm_commandCompleted(&Serial);
        }
        dispenseAsyncCompleted[i] = true;
      }
      continue;
    }

    // 2. Skip checks if in manual control
    if (valveControls[i].manualControl)
    {
      valveControls[i].lastFlowCheckTime = 0;
      valveControls[i].lastFlowChangeTime = 0;
      continue;
    }

    // 3. Check for flow timeout
    if (sensor->flowRate < MIN_FLOW_RATE_THRESHOLD)
    {
      if (valveControls[i].lastFlowCheckTime == 0)
      {
        valveControls[i].lastFlowCheckTime = currentTime;
      }
      else if (currentTime - valveControls[i].lastFlowCheckTime >= FLOW_TIMEOUT_MS)
      {
        handleTimeoutCondition(i + 1);
        if (!dispenseAsyncCompleted[i])
        {
          if (hasActiveClient)
          {
            cm_commandCompleted(&currentClient);
          }
          else
          {
            cm_commandCompleted(&Serial);
          }
          dispenseAsyncCompleted[i] = true;
        }
        continue;
      }
    }
    else
    {
      valveControls[i].lastFlowCheckTime = 0;
    }

    // 4. Check if target volume reached
    if (valveControls[i].targetVolume > 0 &&
        sensor->dispenseVolume >= valveControls[i].targetVolume)
    {
      flow_handleVolumeComplete(i, sensor);
      if (!dispenseAsyncCompleted[i])
      {
        if (hasActiveClient)
        {
          cm_commandCompleted(&currentClient);
        }
        else
        {
          cm_commandCompleted(&Serial);
        }
        dispenseAsyncCompleted[i] = true;
      }
      continue;
    }

    // 5. Safety check for maximum volume
    if (sensor->dispenseVolume >= MAX_TROUGH_VOLUME)
    {
      flow_handleSafetyLimitExceeded(i, sensor, MAX_TROUGH_VOLUME);
      if (!dispenseAsyncCompleted[i])
      {
        if (hasActiveClient)
        {
          cm_commandCompleted(&currentClient);
        }
        else
        {
          cm_commandCompleted(&Serial);
        }
        dispenseAsyncCompleted[i] = true;
      }
    }
  }
}

void flow_handleDispenseOverflow(int i, FlowSensor *sensor)
{
  closeDispenseValves(i + 1);
  sendMessage(F("[WARNING] Overflow detected for Trough "), &Serial, currentClient, false);
  sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(". Dispensed volume: "), &Serial, currentClient, false);
  sendMessage(String(sensor->dispenseVolume, 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(" mL."), &Serial, currentClient);

  resetFlowSensorDispenseVolume(*sensor);
  stopFlowSensorMeasurement(*sensor);
  valveControls[i].isDispensing = false;
}

void flow_handleVolumeComplete(int i, FlowSensor *sensor)
{
  closeDispenseValves(i + 1);
  sendMessage(F("[MESSAGE] Dispense complete for Trough "), &Serial, currentClient, false);
  sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(". Final volume dispensed: "), &Serial, currentClient, false);
  sendMessage(String(sensor->dispenseVolume, 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(" mL."), &Serial, currentClient);

  resetFlowSensorDispenseVolume(*sensor);
  stopFlowSensorMeasurement(*sensor);
  valveControls[i].isDispensing = false;
}

void flow_handleSafetyLimitExceeded(int i, FlowSensor *sensor, float maxVolume)
{
  closeDispenseValves(i + 1);
  sendMessage(F("[ERROR] Safety Limit Reached! Dispense stopped for Trough "), &Serial, currentClient, false);
  sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(". Final dispensed volume: "), &Serial, currentClient, false);
  sendMessage(String(sensor->dispenseVolume, 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(" mL. (Max Allowed: "), &Serial, currentClient, false);
  sendMessage(String(maxVolume).c_str(), &Serial, currentClient, false);
  sendMessage(F(" mL)"), &Serial, currentClient);

  resetFlowSensorDispenseVolume(*sensor);
  stopFlowSensorMeasurement(*sensor);
  valveControls[i].isDispensing = false;
}

// ============================================================
// Prime System Implementation
// ============================================================

void monitorPrimeSensors(unsigned long currentTime)
{
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    if (!valveControls[i].isPriming)
    {
      resetPrimingStates(i);
      continue;
    }

    FlowSensor *sensor = flowSensors[i];
    if (!sensor)
      continue;

    // Initialize priming if needed
    if (primeModeStartTime[i] == 0)
    {
      primeModeStartTime[i] = currentTime;
    }

    // 1. Check overflow (highest priority)
    if (readBinarySensor(overflowSensors[i]))
    {
      handlePrimingOverflow(i);
      continue;
    }

    // 2. Check flow rate
    if (sensor->flowRate < MIN_FLOW_RATE_PRIME)
    {
      if (handleLowFlowCondition(i, currentTime))
      {
        continue;
      }
    }
    else
    {
      primeLowFlowTime[i] = 0;
    }

    // 3. Bubble Detection Logic
    if (!primeModeFailed[i] && !primeModeSuccess[i])
    {
      if (readBinarySensor(reagentBubbleSensors[i]))
      {
        handleBubbleDetected(i, currentTime);
      }
      else
      {
        if (handleNoBubbleDetected(i, currentTime))
        {
          continue;
        }
      }
    }

    // 4. Additional Prime Time Check
    if (primeAdditionalTime[i] != 0)
    {
      if (currentTime - primeAdditionalTime[i] >= ADDITIONAL_PRIME_TIME_MS)
      {
        handlePrimingComplete(i);
      }
    }
  }
}

// Helper functions to improve readability

void resetPrimingStates(int i)
{
  primeAsyncCompleted[i] = false;
  primeModeFailed[i] = false;
  primeModeSuccess[i] = false;
  primeModeStartTime[i] = 0;
  primeStableDetectTime[i] = 0;
  primeAdditionalTime[i] = 0;
  primeLowFlowTime[i] = 0;
}

void handlePrimingOverflow(int i)
{
  sendMessage(F("[ERROR] Priming aborted for valve "), &Serial, currentClient, false);
  sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
  sendMessage(F(" due to overflow detected."), &Serial, currentClient);

  closeDispenseValves(i + 1);
  valveControls[i].isPriming = false;
  primeAsyncCompleted[i] = true;

  if (hasActiveClient)
  {
    cm_commandCompleted(&currentClient);
  }
  else
  {
    cm_commandCompleted(&Serial);
  }

  resetPrimingStates(i);
  primeModeFailed[i] = true;
}

bool handleLowFlowCondition(int i, unsigned long currentTime)
{
  if (primeLowFlowTime[i] == 0)
  {
    primeLowFlowTime[i] = currentTime;
    return false;
  }

  if (currentTime - primeLowFlowTime[i] >= PRIMING_FLOW_TIMEOUT_MS)
  {
    sendMessage(F("[ERROR] Priming failed for valve "), &Serial, currentClient, false);
    sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
    sendMessage(F(" due to insufficient flow."), &Serial, currentClient);

    closeDispenseValves(i + 1);
    valveControls[i].isPriming = false;
    primeAsyncCompleted[i] = true;

    if (hasActiveClient)
    {
      cm_commandCompleted(&currentClient);
    }
    else
    {
      cm_commandCompleted(&Serial);
    }

    resetPrimingStates(i);
    primeModeFailed[i] = true;
    return true;
  }
  return false;
}

void handleBubbleDetected(int i, unsigned long currentTime)
{
  if (primeStableDetectTime[i] == 0)
  {
    primeStableDetectTime[i] = currentTime;
  }
  else if (currentTime - primeStableDetectTime[i] >= STABLE_DETECTION_PERIOD_MS)
  {
    if (primeAdditionalTime[i] == 0)
    {
      primeAdditionalTime[i] = currentTime;
      primeModeStartTime[i] = 0;
    }
  }
}

bool handleNoBubbleDetected(int i, unsigned long currentTime)
{
  primeStableDetectTime[i] = 0;
  if (currentTime - primeModeStartTime[i] >= PRIME_TIMEOUT_MS)
  {
    sendMessage(F("[ERROR] Priming failed for valve "), &Serial, currentClient, false);
    sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
    sendMessage(F(" due to no liquid detected."), &Serial, currentClient);

    closeDispenseValves(i + 1);
    valveControls[i].isPriming = false;
    primeAsyncCompleted[i] = true;

    if (hasActiveClient)
    {
      cm_commandCompleted(&currentClient);
    }
    else
    {
      cm_commandCompleted(&Serial);
    }

    primeModeFailed[i] = true;
    resetPrimingStates(i);
    return true;
  }
  return false;
}

void handlePrimingComplete(int i)
{
  closeDispenseValves(i + 1);

  sendMessage(F("[MESSAGE] Priming complete for valve "), &Serial, currentClient, false);
  sendMessage(String(i + 1).c_str(), &Serial, currentClient);

  valveControls[i].isPriming = false;
  primeModeSuccess[i] = true;
  primeAsyncCompleted[i] = true;

  if (hasActiveClient)
  {
    cm_commandCompleted(&currentClient);
  }
  else
  {
    cm_commandCompleted(&Serial);
  }

  resetPrimingStates(i);
}

// ============================================================
// Fill System Implementation
// ============================================================

void monitorFillSensors(unsigned long currentTime)
{
  const float MAX_FILL_VOLUME_ML = 200.0;
  const unsigned long MAX_FILL_TIME_MS = 60000;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_FILL = 1;
  const unsigned long SENSOR_CHECK_INTERVAL = 500;

  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    if (!valveControls[i].fillMode)
    {
      // Reset states when not in fill mode
      fillModeStartTime[i] = 0;
      fillModeLowFlowTime[i] = 0;
      fillModeLastCheck[i] = 0;
      continue;
    }

    FlowSensor *sensor = flowSensors[i];
    if (!sensor)
      continue;

    // Initialize fill operation if needed
    if (fillModeStartTime[i] == 0)
    {
      fillModeStartTime[i] = currentTime;
      fillModeInitialVolume[i] = sensor->dispenseVolume;
      fillModeLowFlowTime[i] = 0;
    }

    float addedVolume = sensor->dispenseVolume - fillModeInitialVolume[i];

    // Check maximum fill time (primary safety check)
    if (currentTime - fillModeStartTime[i] >= MAX_FILL_TIME_MS)
    {
      fill_handleMaxTimeReached(i + 1);
      continue;
    }

    // Check maximum volume
    if (addedVolume >= MAX_FILL_VOLUME_ML)
    {
      fill_handleMaxVolumeReached(i + 1);
      continue;
    }

    // Monitor flow rate
    if (sensor->flowRate < MIN_FLOW_RATE_FILL)
    {
      if (fillModeLowFlowTime[i] == 0)
      {
        fillModeLowFlowTime[i] = currentTime;
      }
      else if ((currentTime - fillModeLowFlowTime[i]) >= FLOW_TIMEOUT_MS)
      {
        fill_handleFlowTimeout(i + 1);
        continue;
      }
    }
    else
    {
      fillModeLowFlowTime[i] = 0;
    }

    // Periodic overflow sensor check
    if (currentTime - fillModeLastCheck[i] >= SENSOR_CHECK_INTERVAL)
    {
      fillModeLastCheck[i] = currentTime;
      fill_handleOverflowCheck(i + 1);
    }
  }
}

void fill_handleMaxTimeReached(int trough)
{
  sendMessage(F("[MESSAGE] Fill complete (max time) for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient);

  closeDispenseValves(trough);
  valveControls[trough - 1].fillMode = false;
  fillModeStartTime[trough - 1] = 0;
}

void fill_handleMaxVolumeReached(int trough)
{
  sendMessage(F("[MESSAGE] Fill complete (max volume) for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient);

  closeDispenseValves(trough);
  valveControls[trough - 1].fillMode = false;
  fillModeStartTime[trough - 1] = 0;
}

void fill_handleFlowTimeout(int trough)
{
  sendMessage(F("[ERROR] Fill timeout (insufficient flow) for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient);

  closeDispenseValves(trough);
  valveControls[trough - 1].fillMode = false;
  fillModeStartTime[trough - 1] = 0;
}

void fill_handleOverflowCheck(int trough)
{
  if (readBinarySensor(overflowSensors[trough - 1]))
  {
    if (areDispenseValvesOpen(trough))
    {
      sendMessage(F("[MESSAGE] Overflow condition detected for trough "), &Serial, currentClient, false);
      sendMessage(String(trough).c_str(), &Serial, currentClient, false);
      sendMessage(F(" - temporarily closing valves to prevent overfill."), &Serial, currentClient);
      closeDispenseValves(trough);
    }
  }
  else
  {
    if (!areDispenseValvesOpen(trough))
    {
      openDispenseValves(trough);
      sendMessage(F("[MESSAGE] No overflow detected for trough "), &Serial, currentClient, false);
      sendMessage(String(trough).c_str(), &Serial, currentClient, false);
      sendMessage(F(" - valves re-opened to resume filling."), &Serial, currentClient);
    }
  }
}

// ============================================================
// Waste System Implementation
// ============================================================

void monitorWasteSensors(unsigned long currentTime)
{
  const unsigned long DRAIN_COMPLETE_DELAY = 3000;
  const unsigned long MAX_DRAIN_TIME = 60000;
  const unsigned long DRAIN_INITIATE_TIMEOUT = 25000;

  for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
  {
    bool liquidDetected = readBinarySensor(wasteLineSensors[sensorIdx]);

    for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
    {
      if (!valveControls[i].isDraining)
        continue;

      // Initialize drain start time if needed
      if (valveControls[i].drainStartTime == 0)
      {
        valveControls[i].drainStartTime = currentTime;
        sendMessage(F("[DEBUG] Trough "), &Serial, currentClient, false);
        sendMessage(String(i + 1).c_str(), &Serial, currentClient, false);
        sendMessage(F(" drainStartTime set to: "), &Serial, currentClient, false);
        sendMessage(String(currentTime).c_str(), &Serial, currentClient);
      }

      // Check for maximum drain time timeout
      if (currentTime - valveControls[i].drainStartTime >= MAX_DRAIN_TIME)
      {
        waste_handleMaxDrainTimeout(i + 1, currentTime - valveControls[i].drainStartTime);
        if (!drainAsyncCompleted[i])
        {
          if (hasActiveClient)
          {
            cm_commandCompleted(&currentClient);
          }
          else
          {
            cm_commandCompleted(&Serial);
          }
          drainAsyncCompleted[i] = true;
        }
        continue;
      }

      // Check for initiation timeout if no liquid detected
      if (!liquidDetected && !wasteLiquidDetected[sensorIdx] &&
          currentTime - valveControls[i].drainStartTime >= DRAIN_INITIATE_TIMEOUT)
      {
        waste_handleInitiationTimeout(i + 1);
        if (!drainAsyncCompleted[i])
        {
          if (hasActiveClient)
          {
            cm_commandCompleted(&currentClient);
          }
          else
          {
            cm_commandCompleted(&Serial);
          }
          drainAsyncCompleted[i] = true;
        }
      }
    }

    // Check waste bottle full condition
    if (readBinarySensor(wasteBottleSensors[sensorIdx]))
    {
      for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
      {
        if (valveControls[i].isDraining)
        {
          waste_handleBottleFull(i + 1);
          if (!drainAsyncCompleted[i])
          {
            if (hasActiveClient)
            {
              cm_commandCompleted(&currentClient);
            }
            else
            {
              cm_commandCompleted(&Serial);
            }
            drainAsyncCompleted[i] = true;
          }
        }
      }
      continue;
    }

    // Monitor drain completion
    if (readBinarySensor(wasteLineSensors[sensorIdx]))
    {
      wasteDrainCompleteTime[sensorIdx] = currentTime;
      wasteLiquidDetected[sensorIdx] = true;
    }
    else if (wasteLiquidDetected[sensorIdx] &&
             (currentTime - wasteDrainCompleteTime[sensorIdx] >= DRAIN_COMPLETE_DELAY))
    {
      for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
      {
        if (valveControls[i].isDraining)
        {
          waste_handleDrainComplete(i + 1);
          if (!drainAsyncCompleted[i])
          {
            if (hasActiveClient)
            {
              cm_commandCompleted(&currentClient);
            }
            else
            {
              cm_commandCompleted(&Serial);
            }
            drainAsyncCompleted[i] = true;
          }
        }
      }
      wasteDrainCompleteTime[sensorIdx] = 0;
      wasteLiquidDetected[sensorIdx] = false;
      wasteVacuumReleased[sensorIdx] = false;
    }

    // Check vacuum release
    if (!wasteVacuumReleased[sensorIdx] && !readBinarySensor(wasteVacuumSensors[sensorIdx]))
    {
      if (sensorIdx == 0)
      {
        wasteValve3 = closeValve(wasteValve3);
        sendMessage(F("[MESSAGE] Vacuum released. Waste valve 3 closed."), &Serial, currentClient);
      }
      else
      {
        wasteValve4 = closeValve(wasteValve4);
        sendMessage(F("[MESSAGE] Vacuum released. Waste valve 4 closed."), &Serial, currentClient);
      }
      wasteVacuumReleased[sensorIdx] = true;
    }
  }
}

// Helper functions updated to use sendMessage
void waste_handleDrainTimeout(int trough, unsigned long drainDuration)
{
  valveControls[trough - 1].isDraining = false;
  valveControls[trough - 1].drainStartTime = 0;

  // Handle valve control based on trough number
  switch (trough)
  {
  case 1:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = openValve(wasteValve3);
    break;
  case 2:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = closeValve(wasteValve3);
    break;
  case 3:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = openValve(wasteValve4);
    break;
  case 4:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = closeValve(wasteValve4);
    break;
  }

  sendMessage(F("[ERROR] Draining timeout for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient, false);
  sendMessage(F(" after "), &Serial, currentClient, false);
  sendMessage(String(drainDuration).c_str(), &Serial, currentClient, false);
  sendMessage(F(" ms (maximum drain time reached)."), &Serial, currentClient);
}

void waste_handleBottleFull(int trough)
{
  valveControls[trough - 1].isDraining = false;
  valveControls[trough - 1].drainStartTime = 0;

  if (trough <= 2)
  {
    wasteValve1 = closeValve(wasteValve1);
  }
  else
  {
    wasteValve2 = closeValve(wasteValve2);
  }

  sendMessage(F("[ERROR] Draining halted for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient, false);
  sendMessage(F(" because the waste bottle is full."), &Serial, currentClient);
}

void waste_handleDrainComplete(int trough)
{
  valveControls[trough - 1].isDraining = false;
  valveControls[trough - 1].drainStartTime = 0;

  switch (trough)
  {
  case 1:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = openValve(wasteValve3);
    break;
  case 2:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = closeValve(wasteValve3);
    break;
  case 3:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = openValve(wasteValve4);
    break;
  case 4:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = closeValve(wasteValve4);
    break;
  }

  sendMessage(F("[MESSAGE] Draining complete for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient);
}

void waste_handleMaxDrainTimeout(int trough, unsigned long drainDuration)
{
  valveControls[trough - 1].isDraining = false;
  valveControls[trough - 1].drainStartTime = 0;

  // Handle valve controls
  switch (trough)
  {
  case 1:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = openValve(wasteValve3);
    break;
  case 2:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = closeValve(wasteValve3);
    break;
  case 3:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = openValve(wasteValve4);
    break;
  case 4:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = closeValve(wasteValve4);
    break;
  }

  sendMessage(F("[ERROR] Draining timeout for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient, false);
  sendMessage(F(" after "), &Serial, currentClient, false);
  sendMessage(String(drainDuration).c_str(), &Serial, currentClient, false);
  sendMessage(F(" ms (maximum drain time reached)."), &Serial, currentClient);
}

void waste_handleInitiationTimeout(int trough)
{
  valveControls[trough - 1].isDraining = false;
  valveControls[trough - 1].drainStartTime = 0;

  // Handle valve controls
  switch (trough)
  {
  case 1:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = openValve(wasteValve3);
    break;
  case 2:
    wasteValve1 = closeValve(wasteValve1);
    wasteValve3 = closeValve(wasteValve3);
    break;
  case 3:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = openValve(wasteValve4);
    break;
  case 4:
    wasteValve2 = closeValve(wasteValve2);
    wasteValve4 = closeValve(wasteValve4);
    break;
  }

  sendMessage(F("[ERROR] Draining initiation timeout for trough "), &Serial, currentClient, false);
  sendMessage(String(trough).c_str(), &Serial, currentClient, false);
  sendMessage(F(" (no liquid detected in drain line)."), &Serial, currentClient);
}

void monitorVacuumRelease(unsigned long currentTime)
{
  for (int bottleIdx = 0; bottleIdx < 2; bottleIdx++)
  {
    // Skip if vacuum monitoring is not active
    if (!globalVacuumMonitoring[bottleIdx])
      continue;

    // Check if vacuum has been released
    if (!readBinarySensor(wasteVacuumSensors[bottleIdx]))
    {
      vacuum_handleVacuumRelease(bottleIdx);
    }
  }
}

void vacuum_handleVacuumRelease(int bottleIdx)
{
  if (bottleIdx == 0)
  {
    wasteValve3 = closeValve(wasteValve3);
    sendMessage(F("[MESSAGE] Vacuum released. Waste valve 3 closed."), &Serial, currentClient);
  }
  else
  {
    wasteValve4 = closeValve(wasteValve4);
    sendMessage(F("[MESSAGE] Vacuum released. Waste valve 4 closed."), &Serial, currentClient);
  }

  globalVacuumMonitoring[bottleIdx] = false;

  sendMessage(F("[MESSAGE] Vacuum monitoring disabled for bottle "), &Serial, currentClient, false);
  sendMessage(String(bottleIdx + 1).c_str(), &Serial, currentClient);

  if (hasActiveClient)
  {
    cm_commandCompleted(&currentClient);
    cm_commandCompleted(&Serial);
  }
  else
  {
    cm_commandCompleted(&Serial);
  }
}

// ============================================================
// Safety Systems Implementation
// ============================================================

// Enclosure Protection
void monitorEnclosureLiquidSensor(unsigned long currentTime)
{

  const unsigned long CHECK_INTERVAL = 25; // check every 25 ms

  if (currentTime - enclosureLeakCheckTime < CHECK_INTERVAL)
  {
    return;
  }
  enclosureLeakCheckTime = currentTime;

  if (readBinarySensor(enclosureLiquidSensor))
  {
    enclosure_handleLeakDetected(currentTime, enclosureLeakErrorTime);
  }
  else
  {
    enclosure_handleNoLeak();
  }
}

void enclosure_handleLeakDetected(unsigned long currentTime, unsigned long &enclosureLeakErrorTime)
{
  const unsigned long ERROR_PRINT_INTERVAL = 15000; // reprint error every 15 sec

  globalEnclosureLiquidError = true;
  if (!enclosureLeakAbortCalled)
  {
    // Abort operations on both streams if network client is connected
    if (hasActiveClient)
    {
      abortAllAutomatedOperations(&currentClient);
    }
    else
    {
      abortAllAutomatedOperations(&Serial);
    }
    enclosureLeakAbortCalled = true;
  }

  if (currentTime - enclosureLeakErrorTime >= ERROR_PRINT_INTERVAL)
  {
    enclosureLeakErrorTime = currentTime;
    sendMessage(F("[ERROR] Enclosure liquid leak detected. Operations halted."), &Serial, currentClient);
  }
}

void enclosure_handleNoLeak()
{
  globalEnclosureLiquidError = false;
  enclosureLeakAbortCalled = false; // Reset when leak is cleared
}

// Temperature Control
void monitorEnclosureTemp(unsigned long currentTime)
{
  // Do nothing if manual mode is active
  if (!fanAutoMode)
    return;

  // Static variables preserve state between calls
  static bool enclosureFanAutoActive = false;
  static unsigned long enclosureTempWarningTime = 0;

  // Read temperature sensor
  TempHumidity th = readTempHumidity();
  if (!th.valid)
  {
    sendMessage(F("[ERROR] Failed to read enclosure temperature!"), &Serial, currentClient);
    return;
  }

  float currentTemp = th.temperature;

  // Temperature control logic
  if (currentTemp > ENCLOSURE_TEMP_SETPOINT)
  {
    temp_handleHighTemperature(currentTemp, enclosureFanAutoActive);
  }
  else if (enclosureFanAutoActive && currentTemp <= 25)
  {
    temp_handleNormalTemperature(enclosureFanAutoActive);
  }

  // Warning message for high temperature
  if (enclosureFanAutoActive)
  {
    temp_printWarning(currentTemp, currentTime, enclosureTempWarningTime);
  }

  // Ensure fan is off if not locked on
  if (!enclosureFanAutoActive)
  {
    setFanState(fan, false);
  }
}

void temp_handleHighTemperature(float currentTemp, bool &enclosureFanAutoActive)
{
  if (!enclosureFanAutoActive)
  {
    enclosureFanAutoActive = true;
    setFanState(fan, true);
  }
}

void temp_handleNormalTemperature(bool &enclosureFanAutoActive)
{
  enclosureFanAutoActive = false;
  setFanState(fan, false);
}

void temp_printWarning(float currentTemp, unsigned long currentTime, unsigned long &enclosureTempWarningTime)
{
  if (currentTime - enclosureTempWarningTime >= 60000)
  {
    sendMessage(F("[WARNING] Enclosure temperature "), &Serial, currentClient, false);
    sendMessage(String(currentTemp).c_str(), &Serial, currentClient, false);
    sendMessage(F("C exceeds threshold. Fan will remain on until temp <= 25C."), &Serial, currentClient);
    enclosureTempWarningTime = currentTime;
  }
}

// Flow Sensor Monitoring
void monitorFlowSensorConnections(unsigned long currentTime)
{
  static unsigned long flowSensorCheckTime = 0;
  const unsigned long PRINT_INTERVAL = 30000; // 30 seconds

  // Only check and print every 30 seconds
  if (currentTime - flowSensorCheckTime < PRINT_INTERVAL)
  {
    return;
  }
  flowSensorCheckTime = currentTime;

  FlowSensor *sensors[] = {&flow1, &flow2, &flow3, &flow4};
  for (int i = 0; i < NUM_FLOW_SENSORS; i++)
  {
    if (!isFlowSensorConnected(*sensors[i]))
    {
      sendMessage(F("[ERROR] Flow sensor on channel "), &Serial, currentClient, false);
      sendMessage(String(sensors[i]->channel).c_str(), &Serial, currentClient, false);
      sendMessage(F(" not connected."), &Serial, currentClient);
    }
  }
}

// ============================================================
// Reset Functions
// ============================================================

void resetPrimeMonitorState()
{
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    primeModeStartTime[i] = 0;
    primeStableDetectTime[i] = 0;
    primeAdditionalTime[i] = 0;
    primeLowFlowTime[i] = 0;
    primeModeFailed[i] = false;
    primeModeSuccess[i] = false;
  }
}

void resetFillMonitorState()
{
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++)
  {
    fillModeStartTime[i] = 0;
    fillModeLowFlowTime[i] = 0;
    fillModeInitialVolume[i] = 0;
    fillModeLastCheck[i] = 0;
  }
}

void resetWasteMonitorState()
{
  for (int i = 0; i < 2; i++)
  {
    wasteDrainCompleteTime[i] = 0;
    wasteLiquidDetected[i] = false;
    wasteVacuumReleased[i] = false;
  }
}

void resetEnclosureLeakMonitorState()
{
  enclosureLeakCheckTime = 0;
  enclosureLeakErrorTime = 0;
  enclosureLeakAbortCalled = false;
}
