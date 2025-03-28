#include "SystemMonitor.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Utils.h"
#include <Wire.h>
#include "CommandSession.h"  // For asyncCommandCompleted()

/************************************************************
 * SystemMonitor.cpp
 *
 * This file implements the monitoring functions for the Bulk 
 * Dispense system. It periodically checks hardware and sensor 
 * states and logs events, handles errors, and triggers asynchronous 
 * command completions via the command session mechanism.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Helper Functions: Overflow and Timeout Handling
// ============================================================
void handleOverflowCondition(int triggeredTrough) {
  if (valveControls[triggeredTrough - 1].isDispensing) {
    Serial.print(F("[WARNING] Overflow detected for Trough "));
    Serial.println(triggeredTrough);
    closeDispenseValves(triggeredTrough);
    Serial.print(F("[MESSAGE] Closed reagent and media valves for Trough "));
    Serial.println(triggeredTrough);
    
    FlowSensor* sensor = flowSensors[triggeredTrough - 1];
    if (sensor) {
      Serial.print(F("[MESSAGE] Dispensed volume before overflow: "));
      Serial.print(sensor->dispenseVolume, 1);
      Serial.println(F(" mL."));
      stopFlowSensorMeasurement(*sensor);
      resetFlowSensorDispenseVolume(*sensor);
    }
    
    valveControls[triggeredTrough - 1].isDispensing = false;
    valveControls[triggeredTrough - 1].lastFlowCheckTime = 0;
    valveControls[triggeredTrough - 1].lastFlowChangeTime = 0;
    valveControls[triggeredTrough - 1].dispensingValveNumber = -1;
  }
}

void handleTimeoutCondition(int troughNumber) {
  if (valveControls[troughNumber - 1].isDispensing) {
    Serial.print(F("[ERROR] Timeout: No or insufficient flow detected for Trough "));
    Serial.print(troughNumber);
    Serial.println(F(". Stopping dispense."));
    
    closeDispenseValves(troughNumber);
    
    FlowSensor* sensor = flowSensors[troughNumber - 1];
    if (sensor) {
      Serial.print(F("[MESSAGE] Dispensed volume before timeout: "));
      Serial.print(sensor->dispenseVolume, 1);
      Serial.println(F(" mL."));
      stopFlowSensorMeasurement(*sensor);
      resetFlowSensorDispenseVolume(*sensor);
    }
    
    valveControls[troughNumber - 1].isDispensing = false;
    valveControls[troughNumber - 1].lastFlowCheckTime = 0;
    valveControls[troughNumber - 1].lastFlowChangeTime = 0;
    valveControls[troughNumber - 1].dispensingValveNumber = -1;
    valveControls[troughNumber - 1].targetVolume = -1;
  }
}

// ============================================================
// Monitor Overflow Sensors
// ============================================================
void monitorOverflowSensors(unsigned long currentTime) {
  static unsigned long previousOverflowCheckTime = 0;
  
  // Check every 25ms (non-blocking)
  if (currentTime - previousOverflowCheckTime >= 25) {
    previousOverflowCheckTime = currentTime;
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      if (readBinarySensor(overflowSensors[i])) {
        handleOverflowCondition(i + 1);
      }
    }
  }
}

// ============================================================
// Monitor Flow Sensors (Dispense Monitoring)
// ============================================================
void monitorFlowSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_THRESHOLD = 1.0;
  const float MAX_TROUGH_VOLUME = 205.0;  // Maximum safe volume per trough
  
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      if (!valveControls[i].isDispensing)
        continue;
      
      FlowSensor* sensor = flowSensors[i];
      if (!sensor)
        continue;
      
      // Check for overflow.
      if (readBinarySensor(overflowSensors[i])) {
        closeDispenseValves(i + 1);
        Serial.print(F("[WARNING] Overflow detected for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Dispensed volume: "));
        Serial.print(sensor->dispenseVolume, 1);
        Serial.println(F(" mL."));
        resetFlowSensorDispenseVolume(*sensor);
        stopFlowSensorMeasurement(*sensor);
        valveControls[i].isDispensing = false;
        // Signal asynchronous completion if not already done.
        if (!dispenseAsyncCompleted[i]) {
          asyncCommandCompleted(&Serial);
          dispenseAsyncCompleted[i] = true;
        }
        continue;
      }
      
      // Skip automated checks if manual control is active.
      if (valveControls[i].manualControl) {
        valveControls[i].lastFlowCheckTime = 0;
        valveControls[i].lastFlowChangeTime = 0;
        continue;
      }
      
      // Check for no-flow timeout.
      if (sensor->flowRate < MIN_FLOW_RATE_THRESHOLD) {
        if (valveControls[i].lastFlowCheckTime == 0) {
          valveControls[i].lastFlowCheckTime = currentTime;
        } else if (currentTime - valveControls[i].lastFlowCheckTime >= FLOW_TIMEOUT_MS) {
          handleTimeoutCondition(i + 1);
          if (!dispenseAsyncCompleted[i]) {
            asyncCommandCompleted(&Serial);
            dispenseAsyncCompleted[i] = true;
          }
          continue;
        }
      } else {
        valveControls[i].lastFlowCheckTime = 0;
      }
      
      // Check if requested volume is reached.
      if (valveControls[i].targetVolume > 0 && sensor->dispenseVolume >= valveControls[i].targetVolume) {
        closeDispenseValves(i + 1);
        Serial.print(F("[MESSAGE] Dispense complete for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Final volume dispensed: "));
        Serial.print(sensor->dispenseVolume, 1);
        Serial.println(F(" mL."));
        resetFlowSensorDispenseVolume(*sensor);
        stopFlowSensorMeasurement(*sensor);
        valveControls[i].isDispensing = false;
        if (!dispenseAsyncCompleted[i]) {
          asyncCommandCompleted(&Serial);
          dispenseAsyncCompleted[i] = true;
        }
        continue;
      }
      
      // Secondary safety: check if maximum trough volume is exceeded.
      if (sensor->dispenseVolume >= MAX_TROUGH_VOLUME) {
        closeDispenseValves(i + 1);
        Serial.print(F("[ERROR] Safety Limit Reached! Dispense stopped for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Final dispensed volume: "));
        Serial.print(sensor->dispenseVolume, 1);
        Serial.print(F(" mL. (Max Allowed: "));
        Serial.print(MAX_TROUGH_VOLUME);
        Serial.println(F(" mL)"));
        resetFlowSensorDispenseVolume(*sensor);
        stopFlowSensorMeasurement(*sensor);
        valveControls[i].isDispensing = false;
        if (!dispenseAsyncCompleted[i]) {
          asyncCommandCompleted(&Serial);
          dispenseAsyncCompleted[i] = true;
        }
      }
    }
  }
}

// ============================================================
// Monitor Prime Sensors
// ============================================================
const unsigned long PRIME_TIMEOUT_MS = 6000;
const unsigned long STABLE_DETECTION_PERIOD_MS = 500;
const unsigned long ADDITIONAL_PRIME_TIME_MS = 2000;
const unsigned long PRIMING_FLOW_TIMEOUT_MS = 5000;
const float MIN_FLOW_RATE_PRIME = 5.0;

static unsigned long primeStartTime[NUM_OVERFLOW_SENSORS] = {0, 0, 0, 0};
static unsigned long stableDetectionStartTime[NUM_OVERFLOW_SENSORS] = {0, 0, 0, 0};
static unsigned long additionalPrimeStartTime[NUM_OVERFLOW_SENSORS] = {0, 0, 0, 0};
static unsigned long lowFlowStartTime[NUM_OVERFLOW_SENSORS] = {0, 0, 0, 0};
static bool primingFailed[NUM_OVERFLOW_SENSORS] = {false, false, false, false};
static bool primingSuccess[NUM_OVERFLOW_SENSORS] = {false, false, false, false};
// static bool primeAsyncCompleted[NUM_OVERFLOW_SENSORS] = {false, false, false, false};

void monitorPrimeSensors(unsigned long currentTime) {
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    if (valveControls[i].isPriming) {
      // Check for overflow.
      if (readBinarySensor(overflowSensors[i])) {
        Serial.print(F("[ERROR] Priming aborted for valve "));
        Serial.print(i + 1);
        Serial.println(F(" due to overflow detected."));
        closeDispenseValves(i + 1);
        valveControls[i].isPriming = false;
        primeAsyncCompleted[i] = true;
        asyncCommandCompleted(&Serial);
        primeStartTime[i] = 0;
        stableDetectionStartTime[i] = 0;
        additionalPrimeStartTime[i] = 0;
        lowFlowStartTime[i] = 0;
        primingFailed[i] = true;
        continue;
      }

      // Start prime timer if not set.
      if (primeStartTime[i] == 0) {
        primeStartTime[i] = currentTime;
      }

      // Check flow rate.
      FlowSensor* sensor = flowSensors[i];
      if (sensor) {
        if (sensor->flowRate < MIN_FLOW_RATE_PRIME) {
          if (lowFlowStartTime[i] == 0) {
            lowFlowStartTime[i] = currentTime;
          } else if (currentTime - lowFlowStartTime[i] >= PRIMING_FLOW_TIMEOUT_MS) {
            Serial.print(F("[ERROR] Priming failed for valve "));
            Serial.print(i + 1);
            Serial.println(F(" due to insufficient flow."));
            closeDispenseValves(i + 1);
            valveControls[i].isPriming = false;
            primeAsyncCompleted[i] = true;
            asyncCommandCompleted(&Serial);
            primeStartTime[i] = 0;
            stableDetectionStartTime[i] = 0;
            additionalPrimeStartTime[i] = 0;
            lowFlowStartTime[i] = 0;
            primingFailed[i] = true;
            continue;
          }
        } else {
          lowFlowStartTime[i] = 0;
        }
      }

      // Check for bubble detection timeout.
      if (!readBinarySensor(reagentBubbleSensors[i]) &&
          (currentTime - primeStartTime[i] >= PRIME_TIMEOUT_MS) &&
          !primingFailed[i] && !primingSuccess[i]) {
        Serial.print(F("[ERROR] Priming failed for valve "));
        Serial.print(i + 1);
        Serial.println(F(" due to no liquid detected."));
        closeDispenseValves(i + 1);
        valveControls[i].isPriming = false;
        primeAsyncCompleted[i] = true;
        asyncCommandCompleted(&Serial);
        primingFailed[i] = true;
        primeStartTime[i] = 0;
        additionalPrimeStartTime[i] = 0;
        stableDetectionStartTime[i] = 0;
        continue;
      }

      // Handle stable bubble detection.
      if (readBinarySensor(reagentBubbleSensors[i]) && stableDetectionStartTime[i] == 0 && !primingFailed[i]) {
        stableDetectionStartTime[i] = currentTime;
      } else if (!readBinarySensor(reagentBubbleSensors[i])) {
        stableDetectionStartTime[i] = 0;
      }

      // Start additional priming once stable detection is confirmed.
      if (stableDetectionStartTime[i] != 0 && (currentTime - stableDetectionStartTime[i] >= STABLE_DETECTION_PERIOD_MS)) {
        if (additionalPrimeStartTime[i] == 0) {
          additionalPrimeStartTime[i] = currentTime;
          primeStartTime[i] = 0;
        }
      }

      // Complete priming.
      if (additionalPrimeStartTime[i] != 0 && (currentTime - additionalPrimeStartTime[i] >= ADDITIONAL_PRIME_TIME_MS)) {
        closeDispenseValves(i + 1);
        Serial.print(F("[MESSAGE] Priming complete for valve "));
        Serial.println(i + 1);
        valveControls[i].isPriming = false;
        primingSuccess[i] = true;
        primeAsyncCompleted[i] = true;
        asyncCommandCompleted(&Serial);
        primeStartTime[i] = 0;
        stableDetectionStartTime[i] = 0;
        additionalPrimeStartTime[i] = 0;
        lowFlowStartTime[i] = 0;
      }
    } else {
      // Reset timers and flags for valves not priming.
      primeAsyncCompleted[i] = false;
      primingFailed[i] = false;
      primingSuccess[i] = false;
      primeStartTime[i] = 0;
      stableDetectionStartTime[i] = 0;
      additionalPrimeStartTime[i] = 0;
      lowFlowStartTime[i] = 0;
    }
  }
}

// ============================================================
// Monitor Fill Sensors
// ============================================================
void monitorFillSensors(unsigned long currentTime) {
  const float MAX_FILL_VOLUME_ML = 200.0;
  const unsigned long MAX_FILL_TIME_MS = 60000;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_FILL = 1;
  const unsigned long SENSOR_CHECK_INTERVAL = 500;
  
  static unsigned long fillStartTime[NUM_OVERFLOW_SENSORS] = {0};
  static unsigned long lowFlowStartTime[NUM_OVERFLOW_SENSORS] = {0};
  static float initialVolume[NUM_OVERFLOW_SENSORS] = {0};
  static unsigned long lastSensorCheck[NUM_OVERFLOW_SENSORS] = {0};
  
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    if (valveControls[i].fillMode) {
      FlowSensor* sensor = flowSensors[i];
      if (!sensor) {
        continue;
      }
      
      if (fillStartTime[i] == 0) {
        fillStartTime[i] = currentTime;
        initialVolume[i] = sensor->dispenseVolume;
        lowFlowStartTime[i] = 0;
      }
      
      float addedVolume = sensor->dispenseVolume - initialVolume[i];
      
      if (sensor->flowRate < MIN_FLOW_RATE_FILL) {
        if (lowFlowStartTime[i] == 0) {
          lowFlowStartTime[i] = currentTime;
        } else if ((currentTime - lowFlowStartTime[i]) >= FLOW_TIMEOUT_MS) {
          Serial.print(F("[ERROR] Fill timeout (insufficient flow) for trough "));
          Serial.println(i + 1);
          closeDispenseValves(i + 1);
          valveControls[i].fillMode = false;
          fillStartTime[i] = 0;
          lowFlowStartTime[i] = 0;
          continue;
        }
      } else {
        lowFlowStartTime[i] = 0;
      }
      
      bool volumeExceeded = addedVolume >= MAX_FILL_VOLUME_ML;
      bool timeExceeded = (currentTime - fillStartTime[i]) >= MAX_FILL_TIME_MS;
      if (volumeExceeded || timeExceeded) {
        Serial.print(F("[MESSAGE] Fill complete for trough "));
        Serial.println(i + 1);
        closeDispenseValves(i + 1);
        valveControls[i].fillMode = false;
        fillStartTime[i] = 0;
        lowFlowStartTime[i] = 0;
        continue;
      }
      
      if ((currentTime - lastSensorCheck[i]) >= SENSOR_CHECK_INTERVAL) {
        lastSensorCheck[i] = currentTime;
        if (readBinarySensor(overflowSensors[i])) {
          Serial.print(F("[MESSAGE] Overflow condition detected for trough "));
          Serial.println(i + 1);
          Serial.println(F(" - temporarily closing valves to prevent overfill."));
          closeDispenseValves(i + 1);
        } else {
          if (!areDispenseValvesOpen(i + 1)) {
            openDispenseValves(i + 1);
            Serial.print(F("[MESSAGE] No overflow detected for trough "));
            Serial.print(i + 1);
            Serial.println(F(" - valves re-opened to resume filling."));
          }
        }
      }
    } else {
      fillStartTime[i] = 0;
      lowFlowStartTime[i] = 0;
      lastSensorCheck[i] = 0;
    }
  }
}

// // ============================================================
// // Monitor Waste Sensors (Drainage Monitoring)
// // ============================================================
// void monitorWasteSensors(unsigned long currentTime)
// {
//   const unsigned long DRAIN_COMPLETE_DELAY = 3000; // Delay after liquid stops for drain to be considered complete
//   const unsigned long MAX_DRAIN_TIME = 60000;      // 30 seconds timeout
//   const unsigned long DRAIN_INITIATE_TIMEOUT = 25000;
//   static unsigned long lastDrainCompleteTime[2] = {0, 0};
//   static bool liquidInitiallyDetected[2] = {false, false};
//   static bool vacuumReleased[2] = {false, false};

//   // First, check if any trough has been draining too long (timeout).
//   for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
//   {
//     for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
//     {
//       // NEW: If draining is active but no start time has been recorded, set it now.
//       if (valveControls[i].isDraining && valveControls[i].drainStartTime == 0)
//       {
//         valveControls[i].drainStartTime = currentTime;
//         Serial.print(F("[DEBUG] Trough "));
//         Serial.print(i + 1);
//         Serial.print(F(" drainStartTime set to: "));
//         Serial.println(currentTime);
//       }

//       // Check if the drain has timed out.
//       if (valveControls[i].isDraining &&
//           (currentTime - valveControls[i].drainStartTime >= MAX_DRAIN_TIME))
//       {
//         // Calculate how long the drain has been active.
//         unsigned long drainDuration = currentTime - valveControls[i].drainStartTime;

//         // Reset draining state.
//         valveControls[i].isDraining = false;
//         valveControls[i].drainStartTime = 0; // clear the start time

//         // Update waste valve states and print a more specific error message.
//         if (sensorIdx == 0)
//         {
//           if (i == 0)
//           { // Trough 1
//             wasteValve1 = closeValve(wasteValve1);
//             wasteValve3 = openValve(wasteValve3);
//             Serial.print(F("[ERROR] Draining timeout for trough 1 after "));
//             Serial.print(drainDuration);
//             Serial.println(F(" ms (maximum drain time reached)."));
//           }
//           else if (i == 1)
//           { // Trough 2
//             wasteValve1 = closeValve(wasteValve1);
//             wasteValve3 = closeValve(wasteValve3);
//             Serial.print(F("[ERROR] Draining timeout for trough 2 after "));
//             Serial.print(drainDuration);
//             Serial.println(F(" ms (maximum drain time reached)."));
//           }
//         }
//         else if (sensorIdx == 1)
//         {
//           if (i == 2)
//           { // Trough 3
//             wasteValve2 = closeValve(wasteValve2);
//             wasteValve4 = openValve(wasteValve4);
//             Serial.print(F("[ERROR] Draining timeout for trough 3 after "));
//             Serial.print(drainDuration);
//             Serial.println(F(" ms (maximum drain time reached)."));
//           }
//           else if (i == 3)
//           { // Trough 4
//             wasteValve2 = closeValve(wasteValve2);
//             wasteValve4 = closeValve(wasteValve4);
//             Serial.print(F("[ERROR] Draining timeout for trough 4 after "));
//             Serial.print(drainDuration);
//             Serial.println(F(" ms (maximum drain time reached)."));
//           }
//         }
//         if (!drainAsyncCompleted[i])
//         {
//           asyncCommandCompleted(&Serial);
//           drainAsyncCompleted[i] = true;
//         }
//       }
//     }
//   }

//   // Second loop: Check for drain initiation (no liquid detected) and completion.
//   for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
//   {
//     // NEW: If no liquid is detected by the waste line sensor for this group
//     // and we haven’t yet seen any liquid, check each trough in the group to see if
//     // it has been draining for longer than DRAIN_INITIATE_TIMEOUT.
//     if (!readBinarySensor(wasteLineSensors[sensorIdx]) && !liquidInitiallyDetected[sensorIdx])
//     {
//       for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
//       {
//         if (valveControls[i].isDraining &&
//             (currentTime - valveControls[i].drainStartTime >= DRAIN_INITIATE_TIMEOUT))
//         {
//           // Abort the drain due to no liquid detected.
//           valveControls[i].isDraining = false;
//           valveControls[i].drainStartTime = 0;
//           // Close/open the valves similar to the timeout handling above.
//           if (i == 0)
//           { // Trough 1
//             wasteValve1 = closeValve(wasteValve1);
//             wasteValve3 = openValve(wasteValve3);
//           }
//           else if (i == 1)
//           { // Trough 2
//             wasteValve1 = closeValve(wasteValve1);
//             wasteValve3 = closeValve(wasteValve3);
//           }
//           else if (i == 2)
//           { // Trough 3
//             wasteValve2 = closeValve(wasteValve2);
//             wasteValve4 = openValve(wasteValve4);
//           }
//           else if (i == 3)
//           { // Trough 4
//             wasteValve2 = closeValve(wasteValve2);
//             wasteValve4 = closeValve(wasteValve4);
//           }
//           Serial.print(F("[ERROR] Draining initiation timeout for trough "));
//           Serial.print(i + 1);
//           Serial.println(F(" (no liquid detected in drain line)."));
//           if (!drainAsyncCompleted[i])
//           {
//             asyncCommandCompleted(&Serial);
//             drainAsyncCompleted[i] = true;
//           }
//         }
//       }
//     }

//     // Continue with the rest of the sensor monitoring logic...
//     for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
//     {
//       if (readBinarySensor(wasteBottleSensors[sensorIdx]))
//       {
//         for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
//         {
//           if (valveControls[i].isDraining)
//           {
//             valveControls[i].isDraining = false;
//             valveControls[i].drainStartTime = 0;
//             if (sensorIdx == 0)
//             {
//               wasteValve1 = closeValve(wasteValve1);
//             }
//             else
//             {
//               wasteValve2 = closeValve(wasteValve2);
//             }
//             Serial.print(F("[ERROR] Draining halted for trough "));
//             Serial.print(i + 1);
//             Serial.println(F(" because the waste bottle is full."));
//             if (!drainAsyncCompleted[i])
//             {
//               asyncCommandCompleted(&Serial);
//               drainAsyncCompleted[i] = true;
//             }
//           }
//         }
//         continue;
//       }

//       if (readBinarySensor(wasteLineSensors[sensorIdx]))
//       {
//         lastDrainCompleteTime[sensorIdx] = currentTime;
//         liquidInitiallyDetected[sensorIdx] = true;
//       }
//       else if (liquidInitiallyDetected[sensorIdx] &&
//                (currentTime - lastDrainCompleteTime[sensorIdx] >= DRAIN_COMPLETE_DELAY))
//       {
//         if (sensorIdx == 0)
//         {
//           if (valveControls[0].isDraining)
//           {
//             valveControls[0].isDraining = false;
//             valveControls[0].drainStartTime = 0;
//             wasteValve1 = closeValve(wasteValve1);
//             wasteValve3 = openValve(wasteValve3);
//             Serial.println(F("[MESSAGE] Draining complete for trough 1"));
//             if (!drainAsyncCompleted[0])
//             {
//               asyncCommandCompleted(&Serial);
//               drainAsyncCompleted[0] = true;
//             }
//           }
//           if (valveControls[1].isDraining)
//           {
//             valveControls[1].isDraining = false;
//             valveControls[1].drainStartTime = 0;
//             wasteValve1 = closeValve(wasteValve1);
//             wasteValve3 = closeValve(wasteValve3);
//             Serial.println(F("[MESSAGE] Draining complete for trough 2"));
//             if (!drainAsyncCompleted[1])
//             {
//               asyncCommandCompleted(&Serial);
//               drainAsyncCompleted[1] = true;
//             }
//           }
//         }
//         else if (sensorIdx == 1)
//         {
//           if (valveControls[2].isDraining)
//           {
//             valveControls[2].isDraining = false;
//             valveControls[2].drainStartTime = 0;
//             wasteValve2 = closeValve(wasteValve2);
//             wasteValve4 = openValve(wasteValve4);
//             Serial.println(F("[MESSAGE] Draining complete for trough 3"));
//             if (!drainAsyncCompleted[2])
//             {
//               asyncCommandCompleted(&Serial);
//               drainAsyncCompleted[2] = true;
//             }
//           }
//           if (valveControls[3].isDraining)
//           {
//             valveControls[3].isDraining = false;
//             valveControls[3].drainStartTime = 0;
//             wasteValve2 = closeValve(wasteValve2);
//             wasteValve4 = closeValve(wasteValve4);
//             Serial.println(F("[MESSAGE] Draining complete for trough 4"));
//             if (!drainAsyncCompleted[3])
//             {
//               asyncCommandCompleted(&Serial);
//               drainAsyncCompleted[3] = true;
//             }
//           }
//         }
//         lastDrainCompleteTime[sensorIdx] = 0;
//         liquidInitiallyDetected[sensorIdx] = false;
//         vacuumReleased[sensorIdx] = false;
//       }

//       if (!vacuumReleased[sensorIdx] && !readBinarySensor(wasteVacuumSensors[sensorIdx]))
//       {
//         if (sensorIdx == 0)
//         {
//           wasteValve3 = closeValve(wasteValve3);
//           Serial.println(F("[MESSAGE] Vacuum released. Waste valve 3 closed."));
//         }
//         else if (sensorIdx == 1)
//         {
//           wasteValve4 = closeValve(wasteValve4);
//           Serial.println(F("[MESSAGE] Vacuum released. Waste valve 4 closed."));
//         }
//         vacuumReleased[sensorIdx] = true;
//       }
//     }
//   }
// }

 // ============================================================
// Monitor Waste Sensors (Drainage Monitoring)
// ============================================================
void monitorWasteSensors(unsigned long currentTime)
{
  const unsigned long DRAIN_COMPLETE_DELAY = 3000; // Delay after liquid stops for drain to be considered complete
  const unsigned long MAX_DRAIN_TIME = 60000;      // 60 seconds timeout
  const unsigned long DRAIN_INITIATE_TIMEOUT = 25000;
  static unsigned long lastDrainCompleteTime[2] = {0, 0};
  static bool liquidInitiallyDetected[2] = {false, false};
  static bool vacuumReleased[2] = {false, false};

  // ---------- 1. Drain Timeout Check ----------
  for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
  {
    for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
    {
      // If draining is active but no start time recorded, set it now.
      if (valveControls[i].isDraining && valveControls[i].drainStartTime == 0)
      {
        valveControls[i].drainStartTime = currentTime;
        Serial.print(F("[DEBUG] Trough "));
        Serial.print(i + 1);
        Serial.print(F(" drainStartTime set to: "));
        Serial.println(currentTime);
      }

      // Check if drain has timed out.
      if (valveControls[i].isDraining &&
          (currentTime - valveControls[i].drainStartTime >= MAX_DRAIN_TIME))
      {
        unsigned long drainDuration = currentTime - valveControls[i].drainStartTime;
        valveControls[i].isDraining = false;
        valveControls[i].drainStartTime = 0; // clear start time

        int trough = i + 1;
        switch (trough)
        {
          case 1:
            wasteValve1 = closeValve(wasteValve1);
            wasteValve3 = openValve(wasteValve3);
            Serial.print(F("[ERROR] Draining timeout for trough 1 after "));
            Serial.print(drainDuration);
            Serial.println(F(" ms (maximum drain time reached)."));
            break;
          case 2:
            wasteValve1 = closeValve(wasteValve1);
            wasteValve3 = closeValve(wasteValve3);
            Serial.print(F("[ERROR] Draining timeout for trough 2 after "));
            Serial.print(drainDuration);
            Serial.println(F(" ms (maximum drain time reached)."));
            break;
          case 3:
            wasteValve2 = closeValve(wasteValve2);
            wasteValve4 = openValve(wasteValve4);
            Serial.print(F("[ERROR] Draining timeout for trough 3 after "));
            Serial.print(drainDuration);
            Serial.println(F(" ms (maximum drain time reached)."));
            break;
          case 4:
            wasteValve2 = closeValve(wasteValve2);
            wasteValve4 = closeValve(wasteValve4);
            Serial.print(F("[ERROR] Draining timeout for trough 4 after "));
            Serial.print(drainDuration);
            Serial.println(F(" ms (maximum drain time reached)."));
            break;
        }
        if (!drainAsyncCompleted[i])
        {
          asyncCommandCompleted(&Serial);
          drainAsyncCompleted[i] = true;
        }
      }
    }
  }

  // ---------- 2. Drain Initiation Timeout Check ----------
  for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
  {
    if (!readBinarySensor(wasteLineSensors[sensorIdx]) && !liquidInitiallyDetected[sensorIdx])
    {
      for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
      {
        if (valveControls[i].isDraining &&
            (currentTime - valveControls[i].drainStartTime >= DRAIN_INITIATE_TIMEOUT))
        {
          valveControls[i].isDraining = false;
          valveControls[i].drainStartTime = 0;
          int trough = i + 1;
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
          Serial.print(F("[ERROR] Draining initiation timeout for trough "));
          Serial.print(trough);
          Serial.println(F(" (no liquid detected in drain line)."));
          if (!drainAsyncCompleted[i])
          {
            asyncCommandCompleted(&Serial);
            drainAsyncCompleted[i] = true;
          }
        }
      }
    }
  }

  // ---------- 3. Waste Bottle and Drain Completion Check ----------
  for (int sensorIdx = 0; sensorIdx < 2; sensorIdx++)
  {
    // If waste bottle is full, halt draining.
    if (readBinarySensor(wasteBottleSensors[sensorIdx]))
    {
      for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
      {
        if (valveControls[i].isDraining)
        {
          valveControls[i].isDraining = false;
          valveControls[i].drainStartTime = 0;
          int trough = i + 1;
          // For troughs 1 & 2 use wasteValve1, for 3 & 4 use wasteValve2.
          switch (trough)
          {
            case 1:
            case 2:
              wasteValve1 = closeValve(wasteValve1);
              break;
            case 3:
            case 4:
              wasteValve2 = closeValve(wasteValve2);
              break;
          }
          Serial.print(F("[ERROR] Draining halted for trough "));
          Serial.print(trough);
          Serial.println(F(" because the waste bottle is full."));
          if (!drainAsyncCompleted[i])
          {
            asyncCommandCompleted(&Serial);
            drainAsyncCompleted[i] = true;
          }
        }
      }
      continue; // Skip remaining checks for this sensor group.
    }

    // If liquid is detected, update timing variables.
    if (readBinarySensor(wasteLineSensors[sensorIdx]))
    {
      lastDrainCompleteTime[sensorIdx] = currentTime;
      liquidInitiallyDetected[sensorIdx] = true;
    }
    else if (liquidInitiallyDetected[sensorIdx] &&
             (currentTime - lastDrainCompleteTime[sensorIdx] >= DRAIN_COMPLETE_DELAY))
    {
      // For each trough in this sensor group, mark the drain as complete.
      for (int i = sensorIdx * 2; i < sensorIdx * 2 + 2; i++)
      {
        if (valveControls[i].isDraining)
        {
          valveControls[i].isDraining = false;
          valveControls[i].drainStartTime = 0;
          int trough = i + 1;
          switch (trough)
          {
            case 1:
              wasteValve1 = closeValve(wasteValve1);
              wasteValve3 = openValve(wasteValve3);
              Serial.println(F("[MESSAGE] Draining complete for trough 1"));
              break;
            case 2:
              wasteValve1 = closeValve(wasteValve1);
              wasteValve3 = closeValve(wasteValve3);
              Serial.println(F("[MESSAGE] Draining complete for trough 2"));
              break;
            case 3:
              wasteValve2 = closeValve(wasteValve2);
              wasteValve4 = openValve(wasteValve4);
              Serial.println(F("[MESSAGE] Draining complete for trough 3"));
              break;
            case 4:
              wasteValve2 = closeValve(wasteValve2);
              wasteValve4 = closeValve(wasteValve4);
              Serial.println(F("[MESSAGE] Draining complete for trough 4"));
              break;
          }
          if (!drainAsyncCompleted[i])
          {
            asyncCommandCompleted(&Serial);
            drainAsyncCompleted[i] = true;
          }
        }
      }
      lastDrainCompleteTime[sensorIdx] = 0;
      liquidInitiallyDetected[sensorIdx] = false;
      vacuumReleased[sensorIdx] = false;
    }

    // ---------- 4. Vacuum Release Check using switch-case on sensorIdx ----------
    switch (sensorIdx)
    {
      case 0:
        if (!vacuumReleased[sensorIdx] && !readBinarySensor(wasteVacuumSensors[sensorIdx]))
        {
          wasteValve3 = closeValve(wasteValve3);
          Serial.println(F("[MESSAGE] Vacuum released. Waste valve 3 closed."));
          vacuumReleased[sensorIdx] = true;
        }
        break;
      case 1:
        if (!vacuumReleased[sensorIdx] && !readBinarySensor(wasteVacuumSensors[sensorIdx]))
        {
          wasteValve4 = closeValve(wasteValve4);
          Serial.println(F("[MESSAGE] Vacuum released. Waste valve 4 closed."));
          vacuumReleased[sensorIdx] = true;
        }
        break;
    }
  }
}


  // ============================================================
  // Monitor Vacuum Release (Stop-Drain Completion)
  // ============================================================
  void monitorVacuumRelease(unsigned long currentTime)
  {
    for (int bottleIdx = 0; bottleIdx < 2; bottleIdx++)
    {
      if (!globalVacuumMonitoring[bottleIdx])
        continue;

      if (!readBinarySensor(wasteVacuumSensors[bottleIdx]))
      {
        if (bottleIdx == 0)
        {
          wasteValve3 = closeValve(wasteValve3);
          Serial.println(F("[MESSAGE] Vacuum released. Waste valve 3 closed."));
        } else if (bottleIdx == 1) {
        wasteValve4 = closeValve(wasteValve4);
        Serial.println(F("[MESSAGE] Vacuum released. Waste valve 4 closed."));
      }
      globalVacuumMonitoring[bottleIdx] = false;
      Serial.print(F("[MESSAGE] Vacuum monitoring disabled for bottle "));
      Serial.println(bottleIdx + 1);
      asyncCommandCompleted(&Serial);
    }
  }
}

// ============================================================
// Monitor Enclosure Liquid Sensor
// ============================================================
void monitorEnclosureLiquidSensor(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  static unsigned long lastErrorPrintTime = 0;
  static bool abortCalled = false; // to ensure abort is called once per leak event
  const unsigned long CHECK_INTERVAL = 25;
  const unsigned long ERROR_PRINT_INTERVAL = 15000;

  if (currentTime - previousCheckTime >= CHECK_INTERVAL) {
    previousCheckTime = currentTime;
    if (readBinarySensor(enclosureLiquidSensor)) {
      globalEnclosureLiquidError = true;
      if (!abortCalled) {
        abortAllAutomatedOperations(&Serial);;
        abortCalled = true;
      }
      if (currentTime - lastErrorPrintTime >= ERROR_PRINT_INTERVAL) {
        lastErrorPrintTime = currentTime;
      }
    } else {
      globalEnclosureLiquidError = false;
      abortCalled = false;  // Reset when leak is cleared.
    }
  }
}


// ============================================================
// Monitor Enclosure Temperature (Fan Control)
// ============================================================
void monitorEnclosureTemp() {
  if (!fanAutoMode) return;
  TempHumidity th = readTempHumidity();
  if (th.valid) {
    if (th.temperature > ENCLOSURE_TEMP_SETPOINT) {
      Serial.println(F("[WARNING] Enclosure temperature exceeded setpoint! Activating fan."));
      setFanState(fan, true);
    } else {
      setFanState(fan, false);
    }
  } else {
    Serial.println(F("[ERROR] Failed to read enclosure temperature!"));
  }
}

// ============================================================
// Monitor Flow Sensor Connections
// ============================================================
void monitorFlowSensorConnections() {
  FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
  for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
    if (!isFlowSensorConnected(*sensors[i])) {
      Serial.print(F("[ERROR] Flow sensor on channel "));
      Serial.print(sensors[i]->channel);
      Serial.println(F(" not connected."));
    }
  }
}
