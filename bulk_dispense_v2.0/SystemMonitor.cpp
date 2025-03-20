#include "SystemMonitor.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Utils.h"
#include <Wire.h>

// --------------------
// Overflow Handling
// --------------------
void handleOverflowCondition(int triggeredTrough) {
  if (valveControls[triggeredTrough - 1].isDispensing) {
    Serial.print(F("[WARNING] Overflow detected in Trough "));
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

// --------------------
// Timeout Handling
// --------------------
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

// --------------------
// monitorOverflowSensors()
// --------------------
void monitorOverflowSensors(unsigned long currentTime) {
  static unsigned long previousOverflowCheckTime = 0;

  // Check every 25ms (non-blocking)
  if (currentTime - previousOverflowCheckTime >= 25) {
    previousOverflowCheckTime = currentTime;
    // Iterate over all troughs (NUM_OVERFLOW_SENSORS)
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      if (readBinarySensor(overflowSensors[i])) {
        handleOverflowCondition(i + 1);  // Handle overflow for trough i+1
      }
    }
  }
}

// --------------------
// monitorFlowSensors()
// --------------------
void monitorFlowSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_THRESHOLD = 1.0;
  const float MAX_TROUGH_VOLUME = 205.0;  // Maximum safe volume per trough (backup safety)

  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;

    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      // Only check if dispensing is in progress.
      if (!valveControls[i].isDispensing) {
        continue;
      }

      FlowSensor* sensor = flowSensors[i];
      if (!sensor) {
        continue;
      }

      // Check for overflow
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
        continue;
      }

      // --- NEW: If manual control is active, reset timers and skip automated checks.
      if (valveControls[i].manualControl) {
        valveControls[i].lastFlowCheckTime = 0;
        valveControls[i].lastFlowChangeTime = 0;
        continue;
      }

      // Check for no flow timeout
      if (sensor->flowRate < MIN_FLOW_RATE_THRESHOLD) {
        if (valveControls[i].lastFlowCheckTime == 0) {
          valveControls[i].lastFlowCheckTime = currentTime;
        } else if (currentTime - valveControls[i].lastFlowCheckTime >= FLOW_TIMEOUT_MS) {
          handleTimeoutCondition(i + 1);
          continue;
        }
      } else {
        valveControls[i].lastFlowCheckTime = 0;
      }

      // Check if requested volume is reached
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
        continue;
      }

      // Secondary safety: Check if maximum trough volume is exceeded
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
      }
    }
  }
}

// --------------------
// monitorPrimeSensors()
// --------------------

// Define timing and threshold constants for priming
const unsigned long PRIME_TIMEOUT_MS = 6000;           // Maximum time allowed for priming (no bubble detected)
const unsigned long STABLE_DETECTION_PERIOD_MS = 500;  // Time bubble sensor must be continuously triggered
const unsigned long ADDITIONAL_PRIME_TIME_MS = 2000;   // Additional priming time after stable detection
const unsigned long PRIMING_FLOW_TIMEOUT_MS = 5000;    // Time allowed with insufficient flow
const float MIN_FLOW_RATE_PRIME = 5.0;                 // Minimum acceptable flow rate during priming

// Static arrays to track timing and low-flow conditions for each valve (assumed 4 valves)
static unsigned long primeStartTime[NUM_OVERFLOW_SENSORS] = { 0, 0, 0, 0 };
static unsigned long stableDetectionStartTime[NUM_OVERFLOW_SENSORS] = { 0, 0, 0, 0 };
static unsigned long additionalPrimeStartTime[NUM_OVERFLOW_SENSORS] = { 0, 0, 0, 0 };
static unsigned long lowFlowStartTime[NUM_OVERFLOW_SENSORS] = { 0, 0, 0, 0 };
static bool primingFailed[NUM_OVERFLOW_SENSORS] = { false, false, false, false };
static bool primingSuccess[NUM_OVERFLOW_SENSORS] = { false, false, false, false };

void monitorPrimeSensors(unsigned long currentTime) {
  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    // If priming is in progress for this valve:
    if (valveControls[i].isPriming) {
      // --- Check for overflow during priming ---
      if (readBinarySensor(overflowSensors[i])) {
        Serial.print(F("[ERROR] Priming aborted for valve "));
        Serial.print(i + 1);
        Serial.println(F(" due to overflow detected."));
        closeDispenseValves(i + 1);
        valveControls[i].isPriming = false;
        // Reset all prime-related timers and flags.
        primeStartTime[i] = 0;
        stableDetectionStartTime[i] = 0;
        additionalPrimeStartTime[i] = 0;
        lowFlowStartTime[i] = 0;
        primingFailed[i] = true;
        continue;
      }

      // --- Start priming timer on first iteration ---
      if (primeStartTime[i] == 0) {
        primeStartTime[i] = currentTime;
      }

      // --- Check for insufficient flow via flow sensor ---
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
            // Reset timers.
            primeStartTime[i] = 0;
            stableDetectionStartTime[i] = 0;
            additionalPrimeStartTime[i] = 0;
            lowFlowStartTime[i] = 0;
            primingFailed[i] = true;
            continue;
          }
        } else {
          // Reset low flow timer if flow is acceptable.
          lowFlowStartTime[i] = 0;
        }
      }

      // --- Check for bubble detection timeout ---
      if (!readBinarySensor(reagentBubbleSensors[i]) && (currentTime - primeStartTime[i] >= PRIME_TIMEOUT_MS) && !primingFailed[i] && !primingSuccess[i]) {
        Serial.print(F("[ERROR] Priming failed for valve "));
        Serial.print(i + 1);
        Serial.println(F(" due to no liquid detected."));
        closeDispenseValves(i + 1);
        valveControls[i].isPriming = false;
        primingFailed[i] = true;
        primeStartTime[i] = 0;
        additionalPrimeStartTime[i] = 0;
        stableDetectionStartTime[i] = 0;
        continue;
      }

      // --- Handle stable bubble detection ---
      if (readBinarySensor(reagentBubbleSensors[i]) && stableDetectionStartTime[i] == 0 && !primingFailed[i]) {
        // Start timing stable liquid detection.
        stableDetectionStartTime[i] = currentTime;
      } else if (!readBinarySensor(reagentBubbleSensors[i])) {
        // Reset if liquid is lost.
        stableDetectionStartTime[i] = 0;
      }

      // --- Once stable detection has occurred for the required period, start additional priming ---
      if (stableDetectionStartTime[i] != 0 && (currentTime - stableDetectionStartTime[i] >= STABLE_DETECTION_PERIOD_MS)) {
        if (additionalPrimeStartTime[i] == 0) {
          additionalPrimeStartTime[i] = currentTime;
          // Optionally, reset primeStartTime now that detection is confirmed.
          primeStartTime[i] = 0;
        }
      }

      // --- Complete priming after additional priming time has passed ---
      if (additionalPrimeStartTime[i] != 0 && (currentTime - additionalPrimeStartTime[i] >= ADDITIONAL_PRIME_TIME_MS)) {
        closeDispenseValves(i + 1);
        Serial.print(F("[MESSAGE] Priming complete for valve "));
        Serial.println(i + 1);
        valveControls[i].isPriming = false;
        primingSuccess[i] = true;
        // Reset all timing variables for the next priming cycle.
        primeStartTime[i] = 0;
        stableDetectionStartTime[i] = 0;
        additionalPrimeStartTime[i] = 0;
        lowFlowStartTime[i] = 0;
      }
    } else {
      // If not priming, reset priming flags and timers.
      primingFailed[i] = false;
      primingSuccess[i] = false;
      primeStartTime[i] = 0;
      stableDetectionStartTime[i] = 0;
      additionalPrimeStartTime[i] = 0;
      lowFlowStartTime[i] = 0;
    }
  }
}

void monitorFillSensors(unsigned long currentTime) {
  // Define constants (adjust as needed).
  const float MAX_FILL_VOLUME_ML = 200.0;           // Maximum fill volume in mL
  const unsigned long MAX_FILL_TIME_MS = 60000;     // Maximum fill time in ms
  const unsigned long FLOW_TIMEOUT_MS = 5000;       // Time allowed with insufficient flow during fill
  const float MIN_FLOW_RATE_FILL = 1;               // Minimum acceptable flow rate during fill
  const unsigned long SENSOR_CHECK_INTERVAL = 500;  // How often to check overflow sensor

  // Static arrays to track fill state for each trough.
  static unsigned long fillStartTime[NUM_OVERFLOW_SENSORS] = { 0 };
  static unsigned long lowFlowStartTime[NUM_OVERFLOW_SENSORS] = { 0 };
  static float initialVolume[NUM_OVERFLOW_SENSORS] = { 0 };
  static unsigned long lastSensorCheck[NUM_OVERFLOW_SENSORS] = { 0 };

  for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
    if (valveControls[i].fillMode) {
      FlowSensor* sensor = flowSensors[i];
      if (!sensor) {
        continue;
      }

      // Record fill start time and initial volume when filling begins.
      if (fillStartTime[i] == 0) {
        fillStartTime[i] = currentTime;
        initialVolume[i] = sensor->dispenseVolume;
        lowFlowStartTime[i] = 0;  // Reset low flow timer.
      }

      // Calculate added volume since fill started.
      float addedVolume = sensor->dispenseVolume - initialVolume[i];

      // Check for no-flow condition: if flow rate is below threshold.
      if (sensor->flowRate < MIN_FLOW_RATE_FILL) {
        if (lowFlowStartTime[i] == 0) {
          lowFlowStartTime[i] = currentTime;
        } else if ((currentTime - lowFlowStartTime[i]) >= FLOW_TIMEOUT_MS) {
          Serial.print(F("[ERROR] Fill timeout (insufficient flow) for trough "));
          Serial.println(i + 1);
          // Close valves and disable fill mode on error.
          closeDispenseValves(i + 1);
          valveControls[i].fillMode = false;
          fillStartTime[i] = 0;
          lowFlowStartTime[i] = 0;
          continue;
        }
      } else {
        // Flow rate is acceptable; reset low flow timer.
        lowFlowStartTime[i] = 0;
      }

      // Check if fill volume or fill time is exceeded.
      bool volumeExceeded = addedVolume >= MAX_FILL_VOLUME_ML;
      bool timeExceeded = (currentTime - fillStartTime[i]) >= MAX_FILL_TIME_MS;
      if (volumeExceeded || timeExceeded) {
        Serial.print(F("[MESSAGE] Fill complete for trough "));
        Serial.println(i + 1);
        // Close valves and disable fill mode.
        closeDispenseValves(i + 1);
        valveControls[i].fillMode = false;
        fillStartTime[i] = 0;
        lowFlowStartTime[i] = 0;
        continue;
      }

      // Periodically check overflow sensor.
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
      // If not in fill mode, ensure our static timers are reset.
      fillStartTime[i] = 0;
      lowFlowStartTime[i] = 0;
      lastSensorCheck[i] = 0;
    }
  }
}




// --------------------
// Other monitor functions (bubble, waste, etc.)
// --------------------

void monitorWasteLineSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_LINE_SENSORS; i++) {
      if (readBinarySensor(wasteLineSensors[i])) {
        Serial.print(F("[WARNING] Waste line sensor "));
        Serial.print(i + 1);
        Serial.println(F(" active."));
      }
    }
  }
}

void monitorWasteBottleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_BOTTLE_SENSORS; i++) {
      if (readBinarySensor(wasteBottleSensors[i])) {
        Serial.print(F("[WARNING] Waste bottle sensor "));
        Serial.print(i + 1);
        Serial.println(F(" active."));
      }
    }
  }
}

void monitorWasteVacuumSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_VACUUM_SENSORS; i++) {
      if (readBinarySensor(wasteVacuumSensors[i])) {
        Serial.print(F("[WARNING] Waste vacuum sensor "));
        Serial.print(i + 1);
        Serial.println(F(" active."));
      }
    }
  }
}

void monitorEnclosureLiquidSensor(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    if (readBinarySensor(enclosureLiquidSensor)) {
      Serial.println(F("[WARNING] Enclosure liquid detected."));
    }
  }
}

void monitorEnclosureTemp() {
  if (!fanAutoMode) return;  // Skip auto control if manual override is active

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
