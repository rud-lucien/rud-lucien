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
// Other monitor functions (bubble, waste, etc.)
// --------------------
void monitorReagentBubbleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_REAGENT_BUBBLE_SENSORS; i++) {
      if (readBinarySensor(reagentBubbleSensors[i])) {
        Serial.print(F("[WARNING] Bubble detected on reagent sensor "));
        Serial.println(i + 1);
      }
    }
  }
}

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

