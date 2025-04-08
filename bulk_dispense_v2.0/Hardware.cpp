#include "Hardware.h"
#include <Wire.h>
#include "Utils.h"

/************************************************************
 * Hardware.cpp
 * 
 * This file contains hardware-specific definitions including:
 * - Pin assignment arrays for valves and sensors.
 * - Global object definitions for hardware components.
 * - Hardware functions for initializing and controlling devices.
 *
 * Author: Rud Lucien
 * Date: 2025-04-08
 * Version: 2.0
 ************************************************************/

// ============================================================
// Constant Arrays for Pin Assignments
// ============================================================
const uint8_t REAGENT_VALVES[NUM_REAGENT_VALVES] = {
  CONTROLLINO_R0, CONTROLLINO_R1, CONTROLLINO_R2, CONTROLLINO_R3
};

const uint8_t MEDIA_VALVES[NUM_MEDIA_VALVES] = {
  CONTROLLINO_DO0, CONTROLLINO_DO1, CONTROLLINO_DO2, CONTROLLINO_DO3
};

const uint8_t WASTE_VALVES[NUM_WASTE_VALVES] = {
  CONTROLLINO_R4, CONTROLLINO_R5, CONTROLLINO_R8, CONTROLLINO_R9
};

const uint8_t OVERFLOW_SENSORS[NUM_OVERFLOW_SENSORS] = {
  CONTROLLINO_DI0, CONTROLLINO_DI1, CONTROLLINO_DI2, CONTROLLINO_DI3
};

const uint8_t BUBBLE_SENSORS[NUM_REAGENT_BUBBLE_SENSORS] = {
  CONTROLLINO_AI0, CONTROLLINO_AI1, CONTROLLINO_AI2, CONTROLLINO_AI3
};

const uint8_t WASTE_LINE_SENSORS[NUM_WASTE_LINE_SENSORS] = {
  CONTROLLINO_AI4, CONTROLLINO_AI5
};

const uint8_t WASTE_BOTTLE_SENSORS[NUM_WASTE_BOTTLE_SENSORS] = {
  CONTROLLINO_AI6, CONTROLLINO_AI7
};

const uint8_t WASTE_VACUUM_SENSORS[NUM_WASTE_VACUUM_SENSORS] = {
  CONTROLLINO_AI8, CONTROLLINO_AI9
};

// ============================================================
// Global Hardware Object Definitions
// ============================================================

// Fan
bool fanAutoMode = true;
const FanControl fan = { FAN_CONTROL_PIN };

// Proportional Valve
ProportionalValve proportionalValve = { PROPORTIONAL_VALVE_CONTROL_PIN, PROPORTIONAL_VALVE_FEEDBACK_PIN, 0.0 };

// Pressure Sensor (example: 0 to 87 psi)
PressureSensor pressureSensor = { PRESSURE_SENSOR_PIN, 0, 87 };

// SHT31 sensor instance
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Flow Sensors (channels 0-3)
FlowSensor flow1 = { MULTIPLEXER_ADDR, 0x08, 0, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };
FlowSensor flow2 = { MULTIPLEXER_ADDR, 0x08, 1, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };
FlowSensor flow3 = { MULTIPLEXER_ADDR, 0x08, 2, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };
FlowSensor flow4 = { MULTIPLEXER_ADDR, 0x08, 3, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };

FlowSensor *flowSensors[NUM_FLOW_SENSORS] = { &flow1, &flow2, &flow3, &flow4 };

// Valve Control Array (one per overflow sensor/trough)
ValveControl valveControls[NUM_OVERFLOW_SENSORS] = {};

// On/Off Valves for reagents, media, and waste
OnOffValve reagentValve1 = { REAGENT_VALVES[0], false };
OnOffValve reagentValve2 = { REAGENT_VALVES[1], false };
OnOffValve reagentValve3 = { REAGENT_VALVES[2], false };
OnOffValve reagentValve4 = { REAGENT_VALVES[3], false };

OnOffValve mediaValve1 = { MEDIA_VALVES[0], false };
OnOffValve mediaValve2 = { MEDIA_VALVES[1], false };
OnOffValve mediaValve3 = { MEDIA_VALVES[2], false };
OnOffValve mediaValve4 = { MEDIA_VALVES[3], false };

OnOffValve wasteValve1 = { WASTE_VALVES[0], false };
OnOffValve wasteValve2 = { WASTE_VALVES[1], false };
OnOffValve wasteValve3 = { WASTE_VALVES[2], false };
OnOffValve wasteValve4 = { WASTE_VALVES[3], false };

// Binary Sensors
BinarySensor overflowSensors[NUM_OVERFLOW_SENSORS] = {
  { OVERFLOW_SENSORS[0], true },
  { OVERFLOW_SENSORS[1], true },
  { OVERFLOW_SENSORS[2], true },
  { OVERFLOW_SENSORS[3], true }
};

BinarySensor reagentBubbleSensors[NUM_REAGENT_BUBBLE_SENSORS] = {
  { BUBBLE_SENSORS[0], true },
  { BUBBLE_SENSORS[1], true },
  { BUBBLE_SENSORS[2], true },
  { BUBBLE_SENSORS[3], true }
};

BinarySensor wasteLineSensors[NUM_WASTE_LINE_SENSORS] = {
  { WASTE_LINE_SENSORS[0], true },
  { WASTE_LINE_SENSORS[1], true }
};

BinarySensor wasteBottleSensors[NUM_WASTE_BOTTLE_SENSORS] = {
  { WASTE_BOTTLE_SENSORS[0], true },
  { WASTE_BOTTLE_SENSORS[1], true }
};

BinarySensor wasteVacuumSensors[NUM_WASTE_VACUUM_SENSORS] = {
  { WASTE_VACUUM_SENSORS[0], true },
  { WASTE_VACUUM_SENSORS[1], true }
};

BinarySensor enclosureLiquidSensor = { ENCLOSURE_LIQUID_SENSOR_PIN, false };

// Global vacuum monitoring flags for waste bottles.
bool globalVacuumMonitoring[NUM_WASTE_VACUUM_SENSORS] = { false, false };

// Global flag for enclosure liquid error state.
bool globalEnclosureLiquidError = false;

// Calibration variable for proportional valve.
float proportionalValveMaxFeedback = 0.0;

// Async command flags.
bool dispenseAsyncCompleted[NUM_OVERFLOW_SENSORS] = { false, false, false, false };
bool drainAsyncCompleted[NUM_OVERFLOW_SENSORS] = { false, false, false, false };
bool primeAsyncCompleted[NUM_OVERFLOW_SENSORS]    = { false, false, false, false };

// ============================================================
// Hardware Functions
// ============================================================

void fanSetup(const FanControl &fc) {
  pinMode(fc.relayPin, OUTPUT);
  digitalWrite(fc.relayPin, LOW);
  sendMessage(F("[MESSAGE] Fan initialized and set to OFF"), &Serial, currentClient);
}

void setFanState(const FanControl &config, bool state) {
  // Read current pin state to decide whether to update.
  bool currentState = digitalRead(config.relayPin) == HIGH;
  if (currentState != state) {
    digitalWrite(config.relayPin, state ? HIGH : LOW);
    sendMessage(F("[MESSAGE] Fan state set to "), &Serial, currentClient, false);
    sendMessage(state ? F("ON") : F("OFF"), &Serial, currentClient);
  }
}

void printFanState(bool state) {
  sendMessage(F("[MESSAGE] Fan is "), &Serial, currentClient, false);
  sendMessage(state ? F("ON") : F("OFF"), &Serial, currentClient);
}

OnOffValve openValve(OnOffValve valve) {
  digitalWrite(valve.controlPin, HIGH);
  valve.isOpen = true;
  return valve;
}

OnOffValve closeValve(OnOffValve valve) {
  digitalWrite(valve.controlPin, LOW);
  valve.isOpen = false;
  return valve;
}

void proportionalValveSetup(const ProportionalValve &valve) {
  pinMode(valve.controlPin, OUTPUT);
  pinMode(valve.feedbackPin, INPUT);
}

ProportionalValve setValvePosition(ProportionalValve valve, float percentage) {
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  valve.controlVoltage = (percentage / 100.0) * 10.0;
  int pwmValue = (int)((valve.controlVoltage / 10.0) * 255);
  analogWrite(valve.controlPin, pwmValue);
  return valve;
}

float getValveFeedback(const ProportionalValve &valve) {
  int analogValue = analogRead(valve.feedbackPin);
  // Map analog reading (0-1023) to 0-10 volts (scaled to 10000 for mV then divided)
  float voltage = map(analogValue, 0, 1023, 0, 10000) / 1000.0;
  return voltage;
}

void calibrateProportionalValve() {
  sendMessage(F("[MESSAGE] Starting proportional valve calibration..."), &Serial, currentClient);
  // Set valve to fully open (100%).
  proportionalValve = setValvePosition(proportionalValve, 100.0);
  delay(1000);  // Wait for stabilization.
  proportionalValveMaxFeedback = getValveFeedback(proportionalValve);
  
  sendMessage(F("[MESSAGE] Calibrated max feedback voltage: "), &Serial, currentClient, false);
  char voltageStr[10];
  dtostrf(proportionalValveMaxFeedback, 4, 2, voltageStr);
  sendMessage(voltageStr, &Serial, currentClient);
}

void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel) {
  Wire.beginTransmission(multiplexerAddr);
  Wire.write(1 << channel);  // Select channel by shifting 1 into the desired bit position.
  Wire.endTransmission();
}

bool readBinarySensor(const BinarySensor &sensor) {
  int reading = digitalRead(sensor.inputPin);
  return sensor.activeHigh ? (reading == HIGH) : (reading == LOW);
}
