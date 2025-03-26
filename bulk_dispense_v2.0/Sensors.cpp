#include "Sensors.h"
#include <Wire.h>
#include <math.h>  // For isnan()

/************************************************************
 * Sensors.cpp
 * 
 * This file implements functions declared in Sensors.h.
 * It handles initialization and data acquisition for the SHT31
 * Temperature/Humidity sensor and the flow sensors.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Internal Helper Functions
// ============================================================
/**
 * Checks if a flow sensor is connected.
 * Selects the appropriate multiplexer channel and attempts a transmission.
 * @param sensor The flow sensor to check.
 * @return true if the sensor is connected, false otherwise.
 */
bool isFlowSensorConnected(FlowSensor &sensor) {
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  Wire.beginTransmission(sensor.sensorAddr);
  if (Wire.endTransmission() == 0) {
    sensor.sensorConnected = 1;
    return true;
  } else {
    sensor.sensorConnected = 0;
    return false;
  }
}

// ============================================================
// Temperature & Humidity Sensor Functions
// ============================================================
bool tempHumSensorInit() {
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  return sht31.begin(TEMP_HUM_SENSOR_ADDR);
}

TempHumidity readTempHumidity() {
  TempHumidity data;
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  data.temperature = sht31.readTemperature();
  data.humidity = sht31.readHumidity();
  data.valid = !(isnan(data.temperature) || isnan(data.humidity));
  return data;
}

// ============================================================
// Flow Sensor Functions
// ============================================================
FlowSensor createFlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd) {
  FlowSensor sensor;
  sensor.multiplexerAddr = muxAddr;
  sensor.sensorAddr = addr;
  sensor.channel = chan;
  sensor.measurementCmd = cmd;
  sensor.sensorInitialized = false;
  sensor.sensorStopped = true;
  sensor.lastUpdateTime = 0;
  sensor.flowRate = 0.0;
  sensor.temperature = 0.0;
  sensor.highFlowFlag = 0;
  sensor.sensorConnected = 0;
  sensor.dispenseVolume = 0.0;
  sensor.totalVolume = 0.0;
  sensor.isValidReading = false;
  return sensor;
}

bool initializeFlowSensor(FlowSensor &sensor) {
  static int resetAttempt = 0;

  // Step 1: Select the multiplexer channel.
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);

  // Step 2: Check sensor connection.
  if (!isFlowSensorConnected(sensor)) {
    Serial.print(F("[ERROR] Flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" is not connected. Aborting initialization."));
    return false;
  }

  // Step 3: On retry, attempt a soft reset.
  if (resetAttempt > 0) {
    Serial.print(F("[DEBUG] Sending soft reset to sensor on channel "));
    Serial.println(sensor.channel);
    Wire.beginTransmission(sensor.sensorAddr);
    Wire.write(0x00);
    Wire.write(0x06);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("[WARNING] Soft reset command was not acknowledged."));
      return false;
    }
    delay(25);
    if (!isFlowSensorConnected(sensor)) {
      Serial.println(F("[ERROR] Sensor did not respond after soft reset."));
      return false;
    }
  }

  // Step 4: Start continuous measurement mode.
  Serial.print(F("[DEBUG] Sending start measurement command to sensor on channel "));
  Serial.println(sensor.channel);
  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(sensor.measurementCmd >> 8);
  Wire.write(sensor.measurementCmd & 0xFF);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[ERROR] Failed to start measurement mode."));
    if (resetAttempt == 0) {
      resetAttempt++;
      return initializeFlowSensor(sensor);
    }
    return false;
  }
  delay(200);

  // Step 5: Mark sensor as initialized and reset volume.
  sensor.sensorInitialized = true;
  sensor.sensorConnected = 1;
  sensor.lastUpdateTime = millis();
  sensor.dispenseVolume = 0.0;
  resetAttempt = 0;
  Serial.print(F("[MESSAGE] Flow sensor on channel "));
  Serial.print(sensor.channel);
  Serial.println(F(" successfully initialized."));
  return true;
}

bool readFlowSensorData(FlowSensor &sensor) {
  static int softResetAttempt = 0;

  if (!sensor.sensorInitialized || sensor.sensorStopped) {
    sensor.flowRate = -1;
    sensor.temperature = -1;
    sensor.highFlowFlag = -1;
    if (sensor.totalVolume == 0.0)
      sensor.dispenseVolume = 0.0;
    return false;
  }

  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  Wire.requestFrom(sensor.sensorAddr, (uint8_t)9);
  if (Wire.available() < 9) {
    Serial.print(F("[ERROR] Not enough bytes received from flow sensor on channel "));
    Serial.println(sensor.channel);
    if (softResetAttempt < 2) {
      Serial.println(F("[WARNING] Attempting soft reset to recover..."));
      Wire.beginTransmission(sensor.sensorAddr);
      Wire.write(0x00);
      Wire.write(0x06);
      if (Wire.endTransmission() == 0) {
        delay(25);
        softResetAttempt++;
        return false;
      }
    }
    Serial.println(F("[ERROR] Multiple failures. Sensor will remain in error state."));
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    sensor.sensorConnected = 0;
    softResetAttempt = 0;
    return false;
  }
  softResetAttempt = 0;

  uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Skip CRC
  uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Skip CRC
  uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Skip CRC

  sensor.flowRate = ((int16_t)flowRaw) / 32.0;
  sensor.temperature = ((int16_t)tempRaw) / 200.0;
  sensor.highFlowFlag = (auxRaw & 0x02) ? 1 : 0;
  sensor.sensorConnected = 1;
  if (sensor.flowRate < 0)
    sensor.flowRate = 0.0;

  unsigned long currentTime = millis();
  if (sensor.lastUpdateTime > 0) {
    float elapsedMinutes = (currentTime - sensor.lastUpdateTime) / 60000.0;
    float increment = sensor.flowRate * elapsedMinutes;
    sensor.dispenseVolume += increment;
    sensor.totalVolume += increment;
  }
  sensor.lastUpdateTime = currentTime;
  sensor.isValidReading = true;
  return true;
}

bool startFlowSensorMeasurement(FlowSensor &sensor) {
  Serial.print(F("[DEBUG] Attempting to start flow measurement for sensor on channel "));
  Serial.println(sensor.channel);
  if (!isFlowSensorConnected(sensor)) {
    Serial.print(F("[ERROR] Cannot start measurement for flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" because it is disconnected."));
    return false;
  }
  sensor.sensorStopped = false;
  for (int i = 0; i < 3; i++) {
    Serial.print(F("[DEBUG] Attempt "));
    Serial.print(i + 1);
    Serial.println(F(" to initialize sensor."));
    if (initializeFlowSensor(sensor)) {
      Serial.print(F("[MESSAGE] Flow sensor on channel "));
      Serial.print(sensor.channel);
      Serial.println(F(" started measurement mode."));
      return true;
    }
    delay(50);
  }
  Serial.print(F("[ERROR] Failed to start measurement for flow sensor on channel "));
  Serial.println(sensor.channel);
  return false;
}

bool stopFlowSensorMeasurement(FlowSensor &sensor) {
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(0x3F);  // Stop command MSB
  Wire.write(0xF9);  // Stop command LSB
  if (Wire.endTransmission() == 0) {
    Serial.print(F("[MESSAGE] Flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" stopped measurement mode."));
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    return true;
  } else {
    Serial.print(F("[ERROR] Failed to stop measurement for flow sensor on channel "));
    Serial.println(sensor.channel);
    return false;
  }
}

// ============================================================
// Pressure Sensor Functions
// ============================================================
float readPressureVoltage(const PressureSensor &sensor) {
  int analogValue = analogRead(sensor.analogPin);
  return (analogValue / 1023.0) * 10.0;
}

float readPressure(const PressureSensor &sensor) {
  float voltage = readPressureVoltage(sensor);
  return (voltage / 10.0) * sensor.maxPressure;
}

// ============================================================
// Flow Sensor Volume Reset Helper Functions
// ============================================================
void resetFlowSensorDispenseVolume(FlowSensor &sensor) {
  sensor.dispenseVolume = 0.0;
  sensor.lastUpdateTime = millis();
  sensor.sensorStopped = true;  // Ensure sensor is stopped
  Serial.print(F("[MESSAGE] Dispense volume reset for flow sensor on channel "));
  Serial.println(sensor.channel);
}

void resetFlowSensorTotalVolume(FlowSensor &sensor) {
  sensor.totalVolume = 0.0;
  Serial.print(F("[MESSAGE] Total volume reset for flow sensor on channel "));
  Serial.println(sensor.channel);
}



