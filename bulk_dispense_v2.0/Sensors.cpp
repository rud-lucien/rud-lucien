#include "Sensors.h"
#include <Wire.h>
#include <math.h> // For isnan()
#include "Utils.h"

/************************************************************
 * Sensors.cpp
 *
 * This file implements functions declared in Sensors.h.
 * It handles initialization and data acquisition for the SHT31
 * Temperature/Humidity sensor and the flow sensors.
 *
 * Author: Rud Lucien
 * Date: 2025-04-08
 * Version: 2.0
 ************************************************************/

// ============================================================
// Internal Helper Functions
// ============================================================

bool isFlowSensorConnected(FlowSensor &sensor)
{
  // Try multiple times with delays
  for (int attempt = 0; attempt < 3; attempt++)
  {
    selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
    delay(20);

    Wire.beginTransmission(sensor.sensorAddr);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      sensor.sensorConnected = 1;
      return true;
    }
    delay(20);
  }

  sensor.sensorConnected = 0;
  return false;
}

// ============================================================
// Temperature & Humidity Sensor Functions
// ============================================================
bool tempHumSensorInit()
{
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  return sht31.begin(TEMP_HUM_SENSOR_ADDR);
}

TempHumidity readTempHumidity()
{
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
FlowSensor createFlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd)
{
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
  sensor.fluidType = WATER; // Add default fluid type initialization
  return sensor;
}

void initializeAllFlowSensors()
{
  sendMessage(F("[STATUS] Initializing all flow sensors..."), &Serial, currentClient);

  for (int i = 0; i < NUM_FLOW_SENSORS; i++)
  {
    sendMessage(F("[STATUS] Initializing flow sensor "), &Serial, currentClient, false);
    sendMessage(String(i + 1).c_str(), &Serial, currentClient);

    // Try up to 3 times to initialize each sensor
    bool success = false;
    for (int attempt = 0; attempt < 3; attempt++)
    {
      // Add * to dereference the pointer
      if (initializeFlowSensor(*flowSensors[i]))
      {
        success = true;
        break;
      }
      delay(100);
    }

    if (success)
    {
      // Stop measurement until needed (but stay initialized)
      // Add * to dereference the pointer
      stopFlowSensorMeasurement(*flowSensors[i]);
    }
    else
    {
      sendMessage(F("[WARNING] Could not initialize flow sensor "), &Serial, currentClient, false);
      sendMessage(String(i + 1).c_str(), &Serial, currentClient);
    }
  }

  sendMessage(F("[STATUS] Flow sensor initialization complete"), &Serial, currentClient);
}

bool initializeFlowSensor(FlowSensor &sensor)
{
  // Use local variable instead of static
  int attempt = 0;

  // Select multiplexer channel
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  delay(50);

  // First check if sensor is connected
  if (!isFlowSensorConnected(sensor))
  {
    sendMessage(F("[ERROR] Flow sensor on channel "), &Serial, currentClient, false);
    sendMessage(String(sensor.channel).c_str(), &Serial, currentClient, false);
    sendMessage(F(" is not connected."), &Serial, currentClient);
    return false;
  }

  // Try to reset sensor first
  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(100);

  // Start measurement mode
  sendMessage(F("[DEBUG] Sending start measurement command to sensor on channel "), &Serial, currentClient, false);
  sendMessage(String(sensor.channel).c_str(), &Serial, currentClient);

  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(sensor.measurementCmd >> 8);
  Wire.write(sensor.measurementCmd & 0xFF);
  if (Wire.endTransmission() != 0)
  {
    sendMessage(F("[ERROR] Failed to start measurement mode."), &Serial, currentClient);
    resetI2CBus(); // Reset the I2C bus if communication fails
    return false;
  }

  // Mark as initialized
  sensor.sensorInitialized = true;
  sensor.sensorStopped = false;
  sensor.dispenseVolume = 0.0;
  sensor.lastUpdateTime = millis();

  return true;
}

bool readFlowSensorData(FlowSensor &sensor)
{
  static int softResetAttempt = 0;

  if (!sensor.sensorInitialized || sensor.sensorStopped)
  {
    sensor.flowRate = -1;
    sensor.temperature = -1;
    sensor.highFlowFlag = -1;
    if (sensor.totalVolume == 0.0)
      sensor.dispenseVolume = 0.0;
    return false;
  }

  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  Wire.requestFrom(sensor.sensorAddr, (uint8_t)9);
  if (Wire.available() < 9)
  {
    sendMessage(F("[ERROR] Not enough bytes received from flow sensor on channel "), &Serial, currentClient, false);
    sendMessage(String(sensor.channel).c_str(), &Serial, currentClient);
    if (softResetAttempt < 2)
    {
      sendMessage(F("[WARNING] Attempting soft reset to recover..."), &Serial, currentClient);
      Wire.beginTransmission(sensor.sensorAddr);
      Wire.write(0x00);
      Wire.write(0x06);
      if (Wire.endTransmission() == 0)
      {
        delay(25);
        softResetAttempt++;
        return false;
      }
    }
    sendMessage(F("[ERROR] Multiple failures. Sensor will remain in error state."), &Serial, currentClient);
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    sensor.sensorConnected = 0;
    softResetAttempt = 0;
    return false;
  }
  softResetAttempt = 0;

  uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); // Skip CRC
  uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); // Skip CRC
  uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); // Skip CRC

  sensor.flowRate = ((int16_t)flowRaw) / 32.0;
  sensor.temperature = ((int16_t)tempRaw) / 200.0;
  sensor.highFlowFlag = (auxRaw & 0x02) ? 1 : 0;
  sensor.sensorConnected = 1;
  if (sensor.flowRate < 0)
    sensor.flowRate = 0.0;

  unsigned long currentTime = millis();
  if (sensor.lastUpdateTime > 0)
  {
    float elapsedMinutes = (currentTime - sensor.lastUpdateTime) / 60000.0;
    float increment = sensor.flowRate * elapsedMinutes;

    // Apply calibration to the increment if enabled
    if (sensor.useCorrection)
    {
      // Apply the linear calibration formula: y = mx + b
      // Where m is slope, b is offset, x is measured increment
      increment = sensor.slopeCorrection * increment + sensor.offsetCorrection * elapsedMinutes;
    }

    sensor.dispenseVolume += increment;
    sensor.totalVolume += increment;
  }
  sensor.lastUpdateTime = currentTime;
  sensor.isValidReading = true;
  return true;
}

bool startFlowSensorMeasurement(FlowSensor &sensor)
{
  sendMessage(F("[DEBUG] Starting flow measurement for sensor on channel "), &Serial, currentClient, false);
  sendMessage(String(sensor.channel).c_str(), &Serial, currentClient);

  // Check connection first
  if (!isFlowSensorConnected(sensor))
  {
    sendMessage(F("[ERROR] Cannot start measurement - sensor not connected"), &Serial, currentClient);
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    return false;
  }

  // ALWAYS mark as not stopped and do full initialization
  sensor.sensorStopped = false;

  // Simple approach: Always do full initialization
  for (int attempt = 0; attempt < 3; attempt++)
  {
    sendMessage(F("[DEBUG] Attempt "), &Serial, currentClient, false);
    sendMessage(String(attempt + 1).c_str(), &Serial, currentClient, false);
    sendMessage(F(" to initialize sensor."), &Serial, currentClient);

    if (initializeFlowSensor(sensor))
    {
      sendMessage(F("[MESSAGE] Flow sensor started measurement mode."), &Serial, currentClient);
      return true;
    }

    delay(100);
  }

  // If all attempts fail, reset I2C bus
  resetI2CBus();
  return false;
}

bool stopFlowSensorMeasurement(FlowSensor &sensor)
{
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  delay(50);

  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(0x3F);
  Wire.write(0xF9);
  int result = Wire.endTransmission();

  if (result == 0)
  {
    sendMessage(F("[MESSAGE] Flow sensor on channel "), &Serial, currentClient, false);
    sendMessage(String(sensor.channel).c_str(), &Serial, currentClient, false);
    sendMessage(F(" stopped measurement mode."), &Serial, currentClient);

    // ALWAYS mark as uninitialized when stopping - just like your prototype
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    return true;
  }

  // If failed, reset I2C bus
  resetI2CBus();

  // Still mark sensor as stopped even if command failed
  sensor.sensorInitialized = false;
  sensor.sensorStopped = true;

  return false;
}

bool setFlowSensorFluidType(FlowSensor &sensor, FluidType fluidType)
{
  // First stop measurement if running
  if (sensor.sensorInitialized && !sensor.sensorStopped)
  {
    if (!stopFlowSensorMeasurement(sensor))
    {
      sendMessage(F("[ERROR] Cannot set fluid type - failed to stop sensor."), &Serial, currentClient);
      return false;
    }
  }

  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);

  uint16_t new_measurement_cmd;
  const char *fluidName;

  // Select appropriate command and name based on fluidType
  switch (fluidType)
  {
  case WATER:
    new_measurement_cmd = FLOW_SENSOR_CMD_WATER;
    fluidName = "Water";
    break;
  case IPA:
    new_measurement_cmd = FLOW_SENSOR_CMD_IPA;
    fluidName = "IPA";
    break;
  default:
    sendMessage(F("[ERROR] Unsupported fluid type specified."), &Serial, currentClient);
    return false;
  }

  // Update the sensor's measurement command and fluid type
  sensor.measurementCmd = new_measurement_cmd;
  sensor.fluidType = fluidType;

  sendMessage(F("[MESSAGE] Flow sensor on channel "), &Serial, currentClient, false);
  sendMessage(String(sensor.channel).c_str(), &Serial, currentClient, false);
  sendMessage(F(" configured for "), &Serial, currentClient, false);
  sendMessage(fluidName, &Serial, currentClient);

  return true;
}

const char *getFluidTypeString(FluidType type)
{
  switch (type)
  {
  case WATER:
    return "Water";
  case IPA:
    return "IPA";
  default:
    return "Unknown";
  }
}

// ============================================================
// Pressure Sensor Functions
// ============================================================
float readPressureVoltage(const PressureSensor &sensor)
{
  int analogValue = analogRead(sensor.analogPin);
  return (analogValue / 1023.0) * 10.0;
}

float readPressure(const PressureSensor &sensor)
{
  float voltage = readPressureVoltage(sensor);
  return (voltage / 10.0) * sensor.maxPressure;
}

// ============================================================
// Flow Sensor Volume Reset Helper Functions
// ============================================================
void resetFlowSensorDispenseVolume(FlowSensor &sensor)
{
  sensor.dispenseVolume = 0.0;
  sensor.lastUpdateTime = millis();
  sensor.sensorStopped = true; // Ensure sensor is stopped
  sendMessage(F("[MESSAGE] Dispense volume reset for flow sensor on channel "), &Serial, currentClient, false);
  sendMessage(String(sensor.channel).c_str(), &Serial, currentClient);
}

void resetFlowSensorTotalVolume(FlowSensor &sensor)
{
  sensor.totalVolume = 0.0;
  sendMessage(F("[MESSAGE] Total volume reset for flow sensor on channel "), &Serial, currentClient, false);
  sendMessage(String(sensor.channel).c_str(), &Serial, currentClient);
}
