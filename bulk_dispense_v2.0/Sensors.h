#ifndef SENSORS_H
#define SENSORS_H

#include "Hardware.h"  // Provides definitions for TempHumidity, FlowSensor, PressureSensor, etc.

/************************************************************
 * Sensors.h
 * 
 * This header declares functions for interfacing with the
 * SHT31 Temperature/Humidity sensor and the flow sensors used 
 * in the Bulk Dispense system.
 *
 * Functions include:
 *  - Initialization and reading of temperature/humidity.
 *  - Creation, initialization, data acquisition, and control 
 *    of flow sensors.
 *  - Reading and conversion of pressure sensor data.
 *  - Resetting of flow sensor volumes.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Temperature & Humidity Sensor Functions
// ============================================================
/**
 * Initializes the SHT31 sensor via the I²C multiplexer.
 * @return true if initialization is successful, false otherwise.
 */
bool tempHumSensorInit();

/**
 * Reads temperature and humidity from the SHT31 sensor.
 * @return A TempHumidity structure containing the values and a validity flag.
 */
TempHumidity readTempHumidity();

// ============================================================
// Flow Sensor Functions
// ============================================================
/**
 * Creates and returns a FlowSensor structure with the specified parameters.
 * @param muxAddr The I²C multiplexer address.
 * @param addr The sensor's I²C address.
 * @param chan The multiplexer channel.
 * @param cmd The measurement command for the sensor.
 * @return A FlowSensor structure.
 */
FlowSensor createFlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd);

/**
 * Initializes the specified flow sensor.
 * @param sensor The flow sensor structure to initialize.
 * @return true if initialization is successful, false otherwise.
 */
bool initializeFlowSensor(FlowSensor &sensor);

/**
 * Reads data from the specified flow sensor and updates its internal state.
 * @param sensor The flow sensor to read.
 * @return true if data was read successfully, false otherwise.
 */
bool readFlowSensorData(FlowSensor &sensor);

/**
 * Starts measurement mode for the specified flow sensor.
 * @param sensor The flow sensor to start.
 * @return true if measurement mode was started successfully.
 */
bool startFlowSensorMeasurement(FlowSensor &sensor);

/**
 * Stops measurement mode for the specified flow sensor.
 * @param sensor The flow sensor to stop.
 * @return true if the sensor was stopped successfully.
 */
bool stopFlowSensorMeasurement(FlowSensor &sensor);

/**
 * Checks if a flow sensor is connected.
 * @param sensor The flow sensor to check.
 * @return true if the sensor is connected, false otherwise.
 */
bool isFlowSensorConnected(FlowSensor &sensor);

/**
 * Resets the dispense volume of the flow sensor.
 * Sets dispenseVolume to 0, updates lastUpdateTime, and marks the sensor as stopped.
 * @param sensor The flow sensor to reset.
 */
void resetFlowSensorDispenseVolume(FlowSensor &sensor);

/**
 * Resets the total volume of the flow sensor.
 * Sets totalVolume to 0.
 * @param sensor The flow sensor to reset.
 */
void resetFlowSensorTotalVolume(FlowSensor &sensor);

// ============================================================
// Pressure Sensor Functions
// ============================================================
/**
 * Reads the raw voltage from the specified pressure sensor.
 * @param sensor The pressure sensor.
 * @return The voltage value.
 */
float readPressureVoltage(const PressureSensor &sensor);

/**
 * Converts the raw voltage from the pressure sensor into a pressure value.
 * @param sensor The pressure sensor.
 * @return The pressure in psi.
 */
float readPressure(const PressureSensor &sensor);

bool diagnoseSensor(FlowSensor &sensor);

#endif // SENSORS_H

