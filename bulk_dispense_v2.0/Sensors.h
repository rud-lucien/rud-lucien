#ifndef SENSORS_H
#define SENSORS_H

#include "Hardware.h"
#include "NetworkConfig.h"  // For sendMessage function

/************************************************************
 * Sensors.h
 * 
 * This header declares functions for interfacing with sensors
 * used in the Bulk Dispense system, including temperature/humidity,
 * flow, and pressure sensors.
 *
 * Author: Rud Lucien
 * Date: 2025-04-08
 * Version: 2.0
 ************************************************************/

// ============================================================
// Temperature & Humidity Sensor Functions
// ============================================================
bool tempHumSensorInit();
TempHumidity readTempHumidity();

// ============================================================
// Flow Sensor Configuration Functions
// ============================================================
FlowSensor createFlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd);
bool isFlowSensorConnected(FlowSensor &sensor);

// ============================================================
// Flow Sensor Operation Functions
// ============================================================
bool initializeFlowSensor(FlowSensor &sensor);
bool readFlowSensorData(FlowSensor &sensor);
bool startFlowSensorMeasurement(FlowSensor &sensor);
bool stopFlowSensorMeasurement(FlowSensor &sensor);

// ============================================================
// Flow Sensor Volume Management Functions
// ============================================================
void resetFlowSensorDispenseVolume(FlowSensor &sensor);
void resetFlowSensorTotalVolume(FlowSensor &sensor);

// ============================================================
// Pressure Sensor Functions
// ============================================================
float readPressureVoltage(const PressureSensor &sensor);
float readPressure(const PressureSensor &sensor);

#endif // SENSORS_H
