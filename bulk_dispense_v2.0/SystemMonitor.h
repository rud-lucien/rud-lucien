#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include <Controllino.h>
#include "Hardware.h"
#include "Sensors.h"
#include "Utils.h"
#include <Wire.h>

/************************************************************
 * SystemMonitor.h
 *
 * Primary monitoring functions and their helpers for the Bulk
 * Dispense system, organized by subsystem:
 *
 * - Prime System: Monitors and controls reagent priming
 * - Fill System: Handles trough filling operations
 * - Waste System: Manages waste drainage and vacuum
 * - Safety Systems: Overflow, enclosure, temperature monitoring
 *
 * Version: 2.0
 ************************************************************/

// ==================== Prime System ====================
void monitorPrimeSensors(unsigned long currentTime);
void handlePrimingOverflow(int i);
bool handleLowFlowCondition(int i, unsigned long currentTime);
void handleBubbleDetected(int i, unsigned long currentTime);
bool handleNoBubbleDetected(int i, unsigned long currentTime);
void handlePrimingComplete(int i);
void resetPrimingStates(int i);
void resetPrimeMonitorState();

// ==================== Fill System ====================
void monitorFillSensors(unsigned long currentTime);
void fill_handleMaxTimeReached(int trough);
void fill_handleMaxVolumeReached(int trough);
void fill_handleFlowTimeout(int trough);
void fill_handleOverflowCheck(int trough);
void resetFillMonitorState();

// ==================== Waste System ====================
void monitorWasteSensors(unsigned long currentTime);
void monitorVacuumRelease(unsigned long currentTime);
void waste_handleMaxDrainTimeout(int trough, unsigned long drainDuration);
void waste_handleInitiationTimeout(int trough);
void waste_handleBottleFull(int trough);
void waste_handleDrainComplete(int trough);
void vacuum_handleVacuumRelease(int bottleIdx);
void resetWasteMonitorState();

// ==================== Safety Systems ====================
// Enclosure Protection
void monitorEnclosureLiquidSensor(unsigned long currentTime);
void enclosure_handleLeakDetected(unsigned long currentTime, unsigned long &lastErrorPrintTime);
void enclosure_handleNoLeak();
void resetEnclosureLeakMonitorState();

// Temperature Control
void monitorEnclosureTemp(unsigned long currentTime);
void temp_handleHighTemperature(float currentTemp, bool &fanOnDueToAutoMode);
void temp_handleNormalTemperature(bool &fanOnDueToAutoMode);
void temp_printWarning(float currentTemp, unsigned long currentTime, unsigned long &lastWarningTime);

// Flow Safety
void monitorFlowSensors(unsigned long currentTime);
void monitorOverflowSensors(unsigned long currentTime);
void monitorFlowSensorConnections(unsigned long currentTime);
void flow_handleDispenseOverflow(int i, FlowSensor *sensor);
void flow_handleVolumeComplete(int i, FlowSensor *sensor);
void flow_handleSafetyLimitExceeded(int i, FlowSensor *sensor, float maxVolume);

#endif // SYSTEMMONITOR_H
