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
 * This header declares the monitoring functions for the Bulk 
 * Dispense system. These functions continuously check sensor 
 * and hardware states, including:
 *  - Overflow, flow, prime, fill, and waste conditions.
 *  - Enclosure liquid detection.
 *  - Temperature-based fan control.
 *  - Flow sensor connection status.
 *
 * It also declares helper functions used by these monitors.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ==================== Monitoring Function Prototypes ====================
void monitorOverflowSensors(unsigned long currentTime);
void monitorFlowSensors(unsigned long currentTime);
void monitorReagentBubbleSensors(unsigned long currentTime);
void monitorWasteLineSensors(unsigned long currentTime);
void monitorWasteBottleSensors(unsigned long currentTime);
void monitorWasteVacuumSensors(unsigned long currentTime);
void monitorEnclosureLiquidSensor(unsigned long currentTime);
void monitorEnclosureTemp();
void monitorFlowSensorConnections();
void monitorPrimeSensors(unsigned long currentTime);
void monitorFillSensors(unsigned long currentTime);
void monitorWasteSensors(unsigned long currentTime);
void monitorVacuumRelease(unsigned long currentTime);

// ==================== Helper Function Prototypes ====================
void handleOverflowCondition(int triggeredTrough);
void handleTimeoutCondition(int troughNumber);

#endif // SYSTEMMONITOR_H


