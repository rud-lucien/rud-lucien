#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include <Controllino.h>
#include "SystemMonitor.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Utils.h"
#include <Wire.h>

/**
 * systemMonitor functions
 * -------------------------
 * These functions continuously check various sensors and hardware states.
 */
void monitorOverflowSensors(unsigned long currentTime);
void monitorFlowSensors(unsigned long currentTime);
void monitorReagentBubbleSensors(unsigned long currentTime);
void monitorWasteLineSensors(unsigned long currentTime);
void monitorWasteBottleSensors(unsigned long currentTime);
void monitorWasteVacuumSensors(unsigned long currentTime);
void monitorEnclosureLiquidSensor(unsigned long currentTime);
void monitorEnclosureTemp();
void monitorFlowSensorConnections();

// Helper functions used by the monitor functions.
void handleOverflowCondition(int triggeredTrough);
void handleTimeoutCondition(int troughNumber);

#endif // SYSTEMMONITOR_H

