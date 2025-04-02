#ifndef LOGGING_H
#define LOGGING_H

#include <Controllino.h>
#include "Sensors.h"

/************************************************************
 * Logging.h
 * 
 * This header declares functions and structures for logging
 * system messages and state for the Bulk Dispense system.
 *
 * - Structure for managing logging timing.
 * - Global logging instance.
 * - Function prototypes for logging messages and system state.
 *
 * Author: Your Name
 * Date: YYYY-MM-DD
 * Version: 2.0
 ************************************************************/

// ============================================================
// Logging Management Structure
// ============================================================
struct LoggingManagement {
  unsigned long previousLogTime;  // Timestamp of last log
  unsigned long logInterval;      // Interval (ms) between logs
};

// ============================================================
// Global Logging Instance
// ============================================================
extern LoggingManagement logging;

// ============================================================
// Logging Function Prototypes
// ============================================================
/**
 * logData()
 * ----------
 * Logs a simple message with a module identifier.
 *
 * @param module  The module name.
 * @param message The message to log.
 */
void logData(const char* module, const char* message);


const char* getFlowDiagString(const FlowSensor &sensor, bool isDispensing);

/**
 * logSystemState()
 * ----------------
 * Gathers current system state (fan, valves, sensors, etc.) and prints a formatted log message.
 */
void logSystemState();



#endif // LOGGING_H


