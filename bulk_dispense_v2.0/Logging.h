#ifndef LOGGING_H
#define LOGGING_H

#include <Controllino.h>

// ------------------------------------------------------------------
// Structure for managing logging timing
// ------------------------------------------------------------------
struct LoggingManagement {
  unsigned long previousLogTime;
  unsigned long logInterval;
};

// Global logging instance (definition is in Logging.cpp)
extern LoggingManagement logging;

// ------------------------------------------------------------------
// Logging Function Prototypes
// ------------------------------------------------------------------
/**
 * logData()
 * ----------
 * Logs a simple message with a module identifier.
 *
 * @param module The module name.
 * @param message The message to log.
 */
void logData(const char* module, const char* message);

/**
 * logSystemState()
 * ----------------
 * Gathers current system state (fan, valves, sensors, etc.) and prints a formatted log message.
 */
void logSystemState();

#endif // LOGGING_H


