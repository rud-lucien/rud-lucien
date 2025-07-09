#ifndef LOGGING_H
#define LOGGING_H

#include "ValveController.h"
#include "MotorController.h"
#include "EncoderController.h"
#include "OutputManager.h"

//=============================================================================
// TYPE DEFINITIONS
//=============================================================================

// Structure to manage periodic logging behavior
struct LoggingManagement
{
    unsigned long previousLogTime; // Time of last log
    unsigned long logInterval;     // Interval (ms) between logs, 0 = disabled
};

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Global logging management instance
extern LoggingManagement logging;

// Default interval between automatic logs in milliseconds (0 = disabled)
extern const unsigned long DEFAULT_LOG_INTERVAL;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// Log the current state of the entire system
void logSystemState();

// Print system state with color enhancement
void printColoredSystemState(const char* msg);

#endif // LOGGING_H
