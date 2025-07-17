#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <Arduino.h>
#include "Utils.h"

//=============================================================================
// CONSTANTS
//=============================================================================
// System reset constants
#define MAX_FAULT_CLEAR_ATTEMPTS 3
#define FAULT_CLEAR_RETRY_DELAY_MS 500
#define DEFAULT_ENCODER_MULTIPLIER 10
#define MILLISECONDS_PER_SECOND 1000

// Rail ID constants
#define FIRST_RAIL_ID 1
#define LAST_RAIL_ID 2

// String termination
#define STRING_TERMINATOR '\0'

//=============================================================================
// SYSTEM STATE COLLECTION
//=============================================================================

// Comprehensive system state collection function
void printSystemState();

// Component state printing functions
void printSafetySystemState();
void printSystemReadinessState();

// Utility functions for state determination
bool isSystemReadyForAutomation();
bool hasSystemErrors();
const char *getSystemErrorSummary();

// State management functions
void setEmergencyStop(bool activated);
bool getEmergencyStopStatus();
void setPositionLimit(bool atLimit);
bool getPositionLimitStatus();

// System reset function
void resetSystemState();

// System homing function
bool homeSystemRails();

// System motor fault clearing function
bool clearSystemMotorFaults();

// System motor initialization function
bool initSystemMotors();

// Timeout reset functions (for system reset)
void resetSystemTimeouts();

#endif // SYSTEM_STATE_H
