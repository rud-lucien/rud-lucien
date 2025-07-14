#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <Arduino.h>

//=============================================================================
// CONSTANTS
//=============================================================================
#define SYSTEM_STATE_MSG_SIZE 150

//=============================================================================
// SYSTEM STATE COLLECTION
//=============================================================================

// Comprehensive system state collection function
void printSystemState();

// Individual component state functions for modularity
void printMotorSystemState();
void printSensorSystemState();
void printValveSystemState();
void printNetworkSystemState();
void printSafetySystemState();
void printEncoderSystemState();
void printLabwareSystemState();
void printSystemReadinessState();

// Utility functions for state determination
bool isSystemReadyForAutomation();
bool hasSystemErrors();
const char* getSystemErrorSummary();

#endif // SYSTEM_STATE_H
