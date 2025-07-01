#ifndef POSITION_CONFIG_H
#define POSITION_CONFIG_H

#include "Arduino.h"
#include "MotorController.h"
#include "OutputManager.h"

//=============================================================================
// POSITION CONFIGURATION SYSTEM
//=============================================================================

// Runtime position variables (override #defines when set)
extern double runtimePosition1Mm;
extern double runtimePosition2Mm; 
extern double runtimePosition3Mm;
extern bool useRuntimePositions;

// SD Card configuration
#define CONFIG_FILE_NAME "POS.TXT"

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// System initialization
bool initPositionConfig();

// Position getter functions (replaces direct #define usage)
double getPosition1Mm();
double getPosition2Mm();
double getPosition3Mm();

// Teaching functions
bool teachPosition1();
bool teachPosition2(); 
bool teachPosition3();
bool teachSavePositions();
bool teachResetPositions();
void teachShowStatus();

// Internal SD Card operations
bool savePositionsToSD();
bool loadPositionsFromSD();
bool isSDCardAvailable();

#endif // POSITION_CONFIG_H