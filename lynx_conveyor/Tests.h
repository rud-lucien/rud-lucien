#ifndef TESTS_H
#define TESTS_H

#include "Arduino.h"
#include "ClearCore.h"
#include "MotorController.h"
#include "EthernetController.h"
#include "CommandController.h"
#include "Utils.h"
#include "ValveController.h"
#include "OutputManager.h"

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// Motor System Tests
// Evaluate motor positioning and homing performance
//-----------------------------------------------------------------------------
extern bool testInProgress;
extern volatile bool testAbortRequested;

bool checkSerialForAbortCommand();   // Check for 'abort' command in serial buffer
bool checkEthernetForAbortCommand(); // Check for 'abort' command in Ethernet buffer
void requestTestAbort(const char *source);
bool handleTestAbort();
bool testHomingRepeatability(); // Test homing repeatability over multiple cycles
bool testPositionCycling();     // Test cycling through all positions
bool testTrayHandling();        // Test tray handling operations

#endif // TESTS_H