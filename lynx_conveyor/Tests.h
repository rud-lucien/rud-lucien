#ifndef TESTS_H
#define TESTS_H

#include "Arduino.h"
#include "ClearCore.h"
#include "MotorController.h"

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// Motor System Tests
// Evaluate motor positioning and homing performance
//-----------------------------------------------------------------------------

bool testHomingRepeatability(); // Test homing repeatability over multiple cycles
bool testPositionCycling();     // Test cycling through all positions
bool testTrayHandling();        // Test tray handling operations

#endif // TESTS_H