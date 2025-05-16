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
bool testHomingRepeatability();  // Test homing repeatability over multiple cycles
bool testMotorRange();           // Test motor movement across its range

#endif // TESTS_H