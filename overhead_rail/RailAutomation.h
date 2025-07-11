#ifndef RAIL_AUTOMATION_H
#define RAIL_AUTOMATION_H

#include "ClearCore.h"
#include "MotorController.h"
#include "ValveController.h"
#include "Sensors.h"
#include "Utils.h"
#include "PositionConfig.h"

//=============================================================================
// RAIL AUTOMATION CONSTANTS
//=============================================================================

// Rail 2 Collision Prevention Safety Constants
// CRITICAL SAFETY: Rail 2 collision zone definition for preventing Rail 1 interference
// 
// Zone Definition (based on system diagram analysis and MotorController.h positions):
//   Safe Zone:      RAIL2_HOME_POSITION to RAIL2_COLLISION_ZONE_START-1 (0-499mm)
//   Collision Zone: RAIL2_COLLISION_ZONE_START to RAIL2_COLLISION_ZONE_END (500-799mm)
//   Safe Zone:      RAIL2_COLLISION_ZONE_END+1 to RAIL2_HANDOFF and beyond (800-1000mm)
//
// Physical Layout Reference:
//   - Home/WC3 position: RAIL2_HOME_POSITION (0mm)
//   - Collision zone: Between WC3 and handoff where Rail 2 interferes with Rail 1 path
//   - Handoff position: RAIL2_HANDOFF (900mm) - at Rail 1-2 transfer station
//
// Safety Rules:
//   - Cylinder MUST be retracted for ANY movement involving collision zone
//   - This includes: entering, exiting, crossing, or moving within the zone
//   - Predefined moves (home, move-wc3, move-handoff) always retract cylinder
//   - Manual moves (move-mm-to, move-rel) conditionally retract based on path analysis
//
// Rationale:
//   - Uses actual position constants from MotorController.h for consistency
//   - Collision zone ends before handoff position to allow safe cylinder extension
//   - Accounts for Rail 1 carriage dimensions and movement envelope
//   - Fail-safe design: better to retract unnecessarily than risk collision
#define RAIL2_COLLISION_ZONE_START 500     // Start of collision risk zone with Rail 1 (mm)
#define RAIL2_COLLISION_ZONE_END   700     // End of collision risk zone with Rail 1 (mm)
#define RAIL2_SAFE_ZONE_END (RAIL2_COLLISION_ZONE_START - 1)  // Last safe position before collision zone

//=============================================================================
// RAIL AUTOMATION FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// Reusable Rail Automation Helper Functions
// Rail-agnostic helper functions for common automation operations
//-----------------------------------------------------------------------------
bool checkRailMovementReadiness(int railNumber);
bool parseAndValidateLabwareParameter(char* param, bool& carriageLoaded);
bool ensureCylinderRetractedForSafeMovement(bool movementInCollisionZone);

//-----------------------------------------------------------------------------
// Rail 1 Specific Automated Movement Functions
// Helper functions for Rail 1 predefined position movements
//-----------------------------------------------------------------------------
bool moveRail1CarriageToWC1(bool carriageLoaded);
bool moveRail1CarriageToWC2(bool carriageLoaded);
bool moveRail1CarriageToStaging(bool carriageLoaded);
bool moveRail1CarriageToHandoff(bool carriageLoaded);

//-----------------------------------------------------------------------------
// Rail 2 Specific Automated Movement Functions
// Helper functions for Rail 2 predefined position movements
//-----------------------------------------------------------------------------
bool moveRail2CarriageToWC3(bool carriageLoaded);
bool moveRail2CarriageToHandoff(bool carriageLoaded);

#endif // RAIL_AUTOMATION_H
