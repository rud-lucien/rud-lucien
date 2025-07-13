#ifndef POSITION_CONFIG_H
#define POSITION_CONFIG_H

#include "Arduino.h"
#include "MotorController.h"
#include "OutputManager.h"

//=============================================================================
// POSITION CONFIGURATION SYSTEM
//=============================================================================

// Runtime position variables for Rail 1 (override #defines when set)
extern double runtimeRail1StagingMm; 
extern double runtimeRail1WC1PickupMm;
extern double runtimeRail1WC2PickupMm;
extern double runtimeRail1HandoffMm;

// Runtime position variables for Rail 2
extern double runtimeRail2HandoffMm;
extern double runtimeRail2WC3PickupMm;

// Control flags
extern bool useRuntimePositions;
extern bool sdCardInitialized;

// SD Card configuration
#define CONFIG_FILE_NAME "POSITIONS.TXT"
#define CONFIG_BACKUP_NAME "POS_BAK.TXT"

//=============================================================================
// POSITION MAPPING
//=============================================================================

// Map PositionTarget enums to teachable positions
typedef struct {
    PositionTarget target;
    const char* name;
    const char* description;
    int rail;
    double factoryDefault;
    double* runtimeVariable;
} TeachablePosition;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// System initialization
bool initPositionConfig();

// Position getter functions (replaces direct #define usage)
double getRail1StagingMm();
double getRail1WC1PickupMm();
double getRail1WC2PickupMm();
double getRail1HandoffMm();
double getRail2HandoffMm();
double getRail2WC3PickupMm();

// Generic position getter (works with PositionTarget enum)
double getTeachablePositionMm(PositionTarget target);

// Teaching functions by rail and position
bool teachCurrentPosition(int rail, PositionTarget target);
bool teachRail1Staging();
bool teachRail1WC1Pickup();
bool teachRail1WC2Pickup();
bool teachRail1Handoff();
bool teachRail2Handoff();
bool teachRail2WC3Pickup();

// Bulk operations
bool teachResetAllPositions();
bool teachResetRail(int rail);
void teachShowStatus();
void teachShowRail(int rail);

// Position validation
bool validateTaughtPosition(int rail, double positionMm, PositionTarget target);
bool isPositionTaught(PositionTarget target);

// SD Card operations
bool savePositionsToSD();
bool loadPositionsFromSD();
bool backupPositionsToSD();
bool isSDCardAvailable();

// Utility functions
const char* getPositionTargetName(PositionTarget target);
const char* getPositionTargetDescription(PositionTarget target);
TeachablePosition* getTeachablePositionInfo(PositionTarget target);

#endif // POSITION_CONFIG_H