#ifndef ENCODER_CONTROLLER_H
#define ENCODER_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <Arduino.h>
#include "ClearCore.h"
#include "MotorController.h"
#include "Utils.h"

//=============================================================================
// ENCODER CONFIGURATION
//=============================================================================
// Hardware settings
#define ENCODER_CPR 100                    // Counts per revolution of MPG handwheel
#define ENCODER_UPDATE_INTERVAL_MS 20      // How often to process encoder (50Hz)

// Integer math scaling for performance (no FPU on ClearCore)


// Multiplier settings for precision control - Integer scaled (units per encoder count)
// These represent movement in scaled units (mm * 100) per encoder count
#define MULTIPLIER_X1_SCALED 10            // x1: Fine control (0.1mm per count = 10 scaled units)
#define MULTIPLIER_X10_SCALED 100          // x10: Medium control (1.0mm per count = 100 scaled units)  
#define MULTIPLIER_X100_SCALED 1000        // x100: Coarse control (10.0mm per count = 1000 scaled units)

// Velocity settings for encoder-driven movement
#define ENCODER_MIN_VELOCITY_RPM 50        // Minimum velocity
#define ENCODER_MAX_VELOCITY_RPM 400       // Maximum velocity
#define ENCODER_DEFAULT_VELOCITY_RPM 100   // Default velocity

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Control state
extern bool encoderControlActive;           // Flag to enable/disable encoder control
extern int activeEncoderRail;               // Which rail is under encoder control (1 or 2)
extern int32_t lastEncoderPosition;         // Last encoder position (for tracking only)
extern unsigned long lastEncoderUpdateTime; // Last time encoder was processed
extern int16_t currentMultiplierScaled;     // Current multiplier in scaled units per count
extern int currentVelocityRpm;             // Current velocity in RPM
extern bool quadratureErrorDetected;       // Error state tracking
extern int32_t mpgBasePositionScaled;      // Base position when MPG was enabled (scaled units)
extern int32_t mpgBaseEncoderCount;        // Base encoder count when MPG was enabled

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================
// Initialization
void initEncoderControl(bool swapDirection = false, bool indexInverted = false);

// Control functions
void enableEncoderControl(int rail);        // Enable encoder control for specific rail
void disableEncoderControl();               // Disable encoder control
bool isEncoderControlActive();              // Check if encoder control is active
int getActiveEncoderRail();                 // Get which rail is under encoder control

// Encoder processing (Teknic approach)
void processEncoderInput();                 // Main encoder processing - call in loop

// Configuration
void setEncoderMultiplier(float multiplier);  // Set multiplier (0.1, 1.0, 10.0) - converts to scaled internally
void setEncoderVelocity(int velocityRpm);     // Set encoder movement velocity

// Status and diagnostics
const char *getMultiplierName(int16_t multiplierScaled);
void printEncoderStatus();
bool hasQuadratureError();
void clearQuadratureError();

// Utility functions for integer math conversion


#endif // ENCODER_CONTROLLER_H