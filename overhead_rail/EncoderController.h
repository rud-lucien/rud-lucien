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
// Note: ENCODER_UPDATE_INTERVAL_MS removed - timing handled dynamically in processEncoderInput()

// Integer math scaling for performance (no FPU on ClearCore)
// Uses Utils.h SCALE_FACTOR = 100 (1mm = 100 scaled units)

// Multiplier settings for precision control - Integer scaled (units per encoder count)
// These represent movement in scaled units (mm * 100) per encoder count
#define MULTIPLIER_X1_SCALED 10            // 1x: Fine control (0.1mm per count = 10 scaled units)
#define MULTIPLIER_X10_SCALED 100          // 10x: General control (1.0mm per count = 100 scaled units)  
#define MULTIPLIER_X100_SCALED 1000        // 100x: Rapid control (10.0mm per count = 1000 scaled units)

// Velocity settings for encoder-driven movement
#define ENCODER_MIN_VELOCITY_RPM 50        // Minimum velocity
#define ENCODER_MAX_VELOCITY_RPM 400       // Maximum velocity
#define ENCODER_DEFAULT_VELOCITY_RPM 100   // Default velocity

// Dynamic velocity adjustment settings
#define ENCODER_VELOCITY_SMOOTHING_FACTOR 4    // Higher = smoother response (1-10)
#define ENCODER_MIN_VELOCITY_SCALE 0.5         // Minimum velocity scale factor (50% of base)
#define ENCODER_MAX_VELOCITY_SCALE 3.0         // Maximum velocity scale factor (300% of base)
#define ENCODER_VELOCITY_THRESHOLD_CPS 2       // Counts per second threshold for velocity scaling

// Timeout safety settings
#define ENCODER_TIMEOUT_MS 300000              // 5 minutes of inactivity timeout
#define ENCODER_ACTIVITY_CHECK_INTERVAL_MS 10000  // Check for timeout every 10 seconds

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

// Dynamic velocity and timeout safety tracking
extern unsigned long lastEncoderActivity;  // Last time encoder moved (for timeout)
extern unsigned long lastTimeoutCheck;     // Last time we checked for timeout
extern float currentVelocityScale;         // Current velocity scaling factor
extern int32_t smoothedEncoderVelocity;    // Smoothed encoder velocity for stable scaling

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
// Note: These are declared in Utils.h and implemented in Utils.cpp
// int32_t mmToScaled(double mm);     - Convert mm to scaled integer units
// double scaledToMm(int32_t scaled); - Convert scaled units back to mm

#endif // ENCODER_CONTROLLER_H