#ifndef ENCODER_CONTROLLER_H
#define ENCODER_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include "ClearCore.h"
#include "MotorController.h"

//=============================================================================
// ENCODER CONFIGURATION
//=============================================================================
// Hardware settings
#define ENCODER_CPR 100                    // Counts per revolution of MPG handwheel
#define ENCODER_UPDATE_INTERVAL_MS 20      // How often to process encoder (50Hz)

// Multiplier settings for precision control - Direct mm per encoder count
#define MULTIPLIER_X1 0.1                  // x1: Fine control (0.1mm per encoder count)
#define MULTIPLIER_X10 1.0                 // x10: Medium control (1.0mm per encoder count)  
#define MULTIPLIER_X100 10.0               // x100: Coarse control (10.0mm per encoder count)

// Velocity settings for encoder-driven movement
#define ENCODER_MIN_VELOCITY_RPM 50        // Minimum velocity
#define ENCODER_MAX_VELOCITY_RPM 400       // Maximum velocity
#define ENCODER_DEFAULT_VELOCITY_RPM 200   // Default velocity

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Control state
extern bool encoderControlActive;           // Flag to enable/disable encoder control
extern int32_t lastEncoderPosition;         // Last encoder position (for tracking only)
extern unsigned long lastEncoderUpdateTime; // Last time encoder was processed
extern float currentMultiplier;             // Current multiplier setting (mm per count)
extern int currentVelocityRpm;             // Current velocity in RPM
extern bool quadratureErrorDetected;       // Error state tracking
extern float mpgBasePositionMm;            // Base position when MPG was enabled
extern int32_t mpgBaseEncoderCount;        // Base encoder count when MPG was enabled

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================
// Initialization
void initEncoderControl(bool swapDirection = false, bool indexInverted = false);

// Control functions
void enableEncoderControl();                // Enable encoder control
void disableEncoderControl();               // Disable encoder control
bool isEncoderControlActive();              // Check if encoder control is active

// Encoder processing (Teknic approach)
void processEncoderInput();                 // Main encoder processing - call in loop

// Configuration
void setEncoderMultiplier(float multiplier);  // Set multiplier (0.1, 1.0, 10.0)
void setEncoderVelocity(int velocityRpm);     // Set encoder movement velocity

// Status and diagnostics
const char *getMultiplierName(float multiplier);
void printEncoderStatus();
bool hasQuadratureError();
void clearQuadratureError();

#endif // ENCODER_CONTROLLER_H