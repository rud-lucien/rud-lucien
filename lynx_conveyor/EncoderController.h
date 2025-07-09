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

// Multiplier settings for precision control - Integer representation for fast math
// Internal values in 0.1mm units to eliminate floating-point operations
#define MULTIPLIER_X1_INT 1                // x1: Fine control (1 = 0.1mm per encoder count)
#define MULTIPLIER_X10_INT 10              // x10: Medium control (10 = 1.0mm per encoder count)  
#define MULTIPLIER_X100_INT 100            // x100: Coarse control (100 = 10.0mm per encoder count)

// User-facing multiplier values (for compatibility)
#define MULTIPLIER_X1 0.1                  // x1: Fine control (0.1mm per encoder count)
#define MULTIPLIER_X10 1.0                 // x10: Medium control (1.0mm per encoder count)  
#define MULTIPLIER_X100 10.0               // x100: Coarse control (10.0mm per encoder count)

// Velocity settings for encoder-driven movement
#define ENCODER_MIN_VELOCITY_RPM 50        // Minimum velocity
#define ENCODER_MAX_VELOCITY_RPM 400       // Maximum velocity
#define ENCODER_DEFAULT_VELOCITY_RPM 100   // Default velocity

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Control state
extern bool encoderControlActive;           // Flag to enable/disable encoder control
extern int32_t lastEncoderPosition;         // Last encoder position (for tracking only)
extern unsigned long lastEncoderUpdateTime; // Last time encoder was processed
extern int16_t currentMultiplierInt;        // Current multiplier setting (0.1mm units) for fast math
extern float currentMultiplier;             // Current multiplier setting (mm per count) for compatibility
extern int currentVelocityRpm;             // Current velocity in RPM
extern bool quadratureErrorDetected;       // Error state tracking
extern int32_t mpgBasePositionDeciMm;      // Base position when MPG was enabled (0.1mm units)
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