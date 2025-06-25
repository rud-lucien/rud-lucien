#ifndef ENCODER_CONTROLLER_H
#define ENCODER_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include "Arduino.h"
#include "ClearCore.h"
#include "MotorController.h"

//=============================================================================
// ENCODER CONFIGURATION
//=============================================================================
// Hardware settings
#define ENCODER_CPR 100       // Counts per revolution of your MPG handwheel
#define ENCODER_DEBOUNCE_MS 5 // Debounce time for encoder readings in milliseconds

// Multiplier settings for precision control
#define MULTIPLIER_X1 1.0     // x1 gives ~1.6mm per full turn (100 counts × 1.0 × 1/61.27)
#define MULTIPLIER_X10 10.0   // x10 gives ~16mm per full turn
#define MULTIPLIER_X100 100.0 // x100 gives ~160mm per full turn

// Velocity settings for encoder-driven movement
#define ENCODER_MIN_VELOCITY 300  // Minimum velocity (steps/sec)
#define ENCODER_MAX_VELOCITY 8000 // Maximum velocity (steps/sec)

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Control state
extern bool encoderControlActive;           // Flag to enable/disable encoder control
extern int32_t lastEncoderPosition;         // Last encoder position
extern unsigned long lastEncoderUpdateTime; // Last time encoder was read
extern float currentMultiplier;             // Current multiplier setting

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================
// Initialization
void initEncoderControl(bool swapDirection = false, bool indexInverted = false);

// Encoder processing
void processEncoderInput();
void enableEncoderControl(bool enable);
void setEncoderMultiplier(int multiplier);

// Utility functions
const char *getMultiplierName(float multiplier);
void testConnections();

#endif // ENCODER_CONTROLLER_H