#include "EncoderController.h"

// Initialize control variables
bool encoderControlActive = false;
int32_t lastEncoderPosition = 0;
unsigned long lastEncoderUpdateTime = 0;
float currentMultiplier = MULTIPLIER_X1;  // Default to x1 multiplier
bool quadratureError = false;

// Helper function to get the multiplier name
const char* getMultiplierName(float multiplier) {
    // Use almost-equal comparison for float values
    if (fabs(multiplier - MULTIPLIER_X1) < 0.001f) return "1";
    if (fabs(multiplier - MULTIPLIER_X10) < 0.001f) return "10";
    if (fabs(multiplier - MULTIPLIER_X100) < 0.001f) return "100";
    
    // Debug output to help diagnose the issue
    char buffer[50];
    sprintf(buffer, "unknown:%.2f", multiplier);
    Serial.print(F("[DEBUG] getMultiplierName received: "));
    Serial.println(buffer);
    
    return "unknown";
}

// Initialize the encoder interface
void initEncoderControl(bool swapDirection, bool indexInverted) {
    // Enable the encoder input feature
    EncoderIn.Enable(true);
    
    // Zero the position to start
    EncoderIn.Position(0);
    
    // Set the encoder direction
    EncoderIn.SwapDirection(swapDirection);
    
    // Set the sense of index detection
    EncoderIn.IndexInverted(indexInverted);
    
    // Reset tracking variables
    lastEncoderPosition = 0;
    lastEncoderUpdateTime = millis();
    currentMultiplier = MULTIPLIER_X1;  // Make sure default is set
    
    Serial.println(F("[MESSAGE] MPG Handwheel interface initialized"));
    Serial.println(F("[MESSAGE] Setup sequence: "));
    Serial.println(F("  1. 'motor init' - Initialize the motor"));
    Serial.println(F("  2. 'home' - Establish a reference position"));
    Serial.println(F("  3. 'encoder enable' - Activate handwheel control"));
    Serial.println(F("  4. 'encoder multiplier [1|10|100]' - Set step multiplier"));
}

// Process encoder movement
void processEncoderInput() {
    if (!encoderControlActive || motorState == MOTOR_STATE_FAULTED) {
        return;  // Only process when encoder control is active and motor is not faulted
    }
    
    static unsigned long lastDebugTime = 0;
    static int32_t accumulatedDelta = 0; // Accumulate encoder movement over time
    static unsigned long lastMoveTime = 0;
    
    unsigned long currentTime = millis();
    
    // Periodically output encoder position for debugging (once per second)
    if (currentTime - lastDebugTime > 1000) {
        lastDebugTime = currentTime;
        Serial.print(F("[DEBUG] Encoder position: "));
        Serial.print(EncoderIn.Position());
        Serial.print(F(", Active: "));
        Serial.print(encoderControlActive ? "Yes" : "No");
        Serial.print(F(", Motor state: "));
        
        // Direct output of motor state
        switch (motorState) {
            case MOTOR_STATE_IDLE: Serial.println(F("IDLE")); break;
            case MOTOR_STATE_MOVING: Serial.println(F("MOVING")); break;
            case MOTOR_STATE_HOMING: Serial.println(F("HOMING")); break;
            case MOTOR_STATE_FAULTED: Serial.println(F("FAULTED")); break;
            case MOTOR_STATE_NOT_READY: Serial.println(F("NOT_READY")); break;
            default: Serial.println(F("UNKNOWN")); break;
        }
    }
    
    // Read current encoder position first
    int32_t currentEncoderPosition = EncoderIn.Position();
    int32_t encoderDelta = currentEncoderPosition - lastEncoderPosition;
    
    // If encoder has moved
    if (encoderDelta != 0) {
        Serial.print(F("[DEBUG] Encoder delta: "));
        Serial.println(encoderDelta);
        
        // Check for quadrature errors after movement detected (less prone to false positives)
        if (EncoderIn.QuadratureError()) {
            Serial.println(F("[ERROR] Quadrature error detected in encoder! Disabling control."));
            
            // The proper way to clear errors is to disable/enable
            EncoderIn.Enable(false);
            EncoderIn.Enable(true);
            
            // If error persists, disable encoder control
            if (EncoderIn.QuadratureError()) {
                encoderControlActive = false;
                Serial.println(F("[MESSAGE] MPG Handwheel control disabled due to persistent error"));
                return;
            }
        }
        
        // Update accumulated delta
        accumulatedDelta += encoderDelta;
        
        // Calculate time since last update
        unsigned long timeDelta = currentTime - lastEncoderUpdateTime;
        if (timeDelta < ENCODER_DEBOUNCE_MS) {
            // Update position but wait to process movement (debounce)
            lastEncoderPosition = currentEncoderPosition;
            return;
        }
        
        // Wait until we have a significant movement or enough time has passed
        bool shouldMove = false;
        
        // Either we have accumulated enough movement or waited long enough since last move
        if (abs(accumulatedDelta) >= 5 || (currentTime - lastMoveTime > 100 && accumulatedDelta != 0)) {
            shouldMove = true;
        }
        
        if (shouldMove && motorState != MOTOR_STATE_MOVING) {
            // Only move if we're not already moving
            // Get velocity directly from the encoder for better accuracy
            int32_t currentVelocity = EncoderIn.Velocity();
            
            // Scale the velocity based on the encoder's velocity and direction
            int32_t absVelocity = abs(currentVelocity);
            int32_t scaledVelocity;
            
            // Adjust velocity scaling for better response
            if (absVelocity < 10) {
                scaledVelocity = ENCODER_MIN_VELOCITY;
            } else if (absVelocity > 100) {
                scaledVelocity = ENCODER_MAX_VELOCITY;
            } else {
                // Linear scaling between min and max
                double velocityRatio = (absVelocity - 10.0) / 90.0;
                scaledVelocity = ENCODER_MIN_VELOCITY + velocityRatio * (ENCODER_MAX_VELOCITY - ENCODER_MIN_VELOCITY);
            }
            
            // Calculate steps to move based on accumulated delta and current multiplier
            float stepsToMoveFloat = accumulatedDelta * currentMultiplier;
            int32_t stepsToMove = (int32_t)stepsToMoveFloat;
            
            // Make sure we move at least a meaningful amount
            if (stepsToMove == 0 && accumulatedDelta != 0) {
                stepsToMove = (accumulatedDelta > 0) ? 1 : -1;
            }
            
            // Calculate target position in mm
            float targetPositionMm = currentPositionMm + (pulsesToMm(stepsToMove));
            
            // Check against limits
            if (targetPositionMm < 0 || targetPositionMm > MAX_TRAVEL_MM) {
                Serial.print(F("[WARNING] Encoder movement rejected: target position "));
                Serial.print(targetPositionMm);
                Serial.print(F(" mm is outside allowed range (0 to "));
                Serial.print(MAX_TRAVEL_MM);
                Serial.println(F(" mm)"));
            } else {
                // We don't need to set those global variables since they're not defined in this scope
                // Just proceed with the movement
                
                Serial.print(F("[MPG] Moving motor by "));
                Serial.print(stepsToMove);
                Serial.print(F(" steps (multiplier: x"));
                Serial.print(getMultiplierName(currentMultiplier));
                Serial.print(F("), velocity: "));
                Serial.print(scaledVelocity);
                Serial.print(F(", target: "));
                Serial.print(targetPositionMm);
                Serial.println(F(" mm"));
                
                // Set velocity for this move
                MOTOR_CONNECTOR.VelMax(scaledVelocity);
                
                // Move the motor relative to current position
                MOTOR_CONNECTOR.Move(stepsToMove);
                
                // Update motor state
                motorState = MOTOR_STATE_MOVING;
                
                // Reset accumulated delta after movement
                accumulatedDelta = 0;
                lastMoveTime = currentTime;
            }
        }
        
        // Update tracking variables
        lastEncoderPosition = currentEncoderPosition;
        lastEncoderUpdateTime = currentTime;
    }
}

// Enable or disable encoder control
void enableEncoderControl(bool enable) {
    // Only allow enabling if motor is both initialized and homed
    if (enable) {
        if (!motorInitialized) {
            Serial.println(F("[ERROR] Motor must be initialized before enabling MPG control"));
            return;
        }
        if (!isHomed) {
            Serial.println(F("[ERROR] Motor must be homed before enabling MPG control"));
            Serial.println(F("[MESSAGE] Use the 'home' command to establish a reference position"));
            return;
        }
        
        // The proper way to reset the encoder and clear errors
        // is to disable and re-enable it
        EncoderIn.Enable(false);
        EncoderIn.Enable(true);
        EncoderIn.Position(0);
        
        // Motor is ready, enable encoder control
        encoderControlActive = true;
        
        // Reset encoder position when enabling
        lastEncoderPosition = 0;
        lastEncoderUpdateTime = millis();
        
        Serial.print(F("[MESSAGE] MPG handwheel control enabled - current position: "));
        Serial.print(currentPositionMm);
        Serial.println(F(" mm"));
        Serial.print(F("[MESSAGE] Using multiplier x"));
        Serial.print(getMultiplierName(currentMultiplier));
        Serial.print(F(" ("));
        Serial.print(currentMultiplier);
        Serial.println(F(")"));
        Serial.println(F("[MESSAGE] Issue 'encoder,disable' when finished with manual control"));
    } else {
        encoderControlActive = false;
        Serial.println(F("[MESSAGE] MPG Handwheel control disabled"));
    }
}

// Set the multiplier (x1, x10, x100)
void setEncoderMultiplier(int multiplier) {
    Serial.print(F("[DEBUG] setEncoderMultiplier called with: "));
    Serial.println(multiplier);
    
    switch (multiplier) {
        case 1:
            currentMultiplier = MULTIPLIER_X1;
            Serial.print(F("[MESSAGE] MPG multiplier set to x1 (fine control): "));
            Serial.println(currentMultiplier);
            break;
        case 10:
            currentMultiplier = MULTIPLIER_X10;
            Serial.print(F("[MESSAGE] MPG multiplier set to x10 (medium control): "));
            Serial.println(currentMultiplier);
            break;
        case 100:
            currentMultiplier = MULTIPLIER_X100;
            Serial.print(F("[MESSAGE] MPG multiplier set to x100 (coarse control): "));
            Serial.println(currentMultiplier);
            break;
        default:
            Serial.println(F("[ERROR] Invalid multiplier. Use 1, 10, or 100"));
            return;
    }
    
    // Verify the currentMultiplier was actually set
    Serial.print(F("[DEBUG] currentMultiplier is now: "));
    Serial.println(currentMultiplier);
}

