#include "EncoderController.h"
#include "Utils.h"
#include "OutputManager.h"

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
bool encoderControlActive = false;
int32_t lastEncoderPosition = 0;
unsigned long lastEncoderUpdateTime = 0;
float currentMultiplier = MULTIPLIER_X1;       // Default to fine control (0.1mm per count)
int currentVelocityRpm = ENCODER_DEFAULT_VELOCITY_RPM;
bool quadratureErrorDetected = false;
float mpgBasePositionMm = 0.0;                 // Base position when MPG was enabled
int32_t mpgBaseEncoderCount = 0;               // Base encoder count when MPG was enabled

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

const char *getMultiplierName(float multiplier)
{
    if (fabs(multiplier - MULTIPLIER_X1) < 0.01f) return "x1 (0.1mm/count)";
    if (fabs(multiplier - MULTIPLIER_X10) < 0.01f) return "x10 (1.0mm/count)";
    if (fabs(multiplier - MULTIPLIER_X100) < 0.01f) return "x100 (10mm/count)";
    
    static char buffer[30];
    sprintf(buffer, "x%.1f (%.1fmm/count)", multiplier * 10, multiplier);
    return buffer;
}

//=============================================================================
// INITIALIZATION
//=============================================================================

void initEncoderControl(bool swapDirection, bool indexInverted)
{
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
    encoderControlActive = false;
    quadratureErrorDetected = false;
    
    Console.serialInfo(F("Manual Pulse Generator (MPG) initialized"));
    Console.serialInfo(F("Use 'encoder,enable' to start manual control"));
}

//=============================================================================
// CONTROL FUNCTIONS (Following Teknic Approach)
//=============================================================================

void enableEncoderControl()
{
    // Check if motor is ready for encoder control
    if (!motorInitialized)
    {
        Console.serialError(F("Motor must be initialized before enabling MPG control"));
        return;
    }
    
    if (!isHomed)
    {
        Console.serialError(F("Motor must be homed before enabling MPG control"));
        Console.serialInfo(F("Use 'motor,home' to establish reference position"));
        return;
    }
    
    // Check that motor is not in a problematic state
    if (motorState == MOTOR_STATE_FAULTED)
    {
        Console.serialError(F("Motor is faulted - clear faults before enabling MPG"));
        return;
    }
    
    if (operationInProgress)
    {
        Console.serialError(F("Cannot enable MPG during automated operation"));
        return;
    }
    
    // Clear any errors but preserve encoder position for continuity
    if (EncoderIn.QuadratureError())
    {
        EncoderIn.Enable(false);
        delay(10);
        EncoderIn.Enable(true);
    }
    
    // Get current encoder position for tracking
    lastEncoderPosition = EncoderIn.Position();
    
    // Capture base position and encoder count for direct position control
    mpgBasePositionMm = currentPositionMm;
    mpgBaseEncoderCount = lastEncoderPosition;
    
    // Enable encoder control
    encoderControlActive = true;
    
    // Reset tracking variables
    lastEncoderUpdateTime = millis();
    quadratureErrorDetected = false;
    
    char msg[150];
    sprintf(msg, "MPG control enabled - Current position: %.2fmm", currentPositionMm);
    Console.serialInfo(msg);
    
    sprintf(msg, "Multiplier: %s, Velocity: %dRPM", 
            getMultiplierName(currentMultiplier), currentVelocityRpm);
    Console.serialInfo(msg);
    
    Console.serialInfo(F("Turn handwheel to move motor. Use 'encoder,disable' to stop"));
}

void disableEncoderControl()
{
    if (encoderControlActive)
    {
        encoderControlActive = false;
        Console.serialInfo(F("MPG control disabled"));
    }
}

bool isEncoderControlActive()
{
    return encoderControlActive;
}

//=============================================================================
// ENCODER PROCESSING (Teknic Direct Approach)
//=============================================================================

void processEncoderInput()
{
    // Only process if encoder control is active
    if (!encoderControlActive)
    {
        return;
    }
    
    // Check if motor is still ready
    if (motorState == MOTOR_STATE_FAULTED || operationInProgress)
    {
        Console.serialWarning(F("Motor state changed - disabling MPG control"));
        disableEncoderControl();
        return;
    }
    
    // Check for quadrature errors first
    if (EncoderIn.QuadratureError())
    {
        if (!quadratureErrorDetected)
        {
            Console.serialError(F("Quadrature error detected! Disabling MPG control"));
            quadratureErrorDetected = true;
            disableEncoderControl();
        }
        return;
    }
    
    // Read current encoder position
    int32_t currentEncoderPosition = EncoderIn.Position();
    
    // Calculate total encoder movement since MPG was enabled
    int32_t totalEncoderDelta = currentEncoderPosition - mpgBaseEncoderCount;
    
    // Calculate target position directly (like real milling machines)
    float totalMovementMm = totalEncoderDelta * currentMultiplier;
    float targetPositionMm = mpgBasePositionMm + totalMovementMm;
    
    // Check travel limits BEFORE attempting move
    if (targetPositionMm < 0.0)
    {
        // Clamp to minimum and update base to prevent further negative movement
        targetPositionMm = 0.0;
        mpgBasePositionMm = 0.0;
        mpgBaseEncoderCount = currentEncoderPosition;
        
        static unsigned long lastNegativeWarning = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastNegativeWarning > 1000)
        {
            Console.serialWarning(F("At negative travel limit"));
            lastNegativeWarning = currentTime;
        }
    }
    else if (targetPositionMm > MAX_TRAVEL_MM)
    {
        // Clamp to maximum and update base to prevent further positive movement
        targetPositionMm = MAX_TRAVEL_MM;
        mpgBasePositionMm = MAX_TRAVEL_MM;
        mpgBaseEncoderCount = currentEncoderPosition;
        
        static unsigned long lastPositiveWarning = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastPositiveWarning > 1000)
        {
            char msg[100];
            sprintf(msg, "At positive travel limit (%.1fmm)", MAX_TRAVEL_MM);
            Console.serialWarning(msg);
            lastPositiveWarning = currentTime;
        }
    }
    
    // Only update motor target if position has changed significantly
    int32_t encoderDelta = currentEncoderPosition - lastEncoderPosition;
    if (abs(encoderDelta) > 0)
    {
        // For MPG control, use faster velocity for more responsive feel
        int32_t velocityPps = (currentVelocityRpm * PULSES_PER_MM * 25.4) / (60 * 8);
        
        // Boost velocity for MPG responsiveness (faster than normal moves)
        velocityPps = velocityPps * 1.5; // 50% faster for immediate response
        
        // Ensure velocity stays within reasonable bounds
        if (velocityPps < 500) velocityPps = 500;   // Higher minimum for responsiveness
        if (velocityPps > 12000) velocityPps = 12000; // Higher maximum for speed
        
        // Set velocity for smooth movement
        MOTOR_CONNECTOR.VelMax(velocityPps);
        
        // Use absolute positioning for immediate response (like real MPG systems)
        int32_t targetPulses = mmToPulses(targetPositionMm);
        MOTOR_CONNECTOR.Move(targetPulses, MotorDriver::MOVE_TARGET_ABSOLUTE);
        
        // Update motor state and target tracking
        updateMotorTarget(targetPositionMm);
        
        // Log the movement with reduced verbosity (every 50ms instead of 100ms)
        unsigned long currentTime = millis();
        if (currentTime - lastEncoderUpdateTime > 50) // More frequent logging for debugging
        {
            char msg[150];
            sprintf(msg, "MPG: %ld counts → %.2fmm target (%s)", 
                    totalEncoderDelta, targetPositionMm, getMultiplierName(currentMultiplier));
            Console.serialDiagnostic(msg);
            lastEncoderUpdateTime = currentTime;
        }
        
        // Update tracking
        lastEncoderPosition = currentEncoderPosition;
    }
}

//=============================================================================
// CONFIGURATION FUNCTIONS
//=============================================================================

void setEncoderMultiplier(float multiplier)
{
    // Validate multiplier against predefined values
    if (fabs(multiplier - MULTIPLIER_X1) < 0.01f)
    {
        currentMultiplier = MULTIPLIER_X1;
    }
    else if (fabs(multiplier - MULTIPLIER_X10) < 0.01f)
    {
        currentMultiplier = MULTIPLIER_X10;
    }
    else if (fabs(multiplier - MULTIPLIER_X100) < 0.01f)
    {
        currentMultiplier = MULTIPLIER_X100;
    }
    else
    {
        Console.serialError(F("Invalid multiplier. Use 0.1, 1.0, or 10.0"));
        return;
    }
    
    char msg[100];
    sprintf(msg, "MPG multiplier set to %s", getMultiplierName(currentMultiplier));
    Console.serialInfo(msg);
    
    sprintf(msg, "One encoder count = %.1fmm movement", currentMultiplier);
    Console.serialInfo(msg);
}

void setEncoderVelocity(int velocityRpm)
{
    if (velocityRpm < ENCODER_MIN_VELOCITY_RPM || velocityRpm > ENCODER_MAX_VELOCITY_RPM)
    {
        char msg[100];
        sprintf(msg, "Velocity must be between %d and %d RPM", 
                ENCODER_MIN_VELOCITY_RPM, ENCODER_MAX_VELOCITY_RPM);
        Console.serialError(msg);
        return;
    }
    
    currentVelocityRpm = velocityRpm;
    
    char msg[100];
    sprintf(msg, "MPG velocity set to %d RPM", currentVelocityRpm);
    Console.serialInfo(msg);
}

//=============================================================================
// STATUS AND DIAGNOSTICS
//=============================================================================

void printEncoderStatus()
{
    char msg[200];
    
    if (!encoderControlActive)
    {
        Console.serialInfo(F("MPG Status: DISABLED"));
    }
    else
    {
        Console.serialInfo(F("MPG Status: ENABLED"));
        sprintf(msg, "Current position: %.2fmm", currentPositionMm);
        Console.serialInfo(msg);
    }
    
    sprintf(msg, "Multiplier: %s", getMultiplierName(currentMultiplier));
    Console.serialInfo(msg);
    
    sprintf(msg, "Velocity: %d RPM", currentVelocityRpm);
    Console.serialInfo(msg);
    
    sprintf(msg, "Encoder position: %ld counts", EncoderIn.Position());
    Console.serialInfo(msg);
    
    sprintf(msg, "Encoder velocity: %ld counts/sec", EncoderIn.Velocity());
    Console.serialInfo(msg);
    
    if (hasQuadratureError())
    {
        Console.serialWarning(F("Quadrature error detected!"));
    }
}

bool hasQuadratureError()
{
    return EncoderIn.QuadratureError() || quadratureErrorDetected;
}

void clearQuadratureError()
{
    // Clear hardware quadrature error by disabling/enabling (Teknic approach)
    EncoderIn.Enable(false);
    delay(10);
    EncoderIn.Enable(true);
    EncoderIn.Position(0);
    
    // Clear our error flag and reset base position tracking
    quadratureErrorDetected = false;
    lastEncoderPosition = 0;
    mpgBaseEncoderCount = 0;
    mpgBasePositionMm = currentPositionMm; // Reset to current motor position
    
    Console.serialInfo(F("Quadrature error cleared"));
}
