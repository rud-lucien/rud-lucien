#include "EncoderController.h"
#include "Utils.h"
#include "OutputManager.h"

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_MULTIPLIER_CUSTOM[] PROGMEM = "x%.1f (%.1fmm/count)";
const char FMT_RAIL_MUST_BE_READY[] PROGMEM = "Rail %d motor must be ready before enabling MPG control";
const char FMT_RAIL_MUST_BE_HOMED[] PROGMEM = "Rail %d must be homed before enabling MPG control";
const char FMT_RAIL_FAULTED_CLEAR[] PROGMEM = "Rail %d motor is faulted - clear faults before enabling MPG";
const char FMT_MPG_ENABLED[] PROGMEM = "MPG control enabled for Rail %d - Current position: %.2fmm";
const char FMT_MPG_MULTIPLIER_VELOCITY[] PROGMEM = "Multiplier: %s, Velocity: %dRPM";
const char FMT_AT_POSITIVE_LIMIT[] PROGMEM = "At positive travel limit (%.1fmm)";
const char FMT_MPG_RAIL_MOVEMENT[] PROGMEM = "MPG Rail %d: %ld counts → %.2fmm target (%s)";
const char FMT_MPG_MULTIPLIER_SET[] PROGMEM = "MPG multiplier set to %s";
const char FMT_ONE_COUNT_EQUALS[] PROGMEM = "One encoder count = %.1fmm movement";
const char FMT_VELOCITY_RANGE[] PROGMEM = "Velocity must be between %d and %d RPM";
const char FMT_MPG_VELOCITY_SET[] PROGMEM = "MPG velocity set to %d RPM";
const char FMT_MPG_STATUS_ENABLED[] PROGMEM = "MPG Status: ENABLED for Rail %d";
const char FMT_POSITION_MOVING[] PROGMEM = "Position: %.2fmm [MOVING]";
const char FMT_POSITION_NOT_READY[] PROGMEM = "Position: %.2fmm [MOTOR NOT READY]";
const char FMT_POSITION_SETTLED[] PROGMEM = "Position: %.2fmm [SETTLED]";
const char FMT_POSITION_SETTLING[] PROGMEM = "Position: %.2fmm [SETTLING]";
const char FMT_MULTIPLIER_STATUS[] PROGMEM = "Multiplier: %s";
const char FMT_VELOCITY_RPM[] PROGMEM = "Velocity: %d RPM";
const char FMT_ENCODER_POSITION[] PROGMEM = "Encoder position: %ld counts";
const char FMT_ENCODER_VELOCITY[] PROGMEM = "Encoder velocity: %ld counts/sec";
const char FMT_MPG_BASE_POSITION[] PROGMEM = "MPG base position: %.2fmm (encoder count %ld)";
const char FMT_MPG_EXPECTED_POSITION[] PROGMEM = "Expected position from MPG: %.2fmm (delta: %ld counts)";

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
bool encoderControlActive = false;
int activeEncoderRail = 0;                     // Which rail is under encoder control
int32_t lastEncoderPosition = 0;
unsigned long lastEncoderUpdateTime = 0;
int16_t currentMultiplierScaled = MULTIPLIER_X1_SCALED;  // Default to fine control (0.1mm per count)
int currentVelocityRpm = ENCODER_DEFAULT_VELOCITY_RPM;
bool quadratureErrorDetected = false;
int32_t mpgBasePositionScaled = 0;             // Base position when MPG was enabled (scaled units)
int32_t mpgBaseEncoderCount = 0;               // Base encoder count when MPG was enabled

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================



const char *getMultiplierName(int16_t multiplierScaled)
{
    if (multiplierScaled == MULTIPLIER_X1_SCALED) return "x1 (0.1mm/count)";
    if (multiplierScaled == MULTIPLIER_X10_SCALED) return "x10 (1.0mm/count)";
    if (multiplierScaled == MULTIPLIER_X100_SCALED) return "x100 (10mm/count)";
    
    static char buffer[30];
    double mmPerCount = (double)multiplierScaled / SCALE_FACTOR;
    sprintf_P(buffer, FMT_MULTIPLIER_CUSTOM, mmPerCount * 10, mmPerCount);
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
    activeEncoderRail = 0;
    quadratureErrorDetected = false;
    
    Console.serialInfo(F("Manual Pulse Generator (MPG) initialized"));
    Console.serialInfo(F("Use 'encoder,enable,1' or 'encoder,enable,2' to start manual control"));
}

//=============================================================================
// CONTROL FUNCTIONS (Following Teknic Approach)
//=============================================================================

void enableEncoderControl(int rail)
{
    // Validate rail number
    if (rail != 1 && rail != 2) {
        Console.serialError(F("Invalid rail number. Use 1 or 2"));
        return;
    }
    
    // Check if motor is ready for encoder control
    if (!isMotorReady(rail))
    {
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_MUST_BE_READY, rail);
        Console.serialError(msg);
        return;
    }
    
    if (!isHomingComplete(rail))
    {
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_MUST_BE_HOMED, rail);
        Console.serialError(msg);
        Console.serialInfo(F("Use homing commands to establish reference position"));
        return;
    }
    
    // Check that motor is not in a problematic state
    MotorState currentState = updateMotorState(rail);
    if (currentState == MOTOR_STATE_FAULTED)
    {
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_FAULTED_CLEAR, rail);
        Console.serialError(msg);
        return;
    }
    
    if (isMotorMoving(rail))
    {
        Console.serialError(F("Cannot enable MPG during automated operation"));
        return;
    }
    
    // Disable previous encoder control if active
    if (encoderControlActive) {
        disableEncoderControl();
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
    mpgBasePositionScaled = mmToScaled(getMotorPositionMm(rail));
    mpgBaseEncoderCount = lastEncoderPosition;
    
    // Enable encoder control
    encoderControlActive = true;
    activeEncoderRail = rail;
    
    // Reset tracking variables
    lastEncoderUpdateTime = millis();
    quadratureErrorDetected = false;
    
    char msg[150];
    sprintf_P(msg, FMT_MPG_ENABLED, 
            rail, scaledToMm(mpgBasePositionScaled));
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_MPG_MULTIPLIER_VELOCITY, 
            getMultiplierName(currentMultiplierScaled), currentVelocityRpm);
    Console.serialInfo(msg);
    
    Console.serialInfo(F("Turn handwheel to move motor. Use 'encoder,disable' to stop"));
}

void disableEncoderControl()
{
    if (encoderControlActive)
    {
        encoderControlActive = false;
        activeEncoderRail = 0;
        Console.serialInfo(F("MPG control disabled"));
    }
}

bool isEncoderControlActive()
{
    return encoderControlActive;
}

int getActiveEncoderRail()
{
    return activeEncoderRail;
}

//=============================================================================
// ENCODER PROCESSING (Teknic Direct Approach)
//=============================================================================

void processEncoderInput()
{
    // Only process if encoder control is active
    if (!encoderControlActive || activeEncoderRail == 0)
    {
        return;
    }
    
    // Check if motor is still ready
    MotorState currentState = updateMotorState(activeEncoderRail);
    if (currentState == MOTOR_STATE_FAULTED || isMotorMoving(activeEncoderRail))
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
    
    // Calculate encoder delta using integer math
    int32_t encoderDelta = currentEncoderPosition - lastEncoderPosition;
    
    // Exit early if no movement detected
    if (encoderDelta == 0) {
        return;
    }
    
    // Calculate movement using integer math (scaled units)
    // This is more efficient than floating point math and avoids precision issues
    int32_t movementScaled = encoderDelta * currentMultiplierScaled;
    
    // Calculate total encoder movement since MPG was enabled
    int32_t totalEncoderDelta = currentEncoderPosition - mpgBaseEncoderCount;
    int32_t targetPositionScaled = mpgBasePositionScaled + (totalEncoderDelta * currentMultiplierScaled);
    
    // Get max travel for this rail (convert to scaled units)
    int32_t maxTravelScaled = (activeEncoderRail == 1) ? 
        mmToScaled(RAIL1_MAX_TRAVEL_MM) : mmToScaled(RAIL2_MAX_TRAVEL_MM);
    
    // Check travel limits BEFORE attempting move
    if (targetPositionScaled < 0)
    {
        // Clamp to minimum and update base to prevent further negative movement
        targetPositionScaled = 0;
        mpgBasePositionScaled = 0;
        mpgBaseEncoderCount = currentEncoderPosition;
        
        static unsigned long lastNegativeWarning = 0;
        unsigned long currentTime = millis();
        if (waitTimeReached(currentTime, lastNegativeWarning, 1000))
        {
            Console.serialWarning(F("At negative travel limit"));
            lastNegativeWarning = currentTime;
        }
    }
    else if (targetPositionScaled > maxTravelScaled)
    {
        // Clamp to maximum and update base to prevent further positive movement
        targetPositionScaled = maxTravelScaled;
        mpgBasePositionScaled = maxTravelScaled;
        mpgBaseEncoderCount = currentEncoderPosition;
        
        static unsigned long lastPositiveWarning = 0;
        unsigned long currentTime = millis();
        if (waitTimeReached(currentTime, lastPositiveWarning, 1000))
        {
            char msg[100];
            sprintf_P(msg, FMT_AT_POSITIVE_LIMIT, scaledToMm(maxTravelScaled));
            Console.serialWarning(msg);
            lastPositiveWarning = currentTime;
        }
    }
    
    // Get motor reference for this rail
    MotorDriver& motor = getMotorByRail(activeEncoderRail);
    
    // Calculate velocity using integer math
    int32_t velocityPps = rpmToPps(currentVelocityRpm);
    
    // Boost velocity for MPG responsiveness (use integer math for 50% boost)
    velocityPps = velocityPps + (velocityPps >> 1); // 50% faster for immediate response
    
    // Ensure velocity stays within reasonable bounds
    if (velocityPps < 500) velocityPps = 500;   // Higher minimum for responsiveness
    if (velocityPps > 12000) velocityPps = 12000; // Higher maximum for speed
    
    // Set velocity for smooth movement
    motor.VelMax(velocityPps);
    
    // Use absolute positioning for immediate response (like real MPG systems)
    // Convert scaled position to mm for motor control (only one conversion needed)
    double targetPositionMm = scaledToMm(targetPositionScaled);
    int32_t targetPulses = mmToPulses(targetPositionMm, activeEncoderRail);
    motor.Move(targetPulses, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Log the movement with reduced verbosity (every 50ms instead of 100ms)
    unsigned long currentTime = millis();
    if (waitTimeReached(currentTime, lastEncoderUpdateTime, 50)) // More frequent logging for debugging
    {
        char msg[150];
        sprintf_P(msg, FMT_MPG_RAIL_MOVEMENT, 
                activeEncoderRail, totalEncoderDelta, targetPositionMm, getMultiplierName(currentMultiplierScaled));
        Console.serialDiagnostic(msg);
        lastEncoderUpdateTime = currentTime;
    }
    
    // Update tracking
    lastEncoderPosition = currentEncoderPosition;
}

//=============================================================================
// CONFIGURATION FUNCTIONS
//=============================================================================

void setEncoderMultiplier(float multiplier)
{
    // Validate multiplier against predefined values and convert to scaled integer
    if (fabs(multiplier - 0.1f) < 0.01f)
    {
        currentMultiplierScaled = MULTIPLIER_X1_SCALED;
    }
    else if (fabs(multiplier - 1.0f) < 0.01f)
    {
        currentMultiplierScaled = MULTIPLIER_X10_SCALED;
    }
    else if (fabs(multiplier - 10.0f) < 0.01f)
    {
        currentMultiplierScaled = MULTIPLIER_X100_SCALED;
    }
    else
    {
        Console.serialError(F("Invalid multiplier. Use 0.1, 1.0, or 10.0"));
        return;
    }
    
    char msg[100];
    sprintf_P(msg, FMT_MPG_MULTIPLIER_SET, getMultiplierName(currentMultiplierScaled));
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_ONE_COUNT_EQUALS, scaledToMm(currentMultiplierScaled));
    Console.serialInfo(msg);
}

void setEncoderVelocity(int velocityRpm)
{
    if (velocityRpm < ENCODER_MIN_VELOCITY_RPM || velocityRpm > ENCODER_MAX_VELOCITY_RPM)
    {
        char msg[100];
        sprintf_P(msg, FMT_VELOCITY_RANGE, 
                ENCODER_MIN_VELOCITY_RPM, ENCODER_MAX_VELOCITY_RPM);
        Console.serialError(msg);
        return;
    }
    
    currentVelocityRpm = velocityRpm;
    
    char msg[100];
    sprintf_P(msg, FMT_MPG_VELOCITY_SET, currentVelocityRpm);
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
        sprintf_P(msg, FMT_MPG_STATUS_ENABLED, activeEncoderRail);
        Console.serialInfo(msg);
        
        // Get the most accurate position reading available from the stepper motor controller
        MotorDriver& motor = getMotorByRail(activeEncoderRail);
        bool motorMoving = isMotorMoving(activeEncoderRail);
        bool motorInPosition = isMotorInPosition(activeEncoderRail);
        bool motorReady = isMotorReady(activeEncoderRail);
        
        // Get commanded position (only position available from stepper controller)
        double commandedPos = pulsesToMm(motor.PositionRefCommanded(), activeEncoderRail);
        
        if (motorMoving) {
            // Motor is actively moving to commanded position
            sprintf_P(msg, FMT_POSITION_MOVING, commandedPos);
            Console.serialInfo(msg);
        } else if (motorInPosition && motorReady) {
            // Motor has completed movement and HLFB indicates it's settled at position
            sprintf_P(msg, FMT_POSITION_SETTLED, commandedPos);
            Console.serialInfo(msg);
        } else if (!motorReady) {
            // Motor has issues (alerts, not enabled, etc.)
            sprintf_P(msg, FMT_POSITION_NOT_READY, commandedPos);
            Console.serialWarning(msg);
        } else {
            // Steps complete but HLFB not asserted - may still be settling
            sprintf_P(msg, FMT_POSITION_SETTLING, commandedPos);
            Console.serialWarning(msg);
        }
    }
    
    sprintf_P(msg, FMT_MULTIPLIER_STATUS, getMultiplierName(currentMultiplierScaled));
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_VELOCITY_RPM, currentVelocityRpm);
    Console.serialInfo(msg);
    
    // Encoder hardware status
    sprintf_P(msg, FMT_ENCODER_POSITION, EncoderIn.Position());
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_ENCODER_VELOCITY, EncoderIn.Velocity());
    Console.serialInfo(msg);
    
    // MPG tracking information (if encoder control is active)
    if (encoderControlActive) {
        int32_t encoderDelta = EncoderIn.Position() - mpgBaseEncoderCount;
        double expectedPositionMm = scaledToMm(mpgBasePositionScaled + (encoderDelta * currentMultiplierScaled));
        sprintf_P(msg, FMT_MPG_BASE_POSITION, scaledToMm(mpgBasePositionScaled), mpgBaseEncoderCount);
        Console.serialInfo(msg);
        sprintf_P(msg, FMT_MPG_EXPECTED_POSITION, expectedPositionMm, encoderDelta);
        Console.serialInfo(msg);
    }
    
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
    
    if (encoderControlActive && activeEncoderRail > 0) {
        mpgBasePositionScaled = mmToScaled(getMotorPositionMm(activeEncoderRail)); // Reset to current motor position
    } else {
        mpgBasePositionScaled = 0;
    }
    
    Console.serialInfo(F("Quadrature error cleared"));
}