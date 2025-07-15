#include "EncoderController.h"
#include "Utils.h"
#include "OutputManager.h"

//=============================================================================
// CONSOLE OUTPUT FORMAT STRINGS
//=============================================================================
// Consolidated format strings for cleaner, more concise encoder control messaging

// Configuration and control format strings
const char FMT_MPG_CONFIG_CHANGE[] PROGMEM = "MPG: %s";
const char FMT_MPG_STATUS_RAIL[] PROGMEM = "MPG Rail %d: %s @ %.2fmm";
const char FMT_MPG_SETTINGS[] PROGMEM = "Settings: %s, %dRPM";

// Error and validation format strings
const char FMT_RAIL_VALIDATION_ERROR[] PROGMEM = "Rail %d: %s";

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
const char FMT_DYNAMIC_VELOCITY[] PROGMEM = "Dynamic velocity: %.1fx (encoder: %ldcps)";
const char FMT_TIMEOUT_REMAINING[] PROGMEM = "Timeout safety: %lu seconds remaining";

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

// Dynamic velocity and timeout safety tracking
unsigned long lastEncoderActivity = 0;         // Last time encoder moved (for timeout)
unsigned long lastTimeoutCheck = 0;            // Last time we checked for timeout  
float currentVelocityScale = 1.0;              // Current velocity scaling factor
int32_t smoothedEncoderVelocity = 0;           // Smoothed encoder velocity for stable scaling

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================



const char *getMultiplierName(int16_t multiplierScaled)
{
    if (multiplierScaled == MULTIPLIER_X1_SCALED) return "1x (0.1mm/count)";
    if (multiplierScaled == MULTIPLIER_X10_SCALED) return "10x (1.0mm/count)";
    if (multiplierScaled == MULTIPLIER_X100_SCALED) return "100x (10mm/count)";
    
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
    
    // Initialize dynamic velocity and timeout tracking
    lastEncoderActivity = millis();
    lastTimeoutCheck = millis();
    currentVelocityScale = 1.0;
    smoothedEncoderVelocity = 0;
    
    Console.serialInfo(F("Manual Pulse Generator (MPG) initialized - use 'encoder,enable,<rail>' to start"));
}

//=============================================================================
// CONTROL FUNCTIONS (Following Teknic Approach)
//=============================================================================

void enableEncoderControl(int rail)
{
    // Validate rail number
    if (rail != 1 && rail != 2) {
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_VALIDATION_ERROR, rail, "invalid rail number");
        Console.serialError(msg);
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
        sprintf_P(msg, FMT_RAIL_VALIDATION_ERROR, rail, "not homed");
        Console.serialError(msg);
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
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_VALIDATION_ERROR, rail, "cannot enable MPG during automated operation");
        Console.serialError(msg);
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
    
    // Initialize dynamic velocity and timeout tracking for this session
    lastEncoderActivity = millis();
    lastTimeoutCheck = millis();
    currentVelocityScale = 1.0;
    smoothedEncoderVelocity = 0;
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_MPG_STATUS_RAIL, rail, "ENABLED", scaledToMm(mpgBasePositionScaled));
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_MPG_SETTINGS, getMultiplierName(currentMultiplierScaled), currentVelocityRpm);
    Console.serialInfo(msg);
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
    
    // Check if motor is still ready - do this once to avoid race conditions
    MotorState currentState = updateMotorState(activeEncoderRail);
    bool motorMoving = isMotorMoving(activeEncoderRail);
    
    if (currentState == MOTOR_STATE_FAULTED || motorMoving)
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
    
    // Check for potential overflow before multiplication
    if (abs(totalEncoderDelta) > (INT32_MAX / abs(currentMultiplierScaled)))
    {
        Console.serialError(F("Encoder movement too large - resetting MPG base position"));
        mpgBasePositionScaled = mmToScaled(getMotorPositionMm(activeEncoderRail));
        mpgBaseEncoderCount = currentEncoderPosition;
        lastEncoderPosition = currentEncoderPosition;
        return;
    }
    
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
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, FMT_AT_POSITIVE_LIMIT, scaledToMm(maxTravelScaled));
            Console.serialWarning(msg);
            lastPositiveWarning = currentTime;
        }
    }
    
    // Get motor reference for this rail
    MotorDriver& motor = getMotorByRail(activeEncoderRail);
    
    // Current time for timing calculations
    unsigned long currentTime = millis();
    
    // **TIMEOUT SAFETY CHECK**
    // Check for encoder timeout periodically to avoid constant checking
    if (waitTimeReached(currentTime, lastTimeoutCheck, ENCODER_ACTIVITY_CHECK_INTERVAL_MS))
    {
        if (waitTimeReached(currentTime, lastEncoderActivity, ENCODER_TIMEOUT_MS))
        {
            Console.serialWarning(F("Encoder timeout (5min) - disabling MPG control for safety"));
            disableEncoderControl();
            return;
        }
        lastTimeoutCheck = currentTime;
    }
    
    // **DYNAMIC VELOCITY ADJUSTMENT**
    // Calculate current encoder velocity (counts per second) with smoothing to prevent jerkiness
    int32_t timeDeltaMs = currentTime - lastEncoderUpdateTime;
    if (timeDeltaMs > 0) 
    {
        // Calculate instantaneous encoder velocity
        int32_t instantVelocityCps = (abs(encoderDelta) * 1000) / timeDeltaMs;
        
        // Apply exponential smoothing to prevent jerky response
        // smoothedVelocity = (old * (factor-1) + new) / factor
        smoothedEncoderVelocity = ((smoothedEncoderVelocity * (ENCODER_VELOCITY_SMOOTHING_FACTOR - 1)) + instantVelocityCps) / ENCODER_VELOCITY_SMOOTHING_FACTOR;
        
        // Calculate velocity scale based on smoothed encoder speed
        if (smoothedEncoderVelocity <= ENCODER_VELOCITY_THRESHOLD_CPS) {
            // Slow encoder movement = precise control (reduced velocity)
            currentVelocityScale = ENCODER_MIN_VELOCITY_SCALE + 
                ((float)smoothedEncoderVelocity / ENCODER_VELOCITY_THRESHOLD_CPS) * (1.0 - ENCODER_MIN_VELOCITY_SCALE);
        } else {
            // Fast encoder movement = rapid positioning (increased velocity)
            float excessSpeed = smoothedEncoderVelocity - ENCODER_VELOCITY_THRESHOLD_CPS;
            float scaleRange = ENCODER_MAX_VELOCITY_SCALE - 1.0;
            currentVelocityScale = 1.0 + (excessSpeed / 10.0) * scaleRange; // Scale gradually
            
            // Clamp to maximum
            if (currentVelocityScale > ENCODER_MAX_VELOCITY_SCALE) {
                currentVelocityScale = ENCODER_MAX_VELOCITY_SCALE;
            }
        }
    }
    
    // Calculate base velocity and apply dynamic scaling
    int32_t baseVelocityPps = rpmToPps(currentVelocityRpm, activeEncoderRail);
    int32_t velocityPps = (int32_t)(baseVelocityPps * currentVelocityScale);
    
    // Apply the existing 50% boost for MPG responsiveness
    velocityPps = velocityPps + (velocityPps >> 1);
    
    // Ensure velocity stays within hardware bounds
    if (velocityPps < 500) velocityPps = 500;       // Higher minimum for responsiveness
    if (velocityPps > 12000) velocityPps = 12000;   // Hardware maximum limit
    
    // Update activity timestamp when encoder moves
    if (encoderDelta != 0) {
        lastEncoderActivity = currentTime;
    }
    
    // Set velocity for smooth movement
    motor.VelMax(velocityPps);
    
    // Use absolute positioning for immediate response (like real MPG systems)
    // Convert scaled position to mm for motor control (only one conversion needed)
    double targetPositionMm = scaledToMm(targetPositionScaled);
    int32_t targetPulses = mmToPulses(targetPositionMm, activeEncoderRail);
    motor.Move(targetPulses, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Log the movement with velocity information (every 50ms for debugging)
    if (waitTimeReached(currentTime, lastEncoderUpdateTime, 50))
    {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_MPG_RAIL_MOVEMENT, 
                activeEncoderRail, totalEncoderDelta, targetPositionMm, getMultiplierName(currentMultiplierScaled));
        Console.serialDiagnostic(msg);
        
        // Additional diagnostic: show dynamic velocity scaling when it's significantly different from 1.0
        if (fabs(currentVelocityScale - 1.0) > 0.2) {
            char velMsg[SMALL_MSG_SIZE];
            sprintf_P(velMsg, FMT_DYNAMIC_VELOCITY, currentVelocityScale, smoothedEncoderVelocity);
            Console.serialDiagnostic(velMsg);
        }
        
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
    // Validate multiplier against standard MPG values (1x, 10x, 100x)
    if (fabs(multiplier - 1.0f) < 0.01f)
    {
        currentMultiplierScaled = MULTIPLIER_X1_SCALED;  // 1x = 0.1mm per count (fine)
    }
    else if (fabs(multiplier - 10.0f) < 0.01f)
    {
        currentMultiplierScaled = MULTIPLIER_X10_SCALED; // 10x = 1.0mm per count (general)
    }
    else if (fabs(multiplier - 100.0f) < 0.01f)
    {
        currentMultiplierScaled = MULTIPLIER_X100_SCALED; // 100x = 10.0mm per count (rapid)
    }
    else
    {
        Console.serialError(F("Invalid multiplier. Use 1, 10, or 100"));
        return;
    }
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_MPG_CONFIG_CHANGE, getMultiplierName(currentMultiplierScaled));
    Console.serialInfo(msg);
}

void setEncoderVelocity(int velocityRpm)
{
    if (velocityRpm < ENCODER_MIN_VELOCITY_RPM || velocityRpm > ENCODER_MAX_VELOCITY_RPM)
    {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_VELOCITY_RANGE, 
                ENCODER_MIN_VELOCITY_RPM, ENCODER_MAX_VELOCITY_RPM);
        Console.serialError(msg);
        return;
    }
    
    currentVelocityRpm = velocityRpm;
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_MPG_CONFIG_CHANGE, "velocity updated");
    Console.serialInfo(msg);
}

//=============================================================================
// STATUS AND DIAGNOSTICS
//=============================================================================

void printEncoderStatus()
{
    char msg[ALERT_MSG_SIZE];
    
    if (!encoderControlActive)
    {
        sprintf_P(msg, FMT_MPG_STATUS_RAIL, 0, "DISABLED", 0.0);
        Console.serialInfo(msg);
    }
    else
    {
        // Get motor position and state efficiently
        MotorDriver& motor = getMotorByRail(activeEncoderRail);
        double commandedPos = pulsesToMm(motor.PositionRefCommanded(), activeEncoderRail);
        
        // Determine motor status concisely
        const char* motorStatus = "SETTLED";
        if (isMotorMoving(activeEncoderRail)) {
            motorStatus = "MOVING";
        } else if (!isMotorReady(activeEncoderRail)) {
            motorStatus = "NOT READY";
        } else if (!isMotorInPosition(activeEncoderRail)) {
            motorStatus = "SETTLING";
        }
        
        sprintf_P(msg, FMT_MPG_STATUS_RAIL, activeEncoderRail, motorStatus, commandedPos);
        Console.serialInfo(msg);
    }
    
    // Consolidated settings display with dynamic velocity info
    char settingsMsg[ALERT_MSG_SIZE];
    if (encoderControlActive && fabs(currentVelocityScale - 1.0) > 0.1) {
        sprintf_P(settingsMsg, "Settings: %s, %dRPM (Dynamic: %.1fx)", 
                getMultiplierName(currentMultiplierScaled), currentVelocityRpm, currentVelocityScale);
    } else {
        sprintf_P(msg, FMT_MPG_SETTINGS, getMultiplierName(currentMultiplierScaled), currentVelocityRpm);
        strcpy(settingsMsg, msg);
    }
    Console.serialInfo(settingsMsg);
    
    // Essential encoder hardware status
    sprintf_P(msg, FMT_ENCODER_POSITION, EncoderIn.Position());
    Console.serialInfo(msg);
    
    // Timeout safety status (only show if encoder is active)
    if (encoderControlActive) {
        unsigned long timeSinceActivity = millis() - lastEncoderActivity;
        unsigned long remainingTimeout = (timeSinceActivity < ENCODER_TIMEOUT_MS) ? 
            (ENCODER_TIMEOUT_MS - timeSinceActivity) / 1000 : 0;
        sprintf_P(msg, FMT_TIMEOUT_REMAINING, remainingTimeout);
        Console.serialInfo(msg);
    }
    
    // Error status if applicable
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

//=============================================================================
// TIMEOUT MANAGEMENT FUNCTIONS
//=============================================================================
void resetEncoderTimeouts()
{
    unsigned long currentTime = millis();
    lastEncoderActivity = currentTime;
    lastTimeoutCheck = currentTime;
    lastEncoderUpdateTime = currentTime;
    
    Console.serialInfo(F("Encoder timeout tracking reset - fresh timeout window started"));
}