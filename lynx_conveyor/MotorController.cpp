#include "MotorController.h"
#include "Utils.h"
#include "PositionConfig.h"

//=============================================================================
// PHASE 1 INTEGER MATH OPTIMIZATIONS
//=============================================================================
// This file has been optimized to use integer math for high-frequency operations
// while maintaining full API compatibility. Key optimizations:
//
// 1. Unit Conversions: mmToPulses() and pulsesToMm() now use integer math internally
//    with 0.01mm precision (MM_SCALE_FACTOR = 100)
//
// 2. Distance Calculations: All move functions use integer math for distance
//    calculations to eliminate floating-point operations in critical paths
//
// 3. Position Limit Checking: Range validation uses integer comparisons
//
// 4. Velocity Selection: Distance-based velocity scaling uses integer thresholds
//    and centralized getVelocityForDistance_i() function
//
// 5. Deceleration Logic: Distance-to-target calculations use integer math
//    in checkMoveProgress() to reduce floating-point load during motion
//
// All user-facing APIs remain unchanged - functions still accept/return doubles
// for backward compatibility and ease of use.
//=============================================================================

// ----------------- Global Variables -----------------
bool motorInitialized = false;
int32_t currentVelMax = 0;
int32_t currentAccelMax = 0;
bool isHomed = false;
double currentPositionMm = 0.0;
MotorState motorState = MOTOR_STATE_NOT_READY;
PositionTarget currentPosition = POSITION_1;
bool homingInProgress = false;
bool homingEncoderState = false;
unsigned long homingStartTime = 0;
double currentJogIncrementMm = DEFAULT_JOG_INCREMENT;
int currentJogSpeedRpm = DEFAULT_JOG_SPEED;
bool cycleFasterHomingInProgress = false;
unsigned long enableToggleStartTime = 0;
bool motorWasDisabled = false;
bool motorEnableCycleInProgress = false;
unsigned long enableCycleStartTime = 0;
bool motorDisablePhaseComplete = false;

// Target tracking for logging
bool hasCurrentTarget = false;
bool hasLastTarget = false;
PositionTarget currentTargetType = POSITION_UNDEFINED;
PositionTarget lastTargetType = POSITION_UNDEFINED;
double currentTargetPositionMm = 0.0;
double lastTargetPositionMm = 0.0;
int32_t currentTargetPulses = 0;
int32_t lastTargetPulses = 0;

// Fault clearing state variables
FaultClearingState faultClearState = FAULT_CLEAR_IDLE;
unsigned long faultClearTimer = 0;
bool faultClearInProgress = false;

// Initialize the deceleration configuration with default values
DecelerationConfig motorDecelConfig = {
    DEFAULT_DECELERATION_DISTANCE_MM, // Start decelerating when 35mm from target
    DEFAULT_MIN_VELOCITY_RPM,         // Minimum velocity of 5 RPM
    DEFAULT_DECELERATION_ENABLED      // Enabled by default
};

// Homing-specific state variables
static bool homing_hlfbWentNonAsserted = false;
static unsigned long homing_hlfbNonAssertedTime = 0;
static bool homing_minDistanceTraveled = false;
static int32_t homing_startPulses = 0;                // To store position at the start of a homing move
static int32_t lastCheckedPosition = 0;               // Added - position tracking for homing
static unsigned long lastPositionCheckTime = 0;       // Added - time tracking for homing
static unsigned long minTimeAfterDistanceReached = 0; // Added - time tracking for minimum distance
static int32_t pulsesTraveledAfterMinDistance = 0;    // Track additional movement
static int32_t positionAtMinDistance = 0;

// ----------------- Utility Functions -----------------

// Convert RPM to Pulses Per Second
int32_t rpmToPps(double rpm)
{
    return (int32_t)((rpm * PULSES_PER_REV) / 60.0);
}

// Convert Pulses Per Second to RPM
double ppsToRpm(int32_t pps)
{
    return (double)pps * 60.0 / PULSES_PER_REV;
}

// Convert RPM/s to Pulses/s^2
int32_t rpmPerSecToPpsPerSec(double rpmPerSec)
{
    return (int32_t)((rpmPerSec * PULSES_PER_REV) / 60.0);
}

// Convert mm to pulses with direction control (optimized with integer math)
int32_t mmToPulses(double mm)
{
    // Convert to 0.01mm units and use integer math for precision
    int32_t mm_cm = (int32_t)(mm * MM_SCALE_FACTOR + 0.5); // Round to nearest 0.01mm
    return mmToPulses_i(mm_cm);
}

// Convert pulses to mm (optimized with integer math)
double pulsesToMm(int32_t pulses)
{
    // Use integer math then convert back to double for API compatibility
    int32_t mm_cm = pulsesToMm_i(pulses);
    return (double)mm_cm / MM_SCALE_FACTOR;
}
// Normalize encoder values for display
int32_t normalizeEncoderValue(int32_t rawValue)
{
    // Apply the same direction multiplier as used in movement calculations
    return rawValue * MOTION_DIRECTION;
}

// ----------------- Integer Math Optimized Unit Conversions (Phase 1) -----------------

// Convert 0.01mm units to pulses using integer math
int32_t mmToPulses_i(int32_t mm_cm)
{
    // mm_cm is in 0.01mm units (e.g., 1215 for 12.15mm)
    // Convert: mm_cm -> mm -> pulses
    // Formula: pulses = (mm_cm / 100) * PULSES_PER_MM
    // Integer math: pulses = (mm_cm * (PULSES_PER_MM * 1000)) / 100000
    // Example: 1215 * 14810 / 100000 = 179 pulses (for 12.15mm)
    int32_t pulsesPerMm1000 = (int32_t)(PULSES_PER_MM * 1000 + 0.5); // ~14810
    return (mm_cm * pulsesPerMm1000 / 100000) * MOTION_DIRECTION;
}

// Convert pulses to 0.01mm units using integer math  
int32_t pulsesToMm_i(int32_t pulses)
{
    // Convert pulses to 0.01mm units with direction control
    // Formula: mm_cm = (pulses / PULSES_PER_MM) * 100
    // Integer math: mm_cm = (pulses * 100000) / (PULSES_PER_MM * 1000)
    // Example: 179 * 100000 / 14810 = 1208 (≈12.08mm in 0.01mm units)
    int32_t pulsesPerMm1000 = (int32_t)(PULSES_PER_MM * 1000 + 0.5); // ~14810
    return (pulses * MOTION_DIRECTION * 100000 / pulsesPerMm1000);
}

// Calculate absolute distance between two positions in 0.01mm units
int32_t distanceAbs_i(int32_t pos1_cm, int32_t pos2_cm)
{
    // Return absolute difference using integer math
    int32_t diff = pos1_cm - pos2_cm;
    return (diff < 0) ? -diff : diff;
}

// Optimized velocity selection based on distance using integer math
int getVelocityForDistance_i(int32_t distance_cm, bool shuttleEmpty)
{
    if (shuttleEmpty)
    {
        return EMPTY_SHUTTLE_VELOCITY_RPM;
    }
    
    // Use integer comparison for distance thresholds
    if (distance_cm < VERY_SHORT_MOVE_THRESHOLD_CM_MM)
    {
        return VERY_SHORT_MOVE_VELOCITY_RPM;
    }
    else if (distance_cm < SHORT_MOVE_THRESHOLD_CM_MM)
    {
        return SHORT_MOVE_VELOCITY_RPM;
    }
    else if (distance_cm < MEDIUM_MOVE_THRESHOLD_CM_MM)
    {
        return MEDIUM_MOVE_VELOCITY_RPM;
    }
    else
    {
        return LOADED_SHUTTLE_VELOCITY_RPM;
    }
}
// ----------------- Basic Setup Functions -----------------

void initMotorSystem()
{
    Console.serialInfo(F("Initializing motor system..."));

    // Set up E-stop input pin with internal pull-up
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    if (isEStopActive())
    {
        Console.serialError(F("E-STOP ACTIVE! Please reset E-stop before continuing."));
    }
    else
    {
        Console.serialInfo(F("E-stop inactive, system ready."));
    }

    // Set the input clocking rate
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Configure motor connector for step and direction mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Set the motor's HLFB mode to bipolar PWM
    MOTOR_CONNECTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

    // Set the HLFB carrier frequency to 482 Hz
    MOTOR_CONNECTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Set velocity and acceleration limits using RPM values
    char msg[200];
    sprintf(msg, "Setting velocity limit to %d RPM", LOADED_SHUTTLE_VELOCITY_RPM);
    Console.serialInfo(msg);

    currentVelMax = rpmToPps(LOADED_SHUTTLE_VELOCITY_RPM); // CHANGED
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    // Set acceleration limit
    sprintf(msg, "Setting acceleration limit to %d RPM/s", MAX_ACCEL_RPM_PER_SEC);
    Console.serialInfo(msg);

    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);

    // Enable the motor
    MOTOR_CONNECTOR.EnableRequest(true);
    Console.serialInfo(F("Motor enable requested"));

    // Wait for HLFB to assert (up to 2 seconds)
    Console.serialInfo(F("Waiting for HLFB..."));
    unsigned long startTime = millis();
    bool ready = false;

    while (!ready && !timeoutElapsed(millis(), startTime, 2000))
    {
        if (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED)
        {
            ready = true;
        }
        else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
        {
            Console.serialError(F("Motor alert detected:"));
            printMotorAlerts();
            break;
        }
        delay(10);
    }

    if (ready)
    {
        Console.serialInfo(F("Motor initialized and ready"));
        motorInitialized = true;
        motorState = MOTOR_STATE_IDLE;
    }
    else
    {
        Console.serialError(F("Motor initialization timed out or failed"));
        sprintf(msg, "HLFB State: %s",
                MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT ASSERTED");
        Console.serialError(msg);
    }
}

// ----------------- Movement Functions -----------------

// For absolute positioning commands
bool moveToAbsolutePosition(int32_t position)
{
    char msg[200];

    // Check if position is within valid range based on MOTION_DIRECTION
    if (MOTION_DIRECTION > 0)
    {
        // For positive direction, valid range is 0 to MAX_TRAVEL_PULSES
        if (position < 0 || position > MAX_TRAVEL_PULSES)
        {
            sprintf(msg, "Requested position %ld pulses is outside valid range (0 to %ld pulses)",
                    position, MAX_TRAVEL_PULSES);
            Console.serialError(msg);
            return false;
        }
    }
    else
    {
        // For negative direction, valid range is -MAX_TRAVEL_PULSES to 0
        if (position > 0 || position < -MAX_TRAVEL_PULSES)
        {
            sprintf(msg, "Requested position %ld pulses is outside valid range (%ld to 0 pulses)",
                    position, -MAX_TRAVEL_PULSES);
            Console.serialError(msg);
            return false;
        }
    }

    // Check if motor has alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.serialError(F("Motor alert detected. Cannot move."));
        printMotorAlerts();
        return false;
    }

    sprintf(msg, "Moving to absolute position: %ld", position);
    Console.serialInfo(msg);

    // Command the absolute move
    MOTOR_CONNECTOR.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

    // Report initial status
    Console.serialInfo(F("Move commanded. Motor in motion..."));

    return true;
}

bool moveToPosition(PositionTarget position)
{
    char debugMsg[200];
    // Set current target for logging
    hasCurrentTarget = true;
    currentTargetType = position;

    // Save original velocity and deceleration state
    int32_t originalVelMax = currentVelMax;
    bool originalDecelEnabled = motorDecelConfig.enableDeceleration;
    double targetPositionMm = 0.0;
    int32_t targetPulses = 0;

    // Determine the target values based on position
    switch (position)
    {
    case POSITION_HOME:
        targetPositionMm = POSITION_HOME_MM;
        targetPulses = POSITION_HOME_PULSES;

        sprintf(debugMsg, "POSITION_HOME_MM=%.2f, mmToPulses()=%ld, normalized=%ld, MOTION_DIRECTION=%d",
                targetPositionMm, targetPulses, normalizeEncoderValue(targetPulses), MOTION_DIRECTION);
        Console.serialDiagnostic(debugMsg);
        break;

    case POSITION_1:
        targetPositionMm = getPosition1Mm(); // Instead of POSITION_1_MM
        targetPulses = mmToPulses(targetPositionMm);

        sprintf(debugMsg, "getPosition1Mm()=%.2f, mmToPulses()=%ld, normalized=%ld, MOTION_DIRECTION=%d",
                targetPositionMm, targetPulses, normalizeEncoderValue(targetPulses), MOTION_DIRECTION);
        Console.serialDiagnostic(debugMsg);
        break;
    case POSITION_2:
        targetPositionMm = getPosition2Mm(); // Instead of POSITION_2_MM
        targetPulses = mmToPulses(targetPositionMm);

        sprintf(debugMsg, "getPosition2Mm()=%.2f, mmToPulses()=%ld, normalized=%ld, MOTION_DIRECTION=%d",
                targetPositionMm, targetPulses, normalizeEncoderValue(targetPulses), MOTION_DIRECTION);
        Console.serialDiagnostic(debugMsg);
        break;
    case POSITION_3:
        targetPositionMm = getPosition3Mm(); // Instead of POSITION_3_MM
        targetPulses = mmToPulses(targetPositionMm);

        sprintf(debugMsg, "getPosition3Mm()=%.2f, mmToPulses()=%ld, normalized=%ld, MOTION_DIRECTION=%d",
                targetPositionMm, targetPulses, normalizeEncoderValue(targetPulses), MOTION_DIRECTION);
        Console.serialDiagnostic(debugMsg);
        break;
    case POSITION_4:
        targetPositionMm = POSITION_4_MM;
        targetPulses = POSITION_4_PULSES;

        sprintf(debugMsg, "POSITION_4_MM=%.2f, mmToPulses()=%ld, normalized=%ld, MOTION_DIRECTION=%d",
                targetPositionMm, targetPulses, normalizeEncoderValue(targetPulses), MOTION_DIRECTION);
        Console.serialDiagnostic(debugMsg);
        break;
    default:
        hasCurrentTarget = false;
        return false;
    }

    // Store target values for tracking
    currentTargetPositionMm = targetPositionMm;
    currentTargetPulses = targetPulses;

    // IMPORTANT: Update currentPositionMm with the latest position before calculating distance
    currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());

    // Debug print to show the actual positions being used
    char msg[200];
    sprintf(msg, "Current position: %.2fmm, Target position: %.2fmm",
            currentPositionMm, targetPositionMm);
    Console.serialDiagnostic(msg);

    // Calculate distance to move with optimized integer math
    int32_t currentPos_cm = (int32_t)(currentPositionMm * MM_SCALE_FACTOR + 0.5);
    int32_t targetPos_cm = (int32_t)(targetPositionMm * MM_SCALE_FACTOR + 0.5);
    int32_t distance_cm = distanceAbs_i(targetPos_cm, currentPos_cm);
    double distanceToMoveMm = (double)distance_cm / MM_SCALE_FACTOR;

    sprintf(msg, "Calculated move distance: %.2fmm", distanceToMoveMm);
    Console.serialDiagnostic(msg);

    // Apply velocity scaling based on move distance using optimized integer math
    // Check if shuttle is retracted (empty) - use higher speed
    SystemState currentState = captureSystemState();
    sprintf(msg, "Shuttle locked state: %s", currentState.shuttleLocked ? "TRUE (not empty)" : "FALSE (empty)");
    Console.serialDiagnostic(msg);

    // Use optimized velocity selection
    int selectedVelocityRPM = getVelocityForDistance_i(distance_cm, !currentState.shuttleLocked);
    currentVelMax = rpmToPps(selectedVelocityRPM);
    
    if (!currentState.shuttleLocked)
    {
        // Shuttle is empty - disable deceleration
        motorDecelConfig.enableDeceleration = false;
        sprintf(msg, "Empty shuttle detected - Using increased speed: %d RPM with deceleration disabled", selectedVelocityRPM);
        Console.serialInfo(msg);
    }
    else
    {
        // Make sure deceleration is enabled for loaded shuttle
        motorDecelConfig.enableDeceleration = true;
        
        // Log the selected velocity with appropriate message
        if (distance_cm < VERY_SHORT_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Very short move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else if (distance_cm < SHORT_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Short move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else if (distance_cm < MEDIUM_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Medium move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else
        {
            sprintf(msg, "Long move detected (%.2fmm) - Using full speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        Console.serialInfo(msg);
    }

    // Apply the velocity limit to the motor
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    // Perform the move
    bool moveResult = moveToAbsolutePosition(targetPulses);

    if (!moveResult)
    {
        // Restore original settings if move command failed
        currentVelMax = originalVelMax;
        MOTOR_CONNECTOR.VelMax(currentVelMax);
        motorDecelConfig.enableDeceleration = originalDecelEnabled;
        hasCurrentTarget = false;
    }

    return moveResult;
}

bool moveToPosition(int positionNumber)
{
    switch (positionNumber)
    {
    case 0: // "home" as position 0
        return moveToPosition(POSITION_HOME);
    case 1:
        return moveToPosition(POSITION_1);
    case 2:
        return moveToPosition(POSITION_2);
    case 3:
        return moveToPosition(POSITION_3);
    case 4:
        return moveToPosition(POSITION_4);
    default:
        return false;
    }
}

bool moveToPositionMm(double positionMm)
{
    char msg[200];

    // Safety check - prevent movement beyond physical limits (optimized with integer math)
    int32_t positionMm_cm = (int32_t)(positionMm * MM_SCALE_FACTOR + 0.5);
    if (positionMm_cm < 0 || positionMm_cm > MAX_TRAVEL_CM_MM)
    {
        sprintf(msg, "Requested position %.2f mm is outside valid range (0 to %.2f mm)", positionMm, MAX_TRAVEL_MM);
        Console.serialError(msg);
        return false;
    }

    // IMPORTANT: Update currentPositionMm with the latest position before calculating distance
    currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());

    // Debug print to show the actual positions being used
    sprintf(msg, "Current position: %.2fmm, Target position: %.2fmm", currentPositionMm, positionMm);
    Console.serialDiagnostic(msg);

    // Calculate distance to move using optimized integer math
    int32_t currentPos_cm = (int32_t)(currentPositionMm * MM_SCALE_FACTOR + 0.5);
    int32_t targetPos_cm = (int32_t)(positionMm * MM_SCALE_FACTOR + 0.5);
    int32_t distance_cm = distanceAbs_i(targetPos_cm, currentPos_cm);
    double distanceToMoveMm = (double)distance_cm / MM_SCALE_FACTOR;

    sprintf(msg, "Calculated move distance: %.2fmm", distanceToMoveMm);
    Console.serialDiagnostic(msg);

    // Save original velocity and deceleration state
    int32_t originalVelMax = currentVelMax;
    bool originalDecelEnabled = motorDecelConfig.enableDeceleration;

    // Apply velocity scaling based on move distance using optimized integer math
    // Check if shuttle is retracted (empty) - use higher speed
    SystemState currentState = captureSystemState();
    sprintf(msg, "Shuttle locked state: %s", currentState.shuttleLocked ? "TRUE (not empty)" : "FALSE (empty)");
    Console.serialDiagnostic(msg);

    // Use optimized velocity selection
    int selectedVelocityRPM = getVelocityForDistance_i(distance_cm, !currentState.shuttleLocked);
    currentVelMax = rpmToPps(selectedVelocityRPM);
    
    if (!currentState.shuttleLocked)
    {
        // Shuttle is empty - disable deceleration
        motorDecelConfig.enableDeceleration = false;
        sprintf(msg, "Empty shuttle detected - Using increased speed: %d RPM with deceleration disabled", selectedVelocityRPM);
        Console.serialInfo(msg);
    }
    else
    {
        // Make sure deceleration is enabled for loaded shuttle
        motorDecelConfig.enableDeceleration = true;
        
        // Log the selected velocity with appropriate message
        if (distance_cm < VERY_SHORT_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Very short move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else if (distance_cm < SHORT_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Short move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else if (distance_cm < MEDIUM_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Medium move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else
        {
            sprintf(msg, "Long move detected (%.2fmm) - Using full speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        Console.serialInfo(msg);
    }

    // Apply the new velocity limit to the motor
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    // Convert to pulses
    int32_t pulsePosition = mmToPulses(positionMm);

    // Set current target for logging
    hasCurrentTarget = true;
    currentTargetType = POSITION_CUSTOM;
    currentTargetPositionMm = positionMm;
    currentTargetPulses = pulsePosition;

    // Perform the move
    bool moveResult = moveToAbsolutePosition(pulsePosition);

    // Note: We DON'T immediately restore the original velocity here
    // The deceleration system will handle velocity during motion
    // And checkMoveProgress() will restore velocity when move completes

    if (!moveResult)
    {
        // Restore original velocity limit if move command failed
        currentVelMax = originalVelMax;
        MOTOR_CONNECTOR.VelMax(currentVelMax);

        // Also restore original deceleration setting if move failed
        motorDecelConfig.enableDeceleration = originalDecelEnabled;

        // Clear current target
        hasCurrentTarget = false;
    }

    return moveResult;
}

bool moveRelative(double relativeMm)
{
    // IMPORTANT: Update currentPositionMm with the latest position before calculating distance
    currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());

    // Calculate the target absolute position
    double targetPositionMm = currentPositionMm + relativeMm;

    // Debug print to show the actual positions being used
    char msg[200];
    sprintf(msg, "Current position: %.2fmm, Target position: %.2fmm", currentPositionMm, targetPositionMm);
    Console.serialDiagnostic(msg);

    // Check if the target position would be out of bounds (optimized with integer math)
    int32_t targetPos_cm = (int32_t)(targetPositionMm * MM_SCALE_FACTOR + (targetPositionMm >= 0 ? 0.5 : -0.5));
    if (targetPos_cm < 0 || targetPos_cm > MAX_TRAVEL_CM_MM)
    {
        sprintf(msg, "Relative move would exceed valid range (0 to %.2f mm)", MAX_TRAVEL_MM);
        Console.serialError(msg);
        sprintf(msg, "Current position: %.2f mm, Requested move: %.2f mm, Target would be: %.2f mm",
                currentPositionMm, relativeMm, targetPositionMm);
        Console.serialError(msg);
        return false;
    }

    // Calculate distance to move using optimized integer math (use absolute value for velocity scaling)
    int32_t relativeMm_cm = (int32_t)(relativeMm * MM_SCALE_FACTOR + (relativeMm >= 0 ? 0.5 : -0.5));
    int32_t distance_cm = (relativeMm_cm < 0) ? -relativeMm_cm : relativeMm_cm;
    double distanceToMoveMm = (double)distance_cm / MM_SCALE_FACTOR;

    sprintf(msg, "Calculated move distance: %.2fmm", distanceToMoveMm);
    Console.serialDiagnostic(msg);

    // Save original velocity and deceleration state
    int32_t originalVelMax = currentVelMax;
    bool originalDecelEnabled = motorDecelConfig.enableDeceleration;

    // Apply velocity scaling based on move distance using optimized integer math
    // Check if shuttle is retracted (empty) - use higher speed
    SystemState currentState = captureSystemState();
    sprintf(msg, "Shuttle locked state: %s", currentState.shuttleLocked ? "TRUE (not empty)" : "FALSE (empty)");
    Console.serialDiagnostic(msg);

    // Use optimized velocity selection
    int selectedVelocityRPM = getVelocityForDistance_i(distance_cm, !currentState.shuttleLocked);
    currentVelMax = rpmToPps(selectedVelocityRPM);
    
    if (!currentState.shuttleLocked)
    {
        // Shuttle is empty - disable deceleration
        motorDecelConfig.enableDeceleration = false;
        sprintf(msg, "Empty shuttle detected - Using increased speed: %d RPM with deceleration disabled", selectedVelocityRPM);
        Console.serialInfo(msg);
    }
    else
    {
        // Make sure deceleration is enabled for loaded shuttle
        motorDecelConfig.enableDeceleration = true;
        
        // Log the selected velocity with appropriate message
        if (distance_cm < VERY_SHORT_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Very short move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else if (distance_cm < SHORT_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Short move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else if (distance_cm < MEDIUM_MOVE_THRESHOLD_CM_MM)
        {
            sprintf(msg, "Medium move detected (%.2fmm) - Using reduced speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        else
        {
            sprintf(msg, "Long move detected (%.2fmm) - Using full speed: %d RPM", distanceToMoveMm, selectedVelocityRPM);
        }
        Console.serialInfo(msg);
    }

    // Apply the new velocity limit to the motor
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    // Convert millimeters to pulses
    int32_t relativePulses = mmToPulses(relativeMm);

    // Set target tracking for consistent deceleration behavior
    hasCurrentTarget = true;
    currentTargetType = POSITION_CUSTOM;
    currentTargetPositionMm = targetPositionMm;
    currentTargetPulses = mmToPulses(targetPositionMm);

    // Call the motor's relative move function
    MOTOR_CONNECTOR.Move(relativePulses, MotorDriver::MOVE_TARGET_REL_END_POSN);

    // Update state
    motorState = MOTOR_STATE_MOVING;
    currentPosition = POSITION_CUSTOM;

    sprintf(msg, "Moving %.2f mm from current position (%ld pulses)", relativeMm, normalizeEncoderValue(relativePulses));
    Console.serialInfo(msg);

    // Note: We don't need to restore deceleration setting here
    // checkMoveProgress() will handle this when the move completes

    return true;
}

double getMotorPositionMm()
{
    return pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
}

void stopMotion()
{
    // Stop the motor abruptly
    MOTOR_CONNECTOR.MoveStopAbrupt();
    Console.serialInfo(F("Motion stopped"));
}

// ----------------- Jogging Functions -----------------

bool jogMotor(bool direction, double customIncrement)
{
    char msg[200];

    // Save current speed setting
    int32_t originalVelMax = currentVelMax;

    // Determine increment to use
    double increment = (customIncrement > 0) ? customIncrement : currentJogIncrementMm;

    // Calculate movement direction (positive or negative)
    double moveMm = direction ? increment : -increment;

    // Get current position
    double currentPosMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());

    // Calculate target position and set target tracking variables
    double targetPositionMm = currentPosMm + moveMm;

    // Update target tracking variables
    hasCurrentTarget = true;
    currentTargetType = POSITION_CUSTOM;
    currentTargetPositionMm = targetPositionMm;
    currentTargetPulses = mmToPulses(targetPositionMm);

    // Use setJogSpeed to apply velocity scaling if using custom increment
    if (customIncrement > 0)
    {
        // This will automatically cap the speed based on the distance
        setJogSpeed(currentJogSpeedRpm, customIncrement);
    }

    // Now set the properly scaled velocity
    currentVelMax = rpmToPps(currentJogSpeedRpm);
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    // Log the jog operation
    sprintf(msg, "Jogging %s by %.2f mm at %d RPM", direction ? "forward" : "backward", increment, currentJogSpeedRpm);
    Console.serialInfo(msg);

    // Use the existing moveRelative function (which now handles distance-based scaling too)
    bool result = moveRelative(moveMm);

    // If movement failed, clear target tracking
    if (!result)
    {
        hasCurrentTarget = false;
    }

    // Reset to original speed after move is commanded
    currentVelMax = originalVelMax;
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    return result;
}

bool setJogIncrement(double increment)
{
    char msg[200];

    // Validate increment is reasonable
    if (increment <= 0 || increment > 100)
    {
        Console.serialError(F("Jog increment must be between 0 and 100mm"));
        return false;
    }

    // Set the increment
    currentJogIncrementMm = increment;
    sprintf(msg, "Jog increment set to %.2f mm", currentJogIncrementMm);
    Console.serialInfo(msg);

    // Re-validate jog speed with the new distance
    setJogSpeed(currentJogSpeedRpm, increment);

    return true;
}

bool setJogSpeed(int speedRpm, double jogDistanceMm)
{
    char msg[200];

    // Get the jog distance - either from parameter or use current increment
    double distanceToMoveMm = (jogDistanceMm > 0) ? jogDistanceMm : currentJogIncrementMm;

    // Validate speed is reasonable
    if (speedRpm < 10 || speedRpm > LOADED_SHUTTLE_VELOCITY_RPM)
    {
        sprintf(msg, "Jog speed must be between 10 and %d RPM", LOADED_SHUTTLE_VELOCITY_RPM);
        Console.serialError(msg);
        return false;
    }

    // Apply distance-based speed caps using optimized integer math
    int32_t distance_cm = (int32_t)(distanceToMoveMm * MM_SCALE_FACTOR + 0.5);
    int cappedSpeed = speedRpm;

    if (distance_cm < VERY_SHORT_MOVE_THRESHOLD_CM_MM)
    {
        // Cap speed for very short moves
        cappedSpeed = min(speedRpm, VERY_SHORT_MOVE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            sprintf(msg, "Speed capped to %d RPM for very short distance (%.2fmm)", cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }
    else if (distance_cm < SHORT_MOVE_THRESHOLD_CM_MM)
    {
        // Cap speed for short moves
        cappedSpeed = min(speedRpm, SHORT_MOVE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            sprintf(msg, "Speed capped to %d RPM for short distance (%.2fmm)", cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }
    else if (distance_cm < MEDIUM_MOVE_THRESHOLD_CM_MM)
    {
        // Cap speed for medium moves
        cappedSpeed = min(speedRpm, MEDIUM_MOVE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            sprintf(msg, "Speed capped to %d RPM for medium distance (%.2fmm)", cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }
    else
    {
        // For long moves, explicitly set to maximum velocity if requested speed is higher
        cappedSpeed = min(speedRpm, LOADED_SHUTTLE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            sprintf(msg, "Speed capped to %d RPM for long distance (%.2fmm)", cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }

    // Set the speed with the potentially capped value
    currentJogSpeedRpm = cappedSpeed;
    sprintf(msg, "Jog speed set to %d RPM", currentJogSpeedRpm);
    Console.serialInfo(msg);

    return true;
}

// ----------------- Status Functions -----------------

bool isMotorReady()
{
    return motorInitialized &&
           MOTOR_CONNECTOR.EnableRequest() &&
           MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED &&
           !MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent;
}

bool isMotorMoving()
{
    // Only consider step completion for motion status
    return !MOTOR_CONNECTOR.StepsComplete();
}

bool isMotorInPosition()
{
    return MOTOR_CONNECTOR.StepsComplete() &&
           MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED;
}

bool isMotorAtPosition()
{
    // This can either be an alias to isMotorInPosition() or implement its own logic
    return isMotorInPosition();
}

bool hasMotorFault()
{
    return MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent;
}

MotorState updateMotorState()
{
    // Check for faults first
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        motorState = MOTOR_STATE_FAULTED;
    }
    // Check if motor is not enabled
    else if (!MOTOR_CONNECTOR.EnableRequest())
    {
        motorState = MOTOR_STATE_NOT_READY;
    }
    // Check if homing is in progress
    else if (homingInProgress)
    {
        motorState = MOTOR_STATE_HOMING;
    }
    // Check if steps are complete (primary indicator of motion completion)
    else if (!MOTOR_CONNECTOR.StepsComplete())
    {
        motorState = MOTOR_STATE_MOVING;
    }
    // Otherwise, motor is idle
    else
    {
        // Even if HLFB isn't asserted yet, if steps are complete,
        // consider the motor idle for state management purposes
        motorState = MOTOR_STATE_IDLE;
    }

    return motorState;
}

void printMotorStatus()
{
    char messageBuffer[800]; // Single buffer for the complete status message

    // Determine HLFB status string
    const char *hlfbStatus;
    switch (MOTOR_CONNECTOR.HlfbState())
    {
    case MotorDriver::HLFB_ASSERTED:
        hlfbStatus = "Asserted (In Position/Ready)";
        break;
    case MotorDriver::HLFB_DEASSERTED:
        hlfbStatus = "Deasserted (Moving/Fault)";
        break;
    case MotorDriver::HLFB_UNKNOWN:
    default:
        hlfbStatus = "Unknown";
        break;
    }

    // Determine alert status
    const char *alertStatus;
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        alertStatus = "Alerts present (see alert details below)";
    }
    else
    {
        alertStatus = "No alerts";
    }

    // Calculate acceleration in RPM/s
    double accelRpmPerSec = (double)currentAccelMax * 60.0 / PULSES_PER_REV;

    // Build the complete status message
    sprintf(messageBuffer,
            "[INFO] Motor Status:\n"
            "  Enabled: %s\n"
            "  Moving: %s\n"
            "  Position: %ld pulses\n"
            "  Current Velocity Limit: %.1f RPM\n"
            "  Current Acceleration Limit: %.1f RPM/s\n"
            "  HLFB Status: %s\n"
            "  %s",
            MOTOR_CONNECTOR.EnableRequest() ? "Yes" : "No",
            isMotorAtPosition() ? "No" : "Yes",
            normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded()),
            ppsToRpm(currentVelMax),
            accelRpmPerSec,
            hlfbStatus,
            alertStatus);

    // Send the complete status message
    Console.print(messageBuffer);

    // If there are alerts, print them separately using the existing function
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        printMotorAlerts();
    }
}

void printMotorAlerts()
{
    char messageBuffer[400];  // Single buffer for alert messages
    char alertList[300] = ""; // Build list of active alerts

    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledInAlert)
    {
        strcat(alertList, "    MotionCanceledInAlert\n");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledPositiveLimit)
    {
        strcat(alertList, "    MotionCanceledPositiveLimit\n");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledNegativeLimit)
    {
        strcat(alertList, "    MotionCanceledNegativeLimit\n");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledSensorEStop)
    {
        strcat(alertList, "    MotionCanceledSensorEStop\n");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledMotorDisabled)
    {
        strcat(alertList, "    MotionCanceledMotorDisabled\n");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted)
    {
        strcat(alertList, "    MotorFaulted\n");
    }

    // Remove the trailing newline if present
    int len = strlen(alertList);
    if (len > 0 && alertList[len - 1] == '\n')
    {
        alertList[len - 1] = '\0';
    }

    sprintf(messageBuffer, "  Alert Details:\n%s", alertList);
    Console.serialError(messageBuffer);
}

void clearMotorFaults()
{
    // If we're not already in the process of clearing faults, start the process
    if (!faultClearInProgress)
    {
        Console.serialDiagnostic(F("Attempting to clear motor faults..."));

        // First check if there are faults present
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
        {
            Console.serialDiagnostic(F("Alerts detected:"));
            printMotorAlerts();

            // Start the fault clearing state machine
            faultClearState = FAULT_CLEAR_DISABLE;
            faultClearTimer = millis();
            faultClearInProgress = true;
        }
        else
        {
            Console.serialInfo(F("No alerts to clear."));
        }
    }
}

void processFaultClearing()
{
    // If we're not clearing faults, just return
    if (!faultClearInProgress)
    {
        return;
    }

    unsigned long currentTime = millis();

    switch (faultClearState)
    {
    case FAULT_CLEAR_DISABLE:
        // Disable the motor
        if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted)
        {
            Console.serialDiagnostic(F("Motor faulted. Cycling enable signal..."));
            MOTOR_CONNECTOR.EnableRequest(false);
        }
        faultClearTimer = currentTime;
        faultClearState = FAULT_CLEAR_WAITING_DISABLE;
        break;

    case FAULT_CLEAR_WAITING_DISABLE:
        // Wait 100ms after disabling
        if (timeoutElapsed(currentTime, faultClearTimer, 100)) // CHANGED: using timeoutElapsed
        {
            faultClearState = FAULT_CLEAR_ENABLE;
        }
        break;

    case FAULT_CLEAR_ENABLE:
        // Re-enable the motor
        MOTOR_CONNECTOR.EnableRequest(true);
        faultClearTimer = currentTime;
        faultClearState = FAULT_CLEAR_WAITING_ENABLE;
        break;

    case FAULT_CLEAR_WAITING_ENABLE:
        // Wait 100ms after enabling
        if (timeoutElapsed(currentTime, faultClearTimer, 100)) // CHANGED: using timeoutElapsed
        {
            faultClearState = FAULT_CLEAR_ALERTS;
        }
        break;

    case FAULT_CLEAR_ALERTS:
        // Clear alerts
        Console.serialDiagnostic(F("Clearing motor alerts..."));
        MOTOR_CONNECTOR.ClearAlerts();

        // Check if alerts were cleared
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
        {
            Console.serialError(F("Alerts are still present after clearing."));
            printMotorAlerts();
        }
        else
        {
            Console.serialInfo(F("Alerts successfully cleared."));
        }

        faultClearState = FAULT_CLEAR_FINISHED;
        break;

    case FAULT_CLEAR_FINISHED:
        // Reset state for next time
        faultClearState = FAULT_CLEAR_IDLE;
        faultClearInProgress = false;
        break;

    default:
        faultClearState = FAULT_CLEAR_IDLE;
        faultClearInProgress = false;
        break;
    }
}

bool isFaultClearingInProgress()
{
    return faultClearInProgress;
}

bool clearMotorFaultWithStatus()
{
    // If fault clearing is already in progress, return false
    if (faultClearInProgress)
    {
        Console.serialInfo(F("Fault clearing already in progress"));
        return false;
    }

    bool hadAlerts = MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent;

    // Start fault clearing process
    clearMotorFaults();

    // Return true if there were no alerts to clear (immediate success)
    // Return false if clearing process has started (delayed result)
    return !hadAlerts;
}

// ----------------- Homing Functions -----------------

bool initiateHomingSequence()
{
    // Check if motor is initialized
    if (!motorInitialized)
    {
        Console.serialError(F("Motor not initialized"));
        return false;
    }

    // Check for faults
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.serialError(F("Motor has active alerts - clear faults before homing"));
        return false;
    }

    // Store encoder state before disabling for homing
    homingEncoderState = encoderControlActive;

    // Disable encoder control if active
    if (encoderControlActive)
    {
        encoderControlActive = false;
        Console.serialInfo(F("MPG handwheel control disabled during homing"));
    }

    // Reset homing state first - this clears variables and captures starting position
    resetHomingState();

    // For "Upon every Enable" configuration, cycle the enable signal
    MOTOR_CONNECTOR.EnableRequest(false);
    delay(200);
    MOTOR_CONNECTOR.EnableRequest(true);
    delay(200);

    // Set velocity for homing
    int32_t homingVelPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
    MOTOR_CONNECTOR.VelMax(homingVelPps);

    // Move in homing direction at continuous velocity
    MOTOR_CONNECTOR.MoveVelocity(HOMING_DIRECTION * homingVelPps);

    homingInProgress = true; // Set this after successfully starting the move
    motorState = MOTOR_STATE_HOMING;
    homingStartTime = millis();

    Console.serialInfo(F("Homing sequence initiated. Motor will move to find home position."));
    return true;
}

void checkHomingProgress()
{
    char msg[200];

    if (!homingInProgress)
        return;

    unsigned long currentTime = millis();

    // Add a delay before starting actual hardstop detection
    static const unsigned long homingStartDelay = 500; // 500ms delay
    if (!timeoutElapsed(currentTime, homingStartTime, homingStartDelay))
    {
        return; // Don't process hardstop detection until initial delay is complete
    }

    static const int32_t minimumMovementPulses = 200;   // Minimum movement before detecting hardstop
    static const int32_t minimumAdditionalPulses = 100; // Must travel at least this much AFTER min distance

    MotorDriver::HlfbStates currentHlfbState = MOTOR_CONNECTOR.HlfbState();
    int32_t currentPosition = MOTOR_CONNECTOR.PositionRefCommanded();

    // Check if motor is actually moving during homing (every 100ms)
    if (timeoutElapsed(currentTime, lastPositionCheckTime, 100))
    {
        // Calculate movement since last check
        int32_t movementSinceLastCheck = abs(currentPosition - lastCheckedPosition);

        // Log movement data when in detail when we've crossed the minimum distance threshold
        if (homing_minDistanceTraveled)
        {
            sprintf(msg, "[HOMING] Position: %ld, Movement: %ld pulses, HLFB: %s",
                    currentPosition, movementSinceLastCheck,
                    currentHlfbState == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT_ASSERTED");
            Console.serialDiagnostic(msg);
        }

        if (movementSinceLastCheck < 10 && homing_hlfbWentNonAsserted)
        {
            // Motor isn't moving but HLFB has changed - possible false trigger
            Console.warning(F("Minimal movement detected during homing"));
        }

        lastCheckedPosition = currentPosition;
        lastPositionCheckTime = currentTime;
    }

    // Check for timeout first
    if (timeoutElapsed(currentTime, homingStartTime, 30000))
    { // 30 seconds timeout
        Console.serialError(F("Homing operation timed out"));
        sprintf(msg, "Final HLFB state: %s",
                currentHlfbState == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT ASSERTED");
        Console.serialDiagnostic(msg);

        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Set current position as home despite timeout
        Console.serialInfo(F("Setting current position as home reference despite timeout"));
        MOTOR_CONNECTOR.PositionRefSet(0);

        // Complete homing setup
        completeHomingSequence();
        return;
    }

    // Check for alerts during homing
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.serialError(F("Motor alert during homing"));
        printMotorAlerts();

        // Stop motion and abort homing
        MOTOR_CONNECTOR.MoveStopAbrupt();
        abortHoming(); // abortHoming should call the updated resetHomingState
        return;
    }

    // Calculate pulses moved during THIS specific homing attempt
    int32_t pulsesMovedThisHoming = abs(currentPosition - homing_startPulses);

    // Check if we've moved enough distance to consider hardstop detection
    if (pulsesMovedThisHoming >= minimumMovementPulses && !homing_minDistanceTraveled)
    {
        homing_minDistanceTraveled = true;
        minTimeAfterDistanceReached = currentTime; // Start the minimum time timer
        positionAtMinDistance = currentPosition;   // Remember position at minimum distance
        sprintf(msg, "Minimum travel distance reached (%ld pulses) - Hardstop detection enabled", pulsesMovedThisHoming);
        Console.serialInfo(msg);
    }

    // After min distance reached, track additional travel
    if (homing_minDistanceTraveled)
    {
        pulsesTraveledAfterMinDistance = abs(currentPosition - positionAtMinDistance);
    }

    // Check for non-asserted state (HLFB indicates movement or fault)
    if (currentHlfbState != MotorDriver::HLFB_ASSERTED)
    {
        if (!homing_hlfbWentNonAsserted)
        {
            homing_hlfbWentNonAsserted = true;
            homing_hlfbNonAssertedTime = currentTime;
            Console.serialInfo(F("HLFB went non-asserted - approaching hardstop"));
        }
    }

    // Add additional constraints for hardstop detection:
    // 1. HLFB previously went non-asserted
    // 2. HLFB is now asserted
    // 3. Debounce time for HLFB has passed (100ms)
    // 4. We've traveled minimum distance
    // 5. At least 300ms have passed since minimum distance was reached (increased from 200ms)
    // 6. We've traveled at least 500 additional pulses after minimum distance
    if (homing_hlfbWentNonAsserted &&
        currentHlfbState == MotorDriver::HLFB_ASSERTED &&
        timeoutElapsed(currentTime, homing_hlfbNonAssertedTime, 250) && // Debounce time check
        homing_minDistanceTraveled &&
        timeoutElapsed(currentTime, minTimeAfterDistanceReached, 500) && // Min time after distance
        pulsesTraveledAfterMinDistance >= minimumAdditionalPulses)
    {

        sprintf(msg, "Hardstop reached - HLFB reasserted after %ldms from minimum distance, additional travel: %ld pulses",
                timeDiff(currentTime, minTimeAfterDistanceReached), pulsesTraveledAfterMinDistance);
        Console.serialInfo(msg);

        // Stop the velocity move
        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Set position to zero at the actual hardstop before offset
        MOTOR_CONNECTOR.PositionRefSet(0);

        // Move away from hardstop to complete homing
        if (HOME_OFFSET_DISTANCE_MM > 0)
        {
            sprintf(msg, "Moving %.2fmm away from hardstop", HOME_OFFSET_DISTANCE_MM);
            Console.serialInfo(msg);

            // Reset velocity to normal (or a specific offset velocity if desired)
            int32_t normalVelPps = rpmToPps(LOADED_SHUTTLE_VELOCITY_RPM); // Or a slower offset speed
            MOTOR_CONNECTOR.VelMax(normalVelPps);

            // Move away from hardstop
            int32_t offsetPulses = mmToPulses(HOME_OFFSET_DISTANCE_MM);
            // The direction of offset should be opposite to HOMING_DIRECTION
            MOTOR_CONNECTOR.Move(-HOMING_DIRECTION * offsetPulses, MotorDriver::MOVE_TARGET_REL_END_POSN);

            // Wait for offset move to complete
            Console.serialInfo(F("Waiting for offset move to complete..."));
            unsigned long offsetMoveStartTime = millis(); // Use a fresh start time for this wait
            while (!MOTOR_CONNECTOR.StepsComplete() &&
                   !timeoutElapsed(millis(), offsetMoveStartTime, 5000))
            { // Timeout for offset move
                delay(10);

                // Check for alerts during offset move
                if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
                {
                    Console.serialError(F("Alert during offset move"));
                    abortHoming(); // abortHoming should call the updated resetHomingState
                    return;
                }
            }
            if (!MOTOR_CONNECTOR.StepsComplete())
            {
                Console.serialError(F("Offset move timed out or failed to complete."));
                // Decide how to handle this: abort or try to complete homing anyway?
                // For now, we'll proceed to set home, but this is a potential issue.
            }

            // Re-zero at offset position
            MOTOR_CONNECTOR.PositionRefSet(0);
            Console.serialInfo(F("Home offset established as zero position"));
        }
        else
        {
            Console.serialInfo(F("Hardstop established as zero position (no offset)"));
        }

        // Complete the homing sequence
        completeHomingSequence();

        // The file-static variables (homing_hlfbWentNonAsserted, etc.)
        // will be reset by resetHomingState() at the start of the *next* homing attempt.
    }
}

void completeHomingSequence()
{
    // Reset to normal operation parameters
    currentVelMax = rpmToPps(LOADED_SHUTTLE_VELOCITY_RPM);
    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);

    // Set homing complete flags
    isHomed = true;
    homingInProgress = false; // This signals the end of the process
    motorState = MOTOR_STATE_IDLE;
    currentPositionMm = 0.0; // Position is now defined as 0

    // Restore encoder control if it was active before homing
    if (homingEncoderState)
    {
        encoderControlActive = true;
        Console.serialInfo(F("MPG handwheel control re-enabled after homing"));
    }

    Console.serialInfo(F("Homing sequence completed successfully"));

    // Now automatically move to position 1 (loading position)
    Console.serialInfo(F("Moving to position 1 (loading position)..."));
    moveToPosition(POSITION_1);
}

bool isHomingComplete()
{
    return isHomed && !homingInProgress;
}

void resetHomingState()
{
    // Reset all homing state variables
    homingInProgress = false;

    // Reset the file-static homing state variables
    homing_hlfbWentNonAsserted = false;
    homing_hlfbNonAssertedTime = 0;
    homing_minDistanceTraveled = false;

    // Reset tracking variables (now using globals directly)
    lastCheckedPosition = 0;
    lastPositionCheckTime = 0;
    minTimeAfterDistanceReached = 0;

    pulsesTraveledAfterMinDistance = 0;
    positionAtMinDistance = 0;

    // Capture the current position AFTER resetting variables
    homing_startPulses = MOTOR_CONNECTOR.PositionRefCommanded();

    Console.serialDiagnostic(F("Homing internal state variables reset."));
}

// Then update abortHoming() to use it:
void abortHoming()
{
    if (homingInProgress) // Check if homing was actually in progress
    {
        Console.serialInfo(F("Aborting homing operation"));
        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Reset to normal operation parameters
        currentVelMax = rpmToPps(LOADED_SHUTTLE_VELOCITY_RPM);
        currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
        MOTOR_CONNECTOR.VelMax(currentVelMax);
        MOTOR_CONNECTOR.AccelMax(currentAccelMax);

        // Use the dedicated reset function which now clears all necessary homing states
        resetHomingState();

        // motorState will be set by resetHomingState or subsequent updateMotorState()
        Console.serialInfo(F("Homing operation aborted successfully"));
    }
    else
    {
        Console.serialInfo(F("No homing operation in progress to abort"));
    }
}

// ----------------- Movement Progress Functions -----------------

void checkMoveProgress()
{
    char msg[200]; // Larger buffer for the complex deceleration message
    static MotorState previousState = MOTOR_STATE_NOT_READY;
    static bool wasMoving = false;
    static int32_t lastSetVelocity = currentVelMax; // Track the last velocity we set

    // Get current movement state
    bool isMoving = !MOTOR_CONNECTOR.StepsComplete();

    // ALWAYS update position when homed - moved to top for hygiene
    if (isHomed)
    {
        currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    }

    // Check if we just transitioned to MOVING state
    if (motorState == MOTOR_STATE_MOVING && previousState != MOTOR_STATE_MOVING)
    {
        wasMoving = true;
        // Serial.println(F("[DIAGNOSTIC] Movement started - tracking for LastTarget update"));
    }

    // Update the current position when moving
    if (motorState == MOTOR_STATE_MOVING)
    {
        // Apply deceleration when approaching target
        if (hasCurrentTarget && motorDecelConfig.enableDeceleration)
        {
            // Get current shuttle state
            SystemState currentState = captureSystemState();
            bool shuttleEmpty = !currentState.shuttleLocked;

            // Only apply deceleration if shuttle is NOT empty
            if (!shuttleEmpty)
            {
                // Calculate absolute distance to target using optimized integer math
                int32_t currentPos_cm = (int32_t)(currentPositionMm * MM_SCALE_FACTOR + 0.5);
                int32_t targetPos_cm = (int32_t)(currentTargetPositionMm * MM_SCALE_FACTOR + 0.5);
                int32_t distance_cm = distanceAbs_i(targetPos_cm, currentPos_cm);
                float distanceToTargetMm = (float)distance_cm / MM_SCALE_FACTOR;

                // Calculate appropriate velocity based on distance
                int32_t newVelocity = calculateDeceleratedVelocity(distanceToTargetMm, currentVelMax);

                // Apply new velocity if different from last set velocity
                if (newVelocity != lastSetVelocity &&
                    abs(ppsToRpm(newVelocity - lastSetVelocity)) > VELOCITY_CHANGE_THRESHOLD)
                {
                    MOTOR_CONNECTOR.VelMax(newVelocity);
                    lastSetVelocity = newVelocity;

                    // Optional: Log velocity changes (for debugging)
                    // sprintf(msg, "[DECEL] Distance: %.2fmm, Velocity: %d RPM, Position: %.2fmm (%ld pulses), Target: %.2fmm, HLFB: %s, Moving: %s, Direction: %s, Delta: %d RPM",
                    //         distanceToTargetMm,
                    //         (int)ppsToRpm(newVelocity),
                    //         pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()),
                    //         MOTOR_CONNECTOR.PositionRefCommanded(),
                    //         currentTargetPositionMm,
                    //         MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT_ASSERTED",
                    //         MOTOR_CONNECTOR.StepsComplete() ? "NO" : "YES",
                    //         currentTargetPulses > MOTOR_CONNECTOR.PositionRefCommanded() ? "POS" : "NEG",
                    //         (int)ppsToRpm(abs(newVelocity - lastSetVelocity)));
                    // Console.serialDiagnostic(msg);
                }
            }
        }
    }

    // Check if the move has just completed (transition from wasMoving to not moving)
    if (wasMoving && !isMoving)
    {
        // Serial.println(F("[DIAGNOSTIC] Move completed successfully"));

        // Reset to normal velocity for future movements
        if (lastSetVelocity != currentVelMax)
        {
            MOTOR_CONNECTOR.VelMax(currentVelMax);
            lastSetVelocity = currentVelMax; // Remember what we set
        }

        // Movement completed successfully, update last target
        if (hasCurrentTarget)
        {
            // sprintf(msg, "Updating LastTarget to: %.2f mm", currentTargetPositionMm);
            // Console.serialDiagnostic(msg);

            hasLastTarget = true;
            lastTargetType = currentTargetType;
            lastTargetPositionMm = currentTargetPositionMm;
            lastTargetPulses = currentTargetPulses;
            hasCurrentTarget = false; // Clear current target
        }
        else
        {
            // Serial.println(F("[DIAGNOSTIC] Move completed but no current target was set!"));
        }

        // Reset the wasMoving flag
        wasMoving = false;
    }

    previousState = motorState;
}

// ----------------- E-stop Functions -----------------

// E-stop detection and handling functions
bool isEStopActive()
{
    // E-stop is wired as normally closed with pull-up
    // When E-stop is triggered (circuit is opened), pin reads LOW
    return digitalRead(E_STOP_PIN) == LOW;
}

void handleEStop()
{
    static bool eStopWasActive = false;
    static unsigned long lastEStopCheckTime = 0;

    // Only check periodically to avoid consuming too much processing time
    unsigned long currentTime = millis();
    if (!timeoutElapsed(currentTime, lastEStopCheckTime, E_STOP_CHECK_INTERVAL_MS))
    {
        return;
    }
    lastEStopCheckTime = currentTime;

    // Check if E-stop is active
    bool eStopActive = isEStopActive();

    // Only take action if E-stop state changes from inactive to active
    if (eStopActive && !eStopWasActive)
    {
        Console.serialError(F("E-STOP TRIGGERED!"));

        // If an automated operation is in progress, abort it
        if (currentOperation.inProgress)
        {
            abortOperation(ABORT_REASON_ESTOP);
        }

        // Stop any motion immediately
        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Disable the motor
        MOTOR_CONNECTOR.EnableRequest(false);

        // If homing was in progress, abort it
        if (homingInProgress)
        {
            Console.serialInfo(F("Aborting homing operation"));
            homingInProgress = false;
        }

        // Set motor state to faulted
        motorState = MOTOR_STATE_FAULTED;
    }
    // Report when E-stop is released
    else if (!eStopActive && eStopWasActive)
    {
        Console.serialInfo(F("E-STOP RELEASED - System remains in fault state until cleared"));
    }

    eStopWasActive = eStopActive;
}

// Implement the function correctly using your existing variable names:
void updateMotorTarget(float targetPositionMm)
{
    // Update the target variables
    hasCurrentTarget = true;
    currentTargetType = POSITION_CUSTOM;
    currentTargetPositionMm = targetPositionMm;
    currentTargetPulses = mmToPulses(targetPositionMm);
}

// ----------------- Deceleration Configuration Functions -----------------
int32_t DecelerationConfig::getMinVelocityPPS() const
{
    return rpmToPps(minVelocityRPM);
}

int32_t calculateDeceleratedVelocity(float distanceToTargetMm, int32_t maxVelocity)
{
    // If deceleration is disabled, just return the max velocity
    if (!motorDecelConfig.enableDeceleration)
    {
        return maxVelocity;
    }

    // Static variables to track move characteristics
    static bool isVeryShortMove = false;
    static float totalMoveDistance = 0.0f;
    static int32_t initialTargetPulses = 0;
    static bool isLongMoveDecelerating = false;
    static float longMoveDecelStartDistance = 0.0f;

    // Only check for move type at the BEGINNING of a move
    if (hasCurrentTarget && initialTargetPulses != currentTargetPulses)
    {
        // This is a new target - analyze the move
        initialTargetPulses = currentTargetPulses;
        float moveDistance = fabs(currentTargetPositionMm - currentPositionMm);
        totalMoveDistance = moveDistance;

        // RESET ALL FLAGS FOR NEW MOVE - This ensures clean state for each move
        isVeryShortMove = false;
        isLongMoveDecelerating = false;

        // Log debug info for any move
        // char msg[100];
        // sprintf(msg, "[DECEL] New move detected: %.2fmm", moveDistance);
        // Console.serialDiagnostic(msg);

        // Check if this is a very short move
        if (moveDistance < motorDecelConfig.decelerationDistanceMm * VERY_SHORT_MOVE_RATIO)
        {
            isVeryShortMove = true;

            // sprintf(msg, "[DECEL] Very short move detected (%.2fmm) - Using special deceleration profile", totalMoveDistance);
            // Console.serialDiagnostic(msg);
        }
        // Handle long moves specifically
        else if (moveDistance > 100.0f)
        {
            isLongMoveDecelerating = true;

            // For long moves, use a longer deceleration distance (2x normal)
            longMoveDecelStartDistance = min(moveDistance * 0.3f, 150.0f);

            // sprintf(msg, "[DECEL] Long move detected (%.2fmm) - Deceleration starts at %.2fmm from target",
            //         totalMoveDistance, longMoveDecelStartDistance);
            // Console.serialDiagnostic(msg);
        }
        // Normal moves (between very short and long) use default behavior
        // Both flags remain false from reset above
    }

    // Special case for long moves
    if (isLongMoveDecelerating)
    {
        if (distanceToTargetMm < longMoveDecelStartDistance)
        {
            // Calculate deceleration ratio for long moves
            float ratio = distanceToTargetMm / longMoveDecelStartDistance;

            // Use an improved deceleration curve (quadratic for smoother decel)
            ratio = ratio * ratio;

            // Ensure minimum velocity at end of move
            int32_t minVelocityPPS = rpmToPps(motorDecelConfig.minVelocityRPM);
            int32_t targetVelocity = minVelocityPPS + ratio * (maxVelocity - minVelocityPPS);

            // Additional logging to track deceleration
            if (distanceToTargetMm < 50.0f && (int)distanceToTargetMm % 10 == 0)
            {
                // char msg[100];
                // sprintf(msg, "[DECEL] Long move: %.2fmm to target, Velocity: %d RPM",
                //         distanceToTargetMm, (int)ppsToRpm(targetVelocity));
                // Console.serialDiagnostic(msg);
            }

            return targetVelocity;
        }
        return maxVelocity;
    }

    // Existing code for very short moves and standard deceleration
    if (distanceToTargetMm < motorDecelConfig.decelerationDistanceMm)
    {
        float ratio;
        int32_t minVelocityPPS = rpmToPps(motorDecelConfig.minVelocityRPM);

        // For very short moves, use existing proportional velocity based on distance
        if (isVeryShortMove)
        {
            // Calculate how far we've gone in the move as a percentage
            float moveProgress = 1.0f - (distanceToTargetMm / totalMoveDistance);

            // Create a triangular velocity profile with peak at 25% of the move
            if (moveProgress < 0.25f)
            {
                // First 25%: Accelerate from 30% to 70% of max velocity
                ratio = 0.3f + (moveProgress / 0.25f) * 0.4f;
            }
            else
            {
                // Remaining 75%: Decelerate from 70% down to min velocity
                float decelProgress = (moveProgress - 0.25f) / 0.75f;
                ratio = 0.7f - (decelProgress * decelProgress) * 0.7f;
            }

            // Safety floor - ensure minimum velocity near the end
            if (distanceToTargetMm < 5.0f)
            {
                float minRatio = minVelocityPPS / (float)maxVelocity;
                if (ratio < minRatio)
                    ratio = minRatio;
            }
        }
        else
        {
            // Normal two-stage deceleration
            const float stageTransitionPoint = motorDecelConfig.decelerationDistanceMm * DECEL_TRANSITION_POINT_RATIO;

            if (distanceToTargetMm > stageTransitionPoint)
            {
                float firstStageRatio = (distanceToTargetMm - stageTransitionPoint) /
                                        (motorDecelConfig.decelerationDistanceMm - stageTransitionPoint);

                ratio = DECEL_FIRST_STAGE_END_RATIO +
                        (1.0f - DECEL_FIRST_STAGE_END_RATIO) * firstStageRatio;

                float normalized = (ratio - DECEL_FIRST_STAGE_END_RATIO) /
                                   (1.0f - DECEL_FIRST_STAGE_END_RATIO);
                ratio = DECEL_FIRST_STAGE_END_RATIO +
                        (1.0f - DECEL_FIRST_STAGE_END_RATIO) * normalized * normalized * DECEL_S_CURVE_MULTIPLIER;
            }
            else
            {
                ratio = DECEL_FIRST_STAGE_END_RATIO * (distanceToTargetMm / stageTransitionPoint);
            }
        }

        // Calculate the scaled velocity
        int32_t scaledVelocity = minVelocityPPS + ratio * (maxVelocity - minVelocityPPS);

        // Safety check - never go below minimum velocity
        if (scaledVelocity < minVelocityPPS)
        {
            scaledVelocity = minVelocityPPS;
        }

        return scaledVelocity;
    }

    // REMOVED: End-of-move reset logic (now handled at beginning of new moves)
    // This prevents flags from being reset while a move is still in progress

    // If we're not in deceleration zone, use maximum velocity
    return maxVelocity;
}
