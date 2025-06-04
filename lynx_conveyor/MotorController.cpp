#include "MotorController.h"

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

// Convert mm to pulses with direction control
int32_t mmToPulses(double mm)
{
    return (int32_t)(mm * PULSES_PER_MM * MOTION_DIRECTION);
}

// Convert pulses to mm
double pulsesToMm(int32_t pulses)
{
    // When converting from pulses to mm for display, include the MOTION_DIRECTION factor
    return (double)pulses / PULSES_PER_MM * MOTION_DIRECTION;
}
// Normalize encoder values for display
int32_t normalizeEncoderValue(int32_t rawValue)
{
    // Apply the same direction multiplier as used in movement calculations
    return rawValue * MOTION_DIRECTION;
}
// ----------------- Basic Setup Functions -----------------

void initMotorSystem()
{
    Console.info(F("Initializing motor system..."));

    // Set up E-stop input pin with internal pull-up
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    Console.print(F("[INFO] Checking E-Stop state: "));
    if (isEStopActive())
    {
        Console.error(F("E-STOP ACTIVE! Please reset E-stop before continuing."));
    }
    else
    {
        Console.info(F("E-stop inactive, system ready."));
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
    Console.print(F("[INFO] Setting velocity limit to "));
    Console.print(MOTOR_VELOCITY_RPM); // CHANGED: Using consolidated constant
    Console.println(F(" RPM"));

    currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM); // CHANGED
    MOTOR_CONNECTOR.VelMax(currentVelMax);

    // Set acceleration limit
    Console.print(F("[INFO] Motor enable requested"));
    Console.print(MAX_ACCEL_RPM_PER_SEC);
    Console.println(F(" RPM"));

    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);

    // Enable the motor
    MOTOR_CONNECTOR.EnableRequest(true);
    Console.info(F("Motor enable requested"));

    // Wait for HLFB to assert (up to 2 seconds)
    Console.info(F("Waiting for HLFB..."));
    unsigned long startTime = millis();
    bool ready = false;

    while (!ready && millis() - startTime < 2000)
    {
        if (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED)
        {
            ready = true;
        }
        else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
        {
            Console.error(F("Motor alert detected:"));
            printMotorAlerts();
            break;
        }
        delay(10);
    }

    if (ready)
    {
        Console.info(F("Motor initialized and ready"));
        motorInitialized = true;
        motorState = MOTOR_STATE_IDLE;
    }
    else
    {
        Console.error(F("Motor initialization timed out or failed"));
        Console.print("HLFB State: ");
        Console.println(MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT ASSERTED");
    }
}

// ----------------- Movement Functions -----------------

// For absolute positioning commands
bool moveToAbsolutePosition(int32_t position)
{
    // Check if position is within valid range (accounting for MOTION_DIRECTION)
    if ((MOTION_DIRECTION * position < 0) ||
        (MOTION_DIRECTION * position > MAX_TRAVEL_PULSES))
    {
        Console.print(F("[ERROR] Requested position "));
        Console.print(position);
        Console.print(F(" pulses is outside valid range (0 to "));
        Console.print(MOTION_DIRECTION * MAX_TRAVEL_PULSES);
        Console.println(F(" pulses)"));
        return false;
    }

    // Check if motor has alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.error(F("Motor alert detected. Cannot move."));
        printMotorAlerts();
        return false;
    }

    Console.print(F("[INFO] Moving to absolute position: "));
    Console.println(normalizeEncoderValue(position));

    // Command the absolute move
    MOTOR_CONNECTOR.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

    // Report initial status
    Console.info(F("Move commanded. Motor in motion..."));

    return true;
}

bool moveToPosition(PositionTarget position)
{
    // Set current target for logging
    hasCurrentTarget = true;
    currentTargetType = position;

    // Save original velocity for potential restoration
    int32_t originalVelMax = currentVelMax;
    double targetPositionMm = 0.0;
    int32_t targetPulses = 0;

    // Determine the target values based on position
    switch (position)
    {
    case POSITION_HOME:
        targetPositionMm = POSITION_HOME_MM;
        targetPulses = POSITION_HOME_PULSES;
        break;
    case POSITION_1:
        targetPositionMm = POSITION_1_MM;
        targetPulses = POSITION_1_PULSES;
        break;
    case POSITION_2:
        targetPositionMm = POSITION_2_MM;
        targetPulses = POSITION_2_PULSES;
        break;
    case POSITION_3:
        targetPositionMm = POSITION_3_MM;
        targetPulses = POSITION_3_PULSES;
        break;
    case POSITION_4:
        targetPositionMm = POSITION_4_MM;
        targetPulses = POSITION_4_PULSES;
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
    Console.print(F("[DIAGNOSTIC] Current position: "));
    Console.print(currentPositionMm);
    Console.print(F("mm, Target position: "));
    Console.print(targetPositionMm);
    Console.println(F("mm"));

    // Calculate distance to move with updated position value
    double distanceToMoveMm = fabs(targetPositionMm - currentPositionMm);

    Console.print(F("[DIAGNOSTIC] Calculated move distance: "));
    Console.print(distanceToMoveMm);
    Console.println(F("mm"));

    // Apply velocity scaling based on move distance
    if (distanceToMoveMm < VERY_SHORT_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(VERY_SHORT_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Very short move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(VERY_SHORT_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else if (distanceToMoveMm < SHORT_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(SHORT_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Short move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(SHORT_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else if (distanceToMoveMm < MEDIUM_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(MEDIUM_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Medium move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(MEDIUM_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else
    {
        // For long moves, explicitly set to maximum velocity
        currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
        Console.print(F("[INFO] Long move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using full speed: "));
        Console.print(MOTOR_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }

    // Rest of function remains the same
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    bool moveResult = moveToAbsolutePosition(normalizeEncoderValue(targetPulses));
    if (!moveResult)
    {
        currentVelMax = originalVelMax;
        MOTOR_CONNECTOR.VelMax(currentVelMax);
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
    // Safety check - prevent movement beyond physical limits
    if (positionMm < 0 || positionMm > MAX_TRAVEL_MM)
    {
        Console.print(F("[ERROR] Requested position "));
        Console.print(positionMm);
        Console.print(F(" mm is outside valid range (0 to "));
        Console.print(MAX_TRAVEL_MM);
        Console.println(F(" mm)"));
        return false;
    }

    // IMPORTANT: Update currentPositionMm with the latest position before calculating distance
    currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());

    // Debug print to show the actual positions being used
    Console.print(F("[DIAGNOSTIC] Current position: "));
    Console.print(currentPositionMm);
    Console.print(F("mm, Target position: "));
    Console.print(positionMm);
    Console.println(F("mm"));

    // Calculate distance to move
    double distanceToMoveMm = fabs(positionMm - currentPositionMm);

    Console.print(F("[DIAGNOSTIC] Calculated move distance: "));
    Console.print(distanceToMoveMm);
    Console.println(F("mm"));

    // Save original velocity
    int32_t originalVelMax = currentVelMax;

    // Apply velocity scaling based on move distance
    if (distanceToMoveMm < VERY_SHORT_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(VERY_SHORT_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Very short move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(VERY_SHORT_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else if (distanceToMoveMm < SHORT_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(SHORT_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Short move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(SHORT_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else if (distanceToMoveMm < MEDIUM_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(MEDIUM_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Medium move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(MEDIUM_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else
    {
        // For long moves, explicitly set to maximum velocity
        currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
        Console.print(F("[INFO] Long move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using full speed: "));
        Console.print(MOTOR_VELOCITY_RPM);
        Console.println(F(" RPM"));
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
    Console.print(F("[DIAGNOSTIC] Current position: "));
    Console.print(currentPositionMm);
    Console.print(F("mm, Target position: "));
    Console.print(targetPositionMm);
    Console.println(F("mm"));

    // Check if the target position would be out of bounds
    if (targetPositionMm < 0 || targetPositionMm > MAX_TRAVEL_MM)
    {
        Console.print(F("[ERROR] Relative move would exceed valid range (0 to "));
        Console.print(MAX_TRAVEL_MM);
        Console.println(F(" mm)"));
        Console.print(F("[ERROR] Current position: "));
        Console.print(currentPositionMm);
        Console.print(F(" mm, Requested move: "));
        Console.print(relativeMm);
        Console.print(F(" mm, Target would be: "));
        Console.print(targetPositionMm);
        Console.println(F(" mm"));
        return false;
    }

    // Calculate distance to move (use absolute value for velocity scaling)
    double distanceToMoveMm = fabs(relativeMm);

    Console.print(F("[DIAGNOSTIC] Calculated move distance: "));
    Console.print(distanceToMoveMm);
    Console.println(F("mm"));

    // Save original velocity for restoration later
    int32_t originalVelMax = currentVelMax;

    // Apply velocity scaling based on move distance
    if (distanceToMoveMm < VERY_SHORT_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(VERY_SHORT_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Very short move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(VERY_SHORT_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else if (distanceToMoveMm < SHORT_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(SHORT_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Short move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(SHORT_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else if (distanceToMoveMm < MEDIUM_MOVE_THRESHOLD_MM)
    {
        currentVelMax = rpmToPps(MEDIUM_MOVE_VELOCITY_RPM);
        Console.print(F("[INFO] Medium move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using reduced speed: "));
        Console.print(MEDIUM_MOVE_VELOCITY_RPM);
        Console.println(F(" RPM"));
    }
    else
    {
        // For long moves, explicitly set to maximum velocity
        currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
        Console.print(F("[INFO] Long move detected ("));
        Console.print(distanceToMoveMm);
        Console.print(F("mm) - Using full speed: "));
        Console.print(MOTOR_VELOCITY_RPM);
        Console.println(F(" RPM"));
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

    Console.print(F("[INFO] Moving "));
    Console.print(relativeMm);
    Console.print(F(" mm from current position ("));
    Console.print(normalizeEncoderValue(relativePulses));
    Console.println(F(" pulses)"));

    // Note: We DON'T restore velocity here - let deceleration system handle it
    // checkMoveProgress() will restore velocity when move completes

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
    Console.info(F("Motion stopped"));
}

// ----------------- Jogging Functions -----------------

bool jogMotor(bool direction, double customIncrement)
{
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
    Console.print(F("[INFO] Jogging "));
    Console.print(direction ? F("forward") : F("backward"));
    Console.print(F(" by "));
    Console.print(increment);
    Console.print(F(" mm at "));
    Console.print(currentJogSpeedRpm);
    Console.println(F(" RPM"));

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
    // Validate increment is reasonable
    if (increment <= 0 || increment > 100)
    {
        Console.error(F("Jog increment must be between 0 and 100mm"));
        return false;
    }

    // Set the increment
    currentJogIncrementMm = increment;
    Console.print(F("[INFO] Jog increment set to "));
    Console.print(currentJogIncrementMm);
    Console.println(F(" mm"));

    // Re-validate jog speed with the new distance
    setJogSpeed(currentJogSpeedRpm, increment);

    return true;
}

bool setJogSpeed(int speedRpm, double jogDistanceMm)
{
    // Get the jog distance - either from parameter or use current increment
    double distanceToMoveMm = (jogDistanceMm > 0) ? jogDistanceMm : currentJogIncrementMm;

    // Validate speed is reasonable
    if (speedRpm < 10 || speedRpm > MOTOR_VELOCITY_RPM)
    {
        Console.print(F("[ERROR] Jog speed must be between 10 and "));
        Console.print(MOTOR_VELOCITY_RPM);
        Console.println(F(" RPM"));
        return false;
    }

    // Apply distance-based speed caps
    int cappedSpeed = speedRpm;

    if (distanceToMoveMm < VERY_SHORT_MOVE_THRESHOLD_MM)
    {
        // Cap speed for very short moves
        cappedSpeed = min(speedRpm, VERY_SHORT_MOVE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            Console.print(F("[INFO] Speed capped to "));
            Console.print(cappedSpeed);
            Console.print(F(" RPM for short distance ("));
            Console.print(distanceToMoveMm);
            Console.println(F("mm)"));
        }
    }
    else if (distanceToMoveMm < SHORT_MOVE_THRESHOLD_MM)
    {
        // Cap speed for short moves
        cappedSpeed = min(speedRpm, SHORT_MOVE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            Console.print(F("[INFO] Speed capped to "));
            Console.print(cappedSpeed);
            Console.print(F(" RPM for short distance ("));
            Console.print(distanceToMoveMm);
            Console.println(F("mm)"));
        }
    }
    else if (distanceToMoveMm < MEDIUM_MOVE_THRESHOLD_MM)
    {
        // Cap speed for medium moves
        cappedSpeed = min(speedRpm, MEDIUM_MOVE_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            Console.print(F("[INFO] Speed capped to "));
            Console.print(cappedSpeed);
            Console.print(F(" RPM for medium distance ("));
            Console.print(distanceToMoveMm);
            Console.println(F("mm)"));
        }
    }
    else
    {
        // For long moves, explicitly set to maximum velocity if requested speed is higher
        cappedSpeed = min(speedRpm, MOTOR_VELOCITY_RPM);

        if (cappedSpeed != speedRpm)
        {
            Console.print(F("[INFO] Speed capped to "));
            Console.print(cappedSpeed);
            Console.print(F(" RPM for long distance ("));
            Console.print(distanceToMoveMm);
            Console.println(F("mm)"));
        }
    }

    // Set the speed with the potentially capped value
    currentJogSpeedRpm = cappedSpeed;
    Console.print(F("[INFO] Jog speed set to "));
    Console.print(currentJogSpeedRpm);
    Console.println(F(" RPM"));

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
    Console.info(F("Motor Status:"));

    Console.print(F("  Enabled: "));
    Console.println(MOTOR_CONNECTOR.EnableRequest() ? F("Yes") : F("No"));

    Console.print(F("  Moving: "));
    Console.println(isMotorAtPosition() ? F("No") : F("Yes"));

    Console.print(F("  Position: "));
    Console.print(normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded()));
    Console.println(F(" pulses"));

    Console.print(F("  Current Velocity Limit: "));
    Console.print(ppsToRpm(currentVelMax));
    Console.println(F(" RPM"));

    Console.print(F("  Current Acceleration Limit: "));
    double accelRpmPerSec = (double)currentAccelMax * 60.0 / PULSES_PER_REV;
    Console.print(accelRpmPerSec);
    Console.println(F(" RPM/s"));

    Console.print(F("  HLFB Status: "));
    switch (MOTOR_CONNECTOR.HlfbState())
    {
    case MotorDriver::HLFB_ASSERTED:
        Console.println(F("Asserted (In Position/Ready)"));
        break;
    case MotorDriver::HLFB_DEASSERTED:
        Console.println(F("Deasserted (Moving/Fault)"));
        break;
    case MotorDriver::HLFB_UNKNOWN:
    default:
        Console.println(F("Unknown"));
        break;
    }

    // Print any alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.println(F("  Alerts present:"));
        printMotorAlerts();
    }
    else
    {
        Console.println(F("  No alerts"));
    }
}

void printMotorAlerts()
{
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledInAlert)
    {
        Console.error(F("    MotionCanceledInAlert"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledPositiveLimit)
    {
        Console.error(F("    MotionCanceledPositiveLimit"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledNegativeLimit)
    {
        Console.error(F("    MotionCanceledNegativeLimit"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledSensorEStop)
    {
        Console.error(F("    MotionCanceledSensorEStop"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledMotorDisabled)
    {
        Console.error(F("    MotionCanceledMotorDisabled"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted)
    {
        Console.error(F("    MotorFaulted"));
    }
}

void clearMotorFaults()
{
    // If we're not already in the process of clearing faults, start the process
    if (!faultClearInProgress)
    {
        Console.println(F("[DIAGNOSTIC] Attempting to clear motor faults..."));

        // First check if there are faults present
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
        {
            Console.println(F("[DIAGNOSTIC] Alerts detected:"));
            printMotorAlerts();

            // Start the fault clearing state machine
            faultClearState = FAULT_CLEAR_DISABLE;
            faultClearTimer = millis();
            faultClearInProgress = true;
        }
        else
        {
            Console.info(F("No alerts to clear."));
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
            Console.println(F("[DIAGNOSTIC] Motor faulted. Cycling enable signal..."));
            MOTOR_CONNECTOR.EnableRequest(false);
        }
        faultClearTimer = currentTime;
        faultClearState = FAULT_CLEAR_WAITING_DISABLE;
        break;

    case FAULT_CLEAR_WAITING_DISABLE:
        // Wait 100ms after disabling
        if (currentTime - faultClearTimer >= 100)
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
        if (currentTime - faultClearTimer >= 100)
        {
            faultClearState = FAULT_CLEAR_ALERTS;
        }
        break;

    case FAULT_CLEAR_ALERTS:
        // Clear alerts
        Console.println(F("[DIAGNOSTIC] Clearing motor alerts..."));
        MOTOR_CONNECTOR.ClearAlerts();

        // Check if alerts were cleared
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
        {
            Console.error(F("Alerts are still present after clearing."));
            printMotorAlerts();
        }
        else
        {
            Console.info(F("Alerts successfully cleared."));
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
        Console.info(F("Fault clearing already in progress"));
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
        Console.error(F("Motor not initialized"));
        return false;
    }

    // Check for faults
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.error(F("Motor has active alerts - clear faults before homing"));
        return false;
    }

    // Store encoder state before disabling for homing
    homingEncoderState = encoderControlActive;

    // Disable encoder control if active
    if (encoderControlActive)
    {
        encoderControlActive = false;
        Console.println(F("[INFO] MPG handwheel control disabled during homing"));
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

    Console.info(F("Homing sequence initiated. Motor will move to find home position."));
    return true;
}

void checkHomingProgress()
{
    if (!homingInProgress)
        return;

    unsigned long currentTime = millis();

    // Add a delay before starting actual hardstop detection
    static const unsigned long homingStartDelay = 500; // 500ms delay
    if (currentTime - homingStartTime < homingStartDelay)
    {
        return; // Don't process hardstop detection until initial delay is complete
    }

    static const int32_t minimumMovementPulses = 3000;  // Minimum movement before detecting hardstop
    static const int32_t minimumAdditionalPulses = 500; // Must travel at least this much AFTER min distance

    MotorDriver::HlfbStates currentHlfbState = MOTOR_CONNECTOR.HlfbState();
    int32_t currentPosition = MOTOR_CONNECTOR.PositionRefCommanded();

    // Check if motor is actually moving during homing (every 100ms)
    if (currentTime - lastPositionCheckTime > 100)
    {
        // Calculate movement since last check
        int32_t movementSinceLastCheck = abs(currentPosition - lastCheckedPosition);

        // Log movement data when in detail when we've crossed the minimum distance threshold
        if (homing_minDistanceTraveled)
        {
            Console.print(F("[HOMING] Position: "));
            Console.print(currentPosition);
            Console.print(F(", Movement: "));
            Console.print(movementSinceLastCheck);
            Console.print(F(" pulses, HLFB: "));
            Console.println(currentHlfbState == MotorDriver::HLFB_ASSERTED ? F("ASSERTED") : F("NOT_ASSERTED"));
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
    if (currentTime - homingStartTime > 30000)
    { // 30 seconds timeout
        Console.error(F("Homing operation timed out"));
        Console.print(F("[DIAGNOSTIC] Final HLFB state: "));
        Console.println(currentHlfbState == MotorDriver::HLFB_ASSERTED ? F("ASSERTED") : F("NOT ASSERTED"));

        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Set current position as home despite timeout
        Console.info(F("Setting current position as home reference despite timeout"));
        MOTOR_CONNECTOR.PositionRefSet(0);

        // Complete homing setup
        completeHomingSequence();
        return;
    }

    // Check for alerts during homing
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Console.error(F("Motor alert during homing"));
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
        Console.print(F("[INFO] Minimum travel distance reached ("));
        Console.print(pulsesMovedThisHoming); // Log actual travel for this homing
        Console.println(F(" pulses) - Hardstop detection enabled"));
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
            Console.info(F("HLFB went non-asserted - approaching hardstop"));
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
        (currentTime - homing_hlfbNonAssertedTime > 100) &&
        homing_minDistanceTraveled &&
        (currentTime - minTimeAfterDistanceReached > 300) && // Increased to 300ms
        pulsesTraveledAfterMinDistance >= minimumAdditionalPulses)
    { // NEW condition

        Console.print(F("[INFO] Hardstop reached - HLFB reasserted after "));
        Console.print(currentTime - minTimeAfterDistanceReached);
        Console.print(F("ms from minimum distance, additional travel: "));
        Console.print(pulsesTraveledAfterMinDistance);
        Console.println(F(" pulses"));

        // Stop the velocity move
        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Set position to zero at the actual hardstop before offset
        MOTOR_CONNECTOR.PositionRefSet(0);

        // Move away from hardstop to complete homing
        if (HOME_OFFSET_DISTANCE_MM > 0)
        {
            Console.print(F("[INFO] Moving "));
            Console.print(HOME_OFFSET_DISTANCE_MM);
            Console.println(F("mm away from hardstop"));

            // Reset velocity to normal (or a specific offset velocity if desired)
            int32_t normalVelPps = rpmToPps(MOTOR_VELOCITY_RPM); // Or a slower offset speed
            MOTOR_CONNECTOR.VelMax(normalVelPps);

            // Move away from hardstop
            int32_t offsetPulses = mmToPulses(HOME_OFFSET_DISTANCE_MM);
            // The direction of offset should be opposite to HOMING_DIRECTION
            MOTOR_CONNECTOR.Move(-HOMING_DIRECTION * offsetPulses, MotorDriver::MOVE_TARGET_REL_END_POSN);

            // Wait for offset move to complete
            Console.info(F("Waiting for offset move to complete..."));
            unsigned long offsetMoveStartTime = millis(); // Use a fresh start time for this wait
            while (!MOTOR_CONNECTOR.StepsComplete() &&
                   (millis() - offsetMoveStartTime < 5000))
            { // Timeout for offset move
                delay(10);

                // Check for alerts during offset move
                if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
                {
                    Console.error(F("Alert during offset move"));
                    abortHoming(); // abortHoming should call the updated resetHomingState
                    return;
                }
            }
            if (!MOTOR_CONNECTOR.StepsComplete())
            {
                Console.error(F("Offset move timed out or failed to complete."));
                // Decide how to handle this: abort or try to complete homing anyway?
                // For now, we'll proceed to set home, but this is a potential issue.
            }

            // Re-zero at offset position
            MOTOR_CONNECTOR.PositionRefSet(0);
            Console.info(F("Home offset established as zero position"));
        }
        else
        {
            Console.info(F("Hardstop established as zero position (no offset)"));
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
    currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
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
        Console.println(F("[INFO] MPG handwheel control re-enabled after homing"));
    }

    Console.info(F("Homing sequence completed successfully"));

    // Now automatically move to position 1 (loading position)
    Console.info(F("Moving to position 1 (loading position)..."));
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

    Console.println(F("[DIAGNOSTIC] Homing internal state variables reset."));
}

// Then update abortHoming() to use it:
void abortHoming()
{
    if (homingInProgress) // Check if homing was actually in progress
    {
        Console.info(F("Aborting homing operation"));
        MOTOR_CONNECTOR.MoveStopAbrupt();

        // Reset to normal operation parameters
        currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
        currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
        MOTOR_CONNECTOR.VelMax(currentVelMax);
        MOTOR_CONNECTOR.AccelMax(currentAccelMax);

        // Use the dedicated reset function which now clears all necessary homing states
        resetHomingState();

        // motorState will be set by resetHomingState or subsequent updateMotorState()
        Console.info(F("Homing operation aborted successfully"));
    }
    else
    {
        Console.info(F("No homing operation in progress to abort"));
    }
}

// ----------------- Movement Progress Functions -----------------

void checkMoveProgress()
{
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
        // Console.println(F("[DIAGNOSTIC] Movement started - tracking for LastTarget update"));
    }

    // Update the current position when moving
    if (motorState == MOTOR_STATE_MOVING)
    {

        // Apply deceleration when approaching target
        if (hasCurrentTarget && motorDecelConfig.enableDeceleration)
        {
            // Calculate absolute distance to target
            float distanceToTargetMm = fabs(currentTargetPositionMm - currentPositionMm);

            // Calculate appropriate velocity based on distance
            int32_t newVelocity = calculateDeceleratedVelocity(distanceToTargetMm, currentVelMax);

            // Apply new velocity if different from last set velocity
            if (newVelocity != lastSetVelocity &&
                abs(ppsToRpm(newVelocity - lastSetVelocity)) > VELOCITY_CHANGE_THRESHOLD)
            {
                MOTOR_CONNECTOR.VelMax(newVelocity);
                lastSetVelocity = newVelocity;

                // Optional: Log velocity changes (for debugging)
                // Console.print(F("[DECEL] Distance: "));
                // Console.print(distanceToTargetMm);
                // Console.print(F("mm, Velocity: "));
                // Console.print(ppsToRpm(newVelocity));
                // Console.print(F(" RPM, Position: "));
                // Console.print(pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()));
                // Console.print(F("mm ("));
                // Console.print(MOTOR_CONNECTOR.PositionRefCommanded());
                // Console.print(F(" pulses), Target: "));
                // Console.print(currentTargetPositionMm);
                // Console.print(F("mm, HLFB: "));
                // Console.print(MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED ? F("ASSERTED") : F("NOT_ASSERTED"));
                // Console.print(F(", Moving: "));
                // Console.print(MOTOR_CONNECTOR.StepsComplete() ? F("NO") : F("YES"));
                // Console.print(F(", Direction: ")); // Fixed: removed "mm"
                // Console.print(currentTargetPulses > MOTOR_CONNECTOR.PositionRefCommanded() ? F("POS") : F("NEG"));
                // Console.print(F(", Delta: "));
                // Console.print(ppsToRpm(abs(newVelocity - lastSetVelocity)));
                // Console.println();
            }
        }
    }

    // Check if the move has just completed (transition from wasMoving to not moving)
    if (wasMoving && !isMoving)
    {
        // Console.println(F("[DIAGNOSTIC] Move completed successfully"));

        // Reset to normal velocity for future movements
        if (lastSetVelocity != currentVelMax)
        {
            MOTOR_CONNECTOR.VelMax(currentVelMax);
            lastSetVelocity = currentVelMax; // Remember what we set
        }

        // Movement completed successfully, update last target
        if (hasCurrentTarget)
        {
            // Console.print(F("[DIAGNOSTIC] Updating LastTarget to: "));
            // Console.print(currentTargetPositionMm);
            // Console.println(F(" mm"));

            hasLastTarget = true;
            lastTargetType = currentTargetType;
            lastTargetPositionMm = currentTargetPositionMm;
            lastTargetPulses = currentTargetPulses;
            hasCurrentTarget = false; // Clear current target
        }
        else
        {
            // Console.println(F("[DIAGNOSTIC] Move completed but no current target was set!"));
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
    if (currentTime - lastEStopCheckTime < E_STOP_CHECK_INTERVAL_MS)
    {
        return;
    }
    lastEStopCheckTime = currentTime;

    // Check if E-stop is active
    bool eStopActive = isEStopActive();

    // Only take action if E-stop state changes from inactive to active
    if (eStopActive && !eStopWasActive)
    {
        Console.error(F("E-STOP TRIGGERED!"));

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
            Console.info(F("Aborting homing operation"));
            homingInProgress = false;
        }

        // Set motor state to faulted
        motorState = MOTOR_STATE_FAULTED;
    }
    // Report when E-stop is released
    else if (!eStopActive && eStopWasActive)
    {
        Console.info(F("E-STOP RELEASED - System remains in fault state until cleared"));
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
        // Console.print(F("[DECEL] New move detected: "));
        // Console.print(moveDistance);
        // Console.println(F("mm"));

        // Check if this is a very short move
        if (moveDistance < motorDecelConfig.decelerationDistanceMm * VERY_SHORT_MOVE_RATIO)
        {
            isVeryShortMove = true;

            // Console.print(F("[DECEL] Very short move detected ("));
            // Console.print(totalMoveDistance);
            // Console.println(F("mm) - Using special deceleration profile"));
        }
        // Handle long moves specifically
        else if (moveDistance > 100.0f)
        {
            isLongMoveDecelerating = true;

            // For long moves, use a longer deceleration distance (2x normal)
            longMoveDecelStartDistance = min(moveDistance * 0.3f, 150.0f);

            // Console.print(F("[DECEL] Long move detected ("));
            // Console.print(totalMoveDistance);
            // Console.print(F("mm) - Deceleration starts at "));
            // Console.print(longMoveDecelStartDistance);
            // Console.println(F("mm from target"));
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
            // if (distanceToTargetMm < 50.0f && (int)distanceToTargetMm % 10 == 0) {
            //     Console.print(F("[DECEL] Long move: "));
            //     Console.print(distanceToTargetMm);
            //     Console.print(F("mm to target, Velocity: "));
            //     Console.print(ppsToRpm(targetVelocity));
            //     Console.println(F(" RPM"));
            // }

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
