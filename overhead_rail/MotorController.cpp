#include "MotorController.h"
#include "Utils.h"

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_MOTOR_ALERT_DETAILS[] PROGMEM = "%s Alert Details:\n%s";
const char FMT_INITIALIZING[] PROGMEM = "Initializing %s...";
const char FMT_VELOCITY_LIMIT[] PROGMEM = "%s: Setting velocity limit to %d RPM";
const char FMT_ACCEL_LIMIT[] PROGMEM = "%s: Setting acceleration limit to %d RPM/s";
const char FMT_ENABLE_REQUESTED[] PROGMEM = "%s: Enable requested";
const char FMT_WAITING_HLFB[] PROGMEM = "%s: Waiting for HLFB...";
const char FMT_ALERT_DETECTED[] PROGMEM = "%s: Motor alert detected";
const char FMT_READY[] PROGMEM = "%s: Initialized and ready";
const char FMT_INIT_FAILED[] PROGMEM = "%s: Initialization timed out or failed";
const char FMT_HLFB_STATE[] PROGMEM = "%s HLFB State: %s";
const char FMT_MOTION_STOPPED[] PROGMEM = "%s motion stopped";
const char FMT_CLEARING_FAULTS[] PROGMEM = "Attempting to clear %s faults...";
const char FMT_ALERTS_DETECTED[] PROGMEM = "%s alerts detected:";
const char FMT_NO_ALERTS[] PROGMEM = "%s: No alerts to clear.";
const char FMT_MOTOR_FAULTED[] PROGMEM = "%s faulted. Cycling enable signal...";
const char FMT_CLEARING_ALERTS[] PROGMEM = "Clearing %s alerts...";
const char FMT_ALERTS_PERSIST[] PROGMEM = "%s: Alerts are still present after clearing.";
const char FMT_ALERTS_CLEARED[] PROGMEM = "%s: Alerts successfully cleared.";
const char FMT_FAULT_IN_PROGRESS[] PROGMEM = "%s fault clearing already in progress";
const char FMT_INVALID_POSITION[] PROGMEM = "%s: Invalid position for rail";
const char FMT_MOTOR_CANNOT_MOVE[] PROGMEM = "%s: Motor alert detected. Cannot move.";
const char FMT_NOT_READY_HOMING[] PROGMEM = "%s: Motor not ready for homing";
const char FMT_ALERTS_BEFORE_HOMING[] PROGMEM = "%s: Motor has active alerts - clear faults before homing";
const char FMT_ALERT_DURING_HOMING[] PROGMEM = "%s: Motor alert during homing";
const char FMT_MOTOR_STATUS[] PROGMEM = "[INFO] %s Status:\n  Enabled: %s\n  Moving: %s\n  Position: %ld pulses (%.1f mm)\n  HLFB Status: %s\n  %s";

// Homing operation format strings
const char FMT_HOMING_INITIATED[] PROGMEM = "%s: Homing sequence initiated. Motor moving to find home position.";
const char FMT_HOMING_DIAGNOSTIC[] PROGMEM = "[HOMING %s] Position: %ld, Movement: %ld pulses, HLFB: %s";
const char FMT_MINIMAL_MOVEMENT[] PROGMEM = "%s: Minimal movement detected during homing";
const char FMT_HOMING_TIMEOUT[] PROGMEM = "%s: Homing operation timed out";
const char FMT_FINAL_HLFB_STATE[] PROGMEM = "%s: Final HLFB state: %s";
const char FMT_SET_HOME_TIMEOUT[] PROGMEM = "%s: Setting current position as home reference despite timeout";
const char FMT_MIN_DISTANCE_REACHED[] PROGMEM = "%s: Minimum travel distance reached (%ld pulses) - Hardstop detection enabled";
const char FMT_HLFB_NON_ASSERTED[] PROGMEM = "%s: HLFB went non-asserted - approaching hardstop";
const char FMT_HARDSTOP_REACHED[] PROGMEM = "%s: Hardstop reached - HLFB reasserted after %ldms, additional travel: %ld pulses";
const char FMT_MOVING_FROM_HARDSTOP[] PROGMEM = "%s: Moving %.2fmm away from hardstop";
const char FMT_WAITING_OFFSET[] PROGMEM = "%s: Waiting for offset move to complete...";
const char FMT_ALERT_OFFSET_MOVE[] PROGMEM = "%s: Alert during offset move";
const char FMT_OFFSET_TIMEOUT[] PROGMEM = "%s: Offset move timed out or failed to complete";
const char FMT_HOME_OFFSET_ESTABLISHED[] PROGMEM = "%s: Home offset established as zero position";
const char FMT_HARDSTOP_AS_ZERO[] PROGMEM = "%s: Hardstop established as zero position (no offset)";
const char FMT_HOMING_COMPLETED[] PROGMEM = "%s: Homing sequence completed successfully in ";
const char FMT_HOMING_STATE_RESET[] PROGMEM = "%s: Homing state variables reset";
const char FMT_ABORTING_HOMING[] PROGMEM = "%s: Aborting homing operation";
const char FMT_HOMING_ABORTED[] PROGMEM = "%s: Homing operation aborted successfully";

// Jogging operation format strings
const char FMT_NOT_READY_JOG[] PROGMEM = "%s: Motor not ready for jogging";
const char FMT_CANNOT_JOG_MOVING[] PROGMEM = "%s: Cannot jog while motor is moving";
const char FMT_JOG_EXCEED_LIMITS[] PROGMEM = "%s: Jog would exceed travel limits (0 to %.1fmm)";
const char FMT_JOG_POSITION_INFO[] PROGMEM = "%s: Current: %.2fmm, Target would be: %.2fmm";
const char FMT_JOGGING[] PROGMEM = "%s: Jogging %s by %.2fmm at %d RPM %s";
const char FMT_JOG_INCREMENT_RANGE[] PROGMEM = "%s: Jog increment must be between 0 and %.1fmm";
const char FMT_JOG_INCREMENT_SET[] PROGMEM = "%s: Jog increment set to %.2fmm";
const char FMT_JOG_SPEED_RANGE[] PROGMEM = "%s: Jog speed must be between 10 and %d RPM";
const char FMT_SPEED_CAPPED_VERY_SHORT[] PROGMEM = "%s: Speed capped to %d RPM for very short jog (%.2fmm)";
const char FMT_SPEED_CAPPED_SHORT[] PROGMEM = "%s: Speed capped to %d RPM for short jog (%.2fmm)";
const char FMT_SPEED_CAPPED_MEDIUM[] PROGMEM = "%s: Speed capped to %d RPM for medium jog (%.2fmm)";
const char FMT_SPEED_CAPPED_LONG[] PROGMEM = "%s: Speed capped to %d RPM for long jog (%.2fmm)";
const char FMT_JOG_SPEED_SET[] PROGMEM = "%s: Jog speed set to %d RPM";
const char FMT_MOVE_POSITIONED[] PROGMEM = "%s: %s→%s (%.1fmm) at %d RPM %s";
const char FMT_MOVE_TO_POSITION[] PROGMEM = "%s: Moving to %s (%.1fmm) at %d RPM %s";
const char FMT_INVALID_POSITION_NUM_RAIL1[] PROGMEM = "%s: Invalid position number %d (valid: 0-4)";
const char FMT_INVALID_POSITION_NUM_RAIL2[] PROGMEM = "%s: Invalid position number %d (valid: 0-2)";
const char FMT_INVALID_RAIL_NUM[] PROGMEM = "Invalid rail number %d (valid: 1-2)";
const char FMT_POSITION_OUT_OF_RANGE[] PROGMEM = "%s: Position %ld pulses outside valid range (0 to %ld)";
const char FMT_MOVE_TO_ABSOLUTE[] PROGMEM = "%s: Moving to absolute position %ld pulses (%.1fmm)";
const char FMT_POSITION_MM_OUT_OF_RANGE[] PROGMEM = "%s: Position %.2fmm outside valid range (0 to %.2fmm)";
const char FMT_MOVE_TO_MM[] PROGMEM = "%s: Moving to %.2fmm (%ld pulses) at %d RPM %s";
const char FMT_RELATIVE_MOVE_RANGE[] PROGMEM = "%s: Relative move would exceed valid range (0 to %.2fmm)";
const char FMT_RELATIVE_MOVE_DETAILS[] PROGMEM = "%s: Current: %.2fmm, Move: %.2fmm, Target would be: %.2fmm";
const char FMT_RELATIVE_MOVE[] PROGMEM = "%s: Moving %.2fmm relative (%ld pulses) at %d RPM %s";
const char FMT_INITIATE_HOMING[] PROGMEM = "Initiating homing sequence for %s";

// Alert message constants
const char ALERT_MOTION_CANCELED_IN_ALERT[] PROGMEM = "    MotionCanceledInAlert\n";
const char ALERT_MOTION_CANCELED_POSITIVE_LIMIT[] PROGMEM = "    MotionCanceledPositiveLimit\n";
const char ALERT_MOTION_CANCELED_NEGATIVE_LIMIT[] PROGMEM = "    MotionCanceledNegativeLimit\n";
const char ALERT_MOTION_CANCELED_SENSOR_ESTOP[] PROGMEM = "    MotionCanceledSensorEStop\n";
const char ALERT_MOTION_CANCELED_MOTOR_DISABLED[] PROGMEM = "    MotionCanceledMotorDisabled\n";
const char ALERT_MOTOR_FAULTED[] PROGMEM = "    MotorFaulted\n";

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// System State
bool motorInitialized = false;
int32_t currentVelMax = 0;
int32_t currentAccelMax = 0;

// Motor-Specific State Tracking
MotorState rail1MotorState = MOTOR_STATE_NOT_READY;
MotorState rail2MotorState = MOTOR_STATE_NOT_READY;

// Fault Clearing State Management
FaultClearingState rail1FaultClearState = FAULT_CLEAR_IDLE;
FaultClearingState rail2FaultClearState = FAULT_CLEAR_IDLE;
bool rail1FaultClearInProgress = false;
bool rail2FaultClearInProgress = false;
unsigned long rail1FaultClearTimer = 0;
unsigned long rail2FaultClearTimer = 0;

// Homing State Management
bool rail1HomingInProgress = false;
bool rail2HomingInProgress = false;
MotorHomingState rail1HomingState = {0};
MotorHomingState rail2HomingState = {0};

// Jogging Configuration
double rail1JogIncrementMm = RAIL1_DEFAULT_JOG_INCREMENT_MM;    // Default jog increment for Rail 1 (1.0mm)
double rail2JogIncrementMm = RAIL2_DEFAULT_JOG_INCREMENT_MM;    // Default jog increment for Rail 2 (0.5mm)
int rail1JogSpeedRpm = RAIL1_DEFAULT_JOG_SPEED_RPM;            // Default jog speed for Rail 1
int rail2JogSpeedRpm = RAIL2_DEFAULT_JOG_SPEED_RPM;            // Default jog speed for Rail 2

// Deceleration Configurations
RailDecelerationConfig rail1DecelConfig = {
    .longMoveDecelerationDistanceMm = 15000,    // 150.0mm decel for long moves
    .mediumMoveDecelerationDistanceMm = 7500,   // 75.0mm decel for medium moves
    .shortMoveDecelerationDistanceMm = 3000,    // 30.0mm decel for short moves
    .longMoveThresholdMm = 300000,              // 3000.0mm = long move
    .mediumMoveThresholdMm = 100000,            // 1000.0mm = medium move
    .minVelocityRPM = 50,                       // Minimum velocity during deceleration
    .enableDeceleration = true
};

RailDecelerationConfig rail2DecelConfig = {
    .longMoveDecelerationDistanceMm = 5000,     // 50.0mm decel for longer Rail 2 moves
    .mediumMoveDecelerationDistanceMm = 3000,   // 30.0mm decel for medium Rail 2 moves
    .shortMoveDecelerationDistanceMm = 2000,    // 20.0mm decel for short Rail 2 moves
    .longMoveThresholdMm = 80000,               // 800.0mm = long move for Rail 2
    .mediumMoveThresholdMm = 40000,             // 400.0mm = medium move for Rail 2
    .minVelocityRPM = 50,                       // Same minimum velocity
    .enableDeceleration = true
};

// Movement Target Tracking State
MotorTargetState rail1TargetState = {0};
MotorTargetState rail2TargetState = {0};

//=============================================================================
// HELPER FUNCTIONS - RAIL-SPECIFIC ACCESS
//=============================================================================

// Get homing state for specific rail
MotorHomingState& getHomingState(int rail) {
    return (rail == 1) ? rail1HomingState : rail2HomingState;
}

// Get target tracking state for specific rail
MotorTargetState& getTargetState(int rail) {
    return (rail == 1) ? rail1TargetState : rail2TargetState;
}

// Get homing direction for specific rail
int getHomingDirection(int rail) {
    return (rail == 1) ? RAIL1_HOMING_DIRECTION : RAIL2_HOMING_DIRECTION;
}

// Get home offset distance for specific rail
double getHomeOffsetDistance(int rail) {
    return (rail == 1) ? RAIL1_HOME_OFFSET_DISTANCE_MM : RAIL2_HOME_OFFSET_DISTANCE_MM;
}

// Get jog parameters by reference for modification
double& getJogIncrementRef(int rail) {
    return (rail == 1) ? rail1JogIncrementMm : rail2JogIncrementMm;
}

int& getJogSpeedRef(int rail) {
    return (rail == 1) ? rail1JogSpeedRpm : rail2JogSpeedRpm;
}

// Get deceleration config for specific rail
RailDecelerationConfig& getDecelerationConfig(int rail) {
    return (rail == 1) ? rail1DecelConfig : rail2DecelConfig;
}

// Get motor reference by rail number
MotorDriver& getMotorByRail(int rail) {
    return (rail == 1) ? RAIL1_MOTOR : RAIL2_MOTOR;
}

// Get motor name by rail number
const char* getMotorName(int rail) {
    return (rail == 1) ? "Rail 1" : "Rail 2";
}

// Get rail-specific carriage velocity
int32_t getCarriageVelocityRpm(int rail, bool carriageLoaded) {
    if (rail == 1) {
        return carriageLoaded ? RAIL1_LOADED_CARRIAGE_VELOCITY_RPM : RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM;
    } else if (rail == 2) {
        return carriageLoaded ? RAIL2_LOADED_CARRIAGE_VELOCITY_RPM : RAIL2_EMPTY_CARRIAGE_VELOCITY_RPM;
    } else {
        // Invalid rail number - this should not happen in normal operation
        // Default to Rail 1 speeds but log an error
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("ERROR: Invalid rail number %d in getCarriageVelocityRpm"), rail);
        Console.serialError(msg);
        return carriageLoaded ? RAIL1_LOADED_CARRIAGE_VELOCITY_RPM : RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM;
    }
}

//=============================================================================
// SYSTEM INITIALIZATION AND SAFETY
//=============================================================================

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
    if (!waitTimeReached(currentTime, lastEStopCheckTime, E_STOP_CHECK_INTERVAL_MS))
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

        // Stop all motion immediately
        stopAllMotion();

        // Disable both motors
        RAIL1_MOTOR.EnableRequest(false);
        RAIL2_MOTOR.EnableRequest(false);

        // If homing was in progress on either rail, abort it
        if (rail1HomingInProgress)
        {
            Console.serialInfo(F("Aborting Rail 1 homing operation"));
            abortHoming(1);
        }
        if (rail2HomingInProgress)
        {
            Console.serialInfo(F("Aborting Rail 2 homing operation"));
            abortHoming(2);
        }

        // Set both motors to faulted state
        rail1MotorState = MOTOR_STATE_FAULTED;
        rail2MotorState = MOTOR_STATE_FAULTED;
    }
    // Report when E-stop is released
    else if (!eStopActive && eStopWasActive)
    {
        Console.serialInfo(F("E-STOP RELEASED - System remains in fault state until cleared"));
        Console.serialInfo(F("Use fault clearing commands to re-enable motors"));
    }

    eStopWasActive = eStopActive;
}

void printMotorAlerts(MotorDriver &motor, const char* motorName)
{
    char msg[LARGE_MSG_SIZE];
    char alertList[ALERT_MSG_SIZE] = "";
    
    if (motor.AlertReg().bit.MotionCanceledInAlert)
        strcat_P(alertList, ALERT_MOTION_CANCELED_IN_ALERT);
    if (motor.AlertReg().bit.MotionCanceledPositiveLimit)
        strcat_P(alertList, ALERT_MOTION_CANCELED_POSITIVE_LIMIT);
    if (motor.AlertReg().bit.MotionCanceledNegativeLimit)
        strcat_P(alertList, ALERT_MOTION_CANCELED_NEGATIVE_LIMIT);
    if (motor.AlertReg().bit.MotionCanceledSensorEStop)
        strcat_P(alertList, ALERT_MOTION_CANCELED_SENSOR_ESTOP);
    if (motor.AlertReg().bit.MotionCanceledMotorDisabled)
        strcat_P(alertList, ALERT_MOTION_CANCELED_MOTOR_DISABLED);
    if (motor.AlertReg().bit.MotorFaulted)
        strcat_P(alertList, ALERT_MOTOR_FAULTED);

    // Remove trailing newline
    int len = strlen(alertList);
    if (len > 0 && alertList[len - 1] == '\n')
        alertList[len - 1] = '\0';

    sprintf_P(msg, FMT_MOTOR_ALERT_DETAILS, motorName, alertList);
    Console.serialError(msg);
}

bool initSingleMotor(MotorDriver &motor, const char* motorName, int32_t velocityRpm, int32_t accelRpmPerSec)
{
    char msg[SMALL_MSG_SIZE];
    sprintf_P(msg, FMT_INITIALIZING, motorName);
    Console.serialInfo(msg);

    // Set the motor's HLFB mode to bipolar PWM
    motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Set velocity and acceleration limits
    sprintf_P(msg, FMT_VELOCITY_LIMIT, motorName, velocityRpm);
    Console.serialInfo(msg);
    
    int32_t velMax = rpmToPps(velocityRpm);
    motor.VelMax(velMax);

    sprintf_P(msg, FMT_ACCEL_LIMIT, motorName, accelRpmPerSec);
    Console.serialInfo(msg);
    
    int32_t accelMax = rpmPerSecToPpsPerSec(accelRpmPerSec);
    motor.AccelMax(accelMax);

    // Enable the motor
    motor.EnableRequest(true);
    sprintf_P(msg, FMT_ENABLE_REQUESTED, motorName);
    Console.serialInfo(msg);

    // Wait for HLFB to assert (up to 2 seconds)
    sprintf_P(msg, FMT_WAITING_HLFB, motorName);
    Console.serialInfo(msg);
    
    unsigned long startTime = millis();
    bool ready = false;

    while (!ready && !timeoutElapsed(millis(), startTime, 2000))
    {
        if (motor.HlfbState() == MotorDriver::HLFB_ASSERTED)
        {
            ready = true;
        }
        else if (motor.StatusReg().bit.AlertsPresent)
        {
            sprintf_P(msg, FMT_ALERT_DETECTED, motorName);
            Console.serialError(msg);
            printMotorAlerts(motor, motorName);
            break;
        }
        delay(10);
    }

    if (ready)
    {
        sprintf_P(msg, FMT_READY, motorName);
        Console.serialInfo(msg);
        return true;
    }
    else
    {
        sprintf_P(msg, FMT_INIT_FAILED, motorName);
        Console.serialError(msg);
        sprintf_P(msg, FMT_HLFB_STATE, motorName,
                motor.HlfbState() == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT ASSERTED");
        Console.serialError(msg);
        return false;
    }
}

void initMotorSystem()
{
    Console.serialInfo(F("Initializing motor system..."));

    // Set up E-stop input pin with internal pull-up
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    if (isEStopActive())
    {
        Console.serialError(F("E-STOP ACTIVE! Please reset E-stop before continuing."));
        return;
    }
    else
    {
        Console.serialInfo(F("E-stop inactive, system ready."));
    }

    // Set the input clocking rate
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Configure motor connector for step and direction mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Initialize both motors with rail-specific loaded carriage velocity as default
    bool rail1Ready = initSingleMotor(RAIL1_MOTOR, "Rail 1", RAIL1_LOADED_CARRIAGE_VELOCITY_RPM, MAX_ACCEL_RPM_PER_SEC);
    bool rail2Ready = initSingleMotor(RAIL2_MOTOR, "Rail 2", RAIL2_LOADED_CARRIAGE_VELOCITY_RPM, MAX_ACCEL_RPM_PER_SEC);

    if (rail1Ready && rail2Ready)
    {
        Console.serialInfo(F("Both motors initialized and ready"));
        motorInitialized = true;
        rail1MotorState = MOTOR_STATE_IDLE;
        rail2MotorState = MOTOR_STATE_IDLE;
    }
    else
    {
        Console.serialError(F("Motor system initialization failed"));
        rail1MotorState = MOTOR_STATE_FAULTED;
        rail2MotorState = MOTOR_STATE_FAULTED;
    }
}

//=============================================================================
// UNIT CONVERSION UTILITIES
//=============================================================================

int32_t rpmToPps(double rpm)
{
    return (int32_t)((rpm * PULSES_PER_REV) / 60.0);
}

double ppsToRpm(int32_t pps)
{
    return (double)pps * 60.0 / PULSES_PER_REV;
}

int32_t rpmPerSecToPpsPerSec(double rpmPerSec)
{
    return (int32_t)((rpmPerSec * PULSES_PER_REV) / 60.0);
}

int32_t rail1MmToPulses(double mm)
{
    return (int32_t)(mm * RAIL1_PULSES_PER_MM);
}

int32_t rail2MmToPulses(double mm)
{
    return (int32_t)(mm * RAIL2_PULSES_PER_MM);
}

double rail1PulsesToMm(int32_t pulses)
{
    return (double)pulses / RAIL1_PULSES_PER_MM;
}

double rail2PulsesToMm(int32_t pulses)
{
    return (double)pulses / RAIL2_PULSES_PER_MM;
}

int32_t mmToPulses(double mm, int rail)
{
    return (rail == 1) ? rail1MmToPulses(mm) : rail2MmToPulses(mm);
}

double pulsesToMm(int32_t pulses, int rail)
{
    return (rail == 1) ? rail1PulsesToMm(pulses) : rail2PulsesToMm(pulses);
}

//=============================================================================
// OPTIMIZED INTEGER MATH CONVERSION FUNCTIONS
//=============================================================================

int32_t mmToPulsesScaled(int32_t mmScaled, int rail)
{
    // Input: mm * 100, Output: pulses
    // For both rails: (mmScaled * PULSES_PER_MM_SCALED) / (100 * 100)
    // mmScaled is mm * 100, PULSES_PER_MM_SCALED is pulses per 100mm
    // So: (mm * 100 * pulses_per_100mm) / 100 = mm * pulses_per_100mm / 100
    
    if (rail == 1) {
        return (mmScaled * RAIL1_PULSES_PER_MM_SCALED) / 10000; // 100 * 100
    } else {
        return (mmScaled * RAIL2_PULSES_PER_MM_SCALED) / 10000;
    }
}

int32_t pulsesToMmScaled(int32_t pulses, int rail)
{
    // Input: pulses, Output: mm * 100
    // pulses * 10000 / PULSES_PER_MM_SCALED = mm * 100
    
    if (rail == 1) {
        return (pulses * 10000) / RAIL1_PULSES_PER_MM_SCALED;
    } else {
        return (pulses * 10000) / RAIL2_PULSES_PER_MM_SCALED;
    }
}

int32_t mmToPulsesInteger(double mm, int rail)
{
    // Optimized version that uses integer math internally
    // Convert mm to scaled integer, then use integer math
    int32_t mmScaled = (int32_t)(mm * 100);
    return mmToPulsesScaled(mmScaled, rail);
}

//=============================================================================
// POSITION AND RAIL UTILITIES
//=============================================================================

int32_t getPositionPulses(PositionTarget target)
{
    switch(target) {
        case RAIL1_HOME_POS: return RAIL1_HOME_POSITION_PULSES;
        case RAIL1_WC2_PICKUP_DROPOFF_POS: return RAIL1_WC2_PICKUP_DROPOFF_PULSES;
        case RAIL1_WC1_PICKUP_DROPOFF_POS: return RAIL1_WC1_PICKUP_DROPOFF_PULSES;
        case RAIL1_STAGING_POS: return RAIL1_STAGING_POSITION_PULSES;
        case RAIL1_HANDOFF_POS: return RAIL1_HANDOFF_PULSES;
        case RAIL2_HOME_POS: return RAIL2_HOME_POSITION_PULSES;
        case RAIL2_HANDOFF_POS: return RAIL2_HANDOFF_PULSES;
        case RAIL2_WC3_PICKUP_DROPOFF_POS: return RAIL2_WC3_PICKUP_DROPOFF_PULSES;
        default: return 0;
    }
}

int getRailFromPosition(PositionTarget target)
{
    if (target >= RAIL1_HOME_POS && target <= RAIL1_HANDOFF_POS) {
        return 1;
    } else if (target >= RAIL2_HOME_POS && target <= RAIL2_WC3_PICKUP_DROPOFF_POS) {
        return 2;
    }
    return 0; // Invalid
}

bool isValidPositionForRail(PositionTarget target, int rail)
{
    return (getRailFromPosition(target) == rail);
}

const char* getPositionName(PositionTarget pos) {
    switch(pos) {
        case RAIL1_HOME_POS: return "Home";
        case RAIL1_WC2_PICKUP_DROPOFF_POS: return "WC2";
        case RAIL1_WC1_PICKUP_DROPOFF_POS: return "WC1";
        case RAIL1_STAGING_POS: return "Staging";
        case RAIL1_HANDOFF_POS: return "R1-Handoff";
        case RAIL2_HOME_POS: return "Home/WC3";
        case RAIL2_HANDOFF_POS: return "R2-Handoff";
        case RAIL2_WC3_PICKUP_DROPOFF_POS: return "WC3";
        default: return "Unknown";
    }
}

//=============================================================================
// MOTOR CONTROL AND STATUS
//=============================================================================

double getMotorPositionMm(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    return pulsesToMm(motor.PositionRefCommanded(), rail);
}

void stopMotion(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    motor.MoveStopAbrupt();
    char msg[50];
    sprintf_P(msg, FMT_MOTION_STOPPED, getMotorName(rail));
    Console.serialInfo(msg);
}

void stopAllMotion()
{
    stopMotion(1);
    stopMotion(2);
}

bool isMotorReady(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    return motorInitialized &&
           motor.EnableRequest() &&
           motor.HlfbState() == MotorDriver::HLFB_ASSERTED &&
           !motor.StatusReg().bit.AlertsPresent;
}

bool isMotorMoving(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    return !motor.StepsComplete();
}

bool isMotorInPosition(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    return motor.StepsComplete() &&
           motor.HlfbState() == MotorDriver::HLFB_ASSERTED;
}

bool hasMotorFault(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    return motor.StatusReg().bit.AlertsPresent;
}

MotorState updateMotorState(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    MotorState* motorState = (rail == 1) ? &rail1MotorState : &rail2MotorState;
    bool* homingInProgress = (rail == 1) ? &rail1HomingInProgress : &rail2HomingInProgress;

    // Check for faults first
    if (motor.StatusReg().bit.AlertsPresent)
        *motorState = MOTOR_STATE_FAULTED;
    // Check if motor is not enabled
    else if (!motor.EnableRequest())
        *motorState = MOTOR_STATE_NOT_READY;
    // Check if homing is in progress
    else if (*homingInProgress)
        *motorState = MOTOR_STATE_HOMING;
    // Check if steps are complete
    else if (!motor.StepsComplete())
        *motorState = MOTOR_STATE_MOVING;
    // Otherwise, motor is idle
    else
        *motorState = MOTOR_STATE_IDLE;

    return *motorState;
}

void printMotorStatus(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[LARGE_MSG_SIZE];

    // Determine HLFB status string
    const char *hlfbStatus;
    switch (motor.HlfbState())
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
    const char *alertStatus = motor.StatusReg().bit.AlertsPresent ? 
        "Alerts present (see alert details below)" : "No alerts";

    // Build the complete status message
    sprintf_P(msg, FMT_MOTOR_STATUS,
            motorName,
            motor.EnableRequest() ? "Yes" : "No",
            isMotorMoving(rail) ? "Yes" : "No",
            motor.PositionRefCommanded(),
            getMotorPositionMm(rail),
            hlfbStatus,
            alertStatus);

    Console.print(msg);

    // If there are alerts, print them
    if (motor.StatusReg().bit.AlertsPresent)
        printMotorAlerts(motor, motorName);
}

void printAllMotorStatus()
{
    // Show system uptime first
    Console.print("[INFO] System Status:\n");
    Console.print("  Uptime: ");
    printHumanReadableTime(millis() / 1000);
    Console.print("\n\n");
    
    printMotorStatus(1);
    printMotorStatus(2);
}

//=============================================================================
// FAULT MANAGEMENT
//=============================================================================

void clearMotorFaults(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    bool* faultClearInProgress = (rail == 1) ? &rail1FaultClearInProgress : &rail2FaultClearInProgress;
    FaultClearingState* faultClearState = (rail == 1) ? &rail1FaultClearState : &rail2FaultClearState;
    unsigned long* faultClearTimer = (rail == 1) ? &rail1FaultClearTimer : &rail2FaultClearTimer;

    if (!*faultClearInProgress)
    {
        char msg[100];
        sprintf_P(msg, FMT_CLEARING_FAULTS, motorName);
        Console.serialDiagnostic(msg);

        if (motor.StatusReg().bit.AlertsPresent)
        {
            sprintf_P(msg, FMT_ALERTS_DETECTED, motorName);
            Console.serialDiagnostic(msg);
            printMotorAlerts(motor, motorName);

            *faultClearState = FAULT_CLEAR_DISABLE;
            *faultClearTimer = millis();
            *faultClearInProgress = true;
        }
        else
        {
            sprintf_P(msg, FMT_NO_ALERTS, motorName);
            Console.serialInfo(msg);
        }
    }
}

void processFaultClearing(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    bool* faultClearInProgress = (rail == 1) ? &rail1FaultClearInProgress : &rail2FaultClearInProgress;
    FaultClearingState* faultClearState = (rail == 1) ? &rail1FaultClearState : &rail2FaultClearState;
    unsigned long* faultClearTimer = (rail == 1) ? &rail1FaultClearTimer : &rail2FaultClearTimer;

    if (!*faultClearInProgress)
        return;

    unsigned long currentTime = millis();

    switch (*faultClearState)
    {
    case FAULT_CLEAR_DISABLE:
        if (motor.AlertReg().bit.MotorFaulted)
        {
            char msg[100];
            sprintf_P(msg, FMT_MOTOR_FAULTED, motorName);
            Console.serialDiagnostic(msg);
            motor.EnableRequest(false);
        }
        *faultClearTimer = currentTime;
        *faultClearState = FAULT_CLEAR_WAITING_DISABLE;
        break;

    case FAULT_CLEAR_WAITING_DISABLE:
        if (timeoutElapsed(currentTime, *faultClearTimer, 100))
            *faultClearState = FAULT_CLEAR_ENABLE;
        break;

    case FAULT_CLEAR_ENABLE:
        motor.EnableRequest(true);
        *faultClearTimer = currentTime;
        *faultClearState = FAULT_CLEAR_WAITING_ENABLE;
        break;

    case FAULT_CLEAR_WAITING_ENABLE:
        if (timeoutElapsed(currentTime, *faultClearTimer, 100))
            *faultClearState = FAULT_CLEAR_ALERTS;
        break;

    case FAULT_CLEAR_ALERTS:
        {
            char msg[100];
            sprintf_P(msg, FMT_CLEARING_ALERTS, motorName);
            Console.serialDiagnostic(msg);
            motor.ClearAlerts();

            if (motor.StatusReg().bit.AlertsPresent)
            {
                sprintf_P(msg, FMT_ALERTS_PERSIST, motorName);
                Console.serialError(msg);
                printMotorAlerts(motor, motorName);
            }
            else
            {
                sprintf_P(msg, FMT_ALERTS_CLEARED, motorName);
                Console.serialInfo(msg);
            }

            *faultClearState = FAULT_CLEAR_FINISHED;
        }
        break;

    case FAULT_CLEAR_FINISHED:
        *faultClearState = FAULT_CLEAR_IDLE;
        *faultClearInProgress = false;
        break;

    default:
        *faultClearState = FAULT_CLEAR_IDLE;
        *faultClearInProgress = false;
        break;
    }
}

void processAllFaultClearing()
{
    processFaultClearing(1);
    processFaultClearing(2);
}

bool isFaultClearingInProgress(int rail)
{
    return (rail == 1) ? rail1FaultClearInProgress : rail2FaultClearInProgress;
}

bool clearMotorFaultWithStatus(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    bool* faultClearInProgress = (rail == 1) ? &rail1FaultClearInProgress : &rail2FaultClearInProgress;
    
    // If fault clearing is already in progress, return false
    if (*faultClearInProgress)
    {
        char msg[100];
        sprintf_P(msg, FMT_FAULT_IN_PROGRESS, motorName);
        Console.serialInfo(msg);
        return false;
    }

    bool hadAlerts = motor.StatusReg().bit.AlertsPresent;

    // Start fault clearing process
    clearMotorFaults(rail);

    // Return true if there were no alerts to clear (immediate success)
    // Return false if clearing process has started (delayed result)
    return !hadAlerts;
}

//=============================================================================
// MOTION DECELERATION
//=============================================================================

int32_t getDecelerationDistanceScaled(int rail, int32_t moveDistanceScaledMm) {
    RailDecelerationConfig& config = getDecelerationConfig(rail);
    
    if (moveDistanceScaledMm >= config.longMoveThresholdMm) {
        return config.longMoveDecelerationDistanceMm;
    } else if (moveDistanceScaledMm >= config.mediumMoveThresholdMm) {
        return config.mediumMoveDecelerationDistanceMm;
    } else {
        return config.shortMoveDecelerationDistanceMm;
    }
}

int32_t calculateDeceleratedVelocity(int rail, int32_t distanceToTargetMm, int32_t totalMoveDistanceMm, int32_t maxVelocity) {
    RailDecelerationConfig& config = getDecelerationConfig(rail);
    
    // If deceleration is disabled, return max velocity
    if (!config.enableDeceleration)
        return maxVelocity;
    
    // Scale distances for fixed-point math
    int32_t distanceToTargetScaled = distanceToTargetMm * SCALE_FACTOR;
    int32_t totalMoveDistanceScaled = totalMoveDistanceMm * SCALE_FACTOR;
    
    // Get appropriate deceleration distance for this move
    int32_t decelDistanceScaled = getDecelerationDistanceScaled(rail, totalMoveDistanceScaled);
    
    // If we're not in deceleration zone yet, use max velocity
    if (distanceToTargetScaled > decelDistanceScaled)
        return maxVelocity;
    
    // Calculate deceleration ratio using integer math (0-1000 range for precision)
    int32_t decelRatio1000 = (distanceToTargetScaled * 1000) / decelDistanceScaled;
    
    // Apply quadratic deceleration curve: ratio^2
    int32_t decelRatioSquared1000 = (decelRatio1000 * decelRatio1000) / 1000;
    
    // Calculate minimum velocity in PPS
    int32_t minVelocityPPS = rpmToPps(config.minVelocityRPM);
    
    // Calculate target velocity using integer math
    int32_t velocityRange = maxVelocity - minVelocityPPS;
    int32_t velocityAdjustment = (velocityRange * decelRatioSquared1000) / 1000;
    int32_t targetVelocity = minVelocityPPS + velocityAdjustment;
    
    // Safety check - never go below minimum
    if (targetVelocity < minVelocityPPS)
        targetVelocity = minVelocityPPS;
    
    return targetVelocity;
}



//=============================================================================
// POSITIONING AND MOVEMENT
//=============================================================================

int32_t selectMoveVelocity(int rail, PositionTarget fromPos, PositionTarget toPos, bool carriageLoaded) {
    // Calculate move distance using integer math
    int32_t fromPulses = getPositionPulses(fromPos);
    int32_t toPulses = getPositionPulses(toPos);
    int32_t pulseDifference = abs(toPulses - fromPulses);
    
    // Convert to mm using integer math where possible
    int32_t moveDistanceMm = pulseDifference / (int32_t)((rail == 1) ? RAIL1_PULSES_PER_MM : RAIL2_PULSES_PER_MM);
    
    // Base velocities - use rail-specific constants
    int32_t emptyCarriageVelocity = getCarriageVelocityRpm(rail, false);
    int32_t loadedCarriageVelocity = getCarriageVelocityRpm(rail, true);
    
    // Special cases for specific move types
    if (rail == 1) {
        // Rail 1 move analysis
        if ((fromPos == RAIL1_STAGING_POS && toPos == RAIL1_HANDOFF_POS) ||
            (fromPos == RAIL1_HANDOFF_POS && toPos == RAIL1_STAGING_POS)) {
            // Staging↔Handoff: Always precision moves, reduce velocity by 30%
            int32_t baseVel = carriageLoaded ? loadedCarriageVelocity : emptyCarriageVelocity;
            return rpmToPps((baseVel * 70) / 100); // 70% using integer math
        }
    } else if (rail == 2) {
        // Rail 2 move analysis
        if ((fromPos == RAIL2_HANDOFF_POS && toPos == RAIL2_WC3_PICKUP_DROPOFF_POS) ||
            (fromPos == RAIL2_WC3_PICKUP_DROPOFF_POS && toPos == RAIL2_HANDOFF_POS)) {
            // Handoff↔WC3: Precision moves when loaded
            if (carriageLoaded) {
                return rpmToPps((loadedCarriageVelocity * 80) / 100); // 80% using integer math
            }
        }
    }
    
    // For very short moves (< 200mm), reduce velocity by 20%
    if (moveDistanceMm < 200) {
        int32_t baseVel = carriageLoaded ? loadedCarriageVelocity : emptyCarriageVelocity;
        return rpmToPps((baseVel * 80) / 100);
    }
    
    // Standard velocity selection based on load
    return rpmToPps(carriageLoaded ? loadedCarriageVelocity : emptyCarriageVelocity);
}

bool moveToPosition(int rail, PositionTarget fromPos, PositionTarget toPos, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    
    // Validate positions
    if (!isValidPositionForRail(fromPos, rail) || !isValidPositionForRail(toPos, rail)) {
        char msg[100];
        sprintf_P(msg, FMT_INVALID_POSITION, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Get positions using integer math
    int32_t fromPulses = getPositionPulses(fromPos);
    int32_t toPulses = getPositionPulses(toPos);
    
    // Convert to mm for display purposes only
    double fromPosMm = pulsesToMm(fromPulses, rail);
    double toPosMm = pulsesToMm(toPulses, rail);
    double moveDistanceMm = fabs(toPosMm - fromPosMm);
    
    // Select appropriate velocity for this move
    int32_t moveVelocity = selectMoveVelocity(rail, fromPos, toPos, carriageLoaded);
    
    // Set motor velocity
    motor.VelMax(moveVelocity);
    
    // Initialize movement tracking before starting the move
    initMovementTracking(rail, toPos, carriageLoaded);
    
    // Start the move
    motor.Move(toPulses);
    
    // Log the move
    char msg[MEDIUM_MSG_SIZE];
    const char* fromName = getPositionName(fromPos);
    const char* toName = getPositionName(toPos);
    sprintf_P(msg, FMT_MOVE_POSITIONED, 
            motorName, fromName, toName, moveDistanceMm,
            (int)ppsToRpm(moveVelocity), carriageLoaded ? "[LOADED]" : "[EMPTY]");
    Console.serialInfo(msg);
    
    return true;
}

bool moveToPositionFromCurrent(int rail, PositionTarget toPos, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    int32_t currentPulses = motor.PositionRefCommanded();
    double currentPosMm = pulsesToMm(currentPulses, rail);
    
    int32_t toPulses = getPositionPulses(toPos);
    double toPosMm = pulsesToMm(toPulses, rail);
    double moveDistanceMm = fabs(toPosMm - currentPosMm);
    
    // Select velocity (without knowing exact from position, use distance-based selection)
    int32_t moveVelocity;
    if (moveDistanceMm < 200.0) {
        // Short move - reduce velocity by 20%
        int32_t baseVel = getCarriageVelocityRpm(rail, carriageLoaded);
        moveVelocity = rpmToPps((baseVel * 80) / 100);
    } else {
        // Normal move - use full rail-specific velocity
        moveVelocity = rpmToPps(getCarriageVelocityRpm(rail, carriageLoaded));
    }
    
    motor.VelMax(moveVelocity);
    
    // Initialize movement tracking before starting the move
    initMovementTracking(rail, toPos, carriageLoaded);
    
    motor.Move(toPulses);
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_MOVE_TO_POSITION, 
            getMotorName(rail), getPositionName(toPos), moveDistanceMm,
            (int)ppsToRpm(moveVelocity), carriageLoaded ? "[LOADED]" : "[EMPTY]");
    Console.serialInfo(msg);
    
    return true;
}

bool moveToPosition(int rail, int positionNumber, bool carriageLoaded) {
    const char* motorName = getMotorName(rail);
    PositionTarget target;
    
    // Convert position number to PositionTarget enum based on rail
    if (rail == 1) {
        switch(positionNumber) {
            case 0: target = RAIL1_HOME_POS; break;
            case 1: target = RAIL1_WC2_PICKUP_DROPOFF_POS; break;
            case 2: target = RAIL1_WC1_PICKUP_DROPOFF_POS; break;
            case 3: target = RAIL1_STAGING_POS; break;
            case 4: target = RAIL1_HANDOFF_POS; break;
            default: 
                {
                    char msg[100];
                    sprintf_P(msg, FMT_INVALID_POSITION_NUM_RAIL1, motorName, positionNumber);
                    Console.serialError(msg);
                    return false;
                }
        }
    } else if (rail == 2) {
        switch(positionNumber) {
            case 0: target = RAIL2_HOME_POS; break;
            case 1: target = RAIL2_HANDOFF_POS; break;
            case 2: target = RAIL2_WC3_PICKUP_DROPOFF_POS; break;
            default: 
                {
                    char msg[100];
                    sprintf_P(msg, FMT_INVALID_POSITION_NUM_RAIL2, motorName, positionNumber);
                    Console.serialError(msg);
                    return false;
                }
        }
    } else {
        char msg[100];
        sprintf_P(msg, FMT_INVALID_RAIL_NUM, rail);
        Console.serialError(msg);
        return false;
    }
    
    // Use the existing moveToPositionFromCurrent function
    return moveToPositionFromCurrent(rail, target, carriageLoaded);
}

bool moveToAbsolutePosition(int rail, int32_t positionPulses) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    
    // Safety check - ensure position is within rail limits
    int32_t maxTravel = mmToPulses((rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM, rail);
    
    if (positionPulses < 0 || positionPulses > maxTravel) {
        char msg[150];
        sprintf_P(msg, FMT_POSITION_OUT_OF_RANGE, 
                motorName, positionPulses, maxTravel);
        Console.serialError(msg);
        return false;
    }
    
    // Check for motor alerts
    if (motor.StatusReg().bit.AlertsPresent) {
        char msg[100];
        sprintf_P(msg, FMT_MOTOR_CANNOT_MOVE, motorName);
        Console.serialError(msg);
        printMotorAlerts(motor, motorName);
        return false;
    }
    
    // Command the absolute move
    motor.Move(positionPulses, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    char msg[150];
    sprintf_P(msg, FMT_MOVE_TO_ABSOLUTE, 
            motorName, positionPulses, pulsesToMm(positionPulses, rail));
    Console.serialInfo(msg);
    
    return true;
}

bool moveToPositionMm(int rail, double positionMm, bool carriageLoaded) {
    const char* motorName = getMotorName(rail);
    double maxTravel = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    // Safety check
    if (positionMm < 0 || positionMm > maxTravel) {
        char msg[150];
        sprintf_P(msg, FMT_POSITION_MM_OUT_OF_RANGE, 
                motorName, positionMm, maxTravel);
        Console.serialError(msg);
        return false;
    }
    
    // Convert to pulses and calculate move parameters
    int32_t targetPulses = mmToPulses(positionMm, rail);
    MotorDriver& motor = getMotorByRail(rail);
    int32_t currentPulses = motor.PositionRefCommanded();
    double currentPosMm = pulsesToMm(currentPulses, rail);
    double moveDistanceMm = fabs(positionMm - currentPosMm);
    
    // Select velocity based on distance and load
    int32_t moveVelocity;
    if (moveDistanceMm < 200.0) {
        // Short move
        int32_t baseVel = getCarriageVelocityRpm(rail, carriageLoaded);
        moveVelocity = rpmToPps((baseVel * 80) / 100);
    } else {
        // Normal move
        moveVelocity = rpmToPps(getCarriageVelocityRpm(rail, carriageLoaded));
    }
    
    // Apply velocity and move
    motor.VelMax(moveVelocity);
    motor.Move(targetPulses, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    char msg[200];
    sprintf_P(msg, FMT_MOVE_TO_MM, 
            motorName, positionMm, targetPulses, (int)ppsToRpm(moveVelocity),
            carriageLoaded ? "[LOADED]" : "[EMPTY]");
    Console.serialInfo(msg);
    
    return true;
}

bool moveRelativeManual(int rail, double relativeMm, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    
    // Calculate target position
    int32_t currentPulses = motor.PositionRefCommanded();
    double currentPosMm = pulsesToMm(currentPulses, rail);
    double targetPosMm = currentPosMm + relativeMm;
    double maxTravel = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    // Safety check
    if (targetPosMm < 0 || targetPosMm > maxTravel) {
        char msg[200];
        sprintf_P(msg, FMT_RELATIVE_MOVE_RANGE, motorName, maxTravel);
        Console.serialError(msg);
        sprintf_P(msg, FMT_RELATIVE_MOVE_DETAILS, 
                motorName, currentPosMm, relativeMm, targetPosMm);
        Console.serialError(msg);
        return false;
    }
    
    // Calculate move distance for velocity selection
    double moveDistanceMm = fabs(relativeMm);
    
    // Select velocity based on distance and load
    int32_t moveVelocity;
    if (moveDistanceMm < 200.0) {
        // Short move
        int32_t baseVel = getCarriageVelocityRpm(rail, carriageLoaded);
        moveVelocity = rpmToPps((baseVel * 80) / 100);
    } else {
        // Normal move
        moveVelocity = rpmToPps(getCarriageVelocityRpm(rail, carriageLoaded));
    }
    
    // Convert to pulses and apply move
    int32_t relativePulses = mmToPulses(relativeMm, rail);
    motor.VelMax(moveVelocity);
    motor.Move(relativePulses, MotorDriver::MOVE_TARGET_REL_END_POSN);
    
    char msg[200];
    sprintf_P(msg, FMT_RELATIVE_MOVE, 
            motorName, relativeMm, relativePulses, (int)ppsToRpm(moveVelocity),
            carriageLoaded ? "[LOADED]" : "[EMPTY]");
    Console.serialInfo(msg);
    
    return true;
}

//=============================================================================
// HOMING OPERATIONS
//=============================================================================

bool initiateHomingSequence(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    MotorHomingState& homingState = getHomingState(rail);
    
    char msg[100];
    sprintf_P(msg, FMT_INITIATE_HOMING, motorName);
    Console.serialInfo(msg);

    // Check if motor is ready
    if (!isMotorReady(rail))
    {
        sprintf_P(msg, FMT_NOT_READY_HOMING, motorName);
        Console.serialError(msg);
        return false;
    }

    // Check for faults
    if (motor.StatusReg().bit.AlertsPresent)
    {
        sprintf_P(msg, FMT_ALERTS_BEFORE_HOMING, motorName);
        Console.serialError(msg);
        return false;
    }

    // Reset homing state first
    resetHomingState(rail);

    // For "Upon every Enable" configuration, cycle the enable signal
    motor.EnableRequest(false);
    delay(200);
    motor.EnableRequest(true);
    delay(200);

    // Set velocity for homing
    int32_t homingVelPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
    motor.VelMax(homingVelPps);

    // Move in homing direction at continuous velocity
    int homingDirection = getHomingDirection(rail);
    motor.MoveVelocity(homingDirection * homingVelPps);

    // Update state
    homingState.homingInProgress = true;
    homingState.homingStartTime = millis();
    
    // Update motor state tracking
    if (rail == 1) {
        rail1HomingInProgress = true;
        rail1MotorState = MOTOR_STATE_HOMING;
    } else {
        rail2HomingInProgress = true;
        rail2MotorState = MOTOR_STATE_HOMING;
    }

    sprintf_P(msg, FMT_HOMING_INITIATED, motorName);
    Console.serialInfo(msg);
    return true;
}

void checkHomingProgress(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    MotorHomingState& homingState = getHomingState(rail);
    
    if (!homingState.homingInProgress)
        return;

    unsigned long currentTime = millis();
    char msg[200];

    // Add a delay before starting actual hardstop detection
    static const unsigned long homingStartDelay = 500;
    if (!waitTimeReached(currentTime, homingState.homingStartTime, homingStartDelay))
        return;

    MotorDriver::HlfbStates currentHlfbState = motor.HlfbState();
    int32_t currentPosition = motor.PositionRefCommanded();

    // Check if motor is actually moving during homing (every 100ms)
    if (waitTimeReached(currentTime, homingState.lastPositionCheckTime, 100))
    {
        // Calculate movement since last check
        int32_t movementSinceLastCheck = abs(currentPosition - homingState.lastCheckedPosition);

        // Log movement data when we've crossed the minimum distance threshold
        if (homingState.minDistanceTraveled)
        {
            sprintf_P(msg, FMT_HOMING_DIAGNOSTIC,
                    motorName, currentPosition, movementSinceLastCheck,
                    currentHlfbState == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT_ASSERTED");
            Console.serialDiagnostic(msg);
        }

        if (movementSinceLastCheck < 10 && homingState.hlfbWentNonAsserted)
        {
            sprintf_P(msg, FMT_MINIMAL_MOVEMENT, motorName);
            Console.serialWarning(msg);
        }

        homingState.lastCheckedPosition = currentPosition;
        homingState.lastPositionCheckTime = currentTime;
    }

    // Check for timeout first
    if (timeoutElapsed(currentTime, homingState.homingStartTime, HOME_TIMEOUT_MS))
    {
        sprintf_P(msg, FMT_HOMING_TIMEOUT, motorName);
        Console.serialError(msg);
        sprintf_P(msg, FMT_FINAL_HLFB_STATE, motorName,
                currentHlfbState == MotorDriver::HLFB_ASSERTED ? "ASSERTED" : "NOT ASSERTED");
        Console.serialDiagnostic(msg);

        motor.MoveStopAbrupt();

        // Set current position as home despite timeout
        sprintf_P(msg, FMT_SET_HOME_TIMEOUT, motorName);
        Console.serialInfo(msg);
        motor.PositionRefSet(0);

        // Complete homing setup
        completeHomingSequence(rail);
        return;
    }

    // Check for alerts during homing
    if (motor.StatusReg().bit.AlertsPresent)
    {
        sprintf_P(msg, FMT_ALERT_DURING_HOMING, motorName);
        Console.serialError(msg);
        printMotorAlerts(motor, motorName);

        // Stop motion and abort homing
        motor.MoveStopAbrupt();
        abortHoming(rail);
        return;
    }

    // Calculate pulses moved during THIS specific homing attempt
    int32_t pulsesMovedThisHoming = abs(currentPosition - homingState.startPulses);

    // Check if we've moved enough distance to consider hardstop detection
    if (pulsesMovedThisHoming >= HOMING_MIN_MOVEMENT_PULSES && !homingState.minDistanceTraveled)
    {
        homingState.minDistanceTraveled = true;
        homingState.minTimeAfterDistanceReached = currentTime;
        homingState.positionAtMinDistance = currentPosition;
        sprintf_P(msg, FMT_MIN_DISTANCE_REACHED, 
                motorName, pulsesMovedThisHoming);
        Console.serialInfo(msg);
    }

    // After min distance reached, track additional travel
    if (homingState.minDistanceTraveled)
    {
        homingState.pulsesTraveledAfterMinDistance = abs(currentPosition - homingState.positionAtMinDistance);
    }

    // Check for non-asserted state (HLFB indicates movement or fault)
    if (currentHlfbState != MotorDriver::HLFB_ASSERTED)
    {
        if (!homingState.hlfbWentNonAsserted)
        {
            homingState.hlfbWentNonAsserted = true;
            homingState.hlfbNonAssertedTime = currentTime;
            sprintf_P(msg, FMT_HLFB_NON_ASSERTED, motorName);
            Console.serialInfo(msg);
        }
    }

    // Hardstop detection logic
    if (homingState.hlfbWentNonAsserted &&
        currentHlfbState == MotorDriver::HLFB_ASSERTED &&
        timeoutElapsed(currentTime, homingState.hlfbNonAssertedTime, HOMING_DEBOUNCE_TIME_MS) &&
        homingState.minDistanceTraveled &&
        timeoutElapsed(currentTime, homingState.minTimeAfterDistanceReached, HOMING_MIN_TIME_AFTER_DISTANCE_MS) &&
        homingState.pulsesTraveledAfterMinDistance >= HOMING_MIN_ADDITIONAL_PULSES)
    {
        sprintf_P(msg, FMT_HARDSTOP_REACHED,
                motorName, timeDiff(currentTime, homingState.minTimeAfterDistanceReached), 
                homingState.pulsesTraveledAfterMinDistance);
        Console.serialInfo(msg);

        // Stop the velocity move
        motor.MoveStopAbrupt();

        // Set position to zero at the actual hardstop before offset
        motor.PositionRefSet(0);

        // Move away from hardstop to complete homing
        double homeOffset = getHomeOffsetDistance(rail);
        if (homeOffset > 0)
        {
            sprintf_P(msg, FMT_MOVING_FROM_HARDSTOP, motorName, homeOffset);
            Console.serialInfo(msg);

            // Reset velocity to normal for homing operation
            int32_t normalVelPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
            motor.VelMax(normalVelPps);

            // Move away from hardstop
            int32_t offsetPulses = mmToPulses(homeOffset, rail);
            int homingDirection = getHomingDirection(rail);
            motor.Move(-homingDirection * offsetPulses, MotorDriver::MOVE_TARGET_REL_END_POSN);

            // Wait for offset move to complete
            sprintf_P(msg, FMT_WAITING_OFFSET, motorName);
            Console.serialInfo(msg);
            
            unsigned long offsetMoveStartTime = millis();
            while (!motor.StepsComplete() && !timeoutElapsed(millis(), offsetMoveStartTime, 5000))
            {
                delay(10);

                // Check for alerts during offset move
                if (motor.StatusReg().bit.AlertsPresent)
                {
                    sprintf_P(msg, FMT_ALERT_OFFSET_MOVE, motorName);
                    Console.serialError(msg);
                    abortHoming(rail);
                    return;
                }
            }
            
            if (!motor.StepsComplete())
            {
                sprintf_P(msg, FMT_OFFSET_TIMEOUT, motorName);
                Console.serialError(msg);
            }

            // Re-zero at offset position
            motor.PositionRefSet(0);
            sprintf_P(msg, FMT_HOME_OFFSET_ESTABLISHED, motorName);
            Console.serialInfo(msg);
        }
        else
        {
            sprintf_P(msg, FMT_HARDSTOP_AS_ZERO, motorName);
            Console.serialInfo(msg);
        }

        // Complete the homing sequence
        completeHomingSequence(rail);
    }
}

void completeHomingSequence(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    MotorHomingState& homingState = getHomingState(rail);
    
    // Reset to normal operation parameters (use homing speed as conservative default)
    int32_t normalVelPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
    int32_t normalAccelPps = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    motor.VelMax(normalVelPps);
    motor.AccelMax(normalAccelPps);

    // Set homing complete flags
    homingState.isHomed = true;
    homingState.homingInProgress = false;
    
    // Update motor state tracking
    if (rail == 1) {
        rail1HomingInProgress = false;
        rail1MotorState = MOTOR_STATE_IDLE;
    } else {
        rail2HomingInProgress = false;
        rail2MotorState = MOTOR_STATE_IDLE;
    }

    char msg[150];
    unsigned long homingDurationMs = timeDiff(millis(), homingState.homingStartTime);
    sprintf_P(msg, FMT_HOMING_COMPLETED, motorName);
    Console.serialInfo(msg);
    printHumanReadableTime(homingDurationMs / 1000);
    Console.println();
}

void resetHomingState(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    MotorHomingState& homingState = getHomingState(rail);
    
    // Reset all homing state variables
    homingState.homingInProgress = false;
    homingState.hlfbWentNonAsserted = false;
    homingState.hlfbNonAssertedTime = 0;
    homingState.minDistanceTraveled = false;
    homingState.lastPositionCheckTime = 0;
    homingState.minTimeAfterDistanceReached = 0;
    homingState.pulsesTraveledAfterMinDistance = 0;
    homingState.positionAtMinDistance = 0;
    homingState.lastCheckedPosition = 0;

    // Capture the current position AFTER resetting variables
    homingState.startPulses = motor.PositionRefCommanded();

    char msg[100];
    sprintf_P(msg, FMT_HOMING_STATE_RESET, getMotorName(rail));
    Console.serialDiagnostic(msg);
}

void abortHoming(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    MotorHomingState& homingState = getHomingState(rail);
    
    if (homingState.homingInProgress)
    {
        char msg[100];
        sprintf_P(msg, FMT_ABORTING_HOMING, motorName);
        Console.serialInfo(msg);
        
        motor.MoveStopAbrupt();

        // Reset to normal operation parameters (use loaded carriage speed as conservative default)
        int32_t normalVelPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
        int32_t normalAccelPps = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
        motor.VelMax(normalVelPps);
        motor.AccelMax(normalAccelPps);

        // Reset state
        resetHomingState(rail);
        
        // Update motor state tracking
        if (rail == 1) {
            rail1HomingInProgress = false;
            rail1MotorState = MOTOR_STATE_IDLE;
        } else {
            rail2HomingInProgress = false;
            rail2MotorState = MOTOR_STATE_IDLE;
        }

        sprintf_P(msg, FMT_HOMING_ABORTED, motorName);
        Console.serialInfo(msg);
    }
}

bool isHomingComplete(int rail)
{
    MotorHomingState& homingState = getHomingState(rail);
    return homingState.isHomed && !homingState.homingInProgress;
}

bool isHomingInProgress(int rail)
{
    MotorHomingState& homingState = getHomingState(rail);
    return homingState.homingInProgress;
}

// Convenience functions for both motors
bool initiateHomingSequenceAll()
{
    Console.serialInfo("Initiating homing sequence for both motors");
    bool rail1Success = initiateHomingSequence(1);
    bool rail2Success = initiateHomingSequence(2);
    return rail1Success && rail2Success;
}

void checkAllHomingProgress()
{
    checkHomingProgress(1);
    checkHomingProgress(2);
}

bool isAllHomingComplete()
{
    return isHomingComplete(1) && isHomingComplete(2);
}

//=============================================================================
// MOVEMENT PROGRESS MONITORING
//=============================================================================

// Initialize movement tracking for a specific rail
void initMovementTracking(int rail, PositionTarget target, bool carriageLoaded) {
    MotorTargetState& state = getTargetState(rail);
    MotorDriver& motor = getMotorByRail(rail);
    
    // Reset and initialize tracking state
    memset(&state, 0, sizeof(MotorTargetState));
    
    state.movementInProgress = true;
    state.targetPosition = target;
    state.targetPositionPulses = getPositionPulses(target);
    state.startPositionPulses = motor.PositionRefCommanded();
    state.totalMoveDistancePulses = abs(state.targetPositionPulses - state.startPositionPulses);
    // Store the rail-specific base velocity that would be used for this type of movement
    state.originalVelocityRpm = getCarriageVelocityRpm(rail, carriageLoaded);
    state.carriageLoaded = carriageLoaded;
    state.movementStartTime = millis();
    state.lastProgressCheck = millis();
    state.lastDecelerationUpdate = millis();
    state.lastPositionCheck = state.startPositionPulses;
    state.decelerationActive = false;
    state.movementValidated = false;
    state.movementTimeoutCount = 0;
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, PSTR("Movement tracking initialized - Rail %d to %s (%.2fmm)"), 
              rail, getPositionName(target), pulsesToMm(state.targetPositionPulses, rail));
    Console.serialInfo(msg);
}

// Reset movement tracking for a specific rail
void resetMovementTracking(int rail) {
    MotorTargetState& state = getTargetState(rail);
    memset(&state, 0, sizeof(MotorTargetState));
}

// Validate that movement completed successfully
bool validateMovementCompletion(int rail) {
    MotorTargetState& state = getTargetState(rail);
    MotorDriver& motor = getMotorByRail(rail);
    
    if (!state.movementInProgress) {
        return true; // No movement to validate
    }
    
    int32_t currentPosition = motor.PositionRefCommanded();
    double positionErrorMm = abs(pulsesToMm(currentPosition - state.targetPositionPulses, rail));
    
    char msg[MEDIUM_MSG_SIZE];
    if (positionErrorMm <= MOVEMENT_POSITION_TOLERANCE_MM) {
        sprintf_P(msg, PSTR("Rail %d: Movement validated - Target reached within %.1fmm tolerance"), 
                  rail, MOVEMENT_POSITION_TOLERANCE_MM);
        Console.serialInfo(msg);
        state.movementValidated = true;
        return true;
    } else {
        sprintf_P(msg, PSTR("Rail %d: Movement validation FAILED - Position error: %.2fmm (tolerance: %.1fmm)"), 
                  rail, positionErrorMm, MOVEMENT_POSITION_TOLERANCE_MM);
        Console.error(msg);
        return false;
    }
}

// Update velocity for real-time deceleration
void updateDecelerationVelocity(int rail) {
    MotorTargetState& state = getTargetState(rail);
    MotorDriver& motor = getMotorByRail(rail);
    unsigned long currentTime = millis();
    
    // Track current velocity setting for this rail
    static int32_t rail1CurrentVelocityPps = 0;
    static int32_t rail2CurrentVelocityPps = 0;
    int32_t* currentVelocityPps = (rail == 1) ? &rail1CurrentVelocityPps : &rail2CurrentVelocityPps;
    
    // Only update at specified intervals to avoid overwhelming the motor
    if (currentTime - state.lastDecelerationUpdate < DECELERATION_UPDATE_INTERVAL_MS) {
        return;
    }
    
    state.lastDecelerationUpdate = currentTime;
    
    if (!state.movementInProgress || !getDecelerationConfig(rail).enableDeceleration) {
        return;
    }
    
    int32_t currentPosition = motor.PositionRefCommanded();
    int32_t distanceToTargetPulses = abs(state.targetPositionPulses - currentPosition);
    double distanceToTargetMm = pulsesToMm(distanceToTargetPulses, rail);
    double totalMoveDistanceMm = pulsesToMm(state.totalMoveDistancePulses, rail);
    
    // Calculate deceleration distance based on move length
    int32_t decelerationDistanceMm = getDecelerationDistanceScaled(rail, (int32_t)(totalMoveDistanceMm * 100)) / 100;
    
    if (distanceToTargetMm <= decelerationDistanceMm) {
        // Enter or continue deceleration
        if (!state.decelerationActive) {
            state.decelerationActive = true;
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("Rail %d: Entering deceleration phase - %.2fmm from target"), 
                      rail, distanceToTargetMm);
            Console.serialInfo(msg);
        }
        
        int32_t maxVelocityRpm = getCarriageVelocityRpm(rail, state.carriageLoaded);
        int32_t newVelocityRpm = calculateDeceleratedVelocity(rail, (int32_t)distanceToTargetMm, 
                                                              (int32_t)totalMoveDistanceMm, maxVelocityRpm);
        
        int32_t newVelocityPps = rpmToPps(newVelocityRpm);
        
        // Only update if velocity changed significantly (avoid constant updates)
        if (abs(*currentVelocityPps - newVelocityPps) > rpmToPps(5)) { // 5 RPM threshold
            motor.VelMax(newVelocityPps);
            *currentVelocityPps = newVelocityPps;
        }
    }
}

// Check for movement timeout and stall conditions
bool checkMovementTimeout(int rail) {
    MotorTargetState& state = getTargetState(rail);
    unsigned long currentTime = millis();
    char msg[MEDIUM_MSG_SIZE];
    
    if (!state.movementInProgress) {
        return false;
    }
    
    // Check for overall timeout
    if (currentTime - state.movementStartTime > MOVEMENT_TIMEOUT_MS) {
        sprintf_P(msg, PSTR("Rail %d: Movement TIMEOUT after %lu ms - Stopping motor"), 
                  rail, currentTime - state.movementStartTime);
        Console.error(msg);
        stopMotion(rail);
        resetMovementTracking(rail);
        return true;
    }
    
    // Check for stall conditions (position not changing)
    if (currentTime - state.lastProgressCheck > MOVEMENT_STALL_CHECK_INTERVAL_MS) {
        MotorDriver& motor = getMotorByRail(rail);
        int32_t currentPosition = motor.PositionRefCommanded();
        double progressMm = abs(pulsesToMm(currentPosition - state.lastPositionCheck, rail));
        
        if (progressMm < MOVEMENT_MIN_PROGRESS_MM) {
            state.movementTimeoutCount++;
            
            if (state.movementTimeoutCount >= 3) {
                sprintf_P(msg, PSTR("Rail %d: Movement STALLED - No progress for %d checks - Stopping motor"), 
                          rail, state.movementTimeoutCount);
                Console.error(msg);
                stopMotion(rail);
                resetMovementTracking(rail);
                return true;
            } else {
                sprintf_P(msg, PSTR("Rail %d: Warning - Minimal progress detected (%.2fmm in %dms)"), 
                          rail, progressMm, MOVEMENT_STALL_CHECK_INTERVAL_MS);
                Console.serialInfo(msg);
            }
        } else {
            // Reset timeout counter if good progress is made
            state.movementTimeoutCount = 0;
        }
        
        state.lastProgressCheck = currentTime;
        state.lastPositionCheck = currentPosition;
    }
    
    return false;
}

// Check progress for a specific rail
bool checkMovementProgress(int rail) {
    MotorTargetState& state = getTargetState(rail);
    bool isCurrentlyMoving = isMotorMoving(rail);
    
    if (!state.movementInProgress) {
        return false; // No movement to track
    }
    
    // Check for timeout and stall conditions
    if (checkMovementTimeout(rail)) {
        return true; // Movement was stopped due to timeout/stall
    }
    
    // Update deceleration velocity if still moving
    if (isCurrentlyMoving) {
        updateDecelerationVelocity(rail);
        return false; // Still moving
    }
    
    // Movement completed - validate and cleanup
    bool validationPassed = validateMovementCompletion(rail);
    
    // Restore original velocity if it was modified during deceleration
    if (state.decelerationActive) {
        MotorDriver& motor = getMotorByRail(rail);
        int32_t originalVelocityPps = rpmToPps(state.originalVelocityRpm);
        motor.VelMax(originalVelocityPps);
        
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("Rail %d: Velocity restored to %d RPM after deceleration"), 
                  rail, state.originalVelocityRpm);
        Console.serialInfo(msg);
    }
    
    // Generate completion message
    char msg[MEDIUM_MSG_SIZE];
    double finalPositionMm = getMotorPositionMm(rail);
    unsigned long totalTime = millis() - state.movementStartTime;
    sprintf_P(msg, PSTR("Rail %d: Movement completed in %lums - Final position: %.2fmm %s"), 
              rail, totalTime, finalPositionMm, validationPassed ? "(VALIDATED)" : "(VALIDATION FAILED)");
    Console.serialInfo(msg);
    
    // Reset tracking state
    resetMovementTracking(rail);
    
    return true; // Movement completed
}

void checkMoveProgress()
{
    char msg[MEDIUM_MSG_SIZE];
    static bool rail1WasMoving = false;
    static bool rail2WasMoving = false;
    
    // Check each rail independently using enhanced movement tracking
    for (int rail = 1; rail <= 2; rail++) {
        MotorTargetState& targetState = getTargetState(rail);
        bool* wasMoving = (rail == 1) ? &rail1WasMoving : &rail2WasMoving;
        bool isCurrentlyMoving = isMotorMoving(rail);
        
        // If we have active target tracking, use the enhanced monitoring
        if (targetState.movementInProgress) {
            // Use enhanced movement tracking with deceleration and validation
            bool movementCompleted = checkMovementProgress(rail);
            
            if (movementCompleted) {
                *wasMoving = false;
            }
        } else {
            // Fallback to basic movement tracking for movements without target tracking
            if (isCurrentlyMoving && !*wasMoving) {
                // Movement started without target tracking
                sprintf_P(msg, PSTR("%s: Movement started (basic tracking)"), getMotorName(rail));
                Console.serialInfo(msg);
                *wasMoving = true;
            } else if (*wasMoving && !isCurrentlyMoving) {
                // Movement completed without target tracking
                sprintf_P(msg, PSTR("%s: Movement completed - Position: %.2fmm"), 
                         getMotorName(rail), getMotorPositionMm(rail));
                Console.serialInfo(msg);
                *wasMoving = false;
            }
        }
    }
    
    // Global operation state management
    extern bool operationInProgress;
    bool anyMotorMoving = isMotorMoving(1) || isMotorMoving(2);
    bool anyTrackingActive = getTargetState(1).movementInProgress || getTargetState(2).movementInProgress;
    
    // Clear global operation flag when all movements complete
    if (operationInProgress && !anyMotorMoving && !rail1WasMoving && !rail2WasMoving && !anyTrackingActive) {
        operationInProgress = false;
        Console.serialInfo(F("All rail operations completed - system ready"));
    }
}

//=============================================================================
// MANUAL JOGGING OPERATIONS
//=============================================================================

bool jogMotor(int rail, bool direction, double customIncrement, bool carriageLoaded)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[200];

    // Check if motor is ready
    if (!isMotorReady(rail))
    {
        sprintf_P(msg, FMT_NOT_READY_JOG, motorName);
        Console.serialError(msg);
        return false;
    }

    // Check if motor is currently moving
    if (isMotorMoving(rail))
    {
        sprintf_P(msg, FMT_CANNOT_JOG_MOVING, motorName);
        Console.serialError(msg);
        return false;
    }

    // Determine increment to use
    double increment = (customIncrement > 0) ? customIncrement : getJogIncrementRef(rail);

    // Calculate movement direction (positive or negative)
    double moveMm = direction ? increment : -increment;

    // Get current position and validate move
    double currentPosMm = getMotorPositionMm(rail);
    double targetPositionMm = currentPosMm + moveMm;
    double maxTravel = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;

    // Safety check - ensure jog won't exceed travel limits
    if (targetPositionMm < 0 || targetPositionMm > maxTravel)
    {
        sprintf_P(msg, FMT_JOG_EXCEED_LIMITS, motorName, maxTravel);
        Console.serialError(msg);
        sprintf_P(msg, FMT_JOG_POSITION_INFO, 
                motorName, currentPosMm, targetPositionMm);
        Console.serialInfo(msg);
        return false;
    }

    // Save current jog velocity setting for restoration
    int32_t originalVelMax = rpmToPps(getJogSpeedRef(rail));

    // Use setJogSpeed to apply velocity scaling if using custom increment
    int jogSpeed = getJogSpeedRef(rail);
    if (customIncrement > 0)
    {
        // This will automatically cap the speed based on the distance
        setJogSpeed(rail, jogSpeed, customIncrement);
        jogSpeed = getJogSpeedRef(rail); // Get the potentially capped speed
    }

    // Apply distance-based velocity capping for safety
    int32_t jogVelocity = rpmToPps(jogSpeed);
    
    // Additional safety: for very short jogs, reduce speed further
    if (increment < 1.0)
    {
        jogVelocity = rpmToPps(min(jogSpeed, 100)); // Cap at 100 RPM for sub-mm moves
    }

    // Set velocity for jog
    motor.VelMax(jogVelocity);

    // Log the jog operation
    sprintf_P(msg, FMT_JOGGING, 
            motorName, direction ? "forward" : "backward", increment,
            (int)ppsToRpm(jogVelocity), carriageLoaded ? "[LOADED]" : "[EMPTY]");
    Console.serialInfo(msg);

    // Perform the relative move
    int32_t relativePulses = mmToPulses(moveMm, rail);
    motor.Move(relativePulses, MotorDriver::MOVE_TARGET_REL_END_POSN);

    return true;
}

bool setJogIncrement(int rail, double increment)
{
    const char* motorName = getMotorName(rail);
    char msg[200];

    // Validate increment is reasonable for the specific rail using header constants
    double maxIncrement = (rail == 1) ? RAIL1_MAX_JOG_INCREMENT_MM : RAIL2_MAX_JOG_INCREMENT_MM;
    
    if (increment <= 0 || increment > maxIncrement)
    {
        sprintf_P(msg, FMT_JOG_INCREMENT_RANGE, motorName, maxIncrement);
        Console.serialError(msg);
        return false;
    }

    // Set the increment
    getJogIncrementRef(rail) = increment;
    sprintf_P(msg, FMT_JOG_INCREMENT_SET, motorName, increment);
    Console.serialInfo(msg);

    // Re-validate jog speed with the new distance
    setJogSpeed(rail, getJogSpeedRef(rail), increment);

    return true;
}

bool setJogSpeed(int rail, int speedRpm, double jogDistanceMm)
{
    const char* motorName = getMotorName(rail);
    char msg[200];

    // Get the jog distance - either from parameter or use current increment
    double distanceToMoveMm = (jogDistanceMm > 0) ? jogDistanceMm : getJogIncrementRef(rail);

    // Validate speed is reasonable - use rail-specific empty carriage speed as max for jogging
    int maxSpeed = (rail == 1) ? RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM : RAIL2_EMPTY_CARRIAGE_VELOCITY_RPM;
    if (speedRpm < 10 || speedRpm > maxSpeed)
    {
        sprintf_P(msg, FMT_JOG_SPEED_RANGE, motorName, maxSpeed);
        Console.serialError(msg);
        return false;
    }

    // Apply distance-based speed caps for safety
    int cappedSpeed = speedRpm;

    // Use your existing velocity selection logic for safety
    if (distanceToMoveMm < 1.0)
    {
        // Very short moves - extra conservative
        cappedSpeed = min(speedRpm, 50);
        if (cappedSpeed != speedRpm)
        {
            sprintf_P(msg, FMT_SPEED_CAPPED_VERY_SHORT, 
                    motorName, cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }
    else if (distanceToMoveMm < 10.0)
    {
        // Short moves
        cappedSpeed = min(speedRpm, 150);
        if (cappedSpeed != speedRpm)
        {
            sprintf_P(msg, FMT_SPEED_CAPPED_SHORT, 
                    motorName, cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }
    else if (distanceToMoveMm < 50.0)
    {
        // Medium moves
        cappedSpeed = min(speedRpm, 250);
        if (cappedSpeed != speedRpm)
        {
            sprintf_P(msg, FMT_SPEED_CAPPED_MEDIUM, 
                    motorName, cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }
    else
    {
        // Long moves - use requested speed up to max
        cappedSpeed = min(speedRpm, maxSpeed);
        if (cappedSpeed != speedRpm)
        {
            sprintf_P(msg, FMT_SPEED_CAPPED_LONG, 
                    motorName, cappedSpeed, distanceToMoveMm);
            Console.serialInfo(msg);
        }
    }

    // Set the speed with the potentially capped value
    getJogSpeedRef(rail) = cappedSpeed;
       sprintf_P(msg, FMT_JOG_SPEED_SET, motorName, cappedSpeed);
    Console.serialInfo(msg);

    return true;
}

// Helper functions to get current jog parameters
double getJogIncrement(int rail)
{
    return getJogIncrementRef(rail);
}

int getJogSpeed(int rail)
{
    return getJogSpeedRef(rail);
}

// Convenience functions for common jog operations
bool jogForward(int rail, bool carriageLoaded = false)
{
    return jogMotor(rail, true, 0, carriageLoaded);
}

bool jogBackward(int rail, bool carriageLoaded = false)
{
    return jogMotor(rail, false, 0, carriageLoaded);
}

// Quick jog functions with predefined increments
bool jogForwardMm(int rail, double distanceMm, bool carriageLoaded = false)
{
    return jogMotor(rail, true, distanceMm, carriageLoaded);
}

bool jogBackwardMm(int rail, double distanceMm, bool carriageLoaded = false)
{
    return jogMotor(rail, false, distanceMm, carriageLoaded);
}

