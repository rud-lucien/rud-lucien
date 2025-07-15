#include "MotorController.h"
#include "Utils.h"
#include "LabwareAutomation.h"

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_MOTOR_ALERT_DETAILS[] PROGMEM = "%s Alert Details:\n%s";
const char FMT_INITIALIZING[] PROGMEM = "Initializing %s...";
const char FMT_ALERT_DETECTED[] PROGMEM = "%s: Motor alert detected";
const char FMT_INIT_SUCCESS[] PROGMEM = "%s: Initialization complete - Ready at %d RPM/%d RPM/s";
const char FMT_INIT_FAILED[] PROGMEM = "%s: Initialization failed - %s";
const char FMT_MOTION_STOPPED[] PROGMEM = "%s motion stopped";
const char FMT_CLEARING_FAULTS[] PROGMEM = "Attempting to clear %s faults...";
const char FMT_ALERTS_DETECTED[] PROGMEM = "%s alerts detected:";
const char FMT_NO_ALERTS[] PROGMEM = "%s: No alerts to clear.";
const char FMT_MOTOR_FAULTED[] PROGMEM = "%s faulted. Cycling enable signal...";
const char FMT_CLEARING_ALERTS[] PROGMEM = "Clearing %s alerts...";
const char FMT_ALERTS_PERSIST[] PROGMEM = "%s: Alerts are still present after clearing.";
const char FMT_ALERTS_CLEARED[] PROGMEM = "%s: Alerts successfully cleared.";
const char FMT_INVALID_POSITION[] PROGMEM = "%s: Invalid position for rail";
const char FMT_MOTOR_CANNOT_MOVE[] PROGMEM = "%s: Motor alert detected. Cannot move.";
const char FMT_NOT_READY_HOMING[] PROGMEM = "%s: Motor not ready for homing";
const char FMT_ALERTS_BEFORE_HOMING[] PROGMEM = "%s: Motor has active alerts - clear faults before homing";
const char FMT_ALERT_DURING_HOMING[] PROGMEM = "%s: Motor alert during homing";
const char FMT_MOTOR_STATUS[] PROGMEM = "[INFO] %s Status:\n  Enabled: %s\n  Homed: %s\n  Moving: %s\n  Position: %ld pulses (%.1f mm)\n  HLFB Status: %s\n  %s";

// Homing operation format strings - streamlined for operator clarity
const char FMT_HOMING_INITIATED[] PROGMEM = "%s: Homing sequence initiated";
const char FMT_MINIMAL_MOVEMENT[] PROGMEM = "%s: Minimal movement detected during homing";
const char FMT_HOMING_TIMEOUT[] PROGMEM = "%s: Homing operation timed out";
const char FMT_FINAL_HLFB_STATE[] PROGMEM = "%s: Final HLFB state: %s";
const char FMT_SET_HOME_TIMEOUT[] PROGMEM = "%s: Setting current position as home reference despite timeout";
const char FMT_HARDSTOP_REACHED[] PROGMEM = "%s: Hardstop reached, establishing home position";
const char FMT_MOVING_FROM_HARDSTOP[] PROGMEM = "%s: Moving %.2fmm away from hardstop";
const char FMT_ALERT_OFFSET_MOVE[] PROGMEM = "%s: Alert during offset move";
const char FMT_OFFSET_TIMEOUT[] PROGMEM = "%s: Offset move timed out or failed to complete";
const char FMT_HOME_OFFSET_ESTABLISHED[] PROGMEM = "%s: Home offset established as zero position";
const char FMT_HARDSTOP_AS_ZERO[] PROGMEM = "%s: Hardstop established as zero position (no offset)";
const char FMT_HOMING_COMPLETED[] PROGMEM = "%s: Homing sequence completed successfully in ";
const char FMT_ABORTING_HOMING[] PROGMEM = "%s: Aborting homing operation";
const char FMT_HOMING_ABORTED[] PROGMEM = "%s: Homing operation aborted successfully";

// Jogging operation format strings
const char FMT_NOT_READY_JOG[] PROGMEM = "%s: Motor not ready for jogging";
const char FMT_CANNOT_JOG_MOVING[] PROGMEM = "%s: Cannot jog while motor is moving";
const char FMT_JOG_EXCEED_LIMITS[] PROGMEM = "%s: Jog would exceed travel limits (0 to %.1fmm)";
const char FMT_JOG_POSITION_INFO[] PROGMEM = "%s: Current: %.2fmm, Target would be: %.2fmm";
const char FMT_JOGGING[] PROGMEM = "%s: Jogging %s %.2fmm at %d RPM%s";
const char FMT_JOG_INCREMENT_RANGE[] PROGMEM = "%s: Jog increment must be between 0 and %.1fmm";
const char FMT_JOG_INCREMENT_SET[] PROGMEM = "%s: Jog increment set to %.2fmm";
const char FMT_JOG_SPEED_RANGE[] PROGMEM = "%s: Jog speed must be between 10 and %d RPM";
const char FMT_JOG_SPEED_SET[] PROGMEM = "%s: Jog speed set to %d RPM";
const char FMT_JOG_SPEED_SET_WITH_DISTANCE[] PROGMEM = "%s: Jog speed set to %d RPM, increment set to %.2fmm";
const char FMT_MOVE_POSITIONED[] PROGMEM = "%s: %s→%s (%.1fmm) at %d RPM %s";
const char FMT_MOVE_TO_POSITION[] PROGMEM = "%s: Moving to %s (%.1fmm) at %d RPM %s";
const char FMT_POSITION_MM_OUT_OF_RANGE[] PROGMEM = "%s: Position %.2fmm outside valid range (0 to %.2fmm)";
const char FMT_MOVE_TO_MM[] PROGMEM = "%s: Moving to %.2fmm (%ld pulses) at %d RPM %s";
const char FMT_RELATIVE_MOVE_RANGE[] PROGMEM = "%s: Relative move would exceed valid range (0 to %.2fmm)";
const char FMT_RELATIVE_MOVE_DETAILS[] PROGMEM = "%s: Current: %.2fmm, Move: %.2fmm, Target would be: %.2fmm";
const char FMT_RELATIVE_MOVE[] PROGMEM = "%s: Moving %.2fmm relative (%ld pulses) at %d RPM %s";
const char FMT_INITIATE_HOMING[] PROGMEM = "Initiating homing sequence for %s";
const char FMT_POSITION_LIMIT_ERROR[] PROGMEM = "POSITION DEFINITION ERROR: %s position %.2fmm exceeds %s travel limit (0-%.0fmm)";

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

// Per-Motor Velocity and Acceleration Tracking
int32_t rail1CurrentVelMax = 0;
int32_t rail1CurrentAccelMax = 0;
int32_t rail2CurrentVelMax = 0;
int32_t rail2CurrentAccelMax = 0;

// Motor-Specific State Tracking
MotorState rail1MotorState = MOTOR_STATE_NOT_READY;
MotorState rail2MotorState = MOTOR_STATE_NOT_READY;

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

// Get homing timeout for specific rail
unsigned long getHomingTimeout(int rail) {
    return (rail == 1) ? RAIL1_HOME_TIMEOUT_MS : RAIL2_HOME_TIMEOUT_MS;
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

// Get smart homing constants in pulses for specific rail
int32_t getHomePrecisionDistancePulses(int rail) {
    return (rail == 1) ? HOME_PRECISION_DISTANCE_PULSES_RAIL1 : HOME_PRECISION_DISTANCE_PULSES_RAIL2;
}

int32_t getHomeMinDistancePulses(int rail) {
    return (rail == 1) ? HOME_MIN_DISTANCE_PULSES_RAIL1 : HOME_MIN_DISTANCE_PULSES_RAIL2;
}

// Helper function to set motor velocity and update per-motor tracking
void setMotorVelocity(int rail, int32_t velocityPps) {
    MotorDriver& motor = getMotorByRail(rail);
    motor.VelMax(velocityPps);
    
    // Update per-motor tracking
    if (rail == 1) {
        rail1CurrentVelMax = velocityPps;
    } else {
        rail2CurrentVelMax = velocityPps;
    }
}

// Helper function to get current motor velocity
int32_t getCurrentMotorVelocity(int rail) {
    return (rail == 1) ? rail1CurrentVelMax : rail2CurrentVelMax;
}

// Helper function to set motor acceleration and update per-motor tracking
void setMotorAcceleration(int rail, int32_t accelPpsPerSec) {
    MotorDriver& motor = getMotorByRail(rail);
    motor.AccelMax(accelPpsPerSec);
    
    // Update per-motor tracking
    if (rail == 1) {
        rail1CurrentAccelMax = accelPpsPerSec;
    } else {
        rail2CurrentAccelMax = accelPpsPerSec;
    }
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

// Global flags for interrupt-detected E-stop
volatile bool eStopInterruptTriggered = false;
volatile unsigned long eStopInterruptTime = 0;

// Interrupt Service Routine (ISR) for E-stop
// Keep minimal and fast - only set flags, no Serial.print or complex operations
void eStopInterruptHandler() {
    eStopInterruptTriggered = true;
    eStopInterruptTime = millis();
}

// Setup interrupt-based E-stop monitoring
void setupEStopInterrupt() {
    // Configure A-12 as digital input with pull-up (required for interrupt)
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    
    // Test if pin supports interrupts
    int interruptNum = digitalPinToInterrupt(E_STOP_PIN);
    if (interruptNum >= 0) {  // Valid interrupt numbers are >= 0
        // Attach interrupt on FALLING edge (normally-closed E-stop going LOW)
        attachInterrupt(interruptNum, eStopInterruptHandler, FALLING);
        
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("E-stop interrupt monitoring enabled on A-12 (interrupt #%d)"), interruptNum);
        Console.serialInfo(msg);
        Console.serialInfo(F("E-stop response time: <1ms (hardware interrupt)"));
    } else {
        Console.serialWarning(F("A-12 does not support interrupts - falling back to polling"));
        Console.serialWarning(F("E-stop response time: up to 10ms (polling)"));
    }
}

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
    static unsigned long lastInterruptDebounce = 0;

    // Handle interrupt-triggered E-stop with debouncing
    if (eStopInterruptTriggered) {
        unsigned long currentTime = millis();
        
        // Debounce the interrupt to prevent false triggers
        if (currentTime - lastInterruptDebounce > E_STOP_INTERRUPT_DEBOUNCE_MS) {
            // Verify E-stop is still active (not a false trigger from electrical noise)
            if (isEStopActive()) {
                Console.serialError(F("E-STOP TRIGGERED! (Hardware Interrupt)"));
                
                // Immediate emergency shutdown
                stopAllMotion();
                RAIL1_MOTOR.EnableRequest(false);
                RAIL2_MOTOR.EnableRequest(false);
                
                // Abort homing operations
                if (rail1HomingInProgress) {
                    Console.serialInfo(F("Aborting Rail 1 homing operation"));
                    abortHoming(1);
                }
                if (rail2HomingInProgress) {
                    Console.serialInfo(F("Aborting Rail 2 homing operation"));
                    abortHoming(2);
                }
                
                // Set motor states to faulted
                rail1MotorState = MOTOR_STATE_FAULTED;
                rail2MotorState = MOTOR_STATE_FAULTED;
                eStopWasActive = true;
            }
            lastInterruptDebounce = currentTime;
        }
        eStopInterruptTriggered = false;  // Clear the interrupt flag
    }

    // Keep existing periodic polling for status monitoring and E-stop release detection
    unsigned long currentTime = millis();
    if (!waitTimeReached(currentTime, lastEStopCheckTime, E_STOP_CHECK_INTERVAL_MS)) {
        return;
    }
    lastEStopCheckTime = currentTime;

    // Check for E-stop release (polling is sufficient for this)
    bool eStopActive = isEStopActive();
    
    // Handle E-stop activation via polling (backup if interrupt missed or not available)
    if (eStopActive && !eStopWasActive) {
        Console.serialError(F("E-STOP TRIGGERED! (Polling Backup)"));
        
        // Same emergency shutdown as interrupt handler
        stopAllMotion();
        RAIL1_MOTOR.EnableRequest(false);
        RAIL2_MOTOR.EnableRequest(false);
        
        if (rail1HomingInProgress) {
            Console.serialInfo(F("Aborting Rail 1 homing operation"));
            abortHoming(1);
        }
        if (rail2HomingInProgress) {
            Console.serialInfo(F("Aborting Rail 2 homing operation"));
            abortHoming(2);
        }
        
        rail1MotorState = MOTOR_STATE_FAULTED;
        rail2MotorState = MOTOR_STATE_FAULTED;
    }
    
    // Report when E-stop is released
    if (!eStopActive && eStopWasActive) {
        Console.serialInfo(F("E-STOP RELEASED - System remains in fault state until cleared"));
        Console.serialInfo(F("Use fault clearing commands to re-enable motors"));
    }

    eStopWasActive = eStopActive;
}

void printMotorAlerts(MotorDriver &motor, const char* motorName)
{
    char msg[LARGE_MSG_SIZE];
    char alertList[ALERT_MSG_SIZE] = "";
    size_t remainingSpace = ALERT_MSG_SIZE - 1; // Reserve space for null terminator
    
    // Helper macro for safe concatenation with bounds checking
    #define SAFE_STRCAT_P(dest, src) do { \
        size_t currentLen = strlen(dest); \
        size_t srcLen = strlen_P(src); \
        if (currentLen + srcLen < remainingSpace) { \
            strncat_P(dest, src, remainingSpace - currentLen); \
        } else { \
            /* Truncation indicator if buffer would overflow */ \
            if (remainingSpace > currentLen + 4) { \
                strncat_P(dest, PSTR("...\n"), remainingSpace - currentLen); \
            } \
            goto alert_concatenation_complete; \
        } \
    } while(0)
    
    if (motor.AlertReg().bit.MotionCanceledInAlert)
        SAFE_STRCAT_P(alertList, ALERT_MOTION_CANCELED_IN_ALERT);
    if (motor.AlertReg().bit.MotionCanceledPositiveLimit)
        SAFE_STRCAT_P(alertList, ALERT_MOTION_CANCELED_POSITIVE_LIMIT);
    if (motor.AlertReg().bit.MotionCanceledNegativeLimit)
        SAFE_STRCAT_P(alertList, ALERT_MOTION_CANCELED_NEGATIVE_LIMIT);
    if (motor.AlertReg().bit.MotionCanceledSensorEStop)
        SAFE_STRCAT_P(alertList, ALERT_MOTION_CANCELED_SENSOR_ESTOP);
    if (motor.AlertReg().bit.MotionCanceledMotorDisabled)
        SAFE_STRCAT_P(alertList, ALERT_MOTION_CANCELED_MOTOR_DISABLED);
    if (motor.AlertReg().bit.MotorFaulted)
        SAFE_STRCAT_P(alertList, ALERT_MOTOR_FAULTED);

alert_concatenation_complete:
    #undef SAFE_STRCAT_P

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

    // Pre-initialization alert clearing: Check for existing alerts and attempt to clear them
    if (motor.StatusReg().bit.AlertsPresent)
    {
        sprintf_P(msg, PSTR("%s: Pre-initialization alerts detected, attempting to clear..."), motorName);
        Console.serialInfo(msg);
        printMotorAlerts(motor, motorName);
        
        // Use E-stop safe alert clearing instead of unsafe blocking delays
        if (!clearAlertsWithEStopMonitoring(motor, motorName))
        {
            sprintf_P(msg, PSTR("%s: Pre-initialization alert clearing failed or E-stop triggered"), motorName);
            Console.serialError(msg);
            return false;
        }
        else
        {
            sprintf_P(msg, PSTR("%s: Pre-initialization alerts successfully cleared"), motorName);
            Console.serialInfo(msg);
        }
    }

    // Set the motor's HLFB mode to bipolar PWM
    motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Set velocity and acceleration limits (silently - details not critical for operator)
    int32_t velMax = rpmToPps(velocityRpm);
    int32_t accelMax = rpmPerSecToPpsPerSec(accelRpmPerSec);
    
    motor.VelMax(velMax);
    motor.AccelMax(accelMax);
    
    // Update per-motor tracking (determine rail from motor reference)
    if (&motor == &RAIL1_MOTOR) {
        rail1CurrentVelMax = velMax;
        rail1CurrentAccelMax = accelMax;
    } else if (&motor == &RAIL2_MOTOR) {
        rail2CurrentVelMax = velMax;
        rail2CurrentAccelMax = accelMax;
    }

    // Enable the motor and wait for HLFB to assert (up to 2 seconds)
    motor.EnableRequest(true);
    
    unsigned long startTime = millis();
    bool ready = false;
    int alertRetryCount = 0;
    const int MAX_ALERT_RETRIES = 2;

    while (!ready && !timeoutElapsed(millis(), startTime, MOTOR_INIT_TIMEOUT_MS))
    {
        if (motor.HlfbState() == MotorDriver::HLFB_ASSERTED)
        {
            ready = true;
        }
        else if (motor.StatusReg().bit.AlertsPresent)
        {
            sprintf_P(msg, FMT_ALERT_DETECTED, motorName);
            Console.serialWarning(msg);
            printMotorAlerts(motor, motorName);
            
            // Attempt to clear alerts during initialization if retries remain
            if (alertRetryCount < MAX_ALERT_RETRIES)
            {
                alertRetryCount++;
                sprintf_P(msg, PSTR("%s: Attempting alert clearing during initialization (attempt %d/%d)"), 
                         motorName, alertRetryCount, MAX_ALERT_RETRIES);
                Console.serialInfo(msg);
                
                // Use E-stop safe alert clearing instead of unsafe blocking delays
                if (!clearAlertsWithEStopMonitoring(motor, motorName))
                {
                    sprintf_P(msg, PSTR("%s: Alert clearing failed or E-stop triggered during initialization"), motorName);
                    Console.serialError(msg);
                    break;
                }
                
                // Reset start time to give full timeout period after each retry
                startTime = millis();
            }
            else
            {
                sprintf_P(msg, PSTR("%s: Maximum alert retry attempts exceeded during initialization"), motorName);
                Console.serialError(msg);
                break;
            }
        }
        delay(MOTOR_INIT_POLL_DELAY_MS);
    }

    if (ready)
    {
        sprintf_P(msg, FMT_INIT_SUCCESS, motorName, velocityRpm, accelRpmPerSec);
        Console.serialInfo(msg);
        return true;
    }
    else
    {
        const char* failureReason = motor.StatusReg().bit.AlertsPresent ? 
            "Motor alerts persist after clearing attempts" : "HLFB timeout - motor not responding";
        sprintf_P(msg, FMT_INIT_FAILED, motorName, failureReason);
        Console.serialError(msg);
        return false;
    }
}

//=============================================================================
// E-STOP INITIALIZATION
//=============================================================================

bool initEStop()
{
    Console.serialInfo(F("Initializing E-stop system..."));
    
    // Set up interrupt-based E-stop monitoring
    setupEStopInterrupt();
    
    if (isEStopActive())
    {
        Console.serialError(F("E-STOP ACTIVE! Please reset E-stop before continuing."));
        return false;
    }
    else
    {
        Console.serialInfo(F("E-stop inactive, system ready."));
        return true;
    }
}

//=============================================================================
// RAIL-SPECIFIC MOTOR INITIALIZATION
//=============================================================================

bool initRailMotor(int railNumber)
{
    char msg[SMALL_MSG_SIZE];
    sprintf_P(msg, PSTR("Initializing Rail %d motor..."), railNumber);
    Console.serialInfo(msg);
    
    MotorDriver* motor;
    MotorState* motorState;
    int32_t velocityRpm;
    const char* railName;
    
    // Select rail-specific parameters
    if (railNumber == 1) {
        motor = &RAIL1_MOTOR;
        motorState = &rail1MotorState;
        velocityRpm = RAIL1_LOADED_CARRIAGE_VELOCITY_RPM;
        railName = "Rail 1";
    } else if (railNumber == 2) {
        motor = &RAIL2_MOTOR;
        motorState = &rail2MotorState;
        velocityRpm = RAIL2_LOADED_CARRIAGE_VELOCITY_RPM;
        railName = "Rail 2";
    } else {
        Console.serialError(F("Invalid rail number. Must be 1 or 2."));
        return false;
    }
    
    // Initialize the motor with rail-specific settings
    bool ready = initSingleMotor(*motor, railName, velocityRpm, MAX_ACCEL_RPM_PER_SEC);
    
    if (ready) {
        *motorState = MOTOR_STATE_IDLE;
        sprintf_P(msg, PSTR("Rail %d motor initialized and ready"), railNumber);
        Console.serialInfo(msg);
        return true;
    } else {
        *motorState = MOTOR_STATE_FAULTED;
        sprintf_P(msg, PSTR("Rail %d motor initialization failed"), railNumber);
        Console.serialError(msg);
        return false;
    }
}

bool initMotorManager()
{
    Console.serialInfo(F("Initializing motor manager configuration..."));

    // Initialize E-stop system first
    if (!initEStop()) {
        Console.serialError(F("E-stop initialization failed - motor system cannot start"));
        return false;
    }

    // Set the input clocking rate
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Configure motor connector for step and direction mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    Console.serialInfo(F("Motor manager configuration complete"));
    return true;
}

//=============================================================================
// UNIT CONVERSION UTILITIES
//=============================================================================

int32_t rpmToPps(double rpm)
{
    return (int32_t)((rpm * PULSES_PER_REV) / 60.0);
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
    // For both rails: (mmScaled * pulses_per_100mm) / 100
    // mmScaled is mm * 100, pulses_per_100mm is pulses per 100mm
    // So: (mm * 100 * pulses_per_100mm) / 100 = mm * pulses_per_100mm / 100
    
    if (rail == 1) {
        return (mmScaled * RAIL1_PULSES_PER_MM_SCALED) / 10000; // 100 * 100
    } else {
        return (mmScaled * RAIL2_PULSES_PER_MM_SCALED) / 10000;
    }
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
    // First check if position belongs to the correct rail
    if (getRailFromPosition(target) != rail) {
        return false;
    }
    
    // Get the position in millimeters for travel limit validation
    int32_t targetPulses = getPositionPulses(target);
    double targetMm = pulsesToMm(targetPulses, rail);
    
    // Get rail-specific max travel limit
    double maxTravelMm = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    // Validate position is within travel limits (0 to maxTravel)
    if (targetMm < 0 || targetMm > maxTravelMm) {
        // Log detailed error for debugging predefined position issues
        char msg[LARGE_MSG_SIZE];
        sprintf_P(msg, FMT_POSITION_LIMIT_ERROR,
                 getPositionName(target), targetMm, getMotorName(rail), maxTravelMm);
        Console.serialError(msg);
        return false;
    }
    
    return true;
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
    char msg[SMALL_MSG_SIZE];
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
            isHomingComplete(rail) ? "Yes" : "No",
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
    char msg[MEDIUM_MSG_SIZE];

    sprintf_P(msg, FMT_CLEARING_FAULTS, motorName);
    Console.serialDiagnostic(msg);

    if (motor.StatusReg().bit.AlertsPresent)
    {
        sprintf_P(msg, FMT_ALERTS_DETECTED, motorName);
        Console.serialDiagnostic(msg);
        printMotorAlerts(motor, motorName);

        // Use E-stop safe fault clearing - the RIGHT way
        if (clearAlertsWithEStopMonitoring(motor, motorName))
        {
            sprintf_P(msg, FMT_ALERTS_CLEARED, motorName);
            Console.serialInfo(msg);
        }
        else
        {
            sprintf_P(msg, FMT_ALERTS_PERSIST, motorName);
            Console.serialError(msg);
        }
    }
    else
    {
        sprintf_P(msg, FMT_NO_ALERTS, motorName);
        Console.serialInfo(msg);
    }
}

bool clearMotorFaultWithStatus(int rail)
{
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    
    bool hadAlerts = motor.StatusReg().bit.AlertsPresent;
    
    if (hadAlerts)
    {
        // Use E-stop safe synchronous fault clearing
        if (clearAlertsWithEStopMonitoring(motor, motorName))
        {
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, FMT_ALERTS_CLEARED, motorName);
            Console.serialInfo(msg);
            return true; // Successfully cleared
        }
        else
        {
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, FMT_ALERTS_PERSIST, motorName);
            Console.serialError(msg);
            return false; // Clearing failed or E-stop triggered
        }
    }
    else
    {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_NO_ALERTS, motorName);
        Console.serialInfo(msg);
        return true; // No alerts to clear (immediate success)
    }
}

bool clearAlertsWithEStopMonitoring(MotorDriver &motor, const char* motorName)
{
    const unsigned long ENABLE_DISABLE_DELAY_MS = 100;
    const unsigned long CLEAR_ALERTS_DELAY_MS = 100;
    const unsigned long TOTAL_TIMEOUT_MS = 2000; // Maximum time to wait for clearing
    
    unsigned long startTime = millis();
    
    // Phase 1: Disable motor with E-stop monitoring
    motor.EnableRequest(false);
    unsigned long phaseStartTime = millis();
    
    while (!timeoutElapsed(millis(), phaseStartTime, ENABLE_DISABLE_DELAY_MS))
    {
        // Critical: Continue monitoring E-stop during delay
        handleEStop();
        
        // Check if E-stop was triggered during clearing
        if (isEStopActive())
        {
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("%s: E-stop triggered during alert clearing - aborting"), motorName);
            Console.serialError(msg);
            return false;
        }
        
        delay(10); // Small delay to prevent excessive CPU usage
    }
    
    // Phase 2: Re-enable motor with E-stop monitoring
    motor.EnableRequest(true);
    phaseStartTime = millis();
    
    while (!timeoutElapsed(millis(), phaseStartTime, ENABLE_DISABLE_DELAY_MS))
    {
        // Critical: Continue monitoring E-stop during delay
        handleEStop();
        
        if (isEStopActive())
        {
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("%s: E-stop triggered during alert clearing - aborting"), motorName);
            Console.serialError(msg);
            return false;
        }
        
        delay(10);
    }
    
    // Phase 3: Clear alerts with E-stop monitoring
    motor.ClearAlerts();
    phaseStartTime = millis();
    
    while (!timeoutElapsed(millis(), phaseStartTime, CLEAR_ALERTS_DELAY_MS))
    {
        // Critical: Continue monitoring E-stop during delay
        handleEStop();
        
        if (isEStopActive())
        {
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("%s: E-stop triggered during alert clearing - aborting"), motorName);
            Console.serialError(msg);
            return false;
        }
        
        delay(10);
    }
    
    // Verify clearing succeeded
    if (motor.StatusReg().bit.AlertsPresent)
    {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("%s: Alert clearing failed - alerts persist"), motorName);
        Console.serialError(msg);
        return false;
    }
    
    return true; // Success
}

//=============================================================================
// HOMING STATE AND PROGRESS FUNCTIONS
//=============================================================================

bool isHomingComplete(int rail) {
    MotorHomingState& homingState = getHomingState(rail);
    return homingState.isHomed && !homingState.homingInProgress;
}

bool isHomingInProgress(int rail) {
    MotorHomingState& homingState = getHomingState(rail);
    return homingState.homingInProgress;
}

void resetHomingState(int rail) {
    MotorHomingState& homingState = getHomingState(rail);
    homingState.homingInProgress = false;
    homingState.isHomed = false;
    homingState.hlfbWentNonAsserted = false;
    homingState.minDistanceTraveled = false;
    homingState.homingStartTime = 0;
    homingState.hlfbNonAssertedTime = 0;
    homingState.lastPositionCheckTime = 0;
    homingState.minTimeAfterDistanceReached = 0;
    homingState.startPulses = 0;
    homingState.lastCheckedPosition = 0;
    homingState.positionAtMinDistance = 0;
    homingState.pulsesTraveledAfterMinDistance = 0;
    homingState.previousEncoderState = false;
    
    // Reset global homing flags
    if (rail == 1) {
        rail1HomingInProgress = false;
    } else if (rail == 2) {
        rail2HomingInProgress = false;
    }
}

//=============================================================================
// HOMING OPERATIONS
//=============================================================================

bool initiateHomingSequence(int rail) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[SMALL_MSG_SIZE];
    
    // Check if motor is ready for homing
    if (!isMotorReady(rail)) {
        sprintf_P(msg, FMT_NOT_READY_HOMING, motorName);
        Console.serialError(msg);
        return false;
    }

    // Check for active alerts before starting
    if (motor.StatusReg().bit.AlertsPresent) {
        sprintf_P(msg, FMT_ALERTS_BEFORE_HOMING, motorName);
        Console.serialError(msg);
        printMotorAlerts(motor, motorName);
        return false;
    }

    // Reset homing state
    resetHomingState(rail);
    
    // Get rail-specific parameters
    int homingDirection = getHomingDirection(rail);
    unsigned long homingTimeout = getHomingTimeout(rail);
    
    // Initialize homing state
    MotorHomingState& homingState = getHomingState(rail);
    homingState.homingInProgress = true;
    homingState.homingStartTime = millis();
    homingState.startPulses = motor.PositionRefCommanded();
    homingState.lastCheckedPosition = homingState.startPulses;
    homingState.lastPositionCheckTime = millis();
    
    // Set global homing flag
    if (rail == 1) {
        rail1HomingInProgress = true;
    } else if (rail == 2) {
        rail2HomingInProgress = true;
    }
    
    // Set homing velocity and direction
    int32_t homingVelPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
    setMotorVelocity(rail, homingVelPps);
    
    // Move in homing direction (relative move to trigger HLFB change)
    int32_t maxTravelPulses = mmToPulses((rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM, rail);
    int32_t homingMovePulses = homingDirection * maxTravelPulses;
    motor.Move(homingMovePulses);
    
    sprintf_P(msg, FMT_HOMING_INITIATED, motorName);
    Console.serialInfo(msg);
    
    return true;
}

void checkHomingProgress(int rail) {
    if (!isHomingInProgress(rail)) {
        return;
    }
    
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    MotorHomingState& homingState = getHomingState(rail);
    unsigned long currentTime = millis();
    char msg[MEDIUM_MSG_SIZE];
    
    // Check for alerts during homing
    if (motor.StatusReg().bit.AlertsPresent) {
        sprintf_P(msg, FMT_ALERT_DURING_HOMING, motorName);
        Console.serialError(msg);
        printMotorAlerts(motor, motorName);
        abortHoming(rail);
        return;
    }
    
    // Check for timeout
    unsigned long homingTimeout = getHomingTimeout(rail);
    if (timeoutElapsed(currentTime, homingState.homingStartTime, homingTimeout)) {
        sprintf_P(msg, FMT_HOMING_TIMEOUT, motorName);
        Console.serialError(msg);
        completeHomingSequence(rail);
        return;
    }
    
    // Get current position and calculate movement
    int32_t currentPosition = motor.PositionRefCommanded();
    int32_t totalMovement = abs(currentPosition - homingState.startPulses);
    
    // Check for minimum distance traveled (diagnostic info reduced)
    if (!homingState.minDistanceTraveled && totalMovement >= HOMING_MIN_MOVEMENT_PULSES) {
        homingState.minDistanceTraveled = true;
        homingState.positionAtMinDistance = currentPosition;
        homingState.minTimeAfterDistanceReached = currentTime;
        // Silently enable hardstop detection - detailed progress not critical for operator
    }
    
    // Monitor HLFB state changes
    bool currentHlfbAsserted = (motor.HlfbState() == MotorDriver::HLFB_ASSERTED);
    
    // Detect HLFB going non-asserted (approaching hardstop)
    if (homingState.minDistanceTraveled && currentHlfbAsserted && !homingState.hlfbWentNonAsserted) {
        // We expect HLFB to be asserted initially, then go non-asserted as we approach hardstop
        // No action needed here, just monitoring
    } else if (homingState.minDistanceTraveled && !currentHlfbAsserted && !homingState.hlfbWentNonAsserted) {
        // HLFB went non-asserted - we're approaching the hardstop (internal tracking)
        homingState.hlfbWentNonAsserted = true;
        homingState.hlfbNonAssertedTime = currentTime;
        // Detailed HLFB transitions not critical for operator - tracked silently
    } else if (homingState.hlfbWentNonAsserted && currentHlfbAsserted) {
        // HLFB reasserted - we've reached the hardstop
        homingState.pulsesTraveledAfterMinDistance = abs(currentPosition - homingState.positionAtMinDistance);
        unsigned long hlfbDuration = timeDiff(currentTime, homingState.hlfbNonAssertedTime);
        
        sprintf_P(msg, FMT_HARDSTOP_REACHED, motorName);
        Console.serialInfo(msg);
        
        // Stop the motor and complete homing
        motor.MoveStopAbrupt();
        completeHomingSequence(rail);
        return;
    }
    
    // Update tracking
    homingState.lastCheckedPosition = currentPosition;
    homingState.lastPositionCheckTime = currentTime;
}

void completeHomingSequence(int rail) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    MotorHomingState& homingState = getHomingState(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    // Stop any ongoing motion
    motor.MoveStopAbrupt();
    
    // Wait for motion to complete
    unsigned long stopTime = millis();
    unsigned long homingTimeout = getHomingTimeout(rail);
    while (!motor.StepsComplete() && !timeoutElapsed(millis(), stopTime, homingTimeout)) {
        delay(10);
    }
    
    // Get offset distance and direction
    double offsetDistanceMm = getHomeOffsetDistance(rail);
    int homingDirection = getHomingDirection(rail);
    
    // Move away from hardstop if offset is needed
    if (offsetDistanceMm > 0) {
        sprintf_P(msg, FMT_MOVING_FROM_HARDSTOP, motorName, offsetDistanceMm);
        Console.serialInfo(msg);
        
        // Calculate offset move in opposite direction from homing
        int32_t offsetPulses = mmToPulses(offsetDistanceMm, rail) * (-homingDirection);
        motor.Move(offsetPulses);
        
        // Wait for offset move to complete (silently - detailed progress not critical)
        unsigned long offsetStartTime = millis();
        unsigned long homingTimeout = getHomingTimeout(rail);
        while (!motor.StepsComplete() && !timeoutElapsed(millis(), offsetStartTime, homingTimeout)) {
            if (motor.StatusReg().bit.AlertsPresent) {
                sprintf_P(msg, FMT_ALERT_OFFSET_MOVE, motorName);
                Console.serialError(msg);
                break;
            }
            delay(10);
        }
        
        if (!motor.StepsComplete()) {
            sprintf_P(msg, FMT_OFFSET_TIMEOUT, motorName);
            Console.serialWarning(msg);
        }
        
        sprintf_P(msg, FMT_HOME_OFFSET_ESTABLISHED, motorName);
        Console.serialInfo(msg);
    } else {
        sprintf_P(msg, FMT_HARDSTOP_AS_ZERO, motorName);
        Console.serialInfo(msg);
    }
    
    // Set current position as home (zero)
    motor.PositionRefSet(0);
    
    // Update homing state
    homingState.isHomed = true;
    homingState.homingInProgress = false;
    
    // Clear global homing flag
    if (rail == 1) {
        rail1HomingInProgress = false;
    } else if (rail == 2) {
        rail2HomingInProgress = false;
    }
    
    // Calculate and display homing duration
    unsigned long homingDuration = timeDiff(millis(), homingState.homingStartTime);
    sprintf_P(msg, FMT_HOMING_COMPLETED, motorName);
    Console.serialInfo(msg);
    printHumanReadableTime(homingDuration / 1000);
    Console.print("\n");
    
    // Automatically update labware state from sensors after successful homing
    performAutomaticLabwareDetectionOnHoming(rail);
    
    // Homing state reset happens silently - not critical operator information
}

void abortHoming(int rail) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[SMALL_MSG_SIZE];
    
    sprintf_P(msg, FMT_ABORTING_HOMING, motorName);
    Console.serialInfo(msg);
    
    // Stop motor immediately
    motor.MoveStopAbrupt();
    
    // Reset homing state
    resetHomingState(rail);
    
    sprintf_P(msg, FMT_HOMING_ABORTED, motorName);
    Console.serialInfo(msg);
}

//=============================================================================
// DUAL-MOTOR HOMING FUNCTIONS
//=============================================================================

bool initiateHomingSequenceAll() {
    bool rail1Success = initiateHomingSequence(1);
    bool rail2Success = initiateHomingSequence(2);
    return rail1Success && rail2Success;
}

void checkAllHomingProgress() {
    checkHomingProgress(1);
    checkHomingProgress(2);
}

bool isAllHomingComplete() {
    return isHomingComplete(1) && isHomingComplete(2);
}

//=============================================================================
// SMART HOMING FUNCTIONS (INTEGER MATH OPTIMIZED)
//=============================================================================

bool isSmartHomingBeneficial(int rail, int32_t* estimatedTimeSavingsMs) {
    MotorDriver& motor = getMotorByRail(rail);
    
    // Check if rail has been homed before
    MotorHomingState& homingState = getHomingState(rail);
    if (!homingState.isHomed) {
        if (estimatedTimeSavingsMs) *estimatedTimeSavingsMs = 0;
        return false; // First-time homing always uses standard approach
    }
    
    // Get current position in pulses
    int32_t currentPositionPulses = motor.PositionRefCommanded();
    int32_t distanceFromHomePulses = abs(currentPositionPulses);
    
    // Check minimum distance requirement using pulse constants
    int32_t minDistancePulses = getHomeMinDistancePulses(rail);
    if (distanceFromHomePulses < minDistancePulses) {
        if (estimatedTimeSavingsMs) *estimatedTimeSavingsMs = 0;
        return false; // Too close to home, no benefit
    }
    
    // Calculate time savings using integer math
    // Time = Distance / Velocity
    // Standard homing: full distance at slow speed
    // Smart homing: most distance at fast speed, small portion at slow speed
    
    int32_t fastVelocityPps = rpmToPps(HOME_FAST_APPROACH_VELOCITY_RPM);
    int32_t slowVelocityPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
    int32_t precisionDistancePulses = getHomePrecisionDistancePulses(rail);
    
    // Standard approach time (all at slow speed)
    int32_t standardTimeMs = (distanceFromHomePulses * 1000) / slowVelocityPps;
    
    // Smart approach time (fast + precision phases)
    int32_t fastPhaseDistance = distanceFromHomePulses - precisionDistancePulses;
    int32_t fastPhaseTimeMs = (fastPhaseDistance * 1000) / fastVelocityPps;
    int32_t precisionPhaseTimeMs = (precisionDistancePulses * 1000) / slowVelocityPps;
    int32_t smartTimeMs = fastPhaseTimeMs + precisionPhaseTimeMs;
    
    // Calculate savings
    int32_t timeSavingsMs = standardTimeMs - smartTimeMs;
    
    if (estimatedTimeSavingsMs) {
        *estimatedTimeSavingsMs = timeSavingsMs;
    }
    
    // Smart homing is beneficial if it saves at least 5 seconds
    return (timeSavingsMs > 5000);
}

void calculateSmartHomingPhases(int rail, int32_t currentPositionPulses, bool* useSmartHoming, 
                               int32_t* fastPhaseDistancePulses, int32_t* precisionPhaseDistancePulses) {
    // Default values
    *useSmartHoming = false;
    *fastPhaseDistancePulses = 0;
    *precisionPhaseDistancePulses = 0;
    
    // Check if smart homing is beneficial
    int32_t timeSavingsMs;
    if (!isSmartHomingBeneficial(rail, &timeSavingsMs)) {
        return;
    }
    
    // Calculate phases using pulse constants
    int32_t distanceFromHomePulses = abs(currentPositionPulses);
    int32_t precisionDistancePulses = getHomePrecisionDistancePulses(rail);
    
    // Ensure we don't exceed available distance
    if (precisionDistancePulses > distanceFromHomePulses) {
        precisionDistancePulses = distanceFromHomePulses;
        *fastPhaseDistancePulses = 0;
    } else {
        *fastPhaseDistancePulses = distanceFromHomePulses - precisionDistancePulses;
    }
    
    *precisionPhaseDistancePulses = precisionDistancePulses;
    *useSmartHoming = true;
}

bool initiateSmartHomingSequence(int rail) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    // Check if motor is ready
    if (!isMotorReady(rail)) {
        sprintf_P(msg, FMT_NOT_READY_HOMING, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Get current position and calculate smart homing phases
    int32_t currentPositionPulses = motor.PositionRefCommanded();
    bool useSmartHoming;
    int32_t fastPhaseDistancePulses, precisionPhaseDistancePulses;
    
    calculateSmartHomingPhases(rail, currentPositionPulses, &useSmartHoming, 
                              &fastPhaseDistancePulses, &precisionPhaseDistancePulses);
    
    if (!useSmartHoming) {
        // Fall back to standard homing
        return initiateHomingSequence(rail);
    }
    
    // Reset homing state for smart sequence
    resetHomingState(rail);
    
    // Initialize homing state
    MotorHomingState& homingState = getHomingState(rail);
    homingState.homingInProgress = true;
    homingState.homingStartTime = millis();
    homingState.startPulses = currentPositionPulses;
    
    // Set global homing flag
    if (rail == 1) {
        rail1HomingInProgress = true;
    } else {
        rail2HomingInProgress = true;
    }
    
    // Phase 1: Fast approach
    if (fastPhaseDistancePulses > 0) {
        int32_t fastVelocityPps = rpmToPps(HOME_FAST_APPROACH_VELOCITY_RPM);
        setMotorVelocity(rail, fastVelocityPps);
        
        int homingDirection = getHomingDirection(rail);
        int32_t fastPhasePulses = homingDirection * fastPhaseDistancePulses;
        motor.Move(fastPhasePulses);
        
        sprintf_P(msg, PSTR("%s: Smart homing initiated - Fast approach phase (%ld pulses at %d RPM)"), 
                 motorName, fastPhaseDistancePulses, HOME_FAST_APPROACH_VELOCITY_RPM);
        Console.serialInfo(msg);
        
        // Wait for fast phase to complete
        unsigned long phaseStartTime = millis();
        unsigned long homingTimeout = getHomingTimeout(rail);
        while (!motor.StepsComplete() && !timeoutElapsed(millis(), phaseStartTime, homingTimeout)) {
            if (motor.StatusReg().bit.AlertsPresent) {
                sprintf_P(msg, FMT_ALERT_DURING_HOMING, motorName);
                Console.serialError(msg);
                abortHoming(rail);
                return false;
            }
            delay(10);
        }
        
        if (!motor.StepsComplete()) {
            sprintf_P(msg, PSTR("%s: Fast phase timeout"), motorName);
            Console.serialWarning(msg);
            abortHoming(rail);
            return false;
        }
        
        sprintf_P(msg, PSTR("%s: Fast approach phase completed"), motorName);
        Console.serialInfo(msg);
    }
    
    // Phase 2: Precision approach (same as standard homing)
    int32_t precisionVelocityPps = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
    setMotorVelocity(rail, precisionVelocityPps);
    
    // Continue with precision homing to find hardstop
    int homingDirection = getHomingDirection(rail);
    int32_t precisionPhasePulses = homingDirection * (precisionPhaseDistancePulses + 1000); // Extra margin
    motor.Move(precisionPhasePulses);
    
    sprintf_P(msg, PSTR("%s: Precision homing phase started"), motorName);
    Console.serialInfo(msg);
    
    // Mark that minimum distance has been traveled (since we did fast approach)
    homingState.minDistanceTraveled = true;
    homingState.positionAtMinDistance = motor.PositionRefCommanded();
    
    return true;
}

//=============================================================================
// MOTION DECELERATION FUNCTIONS
//=============================================================================

// Scale factor for fixed-point math in deceleration calculations
#define SCALE_FACTOR 100

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

int32_t calculateDeceleratedVelocity(int rail, int32_t distanceToTargetMm, int32_t totalMoveDistanceMm, int32_t maxVelocityPps) {
    RailDecelerationConfig& config = getDecelerationConfig(rail);
    
    if (!config.enableDeceleration) {
        return maxVelocityPps;
    }
    
    // Use the enhanced deceleration distance calculation
    int32_t decelDistanceMm = getDecelerationDistanceScaled(rail, totalMoveDistanceMm * SCALE_FACTOR) / SCALE_FACTOR;
    
    // No deceleration needed if we're not in deceleration zone
    if (distanceToTargetMm > decelDistanceMm) {
        return maxVelocityPps;
    }

    // Safeguard against division by zero
    if (decelDistanceMm <= 0) {
        // If deceleration distance is invalid, use minimum velocity for safety
        return rpmToPps(config.minVelocityRPM);
    }
    
    // Calculate deceleration ratio using integer math (0-1000 range for precision)
    int32_t decelRatio1000 = (distanceToTargetMm * 1000) / decelDistanceMm;
    
    // Apply quadratic deceleration curve: ratio^2 for smoother motion
    int32_t decelRatioSquared1000 = (decelRatio1000 * decelRatio1000) / 1000;
    
    // Calculate minimum velocity in PPS
    int32_t minVelocityPps = rpmToPps(config.minVelocityRPM);
    
    // Calculate target velocity using integer math with quadratic curve
    int32_t velocityRange = maxVelocityPps - minVelocityPps;
    int32_t velocityAdjustment = (velocityRange * decelRatioSquared1000) / 1000;
    int32_t targetVelocity = minVelocityPps + velocityAdjustment;
    
    // Ensure we don't go below minimum or above maximum
    if (targetVelocity < minVelocityPps) {
        targetVelocity = minVelocityPps;
    } else if (targetVelocity > maxVelocityPps) {
        targetVelocity = maxVelocityPps;
    }
    
    return targetVelocity;
}

//=============================================================================
// CORE MOVEMENT FUNCTIONS
//=============================================================================

bool moveToPositionFromCurrent(int rail, PositionTarget target, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    // Validate inputs
    if (!isValidPositionForRail(target, rail)) {
        sprintf_P(msg, FMT_INVALID_POSITION, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Check if motor is ready
    if (!isMotorReady(rail)) {
        sprintf_P(msg, FMT_MOTOR_CANNOT_MOVE, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Get target position and current position
    int32_t targetPulses = getPositionPulses(target);
    int32_t currentPulses = motor.PositionRefCommanded();
    int32_t movePulses = targetPulses - currentPulses;
    
    if (movePulses == 0) {
        sprintf_P(msg, PSTR("%s: Already at target position %s"), motorName, getPositionName(target));
        Console.serialInfo(msg);
        return true;
    }
    
    // Calculate movement distance and select velocity
    double moveDistanceMm = abs(pulsesToMm(movePulses, rail));
    int32_t velocityRpm = getCarriageVelocityRpm(rail, carriageLoaded);
    int32_t velocityPps = rpmToPps(velocityRpm);
    
    // Set velocity and initiate move
    setMotorVelocity(rail, velocityPps);
    motor.Move(movePulses);
    
    // Log the movement
    sprintf_P(msg, FMT_MOVE_TO_POSITION, motorName, getPositionName(target), 
             pulsesToMm(targetPulses, rail), velocityRpm, 
             carriageLoaded ? "(loaded)" : "(empty)");
    Console.serialInfo(msg);
    
    return true;
}

bool moveToPositionMm(int rail, double targetMm, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    // Check if motor is ready
    if (!isMotorReady(rail)) {
        sprintf_P(msg, FMT_MOTOR_CANNOT_MOVE, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Validate position range
    double maxTravelMm = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    if (targetMm < 0 || targetMm > maxTravelMm) {
        sprintf_P(msg, FMT_POSITION_MM_OUT_OF_RANGE, motorName, targetMm, maxTravelMm);
        Console.serialError(msg);
        return false;
    }
    
    // Convert to pulses and calculate move
    int32_t targetPulses = mmToPulses(targetMm, rail);
    int32_t currentPulses = motor.PositionRefCommanded();
    int32_t movePulses = targetPulses - currentPulses;
    
    if (movePulses == 0) {
        sprintf_P(msg, PSTR("%s: Already at target position %.2fmm"), motorName, targetMm);
        Console.serialInfo(msg);
        return true;
    }
    
    // Calculate movement distance and select velocity
    double moveDistanceMm = abs(pulsesToMm(movePulses, rail));
    int32_t velocityRpm = getCarriageVelocityRpm(rail, carriageLoaded);
    int32_t velocityPps = rpmToPps(velocityRpm);
    
    // Set velocity and initiate move
    setMotorVelocity(rail, velocityPps);
    motor.Move(movePulses);
    
    // Log the movement
    sprintf_P(msg, FMT_MOVE_TO_MM, motorName, targetMm, targetPulses, velocityRpm,
             carriageLoaded ? "(loaded)" : "(empty)");
    Console.serialInfo(msg);
    
    return true;
}

bool moveRelativeMm(int rail, double distanceMm, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    // Check if motor is ready
    if (!isMotorReady(rail)) {
        sprintf_P(msg, FMT_MOTOR_CANNOT_MOVE, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Calculate target position
    double currentMm = getMotorPositionMm(rail);
    double targetMm = currentMm + distanceMm;
    double maxTravelMm = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    // Validate move doesn't exceed limits
    if (targetMm < 0 || targetMm > maxTravelMm) {
        sprintf_P(msg, FMT_RELATIVE_MOVE_RANGE, motorName, maxTravelMm);
        Console.serialError(msg);
        sprintf_P(msg, FMT_RELATIVE_MOVE_DETAILS, motorName, currentMm, distanceMm, targetMm);
        Console.serialError(msg);
        return false;
    }
    
    // Convert to pulses and initiate move
    int32_t movePulses = mmToPulses(distanceMm, rail);
    
    if (movePulses == 0) {
        sprintf_P(msg, PSTR("%s: Zero distance move requested"), motorName);
        Console.serialInfo(msg);
        return true;
    }
    
    // Calculate movement distance and select velocity
    double moveDistanceMm = abs(distanceMm);
    int32_t velocityRpm = getCarriageVelocityRpm(rail, carriageLoaded);
    int32_t velocityPps = rpmToPps(velocityRpm);
    
    // Set velocity and initiate move
    setMotorVelocity(rail, velocityPps);
    motor.Move(movePulses);
    
    // Log the movement
    sprintf_P(msg, FMT_RELATIVE_MOVE, motorName, distanceMm, movePulses, velocityRpm,
             carriageLoaded ? "(loaded)" : "(empty)");
    Console.serialInfo(msg);
    
    return true;
}

void checkMoveProgress() {
    // Check progress for both rails
    checkMovementProgress(1);
    checkMovementProgress(2);
}

//=============================================================================
// JOGGING SYSTEM FUNCTIONS
//=============================================================================

bool jogMotor(int rail, bool direction, double customIncrement, bool carriageLoaded) {
    MotorDriver& motor = getMotorByRail(rail);
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    // Check if motor is ready for jogging
    if (!isMotorReady(rail)) {
        sprintf_P(msg, FMT_NOT_READY_JOG, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Check if motor is currently moving
    if (isMotorMoving(rail)) {
        sprintf_P(msg, FMT_CANNOT_JOG_MOVING, motorName);
        Console.serialError(msg);
        return false;
    }
    
    // Get jog parameters - use custom increment if provided, otherwise use default
    double jogIncrementMm = (customIncrement > 0) ? customIncrement : getJogIncrementRef(rail);
    int jogSpeedRpm = getJogSpeedRef(rail);
    
    // Calculate jog distance with direction
    double jogDistanceMm = direction ? jogIncrementMm : -jogIncrementMm;
    
    // Check if jog would exceed travel limits
    double currentMm = getMotorPositionMm(rail);
    double targetMm = currentMm + jogDistanceMm;
    double maxTravelMm = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    if (targetMm < 0 || targetMm > maxTravelMm) {
        sprintf_P(msg, FMT_JOG_EXCEED_LIMITS, motorName, maxTravelMm);
        Console.serialError(msg);
        sprintf_P(msg, FMT_JOG_POSITION_INFO, motorName, currentMm, targetMm);
        Console.serialError(msg);
        return false;
    }
    
    // Apply speed capping based on jog distance (silently for common operations)
    int cappedSpeedRpm = jogSpeedRpm;
    if (jogIncrementMm <= JOG_VERY_SHORT_THRESHOLD_MM) { // Very short jog
        cappedSpeedRpm = min(jogSpeedRpm, JOG_VERY_SHORT_MAX_SPEED_RPM);
    } else if (jogIncrementMm <= JOG_SHORT_THRESHOLD_MM) { // Short jog
        cappedSpeedRpm = min(jogSpeedRpm, JOG_SHORT_MAX_SPEED_RPM);
    } else if (jogIncrementMm <= JOG_MEDIUM_THRESHOLD_MM) { // Medium jog
        cappedSpeedRpm = min(jogSpeedRpm, JOG_MEDIUM_MAX_SPEED_RPM);
    } else { // Long jog
        cappedSpeedRpm = min(jogSpeedRpm, JOG_LONG_MAX_SPEED_RPM);
    }
    
    // Convert to pulses and set velocity
    int32_t jogPulses = mmToPulses(jogDistanceMm, rail);
    int32_t jogVelocityPps = rpmToPps(cappedSpeedRpm);
    
    setMotorVelocity(rail, jogVelocityPps);
    motor.Move(jogPulses);
    
    // Log the jog operation with speed capping info included if relevant
    const char* speedNote = "";
    if (cappedSpeedRpm < jogSpeedRpm) {
        speedNote = " (speed capped)";
    }
    
    sprintf_P(msg, FMT_JOGGING, motorName, 
             direction ? "forward" : "backward", 
             jogIncrementMm, cappedSpeedRpm, speedNote);
    Console.serialInfo(msg);
    
    return true;
}

bool setJogIncrement(int rail, double incrementMm) {
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    double maxTravelMm = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    // Validate jog increment range
    if (incrementMm <= 0 || incrementMm > maxTravelMm) {
        sprintf_P(msg, FMT_JOG_INCREMENT_RANGE, motorName, maxTravelMm);
        Console.serialError(msg);
        return false;
    }
    
    // Set the jog increment
    getJogIncrementRef(rail) = incrementMm;
    
    sprintf_P(msg, FMT_JOG_INCREMENT_SET, motorName, incrementMm);
    Console.serialInfo(msg);
    
    return true;
}

bool setJogSpeed(int rail, int speedRpm, double jogDistanceMm) {
    const char* motorName = getMotorName(rail);
    char msg[MEDIUM_MSG_SIZE];
    
    int maxSpeedRpm = (rail == 1) ? RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM : RAIL2_EMPTY_CARRIAGE_VELOCITY_RPM;
    
    // Validate jog speed range
    if (speedRpm < 10 || speedRpm > maxSpeedRpm) {
        sprintf_P(msg, FMT_JOG_SPEED_RANGE, motorName, maxSpeedRpm);
        Console.serialError(msg);
        return false;
    }
    
    // Set the jog speed
    getJogSpeedRef(rail) = speedRpm;
    
    // If a jog distance is provided, also set that
    if (jogDistanceMm > 0) {
        setJogIncrement(rail, jogDistanceMm);
        sprintf_P(msg, FMT_JOG_SPEED_SET_WITH_DISTANCE, motorName, speedRpm, jogDistanceMm);
    } else {
        sprintf_P(msg, FMT_JOG_SPEED_SET, motorName, speedRpm);
    }
    Console.serialInfo(msg);
    
    return true;
}

double getJogIncrement(int rail) {
    return getJogIncrementRef(rail);
}

int getJogSpeed(int rail) {
    return getJogSpeedRef(rail);
}

//=============================================================================
// SMART VELOCITY SELECTION
//=============================================================================

int32_t selectMoveVelocityByDistance(int rail, double moveDistanceMm, bool carriageLoaded) {
    // Get base velocity based on carriage load
    int32_t baseVelocityRpm = getCarriageVelocityRpm(rail, carriageLoaded);
    
    // Apply distance-based adjustments for better motion profiles
    int32_t adjustedVelocityRpm = baseVelocityRpm;
    
    if (moveDistanceMm < 10.0) {
        // Very short moves: reduce to 40% of base speed
        adjustedVelocityRpm = (baseVelocityRpm * 40) / 100;
    } else if (moveDistanceMm < 50.0) {
        // Short moves: reduce to 60% of base speed
        adjustedVelocityRpm = (baseVelocityRpm * 60) / 100;
    } else if (moveDistanceMm < 200.0) {
        // Medium moves: reduce to 80% of base speed
        adjustedVelocityRpm = (baseVelocityRpm * 80) / 100;
    } else {
        // Long moves: use full base speed
        adjustedVelocityRpm = baseVelocityRpm;
    }
    
    // Ensure minimum velocity for reliable operation
    const int32_t minVelocityRpm = 30;
    if (adjustedVelocityRpm < minVelocityRpm) {
        adjustedVelocityRpm = minVelocityRpm;
    }
    
    return adjustedVelocityRpm;
}

//=============================================================================
// ENHANCED MOVEMENT VALIDATION AND PROGRESS MONITORING
//=============================================================================

bool checkMovementTimeout(int rail, unsigned long timeoutMs) {
    MotorTargetState& targetState = getTargetState(rail);
    
    if (!targetState.movementInProgress) {
        return false; // No move in progress, no timeout
    }
    
    unsigned long currentTime = millis();
    return timeoutElapsed(currentTime, targetState.movementStartTime, timeoutMs);
}

bool checkMovementProgress(int rail) {
    MotorDriver& motor = getMotorByRail(rail);
    MotorTargetState& targetState = getTargetState(rail);
    const char* motorName = getMotorName(rail);
    
    // Check if motor is moving
    if (!isMotorMoving(rail)) {
        if (targetState.movementInProgress) {
            // Movement completed
            targetState.movementInProgress = false;
            targetState.lastProgressCheck = millis();
            return true; // Movement completed successfully
        }
        return false; // No movement to monitor
    }
    
    // Update move tracking if not already started
    if (!targetState.movementInProgress) {
        targetState.movementInProgress = true;
        targetState.movementStartTime = millis();
        targetState.lastProgressCheck = millis();
        targetState.lastPositionCheck = motor.PositionRefCommanded();
        return false; // Just started, continue monitoring
    }
    
    // Check for stall detection
    unsigned long currentTime = millis();
       int32_t currentPosition = motor.PositionRefCommanded();
    
    if (timeoutElapsed(currentTime, targetState.lastProgressCheck, MOVEMENT_STALL_TIMEOUT_MS)) {
        int32_t positionChange = abs(currentPosition - targetState.lastPositionCheck);
        
        if (positionChange < MOVEMENT_MIN_PROGRESS_PULSES) {
            // Stall detected
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("%s: Movement stall detected - stopping"), motorName);
            Console.serialError(msg);
            
            motor.MoveStopAbrupt();
            targetState.movementInProgress = false;
            return false; // Movement failed due to stall
        }
        
        // Update progress tracking
        targetState.lastProgressCheck = currentTime;
        targetState.lastPositionCheck = currentPosition;
    }
    
    // Check for overall timeout
    if (checkMovementTimeout(rail, MOVEMENT_TIMEOUT_MS)) {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("%s: Movement timeout - stopping"), motorName);
        Console.serialError(msg);
        
        motor.MoveStopAbrupt();
        targetState.movementInProgress = false;
        return false; // Movement failed due to timeout
    }
    
    return false; // Movement still in progress
}

void updateDecelerationVelocity(int rail) {
    MotorDriver& motor = getMotorByRail(rail);
    MotorTargetState& targetState = getTargetState(rail);
    
    if (!targetState.movementInProgress || !isMotorMoving(rail)) {
        return; // No active movement to update
    }
    
    // Calculate distance to target
    int32_t currentPosition = motor.PositionRefCommanded();
    int32_t distanceToTarget = abs(targetState.targetPositionPulses - currentPosition);
    double distanceToTargetMm = pulsesToMm(distanceToTarget, rail);
    double totalMoveDistanceMm = pulsesToMm(abs(targetState.targetPositionPulses - targetState.startPositionPulses), rail);
    
    // Calculate decelerated velocity
    int32_t maxVelocityPps = getCurrentMotorVelocity(rail); // Use rail-specific velocity limit
    int32_t newVelocityPps = calculateDeceleratedVelocity(rail, (int32_t)(distanceToTargetMm * 100), 
                                                         (int32_t)(totalMoveDistanceMm * 100), maxVelocityPps);
    
    // Only update if velocity should change significantly
    int32_t currentVelocity = getCurrentMotorVelocity(rail);
    
    // Safeguard against division by zero in velocity comparison
    if (currentVelocity > 0) {
        int32_t velocityChangeThreshold = currentVelocity / 20; // 5% threshold
        if (abs(newVelocityPps - currentVelocity) > velocityChangeThreshold) {
            setMotorVelocity(rail, newVelocityPps);
        }
    } else {
        // If current velocity is 0, always update to non-zero velocity
        if (newVelocityPps > 0) {
            setMotorVelocity(rail, newVelocityPps);
        }
    }
}

//=============================================================================
// POSITION NUMBER INTERFACE FUNCTIONS
//=============================================================================
// POSITION VALIDATION FUNCTION
//=============================================================================
// POSITION VALIDATION FUNCTION
//=============================================================================

bool validateAllPredefinedPositions() {
    Console.serialInfo(F("Validating all predefined positions against travel limits..."));
    
    bool allValid = true;
    int validPositions = 0;
    int totalPositions = 0;
    
    // Rail 1 positions to validate
    PositionTarget rail1Positions[] = {
        RAIL1_HOME_POS,
        RAIL1_WC2_PICKUP_DROPOFF_POS,
        RAIL1_WC1_PICKUP_DROPOFF_POS,
        RAIL1_STAGING_POS,
        RAIL1_HANDOFF_POS
    };
    
    // Rail 2 positions to validate
    PositionTarget rail2Positions[] = {
        RAIL2_HOME_POS,
        RAIL2_HANDOFF_POS,
        RAIL2_WC3_PICKUP_DROPOFF_POS
    };
    
    // Validate Rail 1 positions
    Console.serialInfo(F("Validating Rail 1 predefined positions..."));
    for (int i = 0; i < sizeof(rail1Positions) / sizeof(rail1Positions[0]); i++) {
        totalPositions++;
        PositionTarget pos = rail1Positions[i];
        
        if (isValidPositionForRail(pos, 1)) {
            validPositions++;
            int32_t pulses = getPositionPulses(pos);
            double mm = pulsesToMm(pulses, 1);
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("  %s: %.2fmm ✓"), getPositionName(pos), mm);
            Console.serialInfo(msg);
        } else {
            allValid = false;
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("  %s: INVALID ✗"), getPositionName(pos));
            Console.serialError(msg);
        }
    }
    
    // Validate Rail 2 positions
    Console.serialInfo(F("Validating Rail 2 predefined positions..."));
    for (int i = 0; i < sizeof(rail2Positions) / sizeof(rail2Positions[0]); i++) {
        totalPositions++;
        PositionTarget pos = rail2Positions[i];
        
        if (isValidPositionForRail(pos, 2)) {
            validPositions++;
            int32_t pulses = getPositionPulses(pos);
            double mm = pulsesToMm(pulses, 2);
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("  %s: %.2fmm ✓"), getPositionName(pos), mm);
            Console.serialInfo(msg);
        } else {
            allValid = false;
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, PSTR("  %s: INVALID ✗"), getPositionName(pos));
            Console.serialError(msg);
        }
    }
    
    // Summary report
    char summary[MEDIUM_MSG_SIZE];
    sprintf_P(summary, PSTR("Position validation complete: %d/%d positions valid"), 
             validPositions, totalPositions);
    
    if (allValid) {
        Console.serialInfo(summary);
        Console.serialInfo(F("All predefined positions are within travel limits"));
    } else {
        Console.serialError(summary);
        Console.serialError(F("CRITICAL: Some predefined positions exceed travel limits!"));
        Console.serialError(F("Review position definitions in MotorController.h"));
    }
    
    return allValid;
}

//=============================================================================
// TIMEOUT RESET FUNCTIONS
//=============================================================================

void resetMotorTimeouts() {
    unsigned long currentTime = millis();
    
    Console.serialInfo(F("MOTOR TIMEOUTS: Clearing movement and homing timeout tracking"));
    
    // Reset movement tracking for both rails
    for (int rail = 1; rail <= 2; rail++) {
        MotorTargetState& targetState = getTargetState(rail);
        
        // Reset movement timeout tracking
        targetState.movementStartTime = currentTime;
        targetState.lastProgressCheck = currentTime;
        targetState.lastDecelerationUpdate = currentTime;
        targetState.movementInProgress = false;  // Clear any stale movement flags
        targetState.movementTimeoutCount = 0;    // Reset timeout warning counter
        
        // Reset movement tracking positions
        targetState.lastPositionCheck = 0;
        
        // Reset homing timeout tracking
        MotorHomingState& homingState = getHomingState(rail);
        homingState.homingStartTime = currentTime;
        homingState.lastPositionCheckTime = currentTime;
        homingState.hlfbNonAssertedTime = currentTime;
        homingState.minTimeAfterDistanceReached = currentTime;
    }
    
    Console.serialInfo(F("MOTOR TIMEOUTS: All motor timeout tracking reset"));
}

