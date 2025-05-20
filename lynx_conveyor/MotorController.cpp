#include "MotorController.h"
#include "Utils.h"

// ----------------- Global Variables -----------------
bool motorInitialized = false;
int32_t currentVelMax = 0;
int32_t currentAccelMax = 0;
bool isHomed = false;
double currentPositionMm = 0.0;
MotorState motorState = MOTOR_STATE_NOT_READY;
PositionTarget currentPosition = POSITION_1;
bool homingInProgress = false;
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

// ----------------- Utility Functions -----------------

// Convert RPM to Pulses Per Second
int32_t rpmToPps(double rpm) {
    return (int32_t)((rpm * PULSES_PER_REV) / 60.0);
}

// Convert Pulses Per Second to RPM
double ppsToRpm(int32_t pps) {
    return (double)pps * 60.0 / PULSES_PER_REV;
}

// Convert RPM/s to Pulses/s^2
int32_t rpmPerSecToPpsPerSec(double rpmPerSec) {
    return (int32_t)((rpmPerSec * PULSES_PER_REV) / 60.0);
}

// Convert mm to pulses with direction control
int32_t mmToPulses(double mm) {
    return (int32_t)(mm * PULSES_PER_MM * MOTION_DIRECTION);
}

// Convert pulses to mm
double pulsesToMm(int32_t pulses) {
    // When converting from pulses to mm for display, include the MOTION_DIRECTION factor
    return (double)pulses / PULSES_PER_MM * MOTION_DIRECTION;
}
// Normalize encoder values for display
int32_t normalizeEncoderValue(int32_t rawValue) {
    // Apply the same direction multiplier as used in movement calculations
    return rawValue * MOTION_DIRECTION;
}
// ----------------- Basic Setup Functions -----------------

void initMotorSystem() {
    Serial.println(F("[MESSAGE] Initializing motor system..."));
    
    // Set up E-stop input pin with internal pull-up
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    Serial.print(F("[MESSAGE] Checking E-Stop state: "));
    if (isEStopActive()) {
        Serial.println(F("[ERROR] E-STOP ACTIVE! Please reset E-stop before continuing."));
    } else {
        Serial.println(F("[MESSAGE] E-stop inactive, system ready."));
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
    Serial.print(F("[MESSAGE] Setting velocity limit to "));
    Serial.print(MOTOR_VELOCITY_RPM); // CHANGED: Using consolidated constant
    Serial.println(F(" RPM"));
    
    currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM); // CHANGED
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    
    // Set acceleration limit
    Serial.print(F("[MESSAGE] Motor enable requested"));
    Serial.print(MAX_ACCEL_RPM_PER_SEC);
    Serial.println(F(" RPM"));
    
    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);
    
    // Enable the motor
    MOTOR_CONNECTOR.EnableRequest(true);
    Serial.println(F("[MESSAGE] Motor enable requested"));
    
    // Wait for HLFB to assert (up to 2 seconds)
    Serial.println(F("[MESSAGE] Waiting for HLFB..."));
    unsigned long startTime = millis();
    bool ready = false;
    
    while (!ready && millis() - startTime < 2000) {
        if (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
            ready = true;
        } else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
            Serial.println(F("[ERROR] Motor alert detected:"));
            printMotorAlerts();
            break;
        }
        delay(10);
    }
    
    if (ready) {
        Serial.println(F("[MESSAGE] Motor initialized and ready"));
        motorInitialized = true;
        motorState = MOTOR_STATE_IDLE;  
    } else {
        Serial.println(F("[ERROR] Motor initialization timed out or failed"));
        Serial.print("HLFB State: ");
        Serial.println(MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED ? 
                      "ASSERTED" : "NOT ASSERTED");
    }
}

// ----------------- Movement Functions -----------------

// For absolute positioning commands
bool moveToAbsolutePosition(int32_t position) {
    // Check if position is within valid range (accounting for MOTION_DIRECTION)
    if ((MOTION_DIRECTION * position < 0) || 
        (MOTION_DIRECTION * position > MAX_TRAVEL_PULSES)) {
        Serial.print(F("[ERROR] Requested position "));
        Serial.print(position);
        Serial.print(F(" pulses is outside valid range (0 to "));
        Serial.print(MOTION_DIRECTION * MAX_TRAVEL_PULSES);
        Serial.println(F(" pulses)"));
        return false;
    }
    
    // Check if motor has alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println(F("[ERROR] Motor alert detected. Cannot move."));
        printMotorAlerts();
        return false;
    }
    
    Serial.print(F("[MESSAGE] Moving to absolute position: "));
    Serial.println(normalizeEncoderValue(position));
    
    // Command the absolute move
    MOTOR_CONNECTOR.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Report initial status
    Serial.println(F("[MESSAGE] Move commanded. Motor in motion..."));
    
    return true;
}

bool moveToPosition(PositionTarget position) {
    // Set current target for logging
    hasCurrentTarget = true;
    currentTargetType = position;
    
    switch (position) {
        case POSITION_HOME:
            currentTargetPositionMm = POSITION_HOME_MM;
            currentTargetPulses = POSITION_HOME_PULSES;
            return moveToAbsolutePosition(normalizeEncoderValue(POSITION_HOME_PULSES));
        case POSITION_1:
            currentTargetPositionMm = POSITION_1_MM;
            currentTargetPulses = POSITION_1_PULSES;
            return moveToAbsolutePosition(normalizeEncoderValue(POSITION_1_PULSES));
        case POSITION_2:
            currentTargetPositionMm = POSITION_2_MM;
            currentTargetPulses = POSITION_2_PULSES;
            return moveToAbsolutePosition(normalizeEncoderValue(POSITION_2_PULSES));
        case POSITION_3:
            currentTargetPositionMm = POSITION_3_MM;
            currentTargetPulses = POSITION_3_PULSES;
            return moveToAbsolutePosition(normalizeEncoderValue(POSITION_3_PULSES));
        case POSITION_4:
            currentTargetPositionMm = POSITION_4_MM;
            currentTargetPulses = POSITION_4_PULSES;
            return moveToAbsolutePosition(normalizeEncoderValue(POSITION_4_PULSES));
        default:
            hasCurrentTarget = false;
            return false;
    }
}

bool moveToPosition(int positionNumber) {
    switch (positionNumber) {
        case 0:  // "home" as position 0
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

bool moveToPositionMm(double positionMm) {
    // Safety check - prevent movement beyond physical limits
    if (positionMm < 0 || positionMm > MAX_TRAVEL_MM) {
        Serial.print(F("[ERROR] Requested position "));
        Serial.print(positionMm);
        Serial.print(F(" mm is outside valid range (0 to "));
        Serial.print(MAX_TRAVEL_MM);
        Serial.println(F(" mm)"));
        return false;
    }
    
    int32_t pulsePosition = mmToPulses(positionMm);
    
    // Set current target for logging
    hasCurrentTarget = true;
    currentTargetType = POSITION_CUSTOM;
    currentTargetPositionMm = positionMm;
    currentTargetPulses = pulsePosition;

       
    if (moveToAbsolutePosition(pulsePosition)) {
        return true;
    } else {
        // Clear current target if movement failed
        hasCurrentTarget = false;
        return false;
    }
}

bool moveRelative(double relativeMm) {
    // Calculate the target absolute position
    double targetPositionMm = currentPositionMm + relativeMm;
    
    // Check if the target position would be out of bounds
    if (targetPositionMm < 0 || targetPositionMm > MAX_TRAVEL_MM) {
        Serial.print(F("[ERROR] Relative move would exceed valid range (0 to "));
        Serial.print(MAX_TRAVEL_MM);
        Serial.println(F(" mm)"));
        Serial.print(F("[ERROR] Current position: "));
        Serial.print(currentPositionMm);
        Serial.print(F(" mm, Requested move: "));
        Serial.print(relativeMm);
        Serial.print(F(" mm, Target would be: "));
        Serial.print(targetPositionMm);
        Serial.println(F(" mm"));
        return false;
    }
    
    // Convert millimeters to pulses
    int32_t relativePulses = mmToPulses(relativeMm);
    
    // Call the motor's relative move function
    MOTOR_CONNECTOR.Move(relativePulses, MotorDriver::MOVE_TARGET_REL_END_POSN);
    
    // Update state
    motorState = MOTOR_STATE_MOVING;
    currentPosition = POSITION_CUSTOM;
    
    Serial.print(F("[MESSAGE] Moving "));
    Serial.print(relativeMm);
    Serial.print(F(" mm from current position ("));
    Serial.print(normalizeEncoderValue(relativePulses));
    Serial.println(F(" pulses)"));
    
    return true;
}

double getMotorPositionMm() {
    return pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
}

void stopMotion() {
    // Stop the motor abruptly
    MOTOR_CONNECTOR.MoveStopAbrupt();
    Serial.println(F("[MESSAGE] Motion stopped"));
}

// ----------------- Jogging Functions -----------------

bool jogMotor(bool direction, double customIncrement) {
    // Save current speed setting
    int32_t originalVelMax = currentVelMax;
    
    // Set jog speed
    currentVelMax = rpmToPps(currentJogSpeedRpm);
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    
    // Determine increment to use
    double increment = (customIncrement > 0) ? customIncrement : currentJogIncrementMm;
    
    // Calculate movement direction (positive or negative)
    double moveMm = direction ? increment : -increment;
    
    // Set current position
    double currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    
    // Calculate target position and set target tracking variables
    double targetPositionMm = currentPositionMm + moveMm;
    
    // Update target tracking variables 
    hasCurrentTarget = true;
    currentTargetType = POSITION_CUSTOM;
    currentTargetPositionMm = targetPositionMm;
    currentTargetPulses = mmToPulses(targetPositionMm);
    
    // Log the jog operation
    Serial.print(F("[MESSAGE] Jogging "));
    Serial.print(direction ? F("forward") : F("backward"));
    Serial.print(F(" by "));
    Serial.print(increment);
    Serial.print(F(" mm at "));
    Serial.print(currentJogSpeedRpm);
    Serial.println(F(" RPM"));
    
    // Use the existing moveRelative function
    bool result = moveRelative(moveMm);
    
    // If movement failed, clear target tracking
    if (!result) {
        hasCurrentTarget = false;
    }
    
    // Reset to original speed after move is commanded
    currentVelMax = originalVelMax;
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    
    return result;
}

bool setJogIncrement(double increment) {
    // Validate increment is reasonable
    if (increment <= 0 || increment > 100) {
        Serial.println(F("[ERROR] Jog increment must be between 0 and 100mm"));
        return false;
    }
    
    // Set the increment
    currentJogIncrementMm = increment;
    Serial.print(F("[MESSAGE] Jog increment set to "));
    Serial.print(currentJogIncrementMm);
    Serial.println(F(" mm"));
    
    return true;
}

bool setJogSpeed(int speedRpm) {
    // Validate speed is reasonable
    if (speedRpm < 10 || speedRpm > MOTOR_VELOCITY_RPM) {
        Serial.print(F("[ERROR] Jog speed must be between 10 and "));
        Serial.print(MOTOR_VELOCITY_RPM);
        Serial.println(F(" RPM"));
        return false;
    }
    
    // Set the speed
    currentJogSpeedRpm = speedRpm;
    Serial.print(F("[MESSAGE] Jog speed set to "));
    Serial.print(currentJogSpeedRpm);
    Serial.println(F(" RPM"));
    
    return true;
}

// ----------------- Status Functions -----------------

bool isMotorReady() {
    return motorInitialized && 
           MOTOR_CONNECTOR.EnableRequest() && 
           MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED &&
           !MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent;
}

bool isMotorMoving() {
    // Only consider step completion for motion status
    return !MOTOR_CONNECTOR.StepsComplete();
}

bool isMotorInPosition() {
    return MOTOR_CONNECTOR.StepsComplete() && 
           MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED;
}

bool isMotorAtPosition() {
    // This can either be an alias to isMotorInPosition() or implement its own logic
    return isMotorInPosition();
}

bool hasMotorFault() {
    return MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent;
}

MotorState updateMotorState() {
    // Check for faults first
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        motorState = MOTOR_STATE_FAULTED;
    }
    // Check if motor is not enabled
    else if (!MOTOR_CONNECTOR.EnableRequest()) {
        motorState = MOTOR_STATE_NOT_READY;
    }
    // Check if homing is in progress
    else if (homingInProgress) {
        motorState = MOTOR_STATE_HOMING;
    }
    // Check if steps are complete (primary indicator of motion completion)
    else if (!MOTOR_CONNECTOR.StepsComplete()) {
        motorState = MOTOR_STATE_MOVING;
    }
    // Otherwise, motor is idle
    else {
        // Even if HLFB isn't asserted yet, if steps are complete,
        // consider the motor idle for state management purposes
        motorState = MOTOR_STATE_IDLE;
    }
    
    return motorState;
}

void printMotorStatus() {
    Serial.println(F("[MESSAGE] Motor Status:"));
    
    Serial.print(F("  Enabled: "));
    Serial.println(MOTOR_CONNECTOR.EnableRequest() ? F("Yes") : F("No"));
    
    Serial.print(F("  Moving: "));
    Serial.println(isMotorAtPosition() ? F("No") : F("Yes"));
    
    Serial.print(F("  Position: "));
    Serial.print(normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded()));
    Serial.println(F(" pulses"));
    
    Serial.print(F("  Current Velocity Limit: "));
    Serial.print(ppsToRpm(currentVelMax));
    Serial.println(F(" RPM"));
    
    Serial.print(F("  Current Acceleration Limit: "));
    double accelRpmPerSec = (double)currentAccelMax * 60.0 / PULSES_PER_REV;
    Serial.print(accelRpmPerSec);
    Serial.println(F(" RPM/s"));
    
    Serial.print(F("  HLFB Status: "));
    switch (MOTOR_CONNECTOR.HlfbState()) {
        case MotorDriver::HLFB_ASSERTED:
            Serial.println(F("Asserted (In Position/Ready)"));
            break;
        case MotorDriver::HLFB_DEASSERTED:
            Serial.println(F("Deasserted (Moving/Fault)"));
            break;
        case MotorDriver::HLFB_UNKNOWN:
        default:
            Serial.println(F("Unknown"));
            break;
    }
    
    // Print any alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println(F("  Alerts present:"));
        printMotorAlerts();
    } else {
        Serial.println(F("  No alerts"));
    }
}

void printMotorAlerts() {
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledInAlert) {
        Serial.println(F("[ERROR]     MotionCanceledInAlert"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledPositiveLimit) {
        Serial.println(F("[ERROR]     MotionCanceledPositiveLimit"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledNegativeLimit) {
        Serial.println(F("[ERROR]     MotionCanceledNegativeLimit"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledSensorEStop) {
        Serial.println(F("[ERROR]     MotionCanceledSensorEStop"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledMotorDisabled) {
        Serial.println(F("[ERROR]     MotionCanceledMotorDisabled"));
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted) {
        Serial.println(F("[ERROR]     MotorFaulted"));
    }
}

void clearMotorFaults() {
    // If we're not already in the process of clearing faults, start the process
    if (!faultClearInProgress) {
        Serial.println(F("[DIAGNOSTIC] Attempting to clear motor faults..."));
        
        // First check if there are faults present
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
            Serial.println(F("[DIAGNOSTIC] Alerts detected:"));
            printMotorAlerts();
            
            // Start the fault clearing state machine
            faultClearState = FAULT_CLEAR_DISABLE;
            faultClearTimer = millis();
            faultClearInProgress = true;
        } else {
            Serial.println(F("[MESSAGE] No alerts to clear."));
        }
    }
}

void processFaultClearing() {
    // If we're not clearing faults, just return
    if (!faultClearInProgress) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    switch (faultClearState) {
        case FAULT_CLEAR_DISABLE:
            // Disable the motor
            if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted) {
                Serial.println(F("[DIAGNOSTIC] Motor faulted. Cycling enable signal..."));
                MOTOR_CONNECTOR.EnableRequest(false);
            }
            faultClearTimer = currentTime;
            faultClearState = FAULT_CLEAR_WAITING_DISABLE;
            break;
            
        case FAULT_CLEAR_WAITING_DISABLE:
            // Wait 100ms after disabling
            if (currentTime - faultClearTimer >= 100) {
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
            if (currentTime - faultClearTimer >= 100) {
                faultClearState = FAULT_CLEAR_ALERTS;
            }
            break;
            
        case FAULT_CLEAR_ALERTS:
            // Clear alerts
            Serial.println(F("[DIAGNOSTIC] Clearing motor alerts..."));
            MOTOR_CONNECTOR.ClearAlerts();
            
            // Check if alerts were cleared
            if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
                Serial.println(F("[ERROR] Alerts are still present after clearing."));
                printMotorAlerts();
            } else {
                Serial.println(F("[MESSAGE] Alerts successfully cleared."));
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

bool isFaultClearingInProgress() {
    return faultClearInProgress;
}

bool clearMotorFaultWithStatus() {
    // If fault clearing is already in progress, return false
    if (faultClearInProgress) {
        Serial.println(F("[MESSAGE] Fault clearing already in progress"));
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

bool initiateHomingSequence() {
    // Check if motor is initialized
    if (!motorInitialized) {
        Serial.println(F("[ERROR] Motor not initialized - run 'motor init' first"));
        return false;
    }
    
    // Check for motor alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println(F("[ERROR] Motor has active alerts - clear faults before homing"));
        printMotorAlerts(); // Print the specific alerts
        Serial.println(F("[ERROR] Please run 'clear fault' command first"));
        return false;
    }
    
    // Set current target for logging to indicate we're homing
    hasCurrentTarget = true;
    currentTargetType = POSITION_HOME;
    currentTargetPositionMm = 0.0;
    currentTargetPulses = 0;
    
    // Start the non-blocking enable cycle
    Serial.println(F("[MESSAGE] Cycling motor enable to trigger automatic homing..."));
    MOTOR_CONNECTOR.EnableRequest(false);
    enableCycleStartTime = millis();
    motorEnableCycleInProgress = true;
    motorDisablePhaseComplete = false;
    
    // Initialize homing state
    homingInProgress = true;
    motorState = MOTOR_STATE_HOMING;
    homingStartTime = millis();
    
    // The rest of the homing process will be handled when the enable cycle completes
    return true;
}

void executeHomingSequence() {
    if (!homingInProgress) {
        return;
    }
    
    // Handle the movement initiation after enable cycle completes
    static bool homingMovementStarted = false;
    
    // Check if enable cycle just completed and movement hasn't started yet
    if (!motorEnableCycleInProgress && !homingMovementStarted) {
        // Start the actual homing movement
        Serial.println(F("[MESSAGE] Starting hard stop homing sequence..."));
        
        // Set velocity for initial approach
        int32_t homingVel = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
        Serial.print(F("[MESSAGE] Moving toward hard stop with velocity: "));
        Serial.print(HOME_APPROACH_VELOCITY_RPM);
        Serial.println(F(" RPM"));
        
        currentVelMax = homingVel;
        MOTOR_CONNECTOR.VelMax(currentVelMax);
        
        // Start moving toward the hard stop
        MOTOR_CONNECTOR.MoveVelocity(HOMING_DIRECTION * MOTION_DIRECTION * homingVel);
        
        homingMovementStarted = true;
    }
    
    // Reset the movement flag if homing is aborted
    if (!homingInProgress) {
        homingMovementStarted = false;
        return;
    }
    
    // Continue with the existing homing state machine
    static enum {
        HOMING_PHASE_INITIAL_APPROACH,
        HOMING_PHASE_SLOW_APPROACH,
        HOMING_PHASE_WAIT_FOR_HARDSTOP,
        HOMING_PHASE_MOVE_OFFSET,
        HOMING_PHASE_WAIT_FOR_COMPLETION
    } homingPhase = HOMING_PHASE_INITIAL_APPROACH;

    static unsigned long phaseStartTime = 0;
    static bool initialPhaseTimeSet = false;

    // Check for timeout
    if (millis() - homingStartTime > HOME_TIMEOUT_MS) {
        Serial.println(F("[ERROR] Homing operation timed out"));
        stopMotion();
        homingInProgress = false;
        homingPhase = HOMING_PHASE_INITIAL_APPROACH; // Reset for next time
        initialPhaseTimeSet = false;
        homingMovementStarted = false;
        motorState = MOTOR_STATE_FAULTED;
        return;
    }

    // Process the homing state machine
    switch (homingPhase) {
        case HOMING_PHASE_INITIAL_APPROACH:
            // This phase is handled by the initial movement setup
            // Just set up the tracking variables
            if (!initialPhaseTimeSet) {
                phaseStartTime = millis();
                initialPhaseTimeSet = true;
            }
            
            // Switch to slower approach after a delay or when close to the hard stop
            if (millis() - phaseStartTime > 2000) {
                Serial.print(F("[DIAGNOSTIC] Switching to slower velocity: "));
                Serial.print(HOME_FINAL_VELOCITY_RPM);
                Serial.println(F(" RPM"));
                
                // Reduce velocity for final approach
                int32_t slowHomingVel = rpmToPps(HOME_FINAL_VELOCITY_RPM);
                MOTOR_CONNECTOR.VelMax(slowHomingVel);
                homingPhase = HOMING_PHASE_SLOW_APPROACH;
            }
            break;
            
        case HOMING_PHASE_SLOW_APPROACH:
            // We're moving at slow speed toward hard stop
            homingPhase = HOMING_PHASE_WAIT_FOR_HARDSTOP;
            Serial.println(F("[DIAGNOSTIC] Contacting hard stop, waiting for motor to settle..."));
            break;
            
        case HOMING_PHASE_WAIT_FOR_HARDSTOP:
            {
                // Use static variables to track timing and messaging
                static unsigned long lastHardstopMessageTime = 0;
                static unsigned long hardStopDetectedTime = 0;
                static bool hardStopDetected = false;
                
                unsigned long currentTime = millis();
                
                // Check HLFB to detect hard stop contact
                if (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED && !hardStopDetected) {
                    // Hard stop detected!
                    Serial.println(F("[MESSAGE] Hard stop detected (HLFB asserted). Stopping motor."));
                    
                    // Stop the motor
                    MOTOR_CONNECTOR.MoveStopAbrupt();
                    
                    // Record when we detected the hard stop
                    hardStopDetected = true;
                    hardStopDetectedTime = currentTime;
                    
                    // Don't transition to next phase yet - wait non-blockingly
                } 
                // If we've detected a hard stop and waited the required time
                else if (hardStopDetected && (currentTime - hardStopDetectedTime >= 100)) {
                    // Reset for next homing operation
                    hardStopDetected = false;
                    
                    // Move to offset phase
                    homingPhase = HOMING_PHASE_MOVE_OFFSET;
                    
                    // Reset the message timer for next time
                    lastHardstopMessageTime = 0;
                }
                // Only print status message every 10 seconds if still waiting
                else if (!hardStopDetected && (currentTime - lastHardstopMessageTime >= 10000)) {
                    Serial.print(F("[DIAGNOSTIC] Waiting for hardstop. HLFB State: DEASSERTED"));
                    Serial.println();
                    
                    // Update last message time
                    lastHardstopMessageTime = currentTime;
                }
            }
            break;
            
        case HOMING_PHASE_MOVE_OFFSET:
            // Move away from hard stop by the configured offset
            Serial.print(F("[MESSAGE] Moving away from hard stop by "));
            Serial.print(HOME_OFFSET_DISTANCE_MM);
            Serial.println(F(" mm"));
            
            {  // Add braces to create a new scope
                // Calculate offset in pulses
                int32_t offsetPulses = mmToPulses(HOME_OFFSET_DISTANCE_MM);
                
                // Use relative move away from hard stop
                MOTOR_CONNECTOR.Move(-offsetPulses, MotorDriver::MOVE_TARGET_REL_END_POSN);
            }  // End of scope
            
            // Set up to wait for completion
            homingPhase = HOMING_PHASE_WAIT_FOR_COMPLETION;
            break;
            
        case HOMING_PHASE_WAIT_FOR_COMPLETION:
            // Wait for offset move to complete
            if (MOTOR_CONNECTOR.StepsComplete()) {
                // Homing is complete!
                Serial.println(F("[MESSAGE] Hard stop homing completed successfully."));
                
                // Set position to zero
                MOTOR_CONNECTOR.PositionRefSet(0);
                
                // Update global state
                isHomed = true;
                homingInProgress = false;
                motorState = MOTOR_STATE_IDLE;
                currentPositionMm = 0.0;
                
                // Reset state machine for next time
                homingPhase = HOMING_PHASE_INITIAL_APPROACH;
                initialPhaseTimeSet = false;
                homingMovementStarted = false;
                
                // Restore original velocity and acceleration settings
                currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
                currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
                MOTOR_CONNECTOR.VelMax(currentVelMax);
                MOTOR_CONNECTOR.AccelMax(currentAccelMax);
                Serial.print(F("[MESSAGE] Restored motor velocity to "));
                Serial.print(MOTOR_VELOCITY_RPM);
                Serial.println(F(" RPM"));
            }
            break;
    }
}

bool isHomingComplete() {
    return isHomed && !homingInProgress;
}

void abortHoming() {
    if (homingInProgress) {
        Serial.println(F("[MESSAGE] Aborting homing operation"));
        MOTOR_CONNECTOR.MoveStopAbrupt();
        
        // Reset to normal operation parameters
        currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
        currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
        MOTOR_CONNECTOR.VelMax(currentVelMax);
        MOTOR_CONNECTOR.AccelMax(currentAccelMax);
        
        homingInProgress = false;
        motorState = MOTOR_STATE_FAULTED;
        
        // Reset static variables in executeHomingSequence
        // This is a way to interact with static variables in another function
        executeHomingSequence(); // This will reset homingMovementStarted when it sees !homingInProgress
    } else {
        Serial.println(F("[MESSAGE] No homing operation in progress"));
    }
}

// ----------------- Movement Progress Functions -----------------

void checkMoveProgress() {
    static MotorState previousState = MOTOR_STATE_NOT_READY;
    static bool wasMoving = false;
    
    // Get current movement state
    bool isMoving = !MOTOR_CONNECTOR.StepsComplete();
    
    // Check if we just transitioned to MOVING state
    if (motorState == MOTOR_STATE_MOVING && previousState != MOTOR_STATE_MOVING) {
        wasMoving = true;
        Serial.println(F("[DIAGNOSTIC] Movement started - tracking for LastTarget update"));
    }
    
    // Update the current position when moving
    if (motorState == MOTOR_STATE_MOVING) {
        currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    }

    // Check if the move has just completed (transition from wasMoving to not moving)
    if (wasMoving && !isMoving) {
        Serial.println(F("[DIAGNOSTIC] Move completed successfully"));
        
        // Movement completed successfully, update last target
        if (hasCurrentTarget) {
            Serial.print(F("[DIAGNOSTIC] Updating LastTarget to: "));
            Serial.print(currentTargetPositionMm);
            Serial.println(F(" mm"));
            
            hasLastTarget = true;
            lastTargetType = currentTargetType;
            lastTargetPositionMm = currentTargetPositionMm;
            lastTargetPulses = currentTargetPulses;
            hasCurrentTarget = false; // Clear current target
            
        } else {
            Serial.println(F("[DIAGNOSTIC] Move completed but no current target was set!"));
        }
        
        // Reset the wasMoving flag
        wasMoving = false;
    }
    
    previousState = motorState;
}

// ----------------- E-stop Functions -----------------

// E-stop detection and handling functions
bool isEStopActive() {
    // E-stop is wired as normally closed with pull-up
    // When E-stop is triggered (circuit is opened), pin reads LOW
    return digitalRead(E_STOP_PIN) == LOW;
}

void handleEStop() {
    static bool eStopWasActive = false;
    static unsigned long lastEStopCheckTime = 0;
    
    // Only check periodically to avoid consuming too much processing time
    unsigned long currentTime = millis();
    if (currentTime - lastEStopCheckTime < E_STOP_CHECK_INTERVAL_MS) {
        return;
    }
    lastEStopCheckTime = currentTime;
    
    // Check if E-stop is active
    bool eStopActive = isEStopActive();
    
    // Only take action if E-stop state changes from inactive to active
    if (eStopActive && !eStopWasActive) {
        Serial.println(F("[ERROR] E-STOP TRIGGERED!"));
        
        // If an automated operation is in progress, abort it
        if (currentOperation.inProgress) {
            abortOperation(ABORT_REASON_ESTOP);
        }
        
        // Stop any motion immediately
        MOTOR_CONNECTOR.MoveStopAbrupt();
        
        // Disable the motor
        MOTOR_CONNECTOR.EnableRequest(false);
        
        // If homing was in progress, abort it
        if (homingInProgress) {
            Serial.println(F("[MESSAGE] Aborting homing operation"));
            homingInProgress = false;
        }
        
        // Set motor state to faulted
        motorState = MOTOR_STATE_FAULTED;
    } 
    // Report when E-stop is released
    else if (!eStopActive && eStopWasActive) {
        Serial.println(F("[MESSAGE] E-STOP RELEASED - System remains in fault state until cleared"));
    }
    
    eStopWasActive = eStopActive;
}

// ----------------- Enable Cycle Functions -----------------

void cycleMotorEnableForHoming() {
    if (!motorEnableCycleInProgress) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // First stage - wait 200ms with motor disabled
    if (!motorDisablePhaseComplete && (currentTime - enableCycleStartTime >= 200)) {
        // Re-enable the motor
        MOTOR_CONNECTOR.EnableRequest(true);
        enableCycleStartTime = currentTime;
        motorDisablePhaseComplete = true;
    }
    // Second stage - wait 200ms after re-enabling
    else if (motorDisablePhaseComplete && (currentTime - enableCycleStartTime >= 200)) {
        // Enable cycle is complete
        motorEnableCycleInProgress = false;
    }
}