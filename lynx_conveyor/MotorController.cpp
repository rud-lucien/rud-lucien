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
unsigned long homingStartTime = 0;

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

// ----------------- Basic Setup Functions -----------------

void initMotorSystem() {
    Serial.println("Initializing motor system...");
    
    // Set up E-stop input pin with internal pull-up
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    Serial.print("Checking E-Stop state: ");
    if (isEStopActive()) {
        Serial.println("E-STOP ACTIVE! Please reset E-stop before continuing.");
    } else {
        Serial.println("E-stop inactive, system ready.");
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
    Serial.print("Setting velocity limit to ");
    Serial.print(MOTOR_VELOCITY_RPM); // CHANGED: Using consolidated constant
    Serial.println(" RPM");
    
    currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM); // CHANGED
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    
    // Set acceleration limit
    Serial.print("Setting acceleration limit to ");
    Serial.print(MAX_ACCEL_RPM_PER_SEC);
    Serial.println(" RPM/s");
    
    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);
    
    // Enable the motor
    MOTOR_CONNECTOR.EnableRequest(true);
    Serial.println("Motor enable requested");
    
    // Wait for HLFB to assert (up to 2 seconds)
    Serial.println("Waiting for HLFB...");
    unsigned long startTime = millis();
    bool ready = false;
    
    while (!ready && millis() - startTime < 2000) {
        if (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
            ready = true;
        } else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
            Serial.println("Motor alert detected:");
            printMotorAlerts();
            break;
        }
        delay(10);
    }
    
    if (ready) {
        Serial.println("Motor initialized and ready");
        motorInitialized = true;
    } else {
        Serial.println("Motor initialization timed out or failed");
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
        Serial.print("Error: Requested position ");
        Serial.print(position);
        Serial.print(" pulses is outside valid range (0 to ");
        Serial.print(MOTION_DIRECTION * MAX_TRAVEL_PULSES);
        Serial.println(" pulses)");
        return false;
    }
    
    // Check if motor has alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor alert detected. Cannot move.");
        printMotorAlerts();
        return false;
    }
    
    Serial.print("Moving to absolute position: ");
    Serial.println(position);
    
    // Command the absolute move
    MOTOR_CONNECTOR.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Report initial status
    Serial.println("Move commanded. Motor in motion...");
    
    return true;
}

bool moveToPosition(int positionNumber) {
    int32_t targetPosition;
    
    // Determine which position to move to
    switch (positionNumber) {
        case 1:
            targetPosition = POSITION_1_PULSES;
            break;
        case 2:
            targetPosition = POSITION_2_PULSES;
            break;
        case 3:
            targetPosition = POSITION_3_PULSES;
            break;
        case 4:
            targetPosition = POSITION_4_PULSES;
            break;
        case 5:
            targetPosition = POSITION_5_PULSES;
            break;
        default:
            Serial.println("Invalid position number. Use 1-5.");
            return false;
    }
    
    return moveToAbsolutePosition(targetPosition);
}

bool moveToPosition(PositionTarget position) {
    double targetPositionMm = 0.0;
    
    switch (position) {
        case POSITION_1:
            targetPositionMm = POSITION_1_MM;
            break;
        case POSITION_2:
            targetPositionMm = POSITION_2_MM;
            break;
        case POSITION_3:
            targetPositionMm = POSITION_3_MM;
            break;
        case POSITION_4:
            targetPositionMm = POSITION_4_MM;
            break;
        case POSITION_5:
            targetPositionMm = POSITION_5_MM;
            break;
        default:
            Serial.println("Invalid position target");
            return false;
    }
    
    return moveToPositionMm(targetPositionMm);
}

bool moveToPositionMm(double positionMm) {
    // Safety check - prevent movement beyond physical limits
    if (positionMm < 0 || positionMm > MAX_TRAVEL_MM) {
        Serial.print("Error: Requested position ");
        Serial.print(positionMm);
        Serial.print(" mm is outside valid range (0 to ");
        Serial.print(MAX_TRAVEL_MM);
        Serial.println(" mm)");
        return false;
    }
    
    int32_t pulsePosition = mmToPulses(positionMm);
    
    if (moveToAbsolutePosition(pulsePosition)) {
        currentPosition = POSITION_CUSTOM;
        currentPositionMm = positionMm;
        motorState = MOTOR_STATE_MOVING;
        return true;
    }
    
    return false;
}

bool moveRelative(double relativeMm) {
    // Calculate the target absolute position
    double targetPositionMm = currentPositionMm + relativeMm;
    
    // Check if the target position would be out of bounds
    if (targetPositionMm < 0 || targetPositionMm > MAX_TRAVEL_MM) {
        Serial.print("Error: Relative move would exceed valid range (0 to ");
        Serial.print(MAX_TRAVEL_MM);
        Serial.println(" mm)");
        Serial.print("Current position: ");
        Serial.print(currentPositionMm);
        Serial.print(" mm, Requested move: ");
        Serial.print(relativeMm);
        Serial.print(" mm, Target would be: ");
        Serial.print(targetPositionMm);
        Serial.println(" mm");
        return false;
    }
    
    // Convert millimeters to pulses
    int32_t relativePulses = mmToPulses(relativeMm);
    
    // Call the motor's relative move function
    MOTOR_CONNECTOR.Move(relativePulses, MotorDriver::MOVE_TARGET_REL_END_POSN);
    
    // Update state
    motorState = MOTOR_STATE_MOVING;
    currentPosition = POSITION_CUSTOM;
    
    Serial.print("Moving ");
    Serial.print(relativeMm);
    Serial.print(" mm from current position (");
    Serial.print(relativePulses);
    Serial.println(" pulses)");
    
    return true;
}

double getMotorPositionMm() {
    return pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
}

void stopMotion() {
    // Stop the motor abruptly
    MOTOR_CONNECTOR.MoveStopAbrupt();
    Serial.println("Motion stopped");
}

// ----------------- Status Functions -----------------

bool isMotorReady() {
    return motorInitialized && 
           MOTOR_CONNECTOR.EnableRequest() && 
           MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED &&
           !MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent;
}

bool isMotorMoving() {
    return !MOTOR_CONNECTOR.StepsComplete() || 
           MOTOR_CONNECTOR.HlfbState() != MotorDriver::HLFB_ASSERTED;
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
    // Check if motor is moving
    else if (isMotorMoving()) {
        motorState = MOTOR_STATE_MOVING;
    }
    // Otherwise, motor is idle
    else {
        motorState = MOTOR_STATE_IDLE;
    }
    
    return motorState;
}

void printMotorStatus() {
    Serial.println("Motor Status:");
    
    Serial.print("  Enabled: ");
    Serial.println(MOTOR_CONNECTOR.EnableRequest() ? "Yes" : "No");
    
    Serial.print("  Moving: ");
    Serial.println(isMotorAtPosition() ? "No" : "Yes");
    
    Serial.print("  Position: ");
    Serial.print(MOTOR_CONNECTOR.PositionRefCommanded());
    Serial.println(" pulses");
    
    Serial.print("  Current Velocity Limit: ");
    Serial.print(ppsToRpm(currentVelMax));
    Serial.println(" RPM");
    
    Serial.print("  Current Acceleration Limit: ");
    double accelRpmPerSec = (double)currentAccelMax * 60.0 / PULSES_PER_REV;
    Serial.print(accelRpmPerSec);
    Serial.println(" RPM/s");
    
    Serial.print("  HLFB Status: ");
    switch (MOTOR_CONNECTOR.HlfbState()) {
        case MotorDriver::HLFB_ASSERTED:
            Serial.println("Asserted (In Position/Ready)");
            break;
        case MotorDriver::HLFB_DEASSERTED:
            Serial.println("Deasserted (Moving/Fault)");
            break;
        case MotorDriver::HLFB_UNKNOWN:
        default:
            Serial.println("Unknown");
            break;
    }
    
    // Print any alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("  Alerts present:");
        printMotorAlerts();
    } else {
        Serial.println("  No alerts");
    }
}

void printMotorAlerts() {
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledInAlert) {
        Serial.println("    MotionCanceledInAlert");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledPositiveLimit) {
        Serial.println("    MotionCanceledPositiveLimit");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledNegativeLimit) {
        Serial.println("    MotionCanceledNegativeLimit");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledSensorEStop) {
        Serial.println("    MotionCanceledSensorEStop");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotionCanceledMotorDisabled) {
        Serial.println("    MotionCanceledMotorDisabled");
    }
    if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted) {
        Serial.println("    MotorFaulted");
    }
}

void clearMotorFaults() {
    Serial.println("Attempting to clear motor faults...");
    
    // First check if there are faults present
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Alerts detected:");
        printMotorAlerts();
        
        // If a motor fault is present, clear it by cycling enable
        if (MOTOR_CONNECTOR.AlertReg().bit.MotorFaulted) {
            Serial.println("Motor faulted. Cycling enable signal...");
            MOTOR_CONNECTOR.EnableRequest(false);
            delay(100);  // Give more time for disable to take effect
            MOTOR_CONNECTOR.EnableRequest(true);
            delay(100);  // Give more time for enable to take effect
        }
        
        // Clear alerts
        Serial.println("Clearing motor alerts...");
        MOTOR_CONNECTOR.ClearAlerts();
        
        // Verify alerts were cleared
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
            Serial.println("Warning: Alerts are still present after clearing.");
            printMotorAlerts();
        } else {
            Serial.println("Alerts successfully cleared.");
        }
    } else {
        Serial.println("No alerts to clear.");
    }
}

void clearMotorFault() {
    clearMotorFaults(); // Use the existing implementation
}

bool testMotorRange() {
    Serial.println("Testing motor range with small steps...");
    
    // Check if the system has been homed
    if (!isHomed) {
        Serial.println("Error: Motor must be homed before running range test");
        Serial.println("Please run the 'home' command first to establish position reference");
        return false;
    }
    
    // Clear any existing faults first
    clearMotorFaults();
    
    // Set conservative velocity and acceleration
    double testVelocityRpm = 30.0;    // 30 RPM (slow)
    double testAccelRpmPerSec = 500.0; // 500 RPM/s (moderate)
    
    Serial.print("Setting test velocity to ");
    Serial.print(testVelocityRpm);
    Serial.println(" RPM");
    
    Serial.print("Setting test acceleration to ");
    Serial.print(testAccelRpmPerSec);
    Serial.println(" RPM/s");
    
    // Convert to pulses for the API
    currentVelMax = rpmToPps(testVelocityRpm);
    currentAccelMax = rpmPerSecToPpsPerSec(testAccelRpmPerSec);
    
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);
    
    // Start with very small movements
    for (int i = 1; i <= 5; i++) {
        int32_t testDistance = i * 100;  // Try 100, 200, 300, 400, 500 pulses
        
        // Limit test distance to max travel (positive direction)
        if (testDistance > MAX_TRAVEL_PULSES) {
            Serial.print("Limiting test distance to maximum travel (");
            Serial.print(MAX_TRAVEL_PULSES);
            Serial.println(" pulses)");
            testDistance = MAX_TRAVEL_PULSES;
        }
        
        // Ensure we don't try to move in negative direction
        // Since positive pulses move toward maximum travel with MOTION_DIRECTION=-1
        if ((testDistance * MOTION_DIRECTION) < 0) {
            Serial.println("Warning: Test would move beyond home position - skipping");
            testDistance = 0; // Safe position - stay at home
        }
        
        Serial.print("Testing movement of ");
        Serial.print(testDistance);
        Serial.print(" pulses (approx. ");
        Serial.print((double)testDistance / PULSES_PER_REV);
        Serial.println(" revolutions)");
        
        // Move to the test distance
        MOTOR_CONNECTOR.Move(testDistance, MotorDriver::MOVE_TARGET_ABSOLUTE);
        
        // Wait for move to complete or fault
        unsigned long startTime = millis();
        while (!MOTOR_CONNECTOR.StepsComplete() && 
               !MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent &&
               millis() - startTime < 2000) {
            delay(10);
        }
        
        // Check what happened
        if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
            Serial.print("Motor fault at ");
            Serial.print(testDistance);
            Serial.println(" pulses.");
            printMotorAlerts();
            return false;
        }
        
        Serial.print("Successfully moved to ");
        Serial.print(testDistance);
        Serial.println(" pulses.");
        delay(500);  // Pause between movements
    }
    
    // Return to position 0
    Serial.println("Returning to home position...");
    MOTOR_CONNECTOR.Move(0, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Wait for move to complete
    unsigned long startTime = millis();
    while (!MOTOR_CONNECTOR.StepsComplete() && 
           !MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent &&
           millis() - startTime < 2000) {
        delay(10);
    }
    
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor fault during return to home.");
        printMotorAlerts();
        return false;
    }
    
    // Restore original velocity and acceleration
    currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);
    
    Serial.println("Motor range test completed successfully.");
    Serial.println("Restored original velocity and acceleration limits.");
    return true;
}

// ----------------- Homing Functions -----------------

bool homeSensorTriggered() {
    static bool lastSensorState = false;
    static unsigned long lastChangeTime = 0;
    
    // Read the current state of the home sensor
    // Note: This assumes the sensor is active-high. Adjust if needed.
    bool currentSensorState = digitalRead(HOME_SENSOR_PIN) == HIGH;
    
    // If the state has changed, record the time
    if (currentSensorState != lastSensorState) {
        lastChangeTime = millis();
        lastSensorState = currentSensorState;
    }
    
    // Only report the sensor as triggered if the state has been stable for the debounce period
    if (currentSensorState && (millis() - lastChangeTime > SENSOR_DEBOUNCE_MS)) {
        return true;
    }
    
    return false;
}

bool startHoming() {
    // Check if motor is initialized
    if (!motorInitialized) {
        Serial.println("Error: Motor not initialized - run 'motor init' first");
        return false;
    }
    
    // Check for motor alerts separately
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Error: Motor has active alerts - clear faults before homing");
        printMotorAlerts(); // Print the specific alerts
        return false;
    }
    
    // Set lower velocity for homing
    double homingVelocityRpm = HOME_VELOCITY_RPM; 
    currentVelMax = rpmToPps(homingVelocityRpm);
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    
    // Initialize homing state
    homingInProgress = true;
    motorState = MOTOR_STATE_HOMING;
    homingStartTime = millis();
    
    // Split homing into discrete steps
    // We'll use this homing step flag to track where we are in the process
    static enum {
        HOMING_STEP_CHECK_SENSOR,
        HOMING_STEP_MOVE_AWAY,
        HOMING_STEP_WAIT_MOVE_AWAY,
        HOMING_STEP_STABILIZE,
        HOMING_STEP_MOVE_TO_HOME,
        HOMING_STEP_COMPLETE
    } homingStep = HOMING_STEP_CHECK_SENSOR;
    
    // Always start with checking the sensor
    homingStep = HOMING_STEP_CHECK_SENSOR;
    Serial.println("Starting homing sequence...");
    
    return true;
}

void checkHomingProgress() {
    if (!homingInProgress) {
        return;
    }
    
    // Static variables to maintain state between function calls
    static enum {
        HOMING_STEP_CHECK_SENSOR,
        HOMING_STEP_MOVE_AWAY,
        HOMING_STEP_WAIT_MOVE_AWAY,
        HOMING_STEP_STABILIZE,
        HOMING_STEP_MOVE_TO_HOME,
        HOMING_STEP_COMPLETE
    } homingStep = HOMING_STEP_CHECK_SENSOR;
    
    static unsigned long stabilizationStartTime = 0;
    
    // Check for timeout first
    if (millis() - homingStartTime > HOME_TIMEOUT_MS) {
        Serial.println("Error: Homing operation timed out");
        stopMotion();
        homingInProgress = false;
        motorState = MOTOR_STATE_FAULTED;
        homingStep = HOMING_STEP_CHECK_SENSOR; // Reset for next time
        return;
    }
    
    // State machine for homing
    switch (homingStep) {
        case HOMING_STEP_CHECK_SENSOR:
            // Check if home sensor is already triggered
            if (homeSensorTriggered()) {
                Serial.println("Home sensor is already triggered - moving away before homing");
                homingStep = HOMING_STEP_MOVE_AWAY;
            } else {
                Serial.println("Home sensor not triggered - starting homing movement");
                homingStep = HOMING_STEP_MOVE_TO_HOME;
            }
            break;
            
        case HOMING_STEP_MOVE_AWAY:
            // Move away in the opposite direction of the homing direction
            {
                // Convert mm to pulses
                double moveAwayMm = MOVE_AWAY_DISTANCE_MM;
                int32_t moveAwayPulses = mmToPulses(moveAwayMm);
                
                // Apply direction - use negative to move away
                moveAwayPulses = -HOMING_DIRECTION * abs(moveAwayPulses);
                
                Serial.print("Moving away from home sensor by ");
                Serial.print(moveAwayMm);
                Serial.println(" mm");
                
                // Execute the move away command as a relative move
                MOTOR_CONNECTOR.Move(moveAwayPulses, MotorDriver::MOVE_TARGET_REL_END_POSN);
                homingStep = HOMING_STEP_WAIT_MOVE_AWAY;
            }
            break;
            
        case HOMING_STEP_WAIT_MOVE_AWAY:
            // Wait until the sensor is no longer triggered or the move is complete
            if (!homeSensorTriggered() || MOTOR_CONNECTOR.StepsComplete()) {
                if (!homeSensorTriggered()) {
                    Serial.println("Successfully moved away from home sensor");
                } else {
                    Serial.println("Move away completed but sensor still triggered");
                    Serial.println("Continuing with caution...");
                }
                
                // Wait for mechanical stabilization
                stabilizationStartTime = millis();
                homingStep = HOMING_STEP_STABILIZE;
            }
            break;
            
        case HOMING_STEP_STABILIZE:
            // Wait for mechanical stabilization
            if (millis() - stabilizationStartTime >= 200) {
                Serial.println("Stabilization complete, beginning homing movement");
                homingStep = HOMING_STEP_MOVE_TO_HOME;
            }
            break;
            
        case HOMING_STEP_MOVE_TO_HOME:
            // Move toward the home sensor
            {
                Serial.print("Moving to home with velocity: ");
                Serial.print(HOME_VELOCITY_RPM);
                Serial.println(" RPM");
                
                // Move in the direction specified by HOMING_DIRECTION
                // Use a large number to ensure we reach the sensor
                MOTOR_CONNECTOR.Move(HOMING_DIRECTION * 100000, MotorDriver::MOVE_TARGET_ABSOLUTE);
                
                Serial.println("Moving toward home position...");
                homingStep = HOMING_STEP_COMPLETE;
            }
            break;
            
        case HOMING_STEP_COMPLETE:
            // Check if the home sensor has been triggered
            if (homeSensorTriggered()) {
                Serial.println("Home sensor triggered. Stopping motor.");
                stopMotion();
                
                // Set position to zero at the home position
                MOTOR_CONNECTOR.PositionRefSet(0);
                currentPositionMm = 0.0;
                currentPosition = POSITION_1;
                
                // Reset motor parameters to normal operation
                currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
                MOTOR_CONNECTOR.VelMax(currentVelMax);
                
                // Set homing flags
                homingInProgress = false;
                isHomed = true;
                motorState = MOTOR_STATE_IDLE;
                
                Serial.println("Homing completed successfully.");
                
                // Reset for next homing operation
                homingStep = HOMING_STEP_CHECK_SENSOR;
            }
            break;
    }
}

bool isHomingComplete() {
    return isHomed && !homingInProgress;
}

void abortHoming() {
    if (homingInProgress) {
        Serial.println("Aborting homing operation");
        MOTOR_CONNECTOR.MoveStopAbrupt();
        homingInProgress = false;
        motorState = MOTOR_STATE_FAULTED;
    } else {
        Serial.println("No homing operation in progress");
    }
}

// ----------------- Movement Progress Functions -----------------

void checkMoveProgress() {
    // Only check if the motor is in the moving state
    if (motorState != MOTOR_STATE_MOVING) {
        return;
    }
    
    // Update the current position based on the commanded position
    currentPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    
    // Check if the move has completed
    if (isMotorInPosition()) {
        Serial.println("Move completed successfully");
        motorState = MOTOR_STATE_IDLE;
    }
    
    // Check for motor faults during motion
    else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor fault detected during movement:");
        printMotorAlerts();
        motorState = MOTOR_STATE_FAULTED;
    }
}

// ----------------- Sensor Functions -----------------

void checkHomeSensor() {
    // Read the raw pin state
    bool pinState = digitalRead(HOME_SENSOR_PIN);
    
    // Convert raw state to sensor triggered status (assuming active-high)
    bool sensorTriggered = (pinState == HIGH);
    
    Serial.print("Home sensor (pin A10) state: ");
    Serial.print(pinState ? "HIGH" : "LOW");
    Serial.print(" (Sensor is ");
    Serial.print(sensorTriggered ? "TRIGGERED" : "NOT TRIGGERED");
    Serial.println(")");
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
        Serial.println("E-STOP TRIGGERED!");
        
        // Stop any motion immediately
        MOTOR_CONNECTOR.MoveStopAbrupt();
        
        // Disable the motor
        MOTOR_CONNECTOR.EnableRequest(false);
        
        // If homing was in progress, abort it
        if (homingInProgress) {
            Serial.println("Aborting homing operation");
            homingInProgress = false;
        }
        
        // Set motor state to faulted
        motorState = MOTOR_STATE_FAULTED;
    } 
    // Report when E-stop is released
    else if (!eStopActive && eStopWasActive) {
        Serial.println("E-STOP RELEASED - System remains in fault state until cleared");
    }
    
    eStopWasActive = eStopActive;
}