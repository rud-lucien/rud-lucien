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

// Convert mm to pulses
int32_t mmToPulses(double mm) {
    return (int32_t)(mm * PULSES_PER_MM);
}

// Convert pulses to mm
double pulsesToMm(int32_t pulses) {
    return (double)pulses / PULSES_PER_MM;
}

// ----------------- Basic Setup Functions -----------------

void initMotorSystem() {
    Serial.println("Initializing motor system...");
    
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
    Serial.print(MAX_VELOCITY_RPM);
    Serial.println(" RPM");
    
    Serial.print("Setting acceleration limit to ");
    Serial.print(MAX_ACCEL_RPM_PER_SEC);
    Serial.println(" RPM/s");
    
    // Convert RPM values to pulses/sec for the ClearCore API
    currentVelMax = rpmToPps(MAX_VELOCITY_RPM);
    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
    
    MOTOR_CONNECTOR.VelMax(currentVelMax);
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

bool moveToAbsolutePosition(int32_t position) {
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
    int32_t pulsePosition = mmToPulses(positionMm);
    
    if (moveToAbsolutePosition(pulsePosition)) {
        currentPosition = POSITION_CUSTOM;
        currentPositionMm = positionMm;
        motorState = MOTOR_STATE_MOVING;
        return true;
    }
    
    return false;
}

bool moveRelative(double distanceMm) {
    // Calculate new absolute position based on current position + distance
    double newPositionMm = currentPositionMm + distanceMm;
    
    // Check if the new position is in valid range
    if (newPositionMm < 0 || newPositionMm > POSITION_5_MM) {
        Serial.println("Error: Relative move would exceed valid position range");
        return false;
    }
    
    return moveToPositionMm(newPositionMm);
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
        
        Serial.print("Testing movement of ");
        Serial.print(testDistance);
        Serial.print(" pulses (approx. ");
        Serial.print((double)testDistance / PULSES_PER_REV);
        Serial.println(" revolutions)");
        
        // Move forward by the test distance
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
    currentVelMax = rpmToPps(MAX_VELOCITY_RPM);
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
    // Note: This assumes the sensor is active-low. Adjust if needed.
    bool currentSensorState = digitalRead(HOME_SENSOR_PIN) == LOW;
    
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
    // Check if motor is ready for homing
    if (!motorInitialized || MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Error: Motor not ready for homing");
        return false;
    }
    
    // Set lower velocity for homing
    double homingVelocityRpm = HOME_VELOCITY_RPM; 
    currentVelMax = rpmToPps(homingVelocityRpm);
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    
    Serial.print("Starting homing with velocity: ");
    Serial.print(homingVelocityRpm);
    Serial.println(" RPM");
    
    // Move in the negative direction (towards home sensor)
    // Use a large number to ensure we reach the sensor
    MOTOR_CONNECTOR.Move(-100000, MotorDriver::MOVE_TARGET_ABSOLUTE);
    
    // Set homing flags
    homingInProgress = true;
    motorState = MOTOR_STATE_HOMING;
    
    return true;
}

void checkHomingProgress() {
    if (!homingInProgress) {
        return;
    }
    
    // Check if the home sensor has been triggered
    if (homeSensorTriggered()) {
        Serial.println("Home sensor triggered. Stopping motor.");
        stopMotion();
        
        // Set position to zero
        MOTOR_CONNECTOR.PositionRefSet(0);
        currentPositionMm = 0.0;
        currentPosition = POSITION_1;
        
        // Reset motor parameters to normal operation
        currentVelMax = rpmToPps(VELOCITY_LIMIT_RPM);
        MOTOR_CONNECTOR.VelMax(currentVelMax);
        
        // Set homing flags
        homingInProgress = false;
        isHomed = true;
        motorState = MOTOR_STATE_IDLE;
        
        Serial.println("Homing completed successfully.");
    }
    
    // Check for timeout
    static unsigned long homingStartTime = millis();
    if (millis() - homingStartTime > HOME_TIMEOUT_MS) {
        Serial.println("Error: Homing operation timed out");
        stopMotion();
        homingInProgress = false;
        motorState = MOTOR_STATE_FAULTED;
    }
}

bool isHomingComplete() {
    return isHomed && !homingInProgress;
}

// ----------------- Movement Progress Functions -----------------

void checkMoveProgress() {
    // Only check if the motor is in the moving state
    if (motorState != MOTOR_STATE_MOVING) {
        return;
    }
    
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
    
    // Convert raw state to sensor triggered status (assuming active-low)
    bool sensorTriggered = (pinState == LOW);
    
    Serial.print("Home sensor (pin A10) state: ");
    Serial.print(pinState ? "HIGH" : "LOW");
    Serial.print(" (Sensor is ");
    Serial.print(sensorTriggered ? "TRIGGERED" : "NOT TRIGGERED");
    Serial.println(")");
}