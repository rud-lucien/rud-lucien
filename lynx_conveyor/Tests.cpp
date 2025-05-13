#include "Tests.h"
#include "MotorController.h"

// Test homing repeatability by performing multiple home-move cycles
bool testHomingRepeatability() {
    const int NUM_CYCLES = 20;          // Number of test cycles to run
    const double TEST_POSITION_MM = 150.0;  // Position to move to during each cycle
    const unsigned long WAIT_TIME_MS = 5000;  // Wait time between operations (5 sec)
    
    int cyclesCompleted = 0;
    bool testRunning = true;
    unsigned long lastActionTime = 0;
    
    // Define all states explicitly instead of using + 1
    enum TestPhase {
        PHASE_START,
        PHASE_INITIAL_HOMING,
        PHASE_WAIT_FOR_HOMING_COMPLETE,    // Renamed: Clearly shows waiting for operation
        PHASE_PAUSE_AFTER_HOMING,          // Renamed: Clearly shows this is just a timed pause
        PHASE_MOVE_TO_POSITION,
        PHASE_WAIT_FOR_MOVE_COMPLETE,      // Renamed: Clearly shows waiting for operation
        PHASE_PAUSE_AFTER_MOVE,            // Renamed: Clearly shows this is just a timed pause
        PHASE_REPEAT_HOMING,
        PHASE_WAIT_FOR_REPEAT_HOME,        // Renamed: Clearer purpose
        PHASE_PAUSE_BEFORE_NEXT_CYCLE,     // Add this new state
        PHASE_COMPLETE
    };
    
    TestPhase currentPhase = PHASE_START;

    // Check for motor alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println(F("[ERROR] Motor has active alerts - clear faults before testing"));
        printMotorAlerts();
        return false;
    }
    
    // Check if motor is initialized
    if (!motorInitialized) {
        Serial.println(F("[ERROR] Motor not initialized - run 'motor init' first"));
        return false;
    }
    
    
    Serial.println(F("[MESSAGE] Starting homing repeatability test"));
    Serial.println(F("[MESSAGE] To abort, type 'a' or 'abort' or any other character"));
    Serial.print(F("[MESSAGE] Will perform "));
    Serial.print(NUM_CYCLES);
    Serial.print(F(" cycles of: home → wait → move to "));
    Serial.print(TEST_POSITION_MM);
    Serial.println(F("mm → wait → repeat"));
    Serial.println(F("[MESSAGE] Press any key to abort test"));
    
    lastActionTime = millis();
    
    while (testRunning) {
        // Check for abort command
        if (Serial.available() > 0) {
            Serial.println(F("[MESSAGE] Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            return false;
        }
        
        // Check for E-Stop condition
        if (isEStopActive()) {
            Serial.println(F("[ERROR] E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            return false;
        }
        
        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();
        
        switch (currentPhase) {
            case PHASE_START:
                Serial.print(F("[MESSAGE] Starting cycle "));
                Serial.print(cyclesCompleted + 1);
                Serial.print(F(" of "));
                Serial.println(NUM_CYCLES);
                currentPhase = PHASE_INITIAL_HOMING;
                lastActionTime = currentTime;
                break;
                
            case PHASE_INITIAL_HOMING:
                // Start homing
                Serial.println(F("[MESSAGE] Homing..."));
                if (!initiateHomingSequence()) {
                    Serial.println(F("[ERROR] Error starting homing operation. Aborting test."));
                    testRunning = false;
                    return false;
                }
                currentPhase = PHASE_WAIT_FOR_HOMING_COMPLETE;
                break;
                
            case PHASE_WAIT_FOR_HOMING_COMPLETE:
                // Print status every 2 seconds for user feedback
                static unsigned long lastStatusPrint = 0;
                if (currentTime - lastStatusPrint > 2000) {
                    Serial.print(F("[DIAGNOSTIC] Waiting for homing to complete. Current state: "));
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.print(F("IDLE")); break;
                        case MOTOR_STATE_MOVING: Serial.print(F("MOVING")); break; 
                        case MOTOR_STATE_HOMING: Serial.print(F("HOMING")); break;
                        case MOTOR_STATE_FAULTED: Serial.print(F("FAULTED")); break;
                        case MOTOR_STATE_NOT_READY: Serial.print(F("NOT_READY")); break;
                        default: Serial.print(F("UNKNOWN"));
                    }
                    Serial.print(F(", Homed: "));
                    Serial.println(isHomed ? F("YES") : F("NO"));
                    lastStatusPrint = currentTime;
                }
                
                // Actively call both functions needed for homing
                if (motorState == MOTOR_STATE_HOMING) {
                    // Process the enable cycling first
                    cycleMotorEnableForHoming();
                    
                    // Then ensure the homing state machine advances
                    executeHomingSequence();
                }
                
                // Wait for homing to complete successfully
                if (motorState == MOTOR_STATE_IDLE && isHomed) {
                    Serial.println(F("[MESSAGE] Homing complete. Waiting..."));
                    lastActionTime = currentTime;
                    currentPhase = PHASE_PAUSE_AFTER_HOMING;  // Use explicit state
                }
                else if (motorState == MOTOR_STATE_FAULTED) {
                    Serial.println(F("[ERROR] Homing failed. Aborting test."));
                    testRunning = false;
                    return false;
                }
                // Use a timeout longer than the internal one in executeHomingSequence (which is 60 seconds)
                // But DO NOT proceed if homing hasn't completed successfully
                else if (currentTime - lastActionTime > 70000) { // 70 seconds (longer than the 60-second internal timeout)
                    Serial.println(F("[ERROR] Timeout waiting for homing to complete."));
                    Serial.print(F("[DIAGNOSTIC] Current state: "));
                    
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.println(F("IDLE")); break;
                        case MOTOR_STATE_MOVING: Serial.println(F("MOVING")); break; 
                        case MOTOR_STATE_HOMING: Serial.println(F("HOMING")); break;
                        case MOTOR_STATE_FAULTED: Serial.println(F("FAULTED")); break;
                        case MOTOR_STATE_NOT_READY: Serial.println(F("NOT_READY")); break;
                        default: Serial.println(F("UNKNOWN"));
                    }
                    
                    // Safety critical: NEVER proceed without successful homing
                    Serial.println(F("[ERROR] CRITICAL: Cannot proceed without successful homing. Aborting test."));
                    stopMotion(); // Stop any motion to be safe
                    testRunning = false; // Abort the test
                    return false;
                }
                break;
                
            case PHASE_PAUSE_AFTER_HOMING:  // Renamed state
                if (currentTime - lastActionTime >= WAIT_TIME_MS) {
                    currentPhase = PHASE_MOVE_TO_POSITION;
                    lastActionTime = currentTime;
                }
                break;
                
            case PHASE_MOVE_TO_POSITION:
                // Move to test position
                Serial.print(F("[MESSAGE] Moving to "));
                Serial.print(TEST_POSITION_MM);
                Serial.println(F("mm..."));
                if (!moveToPositionMm(TEST_POSITION_MM)) {
                    Serial.println(F("[ERROR] Error during movement. Aborting test."));
                    testRunning = false;
                    return false;
                }
                currentPhase = PHASE_WAIT_FOR_MOVE_COMPLETE;
                break;
                
            case PHASE_WAIT_FOR_MOVE_COMPLETE:
                // Add diagnostic printing to track move progress
                static unsigned long lastMoveStatusPrint = 0;
                if (currentTime - lastMoveStatusPrint > 2000) { // Print every 2 seconds
                    Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                    Serial.print(getMotorPositionMm());
                    Serial.print(F("mm, Target: "));
                    Serial.print(TEST_POSITION_MM);
                    Serial.print(F("mm, State: "));
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.print(F("IDLE")); break;
                        case MOTOR_STATE_MOVING: Serial.print(F("MOVING")); break; 
                        case MOTOR_STATE_HOMING: Serial.print(F("HOMING")); break;
                        case MOTOR_STATE_FAULTED: Serial.print(F("FAULTED")); break;
                        case MOTOR_STATE_NOT_READY: Serial.print(F("NOT_READY")); break;
                        default: Serial.print(F("UNKNOWN"));
                    }
                    Serial.print(F(", StepsComplete: "));
                    Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                    lastMoveStatusPrint = currentTime;
                }
                
                // Wait for move to complete
                if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED) {
                    // This is a more reliable way to check for move completion
                    Serial.print(F("[MESSAGE] Position reached: "));
                    Serial.print(getMotorPositionMm());
                    Serial.println(F("mm. Waiting..."));
                    motorState = MOTOR_STATE_IDLE; // Force the state update if needed
                    lastActionTime = currentTime;
                    currentPhase = PHASE_PAUSE_AFTER_MOVE;
                }
                else if (motorState == MOTOR_STATE_FAULTED) {
                    Serial.println(F("[ERROR] Movement failed. Aborting test."));
                    testRunning = false;
                    return false;
                }
                // Add a timeout after a reasonable amount of time
                else if (currentTime - lastActionTime > 60000) { // Increased to 60 seconds
                    Serial.println(F("[ERROR] Timeout waiting for move to complete."));
                    Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                    stopMotion(); // Safety stop
                    testRunning = false;
                    return false;
                }
                break;
                
            case PHASE_PAUSE_AFTER_MOVE:  // Renamed state
                if (currentTime - lastActionTime >= WAIT_TIME_MS) {
                    currentPhase = PHASE_REPEAT_HOMING;
                    lastActionTime = currentTime;
                }
                break;
                
            case PHASE_REPEAT_HOMING:
                // Home again
                Serial.println(F("[MESSAGE] Homing again..."));
                if (!initiateHomingSequence()) {
                    Serial.println(F("[ERROR] Error starting repeat homing operation. Aborting test."));
                    testRunning = false;
                    return false;
                }
                currentPhase = PHASE_WAIT_FOR_REPEAT_HOME;
                break;
                
            case PHASE_WAIT_FOR_REPEAT_HOME:
                // Print status periodically
                static unsigned long lastRepeatStatusPrint = 0;
                if (currentTime - lastRepeatStatusPrint > 2000) {
                    Serial.print(F("[DIAGNOSTIC] Waiting for repeat homing to complete. State: "));
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.print(F("IDLE")); break;
                        case MOTOR_STATE_MOVING: Serial.print(F("MOVING")); break; 
                        case MOTOR_STATE_HOMING: Serial.print(F("HOMING")); break;
                        case MOTOR_STATE_FAULTED: Serial.print(F("FAULTED")); break;
                        case MOTOR_STATE_NOT_READY: Serial.print(F("NOT_READY")); break;
                        default: Serial.print(F("UNKNOWN"));
                    }
                    Serial.print(F(", Homed: "));
                    Serial.println(isHomed ? F("YES") : F("NO"));
                    lastRepeatStatusPrint = currentTime;
                }
                
                // Actively call both functions needed for homing
                if (motorState == MOTOR_STATE_HOMING) {
                    // Process the enable cycling first
                    cycleMotorEnableForHoming();
                    
                    // Then ensure the homing state machine advances
                    executeHomingSequence();
                }
                
                // Wait for homing to complete successfully
                if (motorState == MOTOR_STATE_IDLE && isHomed) {
                    cyclesCompleted++;
                    Serial.print(F("[MESSAGE] Cycle "));
                    Serial.print(cyclesCompleted);
                    Serial.print(F(" completed. Position after homing: "));
                    Serial.print(getMotorPositionMm());
                    Serial.println(F("mm"));
                    
                    if (cyclesCompleted >= NUM_CYCLES) {
                        currentPhase = PHASE_COMPLETE;
                    } else {
                        // Instead of delay, set up for non-blocking pause
                        lastActionTime = currentTime; // Reset timer
                        currentPhase = PHASE_PAUSE_BEFORE_NEXT_CYCLE; // Move to pause state
                    }
                }
                else if (motorState == MOTOR_STATE_FAULTED) {
                    Serial.println(F("[ERROR] Repeat homing failed. Aborting test."));
                    testRunning = false;
                    return false;
                }
                // Use a timeout longer than the internal one
                else if (currentTime - lastActionTime > 70000) {
                    Serial.println(F("[ERROR] Timeout waiting for repeat homing to complete."));
                    Serial.println(F("[ERROR] Cannot proceed without successful homing. Aborting test."));
                    stopMotion(); // Safety stop
                    testRunning = false;
                    return false;
                }
                break;
                
            case PHASE_PAUSE_BEFORE_NEXT_CYCLE:
                if (currentTime - lastActionTime >= 2000) { // 2 second non-blocking pause
                    currentPhase = PHASE_START; // Move to next cycle after pause
                }
                break;
                
            case PHASE_COMPLETE:
                Serial.println(F("[MESSAGE] Homing repeatability test completed successfully."));
                Serial.print(F("[MESSAGE] Completed "));
                Serial.print(cyclesCompleted);
                Serial.println(F(" cycles."));
                testRunning = false;
                return true; // Success! All cycles completed.
                break;
        }
        
        // Give time for processing other operations
        delayMicroseconds(100);
    }
    
    // This line should only be reached if testRunning became false without hitting a return
    // In that case, the test did not complete successfully
    return false;
}

bool testMotorRange() {
    Serial.println(F("[MESSAGE] Testing motor range with small steps..."));
    
    // State variables for non-blocking operation
    enum TestState {
        STATE_START,
        STATE_MOVING,
        STATE_WAIT_BETWEEN_MOVES,
        STATE_RETURN_HOME,
        STATE_COMPLETE
    };
    
    static TestState currentState = STATE_START;
    static int currentStep = 1;
    static unsigned long lastActionTime = 0;
    static int32_t testDistance = 0;
    
    // Check if the system has been homed
    if (!isHomed) {
        Serial.println(F("[ERROR] Motor must be homed before running range test"));
        Serial.println(F("[MESSAGE] Please run the 'home' command first to establish position reference"));
        return false;
    }
    
    // Clear any existing faults first
    clearMotorFaults();
    
    // Set conservative velocity and acceleration
    double testVelocityRpm = 30.0;    // 30 RPM (slow)
    double testAccelRpmPerSec = 500.0; // 500 RPM/s (moderate)
    
    Serial.print(F("[MESSAGE] Setting test velocity to "));
    Serial.print(testVelocityRpm);
    Serial.println(F(" RPM"));
    
    Serial.print(F("[MESSAGE] Setting test acceleration to "));
    Serial.print(testAccelRpmPerSec);
    Serial.println(F(" RPM/s"));
    
    // Convert to pulses for the API
    currentVelMax = rpmToPps(testVelocityRpm);
    currentAccelMax = rpmPerSecToPpsPerSec(testAccelRpmPerSec);
    
    MOTOR_CONNECTOR.VelMax(currentVelMax);
    MOTOR_CONNECTOR.AccelMax(currentAccelMax);
    
    bool testRunning = true;
    
    while (testRunning) {
        // Always check for E-STOP first (non-blocking)
        if (isEStopActive()) {
            Serial.println(F("[ERROR] E-STOP detected during test! Aborting immediately."));
            currentState = STATE_COMPLETE;
            return false;
        }
        
        // Check for user abort
        if (Serial.available() > 0) {
            Serial.println(F("[MESSAGE] Test aborted by user"));
            stopMotion();
            currentState = STATE_COMPLETE;
            return false;
        }
        
        unsigned long currentTime = millis();
        
        switch (currentState) {
            case STATE_START:
                if (currentStep <= 5) {
                    testDistance = currentStep * 100;  // Try 100, 200, 300, 400, 500 pulses
                    
                    // Limit test distance to max travel (positive direction)
                    if (testDistance > MAX_TRAVEL_PULSES) {
                        Serial.print(F("[DIAGNOSTIC] Limiting test distance to maximum travel ("));
                        Serial.print(MAX_TRAVEL_PULSES);
                        Serial.println(F(" pulses)"));
                        testDistance = MAX_TRAVEL_PULSES;
                    }
                    
                    // Ensure we don't try to move in negative direction
                    if ((testDistance * MOTION_DIRECTION) < 0) {
                        Serial.println(F("[DIAGNOSTIC] Warning: Test would move beyond home position - skipping"));
                        testDistance = 0; // Safe position - stay at home
                    }
                    
                    Serial.print(F("[MESSAGE] Testing movement of "));
                    Serial.print(testDistance);
                    Serial.print(F(" pulses (approx. "));
                    Serial.print((double)testDistance / PULSES_PER_REV);
                    Serial.println(F(" revolutions)"));
                    
                    // Move to the test distance
                    MOTOR_CONNECTOR.Move(testDistance, MotorDriver::MOVE_TARGET_ABSOLUTE);
                    lastActionTime = currentTime;
                    currentState = STATE_MOVING;
                } else {
                    // All steps done, return home
                    Serial.println(F("[MESSAGE] Returning to home position..."));
                    MOTOR_CONNECTOR.Move(0, MotorDriver::MOVE_TARGET_ABSOLUTE);
                    lastActionTime = currentTime;
                    currentState = STATE_RETURN_HOME;
                }
                break;
                
            case STATE_MOVING:
                // Check for move completion or fault
                if (MOTOR_CONNECTOR.StepsComplete()) {
                    Serial.print(F("[MESSAGE] Successfully moved to "));
                    Serial.print(testDistance);
                    Serial.println(F(" pulses."));
                    lastActionTime = currentTime;
                    currentState = STATE_WAIT_BETWEEN_MOVES;
                }
                else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
                    Serial.print(F("[ERROR] Motor fault at "));
                    Serial.print(testDistance);
                    Serial.println(F(" pulses."));
                    printMotorAlerts();
                    return false;
                }
                else if (currentTime - lastActionTime > 5000) {
                    // Timeout - 5 seconds should be enough for these short moves
                    Serial.println(F("[ERROR] Move timed out. Aborting test."));
                    return false;
                }
                break;
                
            case STATE_WAIT_BETWEEN_MOVES:
                // Non-blocking wait between moves (500ms)
                if (currentTime - lastActionTime >= 500) {
                    currentStep++;
                    currentState = STATE_START;
                }
                break;
                
            case STATE_RETURN_HOME:
                // Check for home move completion or fault
                if (MOTOR_CONNECTOR.StepsComplete()) {
                    // Test complete, restore original settings
                    currentVelMax = rpmToPps(MOTOR_VELOCITY_RPM);
                    currentAccelMax = rpmPerSecToPpsPerSec(MAX_ACCEL_RPM_PER_SEC);
                    
                    MOTOR_CONNECTOR.VelMax(currentVelMax);
                    MOTOR_CONNECTOR.AccelMax(currentAccelMax);
                    
                    Serial.println(F("[MESSAGE] Motor range test completed successfully."));
                    Serial.println(F("[MESSAGE] Restored original velocity and acceleration limits."));
                    currentState = STATE_COMPLETE;
                    testRunning = false;
                    return true;
                }
                else if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
                    Serial.println(F("[ERROR] Motor fault during return to home."));
                    printMotorAlerts();
                    return false;
                }
                else if (currentTime - lastActionTime > 5000) {
                    // Timeout
                    Serial.println(F("[ERROR] Return to home timed out. Aborting test."));
                    return false;
                }
                break;
                
            case STATE_COMPLETE:
                testRunning = false;
                return true;
        }
        
        delayMicroseconds(100); // Much shorter than delay(10)
    }
    
    // Reset state for next test
    currentState = STATE_START;
    currentStep = 1;
    
    return false; // Should never reach here
}