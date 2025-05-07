#include "Tests.h"
#include "MotorController.h"

// Test homing repeatability by performing multiple home-move cycles
void testHomingRepeatability() {
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
        PHASE_COMPLETE
    };
    
    TestPhase currentPhase = PHASE_START;

    // Check for motor alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Error: Motor has active alerts - clear faults before testing");
        printMotorAlerts();
        return;
    }
    
    // Check if motor is initialized
    if (!motorInitialized) {
        Serial.println("Error: Motor not initialized - run 'motor init' first");
        return;
    }
    
    
    Serial.println("Starting homing repeatability test");
    Serial.println("To abort, type 'a' or 'abort' or any other character");
    Serial.print("Will perform ");
    Serial.print(NUM_CYCLES);
    Serial.print(" cycles of: home → wait → move to ");
    Serial.print(TEST_POSITION_MM);
    Serial.println("mm → wait → repeat");
    Serial.println("Press any key to abort test");
    
    lastActionTime = millis();
    
    while (testRunning) {
        // Check for abort command
        if (Serial.available() > 0) {
            Serial.println("Test aborted by user");
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            break;
        }
        
        // Check for E-Stop condition
        if (isEStopActive()) {
            Serial.println("E-STOP detected during test! Aborting immediately.");
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            break;
        }
        
        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();
        
        switch (currentPhase) {
            case PHASE_START:
                Serial.print("Starting cycle ");
                Serial.print(cyclesCompleted + 1);
                Serial.print(" of ");
                Serial.println(NUM_CYCLES);
                currentPhase = PHASE_INITIAL_HOMING;
                lastActionTime = currentTime;
                break;
                
            case PHASE_INITIAL_HOMING:
                // Start homing
                Serial.println("Homing...");
                if (!initiateHomingSequence()) {
                    Serial.println("Error starting homing operation. Aborting test.");
                    testRunning = false;
                    break;
                }
                currentPhase = PHASE_WAIT_FOR_HOMING_COMPLETE;
                break;
                
            case PHASE_WAIT_FOR_HOMING_COMPLETE:
                // Print status every 2 seconds for user feedback
                static unsigned long lastStatusPrint = 0;
                if (currentTime - lastStatusPrint > 2000) {
                    Serial.print("Waiting for homing to complete. Current state: ");
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.print("IDLE"); break;
                        case MOTOR_STATE_MOVING: Serial.print("MOVING"); break; 
                        case MOTOR_STATE_HOMING: Serial.print("HOMING"); break;
                        case MOTOR_STATE_FAULTED: Serial.print("FAULTED"); break;
                        case MOTOR_STATE_NOT_READY: Serial.print("NOT_READY"); break;
                        default: Serial.print("UNKNOWN");
                    }
                    Serial.print(", Homed: ");
                    Serial.println(isHomed ? "YES" : "NO");
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
                    Serial.println("Homing complete. Waiting...");
                    lastActionTime = currentTime;
                    currentPhase = PHASE_PAUSE_AFTER_HOMING;  // Use explicit state
                }
                else if (motorState == MOTOR_STATE_FAULTED) {
                    Serial.println("Homing failed. Aborting test.");
                    testRunning = false;
                }
                // Use a timeout longer than the internal one in executeHomingSequence (which is 60 seconds)
                // But DO NOT proceed if homing hasn't completed successfully
                else if (currentTime - lastActionTime > 70000) { // 70 seconds (longer than the 60-second internal timeout)
                    Serial.println("Timeout waiting for homing to complete.");
                    Serial.print("Current state: ");
                    
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.println("IDLE"); break;
                        case MOTOR_STATE_MOVING: Serial.println("MOVING"); break; 
                        case MOTOR_STATE_HOMING: Serial.println("HOMING"); break;
                        case MOTOR_STATE_FAULTED: Serial.println("FAULTED"); break;
                        case MOTOR_STATE_NOT_READY: Serial.println("NOT_READY"); break;
                        default: Serial.println("UNKNOWN");
                    }
                    
                    // Safety critical: NEVER proceed without successful homing
                    Serial.println("CRITICAL: Cannot proceed without successful homing. Aborting test.");
                    stopMotion(); // Stop any motion to be safe
                    testRunning = false; // Abort the test
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
                Serial.print("Moving to ");
                Serial.print(TEST_POSITION_MM);
                Serial.println("mm...");
                if (!moveToPositionMm(TEST_POSITION_MM)) {
                    Serial.println("Error during movement. Aborting test.");
                    testRunning = false;
                    break;
                }
                currentPhase = PHASE_WAIT_FOR_MOVE_COMPLETE;
                break;
                
            case PHASE_WAIT_FOR_MOVE_COMPLETE:
                // Add diagnostic printing to track move progress
                static unsigned long lastMoveStatusPrint = 0;
                if (currentTime - lastMoveStatusPrint > 2000) { // Print every 2 seconds
                    Serial.print("Move status - Position: ");
                    Serial.print(getMotorPositionMm());
                    Serial.print("mm, Target: ");
                    Serial.print(TEST_POSITION_MM);
                    Serial.print("mm, State: ");
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.print("IDLE"); break;
                        case MOTOR_STATE_MOVING: Serial.print("MOVING"); break; 
                        case MOTOR_STATE_HOMING: Serial.print("HOMING"); break;
                        case MOTOR_STATE_FAULTED: Serial.print("FAULTED"); break;
                        case MOTOR_STATE_NOT_READY: Serial.print("NOT_READY"); break;
                        default: Serial.print("UNKNOWN");
                    }
                    Serial.print(", StepsComplete: ");
                    Serial.println(MOTOR_CONNECTOR.StepsComplete() ? "YES" : "NO");
                    lastMoveStatusPrint = currentTime;
                }
                
                // Wait for move to complete
                if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED) {
                    // This is a more reliable way to check for move completion
                    Serial.print("Position reached: ");
                    Serial.print(getMotorPositionMm());
                    Serial.println("mm. Waiting...");
                    motorState = MOTOR_STATE_IDLE; // Force the state update if needed
                    lastActionTime = currentTime;
                    currentPhase = PHASE_PAUSE_AFTER_MOVE;
                }
                else if (motorState == MOTOR_STATE_FAULTED) {
                    Serial.println("Movement failed. Aborting test.");
                    testRunning = false;
                }
                // Add a timeout after a reasonable amount of time
                else if (currentTime - lastActionTime > 60000) { // Increased to 60 seconds
                    Serial.println("Timeout waiting for move to complete.");
                    Serial.println("Movement took too long. Aborting test.");
                    stopMotion(); // Safety stop
                    testRunning = false;
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
                Serial.println("Homing again...");
                if (!initiateHomingSequence()) {
                    Serial.println("Error starting repeat homing operation. Aborting test.");
                    testRunning = false;
                    break;
                }
                currentPhase = PHASE_WAIT_FOR_REPEAT_HOME;
                break;
                
            case PHASE_WAIT_FOR_REPEAT_HOME:
                // Print status periodically
                static unsigned long lastRepeatStatusPrint = 0;
                if (currentTime - lastRepeatStatusPrint > 2000) {
                    Serial.print("Waiting for repeat homing to complete. State: ");
                    switch (motorState) {
                        case MOTOR_STATE_IDLE: Serial.print("IDLE"); break;
                        case MOTOR_STATE_MOVING: Serial.print("MOVING"); break; 
                        case MOTOR_STATE_HOMING: Serial.print("HOMING"); break;
                        case MOTOR_STATE_FAULTED: Serial.print("FAULTED"); break;
                        case MOTOR_STATE_NOT_READY: Serial.print("NOT_READY"); break;
                        default: Serial.print("UNKNOWN");
                    }
                    Serial.print(", Homed: ");
                    Serial.println(isHomed ? "YES" : "NO");
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
                    Serial.print("Cycle ");
                    Serial.print(cyclesCompleted);
                    Serial.print(" completed. Position after homing: ");
                    Serial.print(getMotorPositionMm());
                    Serial.println("mm");
                    
                    if (cyclesCompleted >= NUM_CYCLES) {
                        currentPhase = PHASE_COMPLETE;
                    } else {
                        // Brief pause before starting next cycle
                        delay(2000);
                        currentPhase = PHASE_START;
                    }
                }
                else if (motorState == MOTOR_STATE_FAULTED) {
                    Serial.println("Repeat homing failed. Aborting test.");
                    testRunning = false;
                }
                // Use a timeout longer than the internal one
                else if (currentTime - lastActionTime > 70000) {
                    Serial.println("Timeout waiting for repeat homing to complete.");
                    Serial.println("Cannot proceed without successful homing. Aborting test.");
                    stopMotion(); // Safety stop
                    testRunning = false;
                }
                break;
                
            case PHASE_COMPLETE:
                Serial.println("Homing repeatability test completed successfully.");
                Serial.print("Completed ");
                Serial.print(cyclesCompleted);
                Serial.println(" cycles.");
                testRunning = false;
                break;
        }
        
        // Give time for processing other operations
        delay(10);
    }
}