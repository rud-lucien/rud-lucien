#include "Tests.h"
#include "MotorController.h"
#include "Utils.h"
#include "ValveController.h"

// Test homing repeatability by performing multiple home-move cycles
bool testHomingRepeatability()
{
    const int NUM_CYCLES = 20;               // Number of test cycles to run
    const double TEST_POSITION_MM = 150.0;   // Position to move to during each cycle
    const unsigned long WAIT_TIME_MS = 5000; // Wait time between operations (5 sec)

    int cyclesCompleted = 0;
    bool testRunning = true;
    unsigned long lastActionTime = 0;

    // Define all states explicitly instead of using + 1
    enum TestPhase
    {
        PHASE_START,
        PHASE_INITIAL_HOMING,
        PHASE_WAIT_FOR_HOMING_COMPLETE, // Renamed: Clearly shows waiting for operation
        PHASE_PAUSE_AFTER_HOMING,       // Renamed: Clearly shows this is just a timed pause
        PHASE_MOVE_TO_POSITION,
        PHASE_WAIT_FOR_MOVE_COMPLETE, // Renamed: Clearly shows waiting for operation
        PHASE_PAUSE_AFTER_MOVE,       // Renamed: Clearly shows this is just a timed pause
        PHASE_REPEAT_HOMING,
        PHASE_WAIT_FOR_REPEAT_HOME,    // Renamed: Clearer purpose
        PHASE_PAUSE_BEFORE_NEXT_CYCLE, // Add this new state
        PHASE_COMPLETE
    };

    TestPhase currentPhase = PHASE_START;

    // Check for motor alerts
    if (MOTOR_CONNECTOR.StatusReg().bit.AlertsPresent)
    {
        Serial.println(F("[ERROR] Motor has active alerts - clear faults before testing"));
        printMotorAlerts();
        return false;
    }

    // Check if motor is initialized
    if (!motorInitialized)
    {
        Serial.println(F("[ERROR] Motor not initialized - run 'motor init' first"));
        return false;
    }

    Serial.println(F("[MESSAGE] Starting homing repeatability test"));
    Serial.println(F("[MESSAGE] To abort, type 'a' or 'abort' or any other character"));
    Serial.print(F("[MESSAGE] Will perform "));
    Serial.print(NUM_CYCLES);
    Serial.print(F(" cycles of: home â†’ wait â†’ move to "));
    Serial.print(TEST_POSITION_MM);
    Serial.println(F("mm â†’ wait â†’ repeat"));
    Serial.println(F("[MESSAGE] Press any key to abort test"));

    lastActionTime = millis();

    while (testRunning)
    {
        // Check for abort command
        if (Serial.available() > 0)
        {
            Serial.println(F("[MESSAGE] Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            return false;
        }

        // Check for E-Stop condition
        if (isEStopActive())
        {
            Serial.println(F("[ERROR] E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            return false;
        }

        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();

        switch (currentPhase)
        {
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
            if (!initiateHomingSequence())
            {
                Serial.println(F("[ERROR] Error starting homing operation. Aborting test."));
                testRunning = false;
                return false;
            }
            currentPhase = PHASE_WAIT_FOR_HOMING_COMPLETE;
            break;

        case PHASE_WAIT_FOR_HOMING_COMPLETE:
            // Print status every 2 seconds for user feedback
            static unsigned long lastStatusPrint = 0;
            if (currentTime - lastStatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting for homing to complete. Current state: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", Homed: "));
                Serial.println(isHomed ? F("YES") : F("NO"));
                lastStatusPrint = currentTime;
            }

            // Actively call both functions needed for homing
            if (motorState == MOTOR_STATE_HOMING)
            {
                // Process the enable cycling first
                cycleMotorEnableForHoming();

                // Then ensure the homing state machine advances
                executeHomingSequence();
            }

            // Wait for homing to complete successfully
            if (motorState == MOTOR_STATE_IDLE && isHomed)
            {
                Serial.println(F("[MESSAGE] Homing complete. Waiting..."));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AFTER_HOMING; // Use explicit state
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Homing failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Use a timeout longer than the internal one in executeHomingSequence (which is 60 seconds)
            // But DO NOT proceed if homing hasn't completed successfully
            else if (currentTime - lastActionTime > 70000)
            { // 70 seconds (longer than the 60-second internal timeout)
                Serial.println(F("[ERROR] Timeout waiting for homing to complete."));
                Serial.print(F("[DIAGNOSTIC] Current state: "));

                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.println(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.println(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.println(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.println(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.println(F("NOT_READY"));
                    break;
                default:
                    Serial.println(F("UNKNOWN"));
                }

                // Safety critical: NEVER proceed without successful homing
                Serial.println(F("[ERROR] CRITICAL: Cannot proceed without successful homing. Aborting test."));
                stopMotion();        // Stop any motion to be safe
                testRunning = false; // Abort the test
                return false;
            }
            break;

        case PHASE_PAUSE_AFTER_HOMING: // Renamed state
            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                currentPhase = PHASE_MOVE_TO_POSITION;
                lastActionTime = currentTime;
            }
            break;

        case PHASE_MOVE_TO_POSITION:
            // Move to test position
            Serial.print(F("[MESSAGE] Moving to "));
            Serial.print(TEST_POSITION_MM);
            Serial.println(F("mm..."));
            if (!moveToPositionMm(TEST_POSITION_MM))
            {
                Serial.println(F("[ERROR] Error during movement. Aborting test."));
                testRunning = false;
                return false;
            }
            currentPhase = PHASE_WAIT_FOR_MOVE_COMPLETE;
            break;

        case PHASE_WAIT_FOR_MOVE_COMPLETE:
            // Add diagnostic printing to track move progress
            static unsigned long lastMoveStatusPrint = 0;
            if (currentTime - lastMoveStatusPrint > 2000)
            { // Print every 2 seconds
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(TEST_POSITION_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastMoveStatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                // This is a more reliable way to check for move completion
                Serial.print(F("[MESSAGE] Position reached: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm. Waiting..."));
                motorState = MOTOR_STATE_IDLE; // Force the state update if needed
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AFTER_MOVE;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add a timeout after a reasonable amount of time
            else if (currentTime - lastActionTime > 60000)
            { // Increased to 60 seconds
                Serial.println(F("[ERROR] Timeout waiting for move to complete."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AFTER_MOVE: // Renamed state
            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                currentPhase = PHASE_REPEAT_HOMING;
                lastActionTime = currentTime;
            }
            break;

        case PHASE_REPEAT_HOMING:
            // Home again
            Serial.println(F("[MESSAGE] Homing again..."));
            if (!initiateHomingSequence())
            {
                Serial.println(F("[ERROR] Error starting repeat homing operation. Aborting test."));
                testRunning = false;
                return false;
            }
            currentPhase = PHASE_WAIT_FOR_REPEAT_HOME;
            break;

        case PHASE_WAIT_FOR_REPEAT_HOME:
            // Print status periodically
            static unsigned long lastRepeatStatusPrint = 0;
            if (currentTime - lastRepeatStatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting for repeat homing to complete. State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", Homed: "));
                Serial.println(isHomed ? F("YES") : F("NO"));
                lastRepeatStatusPrint = currentTime;
            }

            // Actively call both functions needed for homing
            if (motorState == MOTOR_STATE_HOMING)
            {
                // Process the enable cycling first
                cycleMotorEnableForHoming();

                // Then ensure the homing state machine advances
                executeHomingSequence();
            }

            // Wait for homing to complete successfully
            if (motorState == MOTOR_STATE_IDLE && isHomed)
            {
                cyclesCompleted++;
                Serial.print(F("[MESSAGE] Cycle "));
                Serial.print(cyclesCompleted);
                Serial.print(F(" completed. Position after homing: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));

                if (cyclesCompleted >= NUM_CYCLES)
                {
                    currentPhase = PHASE_COMPLETE;
                }
                else
                {
                    // Instead of delay, set up for non-blocking pause
                    lastActionTime = currentTime;                 // Reset timer
                    currentPhase = PHASE_PAUSE_BEFORE_NEXT_CYCLE; // Move to pause state
                }
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Repeat homing failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Use a timeout longer than the internal one
            else if (currentTime - lastActionTime > 70000)
            {
                Serial.println(F("[ERROR] Timeout waiting for repeat homing to complete."));
                Serial.println(F("[ERROR] Cannot proceed without successful homing. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;

        case PHASE_PAUSE_BEFORE_NEXT_CYCLE:
            if (currentTime - lastActionTime >= 2000)
            {                               // 2 second non-blocking pause
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

// Test position cycling for tray loading
bool testPositionCycling()
{
    const int NUM_CYCLES = 10;               // Number of test cycles to run
    const unsigned long WAIT_TIME_MS = 5000; // Fixed 5-second wait time at each position
    int cyclesCompleted = 0;
    bool testRunning = true;
    unsigned long lastActionTime = 0;

    // Define test phases
    enum TestPhase
    {
        PHASE_START,
        PHASE_MOVE_TO_POSITION_3, // Position 1 -> Position 3
        PHASE_WAIT_FOR_MOVE_TO_3,
        PHASE_PAUSE_AT_POSITION_3, // Wait at Position 3
        PHASE_MOVE_TO_POSITION_1,  // Position 3 -> Position 1
        PHASE_WAIT_FOR_MOVE_TO_1,
        PHASE_PAUSE_AT_POSITION_1, // Wait at Position 1
        PHASE_MOVE_TO_POSITION_2,  // Position 1 -> Position 2
        PHASE_WAIT_FOR_MOVE_TO_2,
        PHASE_PAUSE_AT_POSITION_2,     // Wait at Position 2
        PHASE_MOVE_BACK_TO_POSITION_1, // Position 2 -> Position 1
        PHASE_WAIT_FOR_MOVE_BACK_TO_1,
        PHASE_PAUSE_BEFORE_NEXT_CYCLE, // Wait before starting next cycle
        PHASE_COMPLETE
    };

    TestPhase currentPhase = PHASE_START;

    // Check if motor is initialized and homed
    if (!motorInitialized)
    {
        Serial.println(F("[ERROR] Motor not initialized - run 'motor init' first"));
        return false;
    }

    if (!isHomed)
    {
        Serial.println(F("[ERROR] Motor not homed - run 'home' command first"));
        return false;
    }

    Serial.println(F("[MESSAGE] Starting position cycling test"));
    Serial.println(F("[MESSAGE] To abort, type any character"));
    Serial.print(F("[MESSAGE] Will perform "));
    Serial.print(NUM_CYCLES);
    Serial.println(F(" cycles of: Pos1â†’Pos3â†’Pos1â†’Pos2â†’Pos1"));
    Serial.print(F("[MESSAGE] Wait time at each position: "));
    Serial.print(WAIT_TIME_MS);
    Serial.println(F("ms"));

    lastActionTime = millis();

    while (testRunning)
    {
        // Check for abort command
        if (Serial.available() > 0)
        {
            Serial.println(F("[MESSAGE] Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            return false;
        }

        // Check for E-Stop condition
        if (isEStopActive())
        {
            Serial.println(F("[ERROR] E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            return false;
        }

        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();

        switch (currentPhase)
        {
        case PHASE_START:
            Serial.print(F("[MESSAGE] Starting cycle "));
            Serial.print(cyclesCompleted + 1);
            Serial.print(F(" of "));
            Serial.println(NUM_CYCLES);

            // IMPROVEMENT 1: Replace blocking code with non-blocking state pattern
            // First check we're at position 1
            if (abs(getMotorPositionMm() - POSITION_1_MM) > POSITION_TOLERANCE_MM)
            {
                Serial.println(F("[MESSAGE] Moving to Position 1 to begin test"));
                if (!moveToPosition(POSITION_1))
                {
                    Serial.println(F("[ERROR] Failed to move to Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
                // Set up for next phase without waiting
                currentPhase = PHASE_WAIT_FOR_MOVE_TO_1;
                lastActionTime = currentTime;
                // Continue execution instead of returning true
            }
            else
            {
                // Already at position 1, proceed to first movement
                currentPhase = PHASE_MOVE_TO_POSITION_3;
                lastActionTime = currentTime;
            }
            // Remove the return true statement
            break; // Exit the switch case, not the function

        case PHASE_MOVE_TO_POSITION_3:
            // Move from Position 1 to Position 3
            Serial.println(F("[MESSAGE] Moving: Position 1 â†’ Position 3"));
            if (!moveToPosition(POSITION_3))
            {
                Serial.println(F("[ERROR] Failed to move to Position 3. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Start timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_TO_3;
            break;

        case PHASE_WAIT_FOR_MOVE_TO_3:
            // IMPROVEMENT 2: Add periodic status updates during waiting
            static unsigned long lastPos3StatusPrint = 0;
            if (currentTime - lastPos3StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_3_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos3StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 3: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AT_POSITION_3;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 3 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 3."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AT_POSITION_3:
            // Wait at Position 3
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastWaitPos3Print = 0;
            if (currentTime - lastWaitPos3Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Pausing at Position 3: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Waiting: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos3Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                currentPhase = PHASE_MOVE_TO_POSITION_1;
                lastActionTime = currentTime;
            }
            break;

        case PHASE_MOVE_TO_POSITION_1:
            // Move from Position 3 to Position 1
            Serial.println(F("[MESSAGE] Moving: Position 3 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Serial.println(F("[ERROR] Failed to move to Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Reset timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_TO_1;
            break;

        case PHASE_WAIT_FOR_MOVE_TO_1:
            // IMPROVEMENT 2: Add periodic status updates during waiting
            static unsigned long lastPos1StatusPrint = 0;
            if (currentTime - lastPos1StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos1StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 1: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AT_POSITION_1;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 1 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 1."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AT_POSITION_1:
            // Wait at Position 1
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastWaitPos1Print = 0;
            if (currentTime - lastWaitPos1Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Pausing at Position 1: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Waiting: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos1Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                currentPhase = PHASE_MOVE_TO_POSITION_2;
                lastActionTime = currentTime;
            }
            break;

        case PHASE_MOVE_TO_POSITION_2:
            // Move from Position 1 to Position 2
            Serial.println(F("[MESSAGE] Moving: Position 1 â†’ Position 2"));
            if (!moveToPosition(POSITION_2))
            {
                Serial.println(F("[ERROR] Failed to move to Position 2. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Reset timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_TO_2;
            break;

        case PHASE_WAIT_FOR_MOVE_TO_2:
            // IMPROVEMENT 2: Add periodic status updates during waiting
            static unsigned long lastPos2StatusPrint = 0;
            if (currentTime - lastPos2StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_2_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos2StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 2: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AT_POSITION_2;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 2 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 2."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AT_POSITION_2:
            // Wait at Position 2
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastWaitPos2Print = 0;
            if (currentTime - lastWaitPos2Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Pausing at Position 2: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Waiting: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos2Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                currentPhase = PHASE_MOVE_BACK_TO_POSITION_1;
                lastActionTime = currentTime;
            }
            break;

        case PHASE_MOVE_BACK_TO_POSITION_1:
            // Move from Position 2 back to Position 1
            Serial.println(F("[MESSAGE] Moving: Position 2 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Serial.println(F("[ERROR] Failed to move back to Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Reset timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_BACK_TO_1;
            break;

        case PHASE_WAIT_FOR_MOVE_BACK_TO_1:
            // IMPROVEMENT 2: Add periodic status updates during waiting
            static unsigned long lastPosBack1StatusPrint = 0;
            if (currentTime - lastPosBack1StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm), State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPosBack1StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Back at Position 1: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));

                // Cycle complete
                cyclesCompleted++;
                lastActionTime = currentTime;

                if (cyclesCompleted >= NUM_CYCLES)
                {
                    currentPhase = PHASE_COMPLETE;
                }
                else
                {
                    currentPhase = PHASE_PAUSE_BEFORE_NEXT_CYCLE;
                }
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement back to Position 1 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 30000)
            { // 30-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement back to Position 1."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;

        case PHASE_PAUSE_BEFORE_NEXT_CYCLE:
            // Short pause before next cycle
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastPauseCycleStatusPrint = 0;
            if (currentTime - lastPauseCycleStatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Preparing for next cycle. Completed: "));
                Serial.print(cyclesCompleted);
                Serial.print(F("/"));
                Serial.println(NUM_CYCLES);
                lastPauseCycleStatusPrint = currentTime;
            }

            if (currentTime - lastActionTime >= 2000)
            { // 2 second pause
                currentPhase = PHASE_START;
            }
            break;

        case PHASE_COMPLETE:
            // IMPROVEMENT 4: Standardize status messages
            Serial.println(F("----------------------------------------"));
            Serial.println(F("[MESSAGE] Position cycling test completed successfully."));
            Serial.print(F("[MESSAGE] Completed "));
            Serial.print(cyclesCompleted);
            Serial.print(F(" cycles of position movement (Pos1â†’Pos3â†’Pos1â†’Pos2â†’Pos1)"));
            Serial.println(F("\n----------------------------------------"));
            testRunning = false;
            return true;
            break;
        }

        // Give time for processing other operations
        delayMicroseconds(100);
    }

    return false;
}

// Test tray handling operations including sensors, locking/unlocking
bool testTrayHandling()
{
    const int NUM_CYCLES = 10;                 // Number of test cycles to run
    const unsigned long WAIT_TIME_MS = 5000;   // Fixed 5-second wait time at each position
    const unsigned long VALVE_DELAY_MS = 1000; // Delay between valve operations to prevent race conditions
    const unsigned long ADDITIONAL_UNLOCK_DELAY_MS = 2000; // Additional safety delay after tray unlock before moving
    int cyclesCompleted = 0;
    bool testRunning = true;
    unsigned long lastActionTime = 0;

    // Helper function for tray presence detection
    auto isTrayPresentAtPosition = [](int position) -> bool
    {
        CylinderSensor *sensor = nullptr;
        switch (position)
        {
        case 1:
            sensor = getTray1DetectionSensor();
            break;
        case 2:
            sensor = getTray2DetectionSensor();
            break;
        case 3:
            sensor = getTray3DetectionSensor();
            break;
        default:
            return false;
        }
        if (!sensor)
            return false;
        return sensorRead(*sensor);
    };

    // Define test phases
    enum TestPhase
    {
        PHASE_START,
        PHASE_CHECK_POSITION_1,                   // Verify we're at position 1
        PHASE_WAIT_FOR_MOVE_TO_POS1,              // Wait for move to position 1 if needed
        PHASE_CHECK_TRAY_AT_POS1,                 // Verify tray at position 1
        PHASE_LOCK_TRAY_AT_POS1,                  // Lock tray at position 1
        PHASE_DELAY_AFTER_LOCK_TRAY_POS1,         // Delay after valve operation
        PHASE_LOCK_SHUTTLE_AT_POS1,               // Lock shuttle
        PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1,      // Delay after valve operation
        PHASE_UNLOCK_TRAY_AT_POS1,                // Unlock tray at position 1
        PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1,       // Delay after valve operation
        PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS1, // Additional delay before verifying tray position
        PHASE_VERIFY_TRAY_STILL_AT_POS1,          // Check tray still present before moving
        PHASE_CHECK_TRAY_AT_POS3,                 // Ensure position 3 is clear before moving
        PHASE_MOVE_TO_POSITION_3,                 // Move to position 3
        PHASE_WAIT_FOR_MOVE_TO_POS3,              // Wait for move to complete
        PHASE_VERIFY_TRAY_AT_POS3,                // Check tray made it to position 3
        PHASE_UNLOCK_SHUTTLE_AT_POS3,             // Unlock shuttle at position 3
        PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS3,    // Delay after valve operation
        PHASE_LOCK_TRAY_AT_POS3,                  // Lock tray at position 3
        PHASE_DELAY_AFTER_LOCK_TRAY_POS3,         // Delay after valve operation
        PHASE_WAIT_AT_POS3,                       // Wait at position 3

        // Empty shuttle return from position 3 to position 1
        PHASE_RETURN_TO_POS1_FROM_POS3_EMPTY, // Return empty to position 1
        PHASE_WAIT_FOR_RETURN_TO_POS1_EMPTY,  // Wait for empty return move to complete
        PHASE_WAIT_AT_POS1_EMPTY,             // Wait at position 1 (empty shuttle)
        PHASE_RETURN_TO_POS3,                 // Return to position 3 to pick up tray
        PHASE_WAIT_FOR_RETURN_TO_POS3,        // Wait for move to position 3

        PHASE_LOCK_SHUTTLE_AT_POS3,                     // Lock shuttle at position 3
        PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS3,            // Delay after valve operation
        PHASE_UNLOCK_TRAY_AT_POS3,                      // Unlock tray at position 3
        PHASE_DELAY_AFTER_UNLOCK_TRAY_POS3,             // Delay after valve operation
        PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS3,       // Additional delay before moving from position 3
        PHASE_CHECK_TRAY_AT_POS1_AGAIN,                 // Check position 1 is clear before returning
        PHASE_MOVE_TO_POSITION_1_FROM_3,                // Move back to position 1
        PHASE_WAIT_FOR_MOVE_TO_POS1_FROM_3,             // Wait for move to complete
        PHASE_VERIFY_TRAY_AT_POS1_FROM_3,               // Check tray made it back to position 1
        PHASE_UNLOCK_SHUTTLE_AT_POS1_FROM_3,            // Unlock shuttle at position 1
        PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS1_FROM_3,   // Delay after valve operation
        PHASE_LOCK_TRAY_AT_POS1_FROM_3,                 // Lock tray at position 1
        PHASE_DELAY_AFTER_LOCK_TRAY_POS1_FROM_3,        // Delay after valve operation
        PHASE_WAIT_AT_POS1,                             // Wait at position 1
        PHASE_LOCK_SHUTTLE_AT_POS1_FROM_3,              // Lock shuttle at position 1
        PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1_FROM_3,     // Delay after valve operation
        PHASE_UNLOCK_TRAY_AT_POS1_AGAIN,                // Unlock tray at position 1 again
        PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1_AGAIN,       // Delay after valve operation
        PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS1_AGAIN, // Additional delay before verifying tray position
        PHASE_VERIFY_TRAY_STILL_AT_POS1_AGAIN,          // Check tray still present for position 2 move
        PHASE_CHECK_TRAY_AT_POS2,                       // Ensure position 2 is clear before moving
        PHASE_MOVE_TO_POSITION_2,                       // Move to position 2
        PHASE_WAIT_FOR_MOVE_TO_POS2,                    // Wait for move to complete
        PHASE_VERIFY_TRAY_AT_POS2,                      // Check tray made it to position 2
        PHASE_UNLOCK_SHUTTLE_AT_POS2,                   // Unlock shuttle at position 2
        PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS2,          // Delay after valve operation
        PHASE_LOCK_TRAY_AT_POS2,                        // Lock tray at position 2
        PHASE_DELAY_AFTER_LOCK_TRAY_POS2,               // Delay after valve operation
        PHASE_WAIT_AT_POS2,                             // Wait at position 2

        // Empty shuttle return from position 2 to position 1
        PHASE_RETURN_TO_POS1_FROM_POS2_EMPTY,          // Return empty to position 1
        PHASE_WAIT_FOR_RETURN_TO_POS1_FROM_POS2_EMPTY, // Wait for empty return move
        PHASE_WAIT_AT_POS1_EMPTY_FROM_POS2,            // Wait at position 1 (empty shuttle)
        PHASE_RETURN_TO_POS2,                          // Return to position 2 to pick up tray
        PHASE_WAIT_FOR_RETURN_TO_POS2,                 // Wait for move to position 2

        PHASE_LOCK_SHUTTLE_AT_POS2,                    // Lock shuttle at position 2
        PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS2,           // Delay after valve operation
        PHASE_UNLOCK_TRAY_AT_POS2,                     // Unlock tray at position 2
        PHASE_DELAY_AFTER_UNLOCK_TRAY_POS2,            // Delay after valve operation
        PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS2,      // Additional delay before moving from position 2
        PHASE_CHECK_TRAY_AT_POS1_BEFORE_RETURN,        // Check position 1 is clear before final return
        PHASE_MOVE_BACK_TO_POSITION_1,                 // Move back to position 1 to complete cycle
        PHASE_WAIT_FOR_MOVE_BACK_TO_POS1,              // Wait for move to complete
        PHASE_UNLOCK_SHUTTLE_END_OF_CYCLE,             // Unlock shuttle at the end of cycle
        PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_END_OF_CYCLE, // Delay after valve operation
        PHASE_PAUSE_BEFORE_NEXT_CYCLE,                 // Wait before starting next cycle
        PHASE_COMPLETE                                 // Test completion
    };

    TestPhase currentPhase = PHASE_START;

    // Check if motor is initialized and homed
    if (!motorInitialized)
    {
        Serial.println(F("[ERROR] Motor not initialized - run 'motor init' first"));
        return false;
    }

    if (!isHomed)
    {
        Serial.println(F("[ERROR] Motor not homed - run 'home' command first"));
        return false;
    }

    Serial.println(F("[MESSAGE] Starting enhanced tray handling test"));
    Serial.println(F("[MESSAGE] This test includes empty shuttle returns and valve delays"));
    Serial.println(F("[MESSAGE] To abort, type any character"));
    Serial.print(F("[MESSAGE] Will perform "));
    Serial.print(NUM_CYCLES);
    Serial.println(F(" cycles of tray handling operations"));
    Serial.print(F("[MESSAGE] Wait time at each position: "));
    Serial.print(WAIT_TIME_MS);
    Serial.println(F("ms"));
    Serial.print(F("[MESSAGE] Delay between valve operations: "));
    Serial.print(VALVE_DELAY_MS);
    Serial.println(F("ms"));
    Serial.print(F("[MESSAGE] Additional safety delay after tray unlock: "));
    Serial.print(ADDITIONAL_UNLOCK_DELAY_MS);
    Serial.println(F("ms"));

    lastActionTime = millis();

    while (testRunning)
    {
        // Check for abort command
        if (Serial.available() > 0)
        {
            Serial.println(F("[MESSAGE] Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            return false;
        }

        // Check for E-Stop condition
        if (isEStopActive())
        {
            Serial.println(F("[ERROR] E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            return false;
        }

        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();

        switch (currentPhase)
        {
        case PHASE_START:
        {
            Serial.print(F("[MESSAGE] Starting tray handling cycle "));
            Serial.print(cyclesCompleted + 1);
            Serial.print(F(" of "));
            Serial.println(NUM_CYCLES);
            currentPhase = PHASE_CHECK_POSITION_1;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_CHECK_POSITION_1:
        {
            // First check we're at position 1
            if (abs(getMotorPositionMm() - POSITION_1_MM) > POSITION_TOLERANCE_MM)
            {
                Serial.println(F("[MESSAGE] Moving to Position 1 to begin test"));
                if (!moveToPosition(POSITION_1))
                {
                    Serial.println(F("[ERROR] Failed to move to Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
                // Wait for move to complete before proceeding
                currentPhase = PHASE_WAIT_FOR_MOVE_TO_POS1;
                lastActionTime = currentTime;
            }
            else
            {
                // Already at position 1, proceed to check if tray is present
                currentPhase = PHASE_CHECK_TRAY_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_WAIT_FOR_MOVE_TO_POS1:
        {
            // Monitor movement to position 1
            static unsigned long lastInitMoveStatusPrint = 0;
            if (currentTime - lastInitMoveStatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm, State: "));
                // Print motor state
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastInitMoveStatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 1: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                currentPhase = PHASE_CHECK_TRAY_AT_POS1;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 1 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 1."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS1:
        {
            // Check if tray is present at position 1
            Serial.println(F("[MESSAGE] Checking for tray at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Serial.println(F("[ERROR] No tray detected at Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Tray detected at Position 1. Continuing test."));
            currentPhase = PHASE_LOCK_TRAY_AT_POS1;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS1:
        {
            // Lock tray at position 1
            Serial.println(F("[MESSAGE] Locking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS1;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock tray at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS1:
        {
            // Add delay between valve operations to prevent race conditions
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to lock shuttle."));
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS1:
        {
            // Lock shuttle (position 1)
            Serial.println(F("[MESSAGE] Locking shuttle at Position 1..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock shuttle at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1:
        {
            // Add delay between valve operations to prevent race conditions
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS1:
        {
            // Unlock tray at position 1
            Serial.println(F("[MESSAGE] Unlocking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray unlocked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock tray at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Adding additional safety delay before movement..."));
                currentPhase = PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS1:
        {
            // Additional delay before checking if we can move
            if (currentTime - lastActionTime >= ADDITIONAL_UNLOCK_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Verifying tray is still at Position 1..."));
                currentPhase = PHASE_VERIFY_TRAY_STILL_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_STILL_AT_POS1:
        {
            // Check if tray is still present before moving
            Serial.println(F("[MESSAGE] Verifying tray is still at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Serial.println(F("[ERROR] Tray not detected at Position 1 after unlock. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Tray confirmed at Position 1. Checking Position 3 before moving."));
            currentPhase = PHASE_CHECK_TRAY_AT_POS3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS3:
        {
            // Check if there's already a tray at position 3 (collision risk)
            Serial.println(F("[MESSAGE] Checking if Position 3 is clear..."));
            if (isTrayPresentAtPosition(3))
            {
                Serial.println(F("[ERROR] Tray detected at Position 3. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Position 3 is clear. Proceeding with move."));
            currentPhase = PHASE_MOVE_TO_POSITION_3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_TO_POSITION_3:
        {
            // Move from Position 1 to Position 3
            Serial.println(F("[MESSAGE] Moving: Position 1 â†’ Position 3"));
            if (!moveToPosition(POSITION_3))
            {
                Serial.println(F("[ERROR] Failed to move to Position 3. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Start timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_TO_POS3;
            break;
        }

        case PHASE_WAIT_FOR_MOVE_TO_POS3:
        {
            // Monitor movement to position 3
            static unsigned long lastPos3StatusPrint = 0;
            if (currentTime - lastPos3StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_3_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos3StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 3: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                currentPhase = PHASE_VERIFY_TRAY_AT_POS3;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 3 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 3."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_AT_POS3:
        {
            // Check if tray moved with shuttle to position 3
            Serial.println(F("[MESSAGE] Checking for tray at Position 3..."));
            if (!isTrayPresentAtPosition(3))
            {
                Serial.println(F("[ERROR] No tray detected at Position 3. Tray was lost during movement. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Tray successfully moved to Position 3. Proceeding to unlock shuttle."));
            currentPhase = PHASE_UNLOCK_SHUTTLE_AT_POS3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_AT_POS3:
        {
            // Unlock shuttle at position 3
            Serial.println(F("[MESSAGE] Unlocking shuttle at Position 3..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle unlocked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock shuttle at Position 3. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS3:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS3:
        {
            // Lock tray at position 3
            Serial.println(F("[MESSAGE] Locking tray at Position 3..."));
            DoubleSolenoidValve *valve3 = getTray3Valve();
            CylinderSensor *sensor3 = getTray3Sensor();

            if (valve3 && sensor3)
            {
                if (safeValveOperation(*valve3, *sensor3, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray locked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock tray at Position 3. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 3. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS3:
        {
            // Add delay between operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Tray locked at Position 3. Waiting for 5 seconds..."));
                currentPhase = PHASE_WAIT_AT_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_WAIT_AT_POS3:
        {
            // Wait at Position 3
            static unsigned long lastWaitPos3Print = 0;
            if (currentTime - lastWaitPos3Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting at Position 3 with tray locked. Elapsed: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos3Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Serial.println(F("[MESSAGE] Moving empty shuttle back to Position 1..."));
                currentPhase = PHASE_RETURN_TO_POS1_FROM_POS3_EMPTY;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS1_FROM_POS3_EMPTY:
        {
            // Return empty shuttle to position 1
            Serial.println(F("[MESSAGE] Moving empty shuttle: Position 3 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Serial.println(F("[ERROR] Failed to move empty shuttle to Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime;
            currentPhase = PHASE_WAIT_FOR_RETURN_TO_POS1_EMPTY;
            break;
        }

        case PHASE_WAIT_FOR_RETURN_TO_POS1_EMPTY:
        {
            // Wait for empty return move to complete
            static unsigned long lastEmptyReturnStatusPrint = 0;
            if (currentTime - lastEmptyReturnStatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Empty return status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastEmptyReturnStatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Empty shuttle reached Position 1: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                currentPhase = PHASE_WAIT_AT_POS1_EMPTY;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Empty shuttle movement to Position 1 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Serial.println(F("[ERROR] Timeout waiting for empty shuttle return to Position 1."));
                stopMotion();
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_WAIT_AT_POS1_EMPTY:
        {
            // Wait at position 1 with empty shuttle
            static unsigned long lastWaitPos1EmptyPrint = 0;
            if (currentTime - lastWaitPos1EmptyPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting at Position 1 with empty shuttle. Elapsed: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos1EmptyPrint = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Serial.println(F("[MESSAGE] Returning to Position 3 to pick up tray..."));
                currentPhase = PHASE_RETURN_TO_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS3:
        {
            // Return to position 3 to pick up tray
            Serial.println(F("[MESSAGE] Moving empty shuttle back: Position 1 â†’ Position 3"));
            if (!moveToPosition(POSITION_3))
            {
                Serial.println(F("[ERROR] Failed to return empty shuttle to Position 3. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime;
            currentPhase = PHASE_WAIT_FOR_RETURN_TO_POS3;
            break;
        }

        case PHASE_WAIT_FOR_RETURN_TO_POS3:
        {
            // Wait for move back to position 3
            static unsigned long lastReturnToPos3StatusPrint = 0;
            if (currentTime - lastReturnToPos3StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Return status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_3_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastReturnToPos3StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Returned to Position 3: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));

                // Verify tray is still at position 3
                if (!isTrayPresentAtPosition(3))
                {
                    Serial.println(F("[ERROR] No tray detected at Position 3 after return. Aborting test."));
                    testRunning = false;
                    return false;
                }
                Serial.println(F("[MESSAGE] Tray confirmed still at Position 3. Proceeding to pick it up."));
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS3;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Return to Position 3 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Serial.println(F("[ERROR] Timeout waiting for return to Position 3."));
                stopMotion();
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS3:
        {
            // Lock shuttle at position 3
            Serial.println(F("[MESSAGE] Locking shuttle at Position 3..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle locked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock shuttle at Position 3. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS3:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS3:
        {
            // Unlock tray at position 3
            Serial.println(F("[MESSAGE] Unlocking tray at Position 3..."));
            DoubleSolenoidValve *valve3 = getTray3Valve();
            CylinderSensor *sensor3 = getTray3Sensor();

            if (valve3 && sensor3)
            {
                if (safeValveOperation(*valve3, *sensor3, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray unlocked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock tray at Position 3. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 3. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS3:
        {
            // Add delay between operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Adding additional safety delay before movement..."));
                currentPhase = PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS3:
        {
            // Additional delay before checking if we can move
            if (currentTime - lastActionTime >= ADDITIONAL_UNLOCK_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Checking Position 1 before moving."));
                currentPhase = PHASE_CHECK_TRAY_AT_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS1_AGAIN:
        {
            // Check if there's already a tray at position 1 (collision risk)
            Serial.println(F("[MESSAGE] Checking if Position 1 is clear for return..."));
            if (isTrayPresentAtPosition(1))
            {
                Serial.println(F("[ERROR] Tray detected at Position 1. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Position 1 is clear. Proceeding with move."));
            currentPhase = PHASE_MOVE_TO_POSITION_1_FROM_3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_TO_POSITION_1_FROM_3:
        {
            // Move from Position 3 to Position 1
            Serial.println(F("[MESSAGE] Moving: Position 3 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Serial.println(F("[ERROR] Failed to move to Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Reset timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_TO_POS1_FROM_3;
            break;
        }

        case PHASE_WAIT_FOR_MOVE_TO_POS1_FROM_3:
        {
            // Monitor movement back to position 1
            static unsigned long lastPos1From3StatusPrint = 0;
            if (currentTime - lastPos1From3StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos1From3StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 1 from Position 3: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                currentPhase = PHASE_VERIFY_TRAY_AT_POS1_FROM_3;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 1 from Position 3 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 1 from Position 3."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_AT_POS1_FROM_3:
        {
            // Check if tray is at position 1 after return from position 3
            Serial.println(F("[MESSAGE] Checking for tray at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Serial.println(F("[ERROR] No tray detected at Position 1 after return from Position 3. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Tray confirmed at Position 1. Proceeding to unlock shuttle."));
            currentPhase = PHASE_UNLOCK_SHUTTLE_AT_POS1_FROM_3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_AT_POS1_FROM_3:
        {
            // Unlock shuttle at position 1 after return from position 3
            Serial.println(F("[MESSAGE] Unlocking shuttle at Position 1..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle unlocked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS1_FROM_3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock shuttle at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS1_FROM_3:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS1_FROM_3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS1_FROM_3:
        {
            // Lock tray at position 1 after return from position 3
            Serial.println(F("[MESSAGE] Locking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS1_FROM_3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock tray at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS1_FROM_3:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Tray locked at Position 1. Waiting for 5 seconds..."));
                currentPhase = PHASE_WAIT_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_WAIT_AT_POS1:
        {
            // Wait at Position 1
            static unsigned long lastWaitPos1Print = 0;
            if (currentTime - lastWaitPos1Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting at Position 1 with tray locked. Elapsed: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos1Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS1_FROM_3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS1_FROM_3:
        {
            // Lock shuttle at position 1 before moving to position 2
            Serial.println(F("[MESSAGE] Locking shuttle at Position 1..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1_FROM_3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock shuttle at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1_FROM_3:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS1_AGAIN:
        {
            // Unlock tray at position 1 before moving to position 2
            Serial.println(F("[MESSAGE] Unlocking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray unlocked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1_AGAIN;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock tray at Position 1. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        // Implementation for Position 1 (again)
        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1_AGAIN:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Adding additional safety delay before movement..."));
                currentPhase = PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS1_AGAIN:
        {
            // Additional delay before checking if we can move
            if (currentTime - lastActionTime >= ADDITIONAL_UNLOCK_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Verifying tray is still at Position 1..."));
                currentPhase = PHASE_VERIFY_TRAY_STILL_AT_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_STILL_AT_POS1_AGAIN:
        {
            // Check if tray is still present before moving to position 2
            Serial.println(F("[MESSAGE] Verifying tray is still at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Serial.println(F("[ERROR] Tray not detected at Position 1 after unlock. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Tray confirmed at Position 1. Checking Position 2 before moving."));
            currentPhase = PHASE_CHECK_TRAY_AT_POS2;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS2:
        {
            // Check if there's already a tray at position 2 (collision risk)
            Serial.println(F("[MESSAGE] Checking if Position 2 is clear..."));
            if (isTrayPresentAtPosition(2))
            {
                Serial.println(F("[ERROR] Tray detected at Position 2. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Position 2 is clear. Proceeding with move."));
            currentPhase = PHASE_MOVE_TO_POSITION_2;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_TO_POSITION_2:
        {
            // Move from Position 1 to Position 2
            Serial.println(F("[MESSAGE] Moving: Position 1 â†’ Position 2"));
            if (!moveToPosition(POSITION_2))
            {
                Serial.println(F("[ERROR] Failed to move to Position 2. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Reset timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_TO_POS2;
            break;
        }

        case PHASE_WAIT_FOR_MOVE_TO_POS2:
        {
            // Monitor movement to position 2
            static unsigned long lastPos2StatusPrint = 0;
            if (currentTime - lastPos2StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_2_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos2StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Reached Position 2: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                currentPhase = PHASE_VERIFY_TRAY_AT_POS2;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement to Position 2 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement to Position 2."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_AT_POS2:
        {
            // Check if tray is at position 2
            Serial.println(F("[MESSAGE] Checking for tray at Position 2..."));
            if (!isTrayPresentAtPosition(2))
            {
                Serial.println(F("[ERROR] No tray detected at Position 2. Tray was lost during movement. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Tray successfully moved to Position 2. Proceeding to unlock shuttle."));
            currentPhase = PHASE_UNLOCK_SHUTTLE_AT_POS2;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_AT_POS2:
        {
            // Unlock shuttle at position 2
            Serial.println(F("[MESSAGE] Unlocking shuttle at Position 2..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle unlocked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock shuttle at Position 2. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS2:
        {
            // Lock tray at position 2
            Serial.println(F("[MESSAGE] Locking tray at Position 2..."));
            DoubleSolenoidValve *valve2 = getTray2Valve();
            CylinderSensor *sensor2 = getTray2Sensor();

            if (valve2 && sensor2)
            {
                if (safeValveOperation(*valve2, *sensor2, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray locked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock tray at Position 2. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 2. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Tray locked at Position 2. Waiting for 5 seconds..."));
                currentPhase = PHASE_WAIT_AT_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_WAIT_AT_POS2:
        {
            // Wait at Position 2
            static unsigned long lastWaitPos2Print = 0;
            if (currentTime - lastWaitPos2Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting at Position 2 with tray locked. Elapsed: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos2Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Serial.println(F("[MESSAGE] Moving empty shuttle back to Position 1..."));
                currentPhase = PHASE_RETURN_TO_POS1_FROM_POS2_EMPTY;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS1_FROM_POS2_EMPTY:
        {
            // Return empty shuttle to position 1 from position 2
            Serial.println(F("[MESSAGE] Moving empty shuttle: Position 2 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Serial.println(F("[ERROR] Failed to move empty shuttle to Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime;
            currentPhase = PHASE_WAIT_FOR_RETURN_TO_POS1_FROM_POS2_EMPTY;
            break;
        }

        case PHASE_WAIT_FOR_RETURN_TO_POS1_FROM_POS2_EMPTY:
        {
            // Wait for empty return move to complete from position 2
            static unsigned long lastEmptyReturnFromPos2StatusPrint = 0;
            if (currentTime - lastEmptyReturnFromPos2StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Empty return status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastEmptyReturnFromPos2StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Empty shuttle reached Position 1 from Position 2: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));
                currentPhase = PHASE_WAIT_AT_POS1_EMPTY_FROM_POS2;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Empty shuttle movement to Position 1 from Position 2 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Serial.println(F("[ERROR] Timeout waiting for empty shuttle return to Position 1."));
                stopMotion();
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_WAIT_AT_POS1_EMPTY_FROM_POS2:
        {
            // Wait at position 1 with empty shuttle from position 2
            static unsigned long lastWaitPos1EmptyFromPos2Print = 0;
            if (currentTime - lastWaitPos1EmptyFromPos2Print > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Waiting at Position 1 with empty shuttle from Position 2. Elapsed: "));
                Serial.print((currentTime - lastActionTime) / 1000);
                Serial.print(F("/"));
                Serial.print(WAIT_TIME_MS / 1000);
                Serial.println(F(" seconds"));
                lastWaitPos1EmptyFromPos2Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Serial.println(F("[MESSAGE] Returning to Position 2 to pick up tray..."));
                currentPhase = PHASE_RETURN_TO_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS2:
        {
            // Return to position 2 to pick up tray
            Serial.println(F("[MESSAGE] Moving empty shuttle back: Position 1 â†’ Position 2"));
            if (!moveToPosition(POSITION_2))
            {
                Serial.println(F("[ERROR] Failed to return empty shuttle to Position 2. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime;
            currentPhase = PHASE_WAIT_FOR_RETURN_TO_POS2;
            break;
        }

        case PHASE_WAIT_FOR_RETURN_TO_POS2:
        {
            // Wait for move back to position 2
            static unsigned long lastReturnToPos2StatusPrint = 0;
            if (currentTime - lastReturnToPos2StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Return status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_2_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastReturnToPos2StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Returned to Position 2: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));

                // Verify tray is still at position 2
                if (!isTrayPresentAtPosition(2))
                {
                    Serial.println(F("[ERROR] No tray detected at Position 2 after return. Aborting test."));
                    testRunning = false;
                    return false;
                }
                Serial.println(F("[MESSAGE] Tray confirmed still at Position 2. Proceeding to pick it up."));
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS2;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Return to Position 2 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Serial.println(F("[ERROR] Timeout waiting for return to Position 2."));
                stopMotion();
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS2:
        {
            // Lock shuttle at position 2
            Serial.println(F("[MESSAGE] Locking shuttle at Position 2..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle locked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to lock shuttle at Position 2. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS2:
        {
            // Unlock tray at position 2
            Serial.println(F("[MESSAGE] Unlocking tray at Position 2..."));
            DoubleSolenoidValve *valve2 = getTray2Valve();
            CylinderSensor *sensor2 = getTray2Sensor();

            if (valve2 && sensor2)
            {
                if (safeValveOperation(*valve2, *sensor2, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Tray unlocked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock tray at Position 2. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access valve or sensor for Position 2. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Adding additional safety delay before movement..."));
                currentPhase = PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_ADDITIONAL_DELAY_AFTER_UNLOCK_POS2:
        {
            // Additional delay before checking if we can move
            if (currentTime - lastActionTime >= ADDITIONAL_UNLOCK_DELAY_MS)
            {
                Serial.println(F("[MESSAGE] Checking Position 1 before final return."));
                currentPhase = PHASE_CHECK_TRAY_AT_POS1_BEFORE_RETURN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS1_BEFORE_RETURN:
        {
            // Check if there's already a tray at position 1 (collision risk)
            Serial.println(F("[MESSAGE] Checking if Position 1 is clear for final return..."));
            if (isTrayPresentAtPosition(1))
            {
                Serial.println(F("[ERROR] Tray detected at Position 1. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                return false;
            }
            Serial.println(F("[MESSAGE] Position 1 is clear. Proceeding with final move back to Position 1."));
            currentPhase = PHASE_MOVE_BACK_TO_POSITION_1;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_BACK_TO_POSITION_1:
        {
            // Move from Position 2 back to Position 1
            Serial.println(F("[MESSAGE] Moving: Position 2 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Serial.println(F("[ERROR] Failed to move back to Position 1. Aborting test."));
                testRunning = false;
                return false;
            }
            lastActionTime = currentTime; // Reset timeout timer
            currentPhase = PHASE_WAIT_FOR_MOVE_BACK_TO_POS1;
            break;
        }

        case PHASE_WAIT_FOR_MOVE_BACK_TO_POS1:
        {
            // Monitor movement back to position 1
            static unsigned long lastMoveBackPos1StatusPrint = 0;
            if (currentTime - lastMoveBackPos1StatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Move status - Position: "));
                Serial.print(getMotorPositionMm());
                Serial.print(F("mm, Target: "));
                Serial.print(POSITION_1_MM);
                Serial.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Serial.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Serial.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Serial.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Serial.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Serial.print(F("NOT_READY"));
                    break;
                default:
                    Serial.print(F("UNKNOWN"));
                }
                Serial.print(F(", StepsComplete: "));
                Serial.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastMoveBackPos1StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Serial.print(F("[MESSAGE] Back at Position 1: "));
                Serial.print(getMotorPositionMm());
                Serial.println(F("mm"));

                // Verify tray made it back to position 1
                if (!isTrayPresentAtPosition(1))
                {
                    Serial.println(F("[ERROR] No tray detected at Position 1 after return from Position 2. Aborting test."));
                    testRunning = false;
                    return false;
                }

                // Cycle complete
                cyclesCompleted++;
                lastActionTime = currentTime;

                if (cyclesCompleted >= NUM_CYCLES)
                {
                    currentPhase = PHASE_COMPLETE;
                }
                else
                {
                    // Change this to go to a new unlock shuttle phase instead
                    currentPhase = PHASE_UNLOCK_SHUTTLE_END_OF_CYCLE; // New phase to unlock shuttle
                    lastActionTime = currentTime;
                }
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Serial.println(F("[ERROR] Movement back to Position 1 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Serial.println(F("[ERROR] Timeout waiting for movement back to Position 1."));
                Serial.println(F("[ERROR] Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_END_OF_CYCLE:
        {
            // Unlock shuttle at the end of cycle
            Serial.println(F("[MESSAGE] Unlocking shuttle at end of cycle..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Serial.println(F("[MESSAGE] Shuttle unlocked at end of cycle."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_END_OF_CYCLE;
                    lastActionTime = currentTime;
                }
                else
                {
                    Serial.println(F("[ERROR] Failed to unlock shuttle at end of cycle. Aborting test."));
                    testRunning = false;
                    return false;
                }
            }
            else
            {
                Serial.println(F("[ERROR] Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_END_OF_CYCLE:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                currentPhase = PHASE_PAUSE_BEFORE_NEXT_CYCLE;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_PAUSE_BEFORE_NEXT_CYCLE:
        {
            // Short pause before next cycle
            static unsigned long lastPauseCycleStatusPrint = 0;
            if (currentTime - lastPauseCycleStatusPrint > 2000)
            {
                Serial.print(F("[DIAGNOSTIC] Preparing for next cycle. Completed: "));
                Serial.print(cyclesCompleted);
                Serial.print(F("/"));
                Serial.println(NUM_CYCLES);
                lastPauseCycleStatusPrint = currentTime;
            }

            if (currentTime - lastActionTime >= 2000)
            { // 2 second pause
                currentPhase = PHASE_START;
            }
            break;
        }

        case PHASE_COMPLETE:
        {
            Serial.println(F("----------------------------------------"));
            Serial.println(F("[MESSAGE] Enhanced tray handling test completed successfully."));
            Serial.print(F("[MESSAGE] Completed "));
            Serial.print(cyclesCompleted);
            Serial.println(F(" cycles of tray handling operations."));
            Serial.println(F("----------------------------------------"));
            testRunning = false;
            return true;
            break;
        }
        }

        // Give time for processing other operations
        delayMicroseconds(100);
    }

    return false;
}