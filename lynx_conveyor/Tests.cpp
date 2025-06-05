#include "Tests.h"

// Test homing repeatability by performing multiple home-move cycles
bool testHomingRepeatability()
{
    // Set test flag
    testInProgress = true;

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
        Console.error(F("Motor has active alerts - clear faults before testing"));
        printMotorAlerts();
        testInProgress = false;
        return false;
    }

    // Check if motor is initialized
    if (!motorInitialized)
    {
        Console.error(F("Motor not initialized - run 'motor init' first"));
        testInProgress = false;
        return false;
    }

    Console.info(F("Starting homing repeatability test"));
    Console.info(F("To abort, type any character"));
    Console.print(F("[INFO] Will perform "));
    Console.print(NUM_CYCLES);
    Console.print(F(" cycles of: home -> wait -> move to "));
    Console.print(TEST_POSITION_MM);
    Console.println(F("mm -> wait -> repeat"));
    Console.info(F("Press any key to abort test"));

    lastActionTime = millis();

    while (testRunning)
    {
        // Check for abort command
        if (testAbortRequested)
        {
            Console.info(F("Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            testInProgress = false;
            testAbortRequested = false; // Reset the flag
            return false;
        }

        // Check for E-Stop condition
        if (isEStopActive())
        {
            Console.error(F("E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            testInProgress = false;
            return false;
        }

        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();

        switch (currentPhase)
        {
        case PHASE_START:
            Console.print(F("[INFO] Starting cycle "));
            Console.print(cyclesCompleted + 1);
            Console.print(F(" of "));
            Console.println(NUM_CYCLES);
            currentPhase = PHASE_INITIAL_HOMING;
            lastActionTime = currentTime;
            break;

        case PHASE_INITIAL_HOMING:
            // Start homing
            Console.info(F("Homing..."));
            if (!initiateHomingSequence())
            {
                Console.error(F("Error starting homing operation. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            currentPhase = PHASE_WAIT_FOR_HOMING_COMPLETE;
            break;

        case PHASE_WAIT_FOR_HOMING_COMPLETE:
            // Print status every 2 seconds for user feedback
            static unsigned long lastStatusPrint = 0;
            if (currentTime - lastStatusPrint > 2000)
            {
                Console.print(F("[DIAGNOSTIC] Waiting for homing to complete. Current state: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", Homed: "));
                Console.println(isHomed ? F("YES") : F("NO"));
                lastStatusPrint = currentTime;
            }

            // Replace both blocks with just:
            if (motorState == MOTOR_STATE_HOMING)
            {
                // Ensure the homing state machine advances
                checkHomingProgress();
            }

            // Wait for homing to complete successfully
            if (motorState == MOTOR_STATE_IDLE && isHomed)
            {
                Console.info(F("Homing complete. Waiting..."));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AFTER_HOMING; // Use explicit state
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Homing failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Use a timeout longer than the internal one in executeHomingSequence (which is 60 seconds)
            // But DO NOT proceed if homing hasn't completed successfully
            else if (currentTime - lastActionTime > 70000)
            { // 70 seconds (longer than the 60-second internal timeout)
                Console.error(F("Timeout waiting for homing to complete."));
                Console.print(F("[DIAGNOSTIC] Current state: "));

                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.println(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.println(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.println(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.println(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.println(F("NOT_READY"));
                    break;
                default:
                    Console.println(F("UNKNOWN"));
                }

                // Safety critical: NEVER proceed without successful homing
                Console.error(F("CRITICAL: Cannot proceed without successful homing. Aborting test."));
                stopMotion();        // Stop any motion to be safe
                testRunning = false; // Abort the test
                testInProgress = false;
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
            Console.print(F("[INFO] Moving to "));
            Console.print(TEST_POSITION_MM);
            Console.println(F("mm..."));
            if (!moveToPositionMm(TEST_POSITION_MM))
            {
                Console.error(F("Error during movement. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            currentPhase = PHASE_WAIT_FOR_MOVE_COMPLETE;
            break;

        case PHASE_WAIT_FOR_MOVE_COMPLETE:
            // Add diagnostic printing to track move progress
            static unsigned long lastMoveStatusPrint = 0;
            if (currentTime - lastMoveStatusPrint > 2000)
            { // Print every 2 seconds
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(TEST_POSITION_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastMoveStatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                // This is a more reliable way to check for move completion
                Console.print(F("[INFO] Position reached: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm. Waiting..."));
                motorState = MOTOR_STATE_IDLE; // Force the state update if needed
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AFTER_MOVE;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add a timeout after a reasonable amount of time
            else if (currentTime - lastActionTime > 60000)
            { // Increased to 60 seconds
                Console.error(F("Timeout waiting for move to complete."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
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
            Console.info(F("Homing again..."));
            if (!initiateHomingSequence())
            {
                Console.error(F("Error starting repeat homing operation. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            currentPhase = PHASE_WAIT_FOR_REPEAT_HOME;
            break;

        case PHASE_WAIT_FOR_REPEAT_HOME:
            // Print status periodically
            static unsigned long lastRepeatStatusPrint = 0;
            if (currentTime - lastRepeatStatusPrint > 2000)
            {
                Console.print(F("[DIAGNOSTIC] Waiting for repeat homing to complete. State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", Homed: "));
                Console.println(isHomed ? F("YES") : F("NO"));
                lastRepeatStatusPrint = currentTime;
            }

            // Replace both blocks with just:
            if (motorState == MOTOR_STATE_HOMING)
            {
                // Ensure the homing state machine advances
                checkHomingProgress();
            }

            // Wait for homing to complete successfully
            if (motorState == MOTOR_STATE_IDLE && isHomed)
            {
                cyclesCompleted++;
                Console.print(F("[INFO] Cycle "));
                Console.print(cyclesCompleted);
                Console.print(F(" completed. Position after homing: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));

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
                Console.error(F("Repeat homing failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Use a timeout longer than the internal one
            else if (currentTime - lastActionTime > 70000)
            {
                Console.error(F("Timeout waiting for repeat homing to complete."));
                Console.error(F("Cannot proceed without successful homing. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
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
            Console.info(F("Homing repeatability test completed successfully."));
            Console.print(F("[INFO] Completed "));
            Console.print(cyclesCompleted);
            Console.println(F(" cycles."));
            testRunning = false;
            testInProgress = false;
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
    // Set test flag
    testInProgress = true;

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
        Console.error(F("Motor not initialized - run 'motor init' first"));
        testInProgress = false;
        return false;
    }

    if (!isHomed)
    {
        Console.error(F("Motor not homed - run 'home' command first"));
        testInProgress = false;
        return false;
    }

    Console.info(F("Starting position cycling test"));
    Console.info(F("To abort, type any character"));
    Console.print(F("[INFO] Will perform "));
    Console.print(NUM_CYCLES);
    Console.println(F(" cycles of: Pos1 -> Pos3 -> Pos1 -> Pos2 -> Pos1"));
    Console.print(F("[INFO] Wait time at each position: "));
    Console.print(WAIT_TIME_MS);
    Console.println(F("ms"));

    lastActionTime = millis();

    while (testRunning)
    {
        // Check for abort command
        if (testAbortRequested)
        {
            Console.info(F("Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            testInProgress = false;
            testAbortRequested = false; // Reset the flag
            return false;
        }

        // Check for E-Stop condition
        if (isEStopActive())
        {
            Console.error(F("E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            testInProgress = false;
            return false;
        }

        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();

        switch (currentPhase)
        {
        case PHASE_START:
            Console.print(F("[INFO] Starting cycle "));
            Console.print(cyclesCompleted + 1);
            Console.print(F(" of "));
            Console.println(NUM_CYCLES);

            // IMPROVEMENT 1: Replace blocking code with non-blocking state pattern
            // First check we're at position 1
            if (abs(getMotorPositionMm() - POSITION_1_MM) > POSITION_TOLERANCE_MM)
            {
                Console.info(F("Moving to Position 1 to begin test"));
                if (!moveToPosition(POSITION_1))
                {
                    Console.error(F("Failed to move to Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
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
            Console.info(F("Moving: Position 1 -> Position 3"));
            if (!moveToPosition(POSITION_3))
            {
                Console.error(F("Failed to move to Position 3. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_3_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos3StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 3: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AT_POSITION_3;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 3 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 3."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AT_POSITION_3:
            // Wait at Position 3
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastWaitPos3Print = 0;
            if (currentTime - lastWaitPos3Print > 2000)
            {
                Console.print(F("[DIAGNOSTIC] Pausing at Position 3: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Waiting: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
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
            Console.info(F("Moving: Position 3 -> Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Console.error(F("Failed to move to Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos1StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 1: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AT_POSITION_1;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 1 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 1."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AT_POSITION_1:
            // Wait at Position 1
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastWaitPos1Print = 0;
            if (currentTime - lastWaitPos1Print > 2000)
            {
                Console.print(F("[DIAGNOSTIC] Pausing at Position 1: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Waiting: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
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
            Console.info(F("Moving: Position 1 -> Position 2"));
            if (!moveToPosition(POSITION_2))
            {
                Console.error(F("Failed to move to Position 2. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_2_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos2StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 2: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                lastActionTime = currentTime;
                currentPhase = PHASE_PAUSE_AT_POSITION_2;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 2 failed. Aborting test."));
                testRunning = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 2."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;

        case PHASE_PAUSE_AT_POSITION_2:
            // Wait at Position 2
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastWaitPos2Print = 0;
            if (currentTime - lastWaitPos2Print > 2000)
            {
                Console.print(F("[DIAGNOSTIC] Pausing at Position 2: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Waiting: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
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
            Console.info(F("Moving: Position 2 -> Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Console.error(F("Failed to move back to Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm), State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPosBack1StatusPrint = currentTime;
            }

            // Wait for move to complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Back at Position 1: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));

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
                Console.error(F("Movement back to Position 1 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // IMPROVEMENT 3: Add timeout handling
            else if (currentTime - lastActionTime > 30000)
            { // 30-second timeout
                Console.error(F("Timeout waiting for movement back to Position 1."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;

        case PHASE_PAUSE_BEFORE_NEXT_CYCLE:
            // Short pause before next cycle
            // IMPROVEMENT 2: Add status updates during wait phases
            static unsigned long lastPauseCycleStatusPrint = 0;
            if (currentTime - lastPauseCycleStatusPrint > 2000)
            {
                Console.print(F("[DIAGNOSTIC] Preparing for next cycle. Completed: "));
                Console.print(cyclesCompleted);
                Console.print(F("/"));
                Console.println(NUM_CYCLES);
                lastPauseCycleStatusPrint = currentTime;
            }

            if (currentTime - lastActionTime >= 2000)
            { // 2 second pause
                currentPhase = PHASE_START;
            }
            break;

        case PHASE_COMPLETE:
            // IMPROVEMENT 4: Standardize status messages
            Console.println(F("----------------------------------------"));
            Console.info(F("Position cycling test completed successfully."));
            Console.print(F("[INFO] Completed "));
            Console.print(cyclesCompleted);
            Console.print(F(" cycles of position movement (Pos1 -> Pos3 -> Pos1 -> Pos2 -> Pos1)"));
            Console.println(F("\n----------------------------------------"));
            testRunning = false;
            testInProgress = false;
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
    // Set test flag
    testInProgress = true;

    const int NUM_CYCLES = 100;                             // Number of test cycles to run
    const unsigned long WAIT_TIME_MS = 5000;               // Fixed 5-second wait time at each position
    const unsigned long VALVE_DELAY_MS = 1000;             // Delay between valve operations to prevent race conditions
    const unsigned long ADDITIONAL_UNLOCK_DELAY_MS = 2000; // Additional safety delay after tray unlock before moving
    int cyclesCompleted = 0;
    bool testRunning = true;
    unsigned long lastActionTime = 0;
    const unsigned long TRAY_SETTLING_DELAY_MS = 750; // Delay to allow tray to fully settle in position

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
        PHASE_TRAY_SETTLING_AT_POS1,              // Allow tray to fully settle at position 1
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
        PHASE_TRAY_SETTLING_AT_POS3,              // Allow tray to fully settle at position 3
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
        PHASE_TRAY_SETTLING_AT_POS1_FROM_3,             // Allow tray to fully settle at position 1
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
        PHASE_TRAY_SETTLING_AT_POS2,                    // Allow tray to fully settle at position 2
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
        PHASE_VERIFY_TRAY_BACK_AT_POS1,                // NEW: Verify tray at position 1 after return
        PHASE_TRAY_SETTLING_BACK_AT_POS1,              // NEW: Allow tray to fully settle at position 1
        PHASE_UNLOCK_SHUTTLE_END_OF_CYCLE,             // Unlock shuttle at the end of cycle
        PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_END_OF_CYCLE, // Delay after valve operation
        PHASE_PAUSE_BEFORE_NEXT_CYCLE,                 // Wait before starting next cycle
        PHASE_COMPLETE                                 // Test completion
    };

    TestPhase currentPhase = PHASE_START;

    // Check if motor is initialized and homed
    if (!motorInitialized)
    {
        Console.error(F("Motor not initialized - run 'motor init' first"));
        testInProgress = false;
        return false;
    }

    if (!isHomed)
    {
        Console.error(F("Motor not homed - run 'home' command first"));
        testInProgress = false;
        return false;
    }

    Console.info(F("This test includes empty shuttle returns and valve delays"));
    Console.info(F("To abort, type any character"));
    Console.print(F("[INFO] Will perform "));
    Console.print(NUM_CYCLES);
    Console.println(F(" cycles of tray handling operations"));
    Console.print(F("[INFO] Wait time at each position: "));
    Console.print(WAIT_TIME_MS);
    Console.println(F("ms"));
    Console.print(F("[INFO] Delay between valve operations: "));
    Console.print(VALVE_DELAY_MS);
    Console.println(F("ms"));
    Console.print(F("[INFO] Additional safety delay after tray unlock: "));
    Console.print(ADDITIONAL_UNLOCK_DELAY_MS);
    Console.println(F("ms"));

    lastActionTime = millis();

    while (testRunning)
    {
        // Check for abort command
        if (testAbortRequested)
        {
            Console.info(F("Test aborted by user"));
            stopMotion();
            motorState = MOTOR_STATE_IDLE;
            testInProgress = false;
            testInProgress = false;
            return false;
        }

        // Check for E-Stop condition
        if (isEStopActive())
        {
            Console.error(F("E-STOP detected during test! Aborting immediately."));
            // No need to call stopMotion() as the main handleEStop() will handle it
            testRunning = false;
            testInProgress = false;
            return false;
        }

        // Check motor state and proceed with test phases
        unsigned long currentTime = millis();

        switch (currentPhase)
        {
        case PHASE_START:
        {
            Console.print(F("[INFO] Starting tray handling cycle "));
            Console.print(cyclesCompleted + 1);
            Console.print(F(" of "));
            Console.println(NUM_CYCLES);
            currentPhase = PHASE_CHECK_POSITION_1;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_CHECK_POSITION_1:
        {
            // First check we're at position 1
            if (abs(getMotorPositionMm() - POSITION_1_MM) > POSITION_TOLERANCE_MM)
            {
                Console.info(F("Moving to Position 1 to begin test"));
                if (!moveToPosition(POSITION_1))
                {
                    Console.error(F("Failed to move to Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm, State: "));
                // Print motor state
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastInitMoveStatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 1: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                currentPhase = PHASE_CHECK_TRAY_AT_POS1;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 1 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 1."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS1:
        {
            // Check if tray is present at position 1
            Console.info(F("Checking for tray at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Console.error(F("No tray detected at Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Tray detected at Position 1. Waiting for tray to settle..."));
            // Change to settling phase instead of going directly to lock
            currentPhase = PHASE_TRAY_SETTLING_AT_POS1;
            lastActionTime = currentTime;
            break;
        }

        // Add new case for tray settling
        case PHASE_TRAY_SETTLING_AT_POS1:
        {
            // Wait 750ms for tray to fully settle
            static unsigned long lastSettlingPrint = 0;
            if (currentTime - lastSettlingPrint > 2000)
            {
                Console.info(F("Waiting for tray to settle at Position 1..."));
                lastSettlingPrint = currentTime;
            }

            if (currentTime - lastActionTime >= TRAY_SETTLING_DELAY_MS)
            {
                Console.info(F("Tray settling complete at Position 1. Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS1:
        {
            // Lock tray at position 1
            Console.info(F("Locking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Tray locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS1;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock tray at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS1:
        {
            // Add delay between valve operations to prevent race conditions
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to lock shuttle."));
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS1:
        {
            // Lock shuttle (position 1)
            Console.info(F("Locking shuttle at Position 1..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Shuttle locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock shuttle at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1:
        {
            // Add delay between valve operations to prevent race conditions
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS1:
        {
            // Unlock tray at position 1
            Console.info(F("Unlocking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Tray unlocked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock tray at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Adding additional safety delay before movement..."));
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
                Console.info(F("Verifying tray is still at Position 1..."));
                currentPhase = PHASE_VERIFY_TRAY_STILL_AT_POS1;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_STILL_AT_POS1:
        {
            // Check if tray is still present before moving
            Console.info(F("Verifying tray is still at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Console.error(F("Tray not detected at Position 1 after unlock. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Tray confirmed at Position 1. Checking Position 3 before moving."));
            currentPhase = PHASE_CHECK_TRAY_AT_POS3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS3:
        {
            // Check if there's already a tray at position 3 (collision risk)
            Console.info(F("Checking if Position 3 is clear..."));
            if (isTrayPresentAtPosition(3))
            {
                Console.error(F("Tray detected at Position 3. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Position 3 is clear. Proceeding with move."));
            currentPhase = PHASE_MOVE_TO_POSITION_3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_TO_POSITION_3:
        {
            // Move from Position 1 to Position 3
            Console.info(F("Moving: Position 1 -> Position 3"));
            if (!moveToPosition(POSITION_3))
            {
                Console.error(F("Failed to move to Position 3. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_3_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos3StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 3: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                currentPhase = PHASE_VERIFY_TRAY_AT_POS3;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 3 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 3."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_AT_POS3:
        {
            // Check if tray moved with shuttle to position 3
            Console.info(F("Checking for tray at Position 3..."));
            if (!isTrayPresentAtPosition(3))
            {
                Console.error(F("No tray detected at Position 3. Tray was lost during movement. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Tray successfully moved to Position 3. Waiting for tray to settle..."));
            // Change to settling phase instead of going directly to unlock shuttle
            currentPhase = PHASE_TRAY_SETTLING_AT_POS3;
            lastActionTime = currentTime;
            break;
        }

        // Add new case for tray settling at position 3
        case PHASE_TRAY_SETTLING_AT_POS3:
        {
            // Wait 750ms for tray to fully settle
            static unsigned long lastSettlingPrint = 0;
            if (currentTime - lastSettlingPrint > 2000)
            {
                Console.info(F("Waiting for tray to settle at Position 3..."));
                lastSettlingPrint = currentTime;
            }

            if (currentTime - lastActionTime >= TRAY_SETTLING_DELAY_MS)
            {
                Console.info(F("Tray settling complete at Position 3. Proceeding to unlock shuttle."));
                currentPhase = PHASE_UNLOCK_SHUTTLE_AT_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_AT_POS3:
        {
            // Unlock shuttle at position 3
            Console.info(F("Unlocking shuttle at Position 3..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Shuttle unlocked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock shuttle at Position 3. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS3:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS3:
        {
            // Lock tray at position 3
            Console.info(F("Locking tray at Position 3..."));
            DoubleSolenoidValve *valve3 = getTray3Valve();
            CylinderSensor *sensor3 = getTray3Sensor();

            if (valve3 && sensor3)
            {
                if (safeValveOperation(*valve3, *sensor3, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Tray locked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock tray at Position 3. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 3. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS3:
        {
            // Add delay between operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Tray locked at Position 3. Waiting for 5 seconds..."));
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
                Console.print(F("[DIAGNOSTIC] Waiting at Position 3 with tray locked. Elapsed: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
                lastWaitPos3Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Console.info(F("Moving empty shuttle back to Position 1..."));
                currentPhase = PHASE_RETURN_TO_POS1_FROM_POS3_EMPTY;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS1_FROM_POS3_EMPTY:
        {
            // Return empty shuttle to position 1
            Console.info(F("Moving empty shuttle: Position 3 -> Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Console.error(F("Failed to move empty shuttle to Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Empty return status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastEmptyReturnStatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Empty shuttle reached Position 1: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                currentPhase = PHASE_WAIT_AT_POS1_EMPTY;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Empty shuttle movement to Position 1 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Console.error(F("Timeout waiting for empty shuttle return to Position 1."));
                stopMotion();
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Waiting at Position 1 with empty shuttle. Elapsed: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
                lastWaitPos1EmptyPrint = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Console.info(F("Returning to Position 3 to pick up tray..."));
                currentPhase = PHASE_RETURN_TO_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS3:
        {
            // Return to position 3 to pick up tray
            Console.info(F("Moving empty shuttle back: Position 1 -> Position 3"));
            if (!moveToPosition(POSITION_3))
            {
                Console.error(F("Failed to return empty shuttle to Position 3. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Return status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_3_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastReturnToPos3StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Returned to Position 3: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));

                // Verify tray is still at position 3
                if (!isTrayPresentAtPosition(3))
                {
                    Console.error(F("No tray detected at Position 3 after return. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
                Console.info(F("Tray confirmed still at Position 3. Proceeding to pick it up."));
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS3;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Return to Position 3 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Console.error(F("Timeout waiting for return to Position 3."));
                stopMotion();
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS3:
        {
            // Lock shuttle at position 3
            Console.info(F("Locking shuttle at Position 3..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Shuttle locked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock shuttle at Position 3. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS3:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS3:
        {
            // Unlock tray at position 3
            Console.info(F("Unlocking tray at Position 3..."));
            DoubleSolenoidValve *valve3 = getTray3Valve();
            CylinderSensor *sensor3 = getTray3Sensor();

            if (valve3 && sensor3)
            {
                if (safeValveOperation(*valve3, *sensor3, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Tray unlocked at Position 3."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock tray at Position 3. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 3. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS3:
        {
            // Add delay between operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Adding additional safety delay before movement..."));
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
                Console.info(F("Checking Position 1 before moving."));
                currentPhase = PHASE_CHECK_TRAY_AT_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS1_AGAIN:
        {
            // Check if there's already a tray at position 1 (collision risk)
            Console.info(F("Checking if Position 1 is clear for return..."));
            if (isTrayPresentAtPosition(1))
            {
                Console.error(F("Tray detected at Position 1. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Position 1 is clear. Proceeding with move."));
            currentPhase = PHASE_MOVE_TO_POSITION_1_FROM_3;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_TO_POSITION_1_FROM_3:
        {
            // Move from Position 3 to Position 1
            Console.info(F("Moving: Position 3 â†’ Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Console.error(F("Failed to move to Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos1From3StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 1 from Position 3: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                currentPhase = PHASE_VERIFY_TRAY_AT_POS1_FROM_3;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 1 from Position 3 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 1 from Position 3."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                return false;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_AT_POS1_FROM_3:
        {
            // Check if tray is at position 1 after return from position 3
            Console.info(F("Checking for tray at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Console.error(F("No tray detected at Position 1 after return from Position 3. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Tray confirmed at Position 1. Waiting for tray to settle..."));
            // Change to settling phase instead of going directly to unlock shuttle
            currentPhase = PHASE_TRAY_SETTLING_AT_POS1_FROM_3;
            lastActionTime = currentTime;
            break;
        }

        // Add new case for tray settling
        case PHASE_TRAY_SETTLING_AT_POS1_FROM_3:
        {
            // Wait 750ms for tray to fully settle
            static unsigned long lastSettlingPrint = 0;
            if (currentTime - lastSettlingPrint > 2000)
            {
                Console.info(F("Waiting for tray to settle at Position 1 after return from Position 3..."));
                lastSettlingPrint = currentTime;
            }

            if (currentTime - lastActionTime >= TRAY_SETTLING_DELAY_MS)
            {
                Console.info(F("Tray settling complete at Position 1. Proceeding to unlock shuttle."));
                currentPhase = PHASE_UNLOCK_SHUTTLE_AT_POS1_FROM_3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_AT_POS1_FROM_3:
        {
            // Unlock shuttle at position 1 after return from position 3
            Console.info(F("Unlocking shuttle at Position 1..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Shuttle unlocked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS1_FROM_3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock shuttle at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS1_FROM_3:
        {
            // Add delay between valve operations
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS1_FROM_3;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS1_FROM_3:
        {
            // Lock tray at position 1 after return from position 3
            Console.info(F("Locking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Tray locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS1_FROM_3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock tray at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS1_FROM_3:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Tray locked at Position 1. Waiting for 5 seconds..."));
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
                Console.print(F("[DIAGNOSTIC] Waiting at Position 1 with tray locked. Elapsed: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
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
            Console.info(F("Locking shuttle at Position 1..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Shuttle locked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1_FROM_3;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock shuttle at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS1_FROM_3:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS1_AGAIN:
        {
            // Unlock tray at position 1 before moving to position 2
            Console.info(F("Unlocking tray at Position 1..."));
            DoubleSolenoidValve *valve1 = getTray1Valve();
            CylinderSensor *sensor1 = getTray1Sensor();

            if (valve1 && sensor1)
            {
                if (safeValveOperation(*valve1, *sensor1, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Tray unlocked at Position 1."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS1_AGAIN;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock tray at Position 1. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.info(F("Adding additional safety delay before movement..."));
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
                Console.info(F("Verifying tray is still at Position 1..."));
                currentPhase = PHASE_VERIFY_TRAY_STILL_AT_POS1_AGAIN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_STILL_AT_POS1_AGAIN:
        {
            // Check if tray is still present before moving to position 2
            Console.info(F("Verifying tray is still at Position 1..."));
            if (!isTrayPresentAtPosition(1))
            {
                Console.error(F("Tray not detected at Position 1 after unlock. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Tray confirmed at Position 1. Checking Position 2 before moving."));
            currentPhase = PHASE_CHECK_TRAY_AT_POS2;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS2:
        {
            // Check if there's already a tray at position 2 (collision risk)
            Console.info(F("Checking if Position 2 is clear..."));
            if (isTrayPresentAtPosition(2))
            {
                Console.error(F("Tray detected at Position 2. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Position 2 is clear. Proceeding with move."));
            currentPhase = PHASE_MOVE_TO_POSITION_2;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_TO_POSITION_2:
        {
            // Move from Position 1 to Position 2
            Console.info(F("Moving: Position 1 -> Position 2"));
            if (!moveToPosition(POSITION_2))
            {
                Console.error(F("Failed to move to Position 2. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_2_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastPos2StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Reached Position 2: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                currentPhase = PHASE_VERIFY_TRAY_AT_POS2;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement to Position 2 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            { // 60-second timeout
                Console.error(F("Timeout waiting for movement to Position 2."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_VERIFY_TRAY_AT_POS2:
        {
            // Check if tray is at position 2
            Console.info(F("Checking for tray at Position 2..."));
            if (!isTrayPresentAtPosition(2))
            {
                Console.error(F("No tray detected at Position 2. Tray was lost during movement. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Tray successfully moved to Position 2. Waiting for tray to settle..."));
            // Change to settling phase instead of going directly to unlock shuttle
            currentPhase = PHASE_TRAY_SETTLING_AT_POS2;
            lastActionTime = currentTime;
            break;
        }

        // Add new case for tray settling
        case PHASE_TRAY_SETTLING_AT_POS2:
        {
            // Wait 750ms for tray to fully settle
            static unsigned long lastSettlingPrint = 0;
            if (currentTime - lastSettlingPrint > 2000)
            {
                Console.info(F("Waiting for tray to settle at Position 2..."));
                lastSettlingPrint = currentTime;
            }

            if (currentTime - lastActionTime >= TRAY_SETTLING_DELAY_MS)
            {
                Console.info(F("Tray settling complete at Position 2. Proceeding to unlock shuttle."));
                currentPhase = PHASE_UNLOCK_SHUTTLE_AT_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_AT_POS2:
        {
            // Unlock shuttle at position 2
            Console.info(F("Unlocking shuttle at Position 2..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Shuttle unlocked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock shuttle at Position 2. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to lock tray."));
                currentPhase = PHASE_LOCK_TRAY_AT_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_LOCK_TRAY_AT_POS2:
        {
            // Lock tray at position 2
            Console.info(F("Locking tray at Position 2..."));
            DoubleSolenoidValve *valve2 = getTray2Valve();
            CylinderSensor *sensor2 = getTray2Sensor();

            if (valve2 && sensor2)
            {
                if (safeValveOperation(*valve2, *sensor2, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Tray locked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_TRAY_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock tray at Position 2. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 2. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_TRAY_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Tray locked at Position 2. Waiting for 5 seconds..."));
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
                Console.print(F("[DIAGNOSTIC] Waiting at Position 2 with tray locked. Elapsed: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
                lastWaitPos2Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Console.info(F("Moving empty shuttle back to Position 1..."));
                currentPhase = PHASE_RETURN_TO_POS1_FROM_POS2_EMPTY;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS1_FROM_POS2_EMPTY:
        {
            // Return empty shuttle to position 1 from position 2
            Console.info(F("Moving empty shuttle: Position 2 -> Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Console.error(F("Failed to move empty shuttle to Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Empty return status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastEmptyReturnFromPos2StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Empty shuttle reached Position 1 from Position 2: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));
                currentPhase = PHASE_WAIT_AT_POS1_EMPTY_FROM_POS2;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Empty shuttle movement to Position 1 from Position 2 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Console.error(F("Timeout waiting for empty shuttle return to Position 1."));
                stopMotion();
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Waiting at Position 1 with empty shuttle from Position 2. Elapsed: "));
                Console.print((currentTime - lastActionTime) / 1000);
                Console.print(F("/"));
                Console.print(WAIT_TIME_MS / 1000);
                Console.println(F(" seconds"));
                lastWaitPos1EmptyFromPos2Print = currentTime;
            }

            if (currentTime - lastActionTime >= WAIT_TIME_MS)
            {
                Console.info(F("Returning to Position 2 to pick up tray..."));
                currentPhase = PHASE_RETURN_TO_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_RETURN_TO_POS2:
        {
            // Return to position 2 to pick up tray
            Console.info(F("Moving empty shuttle back: Position 1 -> Position 2"));
            if (!moveToPosition(POSITION_2))
            {
                Console.error(F("Failed to return empty shuttle to Position 2. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Return status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_2_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastReturnToPos2StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Returned to Position 2: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));

                // Verify tray is still at position 2
                if (!isTrayPresentAtPosition(2))
                {
                    Console.error(F("No tray detected at Position 2 after return. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
                Console.info(F("Tray confirmed still at Position 2. Proceeding to pick it up."));
                currentPhase = PHASE_LOCK_SHUTTLE_AT_POS2;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Return to Position 2 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000)
            {
                Console.error(F("Timeout waiting for return to Position 2."));
                stopMotion();
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_LOCK_SHUTTLE_AT_POS2:
        {
            // Lock shuttle at position 2
            Console.info(F("Locking shuttle at Position 2..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_LOCK, 1000))
                {
                    Console.info(F("Shuttle locked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to lock shuttle at Position 2. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_LOCK_SHUTTLE_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Proceeding to unlock tray."));
                currentPhase = PHASE_UNLOCK_TRAY_AT_POS2;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_UNLOCK_TRAY_AT_POS2:
        {
            // Unlock tray at position 2
            Console.info(F("Unlocking tray at Position 2..."));
            DoubleSolenoidValve *valve2 = getTray2Valve();
            CylinderSensor *sensor2 = getTray2Sensor();

            if (valve2 && sensor2)
            {
                if (safeValveOperation(*valve2, *sensor2, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Tray unlocked at Position 2."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_TRAY_POS2;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock tray at Position 2. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access valve or sensor for Position 2. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        case PHASE_DELAY_AFTER_UNLOCK_TRAY_POS2:
        {
            // Add delay after valve operation
            if (currentTime - lastActionTime >= VALVE_DELAY_MS)
            {
                Console.info(F("Adding additional safety delay before movement..."));
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
                Console.info(F("Checking Position 1 before final return."));
                currentPhase = PHASE_CHECK_TRAY_AT_POS1_BEFORE_RETURN;
                lastActionTime = currentTime;
            }
            break;
        }

        case PHASE_CHECK_TRAY_AT_POS1_BEFORE_RETURN:
        {
            // Check if there's already a tray at position 1 (collision risk)
            Console.info(F("Checking if Position 1 is clear for final return..."));
            if (isTrayPresentAtPosition(1))
            {
                Console.error(F("Tray detected at Position 1. Cannot move - collision risk. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            Console.info(F("Position 1 is clear. Proceeding with final move back to Position 1."));
            currentPhase = PHASE_MOVE_BACK_TO_POSITION_1;
            lastActionTime = currentTime;
            break;
        }

        case PHASE_MOVE_BACK_TO_POSITION_1:
        {
            // Move from Position 2 back to Position 1
            Console.info(F("Moving: Position 2 -> Position 1"));
            if (!moveToPosition(POSITION_1))
            {
                Console.error(F("Failed to move back to Position 1. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Move status - Position: "));
                Console.print(getMotorPositionMm());
                Console.print(F("mm, Target: "));
                Console.print(POSITION_1_MM);
                Console.print(F("mm, State: "));
                switch (motorState)
                {
                case MOTOR_STATE_IDLE:
                    Console.print(F("IDLE"));
                    break;
                case MOTOR_STATE_MOVING:
                    Console.print(F("MOVING"));
                    break;
                case MOTOR_STATE_HOMING:
                    Console.print(F("HOMING"));
                    break;
                case MOTOR_STATE_FAULTED:
                    Console.print(F("FAULTED"));
                    break;
                case MOTOR_STATE_NOT_READY:
                    Console.print(F("NOT_READY"));
                    break;
                default:
                    Console.print(F("UNKNOWN"));
                }
                Console.print(F(", StepsComplete: "));
                Console.println(MOTOR_CONNECTOR.StepsComplete() ? F("YES") : F("NO"));
                lastMoveBackPos1StatusPrint = currentTime;
            }

            // Check if move is complete
            if (MOTOR_CONNECTOR.StepsComplete() && motorState != MOTOR_STATE_FAULTED)
            {
                Console.print(F("[INFO] Back at Position 1: "));
                Console.print(getMotorPositionMm());
                Console.println(F("mm"));

                // Verify tray made it back to position 1
                if (!isTrayPresentAtPosition(1))
                {
                    Console.error(F("No tray detected at Position 1 after return from Position 2. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }

                // REMOVED: Don't increment cycle count here
                // Instead, proceed to verification and settling phase
                Console.info(F("Tray detected at Position 1. Moving to verification phase."));
                currentPhase = PHASE_VERIFY_TRAY_BACK_AT_POS1;
                lastActionTime = currentTime;
            }
            else if (motorState == MOTOR_STATE_FAULTED)
            {
                Console.error(F("Movement back to Position 1 failed. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }
            // Add timeout handling
            else if (currentTime - lastActionTime > 60000) // 60-second timeout
            {
                Console.error(F("Timeout waiting for movement back to Position 1."));
                Console.error(F("Movement took too long. Aborting test."));
                stopMotion(); // Safety stop
                testRunning = false;
                testInProgress = false;
                return false;
            }
            break;
        }

        // Verification phase - checks tray is present before settling
        case PHASE_VERIFY_TRAY_BACK_AT_POS1:
        {
            // Double-check tray is present at position 1
            if (!isTrayPresentAtPosition(1))
            {
                Console.error(F("Tray not detected at Position 1 during final verification. Aborting test."));
                testRunning = false;
                testInProgress = false;
                return false;
            }

            Console.info(F("Tray confirmed at Position 1. Waiting for tray to settle..."));
            currentPhase = PHASE_TRAY_SETTLING_BACK_AT_POS1;
            lastActionTime = currentTime;
            break;
        }

        // Tray settling phase - adds delay before continuing
        case PHASE_TRAY_SETTLING_BACK_AT_POS1:
        {
            // Wait 750ms for tray to fully settle
            static unsigned long lastSettlingPrint = 0;
            if (currentTime - lastSettlingPrint > 2000)
            {
                Console.info(F("Waiting for tray to settle at Position 1 after cycle completion..."));
                lastSettlingPrint = currentTime;
            }

            if (currentTime - lastActionTime >= TRAY_SETTLING_DELAY_MS)
            {
                Console.info(F("Tray settling complete at Position 1."));

                // NOW increment cycle count after tray has fully settled
                cyclesCompleted++;

                // Report cycle status
                Console.print(F("[INFO] Cycle "));
                Console.print(cyclesCompleted);
                Console.print(F(" of "));
                Console.print(NUM_CYCLES);
                Console.println(F(" completed"));

                lastActionTime = currentTime;

                if (cyclesCompleted >= NUM_CYCLES)
                {
                    currentPhase = PHASE_COMPLETE;
                }
                else
                {
                    currentPhase = PHASE_UNLOCK_SHUTTLE_END_OF_CYCLE;
                }
            }
            break;
        }

        case PHASE_UNLOCK_SHUTTLE_END_OF_CYCLE:
        {
            // Unlock shuttle at the end of cycle
            Console.info(F("Unlocking shuttle at end of cycle..."));
            DoubleSolenoidValve *shuttleValve = getShuttleValve();
            CylinderSensor *shuttleSensor = getShuttleSensor();

            if (shuttleValve && shuttleSensor)
            {
                if (safeValveOperation(*shuttleValve, *shuttleSensor, VALVE_POSITION_UNLOCK, 1000))
                {
                    Console.info(F("Shuttle unlocked at end of cycle."));
                    currentPhase = PHASE_DELAY_AFTER_UNLOCK_SHUTTLE_END_OF_CYCLE;
                    lastActionTime = currentTime;
                }
                else
                {
                    Console.error(F("Failed to unlock shuttle at end of cycle. Aborting test."));
                    testRunning = false;
                    testInProgress = false;
                    return false;
                }
            }
            else
            {
                Console.error(F("Failed to access shuttle valve or sensor. Aborting test."));
                testRunning = false;
                testInProgress = false;
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
                Console.print(F("[DIAGNOSTIC] Preparing for next cycle. Completed: "));
                Console.print(cyclesCompleted);
                Console.print(F("/"));
                Console.println(NUM_CYCLES);
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
            Console.println(F("----------------------------------------"));
            Console.info(F("Enhanced tray handling test completed successfully."));
            Console.print(F("[INFO] Completed "));
            Console.print(cyclesCompleted);
            Console.println(F(" cycles of tray handling operations."));
            Console.println(F("----------------------------------------"));
            testRunning = false;
            testInProgress = false;
            return true;
            break; // Note: This break is never reached because of the return
        }
        } 

        
        delayMicroseconds(100);

    } 

    
    return false;
} 