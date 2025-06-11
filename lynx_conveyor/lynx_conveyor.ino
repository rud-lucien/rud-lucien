/*
 Name:		lynx_linear_actuator.ino
 Created:	4/14/2025 12:00:17 PM
 Author:	rlucien
*/

#include "Arduino.h"
#include "ClearCore.h"
#include "ValveController.h"
#include "MotorController.h"
#include "Logging.h"
#include "Tests.h"
#include "CommandController.h"
#include "Commands.h"
#include "Utils.h"
#include "EncoderController.h"
#include "OutputManager.h"
#include "EthernetController.h"

// Specify which ClearCore serial COM port is connected to the CCIO-8 board
#define CcioPort ConnectorCOM0

uint8_t ccioBoardCount; // Store the number of connected CCIO-8 boards here
uint8_t ccioPinCount;   // Store the number of connected CCIO-8 pins here

// The setup function
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Console.info(F("Initializing Ethernet interface..."));
    initEthernetController(true); // true = use DHCP

    // Initialize the output manager (must be before any Console calls)
    initOutputManager();

    Console.info(F("Lynx Conveyor Controller starting up..."));

    // First set up the CCIO board
    Console.info(F("Initializing CCIO-8 expansion boards..."));
    CcioPort.Mode(Connector::CCIO);
    CcioPort.PortOpen();

    // Get count of CCIO-8 boards
    ccioBoardCount = CcioMgr.CcioCount();
    Console.print(F("[INFO] Discovered CCIO boards: "));
    Console.println(ccioBoardCount);

    // Now initialize sensor systems
    Console.info(F("Initializing sensor systems..."));
    initSensorSystem();

    // Initialize valve system with CCIO board status
    Console.info(F("Initializing valve controller..."));
    initValveSystem(ccioBoardCount > 0);

    // Initialize encoder with default direction (modify if needed)
    Console.info(F("Initializing MPG handwheel interface..."));
    initEncoderControl(true, false);

    // Rest of your setup code...
    Console.info(F("Motor controller ready for initialization."));
    Console.info(F("Use 'motor init' command to initialize the motor."));

    commander.attachTree(API_tree);
    commander.init();
    initTestFlags();

    Console.info(F("System ready."));
    Console.info(F("Type 'help' for available commands"));
}

// The main loop
void loop()
{

    unsigned long currentTime = millis();

    // Check for E-stop condition (highest priority)
    handleEStop();

    // Check for test abort requested
    if (testAbortRequested && testInProgress)
    {
        Console.info(F("Test abort detected in main loop"));
        handleTestAbort();
        Console.info(F("Test aborted successfully"));
    }

    // Always capture current system state - it's the foundation of safety
    SystemState currentState = captureSystemState();

    // Update tray tracking from physical sensors each cycle
    updateTrayTrackingFromSensors(currentState);

    // Handle incoming serial commands using Commander API
    handleSerialCommands();

    // Process Ethernet connections and commands
    // This will handle both incoming connections and command processing
    processEthernetConnections();
    handleEthernetCommands();

    // Process fault clearing if in progress
    processFaultClearing();

    // Always check move progress - not just when MOVING
    // This ensures we catch the transition from moving to stopped
    checkMoveProgress();

    // Then update motor state
    updateMotorState();

    // Check homing progress if in progress
    if (motorState == MOTOR_STATE_HOMING)
    {
        checkHomingProgress();
    }

    // Log system state periodically if logging is enabled
    if (logging.logInterval > 0 && currentTime - logging.previousLogTime >= logging.logInterval)
    {
        logging.previousLogTime = currentTime;
        logSystemState();
    }

    // Periodic safety validation using already captured state
    SafetyValidationResult safety = validateSafety(currentState);

    // Check for safety violations that require immediate action
    if (operationInProgress &&
        (!safety.operationWithinTimeout || !safety.operationSequenceValid))
    {
        Console.print(F("[SAFETY] VIOLATION: "));
        Console.println(safety.operationSequenceMessage);

        // Emergency stop or other recovery action
        // Use the failureReason from safety validation
        abortOperation(safety.failureReason);
    }

    // Process tray operations if any are in progress
    processTrayOperations();

    // Store current state as previous for next cycle
    previousState = currentState;

    // Process encoder input if enabled
    if (encoderControlActive)
    {
        processEncoderInput();
    }

    static unsigned long lastPressureCheckTime = 0;
    if (currentTime - lastPressureCheckTime > 10000)
    { // Check every 10 seconds
        if (!isPressureSufficient())
        {
            Console.warning(F("System pressure below minimum threshold (21.75 PSI)"));
        }
        lastPressureCheckTime = currentTime;
    }
}

/* TODO: Figure out how to implement collision detection
This task is about implementing a collision detection system for the conveyor controller to prevent mechanical damage during operations.
in the clearpath settings I lowered the max torque to 20%, basically if the motor is stalled it will not try to push through the stall, it will just stop.
How do I update code to handle this? when the motor is stalled what message does it send and how do we deal with it currently? how do we make sure that stalling
prints a helpful message to the user that this is potentially a collision detection issue and not just a motor fault?
*/

/* # TODO: Implement Operation Step Sequence Validation

This TODO task should focus on improving the operation step sequence validation system. Here's what should be included:

## Task Description
Enhance the operation sequence validation system to properly track and validate operation steps during automated processes.

## Current Implementation Issues
- Both `currentOperationStep` and `expectedOperationStep` are initialized to 0 in `beginOperation()`
- `currentOperationStep` is updated when operation steps advance
- `expectedOperationStep` is not being updated anywhere, making the mismatch check ineffective
- The sequence validation cannot properly detect out-of-order steps

## Proposed Solutions
1. Define clear operation sequences for each operation type (loading, unloading, etc.)
2. Create an update mechanism for `expectedOperationStep` that:
   - Validates transitions between steps
   - Defines allowed sequences for each operation
   - Updates `expectedOperationStep` when a step completes successfully

## Implementation Approach
1. Create a data structure defining valid step transitions for each operation type
   ```cpp
   struct OperationSequence {
       OperationType type;
       int totalSteps;
       bool validTransitions[MAX_STEPS][MAX_STEPS]; // Matrix of allowed transitions
   };
   ```

2. Implement a validation function that checks if a step transition is valid
   ```cpp
   bool isValidStepTransition(OperationType type, int currentStep, int nextStep);
   ```

3. Update the step tracking system to advance `expectedOperationStep` when appropriate

## Integration with Abort System
Once implemented, update the state mismatch check to set the proper abort reason:
```cpp
if (operationInProgress && currentOperationStep != expectedOperationStep) {
    result.operationSequenceValid = false;
    result.operationSequenceMessage = F("Operation sequence mismatch");
    result.failureReason = ABORT_REASON_SEQUENCE_ERROR; // New reason code
}
```

## Priority
Medium - This is important for robust operation but not blocking immediate abort functionality implementation.

*/
