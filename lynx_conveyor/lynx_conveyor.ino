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
#include "Commands.h"
#include "Utils.h"
#include "EncoderController.h"

// Specify which ClearCore serial COM port is connected to the CCIO-8 board
#define CcioPort ConnectorCOM0

uint8_t ccioBoardCount; // Store the number of connected CCIO-8 boards here
uint8_t ccioPinCount;   // Store the number of connected CCIO-8 pins here

// The setup function
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("[MESSAGE] Lynx Conveyor Controller starting up..."));

    // First set up the CCIO board
    Serial.println(F("[MESSAGE] Initializing CCIO-8 expansion boards..."));
    CcioPort.Mode(Connector::CCIO);
    CcioPort.PortOpen();

    // Get count of CCIO-8 boards
    ccioBoardCount = CcioMgr.CcioCount();
    Serial.print(F("[MESSAGE] Discovered CCIO boards: "));
    Serial.println(ccioBoardCount);

    // Now initialize sensor systems
    Serial.println(F("[MESSAGE] Initializing sensor systems..."));
    initSensorSystem();

    // Initialize valve system with CCIO board status
    Serial.println(F("[MESSAGE] Initializing valve controller..."));
    initValveSystem(ccioBoardCount > 0);

    // Initialize encoder with default direction (modify if needed)
    Serial.println(F("[MESSAGE] Initializing MPG handwheel interface..."));
    initEncoderControl(true, false);

    // Rest of your setup code...
    Serial.println(F("[MESSAGE] Motor controller ready for initialization."));
    Serial.println(F("[MESSAGE] Use 'motor init' command to initialize the motor."));

    commander.attachTree(API_tree);
    commander.init();

    Serial.println(F("[MESSAGE] System ready."));
    Serial.println(F("[MESSAGE] Type 'help' for available commands"));
}

// The main loop
void loop()
{
    unsigned long currentTime = millis();

    // Check for E-stop condition (highest priority)
    handleEStop();

    // Always capture current system state - it's the foundation of safety
    SystemState currentState = captureSystemState();

    // Handle incoming serial commands using Commander API
    handleSerialCommands();

    // Process fault clearing if in progress
    processFaultClearing();

    // Always check move progress - not just when MOVING
    // This ensures we catch the transition from moving to stopped
    checkMoveProgress();

    // Then update motor state
    updateMotorState();

    // Process enable cycling for homing if in progress
    cycleMotorEnableForHoming();

    // Check homing progress if in progress
    if (motorState == MOTOR_STATE_HOMING)
    {
        executeHomingSequence();
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
        (!safety.operationWithinTimeout || !safety.operationSequenceValid)) {
        Serial.print(F("SAFETY VIOLATION: "));
        Serial.println(safety.operationSequenceMessage);

        // Emergency stop or other recovery action
        // Use the failureReason from safety validation
        abortOperation(safety.failureReason);
    }

    // Process tray operations if any are in progress
    processTrayOperations();

    // Store current state as previous for next cycle
    previousState = currentState;

    // Process encoder input if enabled
    if (encoderControlActive) {
        processEncoderInput();
    }
}

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