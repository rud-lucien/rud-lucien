#line 1 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/lynx_conveyor.ino"
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
#line 27 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/lynx_conveyor.ino"
void setup();
#line 80 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/lynx_conveyor.ino"
void loop();
#line 27 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/lynx_conveyor.ino"
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Console.serialInfo(F("Lynx Conveyor Controller starting up..."));

    Console.serialInfo(F("Initializing Ethernet interface..."));
    initEthernetController(false); // true = use DHCP

    // Initialize the output manager (must be before any Console calls)
    initOutputManager();

    // First set up the CCIO board
    Console.serialInfo(F("Initializing CCIO-8 expansion boards..."));
    CcioPort.Mode(Connector::CCIO);
    CcioPort.PortOpen();

    // Get count of CCIO-8 boards
    ccioBoardCount = CcioMgr.CcioCount();
    char msg[50];
    sprintf(msg, "[INFO] Discovered CCIO boards: %d", ccioBoardCount);
    Console.serialInfo(msg);

    // Now initialize sensor systems
    Console.serialInfo(F("Initializing sensor systems..."));
    initSensorSystem();

    // Initialize valve system with CCIO board status
    Console.serialInfo(F("Initializing valve controller..."));
    initValveSystem(ccioBoardCount > 0);

    // Initialize encoder with default direction (modify if needed)
    Console.serialInfo(F("Initializing MPG handwheel interface..."));
    initEncoderControl(true, false);

    // Rest of your setup code...
    Console.serialInfo(F("Motor controller ready for initialization."));
    Console.serialInfo(F("Use 'motor init' command to initialize the motor."));

    // Initialize system state variables including target position tracking
    Console.serialInfo(F("Initializing system state variables..."));
    initSystemStateVariables();

    commander.attachTree(API_tree);
    commander.init();
    initTestFlags();

    Console.serialInfo(F("System ready."));
    Console.serialInfo(F("Type 'help' for available commands"));
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
        Console.serialInfo(F("Test abort detected in main loop"));
        handleTestAbort();
        Console.acknowledge(F("Test aborted successfully"));
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
    if (logging.logInterval > 0 && waitTimeReached(currentTime, logging.previousLogTime, logging.logInterval))
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
        char errorMsg[200];
        sprintf(errorMsg, "SAFETY VIOLATION: %s", safety.operationSequenceMessage.c_str());
        Console.error(errorMsg);

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

    // Pressure check using waitTimeReached helper for safe rollover handling
    static const unsigned long PRESSURE_CHECK_INTERVAL = 10000; // 10 seconds
    static unsigned long lastPressureCheckTime = 0;

    if (waitTimeReached(currentTime, lastPressureCheckTime, PRESSURE_CHECK_INTERVAL))
    {
        if (!isPressureSufficient())
        {
            Console.serialWarning(F("System pressure below minimum threshold (21.75 PSI)"));
        }
        lastPressureCheckTime = currentTime;
    }
}

