/*
 Name:		lynx_conveyor.ino
 Created:	4/14/2025 12:00:17 PM
 Author:	rlucien
 
 LYNX CONVEYOR CONTROLLER
 ========================
 
 SYSTEM PURPOSE:
 This program controls an automated linear rail conveyor system designed for precise tray handling
 and positioning. The system manages tray loading, transport between stations, unloading operations, 
 and comprehensive safety monitoring with pneumatic shuttle and locking control.
 
 CORE FUNCTIONALITY:
 - Precise linear motion control with 3 tray loading positions plus home position
 - Pneumatic shuttle system that can lock/unlock to grab and release trays during transport
 - Individual pneumatic tray locks at each position (lock1, lock2, lock3) to secure trays
 - Tray detection sensors throughout the system to track tray locations
 - Pressure monitoring to ensure pneumatic systems have adequate pressure for operation
 - Manual positioning control via MPG handwheel encoder interface
 - Comprehensive logging with circular buffer for error diagnosis and system history
 - Serial and Ethernet command interfaces with extensive help documentation
 - Automated test sequences for system validation and troubleshooting
 
 REQUIRED HARDWARE COMPONENTS:
 
 1. CONTROLLER & I/O:
    - ClearCore Industrial I/O and Motion Controller (main controller)
    - CCIO-8 Digital I/O Expansion (8-point expansion for additional sensors/outputs)
    - CABLE-RIBBON6 (6 inch ribbon cable for CCIO connection)
 
 2. MOTION SYSTEM:
    - NEMA 23 ClearPath-SDSK Model CPM-SDSK-2321S-RLS (servo motor with integrated drive)
    - Linear rail system with precision positioning
    
    REQUIRED MOTOR CONFIGURATION (using Teknic ClearPath-MSP software):
    - Input Resolution: 800 pulses per revolution
    - Input Format: Step and Direction
    - Torque Limit: 50%
    - HLFB Output: ASG-POSITION WITH MEASURED TORQUE
    - Homing: Enabled
      * Homing Trigger: Upon Every Enable
      * Homing Torque Limit: 40%
      * Homing Mode: Normal
      * Homing Style: User Seeks Home
      * Precision Homing: Must be calibrated per installation
      * Homing RPM: 50 RPM
      * Acceleration/Deceleration: 1500 RPM/s
    
    MOTOR SETUP PROCEDURE:
    1. Configure all settings above using Teknic ClearPath-MSP software
    2. Load configuration to motor (download settings to motor memory)
    3. Perform auto-tuning with realistic load conditions:
       * Lock a tray to the shuttle to simulate actual operating load
       * Set auto-tune torque limit to 50%
       * Set auto-tune RPM to 350 RPM (matches operational velocity with tray)
       * Run auto-tune sequence to optimize motor performance
    4. Verify homing operation and position accuracy after installation
    
    NOTE: Motor must be configured and auto-tuned before use. Auto-tuning with 
    actual load is critical for optimal performance and accuracy.
 
 3. FEEDBACK & CONTROL:
    - CL-ENCDR-DFIN Encoder Input Adapter (for MPG handwheel manual control)
    - MPG handwheel encoder (manual positioning interface)
    - Tray detection sensors positioned throughout the conveyor system
 
 4. PNEUMATIC SYSTEM:
    - Pneumatic shuttle mechanism with lock/unlock capability for tray grabbing
    - Individual pneumatic tray locks at each of the 3 loading positions
    - Pressure sensor for pneumatic system monitoring (minimum 21.75 PSI)
    - Solenoid valves for shuttle and tray lock control
    - Compressed air supply system
 
 5. POWER SYSTEM:
    - IPC-5 DC Power Supply (350/500W, 75VDC output for motor power)
    - POWER4-STRIP DC Bus Distribution Strip (power distribution)
    - 24VDC supply for logic and pneumatics
 
 6. SAFETY SYSTEMS:
    - Emergency stop (E-stop) circuit with normally closed contacts
    - Pressure monitoring system (minimum 21.75 PSI threshold)
    - Tray detection sensors for position tracking and safety validation
    - Comprehensive error detection with historical logging
 
 COMMUNICATION INTERFACES:
 - Serial (USB): Direct command interface and diagnostics (115200 baud)
 - Ethernet: Remote command interface and monitoring
 - Extensive help system: Type "help" for available commands, "help <command>" for specific usage
 - All commands include detailed help documentation and usage examples
 
 USAGE:
 1. Configure ClearPath motor using Teknic ClearPath-MSP software (see MOTION SYSTEM requirements)
 2. Connect all hardware components per system documentation
 3. Power up system and connect via Serial or Ethernet
 4. Initialize motor system: "motor init"
 5. Home the system: "motor home" 
 6. Position control: "move 1" (positions 1-3) or "move <mm>"
 7. Tray operations: "tray load", "tray unload" 
 8. Tray locking: "lock1", "lock2", "lock3" for individual position locks
 9. Shuttle control: Lock shuttle to grab trays, unlock to release
 10. Manual control: Enable handwheel with "encoder enable"
 11. System monitoring: "system status", "motor status", pressure monitoring
 12. Help system: "help" for command list, "help <command>" for detailed usage
 13. Error diagnosis: Check circular buffer log when issues occur

 SAFETY NOTES:
 - Always ensure E-stop circuit is properly connected and functional
 - Verify adequate air pressure (21.75+ PSI) before pneumatic operations
 - System includes comprehensive safety monitoring and automatic fault detection
 - Circular buffer maintains operation history for troubleshooting errors
 - Use "system reset" to clear fault conditions after resolving issues
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
