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
 - Dynamic position teaching system with SD card persistence for field adjustment
 - Pneumatic shuttle system that can lock/unlock to grab and release trays during transport
 - Individual pneumatic tray locks at each position (lock1, lock2, lock3) to secure trays
 - Tray detection sensors throughout the system to track tray locations
 - Pressure monitoring to ensure pneumatic systems have adequate pressure for operation
 - Manual positioning control via MPG handwheel encoder interface
 - Comprehensive logging with circular buffer for error diagnosis and system history
 - Serial and Ethernet command interfaces with extensive help documentation

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
      * Homing Mode: Normal
      * Homing Style: User seeks home; ClearPath ASG signals when homing is complete
      * Homing Occurs: Upon every Enable
      * Homing Direction: CCW (Counter-clockwise)
      * Homing Torque Limit: 40%
      * Speed (RPM): 40.00
      * Accel/Decel (RPM/s): 5,000
      * Precision Homing: Use Precision Homing (enabled)
      * Physical Home Clearance: 200 cnts
      * Home Offset Move Distance: 0 cnts

    MOTOR SETUP PROCEDURE:
    1. Configure all settings above using Teknic ClearPath-MSP software
    2. Load configuration to motor (download settings to motor memory)
    3. Perform auto-tuning with realistic load conditions:
       * Lock a tray to the shuttle to simulate actual operating load
       * Set auto-tune torque limit to 50%
       * Set auto-tune RPM to 325 RPM (matches operational velocity with tray)
       * Run auto-tune sequence to optimize motor performance
    4. Verify homing operation and position accuracy after installation
    5. Calibrate precision homing if home reference moves or mechanics change

    NOTE: Motor must be configured and auto-tuned before use. Auto-tuning with
    actual load is critical for optimal performance and accuracy. Precision homing
    provides excellent repeatability when properly calibrated.

 3. FEEDBACK & CONTROL:
    - CL-ENCDR-DFIN Encoder Input Adapter (for MPG handwheel manual control)
    - MPG handwheel encoder (manual positioning interface with immediate response)
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

OPERATIONAL MODES:
The system operates in two primary modes:

A. AUTOMATED MODE (Default):
   - System executes pre-programmed tray handling sequences
   - Automatic tray detection, transport, and positioning
   - Pneumatic shuttle automatically grabs/releases trays during transport
   - Position locks engage/disengage automatically based on sequence
   - Full safety monitoring and fault detection active

B. MANUAL MODE:
   - Direct command control for setup and maintenance
   - Manual positioning via MPG handwheel encoder
   - Individual control of all pneumatic systems
   - Step-by-step operation for troubleshooting

BASIC SETUP AND INITIALIZATION:
 1. Configure ClearPath motor using Teknic ClearPath-MSP software (see MOTION SYSTEM requirements)
 2. Connect all hardware components per system documentation
 3. Power up system and connect via Serial or Ethernet
 4. Initialize motor system: "motor,init"
 5. Home the system: "motor,home"
 6. Verify all sensors and pneumatics: "system,status"

POSITION TEACHING AND CONFIGURATION:
 7. Position motor to desired location using your preferred method:
    - HANDWHEEL (recommended): "encoder,enable" for precise manual positioning
    - MOVE COMMANDS: "move,<mm>" to position directly by millimeter coordinate
    - JOG COMMANDS: Use incremental positioning commands for fine adjustment
 8. Teach positions: "teach,1", "teach,2", "teach,3" (auto-saves to SD card)
 9. Verify taught positions: "teach,status" shows current configuration
 10. Test movements: "move,1", "move,2", "move,3" to verify accuracy
 11. Reset if needed: "teach,reset" returns to factory defaults

MANUAL OPERATION COMMANDS:
 13. Position control: "move,1" (positions 1-3) or "move,<mm>" for precise positioning
 14. Individual tray locking: "lock,1", "lock,2", "lock,3" for position locks
 15. Shuttle control: "lock,shuttle" to grab trays, "unlock,shuttle" to release
 16. Manual positioning: "encoder,enable" for handwheel control, "encoder,disable" to return to automated

AUTOMATED OPERATION COMMANDS:
 17. Automated sequences: "tray,load" (full loading sequence), "tray,unload" (full unloading sequence)
 18. System monitoring: "system,status", "motor,status" for real-time status

DIAGNOSTICS AND TROUBLESHOOTING:
 19. Help system: "help" for command list, "help,<command>" for specific usage
 20. Error diagnosis: Use log commands for operation history and troubleshooting:
    - "log,history" - View complete operation log with timestamps
    - "log,errors" - View only error entries for quick diagnosis
    - "log,last,N" - View last N log entries (e.g., "log,last,10")
    - "log,stats" - View log statistics and system health overview
 21. System reset: "system,reset" to clear fault conditions
 22. Network management: "network,status" for Ethernet connection info
 23. Position diagnostics: "teach,status" to verify position configuration

POSITION TEACHING SYSTEM:
The system supports dynamic position teaching for field adjustment without code changes:

- FACTORY DEFAULTS: Hardcoded positions used when no taught positions exist
  * Position 1: 36.57 mm (loading position)
  * Position 2: 477.79 mm (middle position)
  * Position 3: 919.75 mm (unloading position)

- TAUGHT POSITIONS: User-defined positions that override factory defaults
  * Captured from current motor position using "teach,1/2/3" commands
  * Automatically saved to SD card (positions.txt) for persistence
  * Loaded automatically at startup if SD card is present
  * Survive firmware updates and power cycles

- POSITION HIERARCHY: System uses positions in this priority order:
  1. Taught positions (highest priority) - if available
  2. SD card positions (medium priority) - loaded at startup
  3. Factory defaults (lowest priority) - fallback values

- TEACHING WORKFLOW:
  1. "motor,init" and "motor,home" - Initialize and reference system
  2. Position motor to desired location using ONE of these methods:
     a) HANDWHEEL METHOD (most precise): "encoder,enable" then use handwheel
     b) MOVE COMMANDS: "move,<mm>" for direct positioning to millimeter coordinates
     c) JOG COMMANDS: Use incremental jog commands for fine adjustment
  3. "teach,1" (or 2,3) - Capture current position and auto-save
  4. "move,1" - Test the newly taught position
  5. Repeat for other positions as needed

- CONFIGURATION COMMANDS:
  * "teach,status" - Show current active positions and SD card status
  * "teach,reset" - Clear taught positions and return to factory defaults
  * "teach,help" - Display comprehensive teaching system documentation

TYPICAL WORKFLOWS:

INITIAL SYSTEM SETUP:
- Power up → Connect (Serial/Ethernet) → motor,init → motor,home → system,status

POSITION TEACHING (choose one positioning method):
- Method A (with handwheel): motor,home → encoder,enable → position manually → teach,X → move,X (test)
- Method B (direct positioning): motor,home → move,mm,XXX → teach,X → move,X (test)
- Method C (incremental): motor,home → jog,inc,X → jog,+/- → teach,X → move,X (test)

DAILY OPERATION:
- Startup → Load taught positions from SD card → Verify system,status → Begin operations

MAINTENANCE/TROUBLESHOOTING:
- Enable manual mode → Use individual commands → Return to automated mode

AUTOMATED SEQUENCES:
- tray,load (complete loading sequence)
- tray,unload (complete unloading sequence)

NOTE: Taught positions provide the flexibility to adjust the system for different
tray sizes, mechanical tolerances, or field conditions without requiring code
modifications or reflashing firmware.
*/

#include <Arduino.h>
#include "ClearCore.h"
#include "ValveController.h"
#include "MotorController.h"
#include "Logging.h"
#include "CommandController.h"
#include "Commands.h"
#include "Utils.h"
#include "EncoderController.h"
#include "OutputManager.h"
#include "EthernetController.h"
#include "PositionConfig.h"

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

    // Initialize position config
    initPositionConfig();

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

    Console.serialInfo(F("System ready."));
    Console.serialInfo(F("Type 'help' for available commands"));
}

// The main loop
void loop()
{
    unsigned long currentTime = millis();

    // Check for E-stop condition (highest priority)
    handleEStop();

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
        sprintf(errorMsg, "SAFETY VIOLATION: %s", safety.operationSequenceMessage);
        Console.error(errorMsg);

        // Emergency stop or other recovery action
        // Use the failureReason from safety validation
        abortOperation(safety.failureReason);
    }

    // Process tray operations if any are in progress
    processTrayOperations();

    // Store current state as previous for next cycle
    previousState = currentState;

    // Process encoder input if enabled (Teknic approach - always process when active)
    processEncoderInput();

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
