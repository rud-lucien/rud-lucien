/*
 Name:		lynx_linear_actuator.ino
 Created:	4/14/2025 12:00:17 PM
 Author:	rlucien
*/

// #include "Arduino.h"
#include "ClearCore.h"
#include "ValveController.h"
#include "MotorController.h"
#include "Logging.h"

// Specify which ClearCore serial COM port is connected to the CCIO-8 board
#define CcioPort ConnectorCOM0

uint8_t ccioBoardCount; // Store the number of connected CCIO-8 boards here
uint8_t ccioPinCount;   // Store the number of connected CCIO-8 pins here

// For formatting text printed to serial port
#define MAX_MSG_LEN 80
char msg[MAX_MSG_LEN + 1];

// Set to true to use CCIO-8 connectors as digital inputs
const bool inputMode = false;

// The setup function
void setup()
{
    Serial.begin(115200);

    // Set up and detect CCIO-8 boards
    CcioPort.Mode(Connector::CCIO);
    CcioPort.PortOpen();
    // Get count of CCIO-8 boards
    ccioBoardCount = CcioMgr.CcioCount();
    ccioPinCount = 0;

    // Count available pins by checking each board
    for (int i = 0; i < ccioBoardCount; i++)
    {
        // Count 8 pins per board
        ccioPinCount += 8;
    }
    // Get count of CCIO-8 boards
    ccioBoardCount = CcioMgr.CcioCount();
    ccioPinCount = ccioBoardCount * 8; // Each CCIO-8 board has 8 pins

    snprintf(msg, MAX_MSG_LEN, "Discovered %d CCIO-8 board", ccioBoardCount);
    Serial.print(msg);
    if (ccioBoardCount != 1)
    {
        Serial.print("s");
    }
    Serial.println("...");

    // Initialize valve controller
    Serial.println("Initializing valve controller...");
    initValveSystem();
    initValveWithCCIO(ccioBoardCount > 0);

    Serial.println("5/2-way valve controller initialized");

    // Don't initialize the motor automatically
    Serial.println("Motor controller ready for initialization.");
    Serial.println("Use 'motor init' command to initialize the motor.");

    Serial.println("Type 'help' for available commands");
}

// The main loop
void loop()
{
    unsigned long currentTime = millis();
    
    // Check for E-stop condition (highest priority)
    handleEStop();
    
    // Check and update motor state
    updateMotorState();
    
    // Check homing progress if in progress
    if (motorState == MOTOR_STATE_HOMING) {
        checkHomingProgress();
    }
    // Check movement progress if moving
    else if (motorState == MOTOR_STATE_MOVING) {
        checkMoveProgress();
    }
    
    // Check if there's data available to read from Serial
    if (Serial.available() > 0)
    {
        // Read the incoming command
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove any whitespace

        // Convert to lowercase for case-insensitive comparison
        command.toLowerCase();

        // Process the command
        if (command == "tray 1 l" || command == "t1l")
        {
            Serial.println("Command received: Locking tray 1");
            DoubleSolenoidValve *valve = getValveByIndex(0);
            if (valve)
                lockValve(*valve);
        }
        else if (command == "tray 1 ul" || command == "t1ul")
        {
            Serial.println("Command received: Unlocking tray 1");
            DoubleSolenoidValve *valve = getValveByIndex(0);
            if (valve)
                unlockValve(*valve);
        }
        else if (command == "tray 2 l" || command == "t2l")
        {
            Serial.println("Command received: Locking tray 2");
            DoubleSolenoidValve *valve = getValveByIndex(1);
            if (valve)
                lockValve(*valve);
        }
        else if (command == "tray 2 ul" || command == "t2ul")
        {
            Serial.println("Command received: Unlocking tray 2");
            DoubleSolenoidValve *valve = getValveByIndex(1);
            if (valve)
                unlockValve(*valve);
        }
        else if (command == "tray 3 l" || command == "t3l")
        {
            Serial.println("Command received: Locking tray 3");
            DoubleSolenoidValve *valve = getValveByIndex(2);
            if (valve)
                lockValve(*valve);
        }
        else if (command == "tray 3 ul" || command == "t3ul")
        {
            Serial.println("Command received: Unlocking tray 3");
            DoubleSolenoidValve *valve = getValveByIndex(2);
            if (valve)
                unlockValve(*valve);
        }
        else if (command == "shuttle l" || command == "sl")
        {
            if (ccioBoardCount > 0)
            {
                Serial.println("Command received: Locking shuttle");
                DoubleSolenoidValve *valve = getValveByIndex(3);
                if (valve)
                    lockValve(*valve);
            }
            else
            {
                Serial.println("ERROR: No CCIO-8 board detected. Shuttle valve not available.");
            }
        }
        else if (command == "shuttle ul" || command == "sul")
        {
            if (ccioBoardCount > 0)
            {
                Serial.println("Command received: Unlocking shuttle");
                DoubleSolenoidValve *valve = getValveByIndex(3);
                if (valve)
                    unlockValve(*valve);
            }
            else
            {
                Serial.println("ERROR: No CCIO-8 board detected. Shuttle valve not available.");
            }
        }
        else if (command == "unlock all" || command == "ua")
        {
            Serial.println("Command received: Unlocking all valves");
            unlockAllValves();
        }
        else if (command == "lock all" || command == "la")
        {
            Serial.println("Command received: Locking all valves");
            lockAllValves();
        }
        else if (command == "status" || command == "s")
        {
            // Report the current valve and sensor status
            printAllValveStatus();
            Serial.println();
            printAllSensorStatus();
        }
        else if (command == "sensors" || command == "ss")
        {
            // Report just the sensor status
            printAllSensorStatus();
        }
        else if (command == "motor init" || command == "mi") {
            Serial.println("Initializing motor...");
            initMotorSystem();
        }
        else if (command == "home" || command == "h") {
            Serial.println("Command received: Homing motor");
            if (startHoming()) {
                Serial.println("Homing sequence started. Motor moving to home position...");
            } else {
                Serial.println("Error: Could not start homing sequence.");
            }
        }
        else if (command == "move 1" || command == "m1") {
            Serial.println("Command received: Moving to position 1");
            moveToPosition(1);
        }
        else if (command == "move 2" || command == "m2") {
            Serial.println("Command received: Moving to position 2");
            moveToPosition(2);
        }
        else if (command == "move 3" || command == "m3") {
            Serial.println("Command received: Moving to position 3");
            moveToPosition(3);
        }
        else if (command == "move 4" || command == "m4") {
            Serial.println("Command received: Moving to position 4");
            moveToPosition(4);
        }
        else if (command == "move 5" || command == "m5") {
            Serial.println("Command received: Moving to position 5");
            moveToPosition(5);
        }
        else if (command.startsWith("move abs ")) {
            // Extract the position from the command (e.g., "move abs 250")
            int32_t targetPos = command.substring(9).toInt();
            Serial.print("Command received: Moving to absolute position ");
            Serial.println(targetPos);
            moveToAbsolutePosition(targetPos);
        }
        else if (command.startsWith("move mm ")) {
            // Extract the position from the command (e.g., "move mm 250.5")
            double targetMm = command.substring(8).toDouble();
            Serial.print("Command received: Moving to ");
            Serial.print(targetMm);
            Serial.println(" mm");
            
            moveToPositionMm(targetMm);
        }
        else if (command.startsWith("move rel ")) {
            // Extract the relative distance from the command (e.g., "move rel 50.5")
            double relativeMm = command.substring(9).toDouble();
            Serial.print("Command received: Moving relative ");
            Serial.print(relativeMm);
            Serial.println(" mm");
            
            moveRelative(relativeMm);
        }
        else if (command == "stop" || command == "e") {
            Serial.println("Command received: Emergency stop");
            stopMotion();
        }
        else if (command == "abort" || command == "a") {
            Serial.println("Command received: Abort operation");
            if (homingInProgress) {
                abortHoming();
            } else if (motorState == MOTOR_STATE_MOVING) {
                stopMotion();
                motorState = MOTOR_STATE_IDLE;
            } else {
                Serial.println("No active operation to abort");
            }
        }
        else if (command == "motor status" || command == "ms") {
            printMotorStatus();
        }
        else if (command == "clear fault" || command == "cf") {
            Serial.println("Command received: Clearing motor fault");
            clearMotorFaults();
        }
        else if (command == "test range" || command == "tr") {
            Serial.println("Command received: Testing motor range");
            testMotorRange();
        }
        else if (command.startsWith("set rpm ")) {
            // Extract the RPM from the command (e.g., "set rpm 75")
            double targetRpm = command.substring(8).toDouble();
            if (targetRpm > 0 && targetRpm <= 200) {
                Serial.print("Setting velocity limit to ");
                Serial.print(targetRpm);
                Serial.println(" RPM");
                currentVelMax = rpmToPps(targetRpm);
                MOTOR_CONNECTOR.VelMax(currentVelMax);
            } else {
                Serial.println("Invalid RPM value. Please use a value between 1 and 200 RPM.");
            }
        }
        else if (command == "check home sensor" || command == "chs") {
            Serial.println("Checking home sensor...");
            checkHomeSensor();
        }
        else if (command.startsWith("log on")) {
            if (command.length() > 6) {
                // Extract optional interval value (e.g., "log on 250")
                unsigned long interval = command.substring(7).toInt();
                if (interval > 0) {
                    logging.logInterval = interval;
                    Serial.print("Logging enabled with interval of ");
                    Serial.print(interval);
                    Serial.println(" ms");
                } else {
                    Serial.println("Invalid interval. Using default.");
                    logging.logInterval = DEFAULT_LOG_INTERVAL;
                }
            } else {
                // Use default interval
                logging.logInterval = DEFAULT_LOG_INTERVAL;
                Serial.print("Logging enabled with default interval of ");
                Serial.print(DEFAULT_LOG_INTERVAL);
                Serial.println(" ms");
            }
            logging.previousLogTime = millis(); // Reset the timer
        }
        else if (command == "log off") {
            Serial.println("Logging disabled");
            logging.logInterval = 0; // Setting to 0 disables logging
        }
        else if (command == "log now") {
            Serial.println("Logging system state now");
            // Log immediately regardless of interval
            logSystemState();
        }
        else if (command == "help" || command == "?") {
            // Help command
            Serial.println("Available commands:");
            Serial.println("  tray 1 l or t1l - Lock tray 1");
            Serial.println("  tray 1 ul or t1ul - Unlock tray 1");
            Serial.println("  tray 2 l or t2l - Lock tray 2");
            Serial.println("  tray 2 ul or t2ul - Unlock tray 2");
            Serial.println("  tray 3 l or t3l - Lock tray 3");
            Serial.println("  tray 3 ul or t3ul - Unlock tray 3");

            if (ccioBoardCount > 0)
            {
                Serial.println("  shuttle l or sl - Lock shuttle");
                Serial.println("  shuttle ul or sul - Unlock shuttle");
            }

            Serial.println("  unlock all or ua - Unlock all valves");
            Serial.println("  lock all or la - Lock all valves");
            Serial.println("  status or s - Show status of all valves and sensors");
            Serial.println("  sensors or ss - Show all sensor readings");
            Serial.println("  check home sensor or chs - Check home sensor");
            Serial.println("  log on [interval] - Enable logging with optional interval in ms");
            Serial.println("  log off - Disable logging");
            Serial.println("  log now - Log system state immediately");
            Serial.println("  log on - Turn on logging with default interval (500ms)");
            Serial.println("  log on X - Turn on logging with interval of X milliseconds");
            Serial.println("  log off - Turn off logging");
            Serial.println("  log now - Log system state once immediately");
            Serial.println("  help or h - Show this help message");
            
            // Motor commands
            Serial.println("\nMotor commands:");
            Serial.println("  motor init or mi - Initialize the motor system");
            Serial.println("  home or h - Home the motor (find zero position)");
            Serial.println("  move 1 or m1 - Move to position 1 (Home)");
            Serial.println("  move 2 or m2 - Move to position 2 (250mm)");
            Serial.println("  move 3 or m3 - Move to position 3 (500mm)");
            Serial.println("  move 4 or m4 - Move to position 4 (750mm)");
            Serial.println("  move 5 or m5 - Move to position 5 (1000mm)");
            Serial.println("  move abs X - Move to absolute position X (in pulses)");
            Serial.println("  move mm X - Move to position X millimeters");
            Serial.println("  move rel X - Move X mm relative to current position");
            Serial.println("  set rpm X - Set velocity to X RPM (1-200)");
            Serial.println("  stop or e - Emergency stop motor movement");
            Serial.println("  abort or a - Abort current operation");
            Serial.println("  motor status or ms - Show motor status");
            Serial.println("  test range or tr - Test motor with small incremental moves");
            Serial.println("  clear fault or cf - Clear motor fault");
            Serial.println("  abort or a - Abort current operation (homing or movement)");
        }
        else {
            Serial.println("Unknown command. Type 'help' for available commands.");
        }
    }

    // Log system state periodically if logging is enabled
    if (logging.logInterval > 0 && currentTime - logging.previousLogTime >= logging.logInterval)
    {
        logging.previousLogTime = currentTime;
        logSystemState();
    }
}
