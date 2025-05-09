
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
#include "Tests.h"
#include "Commands.h"

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

    // --- Setup Commander API ---
    commander.attachTree(API_tree);
    commander.init();

    Serial.println(F("[MESSAGE] System ready."));

    Serial.println("Type 'help' for available commands");
}

// The main loop
void loop()
{
    unsigned long currentTime = millis();

    // Check for E-stop condition (highest priority)
    handleEStop();

    // Handle incoming serial commands using Commander API
    handleSerialCommands();

    // Check and update motor state
    updateMotorState();

    // Process enable cycling for homing if in progress
    cycleMotorEnableForHoming();

    // Check homing progress if in progress
    if (motorState == MOTOR_STATE_HOMING)
    {
        executeHomingSequence();
    }
    // Check movement progress if moving
    else if (motorState == MOTOR_STATE_MOVING)
    {
        checkMoveProgress();
    }

    // Log system state periodically if logging is enabled
    if (logging.logInterval > 0 && currentTime - logging.previousLogTime >= logging.logInterval)
    {
        logging.previousLogTime = currentTime;
        logSystemState();
    }
}
