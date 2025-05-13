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

// Specify which ClearCore serial COM port is connected to the CCIO-8 board
#define CcioPort ConnectorCOM0

uint8_t ccioBoardCount; // Store the number of connected CCIO-8 boards here
uint8_t ccioPinCount;   // Store the number of connected CCIO-8 pins here

// For formatting text printed to serial port
#define MAX_MSG_LEN 80
char msg[MAX_MSG_LEN + 1];


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

    // Handle incoming serial commands using Commander API
    handleSerialCommands();
    
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
}


