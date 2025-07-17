#include <Arduino.h>
#include "ClearCore.h"
#include "Utils.h"
#include "MotorController.h"
#include "OutputManager.h"
#include "EncoderController.h"
#include "PositionConfig.h"
#include "Sensors.h"
#include "ValveController.h"
#include "EthernetController.h"
#include "CommandController.h"
#include "Commands.h"
#include "Logging.h"
#include "HandoffController.h"
#include "LabwareAutomation.h"

// Specify which ClearCore serial COM port is connected to the CCIO-8 board
#define CcioPort ConnectorCOM0

uint8_t ccioBoardCount; // Store the number of connected CCIO-8 boards here
uint8_t ccioPinCount;   // Store the number of connected CCIO-8 pins here

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Initialize system start time for uptime tracking
    initializeSystemStartTime();

    // Core system initialization
    initMotorManager();
    initPositionConfig();
    initOutputManager();

    // Manual control interface
    Console.serialInfo(F("Initializing handwheel interface..."));
    initEncoderControl(true, false);

    // First set up the CCIO board
    Console.serialInfo(F("Initializing CCIO-8 expansion boards..."));
    CcioPort.Mode(Connector::CCIO);
    CcioPort.PortOpen();


    // CCIO board detection
    ccioBoardCount = CcioMgr.CcioCount();
    bool hasCCIOBoard = (ccioBoardCount > 0);
    
    char msg[SMALL_MSG_SIZE];
    sprintf_P(msg, PSTR("CCIO boards detected: %d"), ccioBoardCount);
    Console.serialInfo(msg);
    
    if (!hasCCIOBoard) {
        Console.serialWarning(F("CCIO board not detected - some sensors will be unavailable"));
    }

    // System hardware initialization
    Console.serialInfo(F("Initializing sensors..."));
    initSensorSystem(hasCCIOBoard);

    Console.serialInfo(F("Initializing pneumatics..."));
    initValveSystem(hasCCIOBoard);

    // Configuration validation
    Console.serialInfo(F("Validating system configuration..."));
    if (!validateAllPredefinedPositions()) {
        Console.serialError(F("CRITICAL: Position validation failed"));
        Console.serialError(F("System will continue but movements may be restricted"));
    }

    // Network and automation
    Console.serialInfo(F("Initializing network interface..."));
    initEthernetController(false); // static IP

    Console.serialInfo(F("Initializing automation system..."));
    initLabwareSystem();

    // Command interface
    commander.attachTree(API_tree);
    commander.init();

    Console.serialInfo(F("System ready - Type 'help' for commands"));
}

void loop()
{
    unsigned long currentTime = millis();
    
    // E-stop monitoring (highest priority)
    handleEStop();

    // Handle commands
    handleSerialCommands();
    handleEthernetCommands();

    // Motor operations
    checkAllHomingProgress();
    checkMoveProgress();

    // System monitoring
    if (ccioBoardCount > 0) {
        updateAllSensors();
    }
    updateLabwareSystemState();

    // Manual control
    processEncoderInput();
    
    // Handoff operations
    if (isHandoffInProgress()) {
        updateHandoff();
    }
    
    // Network management
    processEthernetConnections();
    testConnections();
    
    // Periodic monitoring
    static unsigned long lastPressureCheck = 0;
    if (waitTimeReached(currentTime, lastPressureCheck, PRESSURE_MONITORING_INTERVAL_MS)) {
        if (!isPressureSufficient()) {
            Console.serialWarning(F("System pressure below minimum threshold"));
        }
        lastPressureCheck = currentTime;
    }
    
    // Periodic logging
    if (logging.logInterval > 0 && waitTimeReached(currentTime, logging.previousLogTime, logging.logInterval))
    {
        logging.previousLogTime = currentTime;
        logSystemState();
    }
}
