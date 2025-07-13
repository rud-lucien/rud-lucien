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

    // Initialize the motor manager (one-time system configuration)
    initMotorManager();

    // Initialize SD card and load taught positions
    initPositionConfig();

    // Initialize the output manager (must be before any Console calls)
    initOutputManager();

    // Initialize encoder with default direction (modify if needed)
    Console.serialInfo(F("Initializing MPG handwheel interface..."));
    initEncoderControl(true, false);

    // Check for CCIO-8 boards before initializing sensors
    ccioBoardCount = CcioMgr.CcioCount();
    bool hasCCIOBoard = (ccioBoardCount > 0);
    
    char msg[SMALL_MSG_SIZE];
    sprintf_P(msg, PSTR("Discovered CCIO boards: %d"), ccioBoardCount);
    Console.serialInfo(msg);
    
    if (hasCCIOBoard) {
        Console.serialInfo(F("CCIO board detected - enabling CCIO-dependent sensors"));
    } else {
        Console.serialWarning(F("No CCIO board detected - CCIO sensors will be disabled"));
    }

    // Initialize the sensor system with CCIO board status
    Console.serialInfo(F("Initializing sensor system..."));
    initSensorSystem(hasCCIOBoard);

    // Initialize the valve system with CCIO board status
    Console.serialInfo(F("Initializing pneumatic valve system..."));
    initValveSystem(hasCCIOBoard);

    // Validate all predefined positions against travel limits
    Console.serialInfo(F("Performing system configuration validation..."));
    if (!validateAllPredefinedPositions()) {
        Console.serialError(F("CRITICAL: Position validation failed - review configuration"));
        Console.serialError(F("System will continue but some movements may be rejected"));
    }

    // Initialize Ethernet interface
    Console.serialInfo(F("Initializing Ethernet interface..."));
    initEthernetController(false); // false = use static IP, true = use DHCP

    // Initialize labware automation system
    Console.serialInfo(F("Initializing labware automation system..."));
    initLabwareSystem();

    commander.attachTree(API_tree);
    commander.init();

    Console.serialInfo(F("System ready."));
    Console.serialInfo(F("Type 'help' for available commands"));
}

void loop()
{
    unsigned long currentTime = millis();
    
    // Handle E-stop monitoring (highest priority)
    handleEStop();

    // Handle incoming commands from serial and network
    handleSerialCommands();
    handleEthernetCommands();

    // Check homing progress for both motors
    checkAllHomingProgress();

    // Process fault clearing for both motors
    processAllFaultClearing();
    
    // Monitor movement progress and completion
    // This is critical for detecting when async movements finish
    // and clearing operationInProgress flag
    checkMoveProgress();

    // Update all sensors and check for alerts
    updateAllSensors();

    // Update labware automation system state
    updateLabwareSystemState();

    // Process encoder input for manual control
    processEncoderInput();
    
    // Update handoff controller (non-blocking state machine)
    if (isHandoffInProgress()) {
        updateHandoff();
    }
    
    // Process Ethernet connections and communication
    processEthernetConnections();
    
    // Test connection health periodically
    testConnections();
    
    // Periodic pressure monitoring (every 10 seconds)
    // Important for pneumatic system health and early warnings
    static unsigned long lastPressureCheck = 0;
    if (waitTimeReached(currentTime, lastPressureCheck, 10000)) {
        if (!isPressureSufficient()) {
            Console.serialWarning(F("System pressure below minimum threshold"));
        }
        if (isPressureWarningLevel()) {
            Console.serialWarning(F("System pressure approaching minimum threshold"));
        }
        lastPressureCheck = currentTime;
    }
    
    // Periodic client timeout management (every 30 seconds)
    // Clean up inactive connections to free resources
    static unsigned long lastClientCheck = 0;
    if (waitTimeReached(currentTime, lastClientCheck, 30000)) {
        // Check for and disconnect inactive clients
        // This function would need to be implemented in EthernetController
        // cleanupInactiveClients();
        lastClientCheck = currentTime;
    }
    
    // Log system state periodically if logging is enabled
    if (logging.logInterval > 0 && waitTimeReached(currentTime, logging.previousLogTime, logging.logInterval))
    {
        logging.previousLogTime = currentTime;
        logSystemState();
    }
}
