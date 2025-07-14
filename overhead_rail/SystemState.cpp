#include "SystemState.h"
#include "CommandController.h"
#include "EncoderController.h"
#include "MotorController.h"
#include "ValveController.h"
#include "Sensors.h"
#include "EthernetController.h"
#include "LabwareAutomation.h"
#include "Utils.h"

// Global system state data
struct SystemStateStruct {
    bool emergencyStopActivated;
    bool atPositionLimit;
};

SystemStateStruct SystemStateData = {
    false,  // emergencyStopActivated
    false   // atPositionLimit
};

//=============================================================================
// MAIN SYSTEM STATE FUNCTION - Uses existing print functions
//=============================================================================
void printSystemState()
{
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("COMPREHENSIVE SYSTEM STATE REPORT"));
    Console.serialDiagnostic(F("================================================================================"));
    
    // Motor system state - use existing functions from MotorController.cpp
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("MOTOR STATUS:"));
    Console.serialDiagnostic(F("-------------"));
    printAllMotorStatus();  // This calls printMotorStatus for both rails
    
    // Sensor states - use existing comprehensive function from Sensors.cpp
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("SENSOR STATUS:"));
    Console.serialDiagnostic(F("--------------"));
    printAllSensorStatus();
    
    // Pneumatic system - use existing functions from ValveController.cpp and Sensors.cpp
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("PNEUMATIC SYSTEMS:"));
    Console.serialDiagnostic(F("------------------"));
    printPressureStatus();
    printValveStatus();
    
    // Labware system - use existing function from LabwareAutomation.cpp
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("LABWARE DETECTION:"));
    Console.serialDiagnostic(F("------------------"));
    printLabwareSystemStatus();
    
    // Network status - use existing function from EthernetController.cpp
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("NETWORK STATUS:"));
    Console.serialDiagnostic(F("---------------"));
    printEthernetStatus();
    
    // Manual controls - use existing function from EncoderController.cpp
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("MANUAL CONTROLS (MPG):"));
    Console.serialDiagnostic(F("----------------------"));
    printEncoderStatus();
    
    // Safety systems and custom state
    printSafetySystemState();
    
    // System activity and readiness summary
    printSystemReadinessState();
    
    Console.serialDiagnostic(F("================================================================================"));
    Console.serialDiagnostic(F(""));
}

//=============================================================================
// SAFETY SYSTEMS STATE (Custom - not available elsewhere)
//=============================================================================
void printSafetySystemState()
{
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("SAFETY SYSTEMS:"));
    Console.serialDiagnostic(F("---------------"));
    
    // Emergency stop status
    char msg[100];
    sprintf_P(msg, PSTR("  Emergency Stop: %s"), 
              SystemStateData.emergencyStopActivated ? "ACTIVATED ⚠️" : "Normal ✓");
    Console.serialDiagnostic(msg);
    
    // System limits
    sprintf_P(msg, PSTR("  Position Limits: %s"), 
              SystemStateData.atPositionLimit ? "AT-LIMIT ⚠️" : "OK ✓");
    Console.serialDiagnostic(msg);
}

//=============================================================================
// SYSTEM READINESS AND ACTIVITY SUMMARY
//=============================================================================
void printSystemReadinessState()
{
    Console.serialDiagnostic(F(""));
    Console.serialDiagnostic(F("SYSTEM ACTIVITY & READINESS:"));
    Console.serialDiagnostic(F("----------------------------"));
    
    char msg[150];
    
    // System timing
    unsigned long uptime = millis() - systemStartTime;
    sprintf_P(msg, PSTR("  System Uptime: %lu seconds"), uptime / 1000);
    Console.serialDiagnostic(msg);
    
    // Command activity from global variables
    if (lastExecutedCommand[0] != '\0') {
        sprintf_P(msg, PSTR("  Last Command: %s"), lastExecutedCommand);
        Console.serialDiagnostic(msg);
        
        unsigned long timeSinceCommand = millis() - lastCommandTime;
        sprintf_P(msg, PSTR("  Time Since Command: %lu seconds"), timeSinceCommand / 1000);
        Console.serialDiagnostic(msg);
        
        sprintf_P(msg, PSTR("  Command Result: %s"), lastCommandSuccess ? "SUCCESS ✓" : "FAILED ❌");
        Console.serialDiagnostic(msg);
    } else {
        Console.serialDiagnostic(F("  Last Command: None"));
    }
    
    // Overall system readiness assessment
    bool systemReady = isSystemReadyForAutomation();
    Console.serialDiagnostic(F(""));
    sprintf_P(msg, PSTR("  OVERALL SYSTEM STATUS: %s"), 
              systemReady ? "READY FOR AUTOMATION ✓" : "NOT READY ❌");
    Console.serialDiagnostic(msg);
    
    if (!systemReady) {
        sprintf_P(msg, PSTR("  Error Summary: %s"), getSystemErrorSummary());
        Console.serialDiagnostic(msg);
    }
}

//=============================================================================
// LEGACY COMPATIBILITY FUNCTIONS (for header interface)
//=============================================================================
void printMotorSystemState()
{
    Console.serialDiagnostic(F("Motor Status:"));
    printAllMotorStatus();  // Use existing comprehensive function
}

void printSensorSystemState()
{
    Console.serialDiagnostic(F("Sensor Status:"));
    printAllSensorStatus();  // Use existing comprehensive function
}

void printValveSystemState()
{
    Console.serialDiagnostic(F("Pneumatic Systems:"));
    printPressureStatus();   // Use existing function
    printValveStatus();      // Use existing function
}

void printNetworkSystemState()
{
    Console.serialDiagnostic(F("Network Status:"));
    printEthernetStatus();   // Use existing function
}

void printEncoderSystemState()
{
    Console.serialDiagnostic(F("Manual Controls:"));
    printEncoderStatus();    // Use existing function
}

void printLabwareSystemState()
{
    Console.serialDiagnostic(F("Labware Detection:"));
    printLabwareSystemStatus();  // Use existing function
}

//=============================================================================
// SYSTEM READINESS AND UTILITY FUNCTIONS
//=============================================================================
bool isSystemReadyForAutomation()
{
    // Check emergency stop
    if (SystemStateData.emergencyStopActivated) {
        return false;
    }
    
    // Check position limits
    if (SystemStateData.atPositionLimit) {
        return false;
    }
    
    // Check motor readiness for both rails
    for (uint8_t railId = 1; railId <= 2; railId++) {
        if (!isHomingComplete(railId)) {
            return false;
        }
        if (hasMotorFault(railId)) {
            return false;
        }
    }
    
    // Check pressure system
    if (!isPressureSufficient()) {
        return false;
    }
    
    // Check for cylinder sensor errors (both sensors triggered is an error state)
    if (isCylinderActuallyRetracted() && isCylinderActuallyExtended()) {
        return false;
    }
    
    return true;
}

bool hasSystemErrors()
{
    return !isSystemReadyForAutomation();
}

const char* getSystemErrorSummary()
{
    static String errorMsg = "";
    errorMsg = "";
    
    if (SystemStateData.emergencyStopActivated) {
        errorMsg += "EMERGENCY_STOP ";
    }
    
    if (SystemStateData.atPositionLimit) {
        errorMsg += "POSITION_LIMIT ";
    }
    
    // Check motors
    for (uint8_t railId = 1; railId <= 2; railId++) {
        if (!isHomingComplete(railId)) {
            errorMsg += "RAIL" + String(railId) + "_NOT_HOMED ";
        }
        if (hasMotorFault(railId)) {
            errorMsg += "RAIL" + String(railId) + "_MOTOR_FAULT ";
        }
    }
    
    // Check pressure
    if (!isPressureSufficient()) {
        errorMsg += "LOW_PRESSURE ";
    }
    
    // Check cylinder sensors
    if (isCylinderActuallyRetracted() && isCylinderActuallyExtended()) {
        errorMsg += "CYLINDER_SENSOR_ERROR ";
    }
    
    if (errorMsg.length() == 0) {
        errorMsg = "NONE";
    } else {
        errorMsg.trim();  // Remove trailing space
    }
    
    return errorMsg.c_str();
}

//=============================================================================
// STATE MANAGEMENT FUNCTIONS  
//=============================================================================
void setEmergencyStop(bool activated)
{
    SystemStateData.emergencyStopActivated = activated;
}

bool getEmergencyStopStatus()
{
    return SystemStateData.emergencyStopActivated;
}

void setPositionLimit(bool atLimit)
{
    SystemStateData.atPositionLimit = atLimit;
}

bool getPositionLimitStatus()
{
    return SystemStateData.atPositionLimit;
}
