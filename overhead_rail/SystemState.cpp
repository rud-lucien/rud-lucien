#include "SystemState.h"
#include "CommandController.h"
#include "EncoderController.h"
#include "MotorController.h"
#include "ValveController.h"
#include "Sensors.h"
#include "EthernetController.h"
#include "HandoffController.h"
#include "LabwareAutomation.h"
#include "RailAutomation.h"
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
    printValveDetailedStatus();  // Use detailed version for comprehensive system state
    
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
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, PSTR("  Emergency Stop: %s"), 
              SystemStateData.emergencyStopActivated ? "ACTIVATED" : "Normal");
    Console.serialDiagnostic(msg);
    
    // System limits
    sprintf_P(msg, PSTR("  Position Limits: %s"), 
              SystemStateData.atPositionLimit ? "AT-LIMIT" : "OK");
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
    
    char msg[MEDIUM_MSG_SIZE];
    
    // System timing
    unsigned long uptime = millis() - systemStartTime;
    sprintf_P(msg, PSTR("  System Uptime: %lu seconds"), uptime / MILLISECONDS_PER_SECOND);
    Console.serialDiagnostic(msg);
    
    // Command activity from global variables
    if (lastExecutedCommand[0] != STRING_TERMINATOR) {
        sprintf_P(msg, PSTR("  Last Command: %s"), lastExecutedCommand);
        Console.serialDiagnostic(msg);
        
        unsigned long timeSinceCommand = millis() - lastCommandTime;
        sprintf_P(msg, PSTR("  Time Since Command: %lu seconds"), timeSinceCommand / MILLISECONDS_PER_SECOND);
        Console.serialDiagnostic(msg);
        
        sprintf_P(msg, PSTR("  Command Result: %s"), lastCommandSuccess ? "SUCCESS" : "FAILED");
        Console.serialDiagnostic(msg);
    } else {
        Console.serialDiagnostic(F("  Last Command: None"));
    }
    
    // Overall system readiness assessment
    bool systemReady = isSystemReadyForAutomation();
    Console.serialDiagnostic(F(""));
    sprintf_P(msg, PSTR("  OVERALL SYSTEM STATUS: %s"), 
              systemReady ? "READY FOR AUTOMATION" : "NOT READY");
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
    printValveDetailedStatus();  // Use detailed version for consistency
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
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
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
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
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

//=============================================================================
// SYSTEM RESET FUNCTION
//=============================================================================
void resetSystemState()
{
    // Track overall reset success
    bool resetSuccessful = true;
    char msg[LARGE_MSG_SIZE];
    
    Console.serialInfo(F("SYSTEM RESET: Clearing operational state"));
    
    // 1. MOTOR FAULT RECOVERY
    // =======================
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
        if (hasMotorFault(railId)) {
            bool faultCleared = false;
            int clearAttempts = 0;
            
            sprintf_P(msg, PSTR("Rail %d: Clearing motor faults (%d attempts max)"), 
                     railId, MAX_FAULT_CLEAR_ATTEMPTS);
            Console.serialInfo(msg);
            
            while (!faultCleared && clearAttempts < MAX_FAULT_CLEAR_ATTEMPTS) {
                clearAttempts++;
                sprintf_P(msg, PSTR("  Attempt %d/%d"), clearAttempts, MAX_FAULT_CLEAR_ATTEMPTS);
                Console.serialInfo(msg);
                
                if (executeRailClearFault(railId)) {
                    faultCleared = true;
                    Console.serialInfo(F("  Faults cleared"));
                } else {
                    Console.serialWarning(F("  Failed"));
                    delay(FAULT_CLEAR_RETRY_DELAY_MS);
                }
            }
            
            if (!faultCleared) {
                sprintf_P(msg, PSTR("Rail %d: Faults persist - manual 'rail%d clear-fault' required"), 
                         railId, railId);
                Console.serialError(msg);
                resetSuccessful = false;
            }
        }
    }
    
    // Re-enable motors if successful
    if (resetSuccessful && !isEStopActive()) {
        for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
            if (!hasMotorFault(railId)) {
                executeRailInit(railId);
                sprintf_P(msg, PSTR("Rail %d: Motor enabled"), railId);
                Console.serialInfo(msg);
            }
        }
    }
    
    // 2. ENCODER/MPG RESET
    // ====================
    Console.serialInfo(F("MPG: Reset to defaults"));
    disableEncoderControl();
    setEncoderMultiplier(DEFAULT_ENCODER_MULTIPLIER);
    resetEncoderTimeouts();
    Console.serialInfo(F("MPG: Disabled, 10x multiplier, timeouts cleared"));
    
    // 3. POSITION RESET
    // =================
    Console.serialInfo(F("POSITIONING: Moving to reset positions"));
    
    if (resetSuccessful && !isEStopActive()) {
        bool positioningSuccessful = true;
        
        // Step 1: Retract cylinder
        Console.serialInfo(F("Rail 2: Retracting cylinder"));
        ValveOperationResult cylinderResult = retractCylinder();
        if (cylinderResult != VALVE_OP_SUCCESS) {
            Console.serialWarning(F("Rail 2: Cylinder retraction failed"));
            positioningSuccessful = false;
        }
        
        // Step 2: Move Rail 2 to Workcell 3
        if (positioningSuccessful && isHomingComplete(2)) {
            Console.serialInfo(F("Rail 2: Moving to Workcell 3"));
            if (!executeRailMoveToPosition(2, RAIL2_WC3_PICKUP_DROPOFF, false)) {
                Console.serialWarning(F("Rail 2: Move to WC3 failed"));
                positioningSuccessful = false;
            }
        }
        
        // Step 3: Move Rail 1 to Staging
        if (positioningSuccessful && isHomingComplete(1)) {
            Console.serialInfo(F("Rail 1: Moving to Staging"));
            if (!executeRailMoveToPosition(1, RAIL1_STAGING_POSITION, false)) {
                Console.serialWarning(F("Rail 1: Move to Staging failed"));
                positioningSuccessful = false;
            }
        }
        
        if (!positioningSuccessful) {
            Console.serialWarning(F("POSITIONING: Some moves failed - manual positioning required"));
            resetSuccessful = false;
        }
    } else {
        Console.serialWarning(F("POSITIONING: Skipped due to faults/E-stop"));
    }
    
    // 4. SYNC AND TIMEOUTS
    // ====================
    Console.serialInfo(F("SYNC: Resetting timeouts and updating state"));
    resetSystemTimeouts();
    
    // Clear any operation in progress state
    resetCommandControllerState();
    Console.serialInfo(F("SYNC: Command controller state reset"));
    
    if (performLabwareAudit()) {
        Console.serialInfo(F("SYNC: Labware state updated"));
    } else {
        Console.serialWarning(F("SYNC: Labware sync incomplete - run 'labware audit'"));
    }
    
    // Final status
    if (resetSuccessful) {
        Console.serialInfo(F("SYSTEM RESET: Complete - ready for automation"));
        Console.acknowledge(F("RESET_SUCCESS"));
    } else {
        Console.serialWarning(F("SYSTEM RESET: Partial - manual intervention required"));
        Console.error(F("RESET_PARTIAL"));
    }
}

//=============================================================================
// SYSTEM HOMING FUNCTION
//=============================================================================
bool homeSystemRails()
{
    char msg[MEDIUM_MSG_SIZE];
    bool homingSuccessful = true;
    
    Console.serialInfo(F("SYSTEM HOME: Starting sequential rail homing"));
    
    // Pre-check: Verify system is safe for homing
    if (isEStopActive()) {
        Console.error(F("HOME FAILED: E-Stop active - release to continue"));
        return false;
    }
    
    // Step 1: Home Rail 1
    // ===================
    Console.serialInfo(F("Rail 1: Homing"));
    
    if (!executeRailHome(1)) {
        Console.error(F("Rail 1: Homing failed"));
        homingSuccessful = false;
    } else {
        if (isHomingComplete(1)) {
            sprintf_P(msg, PSTR("Rail 1: Homed at %.2fmm"), getMotorPositionMm(1));
            Console.serialInfo(msg);
        } else {
            Console.error(F("Rail 1: Homing verification failed"));
            homingSuccessful = false;
        }
    }
    
    // Step 2: Home Rail 2 (only if Rail 1 succeeded)
    // ===============================================
    if (homingSuccessful) {
        Console.serialInfo(F("Rail 2: Homing"));
        
        if (!executeRailHome(2)) {
            Console.error(F("Rail 2: Homing failed"));
            homingSuccessful = false;
        } else {
            if (isHomingComplete(2)) {
                sprintf_P(msg, PSTR("Rail 2: Homed at %.2fmm"), getMotorPositionMm(2));
                Console.serialInfo(msg);
            } else {
                Console.error(F("Rail 2: Homing verification failed"));
                homingSuccessful = false;
            }
        }
    } else {
        Console.serialWarning(F("Rail 2: Skipped due to Rail 1 failure"));
    }
    
    // Final status
    if (homingSuccessful) {
        Console.serialInfo(F("SYSTEM HOME: Complete - ready for automation"));
        Console.acknowledge(F("HOME_SUCCESS"));
    } else {
        Console.serialWarning(F("SYSTEM HOME: Partial - use individual rail commands"));
        Console.error(F("HOME_PARTIAL"));
    }
    
    return homingSuccessful;
}

//=============================================================================
// SYSTEM MOTOR FAULT CLEARING FUNCTION
//=============================================================================
bool clearSystemMotorFaults()
{
    char msg[MEDIUM_MSG_SIZE];
    bool anyFaultsCleared = false;
    bool allClearSuccessful = true;
    
    Console.serialInfo(F("SYSTEM CLEAR: Checking motor fault states"));
    
    // Check each rail for faults
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
        if (hasMotorFault(railId)) {
            anyFaultsCleared = true;
            sprintf_P(msg, PSTR("Rail %d: Motor fault detected - clearing"), railId);
            Console.serialInfo(msg);
            
            if (clearMotorFaultWithStatus(railId)) {
                sprintf_P(msg, PSTR("Rail %d: Motor faults cleared successfully"), railId);
                Console.serialInfo(msg);
            } else {
                sprintf_P(msg, PSTR("Rail %d: Failed to clear motor faults"), railId);
                Console.serialError(msg);
                allClearSuccessful = false;
            }
        } else {
            sprintf_P(msg, PSTR("Rail %d: No motor faults detected"), railId);
            Console.serialInfo(msg);
        }
    }
    
    // Summary message
    if (!anyFaultsCleared) {
        Console.serialInfo(F("SYSTEM CLEAR: No faults detected"));
        Console.acknowledge(F("CLEAR_NOT_NEEDED"));
    } else if (allClearSuccessful) {
        Console.serialInfo(F("SYSTEM CLEAR: All faults cleared"));
        Console.acknowledge(F("CLEAR_SUCCESS"));
    } else {
        Console.serialWarning(F("SYSTEM CLEAR: Partial success"));
        Console.error(F("CLEAR_PARTIAL"));
    }
    
    return anyFaultsCleared ? allClearSuccessful : true;
}

//=============================================================================
// SYSTEM MOTOR INITIALIZATION FUNCTION  
//=============================================================================
bool initSystemMotors()
{
    char msg[MEDIUM_MSG_SIZE];
    bool anyMotorsInitialized = false;
    bool allInitSuccessful = true;
    
    Console.serialInfo(F("SYSTEM INIT: Checking motor states"));
    
    // Pre-check: Verify E-Stop is not active
    if (isEStopActive()) {
        Console.error(F("SYSTEM INIT: E-Stop active - cannot initialize"));
        Console.error(F("INIT_ESTOP_ACTIVE"));
        return false;
    }
    
    // Check each rail for initialization needs
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
        if (hasMotorFault(railId)) {
            sprintf_P(msg, PSTR("Rail %d: Has faults - clear first"), railId);
            Console.serialWarning(msg);
            allInitSuccessful = false;
            continue;
        }
        
        // Check if motor needs initialization
        if (!isMotorReady(railId)) {
            anyMotorsInitialized = true;
            sprintf_P(msg, PSTR("Rail %d: Initializing"), railId);
            Console.serialInfo(msg);
            
            if (initRailMotor(railId)) {
                sprintf_P(msg, PSTR("Rail %d: Initialized successfully"), railId);
                Console.serialInfo(msg);
            } else {
                sprintf_P(msg, PSTR("Rail %d: Init failed"), railId);
                Console.serialError(msg);
                allInitSuccessful = false;
            }
        } else {
            sprintf_P(msg, PSTR("Rail %d: Already initialized"), railId);
            Console.serialInfo(msg);
        }
    }
    
    // Summary message
    if (!anyMotorsInitialized) {
        Console.serialInfo(F("SYSTEM INIT: All motors ready"));
        Console.acknowledge(F("INIT_NOT_NEEDED"));
    } else if (allInitSuccessful) {
        Console.serialInfo(F("SYSTEM INIT: All motors initialized"));
        Console.acknowledge(F("INIT_SUCCESS"));
    } else {
        Console.serialWarning(F("SYSTEM INIT: Partial success"));
        Console.error(F("INIT_PARTIAL"));
    }
    
    return anyMotorsInitialized ? allInitSuccessful : true;
}

//=============================================================================
// TIMEOUT RESET FUNCTIONS
//=============================================================================
void resetSystemTimeouts()
{
    Console.serialInfo(F("TIMEOUTS: Clearing stale timeout tracking"));
    
    // Reset encoder timeouts (critical for MPG safety)
    resetEncoderTimeouts();
    
    // Reset network client timeouts (prevent immediate disconnections)
    resetClientTimeouts();
    
    // Reset motor timeouts (prevent movement/homing failures)
    resetMotorTimeouts();
    
    // Reset valve timeouts (prevent pneumatic operation failures)
    resetValveTimeouts();
    
    // Reset sensor timeouts (prevent debounce/validation failures)
    resetSensorTimeouts();
    
    // Reset handoff timeouts (prevent handoff operation failures)
    resetHandoffTimeouts();
    
    // Reset labware session timeouts (preserve operational history)
    resetLabwareTimeouts();
    
    // Reset command timing for accurate diagnostics
    lastCommandTime = millis();
    
    Console.serialInfo(F("TIMEOUTS: Reset complete - all module timeouts cleared"));
}
