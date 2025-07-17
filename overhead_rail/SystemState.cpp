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
    Console.println(F(""));
    Console.println(F("COMPREHENSIVE SYSTEM STATE REPORT"));
    Console.println(F("================================================================================"));
    
    // Motor system state - direct output without tags
    Console.println(F(""));
    Console.println(F("MOTOR STATUS:"));
    Console.println(F("-------------"));
    
    // Use Console.print for both serial and network clients
    Console.print(F("System Status:\n"));
    Console.print(F("  Uptime: "));
    
    char timeBuffer[80];
    unsigned long uptimeSeconds = timeDiff(millis(), systemStartTime) / 1000;
    formatHumanReadableTime(uptimeSeconds, timeBuffer, sizeof(timeBuffer));
    Console.print(timeBuffer);
    Console.print(F("\n\n"));
    
    // Motor status for both rails
    for (uint8_t railId = 1; railId <= 2; railId++) {
        Console.print(F("Rail ")); Console.print(railId); Console.print(F(" Status:\n"));
        
        // Motor state - more informative than separate ready/moving flags
        Console.print(F("  Motor State: "));
        MotorState state = updateMotorState(railId);
        switch (state) {
            case MOTOR_STATE_NOT_READY:
                printColoredState("NOT_READY");
                break;
            case MOTOR_STATE_IDLE:
                printColoredState("IDLE");
                break;
            case MOTOR_STATE_MOVING:
                printColoredState("MOVING");
                break;
            case MOTOR_STATE_HOMING:
                printColoredState("HOMING");
                break;
            case MOTOR_STATE_FAULTED:
                printColoredState("FAULTED");
                break;
            default:
                printColoredState("UNKNOWN");
                break;
        }
        Console.print(F("\n"));
        
        Console.print(F("  Homed: ")); printColoredYesNo(isHomingComplete(railId)); Console.print(F("\n"));
        
        // Position display - show UNKNOWN if not homed
        Console.print(F("  Position: "));
        if (isHomingComplete(railId)) {
            Console.print(getMotorPositionMm(railId)); Console.print(F(" mm"));
        } else {
            printColoredState("UNKNOWN");
        }
        Console.print(F("\n"));
        
        // HLFB Status - hardware-level feedback
        MotorDriver& motor = getMotorByRail(railId);
        Console.print(F("  HLFB Status: "));
        switch (motor.HlfbState()) {
            case MotorDriver::HLFB_ASSERTED:
                Console.print(F("Asserted (Motor hardware confirms it's at the target position)"));
                break;
            case MotorDriver::HLFB_DEASSERTED:
                Console.print(F("Deasserted (Motor hardware indicates it's either moving or has a problem)"));
                break;
            case MotorDriver::HLFB_UNKNOWN:
            default:
                Console.print(F("Unknown (Hardware status cannot be determined)"));
                break;
        }
        Console.print(F("\n"));
        
        if (hasMotorFault(railId)) {
            Console.print(F("  Motor Fault: ACTIVE\n"));
        } else {
            Console.print(F("  No faults\n"));
        }
        Console.print(F("\n"));
    }
    
    // Sensor states - direct output without tags
    Console.println(F(""));
    Console.println(F("SENSOR STATUS:"));
    Console.println(F("--------------"));
    
    // Use Console.print for both serial and network clients
    Console.print(F("Carriage Sensors:\n"));
    Console.print(F("  WC1: ")); printColoredActiveInactive(isCarriageAtWC1()); Console.print(F("\n"));
    Console.print(F("  WC2: ")); printColoredActiveInactive(isCarriageAtWC2()); Console.print(F("\n"));
    Console.print(F("  WC3: ")); printColoredActiveInactive(isCarriageAtWC3()); Console.print(F("\n"));
    Console.print(F("  Rail1 Handoff: ")); printColoredActiveInactive(isCarriageAtRail1Handoff()); Console.print(F("\n"));
    Console.print(F("  Rail2 Handoff: ")); printColoredActiveInactive(isCarriageAtRail2Handoff()); Console.print(F("\n"));
    Console.print(F("\n"));
    
    Console.print(F("Labware Sensors:\n"));
    Console.print(F("  WC1: ")); printColoredActiveInactive(isLabwarePresentAtWC1()); Console.print(F("\n"));
    Console.print(F("  WC2: ")); printColoredActiveInactive(isLabwarePresentAtWC2()); Console.print(F("\n"));
    Console.print(F("  Rail2: ")); printColoredActiveInactive(isLabwarePresentOnRail2()); Console.print(F("\n"));
    Console.print(F("  Handoff: ")); printColoredActiveInactive(isLabwarePresentAtRail1Handoff()); Console.print(F("\n"));
    Console.print(F("\n"));
    
    Console.print(F("Cylinder Sensors:\n"));
    Console.print(F("  Extended: ")); printColoredActiveInactive(isCylinderExtended()); Console.print(F("\n"));
    Console.print(F("  Retracted: ")); printColoredActiveInactive(isCylinderRetracted()); Console.print(F("\n"));
    
    // Pneumatic system - direct output without tags
    Console.println(F(""));
    Console.println(F("PNEUMATIC SYSTEMS:"));
    Console.println(F("------------------"));
    
    // Use Console.print for both serial and network clients
    Console.print(F("Air Pressure: ")); Console.print(getPressurePsi()); Console.print(F(" PSI"));
    Console.print(F(" ")); printColoredSufficient(isPressureSufficient()); Console.print(F("\n"));
    Console.print(F("\n"));
    
    Console.print(F("Valve Status:\n"));
    Console.print(F("  Current Position: ")); Console.print(getValvePositionName(getValvePosition())); Console.print(F("\n"));
    Console.print(F("  Valve Output: ")); Console.print(digitalRead(PNEUMATIC_CYLINDER_VALVE_PIN) ? "HIGH" : "LOW"); Console.print(F("\n"));
    
    // Validate position
    Console.print(F("  Position Validation: "));
    if (validateValvePosition()) {
        printColoredPassed(true);
        Console.print(F(" (Controller state matches sensor readings)\n"));
    } else {
        printColoredPassed(false);
        Console.print(F(" (Controller state does not match sensors - check wiring/sensors)\n"));
    }
    
    // Labware automation - high-level tracking state
    Console.println(F(""));
    Console.println(F("LABWARE AUTOMATION:"));
    Console.println(F("-------------------"));
    
    // Rail 1 labware state
    Console.print(F("  Rail 1: "));
    if (labwareSystem.rail1.hasLabware) {
        Console.print(F("\x1b[32mHAS_LABWARE\x1b[0m at "));
        Console.print(getLocationName(labwareSystem.rail1.lastKnownLocation));
        Console.print(F(" (confidence: "));
        Console.print(getConfidenceName(labwareSystem.rail1.confidence));
        Console.print(F(")"));
    } else {
        Console.print(F("\x1b[90mNO_LABWARE\x1b[0m (confidence: "));
        Console.print(getConfidenceName(labwareSystem.rail1.confidence));
        Console.print(F(")"));
    }
    Console.print(F("\n"));
    
    // Rail 2 labware state
    Console.print(F("  Rail 2: "));
    if (labwareSystem.rail2.hasLabware) {
        Console.print(F("\x1b[32mHAS_LABWARE\x1b[0m"));
        if (labwareSystem.rail2.labwareSource != LOCATION_UNKNOWN) {
            Console.print(F(" from "));
            Console.print(getLocationName(labwareSystem.rail2.labwareSource));
        }
        Console.print(F(" (confidence: "));
        Console.print(getConfidenceName(labwareSystem.rail2.confidence));
        Console.print(F(")"));
    } else {
        Console.print(F("\x1b[90mNO_LABWARE\x1b[0m (confidence: "));
        Console.print(getConfidenceName(labwareSystem.rail2.confidence));
        Console.print(F(")"));
    }
    Console.print(F("\n"));
    
    // Automation status
    Console.print(F("  Automation: "));
    if (labwareSystem.automationEnabled) {
        Console.print(F("\x1b[32mENABLED\x1b[0m"));
    } else {
        Console.print(F("\x1b[90mDISABLED\x1b[0m"));
    }
    Console.print(F("\n"));
    
    // Conflict status
    Console.print(F("  Conflicts: "));
    if (labwareSystem.dualLabwareConflict) {
        Console.print(F("\x1b[1;31mDUAL_LABWARE_CONFLICT\x1b[0m"));
    } else {
        Console.print(F("\x1b[32mNONE\x1b[0m"));
    }
    Console.print(F("\n"));
    
    // Last audit info
    Console.print(F("  Last Audit: "));
    if (labwareSystem.lastSystemAudit > 0) {
        char timeBuffer[80];
        unsigned long timeSinceAudit = timeDiff(millis(), labwareSystem.lastSystemAudit) / 1000;
        formatHumanReadableTime(timeSinceAudit, timeBuffer, sizeof(timeBuffer));
        Console.print(timeBuffer);
        Console.print(F(" ago"));
    } else {
        Console.print(F("\x1b[90mNever performed\x1b[0m"));
    }
    Console.print(F("\n"));
    
    // Network status - direct output without tags
    Console.println(F(""));
    Console.println(F("NETWORK STATUS:"));
    Console.println(F("---------------"));
    
    // Use Console.print for both serial and network clients
    Console.print(F("Ethernet Status:\n"));
    Console.print(F("  Connected Clients: ")); Console.print(getConnectedClientCount()); Console.print(F("\n"));
    
    // Manual controls - direct output without tags
    Console.println(F(""));
    Console.println(F("MANUAL CONTROLS (MPG):"));
    Console.println(F("----------------------"));
    
    // Use Console.print for both serial and network clients
    Console.print(F("MPG Status:\n"));
    Console.print(F("  Active: "));
    if (encoderControlActive) {
        Console.print(F("\x1b[32mYES\x1b[0m"));
    } else {
        Console.print(F("\x1b[90mNO\x1b[0m"));
    }
    Console.print(F("\n"));
    if (encoderControlActive) {
        Console.print(F("  Active Rail: ")); Console.print(activeEncoderRail); Console.print(F("\n"));
        Console.print(F("  Multiplier: ")); Console.print(currentMultiplierScaled / 100.0); Console.print(F("x\n"));
    }
    
    // Safety systems and custom state
    printSafetySystemState();
    
    // System activity and readiness summary
    printSystemReadinessState();
    
    Console.println(F("================================================================================"));
    Console.println(F(""));
}

//=============================================================================
// SAFETY SYSTEMS STATE (Custom - not available elsewhere)
//=============================================================================
void printSafetySystemState()
{
    Console.println(F(""));
    Console.println(F("SAFETY SYSTEMS:"));
    Console.println(F("---------------"));
    
    // Emergency stop status - check actual hardware state
    Console.print(F("  Emergency Stop: "));
    if (isEStopActive()) {
        Console.print(F("\x1b[1;31mACTIVE\x1b[0m (UNSAFE - System disabled)"));
    } else {
        Console.print(F("\x1b[32mINACTIVE\x1b[0m (Safe - System operational)"));
    }
    Console.print(F("\n"));
    
    // Position limits - check if any rail is at travel boundaries
    Console.print(F("  Position Limits: "));
    bool anyRailAtLimit = false;
    for (uint8_t railId = 1; railId <= 2; railId++) {
        if (isHomingComplete(railId)) {
            float position = getMotorPositionMm(railId);
            float maxTravel = (railId == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
            
            // Check if at or very close to limits (within 5mm)
            if (position <= 5.0 || position >= (maxTravel - 5.0)) {
                anyRailAtLimit = true;
                break;
            }
        }
    }
    
    if (anyRailAtLimit) {
        Console.print(F("\x1b[1;31mNEAR-LIMIT\x1b[0m (Rail approaching travel boundary)"));
    } else {
        Console.print(F("\x1b[32mOK\x1b[0m (All rails within safe travel range)"));
    }
    Console.print(F("\n"));
}

//=============================================================================
// SYSTEM READINESS AND ACTIVITY SUMMARY
//=============================================================================
void printSystemReadinessState()
{
    Console.println(F(""));
    Console.println(F("SYSTEM ACTIVITY & READINESS:"));
    Console.println(F("----------------------------"));
    
    char msg[MEDIUM_MSG_SIZE];
    char timeBuffer[80];
    
    // System timing
    unsigned long uptime = timeDiff(millis(), systemStartTime);
    unsigned long uptimeSeconds = uptime / MILLISECONDS_PER_SECOND;
    formatHumanReadableTime(uptimeSeconds, timeBuffer, sizeof(timeBuffer));
    sprintf_P(msg, PSTR("  System Uptime: %s"), timeBuffer);
    Console.println(msg);
    
    // Command activity from global variables
    if (lastExecutedCommand[0] != STRING_TERMINATOR) {
        Console.print(F("  Last Command: "));
        Console.print(F("\x1b[33m")); // Yellow for command
        Console.print(lastExecutedCommand);
        Console.print(F("\x1b[0m"));
        Console.println(F(""));
        
        unsigned long timeSinceCommand = timeDiff(millis(), lastCommandTime);
        unsigned long timeSinceCommandSeconds = timeSinceCommand / MILLISECONDS_PER_SECOND;
        formatHumanReadableTime(timeSinceCommandSeconds, timeBuffer, sizeof(timeBuffer));
        sprintf_P(msg, PSTR("  Time Since Command: %s"), timeBuffer);
        Console.println(msg);
        
        Console.print(F("  Command Result: "));
        if (lastCommandSuccess) {
            Console.print(F("\x1b[32mSUCCESS\x1b[0m"));
        } else {
            Console.print(F("\x1b[1;31mFAILED\x1b[0m"));
        }
        Console.println(F(""));
    } else {
        Console.println(F("  Last Command: None"));
    }
    
    // Overall system readiness assessment
    bool systemReady = isSystemReadyForAutomation();
    Console.println(F(""));
    Console.print(F("  OVERALL SYSTEM STATUS: "));
    if (systemReady) {
        Console.print(F("\x1b[1;32mREADY FOR AUTOMATION\x1b[0m"));
    } else {
        Console.print(F("\x1b[1;31mNOT READY\x1b[0m"));
    }
    Console.println(F(""));
    
    if (!systemReady) {
        Console.print(F("  Error Summary: "));
        Console.print(F("\x1b[1;31m"));
        Console.print(getSystemErrorSummary());
        Console.print(F("\x1b[0m"));
        Console.println(F(""));
    }
}


//=============================================================================
// SYSTEM READINESS AND UTILITY FUNCTIONS
//=============================================================================
bool isSystemReadyForAutomation()
{
    // Check emergency stop - use actual hardware state
    if (isEStopActive()) {
        return false;
    }
    
    // Check position limits - check if any rail is near travel boundaries
    for (uint8_t railId = 1; railId <= 2; railId++) {
        if (isHomingComplete(railId)) {
            float position = getMotorPositionMm(railId);
            float maxTravel = (railId == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
            
            // Check if at or very close to limits (within 5mm)
            if (position <= 5.0 || position >= (maxTravel - 5.0)) {
                return false;
            }
        }
    }
    
    // Check motor readiness for both rails
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
        // Check motor initialization first
        if (!isMotorReady(railId)) {
            return false;
        }
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
    
    // Safety errors first (highest priority)
    if (isEStopActive()) {
        if (errorMsg.length() > 0) errorMsg += " | ";
        errorMsg += "EMERGENCY STOP ACTIVE";
    }
    
    // Motor system errors
    String motorErrors = "";
    for (uint8_t railId = FIRST_RAIL_ID; railId <= LAST_RAIL_ID; railId++) {
        String railErrors = "";
        
        // Check motor initialization (most common issue)
        if (!isMotorReady(railId)) {
            railErrors += "not initialized";
        }
        
        // Check homing status
        if (!isHomingComplete(railId)) {
            if (railErrors.length() > 0) railErrors += ", ";
            railErrors += "not homed";
        }
        
        // Check motor faults
        if (hasMotorFault(railId)) {
            if (railErrors.length() > 0) railErrors += ", ";
            railErrors += "motor fault";
        }
        
        // Add rail-specific errors to motor errors
        if (railErrors.length() > 0) {
            if (motorErrors.length() > 0) motorErrors += "; ";
            motorErrors += "Rail " + String(railId) + ": " + railErrors;
        }
    }
    
    if (motorErrors.length() > 0) {
        if (errorMsg.length() > 0) errorMsg += " | ";
        errorMsg += "MOTORS: " + motorErrors;
    }
    
    // Position limit warnings
    for (uint8_t railId = 1; railId <= 2; railId++) {
        if (isHomingComplete(railId)) {
            float position = getMotorPositionMm(railId);
            float maxTravel = (railId == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
            
            // Check if at or very close to limits (within 5mm)
            if (position <= 5.0 || position >= (maxTravel - 5.0)) {
                if (errorMsg.length() > 0) errorMsg += " | ";
                errorMsg += "POSITION: Rail " + String(railId) + " near travel limit";
                break; // Only report once if any rail is near limit
            }
        }
    }
    
    // Pneumatic system errors
    if (!isPressureSufficient()) {
        if (errorMsg.length() > 0) errorMsg += " | ";
        errorMsg += "PNEUMATICS: Low air pressure";
    }
    
    // Cylinder sensor errors
    if (isCylinderActuallyRetracted() && isCylinderActuallyExtended()) {
        if (errorMsg.length() > 0) errorMsg += " | ";
        errorMsg += "SENSORS: Cylinder position conflict";
    }
    
    if (errorMsg.length() == 0) {
        errorMsg = "NONE";
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
