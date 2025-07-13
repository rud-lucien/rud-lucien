#include "LabwareAutomation.h"
#include "OutputManager.h"
#include "Sensors.h"
#include "MotorController.h"
#include "RailAutomation.h"
#include "PositionConfig.h"

//=============================================================================
// GLOBAL LABWARE STATE INSTANCE
//=============================================================================

// Global instance of the labware automation state
SystemLabwareState labwareSystem;

//=============================================================================
// INITIALIZATION AND SETUP
//=============================================================================

void initLabwareSystem() {
    // Initialize labware tracking system
    Console.serialInfo(F("Initializing labware automation system..."));
    
    // Clear all state to start fresh
    clearLabwareState();
    
    // TODO: Load any saved state from SD card if available
    // TODO: Perform initial sensor reading to establish baseline
    
    Console.serialInfo(F("Labware automation system initialized"));
}

//=============================================================================
// STATE MANAGEMENT FUNCTIONS
//=============================================================================

void updateLabwareSystemState() {
    // Update system state based on current sensor readings
    // This should be called periodically from main loop
    
    // TODO: Update Rail 2 state from carriage sensor
    updateRail2LabwareFromSensor();
    
    // TODO: Update timestamp and confidence levels
    // TODO: Check for dual labware conflicts
    // TODO: Validate state consistency
}

void clearLabwareState() {
    // Clear all labware tracking state (nuclear reset)
    Console.serialInfo(F("Clearing all labware tracking state..."));
    
    // Reset Rail 1 state
    labwareSystem.rail1.hasLabware = false;
    labwareSystem.rail1.lastKnownLocation = LOCATION_UNKNOWN;
    labwareSystem.rail1.labwareSource = LOCATION_UNKNOWN;
    labwareSystem.rail1.validated = false;
    labwareSystem.rail1.uncertainDueToFault = false;
    labwareSystem.rail1.lastValidated = 0;
    labwareSystem.rail1.confidence = CONFIDENCE_UNKNOWN;
    
    // Reset Rail 2 state
    labwareSystem.rail2.hasLabware = false;
    labwareSystem.rail2.labwareSource = LOCATION_UNKNOWN;
    labwareSystem.rail2.lastValidated = 0;
    labwareSystem.rail2.confidence = CONFIDENCE_UNKNOWN;
    
    // Reset system state
    labwareSystem.automationEnabled = false;
    labwareSystem.dualLabwareConflict = false;
    labwareSystem.lastSystemAudit = 0;
    
    Console.serialInfo(F("Labware state cleared - system requires audit before automation"));
}

void setLabwareStateUncertain(int railNumber, const char* reason) {
    // Mark labware state as uncertain due to fault or error
    unsigned long currentTime = millis();
    
    if (railNumber == 1) {
        labwareSystem.rail1.uncertainDueToFault = true;
        labwareSystem.rail1.confidence = CONFIDENCE_UNKNOWN;
        labwareSystem.rail1.validated = false;
    } else if (railNumber == 2) {
        labwareSystem.rail2.confidence = CONFIDENCE_UNKNOWN;
    }
    
    // Disable automation until state is validated
    labwareSystem.automationEnabled = false;
    
    Console.serialWarning((String(F("Rail ")) + String(railNumber) + F(" labware state uncertain: ") + String(reason)).c_str());
}

//=============================================================================
// STATE QUERY FUNCTIONS
//=============================================================================

bool isLabwareSystemReady() {
    // Check if system is ready for automation
    return labwareSystem.automationEnabled && !labwareSystem.dualLabwareConflict;
}

bool hasLabwareConflict() {
    // Check for dual labware conflict
    return labwareSystem.dualLabwareConflict;
}

ConfidenceLevel getLabwareConfidence(int railNumber) {
    // Get confidence level for specified rail
    if (railNumber == 1) {
        return labwareSystem.rail1.confidence;
    } else if (railNumber == 2) {
        return labwareSystem.rail2.confidence;
    }
    return CONFIDENCE_UNKNOWN;
}

const char* getLocationName(Location loc) {
    // Convert location enum to string
    switch (loc) {
        case LOCATION_WC1: return "WC1";
        case LOCATION_WC2: return "WC2";
        case LOCATION_WC3: return "WC3";
        case LOCATION_HANDOFF: return "HANDOFF";
        case LOCATION_STAGING: return "STAGING";
        case LOCATION_UNKNOWN: 
        default: return "UNKNOWN";
    }
}

const char* getConfidenceName(ConfidenceLevel confidence) {
    // Convert confidence enum to string
    switch (confidence) {
        case CONFIDENCE_HIGH: return "HIGH";
        case CONFIDENCE_MEDIUM: return "MEDIUM";
        case CONFIDENCE_LOW: return "LOW";
        case CONFIDENCE_UNKNOWN:
        default: return "UNKNOWN";
    }
}

//=============================================================================
// SENSOR INTEGRATION FUNCTIONS
//=============================================================================

void updateRail1LabwareFromSensor(Location sensorLocation) {
    // Update Rail 1 labware state based on sensor reading at specific location
    bool sensorReading = false;
    
    // Read appropriate sensor based on location
    switch (sensorLocation) {
        case LOCATION_WC1:
            sensorReading = isLabwarePresentAtWC1();
            break;
        case LOCATION_WC2:
            sensorReading = isLabwarePresentAtWC2();
            break;
        case LOCATION_HANDOFF:
            sensorReading = isLabwarePresentAtHandoff();
            break;
        default:
            Console.serialError(F("updateRail1LabwareFromSensor: Invalid sensor location"));
            return;
    }
    
    // Update Rail 1 state based on sensor reading
    if (sensorReading) {
        // Labware detected at sensor location
        labwareSystem.rail1.hasLabware = true;
        labwareSystem.rail1.lastKnownLocation = sensorLocation;
        labwareSystem.rail1.validated = true;
        labwareSystem.rail1.uncertainDueToFault = false;
        labwareSystem.rail1.confidence = CONFIDENCE_MEDIUM;
    } else {
        // No labware at sensor location
        if (labwareSystem.rail1.lastKnownLocation == sensorLocation) {
            // Rail 1 was expected to have labware here but doesn't
            labwareSystem.rail1.hasLabware = false;
            labwareSystem.rail1.validated = true;
            labwareSystem.rail1.uncertainDueToFault = false;
            labwareSystem.rail1.confidence = CONFIDENCE_MEDIUM;
        }
    }
    
    labwareSystem.rail1.lastValidated = millis();
}

void updateRail2LabwareFromSensor() {
    // Update Rail 2 labware state from carriage sensor
    // TODO: Implement when Rail 2 carriage sensor function is available
    
    // Placeholder implementation
    // bool carriageSensor = isLabwarePresentOnRail2Carriage();
    // labwareSystem.rail2.hasLabware = carriageSensor;
    // labwareSystem.rail2.confidence = CONFIDENCE_HIGH;
    // labwareSystem.rail2.lastValidated = millis();
    
    Console.serialInfo(F("updateRail2LabwareFromSensor: Not yet implemented"));
}

bool validateLabwareStateAtLocation(Location location) {
    // Validate labware state at specific location
    // TODO: Implement location-specific validation
    Console.serialInfo(F("validateLabwareStateAtLocation: Not yet implemented"));
    return false;
}

//=============================================================================
// AUDIT AND RECOVERY FUNCTIONS
//=============================================================================

bool performLabwareAudit() {
    // Perform complete labware system audit
    Console.serialInfo(F("Starting labware system audit..."));
    
    // TODO: Implement complete audit process
    // 1. Check Rail 1 position and move to nearest sensor
    // 2. Validate Rail 1 labware state
    // 3. Update Rail 2 state from carriage sensor  
    // 4. Check for conflicts and inconsistencies
    // 5. Enable automation if all validations pass
    
    Console.serialInfo(F("Labware audit placeholder - implementation pending"));
    return true;
}

Location findNearestSafeSensorForRail1() {
    // Find nearest safe sensor location for Rail 1 audit
    if (!isHomingComplete(1)) {
        Console.serialError(F("Cannot find nearest sensor - Rail 1 not homed"));
        return LOCATION_UNKNOWN;
    }
    
    double currentPos = getMotorPositionMm(1);
    double distanceToWC1 = abs(currentPos - getRail1WC1PickupMm());
    double distanceToWC2 = abs(currentPos - getRail1WC2PickupMm());
    
    // Return nearest safe sensor (WC1 or WC2 only - avoid handoff collision zone)
    return (distanceToWC1 <= distanceToWC2) ? LOCATION_WC1 : LOCATION_WC2;
}

bool moveRail1ToNearestSensorAndValidate() {
    // Move Rail 1 to nearest sensor and validate labware state
    Location targetLocation = findNearestSafeSensorForRail1();
    
    if (targetLocation == LOCATION_UNKNOWN) {
        Console.serialError(F("Cannot perform audit - no valid sensor location"));
        return false;
    }
    
    Console.serialInfo((String(F("Moving Rail 1 to ")) + getLocationName(targetLocation) + F(" for validation")).c_str());
    
    // TODO: Implement actual movement and validation
    // 1. Move to target location at with-labware speeds (conservative)
    // 2. Read sensor at destination
    // 3. Update labware state based on sensor reading
    // 4. Clear uncertainty flags
    
    Console.serialInfo(F("Rail 1 audit movement placeholder - implementation pending"));
    return true;
}

//=============================================================================
// STATUS REPORTING FUNCTIONS
//=============================================================================

void printLabwareSystemStatus() {
    // Print complete labware system status
    Console.serialInfo(F("LABWARE_SYSTEM_STATUS: Current tracking state"));
    Console.serialInfo(F("============================================"));
    
    printLabwareStateDetails();
    printSensorReadings();
    
    Console.serialInfo(F("SYSTEM STATUS:"));
    Console.serialInfo(labwareSystem.automationEnabled ? F("  Automation: ENABLED") : F("  Automation: DISABLED"));
    Console.serialInfo(labwareSystem.dualLabwareConflict ? F("  Conflicts: DUAL_LABWARE_CONFLICT") : F("  Conflicts: NONE"));
    
    if (labwareSystem.lastSystemAudit > 0) {
        unsigned long timeSinceAudit = (millis() - labwareSystem.lastSystemAudit) / 1000;
        Console.serialInfo((String(F("  Last Audit: ")) + String(timeSinceAudit) + F(" seconds ago")).c_str());
    } else {
        Console.serialInfo(F("  Last Audit: NEVER"));
    }
    
    Console.serialInfo(F("============================================"));
}

void printLabwareStateDetails() {
    // Print detailed rail state information
    Console.serialInfo(F("RAIL STATUS:"));
    
    // Rail 1 status
    String rail1Status = String(F("Rail 1: "));
    if (labwareSystem.rail1.hasLabware) {
        rail1Status += String(F("HAS_LABWARE (from "));
        rail1Status += String(getLocationName(labwareSystem.rail1.labwareSource));
        rail1Status += String(F(", at "));
        rail1Status += String(getLocationName(labwareSystem.rail1.lastKnownLocation));
        rail1Status += String(F(")"));
    } else {
        rail1Status += String(F("NO_LABWARE"));
    }
    rail1Status += String(F(" - confidence: "));
    rail1Status += String(getConfidenceName(labwareSystem.rail1.confidence));
    Console.serialInfo(rail1Status.c_str());
    
    // Rail 2 status  
    String rail2Status = String(F("Rail 2: "));
    if (labwareSystem.rail2.hasLabware) {
        rail2Status += String(F("HAS_LABWARE (from "));
        rail2Status += String(getLocationName(labwareSystem.rail2.labwareSource));
        rail2Status += String(F(")"));
    } else {
        rail2Status += String(F("NO_LABWARE"));
    }
    rail2Status += String(F(" - confidence: "));
    rail2Status += String(getConfidenceName(labwareSystem.rail2.confidence));
    Console.serialInfo(rail2Status.c_str());
    
    Console.serialInfo(F(""));
}

void printSensorReadings() {
    // Print current sensor readings
    Console.serialInfo(F("SENSOR READINGS:"));
    Console.serialInfo(isLabwarePresentAtWC1() ? F("  WC1: LABWARE_PRESENT") : F("  WC1: NO_LABWARE"));
    Console.serialInfo(isLabwarePresentAtWC2() ? F("  WC2: LABWARE_PRESENT") : F("  WC2: NO_LABWARE"));
    Console.serialInfo(isLabwarePresentAtWC3() ? F("  WC3: LABWARE_PRESENT") : F("  WC3: NO_LABWARE"));
    Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff: LABWARE_PRESENT") : F("  Handoff: NO_LABWARE"));
    
    // TODO: Add Rail 2 carriage sensor when available
    Console.serialInfo(F("  Rail 2 Carriage: SENSOR_NOT_IMPLEMENTED"));
    Console.serialInfo(F(""));
}
