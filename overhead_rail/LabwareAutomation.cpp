#include "LabwareAutomation.h"
#include "OutputManager.h"
#include "Sensors.h"
#include "MotorController.h"
#include "RailAutomation.h"
#include "PositionConfig.h"
#include "Utils.h"

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
    
    // Initialize operation counters if this is the first time
    if (labwareSystem.counters.startTime == 0) {
        resetOperationCounters();
    }
    
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
    
    // Reset operation counters
    resetOperationCounters();
    
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
    Console.serialInfo(F("LABWARE_AUDIT_INITIATED: Starting comprehensive system validation"));
    
    // Step 1: Validate system preconditions
    if (isEStopActive()) {
        Console.error(F("AUDIT_FAILED: E-Stop is active - cannot perform movements"));
        return false;
    }
    
    if (!isHomingComplete(1)) {
        Console.error(F("AUDIT_FAILED: Rail 1 not homed - use 'rail1 home' first"));
        return false;
    }
    
    if (!isHomingComplete(2)) {
        Console.error(F("AUDIT_FAILED: Rail 2 not homed - use 'rail2 home' first"));
        return false;
    }
    
    Console.serialInfo(F("AUDIT_PRECONDITIONS: All preconditions satisfied"));
    
    // Step 2: Determine Rail 1 audit strategy
    Location nearestSensor = findNearestSafeSensorForRail1();
    if (nearestSensor == LOCATION_UNKNOWN) {
        Console.error(F("AUDIT_FAILED: Cannot determine safe sensor location for Rail 1"));
        return false;
    }
    
    Console.serialInfo((String(F("AUDIT_STRATEGY: Moving Rail 1 to ")) + getLocationName(nearestSensor) + F(" for sensor validation")).c_str());
    
    // Step 3: Move Rail 1 to sensor location and validate
    bool rail1ValidationSuccess = false;
    
    if (nearestSensor == LOCATION_WC1) {
        // Move to WC1 at conservative speed and read sensor
        Console.serialInfo(F("AUDIT_MOVEMENT: Moving to WC1 at with-labware speeds (conservative)"));
        rail1ValidationSuccess = moveRail1CarriageToWC1(true); // Use conservative speeds
        
        if (rail1ValidationSuccess) {
            // Read WC1 sensor and update state
            bool labwareDetected = isLabwarePresentAtWC1();
            labwareSystem.rail1.hasLabware = labwareDetected;
            labwareSystem.rail1.lastKnownLocation = LOCATION_WC1;
            labwareSystem.rail1.validated = true;
            labwareSystem.rail1.uncertainDueToFault = false;
            labwareSystem.rail1.confidence = CONFIDENCE_MEDIUM;
            labwareSystem.rail1.lastValidated = millis();
            
            Console.serialInfo(labwareDetected ? F("SENSOR_VALIDATION: WC1 labware detected") : F("SENSOR_VALIDATION: WC1 no labware"));
        }
        
    } else if (nearestSensor == LOCATION_WC2) {
        // Move to WC2 at conservative speed and read sensor
        Console.serialInfo(F("AUDIT_MOVEMENT: Moving to WC2 at with-labware speeds (conservative)"));
        rail1ValidationSuccess = moveRail1CarriageToWC2(true); // Use conservative speeds
        
        if (rail1ValidationSuccess) {
            // Read WC2 sensor and update state
            bool labwareDetected = isLabwarePresentAtWC2();
            labwareSystem.rail1.hasLabware = labwareDetected;
            labwareSystem.rail1.lastKnownLocation = LOCATION_WC2;
            labwareSystem.rail1.validated = true;
            labwareSystem.rail1.uncertainDueToFault = false;
            labwareSystem.rail1.confidence = CONFIDENCE_MEDIUM;
            labwareSystem.rail1.lastValidated = millis();
            
            Console.serialInfo(labwareDetected ? F("SENSOR_VALIDATION: WC2 labware detected") : F("SENSOR_VALIDATION: WC2 no labware"));
        }
    }
    
    if (!rail1ValidationSuccess) {
        Console.error(F("AUDIT_FAILED: Unable to move Rail 1 to sensor location"));
        return false;
    }
    
    // Step 4: Validate Rail 2 state (if carriage sensor available)
    Console.serialInfo(F("AUDIT_RAIL2: Validating Rail 2 labware state"));
    // For now, use WC3 sensor as proxy since carriage sensor not implemented
    bool rail2HasLabware = isLabwarePresentOnRail2();
    labwareSystem.rail2.hasLabware = rail2HasLabware;
    labwareSystem.rail2.confidence = CONFIDENCE_MEDIUM; // Using WC3 sensor proxy
    labwareSystem.rail2.lastValidated = millis();
    
    Console.serialInfo(rail2HasLabware ? F("RAIL2_VALIDATION: Labware detected at WC3") : F("RAIL2_VALIDATION: No labware at WC3"));
    
    // Step 5: Check for dual labware conflicts
    labwareSystem.dualLabwareConflict = (labwareSystem.rail1.hasLabware && labwareSystem.rail2.hasLabware);
    
    if (labwareSystem.dualLabwareConflict) {
        Console.serialWarning(F("AUDIT_WARNING: Dual labware detected - both rails have labware"));
        Console.serialInfo(F("  This requires manual resolution before automation can proceed"));
    }
    
    // Step 6: Enable automation if no conflicts
    if (!labwareSystem.dualLabwareConflict) {
        labwareSystem.automationEnabled = true;
        Console.serialInfo(F("AUTOMATION_ENABLED: System ready for goto commands"));
    } else {
        labwareSystem.automationEnabled = false;
        Console.serialInfo(F("AUTOMATION_DISABLED: Resolve dual labware conflict first"));
    }
    
    // Step 7: Update audit timestamp and report results
    labwareSystem.lastSystemAudit = millis();
    
    Console.acknowledge(F("AUDIT_COMPLETE: Labware state validated and updated"));
    Console.serialInfo(F("Use 'labware status' to see updated tracking state"));
    
    return true;
}

Location findNearestSafeSensorForRail1() {
    // Find nearest safe sensor location for Rail 1 audit
    if (!isHomingComplete(1)) {
        Console.serialError(F("Cannot find nearest sensor - Rail 1 not homed"));
        return LOCATION_UNKNOWN;
    }
    
    double currentPos = getMotorPositionMm(1);
    double distanceToWC1 = abs(currentPos - RAIL1_WC1_PICKUP_DROPOFF);
    double distanceToWC2 = abs(currentPos - RAIL1_WC2_PICKUP_DROPOFF);
    
    // Return nearest safe sensor (WC1 or WC2 only - avoid handoff collision zone)
    return (distanceToWC1 <= distanceToWC2) ? LOCATION_WC1 : LOCATION_WC2;
}

bool moveRail1ToNearestSensorAndValidate() {
    // This function is deprecated - logic moved into performLabwareAudit()
    // Kept for compatibility but should not be called
    Console.serialError(F("moveRail1ToNearestSensorAndValidate: Function deprecated - use performLabwareAudit() instead"));
    return false;
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
    Console.serialInfo(labwareSystem.automationEnabled ? F("  Goto Commands: ENABLED") : F("  Goto Commands: DISABLED"));
    Console.serialInfo(labwareSystem.dualLabwareConflict ? F("  Conflicts: DUAL_LABWARE_CONFLICT") : F("  Conflicts: NONE"));
    
    if (labwareSystem.lastSystemAudit > 0) {
        unsigned long timeSinceAudit = (millis() - labwareSystem.lastSystemAudit) / 1000;
        char timeBuffer[80];
        formatHumanReadableTime(timeSinceAudit, timeBuffer, sizeof(timeBuffer));
        Console.serialInfo((String(F("  Last Audit: ")) + String(timeBuffer) + F(" ago")).c_str());
    } else {
        Console.serialInfo(F("  Last Audit: NEVER"));
    }
    
    Console.serialInfo(F(""));
    printOperationCounters();
    
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
    Console.serialInfo(isLabwarePresentOnRail2() ? F("  Rail 2: LABWARE_PRESENT") : F("  Rail 2: NO_LABWARE"));
    Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff: LABWARE_PRESENT") : F("  Handoff: NO_LABWARE"));
    
    // TODO: Add Rail 2 carriage sensor when available
    Console.serialInfo(F("  Rail 2 Carriage: SENSOR_NOT_IMPLEMENTED"));
    Console.serialInfo(F(""));
}

//=============================================================================
// AUTOMATIC LABWARE DETECTION FUNCTIONS (FOR HOMING INTEGRATION)
//=============================================================================

void performAutomaticLabwareDetectionOnHoming(int railNumber) {
    Console.serialInfo((String(F("AUTO_DETECTION: Updating labware state after Rail ")) + String(railNumber) + F(" homing")).c_str());
    
    if (railNumber == 1) {
        updateRail1LabwareStateAfterHoming();
    } else if (railNumber == 2) {
        updateRail2LabwareStateAfterHoming();
    }
    
    // Attempt to enable automation if both rails have been homed and validated
    if (isHomingComplete(1) && isHomingComplete(2)) {
        attemptToEnableAutomationAfterHoming();
    }
}

void updateRail1LabwareStateAfterHoming() {
    Console.serialInfo(F("RAIL1_AUTO_UPDATE: Reading handoff sensor after homing"));
    
    // Rail 1 homes to position 0mm (handoff location)
    // Check the handoff sensor for labware presence
    bool labwareDetected = isLabwarePresentAtHandoff();
    
    // Update Rail 1 state with sensor reading
    labwareSystem.rail1.hasLabware = labwareDetected;
    labwareSystem.rail1.lastKnownLocation = LOCATION_HANDOFF;
    labwareSystem.rail1.validated = true;
    labwareSystem.rail1.uncertainDueToFault = false;
    labwareSystem.rail1.lastValidated = millis();
    labwareSystem.rail1.confidence = CONFIDENCE_MEDIUM; // Sensor checkpoint confirmation
    
    if (labwareDetected) {
        labwareSystem.rail1.labwareSource = LOCATION_HANDOFF; // Assume it was placed there manually
        Console.serialInfo(F("  DETECTED: Labware present at handoff"));
    } else {
        Console.serialInfo(F("  CLEAR: No labware at handoff"));
    }
    
    Console.serialInfo(F("  RAIL1_STATE: Updated from handoff sensor after homing"));
}

void updateRail2LabwareStateAfterHoming() {
    Console.serialInfo(F("RAIL2_AUTO_UPDATE: Reading carriage sensor after homing"));
    
    // Rail 2 has carriage-mounted sensor, read current state
    // For now, use WC3 sensor as proxy (since carriage homes to WC3)
    bool labwareDetected = isLabwarePresentOnRail2();
    
    // Update Rail 2 state with sensor reading
    labwareSystem.rail2.hasLabware = labwareDetected;
    labwareSystem.rail2.labwareSource = labwareDetected ? LOCATION_WC3 : LOCATION_UNKNOWN;
    labwareSystem.rail2.lastValidated = millis();
    labwareSystem.rail2.confidence = CONFIDENCE_HIGH; // Rail 2 always has high confidence
    
    if (labwareDetected) {
        Console.serialInfo(F("  DETECTED: Labware present on Rail 2 carriage"));
    } else {
        Console.serialInfo(F("  CLEAR: No labware on Rail 2 carriage"));
    }
    
    Console.serialInfo(F("  RAIL2_STATE: Updated from carriage sensor after homing"));
}

bool attemptToEnableAutomationAfterHoming() {
    Console.serialInfo(F("AUTO_ENABLE: Evaluating automation enablement after homing"));
    
    // Check for dual labware conflicts
    labwareSystem.dualLabwareConflict = (labwareSystem.rail1.hasLabware && labwareSystem.rail2.hasLabware);
    
    if (labwareSystem.dualLabwareConflict) {
        labwareSystem.automationEnabled = false;
        Console.serialWarning(F("  CONFLICT: Dual labware detected - automation remains disabled"));
        Console.serialInfo(F("  SOLUTION: Use manual rail commands to resolve, then 'labware audit'"));
        return false;
    } else {
        labwareSystem.automationEnabled = true;
        labwareSystem.lastSystemAudit = millis(); // Mark as validated
        Console.acknowledge(F("  SUCCESS: Automation ENABLED automatically after homing"));
        Console.serialInfo(F("  READY: Goto commands now available"));
        return true;
    }
}

//=============================================================================
// OPERATION TRACKING FUNCTIONS
//=============================================================================

void incrementPickupCounter() {
    labwareSystem.counters.pickupCount++;
    labwareSystem.counters.lastPickupTime = millis();
    Console.serialInfo((String(F("PICKUP_COUNTER: ")) + String(labwareSystem.counters.pickupCount) + F(" total pickups")).c_str());
}

void incrementDeliveryCounter() {
    labwareSystem.counters.deliveryCount++;
    labwareSystem.counters.lastDeliveryTime = millis();
    Console.serialInfo((String(F("DELIVERY_COUNTER: ")) + String(labwareSystem.counters.deliveryCount) + F(" total deliveries")).c_str());
}

void incrementCrossRailCounter() {
    labwareSystem.counters.crossRailCount++;
    labwareSystem.counters.lastCrossRailTime = millis();
    Console.serialInfo((String(F("CROSSRAIL_COUNTER: ")) + String(labwareSystem.counters.crossRailCount) + F(" total cross-rail transfers")).c_str());
}

void resetOperationCounters() {
    labwareSystem.counters.pickupCount = 0;
    labwareSystem.counters.deliveryCount = 0;
    labwareSystem.counters.crossRailCount = 0;
    labwareSystem.counters.startTime = millis();
    labwareSystem.counters.lastPickupTime = 0;
    labwareSystem.counters.lastDeliveryTime = 0;
    labwareSystem.counters.lastCrossRailTime = 0;
    Console.serialInfo(F("COUNTERS_RESET: All operation counters cleared"));
}

unsigned long getUptimeHours() {
    if (labwareSystem.counters.startTime == 0) {
        return 0; // Not yet initialized
    }
    return (millis() - labwareSystem.counters.startTime) / (1000UL * 60UL * 60UL);
}

void printOperationCounters() {
    Console.serialInfo(F("OPERATION_STATISTICS:"));
    Console.serialInfo((String(F("  Total Pickups: ")) + String(labwareSystem.counters.pickupCount)).c_str());
    Console.serialInfo((String(F("  Total Deliveries: ")) + String(labwareSystem.counters.deliveryCount)).c_str());
    Console.serialInfo((String(F("  Total Cross-Rail Transfers: ")) + String(labwareSystem.counters.crossRailCount)).c_str());
    
    // Calculate uptime since counter reset
    if (labwareSystem.counters.startTime > 0) {
        unsigned long uptimeSeconds = (millis() - labwareSystem.counters.startTime) / 1000;
        
        if (uptimeSeconds > 0) {
            // Use human-readable time formatting
            char timeBuffer[80];
            formatHumanReadableTime(uptimeSeconds, timeBuffer, sizeof(timeBuffer));
            Console.serialInfo((String(F("  Uptime Since Reset: ")) + String(timeBuffer) + F(" ago")).c_str());
        } else {
            Console.serialInfo(F("  Uptime Since Reset: Just started"));
        }
    } else {
        Console.serialInfo(F("  Uptime Since Reset: Not initialized"));
    }
    
    // Show time since last operations
    unsigned long currentTime = millis();
    
    // Time since last pickup
    if (labwareSystem.counters.lastPickupTime > 0) {
        unsigned long timeSincePickup = (currentTime - labwareSystem.counters.lastPickupTime) / 1000;
        char timeBuffer[80];
        formatHumanReadableTime(timeSincePickup, timeBuffer, sizeof(timeBuffer));
        Console.serialInfo((String(F("  Last Pickup: ")) + String(timeBuffer) + F(" ago")).c_str());
    } else {
        Console.serialInfo(F("  Last Pickup: Never"));
    }
    
    // Time since last delivery
    if (labwareSystem.counters.lastDeliveryTime > 0) {
        unsigned long timeSinceDelivery = (currentTime - labwareSystem.counters.lastDeliveryTime) / 1000;
        char timeBuffer[80];
        formatHumanReadableTime(timeSinceDelivery, timeBuffer, sizeof(timeBuffer));
        Console.serialInfo((String(F("  Last Delivery: ")) + String(timeBuffer) + F(" ago")).c_str());
    } else {
        Console.serialInfo(F("  Last Delivery: Never"));
    }
    
    // Time since last cross-rail transfer
    if (labwareSystem.counters.lastCrossRailTime > 0) {
        unsigned long timeSinceCrossRail = (currentTime - labwareSystem.counters.lastCrossRailTime) / 1000;
        char timeBuffer[80];
        formatHumanReadableTime(timeSinceCrossRail, timeBuffer, sizeof(timeBuffer));
        Console.serialInfo((String(F("  Last Cross-Rail Transfer: ")) + String(timeBuffer) + F(" ago")).c_str());
    } else {
        Console.serialInfo(F("  Last Cross-Rail Transfer: Never"));
    }
    
    // Show time since any operation (most recent work activity)
    unsigned long lastWorkTime = 0;
    if (labwareSystem.counters.lastPickupTime > lastWorkTime) lastWorkTime = labwareSystem.counters.lastPickupTime;
    if (labwareSystem.counters.lastDeliveryTime > lastWorkTime) lastWorkTime = labwareSystem.counters.lastDeliveryTime;
    
    if (lastWorkTime > 0) {
        unsigned long timeSinceLastWork = (currentTime - lastWorkTime) / 1000;
        char timeBuffer[80];
        formatHumanReadableTime(timeSinceLastWork, timeBuffer, sizeof(timeBuffer));
        Console.serialInfo((String(F("  Last Work Activity: ")) + String(timeBuffer) + F(" ago")).c_str());
    } else {
        Console.serialInfo(F("  Last Work Activity: Never"));
    }
}
