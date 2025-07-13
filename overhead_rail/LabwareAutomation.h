#ifndef LABWARE_AUTOMATION_H
#define LABWARE_AUTOMATION_H

#include "Arduino.h"

//=============================================================================
// LABWARE AUTOMATION CONSTANTS
//=============================================================================

// Location definitions for labware tracking
enum Location {
    LOCATION_UNKNOWN = 0,
    LOCATION_WC1,
    LOCATION_WC2, 
    LOCATION_WC3,
    LOCATION_HANDOFF,
    LOCATION_STAGING
};

// Confidence levels for labware state tracking
enum ConfidenceLevel {
    CONFIDENCE_UNKNOWN = 0,    // State uncertain due to fault/reset
    CONFIDENCE_LOW,            // Rail 1 between checkpoints (inferred state)
    CONFIDENCE_MEDIUM,         // Rail 1 at sensor checkpoint
    CONFIDENCE_HIGH            // Rail 2 carriage sensor, recent sensor reading
};

//=============================================================================
// LABWARE STATE STRUCTURES
//=============================================================================

// Rail 1 (Long Rail ~8m) - Checkpoint-Based Tracking
struct Rail1LabwareState {
    bool hasLabware;              // Current labware status
    Location lastKnownLocation;   // WC1, WC2, or HANDOFF
    Location labwareSource;       // Where did this labware originate?
    bool validated;               // Confirmed by sensor reading?
    bool uncertainDueToFault;     // State uncertain after fault?
    unsigned long lastValidated;  // Timestamp of last sensor confirmation
    ConfidenceLevel confidence;   // Current confidence in this state
};

// Rail 2 (Short Rail) - Continuous Tracking  
struct Rail2LabwareState {
    bool hasLabware;              // Real-time sensor reading
    Location labwareSource;       // Where did this labware originate?
    unsigned long lastValidated;  // Always current (continuous sensor)
    ConfidenceLevel confidence;   // Always HIGH due to continuous sensor
};

// Global System State
struct SystemLabwareState {
    Rail1LabwareState rail1;
    Rail2LabwareState rail2;
    bool automationEnabled;       // Can execute goto commands?
    bool dualLabwareConflict;     // Both rails have labware?
    unsigned long lastSystemAudit; // When was last full system audit?
};

//=============================================================================
// GLOBAL LABWARE STATE INSTANCE
//=============================================================================

// Global instance of the labware automation state
extern SystemLabwareState labwareSystem;

//=============================================================================
// LABWARE AUTOMATION FUNCTION DECLARATIONS
//=============================================================================

//-----------------------------------------------------------------------------
// State Management Functions
//-----------------------------------------------------------------------------
void initLabwareSystem();
void updateLabwareSystemState();
void clearLabwareState();
void setLabwareStateUncertain(int railNumber, const char* reason);

//-----------------------------------------------------------------------------
// State Query Functions  
//-----------------------------------------------------------------------------
bool isLabwareSystemReady();
bool hasLabwareConflict();
ConfidenceLevel getLabwareConfidence(int railNumber);
const char* getLocationName(Location loc);
const char* getConfidenceName(ConfidenceLevel confidence);

//-----------------------------------------------------------------------------
// Sensor Integration Functions
//-----------------------------------------------------------------------------
void updateRail1LabwareFromSensor(Location sensorLocation);
void updateRail2LabwareFromSensor();
bool validateLabwareStateAtLocation(Location location);

//-----------------------------------------------------------------------------
// Audit and Recovery Functions
//-----------------------------------------------------------------------------
bool performLabwareAudit();
Location findNearestSafeSensorForRail1();
bool moveRail1ToNearestSensorAndValidate();

//-----------------------------------------------------------------------------
// Status Reporting Functions
//-----------------------------------------------------------------------------
void printLabwareSystemStatus();
void printLabwareStateDetails();
void printSensorReadings();

#endif // LABWARE_AUTOMATION_H
