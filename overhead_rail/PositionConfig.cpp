#include "PositionConfig.h"
#include "Utils.h"
#include <SPI.h>
#include <SD.h>

//=============================================================================
// CONSOLE OUTPUT FORMAT STRINGS
//=============================================================================
// Consolidated format strings for cleaner, more concise position configuration messaging

// System status and configuration format strings
const char FMT_POSITION_SYSTEM_INIT[] PROGMEM = "Position system: %s";
const char FMT_POSITION_OPERATION[] PROGMEM = "Position %s: %s";
const char FMT_RAIL_POSITIONS_STATUS[] PROGMEM = "Rail %d: %d taught, %d default";
const char FMT_POSITION_TAUGHT_SIMPLE[] PROGMEM = "%s taught @ %.2fmm";

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_LOADED_POSITIONS[] PROGMEM = "Loaded %d taught positions, %d using factory defaults";
const char FMT_RAIL_NOT_HOMED[] PROGMEM = "Rail %d is not homed. Use homing commands first.";
const char FMT_WRONG_RAIL_POSITION[] PROGMEM = "Position %s belongs to Rail %d, not Rail %d";
const char FMT_POSITION_TAUGHT_ACK[] PROGMEM = "POSITION_TAUGHT_%s_%.2f";
const char FMT_TAUGHT_SAVED[] PROGMEM = "%s taught at %.2fmm and saved to SD card";
const char FMT_TAUGHT_NOT_SAVED[] PROGMEM = "%s taught at %.2fmm but failed to save to SD card";
const char FMT_SAVED_COUNT[] PROGMEM = "Saved %d taught positions to SD card";
const char FMT_FACTORY_DEFAULT[] PROGMEM = "  %s: %.2fmm";
const char FMT_RAIL_RESET_ACK[] PROGMEM = "RAIL_%d_POSITIONS_RESET";
const char FMT_RAIL_RESET_MSG[] PROGMEM = "Rail %d positions reset to factory defaults (%d positions)";
const char FMT_POSITION_STATUS[] PROGMEM = "  %s: %.2fmm %s";
const char FMT_POSITION_OUT_OF_RANGE[] PROGMEM = "Position %.2fmm is outside valid range (0 - %.0fmm)";

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Runtime position overrides (-1 means use factory default)
double runtimeRail1StagingMm = -1.0;
double runtimeRail1WC1PickupMm = -1.0;
double runtimeRail1WC2PickupMm = -1.0;
double runtimeRail1HandoffMm = -1.0;
double runtimeRail2HandoffMm = -1.0;
double runtimeRail2WC3PickupMm = -1.0;

bool useRuntimePositions = false;
bool sdCardInitialized = false;

// Teachable position lookup table
TeachablePosition teachablePositions[] = {
    {RAIL1_STAGING_POS, "RAIL1_STAGING", "Rail 1 Staging Position", 1, RAIL1_STAGING_POSITION, &runtimeRail1StagingMm},
    {RAIL1_WC1_PICKUP_DROPOFF_POS, "RAIL1_WC1", "Rail 1 Workcell 1 Pickup/Dropoff", 1, RAIL1_WC1_PICKUP_DROPOFF, &runtimeRail1WC1PickupMm},
    {RAIL1_WC2_PICKUP_DROPOFF_POS, "RAIL1_WC2", "Rail 1 Workcell 2 Pickup/Dropoff", 1, RAIL1_WC2_PICKUP_DROPOFF, &runtimeRail1WC2PickupMm},
    {RAIL1_HANDOFF_POS, "RAIL1_HANDOFF", "Rail 1 Handoff Position", 1, RAIL1_HANDOFF, &runtimeRail1HandoffMm},
    {RAIL2_HANDOFF_POS, "RAIL2_HANDOFF", "Rail 2 Handoff Position", 2, RAIL2_HANDOFF, &runtimeRail2HandoffMm},
    {RAIL2_WC3_PICKUP_DROPOFF_POS, "RAIL2_WC3", "Rail 2 Workcell 3 Pickup/Dropoff", 2, RAIL2_WC3_PICKUP_DROPOFF, &runtimeRail2WC3PickupMm}
};

const int NUM_TEACHABLE_POSITIONS = sizeof(teachablePositions) / sizeof(TeachablePosition);

//=============================================================================
// SYSTEM INITIALIZATION
//=============================================================================

bool initPositionConfig() {
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_POSITION_SYSTEM_INIT, "initializing...");
    Console.serialInfo(msg);
    
    // Initialize SD card
    sdCardInitialized = SD.begin();
    if (!sdCardInitialized) {
        sprintf_P(msg, FMT_POSITION_SYSTEM_INIT, "SD failed - using defaults");
        Console.serialWarning(msg);
        return false;
    }
    
    // Try to load positions from SD card
    if (loadPositionsFromSD()) {
        // Show summary of loaded positions
        int taughtCount = 0;
        for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
            if (*(teachablePositions[i].runtimeVariable) >= 0) {
                taughtCount++;
            }
        }
        
        sprintf_P(msg, FMT_LOADED_POSITIONS, 
                taughtCount, NUM_TEACHABLE_POSITIONS - taughtCount);
        Console.serialInfo(msg);
    } else {
        sprintf_P(msg, FMT_POSITION_SYSTEM_INIT, "using factory defaults");
        Console.serialInfo(msg);
    }
    
    return true;
}

//=============================================================================
// POSITION GETTER FUNCTIONS
//=============================================================================

double getRail1StagingMm() {
    if (useRuntimePositions && runtimeRail1StagingMm >= 0) {
        return runtimeRail1StagingMm;
    }
    return RAIL1_STAGING_POSITION;
}

double getRail1WC1PickupMm() {
    if (useRuntimePositions && runtimeRail1WC1PickupMm >= 0) {
        return runtimeRail1WC1PickupMm;
    }
    return RAIL1_WC1_PICKUP_DROPOFF;
}

double getRail1WC2PickupMm() {
    if (useRuntimePositions && runtimeRail1WC2PickupMm >= 0) {
        return runtimeRail1WC2PickupMm;
    }
    return RAIL1_WC2_PICKUP_DROPOFF;
}

double getRail1HandoffMm() {
    if (useRuntimePositions && runtimeRail1HandoffMm >= 0) {
        return runtimeRail1HandoffMm;
    }
    return RAIL1_HANDOFF;
}

double getRail2HandoffMm() {
    if (useRuntimePositions && runtimeRail2HandoffMm >= 0) {
        return runtimeRail2HandoffMm;
    }
    return RAIL2_HANDOFF;
}

double getRail2WC3PickupMm() {
    if (useRuntimePositions && runtimeRail2WC3PickupMm >= 0) {
        return runtimeRail2WC3PickupMm;
    }
    return RAIL2_WC3_PICKUP_DROPOFF;
}

double getTeachablePositionMm(PositionTarget target) {
    TeachablePosition* pos = getTeachablePositionInfo(target);
    if (pos) {
        if (useRuntimePositions && *(pos->runtimeVariable) >= 0) {
            return *(pos->runtimeVariable);
        }
        return pos->factoryDefault;
    }
    
    // Fallback to existing system for non-teachable positions
    return getPositionPulses(target) / (double)((target <= RAIL1_HANDOFF_POS) ? RAIL1_PULSES_PER_MM : RAIL2_PULSES_PER_MM);
}

//=============================================================================
// TEACHING FUNCTIONS
//=============================================================================

bool teachCurrentPosition(int rail, PositionTarget target) {
    // Validate rail number
    if (rail != 1 && rail != 2) {
        Console.serialError(F("Invalid rail number. Use 1 or 2"));
        return false;
    }
    
    // Check if rail is homed
    if (!isHomingComplete(rail)) {
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_RAIL_NOT_HOMED, rail);
        Console.serialError(msg);
        return false;
    }
    
    // Get teachable position info
    TeachablePosition* posInfo = getTeachablePositionInfo(target);
    if (!posInfo) {
        Console.serialError(F("Position is not teachable"));
        return false;
    }
    
    // Verify rail matches
    if (posInfo->rail != rail) {
        char msg[SMALL_MSG_SIZE];
        sprintf_P(msg, FMT_WRONG_RAIL_POSITION, 
                posInfo->name, posInfo->rail, rail);
        Console.serialError(msg);
        return false;
    }
    
    // Get current position
    double currentPos = getMotorPositionMm(rail);
    
    // Validate position is within travel limits
    if (!validateTaughtPosition(rail, currentPos, target)) {
        return false;
    }
    
    // Store the taught position
    *(posInfo->runtimeVariable) = currentPos;
    useRuntimePositions = true;
    
    // Auto-save to SD card immediately
    if (savePositionsToSD()) {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_POSITION_TAUGHT_ACK, posInfo->name, currentPos);
        Console.acknowledge(msg);
        
        sprintf_P(msg, FMT_POSITION_TAUGHT_SIMPLE, posInfo->description, currentPos);
        Console.serialInfo(msg);
    } else {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_POSITION_TAUGHT_ACK, posInfo->name, currentPos);
        Console.acknowledge(msg);
        
        sprintf_P(msg, FMT_POSITION_OPERATION, posInfo->description, "taught but save failed");
        Console.serialWarning(msg);
    }
    
    return true;
}

// Individual teaching functions for convenience
bool teachRail1Staging() {
    return teachCurrentPosition(1, RAIL1_STAGING_POS);
}

bool teachRail1WC1Pickup() {
    return teachCurrentPosition(1, RAIL1_WC1_PICKUP_DROPOFF_POS);
}

bool teachRail1WC2Pickup() {
    return teachCurrentPosition(1, RAIL1_WC2_PICKUP_DROPOFF_POS);
}

bool teachRail1Handoff() {
    return teachCurrentPosition(1, RAIL1_HANDOFF_POS);
}

bool teachRail2Handoff() {
    return teachCurrentPosition(2, RAIL2_HANDOFF_POS);
}

bool teachRail2WC3Pickup() {
    return teachCurrentPosition(2, RAIL2_WC3_PICKUP_DROPOFF_POS);
}

//=============================================================================
// BULK OPERATIONS
//=============================================================================

bool teachResetAllPositions() {
    // Clear all runtime positions
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        *(teachablePositions[i].runtimeVariable) = -1.0;
    }
    useRuntimePositions = false;
    
    Console.acknowledge(F("ALL_POSITIONS_RESET"));
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_POSITION_OPERATION, "ALL", "reset to factory defaults");
    Console.serialInfo(msg);
    
    return true;
}

bool teachResetRail(int rail) {
    if (rail != 1 && rail != 2) {
        Console.serialError(F("Invalid rail number. Use 1 or 2"));
        return false;
    }
    
    int resetCount = 0;
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == rail) {
            *(teachablePositions[i].runtimeVariable) = -1.0;
            resetCount++;
        }
    }
    
    char msg[SMALL_MSG_SIZE];
    sprintf_P(msg, FMT_RAIL_RESET_ACK, rail);
    Console.acknowledge(msg);
    
    sprintf_P(msg, FMT_RAIL_RESET_MSG, 
            rail, resetCount);
    Console.serialInfo(msg);
    
    return true;
}

void teachShowStatus() {
    Console.acknowledge(F("TEACH_STATUS"));
    
    // Show rail summaries
    char msg[MEDIUM_MSG_SIZE];
    
    // Rail 1 summary
    int rail1Taught = 0, rail1Default = 0;
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == 1) {
            if (*(teachablePositions[i].runtimeVariable) >= 0) {
                rail1Taught++;
            } else {
                rail1Default++;
            }
        }
    }
    sprintf_P(msg, FMT_RAIL_POSITIONS_STATUS, 1, rail1Taught, rail1Default);
    Console.serialInfo(msg);
    
    // Rail 2 summary
    int rail2Taught = 0, rail2Default = 0;
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == 2) {
            if (*(teachablePositions[i].runtimeVariable) >= 0) {
                rail2Taught++;
            } else {
                rail2Default++;
            }
        }
    }
    sprintf_P(msg, FMT_RAIL_POSITIONS_STATUS, 2, rail2Taught, rail2Default);
    Console.serialInfo(msg);
    
    // SD card status
    sprintf_P(msg, FMT_POSITION_OPERATION, "SD Card", 
        isSDCardAvailable() ? "available" : "not available");
    Console.serialInfo(msg);
}

void teachShowRail(int rail) {
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == rail) {
            bool isTaught = (*(teachablePositions[i].runtimeVariable) >= 0);
            double currentValue = isTaught ? 
                *(teachablePositions[i].runtimeVariable) : 
                teachablePositions[i].factoryDefault;
            
            char msg[MEDIUM_MSG_SIZE];
            sprintf_P(msg, FMT_POSITION_STATUS, 
                    teachablePositions[i].description,
                    currentValue,
                    isTaught ? "(TAUGHT)" : "(DEFAULT)");
            Console.println(msg);
        }
    }
}

//=============================================================================
// VALIDATION FUNCTIONS
//=============================================================================

bool validateTaughtPosition(int rail, double positionMm, PositionTarget target) {
    // Check basic travel limits
    double maxTravel = (rail == 1) ? RAIL1_MAX_TRAVEL_MM : RAIL2_MAX_TRAVEL_MM;
    
    if (positionMm < 0 || positionMm > maxTravel) {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_POSITION_OUT_OF_RANGE, 
                positionMm, maxTravel);
        Console.serialError(msg);
        return false;
    }
    
    // Add position-specific validation if needed
    TeachablePosition* pos = getTeachablePositionInfo(target);
    if (pos) {
        // Ensure handoff positions are compatible between rails
        if (target == RAIL1_HANDOFF_POS && runtimeRail2HandoffMm >= 0) {
            // Could add validation that handoff positions are reasonable relative to each other
        }
        
        // Could add other position-specific validations here
        // For example, ensure staging position is reasonable, etc.
    }
    
    return true;
}

bool isPositionTaught(PositionTarget target) {
    TeachablePosition* pos = getTeachablePositionInfo(target);
    if (pos) {
        return *(pos->runtimeVariable) >= 0;
    }
    return false;
}

//=============================================================================
// SD CARD OPERATIONS
//=============================================================================

bool savePositionsToSD() {
    if (!isSDCardAvailable()) {
        Console.serialError(F("SD card not available"));
        return false;
    }
    
    // Create backup first if original exists
    if (SD.exists(CONFIG_FILE_NAME)) {
        backupPositionsToSD();
    }
    
    File configFile = SD.open(CONFIG_FILE_NAME, FILE_WRITE);
    if (!configFile) {
        Console.serialError(F("Failed to open config file for writing"));
        return false;
    }
    
    // Write header
    configFile.println("# Overhead Rail Position Configuration");
    configFile.println("# Generated automatically - do not edit manually");
    configFile.print("# Saved at: ");
    configFile.println(millis());
    configFile.println();
    
    // Write Rail 1 positions
    configFile.println("# Rail 1 Positions");
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == 1 && *(teachablePositions[i].runtimeVariable) >= 0) {
            configFile.print(teachablePositions[i].name);
            configFile.print("=");
            configFile.println(*(teachablePositions[i].runtimeVariable), 2);
        }
    }
    
    configFile.println();
    
    // Write Rail 2 positions
    configFile.println("# Rail 2 Positions");
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == 2 && *(teachablePositions[i].runtimeVariable) >= 0) {
            configFile.print(teachablePositions[i].name);
            configFile.print("=");
            configFile.println(*(teachablePositions[i].runtimeVariable), 2);
        }
    }
    
    configFile.println();
    configFile.print("SAVED_TIME=");
    configFile.println(millis());
    
    configFile.flush();
    configFile.close();
    
    // Verify the file was created
    if (SD.exists(CONFIG_FILE_NAME)) {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_POSITION_OPERATION, "config", "saved successfully");
        Console.serialInfo(msg);
        return true;
    } else {
        Console.serialError(F("Config file not found after writing"));
        return false;
    }
}

bool loadPositionsFromSD() {
    if (!isSDCardAvailable()) {
        return false;
    }
    
    File configFile = SD.open(CONFIG_FILE_NAME);
    if (!configFile) {
        return false;
    }
    
    String line;
    bool foundPositions = false;
    
    while (configFile.available()) {
        line = configFile.readStringUntil('\n');
        line.trim();
        
        // Skip comments and empty lines
        if (line.startsWith("#") || line.length() == 0) {
            continue;
        }
        
        // Parse position lines
        int equalsIndex = line.indexOf('=');
        if (equalsIndex > 0) {
            String positionName = line.substring(0, equalsIndex);
            double positionValue = line.substring(equalsIndex + 1).toFloat();
            
            // Find matching position in our table
            for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
                if (positionName.equals(teachablePositions[i].name)) {
                    *(teachablePositions[i].runtimeVariable) = positionValue;
                    foundPositions = true;
                    break;
                }
            }
        }
    }
    
    configFile.close();
    
    if (foundPositions) {
        useRuntimePositions = true;
        return true;
    }
    
    return false;
}

bool backupPositionsToSD() {
    if (!isSDCardAvailable() || !SD.exists(CONFIG_FILE_NAME)) {
        return false;
    }
    
    // Remove old backup
    if (SD.exists(CONFIG_BACKUP_NAME)) {
        SD.remove(CONFIG_BACKUP_NAME);
    }
    
    // Copy current config to backup
    File source = SD.open(CONFIG_FILE_NAME);
    File backup = SD.open(CONFIG_BACKUP_NAME, FILE_WRITE);
    
    if (!source || !backup) {
        if (source) source.close();
        if (backup) backup.close();
        return false;
    }
    
    while (source.available()) {
        backup.write(source.read());
    }
    
    source.close();
    backup.close();
    
    Console.serialInfo(F("Position config backed up"));
    return true;
}

bool isSDCardAvailable() {
    return sdCardInitialized;
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

const char* getPositionTargetName(PositionTarget target) {
    TeachablePosition* pos = getTeachablePositionInfo(target);
    return pos ? pos->name : "UNKNOWN";
}

const char* getPositionTargetDescription(PositionTarget target) {
    TeachablePosition* pos = getTeachablePositionInfo(target);
    return pos ? pos->description : "Unknown Position";
}

TeachablePosition* getTeachablePositionInfo(PositionTarget target) {
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].target == target) {
            return &teachablePositions[i];
        }
    }
    return nullptr;
}