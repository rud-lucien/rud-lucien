#include "PositionConfig.h"
#include <SPI.h>
#include <SD.h>

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
double runtimeRail1HomeMm = -1.0;
double runtimeRail1StagingMm = -1.0;
double runtimeRail1WC1PickupMm = -1.0;
double runtimeRail1WC2PickupMm = -1.0;
double runtimeRail2HomeMm = -1.0;
double runtimeRail2HandoffMm = -1.0;

bool useRuntimePositions = false;
bool sdCardInitialized = false;

// Teachable position lookup table
TeachablePosition teachablePositions[] = {
    {RAIL1_HOME_POS, "RAIL1_HOME", "Rail 1 Home (Handoff)", 1, RAIL1_HOME_POSITION, &runtimeRail1HomeMm},
    {RAIL1_STAGING_POS, "RAIL1_STAGING", "Rail 1 Staging Position", 1, RAIL1_STAGING_POSITION, &runtimeRail1StagingMm},
    {RAIL1_WC1_PICKUP_DROPOFF_POS, "RAIL1_WC1", "Rail 1 Workcell 1 Pickup/Dropoff", 1, RAIL1_WC1_PICKUP_DROPOFF, &runtimeRail1WC1PickupMm},
    {RAIL1_WC2_PICKUP_DROPOFF_POS, "RAIL1_WC2", "Rail 1 Workcell 2 Pickup/Dropoff", 1, RAIL1_WC2_PICKUP_DROPOFF, &runtimeRail1WC2PickupMm},
    {RAIL2_HOME_POS, "RAIL2_HOME", "Rail 2 Home (WC3)", 2, RAIL2_HOME_POSITION, &runtimeRail2HomeMm},
    {RAIL2_HANDOFF_POS, "RAIL2_HANDOFF", "Rail 2 Handoff Position", 2, RAIL2_HANDOFF, &runtimeRail2HandoffMm}
};

const int NUM_TEACHABLE_POSITIONS = sizeof(teachablePositions) / sizeof(TeachablePosition);

//=============================================================================
// SYSTEM INITIALIZATION
//=============================================================================

bool initPositionConfig() {
    Console.serialInfo(F("Initializing dual-rail position configuration system..."));
    
    // Initialize SD card
    sdCardInitialized = SD.begin();
    if (!sdCardInitialized) {
        Console.serialWarning(F("SD card initialization failed - using factory defaults"));
        Console.serialInfo(F("Factory defaults: Rail1 positions, Rail2 positions available"));
        return false;
    }
    
    Console.serialInfo(F("SD card initialized successfully"));
    
    // Try to load positions from SD card
    if (loadPositionsFromSD()) {
        Console.serialInfo(F("Taught positions loaded from SD card"));
        
        // Show summary of loaded positions
        int taughtCount = 0;
        for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
            if (*(teachablePositions[i].runtimeVariable) >= 0) {
                taughtCount++;
            }
        }
        
        char msg[80];
        sprintf_P(msg, FMT_LOADED_POSITIONS, 
                taughtCount, NUM_TEACHABLE_POSITIONS - taughtCount);
        Console.serialInfo(msg);
    } else {
        Console.serialInfo(F("No taught positions found - using factory defaults"));
    }
    
    return true;
}

//=============================================================================
// POSITION GETTER FUNCTIONS
//=============================================================================

double getRail1HomeMm() {
    if (useRuntimePositions && runtimeRail1HomeMm >= 0) {
        return runtimeRail1HomeMm;
    }
    return RAIL1_HOME_POSITION;
}

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

double getRail2HomeMm() {
    if (useRuntimePositions && runtimeRail2HomeMm >= 0) {
        return runtimeRail2HomeMm;
    }
    return RAIL2_HOME_POSITION;
}

double getRail2HandoffMm() {
    if (useRuntimePositions && runtimeRail2HandoffMm >= 0) {
        return runtimeRail2HandoffMm;
    }
    return RAIL2_HANDOFF;
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
        char msg[80];
        sprintf_P(msg, FMT_RAIL_NOT_HOMED, rail);
        Console.serialError(msg);
        Console.serialInfo(F("Teaching positions requires a proper reference point"));
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
        char msg[80];
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
        char msg[120];
        sprintf_P(msg, FMT_POSITION_TAUGHT_ACK, posInfo->name, currentPos);
        Console.acknowledge(msg);
        
        sprintf_P(msg, FMT_TAUGHT_SAVED, 
                posInfo->description, currentPos);
        Console.serialInfo(msg);
    } else {
        char msg[120];
        sprintf_P(msg, FMT_POSITION_TAUGHT_ACK, posInfo->name, currentPos);
        Console.acknowledge(msg);
        
        sprintf_P(msg, FMT_TAUGHT_NOT_SAVED, 
                posInfo->description, currentPos);
        Console.serialWarning(msg);
    }
    
    return true;
}

// Individual teaching functions for convenience
bool teachRail1Home() {
    return teachCurrentPosition(1, RAIL1_HOME_POS);
}

bool teachRail1Staging() {
    return teachCurrentPosition(1, RAIL1_STAGING_POS);
}

bool teachRail1WC1Pickup() {
    return teachCurrentPosition(1, RAIL1_WC1_PICKUP_DROPOFF_POS);
}

bool teachRail1WC2Pickup() {
    return teachCurrentPosition(1, RAIL1_WC2_PICKUP_DROPOFF_POS);
}

bool teachRail2Home() {
    return teachCurrentPosition(2, RAIL2_HOME_POS);
}

bool teachRail2Handoff() {
    return teachCurrentPosition(2, RAIL2_HANDOFF_POS);
}

//=============================================================================
// BULK OPERATIONS
//=============================================================================

bool teachSaveAllPositions() {
    if (savePositionsToSD()) {
        Console.acknowledge(F("ALL_POSITIONS_SAVED"));
        Console.serialInfo(F("All taught positions saved to SD card"));
        
        // Show summary
        int taughtCount = 0;
        for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
            if (*(teachablePositions[i].runtimeVariable) >= 0) {
                taughtCount++;
            }
        }
        
        char msg[100];
        sprintf_P(msg, FMT_SAVED_COUNT, taughtCount);
        Console.serialInfo(msg);
        return true;
    } else {
        Console.serialError(F("Failed to save positions to SD card"));
        return false;
    }
}

bool teachResetAllPositions() {
    // Clear all runtime positions
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        *(teachablePositions[i].runtimeVariable) = -1.0;
    }
    useRuntimePositions = false;
    
    Console.acknowledge(F("ALL_POSITIONS_RESET"));
    Console.serialInfo(F("All positions reset to factory defaults"));
    
    // Show factory defaults
    Console.serialInfo(F("Factory default positions:"));
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        char msg[120];
        sprintf_P(msg, FMT_FACTORY_DEFAULT, 
                teachablePositions[i].description, 
                teachablePositions[i].factoryDefault);
        Console.serialInfo(msg);
    }
    
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
    
    char msg[80];
    sprintf_P(msg, FMT_RAIL_RESET_ACK, rail);
    Console.acknowledge(msg);
    
    sprintf_P(msg, FMT_RAIL_RESET_MSG, 
            rail, resetCount);
    Console.serialInfo(msg);
    
    return true;
}

void teachShowStatus() {
    Console.acknowledge(F("TEACH_STATUS"));
    
    Console.serialInfo(F("\n===== POSITION CONFIGURATION STATUS ====="));
    
    // Show Rail 1 positions
    Console.serialInfo(F("\nRAIL 1 POSITIONS:"));
    teachShowRail(1);
    
    // Show Rail 2 positions
    Console.serialInfo(F("\nRAIL 2 POSITIONS:"));
    teachShowRail(2);
    
    // Show SD card status
    Console.serialInfo(F("\nSD CARD STATUS:"));
    Console.print(F("  SD Card: "));
    Console.println(isSDCardAvailable() ? F("AVAILABLE") : F("NOT AVAILABLE"));
    
    if (isSDCardAvailable()) {
        Console.print(F("  Config file: "));
        Console.println(SD.exists(CONFIG_FILE_NAME) ? F("EXISTS") : F("NOT FOUND"));
        
        if (SD.exists(CONFIG_BACKUP_NAME)) {
            Console.print(F("  Backup file: EXISTS"));
        }
    }
    
    Console.serialInfo(F("==========================================\n"));
}

void teachShowRail(int rail) {
    for (int i = 0; i < NUM_TEACHABLE_POSITIONS; i++) {
        if (teachablePositions[i].rail == rail) {
            bool isTaught = (*(teachablePositions[i].runtimeVariable) >= 0);
            double currentValue = isTaught ? 
                *(teachablePositions[i].runtimeVariable) : 
                teachablePositions[i].factoryDefault;
            
            char msg[150];
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
        char msg[100];
        sprintf_P(msg, FMT_POSITION_OUT_OF_RANGE, 
                positionMm, maxTravel);
        Console.serialError(msg);
        return false;
    }
    
    // Add position-specific validation if needed
    TeachablePosition* pos = getTeachablePositionInfo(target);
    if (pos) {
        // For example, ensure home position is near 0
        if (target == RAIL1_HOME_POS || target == RAIL2_HOME_POS) {
            if (positionMm > 100) {  // Allow some tolerance for home position
                Console.serialWarning(F("Home position is far from 0mm - verify this is correct"));
            }
        }
        
        // Ensure handoff positions are compatible between rails
        if (target == RAIL1_HANDOFF_POS && runtimeRail2HandoffMm >= 0) {
            // Could add validation that handoff positions are reasonable relative to each other
        }
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
    
    Console.serialInfo(F("Saving positions to SD card..."));
    
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
        Console.serialInfo(F("Position config file saved successfully"));
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
    
    Console.serialInfo(F("Loading positions from SD card..."));
    
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
        Console.serialInfo(F("Taught positions loaded from SD card"));
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