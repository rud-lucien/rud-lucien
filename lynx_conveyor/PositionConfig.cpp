#include "PositionConfig.h"
#include <SPI.h>
#include <SD.h>

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Runtime position overrides (-1 means use default)
double runtimePosition1Mm = -1.0;
double runtimePosition2Mm = -1.0;
double runtimePosition3Mm = -1.0;
bool useRuntimePositions = false;

//=============================================================================
// SYSTEM INITIALIZATION
//=============================================================================

bool initPositionConfig() {
    Console.serialInfo(F("Initializing position configuration system..."));
    
    // Initialize SD card
    if (!SD.begin()) {
        Console.serialWarning(F("SD card initialization failed - using default positions"));
        return false;
    }
    
    Console.serialInfo(F("SD card initialized successfully"));
    
    // Try to load positions from SD card
    if (loadPositionsFromSD()) {
        Console.serialInfo(F("Taught positions loaded from SD card"));
    } else {
        Console.serialInfo(F("No taught positions found - using factory defaults"));
    }
    
    return true;
}

//=============================================================================
// POSITION GETTER FUNCTIONS
//=============================================================================

double getPosition1Mm() {
    if (useRuntimePositions && runtimePosition1Mm >= 0) {
        return runtimePosition1Mm;
    }
    return POSITION_1_MM;  // Factory default
}

double getPosition2Mm() {
    if (useRuntimePositions && runtimePosition2Mm >= 0) {
        return runtimePosition2Mm;
    }
    return POSITION_2_MM;  // Factory default
}

double getPosition3Mm() {
    if (useRuntimePositions && runtimePosition3Mm >= 0) {
        return runtimePosition3Mm;
    }
    return POSITION_3_MM;  // Factory default
}

//=============================================================================
// TEACHING FUNCTIONS
//=============================================================================

bool teachPosition1() {   
    if (!isHomed) {
        Console.error(F("Motor is not homed. Use 'motor,home' first."));
        Console.serialInfo(F("Teaching positions requires a proper reference point"));
        return false;
    }

    // Get current position
    double currentPos = getMotorPositionMm();
    
    // Validate position is within reasonable bounds
    if (currentPos < 0 || currentPos > MAX_TRAVEL_MM) {
        Console.error(F("Current position is out of valid range"));
        return false;
    }
    
    // Store the taught position
    runtimePosition1Mm = currentPos;
    useRuntimePositions = true;
    
    // Auto-save to SD card immediately
    if (savePositionsToSD()) {
        char msg[100];
        sprintf(msg, "POSITION_1_TAUGHT_%.2f", currentPos);
        Console.acknowledge(msg);
        Console.serialInfo(F("Position 1 taught and saved to SD card"));
    } else {
        char msg[100];
        sprintf(msg, "POSITION_1_TAUGHT_%.2f", currentPos);
        Console.acknowledge(msg);
        Console.serialWarning(F("Position 1 taught but failed to save to SD card"));
    }
    
    return true;
}

bool teachPosition2() {
    if (!isHomed) {
        Console.error(F("Motor is not homed. Use 'motor,home' first."));
        Console.serialInfo(F("Teaching positions requires a proper reference point"));
        return false;
    }

    double currentPos = getMotorPositionMm();
    
    if (currentPos < 0 || currentPos > MAX_TRAVEL_MM) {
        Console.error(F("Current position is out of valid range"));
        return false;
    }
    
    runtimePosition2Mm = currentPos;
    useRuntimePositions = true;
    
    // Auto-save to SD card immediately
    if (savePositionsToSD()) {
        char msg[100];
        sprintf(msg, "POSITION_2_TAUGHT_%.2f", currentPos);
        Console.acknowledge(msg);
        Console.serialInfo(F("Position 2 taught and saved to SD card"));
    } else {
        char msg[100];
        sprintf(msg, "POSITION_2_TAUGHT_%.2f", currentPos);
        Console.acknowledge(msg);
        Console.serialWarning(F("Position 2 taught but failed to save to SD card"));
    }
    
    return true;
}

bool teachPosition3() {
    if (!isHomed) {
        Console.error(F("Motor is not homed. Use 'motor,home' first."));
        Console.serialInfo(F("Teaching positions requires a proper reference point"));
        return false;
    }

    double currentPos = getMotorPositionMm();
    
    if (currentPos < 0 || currentPos > MAX_TRAVEL_MM) {
        Console.error(F("Current position is out of valid range"));
        return false;
    }
    
    runtimePosition3Mm = currentPos;
    useRuntimePositions = true;
    
    // Auto-save to SD card immediately
    if (savePositionsToSD()) {
        char msg[100];
        sprintf(msg, "POSITION_3_TAUGHT_%.2f", currentPos);
        Console.acknowledge(msg);
        Console.serialInfo(F("Position 3 taught and saved to SD card"));
    } else {
        char msg[100];
        sprintf(msg, "POSITION_3_TAUGHT_%.2f", currentPos);
        Console.acknowledge(msg);
        Console.serialWarning(F("Position 3 taught but failed to save to SD card"));
    }
    
    return true;
}

bool teachSavePositions() {
    if (savePositionsToSD()) {
        Console.acknowledge(F("POSITIONS_SAVED"));
        Console.serialInfo(F("All taught positions saved to SD card"));
        char msg[200];
        sprintf(msg, "Saved: P1=%.2f, P2=%.2f, P3=%.2f", 
                getPosition1Mm(), getPosition2Mm(), getPosition3Mm());
        Console.serialInfo(msg);
        return true;
    } else {
        Console.error(F("Failed to save positions to SD card"));
        return false;
    }
}

bool teachResetPositions() {
    // Clear runtime positions
    runtimePosition1Mm = -1.0;
    runtimePosition2Mm = -1.0;
    runtimePosition3Mm = -1.0;
    useRuntimePositions = false;
    
    Console.acknowledge(F("POSITIONS_RESET"));
    Console.serialInfo(F("All positions reset to factory defaults"));
    
    char msg[200];
    sprintf(msg, "Default positions: P1=%.2f, P2=%.2f, P3=%.2f", 
            POSITION_1_MM, POSITION_2_MM, POSITION_3_MM);
    Console.serialInfo(msg);
    
    return true;
}

void teachShowStatus() {
    Console.acknowledge(F("TEACH_STATUS"));
    
    char msg[100];
    
    // Show current active positions
    sprintf(msg, "Position 1: %.2f mm %s", getPosition1Mm(), 
            (useRuntimePositions && runtimePosition1Mm >= 0) ? "(TAUGHT)" : "(DEFAULT)");
    Console.println(msg);
    
    sprintf(msg, "Position 2: %.2f mm %s", getPosition2Mm(), 
            (useRuntimePositions && runtimePosition2Mm >= 0) ? "(TAUGHT)" : "(DEFAULT)");
    Console.println(msg);
    
    sprintf(msg, "Position 3: %.2f mm %s", getPosition3Mm(), 
            (useRuntimePositions && runtimePosition3Mm >= 0) ? "(TAUGHT)" : "(DEFAULT)");
    Console.println(msg);
    
    // Show SD card status
    Console.print(F("SD Card: "));
    Console.println(isSDCardAvailable() ? F("AVAILABLE") : F("NOT AVAILABLE"));
    
    // Show if config file exists
    if (isSDCardAvailable()) {
        Console.print(F("Config file: "));
        Console.println(SD.exists(CONFIG_FILE_NAME) ? F("EXISTS") : F("NOT FOUND"));
    }
}

//=============================================================================
// INTERNAL SD CARD OPERATIONS
//=============================================================================

bool savePositionsToSD() {
    if (!isSDCardAvailable()) {
        Console.serialError(F("SD card not available"));
        return false;
    }
    
    File configFile = SD.open(CONFIG_FILE_NAME, FILE_WRITE);
    if (!configFile) {
        Console.serialError(F("Failed to open config file for writing"));
        return false;
    }
    
    // Write header
    configFile.println("# Lynx Conveyor Position Configuration");
    configFile.println("# Generated automatically - do not edit manually");
    
    // Only write actually taught positions
    if (runtimePosition1Mm >= 0) {
        configFile.print("POSITION_1_MM=");
        configFile.println(runtimePosition1Mm, 2);
    }
    
    if (runtimePosition2Mm >= 0) {
        configFile.print("POSITION_2_MM=");
        configFile.println(runtimePosition2Mm, 2);
    }
    
    if (runtimePosition3Mm >= 0) {
        configFile.print("POSITION_3_MM=");
        configFile.println(runtimePosition3Mm, 2);
    }
    
    configFile.print("SAVED_TIME=");
    configFile.println(millis());
    
    configFile.close();
    return true;
}

bool loadPositionsFromSD() {
    if (!isSDCardAvailable()) {
        return false;
    }
    
    File configFile = SD.open(CONFIG_FILE_NAME);
    if (!configFile) {
        // File doesn't exist - not an error, just use defaults
        return false;
    }
    
    Console.serialInfo(F("Loading positions from SD card..."));
    
    // Read file line by line
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
        if (line.startsWith("POSITION_1_MM=")) {
            runtimePosition1Mm = line.substring(14).toFloat();
            foundPositions = true;
        }
        else if (line.startsWith("POSITION_2_MM=")) {
            runtimePosition2Mm = line.substring(14).toFloat();
            foundPositions = true;
        }
        else if (line.startsWith("POSITION_3_MM=")) {
            runtimePosition3Mm = line.substring(14).toFloat();
            foundPositions = true;
        }
    }
    
    configFile.close();
    
    if (foundPositions) {
        useRuntimePositions = true;
        char msg[200];
        sprintf(msg, "Loaded positions: P1=%.2f, P2=%.2f, P3=%.2f", 
                getPosition1Mm(), getPosition2Mm(), getPosition3Mm());
        Console.serialInfo(msg);
        return true;
    }
    
    return false;
}

bool isSDCardAvailable() {
    // Simple check - try to open the SD card
    return SD.begin();
}