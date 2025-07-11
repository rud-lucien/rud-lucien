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
bool sdCardInitialized = false; 

//=============================================================================
// SYSTEM INITIALIZATION
//=============================================================================

bool initPositionConfig() {
    Console.serialInfo(F("Initializing position configuration system..."));
    
    // Initialize SD card - save result to global flag
    sdCardInitialized = SD.begin();
    if (!sdCardInitialized) {
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
    
    // Show what file we're working with
    Console.serialInfo(F("Config filename: "));
    Console.serialInfo(CONFIG_FILE_NAME);
    Console.serialInfo(F("Opening config file for writing..."));
    
    // Attempt to create/open the file
    File configFile = SD.open(CONFIG_FILE_NAME, FILE_WRITE);
    if (!configFile) {
        Console.serialError(F("Failed to open config file for writing"));
        
        // Diagnostic test with a simple file
        Console.serialInfo(F("Trying test file write..."));
        File testFile = SD.open("test.txt", FILE_WRITE);
        if (!testFile) {
            Console.serialError(F("SD card test file also failed - card may be write-protected"));
        } else {
            testFile.println("Test write");
            testFile.close();
            Console.serialInfo(F("Test file created successfully - issue is specific to config file"));
        }
        
        return false;
    }
    
    Console.serialInfo(F("Config file opened successfully, writing data..."));
    
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
    
    Console.serialInfo(F("Flushing and closing file..."));
    
    // Explicitly flush and close the file
    configFile.flush();
    configFile.close();
    
    // Verify the file was created
    if (SD.exists(CONFIG_FILE_NAME)) {
        Console.serialInfo(F("Position config file saved successfully"));
        return true;
    } else {
        Console.serialError(F("Config file not found after writing - SD card may have issues"));
        return false;
    }
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
    
    // Read file line by line using fixed buffer
    char line[128];
    bool foundPositions = false;
    
    while (configFile.available()) {
        // Read line into fixed buffer
        int i = 0;
        while (configFile.available() && i < sizeof(line) - 1) {
            char c = configFile.read();
            if (c == '\n' || c == '\r') break;
            line[i++] = c;
        }
        line[i] = '\0';
        
        // Trim leading/trailing spaces
        char* trimmed = line;
        while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
        int len = strlen(trimmed);
        while (len > 0 && (trimmed[len-1] == ' ' || trimmed[len-1] == '\t' || trimmed[len-1] == '\r')) {
            trimmed[--len] = '\0';
        }
        
        // Skip comments and empty lines
        if (trimmed[0] == '#' || strlen(trimmed) == 0) {
            continue;
        }
        
        // Parse position lines
        if (strncmp(trimmed, "POSITION_1_MM=", 14) == 0) {
            runtimePosition1Mm = atof(trimmed + 14);
            foundPositions = true;
        }
        else if (strncmp(trimmed, "POSITION_2_MM=", 14) == 0) {
            runtimePosition2Mm = atof(trimmed + 14);
            foundPositions = true;
        }
        else if (strncmp(trimmed, "POSITION_3_MM=", 14) == 0) {
            runtimePosition3Mm = atof(trimmed + 14);
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
    // Use the flag instead of calling SD.begin() again
    return sdCardInitialized;
}