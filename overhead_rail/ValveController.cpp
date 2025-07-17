#include "ValveController.h"
#include "Sensors.h"
#include "Utils.h"

//=============================================================================
// PROGMEM FORMAT STRINGS
//=============================================================================
// Core valve operation formats - streamlined for operator clarity
const char FMT_VALVE_INIT_SUCCESS[] PROGMEM = "Valve system: Initialization complete - %s position verified";
const char FMT_VALVE_INIT_PRESSURE_WARNING[] PROGMEM = "Valve system: Initialization complete - WARNING: Low pressure (%d.%02d PSI), manual position verification required";
const char FMT_VALVE_OPERATION_RESULT[] PROGMEM = "Valve: %s → %s%s";

// Operation formats (consolidated for reduced verbosity)  
const char FMT_VALVE_ALREADY_AT[] PROGMEM = "Valve already at %s position";
const char FMT_MOVING_VALVE[] PROGMEM = "Moving valve to %s position...";
const char FMT_VALVE_SUCCESS[] PROGMEM = "Valve successfully moved to %s position";

// Error and diagnostic formats (preserved for troubleshooting)
const char FMT_VALVE_SET[] PROGMEM = "Valve set to %s";
const char FMT_INSUFFICIENT_PRESSURE[] PROGMEM = "Insufficient pressure: %d.%02d PSI (min: %d.%02d PSI)";
const char FMT_VALVE_TIMEOUT[] PROGMEM = "Timeout waiting for %s position confirmation (%lu ms)";
const char FMT_SENSOR_ERROR[] PROGMEM = "Sensor error during %s operation";
const char FMT_PNEUMATIC_VALVE[] PROGMEM = "Pneumatic Cylinder Valve: %s";
const char FMT_CYLINDER_SENSORS[] PROGMEM = "Cylinder sensors - Extended: %s, Retracted: %s";
const char FMT_AIR_PRESSURE_SUFFICIENT[] PROGMEM = "Air pressure: %d.%02d PSI [SUFFICIENT]";
const char FMT_AIR_PRESSURE_LOW[] PROGMEM = "Air pressure: %d.%02d PSI [LOW - min: %d.%02d PSI]";
const char FMT_LAST_OP_FAILED[] PROGMEM = "Last operation: FAILED - %s";
const char FMT_LAST_OP_SUCCESS[] PROGMEM = "Last operation: SUCCESS (%lu ms ago)";
const char FMT_CCIO_BOARD[] PROGMEM = "CCIO Board: %s";
const char FMT_VALVE_SYSTEM[] PROGMEM = "Valve System: %s";
const char FMT_CURRENT_POSITION[] PROGMEM = "Current Position: %s";
const char FMT_CONTROL_PIN[] PROGMEM = "Control Pin: CCIO-A4 (%s)";
const char FMT_EXTENDED_SENSOR[] PROGMEM = "  Extended Sensor: %s";
const char FMT_RETRACTED_SENSOR[] PROGMEM = "  Retracted Sensor: %s";
const char FMT_AIR_PRESSURE[] PROGMEM = "Air Pressure: %d.%02d PSI";
const char FMT_PRESSURE_STATUS[] PROGMEM = "Pressure Status: %s (min required: %d.%02d PSI)";
const char FMT_LAST_OPERATION[] PROGMEM = "Last Operation: %s ago";
const char FMT_LAST_RESULT_FAILED[] PROGMEM = "Last Result: FAILED - %s";

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
PneumaticValve cylinderValve;              // Main pneumatic cylinder valve
unsigned long lastValveOperationTime = 0; // Global timestamp for valve operations
bool lastValveOperationFailed = false;    // Status of last valve operation
char lastValveFailureDetails[MEDIUM_MSG_SIZE] = "";   // Details of last failure

//=============================================================================
// INITIALIZATION
//=============================================================================

void initValveSystem(bool hasCCIOBoard)
{
    // Store CCIO status
    hasCCIO = hasCCIOBoard;
    
    // Initialize valve structure
    cylinderValve.controlPin = PNEUMATIC_CYLINDER_VALVE_PIN;
    cylinderValve.lastOperationTime = 0;
    cylinderValve.initialized = false;
    // Note: currentPosition will be set after we verify actual cylinder position
    
    // Reset error tracking
    lastValveOperationFailed = false;
    lastValveOperationTime = 0;
    memset(lastValveFailureDetails, 0, sizeof(lastValveFailureDetails));

    if (!hasCCIO)
    {
        Console.serialError(F("No CCIO board detected - valve control unavailable"));
        return;
    }

    Console.serialInfo(F("Initializing valve system..."));

    // Configure valve control pin as output
    pinMode(cylinderValve.controlPin, OUTPUT);
    
    // Allow CCIO board and sensors to stabilize after initialization
    delay(100);
    
    // Check current sensor state before any changes
    updateAllSensors(); // Make sure sensor readings are current
    bool initialExtended = readDigitalSensor(cylinderExtendedSensor);
    bool initialRetracted = readDigitalSensor(cylinderRetractedSensor);
    
    // Ensure valve starts in retracted position (energized HIGH for retraction)
    Console.serialInfo(F("Setting cylinder to retracted position..."));
    digitalWrite(cylinderValve.controlPin, HIGH);
    
    // Give cylinder time to move to retracted position and sensors to stabilize
    delay(1500); // Increased from 1000ms to allow for full travel + sensor stabilization
    
    // Check sensor state after valve change
    updateAllSensors(); // Make sure sensor readings are current after delay
    bool afterExtended = readDigitalSensor(cylinderExtendedSensor);
    bool afterRetracted = readDigitalSensor(cylinderRetractedSensor);
    
    // Mark as initialized
    cylinderValve.initialized = true;
    
    // Check if cylinder actually moved to retracted position
    if (afterRetracted && !afterExtended) {
        Console.serialInfo(F("Cylinder successfully retracted - system ready"));
        cylinderValve.currentPosition = VALVE_POSITION_RETRACTED;
    } else if (afterExtended && !afterRetracted) {
        Console.serialWarning(F("Cylinder failed to retract - check pressure or mechanical issues"));
        // Update controller state to match physical reality
        cylinderValve.currentPosition = VALVE_POSITION_EXTENDED;
    } else {
        Console.serialWarning(F("Cylinder position unclear - sensor malfunction"));
        // Keep default retracted state but mark as uncertain
        cylinderValve.currentPosition = VALVE_POSITION_RETRACTED;
    }
    
    // SAFETY: Final verification with longer timeout for startup conditions
    if (isPressureSufficientForValve()) {
        // Allow extra time for startup conditions - sensors may still be settling
        unsigned long startupTimeout = VALVE_SENSOR_TIMEOUT_MS * 2; // Double normal timeout
        
        // Only do verification if sensors showed unclear state
        if (afterRetracted && !afterExtended) {
            // Sensors already confirmed retracted position - no need for re-verification
            Console.serialInfo(F("Valve system: Initialization complete - Rail 2 collision-safe"));
        } else {
            // Attempt explicit retraction with extended timeout for startup
            ValveOperationResult result = retractCylinder(startupTimeout);
            
            if (result == VALVE_OP_SUCCESS && isCylinderActuallyRetracted()) {
                Console.serialInfo(F("Valve system: Initialization complete - Rail 2 collision-safe"));
            } else if (result == VALVE_OP_ALREADY_AT_POSITION && isCylinderActuallyRetracted()) {
                Console.serialInfo(F("Valve system: Initialization complete - Rail 2 collision-safe"));
            } else {
                Console.serialWarning(F("Valve system: Sensor verification incomplete during startup - manual verification recommended"));
                
                // Set warning but don't prevent system startup
                lastValveOperationFailed = true;
                strcpy(lastValveFailureDetails, "Startup sensor verification incomplete");
            }
        }
    } else {
        // Low pressure warning
        uint16_t currentPressure = readPressureScaled(airPressureSensor);
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, PSTR("Valve system: Low pressure (%d.%02d PSI) - verify cylinder position manually"), 
                 currentPressure / 100, currentPressure % 100);
        Console.serialWarning(msg);
    }
}

//=============================================================================
// LOW-LEVEL VALVE CONTROL
//=============================================================================

void setValvePosition(ValvePosition position)
{
    if (!hasCCIO || !cylinderValve.initialized)
    {
        Console.serialError(F("Cannot set valve position - system not initialized"));
        return;
    }
    
    // Set digital output based on position
    // HIGH = RETRACTED (energized for retraction)
    // LOW = EXTENDED (de-energized for extension)
    bool outputState = (position == VALVE_POSITION_RETRACTED);
    digitalWrite(cylinderValve.controlPin, outputState);
    
    // Update internal state
    cylinderValve.currentPosition = position;
    cylinderValve.lastOperationTime = millis();
    lastValveOperationTime = millis();
    
    char msg[MEDIUM_MSG_SIZE];
    sprintf_P(msg, FMT_VALVE_SET, getValvePositionName(position));
    Console.serialDiagnostic(msg);
}

ValvePosition getValvePosition()
{
    return cylinderValve.currentPosition;
}

bool isValveAtPosition(ValvePosition position)
{
    return (cylinderValve.currentPosition == position);
}

//=============================================================================
// SAFE VALVE OPERATIONS WITH SENSOR FEEDBACK
//=============================================================================

ValveOperationResult safeSetValvePosition(ValvePosition targetPosition, unsigned long timeoutMs)
{
    char msg[MEDIUM_MSG_SIZE];
    
    // Check if system is ready
    if (!hasCCIO)
    {
        strcpy(lastValveFailureDetails, "CCIO board not available");
        lastValveOperationFailed = true;
        return VALVE_OP_NO_CCIO;
    }
    
    if (!cylinderValve.initialized)
    {
        strcpy(lastValveFailureDetails, "Valve system not initialized");
        lastValveOperationFailed = true;
        return VALVE_OP_SENSOR_ERROR;
    }
    
    // Check if already at target position (silently handle - not critical operator info)
    if (isValveAtPosition(targetPosition))
    {
        return VALVE_OP_ALREADY_AT_POSITION;
    }
    
    // Check air pressure before operation
    if (!isPressureSufficientForValve())
    {
        uint16_t currentPressure = readPressureScaled(airPressureSensor);
        sprintf_P(lastValveFailureDetails, FMT_INSUFFICIENT_PRESSURE,
                currentPressure / 100, currentPressure % 100,
                MIN_VALVE_PRESSURE_SCALED / 100, MIN_VALVE_PRESSURE_SCALED % 100);
        lastValveOperationFailed = true;
        Console.serialError(lastValveFailureDetails);
        return VALVE_OP_PRESSURE_LOW;
    }
    
    // Perform the valve operation (combine operation and result into single message)
    setValvePosition(targetPosition);
    
    // Wait for sensor confirmation (if sensors are available and working)
    unsigned long startTime = millis();
    bool sensorConfirmation = false;
    bool sensorTimeout = false;
    
    // Determine expected sensor state based on valve position
    bool expectedExtendedState = (targetPosition == VALVE_POSITION_EXTENDED);
    
    while (!sensorTimeout)
    {
        // Check timeout
        if (timeoutElapsed(millis(), startTime, timeoutMs))
        {
            sensorTimeout = true;
            break;
        }
        
        // Read cylinder sensors to confirm position
        bool cylinderExtended = readDigitalSensor(cylinderExtendedSensor);
        bool cylinderRetracted = readDigitalSensor(cylinderRetractedSensor);
        
        // Check for valid sensor state (log once per operation, not continuously)
        if (cylinderExtended && cylinderRetracted && !sensorConfirmation)
        {
            Console.serialWarning(F("Warning: Both cylinder sensors active - check sensor wiring"));
        }
        
        // Check if we've reached the expected position
        if (expectedExtendedState && cylinderExtended && !cylinderRetracted)
        {
            sensorConfirmation = true;
            break;
        }
        else if (!expectedExtendedState && cylinderRetracted && !cylinderExtended)
        {
            sensorConfirmation = true;
            break;
        }
        
        delay(10); // Short delay to prevent excessive CPU usage
    }
    
    // Evaluate results and provide consolidated operation feedback
    if (sensorConfirmation)
    {
        char msg[MEDIUM_MSG_SIZE];
        sprintf_P(msg, FMT_VALVE_OPERATION_RESULT, 
                 getValvePositionName(cylinderValve.currentPosition), 
                 getValvePositionName(targetPosition),
                 " (confirmed)");
        Console.serialInfo(msg);
        lastValveOperationFailed = false;
        return VALVE_OP_SUCCESS;
    }
    else if (sensorTimeout)
    {
        sprintf_P(lastValveFailureDetails, FMT_VALVE_TIMEOUT,
                getValvePositionName(targetPosition), timeoutMs);
        lastValveOperationFailed = true;
        Console.serialError(lastValveFailureDetails);
        return VALVE_OP_TIMEOUT;
    }
    else
    {
        sprintf_P(lastValveFailureDetails, FMT_SENSOR_ERROR,
                getValvePositionName(targetPosition));
        lastValveOperationFailed = true;
        Console.serialError(lastValveFailureDetails);
        return VALVE_OP_SENSOR_ERROR;
    }
}

ValveOperationResult extendCylinder(unsigned long timeoutMs)
{
    return safeSetValvePosition(VALVE_POSITION_EXTENDED, timeoutMs);
}

ValveOperationResult retractCylinder(unsigned long timeoutMs)
{
    return safeSetValvePosition(VALVE_POSITION_RETRACTED, timeoutMs);
}

//=============================================================================
// STATUS AND DIAGNOSTICS
//=============================================================================

void printValveDetailedStatus()
{
    char msg[ALERT_MSG_SIZE];
    
    Console.serialInfo(F("=== DETAILED VALVE STATUS ==="));
    
    // System status
    sprintf_P(msg, FMT_CCIO_BOARD, hasCCIO ? "AVAILABLE" : "NOT AVAILABLE");
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_VALVE_SYSTEM, cylinderValve.initialized ? "INITIALIZED" : "NOT INITIALIZED");
    Console.serialInfo(msg);
    
    if (!hasCCIO || !cylinderValve.initialized)
    {
        Console.serialInfo(F("=== END VALVE STATUS ==="));
        return;
    }
    
    // Current valve state
    sprintf_P(msg, FMT_CURRENT_POSITION, getValvePositionName(cylinderValve.currentPosition));
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_CONTROL_PIN, 
            digitalRead(cylinderValve.controlPin) ? "HIGH/Energized" : "LOW/De-energized");
    Console.serialInfo(msg);
    
    // Sensor readings
    Console.serialInfo(F("Cylinder Sensors:"));
    bool extended = readDigitalSensor(cylinderExtendedSensor);
    bool retracted = readDigitalSensor(cylinderRetractedSensor);
    
    sprintf_P(msg, FMT_EXTENDED_SENSOR, extended ? "ACTIVE" : "inactive");
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_RETRACTED_SENSOR, retracted ? "ACTIVE" : "inactive");
    Console.serialInfo(msg);
    
    // Sensor validation
    if (extended && retracted)
    {
        Console.serialError(F("  ERROR: Both sensors active - check wiring"));
    }
    else if (!extended && !retracted)
    {
        Console.serialWarning(F("  WARNING: No sensors active - position uncertain"));
    }
    else if ((cylinderValve.currentPosition == VALVE_POSITION_EXTENDED && !extended) ||
             (cylinderValve.currentPosition == VALVE_POSITION_RETRACTED && !retracted))
    {
        Console.serialWarning(F("  WARNING: Valve position doesn't match sensor reading"));
    }
    else
    {
        Console.serialInfo(F("  Sensor readings consistent with valve position"));
    }
    
    // Pressure status
    uint16_t pressure = readPressureScaled(airPressureSensor);
    sprintf_P(msg, FMT_AIR_PRESSURE, pressure / 100, pressure % 100);
    Console.serialInfo(msg);
    
    sprintf_P(msg, FMT_PRESSURE_STATUS,
            isPressureSufficientForValve() ? "SUFFICIENT" : "LOW",
            MIN_VALVE_PRESSURE_SCALED / 100, MIN_VALVE_PRESSURE_SCALED % 100);
    Console.serialInfo(msg);
    
    // Operation history
    if (lastValveOperationTime > 0)
    {
        unsigned long timeSince = getTimeSinceLastValveOperation();
        char timeBuffer[40];
        formatHumanReadableTime(timeSince / 1000, timeBuffer, sizeof(timeBuffer));
        sprintf_P(msg, FMT_LAST_OPERATION, timeBuffer);
        Console.serialInfo(msg);
        
        if (lastValveOperationFailed)
        {
            sprintf_P(msg, FMT_LAST_RESULT_FAILED, lastValveFailureDetails);
            Console.serialError(msg);
        }
        else
        {
            Console.serialInfo(F("Last Result: SUCCESS"));
        }
    }
    else
    {
        Console.serialInfo(F("Last Operation: None since initialization"));
    }
    
    Console.serialInfo(F("=== END VALVE STATUS ==="));
}

const char* getValvePositionName(ValvePosition position)
{
    switch (position)
    {
        case VALVE_POSITION_RETRACTED: return "RETRACTED";
        case VALVE_POSITION_EXTENDED: return "EXTENDED";
        default: return "UNKNOWN";
    }
}

const char* getValveOperationResultName(ValveOperationResult result)
{
    switch (result)
    {
        case VALVE_OP_SUCCESS: return "SUCCESS";
        case VALVE_OP_TIMEOUT: return "TIMEOUT";
        case VALVE_OP_PRESSURE_LOW: return "PRESSURE_LOW";
        case VALVE_OP_NO_CCIO: return "NO_CCIO";
        case VALVE_OP_ALREADY_AT_POSITION: return "ALREADY_AT_POSITION";
        case VALVE_OP_SENSOR_ERROR: return "SENSOR_ERROR";
        default: return "UNKNOWN";
    }
}

//=============================================================================
// SAFETY AND VALIDATION
//=============================================================================

bool isPressureSufficientForValve()
{
    uint16_t currentPressure = readPressureScaled(airPressureSensor);
    return (currentPressure >= MIN_VALVE_PRESSURE_SCALED);
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

unsigned long getTimeSinceLastValveOperation()
{
    if (lastValveOperationTime == 0)
    {
        return 0;
    }
    return timeDiff(millis(), lastValveOperationTime);
}

//=============================================================================
// VALVE POSITION VALIDATION FUNCTIONS
//=============================================================================

bool validateValvePosition()
{
    // Check if the valve controller's state matches the actual sensor readings
    if (!hasCCIO) {
        // Can't validate without sensors, assume valve state is correct
        return true;
    }
    
    bool sensorRetracted = isCylinderRetracted();
    bool sensorExtended = isCylinderExtended();
    ValvePosition currentValveState = getValvePosition();
    
    // Check for sensor error conditions
    if (sensorRetracted && sensorExtended) {
        Console.serialWarning(F("VALVE VALIDATION WARNING: Both cylinder sensors active - check wiring"));
        return false;
    }
    
    if (!sensorRetracted && !sensorExtended) {
        Console.serialWarning(F("VALVE VALIDATION WARNING: No cylinder sensors active - position unknown"));
        return false;
    }
    
    // Check for state mismatch
    if (currentValveState == VALVE_POSITION_RETRACTED && !sensorRetracted) {
        Console.serialWarning(F("VALVE STATE MISMATCH: Controller thinks cylinder is retracted but sensor disagrees"));
        return false;
    }
    
    if (currentValveState == VALVE_POSITION_EXTENDED && !sensorExtended) {
        Console.serialWarning(F("VALVE STATE MISMATCH: Controller thinks cylinder is extended but sensor disagrees"));
        return false;
    }
    
    return true;
}

bool isCylinderActuallyRetracted()
{
    // Always check sensors regardless of valve controller state
    if (!hasCCIO) {
        Console.serialWarning(F("Cannot verify cylinder position: CCIO board not detected"));
        // Fall back to valve controller state if no sensors
        return (getValvePosition() == VALVE_POSITION_RETRACTED);
    }
    
    bool sensorRetracted = isCylinderRetracted();
    bool sensorExtended = isCylinderExtended();
    
    // Validate sensor readings
    if (sensorRetracted && sensorExtended) {
        Console.serialError(F("SENSOR ERROR: Both cylinder sensors active - check wiring"));
        return false;
    }
    
    return sensorRetracted && !sensorExtended;
}

bool isCylinderActuallyExtended()
{
    // Always check sensors regardless of valve controller state
    if (!hasCCIO) {
        Console.serialWarning(F("Cannot verify cylinder position: CCIO board not detected"));
        // Fall back to valve controller state if no sensors
        return (getValvePosition() == VALVE_POSITION_EXTENDED);
    }
    
    bool sensorRetracted = isCylinderRetracted();
    bool sensorExtended = isCylinderExtended();
    
    // Validate sensor readings
    if (sensorRetracted && sensorExtended) {
        Console.serialError(F("SENSOR ERROR: Both cylinder sensors active - check wiring"));
        return false;
    }
    
    return sensorExtended && !sensorRetracted;
}

//=============================================================================
// TIMEOUT RESET FUNCTIONS
//=============================================================================

void resetValveTimeouts() {
    unsigned long currentTime = millis();
    
    Console.serialInfo(F("VALVE TIMEOUTS: Clearing valve operation timeout tracking"));
    
    // Reset valve operation timestamp tracking
    lastValveOperationTime = currentTime;
    cylinderValve.lastOperationTime = currentTime;
    
    // Clear any failure state that might be timeout-related
    lastValveOperationFailed = false;
    lastValveFailureDetails[0] = '\0';  // Clear failure details
    
    Console.serialInfo(F("VALVE TIMEOUTS: All valve timeout tracking reset"));
}
