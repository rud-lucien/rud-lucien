#include "ValveController.h"
#include "Sensors.h"
#include "Utils.h"

//=============================================================================
// PROGMEM FORMAT STRINGS
//=============================================================================
const char FMT_VALVE_SET[] PROGMEM = "Valve set to %s";
const char FMT_VALVE_ALREADY_AT[] PROGMEM = "Valve already at %s position";
const char FMT_INSUFFICIENT_PRESSURE[] PROGMEM = "Insufficient pressure: %d.%02d PSI (min: %d.%02d PSI)";
const char FMT_MOVING_VALVE[] PROGMEM = "Moving valve to %s position...";
const char FMT_VALVE_SUCCESS[] PROGMEM = "Valve successfully moved to %s position";
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
const char FMT_LAST_OPERATION[] PROGMEM = "Last Operation: %lu ms ago";
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
    cylinderValve.currentPosition = VALVE_POSITION_RETRACTED; // Default to spring return position
    cylinderValve.lastOperationTime = 0;
    cylinderValve.initialized = false;
    
    // Reset error tracking
    lastValveOperationFailed = false;
    lastValveOperationTime = 0;
    memset(lastValveFailureDetails, 0, sizeof(lastValveFailureDetails));

    if (!hasCCIO)
    {
        Console.serialError(F("No CCIO board detected - valve control unavailable"));
        return;
    }

    Console.serialInfo(F("Initializing pneumatic cylinder valve with CCIO board..."));

    // Configure valve control pin as output
    pinMode(cylinderValve.controlPin, OUTPUT);
    
    // Ensure valve starts in retracted position (de-energized)
    digitalWrite(cylinderValve.controlPin, LOW);
    
    // Mark as initialized
    cylinderValve.initialized = true;
    
    Console.serialInfo(F("Pneumatic cylinder valve initialized - Position: RETRACTED"));
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
    // LOW = RETRACTED (spring return, de-energized)
    // HIGH = EXTENDED (energized)
    bool outputState = (position == VALVE_POSITION_EXTENDED);
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
    
    // Check if already at target position
    if (isValveAtPosition(targetPosition))
    {
        sprintf_P(msg, FMT_VALVE_ALREADY_AT, getValvePositionName(targetPosition));
        Console.serialDiagnostic(msg);
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
    
    // Perform the valve operation
    sprintf_P(msg, FMT_MOVING_VALVE, getValvePositionName(targetPosition));
    Console.serialInfo(msg);
    
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
        
        // Check for valid sensor state (both sensors shouldn't be active simultaneously)
        if (cylinderExtended && cylinderRetracted)
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
    
    // Evaluate results
    if (sensorConfirmation)
    {
        sprintf_P(msg, FMT_VALVE_SUCCESS, getValvePositionName(targetPosition));
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

void printValveStatus()
{
    char msg[ALERT_MSG_SIZE];
    
    if (!hasCCIO)
    {
        Console.serialWarning(F("Valve Status: CCIO BOARD NOT AVAILABLE"));
        return;
    }
    
    if (!cylinderValve.initialized)
    {
        Console.serialWarning(F("Valve Status: NOT INITIALIZED"));
        return;
    }
    
    sprintf_P(msg, FMT_PNEUMATIC_VALVE, getValvePositionName(cylinderValve.currentPosition));
    Console.serialInfo(msg);
    
    // Show sensor states for confirmation
    if (hasCCIO)
    {
        bool extended = readDigitalSensor(cylinderExtendedSensor);
        bool retracted = readDigitalSensor(cylinderRetractedSensor);
        
        sprintf_P(msg, FMT_CYLINDER_SENSORS,
                extended ? "ACTIVE" : "inactive",
                retracted ? "ACTIVE" : "inactive");
        Console.serialInfo(msg);
        
        // Check for sensor consistency
        if (extended && retracted)
        {
            Console.serialWarning(F("Warning: Both cylinder sensors active simultaneously"));
        }
        else if (!extended && !retracted)
        {
            Console.serialWarning(F("Warning: No cylinder sensors active - position uncertain"));
        }
    }
    
    // Show pressure status
    if (isPressureSufficientForValve())
    {
        uint16_t pressure = readPressureScaled(airPressureSensor);
        sprintf_P(msg, FMT_AIR_PRESSURE_SUFFICIENT, pressure / 100, pressure % 100);
        Console.serialInfo(msg);
    }
    else
    {
        uint16_t pressure = readPressureScaled(airPressureSensor);
        sprintf_P(msg, FMT_AIR_PRESSURE_LOW, 
                pressure / 100, pressure % 100,
                MIN_VALVE_PRESSURE_SCALED / 100, MIN_VALVE_PRESSURE_SCALED % 100);
        Console.serialWarning(msg);
    }
    
    // Show last operation status
    if (lastValveOperationFailed)
    {
        sprintf_P(msg, FMT_LAST_OP_FAILED, lastValveFailureDetails);
        Console.serialError(msg);
    }
    else
    {
        unsigned long timeSince = getTimeSinceLastValveOperation();
        sprintf_P(msg, FMT_LAST_OP_SUCCESS, timeSince);
        Console.serialInfo(msg);
    }
}

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
        sprintf_P(msg, FMT_LAST_OPERATION, timeSince);
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

bool isValveOperationSafe()
{
    // Check basic system requirements
    if (!hasCCIO || !cylinderValve.initialized)
    {
        return false;
    }
    
    // Check air pressure
    if (!isPressureSufficientForValve())
    {
        return false;
    }
    
    // All checks passed
    return true;
}

bool isPressureSufficientForValve()
{
    uint16_t currentPressure = readPressureScaled(airPressureSensor);
    return (currentPressure >= MIN_VALVE_PRESSURE_SCALED);
}

bool isValveSystemReady()
{
    return (hasCCIO && cylinderValve.initialized && isPressureSufficientForValve());
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

void resetValveErrorState()
{
    lastValveOperationFailed = false;
    memset(lastValveFailureDetails, 0, sizeof(lastValveFailureDetails));
    Console.serialInfo(F("Valve error state cleared"));
}

unsigned long getTimeSinceLastValveOperation()
{
    if (lastValveOperationTime == 0)
    {
        return 0;
    }
    return millis() - lastValveOperationTime;
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
