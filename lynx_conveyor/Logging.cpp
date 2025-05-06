#include "Logging.h"
#include "ValveController.h"
#include "MotorController.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0}; // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 500; // Default interval of 500ms

void logSystemState()
{
    // Create a more compact, single-line log format
    Serial.print("STATE: ");
    
    // Valve status in compact format
    for (int i = 0; i < valveCount; i++) {
        DoubleSolenoidValve *valve = getValveByIndex(i);
        if (valve) {
            // Print valve name and status
            Serial.print(valveNames[i]);
            Serial.print("=");
            
            if (valve->position == VALVE_POSITION_UNLOCK) {
                Serial.print("UNLOCKED");
            } else if (valve->position == VALVE_POSITION_LOCK) {
                Serial.print("LOCKED");
            } else {
                Serial.print("UNKNOWN");
            }
            
            Serial.print(", ");
        }
    }
    
    // Motor state in compact format
    Serial.print("MotorState=");
    switch (motorState) {
        case MOTOR_STATE_IDLE:
            Serial.print("IDLE");
            break;
        case MOTOR_STATE_MOVING:
            Serial.print("MOVING");
            break;
        case MOTOR_STATE_HOMING:
            Serial.print("HOMING");
            break;
        case MOTOR_STATE_FAULTED:
            Serial.print("FAULTED");
            break;
        case MOTOR_STATE_NOT_READY:
            Serial.print("NOT_READY");
            break;
        default:
            Serial.print("UNKNOWN");
            break;
    }
    
    // Position and other motor data
    Serial.print(", Position=");
    // Get the current position, apply abs() to always show positive
    double calculatedPositionMm = abs(pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()));
    Serial.print(calculatedPositionMm, 2);  // Display with 2 decimal places
    Serial.print("mm");

    // Add encoder count information with available data
    Serial.print(" (Commanded: ");
    // Show counts as positive 
    Serial.print(abs(MOTOR_CONNECTOR.PositionRefCommanded()));
    Serial.print(" counts, Speed: ");
    // Get the absolute value of velocity for display purposes
    int32_t velocity = MOTOR_CONNECTOR.VelocityRefCommanded();
    Serial.print(abs(velocity));
    Serial.print(" counts/sec = ");
    Serial.print(abs((double)velocity * 60.0 / PULSES_PER_REV), 1);
    Serial.print(" RPM");
    
    Serial.print(", Homed=");
    Serial.print(isHomed ? "YES" : "NO");
    
    // Home sensor status
    Serial.print(", HomeSensor=");
    bool pinState = digitalRead(HOME_SENSOR_PIN);
    Serial.print(pinState ? "HIGH" : "LOW");
    Serial.print(" (");
    Serial.print(pinState == HIGH ? "TRIGGERED" : "NOT_TRIGGERED");
    Serial.print(")");
    
    // Add to the end of your logSystemState() function:
    Serial.print(", E-Stop=");
    if (isEStopActive()) {
        Serial.print("TRIGGERED (EMERGENCY STOP)");
    } else {
        Serial.print("RELEASED (READY)");
    }
    
    // End the line
    Serial.println();
}