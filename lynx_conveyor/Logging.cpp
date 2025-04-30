#include "Logging.h"
#include "ValveController.h"
#include "MotorController.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0}; // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 500; // Default interval of 500ms

void logSystemState()
{
    // Log header
    Serial.println("\n--- System Status ---");
    
    // Log tray lock states
    Serial.println("Tray Lock Status:");
    
    // Loop through the known valves using the existing valveCount
    for (int i = 0; i < valveCount; i++) {
        DoubleSolenoidValve *valve = getValveByIndex(i);
        if (valve) {
            // Use the existing valveNames array for consistent naming
            Serial.print("  ");
            Serial.print(valveNames[i]);
            Serial.print(": ");
            
            // Check valve position using the existing function
            if (valve->position == VALVE_POSITION_UNLOCK) {
                Serial.println("UNLOCKED");
            } else if (valve->position == VALVE_POSITION_LOCK) {
                Serial.println("LOCKED");
            } else {
                Serial.println("UNKNOWN");
            }
        }
    }
    
    // Log motor status
    Serial.println("Motor Status:");
    Serial.print("  State: ");
    switch (motorState) {
        case MOTOR_STATE_IDLE:
            Serial.println("IDLE");
            break;
        case MOTOR_STATE_MOVING:
            Serial.println("MOVING");
            break;
        case MOTOR_STATE_HOMING:
            Serial.println("HOMING");
            break;
        case MOTOR_STATE_FAULTED:
            Serial.println("FAULTED");
            break;
        case MOTOR_STATE_NOT_READY:
            Serial.println("NOT READY");
            break;
        default:
            Serial.println("UNKNOWN");
            break;
    }
    
    Serial.print("  Position: ");
    Serial.print(currentPositionMm);
    Serial.println(" mm");
    
    Serial.print("  Homed: ");
    Serial.println(isHomed ? "YES" : "NO");
    
    // Log home sensor status
    Serial.print("  Home Sensor: ");
    bool pinState = digitalRead(HOME_SENSOR_PIN);
    Serial.print(pinState ? "HIGH" : "LOW");
    Serial.print(" (");
    Serial.print(pinState == LOW ? "TRIGGERED" : "NOT TRIGGERED"); // Assuming active low
    Serial.println(")");
    
    Serial.println("-------------------");
}