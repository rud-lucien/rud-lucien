#include "Logging.h"
#include "ValveController.h"
#include "MotorController.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0}; // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 500; // Default interval of 500ms

void logSystemState()
{
    Serial.print(F("[LOG] "));
    
    // 1. DEVICE STATUS GROUP
    // Valve status
    for (int i = 0; i < valveCount; i++) {
        DoubleSolenoidValve *valve = getValveByIndex(i);
        if (valve) {
            Serial.print(valveNames[i]);
            Serial.print(F("="));
            
            if (valve->position == VALVE_POSITION_UNLOCK) {
                Serial.print(F("UNLOCKED"));
            } else if (valve->position == VALVE_POSITION_LOCK) {
                Serial.print(F("LOCKED"));
            } else {
                Serial.print(F("UNKNOWN"));
            }
            
            Serial.print(F(", "));
        }
    }

    // Individual tray sensor readings
    Serial.print(F("Tray1="));
    Serial.print(sensorRead(tray1DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Serial.print(F(", Tray2="));
    Serial.print(sensorRead(tray2DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Serial.print(F(", Tray3="));
    Serial.print(sensorRead(tray3DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Serial.print(F(", "));
    
    // Motor and safety state
    Serial.print(F("Motor="));
    switch (motorState) {
        case MOTOR_STATE_IDLE: Serial.print(F("IDLE")); break;
        case MOTOR_STATE_MOVING: Serial.print(F("MOVING")); break;
        case MOTOR_STATE_HOMING: Serial.print(F("HOMING")); break;
        case MOTOR_STATE_FAULTED: Serial.print(F("FAULTED")); break;
        case MOTOR_STATE_NOT_READY: Serial.print(F("NOT_READY")); break;
        default: Serial.print(F("UNKNOWN")); break;
    }
    
    Serial.print(F(", Homed="));
    Serial.print(isHomed ? F("YES") : F("NO"));
    
    Serial.print(F(", E-Stop="));
    Serial.print(isEStopActive() ? F("TRIGGERED") : F("RELEASED"));
    
    // 2. POSITION GROUP - Simple, clear format
    Serial.print(F(" | Pos="));
    double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    int32_t currentPulses = normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded());
    Serial.print(calculatedPositionMm, 2);
    Serial.print(F("mm ("));
    Serial.print(currentPulses);
    Serial.print(F(" counts)"));
    
    // Target information - just show the numerical value if moving
    Serial.print(F(", Target="));
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING) {
        if (hasCurrentTarget) {
            // For all target types, just show the actual position value
            Serial.print(currentTargetPositionMm, 2);
            Serial.print(F("mm ("));
            Serial.print(normalizeEncoderValue(currentTargetPulses)); // Normalize here
            Serial.print(F(" counts)"));
        } else {
            Serial.print(F("None"));
        }
    } else {
        Serial.print(F("None"));
    }
    
    // Last target - simplified to just show the completed position
    Serial.print(F(", LastTarget="));
    if (hasLastTarget) {
        Serial.print(lastTargetPositionMm, 2);
        Serial.print(F("mm ("));
        Serial.print(normalizeEncoderValue(lastTargetPulses)); // Normalize here
        Serial.print(F(" counts)"));
    } else {
        Serial.print(F("None"));
    }
    
    // 3. MOTION PARAMETERS GROUP
    // Current velocity (removed redundant counts/sec display)
    Serial.print(F(" | Vel="));
    double currentVelocityRpm = abs((double)MOTOR_CONNECTOR.VelocityRefCommanded() * 60.0 / PULSES_PER_REV);
    Serial.print(currentVelocityRpm, 1);
    Serial.print(F("RPM"));
    if (currentVelocityRpm > 0) {
        Serial.print(F(" ("));
        Serial.print((int)(currentVelocityRpm * 100 / ppsToRpm(currentVelMax)));
        Serial.print(F("%)"));
    }
    
    // Limits (condensed)
    Serial.print(F(", Limits: "));
    Serial.print(ppsToRpm(currentVelMax), 0);
    Serial.print(F("RPM/"));
    Serial.print((double)currentAccelMax * 60.0 / PULSES_PER_REV, 0);
    Serial.print(F("RPM/s"));
    
    // 4. JOG SETTINGS GROUP
    Serial.print(F(" | Jog: "));
    Serial.print(currentJogIncrementMm, 1);
    Serial.print(F("mm"));
    if (currentJogIncrementMm == DEFAULT_JOG_INCREMENT) 
        Serial.print(F("(D)"));
    
    Serial.print(F("/"));
    Serial.print(currentJogSpeedRpm);
    Serial.print(F("RPM"));
    if (currentJogSpeedRpm == DEFAULT_JOG_SPEED)
        Serial.print(F("(D)"));
    
    // End the line
    Serial.println();
}