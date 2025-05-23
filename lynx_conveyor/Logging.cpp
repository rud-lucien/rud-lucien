#include "Logging.h"
#include "ValveController.h"
#include "MotorController.h"
#include "EncoderController.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0}; // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 500; // Default interval of 500ms

void logSystemState()
{
    Serial.print(F("[LOG] "));
    
    // 1. VALVES section - Enhanced to include sensor feedback
    Serial.print(F("Valves: "));
    const char* updatedValveNames[4] = {"Lock1", "Lock2", "Lock3", "Shuttle"};
    
    // Get the appropriate valves and sensors
    DoubleSolenoidValve* valves[4] = {
        getTray1Valve(), getTray2Valve(), getTray3Valve(), getShuttleValve()
    };
    
    CylinderSensor* sensors[4] = {
        getTray1Sensor(), getTray2Sensor(), getTray3Sensor(), getShuttleSensor()
    };
    
    for (int i = 0; i < valveCount; i++) {
        if (valves[i]) {
            Serial.print(updatedValveNames[i]);
            Serial.print(F("="));
            
            // Determine commanded position
            bool isLocked = (valves[i]->position == VALVE_POSITION_LOCK);
            
            // Read actual sensor state
            bool sensorState = sensorRead(*sensors[i]);
            
            // Verify if they match (sensor state should be TRUE when locked)
            bool positionVerified = (sensorState == !isLocked);
            
            // Display position with verification indicator
            if (isLocked) {
                Serial.print(positionVerified ? F("LOCKED") : F("LOCKED?"));
            } else {
                Serial.print(positionVerified ? F("UNLOCKED") : F("UNLOCKED?"));
            }
            
            // Add ? indicator for mismatches
            if (!positionVerified) {
                Serial.print(F("[!]"));
            }
            
            // Add comma except after last item
            if (i < valveCount - 1) {
                Serial.print(F(", "));
            }
        }
    }
    
    // 2. SENSORS section
    Serial.print(F(" | Sensors: "));
    Serial.print(F("Tray1="));
    Serial.print(sensorRead(tray1DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Serial.print(F(", Tray2="));
    Serial.print(sensorRead(tray2DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Serial.print(F(", Tray3="));
    Serial.print(sensorRead(tray3DetectSensor) ? F("PRESENT") : F("EMPTY"));
    
    // 3. SYSTEM section
    Serial.print(F(" | System: "));
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
    
    // 4. POSITION GROUP with improved naming
    Serial.print(F(" | Position: "));
    double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    int32_t currentPulses = normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded());
    Serial.print(calculatedPositionMm, 2);
    Serial.print(F("mm ("));
    Serial.print(currentPulses);
    Serial.print(F(" counts)"));
    
    // Target information
    Serial.print(F(", Target="));
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING) {
        if (hasCurrentTarget) {
            Serial.print(currentTargetPositionMm, 2);
            Serial.print(F("mm ("));
            Serial.print(normalizeEncoderValue(currentTargetPulses));
            Serial.print(F(" counts)"));
        } else {
            Serial.print(F("None"));
        }
    } else {
        Serial.print(F("None"));
    }
    
    // Last target position
    Serial.print(F(", LastTarget="));
    if (hasLastTarget) {
        Serial.print(lastTargetPositionMm, 2);
        Serial.print(F("mm ("));
        Serial.print(normalizeEncoderValue(lastTargetPulses));
        Serial.print(F(" counts)"));
    } else {
        Serial.print(F("None"));
    }
    
    // 5. VELOCITY GROUP with improved naming
    Serial.print(F(" | Velocity: "));
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
    
    // 6. JOG SETTINGS GROUP - removed (D) indicators
    Serial.print(F(" | Jog: "));
    Serial.print(currentJogIncrementMm, 1);
    Serial.print(F("mm/"));
    Serial.print(currentJogSpeedRpm);
    Serial.print(F("RPM"));

    // Encoder status
    Serial.print(F(" | MPG: "));
    if (encoderControlActive) {
        Serial.print(F("ON x"));
        Serial.print(getMultiplierName(currentMultiplier));
        
        // Add mm/rotation information in a compact format
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        Serial.print(F(" ("));
        Serial.print(mmPerRotation, 2);
        Serial.print(F("mm/rot)"));
    } else {
        Serial.print(F("OFF"));
    }
    
    // End the line
    Serial.println();
}