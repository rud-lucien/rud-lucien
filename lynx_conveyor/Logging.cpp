#include "Logging.h"
#include "ValveController.h"
#include "MotorController.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0}; // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 500; // Default interval of 500ms

void logSystemState()
{
    Serial.print("STATE: ");
    
    // 1. DEVICE STATUS GROUP
    // Valve status
    for (int i = 0; i < valveCount; i++) {
        DoubleSolenoidValve *valve = getValveByIndex(i);
        if (valve) {
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
    
    // Motor and safety state
    Serial.print("Motor=");
    switch (motorState) {
        case MOTOR_STATE_IDLE: Serial.print("IDLE"); break;
        case MOTOR_STATE_MOVING: Serial.print("MOVING"); break;
        case MOTOR_STATE_HOMING: Serial.print("HOMING"); break;
        case MOTOR_STATE_FAULTED: Serial.print("FAULTED"); break;
        case MOTOR_STATE_NOT_READY: Serial.print("NOT_READY"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    
    Serial.print(", Homed=");
    Serial.print(isHomed ? "YES" : "NO");
    
    Serial.print(", E-Stop=");
    Serial.print(isEStopActive() ? "TRIGGERED" : "RELEASED");
    
    // 2. POSITION GROUP
    // Current position with encoder counts
    Serial.print(" | Pos=");
    double calculatedPositionMm = abs(pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded()));
    Serial.print(calculatedPositionMm, 2);
    Serial.print("mm (");
    Serial.print(abs(MOTOR_CONNECTOR.PositionRefCommanded()));
    Serial.print(" counts)");
    
    // Target information (moved next to position)
    Serial.print(", Target=");
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING) {
        if (hasCurrentTarget) {
            switch (currentTargetType) {
                case POSITION_HOME: Serial.print("Home"); break;
                case POSITION_1: Serial.print("Pos1"); break;
                case POSITION_2: Serial.print("Pos2"); break;
                case POSITION_3: Serial.print("Pos3"); break;
                case POSITION_4: Serial.print("Pos4"); break;
                case POSITION_CUSTOM: 
                    Serial.print(currentTargetPositionMm, 2);
                    Serial.print("mm (");
                    Serial.print(abs(currentTargetPulses));
                    Serial.print(" counts)");
                    break;
                default: Serial.print("Unknown"); break;
            }
        } else {
            Serial.print("None");
        }
    } else {
        Serial.print("None");
    }
    
    Serial.print(", CurrTarget=");
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING) {
        if (hasCurrentTarget) {
            switch (currentTargetType) {
                case POSITION_HOME:
                    Serial.print("Home(0.00mm, 0 counts)");
                    break;
                case POSITION_1:
                    Serial.print("Pos1(");
                    Serial.print(POSITION_1_MM, 2);
                    Serial.print("mm, ");
                    Serial.print(abs(POSITION_1_PULSES));
                    Serial.print(" counts)");
                    break;
                case POSITION_2:
                    Serial.print("Pos2(");
                    Serial.print(POSITION_2_MM, 2);
                    Serial.print("mm, ");
                    Serial.print(abs(POSITION_2_PULSES));
                    Serial.print(" counts)");
                    break;
                case POSITION_3:
                    Serial.print("Pos3(");
                    Serial.print(POSITION_3_MM, 2);
                    Serial.print("mm, ");
                    Serial.print(abs(POSITION_3_PULSES));
                    Serial.print(" counts)");
                    break;
                case POSITION_4:
                    Serial.print("Pos4(");
                    Serial.print(POSITION_4_MM, 2);
                    Serial.print("mm, ");
                    Serial.print(abs(POSITION_4_PULSES));
                    Serial.print(" counts)");
                    break;
                case POSITION_CUSTOM:
                    Serial.print(currentTargetPositionMm, 2);
                    Serial.print("mm, ");
                    Serial.print(abs(currentTargetPulses));
                    Serial.print(" counts");
                    break;
                default:
                    Serial.print("Unknown");
                    break;
            }
        } else {
            Serial.print("None");
        }
    } else {
        Serial.print("None");
    }
    
    Serial.print(", LastTarget=");
    if (hasLastTarget) {
        switch (lastTargetType) {
            case POSITION_HOME:
                Serial.print("Home(0.00mm, 0 counts)");
                break;
            case POSITION_1:
                Serial.print("Pos1(");
                Serial.print(POSITION_1_MM, 2);
                Serial.print("mm, ");
                Serial.print(abs(POSITION_1_PULSES));
                Serial.print(" counts)");
                break;
            case POSITION_2:
                Serial.print("Pos2(");
                Serial.print(POSITION_2_MM, 2);
                Serial.print("mm, ");
                Serial.print(abs(POSITION_2_PULSES));
                Serial.print(" counts)");
                break;
            case POSITION_3:
                Serial.print("Pos3(");
                Serial.print(POSITION_3_MM, 2);
                Serial.print("mm, ");
                Serial.print(abs(POSITION_3_PULSES));
                Serial.print(" counts)");
                break;
            case POSITION_4:
                Serial.print("Pos4(");
                Serial.print(POSITION_4_MM, 2);
                Serial.print("mm, ");
                Serial.print(abs(POSITION_4_PULSES));
                Serial.print(" counts)");
                break;
            case POSITION_CUSTOM:
                Serial.print(lastTargetPositionMm, 2);
                Serial.print("mm, ");
                Serial.print(abs(lastTargetPulses));
                Serial.print(" counts");
                break;
            default:
                Serial.print("Unknown");
                break;
        }
    } else {
        Serial.print("None");
    }
    
    // 3. MOTION PARAMETERS GROUP
    // Current velocity (removed redundant counts/sec display)
    Serial.print(" | Vel=");
    double currentVelocityRpm = abs((double)MOTOR_CONNECTOR.VelocityRefCommanded() * 60.0 / PULSES_PER_REV);
    Serial.print(currentVelocityRpm, 1);
    Serial.print("RPM");
    if (currentVelocityRpm > 0) {
        Serial.print(" (");
        Serial.print((int)(currentVelocityRpm * 100 / ppsToRpm(currentVelMax)));
        Serial.print("%)");
    }
    
    // Limits (condensed)
    Serial.print(", Limits: ");
    Serial.print(ppsToRpm(currentVelMax), 0);
    Serial.print("RPM/");
    Serial.print((double)currentAccelMax * 60.0 / PULSES_PER_REV, 0);
    Serial.print("RPM/s");
    
    // 4. JOG SETTINGS GROUP
    Serial.print(" | Jog: ");
    Serial.print(currentJogIncrementMm, 1);
    Serial.print("mm");
    if (currentJogIncrementMm == DEFAULT_JOG_INCREMENT) 
        Serial.print("(D)");
    
    Serial.print("/");
    Serial.print(currentJogSpeedRpm);
    Serial.print("RPM");
    if (currentJogSpeedRpm == DEFAULT_JOG_SPEED)
        Serial.print("(D)");
    
    // End the line
    Serial.println();
}