#line 1 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/Logging.cpp"
#include "Logging.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0};             // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 250; // Default interval of 500ms

void logSystemState()
{
    char msg[800];     // Large buffer for the complete log message
    char section[200]; // Buffer for building individual sections

    strcpy(msg, "[LOG] ");

    // 1. VALVES section - Enhanced to include sensor feedback
    strcat(msg, "Valves: ");
    const char *updatedValveNames[4] = {"Lock1", "Lock2", "Lock3", "Shuttle"};

    DoubleSolenoidValve *valves[4] = {
        getTray1Valve(), getTray2Valve(), getTray3Valve(), getShuttleValve()};
    CylinderSensor *sensors[4] = {
        getTray1Sensor(), getTray2Sensor(), getTray3Sensor(), getShuttleSensor()};

    for (int i = 0; i < valveCount; i++)
    {
        if (valves[i])
        {
            bool isLocked = (valves[i]->position == VALVE_POSITION_LOCK);
            bool sensorState = sensorRead(*sensors[i]);
            bool positionVerified = (sensorState == !isLocked);

            sprintf(section, "%s=%s%s%s",
                    updatedValveNames[i],
                    isLocked ? (positionVerified ? "LOCKED" : "LOCKED?") : (positionVerified ? "UNLOCKED" : "UNLOCKED?"),
                    positionVerified ? "" : "[!]",
                    (i < valveCount - 1) ? ", " : "");
            strcat(msg, section);
        }
    }

    // 2. SENSORS section
    sprintf(section, " | Sensors: Tray1=%s, Tray2=%s, Tray3=%s",
            sensorRead(tray1DetectSensor) ? "PRESENT" : "EMPTY",
            sensorRead(tray2DetectSensor) ? "PRESENT" : "EMPTY",
            sensorRead(tray3DetectSensor) ? "PRESENT" : "EMPTY");
    strcat(msg, section);

    // 3. SYSTEM section
    const char *motorStateStr;
    switch (motorState)
    {
    case MOTOR_STATE_IDLE:
        motorStateStr = "IDLE";
        break;
    case MOTOR_STATE_MOVING:
        motorStateStr = "MOVING";
        break;
    case MOTOR_STATE_HOMING:
        motorStateStr = "HOMING";
        break;
    case MOTOR_STATE_FAULTED:
        motorStateStr = "FAULTED";
        break;
    case MOTOR_STATE_NOT_READY:
        motorStateStr = "NOT_READY";
        break;
    default:
        motorStateStr = "UNKNOWN";
        break;
    }

    float pressure = getPressurePsi();
    sprintf(section, " | System: Motor=%s, Homed=%s, E-Stop=%s, HLFB=%s, Clients=%d, Pressure=%.1f PSI%s",
            motorStateStr,
            isHomed ? "YES" : "NO",
            isEStopActive() ? "TRIGGERED" : "RELEASED",
            (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) ? "ASSERTED" : "NOT_ASSERTED",
            getConnectedClientCount(),
            pressure,
            (pressure < MIN_SAFE_PRESSURE) ? " (LOW)" : "");
    strcat(msg, section);

    // 4. POSITION section
    double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    int32_t currentPulses = normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded());

    sprintf(section, " | Position: %.2fmm (%ld counts), Target=", calculatedPositionMm, currentPulses);
    strcat(msg, section);

    if ((motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING) && hasCurrentTarget)
    {
        sprintf(section, "%.2fmm (%ld counts)", currentTargetPositionMm, normalizeEncoderValue(currentTargetPulses));
    }
    else
    {
        strcpy(section, "None");
    }
    strcat(msg, section);

    strcat(msg, ", LastTarget=");
    if (hasLastTarget)
    {
        sprintf(section, "%.2fmm (%ld counts)", lastTargetPositionMm, normalizeEncoderValue(lastTargetPulses));
    }
    else
    {
        strcpy(section, "None");
    }
    strcat(msg, section);

    // 5. VELOCITY section
    double currentVelocityRpm = abs((double)MOTOR_CONNECTOR.VelocityRefCommanded() * 60.0 / PULSES_PER_REV);
    sprintf(section, " | Velocity: %.1fRPM", currentVelocityRpm);
    strcat(msg, section);

    if (currentVelocityRpm > 0)
    {
        sprintf(section, " (%d%%)", (int)(currentVelocityRpm * 100 / ppsToRpm(currentVelMax)));
        strcat(msg, section);
    }

    sprintf(section, ", Limits: %.0fRPM/%.0fRPM/s",
            ppsToRpm(currentVelMax),
            (double)currentAccelMax * 60.0 / PULSES_PER_REV);
    strcat(msg, section);

    // 6. JOG and MPG sections
    sprintf(section, " | Jog: %.1fmm/%dRPM | MPG: ", currentJogIncrementMm, currentJogSpeedRpm);
    strcat(msg, section);

    if (encoderControlActive)
    {
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        sprintf(section, "ON x%s (%.2fmm/rot)", getMultiplierName(currentMultiplier), mmPerRotation);
    }
    else
    {
        strcpy(section, "OFF");
    }
    strcat(msg, section);

    // Output the complete message
    Serial.println(msg);
}