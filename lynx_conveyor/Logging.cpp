#include "Logging.h"

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 500ms for when it's enabled
LoggingManagement logging = {0, 0};             // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 250; // Default interval of 500ms

void logSystemState()
{
    Console.print(F("[LOG] "));

    // 1. VALVES section - Enhanced to include sensor feedback
    Console.print(F("Valves: "));
    const char *updatedValveNames[4] = {"Lock1", "Lock2", "Lock3", "Shuttle"};

    // Get the appropriate valves and sensors
    DoubleSolenoidValve *valves[4] = {
        getTray1Valve(), getTray2Valve(), getTray3Valve(), getShuttleValve()};

    CylinderSensor *sensors[4] = {
        getTray1Sensor(), getTray2Sensor(), getTray3Sensor(), getShuttleSensor()};

    for (int i = 0; i < valveCount; i++)
    {
        if (valves[i])
        {
            Console.print(updatedValveNames[i]);
            Console.print(F("="));

            // Determine commanded position
            bool isLocked = (valves[i]->position == VALVE_POSITION_LOCK);

            // Read actual sensor state
            bool sensorState = sensorRead(*sensors[i]);

            // Verify if they match (sensor state should be TRUE when locked)
            bool positionVerified = (sensorState == !isLocked);

            // Display position with verification indicator
            if (isLocked)
            {
                Console.print(positionVerified ? F("LOCKED") : F("LOCKED?"));
            }
            else
            {
                Console.print(positionVerified ? F("UNLOCKED") : F("UNLOCKED?"));
            }

            // Add ? indicator for mismatches
            if (!positionVerified)
            {
                Console.print(F("[!]"));
            }

            // Add comma except after last item
            if (i < valveCount - 1)
            {
                Console.print(F(", "));
            }
        }
    }

    // 2. SENSORS section
    Console.print(F(" | Sensors: "));
    Console.print(F("Tray1="));
    Console.print(sensorRead(tray1DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Console.print(F(", Tray2="));
    Console.print(sensorRead(tray2DetectSensor) ? F("PRESENT") : F("EMPTY"));
    Console.print(F(", Tray3="));
    Console.print(sensorRead(tray3DetectSensor) ? F("PRESENT") : F("EMPTY"));

    // 3. SYSTEM section
    Console.print(F(" | System: "));
    Console.print(F("Motor="));
    switch (motorState)
    {
    case MOTOR_STATE_IDLE:
        Console.print(F("IDLE"));
        break;
    case MOTOR_STATE_MOVING:
        Console.print(F("MOVING"));
        break;
    case MOTOR_STATE_HOMING:
        Console.print(F("HOMING"));
        break;
    case MOTOR_STATE_FAULTED:
        Console.print(F("FAULTED"));
        break;
    case MOTOR_STATE_NOT_READY:
        Console.print(F("NOT_READY"));
        break;
    default:
        Console.print(F("UNKNOWN"));
        break;
    }

    Console.print(F(", Homed="));
    Console.print(isHomed ? F("YES") : F("NO"));

    Console.print(F(", E-Stop="));
    Console.print(isEStopActive() ? F("TRIGGERED") : F("RELEASED"));

    Console.print(F(", HLFB="));
    bool hlfbAsserted = MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED;
    Console.print(hlfbAsserted ? F("ASSERTED") : F("NOT_ASSERTED"));

    // 4. POSITION GROUP with improved naming
    Console.print(F(" | Position: "));
    double calculatedPositionMm = pulsesToMm(MOTOR_CONNECTOR.PositionRefCommanded());
    int32_t currentPulses = normalizeEncoderValue(MOTOR_CONNECTOR.PositionRefCommanded());
    Console.print(calculatedPositionMm, 2);
    Console.print(F("mm ("));
    Console.print(currentPulses);
    Console.print(F(" counts)"));

    // Target information
    Console.print(F(", Target="));
    if (motorState == MOTOR_STATE_MOVING || motorState == MOTOR_STATE_HOMING)
    {
        if (hasCurrentTarget)
        {
            Console.print(currentTargetPositionMm, 2);
            Console.print(F("mm ("));
            Console.print(normalizeEncoderValue(currentTargetPulses));
            Console.print(F(" counts)"));
        }
        else
        {
            Console.print(F("None"));
        }
    }
    else
    {
        Console.print(F("None"));
    }

    // Last target position
    Console.print(F(", LastTarget="));
    if (hasLastTarget)
    {
        Console.print(lastTargetPositionMm, 2);
        Console.print(F("mm ("));
        Console.print(normalizeEncoderValue(lastTargetPulses));
        Console.print(F(" counts)"));
    }
    else
    {
        Console.print(F("None"));
    }

    // 5. VELOCITY GROUP with improved naming
    Console.print(F(" | Velocity: "));
    double currentVelocityRpm = abs((double)MOTOR_CONNECTOR.VelocityRefCommanded() * 60.0 / PULSES_PER_REV);
    Console.print(currentVelocityRpm, 1);
    Console.print(F("RPM"));
    if (currentVelocityRpm > 0)
    {
        Console.print(F(" ("));
        Console.print((int)(currentVelocityRpm * 100 / ppsToRpm(currentVelMax)));
        Console.print(F("%)"));
    }

    // Limits (condensed)
    Console.print(F(", Limits: "));
    Console.print(ppsToRpm(currentVelMax), 0);
    Console.print(F("RPM/"));
    Console.print((double)currentAccelMax * 60.0 / PULSES_PER_REV, 0);
    Console.print(F("RPM/s"));

    // 6. JOG SETTINGS GROUP - removed (D) indicators
    Console.print(F(" | Jog: "));
    Console.print(currentJogIncrementMm, 1);
    Console.print(F("mm/"));
    Console.print(currentJogSpeedRpm);
    Console.print(F("RPM"));

    // Encoder status
    Console.print(F(" | MPG: "));
    if (encoderControlActive)
    {
        Console.print(F("ON x"));
        Console.print(getMultiplierName(currentMultiplier));

        // Add mm/rotation information in a compact format
        double mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
        Console.print(F(" ("));
        Console.print(mmPerRotation, 2);
        Console.print(F("mm/rot)"));
    }
    else
    {
        Console.print(F("OFF"));
    }

    // End the line
    Console.println();
}