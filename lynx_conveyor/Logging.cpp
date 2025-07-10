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

    uint16_t pressure = getPressurePsi();
    sprintf(section, " | System: Motor=%s, Homed=%s, E-Stop=%s, HLFB=%s, Clients=%d, Pressure=%d.%02d PSI%s",
            motorStateStr,
            isHomed ? "YES" : "NO",
            isEStopActive() ? "TRIGGERED" : "RELEASED",
            (MOTOR_CONNECTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) ? "ASSERTED" : "NOT_ASSERTED",
            getConnectedClientCount(),
            pressure / 100, pressure % 100,
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

    // Output the complete message with color enhancement
    printColoredSystemState(msg);
}

// Function to print colored system state output
void printColoredSystemState(const char* msg)
{
    // Start with bold white [LOG] tag
    Console.print("\x1b[1;37m[LOG]\x1b[0m ");
    
    const char* ptr = msg + 6; // Skip the "[LOG] " part
    
    while (*ptr) {
        // Section headers - Bold cyan
        if (strncmp(ptr, "Valves:", 7) == 0) {
            Console.print("\x1b[1;36mValves:\x1b[0m");
            ptr += 7;
        }
        else if (strncmp(ptr, "Sensors:", 8) == 0) {
            Console.print("\x1b[1;36mSensors:\x1b[0m");
            ptr += 8;
        }
        else if (strncmp(ptr, "System:", 7) == 0) {
            Console.print("\x1b[1;36mSystem:\x1b[0m");
            ptr += 7;
        }
        else if (strncmp(ptr, "Position:", 9) == 0) {
            Console.print("\x1b[1;36mPosition:\x1b[0m");
            ptr += 9;
        }
        else if (strncmp(ptr, "Velocity:", 9) == 0) {
            Console.print("\x1b[1;36mVelocity:\x1b[0m");
            ptr += 9;
        }
        else if (strncmp(ptr, "Jog:", 4) == 0) {
            Console.print("\x1b[1;36mJog:\x1b[0m");
            ptr += 4;
        }
        else if (strncmp(ptr, "MPG:", 4) == 0) {
            Console.print("\x1b[1;36mMPG:\x1b[0m");
            ptr += 4;
        }
        // Critical safety indicator - BRIGHT RED
        else if (strncmp(ptr, "[!]", 3) == 0) {
            Console.print("\x1b[1;31m[!]\x1b[0m"); // Bold red for position mismatch warning
            ptr += 3;
        }
        // Valve states
        else if (strncmp(ptr, "LOCKED?", 7) == 0) {
            Console.print("\x1b[1;31mLOCKED?\x1b[0m"); // Bold red for uncertain locked state
            ptr += 7;
        }
        else if (strncmp(ptr, "UNLOCKED?", 9) == 0) {
            Console.print("\x1b[1;31mUNLOCKED?\x1b[0m"); // Bold red for uncertain unlocked state
            ptr += 9;
        }
        else if (strncmp(ptr, "LOCKED", 6) == 0) {
            Console.print("\x1b[32mLOCKED\x1b[0m"); // Green for confirmed locked
            ptr += 6;
        }
        else if (strncmp(ptr, "UNLOCKED", 8) == 0) {
            Console.print("\x1b[33mUNLOCKED\x1b[0m"); // Yellow for confirmed unlocked
            ptr += 8;
        }
        // Sensor states
        else if (strncmp(ptr, "PRESENT", 7) == 0) {
            Console.print("\x1b[32mPRESENT\x1b[0m"); // Green for tray present
            ptr += 7;
        }
        else if (strncmp(ptr, "EMPTY", 5) == 0) {
            Console.print("EMPTY"); // White (normal text) - readable
            ptr += 5;
        }
        // Motor states
        else if (strncmp(ptr, "IDLE", 4) == 0) {
            Console.print("\x1b[32mIDLE\x1b[0m"); // Green for ready state
            ptr += 4;
        }
        else if (strncmp(ptr, "MOVING", 6) == 0) {
            Console.print("\x1b[33mMOVING\x1b[0m"); // Yellow for active
            ptr += 6;
        }
        else if (strncmp(ptr, "HOMING", 6) == 0) {
            Console.print("\x1b[33mHOMING\x1b[0m"); // Yellow for active
            ptr += 6;
        }
        else if (strncmp(ptr, "FAULTED", 7) == 0) {
            Console.print("\x1b[1;31mFAULTED\x1b[0m"); // Bold red for fault
            ptr += 7;
        }
        else if (strncmp(ptr, "NOT_READY", 9) == 0) {
            Console.print("\x1b[1;31mNOT_READY\x1b[0m"); // Bold red for not ready
            ptr += 9;
        }
        else if (strncmp(ptr, "UNKNOWN", 7) == 0) {
            Console.print("\x1b[1;31mUNKNOWN\x1b[0m"); // Bold red for unknown state
            ptr += 7;
        }
        // Homing status
        else if (strncmp(ptr, "YES", 3) == 0 && (ptr[3] == ',' || ptr[3] == ' ' || ptr[3] == '\0')) {
            Console.print("\x1b[32mYES\x1b[0m"); // Green for homed
            ptr += 3;
        }
        else if (strncmp(ptr, "NO", 2) == 0 && (ptr[2] == ',' || ptr[2] == ' ' || ptr[2] == '\0')) {
            Console.print("\x1b[33mNO\x1b[0m"); // Yellow for not homed
            ptr += 2;
        }
        // Safety states
        else if (strncmp(ptr, "TRIGGERED", 9) == 0) {
            Console.print("\x1b[1;31mTRIGGERED\x1b[0m"); // Bold red for E-stop
            ptr += 9;
        }
        else if (strncmp(ptr, "RELEASED", 8) == 0) {
            Console.print("\x1b[32mRELEASED\x1b[0m"); // Green for normal
            ptr += 8;
        }
        // HLFB states
        else if (strncmp(ptr, "ASSERTED", 8) == 0) {
            Console.print("\x1b[32mASSERTED\x1b[0m"); // Green for motor happy
            ptr += 8;
        }
        else if (strncmp(ptr, "NOT_ASSERTED", 12) == 0) {
            Console.print("\x1b[33mNOT_ASSERTED\x1b[0m"); // Yellow for motor busy
            ptr += 12;
        }
        // Pressure warning
        else if (strncmp(ptr, " (LOW)", 6) == 0) {
            Console.print("\x1b[1;31m (LOW)\x1b[0m"); // Bold red for low pressure
            ptr += 6;
        }
        // MPG states  
        else if (strncmp(ptr, "ON ", 3) == 0) {
            Console.print("\x1b[32mON\x1b[0m "); // Green for MPG active
            ptr += 3;
        }
        else if (strncmp(ptr, "OFF", 3) == 0 && (ptr[3] == ' ' || ptr[3] == '\0')) {
            Console.print("OFF"); // White (normal text) instead of gray
            ptr += 3;
        }
        // Target states
        else if (strncmp(ptr, "None", 4) == 0) {
            Console.print("None"); // White (normal text) for better readability
            ptr += 4;
        }
        // Separators - subtle gray
        else if (*ptr == '|') {
            Console.print("\x1b[90m|\x1b[0m"); // Keep separators gray for structure
            ptr++;
        }
        else {
            Console.write(*ptr); // Print regular characters as normal white text
            ptr++;
        }
    }
    Console.println();
}