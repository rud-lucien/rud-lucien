#include "Logging.h"
#include "Utils.h"

//=============================================================================
// PROGMEM STRING CONSTANTS FOR MEMORY EFFICIENCY
//=============================================================================
// Format strings for sprintf_P()
const char FMT_SENSOR_SECTION[] PROGMEM = " | Sensors: R1-WC1=%s, R1-WC1-Lab=%s, R1-WC2=%s, R1-WC2-Lab=%s, R1-HANDOFF=%s, R2-WC3=%s, R2-WC3-Lab=%s, R2-HANDOFF=%s, HANDOFF-Lab=%s";
const char FMT_SYSTEM_SECTION[] PROGMEM = " | System: E-Stop=%s, Pressure=%d.%02d PSI%s, Clients=%d";
const char FMT_MOTOR_SECTION[] PROGMEM = " | R%d-Motor: State=%s, Homed=%s, HLFB=%s";
const char FMT_POSITION_UNHOMED[] PROGMEM = " | R%d-Position: UNKNOWN (not homed), Target=None, LastTarget=None";
const char FMT_POSITION_HOMED[] PROGMEM = " | R%d-Position: %.2fmm (%ld counts), Target=%s, LastTarget=%s";
const char FMT_VELOCITY_SECTION[] PROGMEM = " | R%d-Velocity: %.1fRPM, Limits: %.0fRPM/%.0fRPM/s";
const char FMT_VELOCITY_PERCENT[] PROGMEM = " (%d%%)";
const char FMT_MPG_ON[] PROGMEM = " | MPG: ON x%s (%.2fmm/rot) on Rail %d";
const char FMT_MPG_OFF[] PROGMEM = " | MPG: OFF";

// Static strings using PROGMEM
const char STR_PRESENT[] PROGMEM = "PRESENT";
const char STR_ABSENT[] PROGMEM = "ABSENT";
const char STR_TRIGGERED[] PROGMEM = "TRIGGERED";
const char STR_RELEASED[] PROGMEM = "RELEASED";
const char STR_LOW[] PROGMEM = " (LOW)";
const char STR_EMPTY[] PROGMEM = "";
const char STR_YES[] PROGMEM = "YES";
const char STR_NO[] PROGMEM = "NO";
const char STR_ASSERTED[] PROGMEM = "ASSERTED";
const char STR_NOT_ASSERTED[] PROGMEM = "NOT_ASSERTED";
const char STR_NONE[] PROGMEM = "None";
const char STR_MOVING[] PROGMEM = "Moving...";
const char STR_CYLINDER_RETRACTED[] PROGMEM = "Cylinder=RETRACTED";
const char STR_CYLINDER_RETRACTED_UNCERTAIN[] PROGMEM = "Cylinder=RETRACTED?[!]";
const char STR_CYLINDER_EXTENDED[] PROGMEM = "Cylinder=EXTENDED";
const char STR_CYLINDER_EXTENDED_UNCERTAIN[] PROGMEM = "Cylinder=EXTENDED?[!]";

// Motor state strings
const char STR_IDLE[] PROGMEM = "IDLE";
const char STR_MOVING_STATE[] PROGMEM = "MOVING";
const char STR_HOMING[] PROGMEM = "HOMING";
const char STR_FAULTED[] PROGMEM = "FAULTED";
const char STR_NOT_READY[] PROGMEM = "NOT_READY";
const char STR_UNKNOWN[] PROGMEM = "UNKNOWN";

// Initialize with logging disabled (logInterval = 0)
// but keep the default interval value of 250ms for when it's enabled
LoggingManagement logging = {0, 0};             // Initially disabled, previousLogTime=0, logInterval=0
const unsigned long DEFAULT_LOG_INTERVAL = 250; // Default interval of 250ms for responsive monitoring

void logSystemState()
{
    // Stream output directly instead of building large buffers
    // Start with bold white [LOG] tag
    Console.print(F("\x1b[1;37m[LOG]\x1b[0m "));
    
    // 1. VALVES section - Pneumatic cylinder status with sensor feedback
    printValveSection();
    
    // 2. SENSORS section - All carriage and labware sensors
    printSensorSection();
    
    // 3. SYSTEM section - E-Stop, pressure, clients
    printSystemSection();
    
    // 4. RAIL 1 sections
    printMotorSection(1);
    printPositionSection(1);
    printVelocitySection(1);
    
    // 5. RAIL 2 sections
    printMotorSection(2);
    printPositionSection(2);
    printVelocitySection(2);
    
    // 6. MPG (Manual Pulse Generator) section
    printMPGSection();
    
    // End with newline
    Console.println();
}

void printValveSection()
{
    Console.print(F("\x1b[1;36mValves:\x1b[0m"));
    
    // Get valve state and sensor feedback
    ValvePosition valveState = getValvePosition();
    bool actuallyRetracted = isCylinderActuallyRetracted();
    bool actuallyExtended = isCylinderActuallyExtended();
    bool positionValidated = validateValvePosition();
    
    if (valveState == VALVE_POSITION_RETRACTED) {
        if (positionValidated && actuallyRetracted) {
            Console.print(F("Cylinder=\x1b[32mRETRACTED\x1b[0m"));
        } else {
            Console.print(F("Cylinder=\x1b[1;31mRETRACTED?\x1b[1;31m[!]\x1b[0m"));
        }
    } else {
        if (positionValidated && actuallyExtended) {
            Console.print(F("Cylinder=\x1b[33mEXTENDED\x1b[0m"));
        } else {
            Console.print(F("Cylinder=\x1b[1;31mEXTENDED?\x1b[1;31m[!]\x1b[0m"));
        }
    }
}

void printSensorSection()
{
    char sensorInfo[300];
    sprintf_P(sensorInfo, FMT_SENSOR_SECTION,
        isCarriageAtWC1() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isLabwarePresentAtWC1() ? PSTR("PRESENT") : PSTR("ABSENT"), 
        isCarriageAtWC2() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isLabwarePresentAtWC2() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isCarriageAtRail1Handoff() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isCarriageAtWC3() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isLabwarePresentAtWC3() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isCarriageAtRail2Handoff() ? PSTR("PRESENT") : PSTR("ABSENT"),
        isLabwarePresentAtHandoff() ? PSTR("PRESENT") : PSTR("ABSENT")
    );
    
    printColoredSensorSection(sensorInfo);
}

void printSystemSection()
{
    char systemInfo[ALERT_MSG_SIZE];
    uint16_t pressurePsi = getPressurePsi();
    sprintf_P(systemInfo, FMT_SYSTEM_SECTION,
        isEStopActive() ? PSTR("TRIGGERED") : PSTR("RELEASED"),
        pressurePsi / 100, pressurePsi % 100,
        isPressureSufficient() ? PSTR("") : PSTR(" (LOW)"),
        getConnectedClientCount()
    );
    
    printColoredSystemSection(systemInfo);
}

void printMotorSection(int railNumber)
{
    char motorInfo[MEDIUM_MSG_SIZE];
    const char* motorStateStr;
    MotorState state = updateMotorState(railNumber);
    
    // Convert motor state to PROGMEM string
    switch (state) {
    case MOTOR_STATE_IDLE:
        motorStateStr = PSTR("IDLE");
        break;
    case MOTOR_STATE_MOVING:
        motorStateStr = PSTR("MOVING");
        break;
    case MOTOR_STATE_HOMING:
        motorStateStr = PSTR("HOMING");
        break;
    case MOTOR_STATE_FAULTED:
        motorStateStr = PSTR("FAULTED");
        break;
    case MOTOR_STATE_NOT_READY:
        motorStateStr = PSTR("NOT_READY");
        break;
    default:
        motorStateStr = PSTR("UNKNOWN");
        break;
    }
    
    // Get HLFB state directly from motor connector
    MotorDriver& motor = getMotorByRail(railNumber);
    bool hlfbAsserted = (motor.HlfbState() == MotorDriver::HLFB_ASSERTED);
    
    sprintf_P(motorInfo, FMT_MOTOR_SECTION,
        railNumber,
        motorStateStr,
        isHomingComplete(railNumber) ? PSTR("YES") : PSTR("NO"),
        hlfbAsserted ? PSTR("ASSERTED") : PSTR("NOT_ASSERTED")
    );
    
    printColoredMotorSection(motorInfo);
}

void printPositionSection(int railNumber)
{
    char positionInfo[ALERT_MSG_SIZE];
    
    if (!isHomingComplete(railNumber)) {
        sprintf_P(positionInfo, FMT_POSITION_UNHOMED, railNumber);
    } else {
        double currentPosMm = getMotorPositionMm(railNumber);
        MotorDriver& motor = getMotorByRail(railNumber);
        int32_t currentPulses = motor.PositionRefCommanded();
        
        const char* targetStr = isMotorMoving(railNumber) ? PSTR("Moving...") : PSTR("None");
        
        sprintf_P(positionInfo, FMT_POSITION_HOMED,
            railNumber, currentPosMm, currentPulses, targetStr, PSTR("None")
        );
    }
    
    printColoredPositionSection(positionInfo);
}

void printVelocitySection(int railNumber)
{
    char velocityInfo[MEDIUM_MSG_SIZE];
    char percentStr[20] = "";
    
    // Get velocity directly from motor connector
    MotorDriver& motor = getMotorByRail(railNumber);
    double currentVelocityRpm = abs((double)motor.VelocityRefCommanded() * 60.0 / PULSES_PER_REV);
    
    // Get velocity limits based on rail
    double maxVelocityRpm = (railNumber == 1) ? RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM : RAIL2_EMPTY_CARRIAGE_VELOCITY_RPM;
    double accelerationRpmPerSec = (double)MAX_ACCEL_RPM_PER_SEC;
    
    sprintf_P(velocityInfo, FMT_VELOCITY_SECTION,
        railNumber, currentVelocityRpm, maxVelocityRpm, accelerationRpmPerSec
    );
    
    if (currentVelocityRpm > 0) {
        // Safeguard against division by zero in percentage calculation
        if (maxVelocityRpm > 0) {
            int percentOfMax = (int)(currentVelocityRpm * 100 / maxVelocityRpm);
            sprintf_P(percentStr, FMT_VELOCITY_PERCENT, percentOfMax);
        } else {
            // Motor is moving but max velocity limit is zero - configuration issue
            strcpy_P(percentStr, PSTR(" (velocity limits not set)"));
        }
        
        // Safe concatenation with bounds checking
        size_t currentLen = strlen(velocityInfo);
        size_t percentLen = strlen(percentStr);
        if (currentLen + percentLen < MEDIUM_MSG_SIZE - 1) {
            strncat(velocityInfo, percentStr, MEDIUM_MSG_SIZE - currentLen - 1);
        } else {
            // Truncation indicator if buffer would overflow
            if (MEDIUM_MSG_SIZE > currentLen + 4) {
                strncat(velocityInfo, "...", MEDIUM_MSG_SIZE - currentLen - 1);
            }
        }
    }
    
    printColoredVelocitySection(velocityInfo);
}

void printMPGSection()
{
    char mpgInfo[MEDIUM_MSG_SIZE];
    
    if (encoderControlActive) {
        const char* multiplierName = getMultiplierName(currentMultiplierScaled);
        double mmPerRotation = (double)currentMultiplierScaled / 100.0;
        sprintf_P(mpgInfo, FMT_MPG_ON,
            multiplierName, mmPerRotation, activeEncoderRail
        );
    } else {
        strcpy_P(mpgInfo, FMT_MPG_OFF);
    }
    
    printColoredMPGSection(mpgInfo);
}

// Memory-efficient color printing functions
void printColoredSensorSection(const char* sensorInfo)
{
    Console.print(F("\x1b[90m|\x1b[0m\x1b[1;36m Sensors:\x1b[0m "));
    
    const char* ptr = sensorInfo + 11; // Skip " | Sensors: "
    while (*ptr) {
        if (strncmp(ptr, "PRESENT", 7) == 0) {
            Console.print(F("\x1b[32mPRESENT\x1b[0m"));
            ptr += 7;
        }
        else if (strncmp(ptr, "ABSENT", 6) == 0) {
            Console.print(F("ABSENT"));
            ptr += 6;
        }
        else {
            Console.write(*ptr);
            ptr++;
        }
    }
}

void printColoredSystemSection(const char* systemInfo)
{
    Console.print(F("\x1b[90m|\x1b[0m\x1b[1;36m System:\x1b[0m "));
    
    const char* ptr = systemInfo + 10; // Skip " | System: "
    while (*ptr) {
        if (strncmp(ptr, "TRIGGERED", 9) == 0) {
            Console.print(F("\x1b[1;31mTRIGGERED\x1b[0m"));
            ptr += 9;
        }
        else if (strncmp(ptr, "RELEASED", 8) == 0) {
            Console.print(F("\x1b[32mRELEASED\x1b[0m"));
            ptr += 8;
        }
        else if (strncmp(ptr, " (LOW)", 6) == 0) {
            Console.print(F("\x1b[1;31m (LOW)\x1b[0m"));
            ptr += 6;
        }
        else {
            Console.write(*ptr);
            ptr++;
        }
    }
}

void printColoredMotorSection(const char* motorInfo)
{
    Console.print(F("\x1b[90m|\x1b[0m"));
    
    const char* ptr = motorInfo + 3; // Skip " | "
    while (*ptr) {
        if (strncmp(ptr, "R1-Motor:", 9) == 0) {
            Console.print(F("\x1b[1;36mR1-Motor:\x1b[0m"));
            ptr += 9;
        }
        else if (strncmp(ptr, "R2-Motor:", 9) == 0) {
            Console.print(F("\x1b[1;36mR2-Motor:\x1b[0m"));
            ptr += 9;
        }
        else if (strncmp(ptr, "IDLE", 4) == 0) {
            Console.print(F("\x1b[32mIDLE\x1b[0m"));
            ptr += 4;
        }
        else if (strncmp(ptr, "MOVING", 6) == 0) {
            Console.print(F("\x1b[33mMOVING\x1b[0m"));
            ptr += 6;
        }
        else if (strncmp(ptr, "HOMING", 6) == 0) {
            Console.print(F("\x1b[33mHOMING\x1b[0m"));
            ptr += 6;
        }
        else if (strncmp(ptr, "FAULTED", 7) == 0) {
            Console.print(F("\x1b[1;31mFAULTED\x1b[0m"));
            ptr += 7;
        }
        else if (strncmp(ptr, "NOT_READY", 9) == 0) {
            Console.print(F("\x1b[1;31mNOT_READY\x1b[0m"));
            ptr += 9;
        }
        else if (strncmp(ptr, "UNKNOWN", 7) == 0) {
            Console.print(F("\x1b[1;31mUNKNOWN\x1b[0m"));
            ptr += 7;
        }
        else if (strncmp(ptr, "YES", 3) == 0 && (ptr[3] == ',' || ptr[3] == ' ' || ptr[3] == '\0')) {
            Console.print(F("\x1b[32mYES\x1b[0m"));
            ptr += 3;
        }
        else if (strncmp(ptr, "NO", 2) == 0 && (ptr[2] == ',' || ptr[2] == ' ' || ptr[2] == '\0')) {
            Console.print(F("\x1b[33mNO\x1b[0m"));
            ptr += 2;
        }
        else if (strncmp(ptr, "ASSERTED", 8) == 0) {
            Console.print(F("\x1b[32mASSERTED\x1b[0m"));
            ptr += 8;
        }
        else if (strncmp(ptr, "NOT_ASSERTED", 12) == 0) {
            Console.print(F("\x1b[33mNOT_ASSERTED\x1b[0m"));
            ptr += 12;
        }
        else {
            Console.write(*ptr);
            ptr++;
        }
    }
}

void printColoredPositionSection(const char* positionInfo)
{
    Console.print(F("\x1b[90m|\x1b[0m"));
    
    const char* ptr = positionInfo + 3; // Skip " | "
    while (*ptr) {
        if (strncmp(ptr, "R1-Position:", 12) == 0) {
            Console.print(F("\x1b[1;36mR1-Position:\x1b[0m"));
            ptr += 12;
        }
        else if (strncmp(ptr, "R2-Position:", 12) == 0) {
            Console.print(F("\x1b[1;36mR2-Position:\x1b[0m"));
            ptr += 12;
        }
        else if (strncmp(ptr, "Moving...", 9) == 0) {
            Console.print(F("\x1b[33mMoving...\x1b[0m"));
            ptr += 9;
        }
        else if (strncmp(ptr, "None", 4) == 0) {
            Console.print(F("None"));
            ptr += 4;
        }
        else {
            Console.write(*ptr);
            ptr++;
        }
    }
}

void printColoredVelocitySection(const char* velocityInfo)
{
    Console.print(F("\x1b[90m|\x1b[0m"));
    
    const char* ptr = velocityInfo;
    while (*ptr) {
        if (strncmp(ptr, "R1-Velocity:", 12) == 0) {
            Console.print(F("\x1b[1;36mR1-Velocity:\x1b[0m"));
            ptr += 12;
        }
        else if (strncmp(ptr, "R2-Velocity:", 12) == 0) {
            Console.print(F("\x1b[1;36mR2-Velocity:\x1b[0m"));
            ptr += 12;
        }
        else {
            Console.write(*ptr);
            ptr++;
        }
    }
}

void printColoredMPGSection(const char* mpgInfo)
{
    Console.print(F("\x1b[90m|\x1b[0m"));
    
    const char* ptr = mpgInfo + 3; // Skip " | "
    while (*ptr) {
        if (strncmp(ptr, "MPG:", 4) == 0) {
            Console.print(F("\x1b[1;36mMPG:\x1b[0m"));
            ptr += 4;
        }
        else if (strncmp(ptr, "ON ", 3) == 0) {
            Console.print(F("\x1b[32mON\x1b[0m "));
            ptr += 3;
        }
        else if (strncmp(ptr, "OFF", 3) == 0 && (ptr[3] == ' ' || ptr[3] == '\0')) {
            Console.print(F("OFF"));
            ptr += 3;
        }
        else {
            Console.write(*ptr);
            ptr++;
        }
    }
}
