#ifndef LOGGING_H
#define LOGGING_H

#include "Sensors.h"
#include "MotorController.h"
#include "ValveController.h"
#include "EncoderController.h"
#include "EthernetController.h"
#include "OutputManager.h"
#include "Utils.h"

//=============================================================================
// TYPE DEFINITIONS
//=============================================================================

// Buffer sizes for logging functions
const size_t SENSOR_INFO_BUFFER_SIZE = 300;   // Buffer for sensor information string
const size_t VELOCITY_PERCENT_BUFFER_SIZE = 20; // Buffer for velocity percentage string

// Structure to manage periodic logging behavior
struct LoggingManagement
{
    unsigned long previousLogTime; // Time of last log
    unsigned long logInterval;     // Interval (ms) between logs, 0 = disabled
};

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Global logging management instance
extern LoggingManagement logging;

// Default interval between automatic logs in milliseconds (0 = disabled)
extern const unsigned long DEFAULT_LOG_INTERVAL;

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================

// Log the current state of the entire dual-rail system
void logSystemState();

// Memory-efficient section printing functions
void printValveSection();
void printSensorSection();
void printMotorSection(int railNumber);
void printPositionSection(int railNumber);
void printVelocitySection(int railNumber);
void printSystemSection();
void printMPGSection();

// Color printing helper functions
void printColoredSensorSection(const char* sensorInfo);
void printColoredSystemSection(const char* systemInfo);
void printColoredMotorSection(const char* motorInfo);
void printColoredPositionSection(const char* positionInfo);
void printColoredVelocitySection(const char* velocityInfo);
void printColoredMPGSection(const char* mpgInfo);

#endif // LOGGING_H
