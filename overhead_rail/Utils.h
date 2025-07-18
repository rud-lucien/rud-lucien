#ifndef UTILS_H
#define UTILS_H

#include "Arduino.h"

//=============================================================================
// SCALE FACTOR CONSTANTS
//=============================================================================
#define SCALE_FACTOR 100  // Scale factor for integer math (1mm = 100 units)

//=============================================================================
// STANDARD MESSAGE BUFFER SIZES
//=============================================================================
// Standard buffer sizes based on actual usage analysis
#define SMALL_MSG_SIZE   80   // Simple status messages, motor names, basic info
#define MEDIUM_MSG_SIZE  140  // Detailed operations, position info, formatted output  
#define LARGE_MSG_SIZE   280  // Complex multi-line status, detailed reports
#define ALERT_MSG_SIZE   220  // Motor alert lists and concatenated messages

//=============================================================================
// SCALING CONVERSION FUNCTIONS  
//=============================================================================
// Convert mm to scaled integer units
int32_t mmToScaled(double mm);

// Convert scaled integer units to mm
double scaledToMm(int32_t scaled);

//=============================================================================
// TIME HANDLING FUNCTIONS
//=============================================================================
// Safe time difference calculation that handles rollover
unsigned long timeDiff(unsigned long current, unsigned long previous);

// Safe timeout check that handles rollover
bool timeoutElapsed(unsigned long current, unsigned long previous, unsigned long timeout);

// Safe waiting check
bool waitTimeReached(unsigned long current, unsigned long previous, unsigned long waitTime);

// Convert seconds to a human-readable format
void printHumanReadableTime(unsigned long secondsAgo);

// Format seconds to a human-readable string
void formatHumanReadableTime(unsigned long secondsAgo, char* buffer, size_t bufferSize);

// Format absolute time (milliseconds since startup) to HH:MM:SS format
void formatAbsoluteTime(unsigned long timeMs, char *buffer);

//=============================================================================
// COLOR OUTPUT FUNCTIONS
//=============================================================================
// Print colored state values for system status output
void printColoredState(const char* state);
void printColoredYesNo(bool value);
void printColoredActiveInactive(bool active);
void printColoredSufficient(bool sufficient);
void printColoredPassed(bool passed);

#endif // UTILS_H