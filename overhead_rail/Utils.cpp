#include "Utils.h"
#include "OutputManager.h"

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_SECONDS[] PROGMEM = "%lu second%s";
const char FMT_MINUTES_SECONDS[] PROGMEM = "%lu minute%s %lu second%s";
const char FMT_MINUTES[] PROGMEM = "%lu minute%s";
const char FMT_HOURS_MINUTES[] PROGMEM = "%lu hour%s %lu minute%s";
const char FMT_HOURS[] PROGMEM = "%lu hour%s";
const char FMT_DAYS_HOURS[] PROGMEM = "%lu day%s %lu hour%s";
const char FMT_DAYS[] PROGMEM = "%lu day%s";

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

// Safe time difference calculation that handles rollover
unsigned long timeDiff(unsigned long current, unsigned long previous)
{
    return current - previous; // Correctly handles rollover with unsigned arithmetic
}

// Safe timeout check that handles rollover
bool timeoutElapsed(unsigned long current, unsigned long previous, unsigned long timeout)
{
    return timeDiff(current, previous) >= timeout;
}

// Safe waiting check
bool waitTimeReached(unsigned long current, unsigned long previous, unsigned long waitTime)
{
    return timeDiff(current, previous) >= waitTime;
}

// Helper function to print time in a human-readable format
void printHumanReadableTime(unsigned long secondsAgo)
{
    char timeMsg[80];

    if (secondsAgo < 60)
    {
        // Less than a minute
        sprintf_P(timeMsg, FMT_SECONDS, secondsAgo, (secondsAgo != 1) ? "s" : "");
    }
    else if (secondsAgo < 3600)
    {
        // Less than an hour
        unsigned long minutes = secondsAgo / 60;
        unsigned long seconds = secondsAgo % 60;
        if (seconds > 0)
        {
            sprintf_P(timeMsg, FMT_MINUTES_SECONDS,
                    minutes, (minutes != 1) ? "s" : "",
                    seconds, (seconds != 1) ? "s" : "");
        }
        else
        {
            sprintf_P(timeMsg, FMT_MINUTES, minutes, (minutes != 1) ? "s" : "");
        }
    }
    else if (secondsAgo < 86400)
    {
        // Less than a day
        unsigned long hours = secondsAgo / 3600;
        unsigned long minutes = (secondsAgo % 3600) / 60;
        if (minutes > 0)
        {
            sprintf_P(timeMsg, FMT_HOURS_MINUTES,
                    hours, (hours != 1) ? "s" : "",
                    minutes, (minutes != 1) ? "s" : "");
        }
        else
        {
            sprintf_P(timeMsg, FMT_HOURS, hours, (hours != 1) ? "s" : "");
        }
    }
    else
    {
        // More than a day
        unsigned long days = secondsAgo / 86400;
        unsigned long hours = (secondsAgo % 86400) / 3600;
        if (hours > 0)
        {
            sprintf_P(timeMsg, FMT_DAYS_HOURS,
                    days, (days != 1) ? "s" : "",
                    hours, (hours != 1) ? "s" : "");
        }
        else
        {
            sprintf_P(timeMsg, FMT_DAYS, days, (days != 1) ? "s" : "");
        }
    }

    Serial.print(timeMsg);
}

// Format time as a human-readable string
void formatHumanReadableTime(unsigned long secondsAgo, char* buffer, size_t bufferSize)
{
    if (secondsAgo < 60)
    {
        // Less than a minute
        snprintf_P(buffer, bufferSize, FMT_SECONDS, secondsAgo, (secondsAgo != 1) ? "s" : "");
    }
    else if (secondsAgo < 3600)
    {
        // Less than an hour
        unsigned long minutes = secondsAgo / 60;
        unsigned long seconds = secondsAgo % 60;
        if (seconds > 0)
        {
            snprintf_P(buffer, bufferSize, FMT_MINUTES_SECONDS,
                    minutes, (minutes != 1) ? "s" : "",
                    seconds, (seconds != 1) ? "s" : "");
        }
        else
        {
            snprintf_P(buffer, bufferSize, FMT_MINUTES, minutes, (minutes != 1) ? "s" : "");
        }
    }
    else if (secondsAgo < 86400)
    {
        // Less than a day
        unsigned long hours = secondsAgo / 3600;
        unsigned long minutes = (secondsAgo % 3600) / 60;
        if (minutes > 0)
        {
            snprintf_P(buffer, bufferSize, FMT_HOURS_MINUTES,
                    hours, (hours != 1) ? "s" : "",
                    minutes, (minutes != 1) ? "s" : "");
        }
        else
        {
            snprintf_P(buffer, bufferSize, FMT_HOURS, hours, (hours != 1) ? "s" : "");
        }
    }
    else
    {
        // More than a day
        unsigned long days = secondsAgo / 86400;
        unsigned long hours = (secondsAgo % 86400) / 3600;
        if (hours > 0)
        {
            snprintf_P(buffer, bufferSize, FMT_DAYS_HOURS,
                    days, (days != 1) ? "s" : "",
                    hours, (hours != 1) ? "s" : "");
        }
        else
        {
            snprintf_P(buffer, bufferSize, FMT_DAYS, days, (days != 1) ? "s" : "");
        }
    }
}

// Formats time as hours:minutes:seconds since system startup
void formatAbsoluteTime(unsigned long timeMs, char *buffer)
{
    // Calculate hours, minutes, seconds from milliseconds
    unsigned long totalSeconds = timeMs / 1000;
    unsigned long hours = totalSeconds / 3600;
    unsigned long minutes = (totalSeconds % 3600) / 60;
    unsigned long seconds = totalSeconds % 60;

    // Format as HH:MM:SS
    snprintf(buffer, 12, "%02lu:%02lu:%02lu", hours, minutes, seconds);
}

//=============================================================================
// COLOR OUTPUT FUNCTIONS
//=============================================================================
void printColoredState(const char* state) {
    if (strcmp(state, "NOT_READY") == 0 || strcmp(state, "FAULTED") == 0) {
        Console.print(F("\x1b[1;31m")); // Red for critical errors
        Console.print(state);
        Console.print(F("\x1b[0m"));
    }
    else if (strcmp(state, "IDLE") == 0) {
        Console.print(F("\x1b[32m")); // Green for ready states
        Console.print(state);
        Console.print(F("\x1b[0m"));
    }
    else if (strcmp(state, "MOVING") == 0 || strcmp(state, "HOMING") == 0) {
        Console.print(F("\x1b[33m")); // Yellow for transitional states
        Console.print(state);
        Console.print(F("\x1b[0m"));
    }
    else if (strcmp(state, "UNKNOWN") == 0) {
        Console.print(F("\x1b[90m")); // Gray for unknown states
        Console.print(state);
        Console.print(F("\x1b[0m"));
    }
    else {
        Console.print(state); // No color for other states
    }
}

void printColoredYesNo(bool value) {
    if (value) {
        Console.print(F("\x1b[32mYes\x1b[0m")); // Green for Yes
    } else {
        Console.print(F("\x1b[1;31mNo\x1b[0m")); // Red for No
    }
}

void printColoredActiveInactive(bool active) {
    if (active) {
        Console.print(F("\x1b[32mACTIVE\x1b[0m")); // Green for active
    } else {
        Console.print(F("\x1b[90mINACTIVE\x1b[0m")); // Gray for inactive
    }
}

void printColoredSufficient(bool sufficient) {
    if (sufficient) {
        Console.print(F("\x1b[32m(SUFFICIENT)\x1b[0m")); // Green for sufficient
    } else {
        Console.print(F("\x1b[1;31m(LOW)\x1b[0m")); // Red for low
    }
}

void printColoredPassed(bool passed) {
    if (passed) {
        Console.print(F("\x1b[32mPASSED\x1b[0m")); // Green for passed
    } else {
        Console.print(F("\x1b[1;31mFAILED\x1b[0m")); // Red for failed
    }
}

//=============================================================================
// SCALING CONVERSION FUNCTIONS
//=============================================================================
int32_t mmToScaled(double mm) {
    return (int32_t)(mm * SCALE_FACTOR);
}

double scaledToMm(int32_t scaled) {
    return (double)scaled / SCALE_FACTOR;
}