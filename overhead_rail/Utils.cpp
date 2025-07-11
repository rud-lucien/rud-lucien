#include "Utils.h"

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

// Safe time comparison that handles rollover (true if timeA is after timeB)
bool isTimeAfter(unsigned long timeA, unsigned long timeB)
{
    // Using signed arithmetic to handle rollover correctly
    // This works because of two's complement arithmetic
    return (long)(timeA - timeB) > 0;
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

bool isRecentEvent(unsigned long eventTime, unsigned long maxAgeMs) {
    return timeDiff(millis(), eventTime) <= maxAgeMs;
}

void printTimeAgo(unsigned long eventTime) {
    unsigned long ageMs = timeDiff(millis(), eventTime);
    Serial.print("(");
    printHumanReadableTime(ageMs / 1000);
    Serial.print(" ago)");
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