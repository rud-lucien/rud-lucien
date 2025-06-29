#include "LogHistory.h"
#include "Utils.h" 

//=============================================================================
// GLOBAL INSTANCE
//=============================================================================
// Initialize the global instance
LogHistory opLogHistory;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
LogHistory::LogHistory() : head(0), count(0)
{
    // Initialize all entries to empty
    clear();
}

//=============================================================================
// LOG OPERATIONS
//=============================================================================
// Add a message to the history - stores exactly what was printed
void LogHistory::addEntry(const char *msg)
{
    // Store message with timestamp
    strncpy(entries[head].message, msg, sizeof(entries[head].message) - 1);
    entries[head].message[sizeof(entries[head].message) - 1] = '\0';
    entries[head].timestamp = millis();

    // Move head to next position (circular)
    head = (head + 1) % LOG_HISTORY_SIZE;

    // Update count
    if (count < LOG_HISTORY_SIZE)
        count++;
}

//=============================================================================
// HISTORY DISPLAY
//=============================================================================
// Print the log history
void LogHistory::printHistory()
{
    if (count == 0)
    {
        Console.println(F("No operation log history available"));
        return;
    }

    Console.println(F("\n----- OPERATION LOG HISTORY (OLDEST TO NEWEST) -----"));

    // Add statistics
    char statsMsg[100];
    sprintf(statsMsg, "History contains %d of %d possible entries", count, LOG_HISTORY_SIZE);
    Console.println(statsMsg);

    // Calculate the index of the oldest entry
    uint8_t oldestIdx = (head - count + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;

    // Start from oldest entry and move forward in time
    for (uint8_t i = 0; i < count; i++)
    {
        // Calculate index going forward from oldest
        uint8_t idx = (oldestIdx + i) % LOG_HISTORY_SIZE;

        // Build complete log entry message
        char timeBuffer[12];
        formatAbsoluteTime(entries[idx].timestamp, timeBuffer);
        
        char logEntry[200];
        sprintf(logEntry, "[%s] %s", timeBuffer, entries[idx].message);
        Console.println(logEntry);
    }
    Console.println(F("-----------------------------------------\n"));
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
// Clear all entries
void LogHistory::clear()
{
    for (uint8_t i = 0; i < LOG_HISTORY_SIZE; i++)
    {
        entries[i].message[0] = '\0';
        entries[i].timestamp = 0;
    }
    head = 0;
    count = 0;
}



