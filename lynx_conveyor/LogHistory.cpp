#include "LogHistory.h"
#include "Utils.h" // For printHumanReadableTime

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

    // Add these statistics lines here
    Console.print(F("History contains "));
    Console.print(count);
    Console.print(F(" of "));
    Console.print(LOG_HISTORY_SIZE);
    Console.println(F(" possible entries"));

    // Calculate the index of the oldest entry
    uint8_t oldestIdx = (head - count + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;

    // Start from oldest entry and move forward in time
    for (uint8_t i = 0; i < count; i++)
    {
        // Calculate index going forward from oldest
        uint8_t idx = (oldestIdx + i) % LOG_HISTORY_SIZE;

        // Calculate absolute time
        unsigned long absoluteTime = entries[idx].timestamp;
        char timeBuffer[12];
        formatAbsoluteTime(absoluteTime, timeBuffer);

        // Print timestamp and message (only once)
        Console.print(F("["));
        Console.print(timeBuffer); // Something like [12:35:15]
        Console.print(F("] "));
        Console.println(entries[idx].message);
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

// Function to filter commands that shouldn't be logged to history
bool isCommandExcludedFromHistory(const char *command)
{
    // Get the first word (before comma or space)
    char firstWord[16] = {0};
    int i = 0;
    while (command[i] && command[i] != ',' && command[i] != ' ' && i < 15) {
        firstWord[i] = command[i];
        i++;
    }
    firstWord[i] = '\0';
    
    // Quick exclusions by first word only (no further parsing needed)
    if (strcmp(firstWord, "help") == 0 ||
        strcmp(firstWord, "status") == 0 ||
        strcmp(firstWord, "encoder") == 0 ||
        strcmp(firstWord, "log") == 0) {
        return true;
    }
    
    // Check combined commands
    if (strcmp(firstWord, "system") == 0) {
        // Get second part
        if (command[i] == ',') {
            const char *secondPart = command + i + 1;
            if (strcmp(secondPart, "state") == 0 ||
                strcmp(secondPart, "safety") == 0 ||
                strcmp(secondPart, "trays") == 0 ||
                strcmp(secondPart, "history") == 0) {
                return true;
            }
        }
        return false;
    }
    
    if (strcmp(firstWord, "jog") == 0) {
        if (command[i] == ',' && (strcmp(command+i+1, "inc") == 0 ||
                                 strcmp(command+i+1, "speed") == 0)) {
            return true;
        }
    }
    
    if (strcmp(firstWord, "network") == 0) {
        if (command[i] == ',') {
            const char *secondPart = command + i + 1;
            if (strncmp(secondPart, "close", 5) == 0 ||
                strcmp(secondPart, "status") == 0 ||
                strcmp(secondPart, "closeall") == 0) {
                return true;
            }
        }
    }
    
    // Include all other commands
    return false;
}