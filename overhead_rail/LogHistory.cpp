#include "LogHistory.h"
#include "Utils.h"
#include "OutputManager.h"  // For Console color methods

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_HISTORY_STATS[] PROGMEM = "History: %d/%d entries, %d overflows, Memory: %d bytes";

//=============================================================================
// GLOBAL INSTANCE
//=============================================================================
// Initialize the global instance
LogHistory opLogHistory;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
LogHistory::LogHistory() : head(0), count(0), overflowCount(0)
{
    // Initialize all entries to empty
    clear();
}

//=============================================================================
// LOG OPERATIONS
//=============================================================================
// Add a message to the history with severity and thread safety
void LogHistory::addEntry(const char *msg, LogEntry::Severity severity)
{
    // Critical: Input validation for overnight reliability
    if (!msg || strlen(msg) == 0) {
        return;  // Don't crash on null/empty messages
    }
    
    // REMOVED: noInterrupts() - could interfere with motor timing
    // Use volatile variables for thread safety instead
    
    // Safe string copy with bounds checking
    strncpy(entries[head].message, msg, LOG_MESSAGE_SIZE - 1);
    entries[head].message[LOG_MESSAGE_SIZE - 1] = '\0';
    entries[head].timestamp = millis();
    entries[head].severity = severity;

    // Atomic operations for thread safety without disabling interrupts
    uint8_t nextHead = (head + 1) % LOG_HISTORY_SIZE;
    head = nextHead;

    // Track overflows - critical for debugging
    if (count < LOG_HISTORY_SIZE) {
        count++;
    } else {
        overflowCount++;  // Know when we're losing data
    }
    
    // REMOVED: interrupts() - not needed
}

//=============================================================================
// HISTORY DISPLAY
//=============================================================================
// Print the complete log history
void LogHistory::printHistory()
{
    if (count == 0)
    {
        Console.println(F("No operation log history available"));
        return;
    }

    Console.println(F("\n----- COMPLETE OPERATION LOG HISTORY -----"));
    
    // Add statistics
    printStats();

    // Calculate the index of the oldest entry
    uint8_t oldestIdx = (head - count + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;

    // Start from oldest entry and move forward in time
    for (uint8_t i = 0; i < count; i++)
    {
        // Calculate index going forward from oldest
        uint8_t idx = (oldestIdx + i) % LOG_HISTORY_SIZE;

        // Print entry with colored severity tag
        printColoredEntry(entries[idx]);
    }
    Console.println(F("-----------------------------------------\n"));
}

// Critical for overnight debugging - show only problems
void LogHistory::printErrors()
{
    Console.println(F("\n----- ERROR/WARNING HISTORY -----"));
    
    uint8_t oldestIdx = (head - count + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;
    uint8_t errorCount = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (oldestIdx + i) % LOG_HISTORY_SIZE;
        
        if (entries[idx].severity >= LogEntry::WARNING) {
            printColoredEntry(entries[idx]);
            errorCount++;
        }
    }
    
    if (errorCount == 0) {
        Console.println(F("No errors or warnings found"));
    }
    Console.println(F("----------------------------\n"));
}

// Show last N entries
void LogHistory::printLastN(uint8_t n)
{
    if (count == 0 || n == 0) return;
    
    Console.print(F("\n----- LAST "));
    Console.print(min(n, count));
    Console.println(F(" ENTRIES -----"));
    
    uint8_t startIdx = (n > count) ? 0 : count - n;
    uint8_t oldestIdx = (head - count + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;
    
    for (uint8_t i = startIdx; i < count; i++) {
        uint8_t idx = (oldestIdx + i) % LOG_HISTORY_SIZE;
        
        printColoredEntry(entries[idx]);
    }
    Console.println(F("-------------------\n"));
}

// Show entries since a specific time
void LogHistory::printSince(unsigned long sinceTime)
{
    Console.println(F("\n----- LOG ENTRIES SINCE SPECIFIED TIME -----"));
    
    uint8_t oldestIdx = (head - count + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;
    uint8_t matchCount = 0;
    
    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (oldestIdx + i) % LOG_HISTORY_SIZE;
        
        if (entries[idx].timestamp >= sinceTime) {
            printColoredEntry(entries[idx]);
            matchCount++;
        }
    }
    
    if (matchCount == 0) {
        Console.println(F("No entries found since specified time"));
    }
    Console.println(F("----------------------------\n"));
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
// Helper function to print a log entry with colored severity tag
void LogHistory::printColoredEntry(const LogEntry& entry)
{
    const char* message = entry.message;
    
    // Simple approach: look for common severity tags and replace with colored versions
    if (strstr(message, "[INFO]")) {
        // Find and replace [INFO] with colored version
        char* infoPos = strstr(message, "[INFO]");
        // Print everything before [INFO]
        size_t prefixLen = infoPos - message;
        Console.write((const uint8_t*)message, prefixLen);
        // Print colored [INFO]
        Console.print("\x1b[1;37m[INFO]\x1b[0m");
        // Print everything after [INFO]
        Console.println(infoPos + 6); // 6 = length of "[INFO]"
    }
    else if (strstr(message, "[WARNING]")) {
        char* pos = strstr(message, "[WARNING]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[1;38;5;208m[WARNING]\x1b[0m"); // Bold orange
        Console.println(pos + 9); // 9 = length of "[WARNING]"
    }
    else if (strstr(message, "[ERROR]")) {
        char* pos = strstr(message, "[ERROR]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[31m[ERROR]\x1b[0m");
        Console.println(pos + 7); // 7 = length of "[ERROR]"
    }
    else if (strstr(message, "[CRITICAL]")) {
        char* pos = strstr(message, "[CRITICAL]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[31m[CRITICAL]\x1b[0m");
        Console.println(pos + 10); // 10 = length of "[CRITICAL]"
    }
    else if (strstr(message, "[DIAGNOSTIC]")) {
        char* pos = strstr(message, "[DIAGNOSTIC]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[1;33m[DIAGNOSTIC]\x1b[0m"); // Bold yellow
        Console.println(pos + 12); // 12 = length of "[DIAGNOSTIC]"
    }
    else if (strstr(message, "[SERIAL COMMAND]")) {
        char* pos = strstr(message, "[SERIAL COMMAND]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[1;36m[SERIAL COMMAND]\x1b[0m"); // Bold cyan
        Console.println(pos + 16); // 16 = length of "[SERIAL COMMAND]"
    }
    else if (strstr(message, "[NETWORK COMMAND]")) {
        char* pos = strstr(message, "[NETWORK COMMAND]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[1;36m[NETWORK COMMAND]\x1b[0m"); // Bold cyan
        Console.println(pos + 17); // 17 = length of "[NETWORK COMMAND]"
    }
    else if (strstr(message, "[ACK]")) {
        char* pos = strstr(message, "[ACK]");
        size_t prefixLen = pos - message;
        Console.write((const uint8_t*)message, prefixLen);
        Console.print("\x1b[1;32m[ACK]\x1b[0m");
        Console.println(pos + 5); // 5 = length of "[ACK]"
    }
    else {
        // No recognized tag, print as-is
        Console.println(message);
    }
}

// Print diagnostic statistics
void LogHistory::printStats()
{
    char statsMsg[120];
    sprintf_P(statsMsg, FMT_HISTORY_STATS, 
            count, LOG_HISTORY_SIZE, overflowCount, 
            (int)(sizeof(LogEntry) * LOG_HISTORY_SIZE));
    Console.println(statsMsg);
}

// Clear all entries
void LogHistory::clear()
{
    // REMOVED: noInterrupts() - could interfere with motor timing
    for (uint8_t i = 0; i < LOG_HISTORY_SIZE; i++)
    {
        entries[i].message[0] = '\0';
        entries[i].timestamp = 0;
        entries[i].severity = LogEntry::INFO;
    }
    head = 0;
    count = 0;
    overflowCount = 0;
    // REMOVED: interrupts() - not needed
}
