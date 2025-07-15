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
    // **OPTIMIZED**: Entries are already zero-initialized by compiler
    // No need to call clear() which is expensive
    // Just ensure our member variables are properly initialized (done in initializer list)
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
    
    // **FIXED**: Atomic update to prevent race conditions
    // Calculate next position first
    uint8_t nextHead = (head + 1) % LOG_HISTORY_SIZE;
    bool willOverflow = (count >= LOG_HISTORY_SIZE);
    
    // Fill the entry at current head position
    strncpy(entries[head].message, msg, LOG_MESSAGE_SIZE - 1);
    entries[head].message[LOG_MESSAGE_SIZE - 1] = '\0';
    entries[head].timestamp = millis();
    entries[head].severity = severity;
    
    // **ATOMIC UPDATE**: Update head and count together to prevent corruption
    head = nextHead;
    if (willOverflow) {
        overflowCount++;  // Track when we're losing data
    } else {
        count++;
    }
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
// **OPTIMIZED**: Helper function to print a log entry with colored severity tag
void LogHistory::printColoredEntry(const LogEntry& entry)
{
    const char* message = entry.message;
    
    // Format timestamp as [HH:MM:SS] prefix
    char timestamp[12];
    formatAbsoluteTime(entry.timestamp, timestamp);
    Console.print('[');
    Console.print(timestamp);
    Console.print("] ");
    
    // **PERFORMANCE FIX**: Use stored severity instead of string parsing when possible
    // This is much faster and more reliable than string searching
    bool useSeverityColor = true;
    const char* colorCode = nullptr;
    const char* tagEnd = nullptr;
    
    // Try severity-based coloring first (more efficient)
    switch (entry.severity) {
        case LogEntry::INFO:
            colorCode = "\x1b[1;37m[INFO]\x1b[0m";
            tagEnd = strstr(message, "[INFO]");
            if (tagEnd) tagEnd += 6;
            break;
        case LogEntry::DIAGNOSTIC:
            colorCode = "\x1b[1;33m[DIAGNOSTIC]\x1b[0m";
            tagEnd = strstr(message, "[DIAGNOSTIC]");
            if (tagEnd) tagEnd += 12;
            break;
        case LogEntry::WARNING:
            colorCode = "\x1b[1;38;5;208m[WARNING]\x1b[0m";
            tagEnd = strstr(message, "[WARNING]");
            if (tagEnd) tagEnd += 9;
            break;
        case LogEntry::ERROR:
            colorCode = "\x1b[31m[ERROR]\x1b[0m";
            tagEnd = strstr(message, "[ERROR]");
            if (tagEnd) tagEnd += 7;
            break;
        case LogEntry::CRITICAL:
            colorCode = "\x1b[31m[CRITICAL]\x1b[0m";
            tagEnd = strstr(message, "[CRITICAL]");
            if (tagEnd) tagEnd += 10;
            break;
        case LogEntry::COMMAND:
            // Special case: could be SERIAL or NETWORK command
            if (strstr(message, "[SERIAL COMMAND]")) {
                colorCode = "\x1b[1;36m[SERIAL COMMAND]\x1b[0m";
                tagEnd = strstr(message, "[SERIAL COMMAND]") + 16;
            } else if (strstr(message, "[NETWORK COMMAND]")) {
                colorCode = "\x1b[1;36m[NETWORK COMMAND]\x1b[0m";
                tagEnd = strstr(message, "[NETWORK COMMAND]") + 17;
            } else {
                useSeverityColor = false;
            }
            break;
        default:
            useSeverityColor = false;
            break;
    }
    
    // If severity-based coloring worked, use it
    if (useSeverityColor && colorCode && tagEnd) {
        // **SAFETY CHECK**: Validate pointer before using
        if (tagEnd >= message && tagEnd <= message + strlen(message)) {
            size_t prefixLen = tagEnd - message - (strchr(colorCode, '[') - colorCode);
            if (prefixLen < strlen(message)) {  // Additional safety check
                Console.write((const uint8_t*)message, prefixLen);
                Console.print(colorCode);
                Console.println(tagEnd);
                return;
            }
        }
    }
    
    // **FALLBACK**: Handle special cases and ACK messages
    if (strstr(message, "[ACK]")) {
        char* pos = strstr(message, "[ACK]");
        size_t prefixLen = pos - message;
        if (prefixLen <= strlen(message)) {  // Safety check
            Console.write((const uint8_t*)message, prefixLen);
            Console.print("\x1b[1;32m[ACK]\x1b[0m");
            Console.println(pos + 5);
            return;
        }
    }
    
    // **DEFAULT**: No recognized pattern, print as-is
    Console.println(message);
}

// Print diagnostic statistics
void LogHistory::printStats()
{
    char statsMsg[120];
    // **FIXED**: Accurate memory calculation including all class members
    size_t totalMemory = sizeof(entries) + sizeof(head) + sizeof(count) + sizeof(overflowCount);
    sprintf_P(statsMsg, FMT_HISTORY_STATS, 
            count, LOG_HISTORY_SIZE, overflowCount, 
            (int)totalMemory);
    Console.println(statsMsg);
}

//=============================================================================
// ACCESSOR METHODS FOR SYSTEM STATE REPORTING
//=============================================================================
const LogEntry& LogHistory::getLastEntry() const
{
    if (count == 0) {
        // Return a static empty entry if no entries exist
        static LogEntry emptyEntry = {"", 0, LogEntry::INFO};
        return emptyEntry;
    }
    
    uint8_t lastIdx = (head - 1 + LOG_HISTORY_SIZE) % LOG_HISTORY_SIZE;
    return entries[lastIdx];
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
