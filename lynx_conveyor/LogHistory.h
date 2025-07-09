#ifndef LOG_HISTORY_H
#define LOG_HISTORY_H

#include <Arduino.h>
// REMOVED: #include "OutputManager.h"  // Fix circular dependency

//=============================================================================
// CONSTANTS
//=============================================================================
// Reduced log history size for better memory management
#define LOG_HISTORY_SIZE 100  
#define LOG_MESSAGE_SIZE 100  

//=============================================================================
// TYPE DEFINITIONS
//=============================================================================
// Log entry structure - enhanced with severity
struct LogEntry
{
    char message[LOG_MESSAGE_SIZE];       // Complete message with tag already included
    unsigned long timestamp; // When the message was logged
    
    // Add severity for quick filtering during debugging
    enum Severity {
        INFO = 0,
        DIAGNOSTIC = 1,
        WARNING = 2,
        ERROR = 3,
        CRITICAL = 4,
        COMMAND = 5     // For serial/network commands (cyan)
    } severity;
};

//=============================================================================
// LOG HISTORY CLASS
//=============================================================================
// Circular buffer for operation log history
class LogHistory
{
private:
    LogEntry entries[LOG_HISTORY_SIZE];
    volatile uint8_t head;      // volatile for thread safety
    volatile uint8_t count;
    uint16_t overflowCount;     // Track lost entries
    
    // Helper function for colored output
    void printColoredEntry(const LogEntry& entry);

public:
    LogHistory();

    // Enhanced addEntry with severity and safety
    void addEntry(const char *msg, LogEntry::Severity severity = LogEntry::INFO);
    
    // Multiple display options for debugging
    void printHistory();
    void printErrors();         // Show only errors/critical
    void printLastN(uint8_t n); // Show last N entries
    void printSince(unsigned long sinceTime);
    
    // Diagnostic info
    void printStats();
    uint16_t getOverflowCount() const { return overflowCount; }
    
    // Clear all entries
    void clear();
};

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Global instance
extern LogHistory opLogHistory;

#endif // LOG_HISTORY_H