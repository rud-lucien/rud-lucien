#line 1 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/LogHistory.h"
#ifndef LOG_HISTORY_H
#define LOG_HISTORY_H

#include "Arduino.h"
#include "OutputManager.h"
#include "CommandController.h"

//=============================================================================
// CONSTANTS
//=============================================================================
// Log history size - how many entries to keep
#define LOG_HISTORY_SIZE 100


//=============================================================================
// TYPE DEFINITIONS
//=============================================================================
// Log entry structure - simplified
struct LogEntry {
    char message[100];        // Complete message with tag already included
    unsigned long timestamp; // When the message was logged
};

//=============================================================================
// LOG HISTORY CLASS
//=============================================================================
// Circular buffer for operation log history
class LogHistory {
private:
    LogEntry entries[LOG_HISTORY_SIZE];
    uint8_t head;
    uint8_t count;
    
public:
    LogHistory();
    
    // Add a message to the history
    void addEntry(const char* msg);
    
    // Print the log history
    void printHistory();
    
    // Clear all entries
    void clear();
};

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// Global instance
extern LogHistory opLogHistory;


#endif // LOG_HISTORY_H