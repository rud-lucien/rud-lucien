#include "OutputManager.h"
#include <Ethernet.h>
#include "LogHistory.h"
#include "CommandController.h"

// ANSI Color Codes for Serial Terminal
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_WHITE   "\x1b[37m"
#define ANSI_COLOR_ORANGE  "\x1b[38;5;208m"  // 256-color orange
#define ANSI_COLOR_GRAY    "\x1b[90m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// Bold variants
#define ANSI_BOLD_RED      "\x1b[1;31m"
#define ANSI_BOLD_GREEN    "\x1b[1;32m"
#define ANSI_BOLD_YELLOW   "\x1b[1;33m"
#define ANSI_BOLD_ORANGE   "\x1b[1;38;5;208m"
#define ANSI_BOLD_MAGENTA  "\x1b[1;35m"
#define ANSI_BOLD_WHITE    "\x1b[1;37m"
#define ANSI_BOLD_CYAN     "\x1b[1;36m"

// Constants
#define LOG_MESSAGE_BUFFER_SIZE 120

// Global instances
MultiPrint Console;

// Initialize the output manager
void initOutputManager()
{
    // Add Serial as default output
    Console.addOutput(&Serial);
    Console.setPrimaryInput(&Serial);
}

// Add an output destination
bool MultiPrint::addOutput(Print *output)
{
    if (outputCount < MAX_OUTPUTS)
    {
        outputs[outputCount++] = output;
        return true;
    }
    return false;
}

// Remove an output destination
bool MultiPrint::removeOutput(Print *output)
{
    for (int i = 0; i < outputCount; i++)
    {
        if (outputs[i] == output)
        {
            // Shift remaining outputs
            for (int j = i; j < outputCount - 1; j++)
            {
                outputs[j] = outputs[j + 1];
            }
            outputCount--;
            return true;
        }
    }
    return false;
}

// Critical fix: Proper implementation of write for single byte
size_t MultiPrint::write(uint8_t c)
{
    size_t written = 0;

    // Check for persistentClient
    if (currentClient == nullptr && persistentClient != nullptr)
    {
        currentClient = persistentClient;
    }

    // Write to all registered outputs
    for (int i = 0; i < outputCount; i++)
    {
        written += outputs[i]->write(c);
    }

    // IMPORTANT: Also write to current client if set AND not already in outputs
    if (currentClient != nullptr)
    {
        bool alreadyIncluded = false;
        for (int i = 0; i < outputCount; i++)
        {
            if (outputs[i] == currentClient)
            {
                alreadyIncluded = true;
                break;
            }
        }

        if (!alreadyIncluded)
        {
            written += currentClient->write(c);
        }
    }

    return written;
}

// Add similar implementation for buffer write
size_t MultiPrint::write(const uint8_t *buffer, size_t size)
{
    size_t written = 0;

    // Check for persistentClient
    if (currentClient == nullptr && persistentClient != nullptr)
    {
        currentClient = persistentClient;
    }

    // Write to all registered outputs
    for (int i = 0; i < outputCount; i++)
    {
        written += outputs[i]->write(buffer, size);
    }

    // Also write to current client if it exists and isn't in outputs
    if (currentClient != nullptr)
    {
        bool alreadyIncluded = false;
        for (int i = 0; i < outputCount; i++)
        {
            if (outputs[i] == currentClient)
            {
                alreadyIncluded = true;
                break;
            }
        }

        if (!alreadyIncluded)
        {
            written += currentClient->write(buffer, size);
        }
    }

    return written;
}

// Add implementation for remaining required Stream methods
int MultiPrint::available()
{
    return primaryInput ? primaryInput->available() : 0;
}

int MultiPrint::read()
{
    return primaryInput ? primaryInput->read() : -1;
}

int MultiPrint::peek()
{
    return primaryInput ? primaryInput->peek() : -1;
}

void MultiPrint::flush()
{
    // Only try to flush the current client if set
    // (Print objects don't have flush method)
    if (currentClient)
    {
        currentClient->flush();
    }
    // Don't try to flush outputs - Print class doesn't have flush()
}

// Add to OutputManager.cpp
void MultiPrint::acknowledge(const char *msg)
{
    print(ANSI_BOLD_GREEN "[ACK]" ANSI_COLOR_RESET ", ");
    println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[ACK] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::INFO);
}

void MultiPrint::acknowledge(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_GREEN "[ACK]" ANSI_COLOR_RESET ", ");
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[ACK] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::INFO);
}

// Helper methods implementation for formatted messages
void MultiPrint::info(const char *msg)
{
    print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    println(msg);

    // Add to history buffer with INFO severity (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[INFO] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::INFO);
}

void MultiPrint::info(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[INFO] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::INFO);
}

// New operational info methods that always get logged to history
void MultiPrint::opInfo(const char *msg)
{
    // Regular output
    this->print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    this->println(msg);

    // Also add to history (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[INFO] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::INFO);
}

void MultiPrint::opInfo(const __FlashStringHelper *msg)
{
    // Regular output
    this->print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    this->println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[INFO] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::INFO);
}

// Updated with history logging
void MultiPrint::error(const char *msg)
{
    print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET ", ");
    println(msg);

    // Always log errors to history with ERROR severity (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[ERROR] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::ERROR);
}

void MultiPrint::error(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET ", ");
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[ERROR] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::ERROR);
}

// Updated with history logging
void MultiPrint::diagnostic(const char *msg)
{
    print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    println(msg);

    // Log diagnostics to history with DIAGNOSTIC severity (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[DIAGNOSTIC] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::DIAGNOSTIC);
}

void MultiPrint::diagnostic(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[DIAGNOSTIC] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::DIAGNOSTIC);
}

// Updated with command filtering for history
void MultiPrint::serialCommand(const char *msg)
{
    print(ANSI_BOLD_CYAN "[SERIAL COMMAND]" ANSI_COLOR_RESET " ");
    println(msg);

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(msg))
    {
        // Debug - print to serial directly
        Serial.print(F("[DIAGNOSTIC] Adding to history: "));
        Serial.println(msg);

        char buffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "[SERIAL COMMAND] %s", msg);
        opLogHistory.addEntry(buffer, LogEntry::COMMAND);
    }
    else
    {
        Serial.print(F("[DIAGNOSTIC] Excluding from history: "));
        Serial.println(msg);
    }
}

void MultiPrint::serialCommand(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_CYAN "[SERIAL COMMAND]" ANSI_COLOR_RESET " ");
    println(msg);

    // Convert to regular string for filtering
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(buffer))
    {
        char logBuffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf(logBuffer, sizeof(logBuffer), "[SERIAL COMMAND] %s", buffer); // FIXED: Use correct label
        opLogHistory.addEntry(logBuffer, LogEntry::COMMAND);
    }
}

void MultiPrint::ethernetCommand(const char *msg)
{
    print(ANSI_BOLD_CYAN "[NETWORK COMMAND]" ANSI_COLOR_RESET " ");
    println(msg);

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(msg))
    { // Use msg directly
        char logBuffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf(logBuffer, sizeof(logBuffer), "[NETWORK COMMAND] %s", msg); // Use msg and fix label
        opLogHistory.addEntry(logBuffer, LogEntry::COMMAND);
    }
}

void MultiPrint::ethernetCommand(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_CYAN "[NETWORK COMMAND]" ANSI_COLOR_RESET " ");
    println(msg);

    // Convert to regular string for filtering
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(buffer))
    {
        char logBuffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf(logBuffer, sizeof(logBuffer), "[NETWORK COMMAND] %s", buffer);
        opLogHistory.addEntry(logBuffer, LogEntry::INFO);
    }
}

// Updated with history logging
void MultiPrint::warning(const char *msg)
{
    print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    println(msg);

    // Log warnings to history (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[WARNING] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::WARNING);
}

void MultiPrint::warning(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[WARNING] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::WARNING);
}

// Updated with history logging
void MultiPrint::safety(const char *msg)
{
    print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    println(msg);

    // Log safety messages to history (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[SAFETY] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::CRITICAL);
}

void MultiPrint::safety(const __FlashStringHelper *msg)
{
    print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[SAFETY] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::CRITICAL);
}

// The serial-only methods don't need to be modified since they don't use the buffer
void MultiPrint::serialInfo(const char *msg)
{
    Serial.print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[INFO] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::INFO);
}

void MultiPrint::serialInfo(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[INFO] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::INFO);
}

void MultiPrint::serialError(const char *msg)
{
    Serial.print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET ", ");
    Serial.println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[ERROR] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::ERROR);
}

void MultiPrint::serialError(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET ", ");
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[ERROR] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::ERROR);
}

void MultiPrint::serialDiagnostic(const char *msg)
{
    Serial.print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[DIAGNOSTIC] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::INFO);
}

void MultiPrint::serialDiagnostic(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[DIAGNOSTIC] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::INFO);
}

void MultiPrint::serialWarning(const char *msg)
{
    Serial.print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[WARNING] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::WARNING);
}

void MultiPrint::serialWarning(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[WARNING] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::WARNING);
}

void MultiPrint::serialSafety(const char *msg)
{
    Serial.print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[SAFETY] %s", msg);
    opLogHistory.addEntry(buffer, LogEntry::CRITICAL);
}

void MultiPrint::serialSafety(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[SAFETY] %s", buffer);
    opLogHistory.addEntry(logBuffer, LogEntry::CRITICAL);
}

// Note: isCommandExcludedFromHistory() is defined in CommandController.cpp

