#line 1 "/Users/rlucien/Documents/GitHub/rud-lucien/lynx_conveyor/OutputManager.cpp"
#include "OutputManager.h"
#include <Ethernet.h>
#include "LogHistory.h"

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
    print(F("[ACK], "));
    println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[ACK] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::acknowledge(const __FlashStringHelper *msg)
{
    print(F("[ACK], "));
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[ACK] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// Helper methods implementation for formatted messages
void MultiPrint::info(const char *msg)
{
    print(F("[INFO] "));
    println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[INFO] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::info(const __FlashStringHelper *msg)
{
    print(F("[INFO] "));
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[INFO] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// New operational info methods that always get logged to history
void MultiPrint::opInfo(const char *msg)
{
    // Regular output
    this->print(F("[INFO] "));
    this->println(msg);

    // Also add to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[INFO] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::opInfo(const __FlashStringHelper *msg)
{
    // Regular output
    this->print(F("[INFO] "));
    this->println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[INFO] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// Updated with history logging
void MultiPrint::error(const char *msg)
{
    print(F("[ERROR], "));
    println(msg);

    // Always log errors to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[ERROR] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::error(const __FlashStringHelper *msg)
{
    print(F("[ERROR], "));
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[ERROR] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// Updated with history logging
void MultiPrint::diagnostic(const char *msg)
{
    print(F("[DIAGNOSTIC] "));
    println(msg);

    // Log diagnostics to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[DIAGNOSTIC] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::diagnostic(const __FlashStringHelper *msg)
{
    print(F("[DIAGNOSTIC] "));
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[DIAGNOSTIC] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// Updated with command filtering for history
void MultiPrint::serialCommand(const char *msg)
{
    print(F("[SERIAL COMMAND] "));
    println(msg);

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(msg))
    {
        // Debug - print to serial directly
        Serial.print(F("[DIAGNOSTIC] Adding to history: "));
        Serial.println(msg);

        char buffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "[SERIAL COMMAND] %s", msg);
        opLogHistory.addEntry(buffer);
    }
    else
    {
        Serial.print(F("[DIAGNOSTIC] Excluding from history: "));
        Serial.println(msg);
    }
}

void MultiPrint::serialCommand(const __FlashStringHelper *msg)
{
    print(F("[SERIAL COMMAND] "));
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
        opLogHistory.addEntry(logBuffer);
    }
}

void MultiPrint::ethernetCommand(const char *msg)
{
    print(F("[NETWORK COMMAND] "));
    println(msg);

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(msg))
    { // Use msg directly
        char logBuffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf(logBuffer, sizeof(logBuffer), "[NETWORK COMMAND] %s", msg); // Use msg and fix label
        opLogHistory.addEntry(logBuffer);
    }
}

void MultiPrint::ethernetCommand(const __FlashStringHelper *msg)
{
    print(F("[NETWORK COMMAND] "));
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
        opLogHistory.addEntry(logBuffer);
    }
}

// Updated with history logging
void MultiPrint::warning(const char *msg)
{
    print(F("[WARNING] "));
    println(msg);

    // Log warnings to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[WARNING] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::warning(const __FlashStringHelper *msg)
{
    print(F("[WARNING] "));
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[WARNING] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// Updated with history logging
void MultiPrint::safety(const char *msg)
{
    print(F("[SAFETY] "));
    println(msg);

    // Log safety messages to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[SAFETY] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::safety(const __FlashStringHelper *msg)
{
    print(F("[SAFETY] "));
    println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[SAFETY] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

// The serial-only methods don't need to be modified since they don't use the buffer
void MultiPrint::serialInfo(const char *msg)
{
    Serial.print(F("[INFO] "));
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[INFO] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialInfo(const __FlashStringHelper *msg)
{
    Serial.print(F("[INFO] "));
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[INFO] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

void MultiPrint::serialError(const char *msg)
{
    Serial.print(F("[ERROR], "));
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[ERROR] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialError(const __FlashStringHelper *msg)
{
    Serial.print(F("[ERROR], "));
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[ERROR] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

void MultiPrint::serialDiagnostic(const char *msg)
{
    Serial.print(F("[DIAGNOSTIC] "));
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[DIAGNOSTIC] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialDiagnostic(const __FlashStringHelper *msg)
{
    Serial.print(F("[DIAGNOSTIC] "));
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[DIAGNOSTIC] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

void MultiPrint::serialWarning(const char *msg)
{
    Serial.print(F("[WARNING] "));
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[WARNING] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialWarning(const __FlashStringHelper *msg)
{
    Serial.print(F("[WARNING] "));
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[WARNING] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}

void MultiPrint::serialSafety(const char *msg)
{
    Serial.print(F("[SAFETY] "));
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "[SAFETY] %s", msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialSafety(const __FlashStringHelper *msg)
{
    Serial.print(F("[SAFETY] "));
    Serial.println(msg);

    // Convert to regular string for history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    strncpy_P(buffer, (PGM_P)msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char logBuffer[80];
    snprintf(logBuffer, sizeof(logBuffer), "[SAFETY] %s", buffer);
    opLogHistory.addEntry(logBuffer);
}
