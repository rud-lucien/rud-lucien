#include "OutputManager.h"
#include <Ethernet.h>
#include "LogHistory.h"

//=============================================================================
// PROGMEM FORMAT STRINGS
//=============================================================================
const char FMT_ACK_LOG[] PROGMEM = "[ACK] %s";
const char FMT_INFO_LOG[] PROGMEM = "[INFO] %s";
const char FMT_ERROR_LOG[] PROGMEM = "[ERROR] %s";
const char FMT_WARNING_LOG[] PROGMEM = "[WARNING] %s";
const char FMT_DIAGNOSTIC_LOG[] PROGMEM = "[DIAGNOSTIC] %s";
const char FMT_SAFETY_LOG[] PROGMEM = "[SAFETY] %s";
const char FMT_SERIAL_COMMAND_LOG[] PROGMEM = "[SERIAL COMMAND] %s";
const char FMT_NETWORK_COMMAND_LOG[] PROGMEM = "[NETWORK COMMAND] %s";
const char FMT_INFO_S[] PROGMEM = "[INFO] %S";
const char FMT_ERROR_S[] PROGMEM = "[ERROR] %S";
const char FMT_DIAGNOSTIC_S[] PROGMEM = "[DIAGNOSTIC] %S";
const char FMT_WARNING_S[] PROGMEM = "[WARNING] %S";
const char FMT_SAFETY_S[] PROGMEM = "[SAFETY] %S";

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

// ACK messages - GREEN
void MultiPrint::acknowledge(const char *msg)
{
    print(ANSI_BOLD_GREEN "[ACK]" ANSI_COLOR_RESET ", ");
    println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_ACK_LOG, msg);
    opLogHistory.addEntry(buffer);
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
    snprintf_P(logBuffer, sizeof(logBuffer), FMT_ACK_LOG, buffer);
    opLogHistory.addEntry(logBuffer);
}

// INFO messages - BOLD WHITE (most common, should be prominent but neutral)
void MultiPrint::info(const char *msg)
{
    print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    println(msg);

    // Add to history buffer (without color codes)
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_INFO_LOG, msg);
    opLogHistory.addEntry(buffer);
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
    snprintf_P(logBuffer, sizeof(logBuffer), FMT_INFO_LOG, buffer);
    opLogHistory.addEntry(logBuffer);
}

// ERROR messages - RED
void MultiPrint::error(const char *msg)
{
    print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET ", ");
    println(msg);
    opLogHistory.addEntry(msg, LogEntry::ERROR);  // Mark as ERROR
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
    snprintf_P(logBuffer, sizeof(logBuffer), FMT_ERROR_LOG, buffer);
    opLogHistory.addEntry(logBuffer);
}

// WARNING messages - ORANGE
void MultiPrint::warning(const char *msg)
{
    print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    println(msg);

    // Log warnings to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_WARNING_LOG, msg);
    opLogHistory.addEntry(buffer);
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
    snprintf_P(logBuffer, sizeof(logBuffer), FMT_WARNING_LOG, buffer);
    opLogHistory.addEntry(logBuffer);
}

// DIAGNOSTIC messages - BOLD YELLOW (easier to read than regular yellow)
void MultiPrint::diagnostic(const char *msg)
{
    print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    println(msg);

    // Log diagnostics to history
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_DIAGNOSTIC_LOG, msg);
    opLogHistory.addEntry(buffer);
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
    snprintf_P(logBuffer, sizeof(logBuffer), FMT_DIAGNOSTIC_LOG, buffer);
    opLogHistory.addEntry(logBuffer);
}

// SAFETY messages - MAGENTA (high visibility)
void MultiPrint::safety(const char *msg)
{
    print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    println(msg);
    opLogHistory.addEntry(msg, LogEntry::CRITICAL);  // Mark as CRITICAL
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
    snprintf_P(logBuffer, sizeof(logBuffer), FMT_SAFETY_LOG, buffer);
    opLogHistory.addEntry(logBuffer);
}

// COMMAND messages - CYAN
void MultiPrint::serialCommand(const char *msg)
{
    print(ANSI_BOLD_CYAN "[SERIAL COMMAND]" ANSI_COLOR_RESET " ");
    println(msg);

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(msg))
    {
        char buffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf_P(buffer, sizeof(buffer), FMT_SERIAL_COMMAND_LOG, msg);
        opLogHistory.addEntry(buffer);
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
        snprintf_P(logBuffer, sizeof(logBuffer), FMT_SERIAL_COMMAND_LOG, buffer);
        opLogHistory.addEntry(logBuffer);
    }
}

void MultiPrint::ethernetCommand(const char *msg)
{
    print(ANSI_BOLD_CYAN "[NETWORK COMMAND]" ANSI_COLOR_RESET " ");
    println(msg);

    // Only log commands that pass the filter
    if (!isCommandExcludedFromHistory(msg))
    {
        char logBuffer[LOG_MESSAGE_BUFFER_SIZE];
        snprintf_P(logBuffer, sizeof(logBuffer), FMT_NETWORK_COMMAND_LOG, msg);
        opLogHistory.addEntry(logBuffer);
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
        snprintf_P(logBuffer, sizeof(logBuffer), FMT_NETWORK_COMMAND_LOG, buffer);
        opLogHistory.addEntry(logBuffer);
    }
}

// Update serial-only variants as well
void MultiPrint::serialInfo(const char *msg)
{
    Serial.print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_INFO_LOG, msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialInfo(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_WHITE "[INFO]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), PSTR("[INFO] %S"), msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialError(const char *msg)
{
    Serial.print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET ", ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_ERROR_LOG, msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialError(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_RED "[ERROR]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer  
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), PSTR("[ERROR] %S"), msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialDiagnostic(const char *msg)
{
    Serial.print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_DIAGNOSTIC_LOG, msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialDiagnostic(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_YELLOW "[DIAGNOSTIC]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), PSTR("[DIAGNOSTIC] %S"), msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialWarning(const char *msg)
{
    Serial.print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_WARNING_LOG, msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialWarning(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_ORANGE "[WARNING]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), PSTR("[WARNING] %S"), msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialSafety(const char *msg)
{
    Serial.print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), FMT_SAFETY_LOG, msg);
    opLogHistory.addEntry(buffer);
}

void MultiPrint::serialSafety(const __FlashStringHelper *msg)
{
    Serial.print(ANSI_BOLD_MAGENTA "[SAFETY]" ANSI_COLOR_RESET " ");
    Serial.println(msg);

    // Add to history buffer
    char buffer[LOG_MESSAGE_BUFFER_SIZE];
    snprintf_P(buffer, sizeof(buffer), PSTR("[SAFETY] %S"), msg);
    opLogHistory.addEntry(buffer);
}