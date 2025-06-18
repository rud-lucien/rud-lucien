#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include <Arduino.h>

// Forward declaration of persistentClient
extern Stream *persistentClient;

// Class to handle output to multiple destinations (Serial, Ethernet, etc.)
class MultiPrint : public Stream
{
private:
    static const int MAX_OUTPUTS = 4; // Support up to 4 outputs
    Print *outputs[MAX_OUTPUTS];
    int outputCount;
    Stream *primaryInput;  // Designated input source (usually Serial)
    Stream *currentClient; // Current client connection for command output

public:
    MultiPrint() : outputCount(0), primaryInput(nullptr), currentClient(nullptr) {}

    // Set the primary input source for reading operations
    void setPrimaryInput(Stream *input)
    {
        primaryInput = input;
    }

    // Set the current client for temporary output redirection
    void setCurrentClient(Stream *client)
    {
        currentClient = client;

        // When setting a new client, also update persistent client
        if (client != nullptr)
        {
            persistentClient = client;
        }
    }

    // Get the current client
    Stream *getCurrentClient()
    {
        return currentClient;
    }

    // Add/remove output destinations
    bool addOutput(Print *output);
    bool removeOutput(Print *output);

    // Required overrides from Print class
    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;

    // Required overrides from Stream class
    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;

    // Helper methods for formatted messages - by message type
    void info(const char *msg);
    void info(const __FlashStringHelper *msg);

    void error(const char *msg);
    void error(const __FlashStringHelper *msg);

    void diagnostic(const char *msg);
    void diagnostic(const __FlashStringHelper *msg);

    void safety(const char *msg);
    void safety(const __FlashStringHelper *msg);

    void warning(const char *msg);
    void warning(const __FlashStringHelper *msg);

    void serialCommand(const char *msg);
    void serialCommand(const __FlashStringHelper *msg);

    void ethernetCommand(const char *msg);
    void ethernetCommand(const __FlashStringHelper *msg);

    // Command acknowledgment method
    void acknowledge(const char *msg);
    void acknowledge(const __FlashStringHelper *msg);

    // Serial-only variants
    void serialInfo(const char *msg);
    void serialInfo(const __FlashStringHelper *msg);
    void serialError(const char *msg);
    void serialError(const __FlashStringHelper *msg);
    void serialDiagnostic(const char *msg);
    void serialDiagnostic(const __FlashStringHelper *msg);
    void serialWarning(const char *msg);
    void serialWarning(const __FlashStringHelper *msg);
    void serialSafety(const char *msg);
    void serialSafety(const __FlashStringHelper *msg);

    // Legacy method (now maps to info)
    void message(const char *msg) { info(msg); }
    void message(const __FlashStringHelper *msg) { info(msg); }

    // Set the client if none is currently set
    void setClientIfNone(Stream *client)
    {
        if (currentClient == nullptr)
        {
            currentClient = client;
        }
    }
};

// Global instance
extern MultiPrint Console;

// Initialize the output manager
void initOutputManager();

#endif // OUTPUT_MANAGER_H