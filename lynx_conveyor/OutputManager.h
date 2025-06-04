#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include <Arduino.h>

// Class to handle output to multiple destinations (Serial, Ethernet, etc.)
class MultiPrint : public Stream
{
private:
    static const int MAX_OUTPUTS = 4; // Support up to 4 outputs
    Print *outputs[MAX_OUTPUTS];
    int outputCount;
    Stream *primaryInput; // Designated input source (usually Serial)

public:
    MultiPrint() : outputCount(0), primaryInput(nullptr) {}

    // Set the primary input source for reading operations
    void setPrimaryInput(Stream *input)
    {
        primaryInput = input;
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

    // Legacy method (now maps to info)
    void message(const char *msg) { info(msg); }
    void message(const __FlashStringHelper *msg) { info(msg); }
};

// Global instance
extern MultiPrint Console;

// Helper macros for common message types
#define LOG_INFO(msg) Console.info(F(msg))
#define LOG_ERR(msg) Console.error(F(msg))
#define LOG_DIAG(msg) Console.diagnostic(F(msg))
#define LOG_SAFETY(msg) Console.safety(F(msg))
#define LOG_WARN(msg) Console.warning(F(msg))
#define LOG_MSG(msg) Console.log(F(msg)) // General logging
#define LOG_SERIAL_CMD(msg) Console.serialCommand(F(msg))
#define LOG_ETH_CMD(msg) Console.ethernetCommand(F(msg))

// Initialize the output manager
void initOutputManager();

#endif // OUTPUT_MANAGER_H