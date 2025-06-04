#include "OutputManager.h"

// Global instance
MultiPrint Console;

// Initialize the output manager
void initOutputManager()
{
    // Start with Serial as the default output and input source
    Console.addOutput(&Serial);
    Console.setPrimaryInput(&Serial);
}

// Add an output destination
bool MultiPrint::addOutput(Print *output)
{
    if (output == nullptr)
    {
        return false;
    }

    // Check if this output is already registered
    for (int i = 0; i < outputCount; i++)
    {
        if (outputs[i] == output)
        {
            return true; // Already registered
        }
    }

    // Add new output if space available
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

// Write a single byte to all outputs
size_t MultiPrint::write(uint8_t c)
{
    size_t total = 0;
    for (int i = 0; i < outputCount; i++)
    {
        if (outputs[i])
        {
            total += outputs[i]->write(c);
        }
    }
    return total;
}

// Write a buffer to all outputs
size_t MultiPrint::write(const uint8_t *buffer, size_t size)
{
    size_t total = 0;
    for (int i = 0; i < outputCount; i++)
    {
        if (outputs[i])
        {
            total += outputs[i]->write(buffer, size);
        }
    }
    return total;
}

// Flush implementation - forward to primary input
void MultiPrint::flush()
{
    if (primaryInput)
    {
        primaryInput->flush();
    }
}

// Stream implementation - forward to primary input
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

// Helper methods for formatted messages
void MultiPrint::info(const char *msg)
{
    print(F("[INFO] "));
    println(msg);
}

void MultiPrint::info(const __FlashStringHelper *msg)
{
    print(F("[INFO] "));
    println(msg);
}

void MultiPrint::error(const char *msg)
{
    print(F("[ERROR] "));
    println(msg);
}

void MultiPrint::error(const __FlashStringHelper *msg)
{
    print(F("[ERROR] "));
    println(msg);
}

void MultiPrint::diagnostic(const char *msg)
{
    print(F("[DIAGNOSTIC] "));
    println(msg);
}

void MultiPrint::diagnostic(const __FlashStringHelper *msg)
{
    print(F("[DIAGNOSTIC] "));
    println(msg);
}

void MultiPrint::safety(const char *msg)
{
    print(F("[SAFETY] "));
    println(msg);
}

void MultiPrint::safety(const __FlashStringHelper *msg)
{
    print(F("[SAFETY] "));
    println(msg);
}

void MultiPrint::warning(const char *msg)
{
    print(F("[WARNING] "));
    println(msg);
}

void MultiPrint::warning(const __FlashStringHelper *msg)
{
    print(F("[WARNING] "));
    println(msg);
}

void MultiPrint::serialCommand(const char *msg)
{
    print(F("[SERIAL COMMAND] "));
    println(msg);
}

void MultiPrint::serialCommand(const __FlashStringHelper *msg)
{
    print(F("[SERIAL COMMAND] "));
    println(msg);
}

void MultiPrint::ethernetCommand(const char *msg)
{
    print(F("[ETHERNET COMMAND] "));
    println(msg);
}

void MultiPrint::ethernetCommand(const __FlashStringHelper *msg)
{
    print(F("[ETHERNET COMMAND] "));
    println(msg);
}