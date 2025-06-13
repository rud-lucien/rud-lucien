#include "OutputManager.h"
#include <Ethernet.h>  // Add this line to include Ethernet definitions

// Global instances
MultiPrint Console;


// Initialize the output manager
void initOutputManager() {
    // Add Serial as default output
    Console.addOutput(&Serial);
    Console.setPrimaryInput(&Serial);
}

// Add an output destination
bool MultiPrint::addOutput(Print *output) {
    if (outputCount < MAX_OUTPUTS) {
        outputs[outputCount++] = output;
        return true;
    }
    return false;
}

// Remove an output destination
bool MultiPrint::removeOutput(Print *output) {
    for (int i = 0; i < outputCount; i++) {
        if (outputs[i] == output) {
            // Shift remaining outputs
            for (int j = i; j < outputCount - 1; j++) {
                outputs[j] = outputs[j + 1];
            }
            outputCount--;
            return true;
        }
    }
    return false;
}

// Critical fix: Proper implementation of write for single byte
size_t MultiPrint::write(uint8_t c) {
    size_t written = 0;
    
    // Check for persistentClient
    if (currentClient == nullptr && persistentClient != nullptr) {
        currentClient = persistentClient;
    }
    
    // Write to all registered outputs
    for (int i = 0; i < outputCount; i++) {
        written += outputs[i]->write(c);
    }
    
    // IMPORTANT: Also write to current client if set AND not already in outputs
    if (currentClient != nullptr) {
        bool alreadyIncluded = false;
        for (int i = 0; i < outputCount; i++) {
            if (outputs[i] == currentClient) {
                alreadyIncluded = true;
                break;
            }
        }
        
        if (!alreadyIncluded) {
            written += currentClient->write(c);
        }
    }
    
    return written;
}

// Add similar implementation for buffer write
size_t MultiPrint::write(const uint8_t *buffer, size_t size) {
    size_t written = 0;
    
    // Check for persistentClient
    if (currentClient == nullptr && persistentClient != nullptr) {
        currentClient = persistentClient;
    }
    
    // Write to all registered outputs
    for (int i = 0; i < outputCount; i++) {
        written += outputs[i]->write(buffer, size);
    }
    
    // Also write to current client if it exists and isn't in outputs
    if (currentClient != nullptr) {
        bool alreadyIncluded = false;
        for (int i = 0; i < outputCount; i++) {
            if (outputs[i] == currentClient) {
                alreadyIncluded = true;
                break;
            }
        }
        
        if (!alreadyIncluded) {
            written += currentClient->write(buffer, size);
        }
    }
    
    return written;
}

// Add implementation for remaining required Stream methods
int MultiPrint::available() {
    return primaryInput ? primaryInput->available() : 0;
}

int MultiPrint::read() {
    return primaryInput ? primaryInput->read() : -1;
}

int MultiPrint::peek() {
    return primaryInput ? primaryInput->peek() : -1;
}

void MultiPrint::flush() {
    // Only try to flush the current client if set
    // (Print objects don't have flush method)
    if (currentClient) {
        currentClient->flush();
    }
    // Don't try to flush outputs - Print class doesn't have flush()
}

// Add to OutputManager.cpp
void MultiPrint::acknowledge(const char *msg) {
    print(F("[ACK], "));
    println(msg);
}

void MultiPrint::acknowledge(const __FlashStringHelper *msg) {
    print(F("[ACK], "));
    println(msg);
}

// Helper methods implementation for formatted messages
void MultiPrint::info(const char *msg) {
    print(F("[INFO] "));
    println(msg);
}

void MultiPrint::info(const __FlashStringHelper *msg) {
    print(F("[INFO] "));
    println(msg);
}

void MultiPrint::error(const char *msg) {
    print(F("[ERROR], "));
    println(msg);
}

void MultiPrint::error(const __FlashStringHelper *msg) {
    print(F("[ERROR], "));
    println(msg);
}

void MultiPrint::diagnostic(const char *msg) {
    print(F("[DIAGNOSTIC] "));
    println(msg);
}

void MultiPrint::diagnostic(const __FlashStringHelper *msg) {
    print(F("[DIAGNOSTIC] "));
    println(msg);
}

void MultiPrint::serialCommand(const char *msg) {
    print(F("[SERIAL COMMAND] "));
    println(msg);
}

void MultiPrint::serialCommand(const __FlashStringHelper *msg) {
    print(F("[SERIAL COMMAND] "));
    println(msg);
}

void MultiPrint::ethernetCommand(const char *msg) {
    print(F("[NETWORK] Command: "));
    println(msg);
}

void MultiPrint::ethernetCommand(const __FlashStringHelper *msg) {
    print(F("[NETWORK] Command: "));
    println(msg);
}

void MultiPrint::warning(const char *msg) {
    print(F("[WARNING] "));
    println(msg);
}

void MultiPrint::warning(const __FlashStringHelper *msg) {
    print(F("[WARNING] "));
    println(msg);
}

void MultiPrint::safety(const char *msg) {
    print(F("[SAFETY] "));
    println(msg);
}

void MultiPrint::safety(const __FlashStringHelper *msg) {
    print(F("[SAFETY] "));
    println(msg);
}

void MultiPrint::serialInfo(const char *msg) {
    Serial.print(F("[INFO] "));
    Serial.println(msg);
}

void MultiPrint::serialInfo(const __FlashStringHelper *msg) {
    Serial.print(F("[INFO] "));
    Serial.println(msg);
}

void MultiPrint::serialError(const char *msg) {
    Serial.print(F("[ERROR], "));
    Serial.println(msg);
}

void MultiPrint::serialError(const __FlashStringHelper *msg) {
    Serial.print(F("[ERROR], "));
    Serial.println(msg);
}

void MultiPrint::serialDiagnostic(const char *msg) {
    Serial.print(F("[DIAGNOSTIC] "));
    Serial.println(msg);
}

void MultiPrint::serialDiagnostic(const __FlashStringHelper *msg) {
    Serial.print(F("[DIAGNOSTIC] "));
    Serial.println(msg);
}

void MultiPrint::serialWarning(const char *msg) {
    Serial.print(F("[WARNING] "));
    Serial.println(msg);
}

void MultiPrint::serialWarning(const __FlashStringHelper *msg) {
    Serial.print(F("[WARNING] "));
    Serial.println(msg);
}

void MultiPrint::serialSafety(const char *msg) {
    Serial.print(F("[SAFETY] "));
    Serial.println(msg);
}

void MultiPrint::serialSafety(const __FlashStringHelper *msg) {
    Serial.print(F("[SAFETY] "));
    Serial.println(msg);
}



