#ifndef UTILS_H
#define UTILS_H

#include <Controllino.h>
#include "Hardware.h"         // For global hardware objects and functions
#include "Sensors.h"          // For sensor-related functions
#include "Commands.h"
#include "Commander-API.hpp"  // For CommandCaller and commander
#include "Commander-IO.hpp"

/**
 * trimLeadingSpaces()
 * ---------------------
 * Removes leading whitespace from the input string.
 * @param str The string to trim.
 * @return Pointer to the first non-space character.
 */
char* trimLeadingSpaces(char* str);

/**
 * processMultipleCommands()
 * ---------------------------
 * Splits the commandLine string using commas as delimiters. Each token is trimmed
 * and then passed to the global commander for execution.
 * @param commandLine The command line string (will be modified).
 * @param stream The output stream (e.g., Serial) for logging.
 */
void processMultipleCommands(char* commandLine, Stream* stream);

/**
 * handleSerialCommands()
 * ------------------------
 * Reads from Serial and dispatches complete command lines.
 */
void handleSerialCommands();

/**
 * checkAndSetPressure()
 * -----------------------
 * Checks if system pressure meets the threshold. If not, it sets the valve and waits
 * until the threshold is reached or a timeout occurs.
 * @param thresholdPressure Pressure threshold (psi).
 * @param valvePosition Desired valve position.
 * @param timeout Maximum wait time (ms).
 * @return true if threshold reached; false otherwise.
 */
bool checkAndSetPressure(float thresholdPressure, int valvePosition, unsigned long timeout);


/**
 * resetI2CBus()
 * -------------
 * Resets the I²C bus.
 */
void resetI2CBus();

/**
 * openDispenseValves()
 * ---------------------
 * Opens the reagent and media valves for a given trough.
 * @param troughNumber Trough number (1 to NUM_OVERFLOW_SENSORS).
 */
void openDispenseValves(int troughNumber);

/**
 * closeDispenseValves()
 * ----------------------
 * Closes the reagent and media valves for a given trough.
 * @param troughNumber Trough number (1 to NUM_OVERFLOW_SENSORS).
 */
void closeDispenseValves(int troughNumber);

/**
 * stopDispenseOperation()
 * ------------------------
 * Stops dispensing for a trough by closing valves, stopping flow sensor measurement, and resetting volumes.
 * @param troughNumber Trough number.
 * @param caller Pointer to a CommandCaller for messages.
 */
void stopDispenseOperation(int troughNumber, CommandCaller* caller);

/**
 * isCommandPrefix()
 * -----------------
 * Checks whether a token starts with a valid command prefix. (This example uses a simple comparison.)
 * @param token The token to check.
 * @return true if token starts with a registered command; false otherwise.
 */
bool isCommandPrefix(const char* token);

bool areDispenseValvesOpen(int troughNumber);

/**
 * enableManualControl()
 * ---------------------
 * Enables manual control for the given trough (index) and prints a message.
 *
 * @param index  The trough index (0-based).
 * @param caller Pointer to the CommandCaller for logging.
 */
void enableManualControl(int index, CommandCaller* caller);

/**
 * disableManualControl()
 * ----------------------
 * Disables manual control for the given trough (index) and prints a message.
 *
 * @param index  The trough index (0-based).
 * @param caller Pointer to the CommandCaller for logging.
 */
void disableManualControl(int index, CommandCaller* caller);

// Function to enable fill mode for a specific trough
void enableFillMode(int troughNumber, CommandCaller* caller);

// Function to disable fill mode for a specific trough
void disableFillMode(int troughNumber, CommandCaller* caller);

// Function to disable fill mode for all troughs
void disableFillModeForAll(CommandCaller* caller);

// Function to check if a specific trough is in fill mode
bool isFillModeActive(int troughNumber);

bool isPressureOK(float thresholdPressure);

void setPressureValve(int valvePosition);


#endif // UTILS_H

