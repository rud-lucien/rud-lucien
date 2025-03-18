#ifndef UTILS_H
#define UTILS_H

#include <Controllino.h>
#include "Hardware.h"         // For global hardware objects and functions (e.g., openValve, closeValve)
#include "Sensors.h"          // For functions like readPressure()
#include "Commands.h"
#include "Commander-API.hpp"  // For CommandCaller and commander
#include "Commander-IO.hpp"


/**
 * trimLeadingSpaces()
 * ---------------------
 * Removes any leading whitespace from the input string.
 *
 * @param str The string to trim.
 * @return A pointer to the first non-whitespace character.
 */
char* trimLeadingSpaces(char* str);

/**
 * processMultipleCommands()
 * ---------------------------
 * Splits a command line (using commas as delimiters), trims each token, and dispatches them
 * via the global commander.
 *
 * @param commandLine The command line (will be modified).
 * @param stream The output stream (e.g., Serial) to print logs.
 */
void processMultipleCommands(char* commandLine, Stream* stream);

/**
 * handleSerialCommands()
 * ------------------------
 * Reads from Serial, builds complete commands, and dispatches them.
 */
void handleSerialCommands();

/**
 * checkAndSetPressure()
 * -----------------------
 * Checks if the system pressure meets the desired threshold.
 * If not, sets the proportional valve to the specified position and waits
 * until the pressure stabilizes or a timeout occurs.
 *
 * @param thresholdPressure The pressure threshold (in psi).
 * @param valvePosition The desired valve position (percentage).
 * @param timeout The maximum wait time (in ms).
 * @return true if the threshold is reached; false otherwise.
 */
bool checkAndSetPressure(float thresholdPressure, float valvePosition, unsigned long timeout);

/**
 * resetI2CBus()
 * -------------
 * Resets the I²C bus by ending the Wire connection, delaying briefly, and restarting Wire.
 */
void resetI2CBus();

/**
 * openDispenseValves()
 * ---------------------
 * Opens the reagent and media valves for the specified trough.
 *
 * @param troughNumber The trough number (1 to NUM_OVERFLOW_SENSORS).
 */
void openDispenseValves(int troughNumber);

/**
 * closeDispenseValves()
 * ----------------------
 * Closes the reagent and media valves for the specified trough.
 *
 * @param troughNumber The trough number (1 to NUM_OVERFLOW_SENSORS).
 */
void closeDispenseValves(int troughNumber);

/**
 * stopDispenseOperation()
 * ------------------------
 * Stops the dispense operation for a given trough by closing its valves,
 * stopping the associated flow sensor measurement, and resetting volumes.
 *
 * @param troughNumber The trough number.
 * @param caller Pointer to a CommandCaller for output messaging.
 */
void stopDispenseOperation(int troughNumber, CommandCaller* caller);

#endif // UTILS_H
