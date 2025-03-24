#ifndef UTILS_H
#define UTILS_H

#include <Controllino.h>
#include "Hardware.h"         // For global hardware objects and functions
#include "Sensors.h"          // For sensor-related functions
#include "Commands.h"
#include "Commander-API.hpp"  // For CommandCaller and commander
#include "Commander-IO.hpp"

// ------------------------------------------------------------
// Command Session Utilities
// ------------------------------------------------------------
void executeCommandWithActionTags(const char* command, Stream* stream);

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

// ------------------------------------------------------------
// Pressure & I2C Utilities
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Valve Control Utilities
// ------------------------------------------------------------

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
 * @param stream Pointer to a Stream (e.g., Serial) for logging.
 */
void stopDispenseOperation(int troughNumber, Stream* stream);

/**
 * isCommandPrefix()
 * -----------------
 * Checks whether a token starts with a valid command prefix.
 * @param token The token to check.
 * @return true if token starts with a registered command; false otherwise.
 */
bool isCommandPrefix(const char* token);

/**
 * areDispenseValvesOpen()
 * ------------------------
 * Checks if the dispensing valves for the specified trough are open.
 * @param troughNumber Trough number.
 * @return true if both reagent and media valves are open.
 */
bool areDispenseValvesOpen(int troughNumber);

// ------------------------------------------------------------
// Manual & Fill Mode Control
// ------------------------------------------------------------

/**
 * enableManualControl()
 * -----------------------
 * Enables manual control for the given trough (index) and prints a message.
 * @param index The trough index (0-based).
 * @param caller Pointer to the CommandCaller for logging.
 */
void enableManualControl(int index, CommandCaller* caller);

/**
 * disableManualControl()
 * ------------------------
 * Disables manual control for the given trough (index) and prints a message.
 * @param index The trough index (0-based).
 * @param caller Pointer to the CommandCaller for logging.
 */
void disableManualControl(int index, CommandCaller* caller);

/**
 * enableFillMode()
 * ----------------
 * Enables fill mode for a specific trough.
 * @param troughNumber Trough number (1-based).
 * @param caller Pointer to the CommandCaller for logging.
 */
void enableFillMode(int troughNumber, CommandCaller* caller);

/**
 * disableFillMode()
 * -----------------
 * Disables fill mode for a specific trough.
 * @param troughNumber Trough number (1-based).
 * @param stream Pointer to the Stream for logging.
 */
void disableFillMode(int troughNumber, Stream* stream);

/**
 * disableFillModeForAll()
 * -----------------------
 * Disables fill mode for all troughs.
 * @param caller Pointer to the CommandCaller for logging.
 */
void disableFillModeForAll(CommandCaller* caller);

/**
 * isFillModeActive()
 * ------------------
 * Checks if fill mode is active for a specific trough.
 * @param troughNumber Trough number.
 * @return true if fill mode is active, false otherwise.
 */
bool isFillModeActive(int troughNumber);

// ------------------------------------------------------------
// Fan & Pressure Utilities
// ------------------------------------------------------------

/**
 * isPressureOK()
 * --------------
 * Checks if the current system pressure meets the threshold.
 * @param thresholdPressure Pressure threshold in psi.
 * @return true if current pressure is at or above threshold.
 */
bool isPressureOK(float thresholdPressure);

/**
 * setPressureValve()
 * ------------------
 * Sets the proportional valve to the specified percentage.
 * @param valvePosition The valve position (percentage).
 */
void setPressureValve(int valvePosition);

// ------------------------------------------------------------
// Helper Functions for Dispensing, Draining, and Priming
// ------------------------------------------------------------

/**
 * stopDispensingIfActive()
 * --------------------------
 * Checks if dispensing is active on a trough and stops it if so.
 * @param troughNumber Trough number.
 * @param stream Pointer to the Stream for logging.
 */
void stopDispensingIfActive(int troughNumber, Stream* stream);

/**
 * isWasteBottleFullForTrough()
 * -----------------------------
 * Checks if the waste bottle for a given trough is full.
 * @param troughNumber Trough number.
 * @param caller Pointer to the CommandCaller for logging.
 * @return true if the waste bottle is full, false otherwise.
 */
bool isWasteBottleFullForTrough(int troughNumber, CommandCaller* caller);

/**
 * hasIncompatibleDrainage()
 * --------------------------
 * Checks for incompatible drainage conditions for a given trough.
 * @param troughNumber Trough number.
 * @param caller Pointer to the CommandCaller for logging.
 * @return true if incompatible drainage is detected, false otherwise.
 */
bool hasIncompatibleDrainage(int troughNumber, CommandCaller* caller);

/**
 * validateTroughNumber()
 * -----------------------
 * Validates that the trough number is within range.
 * @param troughNumber Trough number.
 * @param caller Pointer to the CommandCaller for logging.
 * @return true if valid, false otherwise.
 */
bool validateTroughNumber(int troughNumber, CommandCaller* caller);

/**
 * stopDispensingForFill()
 * -------------------------
 * Stops dispensing on a trough when a fill command is issued.
 * @param troughNumber Trough number.
 * @param stream Pointer to the Stream for logging.
 */
void stopDispensingForFill(int troughNumber, Stream* stream);

/**
 * stopPrimingForFill()
 * ----------------------
 * Stops priming on a trough when a fill command is issued.
 * @param troughNumber Trough number.
 * @param stream Pointer to the Stream for logging.
 */
void stopPrimingForFill(int troughNumber, Stream* stream);

/**
 * isValveAlreadyPrimed()
 * ------------------------
 * Checks if a valve is already primed.
 * @param valveNumber Valve number (1-based).
 * @param caller Pointer to the CommandCaller for logging.
 * @return true if already primed, false otherwise.
 */
bool isValveAlreadyPrimed(int valveNumber, CommandCaller* caller);

/**
 * validateValveNumber()
 * -----------------------
 * Validates that the valve number is between 1 and 4.
 * @param valveNumber Valve number.
 * @param caller Pointer to the CommandCaller for logging.
 * @return true if valid, false otherwise.
 */
bool validateValveNumber(int valveNumber, CommandCaller* caller);

/**
 * setVacuumMonitoringAndCloseMainValve()
 * ----------------------------------------
 * Sets vacuum monitoring for the given trough and closes the main waste valve.
 * @param troughNumber Trough number.
 * @param caller Pointer to the CommandCaller for logging.
 */
void setVacuumMonitoringAndCloseMainValve(int troughNumber, CommandCaller* caller);

/**
 * abortAllAutomatedOperations()
 * ------------------------------
 * Aborts all automated operations (dispensing, priming, filling, draining)
 * across all troughs in response to a critical system condition such as
 * enclosure liquid detection.
 * 
 * This function:
 *   - Stops dispensing operations if active.
 *   - Stops priming operations if active.
 *   - Disables fill mode if active.
 *   - Cancels draining by clearing isDraining flags.
 *   - Ends any active asynchronous command session.
 * 
 * @param stream Pointer to the Stream for logging.
 */
void abortAllAutomatedOperations(Stream* stream);

#endif // UTILS_H



