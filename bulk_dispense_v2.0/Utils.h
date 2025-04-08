#ifndef UTILS_H
#define UTILS_H

#include <Controllino.h>
#include "Hardware.h" // For global hardware objects and functions
#include "Sensors.h"  // For sensor-related functions
#include "Commands.h"
#include "Commander-API.hpp" // For CommandCaller and commander
#include "Commander-IO.hpp"
#include "NetworkConfig.h"

// Enumeration to distinguish valve types.
enum ValveType
{
  REAGENT,
  MEDIA,
  WASTE
};

enum CommandSource
{
  SOURCE_SERIAL,
  SOURCE_NETWORK
};

// Command Processing
void executeCommandWithActionTags(const char *command, Stream *stream);
char *trimLeadingSpaces(char *str);
bool isCommandPrefix(const char *token);
void processMultipleCommands(char *commandLine, Stream *stream, CommandSource source);
void handleSerialCommands();
String processClientData();
void handleNetworkCommands();
void sendMessage(const char *message, Stream *response, EthernetClient client, bool addNewline = true);
void sendMessage(const __FlashStringHelper *message, Stream *response, EthernetClient client, bool addNewline = true);

// Pressure & I2C Utilities
bool isPressureOK(float thresholdPressure);
void setPressureValve(int valvePosition);
bool checkAndSetPressure(float thresholdPressure, int valvePosition, unsigned long timeout);
void resetI2CBus();

// Valve Control Utilities
void openDispenseValves(int troughNumber);
void closeDispenseValves(int troughNumber);
void stopDispenseOperation(int troughNumber, Stream *stream);
bool areDispenseValvesOpen(int troughNumber);

// Manual & Fill Mode Control
void enableManualControl(int index, Stream *stream);
void disableManualControl(int index, Stream *stream);
void enableFillMode(int troughNumber, Stream *stream);
void disableFillMode(int troughNumber, Stream *stream);
void disableFillModeForAll(Stream *stream);
bool isFillModeActive(int troughNumber);

// Helper Functions
void stopDispensingIfActive(int troughNumber, Stream *stream);
bool isWasteBottleFullForTrough(int troughNumber, Stream *stream);
bool hasIncompatibleDrainage(int troughNumber, Stream *stream);
bool validateTroughNumber(int troughNumber, Stream *stream);
void stopDispensingForFill(int troughNumber, Stream *stream);
void stopPrimingForFill(int troughNumber, Stream *stream);
bool isValveAlreadyPrimed(int valveNumber, Stream *stream);
bool validateValveNumber(int valveNumber, Stream *stream);
void setVacuumMonitoringAndCloseMainValve(int troughNumber, Stream *stream);
void stopDrainOperation(int trough, Stream *stream);
void abortAllAutomatedOperations(Stream *stream);

// State Management
String getOverallTroughState();
String getOpenValvesString(bool v1, bool v2, bool v3, bool v4);
void resetAsyncFlagsForTrough(int troughNumber);
void resetAsyncFlagsForCommand(const char *token);
bool isValveClosed(const OnOffValve &valve);
bool areAllValvesClosedForTrough(int troughNumber);
void set_valve_state(OnOffValve &valveVar, bool state, int valveNumber, ValveType type, CommandCaller *caller);
void updateTroughManualControlFlag(ValveType type, int valveNumber, CommandCaller *caller);

#endif // UTILS_H
