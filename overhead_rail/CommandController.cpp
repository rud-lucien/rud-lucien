#include "CommandController.h"
#include "Utils.h"
#include <Ethernet.h>

//=============================================================================
// PROGMEM STRING CONSTANTS
//=============================================================================
// Format strings for sprintf_P()
const char FMT_SERIAL_COMMAND[] PROGMEM = "[SERIAL COMMAND] %s";
const char FMT_NETWORK_COMMAND[] PROGMEM = "[NETWORK COMMAND] %s";
const char FMT_COMMAND_WITH_SOURCE[] PROGMEM = "%s (from %d.%d.%d.%d)";
const char FMT_COMMAND_ERROR[] PROGMEM = "[ERROR] Cannot execute '%s' - %s";
const char FMT_COMMAND_OVERRIDE[] PROGMEM = "[WARNING] Command '%s' overridden - %s";
const char FMT_COMMAND_BUSY[] PROGMEM = "[BUSY] Cannot execute '%s' - %s. Use 'abort' to cancel.";

//=============================================================================
// EXTERNAL DECLARATIONS
//=============================================================================
extern bool ethernetInitialized;
extern EthernetClient clients[];

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Command lookup table - MUST BE ALPHABETICALLY SORTED for binary search
// Note: This is a basic set for the overhead rail system - expand as needed
const CommandInfo COMMAND_TABLE[] = {
    {"abort", CMD_EMERGENCY, 0},
    {"encoder", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY | CMD_FLAG_ASYNC},
    {"estop", CMD_EMERGENCY, 0},
    {"help", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"home", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"motor", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"move", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"network", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"position", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"sensor", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"status", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"stop", CMD_EMERGENCY, 0},
    {"system", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"teach", CMD_MODIFYING, 0},
    {"valve", CMD_MODIFYING, CMD_FLAG_ASYNC}
};

// Number of commands in the table
const size_t COMMAND_TABLE_SIZE = sizeof(COMMAND_TABLE) / sizeof(CommandInfo);

// Operation state tracking
bool operationInProgress = false;

// Client tracking for async operations
Stream *persistentClient = nullptr;

// Command processing buffers
char serialCommandBuffer[MAX_COMMAND_LENGTH];
char ethernetCommandBuffer[MAX_COMMAND_LENGTH];

//=============================================================================
// COMMAND LOOKUP
//=============================================================================

// Binary search function for command lookup
const CommandInfo *findCommand(const char *cmdName)
{
    int left = 0;
    int right = COMMAND_TABLE_SIZE - 1;

    while (left <= right)
    {
        int mid = left + (right - left) / 2;
        int cmp = strcmp(cmdName, COMMAND_TABLE[mid].name);

        if (cmp == 0)
            return &COMMAND_TABLE[mid]; // Found

        if (cmp < 0)
            right = mid - 1;
        else
            left = mid + 1;
    }

    return nullptr; // Not found
}

//=============================================================================
// CORE COMMAND PROCESSING FUNCTIONS
//=============================================================================

void handleSerialCommands()
{
    static uint8_t commandIndex = 0;

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            serialCommandBuffer[commandIndex] = '\0'; // Null-terminate the command

            if (commandIndex > 0) // Only process non-empty commands
            {
                char logMsg[MEDIUM_MSG_SIZE];
                sprintf_P(logMsg, FMT_SERIAL_COMMAND, serialCommandBuffer);
                Console.serialInfo(logMsg);

                // Tag for operation log
                char taggedCommand[SMALL_MSG_SIZE];
                sprintf_P(taggedCommand, FMT_SERIAL_COMMAND, serialCommandBuffer);

                // Pass tag to processCommand
                processCommand(serialCommandBuffer, &Serial, taggedCommand);
            }

            commandIndex = 0; // Reset buffer index
        }
        else if (c != '\r')
        { // Ignore carriage returns
            if (commandIndex < (sizeof(serialCommandBuffer) - 1))
            {
                serialCommandBuffer[commandIndex++] = c;
            }
            else
            {
                // Command too long - prevent buffer overflow
                commandIndex = sizeof(serialCommandBuffer) - 1;
                Console.serialError(F("Command truncated - exceeded maximum length"));
            }
        }
    }
}

void handleEthernetCommands()
{
    if (!ethernetInitialized)
    {
        return;
    }

    // Check each client for incoming commands
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && clients[i].connected() && clients[i].available())
        {
            updateClientActivity(i); // Update activity timestamp

            int j = 0;
            while (clients[i].available() && j < (MAX_COMMAND_LENGTH - 1))
            {
                char c = clients[i].read();
                if (c == '\n' || c == '\r')
                {
                    ethernetCommandBuffer[j] = '\0';

                    if (j > 0)
                    {
                        char commandWithSource[MEDIUM_MSG_SIZE];
                        sprintf_P(commandWithSource, FMT_COMMAND_WITH_SOURCE,
                                 ethernetCommandBuffer,
                                 clients[i].remoteIP()[0], clients[i].remoteIP()[1],
                                 clients[i].remoteIP()[2], clients[i].remoteIP()[3]);

                        char logMsg[ALERT_MSG_SIZE];
                        sprintf_P(logMsg, FMT_NETWORK_COMMAND, commandWithSource);
                        Console.serialInfo(logMsg);

                        // Tag for operation log
                        char taggedCommand[MEDIUM_MSG_SIZE];
                        sprintf_P(taggedCommand, FMT_NETWORK_COMMAND, commandWithSource);

                        processCommand(ethernetCommandBuffer, &clients[i], taggedCommand);
                    }

                    j = 0;
                    continue;
                }

                ethernetCommandBuffer[j++] = c;
            }

            if (j == (MAX_COMMAND_LENGTH - 1))
            {
                ethernetCommandBuffer[j] = '\0';
                clients[i].println(F("[ERROR] Command too long - truncated"));
                j = 0;
            }
        }
    }
}

bool processCommand(const char *rawCommand, Stream *output, const char *sourceTag)
{
    Console.setCurrentClient(output);

    // Extract first word (command name)
    char firstWord[16] = {0};
    int i = 0;
    while (rawCommand[i] && rawCommand[i] != ',' && rawCommand[i] != ' ' && i < 15)
    {
        firstWord[i] = rawCommand[i];
        i++;
    }
    firstWord[i] = '\0';

    const CommandInfo *cmdInfo = findCommand(firstWord);

    // Handle abort command immediately
    if (cmdInfo && strcmp(cmdInfo->name, "abort") == 0)
    {
        Console.acknowledge(F("Abort command received"));
        operationInProgress = false; // Clear any operation in progress
        clearPersistentClient();
        return true;
    }

    bool isAsyncCommand = (cmdInfo && (cmdInfo->flags & CMD_FLAG_ASYNC));

    // Create a copy of the original command
    char originalCommand[MAX_COMMAND_LENGTH];
    strncpy(originalCommand, rawCommand, MAX_COMMAND_LENGTH - 1);
    originalCommand[MAX_COMMAND_LENGTH - 1] = '\0';

    // Check if command can be executed
    if (canExecuteCommand(originalCommand))
    {
        // For async commands, store the client for later use
        if (isAsyncCommand)
        {
            persistentClient = output;
        }

        // Execute the command (placeholder for now - will be implemented when Commands.h is ready)
        bool success = executeCommand(originalCommand, output);

        if (!isAsyncCommand)
        {
            Console.setCurrentClient(nullptr);
        }

        return success;
    }

    Console.setCurrentClient(nullptr);
    return false;
}

//=============================================================================
// COMMAND EXECUTION (PLACEHOLDER)
//=============================================================================

// This is a placeholder function that will be replaced when Commands.h is implemented
bool executeCommand(const char *command, Stream *output)
{
    char firstWord[16] = {0};
    int i = 0;
    while (command[i] && command[i] != ',' && command[i] != ' ' && i < 15)
    {
        firstWord[i] = command[i];
        i++;
    }
    firstWord[i] = '\0';

    // Basic command implementations for testing
    if (strcmp(firstWord, "help") == 0)
    {
        output->println(F("Available commands:"));
        output->println(F("  help - Show this help"));
        output->println(F("  status - Show system status"));
        output->println(F("  network - Show network status"));
        output->println(F("  system - Show system information"));
        output->println(F("  motor - Motor control (home, move, stop)"));
        output->println(F("  encoder - Encoder/MPG control"));
        output->println(F("  valve - Valve control"));
        output->println(F("  sensor - Sensor status"));
        output->println(F("  position - Position information"));
        output->println(F("  teach - Teach position commands"));
        output->println(F("  stop - Emergency stop"));
        output->println(F("  abort - Abort current operation"));
        return true;
    }
    else if (strcmp(firstWord, "status") == 0)
    {
        output->println(F("System Status: OK"));
        output->print(F("Operation in progress: "));
        output->println(operationInProgress ? "YES" : "NO");
        return true;
    }
    else if (strcmp(firstWord, "network") == 0)
    {
        printEthernetStatus();
        return true;
    }
    else if (strcmp(firstWord, "system") == 0)
    {
        output->println(F("Overhead Rail Control System"));
        output->println(F("Firmware Version: 1.0"));
        return true;
    }
    else
    {
        output->println(F("[ERROR] Command not implemented yet"));
        return false;
    }
}

//=============================================================================
// COMMAND VALIDATION
//=============================================================================

CommandType getCommandType(const char *originalCommand)
{
    // Extract first word (main command)
    char firstWord[16] = {0};
    int i = 0;
    while (originalCommand[i] && originalCommand[i] != ',' && originalCommand[i] != ' ' && i < 15)
    {
        firstWord[i] = originalCommand[i];
        i++;
    }
    firstWord[i] = '\0';

    // Look up the command in our dispatch table
    const CommandInfo *cmdInfo = findCommand(firstWord);
    if (cmdInfo)
    {
        // For most commands, just return the type from the table
        if (strcmp(firstWord, "motor") != 0 && strcmp(firstWord, "system") != 0)
        {
            return cmdInfo->type;
        }

        // Special handling for motor commands
        if (strcmp(firstWord, "motor") == 0)
        {
            // Check for motor stop/abort which are emergency commands
            if (strstr(originalCommand, "stop") != nullptr ||
                strstr(originalCommand, "abort") != nullptr)
            {
                return CMD_EMERGENCY;
            }

            // Check for motor status which is read-only
            if (strstr(originalCommand, "status") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // All other motor commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for system commands
        if (strcmp(firstWord, "system") == 0)
        {
            // System status commands are read-only
            if (strstr(originalCommand, "status") != nullptr ||
                strstr(originalCommand, "info") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // System reset/config commands are modifying
            return CMD_MODIFYING;
        }
    }

    // Unknown commands are treated as modifying (conservative approach)
    return CMD_MODIFYING;
}

bool canExecuteCommand(const char *command)
{
    CommandType cmdType = getCommandType(command);

    // Emergency commands are always allowed
    if (cmdType == CMD_EMERGENCY)
    {
        return true;
    }

    // Read-only commands are always allowed
    if (cmdType == CMD_READ_ONLY)
    {
        return true;
    }

    // For other modifying commands, check if an operation is in progress
    if (operationInProgress)
    {
        sendCommandRejection(command, "Operation in progress");
        return false;
    }

    // Command is allowed
    return true;
}

bool isCommandExcludedFromHistory(const char *command)
{
    // Extract first word
    char firstWord[16] = {0};
    int i = 0;
    while (command[i] && command[i] != ',' && command[i] != ' ' && i < 15)
    {
        firstWord[i] = command[i];
        i++;
    }
    firstWord[i] = '\0';

    // Look up the command in our dispatch table
    const CommandInfo *cmdInfo = findCommand(firstWord);
    if (cmdInfo)
    {
        // Use the flag to determine if command should be excluded
        return (cmdInfo->flags & CMD_FLAG_NO_HISTORY);
    }

    // By default, include command in history
    return false;
}

//=============================================================================
// CLIENT MANAGEMENT
//=============================================================================

Stream *getPersistentClient()
{
    return persistentClient;
}

void clearPersistentClient()
{
    persistentClient = nullptr;
}

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

void sendCommandRejection(const char *command, const char *reason)
{
    char msg[MEDIUM_MSG_SIZE];

    if (operationInProgress)
    {
        // This is truly a BUSY condition
        sprintf_P(msg, FMT_COMMAND_BUSY, command, reason);
    }
    else
    {
        // This is an ERROR condition, not BUSY
        sprintf_P(msg, FMT_COMMAND_ERROR, command, reason);
    }

    Console.println(msg);
}

const char *getOperationTypeName(int type)
{
    switch (type)
    {
    case 1:
        return "Homing";
    case 2:
        return "Movement";
    case 3:
        return "Teaching";
    case 4:
        return "Valve operation";
    default:
        return "Automated";
    }
}

char *trimLeadingSpaces(char *str)
{
    while (*str && isspace(*str))
    {
        str++;
    }
    return str;
}
