#include "CommandController.h"
#include <Ethernet.h>      // For EthernetClient type
#include "OutputManager.h" // Make sure this is included

//-------------------------------------------------------------------------
// External declarations for Ethernet support
//-------------------------------------------------------------------------
extern bool ethernetInitialized;
extern EthernetClient clients[];
extern OperationStatus currentOperation; // This is referenced in sendCommandRejection

//-------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------
bool testInProgress = false;
volatile bool testAbortRequested = false;

// Command lookup table - MUST BE ALPHABETICALLY SORTED for binary search
const CommandInfo COMMAND_TABLE[] = {
    {"H", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"abort", CMD_EMERGENCY, 0},
    {"encoder", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY | CMD_FLAG_ASYNC},
    {"estop", CMD_EMERGENCY, 0},
    {"h", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"help", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"jog", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"lock", CMD_MODIFYING, 0},
    {"log", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"motor", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"move", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"network", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"stop", CMD_EMERGENCY, 0},
    {"system", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"teach", CMD_MODIFYING, 0},  // ADD THIS LINE
    {"test", CMD_TEST, CMD_FLAG_ASYNC},
    {"tray", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"unlock", CMD_MODIFYING, 0}};

// Number of commands in the table
const size_t COMMAND_TABLE_SIZE = sizeof(COMMAND_TABLE) / sizeof(CommandInfo);

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

// Buffer for Ethernet commands - define it here to avoid duplication
extern char ethernetCommandBuffer[];

// Add this global variable at the top of the file
Stream *persistentClient = nullptr;

//-------------------------------------------------------------------------
// Core Command Processing Functions
//-------------------------------------------------------------------------

// Initialization
void initTestFlags()
{
    // Initialize command controller state
    testInProgress = false;
    testAbortRequested = false;
}

// Command Handling Functions
void handleSerialCommands()
{
    static char commandBuffer[64]; // Buffer for incoming commands
    static uint8_t commandIndex = 0;

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            commandBuffer[commandIndex] = '\0'; // Null-terminate the command

            char logMsg[128];
            snprintf(logMsg, sizeof(logMsg), "[SERIAL COMMAND] %s", commandBuffer);
            Console.serialInfo(logMsg);

            // Tag for operation log
            char taggedCommand[96];
            snprintf(taggedCommand, sizeof(taggedCommand), "[SERIAL COMMAND] %s", commandBuffer);

            // Pass tag to processCommand
            processCommand(commandBuffer, &Serial, taggedCommand);

            commandIndex = 0; // Reset buffer index
        }
        else if (c != '\r')
        { // Ignore carriage returns
            if (commandIndex < (sizeof(commandBuffer) - 1))
            {
                commandBuffer[commandIndex++] = c;
            }
            else
            {
                // Command too long - prevent buffer overflow
                commandIndex = sizeof(commandBuffer) - 1;
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
            extern unsigned long clientLastActivityTime[];
            clientLastActivityTime[i] = millis();

            int j = 0;
            while (clients[i].available() && j < 63)
            {
                char c = clients[i].read();
                if (c == '\n' || c == '\r')
                {
                    ethernetCommandBuffer[j] = '\0';

                    if (j > 0)
                    {
                        char commandWithSource[128];
                        snprintf(commandWithSource, sizeof(commandWithSource),
                                 "from %d.%d.%d.%d: %s",
                                 clients[i].remoteIP()[0], clients[i].remoteIP()[1],
                                 clients[i].remoteIP()[2], clients[i].remoteIP()[3],
                                 ethernetCommandBuffer);

                        char logMsg[200];
                        snprintf(logMsg, sizeof(logMsg), "[NETWORK COMMAND] %s", commandWithSource);
                        Console.serialInfo(logMsg);

                        // Tag for operation log
                        char taggedCommand[160];
                        snprintf(taggedCommand, sizeof(taggedCommand), "[NETWORK COMMAND] %s", commandWithSource);

                        processCommand(ethernetCommandBuffer, &clients[i], taggedCommand);
                    }

                    j = 0;
                    continue;
                }

                ethernetCommandBuffer[j++] = c;
            }

            if (j == 63)
            {
                ethernetCommandBuffer[j] = '\0';
                clients[i].println(F("[ERROR], Command too long - truncated"));
                j = 0;
            }
        }
    }
}

// Updated processCommand to accept sourceTag
bool processCommand(const char *rawCommand, Stream *output, const char *sourceTag)
{
    Console.setCurrentClient(output);

    char firstWord[16] = {0};
    int i = 0;
    while (rawCommand[i] && rawCommand[i] != ',' && rawCommand[i] != ' ' && i < 15)
    {
        firstWord[i] = rawCommand[i];
        i++;
    }
    firstWord[i] = '\0';

    const CommandInfo *cmdInfo = findCommand(firstWord);

    if (cmdInfo && strcmp(cmdInfo->name, "abort") == 0)
    {
        requestTestAbort("command interface");
        Console.acknowledge(F("Test abort requested"));
        return true;
    }

    bool isAsyncCommand = (cmdInfo && (cmdInfo->flags & CMD_FLAG_ASYNC));

    char originalCommand[64];
    strncpy(originalCommand, rawCommand, 63);
    originalCommand[63] = '\0';

    char processedCommand[64];
    strncpy(processedCommand, rawCommand, 63);
    processedCommand[63] = '\0';

    for (int i = 0; processedCommand[i]; i++)
    {
        if (processedCommand[i] == ',')
        {
            processedCommand[i] = ' ';
        }
    }

    if (canExecuteCommand(originalCommand))
    {
        // Use tag if provided, otherwise log the raw command
        if (!isCommandExcludedFromHistory(originalCommand))
        {
            if (sourceTag && sourceTag[0])
            {
                opLogHistory.addEntry(sourceTag);
            }
            else
            {
                opLogHistory.addEntry(originalCommand);
            }
        }

        bool success = commander.execute(processedCommand, output);

        if (!success)
        {
            bool isKnownCommand = false;
            for (size_t i = 0; i < API_tree_size; i++)
            {
                const char *cmdName = API_tree[i].name;
                if (strncmp(processedCommand, cmdName, strlen(cmdName)) == 0)
                {
                    isKnownCommand = true;
                    break;
                }
            }
            if (!isKnownCommand)
            {
                Console.serialError(F("Command not found"));
            }
        }

        if (!isAsyncCommand)
        {
            Console.setCurrentClient(nullptr);
        }

        return success;
    }

    Console.setCurrentClient(nullptr);
    return false;
}

// Add a function to get the persistent client
Stream *getPersistentClient()
{
    return persistentClient;
}

// Add function to clear persistent client at the end of async operations
void clearPersistentClient()
{
    persistentClient = nullptr;
}

// Command Validation Functions
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
        if (strcmp(firstWord, "motor") != 0 && strcmp(firstWord, "system") != 0 &&
            strcmp(firstWord, "jog") != 0)
        {
            return cmdInfo->type;
        }

        // Special handling for commands that need subcommand checking
        if (strcmp(firstWord, "motor") == 0)
        {
            // Make a local copy that we can modify
            char command[64];
            strncpy(command, originalCommand, 63);
            command[63] = '\0';

            // Convert commas to spaces
            for (int i = 0; command[i]; i++)
            {
                if (command[i] == ',')
                {
                    command[i] = ' ';
                }
            }

            // Check for motor stop/abort which are emergency commands
            if (strncmp(command + 6, "stop", 4) == 0 ||
                strncmp(command + 6, "abort", 5) == 0)
            {
                return CMD_EMERGENCY;
            }

            // Check for motor status which is read-only
            if (strncmp(command + 6, "status", 6) == 0)
            {
                return CMD_READ_ONLY;
            }

            // All other motor commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for system commands
        if (strcmp(firstWord, "system") == 0)
        {
            // Make a local copy that we can modify
            char command[64];
            strncpy(command, originalCommand, 63);
            command[63] = '\0';

            // Convert commas to spaces
            for (int i = 0; command[i]; i++)
            {
                if (command[i] == ',')
                {
                    command[i] = ' ';
                }
            }

            // These are all read-only
            if (strncmp(command + 7, "state", 5) == 0 ||
                strncmp(command + 7, "safety", 6) == 0 ||
                strncmp(command + 7, "trays", 5) == 0 ||
                strncmp(command + 7, "history", 7) == 0)
            {
                return CMD_READ_ONLY;
            }

            // system reset is modifying
            return CMD_MODIFYING;
        }

        // Special handling for teach commands
        if (strcmp(firstWord, "teach") == 0)
        {
            // Make a local copy that we can modify
            char command[64];
            strncpy(command, originalCommand, 63);
            command[63] = '\0';

            // Convert commas to spaces
            for (int i = 0; command[i]; i++)
            {
                if (command[i] == ',')
                {
                    command[i] = ' ';
                }
            }

            // Read-only teach commands
            if (strncmp(command + 6, "status", 6) == 0 ||
                strncmp(command + 6, "help", 4) == 0)
            {
                return CMD_READ_ONLY;
            }

            // All other teach commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for jog commands
        if (strcmp(firstWord, "jog") == 0)
        {
            // Make a local copy that we can modify
            char command[64];
            strncpy(command, originalCommand, 63);
            command[63] = '\0';

            // Convert commas to spaces
            for (int i = 0; command[i]; i++)
            {
                if (command[i] == ',')
                {
                    command[i] = ' ';
                }
            }

            // Read-only jog commands
            if (strncmp(command + 4, "status", 6) == 0 ||
                strcmp(command, "jog inc") == 0 ||
                strcmp(command, "jog speed") == 0)
            {
                return CMD_READ_ONLY;
            }

            // All other jog commands are modifying
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

    // For test commands, check if any operation or another test is in progress
    if (cmdType == CMD_TEST)
    {
        if (operationInProgress)
        {
            sendCommandRejection(command, "Operation in progress");
            return false;
        }
        if (testInProgress)
        {
            sendCommandRejection(command, "Another test is already running");
            return false;
        }
        return true;
    }

    // For other modifying commands, check if an operation or test is in progress
    if (operationInProgress)
    {
        sendCommandRejection(command, "Operation in progress");
        return false;
    }

    if (testInProgress)
    {
        sendCommandRejection(command, "Test in progress");
        return false;
    }

    // Command is allowed
    return true;
}

// Function to filter commands that shouldn't be logged to history
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

//-------------------------------------------------------------------------
// Utility Functions
//-------------------------------------------------------------------------

void sendCommandRejection(const char *command, const char *reason)
{
    char msg[128];

    if (operationInProgress)
    {
        // This is truly a BUSY condition
        snprintf(msg, sizeof(msg),
                 "[BUSY], Cannot execute '%s' - %s operation in progress. Use 'abort' to cancel.",
                 command, getOperationTypeName(currentOperation.type));
    }
    else if (testInProgress)
    {
        // This is also a BUSY condition
        snprintf(msg, sizeof(msg),
                 "[BUSY], Cannot execute '%s' - Test in progress. Use 'abort' to cancel.",
                 command);
    }
    else
    {
        // This is an ERROR condition, not BUSY
        snprintf(msg, sizeof(msg),
                 "[ERROR], Cannot execute '%s' - %s",
                 command, reason);
    }

    Console.println(msg);
}

const char *getOperationTypeName(int type)
{
    switch (type)
    {
    case 1: // OPERATION_LOAD_TRAY
        return "Tray loading";
    case 2: // OPERATION_UNLOAD_TRAY
        return "Tray unloading";
    case 3: // OPERATION_HOME
        return "Homing";
    // Add other operation types as needed
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