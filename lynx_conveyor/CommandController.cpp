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

            // Print the received command for debugging
            Serial.println(commandBuffer);

            // Process the command using the shared function
            processCommand(commandBuffer, &Serial); // Or whatever Stream object Console is

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
                // Add notification of truncation
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
            // Update activity timestamp when client sends data
            extern unsigned long clientLastActivityTime[];
            clientLastActivityTime[i] = millis();

            // Read the command
            int j = 0;
            while (clients[i].available() && j < 63)
            { // Standardize to 63 bytes + null terminator
                char c = clients[i].read();
                if (c == '\n' || c == '\r')
                {
                    // End of command
                    ethernetCommandBuffer[j] = '\0';

                    // Process the command if it's not empty
                    if (j > 0)
                    {
                        // Log the received command
                        Serial.print(F("[NETWORK] Command from "));
                        Serial.print(clients[i].remoteIP());
                        Serial.print(F(": "));
                        Serial.println(ethernetCommandBuffer);

                        // Use the shared process function instead of duplicating logic
                        processCommand(ethernetCommandBuffer, &clients[i]);
                    }

                    // Reset for next command
                    j = 0;
                    continue;
                }

                // Store character in buffer
                ethernetCommandBuffer[j++] = c;
            }

            // If buffer is full but no newline found, warn and clear buffer
            if (j == 63)
            {
                ethernetCommandBuffer[j] = '\0';
                clients[i].println(F("[ERROR], Command too long - truncated"));
                j = 0;
            }
        }
    }
}

bool processCommand(const char *rawCommand, Stream *output)
{
    // Set current client
    Console.setCurrentClient(output);

    // Check for abort command with priority
    if (strncmp(rawCommand, "abort", 5) == 0)
    {
        requestTestAbort("command interface");
        Console.acknowledge(F("Test abort requested"));
        return true;
    }

    // Check if this is a potentially async command
    bool isAsyncCommand =
        (strncmp(rawCommand, "motor,", 6) == 0) ||
        (strncmp(rawCommand, "move,", 5) == 0) ||
        (strncmp(rawCommand, "test,", 5) == 0) ||
        (strncmp(rawCommand, "jog,", 4) == 0) ||
        (strncmp(rawCommand, "tray,", 5) == 0) ||
        (strncmp(rawCommand, "encoder,", 8) == 0) ||
        (strncmp(rawCommand, "lock,", 5) == 0) ||
        (strncmp(rawCommand, "unlock,", 7) == 0);

    // Make a copy of the original command for permission checking
    char originalCommand[64];
    strncpy(originalCommand, rawCommand, 63);
    originalCommand[63] = '\0';

    // Pre-process: Convert command for Commander API
    char processedCommand[64];
    strncpy(processedCommand, rawCommand, 63);
    processedCommand[63] = '\0';

    // Replace all commas with spaces for the Commander API
    for (int i = 0; processedCommand[i]; i++)
    {
        if (processedCommand[i] == ',')
        {
            processedCommand[i] = ' ';
        }
    }

    // Check if this command can be executed
    if (canExecuteCommand(originalCommand))
    {
        // Execute the command
        bool success = commander.execute(processedCommand, output);

        // Handle error reporting if command not found
        if (!success)
        {
            // Check if this looks like a valid main command
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

            // Only print generic error if command wasn't found at all
            if (!isKnownCommand)
            {
                Console.serialError(F("Command not found"));
            }
        }

        // Only reset currentClient if it's not an async command
        if (!isAsyncCommand)
        {
            Console.setCurrentClient(nullptr);
        }

        return success;
    }

    // Reset the client when done
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
    // Make a local copy that we can modify
    char command[64];
    strncpy(command, originalCommand, 63);
    command[63] = '\0';

    // Convert commas to spaces for consistent pattern matching
    for (int i = 0; command[i]; i++)
    {
        if (command[i] == ',')
        {
            command[i] = ' ';
        }
    }

    // Emergency commands - always allowed
    if (strstr(command, "motor stop") ||
        strstr(command, "motor abort") ||
        strstr(command, "stop") ||
        strstr(command, "abort") ||
        strstr(command, "estop"))
    {
        return CMD_EMERGENCY;
    }

    // Read-only commands - always allowed
    if (strstr(command, "help") ||
        strstr(command, "h ") ||
        strstr(command, "H ") ||

        // All status commands
        strstr(command, "motor status") ||
        strstr(command, "jog status") ||
        strstr(command, "tray status") ||
        strstr(command, "encoder status") ||
        strstr(command, "network status") ||

        // All help subcommands
        strstr(command, "lock help") ||
        strstr(command, "unlock help") ||
        strstr(command, "log help") ||
        strstr(command, "system help") ||
        strstr(command, "motor help") ||
        strstr(command, "move help") ||
        strstr(command, "jog help") ||
        strstr(command, "tray help") ||
        strstr(command, "test help") ||
        strstr(command, "encoder help") ||
        strstr(command, "network help") ||

        // System information commands
        strstr(command, "system state") ||
        strstr(command, "system safety") ||
        strstr(command, "system trays") ||

        // Other read-only commands that need exact matching
        strstr(command, "log now") ||
        // Much simpler checks for the exact commands without parameters
        strcmp(command, "jog inc") == 0 ||
        strcmp(command, "jog speed") == 0 ||
        strcmp(command, "encoder multiplier") == 0)
    {
        return CMD_READ_ONLY;
    }

    // Test commands - special handling
    if (strstr(command, "test"))
    {
        return CMD_TEST;
    }

    // All other commands are modifying
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