#include "CommandHandler.h"

// Global test control flags
bool testInProgress = false;
bool testAbortRequested = false;

void initCommandHandler()
{
    // Initialize any command handler state here
    testInProgress = false;
    testAbortRequested = false;
}

// Function to determine command type for filtering
CommandType getCommandType(const char *command)
{
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
    if (strstr(command, "status") ||
        strstr(command, "motor status") ||
        strstr(command, "position") ||
        strstr(command, "system state") ||
        strstr(command, "system safety") ||
        strstr(command, "system trays") ||
        strstr(command, "help") ||
        strstr(command, "h ") ||
        strstr(command, "encoder status") ||
        strstr(command, "jog status") ||
        strstr(command, "tray status"))
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

// Check if a command can be executed given the current system state
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

// Format and send a command rejection message
void sendCommandRejection(const char *command, const char *reason)
{
    char errorMsg[128];
    snprintf(errorMsg, sizeof(errorMsg), "Cannot execute '%s' - %s", command, reason);
    Console.error(errorMsg);

    if (operationInProgress)
    {
        char infoMsg[128];
        snprintf(infoMsg, sizeof(infoMsg), "%s operation in progress. Use 'abort' to cancel.",
                 getOperationTypeName(currentOperation.type));
        Console.info(infoMsg);
    }
}

// Get operation type name for error messages
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

// Trim leading whitespace from a string
char *trimLeadingSpaces(char *str)
{
    while (*str && isspace(*str))
    {
        str++;
    }
    return str;
}

// Main command handling function
void handleSerialCommands()
{
    static char commandBuffer[64]; // Buffer for incoming commands
    static uint8_t commandIndex = 0;

    while (Serial.available())
    {
        char c = Serial.read();

        // Special handling for test mode - any input aborts
        if (testInProgress && c != '\r' && c != '\n')
        {
            // Set the abort flag instead of just returning
            testAbortRequested = true;

            // Clear the entire buffer to prevent multiple abort triggers
            while (Serial.available() > 0)
            {
                Serial.read();
            }

            // Don't process the character as a command
            return;
        }

        if (c == '\n')
        {
            commandBuffer[commandIndex] = '\0'; // Null-terminate the command

            // Print the received command for debugging
            Console.serialCommand(commandBuffer);

            // Make a copy of the original command for permission checking
            char originalCommand[64];
            strncpy(originalCommand, commandBuffer, 63);
            originalCommand[63] = '\0';

            // Pre-process: Replace all commas with spaces
            for (uint8_t i = 0; i < commandIndex; i++)
            {
                if (commandBuffer[i] == ',')
                {
                    commandBuffer[i] = ' ';
                }
            }

            // Check if this command can be executed
            if (canExecuteCommand(originalCommand))
            {
                // Command is allowed, execute it
                // bool success = commander.execute(commandBuffer, &Serial);
                bool success = commander.execute(commandBuffer, &Console);

                // Print error ONLY when commander couldn't find the command
                if (!success)
                {
                    // Check if this looks like a valid main command by checking against the command tree
                    bool isKnownCommand = false;
                    for (size_t i = 0; i < sizeof(API_tree) / sizeof(Commander::systemCommand_t); i++)
                    {
                        const char *cmdName = API_tree[i].name;
                        // See if the command matches the first part of the input
                        if (strncmp(commandBuffer, cmdName, strlen(cmdName)) == 0)
                        {
                            isKnownCommand = true;
                            break;
                        }
                    }

                    // Only print generic error if command wasn't found in the command tree
                    if (!isKnownCommand)
                    {
                        Console.error(F("Command not found"));
                    }
                }
            }
            // If command is not allowed, canExecuteCommand already printed the error message

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
                Console.warning(F("Command truncated - exceeded maximum length"));
            }
        }
    }
}

/*TODO: implement handleEthernetCommands after setting up ethernet*/
// void handleEthernetCommands(EthernetClient& client) {
//     static char commandBuffer[64];
//     static uint8_t commandIndex = 0;

//     while (client.available()) {
//         char c = client.read();

//         // Similar logic to handleSerialCommands

//         if (c == '\n') {
//             commandBuffer[commandIndex] = '\0';

//             // Use ethernetCommand instead of serialCommand
//             Console.ethernetCommand(commandBuffer);

//             // Rest of the command processing
//             // ...
//         }
//     }
// }