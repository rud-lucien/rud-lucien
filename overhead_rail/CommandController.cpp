#include "CommandController.h"
#include "Commands.h"
#include "Utils.h"
#include "SystemState.h"
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
// Complete command set for the overhead rail system
const CommandInfo COMMAND_TABLE[] = {
    {"abort", CMD_EMERGENCY, 0},
    {"encoder", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY | CMD_FLAG_ASYNC},
    {"goto", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"h", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"help", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"jog", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"labware", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"log", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"network", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"rail1", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"rail2", CMD_MODIFYING, CMD_FLAG_ASYNC},
    {"system", CMD_READ_ONLY, CMD_FLAG_NO_HISTORY},
    {"teach", CMD_MODIFYING, 0}
};

// Number of commands in the table
const size_t COMMAND_TABLE_SIZE = sizeof(COMMAND_TABLE) / sizeof(CommandInfo);

// Operation state tracking
bool operationInProgress = false;

// Command tracking for system state reporting
char lastExecutedCommand[MAX_COMMAND_LENGTH] = "";
unsigned long lastCommandTime = 0;
bool lastCommandSuccess = false;
CommandType lastCommandType = CMD_READ_ONLY;
char lastCommandSource[16] = "";
unsigned long systemStartTime = 0;

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
// COMMAND EXECUTION - Direct dispatch to Commands.cpp functions
//=============================================================================

bool executeCommand(const char *command, Stream *output)
{
    // Store command info for system state reporting
    strncpy(lastExecutedCommand, command, MAX_COMMAND_LENGTH - 1);
    lastExecutedCommand[MAX_COMMAND_LENGTH - 1] = '\0';
    lastCommandTime = millis();
    lastCommandType = getCommandType(command);
    strcpy(lastCommandSource, (output == &Serial) ? "SERIAL" : "NETWORK");
    
    char firstWord[16] = {0};
    int i = 0;
    while (command[i] && command[i] != ',' && command[i] != ' ' && i < 15)
    {
        firstWord[i] = command[i];
        i++;
    }
    firstWord[i] = '\0';

    // Calculate args pointer (skip "command,")
    const char* argsStart = strchr(command, ',');
    char* args = const_cast<char*>(argsStart ? argsStart + 1 : "");
    
    // Create a CommandCaller wrapper for the Stream
    class StreamCommandCaller : public CommandCaller {
    private:
        Stream* stream;
    public:
        StreamCommandCaller(Stream* s) : stream(s) {}
        
        size_t write(uint8_t c) override {
            return stream->write(c);
        }
        
        size_t write(const uint8_t *buffer, size_t size) override {
            return stream->write(buffer, size);
        }
        
        int available() override {
            return stream->available();
        }
        
        int read() override {
            return stream->read();
        }
        
        int peek() override {
            return stream->peek();
        }
        
        void flush() override {
            stream->flush();
        }
    };
    
    StreamCommandCaller caller(output);

    // Dispatch to appropriate command function
    bool success = false;
    
    if (strcmp(firstWord, "rail1") == 0) {
        success = cmd_rail1(args, &caller);
    }
    else if (strcmp(firstWord, "rail2") == 0) {
        success = cmd_rail2(args, &caller);
    }
    else if (strcmp(firstWord, "goto") == 0) {
        success = cmd_goto(args, &caller);
    }
    else if (strcmp(firstWord, "jog") == 0) {
        success = cmd_jog(args, &caller);
    }
    else if (strcmp(firstWord, "encoder") == 0) {
        success = cmd_encoder(args, &caller);
    }
    else if (strcmp(firstWord, "teach") == 0) {
        success = cmd_teach(args, &caller);
    }
    else if (strcmp(firstWord, "labware") == 0) {
        success = cmd_labware(args, &caller);
    }
    else if (strcmp(firstWord, "log") == 0) {
        success = cmd_log(args, &caller);
    }
    else if (strcmp(firstWord, "system") == 0) {
        success = cmd_system(args, &caller);
    }
    else if (strcmp(firstWord, "help") == 0 || strcmp(firstWord, "h") == 0) {
        success = cmd_print_help(args, &caller);
    }
    else if (strcmp(firstWord, "network") == 0) {
        success = cmd_network(args, &caller);
    }
    else {
        output->println(F("[ERROR] Command not recognized"));
        success = false;
    }

    lastCommandSuccess = success;
    return success;
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
        // Special handling for rail commands that have emergency subcommands
        if (strcmp(firstWord, "rail1") == 0 || strcmp(firstWord, "rail2") == 0)
        {
            // Check for emergency subcommands
            if (strstr(originalCommand, ",abort") != nullptr ||
                strstr(originalCommand, ",stop") != nullptr)
            {
                return CMD_EMERGENCY;
            }

            // Check for read-only subcommands
            if (strstr(originalCommand, ",status") != nullptr ||
                strstr(originalCommand, ",help") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // All other rail commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for system commands
        if (strcmp(firstWord, "system") == 0)
        {
            // System status commands are read-only
            if (strstr(originalCommand, ",state") != nullptr ||
                strstr(originalCommand, ",help") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // System reset/home commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for teach commands
        if (strcmp(firstWord, "teach") == 0)
        {
            // Status commands are read-only
            if (strstr(originalCommand, ",status") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // All other teach commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for labware commands
        if (strcmp(firstWord, "labware") == 0)
        {
            // Status and help are read-only
            if (strstr(originalCommand, ",status") != nullptr ||
                strstr(originalCommand, ",help") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // Audit and reset are modifying
            return CMD_MODIFYING;
        }

        // Special handling for encoder commands
        if (strcmp(firstWord, "encoder") == 0)
        {
            // Status and help are read-only
            if (strstr(originalCommand, ",status") != nullptr ||
                strstr(originalCommand, ",help") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // All other encoder commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for jog commands (all modifying)
        if (strcmp(firstWord, "jog") == 0)
        {
            // Status and help are read-only
            if (strstr(originalCommand, ",status") != nullptr ||
                strstr(originalCommand, ",help") != nullptr)
            {
                return CMD_READ_ONLY;
            }

            // All other jog commands are modifying
            return CMD_MODIFYING;
        }

        // Special handling for network commands (all read-only)
        if (strcmp(firstWord, "network") == 0)
        {
            return CMD_READ_ONLY;
        }

        // Special handling for log commands (all read-only)
        if (strcmp(firstWord, "log") == 0)
        {
            return CMD_READ_ONLY;
        }

        // For other commands, use the type from the table
        return cmdInfo->type;
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
// COMMAND TRACKING FUNCTIONS
//=============================================================================

void initializeSystemStartTime()
{
    systemStartTime = millis();
}

const char* getLastCommandStatus()
{
    static char statusBuffer[80];
    
    if (lastExecutedCommand[0] == '\0') {
        // No commands issued yet
        unsigned long uptime = millis() - systemStartTime;
        if (uptime < 60000) { // Less than 1 minute
            return "System starting up - no commands yet";
        } else {
            return "Ready - awaiting first command";
        }
    }
    
    // Show actual command info
    unsigned long age = millis() - lastCommandTime;
    
    if (age < 5000) { // Less than 5 seconds - show recent
        snprintf(statusBuffer, sizeof(statusBuffer), 
                "RECENT: %s (%s) - %s", 
                lastExecutedCommand,
                lastCommandSource,
                lastCommandSuccess ? "OK" : "FAILED");
    } else if (age < 300000) { // Less than 5 minutes - show with time
        snprintf(statusBuffer, sizeof(statusBuffer), 
                "LAST: %s (%lus ago)", 
                lastExecutedCommand, 
                age / 1000);
    } else {
        // Old command - just show it was executed
        snprintf(statusBuffer, sizeof(statusBuffer), 
                "LAST: %s (>5min ago)", 
                lastExecutedCommand);
    }
    
    return statusBuffer;
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
