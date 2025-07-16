#include "Commands.h"
#include "CommandController.h"
#include "Logging.h"
#include "RailAutomation.h"
#include "HandoffController.h"
#include "LabwareAutomation.h"

/*
=============================================================================
COMMAND FUNCTION LOCATIONS
=============================================================================
SYSTEM LEVEL:
  cmd_system()     - Line 407   (state, home, reset)
  cmd_log()        - Line 215   (monitoring, history)
  cmd_network()    - Line 1692  (connectivity)

HARDWARE CONTROL:
  cmd_rail1()      - Line 1047  (Rail 1 operations)
  cmd_rail2()      - Line 686   (Rail 2 operations)
  cmd_encoder()    - Line 1815  (manual control)
  cmd_jog()        - Line 1990  (manual movement)

AUTOMATION:
  cmd_labware()    - Line 1335  (state management)
  cmd_goto()       - Line 1478  (coordinated movement)
  cmd_teach()      - Line 517   (position setup)
=============================================================================
*/

// ============================================================
// Binary search function for subcommand lookup
// ============================================================
int findSubcommandCode(const char *subcommand, const SubcommandInfo *commandTable, size_t tableSize)
{
    int left = 0;
    int right = tableSize - 1;

    while (left <= right)
    {
        int mid = left + (right - left) / 2;
        int cmp = strcmp(subcommand, commandTable[mid].name);

        if (cmp == 0)
            return commandTable[mid].code; // Found - return the code

        if (cmp < 0)
            right = mid - 1;
        else
            left = mid + 1;
    }

    return 0; // Not found - return 0 (default case)
}

// ============================================================
// Global Command Tree and Commander Object
// ============================================================

bool cmd_print_help(char *args, CommandCaller *caller)
{
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    char *trimmed = trimLeadingSpaces(localArgs);

    // If the user requested help for a specific command,
    // we currently do not have detailed help implemented.
    if (strlen(trimmed) > 0)
    {
        Console.error(F("Detailed help for specific commands is not implemented."));
        return false;
    }
    else
    {
        // No specific command requested; print general help.
        // Pass a pointer to the Serial stream.

        // Print general help information.
        Console.println(F("--------------------------------------------------"));
        Console.println(F("Overhead Rail System Command Help:"));
        Console.println(F("--------------------------------------------------"));

        commander.printHelp(caller, true, true);

        Console.println(F("--------------------------------------------------"));
        return true;
    }
}

Commander commander;

Commander::systemCommand_t API_tree[] = {
    systemCommand("help", "Display help information for all commands", cmd_print_help),
    systemCommand("h", "Display help information for all commands", cmd_print_help),
    systemCommand("H", "Display help information for all commands", cmd_print_help),

    // Logging command
    systemCommand("log", "Logging controls and history:\r\n"
                         "  log,on,[interval] - Enable periodic logging (interval in ms, default 250)\r\n"
                         "  log,off           - Disable periodic logging\r\n"
                         "  log,now           - Log system state immediately\r\n"
                         "  log,history       - Show complete operation log history\r\n"
                         "  log,errors        - Show only errors and warnings for quick debugging\r\n"
                         "  log,last,[count]  - Show last N log entries (default: 10)\r\n"
                         "  log,stats         - Show log buffer statistics and overflow info\r\n"
                         "  log,help          - Display detailed logging information",
                  cmd_log),

    // Labware automation command
    systemCommand("labware", "Labware automation and state management:\r\n"
                             "  labware,status      - Display current labware tracking state and operation history\r\n"
                             "  labware,audit       - Automatically validate and fix labware state\r\n"
                             "  labware,reset       - Clear all labware tracking and reset operation history\r\n"
                             "  labware,help        - Display detailed labware automation instructions",
                  cmd_labware),

    // Automated labware movement command
    systemCommand("goto", "Automated work cell movement with labware tracking:\r\n"
                          "  goto,<location>,<status>  - Move to work cell with labware status\r\n"
                          "  Locations: wc1, wc2, wc3\r\n"
                          "  Status: with-labware, no-labware\r\n"
                          "  Examples:\r\n"
                          "    goto,wc1,with-labware   - Move to WC1 with labware\r\n"
                          "    goto,wc2,no-labware     - Move to WC2 without labware\r\n"
                          "    goto,wc3,with-labware   - Move to WC3 with labware\r\n"
                          "  goto,help               - Display detailed goto command instructions",
                  cmd_goto),

    // System state command to display comprehensive system status
    systemCommand("system", "System commands:\r\n"
                            "  system,state    - Display comprehensive system status with readiness assessment\r\n"
                            "  system,home     - Home both rails sequentially (Rail 1 first, then Rail 2)\r\n"
                            "  system,reset    - Clear operational state for clean automation (motor faults, encoder, etc.)\r\n"
                            "  system,help     - Display detailed instructions for system commands\r\n"
                            "                    (Use 'log,history' or 'log,errors' for operation troubleshooting)",
                  cmd_system),

    // Encoder control commands
    systemCommand("encoder", "Manual Pulse Generator (MPG) handwheel control:\r\n"
                             "  encoder,enable,<rail>    - Enable encoder control for Rail 1 or 2\r\n"
                             "  encoder,disable          - Disable encoder control\r\n"
                             "  encoder,multiplier,<X>   - Set encoder multiplier (X = 1, 10, or 100)\r\n"
                             "  encoder,velocity,<RPM>   - Set encoder velocity (50-400 RPM)\r\n"
                             "  encoder,status           - Display current encoder status and settings\r\n"
                             "  encoder,help             - Display detailed setup and usage instructions",
                  cmd_encoder),

    // Jog command
    systemCommand("jog", "Manual jogging control for dual-rail system:\r\n"
                         "  jog,<rail>,+,[mm]        - Jog rail forward by increment or custom distance\r\n"
                         "  jog,<rail>,-,[mm]        - Jog rail backward by increment or custom distance\r\n"
                         "  jog,<rail>,increment,<mm> - Set default jog increment for rail\r\n"
                         "  jog,<rail>,speed,<rpm>   - Set jog speed for rail\r\n"
                         "  jog,<rail>,status        - Display jog settings for specific rail\r\n"
                         "  jog,status               - Display jog settings for all rails\r\n"
                         "  jog,help                 - Display detailed usage instructions",
                  cmd_jog),

    // Network management command
    systemCommand("network", "Network management:\r\n"
                             "  network,status     - Display current network status and client info\r\n"
                             "  network,disconnect - Disconnect the current client\r\n"
                             "  network,help       - Display detailed network management instructions",
                  cmd_network),

    // Teach position command
    systemCommand("teach", "Position teaching system with automatic SD card persistence:\r\n"
                           "  teach,<rail>,<position>  - Teach current position and auto-save to SD card\r\n"
                           "  teach,<rail>,status      - Show taught positions for specific rail\r\n"
                           "  teach,status             - Show all taught positions and system status\r\n"
                           "  teach,<rail>,reset       - Reset rail positions to factory defaults\r\n"
                           "  teach,reset              - Reset all positions to factory defaults\r\n"
                           "  \r\n"
                           "  Rail 1 positions: staging, wc1, wc2, handoff\r\n"
                           "  Rail 2 positions: handoff, wc3\r\n"
                           "  \r\n"
                           "  Examples:\r\n"
                           "    teach,1,staging        - Teach Rail 1 staging position\r\n"
                           "    teach,2,wc3            - Teach Rail 2 WC3 position\r\n"
                           "    teach,1,status         - Show Rail 1 position status\r\n"
                           "    teach,1,reset          - Reset Rail 1 to defaults",
                  cmd_teach),

    // Rail 1 control command
    systemCommand("rail1", "Rail 1 Control Commands:\r\n"
                           "  rail1,init          - Initialize Rail 1 motor system\r\n"
                           "  rail1,clear-fault   - Clear motor fault condition\r\n"
                           "  rail1,abort         - Abort current operation gracefully\r\n"
                           "  rail1,stop          - Emergency stop motor movement\r\n"
                           "  rail1,home          - Home carriage to reference position\r\n"
                           "  rail1,move-wc1,no-labware     - Move empty carriage to WC1\r\n"
                           "  rail1,move-wc1,with-labware   - Move carriage with labware to WC1\r\n"
                           "  rail1,move-wc2,no-labware     - Move empty carriage to WC2\r\n"
                           "  rail1,move-wc2,with-labware   - Move carriage with labware to WC2\r\n"
                           "  rail1,move-staging,no-labware     - Move empty carriage to staging position\r\n"
                           "  rail1,move-staging,with-labware   - Move carriage with labware to staging position\r\n"
                           "  rail1,move-handoff,no-labware - Move empty carriage to handoff\r\n"
                           "  rail1,move-handoff,with-labware - Move carriage with labware to handoff\r\n"
                           "  rail1,move-mm-to,X,no-labware - Move empty carriage to absolute position X mm\r\n"
                           "  rail1,move-mm-to,X,with-labware - Move carriage with labware to absolute position X mm\r\n"
                           "  rail1,move-rel,X,no-labware   - Move empty carriage X mm relative to current position\r\n"
                           "  rail1,move-rel,X,with-labware - Move carriage with labware X mm relative to current position\r\n"
                           "  rail1,status        - Show comprehensive system status and diagnostics\r\n"
                           "  rail1,help          - Display detailed usage instructions",
                  cmd_rail1),

    // Rail 2 control command
    systemCommand("rail2", "Rail 2 Control Commands:\r\n"
                           "  rail2,init          - Initialize Rail 2 motor system\r\n"
                           "  rail2,clear-fault   - Clear motor fault condition\r\n"
                           "  rail2,abort         - Abort current operation gracefully\r\n"
                           "  rail2,stop          - Emergency stop motor movement\r\n"
                           "  rail2,extend        - Extend pneumatic drive\r\n"
                           "  rail2,retract       - Retract pneumatic drive\r\n"
                           "  rail2,home          - Home carriage to reference position\r\n"
                           "  rail2,move-wc3,no-labware     - Move empty carriage to WC3\r\n"
                           "  rail2,move-wc3,with-labware   - Move carriage with labware to WC3\r\n"
                           "  rail2,move-handoff,no-labware - Move empty carriage to handoff\r\n"
                           "  rail2,move-handoff,with-labware - Move carriage with labware to handoff\r\n"
                           "  rail2,move-mm-to,X,no-labware - Move empty carriage to absolute position X mm\r\n"
                           "  rail2,move-mm-to,X,with-labware - Move carriage with labware to absolute position X mm\r\n"
                           "  rail2,move-rel,X,no-labware   - Move empty carriage X mm relative to current position\r\n"
                           "  rail2,move-rel,X,with-labware - Move carriage with labware X mm relative to current position\r\n"
                           "  rail2,status        - Show comprehensive system status and diagnostics\r\n"
                           "  rail2,help          - Display detailed usage instructions\r\n"
                           "  SAFETY: Cylinder auto-retracts for ANY movement involving collision zone (500-700mm)",
                  cmd_rail2),
};

const size_t API_tree_size = sizeof(API_tree) / sizeof(Commander::systemCommand_t);

// ============================================================
// Log Command Implementation
// ============================================================

bool cmd_log(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: log,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");

    if (action == NULL)
    {
        Console.error(F("Invalid format. Usage: log,<action>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Handle different log commands
    if (strcmp(action, "on") == 0)
    {
        // Enable periodic logging
        unsigned long interval = DEFAULT_LOG_INTERVAL;

        if (param1 != NULL)
        {
            interval = atol(param1);
            if (interval < 100)
            {
                Console.error(F("LOG_INTERVAL_TOO_SMALL: Minimum logging interval is 100ms"));
                return false;
            }
            if (interval > 60000)
            {
                Console.error(F("LOG_INTERVAL_TOO_LARGE: Maximum logging interval is 60000ms (1 minute)"));
                return false;
            }
        }

        logging.logInterval = interval;
        logging.previousLogTime = millis(); // Reset timer

        Console.acknowledge((String(F("PERIODIC_LOGGING_ENABLED: Interval set to ")) + String(interval) + F("ms")).c_str());
        return true;
    }
    else if (strcmp(action, "off") == 0)
    {
        // Disable periodic logging
        logging.logInterval = 0;

        Console.acknowledge(F("PERIODIC_LOGGING_DISABLED: No automatic logging"));
        return true;
    }
    else if (strcmp(action, "now") == 0)
    {
        // Log system state immediately
        Console.acknowledge(F("SYSTEM_STATE_LOGGED: Current system state captured"));
        logSystemState();
        return true;
    }
    else if (strcmp(action, "history") == 0)
    {
        // Show complete operation log history
        Console.acknowledge(F("DISPLAYING_LOG_HISTORY: Complete operation log follows:"));
        opLogHistory.printHistory();
        return true;
    }
    else if (strcmp(action, "errors") == 0)
    {
        // Show only errors and warnings
        Console.acknowledge(F("DISPLAYING_ERROR_LOG: Error and warning entries follow:"));
        opLogHistory.printErrors();
        return true;
    }
    else if (strcmp(action, "last") == 0)
    {
        // Show last N log entries (default: 10)
        uint8_t count = 10;

        if (param1 != NULL)
        {
            count = atoi(param1);
            if (count > 50)
            {
                count = 50; // Limit to history size
            }
            if (count < 1)
            {
                count = 1;
            }
        }

        Console.acknowledge((String(F("DISPLAYING_LOG_LAST: Last ")) + String(count) + F(" log entries follow:")).c_str());
        opLogHistory.printLastN(count);
        return true;
    }
    else if (strcmp(action, "stats") == 0)
    {
        // Show log buffer statistics
        Console.acknowledge(F("DISPLAYING_LOG_STATS: Buffer statistics and status follow:"));
        opLogHistory.printStats();

        // Also show current logging status
        Console.serialInfo(F("Current logging status:"));
        if (logging.logInterval > 0)
        {
            Console.serialInfo((String(F("  Periodic logging: ENABLED (")) + String(logging.logInterval) + F("ms interval)")).c_str());
        }
        else
        {
            Console.serialInfo(F("  Periodic logging: DISABLED"));
        }
        return true;
    }
    else if (strcmp(action, "help") == 0)
    {
        // Display detailed help information
        Console.acknowledge(F("DISPLAYING_LOG_HELP: Logging system guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("Logging System Commands"));
        Console.println(F("============================================"));
        Console.println(F("PERIODIC LOGGING:"));
        Console.println(F("  log,on,[interval]   - Enable periodic system state logging"));
        Console.println(F("                        interval: 100-60000ms (default: 250ms)"));
        Console.println(F("  log,off             - Disable periodic logging"));
        Console.println(F(""));
        Console.println(F("IMMEDIATE LOGGING:"));
        Console.println(F("  log,now             - Log current system state immediately"));
        Console.println(F(""));
        Console.println(F("LOG HISTORY REVIEW:"));
        Console.println(F("  log,history         - Show complete operation log history"));
        Console.println(F("  log,errors          - Show only errors and warnings"));
        Console.println(F("  log,last,[count]    - Show last N entries (default: 10, max: 50)"));
        Console.println(F(""));
        Console.println(F("DIAGNOSTICS:"));
        Console.println(F("  log,stats           - Show log buffer statistics and current status"));
        Console.println(F(""));
        Console.println(F("LOGGED INFORMATION:"));
        Console.println(F("- Valve states and sensor feedback"));
        Console.println(F("- All carriage and labware sensors"));
        Console.println(F("- Motor states, homing status, HLFB for both rails"));
        Console.println(F("- Position, target, velocity for both rails"));
        Console.println(F("- E-stop status and air pressure"));
        Console.println(F("- Network client connections"));
        Console.println(F("- MPG (encoder) status and active rail"));
        Console.println(F(""));
        Console.println(F("COLOR CODING:"));
        Console.println(F("- Green: Normal/good states (homed, sensors active, etc.)"));
        Console.println(F("- Yellow: Active states (moving, extended, not homed)"));
        Console.println(F("- Red: Problem states (faults, low pressure, E-stop)"));
        Console.println(F("- Cyan: Section headers for easy scanning"));
        Console.println(F("============================================"));
        return true;
    }
    else
    {
        Console.error(F("Unknown log command. Available: on, off, now, history, errors, last, stats, help"));
        return false;
    }
}

// ============================================================
// System State Command Implementation
// ============================================================

// Define the system subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo SYSTEM_COMMANDS[] = {
    {"clear", 0},
    {"help", 1},
    {"home", 2},
    {"init", 3},
    {"reset", 4},
    {"state", 5}};

static const size_t SYSTEM_COMMAND_COUNT = sizeof(SYSTEM_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_system(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: system,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");

    // Find the command code
    int commandCode = findSubcommandCode(action, SYSTEM_COMMANDS, SYSTEM_COMMAND_COUNT);

    switch (commandCode)
    {
    case 0: // clear
        return clearSystemMotorFaults();

    case 1: // help
        Console.acknowledge(F("DISPLAYING_SYSTEM_HELP: System command guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("System State Commands"));
        Console.println(F("============================================"));
        Console.println(F("STATUS COMMAND:"));
        Console.println(F("  system,state        - Display comprehensive system status"));
        Console.println(F("                        (motors, sensors, pneumatics, network, safety)"));
        Console.println(F("                        Includes overall readiness assessment and error summary"));
        Console.println(F(""));
        Console.println(F("INITIALIZATION COMMANDS:"));
        Console.println(F("  system,init         - Initialize all motor systems"));
        Console.println(F("                        Initializes motors that need it, skips ready motors"));
        Console.println(F("                        Required after system startup or motor faults"));
        Console.println(F("  system,clear        - Clear motor faults for system readiness"));
        Console.println(F("                        Clears faults only on motors that have them"));
        Console.println(F("                        Use before init if motors are faulted"));
        Console.println(F(""));
        Console.println(F("HOMING COMMAND:"));
        Console.println(F("  system,home         - Home both rails sequentially (Rail 1 first, then Rail 2)"));
        Console.println(F("                        Verifies successful homing of each rail before proceeding"));
        Console.println(F("                        Use for first-time system initialization"));
        Console.println(F(""));
        Console.println(F("RESET COMMAND:"));
        Console.println(F("  system,reset        - Clear operational state for clean automation"));
        Console.println(F("                        Clears motor faults, resets encoder, syncs hardware state"));
        Console.println(F("                        Prepares system for fresh goto commands"));
        Console.println(F(""));
        Console.println(F("SYSTEM,STATE DISPLAYS:"));
        Console.println(F("- Motor status, sensors, pneumatics, network, safety"));
        Console.println(F("- Labware tracking and automation readiness"));
        Console.println(F("- Overall system health with error summary"));
        Console.println(F("============================================"));
        return true;

    case 2: // home
        return homeSystemRails();

    case 3: // init
        return initSystemMotors();

    case 4: // reset
        Console.acknowledge(F("SYSTEM_RESET_INITIATED: Clearing operational state for clean automation"));
        resetSystemState();
        return true;

    case 5: // state
        Console.acknowledge(F("DISPLAYING_SYSTEM_STATE: Comprehensive system status follows:"));
        printSystemState();
        return true;

    default:
        Console.error(F("Unknown system command. Available: state, clear, init, home, reset, help"));
        return false;
    }
}

// ============================================================
// Teach Position Command Implementation
// ============================================================

// Define global teach commands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo GLOBAL_TEACH_COMMANDS[] = {
    {"help", 0},
    {"reset", 1},
    {"status", 2}};

static const size_t GLOBAL_TEACH_COMMAND_COUNT = sizeof(GLOBAL_TEACH_COMMANDS) / sizeof(SubcommandInfo);

// Define Rail 1 position lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo RAIL1_POSITIONS[] = {
    {"handoff", 3},
    {"staging", 0},
    {"wc1", 1},
    {"wc2", 2}};

static const size_t RAIL1_POSITION_COUNT = sizeof(RAIL1_POSITIONS) / sizeof(SubcommandInfo);

// Define Rail 2 position lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo RAIL2_POSITIONS[] = {
    {"handoff", 0},
    {"wc3", 1}};

static const size_t RAIL2_POSITION_COUNT = sizeof(RAIL2_POSITIONS) / sizeof(SubcommandInfo);

bool cmd_teach(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameters. Usage: teach,<rail|status|reset|help>,[position]"));
        Console.error(F("Examples: teach 1 staging, teach status, teach reset, teach help"));
        return false;
    }

    // Parse the arguments - use spaces as separators
    char *param1 = strtok(trimmed, " ");
    char *param2 = strtok(nullptr, " ");

    if (param1 == NULL)
    {
        Console.error(F("Invalid format. Usage: teach,<rail|status|reset|help>,[position]"));
        return false;
    }

    // Trim leading spaces from param1
    param1 = trimLeadingSpaces(param1);

    // Convert param1 to lowercase for case-insensitive comparison
    for (int i = 0; param1[i]; i++)
    {
        param1[i] = tolower(param1[i]);
    }

    // First check if param1 is a global command
    int globalCmdCode = findSubcommandCode(param1, GLOBAL_TEACH_COMMANDS, GLOBAL_TEACH_COMMAND_COUNT);
    
    if (globalCmdCode != -1) {
        // Handle global commands
        switch (globalCmdCode) {
        case 0: // "help" - Display help information
            Console.acknowledge(F("DISPLAYING_TEACH_HELP: Position teaching system guide follows:"));
            Console.println(F("============================================"));
            Console.println(F("Position Teaching System Commands"));
            Console.println(F("============================================"));
            Console.println(F("GLOBAL COMMANDS:"));
            Console.println(F("  teach,status             - Show all taught positions and system status"));
            Console.println(F("                             Displays both rails with position values and validation"));
            Console.println(F("  teach,reset              - Reset all positions to factory defaults"));
            Console.println(F("                             Clears all custom positions and restores original values"));
            Console.println(F("  teach,help               - Display this comprehensive help guide"));
            Console.println(F(""));
            Console.println(F("POSITION TEACHING:"));
            Console.println(F("  teach,<rail>,<position>  - Teach current position and auto-save to SD card"));
            Console.println(F("                             Rail must be at desired position before teaching"));
            Console.println(F("                             Position is immediately saved to persistent storage"));
            Console.println(F(""));
            Console.println(F("RAIL 1 POSITIONS:"));
            Console.println(F("  teach,1,staging          - Teach Rail 1 staging position"));
            Console.println(F("                             (coordination position for Rail 1-2 transfers)"));
            Console.println(F("  teach,1,wc1              - Teach Rail 1 WC1 pickup/dropoff position"));
            Console.println(F("  teach,1,wc2              - Teach Rail 1 WC2 pickup/dropoff position"));
            Console.println(F("  teach,1,handoff          - Teach Rail 1 handoff position"));
            Console.println(F("                             (transfer point to Rail 2, typically same as home)"));
            Console.println(F(""));
            Console.println(F("RAIL 2 POSITIONS:"));
            Console.println(F("  teach,2,handoff          - Teach Rail 2 handoff position"));
            Console.println(F("                             (receive point from Rail 1)"));
            Console.println(F("  teach,2,wc3              - Teach Rail 2 WC3 pickup/dropoff position"));
            Console.println(F(""));
            Console.println(F("TEACHING WORKFLOW:"));
            Console.println(F("1. Home the rail to establish reference position"));
            Console.println(F("2. Move carriage to desired position using manual controls"));
            Console.println(F("3. Execute teach command to capture and save position"));
            Console.println(F("4. Verify with 'teach,status' command"));
            Console.println(F("5. Test movement with rail movement commands"));
            Console.println(F(""));
            Console.println(F("PERSISTENT STORAGE:"));
            Console.println(F("- All positions automatically saved to SD card"));
            Console.println(F("- Positions restored on system startup"));
            Console.println(F("- Factory defaults available as backup"));
            Console.println(F("- Position validation ensures reasonable values"));
            Console.println(F("============================================"));
            return true;

        case 1: // "reset" - Reset all positions
            return teachResetAllPositions();

        case 2: // "status" - Show all positions
            teachShowStatus();
            return true;

        default:
            // Should never reach here due to previous check
            break;
        }
    }

    // If not a global command, check if param1 is a rail number
    int rail = atoi(param1);
    if (rail != 1 && rail != 2)
    {
        Console.error(F("Invalid rail number or command. Use: 1, 2, status, reset, or help"));
        Console.error(F("Examples: teach 1 staging, teach 2 wc3, teach status, teach help"));
        return false;
    }

    // We have a valid rail number, check for second parameter
    if (param2 == NULL)
    {
        Console.error(F("Missing position. Usage: teach,<rail>,<position>"));
        if (rail == 1)
        {
            Console.error(F("Rail 1 positions: staging, wc1, wc2, handoff"));
        }
        else
        {
            Console.error(F("Rail 2 positions: handoff, wc3"));
        }
        return false;
    }

    // Trim and convert param2 to lowercase
    param2 = trimLeadingSpaces(param2);
    for (int i = 0; param2[i]; i++)
    {
        param2[i] = tolower(param2[i]);
    }

    // Try to find the position for the specified rail
    int positionCode = -1;

    if (rail == 1)
    {
        positionCode = findSubcommandCode(param2, RAIL1_POSITIONS, RAIL1_POSITION_COUNT);

        switch (positionCode)
        {
        case 0: // "staging"
            return teachRail1Staging();

        case 1: // "wc1"
            return teachRail1WC1Pickup();

        case 2: // "wc2"
            return teachRail1WC2Pickup();

        case 3: // "handoff"
            return teachRail1Handoff();

        default:
            Console.error(F("Unknown Rail 1 position. Available: staging, wc1, wc2, handoff"));
            return false;
        }
    }
    else if (rail == 2)
    {
        positionCode = findSubcommandCode(param2, RAIL2_POSITIONS, RAIL2_POSITION_COUNT);

        switch (positionCode)
        {
        case 0: // "handoff"
            return teachRail2Handoff();

        case 1: // "wc3"
            return teachRail2WC3Pickup();

        default:
            Console.error(F("Unknown Rail 2 position. Available: handoff, wc3"));
            return false;
        }
    }

    return false; // Should never reach here
}

// ============================================================
// Rail 2 Control Commands
// ============================================================

// Define the rail2 subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo RAIL2_COMMANDS[] = {
    {"abort", 0},
    {"clear-fault", 1},
    {"extend", 2},
    {"help", 3},
    {"home", 4},
    {"init", 5},
    {"move-handoff", 6},
    {"move-mm-to", 7},
    {"move-rel", 8},
    {"move-wc3", 9},
    {"retract", 10},
    {"status", 11},
    {"stop", 12}};

static const size_t RAIL2_COMMAND_COUNT = sizeof(RAIL2_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_rail2(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: rail2,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");
    char *param2 = strtok(nullptr, " ");

    if (action == NULL)
    {
        Console.error(F("Invalid format. Usage: rail2,<action>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Declare all variables at the beginning before switch
    ValveOperationResult result;
    double currentPos = 0.0;
    double targetPosition = 0.0;
    double calculatedTargetPos = 0.0;
    PositionTarget targetPos;
    bool carriageLoaded = false;
    bool movementInCollisionZone = false;
    ValvePosition valveState;
    bool sensorRetracted = false;
    bool sensorExtended = false;
    bool validationResult = false;
    bool actuallyRetracted = false;
    bool actuallyExtended = false;
    String motorStatus; // For status consolidation

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, RAIL2_COMMANDS, RAIL2_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {

    case 0: // "abort" - Abort current operation gracefully
        return executeRailAbort(2);

    case 1: // "clear-fault" - Clear motor fault condition
        return executeRailClearFault(2);

    case 2: // "extend" - Extend pneumatic drive
        if (!isPressureSufficient())
        {
            Console.error(F("INSUFFICIENT_PRESSURE: Air pressure too low for valve operation"));
            return false;
        }

        Console.serialInfo(F("Extending pneumatic drive..."));
        result = extendCylinder();

        if (result == VALVE_OP_SUCCESS)
        {
            Console.acknowledge(F("CYLINDER_EXTENDED: Pneumatic drive is now extended"));
            return true;
        }
        else
        {
            Console.error((String(F("EXTEND_FAILED: ")) + getValveOperationResultName(result)).c_str());
            return false;
        }

    case 3: // "help" - Display help information
        Console.acknowledge(F("DISPLAYING_RAIL2_HELP: Command reference follows:"));
        Console.println(F("============================================"));
        Console.println(F("Rail 2 Control Commands"));
        Console.println(F("============================================"));
        Console.println(F("MOTOR CONTROL:"));
        Console.println(F("  rail2,init          - Initialize Rail 2 motor system"));
        Console.println(F("  rail2,clear-fault   - Clear motor fault condition"));
        Console.println(F("  rail2,abort         - Abort current operation gracefully"));
        Console.println(F("  rail2,stop          - Emergency stop motor movement"));
        Console.println(F(""));
        Console.println(F("PNEUMATIC DRIVE CONTROL:"));
        Console.println(F("  rail2,extend        - Extend pneumatic drive"));
        Console.println(F("  rail2,retract       - Retract pneumatic drive"));
        Console.println(F(""));
        Console.println(F("HOMING OPERATION:"));
        Console.println(F("  rail2,home          - Home carriage (find WC3 position)"));
        Console.println(F("                        Automatically detects labware on carriage"));
        Console.println(F(""));
        Console.println(F("CARRIAGE MOVEMENT:"));
        Console.println(F("  rail2,move-wc3,no-labware     - Move empty carriage to WC3"));
        Console.println(F("  rail2,move-handoff,no-labware - Move empty carriage to handoff"));
        Console.println(F("  rail2,move-wc3,with-labware   - Move carriage with labware to WC3"));
        Console.println(F("  rail2,move-handoff,with-labware - Move carriage with labware to handoff"));
        Console.println(F(""));
        Console.println(F("MANUAL POSITIONING:"));
        Console.print(F("  rail2,move-mm-to,X,no-labware   - Move empty carriage to absolute position X mm (0-"));
        Console.print(RAIL2_MAX_TRAVEL_MM);
        Console.println(F(")"));
        Console.println(F("  rail2,move-mm-to,X,with-labware - Move carriage with labware to absolute position X mm"));
        Console.println(F("  rail2,move-rel,X,no-labware     - Move empty carriage X mm relative (+ forward, - backward)"));
        Console.println(F("  rail2,move-rel,X,with-labware   - Move carriage with labware X mm relative"));
        Console.println(F(""));
        Console.println(F("STATUS AND DIAGNOSTICS:"));
        Console.println(F("  rail2,status        - Show comprehensive system status"));
        Console.println(F(""));
        Console.println(F("SAFETY NOTES:"));
        Console.println(F("- Always specify labware status for movement commands"));
        Console.println(F("- Home carriage before first use"));
        Console.println(F("- Homing automatically updates labware state and enables goto commands"));
        Console.println(F("- Check sensors before movement operations"));
        Console.println(F("- Ensure sufficient air pressure for pneumatic operations"));
        Console.println(F("- CRITICAL: Cylinder automatically retracts during ANY movement that"));
        Console.println(F("  involves collision zone (500-700mm), including"));
        Console.println(F("  crossing, entering, exiting, or moving within the zone to prevent Rail 1 collision"));
        Console.println(F("- Manual extension required after reaching safe positions"));
        Console.println(F("- Safe zones: 0-499mm and 701-1000mm (cylinder can remain extended)"));
        Console.println(F("============================================"));
        return true;

    case 5: // "init" - Initialize Rail 2 motor system
        return executeRailInit(2);

    case 10: // "retract" - Retract pneumatic drive
        if (!isPressureSufficient())
        {
            Console.error(F("INSUFFICIENT_PRESSURE: Air pressure too low for valve operation"));
            return false;
        }

        Console.serialInfo(F("Retracting pneumatic drive..."));
        result = retractCylinder();

        if (result == VALVE_OP_SUCCESS)
        {
            Console.acknowledge(F("CYLINDER_RETRACTED: Pneumatic drive is now retracted"));
            return true;
        }
        else
        {
            Console.error((String(F("RETRACT_FAILED: ")) + getValveOperationResultName(result)).c_str());
            return false;
        }

    case 4: // "home" - Home carriage
        return executeRailHome(2);

    case 7: // "move-mm-to" - Move to absolute millimeter position
        if (param1 == NULL || param2 == NULL)
        {
            Console.error(F("Missing parameters. Usage: rail2,move-mm-to,<position_mm>,<with-labware|no-labware>"));
            return false;
        }

        // Parse the position value
        targetPosition = atof(param1);

        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded))
            return false;

        // Execute the movement using helper function
        return executeRailMoveToPosition(2, targetPosition, carriageLoaded);

    case 8: // "move-rel" - Move relative distance
        if (param1 == NULL || param2 == NULL)
        {
            Console.error(F("Missing parameters. Usage: rail2,move-rel,<distance_mm>,<with-labware|no-labware>"));
            return false;
        }

        // Parse the relative distance value
        targetPosition = atof(param1);

        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded))
            return false;

        // Execute the movement using helper function
        return executeRailMoveRelative(2, targetPosition, carriageLoaded);

    case 9: // "move-wc3" - Move carriage to WC3
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded))
            return false;

        // Use Rail 2-specific helper function
        return moveRail2CarriageToWC3(carriageLoaded);

    case 6: // "move-handoff" - Move carriage to handoff
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded))
            return false;

        // Use Rail 2-specific helper function
        return moveRail2CarriageToHandoff(carriageLoaded);

    case 11: // "status" - Show system status
        Console.acknowledge(F("DISPLAYING_RAIL2_STATUS: Comprehensive system diagnostics follow:"));
        Console.serialInfo(F("============================================"));
        Console.serialInfo(F("Rail 2 System Status"));
        Console.serialInfo(F("============================================"));

        // Safety status
        Console.serialInfo(F("SAFETY STATUS:"));
        if (isEStopActive())
        {
            Console.serialInfo(F("  E-Stop Status: ACTIVE (UNSAFE)"));
        }
        else
        {
            Console.serialInfo(F("  E-Stop Status: INACTIVE (Safe)"));
        }

        // Motor status
        Console.serialInfo(F("MOTOR STATUS:"));
        Console.serialInfo(isMotorReady(2) ? F("  Motor Ready: YES") : F("  Motor Ready: NO"));
        Console.serialInfo(isHomingComplete(2) ? F("  Motor Homed: YES") : F("  Motor Homed: NO"));
        Console.serialInfo(isMotorMoving(2) ? F("  Motor Moving: YES") : F("  Motor Moving: NO"));
        Console.serialInfo(isHomingInProgress(2) ? F("  Motor Homing: YES") : F("  Motor Homing: NO"));

        // Current position
        Console.serialInfo(F("CURRENT POSITION:"));
        if (!isHomingComplete(2))
        {
            Console.serialInfo(F("  Position: UNKNOWN (not homed) - Use 'rail2,home' first"));
        }
        else
        {
            currentPos = getMotorPositionMm(2);
            Console.serialInfo((String(F("  Position: ")) + String(currentPos, 2) + F("mm")).c_str());

            if (isCarriageAtWC3())
            {
                Console.serialInfo(F("  Location: AT WC3"));
            }
            else if (isCarriageAtRail2Handoff())
            {
                Console.serialInfo(F("  Location: AT HANDOFF"));
            }
            else if (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END)
            {
                Console.serialInfo(F("  Location: IN COLLISION ZONE"));
            }
            else
            {
                Console.serialInfo(F("  Location: BETWEEN POSITIONS"));
            }
        }

        // Pneumatic system
        Console.serialInfo(F("PNEUMATIC SYSTEM:"));
        Console.serialInfo((String(F("  Air Pressure: ")) + String(getPressurePsi(), 1) + F(" PSI")).c_str());
        Console.serialInfo(isPressureSufficient() ? F("  Pressure Status: OK") : F("  Pressure Status: LOW"));
        valveState = getValvePosition();
        Console.serialInfo((String(F("  Valve Position: ")) + getValvePositionName(valveState)).c_str());

        // Cylinder sensors
        Console.serialInfo(F("CYLINDER SENSORS:"));
        sensorRetracted = isCylinderRetracted();
        sensorExtended = isCylinderExtended();
        Console.serialInfo(sensorRetracted ? F("  Retracted Sensor: ACTIVE") : F("  Retracted Sensor: INACTIVE"));
        Console.serialInfo(sensorExtended ? F("  Extended Sensor: ACTIVE") : F("  Extended Sensor: INACTIVE"));
        validationResult = validateValvePosition();
        Console.serialInfo(validationResult ? F("  Sensor Validation: PASS") : F("  Sensor Validation: FAIL"));

        // Position detection
        Console.serialInfo(F("POSITION DETECTION:"));
        Console.serialInfo(isCarriageAtWC3() ? F("  WC3 Detection: YES") : F("  WC3 Detection: NO"));
        Console.serialInfo(isCarriageAtRail2Handoff() ? F("  Handoff Detection: YES") : F("  Handoff Detection: NO"));

        // Labware detection
        Console.serialInfo(F("LABWARE DETECTION:"));
        Console.serialInfo(isLabwarePresentOnRail2() ? F("  Rail 2 Labware Present: YES") : F("  Rail 2 Labware Present: NO"));
        Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff Labware Present: YES") : F("  Handoff Labware Present: NO"));

        // Collision zone analysis
        Console.serialInfo(F("COLLISION ZONE ANALYSIS:"));
        if (isHomingComplete(2))
        {
            bool inCollisionZone = (currentPos >= RAIL2_COLLISION_ZONE_START && currentPos <= RAIL2_COLLISION_ZONE_END);
            actuallyRetracted = isCylinderActuallyRetracted();
            actuallyExtended = isCylinderActuallyExtended();

            if (inCollisionZone)
            {
                Console.serialInfo(F("  Current Zone: COLLISION"));
                if (actuallyRetracted)
                {
                    Console.serialInfo(F("  Collision Status: SAFE (cylinder retracted)"));
                }
                else
                {
                    Console.serialInfo(F("  Collision Status: UNSAFE (cylinder extended in collision zone)"));
                }
            }
            else
            {
                Console.serialInfo(F("  Current Zone: SAFE"));
                Console.serialInfo(F("  Collision Status: SAFE"));
            }
        }
        else
        {
            Console.serialInfo(F("  Current Zone: UNKNOWN (motor not homed)"));
            Console.serialInfo(F("  Collision Status: UNKNOWN"));
        }

        Console.serialInfo(F("============================================"));
        return true;

    case 12: // "stop" - Emergency stop motor movement
        return executeRailStop(2);

    default: // Unknown command
        Console.error(F("Unknown action. Available: init, clear-fault, abort, stop, extend, retract, home, move-wc3, move-handoff, move-mm-to, move-rel, status, and help"));
        return false;
    }

    return false; // Should never reach here
}

// ============================================================
// Rail 1 Control Commands
// ============================================================

// Define the rail1 subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo RAIL1_COMMANDS[] = {
    {"abort", 0},
    {"clear-fault", 1},
    {"help", 2},
    {"home", 3},
    {"init", 4},
    {"move-handoff", 5},
    {"move-mm-to", 6},
    {"move-rel", 7},
    {"move-staging", 8},
    {"move-wc1", 9},
    {"move-wc2", 10},
    {"status", 11},
    {"stop", 12}};

static const size_t RAIL1_COMMAND_COUNT = sizeof(RAIL1_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_rail1(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: rail1,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");
    char *param2 = strtok(nullptr, " ");

    if (action == NULL)
    {
        Console.error(F("Invalid format. Usage: rail1,<action>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Declare all variables at the beginning before switch
    double currentPos = 0.0;
    double targetPosition = 0.0;
    double calculatedTargetPos = 0.0;
    bool carriageLoaded = false;
    String motorStatus; // For status consolidation
    String location;    // For location description

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, RAIL1_COMMANDS, RAIL1_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {

    case 0: // "abort" - Abort current operation gracefully
        return executeRailAbort(1);

    case 1: // "clear-fault" - Clear motor fault condition
        return executeRailClearFault(1);

    case 2: // "help" - Display help information
        Console.acknowledge(F("DISPLAYING_RAIL1_HELP: Command reference follows:"));
        Console.println(F("============================================"));
        Console.println(F("Rail 1 Control Commands"));
        Console.println(F("============================================"));
        Console.println(F("MOTOR CONTROL:"));
        Console.println(F("  rail1,init          - Initialize Rail 1 motor system"));
        Console.println(F("  rail1,clear-fault   - Clear motor fault condition"));
        Console.println(F("  rail1,abort         - Abort current operation gracefully"));
        Console.println(F("  rail1,stop          - Emergency stop motor movement"));
        Console.println(F(""));
        Console.println(F("HOMING OPERATION:"));
        Console.println(F("  rail1,home          - Home carriage (find home position)"));
        Console.println(F("                        Automatically detects labware at handoff sensor"));
        Console.println(F(""));
        Console.println(F("CARRIAGE MOVEMENT:"));
        Console.println(F("  rail1,move-wc1,no-labware     - Move empty carriage to WC1"));
        Console.println(F("  rail1,move-wc2,no-labware     - Move empty carriage to WC2"));
        Console.println(F("  rail1,move-staging,no-labware - Move empty carriage to staging"));
        Console.println(F("  rail1,move-handoff,no-labware - Move empty carriage to handoff"));
        Console.println(F("  rail1,move-wc1,with-labware   - Move carriage with labware to WC1"));
        Console.println(F("  rail1,move-wc2,with-labware   - Move carriage with labware to WC2"));
        Console.println(F("  rail1,move-staging,with-labware - Move carriage with labware to staging"));
        Console.println(F("  rail1,move-handoff,with-labware - Move carriage with labware to handoff"));
        Console.println(F(""));
        Console.println(F("MANUAL POSITIONING:"));
        Console.print(F("  rail1,move-mm-to,X,no-labware   - Move empty carriage to absolute position X mm (0-"));
        Console.print(RAIL1_MAX_TRAVEL_MM);
        Console.println(F(")"));
        Console.println(F("  rail1,move-mm-to,X,with-labware - Move carriage with labware to absolute position X mm"));
        Console.println(F("  rail1,move-rel,X,no-labware     - Move empty carriage X mm relative (+ forward, - backward)"));
        Console.println(F("  rail1,move-rel,X,with-labware   - Move carriage with labware X mm relative"));
        Console.println(F(""));
        Console.println(F("STATUS AND DIAGNOSTICS:"));
        Console.println(F("  rail1,status        - Show comprehensive system status"));
        Console.println(F(""));
        Console.println(F("POSITION REFERENCE:"));
        Console.print(F("- Home: "));
        Console.print(RAIL1_HOME_POSITION);
        Console.println(F("mm (reference position)"));
        Console.print(F("- WC2: "));
        Console.print(RAIL1_WC2_PICKUP_DROPOFF);
        Console.println(F("mm (Workcell 2 pickup/dropoff)"));
        Console.print(F("- WC1: "));
        Console.print(RAIL1_WC1_PICKUP_DROPOFF);
        Console.println(F("mm (Workcell 1 pickup/dropoff)"));
        Console.print(F("- Staging: "));
        Console.print(RAIL1_STAGING_POSITION);
        Console.println(F("mm (coordination position for Rail 1-2 transfers)"));
        Console.print(F("- Handoff: "));
        Console.print(RAIL1_HANDOFF);
        Console.println(F("mm (transfer to Rail 2, same as home)"));
        Console.println(F(""));
        Console.println(F("SAFETY NOTES:"));
        Console.println(F("- Always specify labware status for movement commands"));
        Console.println(F("- Home carriage before first use"));
        Console.println(F("- Homing automatically updates labware state and enables goto commands"));
        Console.println(F("- Check sensors before movement operations"));
        Console.println(F("- Staging position is critical for coordinated Rail 1-2 operations"));
        Console.println(F("============================================"));
        return true;

    case 4: // "init" - Initialize Rail 1 motor system
        return executeRailInit(1);

    case 3: // "home" - Home carriage
        return executeRailHome(1);

    case 6: // "move-mm-to" - Move to absolute millimeter position
        if (param1 == NULL || param2 == NULL)
        {
            Console.error(F("Missing parameters. Usage: rail1,move-mm-to,<position_mm>,<with-labware|no-labware>"));
            return false;
        }

        // Parse the position value
        targetPosition = atof(param1);

        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded))
            return false;

        // Execute the movement using helper function
        return executeRailMoveToPosition(1, targetPosition, carriageLoaded);

    case 7: // "move-rel" - Move relative distance
        if (param1 == NULL || param2 == NULL)
        {
            Console.error(F("Missing parameters. Usage: rail1,move-rel,<distance_mm>,<with-labware|no-labware>"));
            return false;
        }

        // Parse the relative distance value
        targetPosition = atof(param1);

        // Use helper functions for common validation
        if (!parseAndValidateLabwareParameter(param2, carriageLoaded))
            return false;

        // Execute the movement using helper function
        return executeRailMoveRelative(1, targetPosition, carriageLoaded);

    case 9: // "move-wc1" - Move carriage to WC1
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded))
            return false;

        // Use Rail 1-specific helper function
        return moveRail1CarriageToWC1(carriageLoaded);

    case 10: // "move-wc2" - Move carriage to WC2
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded))
            return false;

        // Use Rail 1-specific helper function
        return moveRail1CarriageToWC2(carriageLoaded);

    case 8: // "move-staging" - Move carriage to staging
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded))
            return false;

        // Use Rail 1-specific helper function
        return moveRail1CarriageToStaging(carriageLoaded);

    case 5: // "move-handoff" - Move carriage to handoff
        // Parse and validate labware parameter
        if (!parseAndValidateLabwareParameter(param1, carriageLoaded))
            return false;

        // Use Rail 1-specific helper function
        return moveRail1CarriageToHandoff(carriageLoaded);

    case 11: // "status" - Show system status
        Console.acknowledge(F("DISPLAYING_RAIL1_STATUS: System diagnostics follow:"));
        Console.serialInfo(F("============================================"));
        Console.serialInfo(F("Rail 1 System Status"));
        Console.serialInfo(F("============================================"));

        // Safety status
        Console.serialInfo(F("SAFETY STATUS:"));
        if (isEStopActive())
        {
            Console.serialInfo(F("  E-Stop Status: ACTIVE (UNSAFE)"));
        }
        else
        {
            Console.serialInfo(F("  E-Stop Status: INACTIVE (Safe)"));
        }

        // Motor status
        Console.serialInfo(F("MOTOR STATUS:"));
        Console.serialInfo(isMotorReady(1) ? F("  Motor Ready: YES") : F("  Motor Ready: NO"));
        Console.serialInfo(isHomingComplete(1) ? F("  Motor Homed: YES") : F("  Motor Homed: NO"));
        Console.serialInfo(isMotorMoving(1) ? F("  Motor Moving: YES") : F("  Motor Moving: NO"));
        Console.serialInfo(isHomingInProgress(1) ? F("  Motor Homing: YES") : F("  Motor Homing: NO"));

        // Current position
        Console.serialInfo(F("CURRENT POSITION:"));
        if (!isHomingComplete(1))
        {
            Console.serialInfo(F("  Position: UNKNOWN (not homed) - Use 'rail1 home' first"));
        }
        else
        {
            currentPos = getMotorPositionMm(1);
            Console.serialInfo((String(F("  Position: ")) + String(currentPos, 2) + F("mm")).c_str());

            if (abs(currentPos - RAIL1_HOME_POSITION) < 50.0)
            {
                Console.serialInfo(F("  Location: AT HOME/HANDOFF"));
            }
            else if (abs(currentPos - RAIL1_WC2_PICKUP_DROPOFF) < 50.0)
            {
                Console.serialInfo(F("  Location: AT WC2"));
            }
            else if (abs(currentPos - RAIL1_WC1_PICKUP_DROPOFF) < 50.0)
            {
                Console.serialInfo(F("  Location: AT WC1"));
            }
            else if (abs(currentPos - RAIL1_STAGING_POSITION) < 50.0)
            {
                Console.serialInfo(F("  Location: AT STAGING"));
            }
            else
            {
                Console.serialInfo(F("  Location: BETWEEN POSITIONS"));
            }
        }

        // Position detection
        Console.serialInfo(F("POSITION DETECTION:"));
        Console.serialInfo(isCarriageAtWC1() ? F("  WC1 Detection: YES") : F("  WC1 Detection: NO"));
        Console.serialInfo(isCarriageAtWC2() ? F("  WC2 Detection: YES") : F("  WC2 Detection: NO"));
        Console.serialInfo(isCarriageAtRail1Handoff() ? F("  Handoff Detection: YES") : F("  Handoff Detection: NO"));

        // Labware detection
        Console.serialInfo(F("LABWARE DETECTION:"));
        Console.serialInfo(isLabwarePresentAtWC1() ? F("  WC1 Labware Present: YES") : F("  WC1 Labware Present: NO"));
        Console.serialInfo(isLabwarePresentAtWC2() ? F("  WC2 Labware Present: YES") : F("  WC2 Labware Present: NO"));
        Console.serialInfo(isLabwarePresentAtHandoff() ? F("  Handoff Labware Present: YES") : F("  Handoff Labware Present: NO"));

        Console.serialInfo(F("============================================"));
        return true;

    case 12: // "stop" - Emergency stop motor movement
        return executeRailStop(1);

    default: // Unknown command
        Console.error(F("Unknown action. Available: init, clear-fault, abort, stop, home, move-wc1, move-wc2, move-staging, move-handoff, move-mm-to, move-rel, status, and help"));
        return false;
    }

    return false; // Should never reach here
}

// ============================================================
// Labware Automation Command Implementation
// ============================================================

// Define the labware subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo LABWARE_COMMANDS[] = {
    {"audit", 0},
    {"help", 1},
    {"reset", 2},
    {"status", 3}};

static const size_t LABWARE_COMMAND_COUNT = sizeof(LABWARE_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_labware(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: labware,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");

    if (action == NULL)
    {
        Console.error(F("Invalid format. Usage: labware,<action>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, LABWARE_COMMANDS, LABWARE_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {

    case 0: // "audit" - Automatically validate and fix labware state
        Console.acknowledge(F("LABWARE_AUDIT_INITIATED: Analyzing system state and validating labware positions"));

        if (performLabwareAudit())
        {
            Console.acknowledge(F("AUDIT_COMPLETE: System ready for automation commands"));
        }
        else
        {
            Console.error(F("AUDIT_FAILED: Unable to validate labware state"));
        }
        return true;

    case 1: // "help" - Display help information
        Console.acknowledge(F("DISPLAYING_LABWARE_HELP: Automation system guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("Labware Automation Commands"));
        Console.println(F("============================================"));
        Console.println(F("STATE MANAGEMENT:"));
        Console.println(F("  labware status      - Display current labware tracking state"));
        Console.println(F("                        Shows rail states, sensor readings, confidence levels"));
        Console.println(F("                        Includes operation counters and time since last work"));
        Console.println(F(""));
        Console.println(F("RECOVERY OPERATIONS:"));
        Console.println(F("  labware audit       - Automatically validate and fix labware state"));
        Console.println(F("                        Moves to nearest sensor, reads actual state"));
        Console.println(F("                        Updates tracking based on ground truth"));
        Console.println(F("  labware reset       - Clear all labware tracking (nuclear option)"));
        Console.println(F("                        Wipes all state, requires manual re-establishment"));
        Console.println(F("                        Resets operation counters and timestamps"));
        Console.println(F(""));
        Console.println(F("SYSTEM ARCHITECTURE:"));
        Console.println(F("- Rail 1: Checkpoint-based tracking (sensors at WC1, WC2, handoff)"));
        Console.println(F("- Rail 2: Continuous tracking (carriage-mounted sensor)"));
        Console.println(F("- Confidence levels: HIGH (real-time), MEDIUM (recent sensor), LOW (inferred)"));
        Console.println(F(""));
        Console.println(F("GOTO COMMAND CONTROL:"));
        Console.println(F("- Rail homing AUTOMATICALLY enables goto commands when possible"));
        Console.println(F("- Each rail homing updates labware state from sensors"));
        Console.println(F("- 'labware audit' provides comprehensive validation if needed"));
        Console.println(F("- Goto commands are DISABLED when system has dual labware conflicts"));
        Console.println(F("- 'labware status' shows current goto command availability"));
        Console.println(F(""));
        Console.println(F("USAGE SCENARIOS:"));
        Console.println(F("- System startup: Home both rails to enable goto commands automatically"));
        Console.println(F("- After motor faults: Home affected rail to update labware state"));
        Console.println(F("- Complex issues: Use 'labware audit' for comprehensive validation"));
        Console.println(F("- System confusion: Use 'labware reset' to start fresh"));
        Console.println(F("- Regular monitoring: Use 'labware status' to check state"));
        Console.println(F("- Goto disabled: Check status, resolve conflicts, re-home or run audit"));
        Console.println(F(""));
        Console.println(F("SAFETY FEATURES:"));
        Console.println(F("- Automatic enablement: Homing both rails enables goto commands"));
        Console.println(F("- Conservative movement (with-labware speeds during audit)"));
        Console.println(F("- Sensor validation (ground truth confirmation)"));
        Console.println(F("- Collision avoidance (audit only moves to WC1/WC2)"));
        Console.println(F("- Manual override: Manual audit available for complex situations"));
        Console.println(F("============================================"));
        return true;

    case 2: // "reset" - Clear all labware tracking (nuclear option)
        Console.acknowledge(F("NUCLEAR_RESET_INITIATED: Clearing all labware tracking state"));
        clearLabwareState();
        Console.acknowledge(F("RESET_COMPLETE: Use 'labware audit' to establish current state"));
        return true;

    case 3: // "status" - Display current labware tracking state
        Console.acknowledge(F("DISPLAYING_LABWARE_STATUS: Current tracking state follows:"));
        printLabwareSystemStatus();
        return true;

    default: // Unknown command
        Console.error(F("Unknown labware command. Available: status, audit, reset, help"));
        return false;
    }

    return false; // Should never reach here
}

//=============================================================================
// AUTOMATED LABWARE MOVEMENT COMMAND IMPLEMENTATION
//=============================================================================

// Goto command lookup table for binary search
const SubcommandInfo GOTO_ACTIONS[] = {
    {"help", 2},        // Display goto command help
    {"no-labware", 0},  // Move to location without labware
    {"with-labware", 1} // Move to location with labware
};

const size_t GOTO_ACTION_COUNT = sizeof(GOTO_ACTIONS) / sizeof(GOTO_ACTIONS[0]);

// Location lookup table for goto command
const SubcommandInfo GOTO_LOCATIONS[] = {
    {"wc1", 0}, // Work Cell 1
    {"wc2", 1}, // Work Cell 2
    {"wc3", 2}  // Work Cell 3
};

const size_t GOTO_LOCATION_COUNT = sizeof(GOTO_LOCATIONS) / sizeof(GOTO_LOCATIONS[0]);

bool cmd_goto(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameters. Usage: goto,<location>,<status>"));
        Console.error(F("Example: goto,wc1,with-labware"));
        Console.error(F("Help: goto,help"));
        return false;
    }

    // Parse location and action arguments
    char *location = strtok(trimmed, " ");
    char *action = strtok(NULL, " ");

    // Special case: handle "goto help" as a single command
    if (location != NULL && strcmp(trimLeadingSpaces(location), "help") == 0)
    {
        // Display comprehensive goto help
        Console.acknowledge(F("DISPLAYING_GOTO_HELP: Automated movement command reference follows:"));
        Console.println(F("============================================"));
        Console.println(F("Goto Command - Automated Labware Movement"));
        Console.println(F("============================================"));
        Console.println(F("COMMAND SYNTAX:"));
        Console.println(F("  goto,<location>,<status>"));
        Console.println(F(""));
        Console.println(F("AVAILABLE LOCATIONS:"));
        Console.println(F("  wc1    - Work Cell 1 (Rail 1)"));
        Console.println(F("  wc2    - Work Cell 2 (Rail 1)"));
        Console.println(F("  wc3    - Work Cell 3 (Rail 2)"));
        Console.println(F(""));
        Console.println(F("LABWARE STATUS:"));
        Console.println(F("  with-labware    - Deliver labware to destination"));
        Console.println(F("  no-labware      - Pickup labware from destination"));
        Console.println(F(""));
        Console.println(F("OPERATION EXAMPLES:"));
        Console.println(F("  goto,wc1,with-labware   - Deliver labware to WC1"));
        Console.println(F("  goto,wc1,no-labware     - Pickup labware from WC1"));
        Console.println(F("  goto,wc2,with-labware   - Deliver labware to WC2"));
        Console.println(F("  goto,wc2,no-labware     - Pickup labware from WC2"));
        Console.println(F("  goto,wc3,with-labware   - Deliver labware to WC3"));
        Console.println(F("  goto,wc3,no-labware     - Pickup labware from WC3"));
        Console.println(F(""));
        Console.println(F("INTELLIGENT FEATURES:"));
        Console.println(F("- Automatic cross-rail transfers (WC1/WC2 ↔ WC3)"));
        Console.println(F("- Comprehensive preflight safety validation"));
        Console.println(F("- Collision zone management for Rail 2"));
        Console.println(F("- Labware state consistency checking"));
        Console.println(F("- Destination occupancy validation"));
        Console.println(F(""));
        Console.println(F("AUTOMATION CONTROL:"));
        Console.println(F("- Goto commands automatically enabled after rail homing"));
        Console.println(F("- Homing reads sensors and updates labware state automatically"));
        Console.println(F("- Use 'labware audit' for comprehensive validation if needed"));
        Console.println(F("- Goto commands disabled if dual labware conflicts exist"));
        Console.println(F("- Check 'labware status' to see current automation state"));
        Console.println(F(""));
        Console.println(F("PREFLIGHT VALIDATION:"));
        Console.println(F("- Automation system enabled (automatically enabled after homing both rails)"));
        Console.println(F("- Emergency stop status"));
        Console.println(F("- Rail homing completion"));
        Console.println(F("- System readiness (motors, sensors)"));
        Console.println(F("- Pneumatic pressure sufficiency"));
        Console.println(F("- Labware state consistency"));
        Console.println(F("- Destination availability"));
        Console.println(F(""));
        Console.println(F("ERROR HANDLING:"));
        Console.println(F("- Clear error messages with specific solutions"));
        Console.println(F("- Alternative manual commands suggested"));
        Console.println(F("- System recovery guidance"));
        Console.println(F(""));
        Console.println(F("SAFETY NOTES:"));
        Console.println(F("- Both Rails 1 & 2 must be homed before use"));
        Console.println(F("- Sufficient air pressure required for operations"));
        Console.println(F("- Delivery blocked if destination already has labware"));
        Console.println(F("- Pickup blocked if destination has no labware"));
        Console.println(F("- Cross-rail transfers handled automatically"));
        Console.println(F(""));
        Console.println(F("RELATED COMMANDS:"));
        Console.println(F("  labware status  - Check current labware state"));
        Console.println(F("  labware audit   - Validate and fix labware tracking"));
        Console.println(F("  rail1 status    - Check Rail 1 system status"));
        Console.println(F("  rail2 status    - Check Rail 2 system status"));
        Console.println(F("============================================"));
        return true;
    }

    if (location == NULL || action == NULL)
    {
        Console.error(F("Invalid format. Usage: goto,<location>,<status>"));
        Console.error(F("Locations: wc1, wc2, wc3"));
        Console.error(F("Status: with-labware, no-labware"));
        Console.error(F("Help: goto help"));
        return false;
    }

    // Trim and convert to lowercase
    location = trimLeadingSpaces(location);
    action = trimLeadingSpaces(action);

    for (int i = 0; location[i]; i++)
    {
        location[i] = tolower(location[i]);
    }
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Use binary search to find location and action codes
    int locationCode = findSubcommandCode(location, GOTO_LOCATIONS, GOTO_LOCATION_COUNT);
    int actionCode = findSubcommandCode(action, GOTO_ACTIONS, GOTO_ACTION_COUNT);

    if (locationCode == -1)
    {
        Console.error((String(F("Unknown location: ")) + String(location)).c_str());
        Console.error(F("Available locations: wc1, wc2, wc3"));
        return false;
    }

    if (actionCode == -1)
    {
        Console.error((String(F("Unknown action: ")) + String(action)).c_str());
        Console.error(F("Available actions: with-labware, no-labware"));
        return false;
    }

    // Convert codes to enums for processing
    Location targetLocation;
    bool hasLabware;

    switch (locationCode)
    {
    case 0:
        targetLocation = LOCATION_WC1;
        break;
    case 1:
        targetLocation = LOCATION_WC2;
        break;
    case 2:
        targetLocation = LOCATION_WC3;
        break;
    default:
        targetLocation = LOCATION_UNKNOWN;
        break;
    }

    switch (actionCode)
    {
    case 0:
        hasLabware = false;
        break; // no-labware
    case 1:
        hasLabware = true;
        break; // with-labware
    default:
        hasLabware = false;
        break;
    }

    // Pre-flight checks before attempting automated movement
    if (!performGotoPreflightChecks(targetLocation, hasLabware))
    {
        return false;
    }

    Console.acknowledge((String(F("GOTO_INITIATED: Moving to ")) + getLocationName(targetLocation) +
                         String(F(" ")) + (hasLabware ? F("with-labware") : F("no-labware")))
                            .c_str());

    // Execute the automated movement based on location and labware status
    switch (locationCode)
    {

    case 0: // "wc1" - Work Cell 1
        Console.serialInfo(hasLabware ? F("WC1_WITH_LABWARE: Moving to WC1 with labware") : F("WC1_NO_LABWARE: Moving to WC1 without labware"));
        return moveRail1CarriageToWC1(hasLabware);

    case 1: // "wc2" - Work Cell 2
        Console.serialInfo(hasLabware ? F("WC2_WITH_LABWARE: Moving to WC2 with labware") : F("WC2_NO_LABWARE: Moving to WC2 without labware"));
        return moveRail1CarriageToWC2(hasLabware);

    case 2: // "wc3" - Work Cell 3
        Console.serialInfo(hasLabware ? F("WC3_WITH_LABWARE: Moving to WC3 with labware") : F("WC3_NO_LABWARE: Moving to WC3 without labware"));
        return moveRail2CarriageToWC3(hasLabware);

    default: // Unknown location (should not reach here due to earlier validation)
        Console.error(F("Internal error: Invalid location code"));
        return false;
    }

    return false; // Should never reach here
}

//=============================================================================
// NETWORK MANAGEMENT COMMAND IMPLEMENTATION
//=============================================================================

// Define the network subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo NETWORK_COMMANDS[] = {
    {"disconnect", 0},
    {"help", 1},
    {"status", 2}};

static const size_t NETWORK_COMMAND_COUNT = sizeof(NETWORK_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_network(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: network,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");

    if (action == NULL)
    {
        Console.error(F("Invalid format. Usage: network,<action>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, NETWORK_COMMANDS, NETWORK_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {

    case 0: // "disconnect" - Disconnect the current client
        Console.acknowledge(F("NETWORK_DISCONNECT_INITIATED: Closing current client connection"));

        if (closeAllConnections())
        {
            Console.acknowledge(F("CLIENT_DISCONNECTED: Network connection closed"));
            return true;
        }
        else
        {
            Console.error(F("DISCONNECT_FAILED: No active connections to close"));
            return false;
        }

    case 1: // "help" - Display network management help
        Console.acknowledge(F("DISPLAYING_NETWORK_HELP: Network management guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("Network Management Commands"));
        Console.println(F("============================================"));
        Console.println(F("CONNECTION STATUS:"));
        Console.println(F("  network status     - Display current network status and client info"));
        Console.println(F("                       Shows IP configuration, client details, activity"));
        Console.println(F(""));
        Console.println(F("CONNECTION CONTROL:"));
        Console.println(F("  network disconnect - Disconnect the current client"));
        Console.println(F("                       Gracefully closes active connection"));
        Console.println(F(""));
        Console.println(F("SYSTEM CONFIGURATION:"));
        Console.println(F("- Single client design: Only one connection allowed at a time"));
        Console.println(F("- Auto-timeout: Inactive clients disconnected after 3 minutes"));
        Console.println(F("- Connection testing: Periodic health checks every 2 minutes"));
        Console.println(F("- Port: 8888 (configurable in EthernetController.h)"));
        Console.println(F(""));
        Console.println(F("NETWORK INFORMATION:"));
        Console.println(F("- Physical link status and cable detection"));
        Console.println(F("- IP address assignment (DHCP or static fallback)"));
        Console.println(F("- Client IP address and port information"));
        Console.println(F("- Last activity timestamps for connection monitoring"));
        Console.println(F(""));
        Console.println(F("USAGE SCENARIOS:"));
        Console.println(F("- Check connectivity: Use 'network status' to verify connection"));
        Console.println(F("- Force disconnect: Use 'network disconnect' to reset connection"));
        Console.println(F("- Troubleshooting: Status shows physical link and client activity"));
        Console.println(F("- System monitoring: Regular status checks for network health"));
        Console.println(F(""));
        Console.println(F("SAFETY FEATURES:"));
        Console.println(F("- Automatic timeout prevents stale connections"));
        Console.println(F("- Connection health monitoring detects network issues"));
        Console.println(F("- Graceful disconnect preserves system stability"));
        Console.println(F("- Single-client design eliminates command conflicts"));
        Console.println(F("============================================"));
        return true;

    case 2: // "status" - Display current network status
        Console.acknowledge(F("NETWORK_STATUS_REQUESTED: Current network diagnostics follow:"));
        printEthernetStatus();
        return true;

    default: // Unknown command
        Console.error(F("Unknown network command. Available: status, disconnect, help"));
        return false;
    }

    return false; // Should never reach here
}

//=============================================================================
// ENCODER COMMAND IMPLEMENTATION
//=============================================================================

// Define the encoder subcommands lookup table (MUST BE SORTED ALPHABETICALLY)
static const SubcommandInfo ENCODER_COMMANDS[] = {
    {"disable", 0},
    {"enable", 1},
    {"help", 2},
    {"multiplier", 3},
    {"status", 4},
    {"velocity", 5}};

static const size_t ENCODER_COMMAND_COUNT = sizeof(ENCODER_COMMANDS) / sizeof(SubcommandInfo);

bool cmd_encoder(char *args, CommandCaller *caller)
{
    // Create a local copy of arguments
    char localArgs[COMMAND_BUFFER_SIZE];
    strncpy(localArgs, args, COMMAND_BUFFER_SIZE);
    localArgs[COMMAND_BUFFER_SIZE - 1] = '\0';

    // Skip leading spaces
    char *trimmed = trimLeadingSpaces(localArgs);

    // Check for empty argument
    if (strlen(trimmed) == 0)
    {
        Console.error(F("Missing parameter. Usage: encoder,<action>"));
        return false;
    }

    // Parse the argument - use spaces as separators
    char *action = strtok(trimmed, " ");
    char *param1 = strtok(nullptr, " ");
    char *param2 = strtok(nullptr, " ");

    if (action == NULL)
    {
        Console.error(F("Invalid format. Usage: encoder,<action>"));
        return false;
    }

    // Trim leading spaces from action
    action = trimLeadingSpaces(action);

    // Convert action to lowercase for case-insensitive comparison
    for (int i = 0; action[i]; i++)
    {
        action[i] = tolower(action[i]);
    }

    // Declare variables at the beginning
    int railNumber = 0;
    float multiplierValue = 0.0;
    int velocityValue = 0;

    // Use binary search to find the command code
    int cmdCode = findSubcommandCode(action, ENCODER_COMMANDS, ENCODER_COMMAND_COUNT);

    // Use switch-case for cleaner flow control
    switch (cmdCode)
    {

    case 0: // "disable" - Disable encoder control
        Console.acknowledge(F("ENCODER_DISABLE_INITIATED: Disabling MPG control"));
        disableEncoderControl();
        return true;

    case 1: // "enable" - Enable encoder control for specific rail
        if (param1 == NULL)
        {
            Console.error(F("Missing rail parameter. Usage: encoder,enable,<rail>"));
            Console.error(F("Example: encoder,enable,1  (for Rail 1)"));
            Console.error(F("Example: encoder,enable,2  (for Rail 2)"));
            return false;
        }

        railNumber = atoi(param1);

        if (railNumber != 1 && railNumber != 2)
        {
            Console.error(F("Invalid rail number. Use 1 or 2"));
            Console.error(F("Example: encoder,enable,1  (for Rail 1)"));
            Console.error(F("Example: encoder,enable,2  (for Rail 2)"));
            return false;
        }

        Console.acknowledge((String(F("ENCODER_ENABLE_INITIATED: Enabling MPG control for Rail ")) + String(railNumber)).c_str());
        enableEncoderControl(railNumber);
        return true;

    case 2: // "help" - Display encoder help
        Console.acknowledge(F("DISPLAYING_ENCODER_HELP: MPG control guide follows:"));
        Console.println(F("============================================"));
        Console.println(F("Manual Pulse Generator (MPG) Commands"));
        Console.println(F("============================================"));
        Console.println(F("CONTROL OPERATIONS:"));
        Console.println(F("  encoder,enable,<rail>    - Enable MPG control for specific rail"));
        Console.println(F("                             Rail 1: Controls WC1, WC2, staging, handoff"));
        Console.println(F("                             Rail 2: Controls WC3, handoff (with collision avoidance)"));
        Console.println(F("  encoder,disable          - Disable MPG control completely"));
        Console.println(F(""));
        Console.println(F("CONFIGURATION:"));
        Console.println(F("  encoder,multiplier,<X>   - Set movement precision per encoder count"));
        Console.println(F("                             1 = Fine (0.1mm per count)"));
        Console.println(F("                             10 = General (1.0mm per count)"));
        Console.println(F("                             100 = Rapid (10.0mm per count)"));
        Console.println(F("  encoder,velocity,<RPM>   - Set movement velocity (50-400 RPM)"));
        Console.println(F(""));
        Console.println(F("STATUS AND DIAGNOSTICS:"));
        Console.println(F("  encoder,status           - Display current MPG status and settings"));
        Console.println(F("                             Shows active rail, position, multiplier, velocity"));
        Console.println(F("                             Includes encoder hardware status"));
        Console.println(F(""));
        Console.println(F("SYSTEM ARCHITECTURE:"));
        Console.println(F("- Single hardware encoder controls one rail at a time"));
        Console.println(F("- Automatic switching: enabling new rail disables previous"));
        Console.println(F("- Global settings: multiplier and velocity apply to active rail"));
        Console.println(F("- Position tracking: absolute positioning for immediate response"));
        Console.println(F(""));
        Console.println(F("USAGE EXAMPLES:"));
        Console.println(F("  encoder,enable,1         - Enable MPG for Rail 1"));
        Console.println(F("  encoder,multiplier,10    - Set general precision"));
        Console.println(F("  encoder,velocity,150     - Set velocity to 150 RPM"));
        Console.println(F("  encoder,enable,2         - Switch MPG to Rail 2"));
        Console.println(F("  encoder,status           - Check current settings"));
        Console.println(F("  encoder,disable          - Stop MPG control"));
        Console.println(F(""));
        Console.println(F("SAFETY REQUIREMENTS:"));
        Console.println(F("- Rail must be homed before enabling MPG control"));
        Console.println(F("- Rail must be ready (not faulted or moving)"));
        Console.println(F("- MPG automatically disabled if motor faults or moves"));
        Console.println(F("- Travel limits enforced (cannot exceed rail boundaries)"));
        Console.println(F("- Quadrature error detection and recovery"));
        Console.println(F(""));
        Console.println(F("OPERATIONAL TIPS:"));
        Console.println(F("- Start with general multiplier (10) for everyday use"));
        Console.println(F("- Use fine multiplier (1) for precise positioning"));
        Console.println(F("- Use rapid multiplier (100) for quick movement"));
        Console.println(F("- Higher velocity = faster response to encoder input"));
        Console.println(F("- Check 'encoder status' to verify active rail and settings"));
        Console.println(F("============================================"));
        return true;

    case 3: // "multiplier" - Set encoder multiplier
        if (param1 == NULL)
        {
            Console.error(F("Missing multiplier value. Usage: encoder,multiplier,<value>"));
            Console.error(F("Valid values: 1 (fine), 10 (general), 100 (rapid)"));
            return false;
        }

        multiplierValue = atof(param1);

        Console.acknowledge((String(F("ENCODER_MULTIPLIER_UPDATE: Setting multiplier to ")) + String(multiplierValue)).c_str());
        setEncoderMultiplier(multiplierValue);
        return true;

    case 4: // "status" - Display encoder status
        Console.acknowledge(F("ENCODER_STATUS_REQUESTED: Current MPG diagnostics follow:"));
        printEncoderStatus();
        return true;

    case 5: // "velocity" - Set encoder velocity
        if (param1 == NULL)
        {
            Console.error(F("Missing velocity value. Usage: encoder,velocity,<RPM>"));
            Console.error(F("Valid range: 50-400 RPM"));
            return false;
        }

        velocityValue = atoi(param1);

        Console.acknowledge((String(F("ENCODER_VELOCITY_UPDATE: Setting velocity to ")) + String(velocityValue) + F(" RPM")).c_str());
        setEncoderVelocity(velocityValue);
        return true;

    default: // Unknown command
        Console.error(F("Unknown encoder command. Available: enable, disable, multiplier, velocity, status, help"));
        return false;
    }

    return false; // Should never reach here
}

//=============================================================================
// JOG COMMAND IMPLEMENTATION
//=============================================================================

bool cmd_jog(char *args, CommandCaller *caller)
{
    // Parse first argument
    char *param1 = strtok(args, " ");
    if (param1 == NULL)
    {
        Console.error(F("Missing jog command. Usage: jog,<rail>,<+|-> [distance] | jog,<rail>,<setting>,<value> | jog,<rail|all>,status | jog,help"));
        return false;
    }

    // Handle global commands
    if (strcmp(param1, "help") == 0)
    {
        Console.println(F("============================================"));
        Console.println(F("             JOG COMMAND HELP"));
        Console.println(F("============================================"));
        Console.println(F(""));
        Console.println(F("DESCRIPTION:"));
        Console.println(F("Manual jogging control for dual-rail overhead system."));
        Console.println(F("Provides precision movement with configurable increments and speeds."));
        Console.println(F(""));
        Console.println(F("BASIC JOGGING:"));
        Console.println(F("  jog,<rail>,+             - Jog rail forward by default increment"));
        Console.println(F("  jog,<rail>,-             - Jog rail backward by default increment"));
        Console.println(F(""));
        Console.println(F("CUSTOM DISTANCE JOGGING:"));
        Console.println(F("  jog,<rail>,+,<mm>        - Jog forward by specific distance"));
        Console.println(F("  jog,<rail>,-,<mm>        - Jog backward by specific distance"));
        Console.println(F(""));
        Console.println(F("CONFIGURATION:"));
        Console.println(F("  jog,<rail>,increment,<mm> - Set default jog increment"));
        Console.println(F("  jog,<rail>,speed,<rpm>    - Set jog speed"));
        Console.println(F(""));
        Console.println(F("STATUS AND INFORMATION:"));
        Console.println(F("  jog,<rail>,status        - Show jog settings for specific rail"));
        Console.println(F("  jog,all,status           - Show jog settings for all rails"));
        Console.println(F("  jog,status               - Show jog settings for all rails"));
        Console.println(F(""));
        Console.println(F("RAIL SPECIFICATION:"));
        Console.println(F("- Rail 1: Process rail (typically higher speeds)"));
        Console.println(F("- Rail 2: WC3 rail (typically lower speeds)"));
        Console.println(F(""));
        Console.println(F("DEFAULTS:"));
        Console.println(F("- Rail 1: 1.0mm increment, 200 RPM"));
        Console.println(F("- Rail 2: 0.5mm increment, 150 RPM"));
        Console.println(F(""));
        Console.println(F("SAFETY FEATURES:"));
        Console.println(F("- Travel limits enforced (cannot exceed rail boundaries)"));
        Console.println(F("- Motor ready state verification"));
        Console.println(F("- Movement conflict detection"));
        Console.println(F("- Intelligent speed capping based on distance"));
        Console.println(F(""));
        Console.println(F("USAGE EXAMPLES:"));
        Console.println(F("  jog,1,+                  - Rail 1 forward by default"));
        Console.println(F("  jog,2,-,5.0              - Rail 2 backward 5.0mm"));
        Console.println(F("  jog,1,increment,2.0      - Set Rail 1 increment to 2.0mm"));
        Console.println(F("  jog,2,speed,100          - Set Rail 2 speed to 100 RPM"));
        Console.println(F("============================================"));
        return true;
    }

    if (strcmp(param1, "status") == 0 || strcmp(param1, "all") == 0)
    {
        Console.println(F("============================================"));
        Console.println(F("           JOG STATUS - ALL RAILS"));
        Console.println(F("============================================"));

        for (int rail = 1; rail <= 2; rail++)
        {
            Console.println((String(F("RAIL ")) + String(rail) + F(":")).c_str());
            Console.println((String(F("  Increment: ")) + String(getJogIncrement(rail), 2) + F(" mm")).c_str());
            Console.println((String(F("  Speed: ")) + String(getJogSpeed(rail)) + F(" RPM")).c_str());
            Console.println((String(F("  Ready: ")) + (isMotorReady(rail) ? F("YES") : F("NO"))).c_str());
            Console.println((String(F("  Moving: ")) + (isMotorMoving(rail) ? F("YES") : F("NO"))).c_str());
            if (rail == 1)
                Console.println(F(""));
        }

        Console.println(F("============================================"));
        return true;
    }

    // Parse rail number
    int rail = atoi(param1);
    if (rail < 1 || rail > 2)
    {
        Console.error(F("Invalid rail number. Use 1 or 2"));
        return false;
    }

    // Parse second argument (command/direction)
    char *param2 = strtok(NULL, " ");
    if (param2 == NULL)
    {
        Console.error(F("Missing command. Usage: jog,<rail>,<+|-> [value]"));
        return false;
    }

    // Handle rail-specific status
    if (strcmp(param2, "status") == 0)
    {
        Console.println((String(F("============ RAIL ")) + String(rail) + F(" JOG STATUS ============")).c_str());
        Console.println((String(F("Increment: ")) + String(getJogIncrement(rail), 2) + F(" mm")).c_str());
        Console.println((String(F("Speed: ")) + String(getJogSpeed(rail)) + F(" RPM")).c_str());
        Console.println((String(F("Motor Ready: ")) + (isMotorReady(rail) ? F("YES") : F("NO"))).c_str());
        Console.println((String(F("Motor Moving: ")) + (isMotorMoving(rail) ? F("YES") : F("NO"))).c_str());
        Console.println(F("============================================"));
        return true;
    }

    // Handle jog settings
    if (strcmp(param2, "increment") == 0)
    {
        char *param3 = strtok(NULL, " ");
        if (param3 == NULL)
        {
            Console.error(F("Missing increment value. Usage: jog,<rail>,increment,<mm>"));
            return false;
        }

        double increment = atof(param3);
        if (increment <= 0 || increment > 100)
        {
            Console.error(F("Invalid increment. Must be between 0.01 and 100.0 mm"));
            return false;
        }

        if (setJogIncrement(rail, increment))
        {
            Console.acknowledge((String(F("JOG_INCREMENT_UPDATE: Rail ")) + String(rail) +
                                 F(" increment set to ") + String(increment, 2) + F(" mm"))
                                    .c_str());
            return true;
        }
        else
        {
            Console.error(F("Failed to set jog increment"));
            return false;
        }
    }

    if (strcmp(param2, "speed") == 0)
    {
        char *param3 = strtok(NULL, " ");
        if (param3 == NULL)
        {
            Console.error(F("Missing speed value. Usage: jog,<rail>,speed,<rpm>"));
            return false;
        }

        int speed = atoi(param3);
        if (speed < 10 || speed > 1000)
        {
            Console.error(F("Invalid speed. Must be between 10 and 1000 RPM"));
            return false;
        }

        if (setJogSpeed(rail, speed))
        {
            Console.acknowledge((String(F("JOG_SPEED_UPDATE: Rail ")) + String(rail) +
                                 F(" speed set to ") + String(speed) + F(" RPM"))
                                    .c_str());
            return true;
        }
        else
        {
            Console.error(F("Failed to set jog speed"));
            return false;
        }
    }

    // Handle jog directions
    bool isForward = false;
    bool isValidDirection = false;

    if (strcmp(param2, "+") == 0)
    {
        isForward = true;
        isValidDirection = true;
    }
    else if (strcmp(param2, "-") == 0)
    {
        isForward = false;
        isValidDirection = true;
    }

    if (!isValidDirection)
    {
        Console.error(F("Invalid direction. Use: + (forward) or - (backward)"));
        return false;
    }

    // Check for custom distance
    char *param3 = strtok(NULL, " ");
    double customDistance = 0;
    bool useCustomDistance = false;

    if (param3 != NULL)
    {
        customDistance = atof(param3);
        if (customDistance <= 0 || customDistance > 200)
        {
            Console.error(F("Invalid distance. Must be between 0.01 and 200.0 mm"));
            return false;
        }
        useCustomDistance = true;
    }

    // Perform jog movement
    String jogMsg = String(F("JOG_EXECUTE: Rail ")) + String(rail) +
                    F(" ") + (isForward ? F("forward") : F("backward"));
    if (useCustomDistance)
    {
        jogMsg += String(F(" ")) + String(customDistance, 2) + String(F("mm"));
    }
    else
    {
        jogMsg += String(F(" (default)"));
    }
    Console.acknowledge(jogMsg.c_str());

    bool success;
    if (useCustomDistance)
    {
        success = jogMotor(rail, isForward, customDistance);
    }
    else
    {
        success = jogMotor(rail, isForward);
    }

    if (!success)
    {
        Console.error(F("Jog operation failed. Check motor status and try again"));
        return false;
    }

    return true;
}
