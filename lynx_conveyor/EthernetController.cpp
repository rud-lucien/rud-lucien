#include "EthernetController.h"

// Global variables
EthernetServer server(ETHERNET_PORT);
EthernetClient clients[MAX_ETHERNET_CLIENTS];
bool ethernetInitialized = false;
char ethernetCommandBuffer[MAX_PACKET_LENGTH];
unsigned long clientLastActivityTime[MAX_ETHERNET_CLIENTS] = {0};

// Timeout constants (all in milliseconds)
const unsigned long CLIENT_TIMEOUT_MS = 180000;           // 3 minute inactivity timeout
const unsigned long PING_TEST_INTERVAL_MS = 120000;       // 2 minute between ping tests
const unsigned long PING_GRACE_PERIOD_MS = 15000;         // 15 seconds grace before pinging new clients
const unsigned long TEST_CONNECTIONS_INTERVAL_MS = 30000; // 30 seconds for testConnections()

// MAC address for the ClearCore
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

void initEthernetController(bool useDhcp)
{
    Console.serialInfo(F("Starting Ethernet initialization..."));

    // Print link status for debugging
    char msg[200];
    if (Ethernet.linkStatus() == LinkOFF)
    {
        Console.serialWarning(F("Ethernet physical link status: DISCONNECTED - cable may not be connected"));
        // Important: Continue anyway to initialize the Ethernet chip
    }
    else
    {
        Console.serialInfo(F("Ethernet physical link status: CONNECTED"));
    }

    // Initialize Ethernet interface
    int result = 0;
    if (useDhcp)
    {
        // Try DHCP
        Console.serialInfo(F("Attempting to get IP from DHCP..."));
        result = Ethernet.begin(mac);
        if (result == 0)
        {
            Console.serialError(F("DHCP failed! Falling back to static IP"));
            // Fall back to static IP
            IPAddress ip(192, 168, 0, 177);
            IPAddress dns(8, 8, 8, 8);
            IPAddress gateway(192, 168, 0, 1);
            IPAddress subnet(255, 255, 255, 0);
            Ethernet.begin(mac, ip, dns, gateway, subnet);
        }
    }
    else
    {
        // Use static IP
        Console.serialInfo(F("Using static IP configuration"));
        IPAddress ip(192, 168, 0, 177);
        IPAddress dns(8, 8, 8, 8);
        IPAddress gateway(192, 168, 0, 1);
        IPAddress subnet(255, 255, 255, 0);
        Ethernet.begin(mac, ip, dns, gateway, subnet);
    }

    // Print assigned IP address
    IPAddress ip = Ethernet.localIP();
    sprintf(msg, "Ethernet IP address: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    Console.serialInfo(msg);

    // Start the server
    server.begin();
    sprintf(msg, "Server started on port %d", ETHERNET_PORT);
    Console.serialInfo(msg);

    // Mark as initialized regardless of link status
    ethernetInitialized = true;

    Console.serialInfo(F("Ethernet initialization complete"));
}

void processEthernetConnections()
{
    if (!ethernetInitialized)
    {
        return;
    }

    // Check for Ethernet cable disconnection with warning throttling
    static bool warningPrinted = false;
    if (Ethernet.linkStatus() == LinkOFF)
    {
        if (ethernetInitialized && !warningPrinted)
        {
            Console.serialWarning(F("Ethernet cable disconnected."));
            warningPrinted = true;
        }
    }
    else
    {
        // Reset warning flag when cable is reconnected
        if (warningPrinted)
        {
            Console.serialInfo(F("Ethernet cable reconnected."));
            warningPrinted = false;
        }
    }

    // Current time for timeout calculations
    unsigned long currentTime = millis();

    // Check for inactive clients and disconnect them
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && clients[i].connected())
        {
            // Check if client has been inactive too long
            if (timeoutElapsed(currentTime, clientLastActivityTime[i], CLIENT_TIMEOUT_MS))
            {
                char msg[200];
                snprintf(msg, sizeof(msg), "[NETWORK] Closing inactive client: %d.%d.%d.%d:%d",
                         clients[i].remoteIP()[0], clients[i].remoteIP()[1],
                         clients[i].remoteIP()[2], clients[i].remoteIP()[3],
                         clients[i].remotePort());
                Console.serialInfo(msg);
                opLogHistory.addEntry(msg);
                clients[i].stop();
            }
        }
    }

    // Periodic connection testing (every PING_TEST_INTERVAL_MS)
    static unsigned long lastConnectionTestTime = 0;
    if (timeoutElapsed(currentTime, lastConnectionTestTime, PING_TEST_INTERVAL_MS))
    {
        lastConnectionTestTime = currentTime;

        // Test each connection by sending a small ping
        for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
        {
            if (clients[i] && clients[i].connected())
            {
                // Add grace period check here
                if (!timeoutElapsed(currentTime, clientLastActivityTime[i], PING_GRACE_PERIOD_MS))
                {
                    // Skip new connections (less than grace period old)
                    continue;
                }

                // Try to write a single byte to test connection
                if (!clients[i].print(""))
                {
                    char msg[200];
                    snprintf(msg, sizeof(msg), "[NETWORK] Detected stale connection: %d.%d.%d.%d:%d",
                             clients[i].remoteIP()[0], clients[i].remoteIP()[1],
                             clients[i].remoteIP()[2], clients[i].remoteIP()[3],
                             clients[i].remotePort());
                    Console.serialDiagnostic(msg);
                    opLogHistory.addEntry(msg);
                    clients[i].stop();
                }
            }
        }
    }

    // Check for new client connections
    EthernetClient newClient = server.accept();
    if (newClient)
    {
        bool clientAdded = false;

        // Find free slot
        for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
        {
            if (!clients[i] || !clients[i].connected())
            {
                // Found a free slot
                char msg[200];
                sprintf(msg, "[NETWORK] New client connected from %d.%d.%d.%d:%d",
                        newClient.remoteIP()[0], newClient.remoteIP()[1],
                        newClient.remoteIP()[2], newClient.remoteIP()[3],
                        newClient.remotePort());
                Console.serialInfo(msg);
                opLogHistory.addEntry(msg);

                // Replace client in this slot
                clients[i] = newClient;
                clientLastActivityTime[i] = millis(); // Initialize activity timestamp
                clientAdded = true;

                // Send welcome message
                // clients[i].println(F("Welcome to Lynx Conveyor Controller"));
                // clients[i].println(F("Type 'help' for available commands"));
                break;
            }
        }

        // If no free slots, reject connection
        if (!clientAdded)
        {
            Console.serialWarning(F("[NETWORK] Rejected client - no free slots"));
            opLogHistory.addEntry("[NETWORK] Rejected client - no free slots");
            newClient.println(F("ERROR: Too many connections"));
            newClient.stop();
        }
    }

    // Check for disconnected clients and remove them
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && !clients[i].connected())
        {
            char msg[200];
            snprintf(msg, sizeof(msg), "[NETWORK] Client disconnected: %d", i);
            Console.serialDiagnostic(msg);
            opLogHistory.addEntry(msg);
            clients[i].stop();
        }
    }
}

void testConnections()
{
    static unsigned long lastConnectionTestTime = 0;
    unsigned long currentTime = millis();

    // Test every TEST_CONNECTIONS_INTERVAL_MS
    if (!timeoutElapsed(currentTime, lastConnectionTestTime, TEST_CONNECTIONS_INTERVAL_MS))
    {
        return;
    }

    lastConnectionTestTime = currentTime;

    // Actively test each connection
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i])
        {
            // The connected() check might not catch all stale connections
            // We can do an additional check by trying to write a single byte
            if (clients[i].connected())
            {
                // Try to send a small non-disruptive ping
                // If this fails, the connection is stale
                if (!clients[i].print(" "))
                {
                    char msg[200];
                    snprintf(msg, sizeof(msg), "[NETWORK] Detected stale connection: %d.%d.%d.%d:%d",
                             clients[i].remoteIP()[0], clients[i].remoteIP()[1],
                             clients[i].remoteIP()[2], clients[i].remoteIP()[3],
                             clients[i].remotePort());
                    Console.serialDiagnostic(msg);
                    opLogHistory.addEntry(msg);
                    clients[i].stop();
                }
            }
        }
    }
}

// Send a message to all connected clients
bool sendToAllClients(const char *message)
{
    bool success = false;

    if (!ethernetInitialized)
    {
        return false;
    }

    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && clients[i].connected())
        {
            if (clients[i].println(message) > 0)
            {
                success = true;
            }
        }
    }

    return success;
}

int getConnectedClientCount()
{
    int count = 0;
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && clients[i].connected())
        {
            count++;
        }
    }
    return count;
}

// Call this whenever data is received from a client
void updateClientActivity(int clientIndex)
{
    if (clientIndex >= 0 && clientIndex < MAX_ETHERNET_CLIENTS)
    {
        clientLastActivityTime[clientIndex] = millis();
    }
}

// Network management functions
bool closeClientConnection(int index)
{
    if (index >= 0 && index < MAX_ETHERNET_CLIENTS &&
        clients[index] && clients[index].connected())
    {

        IPAddress ip = clients[index].remoteIP();
        int port = clients[index].remotePort();
        clients[index].stop();

        char msg[200];
        snprintf(msg, sizeof(msg), "[NETWORK] Manually closed connection from %d.%d.%d.%d:%d",
                 ip[0], ip[1], ip[2], ip[3], port);
        Console.serialInfo(msg);
        opLogHistory.addEntry(msg);
        return true;
    }
    return false;
}

bool closeAllConnections()
{
    int count = 0;
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && clients[i].connected())
        {
            clients[i].stop();
            count++;
        }
    }
    char msg[200];
    snprintf(msg, sizeof(msg), "[NETWORK] Closed %d connections", count);
    Console.serialInfo(msg);
    opLogHistory.addEntry(msg);
    return count > 0;
}