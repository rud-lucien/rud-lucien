#include "EthernetController.h"



// Global variables
EthernetServer server(ETHERNET_PORT);
EthernetClient clients[MAX_ETHERNET_CLIENTS];
bool ethernetInitialized = false;
char ethernetCommandBuffer[MAX_PACKET_LENGTH];

// MAC address for the ClearCore
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

void initEthernetController(bool useDhcp)
{
    Console.info(F("Starting Ethernet initialization..."));

    // Print link status for debugging
    Console.print(F("Ethernet physical link status: "));
    if (Ethernet.linkStatus() == LinkOFF)
    {
        Console.warning(F("DISCONNECTED - cable may not be connected"));
        // Important: Continue anyway to initialize the Ethernet chip
    }
    else
    {
        Console.info(F("CONNECTED"));
    }

    // Initialize Ethernet interface
    int result = 0;
    if (useDhcp)
    {
        // Try DHCP
        Console.info(F("Attempting to get IP from DHCP..."));
        result = Ethernet.begin(mac);
        if (result == 0)
        {
            Console.error(F("DHCP failed! Falling back to static IP"));
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
        Console.info(F("Using static IP configuration"));
        IPAddress ip(192, 168, 0, 177);
        IPAddress dns(8, 8, 8, 8);
        IPAddress gateway(192, 168, 0, 1);
        IPAddress subnet(255, 255, 255, 0);
        Ethernet.begin(mac, ip, dns, gateway, subnet);
    }

    // Print assigned IP address
    IPAddress ip = Ethernet.localIP();
    Console.print(F("Ethernet IP address: "));
    Console.println(ip);

    // Start the server
    server.begin();
    Console.print(F("Server started on port "));
    Console.println(ETHERNET_PORT);

    // Mark as initialized regardless of link status
    ethernetInitialized = true;

    Console.info(F("Ethernet initialization complete"));
}

void processEthernetConnections()
{
    if (!ethernetInitialized)
    {
        return;
    }

    // Check for Ethernet cable disconnection with warning throttling
    static bool warningPrinted = false;
    if (Ethernet.linkStatus() == LinkOFF) {
        if (ethernetInitialized && !warningPrinted) {
            Console.warning(F("Ethernet cable disconnected."));
            warningPrinted = true;
        }
    } else {
        // Reset warning flag when cable is reconnected
        if (warningPrinted) {
            Console.info(F("Ethernet cable reconnected."));
            warningPrinted = false;
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
                Console.print(F("[NETWORK] New client connected from "));
                Console.print(newClient.remoteIP());
                Console.print(F(":"));
                Console.println(newClient.remotePort());

                // Replace client in this slot
                clients[i] = newClient;
                clientAdded = true;

                // Send welcome message
                clients[i].println(F("Welcome to Lynx Conveyor Controller"));
                clients[i].println(F("Type 'help' for available commands"));
                break;
            }
        }

        // If no free slots, reject connection
        if (!clientAdded)
        {
            Console.warning(F("[NETWORK] Rejected client - no free slots"));
            newClient.println(F("ERROR: Too many connections"));
            newClient.stop();
        }
    }

    // Check for disconnected clients and remove them
    for (int i = 0; i < MAX_ETHERNET_CLIENTS; i++)
    {
        if (clients[i] && !clients[i].connected())
        {
            Console.print(F("[NETWORK] Client disconnected: "));
            Console.println(i);
            clients[i].stop();
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