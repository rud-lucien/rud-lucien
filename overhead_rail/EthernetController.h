#ifndef ETHERNET_CONTROLLER_H
#define ETHERNET_CONTROLLER_H

//=============================================================================
// INCLUDES
//=============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include "OutputManager.h"
#include "Utils.h"

//=============================================================================
// CONFIGURATION
//=============================================================================
// Network settings
#define MAX_ETHERNET_CLIENTS 1    // Maximum number of simultaneous client connections
#define ETHERNET_PORT 8888        // TCP port to listen on
#define MAX_PACKET_LENGTH 100     // Maximum length of received command packets

//=============================================================================
// EXTERNAL VARIABLES
//=============================================================================
extern EthernetServer server;                                      // Ethernet server instance
extern EthernetClient clients[MAX_ETHERNET_CLIENTS];               // Array of client connections
extern bool ethernetInitialized;                                   // Initialization status flag
extern unsigned long clientLastActivityTime[MAX_ETHERNET_CLIENTS]; // Last activity timestamps
extern const unsigned long CLIENT_TIMEOUT_MS;                      // Client inactivity timeout

//=============================================================================
// FUNCTION DECLARATIONS
//=============================================================================
// Initialization
void initEthernetController(bool useDhcp = true); // Initialize Ethernet with DHCP or static IP

// Connection management
void processEthernetConnections();          // Process new/existing connections
void testConnections();                     // Actively test connection health
void updateClientActivity(int clientIndex); // Record client activity to prevent timeout
bool closeClientConnection(int index);      // Close a specific client connection
bool closeAllConnections();                 // Close all client connections

// Communication
bool sendToAllClients(const char *message); // Send message to all connected clients
int getConnectedClientCount();              // Get count of currently connected clients

// Status and diagnostics
void printEthernetStatus();                 // Print Ethernet connection status

// Timeout management
void resetClientTimeouts();                 // Reset all client activity timeouts to current time

#endif // ETHERNET_CONTROLLER_H
