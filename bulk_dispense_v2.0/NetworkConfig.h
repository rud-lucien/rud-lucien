#ifndef NETWORKCONFIG_H
#define NETWORKCONFIG_H

#include <Ethernet.h>

// Network configuration constants
extern byte MAC_ADDRESS[6];
extern IPAddress DEVICE_IP;
extern const uint16_t TCP_PORT;

// Global network state variables
extern EthernetServer tcpServer;
extern EthernetClient currentClient;
extern bool hasActiveClient;

// Core network functions
void initializeNetwork();
void handleTcpConnections();
void disconnectClient();
bool isClientConnected();

#endif