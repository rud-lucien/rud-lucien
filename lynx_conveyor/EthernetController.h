#ifndef ETHERNET_CONTROLLER_H
#define ETHERNET_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include "OutputManager.h"
#include "CommandController.h"

// Constants
#define MAX_ETHERNET_CLIENTS 4
#define ETHERNET_PORT 8888
#define MAX_PACKET_LENGTH 100

// External variable declarations
extern EthernetServer server;
extern EthernetClient clients[MAX_ETHERNET_CLIENTS];
extern bool ethernetInitialized;
extern char ethernetCommandBuffer[MAX_PACKET_LENGTH];

// Function declarations
void initEthernetController(bool useDhcp = true);
void processEthernetConnections();
bool sendToAllClients(const char* message);

#endif // ETHERNET_CONTROLLER_H