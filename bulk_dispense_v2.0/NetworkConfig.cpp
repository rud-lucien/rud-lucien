#include "NetworkConfig.h"
#include "Commands.h"
#include <Controllino.h>

// Network configuration
byte MAC_ADDRESS[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress DEVICE_IP(192, 168, 0, 13);
const uint16_t TCP_PORT = 8080;

// Global state variables
EthernetServer tcpServer(TCP_PORT);
EthernetClient currentClient;
bool hasActiveClient = false;
String currentCommand = "";

void initializeNetwork() {
    // Initialize Ethernet with MAC and IP
    Ethernet.begin(MAC_ADDRESS, DEVICE_IP);
    
    // Print network information
    Serial.print(F("[MESSAGE] Device Ethernet IP Address: "));
    Serial.println(Ethernet.localIP());
    
    delay(1000); // Allow time for Ethernet initialization
    
    // Start TCP server
    tcpServer.begin();
    Serial.println(F("[MESSAGE] TCP server initialized."));
    Serial.print(F("[MESSAGE] TCP/IP Address: "));
    Serial.println(DEVICE_IP);
    Serial.print(F("[MESSAGE] TCP/IP Port: "));
    Serial.println(TCP_PORT);
    
    delay(500);
}

void handleTcpConnections() {
    // Check for new clients
    if (!hasActiveClient) {
        EthernetClient newClient = tcpServer.available();
        if (newClient) {
            currentClient = newClient;
            hasActiveClient = true;
            Serial.println(F("[MESSAGE] New client connected"));
        }
    }
}



void disconnectClient() {
    if (hasActiveClient && !currentClient.connected()) {
        currentClient.stop();
        hasActiveClient = false;
        Serial.println(F("[MESSAGE] Client disconnected"));
    }
}

bool isClientConnected() {
    return hasActiveClient && currentClient.connected();
}





