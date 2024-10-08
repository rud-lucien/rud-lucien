#include "TCPServer.h"
#include <Ethernet.h>

// Constructor
TCPServer::TCPServer(IPAddress _ip, int _port) : ip(_ip), port(_port), server(_port) {}

// Start the TCP server
void TCPServer::begin() {
    server.begin();  // Start the server
    Serial.print("TCP Server started on IP: ");
    Serial.println(Ethernet.localIP());
}

EthernetClient& TCPServer::getClient() {
    return client;  // Return a reference to the connected client
}


// Handle incoming client connections and return received commands
String TCPServer::handleClient() {
    client = server.available();  // Accept an incoming connection
    String command = "";  // Store the command received from the client

    if (client) {
        Serial.println("Client connected.");
        
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') {
                    // Command ends when a newline is received
                    Serial.println("Command received: " + command);
                    // Process the command here
                    return command;  // Return the received command
                } else if (c != '\r') {
                    command += c;  // Append the character to the command string
                }
            }
        }
        
        // Optional: handle client disconnection
        if (!client.connected()) {
            Serial.println("Client disconnected.");
        }
    }
    
    return "";  // Return empty string if no valid command
}



