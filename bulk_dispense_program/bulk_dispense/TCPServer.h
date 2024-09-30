#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <Ethernet.h>

class TCPServer {
private:
    EthernetServer server;
    EthernetClient client;
    int port;
    IPAddress ip;
    
public:
    // Constructor
    TCPServer(IPAddress _ip, int _port);
    
    // Start the TCP server
    void begin();

    EthernetClient getClient();


    // Handle incoming client connections and return received commands
    String handleClient();
};

#endif




