#ifndef TCPSTREAM_H
#define TCPSTREAM_H

#include <Ethernet.h>

class TCPStream: public Stream {
private:
    EthernetClient client;
    
public:
    // Constructor
    TCPStream();
    bool hasClient;
    
    // Start the TCP server
    void begin(EthernetClient _client);
    void print (const char* str);
    void println (const char* str);
    int available();
    int read();
    int peek();
    size_t write(uint8_t);
};

#endif




