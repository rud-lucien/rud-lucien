#include "TCPStream.h"
#include <Ethernet.h>
#include <String.h>

// Constructor
TCPStream::TCPStream() {}

// Start the TCP server
void TCPStream::begin(EthernetClient _client)
{
    client = _client; // Set the client
    if (client)
    {
    hasClient = true; // Set client connected flag
    }

}

void TCPStream::print(const char* str)
{
    client.print(str);
}

void TCPStream::println(const char* str)
{
    client.println(str);
}

int TCPStream::available()
{
    return 1;
}

int TCPStream::read()
{
    return 1;
}

int TCPStream::peek()
{
    return 1;
}
size_t TCPStream::write(uint8_t c)
{
    
}   