#ifndef MODBUSCONNECTION_H
#define MODBUSCONNECTION_H

#include <Ethernet.h>
#include <ArduinoModbus.h>

class ModbusConnection {
private:
    byte mac[6];
    IPAddress ip;
    IPAddress server;
    EthernetClient ethClient;
    ModbusTCPClient modbusTCPClient;

    bool wasConnected = false;  // Track the Modbus connection status

public:
    // Constructor
    ModbusConnection(byte macAddr[], IPAddress ipAddr, IPAddress serverAddr);

    // Check and reconnect if needed
    void checkConnection();

    // Method to check if Modbus is connected
    bool isConnected() const;  // Return the connection status

    // Read Modbus registers for a given address and quantity
    bool readRegisters(int registerAddress, int registerQuantity, uint32_t &result);

    // Getter to access the Modbus server IP
    IPAddress getServerAddress() const;  // Add this line to the header
};

#endif

