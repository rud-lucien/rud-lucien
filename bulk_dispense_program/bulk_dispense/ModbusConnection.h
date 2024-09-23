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

    bool wasConnected = false;

public:
    ModbusConnection(byte macAddr[], IPAddress ipAddr, IPAddress serverAddr);

    // Setup Ethernet connection
    void setupEthernet();

    // Check and reconnect if needed
    void checkConnection();

    // Read modbus registers for a given address and quantity
    bool readRegisters(int registerAddress, int registerQuantity, uint32_t &result);
};

#endif
