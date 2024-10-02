#include "ModbusConnection.h"

ModbusConnection::ModbusConnection(byte macAddr[], IPAddress ipAddr, IPAddress serverAddr)
    : mac{macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]},
      ip(ipAddr), server(serverAddr), modbusTCPClient(ethClient) {}

void ModbusConnection::checkConnection() {
    if (!modbusTCPClient.connected()) {
        if (wasConnected) {
            Serial.println("Lost connection to the Modbus server!");
            wasConnected = false;
        }

        if (modbusTCPClient.begin(server, 502)) {
            Serial.println("Successfully connected to the Modbus server!");
            wasConnected = true;
        } else {
            Serial.println("Connection to Modbus server failed.");
        }
    }
}

// New method to check the connection status
bool ModbusConnection::isConnected() const {
    return wasConnected;  // Return the current connection status
}

bool ModbusConnection::readRegisters(int registerAddress, int registerQuantity, uint32_t &result) {
    if (modbusTCPClient.requestFrom(INPUT_REGISTERS, registerAddress, registerQuantity)) {
        if (modbusTCPClient.available() >= registerQuantity) {
            uint16_t highWord = modbusTCPClient.read();
            uint16_t lowWord = modbusTCPClient.read();
            if (highWord != -1 && lowWord != -1) {
                result = (static_cast<uint32_t>(highWord) << 16) | lowWord;
                return true;
            }
        }
    }
    Serial.println("Failed to read from Modbus server!");
    return false;
}

// Getter to return the Modbus server IP address
IPAddress ModbusConnection::getServerAddress() const {
    return server;
}

