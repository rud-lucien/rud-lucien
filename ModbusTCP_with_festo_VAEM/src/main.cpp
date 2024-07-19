#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Ethernet settings for Controllino Maxi Automation
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(169, 254, 0, 11); // IP address of your Controllino Maxi Automation
EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient); // Modbus TCP client using the Ethernet client
IPAddress server(169, 254, 0, 15);          // IP address of your Modbus server (VAEM)

bool wasConnected = false;

// Function to check and reconnect Modbus client
void checkModbusConnection()
{
  if (!modbusTCPClient.connected())
  {
    if (wasConnected)
    {
      Serial.println("Lost connection to the server!");
      wasConnected = false;
    }

    Serial.println("Attempting to reconnect to the server...");
    ethClient.setConnectionTimeout(2100); // Set the timeout duration for client.connect() and client.stop()
    if (modbusTCPClient.begin(server, 502))
    {
      Serial.println("Successfully reconnected to the server!");
      wasConnected = true;

      // Set operating mode to 1
      if (!modbusTCPClient.holdingRegisterWrite(9, 0x00)) {
        Serial.println("Failed to set operating mode to 1");
      } else {
        Serial.println("Operating mode set to 1");
      }
    }
    else
    {
      Serial.println("Reconnection attempt failed.");
    }
  }
}

void setup()
{

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  while (!Serial)
    ; // Wait for serial port to connect

  // Start the Ethernet connection
  Serial.println("Starting Ethernet connection...");
  Ethernet.begin(mac, ip); // Initialize Ethernet with the MAC address and IP address

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true)
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }

  // Give the Ethernet shield a time to initialize
  delay(2000);

  // Print IP address
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

  // Attempt to connect to the Modbus server
  checkModbusConnection();

  // // Set operating mode to 1
  
  // if (modbusTCPClient.connected()){
  //   if (!modbusTCPClient.holdingRegisterWrite(9, 0x00)) {
  //     Serial.println("Failed to set operating mode to 1");
  //   } else {
  //     Serial.println("Operating mode set to 1");
  //   }

}

void loop()
{
  // checkModbusConnection();
}