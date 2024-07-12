#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Ethernet settings for Controllino Maxi
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress ip(169, 254, 0, 11); // IP address of your Controllino Maxi
EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient); // Modbus TCP client using the Ethernet client
IPAddress server(169, 254, 0, 10);          // IP address of your Modbus server (NU-EP1)

void setup()
{
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  while (!Serial)       // Wait for serial port to connect
    ;
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

  // Give the Ethernet shield a second to initialize
  delay(2000);

  // Print IP address
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

  // Attempt to connect to the Modbus server
  if (!modbusTCPClient.connected())
  {
    while (!modbusTCPClient.begin(server, 502)) // Keep trying to connect to the server on port 502
    {
      Serial.print("trying to connect\n");
      delay(1000);
    }
    Serial.println("Successfully connected to the server!");
  }
}

void loop()
{
}