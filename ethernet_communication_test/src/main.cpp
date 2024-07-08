#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Ethernet settings for Controllino Maxi
byte mac[] = { 0x50, 0xD7, 0x53, 0x00, 0xD3, 0x59};

IPAddress ip(169, 254, 168, 10); // IP address of your Controllino Maxi
IPAddress server(169, 254, 168, 100); // IP address of your Modbus server (NU-EP1)

EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 // Start the Ethernet connection
  Serial.println("Starting Ethernet connection...");
  Ethernet.begin(mac, ip);
 
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  // Give the Ethernet shield a second to initialize
  delay(2000);

 // Print IP address
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

if(!modbusTCPClient.connected())
{
  while(!modbusTCPClient.begin(server, 502))
  {
    Serial.print("trying to connect\n");
    delay(1000);
  }
}

  // Connect to the Modbus TCP server
  // Serial.println("Connecting to Modbus TCP server...");
  // if (!modbusTCPClient.begin(server, 502)) {
  //   Serial.println("Failed to connect to Modbus TCP server!");
  //   while (1);
  // }
  // Serial.println("Connected to Modbus TCP server");

 
}



void loop()
{


}