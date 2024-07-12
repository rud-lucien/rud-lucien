#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Ethernet settings for Controllino Maxi Automation
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress ip(169, 254, 0, 11); // IP address of your Controllino Maxi
EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient); // Modbus TCP client using the Ethernet client
IPAddress server(169, 254, 0, 10);          // IP address of your Modbus server (NU-EP1)

bool wasConnected = false;

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
    wasConnected = true;
  }
}

void loop()
{
  // Check if the client is still connected
  if (!modbusTCPClient.connected())
  {
    if (wasConnected)
    {
      Serial.println("Lost connection to the server!");
      wasConnected = false;
    }

    // Attempt to reconnect
    Serial.println("Attempting to reconnect to the server...");
    if (modbusTCPClient.begin(server, 502))
    {
      Serial.println("Successfully reconnected to the server!");
      wasConnected = true;
    }
    else
    {
      Serial.println("Reconnection attempt failed.");
      delay(1000); // Wait before retrying
      return; // Exit the loop function to avoid further processing until reconnected
    }
  }

  // Address and quantity of the register to read
  int registerAddress = 0x0002; // Base address for Port 1 process data
  int registerQuantity = 6;     // Quantity of registers to read (7 x 16-bit values = 14 bytes)

  // Read holding registers
  if (modbusTCPClient.requestFrom(HOLDING_REGISTERS, registerAddress, registerQuantity))
  {
    if (modbusTCPClient.available() >= registerQuantity)  // 
    {
      // Print the raw data for each register
      Serial.println("Raw Process Data:");
      for (int i = 0; i < registerQuantity; i++)
      {
        long rawData = modbusTCPClient.read();
        
        if(i == 0)
        {
        long flowrate = rawData >> 14;
        float flowrate_ml = float(flowrate)*0.1;
        Serial.print("flowrate ml: ");
        Serial.println(flowrate_ml);
        }

        if (i == 1)
        {
          long shot_amount = rawData >> 14;
          float shot_amount_ml = float(shot_amount) * 0.001;
          Serial.print("Shot amount mL: ");
          Serial.println(shot_amount_ml);
        }

        if(i == 2)
        {
        long integrated_flow = rawData >> 14;
        float integrated_flow_ml = float(integrated_flow)*.01;
        Serial.print("Integrated flow: ");
        Serial.println(integrated_flow_ml);
        }

        Serial.print("Process Data Structure: ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(rawData, HEX); // Print the data in hexadecimal format
      }
    }
  }
  else
  {
    Serial.println("Failed to read from Modbus server!");
  }

  // Wait before reading again
  delay(1000);
}