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

// Function to reset IntegratedFlow
void resetIntegratedFlow()
{
  Serial.println("Resetting Integrated Flow...");
  digitalWrite(CONTROLLINO_D1, HIGH); // Write the specified state to the pin
  delay(250);  // Ensure the signal is recognized (increase the delay if needed)
  digitalWrite(CONTROLLINO_D1, LOW);
  Serial.println("Integrated Flow Reset Completed");
}

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
      return;      // Exit the loop function to avoid further processing until reconnected
    }
  }

 // Check for reset command
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("rst"))
    {
      resetIntegratedFlow();
    }
  }

   // Address and quantity of the register to read
  int registerAddress = 0x0002; // Base address for instantaneous flow rate in Process Data Structure 0
  int registerQuantity = 2;     // Quantity of registers to read (2 x 16-bit values = 32 bits)

  // Read input registers
  if (modbusTCPClient.requestFrom(INPUT_REGISTERS, registerAddress, registerQuantity))
  {
    if (modbusTCPClient.available() >= registerQuantity) // Ensure there are at least 2 values (32 bits)
    {
      long highWord = modbusTCPClient.read();
      long lowWord = modbusTCPClient.read();
      
      if (highWord != -1 && lowWord != -1) {
        // Combine the two registers into a 32-bit value
        uint32_t combined = (static_cast<uint32_t>(highWord) << 16) | static_cast<uint16_t>(lowWord);
        
        // Extract data from the combined value
        uint32_t integratedFlow = combined >> 14; // First 18 bits
        
        // uint8_t error = (combined >> 18) & 0x7; // Next 3 bits
        // uint8_t stabilityLevel = (combined >> 21) & 0x7; // Next 3 bits
        // uint8_t alert = (combined >> 24) & 0x1; // Next 1 bit
        // uint8_t output2 = (combined >> 25) & 0x1; // Next 1 bit
        uint8_t output1 = combined & 0x1; // Next 1 bit

        // Print the extracted data
        Serial.print("IntegratedFlow_mL: ");
        Serial.print(integratedFlow);
        Serial.println("\n");
        Serial.print("Output1_value: ");
        Serial.print(output1);
        Serial.println("\n");

      } else {
        Serial.println("Failed to read register.");
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