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
IPAddress server(169, 254, 0, 10);          // IP address of your Modbus server (NQ-EP4L)

bool wasConnected = false;

#define RST_CHNL_1 CONTROLLINO_D1 // Aliase for integrated flow reset channel for FD-XA1 Sensor on port 1 of NQ-EP4L

// Function to reset IntegratedFlow
void resetIntegratedFlow(int reset_channel)
{
  Serial.println("Resetting Integrated Flow...");
  digitalWrite(reset_channel, HIGH); // Write the specified state to the pin
  delay(100);                        // Ensure the signal is recognized (increase the delay if needed)
  digitalWrite(reset_channel, LOW);
  Serial.println("Integrated Flow Reset Completed");
}

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
    ethClient.setConnectionTimeout(100); // Set the timeout duration for client.connect() and client.stop()
    if (modbusTCPClient.begin(server, 502))
    {
      Serial.println("Successfully reconnected to the server!");
      wasConnected = true;
    }
    else
    {
      Serial.println("Reconnection attempt failed.");
      delay(100); // Wait before retrying
    }
  }
}

void setup()
{
  pinMode(RST_CHNL_1, OUTPUT);

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  while (!Serial);      // Wait for serial port to connect
    

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
}

void loop()
{
  // Check Modbus connection
  checkModbusConnection();

  // Check for reset command
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("rst"))
    {
      resetIntegratedFlow(RST_CHNL_1);
    }
  }

  // Address and quantity of the register to read
  int registerAddress = 0x0002; // Base address for senor connected to Port 1 of NQ-EP4L
  int registerQuantity = 2;     // Quantity of registers to read (2 x 16-bit values = 32 bits)

  // Read input registers
  if (modbusTCPClient.requestFrom(INPUT_REGISTERS, registerAddress, registerQuantity))
  {
    if (modbusTCPClient.available() >= registerQuantity) // Ensure there are at least 2 values (32 bits)
    {
      long highWord = modbusTCPClient.read();
      long lowWord = modbusTCPClient.read();

      if (highWord != -1 && lowWord != -1)
      {
        // Combine the two registers into a 32-bit value
        uint32_t combined = (static_cast<uint32_t>(highWord) << 16) | static_cast<uint16_t>(lowWord);

        // Extract data from the combined value
        uint32_t integratedFlow = combined >> 14; // First 18 bits

        // Print the extracted data
        Serial.print("IntegratedFlow_mL: ");
        Serial.println(integratedFlow);
      }
      else
      {
        Serial.println("Failed to read register.");
      }
    }
  }
  else
  {
    Serial.println("Failed to read from Modbus server!");
  }

  // Wait before reading again
  delay(250);
}