#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// // Ethernet settings for Controllino Maxi Automation
// byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// IPAddress ip(169, 254, 0, 11); // IP address of your Controllino Maxi Automation
// EthernetClient ethClient;
// ModbusTCPClient modbusTCPClient(ethClient); // Modbus TCP client using the Ethernet client
// IPAddress server(169, 254, 0, 13);          // IP address of your Modbus server (VAEM)

// void connectToServer() {
//   // Stop any existing connections
//   modbusTCPClient.stop();
//   ethClient.stop();
//   delay(1000);

//   if (modbusTCPClient.begin(server, 502)) {
//     Serial.println("Successfully connected to the server!");
//   } else {
//     Serial.println("Connection attempt failed.");
//     // Retry after a short delay
//     delay(5000);
//   }
// }

// void setup(){
//   Serial.begin(115200);
//   while(!Serial);

//  // start the Ethernet connection and the server:
//   Ethernet.begin(mac, ip);

//   if (Ethernet.hardwareStatus() == EthernetNoHardware)
//   {
//     Serial.println("Ethernet shield was not found. Can't run without hardware.");
//     while (true)
//     {
//       delay(1);
//     }
//   }

// delay(2000);

// Serial.print("IP Address: ");
// Serial.println(Ethernet.localIP());

// // Try to connect to the Modbus server
//   connectToServer();
// }

// void loop()
// {
// // Check if the client is connected, if not try to reconnect
//   if (!modbusTCPClient.connected()) {
//     Serial.println("Disconnected from server, attempting to reconnect...");
//     connectToServer();
//   }

//   // Add your regular code here

//   delay(5000); // Adjust the delay as necessary
// }

#define V1 CONTROLLINO_D0 // Aliase for valve 1

void setup()
{
  pinMode(V1, OUTPUT);

  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  while (!Serial)
    ; // Wait for serial port to connect

}

void loop()
{

  // Check for reset command from serial input
  // Check for command from serial input
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("v1_on"))
    {
      digitalWrite(V1, HIGH);
      Serial.println("V1 is ON");
    }
    else if (command.equals("v1_off"))
    {
      digitalWrite(V1, LOW);
      Serial.println("V1 is OFF");
    }
  }
}