#include <Arduino.h>
#include <Controllino.h>

// Function to toggle digital output pins
void toggleDigitalOutput(int pin, int state) {
  pinMode(pin, OUTPUT);     // Set the pin mode to OUTPUT
  digitalWrite(pin, state); // Write the specified state to the pin
}

// // Function to map pin alias to actual pin number
// int getPinAlias(const char *alias) {
//   if (strcmp(alias, "D0") == 0) return CONTROLLINO_D0;
//   if (strcmp(alias, "D1") == 0) return CONTROLLINO_D1;
//   if (strcmp(alias, "D2") == 0) return CONTROLLINO_D2;
//   if (strcmp(alias, "D3") == 0) return CONTROLLINO_D3;
//   if (strcmp(alias, "D4") == 0) return CONTROLLINO_D4;
//   if (strcmp(alias, "D5") == 0) return CONTROLLINO_D5;
//   if (strcmp(alias, "D6") == 0) return CONTROLLINO_D6;
//   if (strcmp(alias, "D7") == 0) return CONTROLLINO_D7;
//   // Add other pin mappings as needed
//   return -1; // Invalid alias
// }

// void setup() {
//   Serial.begin(115200); // Initialize serial communication at 115200 baud rate
//   while (!Serial) {
//     ; // Wait for serial port to connect. Needed for native USB port only
//   }
//   Serial.println("Ready to receive commands.");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     String command = Serial.readStringUntil('\n'); // Read the incoming command
//     command.trim();                                // Remove any leading/trailing whitespace

//     // Parse the command
//     if (command.startsWith("TOGGLE")) {
//       int pin, state;
//       char pinAlias[10];
//       char stateStr[10];

//       sscanf(command.c_str(), "TOGGLE %s %s", pinAlias, stateStr);
//       pin = getPinAlias(pinAlias);
//       state = strcmp(stateStr, "HIGH") == 0 ? HIGH : LOW;

//       if (pin != -1) {
//         toggleDigitalOutput(pin, state);
//         Serial.print("Toggled pin ");
//         Serial.print(pinAlias);
//         Serial.print(" to ");
//         Serial.println(state == HIGH ? "HIGH" : "LOW");
//       } else {
//         Serial.println("Invalid pin alias.");
//       }
//     } else {
//       Serial.println("Invalid command.");
//     }
//   }
// }

// Function to map pin alias to actual pin number
const char* getPinAlias(const char *alias, int &pin) {
  if (strcmp(alias, "D0") == 0) {
    pin = CONTROLLINO_D0;
    return "";
  }
  if (strcmp(alias, "D1") == 0) {
    pin = CONTROLLINO_D1;
    return "";
  }
  if (strcmp(alias, "D2") == 0) {
    pin = CONTROLLINO_D2;
    return "";
  }
  if (strcmp(alias, "D3") == 0) {
    pin = CONTROLLINO_D3;
    return "";
  }
  if (strcmp(alias, "D4") == 0) {
    pin = CONTROLLINO_D4;
    return "";
  }
  if (strcmp(alias, "D5") == 0) {
    pin = CONTROLLINO_D5;
    return "";
  }
  if (strcmp(alias, "D6") == 0) {
    pin = CONTROLLINO_D6;
    return "";
  }
  if (strcmp(alias, "D7") == 0) {
    pin = CONTROLLINO_D7;
    return "";
  }
  // Add other pin mappings as needed

  // If alias is invalid
  return "Invalid pin alias.";
}

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Ready to receive commands.");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the incoming command
    command.trim();                                // Remove any leading/trailing whitespace

    // Parse the command
    if (command.startsWith("TOGGLE")) {
      int pin;
      char pinAlias[10];
      char stateStr[10];

      sscanf(command.c_str(), "TOGGLE %s %s", pinAlias, stateStr);
      const char* error = getPinAlias(pinAlias, pin);
      if (strcmp(error, "") == 0) {
        int state = (strcmp(stateStr, "HIGH") == 0) ? HIGH : LOW;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, state);
        Serial.print("Toggled pin ");
        Serial.print(pinAlias);
        Serial.print(" to ");
        Serial.println(state == HIGH ? "HIGH" : "LOW");
      } else {
        Serial.println(error); // Print the error message
      }
    } else {
      Serial.println("Invalid command.");
    }
  }
}