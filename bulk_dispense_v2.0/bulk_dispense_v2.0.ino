/*
  Bulk Dispense v2.0 Code
  -----------------------
  This sketch implements 
*/

// =====================
// Include Libraries
// =====================
#include <Controllino.h>
#include "Commander-API.hpp"
#include "Commander-IO.hpp"

// =====================
// Global Constants
// =====================
#define FAN_CONTROL_PIN CONTROLLINO_R6

// Reagent Valves
#define REAGENT_VALVE_1_PIN CONTROLLINO_R0
#define REAGENT_VALVE_2_PIN CONTROLLINO_R1
#define REAGENT_VALVE_3_PIN CONTROLLINO_R2
#define REAGENT_VALVE_4_PIN CONTROLLINO_R3

// Media Valves
#define MEDIA_VALVE_1_PIN CONTROLLINO_DO0
#define MEDIA_VALVE_2_PIN CONTROLLINO_DO1
#define MEDIA_VALVE_3_PIN CONTROLLINO_DO2
#define MEDIA_VALVE_4_PIN CONTROLLINO_DO3

// Waste Valves
#define WASTE_VALVE_1_PIN CONTROLLINO_R4
#define WASTE_VALVE_2_PIN CONTROLLINO_R5
#define WASTE_VALVE_3_PIN CONTROLLINO_R8
#define WASTE_VALVE_4_PIN CONTROLLINO_R9

// Overflow Sensors
#define OVERFLOW_SENSOR_TROUGH_1_PIN CONTROLLINO_DI0
#define OVERFLOW_SENSOR_TROUGH_2_PIN CONTROLLINO_DI1
#define OVERFLOW_SENSOR_TROUGH_3_PIN CONTROLLINO_DI2
#define OVERFLOW_SENSOR_TROUGH_4_PIN CONTROLLINO_DI3

// Bubble Sensors
#define REAGENT_1_BUBBLE_SENSOR_PIN CONTROLLINO_AI0
#define REAGENT_2_BUBBLE_SENSOR_PIN CONTROLLINO_AI1
#define REAGENT_3_BUBBLE_SENSOR_PIN CONTROLLINO_AI2
#define REAGENT_4_BUBBLE_SENSOR_PIN CONTROLLINO_AI3

// Waste Line Liquid Sensors
#define WASTE_1_LIQUID_SENSOR_PIN CONTROLLINO_AI4
#define WASTE_2_LIQUID_SENSOR_PIN CONTROLLINO_AI5

// Waste Bottle Liquid Sensors
#define WASTE_BOTTLE_1_LIQUID_SENSOR_PIN CONTROLLINO_AI6
#define WASTE_BOTTLE_2_LIQUID_SENSOR_PIN CONTROLLINO_AI7

// Waste Bottle Vaccum Detection Switch
#define WASTE_BOTTLE_1_VACUUM_SENSOR_PIN CONTROLLINO_AI8
#define WASTE_BOTTLE_2_VACUUM_SENSOR_PIN CONTROLLINO_AI9

// Define array sizes
#define NUM_REAGENT_VALVES 4
#define NUM_MEDIA_VALVES 4
#define NUM_WASTE_VALVES 4


#define NUM_OVERFLOW_SENSORS 4
#define NUM_REAGENT_BUBBLE_SENSORS 4
#define NUM_WASTE_LINE_SENSORS 2
#define NUM_WASTE_BOTTLE_SENSORS 2
#define NUM_WASTE_VACUUM_SENSORS 2

// =====================
// Struct Prototypes
// =====================
struct OnOffValve;
struct FanControl;
struct LoggingManagement;
struct BinarySensor;

// =====================
// Command Function Prototypes
// =====================
void cmd_set_log_frequency(char *args, Stream *response);
void cmd_fan(char *args, Stream *response);
void cmd_set_reagent_valve(char *args, Stream *response);
void cmd_set_media_valve(char *args, Stream *response);
void cmd_set_waste_valve(char *args, Stream *response);

// =====================
// Helper Function Prototypes
// =====================
void handleSerialCommands();
void logSystemState();
void monitorOverflowSensors(unsigned long currentTime);
void monitorReagentBubbleSensors(unsigned long currentTime);
void monitorWasteLineSensors(unsigned long currentTime);
void monitorWasteBottleSensors(unsigned long currentTime);
void monitorWasteVacuumSensors(unsigned long currentTime);


// =====================
// OnOffValve: Functional Style for Reagent/Media Valves
// =====================
struct OnOffValve {
  uint8_t controlPin;
  bool isOpen;
};

OnOffValve createValve(uint8_t pin) {
  OnOffValve valve;
  valve.controlPin = pin;
  valve.isOpen = false;  // Valve starts closed
  return valve;
}

void valveSetup(const OnOffValve &valve) {
  pinMode(valve.controlPin, OUTPUT);
  digitalWrite(valve.controlPin, LOW);  // Ensure valve is closed initially
}

OnOffValve openValve(OnOffValve valve) {
  digitalWrite(valve.controlPin, HIGH);
  valve.isOpen = true;
  return valve;
}

OnOffValve closeValve(OnOffValve valve) {
  digitalWrite(valve.controlPin, LOW);
  valve.isOpen = false;
  return valve;
}

bool isValveOpen(const OnOffValve &valve) {
  return valve.isOpen;
}

// =====================
// Fan Control Struct and Functions
// =====================
struct FanControl {
  uint8_t relayPin;  // Pin connected to Relay 6 (fan control)
};

void setFanState(const FanControl &config, bool state) {
  digitalWrite(config.relayPin, state ? HIGH : LOW);
}

void printFanState(bool state) {
  Serial.print(F("Fan is "));
  Serial.println(state ? F("ON") : F("OFF"));
}

void fanSetup(const FanControl &fc) {
  pinMode(fc.relayPin, OUTPUT);
  digitalWrite(fc.relayPin, LOW);  // Ensure fan is off initially
}

// Global fan configuration
const FanControl fan = { FAN_CONTROL_PIN };

// =====================
// Logging Management
// =====================
struct LoggingManagement {
  unsigned long previousLogTime;
  unsigned long logInterval;  // Log interval in milliseconds
};

LoggingManagement logging = {0, 250};

void logData(const char* module, const char* message) {
  // Simple log function for immediate events
  Serial.print(F("[LOG] "));
  Serial.print(module);
  Serial.print(F(" - "));
  Serial.println(message);
}

// =====================
// BinarySensor: For Sensors with a Digital Output (0 or 1)
// =====================
struct BinarySensor {
  uint8_t inputPin;   // Digital input pin for the sensor
  bool activeHigh;    // true if HIGH means triggered, false if LOW means triggered
};

BinarySensor createBinarySensor(uint8_t pin, bool activeHigh) {
  BinarySensor sensor;
  sensor.inputPin = pin;
  sensor.activeHigh = activeHigh;
  return sensor;
}

void binarySensorSetup(const BinarySensor &sensor) {
  pinMode(sensor.inputPin, INPUT);
}

bool readBinarySensor(const BinarySensor &sensor) {
  int reading = digitalRead(sensor.inputPin);
  return sensor.activeHigh ? (reading == HIGH) : (reading == LOW);
}

// =====================
// Global Valve Variables (Individual and Array Forms)
// =====================

// Individual global valve variables (if needed later)
OnOffValve reagentValve1 = createValve(REAGENT_VALVE_1_PIN);
OnOffValve reagentValve2 = createValve(REAGENT_VALVE_2_PIN);
OnOffValve reagentValve3 = createValve(REAGENT_VALVE_3_PIN);
OnOffValve reagentValve4 = createValve(REAGENT_VALVE_4_PIN);

OnOffValve mediaValve1 = createValve(MEDIA_VALVE_1_PIN);
OnOffValve mediaValve2 = createValve(MEDIA_VALVE_2_PIN);
OnOffValve mediaValve3 = createValve(MEDIA_VALVE_3_PIN);
OnOffValve mediaValve4 = createValve(MEDIA_VALVE_4_PIN);

OnOffValve wasteValve1 = createValve(WASTE_VALVE_1_PIN);
OnOffValve wasteValve2 = createValve(WASTE_VALVE_2_PIN);
OnOffValve wasteValve3 = createValve(WASTE_VALVE_3_PIN);
OnOffValve wasteValve4 = createValve(WASTE_VALVE_4_PIN);

// Alternatively, global arrays (this is more in line with functional style):
OnOffValve reagentValves[NUM_REAGENT_VALVES] = {
  createValve(REAGENT_VALVE_1_PIN),
  createValve(REAGENT_VALVE_2_PIN),
  createValve(REAGENT_VALVE_3_PIN),
  createValve(REAGENT_VALVE_4_PIN)
};

OnOffValve mediaValves[NUM_MEDIA_VALVES] = {
  createValve(MEDIA_VALVE_1_PIN),
  createValve(MEDIA_VALVE_2_PIN),
  createValve(MEDIA_VALVE_3_PIN),
  createValve(MEDIA_VALVE_4_PIN)
};

OnOffValve wasteValves[NUM_WASTE_VALVES] = {
  createValve(WASTE_VALVE_1_PIN),
  createValve(WASTE_VALVE_2_PIN),
  createValve(WASTE_VALVE_3_PIN),
  createValve(WASTE_VALVE_4_PIN)
};

// =====================
// Global Sensor Variables
// =====================

// Overflow Sensors
BinarySensor overflowSensorTrough1 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_1_PIN, true);
BinarySensor overflowSensorTrough2 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_2_PIN, true);
BinarySensor overflowSensorTrough3 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_3_PIN, true);
BinarySensor overflowSensorTrough4 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_4_PIN, true);

// Bubble Sensors
BinarySensor reagent1BubbleSensor1 = createBinarySensor(REAGENT_1_BUBBLE_SENSOR_PIN, true);
BinarySensor reagent1BubbleSensor2 = createBinarySensor(REAGENT_2_BUBBLE_SENSOR_PIN, true);
BinarySensor reagent1BubbleSensor3 = createBinarySensor(REAGENT_3_BUBBLE_SENSOR_PIN, true);
BinarySensor reagent1BubbleSensor4 = createBinarySensor(REAGENT_4_BUBBLE_SENSOR_PIN, true);

// Waste Line Sensors
BinarySensor waste1LiquidSensor = createBinarySensor(WASTE_1_LIQUID_SENSOR_PIN, true);
BinarySensor waste2LiquidSensor = createBinarySensor(WASTE_2_LIQUID_SENSOR_PIN, true);

// Waste Bottle Sensors
BinarySensor overflowSensorWasteBottle1 = createBinarySensor(WASTE_BOTTLE_1_LIQUID_SENSOR_PIN, true);
BinarySensor overflowSensorWasteBottle2 = createBinarySensor(WASTE_BOTTLE_2_LIQUID_SENSOR_PIN, true);

// Waste Bottle Sensors
BinarySensor waste1VacuumSensor = createBinarySensor(WASTE_BOTTLE_1_VACUUM_SENSOR_PIN, true);
BinarySensor waste2VacuumSensor = createBinarySensor(WASTE_BOTTLE_2_VACUUM_SENSOR_PIN, true);

// Create sensor arrays for easier looping:
BinarySensor overflowSensors[NUM_OVERFLOW_SENSORS] = {
  overflowSensorTrough1,
  overflowSensorTrough2,
  overflowSensorTrough3,
  overflowSensorTrough4
};

BinarySensor reagentBubbleSensors[NUM_REAGENT_BUBBLE_SENSORS] = {
  reagent1BubbleSensor1,
  reagent1BubbleSensor2,
  reagent1BubbleSensor3,
  reagent1BubbleSensor4
};

BinarySensor wasteLineSensors[NUM_WASTE_LINE_SENSORS] = {
  waste1LiquidSensor,
  waste2LiquidSensor
};

BinarySensor wasteBottleSensors[NUM_WASTE_BOTTLE_SENSORS] = {
  overflowSensorWasteBottle1,
  overflowSensorWasteBottle2
};

BinarySensor wasteVacuumSensors[NUM_WASTE_VACUUM_SENSORS] = {
  waste1VacuumSensor,
  waste2VacuumSensor
};

// =====================
// Commander API Command Functions
// =====================

// Command callback to set the log frequency ("LF <ms>")
void cmd_set_log_frequency(char *args, Stream *response) {
  int newInterval = -1;
  if (sscanf(args, "%d", &newInterval) == 1 && newInterval > 0) {
    logging.logInterval = newInterval;
    response->print(F("Log frequency set to "));
    response->print(newInterval);
    response->println(F(" ms"));
  } else {
    response->println(F("Invalid log frequency. Use: LF <positive number>"));
  }
}

// Command callback to control the fan ("FN <0/1>")
void cmd_fan(char *args, Stream *response) {
  int fanState = -1;
  if (sscanf(args, "%d", &fanState) == 1 && (fanState == 0 || fanState == 1)) {
    bool state = (fanState == 1);
    setFanState(fan, state);
    printFanState(state);
    logData("Fan", state ? "1" : "0");
    response->println(F("Fan command executed."));
  } else {
    response->println(F("Invalid fan command. Use: FN <0/1>"));
  }
}

// Command callback to control reagent valves ("R <valve number> <0/1>")
void cmd_set_reagent_valve(char *args, Stream *response) {
  int valveNumber = -1;
  int valveState = -1;
  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_REAGENT_VALVES &&
      (valveState == 0 || valveState == 1)) {
    bool state = (valveState == 1);
    // Using individual global variables (or you can use the array if preferred)
    switch (valveNumber) {
      case 1:
        reagentValve1 = state ? openValve(reagentValve1) : closeValve(reagentValve1);
        response->print(F("Reagent valve 1 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 2:
        reagentValve2 = state ? openValve(reagentValve2) : closeValve(reagentValve2);
        response->print(F("Reagent valve 2 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 3:
        reagentValve3 = state ? openValve(reagentValve3) : closeValve(reagentValve3);
        response->print(F("Reagent valve 3 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 4:
        reagentValve4 = state ? openValve(reagentValve4) : closeValve(reagentValve4);
        response->print(F("Reagent valve 4 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
    }
    // Log reagent valve states in compact format.
    char rvBuffer[6];
    sprintf(rvBuffer, "%c%c%c%c", 
            (reagentValve1.isOpen ? '1' : '0'),
            (reagentValve2.isOpen ? '1' : '0'),
            (reagentValve3.isOpen ? '1' : '0'),
            (reagentValve4.isOpen ? '1' : '0'));
    logData("RV", rvBuffer);
  } else {
    response->println(F("Invalid reagent valve command. Use: R <1-4> <0/1>"));
  }
}

// Command callback to control media valves ("M <valve number> <0/1>")
void cmd_set_media_valve(char *args, Stream *response) {
  int valveNumber = -1;
  int valveState = -1;
  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_MEDIA_VALVES &&
      (valveState == 0 || valveState == 1)) {
    bool state = (valveState == 1);
    switch (valveNumber) {
      case 1:
        mediaValve1 = state ? openValve(mediaValve1) : closeValve(mediaValve1);
        response->print(F("Media valve 1 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 2:
        mediaValve2 = state ? openValve(mediaValve2) : closeValve(mediaValve2);
        response->print(F("Media valve 2 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 3:
        mediaValve3 = state ? openValve(mediaValve3) : closeValve(mediaValve3);
        response->print(F("Media valve 3 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 4:
        mediaValve4 = state ? openValve(mediaValve4) : closeValve(mediaValve4);
        response->print(F("Media valve 4 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
    }
    // Log media valve states in compact format.
    char mvBuffer[6];
    sprintf(mvBuffer, "%c%c%c%c", 
            (mediaValve1.isOpen ? '1' : '0'),
            (mediaValve2.isOpen ? '1' : '0'),
            (mediaValve3.isOpen ? '1' : '0'),
            (mediaValve4.isOpen ? '1' : '0'));
    logData("MV", mvBuffer);
  } else {
    response->println(F("Invalid media valve command. Use: M <1-4> <0/1>"));
  }
}

// Command callback to control waste valves ("W <valve number> <0/1>")
void cmd_set_waste_valve(char *args, Stream *response) {
  int valveNumber = -1;
  int valveState = -1;
  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 &&
      valveNumber >= 1 && valveNumber <= NUM_WASTE_VALVES &&
      (valveState == 0 || valveState == 1)) {
    bool state = (valveState == 1);
    switch (valveNumber) {
      case 1:
        wasteValve1 = state ? openValve(wasteValve1) : closeValve(wasteValve1);
        response->print(F("Waste valve 1 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 2:
        wasteValve2 = state ? openValve(wasteValve2) : closeValve(wasteValve2);
        response->print(F("Waste valve 2 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 3:
        wasteValve3 = state ? openValve(wasteValve3) : closeValve(wasteValve3);
        response->print(F("Waste valve 3 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
      case 4:
        wasteValve4 = state ? openValve(wasteValve4) : closeValve(wasteValve4);
        response->print(F("Waste valve 4 set to "));
        response->println(state ? F("open") : F("closed"));
        break;
    }
    // Log waste valve states in compact format.
    char wvBuffer[6];
    sprintf(wvBuffer, "%c%c%c%c", 
            (wasteValve1.isOpen ? '1' : '0'),
            (wasteValve2.isOpen ? '1' : '0'),
            (wasteValve3.isOpen ? '1' : '0'),
            (wasteValve4.isOpen ? '1' : '0'));
    logData("WV", wvBuffer);
  } else {
    response->println(F("Invalid waste valve command. Use: W <1-4> <0/1>"));
  }
}

// =====================
// Commander API Setup
// =====================
Commander commander;

Commander::API_t API_tree[] = {
  apiElement("LF", "Set log frequency: LF <ms>", cmd_set_log_frequency),
  apiElement("FN", "Fan: FN <0/1> (0 = off, 1 = on)", cmd_fan),
  apiElement("R", "Reagent valve: R <1-4> <0/1>", cmd_set_reagent_valve),
  apiElement("M", "Media valve: M <1-4> <0/1>", cmd_set_media_valve),
  apiElement("W", "Waste valve: W <1-4> <0/1> (0 = close, 1 = open)", cmd_set_waste_valve)
};

// =====================
// Setup Function
// =====================
void setup() {
  // Initialize Serial communication.
  Serial.begin(115200);
  Serial.println(F("System ready."));

  // Set up fan.
  fanSetup(fan);

  // Setup reagent valves.
  valveSetup(reagentValve1);
  valveSetup(reagentValve2);
  valveSetup(reagentValve3);
  valveSetup(reagentValve4);

  // Setup media valves.
  valveSetup(mediaValve1);
  valveSetup(mediaValve2);
  valveSetup(mediaValve3);
  valveSetup(mediaValve4);

  // Setup waste valves.
  valveSetup(wasteValve1);
  valveSetup(wasteValve2);
  valveSetup(wasteValve3);
  valveSetup(wasteValve4);

  // Setup overflow sensors.
  binarySensorSetup(overflowSensorTrough1);
  binarySensorSetup(overflowSensorTrough2);
  binarySensorSetup(overflowSensorTrough3);
  binarySensorSetup(overflowSensorTrough4);

  // Setup bubble sensors.
  binarySensorSetup(reagent1BubbleSensor1);
  binarySensorSetup(reagent1BubbleSensor2);
  binarySensorSetup(reagent1BubbleSensor3);
  binarySensorSetup(reagent1BubbleSensor4);

  // Attach and initialize the Commander API.
  commander.attachTree(API_tree);
  commander.init();
}

// =====================
// Main Loop
// =====================
void loop() {
  unsigned long currentTime = millis();
  
  // Process serial commands using your custom handler.
  handleSerialCommands();
  
  // Monitor sensors at a fast rate.
  monitorOverflowSensors(currentTime);
  monitorReagentBubbleSensors(currentTime);
  monitorWasteLineSensors(currentTime);
  monitorWasteBottleSensors(currentTime);
  monitorWasteVacuumSensors(currentTime);



  // Periodically log overall system state.
  if (currentTime - logging.previousLogTime >= logging.logInterval) {
    logging.previousLogTime = currentTime;
    logSystemState();
  }
}

// =====================
// Helper Function: Handle Serial Input for Commands
// =====================
void handleSerialCommands() {
  static char commandBuffer[32]; // Buffer for incoming command
  static uint8_t commandIndex = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      commandBuffer[commandIndex] = '\0'; // Terminate the string
      commander.execute(commandBuffer, &Serial);
      commandIndex = 0; // Reset buffer index
    }
    else if (c != '\r') {
      commandBuffer[commandIndex++] = c;
      if (commandIndex >= sizeof(commandBuffer) - 1) {
        commandIndex = sizeof(commandBuffer) - 1;
      }
    }
  }
}

// =====================
// Helper Function: Log Overall System State
// =====================
// Format: [LOG] F<fanState>, RV<r1><r2><r3><r4>, MV<m1><m2><m3><m4>, WV<w1><w2><w3><w4>, BS<b1><b2><b3><b4>, OS<o1><o2><o3><o4>
void logSystemState() {
  char buffer[100];
  // Fan state
  char fState = (digitalRead(fan.relayPin) == HIGH ? '1' : '0');

  // Reagent valve states
  char rv1 = (reagentValve1.isOpen ? '1' : '0');
  char rv2 = (reagentValve2.isOpen ? '1' : '0');
  char rv3 = (reagentValve3.isOpen ? '1' : '0');
  char rv4 = (reagentValve4.isOpen ? '1' : '0');

  // Media valve states
  char mv1 = (mediaValve1.isOpen ? '1' : '0');
  char mv2 = (mediaValve2.isOpen ? '1' : '0');
  char mv3 = (mediaValve3.isOpen ? '1' : '0');
  char mv4 = (mediaValve4.isOpen ? '1' : '0');

  // Waste valve states
  char wv1 = (wasteValve1.isOpen ? '1' : '0');
  char wv2 = (wasteValve2.isOpen ? '1' : '0');
  char wv3 = (wasteValve3.isOpen ? '1' : '0');
  char wv4 = (wasteValve4.isOpen ? '1' : '0');

  // Bubble sensor states (BS)
  char bs1 = (readBinarySensor(reagent1BubbleSensor1) ? '1' : '0');
  char bs2 = (readBinarySensor(reagent1BubbleSensor2) ? '1' : '0');
  char bs3 = (readBinarySensor(reagent1BubbleSensor3) ? '1' : '0');
  char bs4 = (readBinarySensor(reagent1BubbleSensor4) ? '1' : '0');

  // Overflow sensor states (OS)
  char os1 = (readBinarySensor(overflowSensorTrough1) ? '1' : '0');
  char os2 = (readBinarySensor(overflowSensorTrough2) ? '1' : '0');
  char os3 = (readBinarySensor(overflowSensorTrough3) ? '1' : '0');
  char os4 = (readBinarySensor(overflowSensorTrough4) ? '1' : '0');

  sprintf(buffer, "[LOG] F%c, RV%c%c%c%c, MV%c%c%c%c, WV%c%c%c%c, BS%c%c%c%c, OS%c%c%c%c",
          fState, 
          rv1, rv2, rv3, rv4, 
          mv1, mv2, mv3, mv4, 
          wv1, wv2, wv3, wv4,
          bs1, bs2, bs3, bs4,
          os1, os2, os3, os4);
  Serial.println(buffer);
}

// =====================
// Helper Function: Monitor Overflow Sensors at Fast Intervals
// =====================
void monitorOverflowSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  // Check every 25ms
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      bool triggered = readBinarySensor(overflowSensors[i]);
      if (triggered) {
        // Add your overflow handling here if needed.
        // For example: handleOverflowCondition(i + 1);
      }
    }
  }
}

// =====================
// Helper Function: Monitor Reagent Bubble Sensors at Fast Intervals
// =====================
void monitorReagentBubbleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  // Check every 25ms
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_REAGENT_BUBBLE_SENSORS; i++) {
      bool detected = readBinarySensor(reagentBubbleSensors[i]);
      if (detected) {
        // Add your bubble handling here if needed.
        // For example: handleBubbleCondition(i + 1);
      }
    }
  }
}

// =====================
// Monitor Waste Line Sensors
// =====================
void monitorWasteLineSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  // Check every 25 ms
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_LINE_SENSORS; i++) {
      bool triggered = readBinarySensor(wasteLineSensors[i]);
      if (triggered) {
        // Insert your handling logic for waste line sensor i+1 here.
        // For example: handleWasteLineCondition(i + 1);
      }
    }
  }
}

// =====================
// Monitor Waste Bottle Sensors
// =====================
void monitorWasteBottleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  // Check every 25 ms
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_BOTTLE_SENSORS; i++) {
      bool triggered = readBinarySensor(wasteBottleSensors[i]);
      if (triggered) {
        // Insert your handling logic for waste bottle sensor i+1 here.
        // For example: handleWasteBottleCondition(i + 1);
      }
    }
  }
}

// =====================
// Monitor Waste Vacuum Sensors
// =====================
void monitorWasteVacuumSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  // Check every 25 ms
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_VACUUM_SENSORS; i++) {
      bool triggered = readBinarySensor(wasteVacuumSensors[i]);
      if (triggered) {
        // Insert your handling logic for waste vacuum sensor i+1 here.
        // For example: handleWasteVacuumCondition(i + 1);
      }
    }
  }
}

