/*
  Bulk Dispense v2.0 Code
  ------------------------
  This sketch controls a bulk dispense system using I2C-based flow sensors,
  proportional and on/off valves, binary sensors, and a Controllino Maxi PLC.

  Key Functionalities:
  - Dispensing reagent through controlled valves
  - Monitoring flow rates, pressure, temperature, and liquid levels
  - Handling errors and sensor timeouts
  - Logging system states for diagnostics
*/

//////////////////////////////////////////////////////////////
//                      INCLUDE LIBRARIES                   //
//////////////////////////////////////////////////////////////
#include <Controllino.h>
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include <Wire.h>
#include <string.h>
#include <ctype.h>
#include "Adafruit_SHT31.h"

//////////////////////////////////////////////////////////////
//              GLOBAL CONSTANTS & PIN DEFINITIONS          //
//////////////////////////////////////////////////////////////

// **General**
#define COMMAND_SIZE 30
#define ENCLOSURE_TEMP_SETPOINT 30.0  // °C for fan activation

// **Fan**
#define FAN_CONTROL_PIN CONTROLLINO_R6

// **Proportional Valve**
#define PROPORTIONAL_VALVE_CONTROL_PIN CONTROLLINO_AO0
#define PROPORTIONAL_VALVE_FEEDBACK_PIN CONTROLLINO_AI13

// **Reagent Valves**
#define NUM_REAGENT_VALVES 4
const uint8_t REAGENT_VALVES[NUM_REAGENT_VALVES] = {
  CONTROLLINO_R0, CONTROLLINO_R1, CONTROLLINO_R2, CONTROLLINO_R3
};

// **Media Valves**
#define NUM_MEDIA_VALVES 4
const uint8_t MEDIA_VALVES[NUM_MEDIA_VALVES] = {
  CONTROLLINO_DO0, CONTROLLINO_DO1, CONTROLLINO_DO2, CONTROLLINO_DO3
};

// **Waste Valves**
#define NUM_WASTE_VALVES 4
const uint8_t WASTE_VALVES[NUM_WASTE_VALVES] = {
  CONTROLLINO_R4, CONTROLLINO_R5, CONTROLLINO_R8, CONTROLLINO_R9
};

// **Overflow Sensors**
#define NUM_OVERFLOW_SENSORS 4
const uint8_t OVERFLOW_SENSORS[NUM_OVERFLOW_SENSORS] = {
  CONTROLLINO_DI0, CONTROLLINO_DI1, CONTROLLINO_DI2, CONTROLLINO_DI3
};

// **Bubble Sensors**
#define NUM_REAGENT_BUBBLE_SENSORS 4
const uint8_t BUBBLE_SENSORS[NUM_REAGENT_BUBBLE_SENSORS] = {
  CONTROLLINO_AI0, CONTROLLINO_AI1, CONTROLLINO_AI2, CONTROLLINO_AI3
};

// **Waste Line Sensors**
#define NUM_WASTE_LINE_SENSORS 2
const uint8_t WASTE_LINE_SENSORS[NUM_WASTE_LINE_SENSORS] = {
  CONTROLLINO_AI4, CONTROLLINO_AI5
};

// **Waste Bottle Sensors**
#define NUM_WASTE_BOTTLE_SENSORS 2
const uint8_t WASTE_BOTTLE_SENSORS[NUM_WASTE_BOTTLE_SENSORS] = {
  CONTROLLINO_AI6, CONTROLLINO_AI7
};

// **Waste Vacuum Sensors**
#define NUM_WASTE_VACUUM_SENSORS 2
const uint8_t WASTE_VACUUM_SENSORS[NUM_WASTE_VACUUM_SENSORS] = {
  CONTROLLINO_AI8, CONTROLLINO_AI9
};

// **Enclosure Liquid Sensor**
#define ENCLOSURE_LIQUID_SENSOR_PIN CONTROLLINO_AI10

// **Pressure Sensor**
#define PRESSURE_SENSOR_PIN CONTROLLINO_AI12

// **I2C Addresses & Flow Sensors**
#define MULTIPLEXER_ADDR 0x70
#define TEMP_HUM_SENSOR_ADDR 0x44
#define TEMP_HUM_SENSOR_CHANNEL 4
#define NUM_FLOW_SENSORS 4
#define FLOW_SENSOR_CMD 0x3608  // Measurement command for flow sensors


//////////////////////////////////////////////////////////////
//                 STRUCT DEFINITIONS                       //
//////////////////////////////////////////////////////////////
struct OnOffValve;
struct FanControl;
struct LoggingManagement;
struct BinarySensor;
struct ProportionalValve;
struct PressureSensor;
struct TempHumidity;
struct FlowData;
struct FlowSensor;
struct ValveControl;

//////////////////////////////////////////////////////////////
//            COMMAND FUNCTION PROTOTYPES                   //
//////////////////////////////////////////////////////////////
void cmd_set_log_frequency(char* args, CommandCaller* caller);
void cmd_fan(char* args, CommandCaller* caller);
void cmd_fan_auto(char* args, CommandCaller* caller);
void cmd_set_reagent_valve(char* args, CommandCaller* caller);
void cmd_set_media_valve(char* args, CommandCaller* caller);
void cmd_set_waste_valve(char* args, CommandCaller* caller);
void cmd_calibrate_pressure_valve(char* args, CommandCaller* caller);
void cmd_start_flow_sensor_manually(char* args, CommandCaller* caller);
void cmd_stop_flow_sensor_manually(char* args, CommandCaller* caller);
void cmd_reset_flow_dispense(char* args, CommandCaller* caller);
void cmd_reset_flow_total(char* args, CommandCaller* caller);
void cmd_reset_i2c(char* args, CommandCaller* caller);
void cmd_stop_dispense(char* args, CommandCaller* caller);

//////////////////////////////////////////////////////////////
//            HELPER FUNCTION PROTOTYPES                    //
//////////////////////////////////////////////////////////////
char* trimLeadingSpaces(char* str);
OnOffValve createValve(uint8_t pin);
void processMultipleCommands(char* commandLine, Stream* stream);
void handleSerialCommands();
void logSystemState();
void monitorOverflowSensors(unsigned long currentTime);
void monitorFlowSensors(unsigned long currentTime);
void monitorReagentBubbleSensors(unsigned long currentTime);
void monitorWasteLineSensors(unsigned long currentTime);
void monitorWasteBottleSensors(unsigned long currentTime);
void monitorWasteVacuumSensors(unsigned long currentTime);
void monitorEnclosureLiquidSensor(unsigned long currentTime);
bool isFlowSensorConnected(FlowSensor& sensor);
void monitorFlowSensorConnections();
void calibrateProportionalValve();
void resetI2CBus();
void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel);
bool tempHumSensorInit();
TempHumidity readTempHumidity();
void monitorEnclosureTemp();
void resetFlowSensorDispenseVolume(FlowSensor& sensor);
void resetFlowSensorTotalVolume(FlowSensor& sensor);
bool stopFlowSensorMeasurement(FlowSensor& sensor);
bool startFlowSensorMeasurement(FlowSensor& sensor);
void handleOverflowCondition(int triggeredTrough);
void handleTimeoutCondition(int troughNumber);
void closeDispenseValves(int troughNumber);
void openDispenseValves(int troughNumber);
bool checkAndSetPressure(float thresholdPressure, float valvePosition, unsigned long timeout);
void stopDispenseOperation(int troughNumber, CommandCaller* caller);

//////////////////////////////////////////////////////////////
//                   GLOBAL VARIABLES                       //
//////////////////////////////////////////////////////////////

// =====================
// Logging Management
// =====================
struct LoggingManagement {
  unsigned long previousLogTime;
  unsigned long logInterval;
};
LoggingManagement logging = { 0, 250 };  // Default log interval: 250ms

// =====================
// Temperature & Humidity Sensor
// =====================
struct TempHumidity {
  float temperature;
  float humidity;
  bool valid;
};

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// =====================
// Fan Control
// =====================
struct FanControl {
  uint8_t relayPin;
};
const FanControl fan = { FAN_CONTROL_PIN };

// =====================
// Proportional Valve Control
// =====================
struct ProportionalValve {
  byte controlPin;
  byte feedbackPin;
  float controlVoltage;
};

ProportionalValve proportionalValve = {
  PROPORTIONAL_VALVE_CONTROL_PIN,
  PROPORTIONAL_VALVE_FEEDBACK_PIN,
  0.0
};

// =====================
// Pressure Sensor
// =====================
struct PressureSensor {
  byte analogPin;
  float minPressure;
  float maxPressure;
};
PressureSensor pressureSensor = { PRESSURE_SENSOR_PIN, 0, 87 };  // 0 to 87 psi

// =====================
// Flow Sensor Data Structure (for output)
// =====================
struct FlowData {
  float flowRate;        // mL/min
  float temperature;     // °C
  int highFlowFlag;      // 1 if high flow, else 0
  float dispenseVolume;  // Volume increment since last read
  float totalVolume;     // Cumulative volume
  bool isValidReading;   // Flag indicating if the reading is valid
};

// =====================
// Flow Sensors
// =====================
struct FlowSensor {
  uint8_t multiplexerAddr;
  uint8_t sensorAddr;
  uint8_t channel;
  uint16_t measurementCmd;
  bool sensorInitialized;
  bool sensorStopped;
  unsigned long lastUpdateTime;
  float flowRate;
  float temperature;
  int highFlowFlag;
  int sensorConnected;
  float dispenseVolume;
  float totalVolume;
  bool isValidReading;
};

// =====================
// Valve Control Structure (for managing dispense operations)
// =====================

struct ValveControl {
  bool isDispensing = false;
  bool manualControl = false;
  bool isPriming = false;
  bool fillMode = false;
  bool isDraining = false;
  float targetVolume = -1.0;
  float lastFlowValue = 0.0;
  unsigned long lastFlowCheckTime = 0;
  unsigned long lastFlowChangeTime = 0;
  unsigned long fillCheckTime = 0;
  int dispensingValveNumber = -1;
};


// =====================
// On/Off Valves
// =====================
struct OnOffValve {
  uint8_t controlPin;
  bool isOpen;
};


// Create valve instances for reagent, media, and waste
// Using arrays from your pin definitions:
OnOffValve reagentValves[NUM_REAGENT_VALVES] = {
  { REAGENT_VALVES[0], false },
  { REAGENT_VALVES[1], false },
  { REAGENT_VALVES[2], false },
  { REAGENT_VALVES[3], false }
};

OnOffValve mediaValves[NUM_MEDIA_VALVES] = {
  { MEDIA_VALVES[0], false },
  { MEDIA_VALVES[1], false },
  { MEDIA_VALVES[2], false },
  { MEDIA_VALVES[3], false }
};

OnOffValve wasteValves[NUM_WASTE_VALVES] = {
  { WASTE_VALVES[0], false },
  { WASTE_VALVES[1], false },
  { WASTE_VALVES[2], false },
  { WASTE_VALVES[3], false }
};

// =====================
// Individual On/Off Valve Instances
// =====================
OnOffValve reagentValve1 = createValve(REAGENT_VALVES[0]);
OnOffValve reagentValve2 = createValve(REAGENT_VALVES[1]);
OnOffValve reagentValve3 = createValve(REAGENT_VALVES[2]);
OnOffValve reagentValve4 = createValve(REAGENT_VALVES[3]);

OnOffValve mediaValve1 = createValve(MEDIA_VALVES[0]);
OnOffValve mediaValve2 = createValve(MEDIA_VALVES[1]);
OnOffValve mediaValve3 = createValve(MEDIA_VALVES[2]);
OnOffValve mediaValve4 = createValve(MEDIA_VALVES[3]);

OnOffValve wasteValve1 = createValve(WASTE_VALVES[0]);
OnOffValve wasteValve2 = createValve(WASTE_VALVES[1]);
OnOffValve wasteValve3 = createValve(WASTE_VALVES[2]);
OnOffValve wasteValve4 = createValve(WASTE_VALVES[3]);


// --- Proportional Valve Calibration Variable ---
float proportionalValveMaxFeedback = 0.0;  // Will be updated during calibration

// =====================
// Binary Sensors
// =====================
struct BinarySensor {
  uint8_t inputPin;
  bool activeHigh;
};

// --- Flow Sensor Instances ---
// Here we name them flow1..flow4 (update code references accordingly)
FlowSensor flow1 = { MULTIPLEXER_ADDR, 0x08, 0, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };
FlowSensor flow2 = { MULTIPLEXER_ADDR, 0x08, 1, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };
FlowSensor flow3 = { MULTIPLEXER_ADDR, 0x08, 2, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };
FlowSensor flow4 = { MULTIPLEXER_ADDR, 0x08, 3, FLOW_SENSOR_CMD, false, true, 0, 0.0, 0.0, 0, 0, 0.0, 0.0, false };

FlowSensor* flowSensors[NUM_FLOW_SENSORS] = { &flow1, &flow2, &flow3, &flow4 };

// --- Valve Control (for managing dispense operations) ---
ValveControl valveControls[NUM_OVERFLOW_SENSORS] = {};

// --- Binary Sensor Instances ---
// (Make sure the type name is “BinarySensor”, not “inarySensor”)
BinarySensor overflowSensors[NUM_OVERFLOW_SENSORS] = {
  { OVERFLOW_SENSORS[0], true },
  { OVERFLOW_SENSORS[1], true },
  { OVERFLOW_SENSORS[2], true },
  { OVERFLOW_SENSORS[3], true }
};

BinarySensor reagentBubbleSensors[NUM_REAGENT_BUBBLE_SENSORS] = {
  { BUBBLE_SENSORS[0], true },
  { BUBBLE_SENSORS[1], true },
  { BUBBLE_SENSORS[2], true },
  { BUBBLE_SENSORS[3], true }
};

BinarySensor wasteLineSensors[NUM_WASTE_LINE_SENSORS] = {
  { WASTE_LINE_SENSORS[0], true },
  { WASTE_LINE_SENSORS[1], true }
};

BinarySensor wasteBottleSensors[NUM_WASTE_BOTTLE_SENSORS] = {
  { WASTE_BOTTLE_SENSORS[0], true },
  { WASTE_BOTTLE_SENSORS[1], true }
};

BinarySensor wasteVacuumSensors[NUM_WASTE_VACUUM_SENSORS] = {
  { WASTE_VACUUM_SENSORS[0], true },
  { WASTE_VACUUM_SENSORS[1], true }
};

BinarySensor enclosureLiquidSensor = { ENCLOSURE_LIQUID_SENSOR_PIN, false };

//////////////////////////////////////////////////////////////
//                  FLOW SENSOR FUNCTIONS                   //
//////////////////////////////////////////////////////////////

FlowSensor createFlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd) {
  FlowSensor sensor;
  sensor.multiplexerAddr = muxAddr;
  sensor.sensorAddr = addr;
  sensor.channel = chan;
  sensor.measurementCmd = cmd;
  sensor.sensorInitialized = false;
  sensor.sensorStopped = true;
  sensor.lastUpdateTime = 0;
  sensor.flowRate = 0.0;
  sensor.temperature = 0.0;
  sensor.highFlowFlag = 0;
  sensor.sensorConnected = 0;
  sensor.dispenseVolume = 0.0;
  sensor.totalVolume = 0.0;
  sensor.isValidReading = false;
  return sensor;
}

bool initializeFlowSensor(FlowSensor& sensor) {
  static int resetAttempt = 0;

  // **Step 1: Select the correct multiplexer channel**
  Serial.print(F("[DEBUG] Selecting multiplexer channel "));
  Serial.println(sensor.channel);
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);

  // **Step 2: Check if Flow Sensor is Connected**
  if (!isFlowSensorConnected(sensor)) {
    Serial.print(F("[ERROR] Flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" is not connected. Aborting initialization."));
    return false;
  }

  // **Step 3: If this is a retry attempt, send a soft reset**
  if (resetAttempt > 0) {
    Serial.print(F("[DEBUG] Sending soft reset to sensor on channel "));
    Serial.println(sensor.channel);

    Wire.beginTransmission(sensor.sensorAddr);
    Wire.write(0x00);
    Wire.write(0x06);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("[WARNING] Soft reset command was not acknowledged."));
      return false;
    }

    delay(25);  // Allow reset to take effect

    // **Recheck sensor after reset**
    if (!isFlowSensorConnected(sensor)) {
      Serial.println(F("[ERROR] Sensor did not respond after soft reset."));
      return false;
    }
  }

  // **Step 4: Start Continuous Measurement Mode**
  Serial.print(F("[DEBUG] Sending start measurement command to sensor on channel "));
  Serial.println(sensor.channel);

  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(sensor.measurementCmd >> 8);
  Wire.write(sensor.measurementCmd & 0xFF);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[ERROR] Failed to start measurement mode."));

    // **Only attempt a reset if the first attempt failed**
    if (resetAttempt == 0) {
      resetAttempt++;
      return initializeFlowSensor(sensor);
    }

    return false;  // Give up after reset attempt
  }

  delay(100);  // Allow sensor to stabilize

  // **Step 5: Sensor Successfully Initialized**
  sensor.sensorInitialized = true;
  sensor.sensorConnected = 1;
  sensor.lastUpdateTime = millis();
  sensor.dispenseVolume = 0.0;
  resetAttempt = 0;  // Reset failure counter after success

  Serial.print(F("[MESSAGE] Flow sensor on channel "));
  Serial.print(sensor.channel);
  Serial.println(F(" successfully initialized."));

  return true;
}

bool readFlowSensorData(FlowSensor& sensor) {
  static int softResetAttempt = 0;

  if (!sensor.sensorInitialized || sensor.sensorStopped) {
    sensor.flowRate = -1;
    sensor.temperature = -1;
    sensor.highFlowFlag = -1;
    if (sensor.totalVolume == 0.0) sensor.dispenseVolume = 0.0;
    return false;
  }

  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  Wire.requestFrom(sensor.sensorAddr, (uint8_t)9);

  if (Wire.available() < 9) {
    Serial.print(F("[ERROR] Not enough bytes received from flow sensor on channel "));
    Serial.println(sensor.channel);

    if (softResetAttempt < 2) {
      Serial.println(F("[WARNING] Attempting soft reset to recover..."));
      Wire.beginTransmission(sensor.sensorAddr);
      Wire.write(0x00);
      Wire.write(0x06);
      if (Wire.endTransmission() == 0) {
        delay(25);
        softResetAttempt++;
        return false;
      }
    }

    Serial.println(F("[ERROR] Multiple failures. Sensor will remain in error state."));
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    sensor.sensorConnected = 0;
    softResetAttempt = 0;
    return false;
  }

  softResetAttempt = 0;
  uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();
  uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();
  uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();

  sensor.flowRate = ((int16_t)flowRaw) / 32.0;
  sensor.temperature = ((int16_t)tempRaw) / 200.0;
  sensor.highFlowFlag = (auxRaw & 0x02) ? 1 : 0;
  sensor.sensorConnected = 1;

  if (sensor.flowRate < 0) sensor.flowRate = 0.0;

  unsigned long currentTime = millis();
  if (sensor.lastUpdateTime > 0) {
    float elapsedMinutes = (currentTime - sensor.lastUpdateTime) / 60000.0;
    float increment = sensor.flowRate * elapsedMinutes;
    sensor.dispenseVolume += increment;
    sensor.totalVolume += increment;
  }

  sensor.lastUpdateTime = currentTime;
  sensor.isValidReading = true;
  return true;
}

//////////////////////////////////////////////////////////////
//                ON/OFF VALVE CONTROL                      //
//////////////////////////////////////////////////////////////

OnOffValve createValve(uint8_t pin) {
  return { pin, false };
}

void valveSetup(const OnOffValve& valve) {
  pinMode(valve.controlPin, OUTPUT);
  digitalWrite(valve.controlPin, LOW);
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

bool isValveOpen(const OnOffValve& valve) {
  return valve.isOpen;
}

//////////////////////////////////////////////////////////////
//                PROPORTIONAL VALVE CONTROL                //
//////////////////////////////////////////////////////////////

ProportionalValve createProportionalValve(byte controlPin, byte feedbackPin) {
  return { controlPin, feedbackPin, 0.0 };
}

void proportionalValveSetup(const ProportionalValve& valve) {
  pinMode(valve.controlPin, OUTPUT);
  pinMode(valve.feedbackPin, INPUT);
}

ProportionalValve setValvePosition(ProportionalValve valve, float percentage) {
  percentage = constrain(percentage, 0, 100);
  valve.controlVoltage = (percentage / 100.0) * 10.0;
  int pwmValue = map(percentage, 0, 100, 0, 255);
  analogWrite(valve.controlPin, pwmValue);
  return valve;
}

float getValveFeedback(const ProportionalValve& valve) {
  int analogValue = analogRead(valve.feedbackPin);
  return map(analogValue, 0, 1023, 0, 10000) / 1000.0;
}

//////////////////////////////////////////////////////////////
//                FAN CONTROL                               //
//////////////////////////////////////////////////////////////

void setFanState(const FanControl& config, bool state) {
  static bool lastState = false;
  if (lastState != state) {
    digitalWrite(config.relayPin, state ? HIGH : LOW);
    Serial.print(F("[MESSAGE] Fan state set to "));
    Serial.println(state ? F("ON") : F("OFF"));
    lastState = state;
  }
}

void printFanState(bool state) {
  Serial.print(F("[MESSAGE] Fan is "));
  Serial.println(state ? F("ON") : F("OFF"));
}

void fanSetup(const FanControl& fc) {
  pinMode(fc.relayPin, OUTPUT);
  digitalWrite(fc.relayPin, LOW);
  Serial.println(F("[MESSAGE] Fan initialized and set to OFF"));
}

//////////////////////////////////////////////////////////////
//                I2C & SENSOR FUNCTIONS                    //
//////////////////////////////////////////////////////////////

void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel) {
  Wire.beginTransmission(multiplexerAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool tempHumSensorInit() {
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  return sht31.begin(TEMP_HUM_SENSOR_ADDR);
}

TempHumidity readTempHumidity() {
  TempHumidity data;
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  data.temperature = sht31.readTemperature();
  data.humidity = sht31.readHumidity();
  data.valid = !(isnan(data.temperature) || isnan(data.humidity));
  return data;
}

//////////////////////////////////////////////////////////////
//                   LOGGING FUNCTIONS                      //
//////////////////////////////////////////////////////////////

void logData(const char* module, const char* message) {
  Serial.print(F("[LOG] "));
  Serial.print(module);
  Serial.print(F(" - "));
  Serial.println(message);
}

//////////////////////////////////////////////////////////////
//                BINARY SENSOR FUNCTIONS                   //
//////////////////////////////////////////////////////////////

BinarySensor createBinarySensor(uint8_t pin, bool activeHigh) {
  return { pin, activeHigh };
}

void binarySensorSetup(const BinarySensor& sensor) {
  pinMode(sensor.inputPin, INPUT);
}

bool readBinarySensor(const BinarySensor& sensor) {
  int reading = digitalRead(sensor.inputPin);
  return sensor.activeHigh ? (reading == HIGH) : (reading == LOW);
}

//////////////////////////////////////////////////////////////
//                PRESSURE SENSOR FUNCTIONS                 //
//////////////////////////////////////////////////////////////

PressureSensor createPressureSensor(byte analogPin, float minPressure, float maxPressure) {
  return { analogPin, minPressure, maxPressure };
}

void pressureSensorSetup(const PressureSensor& sensor) {
  pinMode(sensor.analogPin, INPUT);
}

float readPressureVoltage(const PressureSensor& sensor) {
  int analogValue = analogRead(sensor.analogPin);
  return (analogValue / 1023.0) * 10.0;
}

float readPressure(const PressureSensor& sensor) {
  float voltage = readPressureVoltage(sensor);
  return (voltage / 10.0) * sensor.maxPressure;
}

//////////////////////////////////////////////////////////////
//                COMMANDER API FUNCTIONS                   //
//////////////////////////////////////////////////////////////

/**
 * @brief Set the logging frequency.
 *
 * Parses the input argument to update the log interval (in milliseconds).
 * If the argument is valid (a positive number), the logging interval is updated;
 * otherwise, an error message is printed.
 *
 * @param args A C-string containing the new log interval value.
 * @param caller Pointer to the CommandCaller used to send responses.
 */
void cmd_set_log_frequency(char* args, CommandCaller* caller) {
  int newInterval = -1;

  if (sscanf(args, "%d", &newInterval) == 1 && newInterval > 0) {
    logging.logInterval = newInterval;
    caller->print(F("[MESSAGE] Log frequency set to "));
    caller->print(newInterval);
    caller->println(F(" ms"));
  } else {
    caller->println(F("[ERROR] Invalid log frequency. Use: LF <positive number>"));
  }
}


bool fanAutoMode = true;  // true = auto control enabled, false = manual override active

/**
 * @brief Manually set the fan state.
 *
 * Reads a 0/1 value from the input arguments and sets the fan state accordingly.
 * It also disables the automatic fan control mode upon manual intervention.
 *
 * @param args A C-string representing the desired fan state (0 for OFF, 1 for ON).
 * @param caller Pointer to the CommandCaller used for sending output messages.
 */
void cmd_fan(char* args, CommandCaller* caller) {
  int fanState = -1;

  if (sscanf(args, "%d", &fanState) == 1 && (fanState == 0 || fanState == 1)) {
    bool state = (fanState == 1);
    setFanState(fan, state);
    printFanState(state);
    // Disable auto mode on manual command.
    fanAutoMode = false;
    caller->println(F("[MESSAGE] Fan manual override active. Use FNAUTO to re-enable auto control."));
  } else {
    caller->println(F("[ERROR] Invalid fan command. Use: FN <0/1>"));
  }
}


/**
 * @brief Re-enable automatic fan control.
 *
 * Sets the global fanAutoMode variable to true and informs the caller.
 *
 * @param args Unused argument.
 * @param caller Pointer to the CommandCaller used for output.
 */
void cmd_fan_auto(char* args, CommandCaller* caller) {
  fanAutoMode = true;
  caller->println(F("[MESSAGE] Fan auto control re-enabled."));
}


/**
 * @brief Set the state of a reagent valve.
 *
 * Parses the command arguments to determine which reagent valve to set and whether
 * it should be opened or closed.
 *
 * @param args A C-string with two numbers: valve number (1-4) and valve state (0 for closed, 1 for open).
 * @param caller Pointer to the CommandCaller used to send response messages.
 */
void cmd_set_reagent_valve(char* args, CommandCaller* caller) {
  int valveNumber = -1, valveState = -1;

  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 && valveNumber >= 1 && valveNumber <= NUM_REAGENT_VALVES && (valveState == 0 || valveState == 1)) {

    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Reagent valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");

    switch (valveNumber) {
      case 1: reagentValve1 = state ? openValve(reagentValve1) : closeValve(reagentValve1); break;
      case 2: reagentValve2 = state ? openValve(reagentValve2) : closeValve(reagentValve2); break;
      case 3: reagentValve3 = state ? openValve(reagentValve3) : closeValve(reagentValve3); break;
      case 4: reagentValve4 = state ? openValve(reagentValve4) : closeValve(reagentValve4); break;
    }
  } else {
    caller->println(F("[ERROR] Invalid reagent valve command. Use: R <1-4> <0/1>"));
  }
}


/**
 * @brief Set the state of a media valve.
 *
 * Parses the input arguments to determine which media valve (1–4) should be set and
 * whether it should be opened or closed.
 *
 * @param args A C-string containing two numbers: the valve number (1–4) and the desired state (0 for closed, 1 for open).
 * @param caller Pointer to the CommandCaller used for sending feedback.
 */
void cmd_set_media_valve(char* args, CommandCaller* caller) {
  int valveNumber = -1, valveState = -1;

  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 && valveNumber >= 1 && valveNumber <= NUM_MEDIA_VALVES && (valveState == 0 || valveState == 1)) {

    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Media valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? F("OPEN") : F("CLOSED"));

    switch (valveNumber) {
      case 1: mediaValve1 = state ? openValve(mediaValve1) : closeValve(mediaValve1); break;
      case 2: mediaValve2 = state ? openValve(mediaValve2) : closeValve(mediaValve2); break;
      case 3: mediaValve3 = state ? openValve(mediaValve3) : closeValve(mediaValve3); break;
      case 4: mediaValve4 = state ? openValve(mediaValve4) : closeValve(mediaValve4); break;
    }
  } else {
    caller->println(F("[ERROR] Invalid media valve command. Use: M <1-4> <0/1>"));
  }
}


/**
 * @brief Set the state of a waste valve.
 *
 * Reads two numbers from the input arguments: the valve number (1–4) and the desired state (0 for closed, 1 for open).
 * Then sets the state of the corresponding waste valve.
 *
 * @param args A C-string containing the valve number and state.
 * @param caller Pointer to the CommandCaller used for output.
 */
void cmd_set_waste_valve(char* args, CommandCaller* caller) {
  int valveNumber = -1, valveState = -1;

  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 && valveNumber >= 1 && valveNumber <= NUM_WASTE_VALVES && (valveState == 0 || valveState == 1)) {

    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Waste valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? F("OPEN") : F("CLOSED"));

    switch (valveNumber) {
      case 1: wasteValve1 = state ? openValve(wasteValve1) : closeValve(wasteValve1); break;
      case 2: wasteValve2 = state ? openValve(wasteValve2) : closeValve(wasteValve2); break;
      case 3: wasteValve3 = state ? openValve(wasteValve3) : closeValve(wasteValve3); break;
      case 4: wasteValve4 = state ? openValve(wasteValve4) : closeValve(wasteValve4); break;
    }
  } else {
    caller->println(F("[ERROR] Invalid waste valve command. Use: W <1-4> <0/1>"));
  }
}


/**
 * @brief Set the position of the pressure (proportional) valve.
 *
 * Parses a percentage value from the command arguments and sets the valve position accordingly.
 * The percentage is translated into a control voltage, and the valve's position is adjusted.
 *
 * @param args A C-string containing the desired valve position as a percentage (0–100).
 * @param caller Pointer to the CommandCaller used for sending responses.
 */
void cmd_set_pressure_valve(char* args, CommandCaller* caller) {
  int percentage = -1;

  if (sscanf(args, "%d", &percentage) == 1 && percentage >= 0 && percentage <= 100) {
    proportionalValve = setValvePosition(proportionalValve, (float)percentage);
    caller->print(F("[MESSAGE] Pressure valve set to "));
    caller->print(percentage);
    caller->println(F("%."));
  } else {
    caller->println(F("[ERROR] Invalid value for pressure valve. Use a percentage between 0 and 100."));
  }
}


/**
 * @brief Calibrate the pressure (proportional) valve.
 *
 * Initiates the calibration process by moving the valve to 100% open, allowing stabilization,
 * and then reading the maximum feedback value. Optionally, the valve can be closed afterward.
 *
 * @param args Unused parameter.
 * @param caller Pointer to the CommandCaller for output.
 */
void cmd_calibrate_pressure_valve(char* args, CommandCaller* caller) {
  caller->println(F("[MESSAGE] Calibrating pressure valve, please wait..."));

  calibrateProportionalValve();
  // Optionally, close the valve after calibration:
  // proportionalValve = setValvePosition(proportionalValve, 0.0);

  caller->println(F("[MESSAGE] Pressure valve calibration complete."));
}



/**
 * @brief Manually start measurement for a specified flow sensor.
 *
 * This command reads the sensor number from the arguments (1-4) and then attempts to
 * start the flow sensor measurement. Appropriate messages are printed for success or error.
 *
 * @param args A C-string containing the sensor number.
 * @param caller Pointer to the CommandCaller used for sending responses.
 */
void cmd_start_flow_sensor_manually(char* args, CommandCaller* caller) {
  int sensorNumber = -1;
  if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= NUM_FLOW_SENSORS) {
    FlowSensor* sensor = flowSensors[sensorNumber - 1];
    if (!sensor) {
      caller->print(F("[ERROR] Flow Sensor "));
      caller->print(sensorNumber);
      caller->println(F(" not found."));
      return;
    }

    if (startFlowSensorMeasurement(*sensor)) {
      caller->print(F("[MESSAGE] Manually started measurement for Flow Sensor "));
      caller->println(sensorNumber);
    } else {
      caller->print(F("[ERROR] Failed to manually start Flow Sensor "));
      caller->println(sensorNumber);
    }
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: STARTFSM <1-4>"));
  }
}

/**
 * @brief Manually stop measurement for a specified flow sensor.
 *
 * This command reads the sensor number (1-4) and then attempts to stop its measurement.
 * It provides feedback indicating whether the stop operation was successful.
 *
 * @param args A C-string containing the sensor number.
 * @param caller Pointer to the CommandCaller used for output.
 */
void cmd_stop_flow_sensor_manually(char* args, CommandCaller* caller) {
  int sensorNumber = -1;
  if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= NUM_FLOW_SENSORS) {
    FlowSensor* sensor = flowSensors[sensorNumber - 1];
    if (!sensor) {
      caller->print(F("[ERROR] Flow Sensor "));
      caller->print(sensorNumber);
      caller->println(F(" not found."));
      return;
    }

    if (stopFlowSensorMeasurement(*sensor)) {
      caller->print(F("[MESSAGE] Manually stopped measurement for Flow Sensor "));
      caller->println(sensorNumber);
    } else {
      caller->print(F("[ERROR] Failed to manually stop Flow Sensor "));
      caller->println(sensorNumber);
    }
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: STOPFSM <1-4>"));
  }
}

/**
 * @brief Reset the dispense volume for a flow sensor.
 *
 * This command resets the volume dispensed for a specific flow sensor (indexed 0–3).
 *
 * @param args A C-string containing the sensor index (0-3).
 * @param caller Pointer to the CommandCaller used for output.
 */
void cmd_reset_flow_dispense(char* args, CommandCaller* caller) {
  int sensorNumber = -1;
  if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber <= 3) {
    // Create a temporary array for clarity (using existing flow sensors)
    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorDispenseVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset dispense volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RESETFS <0-3>"));
  }
}

/**
 * @brief Reset the total volume for a flow sensor.
 *
 * This command resets the total volume accumulated for a specific flow sensor (indexed 0–3).
 *
 * @param args A C-string containing the sensor index (0-3).
 * @param caller Pointer to the CommandCaller used for sending feedback.
 */
void cmd_reset_flow_total(char* args, CommandCaller* caller) {
  int sensorNumber = -1;
  if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber <= 3) {
    FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
    resetFlowSensorTotalVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset total volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RESETTOTALFS <0-3>"));
  }
}


/**
 * @brief Resets the I2C bus manually.
 *
 * This command function prints a message, calls the resetI2CBus() helper,
 * and notifies the caller that the I2C bus reset is complete.
 *
 * @param args Unused arguments.
 * @param caller Pointer to the CommandCaller used for output.
 */
void cmd_reset_i2c(char* args, CommandCaller* caller) {
  Serial.println(F("[MESSAGE] Manual I2C bus reset initiated."));
  resetI2CBus();
  caller->println(F("[MESSAGE] I2C bus reset complete."));
}

/**
 * @brief Dispense reagent to a trough.
 *
 * Command format: D <trough> [volume]
 * If a volume is specified, dispense that exact volume (in mL); if omitted, the
 * system enters continuous dispensing mode.
 *
 * This function:
 * - Parses the input arguments.
 * - Validates the trough number (must be 1–4) and the volume (if specified).
 * - Checks that a dispense isn’t already in progress.
 * - Verifies system pressure meets the threshold.
 * - Checks for overflow before starting.
 * - Starts the flow sensor measurement and opens the appropriate valves.
 *
 * @param args A C-string containing the trough number and an optional volume.
 * @param caller Pointer to the CommandCaller used for sending messages.
 */
void cmd_dispense_reagent(char* args, CommandCaller* caller) {
  int troughNumber = -1;
  float requestedVolume = -1.0;
  const float MIN_VOLUME = 1.0;
  const float MAX_VOLUME = 200.0;

  // Command acknowledgment
  caller->print(F("[MESSAGE] Received command: D "));
  caller->println(args);

  // Parse input: if only trough number is provided, set continuous mode (-1)
  int parsedItems = sscanf(args, "%d %f", &troughNumber, &requestedVolume);
  if (parsedItems == 1) {
    requestedVolume = -1.0;  // Continuous mode
  } else if (parsedItems != 2) {
    caller->println(F("[ERROR] Invalid command format. Use: D <1-4> [volume]"));
    return;
  }

  // Validate trough number
  if (troughNumber < 1 || troughNumber > 4) {
    caller->println(F("[ERROR] Invalid trough number. Use 1-4."));
    return;
  }

  // Check if a dispense is already in progress
  if (valveControls[troughNumber - 1].isDispensing) {
    caller->print(F("[WARNING] A dispense is already in progress for Trough "));
    caller->println(troughNumber);
    caller->println(F("Use STOPD <trough number> to stop it first."));
    return;
  }

  // Validate requested volume if specified
  if (requestedVolume > 0) {
    if (requestedVolume < MIN_VOLUME) {
      caller->print(F("[ERROR] Requested volume too low. Minimum: "));
      caller->print(MIN_VOLUME);
      caller->println(F(" mL."));
      return;
    } else if (requestedVolume > MAX_VOLUME) {
      caller->print(F("[ERROR] Requested volume too high. Maximum: "));
      caller->print(MAX_VOLUME);
      caller->println(F(" mL."));
      return;
    }
  }

  // Define pressure settings
  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  // Pressure check: if not met, abort dispensing
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Dispense aborted."));
    return;
  }

  // Get the flow sensor for the trough
  FlowSensor* sensor = flowSensors[troughNumber - 1];
  if (!sensor) {
    caller->print(F("[ERROR] No flow sensor found for Trough "));
    caller->println(troughNumber);
    return;
  }

  // Check for overflow condition before dispensing
  if (readBinarySensor(overflowSensors[troughNumber - 1])) {
    caller->print(F("[ERROR] Cannot dispense: Overflow detected for Trough "));
    caller->println(troughNumber);
    return;
  }

  // Start the flow sensor measurement
  if (!startFlowSensorMeasurement(*sensor)) {
    caller->print(F("[ERROR] Failed to start flow sensor for Trough "));
    caller->println(troughNumber);
    return;
  }
  caller->print(F("[MESSAGE] Flow sensor measurement started for Trough "));
  caller->println(troughNumber);

  // Open the dispense valves and update state
  openDispenseValves(troughNumber);
  caller->print(F("[MESSAGE] Dispensing started for Trough "));
  caller->println(troughNumber);
  valveControls[troughNumber - 1].isDispensing = true;
  valveControls[troughNumber - 1].targetVolume = requestedVolume;
}

/**
 * @brief Stop dispensing operation.
 *
 * Command: STOPD <1-4> | STOPD all
 * - STOPD <1-4>: Stops dispensing for a specific trough (1-4).
 * - STOPD all  : Stops dispensing for all troughs immediately.
 *
 * This function ensures that:
 * - Valves are closed.
 * - Flow sensors are stopped and reset properly.
 * - The total dispensed volume is displayed for the affected trough(s).
 *
 * @param args A C-string containing either the trough number (1-4) or "all".
 * @param caller Pointer to the CommandCaller for output messaging.
 */
void cmd_stop_dispense(char* args, CommandCaller* caller) {
  int troughNumber = -1;
  bool stopAll = false;

  // Check if the argument is "all" to stop dispensing for all troughs.
  if (strncmp(args, "all", 3) == 0) {
    stopAll = true;
  }
  // Otherwise, try to parse a trough number.
  else if (sscanf(args, "%d", &troughNumber) != 1 || troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    caller->println(F("[ERROR] Invalid trough number. Use STOPD <1-4> or STOPD all."));
    return;
  }

  if (stopAll) {
    // Stop dispensing for all troughs.
    caller->println(F("[MESSAGE] Stopping all dispensing operations..."));
    for (int i = 1; i <= NUM_OVERFLOW_SENSORS; i++) {
      stopDispenseOperation(i, caller);
    }
    caller->println(F("[MESSAGE] All dispensing operations stopped."));
  } else {
    // Stop dispensing for the specified trough.
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispensing stopped for Trough "));
    caller->println(troughNumber);
  }
}



//////////////////////////////////////////////////////////////
//                COMMANDER API SETUP                       //
//////////////////////////////////////////////////////////////
// ===========================================================
// Commander API Setup
// ===========================================================

/**
 * @brief Global Commander instance.
 */
Commander commander;

/**
 * @brief Command Tree for the Commander API.
 *
 * This array defines the mapping between command strings and the 
 * corresponding command function pointers. Each system command includes:
 *  - A command identifier string.
 *  - A description for usage.
 *  - A pointer to the command handling function.
 */
Commander::systemCommand_t API_tree[] = {
  systemCommand("LF", "Set log frequency: LF <ms>", cmd_set_log_frequency),
  systemCommand("FN", "Fan: FN <0/1> (0 = off, 1 = on)", cmd_fan),
  systemCommand("FNAUTO", "Enable fan auto control", cmd_fan_auto),
  systemCommand("R", "Reagent valve: R <1-4> <0/1>", cmd_set_reagent_valve),
  systemCommand("M", "Media valve: M <1-4> <0/1>", cmd_set_media_valve),
  systemCommand("W", "Waste valve: W <1-4> <0/1>", cmd_set_waste_valve),
  systemCommand("PV", "Pressure valve: PV <percentage>", cmd_set_pressure_valve),
  systemCommand("CALPV", "Calibrate pressure valve", cmd_calibrate_pressure_valve),
  systemCommand("STARTFSM", "Manually start flow sensor measurement: STARTFSM <1-4>", cmd_start_flow_sensor_manually),
  systemCommand("STOPFSM", "Manually stop flow sensor measurement: STOPFSM <1-4>", cmd_stop_flow_sensor_manually),
  systemCommand("RF", "Reset flow sensor dispense volume: RFS <0-3>", cmd_reset_flow_dispense),
  systemCommand("RTF", "Reset total volume for Flow Sensor: RTF <0-3>", cmd_reset_flow_total),
  systemCommand("RESETI2C", "Manually reset the I2C bus", cmd_reset_i2c),
  systemCommand("D", "Dispense reagent: D <1-4> [volume] (volume in mL, continuous if omitted)", cmd_dispense_reagent),
  systemCommand("STOPD", "Stop dispensing: STOPD <1-4> (stop specific trough) or STOPD ALL", cmd_stop_dispense)
};

/**
 * @brief Global command buffer for serial command processing.
 */
char commandBuffer[COMMAND_SIZE];

//////////////////////////////////////////////////////////////
//                SETUP FUNCTION                            //
//////////////////////////////////////////////////////////////

/**
 * @brief System initialization function.
 *
 * Sets up serial communication, initializes hardware (fan, valves, sensors),
 * calibrates the proportional valve, and sets up the Commander API.
 */
void setup() {
  // --------------------------
  // Initialize Serial Communication
  // --------------------------
  Serial.begin(115200);
  Serial.println(F("[MESSAGE] System starting..."));

  // --------------------------
  // Fan Setup
  // --------------------------
  fanSetup(fan);

  // --------------------------
  // Proportional Valve & Pressure Sensor Setup
  // --------------------------
  proportionalValveSetup(proportionalValve);
  calibrateProportionalValve();

  // Check system air pressure after calibration
  float systemPressure = readPressure(pressureSensor);
  if (systemPressure > 15.0) {
    Serial.print(F("[MESSAGE] System air pressure available: "));
    Serial.print(systemPressure);
    Serial.println(F(" psi."));
  } else {
    Serial.print(F("[WARNING] Low air pressure detected! Current pressure: "));
    Serial.print(systemPressure);
    Serial.println(F(" psi. Ensure air supply is available."));
  }
  // Close the valve after calibration
  setValvePosition(proportionalValve, 0.0);

  // --------------------------
  // Flow Sensor Initialization
  // --------------------------
  Serial.println(F("[MESSAGE] Initializing Flow Sensors..."));

  bool allFailed = true;      // Assume all flow sensors fail until one succeeds.
  bool anyStopped = false;    // Check if any sensors are intentionally stopped.
  String failedSensors = "";  // To store indices of sensors that fail to initialize.
  FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };

  // Attempt to initialize each active flow sensor.
  for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
    if (sensors[i]->sensorStopped) {
      anyStopped = true;
      continue;  // Skip sensors that are intentionally stopped.
    }
    if (initializeFlowSensor(*sensors[i])) {
      allFailed = false;
    } else {
      failedSensors += String(i) + " ";
    }
  }

  // If all active sensors failed, try resetting the I2C bus and retry.
  if (allFailed) {
    Serial.println(F("[WARNING] All flow sensors failed. Resetting I2C bus..."));
    resetI2CBus();
    delay(50);
    // Reset tracking variables and try again.
    failedSensors = "";
    allFailed = true;
    for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
      if (!sensors[i]->sensorInitialized && !sensors[i]->sensorStopped) {
        if (initializeFlowSensor(*sensors[i])) {
          allFailed = false;
        } else {
          failedSensors += String(i) + " ";
        }
      }
    }
  }

  // Report the initialization results.
  if (anyStopped) {
    Serial.println(F("[MESSAGE] Some flow sensors are intentionally stopped."));
  }
  if (allFailed) {
    Serial.println(F("[ERROR] All active flow sensors failed to initialize!"));
  } else if (failedSensors.length() > 0) {
    Serial.print(F("[ERROR] The following Flow Sensors failed to initialize: "));
    Serial.println(failedSensors);
  } else {
    Serial.println(F("[MESSAGE] All active flow sensors initialized successfully."));
  }
  delay(500);  // Allow additional stabilization time for sensors.

  // --------------------------
  // Temperature/Humidity Sensor Setup
  // --------------------------
  if (!tempHumSensorInit()) {
    Serial.println(F("[ERROR] Temp/Humidity sensor not detected!"));
  } else {
    Serial.println(F("[MESSAGE] Temp/Humidity sensor initialized successfully."));
  }

  // --------------------------
  // Commander API Setup
  // --------------------------
  commander.attachTree(API_tree);
  commander.init();

  Serial.println(F("[MESSAGE] System ready."));
}

//////////////////////////////////////////////////////////////
//                        MAIN LOOP                         //
//////////////////////////////////////////////////////////////

void loop() {
  // Get the current system time for non-blocking operations.
  unsigned long currentTime = millis();

  // --------------------------
  // Serial Command Processing
  // --------------------------
  // Check and process incoming commands from the serial buffer.
  handleSerialCommands();

  // --------------------------
  // Sensor Monitoring
  // --------------------------
  // Continuously monitor various sensors with non-blocking checks.
  monitorOverflowSensors(currentTime);
  monitorFlowSensors(currentTime);
  monitorReagentBubbleSensors(currentTime);
  monitorWasteLineSensors(currentTime);
  monitorWasteBottleSensors(currentTime);
  monitorWasteVacuumSensors(currentTime);
  monitorEnclosureLiquidSensor(currentTime);
  monitorEnclosureTemp();
  monitorFlowSensorConnections();

  // --------------------------
  // Flow Sensor Updates
  // --------------------------
  // Update flow sensor readings (could be optimized with a loop if desired).
  readFlowSensorData(flow1);
  readFlowSensorData(flow2);
  readFlowSensorData(flow3);
  readFlowSensorData(flow4);

  // --------------------------
  // Periodic System State Logging
  // --------------------------
  // Log the system state if the log interval has passed.
  if (currentTime - logging.previousLogTime >= logging.logInterval) {
    logging.previousLogTime = currentTime;
    logSystemState();
  }
}

//////////////////////////////////////////////////////////////
//                   HELPER FUNCTIONS                       //
//////////////////////////////////////////////////////////////

/**
 * trimLeadingSpaces()
 * ---------------------
 * Removes leading whitespace from a given C-string.
 *
 * @param str: Pointer to the string to trim.
 * @return A pointer to the first non-whitespace character in the string.
 */
char* trimLeadingSpaces(char* str) {
  while (*str && isspace(*str)) {
    str++;
  }
  return str;
}

/**
 * processMultipleCommands()
 * ---------------------------
 * Splits the input command line into individual commands (using commas as delimiters),
 * trims leading whitespace from each token, and then executes each command via the commander.
 *
 * @param commandLine: The command string to process (will be modified).
 * @param stream: The Stream object used for logging (typically Serial).
 */
void processMultipleCommands(char* commandLine, Stream* stream) {
  char* commandToken = strtok(commandLine, ",");  // Split at commas

  while (commandToken != NULL) {
    commandToken = trimLeadingSpaces(commandToken);

    if (strlen(commandToken) > 0) {
      Serial.print(F("[COMMAND] Processing: "));
      Serial.println(commandToken);
      commander.execute(commandToken, stream);
    }

    commandToken = strtok(NULL, ",");  // Get the next token
  }
}

/**
 * handleSerialCommands()
 * ------------------------
 * Reads characters from the Serial port, builds complete command strings,
 * and dispatches them for processing.
 */
void handleSerialCommands() {
  static char commandBuffer[COMMAND_SIZE];
  static uint8_t commandIndex = 0;

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      commandBuffer[commandIndex] = '\0';  // Null-terminate the command

      // Log the received command before processing
      Serial.print(F("[COMMAND] Received: "));
      Serial.println(commandBuffer);

      // Process and execute the command(s)
      processMultipleCommands(commandBuffer, &Serial);

      // Reset the command buffer index for the next command
      commandIndex = 0;
    } else if (incomingChar != '\r') {
      // Store character in buffer if there's space left
      if (commandIndex < (COMMAND_SIZE - 1)) {
        commandBuffer[commandIndex++] = incomingChar;
      }
    }
  }
}


/**
 * logSystemState()
 * ----------------
 * Gathers current states from various sensors and hardware (fan, valves, sensors, etc.),
 * formats the data into a single log string, and prints it to the Serial console.
 *
 * This function uses dtostrf() to convert floating-point numbers into formatted strings.
 */
void logSystemState() {
  char logBuffer[300];  // Buffer for the formatted log message

  // --- Gather Fan and Valve States ---
  char fanState = (digitalRead(fan.relayPin) == HIGH ? '1' : '0');

  // Reagent valve states (R1-R4)
  char rValve1 = (reagentValve1.isOpen ? '1' : '0');
  char rValve2 = (reagentValve2.isOpen ? '1' : '0');
  char rValve3 = (reagentValve3.isOpen ? '1' : '0');
  char rValve4 = (reagentValve4.isOpen ? '1' : '0');

  // Media valve states (M1-M4)
  char mValve1 = (mediaValve1.isOpen ? '1' : '0');
  char mValve2 = (mediaValve2.isOpen ? '1' : '0');
  char mValve3 = (mediaValve3.isOpen ? '1' : '0');
  char mValve4 = (mediaValve4.isOpen ? '1' : '0');

  // Waste valve states (W1-W4)
  char wValve1 = (wasteValve1.isOpen ? '1' : '0');
  char wValve2 = (wasteValve2.isOpen ? '1' : '0');
  char wValve3 = (wasteValve3.isOpen ? '1' : '0');
  char wValve4 = (wasteValve4.isOpen ? '1' : '0');

  // --- Proportional Valve Feedback ---
  char pFeedbackStr[8];
  dtostrf(getValveFeedback(proportionalValve), 4, 1, pFeedbackStr);
  float feedbackVal = getValveFeedback(proportionalValve);
  float valvePct = (feedbackVal / proportionalValveMaxFeedback) * 100.0;
  char pPctStr[8];
  dtostrf(valvePct, 4, 1, pPctStr);

  // --- Sensor States: Waste Lines, Bottles, Vacuum, Enclosure Liquid ---
  char wasteLine1 = (readBinarySensor(wasteLineSensors[0]) ? '1' : '0');
  char wasteLine2 = (readBinarySensor(wasteLineSensors[1]) ? '1' : '0');

  char wasteBottle1 = (readBinarySensor(wasteBottleSensors[0]) ? '1' : '0');
  char wasteBottle2 = (readBinarySensor(wasteBottleSensors[1]) ? '1' : '0');

  char wasteVacuum1 = (readBinarySensor(wasteVacuumSensors[0]) ? '1' : '0');
  char wasteVacuum2 = (readBinarySensor(wasteVacuumSensors[1]) ? '1' : '0');

  char enclosureLiquid = (readBinarySensor(enclosureLiquidSensor) ? '1' : '0');

  // --- Bubble Sensor States (Reagent) ---
  char bubble1 = (readBinarySensor(reagentBubbleSensors[0]) ? '1' : '0');
  char bubble2 = (readBinarySensor(reagentBubbleSensors[1]) ? '1' : '0');
  char bubble3 = (readBinarySensor(reagentBubbleSensors[2]) ? '1' : '0');
  char bubble4 = (readBinarySensor(reagentBubbleSensors[3]) ? '1' : '0');

  // --- Overflow Sensor States ---
  char overflow1 = (readBinarySensor(overflowSensors[0]) ? '1' : '0');
  char overflow2 = (readBinarySensor(overflowSensors[1]) ? '1' : '0');
  char overflow3 = (readBinarySensor(overflowSensors[2]) ? '1' : '0');
  char overflow4 = (readBinarySensor(overflowSensors[3]) ? '1' : '0');

  // --- Pressure Sensor Reading ---
  float pressureVal = readPressure(pressureSensor);
  char pressureStr[8];
  dtostrf(pressureVal, 4, 1, pressureStr);

  // --- Temperature & Humidity Reading ---
  TempHumidity thData = readTempHumidity();
  char tempStr[8], humStr[8];
  if (thData.valid) {
    dtostrf(thData.temperature, 4, 1, tempStr);
    dtostrf(thData.humidity, 4, 1, humStr);
  } else {
    strcpy(tempStr, "-1");
    strcpy(humStr, "-1");
  }

  // --- Flow Sensor Data ---
  char f1Rate[8], f1Temp[8], f2Rate[8], f2Temp[8],
    f3Rate[8], f3Temp[8], f4Rate[8], f4Temp[8];
  char f1Disp[8], f1Total[8], f2Disp[8], f2Total[8],
    f3Disp[8], f3Total[8], f4Disp[8], f4Total[8];
  char f1Flag[4], f2Flag[4], f3Flag[4], f4Flag[4];

  dtostrf(flow1.isValidReading ? flow1.flowRate : -1, 4, 1, f1Rate);
  dtostrf(flow1.isValidReading ? flow1.temperature : -1, 4, 1, f1Temp);
  dtostrf(flow1.dispenseVolume, 4, 1, f1Disp);
  dtostrf(flow1.totalVolume, 4, 1, f1Total);
  snprintf(f1Flag, sizeof(f1Flag), "%d", flow1.isValidReading ? flow1.highFlowFlag : -1);

  dtostrf(flow2.isValidReading ? flow2.flowRate : -1, 4, 1, f2Rate);
  dtostrf(flow2.isValidReading ? flow2.temperature : -1, 4, 1, f2Temp);
  dtostrf(flow2.dispenseVolume, 4, 1, f2Disp);
  dtostrf(flow2.totalVolume, 4, 1, f2Total);
  snprintf(f2Flag, sizeof(f2Flag), "%d", flow2.isValidReading ? flow2.highFlowFlag : -1);

  dtostrf(flow3.isValidReading ? flow3.flowRate : -1, 4, 1, f3Rate);
  dtostrf(flow3.isValidReading ? flow3.temperature : -1, 4, 1, f3Temp);
  dtostrf(flow3.dispenseVolume, 4, 1, f3Disp);
  dtostrf(flow3.totalVolume, 4, 1, f3Total);
  snprintf(f3Flag, sizeof(f3Flag), "%d", flow3.isValidReading ? flow3.highFlowFlag : -1);

  dtostrf(flow4.isValidReading ? flow4.flowRate : -1, 4, 1, f4Rate);
  dtostrf(flow4.isValidReading ? flow4.temperature : -1, 4, 1, f4Temp);
  dtostrf(flow4.dispenseVolume, 4, 1, f4Disp);
  dtostrf(flow4.totalVolume, 4, 1, f4Total);
  snprintf(f4Flag, sizeof(f4Flag), "%d", flow4.isValidReading ? flow4.highFlowFlag : -1);

  // --- Format the Log Message ---
  sprintf(logBuffer,
          "[LOG] F%s, RV%s%s%s%s, MV%s%s%s%s, WV%s%s%s%s, PV,%s, PV%%,%s, "
          "WSL%s%s, WBL%s%s, WVS%s%s, ELS%s, BS%s%s%s%s, OS%s%s%s%s, "
          "PS,%s, ETS,%s, EHS,%s, FS1,%s,%s,%s,%s,%s; FS2,%s,%s,%s,%s,%s; "
          "FS3,%s,%s,%s,%s,%s; FS4,%s,%s,%s,%s,%s",
          // Fan and Valve states
          &fanState, &rValve1, &rValve2, &rValve3, &rValve4,
          &mValve1, &mValve2, &mValve3, &mValve4,
          &wValve1, &wValve2, &wValve3, &wValve4,
          // Proportional Valve Feedback
          pFeedbackStr, pPctStr,
          // Waste sensor states
          &wasteLine1, &wasteLine2,
          &wasteBottle1, &wasteBottle2,
          &wasteVacuum1, &wasteVacuum2,
          &enclosureLiquid,
          // Bubble sensor states
          &bubble1, &bubble2, &bubble3, &bubble4,
          // Overflow sensor states
          &overflow1, &overflow2, &overflow3, &overflow4,
          // Pressure, Temperature, Humidity
          pressureStr, tempStr, humStr,
          // Flow sensor 1 data
          f1Rate, f1Temp, f1Disp, f1Total, f1Flag,
          // Flow sensor 2 data
          f2Rate, f2Temp, f2Disp, f2Total, f2Flag,
          // Flow sensor 3 data
          f3Rate, f3Temp, f3Disp, f3Total, f3Flag,
          // Flow sensor 4 data
          f4Rate, f4Temp, f4Disp, f4Total, f4Flag);

  Serial.println(logBuffer);
}



/**
 * monitorOverflowSensors()
 * --------------------------
 * Checks every 25ms for any overflow condition in the troughs.
 * If an overflow is detected, it calls the overflow handler for that trough.
 */
void monitorOverflowSensors(unsigned long currentTime) {
  static unsigned long previousOverflowCheckTime = 0;

  if (currentTime - previousOverflowCheckTime >= 25) {
    previousOverflowCheckTime = currentTime;

    // Iterate over all troughs (represented by overflow sensors)
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      if (readBinarySensor(overflowSensors[i])) {
        handleOverflowCondition(i + 1);
      }
    }
  }
}

/**
 * monitorFlowSensors()
 * ----------------------
 * Monitors each active flow sensor for:
 *   - Overflow detection
 *   - No flow timeout (if flow rate falls below a minimum threshold)
 *   - Dispense volume target reached
 *   - Exceeding maximum safe trough volume (secondary safety check)
 * Checks every 25ms and takes appropriate actions when any condition is met.
 */
void monitorFlowSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_THRESHOLD = 1.0;
  const float MAX_TROUGH_VOLUME = 205.0;  // Maximum safe volume per trough

  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;

    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      if (!valveControls[i].isDispensing) {
        continue;
      }

      FlowSensor* sensor = flowSensors[i];
      if (!sensor) {
        continue;
      }

      // Overflow check
      if (readBinarySensor(overflowSensors[i])) {
        closeDispenseValves(i + 1);
        Serial.print(F("[WARNING] Overflow detected for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Dispensed volume: "));
        Serial.print(sensor->dispenseVolume, 1);
        Serial.println(F(" mL."));
        resetFlowSensorDispenseVolume(*sensor);
        stopFlowSensorMeasurement(*sensor);
        valveControls[i].isDispensing = false;
        continue;
      }

      // No flow timeout check
      if (sensor->flowRate < MIN_FLOW_RATE_THRESHOLD) {
        if (valveControls[i].lastFlowCheckTime == 0) {
          valveControls[i].lastFlowCheckTime = currentTime;
        } else if (currentTime - valveControls[i].lastFlowCheckTime >= FLOW_TIMEOUT_MS) {
          handleTimeoutCondition(i + 1);
          continue;
        }
      } else {
        valveControls[i].lastFlowCheckTime = 0;
      }

      // Requested volume reached check
      if (valveControls[i].targetVolume > 0 && sensor->dispenseVolume >= valveControls[i].targetVolume) {
        closeDispenseValves(i + 1);
        Serial.print(F("[MESSAGE] Dispense complete for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Final volume dispensed: "));
        Serial.print(sensor->dispenseVolume, 1);
        Serial.println(F(" mL."));
        resetFlowSensorDispenseVolume(*sensor);
        stopFlowSensorMeasurement(*sensor);
        valveControls[i].isDispensing = false;
        continue;
      }

      // Secondary safety check: maximum trough volume exceeded
      if (sensor->dispenseVolume >= MAX_TROUGH_VOLUME) {
        closeDispenseValves(i + 1);
        Serial.print(F("[ERROR] Safety Limit Reached! Dispense stopped for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Final dispensed volume: "));
        Serial.print(sensor->dispenseVolume, 1);
        Serial.print(F(" mL. (Max Allowed: "));
        Serial.print(MAX_TROUGH_VOLUME);
        Serial.println(F(" mL)"));
        resetFlowSensorDispenseVolume(*sensor);
        stopFlowSensorMeasurement(*sensor);
        valveControls[i].isDispensing = false;
      }
    }
  }
}






/**
 * monitorReagentBubbleSensors()
 * ------------------------------
 * Checks every 25ms for bubble events on the reagent bubble sensors.
 * If a bubble is detected, bubble handling logic should be executed.
 */
void monitorReagentBubbleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_REAGENT_BUBBLE_SENSORS; i++) {
      if (readBinarySensor(reagentBubbleSensors[i])) {
        // TODO: Insert bubble handling logic here.
      }
    }
  }
}

/**
 * monitorWasteLineSensors()
 * --------------------------
 * Checks every 25ms for events on the waste line sensors.
 * If a sensor is triggered, appropriate waste line handling logic should be executed.
 */
void monitorWasteLineSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_LINE_SENSORS; i++) {
      if (readBinarySensor(wasteLineSensors[i])) {
        // TODO: Insert waste line handling logic here.
      }
    }
  }
}

/**
 * monitorWasteBottleSensors()
 * ----------------------------
 * Checks every 25ms for events on the waste bottle sensors.
 * If a sensor is triggered, appropriate waste bottle handling logic should be executed.
 */
void monitorWasteBottleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_BOTTLE_SENSORS; i++) {
      if (readBinarySensor(wasteBottleSensors[i])) {
        // TODO: Insert waste bottle handling logic here.
      }
    }
  }
}

/**
 * monitorWasteVacuumSensors()
 * ----------------------------
 * Checks every 25ms for events on the waste vacuum sensors.
 * If a sensor is triggered, appropriate waste vacuum handling logic should be executed.
 */
void monitorWasteVacuumSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_VACUUM_SENSORS; i++) {
      if (readBinarySensor(wasteVacuumSensors[i])) {
        // TODO: Insert waste vacuum handling logic here.
      }
    }
  }
}

/**
 * monitorEnclosureLiquidSensor()
 * -------------------------------
 * Checks every 25ms for events on the enclosure liquid sensor.
 * If the sensor is triggered, the enclosure liquid handling logic should be executed.
 */
void monitorEnclosureLiquidSensor(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    if (readBinarySensor(enclosureLiquidSensor)) {
      // TODO: Insert enclosure liquid handling logic here.
    }
  }
}


/**
 * isFlowSensorConnected()
 * -------------------------
 * Selects the correct multiplexer channel for the given flow sensor,
 * then checks if the sensor responds via I2C.
 * Updates the sensor's connected flag and returns true if connected,
 * or false if disconnected.
 */
bool isFlowSensorConnected(FlowSensor& sensor) {
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);

  Wire.beginTransmission(sensor.sensorAddr);
  if (Wire.endTransmission() == 0) {
    sensor.sensorConnected = 1;  // Mark as connected
    return true;
  } else {
    sensor.sensorConnected = 0;  // Mark as disconnected
    return false;
  }
}

/**
 * monitorFlowSensorConnections()
 * -------------------------------
 * Iterates over all flow sensors and updates their connection status.
 * This ensures the sensorConnected flag is current for each sensor.
 */
void monitorFlowSensorConnections() {
  FlowSensor* sensors[] = { &flow1, &flow2, &flow3, &flow4 };
  for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
    if (!isFlowSensorConnected(*sensors[i])) {
      sensors[i]->sensorConnected = 0;  // Mark as disconnected
    }
  }
}

/**
 * calibrateProportionalValve()
 * -----------------------------
 * Calibrates the proportional valve by setting it to 100% open,
 * waiting for stabilization, and then reading the maximum feedback voltage.
 * The calibrated value is stored in proportionalValveMaxFeedback.
 */
void calibrateProportionalValve() {
  Serial.println(F("[MESSAGE] Starting proportional valve calibration..."));
  proportionalValve = setValvePosition(proportionalValve, 100.0);
  delay(1000);  // Allow time for the valve to settle and feedback to stabilize
  proportionalValveMaxFeedback = getValveFeedback(proportionalValve);
  Serial.print(F("[MESSAGE] Calibrated max feedback voltage: "));
  Serial.println(proportionalValveMaxFeedback);
}

/**
 * monitorEnclosureTemp()
 * -----------------------
 * Reads the enclosure temperature using the SHT31 sensor.
 * If the temperature exceeds the defined setpoint and the fan is in auto mode,
 * activates the fan; otherwise, deactivates the fan.
 * If the sensor reading is invalid, logs an error.
 */
void monitorEnclosureTemp() {
  if (!fanAutoMode) return;  // Skip if manual override is active

  TempHumidity th = readTempHumidity();
  if (th.valid) {
    if (th.temperature > ENCLOSURE_TEMP_SETPOINT) {
      Serial.println(F("[WARNING] Enclosure temperature exceeded setpoint! Activating fan."));
      setFanState(fan, true);
    } else {
      setFanState(fan, false);
    }
  } else {
    Serial.println(F("[ERROR] Failed to read enclosure temperature!"));
  }
}


/**
 * resetI2CBus()
 * -------------
 * Resets the I2C bus by ending the current Wire connection,
 * delaying briefly, and then reinitializing it.
 */
void resetI2CBus() {
  Serial.println(F("[MESSAGE] Resetting I2C bus..."));
  Wire.end();
  delay(100);
  Wire.begin();
}

/**
 * resetFlowSensorDispenseVolume()
 * -------------------------------
 * Resets the dispense volume of the given flow sensor, updates its
 * last update time, and marks the sensor as stopped.
 * Logs the action along with the sensor channel.
 */
void resetFlowSensorDispenseVolume(FlowSensor& sensor) {
  sensor.dispenseVolume = 0.0;
  sensor.lastUpdateTime = millis();
  sensor.sensorStopped = true;  // Ensure sensor is actually stopped

  Serial.print(F("[MESSAGE] Dispense volume reset for flow sensor on channel "));
  Serial.println(sensor.channel);
}

/**
 * resetFlowSensorTotalVolume()
 * ----------------------------
 * Resets the total volume measured by the given flow sensor.
 * Logs the reset action with the sensor channel.
 */
void resetFlowSensorTotalVolume(FlowSensor& sensor) {
  sensor.totalVolume = 0.0;
  Serial.print(F("[MESSAGE] Total volume reset for flow sensor on channel "));
  Serial.println(sensor.channel);
}

/**
 * startFlowSensorMeasurement()
 * -----------------------------
 * Attempts to start the measurement mode of a given flow sensor.
 * - Checks if the sensor is connected.
 * - Resets the sensorStopped flag.
 * - Retries initialization up to 3 times.
 * Returns true if the sensor starts measuring, false otherwise.
 */
bool startFlowSensorMeasurement(FlowSensor& sensor) {
  Serial.print(F("[DEBUG] Attempting to start flow measurement for sensor on channel "));
  Serial.println(sensor.channel);

  if (!isFlowSensorConnected(sensor)) {
    Serial.print(F("[ERROR] Cannot start measurement for flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" because it is disconnected."));
    return false;
  }

  // Reset sensorStopped before initialization
  sensor.sensorStopped = false;

  // Retry initialization up to 3 times
  for (int i = 0; i < 3; i++) {
    Serial.print(F("[DEBUG] Attempt "));
    Serial.print(i + 1);
    Serial.println(F(" to initialize sensor."));

    if (initializeFlowSensor(sensor)) {
      Serial.print(F("[MESSAGE] Flow sensor on channel "));
      Serial.print(sensor.channel);
      Serial.println(F(" started measurement mode."));
      return true;
    }
    delay(50);
  }

  Serial.print(F("[ERROR] Failed to start measurement for flow sensor on channel "));
  Serial.println(sensor.channel);
  return false;
}


/**
 * stopFlowSensorMeasurement()
 * -----------------------------
 * Sends a stop command to the specified flow sensor over I2C.
 * Marks the sensor as not initialized and stopped upon success.
 * Returns true if the command was acknowledged, otherwise false.
 */
bool stopFlowSensorMeasurement(FlowSensor& sensor) {
  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);

  Wire.beginTransmission(sensor.sensorAddr);
  Wire.write(0x3F);  // Stop measurement command MSB
  Wire.write(0xF9);  // Stop measurement command LSB

  if (Wire.endTransmission() == 0) {
    Serial.print(F("[MESSAGE] Flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" stopped measurement mode."));

    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    return true;
  } else {
    Serial.print(F("[ERROR] Failed to stop measurement for flow sensor on channel "));
    Serial.println(sensor.channel);
    return false;
  }
}

/**
 * handleOverflowCondition()
 * ---------------------------
 * Handles an overflow condition for the given trough.
 * If a dispensing operation is in progress and an overflow is detected,
 * it closes the valves, stops and resets the corresponding flow sensor,
 * and logs the dispensed volume.
 */
void handleOverflowCondition(int triggeredTrough) {
  if (valveControls[triggeredTrough - 1].isDispensing) {
    Serial.print(F("[WARNING] Overflow detected in Trough "));
    Serial.println(triggeredTrough);

    closeDispenseValves(triggeredTrough);
    Serial.print(F("[MESSAGE] Closed reagent and media valves for Trough "));
    Serial.println(triggeredTrough);

    FlowSensor* sensor = flowSensors[triggeredTrough - 1];
    if (sensor) {
      Serial.print(F("[MESSAGE] Dispensed volume before overflow: "));
      Serial.print(sensor->dispenseVolume, 1);
      Serial.println(F(" mL."));
      stopFlowSensorMeasurement(*sensor);
      resetFlowSensorDispenseVolume(*sensor);
    }

    valveControls[triggeredTrough - 1].isDispensing = false;
    valveControls[triggeredTrough - 1].lastFlowCheckTime = 0;
    valveControls[triggeredTrough - 1].lastFlowChangeTime = 0;
    valveControls[triggeredTrough - 1].dispensingValveNumber = -1;
  }
}

/**
 * handleTimeoutCondition()
 * --------------------------
 * Handles a timeout condition for the specified trough.
 * If no or insufficient flow is detected during dispensing, it stops the operation,
 * closes valves, stops and resets the flow sensor, and logs the final volume.
 */
void handleTimeoutCondition(int troughNumber) {
  if (valveControls[troughNumber - 1].isDispensing) {
    Serial.print(F("[ERROR] Timeout: No or insufficient flow detected for Trough "));
    Serial.print(troughNumber);
    Serial.println(F(". Stopping dispense."));

    closeDispenseValves(troughNumber);

    FlowSensor* sensor = flowSensors[troughNumber - 1];
    if (sensor) {
      Serial.print(F("[MESSAGE] Dispensed volume before timeout: "));
      Serial.print(sensor->dispenseVolume, 1);
      Serial.println(F(" mL."));
      stopFlowSensorMeasurement(*sensor);
      resetFlowSensorDispenseVolume(*sensor);
    }

    valveControls[troughNumber - 1].isDispensing = false;
    valveControls[troughNumber - 1].lastFlowCheckTime = 0;
    valveControls[troughNumber - 1].lastFlowChangeTime = 0;
    valveControls[troughNumber - 1].dispensingValveNumber = -1;
    valveControls[troughNumber - 1].targetVolume = -1;
  }
}

/**
 * closeDispenseValves()
 * ----------------------
 * Closes the reagent and media valves for the specified trough.
 * Logs an error if the provided trough number is invalid.
 */
void closeDispenseValves(int troughNumber) {
  if (troughNumber >= 1 && troughNumber <= NUM_OVERFLOW_SENSORS) {
    // Close the corresponding reagent valve
    switch (troughNumber) {
      case 1: reagentValve1 = closeValve(reagentValve1); break;
      case 2: reagentValve2 = closeValve(reagentValve2); break;
      case 3: reagentValve3 = closeValve(reagentValve3); break;
      case 4: reagentValve4 = closeValve(reagentValve4); break;
    }

    // Close the corresponding media valve
    switch (troughNumber) {
      case 1: mediaValve1 = closeValve(mediaValve1); break;
      case 2: mediaValve2 = closeValve(mediaValve2); break;
      case 3: mediaValve3 = closeValve(mediaValve3); break;
      case 4: mediaValve4 = closeValve(mediaValve4); break;
    }
  } else {
    Serial.println(F("[ERROR] Invalid trough number provided to closeDispenseValves()"));
  }
}

/**
 * openDispenseValves()
 * ---------------------
 * Opens the reagent and media valves for the specified trough.
 * Logs an error if the provided trough number is invalid.
 */
void openDispenseValves(int troughNumber) {
  if (troughNumber >= 1 && troughNumber <= NUM_OVERFLOW_SENSORS) {
    // Open the corresponding reagent valve
    switch (troughNumber) {
      case 1: reagentValve1 = openValve(reagentValve1); break;
      case 2: reagentValve2 = openValve(reagentValve2); break;
      case 3: reagentValve3 = openValve(reagentValve3); break;
      case 4: reagentValve4 = openValve(reagentValve4); break;
    }

    // Open the corresponding media valve
    switch (troughNumber) {
      case 1: mediaValve1 = openValve(mediaValve1); break;
      case 2: mediaValve2 = openValve(mediaValve2); break;
      case 3: mediaValve3 = openValve(mediaValve3); break;
      case 4: mediaValve4 = openValve(mediaValve4); break;
    }

    Serial.print(F("[MESSAGE] Opened reagent and media valves for Trough "));
    Serial.println(troughNumber);
  } else {
    Serial.println(F("[ERROR] Invalid trough number provided to openDispenseValves()"));
  }
}

/**
 * checkAndSetPressure()
 * -----------------------
 * Checks if the system pressure has reached a specified threshold and if the valve is at the
 * desired position. If not, adjusts the valve and waits until the pressure stabilizes or a timeout occurs.
 * Returns true if the threshold is reached within the timeout, otherwise false.
 */
bool checkAndSetPressure(float thresholdPressure, float valvePosition, unsigned long timeout) {
  unsigned long pressureCheckStartTime = millis();
  float currentPressure = readPressure(pressureSensor);
  float currentValvePosition = proportionalValve.controlVoltage;  // Get current valve setting

  if (currentPressure >= thresholdPressure && currentValvePosition == valvePosition) {
    Serial.println(F("[MESSAGE] System is already pressurized and valve is at the correct position."));
    return true;
  }

  setValvePosition(proportionalValve, valvePosition);
  Serial.print(F("[MESSAGE] Pressure valve set to "));
  Serial.print(valvePosition);
  Serial.println(F("%. Waiting for pressure stabilization..."));

  while (millis() - pressureCheckStartTime < timeout) {
    currentPressure = readPressure(pressureSensor);
    if (currentPressure >= thresholdPressure) {
      Serial.println(F("[MESSAGE] Pressure threshold reached."));
      return true;
    }
    delay(100);
  }

  Serial.print(F("[ERROR] Pressure threshold not reached. Current pressure: "));
  Serial.print(currentPressure);
  Serial.println(F(" psi. Operation aborted."));
  return false;
}

/**
 * stopDispenseOperation()
 * -------------------------
 * Stops the dispensing operation for a specific trough by closing the valves,
 * stopping and resetting the associated flow sensor, and updating the dispensing state.
 */
void stopDispenseOperation(int troughNumber, CommandCaller* caller) {
  // Close valves for the specified trough
  closeDispenseValves(troughNumber);

  // Stop and reset the flow sensor for that trough
  FlowSensor* sensor = flowSensors[troughNumber - 1];  // Directly reference from array
  if (sensor) {
    caller->print(F("[MESSAGE] Trough "));
    caller->print(troughNumber);
    caller->print(F(" Dispense Stopped. Total Volume: "));
    caller->print(sensor->dispenseVolume, 1);
    caller->println(F(" mL."));

    stopFlowSensorMeasurement(*sensor);
    resetFlowSensorDispenseVolume(*sensor);
  }

  // Update dispensing state in ValveControl
  valveControls[troughNumber - 1].isDispensing = false;
}
