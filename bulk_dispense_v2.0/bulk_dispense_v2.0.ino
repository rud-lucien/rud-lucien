/*
  Bulk Dispense v2.0 Code
  -----------------------
  This sketch implements ...
*/

// =====================
// Include Libraries
// =====================
#include <Controllino.h>
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include <string.h>  // for strtok
#include <ctype.h>   // for isspace
#include <Wire.h>
#include "Adafruit_SHT31.h"

// =====================
// Global Constants & Pin Definitions
// =====================
#define COMMAND_SIZE 30
#define FAN_CONTROL_PIN CONTROLLINO_R6

// Proportional Valve Pins
#define PROPORTIONAL_VALVE_CONTROL_PIN CONTROLLINO_AO0
#define PROPORTIONAL_VALVE_FEEDBACK_PIN CONTROLLINO_AI13

// Reagent Valve Pins
#define REAGENT_VALVE_1_PIN CONTROLLINO_R0
#define REAGENT_VALVE_2_PIN CONTROLLINO_R1
#define REAGENT_VALVE_3_PIN CONTROLLINO_R2
#define REAGENT_VALVE_4_PIN CONTROLLINO_R3

// Media Valve Pins
#define MEDIA_VALVE_1_PIN CONTROLLINO_DO0
#define MEDIA_VALVE_2_PIN CONTROLLINO_DO1
#define MEDIA_VALVE_3_PIN CONTROLLINO_DO2
#define MEDIA_VALVE_4_PIN CONTROLLINO_DO3

// Waste Valve Pins
#define WASTE_VALVE_1_PIN CONTROLLINO_R4
#define WASTE_VALVE_2_PIN CONTROLLINO_R5
#define WASTE_VALVE_3_PIN CONTROLLINO_R8
#define WASTE_VALVE_4_PIN CONTROLLINO_R9

// Overflow Sensor Pins
#define OVERFLOW_SENSOR_TROUGH_1_PIN CONTROLLINO_DI0
#define OVERFLOW_SENSOR_TROUGH_2_PIN CONTROLLINO_DI1
#define OVERFLOW_SENSOR_TROUGH_3_PIN CONTROLLINO_DI2
#define OVERFLOW_SENSOR_TROUGH_4_PIN CONTROLLINO_DI3

// Bubble Sensor Pins
#define REAGENT_1_BUBBLE_SENSOR_PIN CONTROLLINO_AI0
#define REAGENT_2_BUBBLE_SENSOR_PIN CONTROLLINO_AI1
#define REAGENT_3_BUBBLE_SENSOR_PIN CONTROLLINO_AI2
#define REAGENT_4_BUBBLE_SENSOR_PIN CONTROLLINO_AI3

// Waste Line Liquid Sensor Pins
#define WASTE_1_LIQUID_SENSOR_PIN CONTROLLINO_AI4
#define WASTE_2_LIQUID_SENSOR_PIN CONTROLLINO_AI5

// Waste Bottle Liquid Sensor Pins
#define WASTE_BOTTLE_1_LIQUID_SENSOR_PIN CONTROLLINO_AI6
#define WASTE_BOTTLE_2_LIQUID_SENSOR_PIN CONTROLLINO_AI7

// Waste Bottle Vacuum Sensor Pins
#define WASTE_BOTTLE_1_VACUUM_SENSOR_PIN CONTROLLINO_AI8
#define WASTE_BOTTLE_2_VACUUM_SENSOR_PIN CONTROLLINO_AI9

// Enclosure Liquid Sensor Pin
#define ENCLOSURE_LIQUID_SENSOR_PIN CONTROLLINO_AI10

// Pressure Sensor Pin
#define PRESSURE_SENSOR_PIN CONTROLLINO_AI12

// I2C Multiplexer & SHT31 Sensor (Temp/Humidity) Definitions
#define MULTIPLEXER_ADDR 0x70
#define TEMP_HUM_SENSOR_ADDR 0x44
#define TEMP_HUM_SENSOR_CHANNEL 4

// Flow Sensor: Multiplexer channels 0-3
#define NUM_FLOW_SENSORS 4
#define FLOW_SENSOR_CMD 0x3608  // measurement command for the flow sensor


// Array Sizes
#define NUM_REAGENT_VALVES 4
#define NUM_MEDIA_VALVES 4
#define NUM_WASTE_VALVES 4
#define NUM_OVERFLOW_SENSORS 4
#define NUM_REAGENT_BUBBLE_SENSORS 4
#define NUM_WASTE_LINE_SENSORS 2
#define NUM_WASTE_BOTTLE_SENSORS 2
#define NUM_WASTE_VACUUM_SENSORS 2
#define NUM_ENCLOSURE_LIQUID_SENSORS 1

#define ENCLOSURE_TEMP_SETPOINT 30.0  // Set the temperature threshold (°C)

// =====================
// Struct Prototypes
// =====================
struct OnOffValve;
struct FanControl;
struct LoggingManagement;
struct BinarySensor;
struct ProportionalValve;
struct PressureSensor;
struct TempHumidity;
struct FlowSensor;
struct FlowData;
struct ValveControl;



// =====================
// Command Function Prototypes
// =====================
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


// =====================
// Helper Function Prototypes
// =====================
char* trimLeadingSpaces(char* str);
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
void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel);  // Updated!
bool tempHumSensorInit();
TempHumidity readTempHumidity();
void monitorEnclosureTemp();
bool isFlowSensorConnected(FlowSensor& sensor);
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



// NEW: Flow Sensor Data Structure (for output)
struct FlowData {
  float flowRate;        // mL/min
  float temperature;     // °C
  int highFlowFlag;      // 1 if high flow, else 0
  float dispenseVolume;  // Volume increment since last read
  float totalVolume;     // Cumulative volume
  bool isValidReading;   // Flag indicating if the reading is valid
};

// NEW: Flow Sensor Struct
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
  bool isValidReading;  // Flag indicating if the reading is valid
};

// NEW: Flow Sensor Functions
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

// =====================
// Flow Sensor Functions
// =====================
bool readFlowSensorData(FlowSensor& sensor) {
  static int softResetAttempt = 0;  // Prevent infinite resets

  if (!sensor.sensorInitialized || sensor.sensorStopped) {
    sensor.flowRate = -1;
    sensor.temperature = -1;
    sensor.highFlowFlag = -1;

    if (sensor.totalVolume == 0.0) {
      sensor.dispenseVolume = 0.0;
    }

    return false;
  }

  selectMultiplexerChannel(sensor.multiplexerAddr, sensor.channel);
  Wire.requestFrom(sensor.sensorAddr, (uint8_t)9);

  if (Wire.available() < 9) {
    Serial.print(F("[ERROR] Not enough bytes received from flow sensor on channel "));
    Serial.println(sensor.channel);

    // Attempt soft reset before giving up
    if (softResetAttempt < 2) {
      Serial.println(F("[WARNING] Attempting soft reset to recover..."));
      Wire.beginTransmission(sensor.sensorAddr);
      Wire.write(0x00);
      Wire.write(0x06);
      if (Wire.endTransmission() == 0) {
        delay(25);  // Allow sensor time to reset
        softResetAttempt++;
        return false;  // Retry reading next cycle
      }
    }

    Serial.println(F("[ERROR] Multiple failures. Sensor will remain in error state."));
    sensor.sensorInitialized = false;
    sensor.sensorStopped = true;
    sensor.sensorConnected = 0;
    softResetAttempt = 0;  // Reset counter after giving up
    return false;
  }

  softResetAttempt = 0;  // Reset failure counter on success

  uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Skip CRC
  uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Skip CRC
  uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Skip CRC

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









// =====================
// OnOffValve Functions
// =====================
struct OnOffValve {
  uint8_t controlPin;
  bool isOpen;
};

OnOffValve createValve(uint8_t pin) {
  OnOffValve valve;
  valve.controlPin = pin;
  valve.isOpen = false;
  return valve;
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

// =====================
// ProportionalValve Functions
// =====================
struct ProportionalValve {
  byte controlPin;
  byte feedbackPin;
  float controlVoltage;
};

ProportionalValve createProportionalValve(byte controlPin, byte feedbackPin) {
  ProportionalValve valve;
  valve.controlPin = controlPin;
  valve.feedbackPin = feedbackPin;
  valve.controlVoltage = 0.0;
  return valve;
}

void proportionalValveSetup(const ProportionalValve& valve) {
  pinMode(valve.controlPin, OUTPUT);
  pinMode(valve.feedbackPin, INPUT);
}

ProportionalValve setValvePosition(ProportionalValve valve, float percentage) {
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  valve.controlVoltage = (percentage / 100.0) * 10.0;
  int pwmValue = (int)((valve.controlVoltage / 10.0) * 255);
  analogWrite(valve.controlPin, pwmValue);
  return valve;
}

float getValveFeedback(const ProportionalValve& valve) {
  int analogValue = analogRead(valve.feedbackPin);
  float voltage = map(analogValue, 0, 1023, 0, 10000) / 1000.0;
  return voltage;
}

// Global calibration value (updated during calibration)
float proportionalValveMaxFeedback = 0.0;

// =====================
// Fan Control Functions
// =====================
struct FanControl {
  uint8_t relayPin;
};

void setFanState(const FanControl& config, bool state) {
  static bool lastState = false;  // Store last fan state to avoid redundant logging
  if (lastState != state) {
    digitalWrite(config.relayPin, state ? HIGH : LOW);
    Serial.print(F("[MESSAGE] Fan state set to "));
    Serial.println(state ? F("ON") : F("OFF"));
    lastState = state;  // Update last known state
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

// Global Fan Configuration
const FanControl fan = { FAN_CONTROL_PIN };

// =====================
// Logging Management Functions
// =====================
struct LoggingManagement {
  unsigned long previousLogTime;
  unsigned long logInterval;
};

LoggingManagement logging = { 0, 250 };

void logData(const char* module, const char* message) {
  Serial.print(F("[LOG] "));
  Serial.print(module);
  Serial.print(F(" - "));
  Serial.println(message);
}

// =====================
// BinarySensor Functions
// =====================
struct BinarySensor {
  uint8_t inputPin;
  bool activeHigh;
};

BinarySensor createBinarySensor(uint8_t pin, bool activeHigh) {
  BinarySensor sensor;
  sensor.inputPin = pin;
  sensor.activeHigh = activeHigh;
  return sensor;
}

void binarySensorSetup(const BinarySensor& sensor) {
  pinMode(sensor.inputPin, INPUT);
}

bool readBinarySensor(const BinarySensor& sensor) {
  int reading = digitalRead(sensor.inputPin);
  return sensor.activeHigh ? (reading == HIGH) : (reading == LOW);
}

// =====================
// Pressure Sensor Functions
// =====================
struct PressureSensor {
  byte analogPin;
  float minPressure;
  float maxPressure;
};

PressureSensor createPressureSensor(byte analogPin, float minPressure, float maxPressure) {
  PressureSensor sensor;
  sensor.analogPin = analogPin;
  sensor.minPressure = minPressure;
  sensor.maxPressure = maxPressure;
  return sensor;
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

// New: Temperature & Humidity Sensor Struct
struct TempHumidity {
  float temperature;
  float humidity;
  bool valid;
};

// =====================
// Temperature & Humidity Sensor Functions
// =====================

// Create a global SHT31 instance
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// =====================
// I2C Multiplexer Function (Updated)
// =====================
void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel) {
  Wire.beginTransmission(multiplexerAddr);
  Wire.write(1 << channel);  // Select the specific channel
  Wire.endTransmission();
}

// Initialize the SHT31 sensor through the multiplexer
bool tempHumSensorInit() {
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  return sht31.begin(TEMP_HUM_SENSOR_ADDR);
}

// Read temperature and humidity data from the sensor
TempHumidity readTempHumidity() {
  TempHumidity data;
  selectMultiplexerChannel(MULTIPLEXER_ADDR, TEMP_HUM_SENSOR_CHANNEL);
  data.temperature = sht31.readTemperature();
  data.humidity = sht31.readHumidity();
  data.valid = !(isnan(data.temperature) || isnan(data.humidity));
  return data;
}


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

// Define global array to track state for each trough
ValveControl valveControls[NUM_OVERFLOW_SENSORS] = {};


// =====================
// Global Hardware Instances
// =====================

// OnOffValve Instances
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

// Proportional Valve Instance
ProportionalValve proportionalValve = createProportionalValve(PROPORTIONAL_VALVE_CONTROL_PIN, PROPORTIONAL_VALVE_FEEDBACK_PIN);

// Pressure Sensor Instance (0 to 87 psi)
PressureSensor pressureSensor = createPressureSensor(PRESSURE_SENSOR_PIN, 0, 87);

// BinarySensor Instances (Individual)
BinarySensor overflowSensorTrough1 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_1_PIN, true);
BinarySensor overflowSensorTrough2 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_2_PIN, true);
BinarySensor overflowSensorTrough3 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_3_PIN, true);
BinarySensor overflowSensorTrough4 = createBinarySensor(OVERFLOW_SENSOR_TROUGH_4_PIN, true);
BinarySensor reagent1BubbleSensor1 = createBinarySensor(REAGENT_1_BUBBLE_SENSOR_PIN, true);
BinarySensor reagent1BubbleSensor2 = createBinarySensor(REAGENT_2_BUBBLE_SENSOR_PIN, true);
BinarySensor reagent1BubbleSensor3 = createBinarySensor(REAGENT_3_BUBBLE_SENSOR_PIN, true);
BinarySensor reagent1BubbleSensor4 = createBinarySensor(REAGENT_4_BUBBLE_SENSOR_PIN, true);
BinarySensor waste1LiquidSensor = createBinarySensor(WASTE_1_LIQUID_SENSOR_PIN, true);
BinarySensor waste2LiquidSensor = createBinarySensor(WASTE_2_LIQUID_SENSOR_PIN, true);
BinarySensor overflowSensorWasteBottle1 = createBinarySensor(WASTE_BOTTLE_1_LIQUID_SENSOR_PIN, true);
BinarySensor overflowSensorWasteBottle2 = createBinarySensor(WASTE_BOTTLE_2_LIQUID_SENSOR_PIN, true);
BinarySensor waste1VacuumSensor = createBinarySensor(WASTE_BOTTLE_1_VACUUM_SENSOR_PIN, true);
BinarySensor waste2VacuumSensor = createBinarySensor(WASTE_BOTTLE_2_VACUUM_SENSOR_PIN, true);
BinarySensor enclosureLiquidSensor = createBinarySensor(ENCLOSURE_LIQUID_SENSOR_PIN, false);

// Global Sensor Arrays (for easier looping)
BinarySensor overflowSensors[NUM_OVERFLOW_SENSORS] = {
  overflowSensorTrough1, overflowSensorTrough2,
  overflowSensorTrough3, overflowSensorTrough4
};

BinarySensor reagentBubbleSensors[NUM_REAGENT_BUBBLE_SENSORS] = {
  reagent1BubbleSensor1, reagent1BubbleSensor2,
  reagent1BubbleSensor3, reagent1BubbleSensor4
};

BinarySensor wasteLineSensors[NUM_WASTE_LINE_SENSORS] = {
  waste1LiquidSensor, waste2LiquidSensor
};

BinarySensor wasteBottleSensors[NUM_WASTE_BOTTLE_SENSORS] = {
  overflowSensorWasteBottle1, overflowSensorWasteBottle2
};

BinarySensor wasteVacuumSensors[NUM_WASTE_VACUUM_SENSORS] = {
  waste1VacuumSensor, waste2VacuumSensor
};

// Create Flow Sensor Instances for channels 0–3 on the multiplexer.
// Adjust sensorAddr and measurementCmd as needed for your sensor.
FlowSensor flow0 = createFlowSensor(0x70, 0x08, 0, FLOW_SENSOR_CMD);
FlowSensor flow1 = createFlowSensor(0x70, 0x08, 1, FLOW_SENSOR_CMD);
FlowSensor flow2 = createFlowSensor(0x70, 0x08, 2, FLOW_SENSOR_CMD);
FlowSensor flow3 = createFlowSensor(0x70, 0x08, 3, FLOW_SENSOR_CMD);

// Create an array to store references to the sensors
FlowSensor* flowSensors[NUM_FLOW_SENSORS] = { &flow0, &flow1, &flow2, &flow3 };

// (For single-instance sensors like the enclosure liquid sensor and pressure sensor, we simply use the variable.)

// =====================
// Commander API Command Functions
// =====================
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

void cmd_fan(char* args, CommandCaller* caller) {
  int fanState = -1;
  if (sscanf(args, "%d", &fanState) == 1 && (fanState == 0 || fanState == 1)) {
    bool state = (fanState == 1);
    setFanState(fan, state);
    printFanState(state);
    // Disable auto mode when a manual command is used.
    fanAutoMode = false;
    caller->println(F("[MESSAGE] Fan manual override active. Use FNAUTO to re-enable auto control."));
  } else {
    caller->println(F("[ERROR] Invalid fan command. Use: FN <0/1>"));
  }
}

void cmd_fan_auto(char* args, CommandCaller* caller) {
  fanAutoMode = true;
  caller->println(F("[MESSAGE] Fan auto control re-enabled."));
}

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

void cmd_set_media_valve(char* args, CommandCaller* caller) {
  int valveNumber = -1, valveState = -1;
  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 && valveNumber >= 1 && valveNumber <= NUM_MEDIA_VALVES && (valveState == 0 || valveState == 1)) {
    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Media valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");

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

void cmd_set_waste_valve(char* args, CommandCaller* caller) {
  int valveNumber = -1, valveState = -1;
  if (sscanf(args, "%d %d", &valveNumber, &valveState) == 2 && valveNumber >= 1 && valveNumber <= NUM_WASTE_VALVES && (valveState == 0 || valveState == 1)) {
    bool state = (valveState == 1);

    caller->print(F("[MESSAGE] Waste valve "));
    caller->print(valveNumber);
    caller->print(F(" set to "));
    caller->println(state ? "OPEN" : "CLOSED");

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

void cmd_calibrate_pressure_valve(char* args, CommandCaller* caller) {
  caller->println(F("[MESSAGE] Calibrating pressure valve, please wait..."));
  calibrateProportionalValve();
  // After calibration, you might choose to close the valve:
  // proportionalValve = setValvePosition(proportionalValve, 0.0);
  caller->println(F("[MESSAGE] Pressure valve calibration complete."));
}


// =====================
// Command: STARTFSM <1-4> - Manually Start Flow Sensor Measurement
// =====================
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


// =====================
// Command: STOPFSM <1-4> - Manually Stop Flow Sensor Measurement
// =====================
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

void cmd_reset_flow_dispense(char* args, CommandCaller* caller) {
  int sensorNumber = -1;
  if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber <= 3) {
    FlowSensor* sensors[] = { &flow0, &flow1, &flow2, &flow3 };
    resetFlowSensorDispenseVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset dispense volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RESETFS <0-3>"));
  }
}

void cmd_reset_flow_total(char* args, CommandCaller* caller) {
  int sensorNumber = -1;
  if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 0 && sensorNumber <= 3) {
    FlowSensor* sensors[] = { &flow0, &flow1, &flow2, &flow3 };
    resetFlowSensorTotalVolume(*sensors[sensorNumber]);
    caller->print(F("[MESSAGE] Reset total volume for Flow Sensor "));
    caller->println(sensorNumber);
  } else {
    caller->println(F("[ERROR] Invalid sensor number. Use: RESETTOTALFS <0-3>"));
  }
}

void cmd_reset_i2c(char* args, CommandCaller* caller) {
  Serial.println(F("[MESSAGE] Manual I2C bus reset initiated."));
  resetI2CBus();
  caller->println(F("[MESSAGE] I2C bus reset complete."));
}

// ======================[Command Function for Dispensing]===============
// Command: D <1-4> [volume] - Dispense reagent to a trough
void cmd_dispense_reagent(char* args, CommandCaller* caller) {
  int troughNumber = -1;
  float requestedVolume = -1.0;
  const float MIN_VOLUME = 1.0;
  const float MAX_VOLUME = 200.0;

  // **Command Acknowledgment**
  caller->print(F("[MESSAGE] Received command: D "));
  caller->println(args);

  // **Parse Input**
  int parsedItems = sscanf(args, "%d %f", &troughNumber, &requestedVolume);
  if (parsedItems == 1) {
    requestedVolume = -1.0;  // Continuous mode
  } else if (parsedItems != 2) {
    caller->println(F("[ERROR] Invalid command format. Use: D <1-4> [volume]"));
    return;
  }

  if (troughNumber < 1 || troughNumber > 4) {
    caller->println(F("[ERROR] Invalid trough number. Use 1-4."));
    return;
  }

  // **Check if a dispense is already in progress**
  if (valveControls[troughNumber - 1].isDispensing) {
    caller->print(F("[WARNING] A dispense is already in progress for Trough "));
    caller->println(troughNumber);
    caller->println(F("Use STOPD <trough number> to stop it first."));
    return;
  }



  // **Validate requested volume (if specified)**
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

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const int VALVE_POSITION = 100;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  // **Pressure Check**
  if (!checkAndSetPressure(PRESSURE_THRESHOLD_PSI, VALVE_POSITION, PRESSURE_TIMEOUT_MS)) {
    caller->println(F("[ERROR] Pressure check failed. Dispense aborted."));
    return;
  }

  // **Get Flow Sensor**
  FlowSensor* sensor = flowSensors[troughNumber - 1];
  if (!sensor) {
    caller->print(F("[ERROR] No flow sensor found for Trough "));
    caller->println(troughNumber);
    return;
  }

  // **Check for Overflow Before Dispensing**
  if (readBinarySensor(overflowSensors[troughNumber - 1])) {
    caller->print(F("[ERROR] Cannot dispense: Overflow detected for Trough "));
    caller->println(troughNumber);
    return;
  }


  // **Start Flow Sensor Measurement**
  if (!startFlowSensorMeasurement(*sensor)) {
    caller->print(F("[ERROR] Failed to start flow sensor for Trough "));
    caller->println(troughNumber);
    return;
  }

  caller->print(F("[MESSAGE] Flow sensor measurement started for Trough "));
  caller->println(troughNumber);

  openDispenseValves(troughNumber);
  caller->print(F("[MESSAGE] Dispensing started for Trough "));
  caller->println(troughNumber);

  valveControls[troughNumber - 1].isDispensing = true;
  valveControls[troughNumber - 1].targetVolume = requestedVolume;
}




// =====================
// Command: STOPD <1-4> | STOPD all - Stop Dispensing Operation
// =====================
// - STOPD <1-4>: Stops dispensing operation for a specific trough (1-4).
// - STOPD all  : Stops all dispensing operations immediately.
// - Ensures valves are closed and flow sensors are stopped/reset properly.
// - Displays the total dispensed volume for each trough before resetting.
void cmd_stop_dispense(char* args, CommandCaller* caller) {
  int troughNumber = -1;
  bool stopAll = false;

  // Check if the command is to stop all dispensing operations
  if (strncmp(args, "all", 3) == 0) {
    stopAll = true;
  } else if (sscanf(args, "%d", &troughNumber) != 1 || troughNumber < 1 || troughNumber > NUM_OVERFLOW_SENSORS) {
    caller->println(F("[ERROR] Invalid trough number. Use STOPD <1-4> or STOPD all."));
    return;
  }

  if (stopAll) {
    // **Stop all dispensing operations**
    caller->println(F("[MESSAGE] Stopping all dispensing operations..."));

    for (int i = 1; i <= NUM_OVERFLOW_SENSORS; i++) {
      stopDispenseOperation(i, caller);
    }

    caller->println(F("[MESSAGE] All dispensing operations stopped."));
  } else {
    // **Stop dispense for specific trough**
    stopDispenseOperation(troughNumber, caller);
    caller->print(F("[MESSAGE] Dispensing stopped for Trough "));
    caller->println(troughNumber);
  }
}



// =====================
// Commander API Setup
// =====================
Commander commander;

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

// =====================
// Global Command Buffer
// =====================
char commandBuffer[COMMAND_SIZE];

// =====================
// Setup Function (Refined I2C Reset & Flow Sensor Initialization)
// =====================
void setup() {
  Serial.begin(115200);
  Serial.println(F("[MESSAGE] System starting..."));

  // Fan Setup
  fanSetup(fan);

  // Proportional Valve & Pressure Sensor Setup
  proportionalValveSetup(proportionalValve);
  calibrateProportionalValve();

  // **Check System Air Pressure After Calibration**
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
  setValvePosition(proportionalValve, 0.0);  // Close valve after calibration

  // **Initialize Flow Sensors**
  Serial.println(F("[MESSAGE] Initializing Flow Sensors..."));

  bool allFailed = true;      // Assume all fail, then disprove if any succeed
  bool anyStopped = false;    // Track if any sensors are intentionally stopped
  String failedSensors = "";  // Track failed sensor indices
  FlowSensor* sensors[] = { &flow0, &flow1, &flow2, &flow3 };

  for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
    if (sensors[i]->sensorStopped) {
      anyStopped = true;
      continue;  // Skip initialization for stopped sensors
    }

    if (initializeFlowSensor(*sensors[i])) {
      allFailed = false;  // At least one sensor succeeded
    } else {
      failedSensors += String(i) + " ";  // Store failed sensor numbers
    }
  }

  // **Only reset I2C if all flow sensors failed**
  if (allFailed) {
    Serial.println(F("[WARNING] All flow sensors failed. Resetting I2C bus..."));
    resetI2CBus();
    delay(50);

    // Retry only for failed sensors
    failedSensors = "";  // Reset failed list
    allFailed = true;    // Reassume all will fail until proven otherwise

    for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
      if (!sensors[i]->sensorInitialized && !sensors[i]->sensorStopped) {
        if (initializeFlowSensor(*sensors[i])) {
          allFailed = false;  // At least one succeeded
        } else {
          failedSensors += String(i) + " ";
        }
      }
    }
  }

  // **Final success/failure messages**
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

  // Delay for sensor stabilization
  delay(500);

  // **Initialize Temperature/Humidity Sensor**
  if (!tempHumSensorInit()) {
    Serial.println(F("[ERROR] Temp/Humidity sensor not detected!"));
  } else {
    Serial.println(F("[MESSAGE] Temp/Humidity sensor initialized successfully."));
  }

  // **Commander API Initialization**
  commander.attachTree(API_tree);
  commander.init();

  Serial.println(F("[MESSAGE] System ready."));
}





// =====================
// Main Loop
// =====================
void loop() {
  unsigned long currentTime = millis();

  // Process serial commands
  handleSerialCommands();

  // Monitor Sensors
  monitorOverflowSensors(currentTime);
  monitorFlowSensors(currentTime);
  monitorReagentBubbleSensors(currentTime);
  monitorWasteLineSensors(currentTime);
  monitorWasteBottleSensors(currentTime);
  monitorWasteVacuumSensors(currentTime);
  monitorEnclosureLiquidSensor(currentTime);
  monitorEnclosureTemp();
  monitorFlowSensorConnections();

  // Update Flow Sensors (you can call readFlowSensorData() for each flow sensor)
  readFlowSensorData(flow0);
  readFlowSensorData(flow1);
  readFlowSensorData(flow2);
  readFlowSensorData(flow3);

  // Log system state periodically
  if (currentTime - logging.previousLogTime >= logging.logInterval) {
    logging.previousLogTime = currentTime;
    logSystemState();
  }
}

// =====================
// Helper Functions: Serial Command Handling
// =====================
char* trimLeadingSpaces(char* str) {
  while (*str && isspace(*str)) {
    str++;
  }
  return str;
}

void processMultipleCommands(char* commandLine, Stream* stream) {
  char* token = strtok(commandLine, ",");  // Split at commas

  while (token != NULL) {
    token = trimLeadingSpaces(token);

    if (strlen(token) > 0) {
      // Log each parsed command separately
      Serial.print(F("[COMMAND] Processing: "));
      Serial.println(token);

      commander.execute(token, stream);
    }

    token = strtok(NULL, ",");  // Move to next command
  }
}


void handleSerialCommands() {
  static char commandBuffer[COMMAND_SIZE];
  static uint8_t commandIndex = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      commandBuffer[commandIndex] = '\0';  // Null-terminate the command

      // Log the received command before processing
      Serial.print(F("[COMMAND] Received: "));
      Serial.println(commandBuffer);

      // Process and execute the command
      processMultipleCommands(commandBuffer, &Serial);

      // Reset command buffer
      commandIndex = 0;
    } else if (c != '\r') {
      // Store character in buffer (if within limit)
      if (commandIndex < (COMMAND_SIZE - 1)) {
        commandBuffer[commandIndex++] = c;
      }
    }
  }
}


// =====================
// Helper Functions: Logging System State
// =====================
void logSystemState() {
  char buffer[300];

  // Fan state
  char fState = (digitalRead(fan.relayPin) == HIGH ? '1' : '0');

  // Valve states
  char rv1 = (reagentValve1.isOpen ? '1' : '0');
  char rv2 = (reagentValve2.isOpen ? '1' : '0');
  char rv3 = (reagentValve3.isOpen ? '1' : '0');
  char rv4 = (reagentValve4.isOpen ? '1' : '0');
  char mv1 = (mediaValve1.isOpen ? '1' : '0');
  char mv2 = (mediaValve2.isOpen ? '1' : '0');
  char mv3 = (mediaValve3.isOpen ? '1' : '0');
  char mv4 = (mediaValve4.isOpen ? '1' : '0');
  char wv1 = (wasteValve1.isOpen ? '1' : '0');
  char wv2 = (wasteValve2.isOpen ? '1' : '0');
  char wv3 = (wasteValve3.isOpen ? '1' : '0');
  char wv4 = (wasteValve4.isOpen ? '1' : '0');

  // Proportional Valve Feedback
  char pvStr[8];
  dtostrf(getValveFeedback(proportionalValve), 4, 1, pvStr);
  float feedback = getValveFeedback(proportionalValve);
  float valvePercent = (feedback / proportionalValveMaxFeedback) * 100.0;
  char pvPercentStr[8];
  dtostrf(valvePercent, 4, 1, pvPercentStr);

  // Waste Line Sensor states
  char wsl1 = (readBinarySensor(wasteLineSensors[0]) ? '1' : '0');
  char wsl2 = (readBinarySensor(wasteLineSensors[1]) ? '1' : '0');

  // Waste Bottle Sensor states
  char wbl1 = (readBinarySensor(wasteBottleSensors[0]) ? '1' : '0');
  char wbl2 = (readBinarySensor(wasteBottleSensors[1]) ? '1' : '0');

  // Waste Vacuum Sensor states
  char wvs1 = (readBinarySensor(wasteVacuumSensors[0]) ? '1' : '0');
  char wvs2 = (readBinarySensor(wasteVacuumSensors[1]) ? '1' : '0');

  // Enclosure Liquid Sensor state
  char els1 = (readBinarySensor(enclosureLiquidSensor) ? '1' : '0');

  // Bubble Sensor states
  char bs1 = (readBinarySensor(reagent1BubbleSensor1) ? '1' : '0');
  char bs2 = (readBinarySensor(reagent1BubbleSensor2) ? '1' : '0');
  char bs3 = (readBinarySensor(reagent1BubbleSensor3) ? '1' : '0');
  char bs4 = (readBinarySensor(reagent1BubbleSensor4) ? '1' : '0');

  // Overflow Sensor states
  char os1 = (readBinarySensor(overflowSensorTrough1) ? '1' : '0');
  char os2 = (readBinarySensor(overflowSensorTrough2) ? '1' : '0');
  char os3 = (readBinarySensor(overflowSensorTrough3) ? '1' : '0');
  char os4 = (readBinarySensor(overflowSensorTrough4) ? '1' : '0');

  // Pressure Sensor Reading
  float pressureValue = readPressure(pressureSensor);
  char pressureStr[8];
  dtostrf(pressureValue, 4, 1, pressureStr);

  // Temperature & Humidity Reading (from SHT31 sensor)
  TempHumidity th = readTempHumidity();
  char tempStr[8], humStr[8];
  if (th.valid) {
    dtostrf(th.temperature, 4, 1, tempStr);
    dtostrf(th.humidity, 4, 1, humStr);
  } else {
    strcpy(tempStr, "-1");
    strcpy(humStr, "-1");
  }

  // Flow Sensor Data: Format flow rate, temperature, dispense volume, total volume, and high flow flag
  char f0Rate[8], f0Temp[8], f1Rate[8], f1Temp[8], f2Rate[8], f2Temp[8], f3Rate[8], f3Temp[8];
  char f0Disp[8], f0Total[8], f1Disp[8], f1Total[8], f2Disp[8], f2Total[8], f3Disp[8], f3Total[8];
  char f0Flag[4], f1Flag[4], f2Flag[4], f3Flag[4];

  dtostrf(flow0.isValidReading ? flow0.flowRate : -1, 4, 1, f0Rate);
  dtostrf(flow0.isValidReading ? flow0.temperature : -1, 4, 1, f0Temp);
  dtostrf(flow0.dispenseVolume, 4, 1, f0Disp);  // Always log accumulated dispense volume
  dtostrf(flow0.totalVolume, 4, 1, f0Total);    // Always log accumulated total volume
  snprintf(f0Flag, sizeof(f0Flag), "%d", flow0.isValidReading ? flow0.highFlowFlag : -1);

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


  sprintf(buffer, "[LOG] F%c, RV%c%c%c%c, MV%c%c%c%c, WV%c%c%c%c, PV,%s, PV%%,%s, WSL%c%c, WBL%c%c, WVS%c%c, ELS%c, BS%c%c%c%c, OS%c%c%c%c, PS,%s, ETS,%s, EHS,%s, FS1,%s,%s,%s,%s,%s; FS2,%s,%s,%s,%s,%s; FS3,%s,%s,%s,%s,%s; FS4,%s,%s,%s,%s,%s",
          fState, rv1, rv2, rv3, rv4, mv1, mv2, mv3, mv4, wv1, wv2, wv3, wv4, pvStr, pvPercentStr,
          wsl1, wsl2, wbl1, wbl2, wvs1, wvs2, els1, bs1, bs2, bs3, bs4, os1, os2, os3, os4,
          pressureStr, tempStr, humStr,
          f0Rate, f0Temp, f0Disp, f0Total, f0Flag, f1Rate, f1Temp, f1Disp, f1Total, f1Flag,
          f2Rate, f2Temp, f2Disp, f2Total, f2Flag, f3Rate, f3Temp, f3Disp, f3Total, f3Flag);
  Serial.println(buffer);
}


// =====================
// Monitor Overflow Sensors (Non-blocking)
// =====================
void monitorOverflowSensors(unsigned long currentTime) {
  static unsigned long previousOverflowCheckTime = 0;

  // Check every 25ms (non-blocking)
  if (currentTime - previousOverflowCheckTime >= 25) {
    previousOverflowCheckTime = currentTime;

    // Iterate over all 4 troughs
    for (int i = 0; i < NUM_OVERFLOW_SENSORS; i++) {
      if (readBinarySensor(overflowSensors[i])) {
        handleOverflowCondition(i + 1);  // Handle overflow for trough i+1
      }
    }
  }
}

// =====================
// Monitor Flow Sensors (Non-blocking) with Timeout
// =====================
void monitorFlowSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  const unsigned long FLOW_TIMEOUT_MS = 5000;
  const float MIN_FLOW_RATE_THRESHOLD = 1.0;
  const float MAX_TROUGH_VOLUME = 205.0;  // Maximum safe volume per trough (backup safety)

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

      // **Check for overflow**
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

      // **Check for No Flow Timeout**
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

      // **Check if requested volume is reached**
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

      // ** Secondary Safety: Check if Maximum Trough Volume is Exceeded**
      if (sensor->dispenseVolume >= MAX_TROUGH_VOLUME) {
        closeDispenseValves(i + 1);
        Serial.print(F("[ERROR] Safety Limit Reached! Dispense stopped for Trough "));
        Serial.print(i + 1);
        Serial.print(F(". Final dispensed volume: "));
        Serial.print(sensor->dispenseVolume, 1);  // Log exact final volume
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





void monitorReagentBubbleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_REAGENT_BUBBLE_SENSORS; i++) {
      if (readBinarySensor(reagentBubbleSensors[i])) {
        // Bubble handling logic.
      }
    }
  }
}

void monitorWasteLineSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_LINE_SENSORS; i++) {
      if (readBinarySensor(wasteLineSensors[i])) {
        // Waste line handling logic.
      }
    }
  }
}

void monitorWasteBottleSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_BOTTLE_SENSORS; i++) {
      if (readBinarySensor(wasteBottleSensors[i])) {
        // Waste bottle handling logic.
      }
    }
  }
}

void monitorWasteVacuumSensors(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    for (int i = 0; i < NUM_WASTE_VACUUM_SENSORS; i++) {
      if (readBinarySensor(wasteVacuumSensors[i])) {
        // Waste vacuum handling logic.
      }
    }
  }
}

void monitorEnclosureLiquidSensor(unsigned long currentTime) {
  static unsigned long previousCheckTime = 0;
  if (currentTime - previousCheckTime >= 25) {
    previousCheckTime = currentTime;
    if (readBinarySensor(enclosureLiquidSensor)) {
      // Enclosure liquid handling logic.
    }
  }
}

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

void monitorFlowSensorConnections() {
  FlowSensor* sensors[] = { &flow0, &flow1, &flow2, &flow3 };
  for (int i = 0; i < NUM_FLOW_SENSORS; i++) {
    if (!isFlowSensorConnected(*sensors[i])) {
      sensors[i]->sensorConnected = 0;  // Mark as disconnected
    }
  }
}


void calibrateProportionalValve() {
  Serial.println(F("[MESSAGE] Starting proportional valve calibration..."));
  proportionalValve = setValvePosition(proportionalValve, 100.0);
  delay(1000);  // Allow time for the valve to settle and feedback to stabilize
  proportionalValveMaxFeedback = getValveFeedback(proportionalValve);
  Serial.print(F("[MESSAGE] Calibrated max feedback voltage: "));
  Serial.println(proportionalValveMaxFeedback);
}

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

// Function to reset the I2C bus
void resetI2CBus() {
  Serial.println(F("[MESSAGE] Resetting I2C bus..."));
  Wire.end();
  delay(100);
  Wire.begin();
}

void resetFlowSensorDispenseVolume(FlowSensor& sensor) {
  sensor.dispenseVolume = 0.0;
  sensor.lastUpdateTime = millis();
  sensor.sensorStopped = true;  // Ensure sensor is actually stopped

  Serial.print(F("[MESSAGE] Dispense volume reset for flow sensor on channel "));
  Serial.println(sensor.channel);
}


void resetFlowSensorTotalVolume(FlowSensor& sensor) {
  sensor.totalVolume = 0.0;
  Serial.print(F("[MESSAGE] Total volume reset for flow sensor on channel "));
  Serial.println(sensor.channel);
}

bool startFlowSensorMeasurement(FlowSensor& sensor) {
  Serial.print(F("[DEBUG] Attempting to start flow measurement for sensor on channel "));
  Serial.println(sensor.channel);

  if (!isFlowSensorConnected(sensor)) {
    Serial.print(F("[ERROR] Cannot start measurement for flow sensor on channel "));
    Serial.print(sensor.channel);
    Serial.println(F(" because it is disconnected."));
    return false;
  }

  // **Reset `sensorStopped` before initialization**
  sensor.sensorStopped = false;

  // **Retry Initialization Before Failing**
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
    return true;  // **Return success**
  } else {
    Serial.print(F("[ERROR] Failed to stop measurement for flow sensor on channel "));
    Serial.println(sensor.channel);

    return false;  // **Return failure**
  }
}



// =====================
// Handle Overflow Condition
// =====================
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



// =====================
// Handle Timeout Conditions
// =====================
void handleTimeoutCondition(int troughNumber) {
  if (valveControls[troughNumber - 1].isDispensing) {
    Serial.print(F("[ERROR] Timeout: No or insufficient flow detected for Trough "));
    Serial.print(troughNumber);
    Serial.println(F(". Stopping dispense."));

    // Close dispense valves
    closeDispenseValves(troughNumber);

    // Stop and reset flow sensor
    FlowSensor* sensor = flowSensors[troughNumber - 1];
    if (sensor) {
      Serial.print(F("[MESSAGE] Dispensed volume before timeout: "));
      Serial.print(sensor->dispenseVolume, 1);
      Serial.println(F(" mL."));
      stopFlowSensorMeasurement(*sensor);
      resetFlowSensorDispenseVolume(*sensor);
    }

    // **Additional Resets**
    valveControls[troughNumber - 1].isDispensing = false;
    valveControls[troughNumber - 1].lastFlowCheckTime = 0;
    valveControls[troughNumber - 1].lastFlowChangeTime = 0;
    valveControls[troughNumber - 1].dispensingValveNumber = -1;
    valveControls[troughNumber - 1].targetVolume = -1;
  }
}





// =====================
// Close Reagent & Media Valves for a Given Trough
// =====================
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


// =====================
// Open Reagent & Media Valves for a Given Trough
// =====================
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

// ======================[Helper function for checking system pressure]==================
bool checkAndSetPressure(float thresholdPressure, float valvePosition, unsigned long timeout) {
  unsigned long pressureCheckStartTime = millis();
  float currentPressure = readPressure(pressureSensor);
  float currentValvePosition = proportionalValve.controlVoltage;  // Get current valve setting

  // If system is already pressurized and valve is at the commanded position, move on
  if (currentPressure >= thresholdPressure && currentValvePosition == valvePosition) {
    Serial.println(F("[MESSAGE] System is already pressurized and valve is at the correct position."));
    return true;
  }

  // If pressure is below threshold OR valve needs adjusting, update settings
  setValvePosition(proportionalValve, valvePosition);
  Serial.print(F("[MESSAGE] Pressure valve set to "));
  Serial.print(valvePosition);
  Serial.println(F("%. Waiting for pressure stabilization..."));

  // Wait for pressure to stabilize within timeout
  while (millis() - pressureCheckStartTime < timeout) {
    currentPressure = readPressure(pressureSensor);
    if (currentPressure >= thresholdPressure) {
      Serial.println(F("[MESSAGE] Pressure threshold reached."));
      return true;  // Pressure is sufficient
    }
    delay(100);  // Short delay to avoid tight looping
  }

  // If pressure is not reached, log error and return false
  Serial.print(F("[ERROR] Pressure threshold not reached. Current pressure: "));
  Serial.print(currentPressure);
  Serial.println(F(" psi. Operation aborted."));

  return false;
}

// =====================
// Helper Function: Stop Dispense Operation for a Specific Trough
// =====================
// - Closes reagent and media valves for the given trough.
// - Stops and resets the flow sensor for that trough.
// - Prints the final dispensed volume before resetting.
void stopDispenseOperation(int troughNumber, CommandCaller* caller) {
  // **Close valves for the given trough**
  closeDispenseValves(troughNumber);

  // **Stop and reset the flow sensor for that trough**
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

  // **Update dispensing state using ValveControl**
  valveControls[troughNumber - 1].isDispensing = false;
}
