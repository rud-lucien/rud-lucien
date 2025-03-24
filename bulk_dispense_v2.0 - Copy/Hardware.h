#ifndef HARDWARE_H
#define HARDWARE_H

#include <Controllino.h>
#include "Adafruit_SHT31.h"

// ------------------------------------------------------------------
// Global Constants & Pin Definitions
// ------------------------------------------------------------------

// General
#define COMMAND_SIZE 30
#define ENCLOSURE_TEMP_SETPOINT 30.0  // °C for fan activation

// Fan
#define FAN_CONTROL_PIN CONTROLLINO_R6

// Proportional Valve
#define PROPORTIONAL_VALVE_CONTROL_PIN CONTROLLINO_AO0
#define PROPORTIONAL_VALVE_FEEDBACK_PIN CONTROLLINO_AI13

// Reagent Valves
#define NUM_REAGENT_VALVES 4
extern const uint8_t REAGENT_VALVES[NUM_REAGENT_VALVES];

// Media Valves
#define NUM_MEDIA_VALVES 4
extern const uint8_t MEDIA_VALVES[NUM_MEDIA_VALVES];

// Waste Valves
#define NUM_WASTE_VALVES 4
extern const uint8_t WASTE_VALVES[NUM_WASTE_VALVES];

// Overflow Sensors
#define NUM_OVERFLOW_SENSORS 4
extern const uint8_t OVERFLOW_SENSORS[NUM_OVERFLOW_SENSORS];

// Reagent Bubble Sensors
#define NUM_REAGENT_BUBBLE_SENSORS 4
extern const uint8_t BUBBLE_SENSORS[NUM_REAGENT_BUBBLE_SENSORS];

// Waste Line Sensors
#define NUM_WASTE_LINE_SENSORS 2
extern const uint8_t WASTE_LINE_SENSORS[NUM_WASTE_LINE_SENSORS];

// Waste Bottle Sensors
#define NUM_WASTE_BOTTLE_SENSORS 2
extern const uint8_t WASTE_BOTTLE_SENSORS[NUM_WASTE_BOTTLE_SENSORS];

// Waste Vacuum Sensors
#define NUM_WASTE_VACUUM_SENSORS 2
extern const uint8_t WASTE_VACUUM_SENSORS[NUM_WASTE_VACUUM_SENSORS];

// Enclosure Liquid Sensor
#define ENCLOSURE_LIQUID_SENSOR_PIN CONTROLLINO_AI10

// Pressure Sensor
#define PRESSURE_SENSOR_PIN CONTROLLINO_AI12

// I2C and Flow Sensor parameters
#define MULTIPLEXER_ADDR 0x70
#define TEMP_HUM_SENSOR_ADDR 0x44
#define TEMP_HUM_SENSOR_CHANNEL 4
#define NUM_FLOW_SENSORS 4
#define FLOW_SENSOR_CMD 0x3608

// ------------------------------------------------------------------
// Structure Definitions
// ------------------------------------------------------------------

// On/Off Valve Structure
struct OnOffValve {
  uint8_t controlPin;
  bool isOpen;
};

// Fan Control Structure
struct FanControl {
  uint8_t relayPin;
};

// Proportional Valve Structure
struct ProportionalValve {
  uint8_t controlPin;
  uint8_t feedbackPin;
  float controlVoltage;
};

// Pressure Sensor Structure
struct PressureSensor {
  uint8_t analogPin;
  float minPressure;
  float maxPressure;
};

// Temperature & Humidity Sensor Structure (SHT31)
struct TempHumidity {
  float temperature;
  float humidity;
  bool valid;
};

// Flow Sensor Structure
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

// Valve Control Structure (for dispense operations)
struct ValveControl {
  bool isDispensing;
  bool manualControl;
  bool isPriming;
  bool fillMode;
  bool isDraining;
  float targetVolume;
  float lastFlowValue;
  unsigned long lastFlowCheckTime;
  unsigned long lastFlowChangeTime;
  unsigned long fillCheckTime;
  int dispensingValveNumber;
};

// Binary Sensor Structure
struct BinarySensor {
  uint8_t inputPin;
  bool activeHigh;
};

// ------------------------------------------------------------------
// Extern Global Hardware Object Declarations
// ------------------------------------------------------------------


// Constant arrays (definitions provided in Hardware.cpp)
extern const uint8_t REAGENT_VALVES[NUM_REAGENT_VALVES];
extern const uint8_t MEDIA_VALVES[NUM_MEDIA_VALVES];
extern const uint8_t WASTE_VALVES[NUM_WASTE_VALVES];
extern const uint8_t OVERFLOW_SENSORS[NUM_OVERFLOW_SENSORS];
extern const uint8_t BUBBLE_SENSORS[NUM_REAGENT_BUBBLE_SENSORS];
extern const uint8_t WASTE_LINE_SENSORS[NUM_WASTE_LINE_SENSORS];
extern const uint8_t WASTE_BOTTLE_SENSORS[NUM_WASTE_BOTTLE_SENSORS];
extern const uint8_t WASTE_VACUUM_SENSORS[NUM_WASTE_VACUUM_SENSORS];

// Global hardware objects
extern const FanControl fan;
extern bool fanAutoMode;



extern ProportionalValve proportionalValve;
extern PressureSensor pressureSensor;
extern Adafruit_SHT31 sht31;  // SHT31 instance

// Flow Sensors (for channels 0-3)
extern FlowSensor flow1, flow2, flow3, flow4;
extern FlowSensor *flowSensors[NUM_FLOW_SENSORS];

// Valve Control Array (one per overflow sensor/trough)
extern ValveControl valveControls[NUM_OVERFLOW_SENSORS];

// On/Off Valves
extern OnOffValve reagentValve1, reagentValve2, reagentValve3, reagentValve4;
extern OnOffValve mediaValve1, mediaValve2, mediaValve3, mediaValve4;
extern OnOffValve wasteValve1, wasteValve2, wasteValve3, wasteValve4;

// Binary Sensors
extern BinarySensor overflowSensors[NUM_OVERFLOW_SENSORS];
extern BinarySensor reagentBubbleSensors[NUM_REAGENT_BUBBLE_SENSORS];
extern BinarySensor wasteLineSensors[NUM_WASTE_LINE_SENSORS];
extern BinarySensor wasteBottleSensors[NUM_WASTE_BOTTLE_SENSORS];
extern BinarySensor wasteVacuumSensors[NUM_WASTE_VACUUM_SENSORS];
extern BinarySensor enclosureLiquidSensor;

// Calibration variable
extern float proportionalValveMaxFeedback;

// Global vacuum monitoring flags for bottle 1 and 2.
extern bool globalVacuumMonitoring[NUM_WASTE_VACUUM_SENSORS];

extern bool globalEnclosureLiquidError;

extern bool dispenseAsyncCompleted[NUM_OVERFLOW_SENSORS];
extern bool drainAsyncCompleted[NUM_OVERFLOW_SENSORS];

// ------------------------------------------------------------------
// Function Prototypes (Hardware Functions)
// ------------------------------------------------------------------

void fanSetup(const FanControl &fc);
void setFanState(const FanControl &config, bool state);
void printFanState(bool state);


OnOffValve openValve(OnOffValve valve);
OnOffValve closeValve(OnOffValve valve);

void proportionalValveSetup(const ProportionalValve &valve);
ProportionalValve setValvePosition(ProportionalValve valve, float percentage);
float getValveFeedback(const ProportionalValve &valve);
void calibrateProportionalValve();

void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel);

bool readBinarySensor(const BinarySensor &sensor);

#endif  // HARDWARE_H
