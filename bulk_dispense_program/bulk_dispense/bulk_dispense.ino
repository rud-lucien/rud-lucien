#include <Controllino.h>
#include "SolenoidValve.h"
#include "BubbleSensor.h"
#include "OverflowSensor.h"
#include "ProportionalValve.h"
#include "PressureSensor.h"
#include "FDXSensor.h"
#include "ModbusConnection.h"
#include "Commander-API.hpp"
#include "Commander-IO.hpp"

// Define constants for the generic reagent valve control pins
#define REAGENT_VALVE_1_PIN CONTROLLINO_R0
#define REAGENT_VALVE_2_PIN CONTROLLINO_R1
#define REAGENT_VALVE_3_PIN CONTROLLINO_R2
#define REAGENT_VALVE_4_PIN CONTROLLINO_R3

// Create objects for each reagent valve using the SolenoidValve class
SolenoidValve reagentValve1(REAGENT_VALVE_1_PIN);  // Reagent Valve 1 (Pressure)
SolenoidValve reagentValve2(REAGENT_VALVE_2_PIN);  // Reagent Valve 2 (Pressure)
SolenoidValve reagentValve3(REAGENT_VALVE_3_PIN);  // Reagent Valve 3 (Pressure)
SolenoidValve reagentValve4(REAGENT_VALVE_4_PIN);  // Reagent Valve 4 (Pressure)

// Define constants for the waste bottle valve control pins
#define WASTE_VALVE_1_PIN CONTROLLINO_R4
#define WASTE_VALVE_2_PIN CONTROLLINO_R5

// Create objects for each waste bottle valve
SolenoidValve wasteValve1(WASTE_VALVE_1_PIN);  // Waste Valve 1 (Vacuum)
SolenoidValve wasteValve2(WASTE_VALVE_2_PIN);  // Waste Valve 2 (Vacuum)

// Define constants for the generic media separated valve control pins
#define MEDIA_VALVE_1_PIN CONTROLLINO_DO0
#define MEDIA_VALVE_2_PIN CONTROLLINO_DO1
#define MEDIA_VALVE_3_PIN CONTROLLINO_DO2
#define MEDIA_VALVE_4_PIN CONTROLLINO_DO3

// Create objects for each media valve using the SolenoidValve class
SolenoidValve mediaValve1(MEDIA_VALVE_1_PIN);  // Media Valve 1
SolenoidValve mediaValve2(MEDIA_VALVE_2_PIN);  // Media Valve 2
SolenoidValve mediaValve3(MEDIA_VALVE_3_PIN);  // Media Valve 3
SolenoidValve mediaValve4(MEDIA_VALVE_4_PIN);  // Media Valve 4

// Define constants for the bubble sensor pins (AI0-AI3)
#define REAGENT_1_BUBBLE_SENSOR_PIN CONTROLLINO_AI0
#define REAGENT_2_BUBBLE_SENSOR_PIN CONTROLLINO_AI1
#define REAGENT_3_BUBBLE_SENSOR_PIN CONTROLLINO_AI2
#define REAGENT_4_BUBBLE_SENSOR_PIN CONTROLLINO_AI3

// Create objects for each bubble sensor associated with reagent bottles
BubbleSensor reagent1BubbleSensor(REAGENT_1_BUBBLE_SENSOR_PIN);  // Sensor for Reagent Bottle 1
BubbleSensor reagent2BubbleSensor(REAGENT_2_BUBBLE_SENSOR_PIN);  // Sensor for Reagent Bottle 2
BubbleSensor reagent3BubbleSensor(REAGENT_3_BUBBLE_SENSOR_PIN);  // Sensor for Reagent Bottle 3
BubbleSensor reagent4BubbleSensor(REAGENT_4_BUBBLE_SENSOR_PIN);  // Sensor for Reagent Bottle 4

// Define constants for the overflow sensor pins
#define OVERFLOW_SENSOR_TROUGH_1_PIN CONTROLLINO_DI0
#define OVERFLOW_SENSOR_TROUGH_2_PIN CONTROLLINO_DI1
#define OVERFLOW_SENSOR_TROUGH_3_PIN CONTROLLINO_DI2
#define OVERFLOW_SENSOR_TROUGH_4_PIN CONTROLLINO_DI3

// Create objects for each overflow sensor associated with the troughs
OverflowSensor overflowSensorTrough1(OVERFLOW_SENSOR_TROUGH_1_PIN);  // Overflow Sensor for Trough 1
OverflowSensor overflowSensorTrough2(OVERFLOW_SENSOR_TROUGH_2_PIN);  // Overflow Sensor for Trough 2
OverflowSensor overflowSensorTrough3(OVERFLOW_SENSOR_TROUGH_3_PIN);  // Overflow Sensor for Trough 3
OverflowSensor overflowSensorTrough4(OVERFLOW_SENSOR_TROUGH_4_PIN);  // Overflow Sensor for Trough 4

// Define constants for the control and feedback pins
#define PRESSURE_VALVE_CONTROL_PIN CONTROLLINO_AO0    // Analog Output pin for controlling the valve
#define PRESSURE_VALVE_FEEDBACK_PIN CONTROLLINO_AI13  // Analog Input pin for reading feedback

// Create an object for the pressure valve
ProportionalValve pressureValve(PRESSURE_VALVE_CONTROL_PIN, PRESSURE_VALVE_FEEDBACK_PIN);

// Define constants for the pressure sensor pin and pressure range
#define PRESSURE_SENSOR_PIN CONTROLLINO_AI12
#define MIN_PRESSURE 0.0   // Minimum pressure in psi
#define MAX_PRESSURE 50.0  // Maximum pressure in psi

// Create an object for the pressure sensor
PressureSensor pressureSensor(PRESSURE_SENSOR_PIN, MIN_PRESSURE, MAX_PRESSURE);

// Define constants for the flow sensor pins and reset pins
#define FLOW_SENSOR_REAGENT_1_RESET_PIN CONTROLLINO_DO4
#define FLOW_SENSOR_REAGENT_2_RESET_PIN CONTROLLINO_DO5
#define FLOW_SENSOR_REAGENT_3_RESET_PIN CONTROLLINO_DO6
#define FLOW_SENSOR_REAGENT_4_RESET_PIN CONTROLLINO_DO7

#define FLOW_SENSOR_REAGENT_1_MODBUS_REGISTER 0x0002   // Modbus register for Reagent 1 flow sensor
#define FLOW_SENSOR_REAGENT_2_MODBUS_REGISTER 0x00012  // Modbus register for Reagent 2 flow sensor
#define FLOW_SENSOR_REAGENT_3_MODBUS_REGISTER 0x00022  // Modbus register for Reagent 3 flow sensor
#define FLOW_SENSOR_REAGENT_4_MODBUS_REGISTER 0x00032  // Modbus register for Reagent 4 flow sensor

// Ethernet settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(169, 254, 0, 11);
IPAddress server(169, 254, 0, 10);

// Create a ModbusConnection object
ModbusConnection modbus(mac, ip, server);

// Create objects for each flow sensor connected to the reagent bottles
FDXSensor flowSensorReagent1(modbus, FLOW_SENSOR_REAGENT_1_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_1_RESET_PIN);
FDXSensor flowSensorReagent2(modbus, FLOW_SENSOR_REAGENT_2_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_2_RESET_PIN);
FDXSensor flowSensorReagent3(modbus, FLOW_SENSOR_REAGENT_3_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_3_RESET_PIN);
FDXSensor flowSensorReagent4(modbus, FLOW_SENSOR_REAGENT_4_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_4_RESET_PIN);

// Create the Commander object
Commander commander;

// Create global variables for command arguments
char valveArg[10];
int valveNumber;
int state;
bool isDispensing[4] = {false, false, false, false};
int dispensingValveNumber[4] = {-1, -1, -1, -1};
float targetVolume[4] = {-1, -1, -1, -1}; // Track the target volume for each valve (1-4)


// Global tracking for each valve
struct ValveState {
  bool isDispensing;
  float targetVolume;
  float lastFlowValue;
  unsigned long lastFlowCheckTime;
  unsigned long lastFlowChangeTime;
};
ValveState valveStates[4]; // Track state for valves 1-4

// Create prototypes for the command functions
void cmd_set_reagent_valve(char *args, Stream *response);
void cmd_set_media_valve(char *args, Stream *response);
void cmd_set_waste_valve(char *args, Stream *response);
void cmd_set_pressure_valve(char *args, Stream *response);
void cmd_reset_flow_sensor(char *args, Stream *response);
void cmd_set_log_frequency(char *args, Stream *response);
void cmd_dispense_reagent(char *args, Stream *response);
void cmd_stop_dispense(char *args, Stream *response);
void handleOverflowCondition(int triggeredValveNumber);
void closeValves(int valveNumber, Stream *response);
void openValves(int valveNumber, Stream *response);
void handleTimeoutCondition(int valveNumber);
void logSystemState();

// Command API tree with updated patterns
Commander::API_t API_tree[] = {
  apiElement("setRV", "Set reagent valve: setRV <1-4> <0/1>", cmd_set_reagent_valve),
  apiElement("setMV", "Set media valve: setMV <1-4> <0/1>", cmd_set_media_valve),
  apiElement("setWV", "Set waste valve: setWV <1-2> <0/1>", cmd_set_waste_valve),
  apiElement("setPV", "Set pressure valve: setPV <percentage>", cmd_set_pressure_valve),
  apiElement("resetFS", "Reset flow sensor: resetFS <1-4>", cmd_reset_flow_sensor),
  apiElement("setLF", "Set log frequency: setLF <milliseconds>", cmd_set_log_frequency),
  apiElement("dispenseR", "Dispense reagent: dispenseR <1-4> [volume]", cmd_dispense_reagent),
  apiElement("stopD", "Stop dispensing reagent: stopD <1-4 or all>", cmd_stop_dispense)
};

// Flow sensor reset variables
bool resetInProgress[4] = {false, false, false, false}; // Track reset progress for each valve
unsigned long resetStartTime[4] = {0, 0, 0, 0};             // Track reset start time for each valve
const unsigned long resetDuration = 200;                       // Duration for flow sensor reset in milliseconds


// Define constants for the logging interval
unsigned long previousLogTime = 0; // Store the time of the last log
unsigned long logInterval = 500;   // Default log frequency (1000 ms = 1 second)

// Global variables for timeout management (Keep these to manage the timeout logic)
const unsigned long flowTimeoutPeriod = 10000;            // Timeout period in milliseconds (e.g., 10 seconds)

// Global variables for volume dispensing (Keep these to handle volume-based dispensing for each valve)
const float MIN_VOLUME = 1.0;   // Minimum volume in mL (adjust as needed)
const float MAX_VOLUME = 200.0; // Maximum volume in mL (adjust as needed)
const float VOLUME_TOLERANCE = 0.5;  // Allowable difference in mL

void setup()
{
  Serial.begin(115200); // Start serial communication

  // Initialize valve states
  for (int i = 0; i < 4; i++) {
    valveStates[i].isDispensing = false;
    valveStates[i].targetVolume = -1;
    valveStates[i].lastFlowValue = -1;
    valveStates[i].lastFlowCheckTime = 0;
    valveStates[i].lastFlowChangeTime = 0;
  }

  // Initialize each valve
  reagentValve1.setup();
  reagentValve2.setup();
  reagentValve3.setup();
  reagentValve4.setup();
  wasteValve1.setup();
  wasteValve2.setup();
  mediaValve1.setup();
  mediaValve2.setup();
  mediaValve3.setup();
  mediaValve4.setup();
  reagent1BubbleSensor.setup();
  reagent2BubbleSensor.setup();
  reagent3BubbleSensor.setup();
  reagent4BubbleSensor.setup();
  overflowSensorTrough1.setup();
  overflowSensorTrough2.setup();
  overflowSensorTrough3.setup();
  overflowSensorTrough4.setup();
  pressureValve.setup();
  pressureValve.setPosition(0.0);
  pressureSensor.setup();
  modbus.setupEthernet();

  // Attach the Commander API tree
  commander.attachTree(API_tree);
  commander.init(); // Initialize Commander API
}

void loop()
{
  unsigned long currentTime = millis();

  // Monitor the flow sensor reset progress in the loop for each valve
  for (int i = 0; i < 4; i++)
  {
    if (resetInProgress[i] && millis() - resetStartTime[i] >= resetDuration)
    {
      resetInProgress[i] = false; // Reset is complete
      Serial.print("Flow sensor reset completed for valve ");
      Serial.println(i + 1);
      // Call the actual flow sensor reset function for the specific valve
      if (i == 0)
        flowSensorReagent1.handleReset();
      else if (i == 1)
        flowSensorReagent2.handleReset();
      else if (i == 2)
        flowSensorReagent3.handleReset();
      else if (i == 3)
        flowSensorReagent4.handleReset();
    }
  }

  static char commandBuffer[16]; // Buffer to store incoming command
  static uint8_t commandIndex = 0;

  /// Read incoming serial commands
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      commandBuffer[commandIndex] = '\0';
      commander.execute(commandBuffer, &Serial);
      commandIndex = 0;
    }
    else if (c != '\r')
    {
      commandBuffer[commandIndex++] = c;
      if (commandIndex >= sizeof(commandBuffer) - 1)
      {
        commandIndex = sizeof(commandBuffer) - 1;
      }
    }
  }

  // Monitor the overflow sensors every 50ms
  static unsigned long previousOverflowCheckTime = 0;
  if (currentTime - previousOverflowCheckTime >= 50)
  {
    previousOverflowCheckTime = currentTime;

    if (overflowSensorTrough1.loop() == 1)
      handleOverflowCondition(1);
    if (overflowSensorTrough2.loop() == 1)
      handleOverflowCondition(2);
    if (overflowSensorTrough3.loop() == 1)
      handleOverflowCondition(3);
    if (overflowSensorTrough4.loop() == 1)
      handleOverflowCondition(4);
  }

  // Monitor flow for each valve independently
  for (int i = 0; i < 4; i++)
  {
    if (valveStates[i].isDispensing && currentTime - valveStates[i].lastFlowCheckTime >= 50)
    {
      valveStates[i].lastFlowCheckTime = currentTime;

      // Get the flow value for the current valve
      float currentFlowValue = -1.0;
      if (i == 0)
        currentFlowValue = flowSensorReagent1.getScaledFlowValue();
      else if (i == 1)
        currentFlowValue = flowSensorReagent2.getScaledFlowValue();
      else if (i == 2)
        currentFlowValue = flowSensorReagent3.getScaledFlowValue();
      else if (i == 3)
        currentFlowValue = flowSensorReagent4.getScaledFlowValue();

      // Handle invalid flow readings
      if (currentFlowValue < 0)
      {
        Serial.print("Invalid flow reading for valve ");
        Serial.println(i + 1);
        handleTimeoutCondition(i + 1);
        continue;
      }

      // Check if the target volume is reached, using tolerance
      if (valveStates[i].targetVolume > 0 && currentFlowValue >= valveStates[i].targetVolume)
      {
        Serial.print("Target volume reached for valve ");
        Serial.println(i + 1);
        handleTimeoutCondition(i + 1); // Close valves and stop dispensing
        continue;
      }

      // Handle flow timeout
      if (currentFlowValue == valveStates[i].lastFlowValue)
      {
        if (currentTime - valveStates[i].lastFlowChangeTime >= flowTimeoutPeriod)
        {
          Serial.print("Timeout occurred for valve ");
          Serial.println(i + 1);
          handleTimeoutCondition(i + 1);
        }
      }
      else
      {
        valveStates[i].lastFlowValue = currentFlowValue;
        valveStates[i].lastFlowChangeTime = currentTime;
      }
    }
  }
  // Check Modbus connection status every 500ms (move to an interval-based check)
  static unsigned long previousModbusCheckTime = 0;
  if (currentTime - previousModbusCheckTime >= 500)
  {
    modbus.checkConnection();
    previousModbusCheckTime = currentTime;
  }

  // Log system state every logInterval milliseconds
  if (currentTime - previousLogTime >= logInterval)
  {
    logSystemState();
    previousLogTime = currentTime;
  }
}

void logSystemState() {
  Serial.print("LOG,");

  // Reagent Valve States: RV1-RV4 (0 for Closed, 1 for Open)
  Serial.print("RV,");
  Serial.print(reagentValve1.isValveOpen() ? "1" : "0");
  Serial.print(reagentValve2.isValveOpen() ? "1" : "0");
  Serial.print(reagentValve3.isValveOpen() ? "1" : "0");
  Serial.print(reagentValve4.isValveOpen() ? "1" : "0");

  // Media Valve States: MV1-MV4 (0 for Closed, 1 for Open)
  Serial.print(",MV,");
  Serial.print(mediaValve1.isValveOpen() ? "1" : "0");
  Serial.print(mediaValve2.isValveOpen() ? "1" : "0");
  Serial.print(mediaValve3.isValveOpen() ? "1" : "0");
  Serial.print(mediaValve4.isValveOpen() ? "1" : "0");

  // Waste Valve States: WV1-WV2
  Serial.print(",WV,");
  Serial.print(wasteValve1.isValveOpen() ? "1" : "0");
  Serial.print(wasteValve2.isValveOpen() ? "1" : "0");

  // Bubble Sensor States: BS1-BS4
  Serial.print(",BS,");
  Serial.print(reagent1BubbleSensor.isLiquidDetected() ? "1" : "0");
  Serial.print(reagent2BubbleSensor.isLiquidDetected() ? "1" : "0");
  Serial.print(reagent3BubbleSensor.isLiquidDetected() ? "1" : "0");
  Serial.print(reagent4BubbleSensor.isLiquidDetected() ? "1" : "0");

  // Overflow Sensor States: OV1-OV4
  Serial.print(",OV,");
  Serial.print(overflowSensorTrough1.isOverflowing() ? "1" : "0");
  Serial.print(overflowSensorTrough2.isOverflowing() ? "1" : "0");
  Serial.print(overflowSensorTrough3.isOverflowing() ? "1" : "0");
  Serial.print(overflowSensorTrough4.isOverflowing() ? "1" : "0");

  // Pressure Valve Feedback
  float feedbackVoltage = pressureValve.getFeedback();
  Serial.print(",PV,");
  Serial.print(feedbackVoltage, 2); // Print the feedback voltage with 2 decimal places

  // Pressure Valve Percentage
  float valvePercent = ((feedbackVoltage - 0.0) / (7.0 - 0.0)) * 100;
  Serial.print(",PV%,");
  Serial.print(valvePercent, 1); // Print the valve percentage with 1 decimal place

  // Pressure Sensor Reading
  Serial.print(",PS1,");
  Serial.print(pressureSensor.readPressure());
  Serial.print(" psi");

  // Flow Rates: FS1-FS4 (use getScaledFlowValue() for each sensor)
  Serial.print(",FS1,");
  float flowValue = flowSensorReagent1.getScaledFlowValue();
  if (flowValue >= 0)
  {                             // Ensure valid flow value
    Serial.print(flowValue, 1); // Print with 1 decimal place
  }
  else
  {
    Serial.print("N/A");
  }

  Serial.print(",FS2,");
  flowValue = flowSensorReagent2.getScaledFlowValue();
  if (flowValue >= 0)
  {
    Serial.print(flowValue, 1);
  }
  else
  {
    Serial.print("N/A");
  }

  Serial.print(",FS3,");
  flowValue = flowSensorReagent3.getScaledFlowValue();
  if (flowValue >= 0)
  {
    Serial.print(flowValue, 1);
  }
  else
  {
    Serial.print("N/A");
  }

  Serial.print(",FS4,");
  flowValue = flowSensorReagent4.getScaledFlowValue();
  if (flowValue >= 0)
  {
    Serial.print(flowValue, 1);
  }
  else
  {
    Serial.print("N/A");
  }

  // *** New Logs for Target Volume and Dispensing State ***
  // Dispensing State for Valves 1-4
  Serial.print(",DS,");
  Serial.print(valveStates[0].isDispensing ? "1" : "0");
  Serial.print(valveStates[1].isDispensing ? "1" : "0");
  Serial.print(valveStates[2].isDispensing ? "1" : "0");
  Serial.print(valveStates[3].isDispensing ? "1" : "0");

  // Target Volume for Valves 1-4
  Serial.print(",TV,");
  Serial.print(valveStates[0].targetVolume, 1);
  Serial.print(",");
  Serial.print(valveStates[1].targetVolume, 1);
  Serial.print(",");
  Serial.print(valveStates[2].targetVolume, 1);
  Serial.print(",");
  Serial.print(valveStates[3].targetVolume, 1);

  Serial.println();
}

// Command to set reagent valves (setRV <valve number> <0/1> or setRV all <0/1>)
void cmd_set_reagent_valve(char *args, Stream *response) {
  // Parse the command input (valve number or "all" and state)
  if (sscanf(args, "%s %d", valveArg, &state) == 2) {
    if (strcmp(valveArg, "all") == 0) {
      // Handle the "all" case for reagent valves
      if (state == 0 || state == 1) {
        if (state == 1) {
          reagentValve1.openValve();
          reagentValve2.openValve();
          reagentValve3.openValve();
          reagentValve4.openValve();
          response->println("All reagent valves opened.");
        } else {
          reagentValve1.closeValve();
          reagentValve2.closeValve();
          reagentValve3.closeValve();
          reagentValve4.closeValve();
          response->println("All reagent valves closed.");
        }
      } else {
        response->println("Invalid state. Use 0 or 1.");
      }
    } else {
      // Handle specific valve number
      valveNumber = atoi(valveArg);
      if (valveNumber >= 1 && valveNumber <= 4 && (state == 0 || state == 1)) {
        SolenoidValve *valve = nullptr;
        if (valveNumber == 1) valve = &reagentValve1;
        else if (valveNumber == 2) valve = &reagentValve2;
        else if (valveNumber == 3) valve = &reagentValve3;
        else if (valveNumber == 4) valve = &reagentValve4;

        if (valve != nullptr) {
          if (state == 1) {
            valve->openValve();
            valveStates[valveNumber - 1].isDispensing = true;  // Track dispensing per valve
            response->println("Reagent valve opened.");
          } else {
            valve->closeValve();
            valveStates[valveNumber - 1].isDispensing = false; // Track dispensing per valve
            response->println("Reagent valve closed.");
          }
        }
      } else {
        response->println("Invalid reagent valve command. Use 1-4 or 'all'.");
      }
    }
  } else {
    response->println("Invalid reagent valve command.");
  }
}


// Command to set media valves (setMV <valve number> <0/1> or setMV all <0/1>)
void cmd_set_media_valve(char *args, Stream *response) {
  if (sscanf(args, "%s %d", valveArg, &state) == 2) {
    if (strcmp(valveArg, "all") == 0) {
      // Handle the "all" case for media valves
      if (state == 0 || state == 1) {
        if (state == 1) {
          mediaValve1.openValve();
          mediaValve2.openValve();
          mediaValve3.openValve();
          mediaValve4.openValve();
          response->println("All media valves opened.");
        } else {
          mediaValve1.closeValve();
          mediaValve2.closeValve();
          mediaValve3.closeValve();
          mediaValve4.closeValve();
          response->println("All media valves closed.");
        }
      } else {
        response->println("Invalid state. Use 0 or 1.");
      }
    } else {
      // Handle specific valve number
      valveNumber = atoi(valveArg);
      if (valveNumber >= 1 && valveNumber <= 4 && (state == 0 || state == 1)) {
        SolenoidValve *valve = nullptr;
        if (valveNumber == 1) valve = &mediaValve1;
        else if (valveNumber == 2) valve = &mediaValve2;
        else if (valveNumber == 3) valve = &mediaValve3;
        else if (valveNumber == 4) valve = &mediaValve4;

        if (valve != nullptr) {
          if (state == 1) {
            valve->openValve();
            response->println("Media valve opened.");
          } else {
            valve->closeValve();
            response->println("Media valve closed.");
          }
        }
      } else {
        response->println("Invalid media valve command. Use 1-4 or 'all'.");
      }
    }
  } else {
    response->println("Invalid media valve command.");
  }
}

// Command to set waste valves (setWV <valve number> <0/1> or setWV all <0/1>)
void cmd_set_waste_valve(char *args, Stream *response) {
  if (sscanf(args, "%s %d", valveArg, &state) == 2) {
    if (strcmp(valveArg, "all") == 0) {
      // Handle the "all" case for waste valves
      if (state == 0 || state == 1) {
        if (state == 1) {
          wasteValve1.openValve();
          wasteValve2.openValve();
          response->println("All waste valves opened.");
        } else {
          wasteValve1.closeValve();
          wasteValve2.closeValve();
          response->println("All waste valves closed.");
        }
      } else {
        response->println("Invalid state. Use 0 or 1.");
      }
    } else {
      // Handle specific valve number
      valveNumber = atoi(valveArg); // Convert valveArg to integer
      if (valveNumber >= 1 && valveNumber <= 2 && (state == 0 || state == 1)) {
        SolenoidValve *valve = nullptr;
        if (valveNumber == 1) valve = &wasteValve1;
        else if (valveNumber == 2) valve = &wasteValve2;

        if (valve != nullptr) {
          if (state == 1) {
            valve->openValve();
            response->println("Waste valve opened.");
          } else {
            valve->closeValve();
            response->println("Waste valve closed.");
          }
        }
      } else {
        response->println("Invalid waste valve command. Use 1-2 or 'all'.");
      }
    }
  } else {
    response->println("Invalid waste valve command.");
  }
}

// Command to set pressure valve (setPV <percentage>)
void cmd_set_pressure_valve(char *args, Stream *response) {
  String command = String(args); // Convert args to String object for easier parsing
  String valueStr = command;     // The entire command is the value for PV (since we're passing it directly)

  // Convert the string value to a float and then to an integer percentage
  float value = valueStr.toFloat(); // Convert the string to a float
  int percentage = int(value);      // Convert the float to an integer percentage

  // Validate the percentage range (0-100)
  if (percentage >= 0 && percentage <= 100) {
    pressureValve.setPosition(percentage); // Set the valve to the percentage position
    response->print("Pressure valve set to ");
    response->print(percentage);
    response->println("%.");
  } else {
    response->println("Invalid value for pressure valve. Use a percentage between 0 and 100.");
  }
}

// Command to reset flow sensor (resetFS <sensor number> or resetFS all)
void cmd_reset_flow_sensor(char *args, Stream *response)
{
  // Check if the argument is "all" to reset all sensors
  if (strcmp(args, "all") == 0)
  {
    // Reset all flow sensors
    flowSensorReagent1.startResetFlow();
    flowSensorReagent2.startResetFlow();
    flowSensorReagent3.startResetFlow();
    flowSensorReagent4.startResetFlow();
    response->println("All flow sensors reset initiated.");
  }
  else
  {
    // Otherwise, parse it as a specific sensor number
    int sensorNumber;
    if (sscanf(args, "%d", &sensorNumber) == 1)
    {
      // Reset the specific flow sensor based on sensorNumber
      if (sensorNumber == 1)
      {
        flowSensorReagent1.startResetFlow();
        response->println("Flow sensor 1 reset initiated.");
      }
      else if (sensorNumber == 2)
      {
        flowSensorReagent2.startResetFlow();
        response->println("Flow sensor 2 reset initiated.");
      }
      else if (sensorNumber == 3)
      {
        flowSensorReagent3.startResetFlow();
        response->println("Flow sensor 3 reset initiated.");
      }
      else if (sensorNumber == 4)
      {
        flowSensorReagent4.startResetFlow();
        response->println("Flow sensor 4 reset initiated.");
      }
      else
      {
        response->println("Invalid flow sensor number. Use 1-4 or 'all'.");
      }
    }
    else
    {
      response->println("Invalid flow sensor command.");
    }
  }
}

// Command to set the logging frequency (setLF <milliseconds>)
void cmd_set_log_frequency(char *args, Stream *response)
{
  int newLogInterval;
  if (sscanf(args, "%d", &newLogInterval) == 1 && newLogInterval > 0)
  {
    logInterval = newLogInterval; // Update the global log interval
    response->print("Log frequency set to ");
    response->print(logInterval);
    response->println(" milliseconds.");
  }
  else
  {
    response->println("Invalid log frequency. Please provide a valid number in milliseconds.");
  }
}

// Command to dispense reagent (dispenseR <valve number> [volume])
void cmd_dispense_reagent(char *args, Stream *response) {
  // Convert the char* args to a String for easier parsing
  String inputString = String(args);
  
  // Trim leading and trailing spaces from the input string
  inputString.trim();
  
  // Variables to store the parsed values
  int valveNumber = -1;       // Default to -1, meaning no specific valve requested
  float requestedVolume = -1; // Default to -1, meaning no specific volume requested

  // Find the position of the first space between the valve number and volume
  int spaceIndex = inputString.indexOf(' ');

  // If there's a space, extract and parse the valve number and volume
  if (spaceIndex != -1) {
    String intPart = inputString.substring(0, spaceIndex);
    String floatPart = inputString.substring(spaceIndex + 1);
    
    // Convert the string parts to int and float
    valveNumber = intPart.toInt();
    requestedVolume = floatPart.toFloat();
  } else {
    valveNumber = inputString.toInt();
  }

  response->print("Valve number: ");
  response->println(valveNumber);
  response->print("Requested volume: ");
  response->println(requestedVolume);

  if (valveNumber >= 1 && valveNumber <= 4) {
    // Validate and set volume
    if (requestedVolume > 0) {
      if (requestedVolume < MIN_VOLUME) {
        response->print("Requested volume too low. Minimum volume is ");
        response->print(MIN_VOLUME);
        response->println(" mL.");
        return;
      } else if (requestedVolume > MAX_VOLUME) {
        response->print("Requested volume too high. Maximum volume is ");
        response->print(MAX_VOLUME);
        response->println(" mL.");
        return;
      }
      // Correctly set the target volume for the valve
      valveStates[valveNumber - 1].targetVolume = requestedVolume;  // **Make sure this is being set correctly**
      response->print("Target volume set for valve ");
      response->print(valveNumber);
      response->print(": ");
      response->print(valveStates[valveNumber - 1].targetVolume);
      response->println(" mL");
    } else {
      valveStates[valveNumber - 1].targetVolume = -1;  // Continuous dispense if no specific volume is set
      response->println("Continuous dispensing (no target volume specified).");
    }

    // *** Pressure Valve Check ***
    float currentPressure = pressureSensor.readPressure();
    if (currentPressure < 20.0) { // Adjust threshold as needed
      pressureValve.setPosition(100); // Set pressure valve to 100%
      response->println("Pressure valve set to 100%.");
    } else {
      response->println("System already pressurized.");
    }

    // Reset flow sensor for the specific valve
    resetInProgress[valveNumber - 1] = true; // Track reset progress
    resetStartTime[valveNumber - 1] = millis(); // Record reset start time

     // Reset the flow sensor for the specific valve
    if (valveNumber == 1) flowSensorReagent1.startResetFlow();
    else if (valveNumber == 2) flowSensorReagent2.startResetFlow();
    else if (valveNumber == 3) flowSensorReagent3.startResetFlow();
    else if (valveNumber == 4) flowSensorReagent4.startResetFlow();

    response->print("Flow sensor reset initiated for reagent ");
    response->println(valveNumber);

    // Open the reagent and media valves for the specific valve
    openValves(valveNumber, response);

    // Track the dispensing state for the valve
    valveStates[valveNumber - 1].isDispensing = true;
  } else {
    response->println("Invalid valve number. Use 1-4.");
  }
}


// Command to stop dispensing (stopD <valve number> or stopD all)
void cmd_stop_dispense(char *args, Stream *response)
{
  if (strcmp(args, "all") == 0)
  {
    // Stop all reagent and media valves
    for (int i = 1; i <= 4; i++)
    {
      closeValves(i, response);
      flowSensorReagent1.startResetFlow();
    }
    response->println("All valves and flow sensors reset.");
    for (int i = 0; i < 4; i++)
    {
      isDispensing[i] = false;
      dispensingValveNumber[i] = -1;
    }
  }
  else
  {
    int valveNumber;
    if (sscanf(args, "%d", &valveNumber) == 1 && valveNumber >= 1 && valveNumber <= 4)
    {
      closeValves(valveNumber, response);
      if (valveNumber == 1) flowSensorReagent1.startResetFlow();
      else if (valveNumber == 2) flowSensorReagent2.startResetFlow();
      else if (valveNumber == 3) flowSensorReagent3.startResetFlow();
      else if (valveNumber == 4) flowSensorReagent4.startResetFlow();

      isDispensing[valveNumber - 1] = false;
      dispensingValveNumber[valveNumber - 1] = -1;
    }
    else
    {
      response->println("Invalid valve number. Use 1-4 or 'all'.");
    }
  }
}


void handleOverflowCondition(int triggeredValveNumber)
{
  if (isDispensing[triggeredValveNumber - 1])
  {
    closeValves(triggeredValveNumber, &Serial);
    isDispensing[triggeredValveNumber - 1] = false;
    dispensingValveNumber[triggeredValveNumber - 1] = -1;
    Serial.print("Overflow detected: Valves closed for valve ");
    Serial.println(triggeredValveNumber);
  }
}


// Helper function to close both reagent and media valves for a given valve number
void closeValves(int valveNumber, Stream *response)
{
  SolenoidValve *reagentValve = nullptr;
  SolenoidValve *mediaValve = nullptr;

  if (valveNumber == 1)
  {
    reagentValve = &reagentValve1;
    mediaValve = &mediaValve1;
  }
  else if (valveNumber == 2)
  {
    reagentValve = &reagentValve2;
    mediaValve = &mediaValve2;
  }
  else if (valveNumber == 3)
  {
    reagentValve = &reagentValve3;
    mediaValve = &mediaValve3;
  }
  else if (valveNumber == 4)
  {
    reagentValve = &reagentValve4;
    mediaValve = &mediaValve4;
  }

  if (reagentValve != nullptr)
  {
    reagentValve->closeValve();
    response->print("Reagent valve ");
    response->print(valveNumber);
    response->println(" closed.");
  }

  if (mediaValve != nullptr)
  {
    mediaValve->closeValve();
    response->print("Media valve ");
    response->print(valveNumber);
    response->println(" closed.");
  }
}

// Helper function to open both reagent and media valves for a given valve number
void openValves(int valveNumber, Stream *response)
{
  SolenoidValve *reagentValve = nullptr;
  SolenoidValve *mediaValve = nullptr;

  if (valveNumber == 1)
  {
    reagentValve = &reagentValve1;
    mediaValve = &mediaValve1;
  }
  else if (valveNumber == 2)
  {
    reagentValve = &reagentValve2;
    mediaValve = &mediaValve2;
  }
  else if (valveNumber == 3)
  {
    reagentValve = &reagentValve3;
    mediaValve = &mediaValve3;
  }
  else if (valveNumber == 4)
  {
    reagentValve = &reagentValve4;
    mediaValve = &mediaValve4;
  }

  if (reagentValve != nullptr)
  {
    reagentValve->openValve();
    response->print("Reagent valve ");
    response->print(valveNumber);
    response->println(" opened.");
  }

  if (mediaValve != nullptr)
  {
    mediaValve->openValve();
    response->print("Media valve ");
    response->print(valveNumber);
    response->println(" opened.");
  }
}

void handleTimeoutCondition(int valveNumber) {
  closeValves(valveNumber, &Serial);
  Serial.print("Timeout occurred: Valves closed for valve ");
  Serial.println(valveNumber);

  // Reset the flow sensor for the specific valve after the dispense is completed
  if (valveNumber == 1) flowSensorReagent1.startResetFlow();
  else if (valveNumber == 2) flowSensorReagent2.startResetFlow();
  else if (valveNumber == 3) flowSensorReagent3.startResetFlow();
  else if (valveNumber == 4) flowSensorReagent4.startResetFlow();

  // Reset the valve state
  valveStates[valveNumber - 1].isDispensing = false;
  valveStates[valveNumber - 1].targetVolume = -1;
}
