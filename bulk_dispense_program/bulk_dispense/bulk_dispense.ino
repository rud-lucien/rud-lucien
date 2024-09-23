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
void handleTimeoutCondition(int triggeredValveNumber);
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
  apiElement("stopD", "Stop dispensing reagent: stopDispenseR <1-4>", cmd_stop_dispense)
};

// Define constants for the logging interval
unsigned long previousLogTime = 0;  // Store the time of the last log
unsigned long logInterval = 500;    // Default log frequency (1000 ms = 1 second)



// Shared command parsing variables
char valveArg[10];  // Used to store valve number or "all"
int valveNumber;    // Used for storing the valve number
int state;          // Used for storing the valve open/close state (0 or 1)

bool isDispensing = false;       // Track whether a dispense operation is in progress
int dispensingValveNumber = -1;  // Global variable to track the valve currently dispensing, initialized to -1 (none)

// Global variables for timeout management
unsigned long lastFlowCheckTime = 0;            // Tracks when we last checked the flow sensor
unsigned long lastFlowChangeTime = 0;           // Tracks the last time the flow value changed
float lastFlowValue = -1.0;                     // Stores the last known flow value
const unsigned long flowTimeoutPeriod = 10000;  // Timeout period in milliseconds (e.g., 10 seconds)

// Global variables for volume dispensing
const float MIN_VOLUME = 1.0;    // Minimum volume in mL (adjust as needed)
const float MAX_VOLUME = 200.0;  // Maximum volume in mL (adjust as needed)
float targetVolume = -1;         // The volume requested by the user


void setup() {
  Serial.begin(115200);  // Start serial communication

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
  commander.init();  // Initialize Commander API
}


void loop() {

  static unsigned long previousOverflowCheckTime = 0;

  // Get the current time
  unsigned long currentTime = millis();

  static char commandBuffer[16];  // Buffer to store incoming command
  static uint8_t commandIndex = 0;

  /// Read incoming serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      commandBuffer[commandIndex] = '\0';
      commander.execute(commandBuffer, &Serial);
      commandIndex = 0;
    } else if (c != '\r') {
      commandBuffer[commandIndex++] = c;
      if (commandIndex >= sizeof(commandBuffer) - 1) {
        commandIndex = sizeof(commandBuffer) - 1;
      }
    }
  }

  // Monitor the overflow sensors every 100ms
  if (currentTime - previousOverflowCheckTime >= 50) {
    previousOverflowCheckTime = currentTime;

    // Monitor all overflow sensors continuously
    if (overflowSensorTrough1.loop() == 1) {
      // Serial.println("Overflow detected on sensor 1.");
      handleOverflowCondition(1);  // Handle overflow by closing valves directly
    }
    if (overflowSensorTrough2.loop() == 1) {
      // Serial.println("Overflow detected on sensor 2.");
      handleOverflowCondition(2);
    }
    if (overflowSensorTrough3.loop() == 1) {
      // Serial.println("Overflow detected on sensor 3.");
      handleOverflowCondition(3);
    }
    if (overflowSensorTrough4.loop() == 1) {
      // Serial.println("Overflow detected on sensor 4.");
      handleOverflowCondition(4);
    }
  }

  // Check flow sensor activity and volume accumulation if dispensing is in progress
  if (isDispensing && currentTime - lastFlowCheckTime >= 50) {  // Check flow every 50 milliseconds
    lastFlowCheckTime = currentTime;

    // Read the current flow value from the active flow sensor
    float currentFlowValue = -1.0;
    if (dispensingValveNumber == 1) currentFlowValue = flowSensorReagent1.getScaledFlowValue();
    else if (dispensingValveNumber == 2) currentFlowValue = flowSensorReagent2.getScaledFlowValue();
    else if (dispensingValveNumber == 3) currentFlowValue = flowSensorReagent3.getScaledFlowValue();
    else if (dispensingValveNumber == 4) currentFlowValue = flowSensorReagent4.getScaledFlowValue();

    if (currentFlowValue >= 0) {
      // Directly compare the current flow sensor value with the target volume
      
      if (targetVolume > 0) {  // Only compare if a positive target volume was provided
        // Stop when the target volume is reached
        if (currentFlowValue >= targetVolume) {
          Serial.print("Target volume reached: ");
          Serial.print(currentFlowValue);
          Serial.println(" mL.");
          handleTimeoutCondition(dispensingValveNumber);  // Stop dispensing
          return;                                         // Exit the loop since dispensing has stopped
        }
      }

      // Handle timeout if the flow value hasn't changed for the flowTimeoutPeriod
      if (currentFlowValue == lastFlowValue) {
        if (currentTime - lastFlowChangeTime >= flowTimeoutPeriod) {
          Serial.print("Timeout occurred: No flow detected for valve ");
          Serial.println(dispensingValveNumber);
          handleTimeoutCondition(dispensingValveNumber);  // Stop dispensing due to timeout
          return;                                         // Exit the loop since dispensing has stopped
        }
      } else {
        // Update lastFlowValue for the next loop
        lastFlowValue = currentFlowValue;
        // Flow has changed, reset the flow change timer
        lastFlowChangeTime = currentTime;
      }
    }
  }

  // Handle reset process for each flow sensor
  flowSensorReagent1.handleReset();
  flowSensorReagent2.handleReset();
  flowSensorReagent3.handleReset();
  flowSensorReagent4.handleReset();

  modbus.checkConnection();

  // Check if enough time has passed since the last log
  if (currentTime - previousLogTime >= logInterval) {
    logSystemState();               // Call the log function
    previousLogTime = currentTime;  // Update the last log time
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
  Serial.print(feedbackVoltage, 2);  // Print the feedback voltage with 2 decimal places

  // Pressure Valve Percentage
  float valvePercent = ((feedbackVoltage - 0.0) / (7.0 - 0.0)) * 100;
  Serial.print(",PV%,");
  Serial.print(valvePercent, 1);  // Print the valve percentage with 1 decimal place

  // Pressure Sensor Reading
  Serial.print(",PS1,");
  Serial.print(pressureSensor.readPressure());
  Serial.print(" psi");

  // Flow Rates: FS1-FS4 (use getScaledFlowValue() for each sensor)
  Serial.print(",FS1,");
  float flowValue = flowSensorReagent1.getScaledFlowValue();
  if (flowValue >= 0) {          // Ensure valid flow value
    Serial.print(flowValue, 1);  // Print with 1 decimal place
  } else {
    Serial.print("N/A");
  }

  Serial.print(",FS2,");
  flowValue = flowSensorReagent2.getScaledFlowValue();
  if (flowValue >= 0) {
    Serial.print(flowValue, 1);
  } else {
    Serial.print("N/A");
  }

  Serial.print(",FS3,");
  flowValue = flowSensorReagent3.getScaledFlowValue();
  if (flowValue >= 0) {
    Serial.print(flowValue, 1);
  } else {
    Serial.print("N/A");
  }

  Serial.print(",FS4,");
  flowValue = flowSensorReagent4.getScaledFlowValue();
  if (flowValue >= 0) {
    Serial.print(flowValue, 1);
  } else {
    Serial.print("N/A");
  }

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
            response->println("Reagent valve opened.");
          } else {
            valve->closeValve();
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
  // Reuse global variables for command parsing
  if (sscanf(args, "%s %d", valveArg, &state) == 2) {
    if (strcmp(valveArg, "all") == 0) {
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
  // Parse the command input (valve number or "all" and state)
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
      valveNumber = atoi(valveArg);  // Convert valveArg to integer
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
  String command = String(args);  // Convert args to String object for easier parsing
  String valueStr = command;      // The entire command is the value for PV (since we're passing it directly)

  // Convert the string value to a float and then to an integer percentage
  float value = valueStr.toFloat();  // Convert the string to a float
  int percentage = int(value);       // Convert the float to an integer percentage

  // Validate the percentage range (0-100)
  if (percentage >= 0 && percentage <= 100) {
    pressureValve.setPosition(percentage);  // Set the valve to the percentage position
    response->print("Pressure valve set to ");
    response->print(percentage);
    response->println("%.");
  } else {
    response->println("Invalid value for pressure valve. Use a percentage between 0 and 100.");
  }
}

// Command to reset flow sensor (resetFS <sensor number> or resetFS all)
void cmd_reset_flow_sensor(char *args, Stream *response) {
  // Check if the argument is "all" to reset all sensors
  if (strcmp(args, "all") == 0) {
    // Reset all flow sensors
    flowSensorReagent1.startResetFlow();
    flowSensorReagent2.startResetFlow();
    flowSensorReagent3.startResetFlow();
    flowSensorReagent4.startResetFlow();
    response->println("All flow sensors reset initiated.");
  } else {
    // Otherwise, parse it as a specific sensor number
    int sensorNumber;
    if (sscanf(args, "%d", &sensorNumber) == 1) {
      // Reset the specific flow sensor based on sensorNumber
      if (sensorNumber == 1) {
        flowSensorReagent1.startResetFlow();
        response->println("Flow sensor 1 reset initiated.");
      } else if (sensorNumber == 2) {
        flowSensorReagent2.startResetFlow();
        response->println("Flow sensor 2 reset initiated.");
      } else if (sensorNumber == 3) {
        flowSensorReagent3.startResetFlow();
        response->println("Flow sensor 3 reset initiated.");
      } else if (sensorNumber == 4) {
        flowSensorReagent4.startResetFlow();
        response->println("Flow sensor 4 reset initiated.");
      } else {
        response->println("Invalid flow sensor number. Use 1-4 or 'all'.");
      }
    } else {
      response->println("Invalid flow sensor command.");
    }
  }
}

// Command to set the logging frequency (setLF <milliseconds>)
void cmd_set_log_frequency(char *args, Stream *response) {
  int newLogInterval;
  if (sscanf(args, "%d", &newLogInterval) == 1 && newLogInterval > 0) {
    logInterval = newLogInterval;  // Update the global log interval
    response->print("Log frequency set to ");
    response->print(logInterval);
    response->println(" milliseconds.");
  } else {
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
  int valveNumber = -1;       // Default to -1 to indicate invalid/unset
  float requestedVolume = -1; // Default to -1, meaning no specific volume requested

  // Find the position of the first space between the valve number and volume
  int spaceIndex = inputString.indexOf(' ');
  
  // If there's a space, extract and parse the valve number and volume
  if (spaceIndex != -1) {
    // Extract the integer part (valve number) before the space
    String intPart = inputString.substring(0, spaceIndex);
    
    // Extract the float part (volume) after the space
    String floatPart = inputString.substring(spaceIndex + 1);
    
    // Convert the string parts to int and float
    valveNumber = intPart.toInt();       // Convert the valve number to an int
    requestedVolume = floatPart.toFloat(); // Convert the volume to a float
  } else {
    // If no space, assume only the valve number was provided (continuous dispense)
    valveNumber = inputString.toInt();  // Convert the entire input as an int for valve number
  }

  // Step 1: Reset the flow sensor first (before any parsing or dispensing logic)
  response->println("Starting flow sensor reset...");

  if (valveNumber == 1) {
    flowSensorReagent1.startResetFlow();
  } else if (valveNumber == 2) {
    flowSensorReagent2.startResetFlow();
  } else if (valveNumber == 3) {
    flowSensorReagent3.startResetFlow();
  } else if (valveNumber == 4) {
    flowSensorReagent4.startResetFlow();
  }

  response->print("Flow sensor reset for reagent ");
  response->println(valveNumber);

  // Step 2: Check if the valve number is valid (should be between 1 and 4)
  if (valveNumber >= 1 && valveNumber <= 4) {
    // Step 3: Handle the optional volume argument
    if (requestedVolume > 0) {
      // Volume was provided, validate the target volume
      if (requestedVolume < MIN_VOLUME) {
        response->print("Requested volume too low. Minimum volume is ");
        response->print(MIN_VOLUME);
        response->println(" mL.");
        return;  // Stop further execution if volume is too low
      } else if (requestedVolume > MAX_VOLUME) {
        response->print("Requested volume too high. Maximum volume is ");
        response->print(MAX_VOLUME);
        response->println(" mL.");
        return;  // Stop further execution if volume is too high
      }
      // Set target volume
      targetVolume = requestedVolume;  // Set the requested target volume
      response->print("Target volume set: ");
      response->println(targetVolume);
    } else {
      // If no valid volume was specified, this is a continuous dispense
      targetVolume = -1;  // No target volume (dispense indefinitely)
      response->println("Continuous dispensing (no target volume specified).");
    }

    // Step 4: Check the overflow sensor for the specific valve
    OverflowSensor *overflowSensor = nullptr;
    if (valveNumber == 1) overflowSensor = &overflowSensorTrough1;
    else if (valveNumber == 2) overflowSensor = &overflowSensorTrough2;
    else if (valveNumber == 3) overflowSensor = &overflowSensorTrough3;
    else if (valveNumber == 4) overflowSensor = &overflowSensorTrough4;

    if (overflowSensor != nullptr && overflowSensor->isOverflowing()) {
      response->print("Cannot dispense: Overflow detected for valve ");
      response->println(valveNumber);
      return;  // Stop the function if overflow is detected
    }

    // Step 5: Check if the pressure valve needs to be set to 100%
    float currentPressure = pressureSensor.readPressure();
    float pressureThreshold = 20.0;  // Example pressure threshold in psi (adjust as needed)

    if (currentPressure < pressureThreshold) {
      pressureValve.setPosition(100);  // Set pressure valve to 100%
      response->println("Pressure valve set to 100%.");
    } else {
      response->println("System already pressurized.");
    }

    // Step 6: Open the reagent and media valves
    SolenoidValve *reagentValve = nullptr;
    SolenoidValve *mediaValve = nullptr;

    if (valveNumber == 1) {
      reagentValve = &reagentValve1;
      mediaValve = &mediaValve1;
    } else if (valveNumber == 2) {
      reagentValve = &reagentValve2;
      mediaValve = &mediaValve2;
    } else if (valveNumber == 3) {
      reagentValve = &reagentValve3;
      mediaValve = &mediaValve3;
    } else if (valveNumber == 4) {
      reagentValve = &reagentValve4;
      mediaValve = &mediaValve4;
    }

    if (reagentValve != nullptr) {
      reagentValve->openValve();
      response->print("Reagent valve ");
      response->print(valveNumber);
      response->println(" opened.");
    }
    if (mediaValve != nullptr) {
      mediaValve->openValve();
      response->print("Media valve ");
      response->print(valveNumber);
      response->println(" opened.");
    }

    // Step 7: Handle dispensing state and volume tracking
    isDispensing = true;  // Set the dispensing state to true
    dispensingValveNumber = valveNumber;  // Track the currently dispensing valve
  } else {
    // If the valve number is invalid, notify the user
    response->println("Invalid valve number. Use 1-4.");
  }
}




// Command to stop dispensing (stopD <valve number> or stopD all)
void cmd_stop_dispense(char *args, Stream *response) {
  // Check if the argument is "all" to stop all valves
  if (strcmp(args, "all") == 0) {
    // Stop all reagent and media valves (emergency stop)
    reagentValve1.closeValve();
    reagentValve2.closeValve();
    reagentValve3.closeValve();
    reagentValve4.closeValve();
    mediaValve1.closeValve();
    mediaValve2.closeValve();
    mediaValve3.closeValve();
    mediaValve4.closeValve();

    // Print messages for all valves closed
    response->println("Emergency stop: All reagent and media valves closed.");
    
    // Reset the dispensing state and active valve tracker
    isDispensing = false;
    dispensingValveNumber = -1;  // Reset the dispensing valve tracker

  } else {
    // Otherwise, parse it as a specific valve number
    int valveNumber;
    if (sscanf(args, "%d", &valveNumber) == 1) {
      // Check if the valve number is valid
      if (valveNumber >= 1 && valveNumber <= 4) {
        // Step 1: Close the reagent valve for the specific valve number
        SolenoidValve *reagentValve = nullptr;
        if (valveNumber == 1) reagentValve = &reagentValve1;
        else if (valveNumber == 2) reagentValve = &reagentValve2;
        else if (valveNumber == 3) reagentValve = &reagentValve3;
        else if (valveNumber == 4) reagentValve = &reagentValve4;

        if (reagentValve != nullptr) {
          reagentValve->closeValve();
          response->print("Reagent valve ");
          response->print(valveNumber);
          response->println(" closed.");
        }

        // Step 2: Close the media valve for the specific valve number
        SolenoidValve *mediaValve = nullptr;
        if (valveNumber == 1) mediaValve = &mediaValve1;
        else if (valveNumber == 2) mediaValve = &mediaValve2;
        else if (valveNumber == 3) mediaValve = &mediaValve3;
        else if (valveNumber == 4) mediaValve = &mediaValve4;

        if (mediaValve != nullptr) {
          mediaValve->closeValve();
          response->print("Media valve ");
          response->print(valveNumber);
          response->println(" closed.");
        }

        // Step 3: Reset the dispensing state and active valve tracker
        isDispensing = false;
        dispensingValveNumber = -1;  // Reset the dispensing valve tracker

      } else {
        response->println("Invalid valve number. Use 1-4.");
      }
    } else {
      response->println("Invalid stop command.");
    }
  }
}


void handleOverflowCondition(int triggeredValveNumber) {
  // Check if the system is actively dispensing and if the overflow occurred on the currently dispensing valve
  if (isDispensing && triggeredValveNumber == dispensingValveNumber) {
    // Stop dispensing for the specific valve
    SolenoidValve *reagentValve = nullptr;
    SolenoidValve *mediaValve = nullptr;

    // Determine which valves to close based on the valve number
    if (triggeredValveNumber == 1) {
      reagentValve = &reagentValve1;
      mediaValve = &mediaValve1;
    } else if (triggeredValveNumber == 2) {
      reagentValve = &reagentValve2;
      mediaValve = &mediaValve2;
    } else if (triggeredValveNumber == 3) {
      reagentValve = &reagentValve3;
      mediaValve = &mediaValve3;
    } else if (triggeredValveNumber == 4) {
      reagentValve = &reagentValve4;
      mediaValve = &mediaValve4;
    }

    // Close the reagent and media valves directly
    if (reagentValve != nullptr) {
      reagentValve->closeValve();
      Serial.print("Reagent valve ");
      Serial.print(triggeredValveNumber);
      Serial.println(" closed.");
    }
    if (mediaValve != nullptr) {
      mediaValve->closeValve();
      Serial.print("Media valve ");
      Serial.print(triggeredValveNumber);
      Serial.println(" closed.");
    }

    // Reset the dispensing state and active valve tracker
    isDispensing = false;
    dispensingValveNumber = -1;  // Reset the dispensing valve tracker

    // Additional action: You can add any additional actions or alerts here (e.g., log the overflow event)
  }
}

void handleTimeoutCondition(int triggeredValveNumber) {
  // Stop dispensing for the specific valve due to timeout
  SolenoidValve *reagentValve = nullptr;
  SolenoidValve *mediaValve = nullptr;

  // Determine which valves to close based on the valve number
  if (triggeredValveNumber == 1) {
    reagentValve = &reagentValve1;
    mediaValve = &mediaValve1;
  } else if (triggeredValveNumber == 2) {
    reagentValve = &reagentValve2;
    mediaValve = &mediaValve2;
  } else if (triggeredValveNumber == 3) {
    reagentValve = &reagentValve3;
    mediaValve = &mediaValve3;
  } else if (triggeredValveNumber == 4) {
    reagentValve = &reagentValve4;
    mediaValve = &mediaValve4;
  }

  // Close the reagent and media valves directly
  if (reagentValve != nullptr) {
    reagentValve->closeValve();
    Serial.print("Reagent valve ");
    Serial.print(triggeredValveNumber);
    Serial.println(" closed due to timeout.");
  }
  if (mediaValve != nullptr) {
    mediaValve->closeValve();
    Serial.print("Media valve ");
    Serial.print(triggeredValveNumber);
    Serial.println(" closed due to timeout.");
  }

  // Reset the dispensing state and active valve tracker
  isDispensing = false;
  dispensingValveNumber = -1;  // Reset the dispensing valve tracker
}
