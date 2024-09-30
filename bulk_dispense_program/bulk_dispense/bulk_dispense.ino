// ======================[Includes and Library Dependencies]======================
#include <Controllino.h>
#include <LibPrintf.h>
#include "SolenoidValve.h"
#include "BubbleSensor.h"
#include "OverflowSensor.h"
#include "ProportionalValve.h"
#include "PressureSensor.h"
#include "FDXSensor.h"
#include "ModbusConnection.h"
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include "TCPServer.h"

// ======================[Pin Assignments and Constants]==========================

// Reagent Valves
#define REAGENT_VALVE_1_PIN CONTROLLINO_R0
#define REAGENT_VALVE_2_PIN CONTROLLINO_R1
#define REAGENT_VALVE_3_PIN CONTROLLINO_R2
#define REAGENT_VALVE_4_PIN CONTROLLINO_R3

// Waste Valves
#define WASTE_VALVE_1_PIN CONTROLLINO_R4
#define WASTE_VALVE_2_PIN CONTROLLINO_R5

// Media Valves
#define MEDIA_VALVE_1_PIN CONTROLLINO_DO0
#define MEDIA_VALVE_2_PIN CONTROLLINO_DO1
#define MEDIA_VALVE_3_PIN CONTROLLINO_DO2
#define MEDIA_VALVE_4_PIN CONTROLLINO_DO3

// Bubble Sensors
#define REAGENT_1_BUBBLE_SENSOR_PIN CONTROLLINO_AI0
#define REAGENT_2_BUBBLE_SENSOR_PIN CONTROLLINO_AI1
#define REAGENT_3_BUBBLE_SENSOR_PIN CONTROLLINO_AI2
#define REAGENT_4_BUBBLE_SENSOR_PIN CONTROLLINO_AI3

// Overflow Sensors
#define OVERFLOW_SENSOR_TROUGH_1_PIN CONTROLLINO_DI0
#define OVERFLOW_SENSOR_TROUGH_2_PIN CONTROLLINO_DI1
#define OVERFLOW_SENSOR_TROUGH_3_PIN CONTROLLINO_DI2
#define OVERFLOW_SENSOR_TROUGH_4_PIN CONTROLLINO_DI3

// Pressure Control and Feedback
#define PRESSURE_VALVE_CONTROL_PIN CONTROLLINO_AO0   // Analog Output for control
#define PRESSURE_VALVE_FEEDBACK_PIN CONTROLLINO_AI13 // Analog Input for feedback

// Pressure Sensor
#define PRESSURE_SENSOR_PIN CONTROLLINO_AI12
#define MIN_PRESSURE 0.0  // psi
#define MAX_PRESSURE 50.0 // psi

// Flow Sensors and Modbus Registers
#define FLOW_SENSOR_REAGENT_1_RESET_PIN CONTROLLINO_DO4
#define FLOW_SENSOR_REAGENT_2_RESET_PIN CONTROLLINO_DO5
#define FLOW_SENSOR_REAGENT_3_RESET_PIN CONTROLLINO_DO6
#define FLOW_SENSOR_REAGENT_4_RESET_PIN CONTROLLINO_DO7

#define FLOW_SENSOR_REAGENT_1_MODBUS_REGISTER 0x0002
#define FLOW_SENSOR_REAGENT_2_MODBUS_REGISTER 0x00012
#define FLOW_SENSOR_REAGENT_3_MODBUS_REGISTER 0x00022
#define FLOW_SENSOR_REAGENT_4_MODBUS_REGISTER 0x00032

// Ethernet/Modbus Settings
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(169, 254, 0, 11);
// Create instances of TCPServer and ModbusConnection
TCPServer tcpServer(ip, 8080);  // Use port 8080 for the TCP server
ModbusConnection modbus(mac, ip, IPAddress(169, 254, 0, 10));  // Modbus server IP

// ======================[Global Command Variables]===============================
// Variables for handling command arguments and valve states
char valveArg[10];                                   // Stores command arguments
int valveNumber = 0;                                 // Tracks which valve number is being used
int state = 0;                                       // Tracks the state of the valve (open/closed)
bool isDispensing[4] = {false, false, false, false}; // Tracks whether each valve is dispensing
int dispensingValveNumber[4] = {-1, -1, -1, -1};     // Tracks the current dispensing valve number (-1 if none)
float targetVolume[4] = {-1, -1, -1, -1};            // Stores target volume for each valve

bool fillMode[4] = {false, false, false, false}; // Tracks if each trough is in fill mode
unsigned long fillCheckTime[4] = {0, 0, 0, 0};   // Time to track periodic checks


// ======================[Valve State Tracking]====================================
// Structure for tracking state of each valve (flow, volume, dispensing)
struct ValveState
{
  bool isDispensing;                // Whether the valve is currently dispensing
  bool manualControl;               // Whether the valve is under manual control
  float targetVolume;               // Target volume for the valve
  float lastFlowValue;              // Last flow sensor reading for the valve
  unsigned long lastFlowCheckTime;  // Last time flow was checked
  unsigned long lastFlowChangeTime; // Last time flow value changed
};

ValveState valveStates[4]; // Array to track state for each of the 4 valves

// ======================[Flow Sensor Reset Variables]=============================
// Variables for tracking flow sensor reset progress for each valve
bool resetInProgress[4] = {false, false, false, false}; // Is the reset in progress for each valve?
unsigned long resetStartTime[4] = {0, 0, 0, 0};         // Start time of the reset for each valve


// ======================[Logging and Timeout Management]==========================
// Variables for managing system logging and timeout behavior
unsigned long previousLogTime = 0;             // Time of the last log
unsigned long logInterval = 250;               // Default log interval (milliseconds)
const unsigned long flowTimeoutPeriod = 10000; // Flow timeout period (milliseconds)

// ======================[Valve and Sensor Objects]===============================

// Reagent Valves
SolenoidValve reagentValve1(REAGENT_VALVE_1_PIN);
SolenoidValve reagentValve2(REAGENT_VALVE_2_PIN);
SolenoidValve reagentValve3(REAGENT_VALVE_3_PIN);
SolenoidValve reagentValve4(REAGENT_VALVE_4_PIN);

// Waste Valves
SolenoidValve wasteValve1(WASTE_VALVE_1_PIN);
SolenoidValve wasteValve2(WASTE_VALVE_2_PIN);

// Media Valves
SolenoidValve mediaValve1(MEDIA_VALVE_1_PIN);
SolenoidValve mediaValve2(MEDIA_VALVE_2_PIN);
SolenoidValve mediaValve3(MEDIA_VALVE_3_PIN);
SolenoidValve mediaValve4(MEDIA_VALVE_4_PIN);

// Bubble Sensors
BubbleSensor reagent1BubbleSensor(REAGENT_1_BUBBLE_SENSOR_PIN);
BubbleSensor reagent2BubbleSensor(REAGENT_2_BUBBLE_SENSOR_PIN);
BubbleSensor reagent3BubbleSensor(REAGENT_3_BUBBLE_SENSOR_PIN);
BubbleSensor reagent4BubbleSensor(REAGENT_4_BUBBLE_SENSOR_PIN);

// Overflow Sensors
OverflowSensor overflowSensorTrough1(OVERFLOW_SENSOR_TROUGH_1_PIN);
OverflowSensor overflowSensorTrough2(OVERFLOW_SENSOR_TROUGH_2_PIN);
OverflowSensor overflowSensorTrough3(OVERFLOW_SENSOR_TROUGH_3_PIN);
OverflowSensor overflowSensorTrough4(OVERFLOW_SENSOR_TROUGH_4_PIN);

// Pressure Valve and Sensor
ProportionalValve pressureValve(PRESSURE_VALVE_CONTROL_PIN, PRESSURE_VALVE_FEEDBACK_PIN);
PressureSensor pressureSensor(PRESSURE_SENSOR_PIN, MIN_PRESSURE, MAX_PRESSURE);

// Flow Sensors (Modbus Connected)
FDXSensor flowSensorReagent1(modbus, FLOW_SENSOR_REAGENT_1_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_1_RESET_PIN);
FDXSensor flowSensorReagent2(modbus, FLOW_SENSOR_REAGENT_2_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_2_RESET_PIN);
FDXSensor flowSensorReagent3(modbus, FLOW_SENSOR_REAGENT_3_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_3_RESET_PIN);
FDXSensor flowSensorReagent4(modbus, FLOW_SENSOR_REAGENT_4_MODBUS_REGISTER, FLOW_SENSOR_REAGENT_4_RESET_PIN);

// ======================[Command Function Prototypes]============================
// Functions to handle commands for setting valve states, pressure, logging, etc.
void cmd_set_reagent_valve(char *args, Stream *response);
void cmd_set_media_valve(char *args, Stream *response);
void cmd_set_waste_valve(char *args, Stream *response);
void cmd_set_pressure_valve(char *args, Stream *response);
void cmd_reset_flow_sensor(char *args, Stream *response);
void cmd_set_log_frequency(char *args, Stream *response);
void cmd_dispense_reagent(char *args, Stream *response);
void cmd_stop_dispense(char *args, Stream *response);
void cmd_fill_reagent(char *args, Stream *response);
void cmd_stop_fill_reagent(char *args, Stream *response);

// ======================[Overflow and Timeout Handling]===========================
// Functions to handle overflow and timeout conditions
void handleOverflowCondition(int triggeredValveNumber);
void handleTimeoutCondition(int triggeredValveNumber);

// ======================[System State Logging and Utility Functions]==============
// Functions for logging system state and printing sensor values
void log();
void printFlowSensorValues(); // Helper function to print flow sensor values

// ======================[Helper Functions for Flow, Serial, and Valve Handling]===
// Helper functions for flow sensor reset, serial command handling, etc.
void initializeValves();
void initializeSensors();
void handleFlowSensorReset(unsigned long currentTime);
void handleSerialCommands();
void monitorOverflowSensors(unsigned long currentTime);
void monitorFlowSensors(unsigned long currentTime);
void monitorFillSensors(unsigned long currentTime);
float getFlowSensorValue(int valveIndex);
void checkModbusConnection(unsigned long currentTime);
void logSystemState(unsigned long currentTime);

// ======================[Valve Control Functions]=================================
// Helper functions to open and close reagent and media valves
void openValves(int valveNumber, Stream *response);
void closeValves(int valveNumber, Stream *response);

// Commander Object for Handling API
Commander commander;

// ======================[Commander API Configuration]============================

Commander::API_t API_tree[] = {
    apiElement("setRV", "Set reagent valve: setRV <1-4> <0/1>", cmd_set_reagent_valve),
    apiElement("setMV", "Set media valve: setMV <1-4> <0/1>", cmd_set_media_valve),
    apiElement("setWV", "Set waste valve: setWV <1-2> <0/1>", cmd_set_waste_valve),
    apiElement("setPV", "Set pressure valve: setPV <percentage>", cmd_set_pressure_valve),
    apiElement("resetFS", "Reset flow sensor: resetFS <1-4>", cmd_reset_flow_sensor),
    apiElement("setLF", "Set log frequency: setLF <milliseconds>", cmd_set_log_frequency),
    apiElement("dispenseR", "Dispense reagent: dispenseR <1-4> [volume]", cmd_dispense_reagent),
    apiElement("stopD", "Stop dispensing reagent: stopD <1-4 or all>", cmd_stop_dispense),
    apiElement("fillR", "Fill reagent trough: fillR <1-4>", cmd_fill_reagent),
    apiElement("stopF", "Stop filling reagent trough: stopF <1-4 or all>", cmd_stop_fill_reagent)};

// ======================[Setup and Loop]==========================================

void setup()
{
  Serial.begin(115200); // Start serial communication

   // Initialize Ethernet and print the IP address (called only once)
    Ethernet.begin(mac, ip);
    Serial.print("IP Address: ");
    Serial.println(Ethernet.localIP());

    delay(1000);  // Allow some time for Ethernet initialization
    
    // Start the TCP server
    tcpServer.begin();
    Serial.println("TCP server initialized.");

    delay(500);  // Add a small delay to ensure the TCP server is ready

    // Initialize Modbus connection (without Ethernet.begin)
    modbus.checkConnection();
    Serial.println("Modbus connection initialized.");

    delay(500);  // Add a small delay after Modbus connection check


  // Initialize valve states
  for (int i = 0; i < 4; i++)
  {
    valveStates[i].isDispensing = false;
    valveStates[i].targetVolume = -1;
    valveStates[i].lastFlowValue = -1;
    valveStates[i].lastFlowCheckTime = 0;
    valveStates[i].lastFlowChangeTime = 0;
    fillMode[i] = false;
  }

  // Initialize each valve and sensor
  initializeValves();
  initializeSensors();
  pressureValve.setup();
  pressureValve.setPosition(0.0);
  pressureSensor.setup();

  // Attach and initialize the Commander API
  commander.attachTree(API_tree);
  commander.init();
}

void loop()
{
  unsigned long currentTime = millis();



  handleFlowSensorReset(currentTime);  // Manage flow sensor reset
  handleSerialCommands();              // Handle serial input
  handleTCPCommands();                 // Handle TCP client requests and commands
  tcpServer.handleClient();            // Handle incoming TCP client requests
  monitorOverflowSensors(currentTime); // Check for overflow sensors
  monitorFlowSensors(currentTime);     // Check flow sensor values
  monitorFillSensors(currentTime);     // Monitor filling process
  checkModbusConnection(currentTime);  // Check Modbus connection
  logSystemState(currentTime);         // Log the system state
}

void log()
{
  // Print initial log prefix
  printf("LOG,MB,%d,RV,%d%d%d%d,MV,%d%d%d%d,WV,%d%d,BS,%d%d%d%d,OV,%d%d%d%d",
         // modbusConnected (1 if connected, 0 if not)
         modbus.isConnected() ? 1 : 0,

         // Reagent Valve States: RV1-RV4 (0 for Closed, 1 for Open)
         reagentValve1.isValveOpen() ? 1 : 0,
         reagentValve2.isValveOpen() ? 1 : 0,
         reagentValve3.isValveOpen() ? 1 : 0,
         reagentValve4.isValveOpen() ? 1 : 0,

         // Media Valve States: MV1-MV4 (0 for Closed, 1 for Open)
         mediaValve1.isValveOpen() ? 1 : 0,
         mediaValve2.isValveOpen() ? 1 : 0,
         mediaValve3.isValveOpen() ? 1 : 0,
         mediaValve4.isValveOpen() ? 1 : 0,

         // Waste Valve States: WV1-WV2
         wasteValve1.isValveOpen() ? 1 : 0,
         wasteValve2.isValveOpen() ? 1 : 0,

         // Bubble Sensor States: BS1-BS4
         reagent1BubbleSensor.isLiquidDetected() ? 1 : 0,
         reagent2BubbleSensor.isLiquidDetected() ? 1 : 0,
         reagent3BubbleSensor.isLiquidDetected() ? 1 : 0,
         reagent4BubbleSensor.isLiquidDetected() ? 1 : 0,

         // Overflow Sensor States: OV1-OV4
         overflowSensorTrough1.isOverflowing() ? 1 : 0,
         overflowSensorTrough2.isOverflowing() ? 1 : 0,
         overflowSensorTrough3.isOverflowing() ? 1 : 0,
         overflowSensorTrough4.isOverflowing() ? 1 : 0);

  // Pressure Valve Feedback and Percentage
  float feedbackVoltage = pressureValve.getFeedback();
  float valvePercent = (feedbackVoltage / 7.0) * 100; // Assuming 7V max feedback voltage
  printf(",PV,%.2f,PV%%,%.1f", feedbackVoltage, valvePercent);

  // Pressure Sensor Reading
  printf(",PS1,%.2f psi", pressureSensor.readPressure());

  // Flow Rates: FS1-FS4 (use getScaledFlowValue() for each sensor)
  printFlowSensorValues();

  // Dispensing State for Valves 1-4
  printf(",DS,%d%d%d%d",
         valveStates[0].isDispensing ? 1 : 0,
         valveStates[1].isDispensing ? 1 : 0,
         valveStates[2].isDispensing ? 1 : 0,
         valveStates[3].isDispensing ? 1 : 0);

  // Target Volume for Valves 1-4
  printf(",TV,%.1f,%.1f,%.1f,%.1f",
         valveStates[0].targetVolume,
         valveStates[1].targetVolume,
         valveStates[2].targetVolume,
         valveStates[3].targetVolume);

  // Filling Mode for Troughs 1-4 (0 = off, 1 = on)
  printf(",FM,%d%d%d%d\n",
         fillMode[0] ? 1 : 0,
         fillMode[1] ? 1 : 0,
         fillMode[2] ? 1 : 0,
         fillMode[3] ? 1 : 0);
}

// Helper function to print flow sensor values
void printFlowSensorValues()
{
  float flowValue = flowSensorReagent1.getScaledFlowValue();
  printf(",FS1,%s", (flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");

  flowValue = flowSensorReagent2.getScaledFlowValue();
  printf(",FS2,%s", (flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");

  flowValue = flowSensorReagent3.getScaledFlowValue();
  printf(",FS3,%s", (flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");

  flowValue = flowSensorReagent4.getScaledFlowValue();
  printf(",FS4,%s", (flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");
}

// ======================[Helper Functions]========================================

// Helper function to initialize valves
void initializeValves()
{
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
}

// Helper function to initialize sensors
void initializeSensors()
{
  reagent1BubbleSensor.setup();
  reagent2BubbleSensor.setup();
  reagent3BubbleSensor.setup();
  reagent4BubbleSensor.setup();
  overflowSensorTrough1.setup();
  overflowSensorTrough2.setup();
  overflowSensorTrough3.setup();
  overflowSensorTrough4.setup();
}

// Function to handle flow sensor reset
void handleFlowSensorReset(unsigned long currentTime)
{
  const unsigned long resetDuration = 5;    // Duration of flow sensor reset (milliseconds)
  // Array of flow sensors
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

  for (int i = 0; i < 4; i++)
  {
    if (resetInProgress[i] && millis() - resetStartTime[i] >= resetDuration)
    {
      resetInProgress[i] = false; // Reset is complete
      printf("Flow sensor reset completed for valve %d\n", i + 1);

      // Reset the corresponding flow sensor using the array
      flowSensors[i]->handleReset();
    }
  }
}

// Function to handle serial input for commands
void handleSerialCommands()
{
  static char commandBuffer[16]; // Buffer to store incoming command
  static uint8_t commandIndex = 0;

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
}

// Handle TCP commands received from the TCP server
void handleTCPCommands() {
    String tcpCommand = tcpServer.handleClient();  // Get the command from the TCP server

    if (tcpCommand.length() > 0) {
        // Convert the String to a char array for compatibility with commander.execute()
        char commandBuffer[16];
        tcpCommand.toCharArray(commandBuffer, sizeof(commandBuffer));

        // Execute the command using Commander and send the result back to the client
        commander.execute(commandBuffer, &tcpServer.getClient());
    }
}


// Function to monitor overflow sensors every 50ms
void monitorOverflowSensors(unsigned long currentTime)
{
  static unsigned long previousOverflowCheckTime = 0;

  // Array of overflow sensors
  OverflowSensor *overflowSensors[] = {&overflowSensorTrough1, &overflowSensorTrough2, &overflowSensorTrough3, &overflowSensorTrough4};

  if (currentTime - previousOverflowCheckTime >= 25)
  { // Check every 25ms
    previousOverflowCheckTime = currentTime;

    // Loop through the overflow sensors and check if any have triggered overflow
    for (int i = 0; i < 4; i++)
    {
      // Skip overflow monitoring if the trough is in fill mode
      if (fillMode[i])
      {
        continue;
      }

      // Proceed with overflow handling for troughs not in fill mode
      if (overflowSensors[i]->loop() == 1)
      {
        handleOverflowCondition(i + 1); // Handle overflow as usual for troughs not in fill mode
      }
    }
  }
}

// Function to monitor flow sensors for each valve
void monitorFlowSensors(unsigned long currentTime)
{
  const unsigned long resetDuration = 5;

  for (int i = 0; i < 4; i++)
  {
    if (valveStates[i].isDispensing && currentTime - valveStates[i].lastFlowCheckTime >= 25)
    {
      valveStates[i].lastFlowCheckTime = currentTime;

      // Skip timeout check if the valve is manually controlled
      if (valveStates[i].manualControl)
      {
        continue;
      }

      // Skip flow check for a brief period after reset to allow valid readings
      if (resetInProgress[i] && currentTime - resetStartTime[i] < resetDuration + 50)
      {
        // Give the sensor some additional time after reset (e.g., 50ms buffer)
        continue;
      }

      float currentFlowValue = getFlowSensorValue(i);

      if (currentFlowValue < 0)
      {
        printf("Invalid flow reading for valve %d\n", i + 1);
        handleTimeoutCondition(i + 1);
        continue;
      }

      if (valveStates[i].targetVolume > 0 && currentFlowValue >= valveStates[i].targetVolume)
      {
        printf("Target volume reached for valve %d\n", i + 1);
        handleTimeoutCondition(i + 1); // Close valves and stop dispensing
        continue;
      }

      if (currentFlowValue == valveStates[i].lastFlowValue)
      {
        if (currentTime - valveStates[i].lastFlowChangeTime >= flowTimeoutPeriod)
        {
          printf("Timeout occurred for valve %d\n", i + 1);
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
}

// Function to get the flow sensor value for a specific valve
float getFlowSensorValue(int valveIndex)
{
  // Array of flow sensors
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

  // Validate the valve index and return the flow sensor value
  if (valveIndex >= 0 && valveIndex < 4)
  {
    return flowSensors[valveIndex]->getScaledFlowValue();
  }
  else
  {
    return -1.0; // Invalid index
  }
}

// Function to check Modbus connection status
void checkModbusConnection(unsigned long currentTime) {
    static unsigned long previousModbusCheckTime = 0;
    if (currentTime - previousModbusCheckTime >= 500) {
        modbus.checkConnection();
        previousModbusCheckTime = currentTime;
    }
}

// Function to log system state at regular intervals
void logSystemState(unsigned long currentTime)
{
  if (currentTime - previousLogTime >= logInterval)
  {
    log();
    previousLogTime = currentTime;
  }
}

// ======================[Command Functions for Valve Control]====================

// Command to set reagent valves (setRV <valve number> <0/1> or setRV all <0/1>)
void cmd_set_reagent_valve(char *args, Stream *response)
{
  if (sscanf(args, "%s %d", valveArg, &state) == 2)
  {
    SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
    SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};

    if (strcmp(valveArg, "all") == 0)
    {
      for (int i = 0; i < 4; i++)
      {
        fillMode[i] = false; // Disable fill mode
        reagentValves[i]->closeValve();
        mediaValves[i]->closeValve();
        valveStates[i].manualControl = (state == 1); // Set manualControl flag
      }

      if (state == 1)
      {
        for (int i = 0; i < 4; i++)
        {
          reagentValves[i]->openValve();
        }
        response->println("All reagent valves opened.");
      }
      else
      {
        for (int i = 0; i < 4; i++)
        {
          reagentValves[i]->closeValve();
        }
        response->println("All reagent valves closed.");
      }
    }
    else
    {
      valveNumber = atoi(valveArg);
      if (valveNumber >= 1 && valveNumber <= 4 && (state == 0 || state == 1))
      {
        fillMode[valveNumber - 1] = false;
        reagentValves[valveNumber - 1]->closeValve();
        mediaValves[valveNumber - 1]->closeValve();

        SolenoidValve *valve = reagentValves[valveNumber - 1];
        if (state == 1)
        {
          valve->openValve();
          valveStates[valveNumber - 1].isDispensing = true;
          valveStates[valveNumber - 1].manualControl = true; // Set manualControl flag
          response->println("Reagent valve opened.");
        }
        else
        {
          valve->closeValve();
          valveStates[valveNumber - 1].isDispensing = false;
          valveStates[valveNumber - 1].manualControl = false; // Reset manualControl flag
          response->println("Reagent valve closed.");
        }
      }
      else
      {
        response->println("Invalid reagent valve command. Use 1-4 or 'all'.");
      }
    }
  }
  else
  {
    response->println("Invalid reagent valve command.");
  }
}

// Command to set media valves (setMV <valve number> <0/1> or setMV all <0/1>)
void cmd_set_media_valve(char *args, Stream *response)
{
  if (sscanf(args, "%s %d", valveArg, &state) == 2)
  {
    SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};
    SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};

    if (strcmp(valveArg, "all") == 0)
    {
      // Disable fill mode for all troughs and close valves
      for (int i = 0; i < 4; i++)
      {
        fillMode[i] = false;                         // Disable fill mode for all troughs
        reagentValves[i]->closeValve();              // Close reagent valves
        mediaValves[i]->closeValve();                // Close media valves
        valveStates[i].manualControl = (state == 1); // Set manualControl flag for all valves
      }

      // Open or close all media valves based on the state
      if (state == 1)
      {
        for (int i = 0; i < 4; i++)
        {
          mediaValves[i]->openValve();
        }
        response->println("All media valves opened.");
      }
      else
      {
        for (int i = 0; i < 4; i++)
        {
          mediaValves[i]->closeValve();
        }
        response->println("All media valves closed.");
      }
    }
    else
    {
      valveNumber = atoi(valveArg);
      if (valveNumber >= 1 && valveNumber <= 4 && (state == 0 || state == 1))
      {
        // Disable fill mode for the specific trough
        fillMode[valveNumber - 1] = false;

        // Close reagent and media valves for the specific trough
        reagentValves[valveNumber - 1]->closeValve();
        mediaValves[valveNumber - 1]->closeValve();

        // Open or close the specific media valve
        SolenoidValve *valve = mediaValves[valveNumber - 1];
        if (valve != nullptr)
        {
          if (state == 1)
          {
            valve->openValve();
            valveStates[valveNumber - 1].manualControl = true; // Set manualControl flag
            response->println("Media valve opened.");
          }
          else
          {
            valve->closeValve();
            valveStates[valveNumber - 1].manualControl = false; // Reset manualControl flag
            response->println("Media valve closed.");
          }
        }
      }
      else
      {
        response->println("Invalid media valve command. Use 1-4 or 'all'.");
      }
    }
  }
  else
  {
    response->println("Invalid media valve command.");
  }
}

// Command to set waste valves (setWV <valve number> <0/1> or setWV all <0/1>)
void cmd_set_waste_valve(char *args, Stream *response)
{
  if (sscanf(args, "%s %d", valveArg, &state) == 2)
  {
    if (strcmp(valveArg, "all") == 0)
    {
      if (state == 0 || state == 1)
      {
        if (state == 1)
        {
          wasteValve1.openValve();
          wasteValve2.openValve();
          response->println("All waste valves opened.");
        }
        else
        {
          wasteValve1.closeValve();
          wasteValve2.closeValve();
          response->println("All waste valves closed.");
        }
      }
      else
      {
        response->println("Invalid state. Use 0 or 1.");
      }
    }
    else
    {
      valveNumber = atoi(valveArg);
      if (valveNumber >= 1 && valveNumber <= 2 && (state == 0 || state == 1))
      {
        SolenoidValve *valve = nullptr;
        if (valveNumber == 1)
          valve = &wasteValve1;
        else if (valveNumber == 2)
          valve = &wasteValve2;

        if (valve != nullptr)
        {
          if (state == 1)
          {
            valve->openValve();
            response->println("Waste valve opened.");
          }
          else
          {
            valve->closeValve();
            response->println("Waste valve closed.");
          }
        }
      }
      else
      {
        response->println("Invalid waste valve command. Use 1-2 or 'all'.");
      }
    }
  }
  else
  {
    response->println("Invalid waste valve command.");
  }
}

// Command to set pressure valve (setPV <percentage>)
void cmd_set_pressure_valve(char *args, Stream *response)
{
  String command = String(args);
  String valueStr = command;
  float value = valueStr.toFloat(); // Convert the string to a float
  int percentage = int(value);      // Convert the float to an integer percentage

  // Validate the percentage range (0-100)
  if (percentage >= 0 && percentage <= 100)
  {
    pressureValve.setPosition(percentage); // Set the valve to the percentage position
    response->print("Pressure valve set to ");
    response->print(percentage);
    response->println("%.");
  }
  else
  {
    response->println("Invalid value for pressure valve. Use a percentage between 0 and 100.");
  }
}

// ======================[Command Functions for Flow Sensors]=====================

// Command to reset flow sensor (resetFS <sensor number> or resetFS all)
void cmd_reset_flow_sensor(char *args, Stream *response)
{
  // Check if Modbus is connected
     if (!modbus.isConnected()) {
        response->println("Modbus not connected. Cannot process dispense command.");
        return;
    }
  
  // Array of flow sensors
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

  if (strcmp(args, "all") == 0)
  {
    // Reset all flow sensors using a loop
    for (int i = 0; i < 4; i++)
    {
      flowSensors[i]->startResetFlow();
    }
    response->println("All flow sensors reset initiated.");
  }
  else
  {
    int sensorNumber;
    if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= 4)
    {
      // Reset the specific flow sensor
      flowSensors[sensorNumber - 1]->startResetFlow();
      response->print("Flow sensor ");
      response->print(sensorNumber);
      response->println(" reset initiated.");
    }
    else
    {
      response->println("Invalid flow sensor number. Use 1-4 or 'all'.");
    }
  }
}

// Command to set the logging frequency (setLF <milliseconds>)
void cmd_set_log_frequency(char *args, Stream *response)
{
  int newLogInterval;
  if (sscanf(args, "%d", &newLogInterval) == 1 && newLogInterval > 0)
  {
    logInterval = newLogInterval;
    response->print("Log frequency set to ");
    response->print(logInterval);
    response->println(" milliseconds.");
  }
  else
  {
    response->println("Invalid log frequency. Please provide a valid number in milliseconds.");
  }
}

// ======================[Helper function for checking system pressure]===============
bool checkAndSetPressure(Stream *response, float thresholdPressure, unsigned long timeout)
{
  unsigned long pressureCheckStartTime = millis();
  float currentPressure = pressureSensor.readPressure();

  // Check if the system is already pressurized
  if (currentPressure >= thresholdPressure)
  {
    response->println("System already pressurized.");
    return true;
  }

  // Set the pressure valve to 100%
  pressureValve.setPosition(100);
  response->println("Pressure valve set to 100%.");

  // Wait for pressure to stabilize within the timeout
  while (millis() - pressureCheckStartTime < timeout)
  {
    currentPressure = pressureSensor.readPressure();
    if (currentPressure >= thresholdPressure)
    {
      return true; // Pressure is sufficient
    }
    delay(100); // Add delay to prevent tight looping
  }

  // If pressure is not reached, log and return false
  response->print("Error: Pressure threshold not reached. Current pressure: ");
  response->print(currentPressure);
  response->println(" psi. Operation aborted.");
  return false;
}

// ======================[Command Functions for Dispensing Control]===============

// Command to dispense reagent (dispenseR <valve number> [volume])
void cmd_dispense_reagent(char *args, Stream *response)
{
  // Check if Modbus is connected
  if (!modbus.isConnected())
  {
    response->println("Modbus not connected. Cannot process dispense command.");
    return;
  }

  const float PRESSURE_THRESHOLD_PSI = 20.0;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  const float MIN_VOLUME = 1.0;   
  const float MAX_VOLUME = 200.0; 

  // Check and set the pressure using the helper function
  if (!checkAndSetPressure(response, PRESSURE_THRESHOLD_PSI, PRESSURE_TIMEOUT_MS))
  {
    return; // If pressure check fails, abort the dispense operation
  }

  // Enable dispensing only after pressure check succeeds
  String inputString = String(args);
  inputString.trim();
  int valveNumber = -1;
  float requestedVolume = -1;

  int spaceIndex = inputString.indexOf(' ');

  // Extract and parse the valve number and volume
  if (spaceIndex != -1)
  {
    String intPart = inputString.substring(0, spaceIndex);
    String floatPart = inputString.substring(spaceIndex + 1);
    valveNumber = intPart.toInt();
    requestedVolume = floatPart.toFloat();
  }
  else
  {
    valveNumber = inputString.toInt();
  }

  response->print("Valve number: ");
  response->println(valveNumber);
  response->print("Requested volume: ");
  response->println(requestedVolume);

  if (valveNumber >= 1 && valveNumber <= 4)
  {

    // Disable fill mode for the specific trough
    fillMode[valveNumber - 1] = false;

    // Close reagent and media valves using a loop
    SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
    SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};

    reagentValves[valveNumber - 1]->closeValve();
    mediaValves[valveNumber - 1]->closeValve();

    // Select the correct overflow sensor
    OverflowSensor *overflowSensors[] = {&overflowSensorTrough1, &overflowSensorTrough2, &overflowSensorTrough3, &overflowSensorTrough4};
    OverflowSensor *overflowSensor = overflowSensors[valveNumber - 1];

    if (overflowSensor != nullptr && overflowSensor->isOverflowing())
    {
      response->print("Cannot dispense: Overflow detected for valve ");
      response->println(valveNumber);
      return; // Stop if overflow is detected
    }

    // Validate and set volume
    if (requestedVolume > 0)
    {
      if (requestedVolume < MIN_VOLUME)
      {
        response->print("Requested volume too low. Minimum volume is ");
        response->print(MIN_VOLUME);
        response->println(" mL.");
        return;
      }
      else if (requestedVolume > MAX_VOLUME)
      {
        response->print("Requested volume too high. Maximum volume is ");
        response->print(MAX_VOLUME);
        response->println(" mL.");
        return;
      }
      valveStates[valveNumber - 1].targetVolume = requestedVolume;
      response->print("Target volume set for valve ");
      response->print(valveNumber);
      response->print(": ");
      response->print(valveStates[valveNumber - 1].targetVolume);
      response->println(" mL");
    }
    else
    {
      valveStates[valveNumber - 1].targetVolume = -1; // Continuous dispense
      response->println("Continuous dispensing (no target volume specified).");
    }

    // Reset flow sensor for the specific valve using a loop
    FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

    resetInProgress[valveNumber - 1] = true;
    resetStartTime[valveNumber - 1] = millis();
    flowSensors[valveNumber - 1]->startResetFlow();

    response->print("Flow sensor reset initiated for reagent ");
    response->println(valveNumber);

    // Open the reagent and media valves for the specific valve
    openValves(valveNumber, response);

    // Track the dispensing state for the valve
    valveStates[valveNumber - 1].isDispensing = true;
    isDispensing[valveNumber - 1] = true; // Set isDispensing flag
  }
  else
  {
    response->println("Invalid valve number. Use 1-4.");
  }
}

// Command to stop dispensing (stopD <valve number> or stopD all)
void cmd_stop_dispense(char *args, Stream *response)
{
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

  if (strcmp(args, "all") == 0)
  {
    // Stop all reagent and media valves and disable fill mode
    for (int i = 0; i < 4; i++)
    {
      fillMode[i] = false;          // Disable fill mode for all troughs
      closeValves(i + 1, response); // Close the reagent and media valves
    }

    // Reset all flow sensors using a loop
    for (int i = 0; i < 4; i++)
    {
      flowSensors[i]->startResetFlow();
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
      fillMode[valveNumber - 1] = false;  // Disable fill mode for the specific trough
      closeValves(valveNumber, response); // Close reagent and media valves

      // Reset the flow sensor for the specific valve
      flowSensors[valveNumber - 1]->startResetFlow();

      isDispensing[valveNumber - 1] = false;
      dispensingValveNumber[valveNumber - 1] = -1;
    }
    else
    {
      response->println("Invalid valve number. Use 1-4 or 'all'.");
    }
  }
}

// Helper function to handle overflow conditions
void handleOverflowCondition(int triggeredValveNumber)
{
  if (isDispensing[triggeredValveNumber - 1])
  {
    closeValves(triggeredValveNumber, &Serial);
    isDispensing[triggeredValveNumber - 1] = false; // Stop the valve from dispensing
    dispensingValveNumber[triggeredValveNumber - 1] = -1;
    Serial.print("Overflow detected: Valves closed for valve ");
    Serial.println(triggeredValveNumber);

    // Reset the flow sensor for the specific valve using an array
    FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};
    flowSensors[triggeredValveNumber - 1]->startResetFlow();

    // Reset the timeout mechanism for this valve
    valveStates[triggeredValveNumber - 1].lastFlowCheckTime = 0;  // Reset the flow check timer
    valveStates[triggeredValveNumber - 1].lastFlowChangeTime = 0; // Reset the last flow change time
  }
}

// Helper function to close both reagent and media valves for a given valve number
void closeValves(int valveNumber, Stream *response)
{
  // Arrays for reagent and media valves
  SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
  SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};

  if (valveNumber >= 1 && valveNumber <= 4)
  {
    // Close the reagent valve
    reagentValves[valveNumber - 1]->closeValve();
    response->print("Reagent valve ");
    response->print(valveNumber);
    response->println(" closed.");

    // Close the media valve
    mediaValves[valveNumber - 1]->closeValve();
    response->print("Media valve ");
    response->print(valveNumber);
    response->println(" closed.");
  }
  else
  {
    response->println("Invalid valve number.");
  }
}

// Helper function to open both reagent and media valves for a given valve number
void openValves(int valveNumber, Stream *response)
{
  // Arrays for reagent and media valves
  SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
  SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};

  if (valveNumber >= 1 && valveNumber <= 4)
  {
    // Open the reagent valve
    reagentValves[valveNumber - 1]->openValve();
    response->print("Reagent valve ");
    response->print(valveNumber);
    response->println(" opened.");

    // Open the media valve
    mediaValves[valveNumber - 1]->openValve();
    response->print("Media valve ");
    response->print(valveNumber);
    response->println(" opened.");
  }
  else
  {
    response->println("Invalid valve number.");
  }
}

// Helper function to handle timeout conditions
void handleTimeoutCondition(int triggeredValveNumber)
{
  closeValves(triggeredValveNumber, &Serial);
  Serial.print("Timeout occurred: Valves closed for valve ");
  Serial.println(triggeredValveNumber);

  // Reset the flow sensor for the specific valve using an array
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};
  flowSensors[triggeredValveNumber - 1]->startResetFlow();

  // Reset the valve state
  valveStates[triggeredValveNumber - 1].isDispensing = false;
  valveStates[triggeredValveNumber - 1].targetVolume = -1;
}

// monitorFillSensors function to monitor overflow sensors and fill the troughs
void monitorFillSensors(unsigned long currentTime)
{
  const float MAX_FILL_VOLUME_ML = 150.0;           // Maximum fill volume in mL
  const unsigned long MAX_FILL_TIME_MS = 60000;    // Maximum fill time in milliseconds (1.5 minutes)

  // Arrays for overflow sensors, reagent valves, media valves, flow sensors
  OverflowSensor *overflowSensors[] = {&overflowSensorTrough1, &overflowSensorTrough2, &overflowSensorTrough3, &overflowSensorTrough4};
  SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
  SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

  static unsigned long fillStartTime[4] = {0, 0, 0, 0};   // Track fill start time for each valve
  static float previousFlowValue[4] = {0, 0, 0, 0};       // Track the last known flow value for each valve

  // 1. Track valve states to know when dispensing starts and stops (outside interval loop)
  for (int i = 0; i < 4; i++)
  {
    bool isReagentValveOpen = reagentValves[i]->isValveOpen();
    bool isMediaValveOpen = mediaValves[i]->isValveOpen();

    // If both valves are open, dispensing has started
    if (isReagentValveOpen && isMediaValveOpen)
    {
      if (!valveStates[i].isDispensing)
      {
        valveStates[i].isDispensing = true;  // Start dispensing
        if (fillStartTime[i] == 0)           // Only set start time if it's not already set
        {
          fillStartTime[i] = currentTime;    // Set the start time when filling starts
          previousFlowValue[i] = 0;          // Reset flow tracking
          printf("Valve %d opened, starting dispensing.\n", i + 1);
        }
      }
    }
    // If both valves are closed, dispensing has stopped
    else if (!isReagentValveOpen && !isMediaValveOpen)
    {
      if (valveStates[i].isDispensing)
      {
        valveStates[i].isDispensing = false;      // Stop dispensing
        printf("Valve %d closed, stopping dispensing.\n", i + 1);

        // Reset the flow sensor when dispensing stops
        resetInProgress[i] = true;                // Mark reset as in progress
        resetStartTime[i] = millis();             // Capture reset start time
        flowSensors[i]->startResetFlow();         // Reset flow sensor
        printf("Flow sensor reset initiated for valve %d.\n", i + 1);
        delay(100);                               // Add a small delay between each valve

        // Reset tracking for the next fill
        fillStartTime[i] = 0;  // Reset fill start time for the next cycle
        previousFlowValue[i] = 0;  // Reset flow tracking
      }
    }
  }

  // 2. Implement the timeout and volume checks outside the interval control loop
  for (int i = 0; i < 4; i++)
  {
    if (fillMode[i] && valveStates[i].isDispensing)  // Check only if fill mode is active and valve is dispensing
    {
      // Access the correct flow sensor
      FDXSensor *flowSensor = flowSensors[i];
      float currentFlowValue = flowSensor->getScaledFlowValue();

      // Calculate the new volume added since the last check
      float addedVolume = currentFlowValue - previousFlowValue[i];
      if (addedVolume < 0) addedVolume = 0;  // Ensure no negative flow values
      previousFlowValue[i] = currentFlowValue;  // Update the previous flow value

      // Check if fill timeout has occurred based on volume or time
      bool volumeExceeded = currentFlowValue >= MAX_FILL_VOLUME_ML;
      bool timeExceeded = currentTime - fillStartTime[i] >= MAX_FILL_TIME_MS;

      if (volumeExceeded || timeExceeded)
      {
        // Disable fill mode for this valve
        fillMode[i] = false;
        reagentValves[i]->closeValve();
        mediaValves[i]->closeValve();

        if (volumeExceeded)
        {
          printf("Fill timeout for trough %d: maximum volume (%.2f mL) reached.\n", i + 1, currentFlowValue);
        }
        else if (timeExceeded)
        {
          printf("Fill timeout for trough %d: maximum time (%.2f seconds) reached.\n", i + 1, MAX_FILL_TIME_MS / 1000.0);
        }

        // Reset tracking for the next fill
        fillStartTime[i] = 0;      // Reset start time when fill completes
        previousFlowValue[i] = 0;  // Reset flow tracking
        continue; // Skip further checks for this valve
      }
    }
  }

  // 3. Interval-based check for overflow and managing valve states for the next fill cycle
  const unsigned long SENSOR_CHECK_INTERVAL = 500; // Check interval for overflow
  for (int i = 0; i < 4; i++)
  {
    if (fillMode[i] && currentTime - fillCheckTime[i] >= SENSOR_CHECK_INTERVAL)
    {
      fillCheckTime[i] = currentTime;             // Reset the check time

      // Access the correct overflow sensor, reagent valve, and media valve
      OverflowSensor *overflowSensor = overflowSensors[i];
      SolenoidValve *reagentValve = reagentValves[i];
      SolenoidValve *mediaValve = mediaValves[i];

      bool isOverflowing = overflowSensor->isOverflowing();

      // If overflow is detected, close the valves
      if (isOverflowing)
      {
        if (reagentValve->isValveOpen() && mediaValve->isValveOpen())
        {
          reagentValve->closeValve();
          mediaValve->closeValve();
          printf("Overflow detected for trough %d, closing valves.\n", i + 1);
        }
      }
      // If overflow is not detected and valves are closed, reopen them and prepare for the next fill cycle
      else if (!reagentValve->isValveOpen())
      {
        reagentValve->openValve();
        mediaValve->openValve();
        printf("Trough %d not overflowing, opening valves to fill.\n", i + 1);
      }
    }
  }
}








// Command to fill the reagent (fillR <valve number> or fillR all)
void cmd_fill_reagent(char *args, Stream *response)
{
  // Check if Modbus is connected
  if (!modbus.isConnected()) {
        response->println("Modbus not connected. Cannot process fill command.");
        return;
    }
  
  const float PRESSURE_THRESHOLD_PSI = 20.0;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  // Check and set the pressure using the helper function
  if (!checkAndSetPressure(response, PRESSURE_THRESHOLD_PSI, PRESSURE_TIMEOUT_MS))
  {
    return; // If pressure check fails, abort the dispense operation
  }

  String inputString = String(args);
  inputString.trim();

  // Arrays for reagent, media valves, and flow sensors
  SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
  SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};
  FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

  // Handle "fillR all" case
  if (inputString.equalsIgnoreCase("all"))
  {
    for (int i = 0; i < 4; i++)
    {

      // Close and then open the valves to start a fresh fill
      reagentValves[i]->closeValve();
      mediaValves[i]->closeValve();

      reagentValves[i]->openValve();
      mediaValves[i]->openValve();

      // Reset the flow sensor after the valves have been opened
      resetInProgress[i] = true;
      resetStartTime[i] = millis();
      flowSensors[i]->startResetFlow(); // Reset flow sensor
      // response->print("Flow sensor reset initiated for valve ");
      response->println(i + 1);
      delay(100); // Add a small delay between each valve

      fillMode[i] = true; // Enable fill mode for this valve
                          // response->print("Filling started for valve ");
                          // response->println(i + 1);
    }
    response->println("Filling started for all valves.");
  }
  else
  {
    int valveNumber = inputString.toInt();

    if (valveNumber >= 1 && valveNumber <= 4)
    {

      // Close and then open the valves to start a fresh fill
      reagentValves[valveNumber - 1]->closeValve();
      mediaValves[valveNumber - 1]->closeValve();

      reagentValves[valveNumber - 1]->openValve();
      mediaValves[valveNumber - 1]->openValve();

      // Reset the flow sensor after the valves have been opened
      resetInProgress[valveNumber - 1] = true;
      resetStartTime[valveNumber - 1] = millis();
      flowSensors[valveNumber - 1]->startResetFlow(); // Reset flow sensor
      // response->print("Flow sensor reset initiated for valve ");
      // response->println(valveNumber);
      delay(100); // Add a small delay between each valve

      fillMode[valveNumber - 1] = true; // Enable fill mode for this valve
                                        // response->print("Filling started for valve ");
                                        // response->println(valveNumber);
    }
    else
    {
      response->println("Invalid valve number. Use 1-4 or 'all'.");
    }
  }
}

// Command to stop filling the reagent (stopF <valve number> or stopF all)
void cmd_stop_fill_reagent(char *args, Stream *response)
{
  if (sscanf(args, "%s", valveArg) == 1)
  {
    if (strcmp(valveArg, "all") == 0)
    {
      // Disable fill mode for all troughs
      for (int i = 0; i < 4; i++)
      {
        fillMode[i] = false;
        closeValves(i + 1, response); // Close all valves
      }
      response->println("Fill mode disabled for all troughs.");
    }
    else
    {
      valveNumber = atoi(valveArg);
      if (valveNumber >= 1 && valveNumber <= 4)
      {
        fillMode[valveNumber - 1] = false;
        closeValves(valveNumber, response); // Close the specific valve
        response->print("Fill mode disabled for trough ");
        response->println(valveNumber);
      }
      else
      {
        response->println("Invalid trough number. Use 1-4 or 'all'.");
      }
    }
  }
  else
  {
    response->println("Invalid stop fill command.");
  }
}