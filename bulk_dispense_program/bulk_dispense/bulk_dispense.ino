// ======================[Includes and Library Dependencies]======================
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
#include "TCPServer.h"
#include "TCPStream.h"

// ======================[Pin Assignments and Constants]==========================

// Reagent Valves
#define REAGENT_VALVE_1_PIN CONTROLLINO_R0
#define REAGENT_VALVE_2_PIN CONTROLLINO_R1
#define REAGENT_VALVE_3_PIN CONTROLLINO_R2
#define REAGENT_VALVE_4_PIN CONTROLLINO_R3

// Modbus reset pin
#define MODBUS_RESET_PIN CONTROLLINO_R6

// Waste Valves
#define WASTE_VALVE_1_PIN CONTROLLINO_R4
#define WASTE_VALVE_2_PIN CONTROLLINO_R5
#define WASTE_VALVE_3_PIN CONTROLLINO_R8
#define WASTE_VALVE_4_PIN CONTROLLINO_R9

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
TCPServer tcpServer(ip, 8080);                                // Use port 8080 for the TCP server
ModbusConnection modbus(mac, ip, IPAddress(169, 254, 0, 10)); // Modbus server IP
TCPStream tcpStream;
// ======================[Global Command Variables]===============================
// struct to store command variables
struct CommandVariables
{
  char valveArg[10]; // Stores command arguments
  int valveNumber;   // Tracks which valve number is being used
};

// Global instance
CommandVariables commandVars = {"", -1};

// ======================[Valve State Tracking]====================================
// Structure for tracking state of each valve (flow, volume, dispensing)
struct ValveControl
{
  bool isDispensing;                // Whether the valve is currently dispensing
  bool manualControl;               // Whether the valve is under manual control
  bool isPriming;                   // Whether the valve is currently priming
  bool fillMode;                    // Whether the valve is in fill mode
  float targetVolume;               // Target volume for the valve
  float lastFlowValue;              // Last flow sensor reading for the valve
  unsigned long lastFlowCheckTime;  // Last time flow was checked
  unsigned long lastFlowChangeTime; // Last time flow value changed
  unsigned long fillCheckTime;      // Time to track periodic checks for filling
  int dispensingValveNumber;        // Tracks the current dispensing valve number (-1 if none)
};

ValveControl valveControls[4]; // Array to track state for each of the 4 valves

// ======================[Flow Sensor Reset Variables]=============================
// Structure for tracking flow sensor reset status
struct FlowSensorReset
{
  bool resetInProgress[4];         // Is the reset in progress for each valve?
  unsigned long resetStartTime[4]; // Start time of the reset for each valve
};

// Global instance
FlowSensorReset flowSensorReset = {{false, false, false, false}, {0, 0, 0, 0}};

// ======================[Logging and Timeout Management]==========================
struct LoggingManagement
{
  unsigned long previousLogTime; // Time of the last log
  unsigned long logInterval;     // Log interval (milliseconds)
};

// Global instance
LoggingManagement logging = {0, 250};

// ======================[Valve and Sensor Objects]===============================

// Reagent Valves
SolenoidValve reagentValve1(REAGENT_VALVE_1_PIN);
SolenoidValve reagentValve2(REAGENT_VALVE_2_PIN);
SolenoidValve reagentValve3(REAGENT_VALVE_3_PIN);
SolenoidValve reagentValve4(REAGENT_VALVE_4_PIN);

// Waste Valves
SolenoidValve wasteValve1(WASTE_VALVE_1_PIN);
SolenoidValve wasteValve2(WASTE_VALVE_2_PIN);
SolenoidValve wasteValve3(WASTE_VALVE_3_PIN);
SolenoidValve wasteValve4(WASTE_VALVE_4_PIN);

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

// ======================[Global Valve and Sensor Arrays]=========================
// Define the global arrays here, after the objects have been created
SolenoidValve *reagentValves[] = {&reagentValve1, &reagentValve2, &reagentValve3, &reagentValve4};
SolenoidValve *mediaValves[] = {&mediaValve1, &mediaValve2, &mediaValve3, &mediaValve4};
SolenoidValve *wasteValves[] = {&wasteValve1, &wasteValve2, &wasteValve3, &wasteValve4}; // For waste valves

BubbleSensor *bubbleSensors[] = {&reagent1BubbleSensor, &reagent2BubbleSensor, &reagent3BubbleSensor, &reagent4BubbleSensor};
OverflowSensor *overflowSensors[] = {&overflowSensorTrough1, &overflowSensorTrough2, &overflowSensorTrough3, &overflowSensorTrough4};
FDXSensor *flowSensors[] = {&flowSensorReagent1, &flowSensorReagent2, &flowSensorReagent3, &flowSensorReagent4};

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
void cmd_print_help(char *args, Stream *response);
void cmd_get_system_state(char *args, Stream *response);
void cmd_modbus_reset(char *args, Stream *response);
void cmd_device_info(char *args, Stream *response);
void cmd_prime_valves(char *args, Stream *response);
void cmd_drain_trough(char *args, Stream *response);
void cmd_stop_drain_trough(char *args, Stream *response);
void print_something(Stream *);

// ======================[Overflow and Timeout Handling]===========================
// Functions to handle overflow and timeout conditions
void handleOverflowCondition(int triggeredValveNumber, Stream *response);
void handleTimeoutCondition(int triggeredValveNumber, Stream *response);

// ======================[System State Logging and Utility Functions]==============
// Functions for logging system state and printing sensor values
void log();
void printFlowSensorValues(); // Helper function to print flow sensor values

// ======================[Helper Functions for Flow, Serial, and Valve Handling]===
// Helper functions for flow sensor reset, serial command handling, etc.
void initializeValves();
void initializeSensors();
void handleFlowSensorReset(unsigned long currentTime, Stream *response, EthernetClient client);
void handleSerialCommands();
void monitorOverflowSensors(unsigned long currentTime, Stream *response, EthernetClient client);
void monitorFlowSensors(unsigned long currentTime, Stream *response, EthernetClient client);
void monitorFillSensors(unsigned long currentTime, Stream *response, EthernetClient client);
float getFlowSensorValue(int valveIndex);
void checkModbusConnection(unsigned long currentTime);
void logSystemState(unsigned long currentTime);
void resetFlowSensor(int sensorIndex, FDXSensor *flowSensors[]);
void monitorPrimeSensors(unsigned long currentTime, Stream *response, EthernetClient client);

// ======================[Valve Control Functions]=================================
// Helper functions to open and close reagent and media valves
void openValves(int valveNumber, Stream *response);
void closeValves(int valveNumber, Stream *response);

// Commander Object for Handling API
Commander commander;

// ======================[Commander API Configuration]============================

Commander::API_t API_tree[] = {
    apiElement("setRV", "Set reagent valve: setRV <1-4> <0/1> (0 = close, 1 = open)", cmd_set_reagent_valve),
    apiElement("setMV", "Set media valve: setMV <1-4> <0/1> (0 = close, 1 = open)", cmd_set_media_valve),
    apiElement("setWV", "Set waste valve: setWV <1-2> <0/1> (0 = close, 1 = open)", cmd_set_waste_valve),
    apiElement("setPV", "Set pressure valve: setPV <percentage> (sets pressure valve to the percentage value)", cmd_set_pressure_valve),
    apiElement("resetFS", "Reset flow sensor: resetFS <1-4> (resets the flow sensor for the selected valve)", cmd_reset_flow_sensor),
    apiElement("setLF", "Set log frequency: setLF <milliseconds> (set log interval in milliseconds)", cmd_set_log_frequency),
    apiElement("dispenseR", "Dispense reagent: dispenseR <1-4> [volume] (volume in mL, continuous if omitted)", cmd_dispense_reagent),
    apiElement("stopD", "Stop dispensing reagent: stopD <1-4 or all> (stops dispensing for a valve or all)", cmd_stop_dispense),
    apiElement("fillR", "Fill reagent trough: fillR <1-4 or all> (initiates filling for a trough or all troughs)", cmd_fill_reagent),
    apiElement("stopF", "Stop filling reagent trough: stopF <1-4 or all> (stops filling for a trough or all)", cmd_stop_fill_reagent),
    apiElement("help", "Print available commands and usage: help", cmd_print_help),
    apiElement("state", "Get system state: state", cmd_get_system_state),
    apiElement("modbusReset", "Reset Modbus device: modbusReset", cmd_modbus_reset),
    apiElement("deviceInfo", "Print device information (IP, TCP, Modbus).", cmd_device_info),
    apiElement("prime", "Prime valves until bubble sensor detects liquid: prime <1-4> or prime all", cmd_prime_valves),
    apiElement("drain", "Drain reagent trough: drain <1-4> (drains the specified trough)", cmd_drain_trough),
    apiElement("prime", "Prime valves until bubble sensor detects liquid: prime <1-4> or prime all", cmd_prime_valves)};

// ======================[Setup and Loop]==========================================

void setup()
{
  Serial.begin(115200); // Start serial communication

  pinMode(MODBUS_RESET_PIN, OUTPUT);    // Set Modbus reset pin as output
  digitalWrite(MODBUS_RESET_PIN, HIGH); // Apply 24V to Modbus reset pin
  delay(5000);                          // Wait for Modbus to initialize

  // Initialize Ethernet and print the IP address (called only once)
  Ethernet.begin(mac, ip);
  Serial.print(F("Device Ethernet IP Address: "));
  Serial.println(Ethernet.localIP());

  delay(1000); // Allow some time for Ethernet initialization

  // Start the TCP server
  tcpServer.begin();
  Serial.println(F("TCP server initialized."));
  Serial.print(F("TCP/IP Address: "));
  Serial.println(ip);
  Serial.print(F("TCP/IP Port: "));
  Serial.println(8080);

  delay(500); // Add a small delay to ensure the TCP server is ready

  // Initialize Modbus connection (without Ethernet.begin)
  modbus.checkConnection();
  Serial.println(F("Modbus connection initialized."));
  Serial.print(F("Modbus Server Address: "));
  Serial.println(modbus.getServerAddress());
  Serial.print(F("Modbus Port: "));
  Serial.println(502); // Default Modbus TCP port

  delay(500); // Add a small delay after Modbus connection check

  // Initialize valve controls
  for (int i = 0; i < 4; i++)
  {
    valveControls[i].isDispensing = false;
    valveControls[i].manualControl = false;
    valveControls[i].isPriming = false;
    valveControls[i].targetVolume = -1;
    valveControls[i].lastFlowValue = -1;
    valveControls[i].lastFlowCheckTime = 0;
    valveControls[i].lastFlowChangeTime = 0;
    valveControls[i].fillMode = false;  // Moved from fillMode array to the struct
    valveControls[i].fillCheckTime = 0; // Initialize fillCheckTime
    valveControls[i].dispensingValveNumber = -1;
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

  tcpStream.begin(tcpServer.getClient());
}

void loop()
{
  unsigned long currentTime = millis();

  // Handle serial commands if available
  if (Serial)
  {
    handleSerialCommands();
    handleFlowSensorReset(currentTime, &Serial, tcpServer.getClient()); // Pass Serial stream
    monitorOverflowSensors(currentTime, &Serial, tcpServer.getClient());
    monitorFlowSensors(currentTime, &Serial, tcpServer.getClient());
    monitorFillSensors(currentTime, &Serial, tcpServer.getClient());
    monitorPrimeSensors(currentTime, &Serial, tcpServer.getClient());
  }

  // Handle TCP commands and updates
  handleTCPCommands();

  if (tcpServer.getClient() && tcpServer.getClient().connected())
  {
    // print_something(&tcpStream);
    handleFlowSensorReset(currentTime, &Serial, tcpServer.getClient()); // Pass TCP client stream
    monitorOverflowSensors(currentTime, &Serial, tcpServer.getClient());
    monitorFlowSensors(currentTime, &Serial, tcpServer.getClient());
    monitorFillSensors(currentTime, &Serial, tcpServer.getClient());
    monitorPrimeSensors(currentTime, &Serial, tcpServer.getClient());
    tcpServer.getClient().flush();
  }

  // Regular system updates
  checkModbusConnection(currentTime);
  logSystemState(currentTime); // Optionally log to Serial and/or TCP
}

void print_something(Stream *ser)
{
  ser->print("H");
}

void log()
{
  int systemState = 1; // Default state is Idle (1)
  bool isDispensing = false;
  bool inFillMode = false;

  // Check Modbus connection first
  if (!modbus.isConnected())
  {
    systemState = 4; // Modbus Disconnected (Error State)
  }
  else
  {
    // Check if any valve is dispensing
    for (int i = 0; i < 4; i++)
    {
      if (valveControls[i].isDispensing) // Use valveControls
      {
        isDispensing = true;
        break; // No need to check further, already found a dispensing valve
      }
    }

    // Check if any trough is in fill mode
    for (int i = 0; i < 4; i++)
    {
      if (valveControls[i].fillMode) // Use valveControls
      {
        inFillMode = true;
        break; // No need to check further, already found a trough in fill mode
      }
    }

    // Determine the system state based on precedence (fill mode takes priority)
    if (inFillMode)
    {
      systemState = 3; // Fill Mode
    }
    else if (isDispensing)
    {
      systemState = 2; // Dispensing
    }
  }

  // Print initial log prefix
  Serial.print(F("LOG,STATE,"));
  Serial.print(systemState);
  Serial.print(F(",RV,"));
  Serial.print(reagentValve1.isValveOpen() ? 1 : 0);
  Serial.print(reagentValve2.isValveOpen() ? 1 : 0);
  Serial.print(reagentValve3.isValveOpen() ? 1 : 0);
  Serial.print(reagentValve4.isValveOpen() ? 1 : 0);

  Serial.print(F(",MV,"));
  Serial.print(mediaValve1.isValveOpen() ? 1 : 0);
  Serial.print(mediaValve2.isValveOpen() ? 1 : 0);
  Serial.print(mediaValve3.isValveOpen() ? 1 : 0);
  Serial.print(mediaValve4.isValveOpen() ? 1 : 0);

  Serial.print(F(",WV,"));
  Serial.print(wasteValve1.isValveOpen() ? 1 : 0);
  Serial.print(wasteValve2.isValveOpen() ? 1 : 0);

  Serial.print(F(",BS,"));
  Serial.print(reagent1BubbleSensor.isLiquidDetected() ? 1 : 0);
  Serial.print(reagent2BubbleSensor.isLiquidDetected() ? 1 : 0);
  Serial.print(reagent3BubbleSensor.isLiquidDetected() ? 1 : 0);
  Serial.print(reagent4BubbleSensor.isLiquidDetected() ? 1 : 0);

  Serial.print(F(",OV,"));
  Serial.print(overflowSensorTrough1.isOverflowing() ? 1 : 0);
  Serial.print(overflowSensorTrough2.isOverflowing() ? 1 : 0);
  Serial.print(overflowSensorTrough3.isOverflowing() ? 1 : 0);
  Serial.print(overflowSensorTrough4.isOverflowing() ? 1 : 0);

  // Pressure Valve Feedback and Percentage
  float feedbackVoltage = pressureValve.getFeedback();
  float valvePercent = (feedbackVoltage / 7.0) * 100; // Assuming 7V max feedback voltage
  Serial.print(F(",PV,"));
  Serial.print(feedbackVoltage, 2); // 2 decimal places for voltage
  Serial.print(F(",PV%,"));
  Serial.print(valvePercent, 1); // 1 decimal place for percentage

  // Pressure Sensor Reading
  Serial.print(F(",PS1,"));
  Serial.print(pressureSensor.readPressure(), 2); // 2 decimal places for pressure

  // Flow Rates: FS1-FS4 (use getScaledFlowValue() for each sensor)
  printFlowSensorValues();

  // Dispensing State for Valves 1-4
  Serial.print(F(",DS,"));
  Serial.print(valveControls[0].isDispensing ? 1 : 0);
  Serial.print(valveControls[1].isDispensing ? 1 : 0);
  Serial.print(valveControls[2].isDispensing ? 1 : 0);
  Serial.print(valveControls[3].isDispensing ? 1 : 0);

  // Target Volume for Valves 1-4
  Serial.print(F(",TV,"));
  Serial.print(valveControls[0].targetVolume, 1);
  Serial.print(F(","));
  Serial.print(valveControls[1].targetVolume, 1);
  Serial.print(F(","));
  Serial.print(valveControls[2].targetVolume, 1);
  Serial.print(F(","));
  Serial.print(valveControls[3].targetVolume, 1);

  // Filling Mode for Troughs 1-4 (0 = off, 1 = on)
  Serial.print(F(",FM,"));
  Serial.print(valveControls[0].fillMode ? 1 : 0);
  Serial.print(valveControls[1].fillMode ? 1 : 0);
  Serial.print(valveControls[2].fillMode ? 1 : 0);
  Serial.print(valveControls[3].fillMode ? 1 : 0);
  Serial.println();
}

// Helper function to print flow sensor values
void printFlowSensorValues()
{
  float flowValue = flowSensorReagent1.getScaledFlowValue();
  Serial.print(F(",FS1,"));
  Serial.print((flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");

  flowValue = flowSensorReagent2.getScaledFlowValue();
  Serial.print(F(",FS2,"));
  Serial.print((flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");

  flowValue = flowSensorReagent3.getScaledFlowValue();
  Serial.print(F(",FS3,"));
  Serial.print((flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");

  flowValue = flowSensorReagent4.getScaledFlowValue();
  Serial.print(F(",FS4,"));
  Serial.print((flowValue >= 0) ? String(flowValue, 1).c_str() : "N/A");
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
void handleFlowSensorReset(unsigned long currentTime, Stream *response, EthernetClient client)
{
  const unsigned long resetDuration = 5;

  for (int i = 0; i < 4; i++)
  {
    if (flowSensorReset.resetInProgress[i] && millis() - flowSensorReset.resetStartTime[i] >= resetDuration)
    {
      flowSensorReset.resetInProgress[i] = false; // Reset is complete
      if (Serial)
      {
        response->print(F("Flow sensor reset completed for valve "));
        response->println(i + 1);
      }
      if (client)
      {
        client.print(F("Flow sensor reset completed for valve "));
        client.println(i + 1);
      }
      flowSensors[i]->handleReset();
    }
  }
}

// Function to handle serial input for commands
void handleSerialCommands()
{
  static char commandBuffer[32]; // Buffer to store incoming command
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
void handleTCPCommands()
{
  String tcpCommand = tcpServer.handleClient(); // Get the command from the TCP server

  if (tcpCommand.length() > 0)
  {

    char commandBuffer[32];
    tcpCommand.toCharArray(commandBuffer, sizeof(commandBuffer));

    tcpServer.getClient().println("Command received: " + tcpCommand);

    commander.execute(commandBuffer, &tcpServer.getClient());

    tcpServer.getClient().println(F("Command processed."));
    tcpServer.getClient().flush(); // Ensure the message is sent
  }
}

// Function to monitor overflow sensors every 50ms
void monitorOverflowSensors(unsigned long currentTime, Stream *response, EthernetClient client)
{
  static unsigned long previousOverflowCheckTime = 0;

  if (currentTime - previousOverflowCheckTime >= 25)
  { // Check every 25ms
    previousOverflowCheckTime = currentTime;

    // Loop through the overflow sensors and check if any have triggered overflow
    for (int i = 0; i < 4; i++)
    {
      // Skip overflow monitoring if the trough is in fill mode
      if (valveControls[i].fillMode)
      {
        continue;
      }

      // Proceed with overflow handling for troughs not in fill mode
      if (overflowSensors[i]->loop() == 1)
      {
        handleOverflowCondition(i + 1, response); // Handle overflow as usual for troughs not in fill mode
      }
    }
  }
}

// Function to monitor flow sensors for each valve
void monitorFlowSensors(unsigned long currentTime, Stream *response, EthernetClient client)
{
  const unsigned long resetDuration = 5;
  const unsigned long flowTimeoutPeriod = 5000;     // Flow timeout period (milliseconds)
  const unsigned long bubbleDetectionPeriod = 5000; // Time to wait for continuous bubbles before timeout

  static unsigned long bubbleStartTime[4] = {0, 0, 0, 0}; // Time when bubbles were first detected for each valve

  for (int i = 0; i < 4; i++)
  {
    if (valveControls[i].isDispensing && currentTime - valveControls[i].lastFlowCheckTime >= 25)
    {
      valveControls[i].lastFlowCheckTime = currentTime;

      // Skip timeout check if the valve is manually controlled
      if (valveControls[i].manualControl)
      {
        continue;
      }

      // Skip flow check for a brief period after reset to allow valid readings
      if (flowSensorReset.resetInProgress[i] && currentTime - flowSensorReset.resetStartTime[i] < resetDuration + 50)
      {
        // Give the sensor some additional time after reset (e.g., 50ms buffer)
        continue;
      }

      float currentFlowValue = getFlowSensorValue(i);

      // Check if bubble sensor indicates an empty line (bubble detected)
      if (!bubbleSensors[i]->isLiquidDetected())
      {
        // Start or continue counting the time bubbles are detected
        if (bubbleStartTime[i] == 0)
        {
          bubbleStartTime[i] = currentTime;
        }
        else if (currentTime - bubbleStartTime[i] >= bubbleDetectionPeriod)
        {
          valveControls[i].fillMode = false;
          if (Serial)
          {
            response->print(F("Timeout occurred: Continuous air detected for valve "));
            response->println(i + 1);
          }
          if (client)
          {
            client.print(F("Timeout occurred: Continuous air detected for valve "));
            client.println(i + 1);
          }
          handleTimeoutCondition(i + 1, response); // Close valves and stop dispensing
          bubbleStartTime[i] = 0;                  // Reset the bubble start time after timeout
          continue;
        }
      }
      else
      {
        // Reset the bubble detection if liquid is detected
        bubbleStartTime[i] = 0;
      }

      if (currentFlowValue < 0)
      {
        if (Serial)
        {
          response->print(F("Invalid flow reading for valve "));
          response->println(i + 1);
        }
        if (client)
        {
          client.print(F("Invalid flow reading for valve "));
          client.println(i + 1);
        }
        handleTimeoutCondition(i + 1, response);
        continue;
      }

      // Check if the flow is improving or stable
      if (valveControls[i].targetVolume > 0 && currentFlowValue >= valveControls[i].targetVolume)
      {
        if (Serial)
        {
          response->print(F("Target volume reached for valve "));
          response->println(i + 1);
        }
        if (client)
        {
          client.print(F("Target volume reached for valve "));
          client.println(i + 1);
        }
        handleTimeoutCondition(i + 1, response); // Close valves and stop dispensing
        continue;
      }

      // Trigger a flow timeout if the flow hasn’t changed for flowTimeoutPeriod
      if (currentFlowValue == valveControls[i].lastFlowValue)
      {
        if (currentTime - valveControls[i].lastFlowChangeTime >= flowTimeoutPeriod)
        {
          valveControls[i].fillMode = false;
          if (Serial)
          {
            response->print(F("Timeout occurred: No flow detected for valve "));
            response->println(i + 1);
          }
          if (client)
          {
            client.print(F("Timeout occurred: No flow detected for valve "));
            client.println(i + 1);
          }
          handleTimeoutCondition(i + 1, response);
        }
      }
      else
      {
        valveControls[i].lastFlowValue = currentFlowValue;
        valveControls[i].lastFlowChangeTime = currentTime;
      }
    }
  }
}

// Function to get the flow sensor value for a specific valve
float getFlowSensorValue(int valveIndex)
{
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
void checkModbusConnection(unsigned long currentTime)
{
  static unsigned long previousModbusCheckTime = 0;
  if (currentTime - previousModbusCheckTime >= 500)
  {
    modbus.checkConnection();
    previousModbusCheckTime = currentTime;
  }
}

// Function to log system state at regular intervals
void logSystemState(unsigned long currentTime)
{
  if (currentTime - logging.previousLogTime >= logging.logInterval)
  {
    if (!Serial.available())
    {
      log();
    }
    logging.previousLogTime = currentTime;
  }
}

// Helper function for resetting flow sensors
void resetFlowSensor(int sensorIndex, FDXSensor *flowSensors[])
{
  // Set the reset flags and start the reset process
  flowSensorReset.resetInProgress[sensorIndex] = true;
  flowSensorReset.resetStartTime[sensorIndex] = millis();
  flowSensors[sensorIndex]->startResetFlow();

  // Allow sensor to stabilize
  delay(100);
}

// ======================[Command Functions for Valve Control]====================

// Command to set reagent valves (setRV <valve number> <0/1> or setRV all <0/1>)
void cmd_set_reagent_valve(char *args, Stream *response)
{
  int localValveNumber = -1;
  int state = -1;

  if (sscanf(args, "%s %d", commandVars.valveArg, &state) == 2)
  {
    if (strncmp(commandVars.valveArg, "all", 3) == 0)
    {
      // Handle "setRV all <0/1>" command to open/close all reagent valves
      for (int i = 0; i < 4; i++)
      {
        valveControls[i].fillMode = false;             // Disable fill mode for all troughs
        valveControls[i].manualControl = (state == 1); // Set manualControl flag

        if (state == 1)
        {
          reagentValves[i]->openValve();
        }
        else
        {
          reagentValves[i]->closeValve();
        }
      }

      // Output response based on state
      if (state == 1)
      {
        response->println(F("All reagent valves opened."));
      }
      else
      {
        response->println(F("All reagent valves closed."));
      }
    }
    else
    {
      // Handle "setRV <valve number> <0/1>" command for individual reagent valves
      localValveNumber = atoi(commandVars.valveArg); // Parse the local valve number
      if (localValveNumber >= 1 && localValveNumber <= 4 && (state == 0 || state == 1))
      {
        // Disable fill mode for the specific trough
        valveControls[localValveNumber - 1].fillMode = false;

        SolenoidValve *valve = reagentValves[localValveNumber - 1];

        if (state == 1)
        {
          valve->openValve();
          valveControls[localValveNumber - 1].isDispensing = true;
          valveControls[localValveNumber - 1].manualControl = true; // Set manualControl flag
          response->print(F("Reagent valve "));
          response->print(localValveNumber); // Print the valve number
          response->println(F(" opened."));
        }
        else
        {
          valve->closeValve();
          valveControls[localValveNumber - 1].isDispensing = false;
          valveControls[localValveNumber - 1].manualControl = false; // Reset manualControl flag
          response->print(F("Reagent valve "));
          response->print(localValveNumber); // Print the valve number
          response->println(F(" closed."));
        }
      }
      else
      {
        response->println(F("Invalid reagent valve command. Use setRV <1-4> <0/1> or setRV <all> <0/1>."));
      }
    }
  }
  else
  {
    response->println(F("Invalid reagent valve command."));
  }
}

// Command to set media valves (setMV <valve number> <0/1> or setMV all <0/1>)
void cmd_set_media_valve(char *args, Stream *response)
{
  int localValveNumber = -1;
  int state = -1;

  if (sscanf(args, "%s %d", commandVars.valveArg, &state) == 2)
  {
    if (strncmp(commandVars.valveArg, "all", 3) == 0)
    {
      // Handle "setMV all <0/1>" command to open/close all media valves
      for (int i = 0; i < 4; i++)
      {
        valveControls[i].fillMode = false;             // Disable fill mode for all troughs
        valveControls[i].manualControl = (state == 1); // Set manualControl flag for all valves

        if (state == 1)
        {
          mediaValves[i]->openValve();
        }
        else
        {
          mediaValves[i]->closeValve();
        }
      }

      // Output response based on state
      if (state == 1)
      {
        response->println(F("All media valves opened."));
      }
      else
      {
        response->println(F("All media valves closed."));
      }
    }
    else
    {
      // Handle "setMV <valve number> <0/1>" command for individual media valves
      localValveNumber = atoi(commandVars.valveArg); // Use local variable

      if (localValveNumber >= 1 && localValveNumber <= 4 && (state == 0 || state == 1))
      {
        // Disable fill mode for the specific trough
        valveControls[localValveNumber - 1].fillMode = false;

        SolenoidValve *valve = mediaValves[localValveNumber - 1];

        if (state == 1)
        {
          valve->openValve();
          valveControls[localValveNumber - 1].manualControl = true; // Set manualControl flag
          response->print(F("Media valve "));
          response->print(localValveNumber); // Print the valve number
          response->println(F(" opened."));
        }
        else
        {
          valve->closeValve();
          valveControls[localValveNumber - 1].manualControl = false; // Reset manualControl flag
          response->print(F("Media valve "));
          response->print(localValveNumber); // Print the valve number
          response->println(F(" closed."));
        }
      }
      else
      {
        response->println(F("Invalid media valve command. Use setMV <1-4> <0/1> or setMV <all> <0/1>."));
      }
    }
  }
  else
  {
    response->println(F("Invalid media valve command."));
  }
}

// Command to set waste valves (setWV <valve number> <0/1> or setWV all <0/1>)
void cmd_set_waste_valve(char *args, Stream *response)
{
  int localValveNumber = -1;
  int state = -1;

  if (sscanf(args, "%s %d", commandVars.valveArg, &state) == 2)
  {
    if (strncmp(commandVars.valveArg, "all", 3) == 0)
    {
      if (state == 0 || state == 1)
      {
        // Handle "setWV all <0/1>" command to open/close all waste valves
        for (int i = 0; i < 4; i++)
        {
          if (state == 1)
          {
            wasteValves[i]->openValve();
          }
          else
          {
            wasteValves[i]->closeValve();
          }
        }

        // Output response based on state
        if (state == 1)
        {
          response->println(F("All waste valves opened."));
        }
        else
        {
          response->println(F("All waste valves closed."));
        }
      }
      else
      {
        // Handle "setWV <valve number> <0/1>" command for individual waste valves
        localValveNumber = atoi(commandVars.valveArg); // Use local variable

        if (localValveNumber >= 1 && localValveNumber <= 4 && (state == 0 || state == 1))
        {
          // Disable fill mode for the specific trough
          valveControls[localValveNumber - 1].fillMode = false;

          SolenoidValve *valve = wasteValves[localValveNumber - 1];

          if (state == 1)
          {
            valve->openValve();
            response->print(F("Waste valve "));
            response->print(localValveNumber); // Print the valve number
            response->println(F(" opened."));
          }
          else
          {
            valve->closeValve();
            response->print(F("Waste valve "));
            response->print(localValveNumber); // Print the valve number
            response->println(F(" closed."));
          }
        }
        else
        {
          response->println(F("Invalid waste valve command. Use setWV <1-4> <0/1> or setWV <all> <0/1>."));
        }
      }
    }
    else
    {
      response->println(F("Invalid waste valve command."));
    }
  }
}

// Command to set pressure valve (setPV <percentage>)
void cmd_set_pressure_valve(char *args, Stream *response)
{
  int percentage;

  // Use sscanf to extract the percentage directly from the args
  if (sscanf(args, "%d", &percentage) == 1 && percentage >= 0 && percentage <= 100)
  {
    // Set the pressure valve to the percentage position
    pressureValve.setPosition(percentage);
    response->print(F("Pressure valve set to "));
    response->print(percentage);
    response->println(F("%."));
  }
  else
  {
    response->println(F("Error: Invalid value for pressure valve. Use a percentage between 0 and 100."));
  }
}

// ======================[Command Functions for Flow Sensors]=====================

// Command to reset flow sensor (resetFS <sensor number> or resetFS all)
void cmd_reset_flow_sensor(char *args, Stream *response)
{
  int sensorNumber = -1;
  if (!modbus.isConnected())
  {
    response->println(F("Error: Modbus not connected. Cannot process reset command."));
    return;
  }

  if (strncmp(commandVars.valveArg, "all", 3) == 0)
  {
    // Reset all flow sensors using a loop
    for (int i = 0; i < 4; i++)
    {
      valveControls[i].fillMode = false;
      valveControls[i].manualControl = true; // Set manual control flag
      resetFlowSensor(i, flowSensors);
      flowSensorReset.resetInProgress[i] = false;
    }
    response->println(F("All flow sensors reset initiated."));
  }
  else
  {
    if (sscanf(args, "%d", &sensorNumber) == 1 && sensorNumber >= 1 && sensorNumber <= 4)
    {
      valveControls[sensorNumber - 1].fillMode = false;
      valveControls[sensorNumber - 1].manualControl = true;
      resetFlowSensor(sensorNumber - 1, flowSensors);
      flowSensorReset.resetInProgress[sensorNumber - 1] = false;

      response->print(F("Flow sensor "));
      response->print(sensorNumber);
      response->println(F(" reset initiated."));
    }
    else
    {
      response->println(F("Error: Invalid flow sensor number. Use 1-4 or 'all'."));
    }
  }
}

// Command to set the logging frequency (setLF <milliseconds>)
void cmd_set_log_frequency(char *args, Stream *response)
{
  int newLogInterval;
  if (sscanf(args, "%d", &newLogInterval) == 1 && newLogInterval > 0)
  {
    logging.logInterval = newLogInterval;
    response->print(F("Log frequency set to "));
    response->print(logging.logInterval);
    response->println(F(" milliseconds."));
  }
  else
  {
    response->println(F("Error: Invalid log frequency. Please provide a valid number in milliseconds."));
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
    response->println(F("System already pressurized."));
    return true;
  }

  // Set the pressure valve to 100%
  pressureValve.setPosition(100);
  response->println(F("Pressure valve set to 100%."));

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
  response->print(F("Error: Pressure threshold not reached. Current pressure: "));
  response->print(currentPressure);
  response->println(F(" psi. Operation aborted."));
  return false;
}

// ======================[Command Functions for Dispensing Control]===============
// Command to dispense reagent (dispenseR <valve number> [volume])
void cmd_dispense_reagent(char *args, Stream *response)
{
  int localValveNumber = -1; // Changed to localValveNumber
  int requestedVolume = -1;  // Changed to int for easier parsing

  // Check if Modbus is connected
  if (!modbus.isConnected())
  {
    response->println(F("Error: Modbus not connected. Cannot process dispense command."));
    return;
  }

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  const int MIN_VOLUME = 1;   // Use int instead of float
  const int MAX_VOLUME = 150; // Use int instead of float

  // Check and set the pressure using the helper function
  if (!checkAndSetPressure(response, PRESSURE_THRESHOLD_PSI, PRESSURE_TIMEOUT_MS))
  {
    return; // If pressure check fails, abort the dispense operation
  }

  // First, try to parse both the valve number and volume as integers
  int parsedItems = sscanf(args, "%d %d", &localValveNumber, &requestedVolume);

  // If only valve number is provided, parsedItems will be 1
  if (parsedItems == 1)
  {
    requestedVolume = -1; // Set for continuous dispensing
  }
  else if (parsedItems != 2) // Invalid input
  {
    response->println(F("Error: Invalid command format. Use 'dispenseR <valve number> [volume]'."));
    return;
  }

  response->print(F("Valve number: "));
  response->println(localValveNumber);
  response->print(F("Requested volume: "));
  if (requestedVolume > 0)
  {
    response->println(requestedVolume);
  }
  else
  {
    response->println(F("Continuous dispense (no volume specified)."));
  }

  if (localValveNumber >= 1 && localValveNumber <= 4)
  {
    // Disable fill mode for the specific trough
    valveControls[localValveNumber - 1].fillMode = false;

    // Close reagent and media valves using a loop
    reagentValves[localValveNumber - 1]->closeValve();
    mediaValves[localValveNumber - 1]->closeValve();

    // Select the correct overflow sensor
    OverflowSensor *overflowSensor = overflowSensors[localValveNumber - 1];

    if (overflowSensor != nullptr && overflowSensor->isOverflowing())
    {
      response->print(F("Cannot dispense: Overflow detected for valve "));
      response->println(localValveNumber);
      return; // Stop if overflow is detected
    }

    // Validate and set volume
    if (requestedVolume > 0)
    {
      if (requestedVolume < MIN_VOLUME)
      {
        response->print(F("Error: Requested volume too low. Minimum volume is "));
        response->print(MIN_VOLUME);
        response->println(F(" mL."));
        return;
      }
      else if (requestedVolume > MAX_VOLUME)
      {
        response->print(F("Error: Requested volume too high. Maximum volume is "));
        response->print(MAX_VOLUME);
        response->println(F(" mL."));
        return;
      }
      valveControls[localValveNumber - 1].targetVolume = requestedVolume;
      response->print(F("Target volume set for valve "));
      response->print(localValveNumber);
      response->print(F(": "));
      response->print(valveControls[localValveNumber - 1].targetVolume);
      response->println(F(" mL"));
    }
    else
    {
      valveControls[localValveNumber - 1].targetVolume = -1; // Continuous dispense
      response->println(F("Continuous dispensing (no target volume specified)."));
    }

    // Reset flow sensor for the specific valve
    resetFlowSensor(localValveNumber - 1, flowSensors);
    response->print(F("Flow sensor reset initiated for reagent "));
    response->println(localValveNumber);

    // Open the reagent and media valves for the specific valve
    openValves(localValveNumber, response);

    // Track the dispensing state for the valve
    valveControls[localValveNumber - 1].isDispensing = true;
  }
  else
  {
    response->println(F("Error: Invalid valve number. Use 1-4."));
  }
}

// Command to stop dispensing (stopD <valve number> or stopD all)
void cmd_stop_dispense(char *args, Stream *response)
{
  int localValveNumber = -1; // Define the local variable at the beginning

  if (strncmp(commandVars.valveArg, "all", 3) == 0)
  {
    // Stop all reagent and media valves and disable fill mode
    for (int i = 0; i < 4; i++)
    {
      valveControls[i].fillMode = false; // Disable fill mode for all troughs
      closeValves(i + 1, response);      // Close the reagent and media valves
    }

    // Reset all flow sensors using a loop
    for (int i = 0; i < 4; i++)
    {
      flowSensors[i]->startResetFlow(); // Reset flow sensors
      delay(100);
    }

    response->println(F("All valves and flow sensors reset."));
    for (int i = 0; i < 4; i++)
    {
      valveControls[i].isDispensing = false; // Stop dispensing
      valveControls[i].dispensingValveNumber = -1;
    }
  }
  else
  {
    if (sscanf(args, "%d", &localValveNumber) == 1 && localValveNumber >= 1 && localValveNumber <= 4)
    {
      valveControls[localValveNumber - 1].fillMode = false; // Disable fill mode for the specific valve
      closeValves(localValveNumber, response);              // Close reagent and media valves

      // Reset the flow sensor for the specific valve
      flowSensors[localValveNumber - 1]->startResetFlow();
      delay(100);

      valveControls[localValveNumber - 1].isDispensing = false;
      valveControls[localValveNumber - 1].dispensingValveNumber = -1;
    }
    else
    {
      response->println(F("Error: Invalid valve number. Use 1-4 or 'all'."));
    }
  }
}

// Helper function to handle overflow conditions
void handleOverflowCondition(int triggeredValveNumber, Stream *response)
{
  if (valveControls[triggeredValveNumber - 1].isDispensing) // Check if the valve is dispensing
  {
    closeValves(triggeredValveNumber, &Serial);                         // Close the valve
    valveControls[triggeredValveNumber - 1].isDispensing = false;       // Stop the valve from dispensing
    valveControls[triggeredValveNumber - 1].dispensingValveNumber = -1; // Reset dispensing valve number

    response->print(F("Overflow detected: Valves closed for valve "));
    response->println(triggeredValveNumber);

    // Reset the flow sensor for the specific valve
    flowSensors[triggeredValveNumber - 1]->startResetFlow();
    delay(100);

    // Reset the timeout mechanism for this valve
    valveControls[triggeredValveNumber - 1].lastFlowCheckTime = 0;  // Reset the flow check timer
    valveControls[triggeredValveNumber - 1].lastFlowChangeTime = 0; // Reset the last flow change time
  }
}

// Helper function to close both reagent and media valves for a given valve number
void closeValves(int valveNumber, Stream *response)
{
  if (valveNumber >= 1 && valveNumber <= 4)
  {
    // Close the reagent valve
    reagentValves[valveNumber - 1]->closeValve();
    response->print(F("Reagent valve "));
    response->print(valveNumber);
    response->println(F(" closed."));

    // Close the media valve
    mediaValves[valveNumber - 1]->closeValve();
    response->print(F("Media valve "));
    response->print(valveNumber);
    response->println(F(" closed."));
  }
  else
  {
    response->println(F("Error: Invalid valve number."));
  }
}

// Helper function to open both reagent and media valves for a given valve number
void openValves(int valveNumber, Stream *response)
{
  if (valveNumber >= 1 && valveNumber <= 4)
  {
    // Open the reagent valve
    reagentValves[valveNumber - 1]->openValve();
    response->print(F("Reagent valve "));
    response->print(valveNumber);
    response->println(F(" opened."));

    // Open the media valve
    mediaValves[valveNumber - 1]->openValve();
    response->print(F("Media valve "));
    response->print(valveNumber);
    response->println(F(" opened."));
  }
  else
  {
    response->println(F("Error: Invalid valve number."));
  }
}

// Helper function to handle timeout conditions
void handleTimeoutCondition(int triggeredValveNumber, Stream *response)
{
  closeValves(triggeredValveNumber, &Serial);

  // Reset the flow sensor for the specific valve using an array
  flowSensors[triggeredValveNumber - 1]->startResetFlow();
  delay(100);

  // Reset the valve state in the valveControls array
  valveControls[triggeredValveNumber - 1].isDispensing = false;
  valveControls[triggeredValveNumber - 1].targetVolume = -1;
}

// monitorFillSensors function to monitor overflow sensors and fill the troughs
void monitorFillSensors(unsigned long currentTime, Stream *response, EthernetClient client)
{
  const float MAX_FILL_VOLUME_ML = 150.0;       // Maximum fill volume in mL
  const unsigned long MAX_FILL_TIME_MS = 60000; // Maximum fill time in milliseconds (1.5 minutes)

  // Arrays for overflow sensors, reagent valves, media valves, flow sensors
  static unsigned long fillStartTime[4] = {0, 0, 0, 0}; // Track fill start time for each valve
  static float previousFlowValue[4] = {0, 0, 0, 0};     // Track the last known flow value for each valve

  // 1. Track valve states to know when dispensing starts and stops (outside interval loop)
  for (int i = 0; i < 4; i++)
  {
    bool isReagentValveOpen = reagentValves[i]->isValveOpen();
    bool isMediaValveOpen = mediaValves[i]->isValveOpen();

    // If both valves are open, dispensing has started
    if (isReagentValveOpen && isMediaValveOpen)
    {
      if (!valveControls[i].isDispensing)
      {
        valveControls[i].isDispensing = true; // Start dispensing
        if (fillStartTime[i] == 0)            // Only set start time if it's not already set
        {
          fillStartTime[i] = currentTime; // Set the start time when filling starts
          previousFlowValue[i] = 0;       // Reset flow tracking
          if (Serial)
          {
            response->print(F("Valve "));
            response->print(i + 1);
            response->println(F(" opened, starting dispensing."));
          }
          if (client)
          {
            client.print(F("Valve "));
            client.print(i + 1);
            client.println(F(" opened, starting dispensing."));
          }
        }
      }
    }
    // If both valves are closed, dispensing has stopped
    else if (!isReagentValveOpen && !isMediaValveOpen)
    {
      if (valveControls[i].isDispensing)
      {
        valveControls[i].isDispensing = false; // Stop dispensing
        if (Serial)
        {
          response->print(F("Valve "));
          response->print(i + 1);
          response->println(F(" closed, stopping dispensing."));
        }
        if (client)
        {
          client.print(F("Valve "));
          client.print(i + 1);
          client.println(F(" closed, stopping dispensing."));
        }

        // Reset the flow sensor when dispensing stops
        resetFlowSensor(i, flowSensors);

        // Reset tracking for the next fill
        fillStartTime[i] = 0;     // Reset fill start time for the next cycle
        previousFlowValue[i] = 0; // Reset flow tracking
      }
    }
  }

  // 2. Implement the timeout and volume checks outside the interval control loop
  for (int i = 0; i < 4; i++)
  {
    if (valveControls[i].fillMode && valveControls[i].isDispensing) // Check only if fill mode is active and valve is dispensing
    {
      // Access the correct flow sensor
      FDXSensor *flowSensor = flowSensors[i];
      float currentFlowValue = flowSensor->getScaledFlowValue();

      // Calculate the new volume added since the last check
      float addedVolume = currentFlowValue - previousFlowValue[i];
      if (addedVolume < 0)
        addedVolume = 0;                       // Ensure no negative flow values
      previousFlowValue[i] = currentFlowValue; // Update the previous flow value

      // Check if fill timeout has occurred based on volume or time
      bool volumeExceeded = currentFlowValue >= MAX_FILL_VOLUME_ML;
      bool timeExceeded = currentTime - fillStartTime[i] >= MAX_FILL_TIME_MS;

      if (volumeExceeded || timeExceeded)
      {
        // Disable fill mode for this valve
        valveControls[i].fillMode = false;
        reagentValves[i]->closeValve();
        mediaValves[i]->closeValve();

        if (volumeExceeded)
        {
          if (Serial)
          {
            response->print(F("Warning: Fill timeout for trough "));
            response->print(i + 1); // Print the trough number
            response->print(F(": maximum volume ("));
            response->print(currentFlowValue, 2); // Print the currentFlowValue with 2 decimal places
            response->println(F(" mL) reached."));
          }
          if (client)
          {
            client.print(F("Warning: Fill timeout for trough "));
            client.print(i + 1); // Print the trough number
            client.print(F(": maximum volume ("));
            client.print(currentFlowValue, 2); // Print the currentFlowValue with 2 decimal places
            client.println(F(" mL) reached."));
          }
        }
        else if (timeExceeded)
        {
          if (Serial)
          {
            response->print(F("Warning: Fill timeout for trough "));
            response->print(i + 1); // Print the trough number
            response->print(F(": maximum time ("));
            response->print(MAX_FILL_TIME_MS / 1000.0, 2); // Print the time in seconds with 2 decimal places
            response->println(F(" seconds) reached."));
          }
          if (client)
          {
            client.print(F("Warning: Fill timeout for trough "));
            client.print(i + 1); // Print the trough number
            client.print(F(": maximum time ("));
            client.print(MAX_FILL_TIME_MS / 1000.0, 2); // Print the time in seconds with 2 decimal places
            client.println(F(" seconds) reached."));
          }
        }

        // Reset tracking for the next fill
        fillStartTime[i] = 0;     // Reset start time when fill completes
        previousFlowValue[i] = 0; // Reset flow tracking
        continue;                 // Skip further checks for this valve
      }
    }
  }

  // 3. Interval-based check for overflow and managing valve states for the next fill cycle
  const unsigned long SENSOR_CHECK_INTERVAL = 500; // Check interval for overflow
  for (int i = 0; i < 4; i++)
  {
    if (valveControls[i].fillMode && currentTime - valveControls[i].fillCheckTime >= SENSOR_CHECK_INTERVAL)
    {
      valveControls[i].fillCheckTime = currentTime; // Reset the check time

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
          if (Serial)
          {
            response->print(F("Overflow detected for trough "));
            response->print(i + 1);                    // Print the trough number
            response->println(F(", closing valves.")); // Print the rest of the message and a newline
          }
          if (client)
          {
            client.print(F("Overflow detected for trough "));
            client.print(i + 1);                    // Print the trough number
            client.println(F(", closing valves.")); // Print the rest of the message and a newline
          }
        }
      }
      // If overflow is not detected and valves are closed, reopen them and prepare for the next fill cycle
      else if (!reagentValve->isValveOpen())
      {
        reagentValve->openValve();
        mediaValve->openValve();
        if (Serial)
        {
          response->print(F("Trough "));
          response->print(i + 1);                                            // Print the trough number
          response->println(F(" not overflowing, opening valves to fill.")); // Print the rest of the message and a newline
        }
        if (client)
        {
          client.print(F("Trough "));
          client.print(i + 1);                                            // Print the trough number
          client.println(F(" not overflowing, opening valves to fill.")); // Print the rest of the message and a newline
        }
      }
    }
  }
}

// Command to fill the reagent (fillR <valve number> or fillR all)
void cmd_fill_reagent(char *args, Stream *response)
{
  int localValveNumber = -1; // Local variable for valve number

  // Check if Modbus is connected
  if (!modbus.isConnected())
  {
    response->println(F("Error: Modbus not connected. Cannot process fill command."));
    return;
  }

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  // Check and set the pressure using the helper function
  if (!checkAndSetPressure(response, PRESSURE_THRESHOLD_PSI, PRESSURE_TIMEOUT_MS))
  {
    return; // If pressure check fails, abort the fill operation
  }

  // Check if the input is "all" or a specific valve number
  if (strncmp(args, "all", 3) == 0)
  {
    // Handle "fillR all" case
    for (int i = 0; i < 4; i++)
    {
      // Close and then open the valves to start a fresh fill
      reagentValves[i]->closeValve();
      mediaValves[i]->closeValve();

      reagentValves[i]->openValve();
      mediaValves[i]->openValve();

      // Reset the flow sensor after the valves have been opened
      resetFlowSensor(i, flowSensors);

      valveControls[i].fillMode = true; // Enable fill mode for this valve
    }
    response->println(F("Filling started for all valves."));
  }
  else if (sscanf(args, "%d", &localValveNumber) == 1 && localValveNumber >= 1 && localValveNumber <= 4)
  {
    // Handle "fillR <valve number>" case
    // Close and then open the valves to start a fresh fill
    reagentValves[localValveNumber - 1]->closeValve();
    mediaValves[localValveNumber - 1]->closeValve();

    reagentValves[localValveNumber - 1]->openValve();
    mediaValves[localValveNumber - 1]->openValve();

    // Reset the flow sensor after the valves have been opened
    resetFlowSensor(localValveNumber - 1, flowSensors);

    valveControls[localValveNumber - 1].fillMode = true; // Enable fill mode for this valve
  }
  else
  {
    response->println(F("Error: Invalid valve number. Use 1-4 or 'all'."));
  }
}

// Command to stop filling the reagent (stopF <valve number> or stopF all)
void cmd_stop_fill_reagent(char *args, Stream *response)
{
  int localValveNumber = -1; // Local variable for valve number

  // Parse the input command
  if (sscanf(args, "%s", commandVars.valveArg) == 1)
  {
    if (strncmp(commandVars.valveArg, "all", 3) == 0)
    {
      // Disable fill mode for all troughs
      for (int i = 0; i < 4; i++)
      {
        valveControls[i].fillMode = false; // Use the updated `fillMode` in the `valveControls` struct
        closeValves(i + 1, response);      // Close all valves
      }
      response->println(F("Fill mode disabled for all troughs."));
    }
    else if (sscanf(commandVars.valveArg, "%d", &localValveNumber) == 1 && localValveNumber >= 1 && localValveNumber <= 4)
    {
      // Disable fill mode for the specific trough
      valveControls[localValveNumber - 1].fillMode = false; // Use `valveControls` instead of the global `fillMode[]`
      closeValves(localValveNumber, response);              // Close the specific valve
      response->print(F("Fill mode disabled for trough "));
      response->println(localValveNumber);
    }
    else
    {
      response->println(F("Error: Invalid trough number. Use 1-4 or 'all'."));
    }
  }
  else
  {
    response->println(F("Error: Invalid stop fill command. Use 'stopF <valve number>' or 'stopF all'."));
  }
}

// Command to print help information
void cmd_print_help(char *args, Stream *response)
{
  commander.printHelp(response);
}

// Command to get the system state
void cmd_get_system_state(char *args, Stream *response)
{
  int systemState = 1; // Default state is Idle (1)
  bool isDispensing = false;
  bool inFillMode = false;
  bool errorState = false;

  // Check Modbus connection, set error state if Modbus is disconnected
  if (!modbus.isConnected())
  {
    errorState = true;
  }

  // Check if any valve is dispensing
  for (int i = 0; i < 4; i++)
  {
    if (valveControls[i].isDispensing) // Updated to use valveControls
    {
      isDispensing = true;
      break;
    }
  }

  // Check if any trough is in fill mode
  for (int i = 0; i < 4; i++)
  {
    if (valveControls[i].fillMode) // Updated to use valveControls
    {
      inFillMode = true;
      break;
    }
  }

  // Determine the system state based on precedence (Error > Fill Mode > Dispensing > Idle)
  if (errorState)
  {
    systemState = 4; // Error State
  }
  else if (inFillMode)
  {
    systemState = 3; // Fill Mode
  }
  else if (isDispensing)
  {
    systemState = 2; // Dispensing
  }

  // Send the system state as a response to the client
  if (systemState == 1)
  {
    response->println(F("System State: Idle"));
  }
  else if (systemState == 2)
  {
    response->println(F("System State: Dispensing"));
  }
  else if (systemState == 3)
  {
    response->println(F("System State: Fill Mode"));
  }
  else if (systemState == 4)
  {
    response->println(F("System State: Error - Modbus Disconnected"));
  }
}

// Command to reset the Modbus connection
void cmd_modbus_reset(char *args, Stream *response)
{
  // Log the start of the reset process
  response->println(F("Starting Modbus reset..."));

  // Turn off the relay (cut power to the NQ-EP4L)
  digitalWrite(MODBUS_RESET_PIN, LOW);
  response->println(F("Power to Modbus device turned off."));

  delay(2000);

  // Turn the relay back on (restore power to the NQ-EP4L)
  digitalWrite(MODBUS_RESET_PIN, HIGH);
  response->println(F("Power to Modbus device restored."));

  delay(5000);

  modbus.checkConnection();

  // Report the Modbus connection status
  if (modbus.isConnected())
  {
    response->println(F("Modbus connection re-established successfully."));
  }
  else
  {
    response->println(F("Error: Modbus connection could not be re-established."));
  }
}

// Command to provide device IP, TCP/IP, and Modbus info (Serial only)
void cmd_device_info(char *args, Stream *response)
{
  // Check if the command is coming from Serial (local access only)
  if (response == &Serial)
  {
    response->println(F("---- Device Information (Serial Only) ----"));

    IPAddress deviceIP = Ethernet.localIP();
    response->print(F("Device IP Address: "));
    response->println(deviceIP);

    response->print(F("TCP Server IP: "));
    response->println(ip);
    response->print(F("TCP Server Port: "));
    response->println(8080);

    response->print(F("Modbus Server Address: "));
    response->println(modbus.getServerAddress());
    response->print(F("Modbus Port: "));
    response->println(502);

    response->println(F("----------------------------------------"));
  }
  else
  {
    response->println(F("Error: Device information can only be accessed via Serial."));
  }
}

// New function to monitor the priming process in the main loop
void monitorPrimeSensors(unsigned long currentTime, Stream *response, EthernetClient client)
{
  const unsigned long ADDITIONAL_PRIME_TIME_MS = 2000;
  static unsigned long additionalPrimeStartTime[4] = {0, 0, 0, 0}; // Time to track additional priming after liquid detected

  for (int i = 0; i < 4; i++)
  {
    if (valveControls[i].isPriming) // Updated to use valveControls
    {
      // If liquid is detected, start additional priming timer
      if (bubbleSensors[i]->isLiquidDetected() && additionalPrimeStartTime[i] == 0)
      {
        additionalPrimeStartTime[i] = currentTime;
        if (Serial)
        {
          response->print(F("Liquid detected for valve "));
          response->println(i + 1);
        }
        if (client)
        {
          client.print(F("Liquid detected for valve "));
          client.println(i + 1);
        }
      }

      // Stop priming once additional time has passed
      if (additionalPrimeStartTime[i] != 0 && currentTime - additionalPrimeStartTime[i] >= ADDITIONAL_PRIME_TIME_MS)
      {
        reagentValves[i]->closeValve();
        mediaValves[i]->closeValve();
        valveControls[i].manualControl = false;
        valveControls[i].isPriming = false;
        if (Serial)
        {
          response->print(F("Priming complete for valve "));
          response->println(i + 1);
        }
        if (client)
        {
          client.print(F("Priming complete for valve "));
          client.println(i + 1);
        }
        additionalPrimeStartTime[i] = 0;
      }
    }
  }
}

// Command to prime valves (prime <valve number> or prime all)
void cmd_prime_valves(char *args, Stream *response)
{
  int localValveNumber = -1; // Local variable for valve number

  // Check if Modbus is connected
  if (!modbus.isConnected())
  {
    response->println(F("Error: Modbus not connected. Cannot process prime command."));
    return;
  }

  const float PRESSURE_THRESHOLD_PSI = 15.0;
  const unsigned long PRESSURE_TIMEOUT_MS = 500;

  // Check and set the pressure using the helper function
  if (!checkAndSetPressure(response, PRESSURE_THRESHOLD_PSI, PRESSURE_TIMEOUT_MS))
  {
    return; // If pressure check fails, abort the prime operation
  }

  // Check if the input is "all" or a specific valve number
  if (strncmp(args, "all", 3) == 0)
  {
    // Prime all valves
    for (int i = 0; i < 4; i++)
    {
      if (!bubbleSensors[i]->isLiquidDetected())
      {
        // Start priming for valves not already primed
        valveControls[i].fillMode = false;
        valveControls[i].manualControl = true;
        reagentValves[i]->openValve();
        mediaValves[i]->openValve();
        valveControls[i].isPriming = true;
        response->print(F("Priming started for valve "));
        response->println(i + 1);
      }
      else
      {
        response->print(F("Valve "));
        response->print(i + 1);
        response->println(F(" already primed."));
      }
    }
  }
  else if (sscanf(args, "%d", &localValveNumber) == 1 && localValveNumber >= 1 && localValveNumber <= 4)
  {
    if (!bubbleSensors[localValveNumber - 1]->isLiquidDetected())
    {
      // Start priming for the specific valve
      valveControls[localValveNumber - 1].fillMode = false;
      valveControls[localValveNumber - 1].manualControl = true;
      reagentValves[localValveNumber - 1]->openValve();
      mediaValves[localValveNumber - 1]->openValve();
      valveControls[localValveNumber - 1].isPriming = true;
      response->print(F("Priming started for valve "));
      response->println(localValveNumber);
    }
    else
    {
      response->print(F("Valve "));
      response->print(localValveNumber);
      response->println(F(" already primed."));
    }
  }
  else
  {
    response->println(F("Error: Invalid valve number. Use 1-4 or 'all'."));
  }
}

// Command to drain reagent trough (drain <1-4>)
void cmd_drain_trough(char *args, Stream *response)
{
  int troughNumber = -1;

  // Parse the trough number from the arguments
  if (sscanf(args, "%d", &troughNumber) == 1 && troughNumber >= 1 && troughNumber <= 4)
  {
    // Execute the drain logic based on the trough number
    switch (troughNumber)
    {
    case 1:
      wasteValves[0]->openValve();
      wasteValves[2]->closeValve();
      response->println(F("Draining trough 1... Waste valve 1 opened, waste valve 3 closed."));
      break;

    case 2:
      wasteValves[0]->openValve();
      wasteValves[2]->openValve();
      response->println(F("Draining trough 2... Waste valve 1 opened, waste valve 3 opened."));
      break;

    case 3:
      wasteValves[1]->openValve();
      wasteValves[3]->closeValve();
      response->println(F("Draining trough 3... Waste valve 2 opened, waste valve 4 closed."));
      break;

    case 4:
      wasteValves[1]->openValve();
      wasteValves[3]->openValve();
      response->println(F("Draining trough 4... Waste valve 2 opened, waste valve 4 opened."));
      break;

    default:
      response->println(F("Invalid trough number. Use 1-4."));
      return;
    }
  }
  else
  {
    response->println(F("Error: Invalid trough number. Use 'drain <1-4>'."));
  }
}

// Command to stop draining reagent trough (stopDrain <1-4> or stopDrain all)
void cmd_stop_drain_trough(char *args, Stream *response)
{
  int troughNumber = -1;

  // Check if "all" was provided as the argument
  if (strncmp(args, "all", 3) == 0)
  {
    // Stop draining all troughs (close all waste valves)
    wasteValves[0]->closeValve();
    wasteValves[1]->closeValve();
    wasteValves[2]->closeValve();
    wasteValves[3]->closeValve();

    response->println(F("Draining stopped for all troughs. Waste valves closed."));
  }
  else if (sscanf(args, "%d", &troughNumber) == 1 && troughNumber >= 1 && troughNumber <= 4)
  {
    // Execute the stop drain logic based on the trough number
    switch (troughNumber)
    {
    case 1:
      wasteValves[0]->closeValve();
      wasteValves[2]->closeValve();
      response->println(F("Draining stopped for trough 1. Waste valves 1 and 3 closed."));
      break;

    case 2:
      wasteValves[0]->closeValve();
      wasteValves[2]->closeValve();
      response->println(F("Draining stopped for trough 2. Waste valves 1 and 3 closed."));
      break;

    case 3:
      wasteValves[1]->closeValve();
      wasteValves[3]->closeValve();
      response->println(F("Draining stopped for trough 3. Waste valves 2 and 4 closed."));
      break;

    case 4:
      wasteValves[1]->closeValve();
      wasteValves[3]->closeValve();
      response->println(F("Draining stopped for trough 4. Waste valves 2 and 4 closed."));
      break;

    default:
      response->println(F("Invalid trough number. Use 1-4 or 'all'."));
      return;
    }
  }
  else
  {
    response->println(F("Error: Invalid trough number. Use 'stopDrain <1-4>' or 'stopDrain all'."));
  }
}
