#include <Wire.h>
#include <Controllino.h>


void resetI2CBus() {
  Serial.println(F("Resetting I2C bus..."));
  Wire.end();
  delay(100);
  Wire.begin();
}

void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel) {
  Wire.beginTransmission(multiplexerAddr);
  Wire.write(1 << channel); // Select specific channel
  Wire.endTransmission();
}

class FlowSensor {
private:
  uint8_t multiplexerAddr; // Multiplexer I2C address
  uint8_t sensorAddr;      // Sensor I2C address
  uint8_t channel;         // Multiplexer channel
  uint16_t measurementCmd; // Measurement command
  bool sensorInitialized;  // Tracks if the sensor is initialized
  bool sensorStopped;      // Tracks if the sensor is intentionally stopped

public:
  float flowRate;     // Flow rate in mL/min
  float temperature;  // Temperature in °C
  int highFlowFlag;   // High flow flag (1 for yes, 0 for no)

  FlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd)
      : multiplexerAddr(muxAddr), sensorAddr(addr), channel(chan),
        measurementCmd(cmd), sensorInitialized(false), sensorStopped(false),
        flowRate(0.0), temperature(0.0), highFlowFlag(0) {}

  bool initializeSensor() {
    if (sensorStopped) {
      Serial.print(F("Sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" is intentionally stopped and will not be initialized."));
      return false;
    }

    selectMultiplexerChannel(multiplexerAddr, channel);

    // Perform a soft reset
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("Soft reset failed"));
      resetI2CBus();
      return false;
    }
    delay(50);

    // Start continuous measurement mode
    Wire.beginTransmission(sensorAddr);
    Wire.write(measurementCmd >> 8); // Send MSB
    Wire.write(measurementCmd & 0xFF); // Send LSB
    if (Wire.endTransmission() != 0) {
      Serial.println(F("Failed to start measurement mode"));
      resetI2CBus();
      return false;
    }

    Serial.println(F("Sensor reinitialized successfully"));
    delay(100); // Allow time for the sensor to stabilize
    sensorInitialized = true;
    return true;
  }

  bool readSensorData() {
    if (!sensorInitialized || sensorStopped) {
      return false; // Skip reading if the sensor is not initialized or stopped
    }

    selectMultiplexerChannel(multiplexerAddr, channel);

    Wire.requestFrom(sensorAddr, (uint8_t)9); // Request 9 bytes from the sensor
    if (Wire.available() < 9) {
      Serial.println(F("Error: Not enough bytes received, sensor disconnected"));
      sensorInitialized = false; // Mark sensor as uninitialized
      return false;
    }

    // Parse data
    uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
    Wire.read(); // Skip flow CRC
    uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
    Wire.read(); // Skip temp CRC
    uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
    Wire.read(); // Skip aux CRC

    // Convert to physical values
    flowRate = (int16_t)flowRaw / 32.0;      // Scale factor for flow
    temperature = (int16_t)tempRaw / 200.0; // Scale factor for temperature
    highFlowFlag = (auxRaw & 0x02) ? 1 : 0; // Bit 1: High flow flag

    return true;
  }

  void stopMeasurement() {
    selectMultiplexerChannel(multiplexerAddr, channel);

    // Send stop measurement command
    Wire.beginTransmission(sensorAddr);
    Wire.write(0x3F); // Stop command MSB
    Wire.write(0xF9); // Stop command LSB
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" stopped measurement mode"));
    } else {
      Serial.print(F("Failed to stop measurement mode for sensor on channel "));
      Serial.println(channel);
    }

    sensorInitialized = false; // Mark the sensor as no longer initialized
    sensorStopped = true;      // Mark the sensor as intentionally stopped
  }

  void startMeasurement() { // Previously resumeMeasurement
    sensorStopped = false; // Clear the stopped state
    if (initializeSensor()) {
      Serial.print(F("Sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" started measurement mode."));
    }
  }
};


FlowSensor flow0(0x70, 0x08, 0, 0x3608); // Multiplexer at 0x70, channel 0
FlowSensor flow1(0x70, 0x08, 1, 0x3608); // Multiplexer at 0x70, channel 1

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("Enter commands to start/stop sensors: start0, stop0, start1, stop1"));

  // // Initialize sensors if you want it to display data upon startup
  // if (!flow0.initializeSensor()) {
  //   Serial.println(F("Flow 0 initialization failed"));
  // }
  // if (!flow1.initializeSensor()) {
  //   Serial.println(F("Flow 1 initialization failed"));
  // }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any extra whitespace or newline characters

    if (command == "start0") {
      flow0.startMeasurement();
      Serial.println(F("Flow 0 started"));
    } else if (command == "stop0") {
      flow0.stopMeasurement();
      Serial.println(F("Flow 0 stopped"));
    } else if (command == "start1") {
      flow1.startMeasurement();
      Serial.println(F("Flow 1 started"));
    } else if (command == "stop1") {
      flow1.stopMeasurement();
      Serial.println(F("Flow 1 stopped"));
    } else {
      Serial.println(F("Unknown command. Use: start0, stop0, start1, stop1"));
    }
  }
}


/////////////////////////////////////////////Code Using String Class///////////////////////////////////////////////////////////
// void loop() {
//   handleSerialCommands();

//   unsigned long currentMillis = millis();
//   static unsigned long lastPrintTime = 0;
//   const unsigned long printInterval = 500;

//   // Temporary variables to hold sensor output
//   String flow0Output = "";
//   String flow1Output = "";

//   // Read and format data from sensor0
//   if (flow0.readSensorData()) {
//     flow0Output = "flow0, " + String(flow0.flowRate, 2) + ", " + String(flow0.temperature, 2) + ", " + String(flow0.highFlowFlag);
//   } else {
//     flow0Output = "flow0, n/a, n/a, n/a"; // Default values for stopped sensor
//   }

//   // Read and format data from sensor1
//   if (flow1.readSensorData()) {
//     flow1Output = "flow1, " + String(flow1.flowRate, 2) + ", " + String(flow1.temperature, 2) + ", " + String(flow1.highFlowFlag);
//   } else {
//     flow1Output = "flow1, n/a, n/a, n/a"; // Default values for stopped sensor
//   }

//   // Print both sensor outputs on the same line
//   if (currentMillis - lastPrintTime >= printInterval) {
//     Serial.println(flow0Output + " | " + flow1Output);
//     lastPrintTime = currentMillis;
//   }
// }


/////////////////////////////////////////////Code Using dtostrf///////////////////////////////////////////////////////////
void loop() {
  handleSerialCommands();

  unsigned long currentMillis = millis();
  static unsigned long lastPrintTime = 0;
  const unsigned long printInterval = 250;

  // Temporary buffers to hold sensor output
  char flow0Output[50]; // Buffer for sensor0 output
  char flow1Output[50]; // Buffer for sensor1 output
  char flow0RateBuffer[10]; // Temporary buffer for flowRate
  char flow0TempBuffer[10]; // Temporary buffer for temperature
  char flow1RateBuffer[10]; // Temporary buffer for flowRate
  char flow1TempBuffer[10]; // Temporary buffer for temperature

  // Read and format data from flow0
  if (flow0.readSensorData()) {
    dtostrf(flow0.flowRate, 6, 2, flow0RateBuffer); // Convert flowRate to string
    dtostrf(flow0.temperature, 6, 2, flow0TempBuffer); // Convert temperature to string
    snprintf(flow0Output, sizeof(flow0Output), "flow0, %s, %s, %d", flow0RateBuffer, flow0TempBuffer, flow0.highFlowFlag);
  } else {
    snprintf(flow0Output, sizeof(flow0Output), "flow0, n/a, n/a, n/a");
  }

  // Read and format data from flow1
  if (flow1.readSensorData()) {
    dtostrf(flow1.flowRate, 6, 2, flow1RateBuffer); // Convert flowRate to string
    dtostrf(flow1.temperature, 6, 2, flow1TempBuffer); // Convert temperature to string
    snprintf(flow1Output, sizeof(flow1Output), "flow1, %s, %s, %d", flow1RateBuffer, flow1TempBuffer, flow1.highFlowFlag);
  } else {
    snprintf(flow1Output, sizeof(flow1Output), "flow1, n/a, n/a, n/a");
  }

  // Print both sensor outputs on the same line
  if (currentMillis - lastPrintTime >= printInterval) {
    Serial.println(String(flow0Output) + " | " + String(flow1Output));
    lastPrintTime = currentMillis;
  }
}

