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
  Wire.write(1 << channel);  // Select specific channel
  Wire.endTransmission();
}

class FlowSensor {
private:
  uint8_t multiplexerAddr;       // Multiplexer I2C address
  uint8_t sensorAddr;            // Sensor I2C address
  uint8_t channel;               // Multiplexer channel
  uint16_t measurementCmd;       // Measurement command
  bool sensorInitialized;        // Tracks if the sensor is initialized
  bool sensorStopped;            // Tracks if the sensor is intentionally stopped
  unsigned long lastUpdateTime;  // Tracks the last time volume was updated

public:
  float flowRate;        // Flow rate in mL/min
  float temperature;     // Temperature in °C
  int highFlowFlag;      // High flow flag (1 for yes, 0 for no)
  int sensorConnected;   // Connection flag (1 = connected, 0 = disconnected)
  float dispenseVolume;  // Volume for specific dispense operations
  float totalVolume;     // Running total volume

  FlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd)
    : multiplexerAddr(muxAddr), sensorAddr(addr), channel(chan),
      measurementCmd(cmd), sensorInitialized(false), sensorStopped(true),
      flowRate(0.0), temperature(0.0), highFlowFlag(0), sensorConnected(0),
      dispenseVolume(0.0), totalVolume(0.0), lastUpdateTime(0) {}

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
      sensorConnected = 0;  // Mark sensor as disconnected
      return false;
    }
    delay(50);

    // Start continuous measurement mode
    Wire.beginTransmission(sensorAddr);
    Wire.write(measurementCmd >> 8);    // Send MSB
    Wire.write(measurementCmd & 0xFF);  // Send LSB
    if (Wire.endTransmission() != 0) {
      Serial.println(F("Failed to start measurement mode"));
      resetI2CBus();
      sensorConnected = 0;  // Mark sensor as disconnected
      return false;
    }

    Serial.println(F("Sensor reinitialized successfully"));
    delay(100);  // Allow time for the sensor to stabilize
    sensorInitialized = true;
    sensorConnected = 1;        // Mark sensor as connected
    lastUpdateTime = millis();  // Initialize the update timestamp
    dispenseVolume = 0.0;       // Reset volume on initialization
    return true;
  }

  bool checkConnection() {
    selectMultiplexerChannel(multiplexerAddr, channel);

    Wire.beginTransmission(sensorAddr);
    if (Wire.endTransmission() == 0) {
      sensorConnected = 1;  // Sensor is connected
      return true;
    } else {
      sensorConnected = 0;  // Sensor is disconnected
      return false;
    }
  }

  bool readSensorData() {
    if (!sensorInitialized || sensorStopped) {
      return false;  // Skip reading if the sensor is not initialized or stopped
    }

    selectMultiplexerChannel(multiplexerAddr, channel);

    Wire.requestFrom(sensorAddr, (uint8_t)9);  // Request 9 bytes from the sensor
    if (Wire.available() < 9) {
      Serial.print(F("Error: Not enough bytes received from sensor on channel "));
      Serial.println(channel);
      sensorInitialized = false;  // Mark sensor as uninitialized
      sensorStopped = true;       // Treat as stopped
      sensorConnected = 0;        // Mark sensor as disconnected
      return false;
    }

    // Parse data
    uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
    Wire.read();  // Skip flow CRC
    uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
    Wire.read();  // Skip temp CRC
    uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
    Wire.read();  // Skip aux CRC

    // Convert to physical values
    flowRate = (int16_t)flowRaw / 32.0;      // Scale factor for flow
    temperature = (int16_t)tempRaw / 200.0;  // Scale factor for temperature
    highFlowFlag = (auxRaw & 0x02) ? 1 : 0;  // Bit 1: High flow flag
    sensorConnected = 1;                     // Mark as connected after successful read

    // Apply flow rate filtering
    if (flowRate < 0) {
      flowRate = 0.0;  // Ignore negative flow rates
    }

    // Update integrated flow (volume)
    unsigned long currentTime = millis();
    if (lastUpdateTime > 0) {
      float elapsedMinutes = (currentTime - lastUpdateTime) / 60000.0;  // Convert ms to minutes
      float increment = flowRate * elapsedMinutes;                      // Calculate volume increment
      dispenseVolume += increment;                                      // Increment specific volume
      totalVolume += increment;                                         // Increment total volume
    }
    lastUpdateTime = currentTime;

    return true;
  }

  void resetVolume() {
    dispenseVolume = 0.0;  // Reset only the dispense volume
    Serial.print(F("Dispense volume reset for sensor on channel "));
    Serial.println(channel);
  }

  void resetTotalVolume() {
    totalVolume = 0.0;  // Reset the total volume
    Serial.print(F("Total volume reset for sensor on channel "));
    Serial.println(channel);
  }

  void stopMeasurement() {
    selectMultiplexerChannel(multiplexerAddr, channel);

    // Send stop measurement command
    Wire.beginTransmission(sensorAddr);
    Wire.write(0x3F);  // Stop command MSB
    Wire.write(0xF9);  // Stop command LSB
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" stopped measurement mode"));
    } else {
      Serial.print(F("Failed to stop measurement mode for sensor on channel "));
      Serial.println(channel);
    }

    sensorInitialized = false;  // Mark the sensor as no longer initialized
    sensorStopped = true;       // Mark the sensor as intentionally stopped
    // Connection status remains updated via checkConnection()
  }

  void startMeasurement() {
    // Check if the sensor is physically connected
    if (!sensorConnected) {
      Serial.print(F("Error: Cannot start measurement for sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" because it is physically disconnected."));
      return;
    }

    sensorStopped = false;  // Clear the stopped state
    if (initializeSensor()) {
      Serial.print(F("Sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" started measurement mode."));
    }
  }
};


FlowSensor flow0(0x70, 0x08, 0, 0x3608);  // Multiplexer at 0x70, channel 0
FlowSensor flow1(0x70, 0x08, 1, 0x3608);  // Multiplexer at 0x70, channel 1

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("Enter commands to start/stop sensors: start0, stop0, start1, stop1, reset0, reset1, resetTotal0, resetTotal1"));
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any extra whitespace or newline characters

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
    } else if (command == "reset0") {
      flow0.resetVolume();
    } else if (command == "reset1") {
      flow1.resetVolume();
    } else if (command == "resetTotal0") {
      flow0.resetTotalVolume();
    } else if (command == "resetTotal1") {
      flow1.resetTotalVolume();
    } else {
      Serial.println(F("Unknown command. Use: start0, stop0, start1, stop1, reset0, reset1, resetTotal0, resetTotal1"));
    }
  }
}

void loop() {
  handleSerialCommands();

  unsigned long currentMillis = millis();
  static unsigned long lastPrintTime = 0;
  const unsigned long printInterval = 500;

  // Check sensor connection status
  flow0.checkConnection();
  flow1.checkConnection();

  // Temporary buffers to hold sensor output
  char flow0Output[100];         // Buffer for flow0 output
  char flow1Output[100];         // Buffer for flow1 output
  char flow0RateBuffer[10];      // Temporary buffer for flowRate
  char flow0TempBuffer[10];      // Temporary buffer for temperature
  char flow0DispenseBuffer[10];  // Temporary buffer for dispense volume
  char flow0TotalBuffer[10];     // Temporary buffer for total volume
  char flow1RateBuffer[10];      // Temporary buffer for flowRate
  char flow1TempBuffer[10];      // Temporary buffer for temperature
  char flow1DispenseBuffer[10];  // Temporary buffer for dispense volume
  char flow1TotalBuffer[10];     // Temporary buffer for total volume

  // Read and format data from flow0
  if (flow0.readSensorData()) {
    dtostrf(flow0.flowRate, 6, 2, flow0RateBuffer);      // Convert flowRate to string
    dtostrf(flow0.temperature, 6, 2, flow0TempBuffer);   // Convert temperature to string
    dtostrf(flow0.volume, 6, 2, flow0VolumeBuffer);      // Convert dispense volume to string
    dtostrf(flow0.totalVolume, 6, 2, flow0TotalBuffer);  // Convert total volume to string
    snprintf(flow0Output, sizeof(flow0Output), "flow0, %d, %s, %s, %s, %s, %d",
             flow0.sensorConnected, flow0RateBuffer, flow0TempBuffer, flow0VolumeBuffer, flow0TotalBuffer, flow0.highFlowFlag);
  } else {
    snprintf(flow0Output, sizeof(flow0Output), "flow0, %d, n/a, n/a, n/a, n/a, n/a", flow0.sensorConnected);
  }

  // Read and format data from flow1
  if (flow1.readSensorData()) {
    dtostrf(flow1.flowRate, 6, 2, flow1RateBuffer);      // Convert flowRate to string
    dtostrf(flow1.temperature, 6, 2, flow1TempBuffer);   // Convert temperature to string
    dtostrf(flow1.volume, 6, 2, flow1VolumeBuffer);      // Convert dispense volume to string
    dtostrf(flow1.totalVolume, 6, 2, flow1TotalBuffer);  // Convert total volume to string
    snprintf(flow1Output, sizeof(flow1Output), "flow1, %d, %s, %s, %s, %s, %d",
             flow1.sensorConnected, flow1RateBuffer, flow1TempBuffer, flow1VolumeBuffer, flow1TotalBuffer, flow1.highFlowFlag);
  } else {
    snprintf(flow1Output, sizeof(flow1Output), "flow1, %d, n/a, n/a, n/a, n/a, n/a", flow1.sensorConnected);
  }

  // Print both sensor outputs on the same line
  if (currentMillis - lastPrintTime >= printInterval) {
    Serial.println(String(flow0Output) + " | " + String(flow1Output));
    lastPrintTime = currentMillis;
  }
}
