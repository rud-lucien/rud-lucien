#include <Wire.h>
#include <Controllino.h>

// ✅ Move struct definition to the top
struct FlowData {
  float flowRate;
  float temperature;
  int highFlowFlag;
  int sensorConnected;
  float dispenseVolume;
  float totalVolume;
  bool valid;
};

// ✅ Function prototypes (declarations) for functions that come later
void processFlowData(class FlowSensor &sensor, void (*outputFunction)(FlowData));
void printFlowData(FlowData data);

// Function to reset the I2C bus
void resetI2CBus() {
  Serial.println(F("Resetting I2C bus..."));
  Wire.end();
  delay(100);
  Wire.begin();
}

// Function to select an I2C channel on the multiplexer
void selectMultiplexerChannel(uint8_t multiplexerAddr, uint8_t channel) {
  Wire.beginTransmission(multiplexerAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ✅ Move class definition above where it is used
class FlowSensor {
private:
  uint8_t multiplexerAddr;
  uint8_t sensorAddr;
  uint8_t channel;
  uint16_t measurementCmd;
  bool sensorInitialized;
  bool sensorStopped;
  unsigned long lastUpdateTime;

public:
  FlowSensor(uint8_t muxAddr, uint8_t addr, uint8_t chan, uint16_t cmd)
    : multiplexerAddr(muxAddr), sensorAddr(addr), channel(chan),
      measurementCmd(cmd), sensorInitialized(false), sensorStopped(true),
      lastUpdateTime(0) {}

  bool initializeSensor() {
    if (sensorStopped) {
      Serial.print(F("Sensor on channel "));
      Serial.print(channel);
      Serial.println(F(" is intentionally stopped."));
      return false;
    }

    selectMultiplexerChannel(multiplexerAddr, channel);
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("Soft reset failed"));
      resetI2CBus();
      return false;
    }
    delay(50);

    Wire.beginTransmission(sensorAddr);
    Wire.write(measurementCmd >> 8);
    Wire.write(measurementCmd & 0xFF);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("Failed to start measurement mode"));
      resetI2CBus();
      return false;
    }

    Serial.println(F("Sensor initialized successfully"));
    sensorInitialized = true;
    sensorStopped = false;
    lastUpdateTime = millis();
    return true;
  }

  bool checkConnection() {
    selectMultiplexerChannel(multiplexerAddr, channel);
    Wire.beginTransmission(sensorAddr);
    return (Wire.endTransmission() == 0);
  }

  FlowData readSensorData() {
    if (!sensorInitialized || sensorStopped) {
      return {0, 0, 0, 0, 0, 0, false};
    }

    selectMultiplexerChannel(multiplexerAddr, channel);
    Wire.requestFrom(sensorAddr, (uint8_t)9);
    
    if (Wire.available() < 9) {
      Serial.print(F("Error: Not enough bytes from sensor on channel "));
      Serial.println(channel);
      return {0, 0, 0, 0, 0, 0, false};
    }

    uint16_t flowRaw = (Wire.read() << 8) | Wire.read();
    Wire.read();
    uint16_t tempRaw = (Wire.read() << 8) | Wire.read();
    Wire.read();
    uint16_t auxRaw = (Wire.read() << 8) | Wire.read();
    Wire.read();

    float flowRate = (int16_t)flowRaw / 32.0;
    float temperature = (int16_t)tempRaw / 200.0;
    int highFlowFlag = (auxRaw & 0x02) ? 1 : 0;

    unsigned long currentTime = millis();
    float elapsedMinutes = (currentTime - lastUpdateTime) / 60000.0;
    float increment = flowRate * elapsedMinutes;

    lastUpdateTime = currentTime;

    return {flowRate, temperature, highFlowFlag, 1, increment, increment, true};
  }
};

// Create Flow Sensor Objects
FlowSensor flow0(0x70, 0x08, 0, 0x3608);
FlowSensor flow1(0x70, 0x08, 1, 0x3608);

// ✅ processFlowData must come **after** FlowSensor is defined
void processFlowData(FlowSensor &sensor, void (*outputFunction)(FlowData)) {
  FlowData data = sensor.readSensorData();
  outputFunction(data);
}

// ✅ printFlowData must come **after** FlowData is defined
void printFlowData(FlowData data) {
  if (data.valid) {
    Serial.print(F("Flow Rate: "));
    Serial.print(data.flowRate);
    Serial.print(F(" mL/min\tTemperature: "));
    Serial.print(data.temperature);
    Serial.print(F(" °C\tHigh Flow: "));
    Serial.println(data.highFlowFlag ? "Yes" : "No");
  } else {
    Serial.println(F("Error: Failed to read flow sensor data."));
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "start0") {
      flow0.initializeSensor();
      Serial.println(F("Flow 0 started"));
    } else if (command == "stop0") {
      Serial.println(F("Flow 0 stopped"));
    } else if (command == "start1") {
      flow1.initializeSensor();
      Serial.println(F("Flow 1 started"));
    } else if (command == "stop1") {
      Serial.println(F("Flow 1 stopped"));
    } else {
      Serial.println(F("Unknown command"));
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("Enter commands: start0, stop0, start1, stop1"));
}

void loop() {
  handleSerialCommands();

  unsigned long currentMillis = millis();
  static unsigned long lastPrintTime = 0;
  const unsigned long printInterval = 500;

  if (currentMillis - lastPrintTime >= printInterval) {
    processFlowData(flow0, printFlowData);
    processFlowData(flow1, printFlowData);
    lastPrintTime = currentMillis;
  }
}
