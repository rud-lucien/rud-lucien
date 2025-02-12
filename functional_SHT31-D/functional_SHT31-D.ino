#include <Wire.h>
#include <Controllino.h>
#include "Adafruit_SHT31.h"

#define MULTIPLEXER_ADDR  0x70  // PCA9548 I2C multiplexer address
#define SENSOR_ADDR       0x44  // SHT31-D default I2C address
#define SENSOR_CHANNEL    1     // Channel where SHT31-D is connected

const long interval = 500; // Sensor read interval in milliseconds
unsigned long previousMillis = 0; // Stores last sensor read time

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Pure function: Reads temperature and humidity, returns as a struct
struct SensorData {
  float temperature;
  float humidity;
  bool valid;
};

// Pure function: Selects multiplexer channel
void selectMultiplexerChannel(uint8_t channel) {
  Wire.beginTransmission(MULTIPLEXER_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Pure function: Initializes the sensor and returns success status
bool initializeSensor() {
  selectMultiplexerChannel(SENSOR_CHANNEL);
  return sht31.begin(SENSOR_ADDR);
}

// Function to read sensor data (pure)
SensorData readSensorData() {
  selectMultiplexerChannel(SENSOR_CHANNEL);

  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    return {0, 0, false}; // Return invalid data
  }
  return {temperature, humidity, true};
}

// Higher-order function: Takes a function pointer for output
void processSensorData(void (*outputFunction)(SensorData)) {
  SensorData data = readSensorData();
  outputFunction(data);
}


void printSensorData(SensorData data) {
  if (data.valid) {
    Serial.print(F("Temperature: "));
    Serial.print(data.temperature);
    Serial.print(F(" °C\tHumidity: "));
    Serial.print(data.humidity);
    Serial.println(F(" %"));
  } else {
    Serial.println(F("Error: Failed to read from SHT31-D sensor."));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println(F("Initializing SHT31-D Sensor via Multiplexer..."));
  if (!initializeSensor()) {
    Serial.println(F("Error: SHT31-D not detected!"));
    while (1) delay(1000);
  }
  Serial.println(F("SHT31-D Sensor initialized successfully!"));
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    processSensorData(printSensorData); // Call function with output handler
  }
}


