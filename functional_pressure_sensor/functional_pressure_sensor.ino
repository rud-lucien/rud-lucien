#include "pressure_sensor.h"

PressureSensorConfig sensor1 = {A0, 0, 87};  // Example: Sensor on pin A0, range 0-87 psi
PressureSensorConfig sensor2 = {A1, 0, 100}; // Another sensor, different range

// Higher-order function: Processes sensor data with a callback
void processPressureData(const PressureSensorConfig &sensor, void (*callback)(float)) {
    float pressure = readPressure(sensor);
    callback(pressure);
}

// Output function: Prints sensor data
void printPressure(float pressure) {
    Serial.print(F("Pressure: "));
    Serial.print(pressure);
    Serial.println(F(" psi"));
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    processPressureData(sensor1, printPressure);
    processPressureData(sensor2, printPressure);
    delay(1000);
}

