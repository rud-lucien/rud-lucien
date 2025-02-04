#include <Controllino.h>
#include "PressureSensor.h"


PressureSensor myPressureSensor(CONTROLLINO_AI12, 0, 87);  // AI12, 0 psi min, 87 psi max

void setup() {
    Serial.begin(115200);
}

void loop() {
    float pressure = myPressureSensor.readPressure();
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" psi");

    delay(500);  // Update every second
}
