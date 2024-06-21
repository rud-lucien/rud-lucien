#include <Arduino.h>
#include <Controllino.h>

#define TROUGH_STATE_SENSOR CONTROLLINO_A0

void setup()
{

  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the digital pin as an input
  pinMode(TROUGH_STATE_SENSOR, INPUT);
}

bool troughState()
{

  // Read the state of the capacitive sensor
  bool troughSensorState = digitalRead(TROUGH_STATE_SENSOR);
  return troughSensorState;
}

void loop()
{
  // Print the state of trough sensor to the serial monitor 1=full, 0=false
  Serial.print("trough state: ");
  Serial.println(troughState());
  delay(200);
}
