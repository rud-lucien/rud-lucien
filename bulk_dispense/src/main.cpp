#include <Arduino.h>
#include <Controllino.h>

#define pin_sensor CONTROLLINO_A0

void setup()
{

  // Start serial communication for debugging
  Serial.begin(115200);

 //Initialize the digital pin as an input
  pinMode(pin_sensor, INPUT);

}
int sensorState;

void loop()
{
  // Read the state of the ca`pacitive sensor
  sensorState = digitalRead(pin_sensor);

  // Print the raw sensor state to the serial monitor
  Serial.print("Raw sensor state: ");
  Serial.println(digitalRead(pin_sensor));

  // Print the sensor state to the serial monitor
  if (sensorState == HIGH)
  {
    Serial.println("D0 is HIGH");
  }
  else
  {
    Serial.println("D0 is LOW");
  }

  // Small delay before next loop
  delay(500);
}



// void setup()
// {
//   // Initialize the analog pin as an input
//   pinMode(CONTROLLINO_A0, INPUT);

//   // Start serial communication for debugging
//   Serial.begin(115200);
// }

// void loop()
// {
//   // Read the analog value from the capacitive sensor
//   int sensorValue = analogRead(CONTROLLINO_A0);

//   // Convert the analog value to voltage
//   float voltage = sensorValue * (5.0 / 1023.0);

//   // Print the raw analog sensor value and the calculated voltage to the serial monitor
//   Serial.print("Raw sensor value: ");
//   Serial.print(sensorValue);
//   Serial.print(" - Voltage: ");
//   Serial.println(voltage);

//   // Small delay before next loop
//   delay(500);
// }
