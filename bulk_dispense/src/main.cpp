#include "project_includes.h" // Include the project includes header
#include "trough_state.h"

void setup()
{
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the analog pin as an input
  pinMode(TROUGH_STATE_SENSOR, INPUT);
}

void loop()
{
  // Print the state of trough sensor to the serial monitor 1=full, 0=false
  Serial.print("trough state: ");
  Serial.println(troughState());

  // Small delay before the next loop
  delay(200);
}
