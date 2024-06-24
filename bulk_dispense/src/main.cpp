#include "project_includes.h" // Include the project includes header
#include "trough_state.h"
#include "bottle_pressure.h"
#include "moving_average.h"

#define PRESSURE_SENSOR_PIN CONTROLLINO_A1

void setup()
{
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the analog pin as an input
  pinMode(TROUGH_STATE_SENSOR, INPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
}

void loop()
{

  Serial.print("trough state: ");
  Serial.println(troughState());
  Serial.print("pressure(Psi): ");
  Serial.println(movingAverage(bottlePressure(), 10), 1);

  // Small delay before the next loop
  delay(200);
}
