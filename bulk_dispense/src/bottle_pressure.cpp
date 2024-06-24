#include "project_includes.h" // Include the project includes header
#include "bottle_pressure.h"  // Include the project-specific header

float bottlePressure()
{
    // Read the analog value from the pressure sensor
    int pressureSensorValue = analogRead(PRESSURE_SENSOR_PIN);

    // Convert the analog value to a voltage
    float pressureSensorVoltage = pressureSensorValue * (5.0 / 1023.0);

    // Convert the voltage to pressure in psi
    float pressure = (pressureSensorVoltage * 50.0) / 5.0;

     // Adjust the pressure value by adding the constant offset
    pressure += 0.2;

    return pressure;
}
