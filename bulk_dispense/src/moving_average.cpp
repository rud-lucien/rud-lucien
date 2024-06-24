#include "moving_average.h"

float movingAverage(float currentReading, int numReadings) {
    static float *readings = nullptr;
    static int readIndex = 0;
    static float total = 0;

    // Allocate the array if it hasn't been done yet
    if (readings == nullptr) {
        readings = new float[numReadings];
        for (int i = 0; i < numReadings; i++) {
            readings[i] = 0;
        }
    }

    // Subtract the oldest reading from the total
    total -= readings[readIndex];

    // Add the current reading to the array and the total
    readings[readIndex] = currentReading;
    total += currentReading;

    // Advance to the next position in the array
    readIndex = (readIndex + 1) % numReadings;

    // Calculate and return the average
    return total / numReadings;
}
