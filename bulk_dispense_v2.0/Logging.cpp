#include "Logging.h"
#include "Hardware.h"  // For global hardware objects and hardware functions
#include "Sensors.h"   // For reading temperature, humidity, pressure, and flow sensor data
#include "Utils.h"     // (Optional) For any additional helper functions

// Define the global logging instance (default log interval: 250 ms)
LoggingManagement logging = { 0, 250 };

void logData(const char* module, const char* message) {
  Serial.print(F("[LOG] "));
  Serial.print(module);
  Serial.print(F(" - "));
  Serial.println(message);
}

void logSystemState() {
  char buffer[400];

  // --- Fan state ---
  char fanState = (digitalRead(fan.relayPin) == HIGH ? '1' : '0');

  // --- Valve states ---
  char rValve1 = (reagentValve1.isOpen ? '1' : '0');
  char rValve2 = (reagentValve2.isOpen ? '1' : '0');
  char rValve3 = (reagentValve3.isOpen ? '1' : '0');
  char rValve4 = (reagentValve4.isOpen ? '1' : '0');

  char mValve1 = (mediaValve1.isOpen ? '1' : '0');
  char mValve2 = (mediaValve2.isOpen ? '1' : '0');
  char mValve3 = (mediaValve3.isOpen ? '1' : '0');
  char mValve4 = (mediaValve4.isOpen ? '1' : '0');

  char wValve1 = (wasteValve1.isOpen ? '1' : '0');
  char wValve2 = (wasteValve2.isOpen ? '1' : '0');
  char wValve3 = (wasteValve3.isOpen ? '1' : '0');
  char wValve4 = (wasteValve4.isOpen ? '1' : '0');

  // --- Proportional Valve Feedback ---
  char pFeedbackStr[8];
  dtostrf(getValveFeedback(proportionalValve), 4, 1, pFeedbackStr);
  float feedback = getValveFeedback(proportionalValve);
  float valvePercent = (proportionalValveMaxFeedback > 0) ? (feedback / proportionalValveMaxFeedback) * 100.0 : 0.0;
  char pPercentStr[8];
  dtostrf(valvePercent, 4, 1, pPercentStr);

  // --- Binary Sensor States ---
  char wsl1 = (readBinarySensor(wasteLineSensors[0]) ? '1' : '0');
  char wsl2 = (readBinarySensor(wasteLineSensors[1]) ? '1' : '0');

  char wbl1 = (readBinarySensor(wasteBottleSensors[0]) ? '1' : '0');
  char wbl2 = (readBinarySensor(wasteBottleSensors[1]) ? '1' : '0');

  char wvs1 = (readBinarySensor(wasteVacuumSensors[0]) ? '1' : '0');
  char wvs2 = (readBinarySensor(wasteVacuumSensors[1]) ? '1' : '0');

  char els1 = (readBinarySensor(enclosureLiquidSensor) ? '1' : '0');

  // --- Bubble Sensors ---
  char bs1 = (readBinarySensor(reagentBubbleSensors[0]) ? '1' : '0');
  char bs2 = (readBinarySensor(reagentBubbleSensors[1]) ? '1' : '0');
  char bs3 = (readBinarySensor(reagentBubbleSensors[2]) ? '1' : '0');
  char bs4 = (readBinarySensor(reagentBubbleSensors[3]) ? '1' : '0');

  // --- Overflow Sensors ---
  char os1 = (readBinarySensor(overflowSensors[0]) ? '1' : '0');
  char os2 = (readBinarySensor(overflowSensors[1]) ? '1' : '0');
  char os3 = (readBinarySensor(overflowSensors[2]) ? '1' : '0');
  char os4 = (readBinarySensor(overflowSensors[3]) ? '1' : '0');

  // --- Pressure Sensor ---
  float pressureValue = readPressure(pressureSensor);
  char pressureStr[8];
  dtostrf(pressureValue, 4, 1, pressureStr);

  // --- Temperature & Humidity ---
  TempHumidity th = readTempHumidity();
  char tempStr[8], humStr[8];
  if (th.valid) {
    dtostrf(th.temperature, 4, 1, tempStr);
    dtostrf(th.humidity, 4, 1, humStr);
  } else {
    strcpy(tempStr, "-1");
    strcpy(humStr, "-1");
  }

  // --- Flow Sensor Data (for 4 sensors) ---
  char f1Rate[8], f1Temp[8], f1Disp[8], f1Total[8], f1Flag[4];
  dtostrf(flow1.isValidReading ? flow1.flowRate : -1, 4, 1, f1Rate);
  dtostrf(flow1.isValidReading ? flow1.temperature : -1, 4, 1, f1Temp);
  dtostrf(flow1.dispenseVolume, 4, 1, f1Disp);
  dtostrf(flow1.totalVolume, 4, 1, f1Total);
  snprintf(f1Flag, sizeof(f1Flag), "%d", flow1.isValidReading ? flow1.highFlowFlag : -1);

  char f2Rate[8], f2Temp[8], f2Disp[8], f2Total[8], f2Flag[4];
  dtostrf(flow2.isValidReading ? flow2.flowRate : -1, 4, 1, f2Rate);
  dtostrf(flow2.isValidReading ? flow2.temperature : -1, 4, 1, f2Temp);
  dtostrf(flow2.dispenseVolume, 4, 1, f2Disp);
  dtostrf(flow2.totalVolume, 4, 1, f2Total);
  snprintf(f2Flag, sizeof(f2Flag), "%d", flow2.isValidReading ? flow2.highFlowFlag : -1);

  char f3Rate[8], f3Temp[8], f3Disp[8], f3Total[8], f3Flag[4];
  dtostrf(flow3.isValidReading ? flow3.flowRate : -1, 4, 1, f3Rate);
  dtostrf(flow3.isValidReading ? flow3.temperature : -1, 4, 1, f3Temp);
  dtostrf(flow3.dispenseVolume, 4, 1, f3Disp);
  dtostrf(flow3.totalVolume, 4, 1, f3Total);
  snprintf(f3Flag, sizeof(f3Flag), "%d", flow3.isValidReading ? flow3.highFlowFlag : -1);

  char f4Rate[8], f4Temp[8], f4Disp[8], f4Total[8], f4Flag[4];
  dtostrf(flow4.isValidReading ? flow4.flowRate : -1, 4, 1, f4Rate);
  dtostrf(flow4.isValidReading ? flow4.temperature : -1, 4, 1, f4Temp);
  dtostrf(flow4.dispenseVolume, 4, 1, f4Disp);
  dtostrf(flow4.totalVolume, 4, 1, f4Total);
  snprintf(f4Flag, sizeof(f4Flag), "%d", flow4.isValidReading ? flow4.highFlowFlag : -1);

  // Dispensing state for valves 1-4
  char ds1 = (valveControls[0].isDispensing ? '1' : '0');
  char ds2 = (valveControls[1].isDispensing ? '1' : '0');
  char ds3 = (valveControls[2].isDispensing ? '1' : '0');
  char ds4 = (valveControls[3].isDispensing ? '1' : '0');

  // Convert target volumes to strings
  char tv1[8], tv2[8], tv3[8], tv4[8];
  dtostrf(valveControls[0].targetVolume, 4, 1, tv1);
  dtostrf(valveControls[1].targetVolume, 4, 1, tv2);
  dtostrf(valveControls[2].targetVolume, 4, 1, tv3);
  dtostrf(valveControls[3].targetVolume, 4, 1, tv4);

  // Format the log message.
  sprintf(buffer,
          "[LOG] F%c, RV%c%c%c%c, MV%c%c%c%c, WV%c%c%c%c, PV,%s, PV%%,%s, "
          "WSL%c%c, WBL%c%c, WVS%c%c, ELS%c, BS%c%c%c%c, OS%c%c%c%c, "
          "PS,%s, T,%s, H,%s, FS1,%s,%s,%s,%s,%s; FS2,%s,%s,%s,%s,%s; "
          "FS3,%s,%s,%s,%s,%s; FS4,%s,%s,%s,%s,%s, ,DS,%c%c%c%c, TV,%s,%s,%s,%s;",
          // Fan state
          fanState,
          // Reagent valves
          rValve1, rValve2, rValve3, rValve4,
          // Media valves
          mValve1, mValve2, mValve3, mValve4,
          // Waste valves
          wValve1, wValve2, wValve3, wValve4,
          // Proportional valve feedback
          pFeedbackStr, pPercentStr,
          // Waste line sensors
          wsl1, wsl2,
          // Waste bottle sensors
          wbl1, wbl2,
          // Waste vacuum sensors
          wvs1, wvs2,
          // Enclosure liquid sensor
          els1,
          // Bubble sensors
          bs1, bs2, bs3, bs4,
          // Overflow sensors
          os1, os2, os3, os4,
          // Pressure, Temperature, Humidity
          pressureStr, tempStr, humStr,
          // Flow Sensor 1 data
          f1Rate, f1Temp, f1Disp, f1Total, f1Flag,
          // Flow Sensor 2 data
          f2Rate, f2Temp, f2Disp, f2Total, f2Flag,
          // Flow Sensor 3 data
          f3Rate, f3Temp, f3Disp, f3Total, f3Flag,
          // Flow Sensor 4 data
          f4Rate, f4Temp, f4Disp, f4Total, f4Flag,
          ds1, ds2, ds3, ds4,
          tv1, tv2, tv3, tv4);

  Serial.println(buffer);
}
